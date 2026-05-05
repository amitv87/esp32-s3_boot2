#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stddef.h>
#include "app_header.h"
#include "board.h"
#include "hal_i2c.h"
#include "lcd_axs15231b.h"
#include "touch_axs15231b.h"
#include "soc/soc.h"
#include "soc/system_reg.h"

#define APP_DEST_ADDR  0x3C000000u

static void app_main(app_context_t *ctx);
extern char __app_image_size_sym[];

__attribute__((section(".app_header"), used))
const app_image_header_t __app_hdr = {
    .magic     = 0xDEADBEEFu,
    .size      = (uint32_t)(uintptr_t)__app_image_size_sym,
    .dest_addr = APP_DEST_ADDR,
    .entry     = app_main,
};

app_context_t *gctx;
static uint8_t loggerBuff[1024];
int pico_vprintf(const char *format, va_list va)
{
    int ret = vsnprintf((char *)loggerBuff, sizeof(loggerBuff), format, va);
    if (ret > 0) gctx->put_bytes((uint8_t *)loggerBuff, (size_t)ret);
    return ret;
}

static uint16_t framebuffer[LCD_WIDTH_NATIVE * LCD_HEIGHT_NATIVE];

static inline uint16_t rgb(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t v = ((uint16_t)(r >> 3) << 11) | ((uint16_t)(g >> 2) << 5) | (b >> 3);
    return (uint16_t)((v >> 8) | (v << 8));
}

static void fb_fill(uint16_t color)
{
    uint32_t n = LCD_WIDTH_NATIVE * LCD_HEIGHT_NATIVE;
    for (uint32_t i = 0; i < n; i++) framebuffer[i] = color;
}

static void fb_draw_square(uint16_t col, uint16_t row, uint16_t sz, uint16_t color)
{
    int16_t r0 = (int16_t)row - (int16_t)(sz / 2);
    int16_t c0 = (int16_t)col - (int16_t)(sz / 2);
    for (int16_t dr = 0; dr < (int16_t)sz; dr++) {
        int16_t r = r0 + dr;
        if (r < 0 || r >= LCD_HEIGHT_NATIVE) continue;
        for (int16_t dc = 0; dc < (int16_t)sz; dc++) {
            int16_t c = c0 + dc;
            if (c < 0 || c >= LCD_WIDTH_NATIVE) continue;
            framebuffer[(uint16_t)r * LCD_WIDTH_NATIVE + (uint16_t)c] = color;
        }
    }
}

/* ---- LEDC PWM on GPIO8 (AP3032 CTRL) -------------------------
 * Timer 3, Channel 1, low-speed mode.
 * APB=80MHz, 8-bit resolution, 50kHz: CLK_DIV = 80M/(50k*256) = 1600 (stored <<4 in reg).
 * Duty = 255 << 4 = 4080 (≈100%).
 * LEDC_LS_SIG_OUT1_IDX=74 routed to GPIO8. */
#define LEDC_BASE       0x60019000UL
#define LEDC_T3_CONF    (*(volatile uint32_t *)(LEDC_BASE + 0xB8))
#define LEDC_CH1_CONF0  (*(volatile uint32_t *)(LEDC_BASE + 0x14))
#define LEDC_CH1_HPOINT (*(volatile uint32_t *)(LEDC_BASE + 0x18))
#define LEDC_CH1_DUTY   (*(volatile uint32_t *)(LEDC_BASE + 0x1C))
#define LEDC_CH1_CONF1  (*(volatile uint32_t *)(LEDC_BASE + 0x20))
#define LEDC_CONF_GLB   (*(volatile uint32_t *)(LEDC_BASE + 0xD0))

static void bl_enable(void)
{
    /* Release GPIO8 (LCD_BL) to high-Z (input).
     * The AP3032 CTRL node is driven HIGH by BL_EN (EXIO1) through R22 (100K).
     * When GPIO8 is output-LOW it overrides BL_EN through the resistor divider.
     * Leaving GPIO8 floating lets BL_EN alone control CTRL → backlight on. */
    GPIO_ENABLE_W1TC_REG = (1u << LCD_PIN_BL);  /* disable output driver */
    printf("GPIO8 released to input; BL_EN (EXIO1) drives CTRL\r\n");
}

/* ---- TCA9554PWR IO expander (0x20, GPIO47/48 bus) ------------
 * MUST be called AFTER lcd_init(): GPIO21 (LCD RST) doubles as
 * TCA9554 RESET# pin, so every lcd_init() hardware-reset pulse
 * returns the expander to all-inputs / default-low-latch state.
 *
 * Sets only EXIO1 (BL_EN=bit1) as output HIGH.  EXIO6 (SYS_EN) stays
 * as input so R38 (10K to GND) keeps Q3 ON and VSYS alive.
 * Reads back both registers for diagnostics. */
static void ioexp_init(bb_i2c_t *bus, uint8_t addr)
{
    uint8_t reg, out = 0, cfg = 0xFF;

    /* readback before */
    reg = 0x01; bb_i2c_write_read(bus, addr, &reg, 1, &out, 1);
    reg = 0x03; bb_i2c_write_read(bus, addr, &reg, 1, &cfg, 1);
    printf("IOEXP before: out=0x%02X cfg=0x%02X\r\n", out, cfg);

    /* Only EXIO1 (BL_EN, bit1) = output HIGH.
     * EXIO6 (SYS_EN) MUST stay as input: R38 (10K to GND) holds Q3 (P-ch FET)
     * gate at 0V → Q3 ON → VSYS powers AP3032.  Driving SYS_EN HIGH raises the
     * FET gate, cuts VSYS, and the AP3032 loses its VIN → LED off. */
    bb_i2c_write_reg(bus, addr, 0x01, (1u<<1));              /* EXIO1=1 (BL_EN on) */
    bb_i2c_write_reg(bus, addr, 0x03, (uint8_t)(~(1u<<1)));  /* only EXIO1 as output (0xFD) */

    /* readback after (output latch + config) */
    reg = 0x01; bb_i2c_write_read(bus, addr, &reg, 1, &out, 1);
    reg = 0x03; bb_i2c_write_read(bus, addr, &reg, 1, &cfg, 1);
    printf("IOEXP after:  out=0x%02X cfg=0x%02X\r\n", out, cfg);

    /* Read actual input port (reg 0x00) — reflects real pin voltages.
     * EXIO1 bit1 should read 1 (BL_EN wire is HIGH).
     * EXIO6 bit6 should read 0 if Q3 gate is at GND (Q3 ON, VSYS powered). */
    uint8_t inport = 0;
    reg = 0x00; bb_i2c_write_read(bus, addr, &reg, 1, &inport, 1);
    printf("IOEXP input:  pins=0x%02X (EXIO1=%d EXIO6=%d)\r\n",
           inport, (inport >> 1) & 1, (inport >> 6) & 1);
}

static void app_main(app_context_t *ctx)
{
    extern char _bss_start[], _bss_end[];
    for (char *p = _bss_start; p < _bss_end; p++) *p = 0;
    gctx = ctx;

    printf("\r\n=== touch-lcd app ===\r\n");

    bb_i2c_t tp_bus, sys_bus;
    bb_i2c_init(&tp_bus,  TOUCH_PIN_SDA, TOUCH_PIN_SCL);
    bb_i2c_init(&sys_bus, 47, 48);

    bb_i2c_scan(&sys_bus, "sys GPIO47/48");

    /* Hardware reset LCD (GPIO21) BEFORE ioexp_init so the TCA9554 is not
     * reset by the pulse that happens inside lcd_init.  gpio_output_init
     * is needed first so GPIO21 is actually driven. */
    gpio_output_init(21);
#define DLYMS(ms) for (volatile uint32_t _i=0; _i<(ms)*60000u; _i++)
    GPIO_OUT_W1TS_REG = 1u << 21;  DLYMS(10);
    GPIO_OUT_W1TC_REG = 1u << 21;  DLYMS(250);
    GPIO_OUT_W1TS_REG = 1u << 21;  DLYMS(120);
    printf("HW reset done\r\n");

    /* IO expander: EXIO1=HIGH drives AP3032 CTRL → backlight on.
     * GPIO8 (LCD_BL) goes to AP3032 FB (brightness), NOT to CTRL.
     * Driving GPIO8 HIGH kills the backlight by overriding FB reference.
     * We never configure GPIO8 as output — it stays INPUT (boot default). */
    ioexp_init(&sys_bus, 0x20);
    /* bl_enable() not needed — GPIO8 is never touched */

    /* LCD init — pass micros_now for accurate hardware-timed delays */
    printf("LCD init...\r\n");
    lcd_init(ctx->micros_now);
    printf("LCD init done.\r\n");

    /* Verify pin states */
    {
        uint32_t gin = GPIO_IN_REG;
        printf("GPIO_IN: GPIO8=%d GPIO17=%d GPIO18=%d\r\n",
               (int)((gin >> 8) & 1), (int)((gin >> 17) & 1), (int)((gin >> 18) & 1));
    }

    /* Touch IC (AXS15231B) probe.
     * Simple address-only probe (START+addr+STOP) may NACK because the IC only
     * ACKs once it receives the full 11-byte command frame.  Try the real read. */
    for (volatile uint32_t i = 0; i < 6000000u; i++);  /* ~1s wait */
    {
        static const uint8_t tp_cmd[11] = {
            0xB5,0xAB,0xA5,0x5A, 0x00,0x00,0x00,0x0E, 0x00,0x00,0x00
        };
        uint8_t tp_data[14] = {0};
        bool ok = bb_i2c_write_read(&tp_bus, TOUCH_I2C_ADDR, tp_cmd, 11, tp_data, 14);
        printf("Touch probe 0x%02X: %s data=%02X %02X %02X %02X %02X %02X\r\n",
               TOUCH_I2C_ADDR, ok ? "ACK" : "NAK",
               tp_data[0], tp_data[1], tp_data[2], tp_data[3], tp_data[4], tp_data[5]);
    }
    bb_i2c_scan(&tp_bus, "touch GPIO17/18");

    printf("Touch init...\r\n");
    touch_init();
    printf("Touch init done.\r\n");

    /* Diagnostic: cycle full-screen solid red → green → blue → bands.
     * Each fill stays visible for ~1 second so we can see if uniform color
     * really covers the whole display.  If it does, addressing is sound and
     * we can move to the bands pattern. */
    uint16_t band_r = rgb(255,   0,   0);
    uint16_t band_g = rgb(  0, 255,   0);
    uint16_t band_b = rgb(  0,   0, 255);
    for (int phase = 0; phase < 3; phase++) {
        uint16_t c = (phase == 0) ? band_r : (phase == 1) ? band_g : band_b;
        fb_fill(c);
        lcd_blit_frame(framebuffer);
        printf("Solid fill phase %d done\r\n", phase);
        for (volatile uint32_t i = 0; i < 10000000u; i++);  /* ~1s */
    }
    /* Final pattern: 3 horizontal bands red/green/blue. */
    {
        uint32_t third = LCD_HEIGHT_NATIVE / 3u;
        for (uint32_t r = 0; r < LCD_HEIGHT_NATIVE; r++) {
            uint16_t c = (r < third) ? band_r : (r < 2u * third) ? band_g : band_b;
            for (uint32_t x = 0; x < LCD_WIDTH_NATIVE; x++) {
                framebuffer[r * LCD_WIDTH_NATIVE + x] = c;
            }
        }
    }
    lcd_blit_frame(framebuffer);
    printf("Frame ready (RGB bands). Touch to paint!\r\n");

    uint16_t bg = rgb(0, 0, 80);
    (void)bg;

    uint16_t red    = rgb(255,  64,  64);
    uint16_t yellow = rgb(255, 220,   0);
    touch_point_t pt;
    bool prev = false;
    uint32_t frame = 0;

    for (;;) {
        touch_read(&pt);
        if (pt.pressed) {
            printf("touch %u,%u\r\n", pt.x, pt.y);
            uint16_t row = (uint16_t)(639u - (pt.x < 640u ? pt.x : 639u));
            uint16_t col = (uint16_t)(pt.y < 172u ? pt.y : 171u);
            fb_draw_square(col, row, 20, (frame & 0x3F) < 20 ? yellow : red);
            lcd_blit_frame(framebuffer);
            prev = true;
        } else if (prev) {
            prev = false;
        }
        frame++;
        // for (volatile int i = 0; i < 10000; i++);
    }
}
