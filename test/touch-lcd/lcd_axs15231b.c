/*
 * Bare-metal QSPI driver for AXS15231B on Waveshare ESP32-S3-Touch-LCD-3.49.
 *
 * Native panel: 172 cols × 640 rows (portrait).
 * We operate in native portrait mode (MADCTL=0x00), no software rotation.
 * Touch X (0..639) maps to screen row; Touch Y (0..171) maps to screen column.
 *
 * QSPI protocol (use_qspi_interface=1):
 *   cmd word = (opcode << 24) | (mipi_cmd << 8)
 *   opcode 0x02 → write cmd/param; opcode 0x32 → write pixels (RAMWR)
 *   All bytes sent quad (4-bit) mode on D0-D3.
 *
 * SPI3 (GPSPI3, base 0x60025000) at 40 MHz.
 * CS driven manually via GPIO so it spans multiple 64-byte FIFO chunks.
 */

#include "lcd_axs15231b.h"
#include "board.h"
#include "hal_gpio.h"

#include "soc/soc.h"
#include "soc/spi_reg.h"
#include "soc/system_reg.h"
#include "soc/io_mux_reg.h"

#define SPI_N  3   /* use GPSPI3 */

/* ---- Delay (busy-loop) ---- */
static void delay_ms(uint32_t ms)
{
    for (volatile uint32_t i = 0; i < ms * 60000u; i++);
}

/* ---- Manual CS ---- */
static inline void cs_lo(void) { GPIO_OUT_W1TC_REG = (1u << LCD_PIN_CS); }
static inline void cs_hi(void) { GPIO_OUT_W1TS_REG = (1u << LCD_PIN_CS); }

/* ---- SPI3 QSPI chunk write (CS already asserted by caller) ----
 * Sends `len` bytes from `buf` in quad mode, chunking through the 64-byte FIFO.
 */
static void spi_write_quad(const void *buf, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    while (len) {
        uint32_t chunk = len > 64u ? 64u : len;
        uint32_t words = (chunk + 3u) >> 2;

        for (uint32_t i = 0; i < words; i++) {
            uint32_t base = i * 4u, w = 0;
            for (uint32_t j = 0; j < 4u && (base + j) < chunk; j++)
                w |= (uint32_t)p[base + j] << (24u - j * 8u);
            REG_WRITE(SPI_W0_REG(SPI_N) + i * 4u, w);
        }

        REG_WRITE(SPI_MS_DLEN_REG(SPI_N), chunk * 8u - 1u);
        REG_WRITE(SPI_USER_REG(SPI_N),
                  SPI_USR_MOSI | SPI_FWRITE_QUAD | SPI_CS_SETUP | SPI_CS_HOLD);

        /* Sync USER/DLEN registers into SPI clock domain before triggering */
        REG_SET_BIT(SPI_CMD_REG(SPI_N), SPI_UPDATE);
        while (REG_GET_BIT(SPI_CMD_REG(SPI_N), SPI_UPDATE)) {}

        REG_SET_BIT(SPI_CMD_REG(SPI_N), SPI_USR);
        while (REG_GET_BIT(SPI_CMD_REG(SPI_N), SPI_USR)) {}

        p   += chunk;
        len -= chunk;
    }
}

/* ---- QSPI command helpers ---- */
static void lcd_cmd(uint8_t cmd)
{
    uint8_t w[4] = {0x02, cmd, 0x00, 0x00};
    cs_lo();
    spi_write_quad(w, 4);
    cs_hi();
}

static void lcd_cmd_data(uint8_t cmd, const uint8_t *data, uint32_t dlen)
{
    uint8_t w[4] = {0x02, cmd, 0x00, 0x00};
    cs_lo();
    spi_write_quad(w, 4);
    if (dlen) spi_write_quad(data, dlen);
    cs_hi();
}

/* ---- SPI3 GPIO routing ----
 * Signal indices from gpio_sig_map.h for SPI3 (GPSPI3):
 *   CLK=66, D(MOSI)=68, Q(MISO)=67, HD=69, WP=70, CS0=71
 */
#define SIG_SPI3_CLK 66
#define SIG_SPI3_D   68
#define SIG_SPI3_Q   67
#define SIG_SPI3_HD  69
#define SIG_SPI3_WP  70

static void spi3_gpio_setup(void)
{
    /* High-drive (FUN_DRV=3) GPIO-matrix function (MCU_SEL=1) for SPI data pins */
    static const int spi_data_pins[] = {
        LCD_PIN_CLK, LCD_PIN_DATA0, LCD_PIN_DATA1, LCD_PIN_DATA2, LCD_PIN_DATA3
    };
    for (int i = 0; i < 5; i++) {
        int p = spi_data_pins[i];
        *iomux_reg(p) = (1u << 12) | (3u << 10);
        GPIO_ENABLE_W1TS_REG = 1u << p;
    }
    gpio_matrix_out(LCD_PIN_CLK,   SIG_SPI3_CLK, false);
    gpio_matrix_out(LCD_PIN_DATA0, SIG_SPI3_D,   false);
    gpio_matrix_out(LCD_PIN_DATA1, SIG_SPI3_Q,   false);
    gpio_matrix_out(LCD_PIN_DATA2, SIG_SPI3_HD,  false);
    gpio_matrix_out(LCD_PIN_DATA3, SIG_SPI3_WP,  false);

    gpio_output_init(LCD_PIN_CS);
    gpio_output_init(LCD_PIN_RST);
    gpio_output_init(LCD_PIN_BL);
    cs_hi();
    GPIO_OUT_W1TS_REG = (1u << LCD_PIN_RST) | (1u << LCD_PIN_BL);
}

/* ---- SPI3 peripheral setup ---- */
static void spi3_periph_setup(void)
{
    /* Enable clock, then do a clean reset (assert → deassert) */
    REG_SET_BIT(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_SPI3_CLK_EN);
    REG_SET_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_SPI3_RST);
    for (volatile int i = 0; i < 200; i++);
    REG_CLR_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_SPI3_RST);

    /* Explicitly put in master mode (slave register = 0) */
    REG_WRITE(SPI_SLAVE_REG(SPI_N), 0);

    /* Enable the SPI peripheral's internal clock gate.
     * SPI_CLK_GATE_REG (offset 0xE8) must be configured or the SPI state
     * machine has no clock and SPI_USR will never clear.
     *   bit0 = CLK_EN      (register-bus clock gate)
     *   bit1 = MST_CLK_ACTIVE (master functional clock)
     *   bit2 = MST_CLK_SEL   (0=XTAL, 1=PLL/APB → use 1) */
    REG_WRITE(SPI_CLK_GATE_REG(SPI_N),
              SPI_CLK_EN | SPI_MST_CLK_ACTIVE | SPI_MST_CLK_SEL);

    /* Use APB pass-through (80 MHz). Can tune to 40 MHz later. */
    REG_WRITE(SPI_CLOCK_REG(SPI_N), SPI_CLK_EQU_SYSCLK);

    /* CTRL: enable both FREAD_QUAD and FWRITE_QUAD so all 4 I/O lines are
     * in QSPI mode; also set WP/HD polarity high so idle lines don't glitch */
    REG_WRITE(SPI_CTRL_REG(SPI_N),
              SPI_FREAD_QUAD | SPI_HOLD_POL | SPI_WP_POL);

    /* Disable all hardware-managed CS (we drive CS manually via GPIO) */
    REG_SET_BIT(SPI_MISC_REG(SPI_N), SPI_CS0_DIS | SPI_CS1_DIS);

    /* SPI mode 3: clock idles high (CPOL=1) */
    REG_SET_BIT(SPI_MISC_REG(SPI_N), SPI_CK_IDLE_EDGE);

    /* Sync all register changes from APB domain into SPI module clock domain */
    REG_SET_BIT(SPI_CMD_REG(SPI_N), SPI_UPDATE);
    while (REG_GET_BIT(SPI_CMD_REG(SPI_N), SPI_UPDATE)) {}
}

/* ---- Full vendor init sequence (from Waveshare reference driver) ---- */
static void lcd_vendor_init(void)
{
    static const uint8_t BB1[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x5A,0xA5};
    lcd_cmd_data(0xBB, BB1, sizeof(BB1));
    static const uint8_t A0[] = {0x00,0x10,0x00,0x02,0x00,0x00,0x64,0x3F,0x20,0x05,0x3F,0x3F,0x00,0x00,0x00,0x00,0x00};
    lcd_cmd_data(0xA0, A0, sizeof(A0));
    static const uint8_t A2[] = {0x30,0x04,0x0A,0x3C,0xEC,0x54,0xC4,0x30,0xAC,0x28,0x7F,0x7F,0x7F,0x20,0xF8,0x10,0x02,0xFF,0xFF,0xF0,0x90,0x01,0x32,0xA0,0x91,0xC0,0x20,0x7F,0xFF,0x00,0x54};
    lcd_cmd_data(0xA2, A2, sizeof(A2));
    static const uint8_t D0[] = {0x30,0xAC,0x21,0x24,0x08,0x09,0x10,0x01,0xAA,0x14,0xC2,0x00,0x22,0x22,0xAA,0x03,0x10,0x12,0x40,0x14,0x1E,0x51,0x15,0x00,0x40,0x10,0x00,0x03,0x3D,0x12};
    lcd_cmd_data(0xD0, D0, sizeof(D0));
    static const uint8_t A3[] = {0xA0,0x06,0xAA,0x08,0x08,0x02,0x0A,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x00,0x55,0x55};
    lcd_cmd_data(0xA3, A3, sizeof(A3));
    static const uint8_t C1[] = {0x33,0x04,0x02,0x02,0x71,0x05,0x24,0x55,0x02,0x00,0x41,0x00,0x53,0xFF,0xFF,0xFF,0x4F,0x52,0x00,0x4F,0x52,0x00,0x45,0x3B,0x0B,0x02,0x0D,0x00,0xFF,0x40};
    lcd_cmd_data(0xC1, C1, sizeof(C1));
    static const uint8_t C3[] = {0x00,0x00,0x00,0x50,0x03,0x00,0x00,0x00,0x01,0x80,0x01};
    lcd_cmd_data(0xC3, C3, sizeof(C3));
    static const uint8_t C4[] = {0x00,0x24,0x33,0x90,0x50,0xEA,0x64,0x32,0xC8,0x64,0xC8,0x32,0x90,0x90,0x11,0x06,0xDC,0xFA,0x04,0x03,0x80,0xFE,0x10,0x10,0x00,0x0A,0x0A,0x44,0x50};
    lcd_cmd_data(0xC4, C4, sizeof(C4));
    static const uint8_t C5[] = {0x18,0x00,0x00,0x03,0xFE,0x78,0x33,0x20,0x30,0x10,0x88,0xDE,0x0D,0x08,0x0F,0x0F,0x01,0x78,0x33,0x20,0x10,0x10,0x80};
    lcd_cmd_data(0xC5, C5, sizeof(C5));
    static const uint8_t C6[] = {0x05,0x0A,0x05,0x0A,0x00,0xE0,0x2E,0x0B,0x12,0x22,0x12,0x22,0x01,0x00,0x00,0x3F,0x6A,0x18,0xC8,0x22};
    lcd_cmd_data(0xC6, C6, sizeof(C6));
    static const uint8_t C7[] = {0x50,0x32,0x28,0x00,0xA2,0x80,0x8F,0x00,0x80,0xFF,0x07,0x11,0x9F,0x6F,0xFF,0x26,0x0C,0x0D,0x0E,0x0F};
    lcd_cmd_data(0xC7, C7, sizeof(C7));
    static const uint8_t C9[] = {0x33,0x44,0x44,0x01};
    lcd_cmd_data(0xC9, C9, sizeof(C9));
    static const uint8_t CF[] = {0x34,0x1E,0x88,0x58,0x13,0x18,0x56,0x18,0x1E,0x68,0xF7,0x00,0x65,0x0C,0x22,0xC4,0x0C,0x77,0x22,0x44,0xAA,0x55,0x04,0x04,0x12,0xA0,0x08};
    lcd_cmd_data(0xCF, CF, sizeof(CF));
    static const uint8_t D5[] = {0x3E,0x3E,0x88,0x00,0x44,0x04,0x78,0x33,0x20,0x78,0x33,0x20,0x04,0x28,0xD3,0x47,0x03,0x03,0x03,0x03,0x86,0x00,0x00,0x00,0x30,0x52,0x3F,0x40,0x40,0x96};
    lcd_cmd_data(0xD5, D5, sizeof(D5));
    static const uint8_t D6[] = {0x10,0x32,0x54,0x76,0x98,0xBA,0xDC,0xFE,0x95,0x00,0x01,0x83,0x75,0x36,0x20,0x75,0x36,0x20,0x3F,0x03,0x03,0x03,0x10,0x10,0x00,0x04,0x51,0x20,0x01,0x00};
    lcd_cmd_data(0xD6, D6, sizeof(D6));
    static const uint8_t D7[] = {0x0A,0x08,0x0E,0x0C,0x1E,0x18,0x19,0x1F,0x00,0x1F,0x1A,0x1F,0x3E,0x3E,0x04,0x00,0x1F,0x1F,0x1F};
    lcd_cmd_data(0xD7, D7, sizeof(D7));
    static const uint8_t D8[] = {0x0B,0x09,0x0F,0x0D,0x1E,0x18,0x19,0x1F,0x01,0x1F,0x1A,0x1F};
    lcd_cmd_data(0xD8, D8, sizeof(D8));
    static const uint8_t D9[] = {0x00,0x0D,0x0F,0x09,0x0B,0x1F,0x18,0x19,0x1F,0x01,0x1E,0x1A,0x1F};
    lcd_cmd_data(0xD9, D9, sizeof(D9));
    static const uint8_t DD[] = {0x0C,0x0E,0x08,0x0A,0x1F,0x18,0x19,0x1F,0x00,0x1E,0x1A,0x1F};
    lcd_cmd_data(0xDD, DD, sizeof(DD));
    static const uint8_t DF[] = {0x44,0x73,0x4B,0x69,0x00,0x0A,0x02,0x90};
    lcd_cmd_data(0xDF, DF, sizeof(DF));
    static const uint8_t E0[] = {0x19,0x20,0x0A,0x13,0x0E,0x09,0x12,0x28,0xD4,0x24,0x0C,0x35,0x13,0x31,0x36,0x2F,0x03};
    lcd_cmd_data(0xE0, E0, sizeof(E0));
    static const uint8_t E1[] = {0x38,0x20,0x09,0x12,0x0E,0x08,0x12,0x28,0xC5,0x24,0x0C,0x34,0x12,0x31,0x36,0x2F,0x27};
    lcd_cmd_data(0xE1, E1, sizeof(E1));
    static const uint8_t E2[] = {0x19,0x20,0x0A,0x11,0x09,0x06,0x11,0x25,0xD4,0x22,0x0B,0x33,0x12,0x2D,0x32,0x2F,0x03};
    lcd_cmd_data(0xE2, E2, sizeof(E2));
    static const uint8_t E3[] = {0x38,0x20,0x0A,0x11,0x09,0x06,0x11,0x25,0xC4,0x21,0x0A,0x32,0x11,0x2C,0x32,0x2F,0x27};
    lcd_cmd_data(0xE3, E3, sizeof(E3));
    static const uint8_t E4[] = {0x19,0x20,0x0D,0x14,0x0D,0x08,0x12,0x2A,0xD4,0x26,0x0E,0x35,0x13,0x34,0x39,0x2F,0x03};
    lcd_cmd_data(0xE4, E4, sizeof(E4));
    static const uint8_t E5[] = {0x38,0x20,0x0D,0x13,0x0D,0x07,0x12,0x29,0xC4,0x25,0x0D,0x35,0x12,0x33,0x39,0x2F,0x27};
    lcd_cmd_data(0xE5, E5, sizeof(E5));
    static const uint8_t BB2[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    lcd_cmd_data(0xBB, BB2, sizeof(BB2));

    lcd_cmd(0x13);          /* Normal Display Mode on */
    lcd_cmd(0x11);          /* Sleep Out */
    delay_ms(200);

    /* RGB565 pixel format */
    static const uint8_t colmod[] = {0x55};
    lcd_cmd_data(0x3A, colmod, 1);

    /* MADCTL: native portrait, no rotation (0x00) */
    static const uint8_t madctl[] = {0x00};
    lcd_cmd_data(0x36, madctl, 1);

    lcd_cmd(0x29);          /* Display on */
    delay_ms(200);
}

/* ======== Public API ======== */

void lcd_init(void)
{
    printf("lcd_init1\n");
    spi3_gpio_setup();
    printf("lcd_init2\n");
    spi3_periph_setup();
    printf("lcd_init3\n");

    /* Hardware reset sequence */
    GPIO_OUT_W1TS_REG = 1u << LCD_PIN_RST;  delay_ms(10);
    GPIO_OUT_W1TC_REG = 1u << LCD_PIN_RST;  delay_ms(30);
    GPIO_OUT_W1TS_REG = 1u << LCD_PIN_RST;  delay_ms(120);

    printf("lcd_init4\n");
    lcd_vendor_init();
    printf("lcd_init5\n");
}

/*
 * Blit a pixel buffer to the display.
 * `fb` must point to LCD_WIDTH_NATIVE × LCD_HEIGHT_NATIVE pixels in RGB565 big-endian.
 * Layout: fb[row * LCD_WIDTH_NATIVE + col], row=0 top, col=0 left.
 * Internally sends CASET(0, LCD_WIDTH_NATIVE-1) + RAMWR + all bytes.
 */
void lcd_blit_frame(const uint16_t *fb)
{
    /* CASET: all columns 0..171 */
    static const uint8_t caset[] = {0x00, 0x00, 0x00, (uint8_t)(LCD_WIDTH_NATIVE - 1)};
    lcd_cmd_data(0x2A, caset, 4);

    /* RAMWR opcode=0x32, cmd=0x2C */
    uint8_t hdr[4] = {0x32, 0x2C, 0x00, 0x00};

    uint32_t total = (uint32_t)LCD_WIDTH_NATIVE * LCD_HEIGHT_NATIVE * 2u;
    const uint8_t *p = (const uint8_t *)fb;

    cs_lo();
    /* First chunk includes RAMWR header (4 bytes) + up to 60 bytes of pixel data */
    {
        uint8_t first[64];
        first[0] = 0x32; first[1] = 0x2C; first[2] = 0x00; first[3] = 0x00;
        uint32_t d = total > 60u ? 60u : total;
        for (uint32_t i = 0; i < d; i++) first[4 + i] = p[i];
        spi_write_quad(first, 4 + d);
        p     += d;
        total -= d;
    }
    /* Remaining: RAMWRC (0x32, 0x3C) + data in 60-byte payloads */
    while (total) {
        uint8_t chunk[64];
        chunk[0] = 0x32; chunk[1] = 0x3C; chunk[2] = 0x00; chunk[3] = 0x00;
        uint32_t d = total > 60u ? 60u : total;
        for (uint32_t i = 0; i < d; i++) chunk[4 + i] = p[i];
        spi_write_quad(chunk, 4 + d);
        p     += d;
        total -= d;
    }
    cs_hi();
    (void)hdr;
}

/* Fill the display with a solid color (no framebuffer needed). */
void lcd_fill(uint16_t color)
{
    static const uint8_t caset[] = {0x00, 0x00, 0x00, (uint8_t)(LCD_WIDTH_NATIVE - 1)};
    lcd_cmd_data(0x2A, caset, 4);

    uint8_t hi = (uint8_t)(color >> 8), lo = (uint8_t)(color & 0xFF);
    uint32_t total = (uint32_t)LCD_WIDTH_NATIVE * LCD_HEIGHT_NATIVE * 2u;
    uint32_t sent = 0;
    bool first_chunk = true;

    cs_lo();
    while (sent < total) {
        uint8_t buf[64];
        buf[0] = 0x32;
        buf[1] = first_chunk ? 0x2Cu : 0x3Cu;
        buf[2] = 0x00; buf[3] = 0x00;
        first_chunk = false;

        uint32_t dmax = 60u;
        uint32_t drem = total - sent;
        uint32_t d    = drem < dmax ? drem : dmax;
        for (uint32_t i = 0; i < d; i += 2) {
            buf[4 + i]     = hi;
            buf[4 + i + 1] = lo;
        }
        spi_write_quad(buf, 4 + d);
        sent += d;
    }
    cs_hi();
}
