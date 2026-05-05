/*
 * Bare-metal QSPI driver for AXS15231B on Waveshare ESP32-S3-Touch-LCD-3.49.
 *
 * Hardware: AXS15231B = 172×640 portrait LCD + integrated I2C touch.
 *
 * QSPI protocol (matches reference esp_lcd_panel_io_spi.c):
 *   - Command headers: 4-byte word `{0x02, 0x00, mipi_cmd, 0x00}` in SINGLE-WIRE
 *     (only D0/MOSI carries data — opcode 0x02 = "write cmd/param")
 *   - Parameter bytes: SINGLE-WIRE on D0
 *   - Pixel data ONLY: 4-wire QUAD on D0..D3
 *     RAMWR header `{0x32, 0x00, 0x2C, 0x00}` is single-wire, but pixel bytes
 *     after the header are quad. CS held LOW throughout RAMWR + pixel data.
 *
 * SPI3 (GPSPI3, base 0x60025000) at 40 MHz, mode 3 (CPOL=1, CPHA=1).
 * CS driven manually via GPIO so it spans header (single) → data (quad).
 *
 * GPIO pin map (board):
 *   CS=GPIO9, CLK=GPIO10, D0=GPIO11, D1=GPIO12, D2=GPIO13, D3=GPIO14, RST=GPIO21
 *   GPIO8 (LCD_BL) goes to AP3032 *FB* (brightness) via R37 — must NOT be
 *   driven HIGH or it kills the LED. Leave as input. Backlight enable is
 *   via EXIO1 → AP3032 CTRL through the TCA9554 IO expander.
 */

#include "lcd_axs15231b.h"
#include "board.h"
#include "hal_gpio.h"
#include "soc/soc.h"
#include "soc/spi_reg.h"
#include "soc/system_reg.h"
#include "soc/io_mux_reg.h"
#include <stdio.h>

#define SPI_N  3   /* GPSPI3 */

/* ---- Accurate delay using ctx->micros_now ---- */
static lcd_micros_fn_t s_micros = NULL;
static void delay_ms(uint32_t ms)
{
    if (s_micros) {
        uint64_t end = s_micros() + (uint64_t)ms * 1000u;
        while (s_micros() < end) {}
    } else {
        for (volatile uint32_t i = 0; i < ms * 240000u; i++);
    }
}

/* ---- Manual CS ---- */
static inline void cs_lo(void) { GPIO_OUT_W1TC_REG = (1u << LCD_PIN_CS); }
static inline void cs_hi(void) { GPIO_OUT_W1TS_REG = (1u << LCD_PIN_CS); }

/* ---- SPI3 chunk write helpers ----
 *
 * spi_write_single: bytes go on D0 only (1-wire).
 * spi_write_quad:   bytes go on all 4 data lines (quad) — pixel data only.
 *
 * Both chunk through the SPI 64-byte FIFO.  Caller manages CS.
 */

static void spi_write_chunk(uint32_t user_flags, const uint8_t *p, uint32_t chunk)
{
    uint32_t words = (chunk + 3u) >> 2;
    for (uint32_t i = 0; i < words; i++) {
        uint32_t base = i * 4u, w = 0;
        /* SPI peripheral sends bytes from W register in LSB-byte-first order
         * (matches ESP-IDF spi_ll_write_buffer's memcpy on little-endian CPU).
         * So buffer[0] must go to bits[7:0], buffer[3] to bits[31:24]. */
        for (uint32_t j = 0; j < 4u && (base + j) < chunk; j++)
            w |= (uint32_t)p[base + j] << (j * 8u);
        REG_WRITE(SPI_W0_REG(SPI_N) + i * 4u, w);
    }
    REG_WRITE(SPI_MS_DLEN_REG(SPI_N), chunk * 8u - 1u);
    REG_WRITE(SPI_USER_REG(SPI_N), user_flags);
    /* Sync USER/DLEN into SPI clock domain before triggering */
    REG_SET_BIT(SPI_CMD_REG(SPI_N), SPI_UPDATE);
    while (REG_GET_BIT(SPI_CMD_REG(SPI_N), SPI_UPDATE)) {}
    REG_SET_BIT(SPI_CMD_REG(SPI_N), SPI_USR);
    while (REG_GET_BIT(SPI_CMD_REG(SPI_N), SPI_USR)) {}
}

static void spi_write_single(const void *buf, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    while (len) {
        uint32_t chunk = len > 64u ? 64u : len;
        spi_write_chunk(SPI_USR_MOSI | SPI_CS_SETUP | SPI_CS_HOLD, p, chunk);
        p   += chunk;
        len -= chunk;
    }
}

static void spi_write_quad(const void *buf, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    while (len) {
        uint32_t chunk = len > 64u ? 64u : len;
        spi_write_chunk(SPI_USR_MOSI | SPI_FWRITE_QUAD | SPI_CS_SETUP | SPI_CS_HOLD,
                        p, chunk);
        p   += chunk;
        len -= chunk;
    }
}

/* ---- QSPI command helpers ----
 * Match upstream esp_lcd_panel_io_spi behavior: with quad_mode=1, only the
 * pixel-data phase of tx_color goes QUAD; cmd phase, params, and the RAMWR
 * header all go SINGLE-WIRE on D0.
 *
 * lcd_cmd_bits=32, value = (opcode<<24)|(cmd<<8), byte-reversed before the
 * SPI peripheral consumes it → wire bytes {opcode, 0x00, cmd, 0x00} on D0.
 */
static void lcd_cmd(uint8_t cmd)
{
    uint8_t w[4] = {0x02, 0x00, cmd, 0x00};
    cs_lo();
    spi_write_single(w, 4);
    cs_hi();
}

static void lcd_cmd_data(uint8_t cmd, const uint8_t *data, uint32_t dlen)
{
    uint8_t w[4] = {0x02, 0x00, cmd, 0x00};
    cs_lo();
    spi_write_single(w, 4);
    if (dlen) spi_write_single(data, dlen);
    cs_hi();
}

/* ---- SPI3 GPIO routing ----
 * Same signal index for IN and OUT for each pin (peripheral has independent
 * I/O for each role, multiplexed by the GPIO matrix configuration). */
#define SIG_SPI3_CLK 66
#define SIG_SPI3_D   68    /* MOSI / D0 */
#define SIG_SPI3_Q   67    /* MISO / D1 */
#define SIG_SPI3_WP  70    /* D2 */
#define SIG_SPI3_HD  69    /* D3 */

static void spi3_gpio_setup(void)
{
    /* High-drive GPIO-matrix function for SPI bus pins.
     * IDF uses GPIO_MODE_INPUT_OUTPUT (FUN_IE=1) on SPI3 pins — without
     * the input buffer enabled the peripheral does not drive the output. */
    static const int spi_data_pins[] = {
        LCD_PIN_CLK, LCD_PIN_DATA0, LCD_PIN_DATA1, LCD_PIN_DATA2, LCD_PIN_DATA3
    };
    for (int i = 0; i < 5; i++) {
        int p = spi_data_pins[i];
        /* MCU_SEL=1 (bit 12), FUN_DRV=3 (bits 11:10), FUN_IE=1 (bit 9) */
        *iomux_reg(p) = (1u << 12) | (3u << 10) | (1u << 9);
        GPIO_ENABLE_W1TS_REG = 1u << p;
    }
    /* Route SPI3 signals to/from the correct pins.  IDF connects BOTH input
     * and output signals to each SPI pin (even output-only pins like MOSI),
     * and uses GPIO_MODE_INPUT_OUTPUT — without the input wiring back, the
     * peripheral's output driver may not be enabled.
     * IMPORTANT: SPI3_WP=70 is D2 (bit 2 of QSPI nibble), SPI3_HD=69 is D3
     * (bit 3 = MSB).  Board has GPIO13=D2, GPIO14=D3 → WP→GPIO13, HD→GPIO14. */
    gpio_matrix_out(LCD_PIN_CLK,   SIG_SPI3_CLK, false);
    gpio_matrix_in (LCD_PIN_CLK,   SIG_SPI3_CLK, false);
    gpio_matrix_out(LCD_PIN_DATA0, SIG_SPI3_D,   false);
    gpio_matrix_in (LCD_PIN_DATA0, SIG_SPI3_D,   false);
    gpio_matrix_out(LCD_PIN_DATA1, SIG_SPI3_Q,   false);
    gpio_matrix_in (LCD_PIN_DATA1, SIG_SPI3_Q,   false);
    gpio_matrix_out(LCD_PIN_DATA2, SIG_SPI3_WP,  false);
    gpio_matrix_in (LCD_PIN_DATA2, SIG_SPI3_WP,  false);
    gpio_matrix_out(LCD_PIN_DATA3, SIG_SPI3_HD,  false);
    gpio_matrix_in (LCD_PIN_DATA3, SIG_SPI3_HD,  false);

    /* CS, RST as plain GPIO outputs.  GPIO8 (LCD_BL) is INTENTIONALLY left
     * as input — driving it HIGH overrides AP3032 FB and kills the backlight. */
    gpio_output_init(LCD_PIN_CS);
    gpio_output_init(LCD_PIN_RST);
    cs_hi();
    GPIO_OUT_W1TS_REG = (1u << LCD_PIN_RST);
}

/* ---- SPI3 peripheral setup ---- */
static void spi3_periph_setup(void)
{
    /* Enable system clock + clean reset of SPI3 */
    REG_SET_BIT(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_SPI3_CLK_EN);
    REG_SET_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_SPI3_RST);
    for (volatile int i = 0; i < 200; i++);
    REG_CLR_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_SPI3_RST);

    /* Master mode (slave register cleared) */
    REG_WRITE(SPI_SLAVE_REG(SPI_N), 0);

    /* Internal clock gate: register-bus clock + master functional clock + APB source.
     * Without this, SPI_USR never clears (state machine has no clock). */
    REG_WRITE(SPI_CLK_GATE_REG(SPI_N),
              SPI_CLK_EN | SPI_MST_CLK_ACTIVE | SPI_MST_CLK_SEL);

    /* SPI clock = APB / 8 = 10 MHz (slow for debug; spec allows up to 40MHz)
     * CLKCNT_N=7, CLKCNT_H=3, CLKCNT_L=7 — 50% duty cycle. */
    REG_WRITE(SPI_CLOCK_REG(SPI_N), (7u << 12) | (3u << 6) | (7u << 0));

    /* CTRL: must use SET_BIT (not WRITE) so SPI_D_POL (bit19) and SPI_Q_POL (bit18)
     * keep their default=1 — clearing them inverts D0/D1 data on the wire. */
    REG_SET_BIT(SPI_CTRL_REG(SPI_N),
                SPI_FREAD_QUAD | SPI_HOLD_POL | SPI_WP_POL);

    /* Disable hardware-managed CS (we drive CS manually via GPIO9) */
    REG_SET_BIT(SPI_MISC_REG(SPI_N), SPI_CS0_DIS | SPI_CS1_DIS);

    /* SPI mode 0: CK_IDLE_EDGE=0 (clock idles LOW), CK_OUT_EDGE=0 (default).
     * Try mode 0 — many panels labeled "mode 3" actually accept mode 0 too. */
    REG_CLR_BIT(SPI_MISC_REG(SPI_N), SPI_CK_IDLE_EDGE);

    /* Sync register changes into SPI clock domain */
    REG_SET_BIT(SPI_CMD_REG(SPI_N), SPI_UPDATE);
    while (REG_GET_BIT(SPI_CMD_REG(SPI_N), SPI_UPDATE)) {}
}

/* ---- Init sequence ----
 * Uses vendor_specific_init_default values from upstream esp_lcd_axs15231b —
 * the only set empirically observed to make the power-on static fade on this
 * Waveshare panel (= IC accepting the QSPI command frames). The test_apps
 * QSPI custom values do NOT make the static fade.
 */
static void lcd_panel_init(void)
{
    /* Software reset first — guarantees identical IC state on cold and warm
     * boot (panel chip retains state across CPU soft-reset since it has its
     * own VCC).  Spec requires 5ms after SWRESET before next command. */
    lcd_cmd(0x01);                                  /* SWRESET */
    delay_ms(150);

    /* Built-in prologue */
    lcd_cmd(0x11);                                  /* SLPOUT */
    delay_ms(120);
    static const uint8_t madctl[] = {0x00};
    lcd_cmd_data(0x36, madctl, 1);                  /* MADCTL portrait */
    static const uint8_t colmod[] = {0x55};
    lcd_cmd_data(0x3A, colmod, 1);                  /* COLMOD RGB565 */

    /* ---- vendor_specific_init_default proprietary calibration ---- */
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

    lcd_cmd(0x13);                                  /* NORON */
    lcd_cmd(0x11);                                  /* SLPOUT */
    delay_ms(120);

    /* Dummy RAMWR with 4 zero bytes — present in upstream init array,
     * appears to "prime" the RAMWR state for QSPI mode */
    static const uint8_t ramwr_dummy[] = {0x00,0x00,0x00,0x00};
    lcd_cmd_data(0x2C, ramwr_dummy, sizeof(ramwr_dummy));

    lcd_cmd(0x29);                                  /* DISPON */
    delay_ms(120);
}

/* ======== Public API ======== */

void lcd_init(lcd_micros_fn_t micros_now)
{
    s_micros = micros_now;

    printf("lcd_init: spi3 setup\r\n");
    spi3_gpio_setup();
    spi3_periph_setup();

    /* Hardware reset.  On cold-power boot the panel internal rails take
     * longer to settle, so we use a generous post-reset wait (500ms).  On
     * warm soft-reset this is overkill but harmless. */
    GPIO_OUT_W1TS_REG = 1u << LCD_PIN_RST;  delay_ms(50);
    GPIO_OUT_W1TC_REG = 1u << LCD_PIN_RST;  delay_ms(250);
    GPIO_OUT_W1TS_REG = 1u << LCD_PIN_RST;  delay_ms(500);
    printf("lcd_init: hw reset done\r\n");

    lcd_panel_init();
    printf("lcd_init: panel init done\r\n");
}

/*
 * Blit a pixel buffer.  fb = LCD_WIDTH_NATIVE × LCD_HEIGHT_NATIVE pixels in
 * RGB565 big-endian (high byte first on wire).
 *
 * Critical: RAMWR header (single-wire) and ALL pixel data (quad) must be
 * sent under a SINGLE CS assertion — toggling CS resets the IC's write
 * pointer.
 */
/* Chunk size matches the reference LVGL flush: 172 cols × 64 rows × 2 bytes */
#define LCD_CHUNK_ROWS   64
#define LCD_CHUNK_BYTES  (LCD_WIDTH_NATIVE * LCD_CHUNK_ROWS * 2)

/* Send one chunk of pixel data using RAMWR (first) or RAMWRC (subsequent),
 * matching panel_axs15231b_draw_bitmap behaviour:
 *   CASET (every chunk) → RAMWR/RAMWRC + chunk_bytes (single CS for hdr+data)
 */
static void lcd_chunk(const uint8_t *p, uint32_t bytes, bool first)
{
    /* CASET — same range every chunk, like the reference */
    static const uint8_t caset[] = {0x00, 0x00, 0x00, (uint8_t)(LCD_WIDTH_NATIVE - 1)};
    lcd_cmd_data(0x2A, caset, 4);

    /* RAMWR header: SINGLE-WIRE on D0 — matches upstream panel_io_spi which
     * sends the cmd phase of tx_color in 1-bit mode even when quad_mode=1.
     * Pixel data follows in QUAD on D0..D3, single CS spans both. */
    uint8_t hdr[4] = {0x32, 0x00, first ? 0x2Cu : 0x3Cu, 0x00};
    cs_lo();
    spi_write_single(hdr, 4);
    spi_write_quad(p, bytes);
    cs_hi();
}

void lcd_blit_frame(const uint16_t *fb)
{
    const uint8_t *p = (const uint8_t *)fb;
    uint32_t total = (uint32_t)LCD_WIDTH_NATIVE * LCD_HEIGHT_NATIVE * 2u;
    bool first = true;
    while (total) {
        uint32_t d = total > LCD_CHUNK_BYTES ? LCD_CHUNK_BYTES : total;
        lcd_chunk(p, d, first);
        first = false;
        p     += d;
        total -= d;
    }
}

void lcd_fill(uint16_t color)
{
    uint8_t hi = (uint8_t)(color >> 8), lo = (uint8_t)(color & 0xFF);
    uint32_t total = (uint32_t)LCD_WIDTH_NATIVE * LCD_HEIGHT_NATIVE * 2u;
    bool first = true;

    /* Use a small static chunk buffer pre-filled with the colour */
    static uint8_t buf[LCD_CHUNK_BYTES];
    for (uint32_t i = 0; i < LCD_CHUNK_BYTES; i += 2) { buf[i] = hi; buf[i+1] = lo; }

    while (total) {
        uint32_t d = total > LCD_CHUNK_BYTES ? LCD_CHUNK_BYTES : total;
        lcd_chunk(buf, d, first);
        first = false;
        total -= d;
    }
}
