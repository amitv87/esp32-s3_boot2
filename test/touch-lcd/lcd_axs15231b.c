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

__attribute__((unused))
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

/* ---- GDMA channel 0 OUT — DMA pixel data into SPI3 TX ----
 * Channel 0 OUT is dedicated to SPI3.  Descriptor chain points at framebuffer
 * pages (≤4080 bytes each) in PSRAM; cache is flushed before kickoff so DMA
 * sees the latest CPU writes. */
#define GDMA_BASE              0x6003F000UL
#define GDMA_OUT_CONF0_CH0     (*(volatile uint32_t *)(GDMA_BASE + 0x60))
#define GDMA_OUT_CONF1_CH0     (*(volatile uint32_t *)(GDMA_BASE + 0x64))
#define GDMA_OUT_INT_RAW_CH0   (*(volatile uint32_t *)(GDMA_BASE + 0x68))
#define GDMA_OUT_INT_CLR_CH0   (*(volatile uint32_t *)(GDMA_BASE + 0x74))
#define GDMA_OUT_LINK_CH0      (*(volatile uint32_t *)(GDMA_BASE + 0x80))
#define GDMA_OUT_PERI_SEL_CH0  (*(volatile uint32_t *)(GDMA_BASE + 0xA8))
#define GDMA_MISC_CONF         (*(volatile uint32_t *)(GDMA_BASE + 0x3C8))

#define GDMA_BIT_OUT_RST           (1u << 0)
#define GDMA_BIT_OUT_EOF_MODE      (1u << 3)
#define GDMA_BIT_OUTDSCR_BURST_EN  (1u << 4)
#define GDMA_BIT_OUT_DATA_BURST_EN (1u << 5)
#define GDMA_BIT_GDMA_CLK_EN       (1u << 4)
#define GDMA_BIT_OUTLINK_START     (1u << 21)
#define GDMA_BIT_OUT_DONE_INT      (1u << 0)
#define GDMA_BIT_OUT_TOTAL_EOF_INT (1u << 3)

typedef struct dma_desc {
    uint32_t dw0;   /* size:12 | length:12 | rsvd:6 | suc_eof:1 | owner:1 */
    uint32_t buf;
    uint32_t next;
} dma_desc_t;

/* Full-frame path uses contiguous descriptors (≤4080 bytes each); partial
 * lcd_blit uses one descriptor per row so it can stride over a wider
 * surface.
 *
 * Pool size cap of 8 is empirical: bumping past 8 (i.e. extending the pool
 * past 0x3FCD00C0) regresses the full-frame color path to the pre-FIFO-prime
 * shift symptoms (red→pink, green→light-green, blue→near-black).  Something
 * the ROM or boot2 latently relies on lives above 0x3FCD00C0 — to grow the
 * pool we'd need to relocate it (e.g. 0x3FCC0000, well below boot2's
 * 0x3FCD8700 IRAM start). */
#define DMA_BYTES_PER_DESC  4080u
#define DMA_DESC_POOL       8
/* Per-chunk row cap for lcd_blit — also limited by MS_DLEN's 32 KB. */
#define LCD_BLIT_CHUNK_ROWS 8u

/* GDMA's OUT_LINK_ADDR field is 20 bits — the peripheral infers upper bits
 * from the address space and expects descriptors in INTERNAL SRAM (matches
 * IDF's MALLOC_CAP_DMA|MALLOC_CAP_INTERNAL constraint).  Our app's BSS lives
 * in PSRAM, so we can't just declare the pool as a static — the GDMA would
 * read the wrong region.  Park it at a fixed unused-SRAM address instead.
 *
 * boot2 occupies SRAM1 from 0x3FCD8700 (iram_seg, IRAM-view 0x403C8700) up
 * through the stack at 0x3FCE9700.  0x3FCD0000 is in the region below that,
 * but only the first ~192 bytes are safe — pool sizes >8 (= 96 bytes used,
 * but 0x3FCD00C0+ corrupts something) regress the full-frame color path.
 * To grow past 8 descriptors, relocate the pool to e.g. 0x3FCC0000. */
#define DMA_DESC_SRAM_ADDR  0x3FCD0000u
static dma_desc_t * const s_dma_descs = (dma_desc_t *)DMA_DESC_SRAM_ADDR;

/* ROM cache flush — CPU writes to PSRAM sit in the L1 cache; force them out
 * to memory so the GDMA peripheral (which reads PSRAM directly) sees them.
 * Direct ROM-symbol address avoids needing to add the ROM linker scripts to
 * the test app's link line. */
typedef int (*cache_writeback_fn)(uint32_t addr, uint32_t size);
static const cache_writeback_fn Cache_WriteBack_Addr =
    (cache_writeback_fn)0x400016c8;

static void gdma_init(void)
{
    /* System-level: enable DMA peripheral clock + reset cycle */
    REG_SET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN);
    REG_SET_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_DMA_RST);
    for (volatile int i = 0; i < 200; i++);
    REG_CLR_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_DMA_RST);

    /* GDMA's internal clock gate */
    GDMA_MISC_CONF |= GDMA_BIT_GDMA_CLK_EN;

    /* Reset OUT channel 0 then configure for descriptor + data burst mode.
     * OUT_EOF_MODE deliberately CLEARED — with mode=1 (popped-from-FIFO),
     * GDMA can over-fetch past MS_DLEN and leak bytes into the next chunk. */
    GDMA_OUT_CONF0_CH0 = GDMA_BIT_OUT_RST;
    GDMA_OUT_CONF0_CH0 = 0;
    GDMA_OUT_CONF0_CH0 = GDMA_BIT_OUT_DATA_BURST_EN |
                         GDMA_BIT_OUTDSCR_BURST_EN;
    GDMA_OUT_CONF1_CH0 = 0;             /* default 16-byte PSRAM block size */

    /* Route TX channel to SPI3 (peri index 1 — SPI2=0, SPI3=1) */
    GDMA_OUT_PERI_SEL_CH0 = 1;

    GDMA_OUT_INT_CLR_CH0 = 0xFFFFFFFFu;
}

/* IDF-style two-phase: caller sends the 4-byte header {opcode, 0x00, mipi_cmd,
 * 0x00} single-wire under CS-low (via spi_write_chunk), then this function
 * sends the pixel bytes — pure DATA phase, QUAD MOSI, via DMA. */

/* Send a pre-built s_dma_descs[] chain of total_bytes via SPI3 QUAD MOSI.
 * Caller is responsible for filling descriptors and CS framing. */
static void spi_dma_send_chain(uint32_t total_bytes)
{
    /* GDMA reset + outlink load — match NuttX's esp32s3_dma_load: pulse
     * OUT_RST, then clear+set the OUTLINK_ADDR field as separate writes
     * BEFORE the START bit. */
    GDMA_OUT_CONF0_CH0 |= GDMA_BIT_OUT_RST;
    GDMA_OUT_CONF0_CH0 &= ~GDMA_BIT_OUT_RST;

    GDMA_OUT_LINK_CH0 = GDMA_OUT_LINK_CH0 & ~0xFFFFFu;
    GDMA_OUT_LINK_CH0 = (GDMA_OUT_LINK_CH0 & ~0xFFFFFu) |
                       ((uint32_t)&s_dma_descs[0] & 0xFFFFFu);

    REG_SET_BIT(SPI_DMA_CONF_REG(SPI_N),
                SPI_DMA_AFIFO_RST | SPI_BUF_AFIFO_RST | SPI_RX_AFIFO_RST);
    REG_SET_BIT(SPI_DMA_CONF_REG(SPI_N), SPI_DMA_TX_ENA);

    REG_WRITE(SPI_USER2_REG(SPI_N), 0);
    REG_WRITE(SPI_USER1_REG(SPI_N), 0);
    REG_WRITE(SPI_MS_DLEN_REG(SPI_N), total_bytes * 8u - 1u);
    REG_WRITE(SPI_USER_REG(SPI_N),
              SPI_USR_MOSI | SPI_FWRITE_QUAD |
              SPI_CS_SETUP | SPI_CS_HOLD);

    uint32_t ctrl = REG_READ(SPI_CTRL_REG(SPI_N));
    ctrl &= ~(SPI_FCMD_DUAL | SPI_FCMD_QUAD |
              SPI_FADDR_DUAL | SPI_FADDR_QUAD |
              SPI_DUMMY_OUT);
    REG_WRITE(SPI_CTRL_REG(SPI_N), ctrl);

    REG_SET_BIT(SPI_CMD_REG(SPI_N), SPI_UPDATE);
    while (REG_GET_BIT(SPI_CMD_REG(SPI_N), SPI_UPDATE)) {}

    REG_WRITE(SPI_DMA_INT_CLR_REG(SPI_N), SPI_TRANS_DONE_INT_CLR);
    GDMA_OUT_INT_CLR_CH0 = 0xFFFFFFFFu;
    GDMA_OUT_LINK_CH0 |= GDMA_BIT_OUTLINK_START;

    /* Wait for DMA to prime the SPI TX FIFO before SPI_USR — otherwise
     * SPI starts the data phase before the FIFO has data and outputs zero
     * for the first cycle (4-bit forward shift on the wire). */
    while (REG_READ(SPI_DMA_CONF_REG(SPI_N)) & SPI_DMA_OUTFIFO_EMPTY) {}

    REG_SET_BIT(SPI_CMD_REG(SPI_N), SPI_USR);
    while (!(REG_READ(SPI_DMA_INT_RAW_REG(SPI_N)) & SPI_TRANS_DONE_INT_RAW)) {}

    REG_CLR_BIT(SPI_DMA_CONF_REG(SPI_N), SPI_DMA_TX_ENA);
    REG_WRITE(SPI_USER_REG(SPI_N), 0);
}

static void spi_write_quad_dma(const void *buf, uint32_t len)
{
    /* Contiguous descriptor chain (each up to DMA_BYTES_PER_DESC). */
    uint32_t a = (uint32_t)buf;
    uint32_t remaining = len;
    int n = 0;
    while (remaining > 0 && n < DMA_DESC_POOL) {
        uint32_t bytes = remaining > DMA_BYTES_PER_DESC ? DMA_BYTES_PER_DESC
                                                        : remaining;
        bool eof = (remaining == bytes);
        s_dma_descs[n].dw0 = (uint32_t)bytes |
                             ((uint32_t)bytes << 12) |
                             (eof ? (1u << 30) : 0u) |
                             (1u << 31);
        s_dma_descs[n].buf  = a;
        s_dma_descs[n].next = eof ? 0u : (uint32_t)&s_dma_descs[n + 1];
        a += bytes;
        remaining -= bytes;
        n++;
    }
    spi_dma_send_chain(len);
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

#if 0
    /* SPI clock = APB / 1 = 80 MHz.  SPI_CLK_EQU_SYSCLK (bit 31) bypasses
     * the divider entirely.  All other CLOCK_REG bits must be 0.  The panel
     * boots fine at this rate but the rightmost columns show garbage from
     * intermittent bit errors at the end of each row chunk — the AXS15231B
     * is rated for 40 MHz max. */
    REG_WRITE(SPI_CLOCK_REG(SPI_N), (1u << 31));
#else
    /* SPI clock = APB / 8 = 10 MHz.  The cmd+addr+data DMA transaction
     * streams pixels continuously (no per-sub-chunk idle like the CPU path
     * had), so we drop to 10 MHz to give the panel plenty of commit margin.
     * CLKCNT_N=7, CLKCNT_H=3, CLKCNT_L=7 — 50% duty. */
    REG_WRITE(SPI_CLOCK_REG(SPI_N), (7u << 12) | (3u << 6) | (7u << 0));
#endif

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

    /* GDMA channel 0 OUT → SPI3 TX, used for pixel-data writes */
    gdma_init();
}

/* ---- Init sequence ----
 * Uses vendor_specific_init_default values from upstream esp_lcd_axs15231b —
 * the only set empirically observed to make the power-on static fade on this
 * Waveshare panel (= IC accepting the QSPI command frames). The test_apps
 * QSPI custom values do NOT make the static fade.
 */
static void lcd_panel_init(void)
{
    lcd_cmd(0x01);                                  /* SWRESET */
    delay_ms(200);

    /* Built-in prologue */
    lcd_cmd(0x11);                                  /* SLPOUT */
    delay_ms(150);
    static const uint8_t madctl[] = {0x00};
    lcd_cmd_data(0x36, madctl, 1);                  /* MADCTL portrait */
    static const uint8_t colmod[] = {0x55};
    lcd_cmd_data(0x3A, colmod, 1);                  /* COLMOD RGB565 */

    /* Gamma-only proprietary calibration.  C-series (VCOM/source driver) and
     * D-series (gate driver) commands break touch and/or cause every-other-
     * boot toggle.  E0-E5 (gamma curves) are the only safe addition that
     * helps low-intensity color rendering.
     * 0xBB is the calibration unlock key, paired with a "lock" 0xBB at the end. */
    static const uint8_t BB1[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x5A,0xA5};
    lcd_cmd_data(0xBB, BB1, sizeof(BB1));
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
    lcd_cmd(0x29);                                  /* DISPON */
    delay_ms(120);
}

/* ======== Public API ======== */

/* QSPI read: send (0x0B, 0x00, cmd, 0x00) single-wire on D0, then read
 * `nbytes` back single-wire on D1 (MISO).  Datasheet section 4.4.2.
 * Used for diagnostic — compares panel-reported IDs against datasheet
 * defaults to verify the wire transport. */
static uint8_t lcd_read1(uint8_t cmd)
{
    /* Disable DMA and clear any leftover cmd/addr/dummy phase config from a
     * previous DMA transaction. */
    REG_CLR_BIT(SPI_DMA_CONF_REG(SPI_N), SPI_DMA_TX_ENA | SPI_DMA_RX_ENA);
    REG_WRITE(SPI_USER1_REG(SPI_N), 0);
    REG_WRITE(SPI_USER2_REG(SPI_N), 0);

    /* Pack 4 bytes (0x0B, 0x00, cmd, 0x00) into W0 — LSB-byte-first packing
     * matches the SPI peripheral's read order. */
    uint32_t w0 = (uint32_t)0x0B | (0u << 8) | ((uint32_t)cmd << 16) | (0u << 24);
    REG_WRITE(SPI_W0_REG(SPI_N), w0);

    /* MOSI 32 bits single-wire, then 8 bits MISO single-wire (no FREAD_QUAD). */
    REG_WRITE(SPI_MS_DLEN_REG(SPI_N), 32u + 8u - 1u);
    REG_WRITE(SPI_USER_REG(SPI_N),
              SPI_USR_MOSI | SPI_USR_MISO);

    /* CTRL: ensure all FREAD/FCMD/FADDR quad bits are clear */
    uint32_t ctrl = REG_READ(SPI_CTRL_REG(SPI_N));
    ctrl &= ~(SPI_FCMD_DUAL | SPI_FCMD_QUAD |
              SPI_FADDR_DUAL | SPI_FADDR_QUAD |
              SPI_FREAD_DUAL | SPI_FREAD_QUAD);
    REG_WRITE(SPI_CTRL_REG(SPI_N), ctrl);

    REG_SET_BIT(SPI_CMD_REG(SPI_N), SPI_UPDATE);
    while (REG_GET_BIT(SPI_CMD_REG(SPI_N), SPI_UPDATE)) {}

    cs_lo();
    REG_SET_BIT(SPI_CMD_REG(SPI_N), SPI_USR);
    while (REG_GET_BIT(SPI_CMD_REG(SPI_N), SPI_USR)) {}
    cs_hi();

    /* MISO data lands in W register at bit position right after MOSI bits.
     * For 32-bit MOSI + 8-bit MISO, the 8 RX bits are packed into the LOWER
     * byte of W0 (the receive side puts data starting at bit 0). */
    uint32_t r = REG_READ(SPI_W0_REG(SPI_N));
    return (uint8_t)(r & 0xFFu);
}

void lcd_read_id(void)
{
    uint8_t id1 = lcd_read1(0xDA);
    uint8_t id2 = lcd_read1(0xDB);
    uint8_t id3 = lcd_read1(0xDC);
    printf("LCD IDs: DA=0x%02X (expect 0x40), DB=0x%02X (expect 0x00), "
           "DC=0x%02X (expect 0x03)\r\n", id1, id2, id3);
}

void lcd_init(lcd_micros_fn_t micros_now)
{
    s_micros = micros_now;

    printf("lcd_init: spi3 setup\r\n");
    spi3_gpio_setup();
    spi3_periph_setup();

    /* Hardware reset.  Long pulses to guarantee a full IC reset regardless
     * of panel's prior state (DISPON+RAMWR after a previous run vs. fresh
     * power-up vs. mid-init from a faulty boot — all three must converge
     * to the same default state). */
    GPIO_OUT_W1TS_REG = 1u << LCD_PIN_RST;  delay_ms(50);
    GPIO_OUT_W1TC_REG = 1u << LCD_PIN_RST;  delay_ms(500);
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
/* Chunk size constraints:
 *   - SPI_MS_DATA_BITLEN is 18-bit → max 32 KB per SPI transaction.
 *   - Each GDMA descriptor holds ≤ 4080 bytes (12-bit size field).
 *   - Per-chunk DMA setup introduces a tiny consistent error that
 *     accumulates into a y-offset, so larger chunks (= fewer transitions
 *     per frame) minimize total drift.
 * 80 rows × 172 cols × 2 bytes = 27520 bytes — fits in MS_DLEN, uses 7
 * descriptors, and gives 8 chunks per frame (vs 64 with 10-row chunks). */
#define LCD_CHUNK_ROWS   80
#define LCD_CHUNK_BYTES  (LCD_WIDTH_NATIVE * LCD_CHUNK_ROWS * 2)

/* Send one chunk of pixel data.  CASET each chunk; RAMWR for the first chunk
 * (which resets the cursor to RASET_start), RAMWRC for subsequent chunks.
 * RASET is sent ONCE per frame from lcd_blit_frame, not per chunk. */
static void lcd_chunk(const uint8_t *p, uint32_t bytes, bool first)
{
    /* CASET — same range every chunk, like the reference */
    static const uint8_t caset[] = {0x00, 0x00, 0x00, (uint8_t)(LCD_WIDTH_NATIVE - 1)};
    lcd_cmd_data(0x2A, caset, 4);

    /* IDF-style two-phase under one CS:
     *   1) 4-byte header {0x32, 0x00, RAMWR/RAMWRC, 0x00} single-wire (CPU)
     *   2) pixel data in QUAD via DMA
     * Both with CS_SETUP|CS_HOLD so timing matches the proven CPU-only path. */
    uint8_t hdr[4] = {0x32, 0x00, first ? 0x2Cu : 0x3Cu, 0x00};
    cs_lo();
    spi_write_chunk(SPI_USR_MOSI | SPI_CS_SETUP | SPI_CS_HOLD, hdr, 4);
    spi_write_quad_dma(p, bytes);
    cs_hi();
}

void lcd_blit_frame(const uint16_t *fb)
{
    const uint8_t *p = (const uint8_t *)fb;
    uint32_t total = (uint32_t)LCD_WIDTH_NATIVE * LCD_HEIGHT_NATIVE * 2u;

    /* Flush the entire framebuffer's L1 cache lines once up front so GDMA
     * sees the latest CPU writes for every chunk.  Cache_WriteBack_Addr has
     * a documented alignment bug — round to 32-byte cache-line bounds. */
    uint32_t flush_addr = (uint32_t)p & ~0x1Fu;
    uint32_t flush_end  = ((uint32_t)p + total + 0x1Fu) & ~0x1Fu;
    Cache_WriteBack_Addr(flush_addr, flush_end - flush_addr);

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

    /* Use a small static chunk buffer pre-filled with the colour. */
    static uint8_t buf[LCD_CHUNK_BYTES] __attribute__((aligned(16)));
    for (uint32_t i = 0; i < LCD_CHUNK_BYTES; i += 2) { buf[i] = hi; buf[i+1] = lo; }

    /* Flush the prefill once — the same buffer is reused for every chunk. */
    Cache_WriteBack_Addr((uint32_t)buf, LCD_CHUNK_BYTES);

    while (total) {
        uint32_t d = total > LCD_CHUNK_BYTES ? LCD_CHUNK_BYTES : total;
        lcd_chunk(buf, d, first);
        first = false;
        total -= d;
    }
}

/* Partial-region blit.  AXS15231B QSPI mode supports CASET (X-window) but
 * RAMWR auto-resets the Y-cursor to 0 — there is no RASET in QSPI mode.
 * To honour an arbitrary y/h, we send rows 0..y+h-1 of the (x..x+w-1)
 * column slice; rows above the AABB come straight from the caller's
 * surface buffer (which already matches the panel's prior state, so
 * over-writing them is a visual no-op).
 *
 * Each row is one GDMA descriptor so we can stride over a wider surface. */
void lcd_blit(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
              uint16_t stride, const uint16_t *buffer)
{
    if (!w || !h) return;

    uint32_t total_rows = (uint32_t)y + h;
    uint32_t row_bytes  = (uint32_t)w * 2u;
    uint32_t stride_b   = (uint32_t)stride * 2u;

    /* Flush from row 0 up through the last AABB row.  Adjacent rows are
     * contiguous, so the flush range covers all rows we'll DMA from. */
    uint32_t fb_lo    = (uint32_t)buffer - (uint32_t)y * stride_b;
    uint32_t fb_hi    = (uint32_t)buffer + (uint32_t)(h - 1) * stride_b + row_bytes;
    uint32_t flush_lo = fb_lo & ~0x1Fu;
    uint32_t flush_hi = (fb_hi + 0x1Fu) & ~0x1Fu;
    Cache_WriteBack_Addr(flush_lo, flush_hi - flush_lo);

    /* CASET — narrow X-window to the dirty columns. */
    uint8_t caset[4] = {(uint8_t)(x >> 8), (uint8_t)x,
                        (uint8_t)((x + w - 1) >> 8), (uint8_t)(x + w - 1)};
    lcd_cmd_data(0x2A, caset, 4);

    /* Per-chunk row cap, bounded by pool size, MS_DLEN's 32 KB, and the
     * compile-time chunk hint. */
    uint32_t chunk_rows = LCD_BLIT_CHUNK_ROWS;
    if (chunk_rows > DMA_DESC_POOL) chunk_rows = DMA_DESC_POOL;
    if (row_bytes > 0 && chunk_rows * row_bytes > 32768u) {
        chunk_rows = 32768u / row_bytes;
    }
    if (chunk_rows == 0) chunk_rows = 1;

    bool first = true;
    uint32_t cur_row = 0;
    while (cur_row < total_rows) {
        uint32_t rows = total_rows - cur_row;
        if (rows > chunk_rows) rows = chunk_rows;

        /* Build per-row descriptor chain.  Row r' (panel row) sources
         * surface[r'][x..x+w-1] = buffer + (r' - y) * stride. */
        for (uint32_t i = 0; i < rows; i++) {
            int32_t row_off = (int32_t)(cur_row + i) - (int32_t)y;
            uint32_t src    = (uint32_t)buffer + (uint32_t)(row_off * (int32_t)stride_b);
            bool eof = (i == rows - 1);
            s_dma_descs[i].dw0  = row_bytes |
                                  (row_bytes << 12) |
                                  (eof ? (1u << 30) : 0u) |
                                  (1u << 31);
            s_dma_descs[i].buf  = src;
            s_dma_descs[i].next = eof ? 0u : (uint32_t)&s_dma_descs[i + 1];
        }

        cs_lo();
        uint8_t hdr[4] = {0x32, 0x00, first ? 0x2Cu : 0x3Cu, 0x00};
        spi_write_chunk(SPI_USR_MOSI | SPI_CS_SETUP | SPI_CS_HOLD, hdr, 4);
        spi_dma_send_chain(rows * row_bytes);
        cs_hi();

        cur_row += rows;
        first = false;
    }
}

void lcd_blit_dma2d(gui_surface_t *surface)
{
    if (!surface || surface->dirty.count == 0 || !surface->buffer) return;

    gui_rect_t aabb = surface->dirty.rects[0];
    for (uint8_t i = 1; i < surface->dirty.count; i++) {
        gui_rect_t *r = &surface->dirty.rects[i];
        if (r->x1 < aabb.x1) aabb.x1 = r->x1;
        if (r->y1 < aabb.y1) aabb.y1 = r->y1;
        if (r->x2 > aabb.x2) aabb.x2 = r->x2;
        if (r->y2 > aabb.y2) aabb.y2 = r->y2;
    }

    uint16_t w = (uint16_t)(aabb.x2 - aabb.x1 + 1);
    uint16_t h = (uint16_t)(aabb.y2 - aabb.y1 + 1);
    uint16_t *buf_start = (uint16_t *)surface->buffer +
                          (uint32_t)aabb.y1 * surface->width + aabb.x1;
    lcd_blit((uint16_t)aabb.x1, (uint16_t)aabb.y1, w, h,
             (uint16_t)surface->width, buf_start);
}
