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

/* 10-row chunks fit in 1 descriptor; 8 covers up to ~32 KB if we ever
 * grow chunks back toward MS_DLEN's 18-bit limit. */
#define DMA_BYTES_PER_DESC  4080u
#define DMA_DESC_POOL       8

/* GDMA's OUT_LINK_ADDR field is 20 bits — the peripheral infers upper bits
 * from the address space and expects descriptors in INTERNAL SRAM (matches
 * IDF's MALLOC_CAP_DMA|MALLOC_CAP_INTERNAL constraint).  Our app's BSS lives
 * in PSRAM, so we can't just declare the pool as a static — the GDMA would
 * read the wrong region.  Park it at a fixed unused-SRAM address instead.
 *
 * boot2 occupies SRAM1 starting at 0x3FCD_1700 (see boot2.ld:
 * iram_seg_start = 0x3FCD_1700 in DRAM view).  0x3FCD_0000 is in the free
 * region just below — 256 bytes is more than enough for 8 × 12-byte descs. */
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

static void spi_write_quad_dma(const void *buf, uint32_t len)
{
    /* Build the descriptor chain.  Each descriptor covers up to 4080 bytes;
     * suc_eof (bit 30) marks the last segment; owner (bit 31) = DMA. */
    uint32_t addr = (uint32_t)buf;
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
        s_dma_descs[n].buf  = addr;
        s_dma_descs[n].next = eof ? 0u : (uint32_t)&s_dma_descs[n + 1];
        addr += bytes;
        remaining -= bytes;
        n++;
    }

    /* Hand SPI's MOSI source from the W register over to the DMA stream.
     * Only reset DMA_AFIFO (matches IDF spi_ll_dma_tx_fifo_reset) — resetting
     * BUF_AFIFO too can drop bytes that the SPI shifter is mid-consuming. */
    REG_SET_BIT(SPI_DMA_CONF_REG(SPI_N), SPI_DMA_AFIFO_RST);
    REG_SET_BIT(SPI_DMA_CONF_REG(SPI_N), SPI_DMA_TX_ENA);

    REG_WRITE(SPI_MS_DLEN_REG(SPI_N), len * 8u - 1u);
    /* USER: just MOSI + QUAD (matches IDF, which doesn't set CS_SETUP/CS_HOLD).
     * Those bits enable cs_setup_time / cs_hold_time extensions which can add
     * extra clock cycles per transaction even when the times are zero. */
    REG_WRITE(SPI_USER_REG(SPI_N), SPI_USR_MOSI | SPI_FWRITE_QUAD);

    REG_SET_BIT(SPI_CMD_REG(SPI_N), SPI_UPDATE);
    while (REG_GET_BIT(SPI_CMD_REG(SPI_N), SPI_UPDATE)) {}

    /* Clear the TRANS_DONE int so we can poll it after kickoff */
    REG_WRITE(SPI_DMA_INT_CLR_REG(SPI_N), SPI_TRANS_DONE_INT_CLR);

    /* Start DMA — load first descriptor address (lower 20 bits) + start bit */
    GDMA_OUT_INT_CLR_CH0 = 0xFFFFFFFFu;
    GDMA_OUT_LINK_CH0 = ((uint32_t)&s_dma_descs[0] & 0xFFFFFu) |
                        GDMA_BIT_OUTLINK_START;

    /* Kick the SPI transaction.  Wait on TRANS_DONE_INT (matches IDF) — the
     * SPI_USR bit clears slightly before the peripheral's MOSI pipeline has
     * fully drained, and disabling DMA_TX_ENA in that window can cut off
     * the last few bytes per chunk and accumulate as a y-offset. */
    REG_SET_BIT(SPI_CMD_REG(SPI_N), SPI_USR);
    while (!(REG_READ(SPI_DMA_INT_RAW_REG(SPI_N)) & SPI_TRANS_DONE_INT_RAW)) {}

    /* Brief settle delay — gives the panel time to commit the chunk's
     * bytes to its internal RAM before CS rises. */
    for (volatile int i = 0; i < 200; i++);

    /* Restore SPI MOSI source to W register for subsequent CPU writes */
    REG_CLR_BIT(SPI_DMA_CONF_REG(SPI_N), SPI_DMA_TX_ENA);
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

    /* Master mode + clean register state (matches Zephyr/IDF spi_ll_master_init).
     * SLAVE, USER, USER1, USER2, ADDR all start zeroed so per-transaction
     * REG_WRITE only needs to set bits it actually wants — no stale
     * usr_command_bitlen, usr_addr_bitlen, usr_dummy_cyclelen, etc. */
    REG_WRITE(SPI_SLAVE_REG(SPI_N), 0);
    REG_WRITE(SPI_USER_REG(SPI_N),  0);
    REG_WRITE(SPI_USER1_REG(SPI_N), 0);
    REG_WRITE(SPI_USER2_REG(SPI_N), 0);
    REG_WRITE(SPI_ADDR_REG(SPI_N),  0);

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
    /* SPI clock = APB / 2 = 40 MHz — the AXS15231B's rated maximum, matches
     * the Waveshare reference (io_config.pclk_hz = 40 MHz).
     * CLKCNT_N=1, CLKCNT_H=0, CLKCNT_L=1 → divide by 2 with 50% duty. */
    REG_WRITE(SPI_CLOCK_REG(SPI_N), (1u << 12) | (1u << 0));
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

    /* Match Zephyr's spi_ll_master_init: zero out dma_conf then set only the
     * SEG_TRANS_CLR enables.  Without `tx_seg_trans_clr_en = 1`, the
     * dma_outfifo_empty_vld signal may stay sticky across transactions and
     * leak bytes into subsequent chunks. */
    REG_WRITE(SPI_DMA_CONF_REG(SPI_N),
              SPI_SLV_TX_SEG_TRANS_CLR_EN | SPI_SLV_RX_SEG_TRANS_CLR_EN);

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
 *   - SPI_MS_DATA_BITLEN is an 18-bit field → max 32 KB per SPI transaction.
 *   - Each GDMA descriptor holds ≤ 4080 bytes (12-bit size field).
 * 10 rows × 172 cols × 2 bytes = 3440 bytes fits in a single descriptor. */
#define LCD_CHUNK_ROWS   10
#define LCD_CHUNK_BYTES  (LCD_WIDTH_NATIVE * LCD_CHUNK_ROWS * 2)

/* Send one chunk of pixel data.  CASET each chunk; RAMWR for the first chunk
 * (which resets the cursor to RASET_start), RAMWRC for subsequent chunks.
 * RASET is sent ONCE per frame from lcd_blit_frame, not per chunk. */
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
#if 0
    /* GDMA channel 0 streams the chunk directly from PSRAM into SPI3 TX.
     * Disabled: produces a consistent ~25-pixel y-offset across the frame
     * that we couldn't pin down without a logic analyzer.  Probable causes
     * tried and ruled out: cache flush alignment, descriptor chaining,
     * MS_DLEN width, EOF mode, BUF_AFIFO reset, FREAD_QUAD, SPI mode 0/3,
     * 80/40/20/10 MHz, RASET window, USER1/USER2 zeroing, seg_trans_clr_en,
     * settle delays, GDMA channel reset.  The CPU path stays in service. */
    spi_write_quad_dma(p, bytes);
#else
    /* CPU walks the 64-byte SPI FIFO in a tight loop — slower but correct. */
    spi_write_quad(p, bytes);
#endif
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
