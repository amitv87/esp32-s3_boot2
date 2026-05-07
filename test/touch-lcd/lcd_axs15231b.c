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
#include <string.h>

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
 * Pool sized 14 to fit two ping-pong chains of 7 descriptors each (one
 * chain per scratch buffer for double-buffered byteswap-DMA pipeline).
 * 14 × 12 = 168 bytes at 0x3FCD0000..0x3FCD00A8 — below the 0x3FCD00C0
 * threshold past which something the ROM relies on lives.  Bumping past 16
 * descriptors would cross that boundary; relocate the pool then. */
#define DMA_BYTES_PER_DESC  4080u
#define DMA_HALF_SIZE       7
#define DMA_DESC_POOL       (DMA_HALF_SIZE * 2)
/* Per-chunk row cap for lcd_blit — also limited by MS_DLEN's 32 KB. */
#define LCD_BLIT_CHUNK_ROWS 7u

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

/* Wait for the in-flight DMA TX (kicked off via spi_dma_kickoff_at) to
 * finish.  Cleans up SPI USER and disables DMA TX on the way out. */
static inline void spi_dma_wait(void)
{
    while (!(REG_READ(SPI_DMA_INT_RAW_REG(SPI_N)) & SPI_TRANS_DONE_INT_RAW)) {}
    REG_CLR_BIT(SPI_DMA_CONF_REG(SPI_N), SPI_DMA_TX_ENA);
    REG_WRITE(SPI_USER_REG(SPI_N), 0);
}

/* Kick off DMA TX of a pre-built descriptor chain rooted at
 * s_dma_descs[desc_base] for `total_bytes` via SPI3 QUAD MOSI.  Returns as
 * soon as SPI_USR is asserted — caller pipelines CPU work for the next
 * chunk while this one streams, then calls spi_dma_wait() before kicking
 * off the next chunk. */
static void spi_dma_kickoff_at(uint32_t desc_base, uint32_t total_bytes)
{
    /* GDMA reset + outlink load — match NuttX's esp32s3_dma_load: pulse
     * OUT_RST, then clear+set the OUTLINK_ADDR field as separate writes
     * BEFORE the START bit. */
    GDMA_OUT_CONF0_CH0 |= GDMA_BIT_OUT_RST;
    GDMA_OUT_CONF0_CH0 &= ~GDMA_BIT_OUT_RST;

    uint32_t addr = (uint32_t)&s_dma_descs[desc_base];
    GDMA_OUT_LINK_CH0 = GDMA_OUT_LINK_CH0 & ~0xFFFFFu;
    GDMA_OUT_LINK_CH0 = (GDMA_OUT_LINK_CH0 & ~0xFFFFFu) | (addr & 0xFFFFFu);

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
}

/* Synchronous send: kick off chain at descs[0..] and block until done.
 * Used by the partial-blit path (lcd_blit) which doesn't pipeline. */
static void spi_dma_send_chain(uint32_t total_bytes)
{
    spi_dma_kickoff_at(0, total_bytes);
    spi_dma_wait();
}

/* Build a contiguous descriptor chain rooted at s_dma_descs[desc_base]
 * pointing at `buf` of `len` bytes.  Each descriptor holds up to
 * DMA_BYTES_PER_DESC; chain is up to DMA_HALF_SIZE descriptors long. */
static void build_dma_chain(uint32_t desc_base, const void *buf, uint32_t len)
{
    uint32_t a = (uint32_t)buf;
    uint32_t remaining = len;
    int n = 0;
    while (remaining > 0 && n < DMA_HALF_SIZE) {
        uint32_t bytes = remaining > DMA_BYTES_PER_DESC ? DMA_BYTES_PER_DESC
                                                        : remaining;
        bool eof = (remaining == bytes);
        s_dma_descs[desc_base + n].dw0 = (uint32_t)bytes |
                                         ((uint32_t)bytes << 12) |
                                         (eof ? (1u << 30) : 0u) |
                                         (1u << 31);
        s_dma_descs[desc_base + n].buf  = a;
        s_dma_descs[desc_base + n].next = eof ? 0u : (uint32_t)&s_dma_descs[desc_base + n + 1];
        a += bytes;
        remaining -= bytes;
        n++;
    }
}

static void spi_write_quad_dma(const void *buf, uint32_t len)
{
    build_dma_chain(0, buf, len);
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
    /* SPI clock = APB / 2 = 40 MHz — panel's rated max.  CLKCNT_N=1,
     * CLKCNT_H=0, CLKCNT_L=1 → 2-cycle period, 1-cycle high (50% duty). */
    REG_WRITE(SPI_CLOCK_REG(SPI_N), (1u << 12) | (0u << 6) | (1u << 0));
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
 *
 * Confirmed: full LilyGO init (with all C/D-series commands) blanks the LCD
 * AND nukes the touch I2C bus on this Waveshare panel, even though it
 * unlocks partial CASET/RASET on the LilyGO board.  The TDDI integrates
 * touch sensing into the source driver, so any C-series command that
 * touches VCOM/source-driver config kills it.  Sticking with the minimal
 * init that keeps both LCD and touch alive — partial windowing stays a
 * "send full-width rows from row 0" workaround in lcd_blit.
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

    #define SEND_LCD_CMD(cmd, data) lcd_cmd_data(cmd, data, sizeof(data))
    #if 0
    #if 0
    SEND_LCD_CMD(0xBB, ((uint8_t[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5A, 0xA5}));
    SEND_LCD_CMD(0xA0, ((uint8_t[]){0x00, 0x10, 0x00, 0x02, 0x00, 0x00, 0x64, 0x3F, 0x20, 0x05, 0x3F, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00}));
    SEND_LCD_CMD(0xA2, ((uint8_t[]){0x30, 0x04, 0x0A, 0x3C, 0xEC, 0x54, 0xC4, 0x30, 0xAC, 0x28, 0x7F, 0x7F, 0x7F, 0x20, 0xF8, 0x10, 0x02, 0xFF, 0xFF, 0xF0, 0x90, 0x01, 0x32, 0xA0, 0x91, 0xC0, 0x20, 0x7F, 0xFF, 0x00, 0x54}));
    SEND_LCD_CMD(0xD0, ((uint8_t[]){0x30, 0xAC, 0x21, 0x24, 0x08, 0x09, 0x10, 0x01, 0xAA, 0x14, 0xC2, 0x00, 0x22, 0x22, 0xAA, 0x03, 0x10, 0x12, 0x40, 0x14, 0x1E, 0x51, 0x15, 0x00, 0x40, 0x10, 0x00, 0x03, 0x3D, 0x12}));
    SEND_LCD_CMD(0xA3, ((uint8_t[]){0xA0, 0x06, 0xAA, 0x08, 0x08, 0x02, 0x0A, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x55, 0x55}));
    SEND_LCD_CMD(0xC1, ((uint8_t[]){0x33, 0x04, 0x02, 0x02, 0x71, 0x05, 0x24, 0x55, 0x02, 0x00, 0x41, 0x00, 0x53, 0xFF, 0xFF, 0xFF, 0x4F, 0x52, 0x00, 0x4F, 0x52, 0x00, 0x45, 0x3B, 0x0B, 0x02, 0x0D, 0x00, 0xFF, 0x40}));
    SEND_LCD_CMD(0xC3, ((uint8_t[]){0x00, 0x00, 0x00, 0x50, 0x03, 0x00, 0x00, 0x00, 0x01, 0x80, 0x01}));
    SEND_LCD_CMD(0xC4, ((uint8_t[]){0x00, 0x24, 0x33, 0x90, 0x50, 0xea, 0x64, 0x32, 0xC8, 0x64, 0xC8, 0x32, 0x90, 0x90, 0x11, 0x06, 0xDC, 0xFA, 0x04, 0x03, 0x80, 0xFE, 0x10, 0x10, 0x00, 0x0A, 0x0A, 0x44, 0x50}));
    SEND_LCD_CMD(0xC5, ((uint8_t[]){0x18, 0x00, 0x00, 0x03, 0xFE, 0x78, 0x33, 0x20, 0x30, 0x10, 0x88, 0xDE, 0x0D, 0x08, 0x0F, 0x0F, 0x01, 0x78, 0x33, 0x20, 0x10, 0x10, 0x80}));
    SEND_LCD_CMD(0xC6, ((uint8_t[]){0x05, 0x0A, 0x05, 0x0A, 0x00, 0xE0, 0x2E, 0x0B, 0x12, 0x22, 0x12, 0x22, 0x01, 0x00, 0x00, 0x3F, 0x6A, 0x18, 0xC8, 0x22}));
    SEND_LCD_CMD(0xC7, ((uint8_t[]){0x50, 0x32, 0x28, 0x00, 0xa2, 0x80, 0x8f, 0x00, 0x80, 0xff, 0x07, 0x11, 0x9F, 0x6f, 0xff, 0x26, 0x0c, 0x0d, 0x0e, 0x0f}));
    SEND_LCD_CMD(0xC9, ((uint8_t[]){0x33, 0x44, 0x44, 0x01}));
    SEND_LCD_CMD(0xCF, ((uint8_t[]){0x34, 0x1E, 0x88, 0x58, 0x13, 0x18, 0x56, 0x18, 0x1E, 0x68, 0xF7, 0x00, 0x65, 0x0C, 0x22, 0xC4, 0x0C, 0x77, 0x22, 0x44, 0xAA, 0x55, 0x04, 0x04, 0x12, 0xA0, 0x08}));
    SEND_LCD_CMD(0xD5, ((uint8_t[]){0x3E, 0x3E, 0x88, 0x00, 0x44, 0x04, 0x78, 0x33, 0x20, 0x78, 0x33, 0x20, 0x04, 0x28, 0xD3, 0x47, 0x03, 0x03, 0x03, 0x03, 0x86, 0x00, 0x00, 0x00, 0x30, 0x52, 0x3f, 0x40, 0x40, 0x96}));
    SEND_LCD_CMD(0xD6, ((uint8_t[]){0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE, 0x95, 0x00, 0x01, 0x83, 0x75, 0x36, 0x20, 0x75, 0x36, 0x20, 0x3F, 0x03, 0x03, 0x03, 0x10, 0x10, 0x00, 0x04, 0x51, 0x20, 0x01, 0x00}));
    SEND_LCD_CMD(0xD7, ((uint8_t[]){0x0a, 0x08, 0x0e, 0x0c, 0x1E, 0x18, 0x19, 0x1F, 0x00, 0x1F, 0x1A, 0x1F, 0x3E, 0x3E, 0x04, 0x00, 0x1F, 0x1F, 0x1F}));
    SEND_LCD_CMD(0xD8, ((uint8_t[]){0x0B, 0x09, 0x0F, 0x0D, 0x1E, 0x18, 0x19, 0x1F, 0x01, 0x1F, 0x1A, 0x1F}));
    SEND_LCD_CMD(0xD9, ((uint8_t[]){0x00, 0x0D, 0x0F, 0x09, 0x0B, 0x1F, 0x18, 0x19, 0x1F, 0x01, 0x1E, 0x1A, 0x1F}));
    SEND_LCD_CMD(0xDD, ((uint8_t[]){0x0C, 0x0E, 0x08, 0x0A, 0x1F, 0x18, 0x19, 0x1F, 0x00, 0x1E, 0x1A, 0x1F}));
    SEND_LCD_CMD(0xDF, ((uint8_t[]){0x44, 0x73, 0x4B, 0x69, 0x00, 0x0A, 0x02, 0x90}));
    SEND_LCD_CMD(0xE0, ((uint8_t[]){0x19, 0x20, 0x0A, 0x13, 0x0E, 0x09, 0x12, 0x28, 0xD4, 0x24, 0x0C, 0x35, 0x13, 0x31, 0x36, 0x2f, 0x03}));
    SEND_LCD_CMD(0xE1, ((uint8_t[]){0x38, 0x20, 0x09, 0x12, 0x0E, 0x08, 0x12, 0x28, 0xC5, 0x24, 0x0C, 0x34, 0x12, 0x31, 0x36, 0x2f, 0x27}));
    SEND_LCD_CMD(0xE2, ((uint8_t[]){0x19, 0x20, 0x0A, 0x11, 0x09, 0x06, 0x11, 0x25, 0xD4, 0x22, 0x0B, 0x33, 0x12, 0x2D, 0x32, 0x2f, 0x03}));
    SEND_LCD_CMD(0xE3, ((uint8_t[]){0x38, 0x20, 0x0A, 0x11, 0x09, 0x06, 0x11, 0x25, 0xC4, 0x21, 0x0A, 0x32, 0x11, 0x2C, 0x32, 0x2f, 0x27}));
    SEND_LCD_CMD(0xE4, ((uint8_t[]){0x19, 0x20, 0x0D, 0x14, 0x0D, 0x08, 0x12, 0x2A, 0xD4, 0x26, 0x0E, 0x35, 0x13, 0x34, 0x39, 0x2f, 0x03}));
    SEND_LCD_CMD(0xE5, ((uint8_t[]){0x38, 0x20, 0x0D, 0x13, 0x0D, 0x07, 0x12, 0x29, 0xC4, 0x25, 0x0D, 0x35, 0x12, 0x33, 0x39, 0x2f, 0x27}));
    SEND_LCD_CMD(0xBB, ((uint8_t[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));

    SEND_LCD_CMD(0x13, ((uint8_t[]){0x00}));
    SEND_LCD_CMD(0x11, ((uint8_t[]){0x00})); delay_ms(200);
    SEND_LCD_CMD(0x29, ((uint8_t[]){0x00})); delay_ms(200);
    SEND_LCD_CMD(0x2C, ((uint8_t[]){0x00, 0x00, 0x00, 0x00}));
    // SEND_LCD_CMD(0x22, ((uint8_t[]){0x00})); delay_ms(200);//All Pixels off

    #else
    SEND_LCD_CMD(0xBB, ((uint8_t []){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5A, 0xA5}));
    /* Suspected `cr_win_en` toggle: short feature-flag command from the
     * AXS15260 sibling chip's init (sent right after BB unlock there).
     * If 0xF8 is the windowing-enable register, this should make partial
     * CASET/RASET stick on AXS15231B without the touch-breaking C/D-series
     * timing tweaks.  If touch dies after this, comment it out. */
    SEND_LCD_CMD(0xF8, ((uint8_t []){0x21, 0xA0}));
    SEND_LCD_CMD(0xA0, ((uint8_t []){0xC0, 0x10, 0x00, 0x02, 0x00, 0x00, 0x04, 0x3F, 0x20, 0x05, 0x3F, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00}));
    SEND_LCD_CMD(0xA2, ((uint8_t []){0x30, 0x3C, 0x24, 0x14, 0xD0, 0x20, 0xFF, 0xE0, 0x40, 0x19, 0x80, 0x80, 0x80, 0x20, 0xf9, 0x10, 0x02, 0xff, 0xff, 0xF0, 0x90, 0x01, 0x32, 0xA0, 0x91, 0xE0, 0x20, 0x7F, 0xFF, 0x00, 0x5A}));
    SEND_LCD_CMD(0xD0, ((uint8_t []){0xE0, 0x40, 0x51, 0x24, 0x08, 0x05, 0x10, 0x01, 0x20, 0x15, 0x42, 0xC2, 0x22, 0x22, 0xAA, 0x03, 0x10, 0x12, 0x60, 0x14, 0x1E, 0x51, 0x15, 0x00, 0x8A, 0x20, 0x00, 0x03, 0x3A, 0x12}));
    SEND_LCD_CMD(0xA3, ((uint8_t []){0xA0, 0x06, 0xAa, 0x00, 0x08, 0x02, 0x0A, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x55, 0x55}));
    SEND_LCD_CMD(0xC1, ((uint8_t []){0x31, 0x04, 0x02, 0x02, 0x71, 0x05, 0x24, 0x55, 0x02, 0x00, 0x41, 0x00, 0x53, 0xFF, 0xFF, 0xFF, 0x4F, 0x52, 0x00, 0x4F, 0x52, 0x00, 0x45, 0x3B, 0x0B, 0x02, 0x0d, 0x00, 0xFF, 0x40}));
    SEND_LCD_CMD(0xC3, ((uint8_t []){0x00, 0x00, 0x00, 0x50, 0x03, 0x00, 0x00, 0x00, 0x01, 0x80, 0x01}));
    SEND_LCD_CMD(0xC4, ((uint8_t []){0x00, 0x24, 0x33, 0x80, 0x00, 0xea, 0x64, 0x32, 0xC8, 0x64, 0xC8, 0x32, 0x90, 0x90, 0x11, 0x06, 0xDC, 0xFA, 0x00, 0x00, 0x80, 0xFE, 0x10, 0x10, 0x00, 0x0A, 0x0A, 0x44, 0x50}));
    SEND_LCD_CMD(0xC5, ((uint8_t []){0x18, 0x00, 0x00, 0x03, 0xFE, 0x3A, 0x4A, 0x20, 0x30, 0x10, 0x88, 0xDE, 0x0D, 0x08, 0x0F, 0x0F, 0x01, 0x3A, 0x4A, 0x20, 0x10, 0x10, 0x00}));
    SEND_LCD_CMD(0xC6, ((uint8_t []){0x05, 0x0A, 0x05, 0x0A, 0x00, 0xE0, 0x2E, 0x0B, 0x12, 0x22, 0x12, 0x22, 0x01, 0x03, 0x00, 0x3F, 0x6A, 0x18, 0xC8, 0x22}));
    SEND_LCD_CMD(0xC7, ((uint8_t []){0x50, 0x32, 0x28, 0x00, 0xa2, 0x80, 0x8f, 0x00, 0x80, 0xff, 0x07, 0x11, 0x9c, 0x67, 0xff, 0x24, 0x0c, 0x0d, 0x0e, 0x0f}));
    SEND_LCD_CMD(0xC9, ((uint8_t []){0x33, 0x44, 0x44, 0x01}));
    SEND_LCD_CMD(0xCF, ((uint8_t []){0x2C, 0x1E, 0x88, 0x58, 0x13, 0x18, 0x56, 0x18, 0x1E, 0x68, 0x88, 0x00, 0x65, 0x09, 0x22, 0xC4, 0x0C, 0x77, 0x22, 0x44, 0xAA, 0x55, 0x08, 0x08, 0x12, 0xA0, 0x08}));
    SEND_LCD_CMD(0xD5, ((uint8_t []){0x40, 0x8E, 0x8D, 0x01, 0x35, 0x04, 0x92, 0x74, 0x04, 0x92, 0x74, 0x04, 0x08, 0x6A, 0x04, 0x46, 0x03, 0x03, 0x03, 0x03, 0x82, 0x01, 0x03, 0x00, 0xE0, 0x51, 0xA1, 0x00, 0x00, 0x00}));
    SEND_LCD_CMD(0xD6, ((uint8_t []){0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE, 0x93, 0x00, 0x01, 0x83, 0x07, 0x07, 0x00, 0x07, 0x07, 0x00, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x00, 0x84, 0x00, 0x20, 0x01, 0x00}));
    SEND_LCD_CMD(0xD7, ((uint8_t []){0x03, 0x01, 0x0b, 0x09, 0x0f, 0x0d, 0x1E, 0x1F, 0x18, 0x1d, 0x1f, 0x19, 0x40, 0x8E, 0x04, 0x00, 0x20, 0xA0, 0x1F}));
    SEND_LCD_CMD(0xD8, ((uint8_t []){0x02, 0x00, 0x0a, 0x08, 0x0e, 0x0c, 0x1E, 0x1F, 0x18, 0x1d, 0x1f, 0x19}));
    SEND_LCD_CMD(0xD9, ((uint8_t []){0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}));
    SEND_LCD_CMD(0xDD, ((uint8_t []){0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}));
    SEND_LCD_CMD(0xDF, ((uint8_t []){0x44, 0x73, 0x4B, 0x69, 0x00, 0x0A, 0x02, 0x90}));
    SEND_LCD_CMD(0xE0, ((uint8_t []){0x3B, 0x28, 0x10, 0x16, 0x0c, 0x06, 0x11, 0x28, 0x5c, 0x21, 0x0D, 0x35, 0x13, 0x2C, 0x33, 0x28, 0x0D}));
    SEND_LCD_CMD(0xE1, ((uint8_t []){0x37, 0x28, 0x10, 0x16, 0x0b, 0x06, 0x11, 0x28, 0x5C, 0x21, 0x0D, 0x35, 0x14, 0x2C, 0x33, 0x28, 0x0F}));
    SEND_LCD_CMD(0xE2, ((uint8_t []){0x3B, 0x07, 0x12, 0x18, 0x0E, 0x0D, 0x17, 0x35, 0x44, 0x32, 0x0C, 0x14, 0x14, 0x36, 0x3A, 0x2F, 0x0D}));
    SEND_LCD_CMD(0xE3, ((uint8_t []){0x37, 0x07, 0x12, 0x18, 0x0E, 0x0D, 0x17, 0x35, 0x44, 0x32, 0x0C, 0x14, 0x14, 0x36, 0x32, 0x2F, 0x0F}));
    SEND_LCD_CMD(0xE4, ((uint8_t []){0x3B, 0x07, 0x12, 0x18, 0x0E, 0x0D, 0x17, 0x39, 0x44, 0x2E, 0x0C, 0x14, 0x14, 0x36, 0x3A, 0x2F, 0x0D}));
    SEND_LCD_CMD(0xE5, ((uint8_t []){0x37, 0x07, 0x12, 0x18, 0x0E, 0x0D, 0x17, 0x39, 0x44, 0x2E, 0x0C, 0x14, 0x14, 0x36, 0x3A, 0x2F, 0x0F}));
    SEND_LCD_CMD(0xA4, ((uint8_t []){0x85, 0x85, 0x95, 0x82, 0xAF, 0xAA, 0xAA, 0x80, 0x10, 0x30, 0x40, 0x40, 0x20, 0xFF, 0x60, 0x30}));
    SEND_LCD_CMD(0xA4, ((uint8_t []){0x85, 0x85, 0x95, 0x85}));
    SEND_LCD_CMD(0xBB, ((uint8_t []){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));

    SEND_LCD_CMD(0x13, ((uint8_t []){0x00}));
    SEND_LCD_CMD(0x11, ((uint8_t []){0x00})); delay_ms(120);
    SEND_LCD_CMD(0x2C, ((uint8_t []){0x00, 0x00, 0x00, 0x00}));
    #endif
    #else

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

    // lcd_cmd(0x21);                                  /* INVON — color inversion on */
    lcd_cmd(0x13);                                  /* NORON */
    lcd_cmd(0x29);                                  /* DISPON */
    delay_ms(120);
    #endif
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

/* Per-chunk scratch — double-buffered (A/B) so CPU byteswap for chunk N+1
 * overlaps with DMA streaming chunk N from the other buffer.  SPI3 has no
 * halfword byte-swap on ESP32-S3 so the byteswap is a CPU pass; pipelining
 * hides it under the 1.4 ms DMA per chunk.
 *
 * Parked in INTERNAL SRAM (0x3FCC0000) rather than PSRAM BSS:
 *  - SRAM is uncached → byteswap reads/writes don't pay PSRAM cache-miss
 *    cost (~3-4× speedup on the bswap-copy loop).
 *  - SRAM is non-cached → no Cache_WriteBack_Addr needed before DMA
 *    (saves another ~500 µs per chunk).
 *  - 2 × LCD_CHUNK_BYTES = 55040 bytes lands at 0x3FCC0000..0x3FCCD700,
 *    well below the descriptor pool at 0x3FCD0000 and boot2's iram_seg
 *    at 0x3FCD8700. */
#define LCD_SCRATCH_A_ADDR  0x3FCC0000u
#define LCD_SCRATCH_B_ADDR  (LCD_SCRATCH_A_ADDR + LCD_CHUNK_BYTES)
static uint16_t * const s_lcd_swap_a = (uint16_t *)LCD_SCRATCH_A_ADDR;
static uint16_t * const s_lcd_swap_b = (uint16_t *)LCD_SCRATCH_B_ADDR;
#define s_lcd_swap_scratch s_lcd_swap_a

static inline void lcd_bswap_copy(uint16_t *dst, const uint16_t *src, uint32_t halfwords)
{
    while (halfwords--) *dst++ = __builtin_bswap16(*src++);
    // memcpy(dst, src, (size_t)halfwords * 2u);
}

/* Cache-flush a byte range, rounding to 32-byte cache-line bounds.  The
 * ROM Cache_WriteBack_Addr drops the trailing partial line if `len` isn't
 * a cache-line multiple — which it rarely is for partial blits — so any
 * caller flushing arbitrary byte counts must round up.
 *
 * Only PSRAM addresses (DBUS, 0x3C000000+) are cached and need flushing.
 * Internal SRAM scratches (0x3FCxxxxx) are uncached; this becomes a no-op. */
static inline void lcd_flush_scratch(uint32_t addr, uint32_t bytes)
{
    if (addr < 0x3C000000u) return;          /* internal SRAM — uncached */
    uint32_t lo = addr & ~0x1Fu;
    uint32_t hi = (addr + bytes + 0x1Fu) & ~0x1Fu;
    Cache_WriteBack_Addr(lo, hi - lo);
}

/* Pipelined frame streamer.  Caller passes a `prepare(idx, dst, bytes)`
 * callback that fills scratch[idx] with `bytes` of wire-BE pixel data for
 * the given chunk index.  The streamer alternates between scratches A/B
 * so CPU prepare for chunk N+1 overlaps with DMA streaming chunk N from
 * the other scratch.
 *
 * Inter-chunk SPI idle drops from ~1.2 ms (CPU prep gap) to a few µs of
 * register writes, lifting full-frame transfer wall time from ~21 ms
 * (~35 FPS) to ~12 ms (~60+ FPS) for 220 KB at 40 MHz QSPI quad. */
typedef void (*lcd_prepare_fn)(uint16_t *dst, uint32_t bytes, void *ctx);

static void lcd_stream_pipelined(uint32_t total_bytes,
                                 lcd_prepare_fn prepare, void *ctx)
{
    static const uint8_t caset[] = {0x00, 0x00, 0x00, (uint8_t)(LCD_WIDTH_NATIVE - 1)};
    lcd_cmd_data(0x2A, caset, 4);

    cs_lo();

    uint16_t *scratches[2] = { s_lcd_swap_a, s_lcd_swap_b };
    int half = 0;
    bool first = true;
    bool dma_in_flight = false;
    uint32_t remaining = total_bytes;

    while (remaining) {
        uint32_t d = remaining > LCD_CHUNK_BYTES ? LCD_CHUNK_BYTES : remaining;
        uint32_t desc_base = (uint32_t)half * DMA_HALF_SIZE;

        /* CPU work for THIS chunk overlaps with DMA streaming PREVIOUS
         * chunk from the other scratch. */
        prepare(scratches[half], d, ctx);
        lcd_flush_scratch((uint32_t)scratches[half], d);
        build_dma_chain(desc_base, scratches[half], d);

        /* Wait for previous DMA to clear SPI_USR before kicking off this
         * chunk — the SPI peripheral can only have one in-flight USR. */
        if (dma_in_flight) {
            spi_dma_wait();
        }

        /* RAMWR header on first chunk only.  Subsequent chunks are pure
         * raw QUAD pixel bytes — cursor streams continuously inside the
         * CASET window. */
        if (first) {
            uint8_t hdr[4] = {0x32, 0x00, 0x2Cu, 0x00};
            spi_write_chunk(SPI_USR_MOSI | SPI_CS_SETUP | SPI_CS_HOLD, hdr, 4);
            first = false;
        }

        spi_dma_kickoff_at(desc_base, d);
        dma_in_flight = true;

        half       = 1 - half;
        remaining -= d;
    }

    if (dma_in_flight) {
        spi_dma_wait();
    }

    cs_hi();
}

/* Prepare callback for lcd_blit_frame: byteswap-copy from the framebuffer
 * cursor (advanced by ctx) into scratch. */
typedef struct { const uint8_t *p; } blit_ctx_t;
static void blit_prepare(uint16_t *dst, uint32_t bytes, void *vctx)
{
    blit_ctx_t *c = (blit_ctx_t *)vctx;
    lcd_bswap_copy(dst, (const uint16_t *)c->p, bytes / 2);
    c->p += bytes;
}

void lcd_blit_frame(const uint16_t *fb)
{
    blit_ctx_t ctx = { .p = (const uint8_t *)fb };
    lcd_stream_pipelined((uint32_t)LCD_WIDTH_NATIVE * LCD_HEIGHT_NATIVE * 2u,
                         blit_prepare, &ctx);
}

/* Prepare callback for lcd_fill: fill scratch with one wire-BE color
 * value (byteswapped from native). */
typedef struct { uint16_t wire_color; } fill_ctx_t;
static void fill_prepare(uint16_t *dst, uint32_t bytes, void *vctx)
{
    fill_ctx_t *c = (fill_ctx_t *)vctx;
    uint32_t halfwords = bytes / 2;
    for (uint32_t i = 0; i < halfwords; i++) dst[i] = c->wire_color;
}

void lcd_fill(uint16_t color)
{
    fill_ctx_t ctx = { .wire_color = __builtin_bswap16(color) };
    lcd_stream_pipelined((uint32_t)LCD_WIDTH_NATIVE * LCD_HEIGHT_NATIVE * 2u,
                         fill_prepare, &ctx);
}

/* Partial-region blit.  Two paths gated by LCD_BLIT_PARTIAL_WINDOW:
 *
 *   1 = "true partial" — partial CASET (x..x+w-1) + RASET (y..y+h-1),
 *       single CS-low across all chunks, RAMWR header on first chunk only,
 *       subsequent chunks pure raw QUAD data (LilyGO T-Display-S3-Long
 *       pattern).  Sends exactly w*h pixels.  Requires the chip's
 *       `cr_win_en` to be set by the init — works only when one of the
 *       C/D-series init commands has flipped that gate.
 *
 *   0 = "full-width fallback" — full-width CASET, send (y+h) full-width
 *       rows from row 0 sourced from the surface, RAMWR/RAMWRC per chunk
 *       with CS-toggle.  Doesn't depend on cr_win_en.  Bandwidth scales as
 *       (y+h)/LCD_HEIGHT_NATIVE of a full frame.
 *
 * Flip the flag and rebuild to A/B-test once an init that potentially
 * unlocks windowing without breaking touch is in place. */
#ifndef LCD_BLIT_PARTIAL_WINDOW
#define LCD_BLIT_PARTIAL_WINDOW 1
#endif

#if LCD_BLIT_PARTIAL_WINDOW
void lcd_blit(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
              uint16_t stride, const uint16_t *buffer)
{
    if (!w || !h) return;

    uint32_t row_pixels = (uint32_t)w;
    uint32_t row_bytes  = row_pixels * 2u;

    /* CASET (X) + RASET (Y) — partial windows.  Each lcd_cmd_data toggles
     * CS internally for the command frame.  Window state persists into
     * the data phase below. */
    uint8_t caset[4] = {(uint8_t)(x >> 8), (uint8_t)x,
                        (uint8_t)((x + w - 1) >> 8), (uint8_t)(x + w - 1)};
    lcd_cmd_data(0x2A, caset, 4);
    uint8_t raset[4] = {(uint8_t)(y >> 8), (uint8_t)y,
                        (uint8_t)((y + h - 1) >> 8), (uint8_t)(y + h - 1)};
    lcd_cmd_data(0x2B, raset, 4);

    // lcd_cmd(0x3C);

    uint32_t chunk_rows = h;
    if (row_bytes > 0) {
        uint32_t scratch_rows = sizeof(s_lcd_swap_scratch) / row_bytes;
        if (chunk_rows > scratch_rows) chunk_rows = scratch_rows;
        if (chunk_rows * row_bytes > 32768u) chunk_rows = 32768u / row_bytes;
        uint32_t pool_max = (uint32_t)DMA_DESC_POOL * DMA_BYTES_PER_DESC;
        if (chunk_rows * row_bytes > pool_max) chunk_rows = pool_max / row_bytes;
    }
    if (chunk_rows == 0) chunk_rows = 1;

    /* Single CS-low across all chunks; RAMWR header only on first chunk;
     * subsequent chunks are pure raw QUAD pixel bytes (LilyGO pattern). */
    cs_lo();

    bool first = true;
    uint32_t cur_row = 0;
    while (cur_row < h) {
        uint32_t rows = h - cur_row;
        if (rows > chunk_rows) rows = chunk_rows;

        for (uint32_t i = 0; i < rows; i++) {
            const uint16_t *src = buffer + (cur_row + i) * stride;
            uint16_t *dst = s_lcd_swap_scratch + i * row_pixels;
            lcd_bswap_copy(dst, src, row_pixels);
        }
        lcd_flush_scratch((uint32_t)s_lcd_swap_scratch, rows * row_bytes);

        if (first) {
            uint8_t hdr[4] = {0x32, 0x00, 0x2Cu, 0x00};
            spi_write_chunk(SPI_USR_MOSI | SPI_CS_SETUP | SPI_CS_HOLD, hdr, 4);
            first = false;
        }
        spi_write_quad_dma(s_lcd_swap_scratch, rows * row_bytes);

        cur_row += rows;
    }

    cs_hi();
}
#else  /* LCD_BLIT_PARTIAL_WINDOW == 0 — full-width fallback */
void lcd_blit(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
              uint16_t stride, const uint16_t *buffer)
{
    (void)w;
    if (!h) return;

    uint32_t total_rows = (uint32_t)y + h;
    uint32_t row_pixels = LCD_WIDTH_NATIVE;
    uint32_t row_bytes  = row_pixels * 2u;
    const uint16_t *surf_origin = buffer - (uint32_t)y * stride - (uint32_t)x;

    static const uint8_t caset[] = {0x00, 0x00, 0x00, (uint8_t)(LCD_WIDTH_NATIVE - 1)};
    lcd_cmd_data(0x2A, caset, 4);

    uint32_t chunk_rows = total_rows;
    {
        uint32_t scratch_rows = sizeof(s_lcd_swap_scratch) / row_bytes;
        if (chunk_rows > scratch_rows) chunk_rows = scratch_rows;
        if (chunk_rows * row_bytes > 32768u) chunk_rows = 32768u / row_bytes;
        uint32_t pool_max = (uint32_t)DMA_DESC_POOL * DMA_BYTES_PER_DESC;
        if (chunk_rows * row_bytes > pool_max) chunk_rows = pool_max / row_bytes;
    }
    if (chunk_rows == 0) chunk_rows = 1;

    bool first = true;
    uint32_t cur_row = 0;
    while (cur_row < total_rows) {
        uint32_t rows = total_rows - cur_row;
        if (rows > chunk_rows) rows = chunk_rows;

        for (uint32_t i = 0; i < rows; i++) {
            const uint16_t *src = surf_origin + (cur_row + i) * stride;
            uint16_t *dst = s_lcd_swap_scratch + i * row_pixels;
            lcd_bswap_copy(dst, src, row_pixels);
        }
        lcd_flush_scratch((uint32_t)s_lcd_swap_scratch, rows * row_bytes);

        cs_lo();
        uint8_t hdr[4] = {0x32, 0x00, first ? 0x2Cu : 0x3Cu, 0x00};
        spi_write_chunk(SPI_USR_MOSI | SPI_CS_SETUP | SPI_CS_HOLD, hdr, 4);
        spi_write_quad_dma(s_lcd_swap_scratch, rows * row_bytes);
        cs_hi();

        cur_row += rows;
        first = false;
    }
}
#endif  /* LCD_BLIT_PARTIAL_WINDOW */

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
