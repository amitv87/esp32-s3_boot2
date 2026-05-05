#include "hal_i2c.h"
#include <stdio.h>

#if 0
/* ============================================================
 * Legacy bit-bang I2C — works on any 2 GPIO pins, ~100 kHz.
 * ============================================================ */

static inline void _i2c_delay(void)
{
    for (volatile int i = 0; i < 200; i++) __asm__ volatile("nop");
}

#define _SDA_HI(b)  gpio_od_drive((b)->sda, true)
#define _SDA_LO(b)  gpio_od_drive((b)->sda, false)
#define _SCL_HI(b)  gpio_od_drive((b)->scl, true)
#define _SCL_LO(b)  gpio_od_drive((b)->scl, false)
#define _SDA_RD(b)  gpio_read((b)->sda)

void bb_i2c_init(bb_i2c_t *b, int sda, int scl)
{
    b->sda = sda; b->scl = scl; b->port = 0;
    gpio_opendrain_init(sda);
    gpio_opendrain_init(scl);
    _SDA_HI(b); _SCL_HI(b);
    _i2c_delay();
}

static void _i2c_start(bb_i2c_t *b)
{
    _SDA_HI(b); _SCL_HI(b); _i2c_delay();
    _SDA_LO(b);              _i2c_delay();
    _SCL_LO(b);              _i2c_delay();
}

static void _i2c_stop(bb_i2c_t *b)
{
    _SDA_LO(b); _SCL_HI(b); _i2c_delay();
    _SDA_HI(b);              _i2c_delay();
}

static bool _i2c_write_byte(bb_i2c_t *b, uint8_t byte)
{
    for (int i = 7; i >= 0; i--) {
        if ((byte >> i) & 1) _SDA_HI(b); else _SDA_LO(b);
        _i2c_delay(); _SCL_HI(b); _i2c_delay(); _SCL_LO(b); _i2c_delay();
    }
    _SDA_HI(b); _i2c_delay();
    _SCL_HI(b); _i2c_delay();
    bool nack = _SDA_RD(b);
    _SCL_LO(b); _i2c_delay();
    return !nack;
}

static uint8_t _i2c_read_byte(bb_i2c_t *b, bool ack)
{
    _SDA_HI(b);
    uint8_t val = 0;
    for (int i = 7; i >= 0; i--) {
        _i2c_delay(); _SCL_HI(b); _i2c_delay();
        if (_SDA_RD(b)) val |= (1u << i);
        _SCL_LO(b);
    }
    if (ack) _SDA_LO(b); else _SDA_HI(b);
    _i2c_delay(); _SCL_HI(b); _i2c_delay(); _SCL_LO(b); _i2c_delay();
    _SDA_HI(b);
    return val;
}

bool bb_i2c_probe(bb_i2c_t *b, uint8_t addr)
{
    _i2c_start(b);
    bool ack = _i2c_write_byte(b, (uint8_t)((addr << 1) | 0));
    _i2c_stop(b);
    return ack;
}

bool bb_i2c_write(bb_i2c_t *b, uint8_t addr, const uint8_t *data, int len)
{
    _i2c_start(b);
    if (!_i2c_write_byte(b, (uint8_t)((addr << 1) | 0))) { _i2c_stop(b); return false; }
    for (int i = 0; i < len; i++) _i2c_write_byte(b, data[i]);
    _i2c_stop(b);
    return true;
}

bool bb_i2c_write_reg(bb_i2c_t *b, uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return bb_i2c_write(b, addr, buf, 2);
}

bool bb_i2c_write_read(bb_i2c_t *b, uint8_t addr,
                       const uint8_t *wbuf, int wlen,
                       uint8_t *rbuf, int rlen)
{
    _i2c_start(b);
    if (!_i2c_write_byte(b, (uint8_t)((addr << 1) | 0))) { _i2c_stop(b); return false; }
    for (int i = 0; i < wlen; i++) _i2c_write_byte(b, wbuf[i]);
    _i2c_start(b);
    if (!_i2c_write_byte(b, (uint8_t)((addr << 1) | 1))) { _i2c_stop(b); return false; }
    for (int i = 0; i < rlen; i++) rbuf[i] = _i2c_read_byte(b, i < rlen - 1);
    _i2c_stop(b);
    return true;
}

#else
/* ============================================================
 * ESP32-S3 hardware I2C0/I2C1, 400 kHz, polled (no IRQ/DMA).
 * Uses the chip's command-list FIFO: each CMD register is a tiny
 * micro-op (RSTART/WRITE/READ/STOP/END) that the I2C state machine
 * executes back-to-back in a single TRANS_START.  Up to 8 cmd slots
 * per port — comfortably fits write+restart+read+stop.
 * ============================================================ */

#include "soc/soc.h"
#include "soc/system_reg.h"

#define I2C0_BASE  0x60013000UL
#define I2C1_BASE  0x60027000UL

/* Register offsets (same for both ports) */
#define I2C_REG_SCL_LOW_PERIOD   0x00
#define I2C_REG_CTR              0x04
#define I2C_REG_SR               0x08
#define I2C_REG_TO               0x0C
#define I2C_REG_FIFO_CONF        0x18
#define I2C_REG_DATA             0x1C
#define I2C_REG_INT_RAW          0x20
#define I2C_REG_INT_CLR          0x24
#define I2C_REG_SDA_HOLD         0x30
#define I2C_REG_SDA_SAMPLE       0x34
#define I2C_REG_SCL_HIGH_PERIOD  0x38
#define I2C_REG_SCL_START_HOLD   0x40
#define I2C_REG_SCL_RSTART_SETUP 0x44
#define I2C_REG_SCL_STOP_HOLD    0x48
#define I2C_REG_SCL_STOP_SETUP   0x4C
#define I2C_REG_FILTER_CFG       0x50
#define I2C_REG_CLK_CONF         0x54
#define I2C_REG_COMD0            0x58

/* CTR bits */
#define CTR_SDA_FORCE_OUT  (1u << 0)
#define CTR_SCL_FORCE_OUT  (1u << 1)
#define CTR_RX_FULL_NACK   (1u << 3)
#define CTR_MS_MODE        (1u << 4)
#define CTR_TRANS_START    (1u << 5)
#define CTR_CLK_EN         (1u << 8)
#define CTR_ARBITRATION_EN (1u << 9)
#define CTR_FSM_RST        (1u << 10)
#define CTR_CONF_UPGATE    (1u << 11)

/* CLK_CONF bits */
#define CLK_SCLK_SEL       (1u << 20)   /* 0=XTAL(40MHz), 1=8MHz */
#define CLK_SCLK_ACTIVE    (1u << 21)

/* FIFO_CONF bits */
#define FIFO_RX_FIFO_RST   (1u << 12)
#define FIFO_TX_FIFO_RST   (1u << 13)
#define FIFO_FIFO_PRT_EN   (1u << 14)

/* INT bits */
#define INT_RXFIFO_WM      (1u << 0)
#define INT_END_DETECT     (1u << 3)
#define INT_TRANS_COMPLETE (1u << 7)
#define INT_TIME_OUT       (1u << 8)
#define INT_NACK           (1u << 10)
#define INT_ARB_LOST       (1u << 5)

/* I2C cmd opcodes (op_code field of CMD reg, bits 13:11).
 * NOTE: the values in the chip's reg-header comment block are wrong —
 * IDF's i2c_ll.h is authoritative.  RESTART=6 (not 0!), READ=3 (not 2),
 * STOP=2 (not 3). */
#define I2C_OP_RSTART  6
#define I2C_OP_WRITE   1
#define I2C_OP_STOP    2
#define I2C_OP_READ    3
#define I2C_OP_END     4

/* GPIO matrix signal indices (gpio_sig_map.h) */
#define SIG_I2CEXT0_SCL 89
#define SIG_I2CEXT0_SDA 90
#define SIG_I2CEXT1_SCL 91
#define SIG_I2CEXT1_SDA 92

static inline volatile uint32_t *I2C_REG(int port, uint32_t off)
{
    uintptr_t base = (port == 0) ? I2C0_BASE : I2C1_BASE;
    return (volatile uint32_t *)(base + off);
}

static inline void i2c_update(int port)
{
    *I2C_REG(port, I2C_REG_CTR) |= CTR_CONF_UPGATE;
}

static void i2c_pin_setup(int gpio_num, int sig_in, int sig_out)
{
    /* Open-drain pad: FUN_SEL=1 (GPIO matrix), FUN_DRV=2, FUN_IE=1, FUN_PU=1 */
    *iomux_reg(gpio_num) = (1u << 12) | (2u << 10) | (1u << 9) | (1u << 8);
    /* Both directions: matrix in/out + pad output enable.  Pad is open-drain
     * (configured below via GPIO_PINn_REG.pad_driver=1). */
    gpio_matrix_in (gpio_num, sig_in,  false);
    gpio_matrix_out(gpio_num, sig_out, false);
    _gpio_oe_set(gpio_num, true);
    /* Open-drain mode: GPIO_PINn_REG bit 2 = PAD_DRIVER (1 = open drain) */
    volatile uint32_t *pin_reg =
        (volatile uint32_t *)(GPIO_BASE + 0x74u + 4u * (uint32_t)gpio_num);
    *pin_reg |= (1u << 2);
}

static void i2c_setup_port(int port, int sda, int scl)
{
    /* Enable peripheral bus clock + reset peripheral */
    if (port == 0) {
        REG_SET_BIT(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_I2C_EXT0_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_I2C_EXT0_RST);
        for (volatile int i = 0; i < 200; i++);
        REG_CLR_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_I2C_EXT0_RST);
    } else {
        REG_SET_BIT(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_I2C_EXT1_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_I2C_EXT1_RST);
        for (volatile int i = 0; i < 200; i++);
        REG_CLR_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_I2C_EXT1_RST);
    }

    /* CTR: master mode, SDA/SCL open-drain, MSB-first.  Match IDF's master
     * init exactly — arbitration disabled, rx_full_ack_level=0, no CLK_EN
     * (reserved bit). */
    *I2C_REG(port, I2C_REG_CTR) =
        CTR_SDA_FORCE_OUT | CTR_SCL_FORCE_OUT | CTR_MS_MODE;

    /* CLK_CONF: source = XTAL (40 MHz, sclk_sel=0), divider = 1, active */
    *I2C_REG(port, I2C_REG_CLK_CONF) = (0u << 0) | CLK_SCLK_ACTIVE;

    /* SCL/SDA timing for 400 kHz from 40 MHz source.
     *   half_cycle = 40M/400k/2 = 50 cycles
     *   scl_low      = 50 (write 49 — reg holds N-1)
     *   scl_wait_high= 23 (50/2 - 2)
     *   scl_high     = 27 (half_cycle - scl_wait_high)
     *   sda_hold     = 12 (write 11)
     *   sda_sample   = 25 (write 24)
     *   setup/hold   = 50 (write 49)
     *   tout         = 10 (encoded as log2-style)
     */
    *I2C_REG(port, I2C_REG_SCL_LOW_PERIOD)   = 49;
    /* SCL_HIGH_PERIOD: bits[8:0]=scl_high, bits[17:9]=scl_wait_high (NOT minus 1) */
    *I2C_REG(port, I2C_REG_SCL_HIGH_PERIOD)  = (uint32_t)27 | ((uint32_t)23 << 9);
    *I2C_REG(port, I2C_REG_SDA_HOLD)         = 11;
    *I2C_REG(port, I2C_REG_SDA_SAMPLE)       = 24;
    *I2C_REG(port, I2C_REG_SCL_RSTART_SETUP) = 49;
    *I2C_REG(port, I2C_REG_SCL_STOP_SETUP)   = 49;
    *I2C_REG(port, I2C_REG_SCL_START_HOLD)   = 49;
    *I2C_REG(port, I2C_REG_SCL_STOP_HOLD)    = 49;
    *I2C_REG(port, I2C_REG_TO)               = (uint32_t)10 | (1u << 5); /* time_out_value | time_out_en */

    /* FIFO_CONF: FIFO mode (NONFIFO_EN=0), enable FIFO pointer, reset both fifos */
    *I2C_REG(port, I2C_REG_FIFO_CONF) =
        FIFO_FIFO_PRT_EN | FIFO_RX_FIFO_RST | FIFO_TX_FIFO_RST;
    *I2C_REG(port, I2C_REG_FIFO_CONF) = FIFO_FIFO_PRT_EN;

    /* No glitch filter (chip default works for 400 kHz w/ pull-ups) */
    *I2C_REG(port, I2C_REG_FILTER_CFG) = 0;

    /* Clear any stale interrupts */
    *I2C_REG(port, I2C_REG_INT_CLR) = 0xFFFFFFFFu;

    /* Latch all the timing/config into the I2C clock domain */
    i2c_update(port);

    /* Pin routing — done last so the SCL line doesn't toggle during setup */
    int sda_in = (port == 0) ? SIG_I2CEXT0_SDA : SIG_I2CEXT1_SDA;
    int sda_out = sda_in;
    int scl_in = (port == 0) ? SIG_I2CEXT0_SCL : SIG_I2CEXT1_SCL;
    int scl_out = scl_in;
    i2c_pin_setup(sda, sda_in, sda_out);
    i2c_pin_setup(scl, scl_in, scl_out);
}

static inline uint32_t cmd_op(int op, int byte_num, int ack_check, int ack_exp, int ack_val)
{
    return ((uint32_t)byte_num & 0xFFu) |
           ((uint32_t)(ack_check & 1) << 8) |
           ((uint32_t)(ack_exp   & 1) << 9) |
           ((uint32_t)(ack_val   & 1) << 10) |
           ((uint32_t)(op        & 7) << 11);
}

/* Wait for the I2C state machine to finish, or timeout/error.
 * Returns true on success (ACK), false on NACK/timeout/arb-lost.
 *
 * NACK and TRANS_COMPLETE can race — when a probe NACKs, TRANS_COMPLETE may
 * latch one polling iteration before NACK does.  Strategy: wait for either
 * NACK or TRANS_COMPLETE, then sleep briefly to let the other settle, and
 * check NACK (sticky/WTC) as the authoritative "did the device ACK?" signal. */
static bool i2c_wait(int port)
{
    /* At 400 kHz, longest realistic transfer is ~1.5 ms.  Cap at ~5 ms. */
    for (int t = 0; t < 5000; t++) {
        uint32_t raw = *I2C_REG(port, I2C_REG_INT_RAW);
        if (raw & (INT_TIME_OUT | INT_ARB_LOST)) return false;
        if (raw & (INT_TRANS_COMPLETE | INT_NACK)) {
            /* Settle for ~1 byte time (~25 us at 400 kHz) before reading
             * NACK — TRANS_COMPLETE can latch first on a NACKed probe. */
            for (volatile int j = 0; j < 2000; j++);
            return !(*I2C_REG(port, I2C_REG_INT_RAW) & INT_NACK);
        }
        for (volatile int j = 0; j < 8; j++);
    }
    return false;
}

/* For debugging — dump key registers after a transaction failure */
void hw_i2c_dump_state(int port)
{
    extern int printf(const char *, ...);
    printf("I2C%d: CTR=%08lx CLK=%08lx FIFO=%08lx INT=%08lx SR=%08lx\r\n",
        port,
        (unsigned long)*I2C_REG(port, I2C_REG_CTR),
        (unsigned long)*I2C_REG(port, I2C_REG_CLK_CONF),
        (unsigned long)*I2C_REG(port, I2C_REG_FIFO_CONF),
        (unsigned long)*I2C_REG(port, I2C_REG_INT_RAW),
        (unsigned long)*I2C_REG(port, I2C_REG_SR));
}

static void i2c_clear_intr(int port)
{
    *I2C_REG(port, I2C_REG_INT_CLR) = 0xFFFFFFFFu;
}

static void i2c_fifo_reset(int port)
{
    *I2C_REG(port, I2C_REG_FIFO_CONF) =
        FIFO_FIFO_PRT_EN | FIFO_RX_FIFO_RST | FIFO_TX_FIFO_RST;
    *I2C_REG(port, I2C_REG_FIFO_CONF) = FIFO_FIFO_PRT_EN;
}

void bb_i2c_init(bb_i2c_t *b, int sda, int scl)
{
    b->sda = sda;
    b->scl = scl;
    /* Pin → port mapping for the Waveshare ESP32-S3-Touch-LCD-3.49 */
    b->port = (sda == 47) ? 1 : 0;
    i2c_setup_port(b->port, sda, scl);
}

bool bb_i2c_probe(bb_i2c_t *b, uint8_t addr)
{
    /* RSTART → WRITE 1 byte (addr+W, ack_check) → STOP */
    i2c_fifo_reset(b->port);
    i2c_clear_intr(b->port);
    *I2C_REG(b->port, I2C_REG_DATA) = (uint32_t)((addr << 1) | 0);

    *I2C_REG(b->port, I2C_REG_COMD0 + 0) = cmd_op(I2C_OP_RSTART, 0, 0, 0, 0);
    *I2C_REG(b->port, I2C_REG_COMD0 + 4) = cmd_op(I2C_OP_WRITE,  1, 1, 0, 0);
    *I2C_REG(b->port, I2C_REG_COMD0 + 8) = cmd_op(I2C_OP_STOP,   0, 0, 0, 0);

    i2c_update(b->port);
    *I2C_REG(b->port, I2C_REG_CTR) |= CTR_TRANS_START;
    bool ok = i2c_wait(b->port);
    /* On the first failure of a freshly-init'd port, dump regs once for debug. */
    static int dumped[2] = {0, 0};
    if (!ok && !dumped[b->port]) {
        dumped[b->port] = 1;
        hw_i2c_dump_state(b->port);
    }
    return ok;
}

bool bb_i2c_write(bb_i2c_t *b, uint8_t addr, const uint8_t *data, int len)
{
    if (len < 0 || len > 31) return false;  /* TXFIFO is 32 bytes; 1 for addr */

    i2c_fifo_reset(b->port);
    i2c_clear_intr(b->port);
    *I2C_REG(b->port, I2C_REG_DATA) = (uint32_t)((addr << 1) | 0);
    for (int i = 0; i < len; i++) *I2C_REG(b->port, I2C_REG_DATA) = data[i];

    *I2C_REG(b->port, I2C_REG_COMD0 + 0) = cmd_op(I2C_OP_RSTART, 0, 0, 0, 0);
    *I2C_REG(b->port, I2C_REG_COMD0 + 4) = cmd_op(I2C_OP_WRITE,  1 + len, 1, 0, 0);
    *I2C_REG(b->port, I2C_REG_COMD0 + 8) = cmd_op(I2C_OP_STOP,   0, 0, 0, 0);

    i2c_update(b->port);
    *I2C_REG(b->port, I2C_REG_CTR) |= CTR_TRANS_START;
    return i2c_wait(b->port);
}

bool bb_i2c_write_reg(bb_i2c_t *b, uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return bb_i2c_write(b, addr, buf, 2);
}

bool bb_i2c_write_read(bb_i2c_t *b, uint8_t addr,
                       const uint8_t *wbuf, int wlen,
                       uint8_t *rbuf, int rlen)
{
    if (wlen < 0 || wlen > 31) return false;
    if (rlen <= 0 || rlen > 32) return false;

    i2c_fifo_reset(b->port);
    i2c_clear_intr(b->port);

    /* TXFIFO: addr+W, write data, then addr+R for repeated start phase */
    *I2C_REG(b->port, I2C_REG_DATA) = (uint32_t)((addr << 1) | 0);
    for (int i = 0; i < wlen; i++) *I2C_REG(b->port, I2C_REG_DATA) = wbuf[i];
    *I2C_REG(b->port, I2C_REG_DATA) = (uint32_t)((addr << 1) | 1);

    int slot = 0;
    *I2C_REG(b->port, I2C_REG_COMD0 + 4 * slot++) = cmd_op(I2C_OP_RSTART, 0, 0, 0, 0);
    *I2C_REG(b->port, I2C_REG_COMD0 + 4 * slot++) = cmd_op(I2C_OP_WRITE,  1 + wlen, 1, 0, 0);
    *I2C_REG(b->port, I2C_REG_COMD0 + 4 * slot++) = cmd_op(I2C_OP_RSTART, 0, 0, 0, 0);
    *I2C_REG(b->port, I2C_REG_COMD0 + 4 * slot++) = cmd_op(I2C_OP_WRITE,  1, 1, 0, 0);
    /* Read all-but-last with ACK, last with NACK */
    if (rlen > 1) {
        *I2C_REG(b->port, I2C_REG_COMD0 + 4 * slot++) = cmd_op(I2C_OP_READ, rlen - 1, 0, 0, 0);
    }
    *I2C_REG(b->port, I2C_REG_COMD0 + 4 * slot++) = cmd_op(I2C_OP_READ,  1, 0, 0, 1);
    *I2C_REG(b->port, I2C_REG_COMD0 + 4 * slot++) = cmd_op(I2C_OP_STOP,  0, 0, 0, 0);

    i2c_update(b->port);
    *I2C_REG(b->port, I2C_REG_CTR) |= CTR_TRANS_START;
    if (!i2c_wait(b->port)) return false;

    for (int i = 0; i < rlen; i++) {
        rbuf[i] = (uint8_t)(*I2C_REG(b->port, I2C_REG_DATA) & 0xFFu);
    }
    return true;
}

#endif

int bb_i2c_scan(bb_i2c_t *b, const char *bus_name)
{
    int found = 0;
    printf("--- I2C scan [%s] ---\r\n", bus_name);
    for (uint8_t addr = 0x01; addr < 0x80; addr++) {
        if (bb_i2c_probe(b, addr)) {
            printf("  found 0x%02X\r\n", addr);
            found++;
        }
    }
    printf("--- scan done: %d device(s) ---\r\n", found);
    return found;
}
