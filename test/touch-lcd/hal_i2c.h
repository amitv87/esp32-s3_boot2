#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "hal_gpio.h"

/*
 * Generic bit-banged I2C for any two GPIO pins (open-drain, 7-bit addresses).
 * Works on both the low (0-31) and high (32-53) GPIO banks.
 */

typedef struct { int sda, scl; } bb_i2c_t;

static inline void _i2c_delay(void)
{
    for (volatile int i = 0; i < 200; i++) __asm__ volatile("nop");
}

#define _SDA_HI(b)  gpio_od_drive((b)->sda, true)
#define _SDA_LO(b)  gpio_od_drive((b)->sda, false)
#define _SCL_HI(b)  gpio_od_drive((b)->scl, true)
#define _SCL_LO(b)  gpio_od_drive((b)->scl, false)
#define _SDA_RD(b)  gpio_read((b)->sda)

static inline void bb_i2c_init(bb_i2c_t *b, int sda, int scl)
{
    b->sda = sda; b->scl = scl;
    gpio_opendrain_init(sda);
    gpio_opendrain_init(scl);
    _SDA_HI(b); _SCL_HI(b);
    _i2c_delay();
}

static inline void _i2c_start(bb_i2c_t *b)
{
    _SDA_HI(b); _SCL_HI(b); _i2c_delay();
    _SDA_LO(b);              _i2c_delay();
    _SCL_LO(b);              _i2c_delay();
}

static inline void _i2c_stop(bb_i2c_t *b)
{
    _SDA_LO(b); _SCL_HI(b); _i2c_delay();
    _SDA_HI(b);              _i2c_delay();
}

/* Returns true if slave ACKed (SDA pulled low during ACK clock) */
static inline bool _i2c_write_byte(bb_i2c_t *b, uint8_t byte)
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

static inline uint8_t _i2c_read_byte(bb_i2c_t *b, bool ack)
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

/* Probe: START + addr+W → returns true if ACK */
static inline bool bb_i2c_probe(bb_i2c_t *b, uint8_t addr)
{
    _i2c_start(b);
    bool ack = _i2c_write_byte(b, (uint8_t)((addr << 1) | 0));
    _i2c_stop(b);
    return ack;
}

/* Write N bytes to device (no register prefix) */
static inline bool bb_i2c_write(bb_i2c_t *b, uint8_t addr,
                                 const uint8_t *data, int len)
{
    _i2c_start(b);
    if (!_i2c_write_byte(b, (uint8_t)((addr << 1) | 0))) { _i2c_stop(b); return false; }
    for (int i = 0; i < len; i++) _i2c_write_byte(b, data[i]);
    _i2c_stop(b);
    return true;
}

/* Write single register byte (addr → reg → val) */
static inline bool bb_i2c_write_reg(bb_i2c_t *b, uint8_t addr,
                                     uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return bb_i2c_write(b, addr, buf, 2);
}

/* Write then read (repeated start) */
static inline bool bb_i2c_write_read(bb_i2c_t *b, uint8_t addr,
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

/* Scan 0x01..0x7F, print results via printf */
static inline int bb_i2c_scan(bb_i2c_t *b, const char *bus_name)
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
