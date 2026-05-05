#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "hal_gpio.h"

/*
 * Generic I2C for any two GPIO pins (open-drain, 7-bit addresses).
 * API matches the original bit-bang version.  Implementation in hal_i2c.c
 * picks between bit-banged (legacy) and ESP32-S3 hardware I2C0/I2C1.
 *
 * Port assignment (HW backend): SDA=17 → I2C0, SDA=47 → I2C1.
 */

typedef struct { int sda, scl, port; } bb_i2c_t;

void bb_i2c_init(bb_i2c_t *b, int sda, int scl);
bool bb_i2c_probe(bb_i2c_t *b, uint8_t addr);
bool bb_i2c_write(bb_i2c_t *b, uint8_t addr, const uint8_t *data, int len);
bool bb_i2c_write_reg(bb_i2c_t *b, uint8_t addr, uint8_t reg, uint8_t val);
bool bb_i2c_write_read(bb_i2c_t *b, uint8_t addr,
                       const uint8_t *wbuf, int wlen,
                       uint8_t *rbuf, int rlen);
int  bb_i2c_scan(bb_i2c_t *b, const char *bus_name);
