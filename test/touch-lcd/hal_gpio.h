#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ESP32-S3 GPIO/IO_MUX base addresses */
#define GPIO_BASE           0x60004000UL
#define IO_MUX_BASE         0x60009000UL

/* ---- GPIO output set/clear ---- */
/* Pins 0-31 */
#define GPIO_OUT_W1TS_REG    (*(volatile uint32_t *)(GPIO_BASE + 0x008))
#define GPIO_OUT_W1TC_REG    (*(volatile uint32_t *)(GPIO_BASE + 0x00C))
/* Pins 32-53 */
#define GPIO_OUT1_W1TS_REG   (*(volatile uint32_t *)(GPIO_BASE + 0x014))
#define GPIO_OUT1_W1TC_REG   (*(volatile uint32_t *)(GPIO_BASE + 0x018))

/* ---- GPIO input ---- */
#define GPIO_IN_REG          (*(volatile uint32_t *)(GPIO_BASE + 0x03C))  /* pins 0-31  */
#define GPIO_IN1_REG         (*(volatile uint32_t *)(GPIO_BASE + 0x040))  /* pins 32-53 */

/* ---- GPIO output-enable set/clear ---- */
#define GPIO_ENABLE_W1TS_REG  (*(volatile uint32_t *)(GPIO_BASE + 0x024))
#define GPIO_ENABLE_W1TC_REG  (*(volatile uint32_t *)(GPIO_BASE + 0x028))
#define GPIO_ENABLE1_W1TS_REG (*(volatile uint32_t *)(GPIO_BASE + 0x030))
#define GPIO_ENABLE1_W1TC_REG (*(volatile uint32_t *)(GPIO_BASE + 0x034))

/* ---- IO_MUX register for a GPIO pin ----
 * Formula: MUX_BASE + 0x04 + gpio_num * 4 is valid for all ESP32-S3 GPIOs:
 *   GPIO 0-21  → offsets 0x04..0x58  (verified against io_mux_reg.h)
 *   GPIO 22-32 don't exist on ESP32-S3 (address holes, never accessed)
 *   GPIO 33-48 → offsets 0x88..0xC4  (verified: 0x04+33*4=0x88, 47*4+4=0xC0) */
static inline volatile uint32_t *iomux_reg(int gpio_num)
{
    return (volatile uint32_t *)(IO_MUX_BASE + 0x04u + (uint32_t)gpio_num * 4u);
}

/* ---- GPIO matrix output: route peripheral signal to pin ----
 * GPIO_BASE + 0x554 + 4*gpio_num; bits[8:0]=signal, bit[9]=invert */
static inline void gpio_matrix_out(int gpio_num, int sig_idx, bool invert)
{
    volatile uint32_t *r = (volatile uint32_t *)(GPIO_BASE + 0x554u + 4u * (uint32_t)gpio_num);
    *r = (uint32_t)(sig_idx & 0x1FF) | (invert ? (1u << 9) : 0u);
}

/* ---- GPIO matrix input: route pin → peripheral signal ----
 * GPIO_FUNCx_IN_SEL_CFG_REG at GPIO_BASE + 0x154 + 4*sig_idx
 * bits[5:0] = source GPIO, bit 6 = invert, bit 7 = SIG_IN_SEL (1=use matrix) */
static inline void gpio_matrix_in(int gpio_num, int sig_idx, bool invert)
{
    volatile uint32_t *r = (volatile uint32_t *)(GPIO_BASE + 0x154u + 4u * (uint32_t)sig_idx);
    *r = (uint32_t)(gpio_num & 0x3F) | (invert ? (1u << 6) : 0u) | (1u << 7);
}

/* ---- Bank-aware helpers (work for all GPIO 0-53) ---- */

static inline void _gpio_out_set(int n, bool v)
{
    if (n < 32) { if (v) GPIO_OUT_W1TS_REG  = 1u<<n; else GPIO_OUT_W1TC_REG  = 1u<<n; }
    else        { if (v) GPIO_OUT1_W1TS_REG = 1u<<(n-32); else GPIO_OUT1_W1TC_REG = 1u<<(n-32); }
}

static inline void _gpio_oe_set(int n, bool en)
{
    if (n < 32) { if (en) GPIO_ENABLE_W1TS_REG  = 1u<<n; else GPIO_ENABLE_W1TC_REG  = 1u<<n; }
    else        { if (en) GPIO_ENABLE1_W1TS_REG = 1u<<(n-32); else GPIO_ENABLE1_W1TC_REG = 1u<<(n-32); }
}

static inline bool _gpio_in_read(int n)
{
    if (n < 32) return (GPIO_IN_REG  >> n) & 1;
    else        return (GPIO_IN1_REG >> (n - 32)) & 1;
}

/* ---- Public API ---- */

/* Simple output (SIG=0x100 = GPIO output register drives the pin) */
static inline void gpio_output_init(int gpio_num)
{
    *iomux_reg(gpio_num) = (1u << 12) | (2u << 10);  /* MCU_SEL=1, FUN_DRV=2 */
    _gpio_oe_set(gpio_num, true);
    gpio_matrix_out(gpio_num, 0x100, false);
}

/* Open-drain (I2C): input enable + pull-up; OE toggled to drive 0 or release */
static inline void gpio_opendrain_init(int gpio_num)
{
    /* MCU_SEL=1, FUN_DRV=2, FUN_IE=1, FUN_PU=1 */
    *iomux_reg(gpio_num) = (1u << 12) | (2u << 10) | (1u << 9) | (1u << 8);
    _gpio_oe_set(gpio_num, false);          /* start released */
    gpio_matrix_out(gpio_num, 0x100, false);
}

/* Drive a plain output high or low */
static inline void gpio_set(int gpio_num, bool level) { _gpio_out_set(gpio_num, level); }

/* Open-drain drive: high = release (OE off), low = drive 0 (OE on) */
static inline void gpio_od_drive(int gpio_num, bool high)
{
    _gpio_out_set(gpio_num, high);
    _gpio_oe_set(gpio_num, !high);
}

/* Read pin state */
static inline bool gpio_read(int gpio_num) { return _gpio_in_read(gpio_num); }
