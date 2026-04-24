/*
 * Minimal libc primitives required by IDF source files we've linked in.
 * Our toolchain has its own libc but we built with -nostdlib and don't
 * link it; these are trivial replacements.
 *
 * Also: the two spi_flash "required regs" hooks that IDF's PSRAM init
 * calls to restore/configure SPI1 after timing tuning. For our
 * board (ESP32-S3R8, OPI flash CS0) the operations match IDF's
 * flash_ops.c bodies, inlined here.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/* --- libc primitives --- */

void *memcpy(void *dst, const void *src, size_t n)
{
    uint8_t *d = dst;
    const uint8_t *s = src;
    while (n--) *d++ = *s++;
    return dst;
}

void *memset(void *dst, int c, size_t n)
{
    uint8_t *d = dst;
    while (n--) *d++ = (uint8_t)c;
    return dst;
}

int memcmp(const void *a, const void *b, size_t n)
{
    const uint8_t *pa = a;
    const uint8_t *pb = b;
    while (n--) {
        if (*pa != *pb) return (int)*pa - (int)*pb;
        ++pa; ++pb;
    }
    return 0;
}

size_t strlen(const char *s)
{
    size_t n = 0;
    while (s[n]) ++n;
    return n;
}

int strcmp(const char *a, const char *b)
{
    while (*a && *a == *b) { ++a; ++b; }
    return (int)(uint8_t)*a - (int)(uint8_t)*b;
}

/* --- Flash IDF shims used only by esp_psram_impl_octal.c --- */

#include "soc/spi_mem_reg.h"
#ifndef SPI_MEM_SPI_FMEM_VAR_DUMMY
/* If the constant isn't reachable, fall back to its known bit. */
#define SPI_MEM_SPI_FMEM_VAR_DUMMY  (1u<<1)
#endif

/* Board uses Winbond W25Q128JVSI — Quad SPI (QIO) flash, NOT OPI.
   IDF's flash_ops.c branches on bootloader_flash_is_octal_mode_enabled():
     - OPI flash: clear FMEM_VAR_DUMMY on SPI1 DDR, set CACHE_USR_CMD_4BYTE on SPI1 FCTRL
     - QIO flash: rom_required does nothing; vendor_required CLEARS CACHE_USR_CMD_4BYTE
   We had the OPI behavior hard-coded, which was setting bits IDF
   explicitly clears for QIO — that disturbed the shared MSPI state
   in a way that broke cache→PSRAM (cache→flash still worked because
   ROM had already set up the flash-side correctly). */

void spi_flash_set_rom_required_regs(void)
{
    /* No-op on QIO flash — IDF only runs this body for OPI flash. */
}

void spi_flash_set_vendor_required_regs(void)
{
    /* Clear CACHE_USR_CMD_4BYTE on SPI1 FCTRL — IDF does this
       explicitly in the QIO-flash branch after PSRAM init. */
    volatile uint32_t *r = (volatile uint32_t *)(uintptr_t)SPI_MEM_CACHE_FCTRL_REG(1);
    *r &= ~(1u << 1);   /* CLEAR SPI_MEM_CACHE_USR_CMD_4BYTE */
}

void __attribute__((noreturn)) abort(void)
{
    for (;;) { __asm__ volatile("" ::: "memory"); }
}
