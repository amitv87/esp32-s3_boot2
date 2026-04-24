/*
 * PSRAM variant of the hello-world test app.
 *
 * Identical to test/hello-world/ except it links at 0x3C000000 (the
 * DBUS alias of PSRAM page 0) instead of 0x3FC90000 (internal SRAM1).
 * Requires boot2 built with BOOT2_PSRAM=OCTAL so the PSRAM has been
 * probed and mapped before the image arrives.
 */

#include <stdint.h>
#include <stddef.h>
#include "app_header.h"

/* PSRAM is mapped by boot2 at this DBUS virtual address (matches
   SOC_DRAM0_CACHE_ADDRESS_LOW). Must match ORIGIN in app.ld.
   Apps can use anywhere in 0x3C000000..0x3DFFFFFF (16 MB window). */
#define APP_DEST_ADDR  0x3C000000u

static void app_main(app_context_t *ctx);

extern char __app_image_size_sym[];

__attribute__((section(".app_header"), used))
const app_image_header_t __app_hdr = {
    .magic     = 0xDEADBEEFu,
    .size      = (uint32_t)(uintptr_t)__app_image_size_sym,
    .dest_addr = APP_DEST_ADDR,
    .entry     = app_main,
};

static void delay_us(uint64_t us, uint64_t (*micros_now)(void))
{
    uint64_t start = micros_now();
    while (micros_now() - start < us) { }
}

static void app_main(app_context_t *ctx)
{
    ctx->printf("\r\n=== Hello from PSRAM ===\r\n");
    for (int tick = 0; ; tick++) {
        ctx->printf("psram tick %d\r\n", tick);
        delay_us(1000000ULL, ctx->micros_now);
    }
}
