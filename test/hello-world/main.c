/*
 * Hello-world test app for ESP32-S3 boot2.
 *
 * Wire format (boot2/loader.c expects):
 *   [magic=0xDEADBEEF : 4]
 *   [size              : 4]   total bytes (header + payload)
 *   [dest_addr         : 4]   where boot2 copies this image
 *   [entry             : 4]   boot2 calls entry(&ctx)
 *   [payload bytes...]
 *
 * The binary produced by this file IS the raw wire stream. Boot2 reads
 * the first 16 bytes as the header, then streams the remaining
 * (size - 16) bytes into dest_addr, then invalidates ICache and jumps
 * to entry with an app_context_t* that gives us printf / micros_now.
 *
 * The image is linked at APP_DEST_ADDR, so `entry` is just &app_main.
 * `size` is patched in by the linker via __app_image_size_sym.
 */

#include <stdint.h>
#include <stddef.h>
#include "app_header.h"

/* Where boot2 drops us.
 *
 * Link address must be the **DBUS alias** of SRAM1 (0x3FCxxxxx), not the
 * IBUS alias (0x403xxxxx): on ESP32-S3 the IBUS view of internal SRAM
 * only permits 32-bit aligned loads/stores. Our rodata (format strings)
 * is read byte-by-byte by ROM's ets_printf via `l8ui`, which faults on
 * IBUS but works on DBUS. SRAM1 is physically the same memory whether
 * accessed via DBUS or IBUS, so the CPU can still fetch instructions
 * from this address.
 *
 * 0x3FC90000 sits well below boot2's DRAM (0x3FCE2700+) and well below
 * the ROM shared buffer range (0x3FCD7E00+). Must match app.ld ORIGIN.
 */
#define APP_DEST_ADDR  0x3FC90000u

static void app_main(app_context_t *ctx);

/* Linker-provided symbol whose *address value* == total image size.
   See `__app_image_size_sym = ...` in app.ld. */
extern char __app_image_size_sym[];

__attribute__((section(".app_header"), used))
const app_image_header_t __app_hdr = {
    .magic     = 0xDEADBEEFu,
    .size      = (uint32_t)(uintptr_t)__app_image_size_sym,
    .dest_addr = APP_DEST_ADDR,
    .entry     = app_main,
};

/* Busy-wait using the ctx-provided microsecond clock (systimer-backed). */
static void delay_us(uint64_t us, uint64_t (*micros_now)(void))
{
    uint64_t start = micros_now();
    while (micros_now() - start < us) {
        /* spin */
    }
}

static void app_main(app_context_t *ctx)
{
    ctx->printf("\r\n=== ESP32-S3 Hello World ===\r\n");
    for (int tick = 0; ; tick++) {
        ctx->printf("Tick %d\r\n", tick);
        delay_us(1000000ULL, ctx->micros_now);
    }
}
