#include <stdint.h>
#include <stddef.h>

#include "io.h"
#include "app_header.h"

#define APP_MAGIC 0xDEADBEEFu

/* ESP32-S3 SRAM1 is dual-mapped: the same physical memory is visible at
 * both a DBUS address (0x3FC88000..0x3FCF0000) and an IBUS address
 * (0x40378000..0x403E0000), with a fixed offset of 0x6F0000 between the
 * two. Apps are linked at DBUS so byte-granular rodata reads (l8ui)
 * work, but the CPU can only fetch instructions through the IBUS alias.
 * We translate a DBUS entry pointer to its IBUS alias before calling.
 * PC-relative calls/branches/literals inside the app remain correct
 * because the IBUS and DBUS virtual addresses span the same physical
 * memory with identical relative offsets. */
#define SOC_I_D_OFFSET              0x6F0000u
#define SOC_SRAM1_DBUS_LOW          0x3FC88000u
#define SOC_SRAM1_DBUS_HIGH         0x3FCF0000u

static inline void *dbus_to_ibus_if_sram1(void *p)
{
    uintptr_t a = (uintptr_t)p;
    if (a >= SOC_SRAM1_DBUS_LOW && a < SOC_SRAM1_DBUS_HIGH) {
        return (void *)(a + SOC_I_D_OFFSET);
    }
    return p;
}

/* ESP32-S3 systimer — a free-running 52-bit counter at 16 MHz (80 MHz APB
   divided by 5). Used to synthesize app_context_t.micros_now. Register
   layout per ESP32-S3 TRM §10. */
#define SYSTIMER_BASE            0x60023000u
#define SYSTIMER_UNIT0_OP_REG    (*(volatile uint32_t *)(SYSTIMER_BASE + 0x04))
#define SYSTIMER_UNIT0_VALUE_HI  (*(volatile uint32_t *)(SYSTIMER_BASE + 0x40))
#define SYSTIMER_UNIT0_VALUE_LO  (*(volatile uint32_t *)(SYSTIMER_BASE + 0x44))
#define SYSTIMER_UPDATE_BIT      (1u << 30)
#define SYSTIMER_VALUE_VALID_BIT (1u << 29)

static uint64_t systimer_micros(void)
{
    SYSTIMER_UNIT0_OP_REG = SYSTIMER_UPDATE_BIT;
    while (!(SYSTIMER_UNIT0_OP_REG & SYSTIMER_VALUE_VALID_BIT)) { }
    uint32_t hi = SYSTIMER_UNIT0_VALUE_HI;
    uint32_t lo = SYSTIMER_UNIT0_VALUE_LO;
    uint64_t ticks = ((uint64_t)hi << 32) | lo;
    return ticks / 16u;
}

/* app_context_t declares put_bytes with a non-const buffer; provide a
   matching adapter over io_put_bytes which takes const. */
static void ctx_put_bytes(uint8_t *buf, size_t len)
{
    io_put_bytes(buf, len);
}

#define IDLE_PRINT_INTERVAL_US  1000000u   /* 1 s */

void loader_poll(void)
{
    app_context_t ctx = {
        .printf     = ets_printf,
        .put_bytes  = ctx_put_bytes,
        .micros_now = systimer_micros,
    };

    uint8_t hdr_buf[sizeof(app_image_header_t)];
    size_t bytes_read = 0;
    app_image_header_t *app_hdr = NULL;

    /* 4-byte staging register for payload writes (see comment below). */
    union { uint32_t w; uint8_t b[4]; } stage = { .w = 0 };

    /* Heartbeat: when we've received no bytes yet (bytes_read == 0), print
       an idle tick every IDLE_PRINT_INTERVAL_US so the operator sees the
       loader is alive and which channel is being polled. Suppressed once a
       transfer has started so it can't interleave with the byte stream.

       We track a plain tick counter (instead of deriving seconds from
       systimer_micros) to avoid a 64-bit divide, which would pull in
       __udivdi3 from libgcc — we're linked -nostdlib. */
    uint64_t next_idle_print_us = systimer_micros() + IDLE_PRINT_INTERVAL_US;
    unsigned idle_ticks = 0;

    for (;;) {
        if (bytes_read == 0) {
            uint64_t now = systimer_micros();
            if (now >= next_idle_print_us) {
                idle_ticks++;
                ets_printf("boot2 idle %us\r\n", idle_ticks);
                next_idle_print_us = now + IDLE_PRINT_INTERVAL_US;
            }
        }

        if (bytes_read < sizeof(hdr_buf)) {
            if (io_rxavailable()) {
                hdr_buf[bytes_read++] = io_getchar();
            }
            if (bytes_read == sizeof(hdr_buf)) {
                app_image_header_t *tmp = (app_image_header_t *)hdr_buf;
                if (tmp->magic == APP_MAGIC) {
                    app_hdr = (app_image_header_t *)(uintptr_t)tmp->dest_addr;
                    *app_hdr = *tmp;
                } else {
                    ets_printf("bad magic=0x%x size=%u dest=0x%x entry=0x%x\r\n",
                               (unsigned)tmp->magic, (unsigned)tmp->size,
                               (unsigned)tmp->dest_addr,
                               (unsigned)(uintptr_t)tmp->entry);
                    bytes_read = 0;
                }
            }
        } else if (app_hdr && bytes_read < app_hdr->size) {
            if (io_rxavailable()) {
                uint8_t c = io_getchar();
                size_t payload_idx = bytes_read - sizeof(*app_hdr);
                size_t lane        = payload_idx & 3u;

                /* Stage into a u32 on the stack (DRAM — byte-store OK),
                   flush to dest as an aligned 32-bit write. ESP32-S3
                   SRAM accessed via the IBUS alias (0x403xxxxx) rejects
                   byte/halfword stores; only aligned u32 writes land. */
                stage.b[lane] = c;
                bytes_read++;

                bool word_complete = (lane == 3u);
                bool final_byte    = (bytes_read == app_hdr->size);

                if (word_complete || final_byte) {
                    uint32_t *dst = (uint32_t *)(void *)
                        ((uint8_t *)app_hdr->bytes + (payload_idx & ~3u));
                    *dst = stage.w;
                    stage.w = 0;
                }
            }
            if (bytes_read == app_hdr->size) {
                /* If the app provided a DBUS entry, map it to the IBUS
                   alias so code fetch works. If it gave us an IBUS
                   address already, leave it alone. */
                void (*entry)(app_context_t *) =
                    (void (*)(app_context_t *))
                        dbus_to_ibus_if_sram1((void *)app_hdr->entry);

                ets_printf("jump 0x%x\r\n", (unsigned)(uintptr_t)entry);
                /* Writes above went to DBUS RAM directly; no DCache
                   writeback on this path. Still invalidate ICache so
                   any stale entries for the dest range are dropped. */
                Cache_Invalidate_ICache_All();
                entry(&ctx);
                ets_printf("app exited\r\n");
                app_hdr    = NULL;
                bytes_read = 0;
                stage.w    = 0;
            }
        }
    }
}
