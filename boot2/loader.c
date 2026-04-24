#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "io.h"
#include "app_header.h"
#include "rom_symbols.h"

#define APP_MAGIC 0xDEADBEEFu

/* OPI PSRAM write/read commands + parameters (mirror boot2/psram.c) */
#define OPI_PSRAM_SYNC_READ             0x0000
#define OPI_PSRAM_SYNC_WRITE            0x8080
#define OCT_PSRAM_CMD_BITLEN            16
#define OCT_PSRAM_ADDR_BITLEN           32
#define OCT_PSRAM_RD_DUMMY_BITLEN       (2 * (10 - 1))
#define OCT_PSRAM_WR_DUMMY_BITLEN       (2 * (5  - 1))
#define ESP_ROM_SPIFLASH_OPI_DTR_MODE   7

/* Apps are linked at a DBUS address so their rodata is byte-accessible,
 * but the CPU can only fetch instructions through an IBUS alias. We
 * translate a DBUS entry pointer to its IBUS alias immediately before
 * calling. PC-relative calls/branches/literals inside the app stay
 * correct because the IBUS and DBUS ranges address the same physical
 * bytes with identical relative offsets.
 *
 * Two cases supported:
 *   SRAM1 (internal):  DBUS 0x3FC88000..0x3FCF0000 ↔ IBUS +0x6F0000
 *   PSRAM (external):  DBUS 0x3C000000..0x3E000000 ↔ IBUS +0x06000000
 *                      (both bases configured by psram.c via Cache_*_MMU_Set) */
#define SRAM1_DBUS_LOW       0x3FC88000u
#define SRAM1_DBUS_HIGH      0x3FCF0000u
#define SRAM1_I_D_OFFSET     0x6F0000u

#define PSRAM_DBUS_LOW       0x3C000000u
#define PSRAM_DBUS_HIGH      0x3E000000u
#define PSRAM_I_D_OFFSET     0x06000000u   /* 0x42000000 - 0x3C000000 */

static inline void *dbus_to_ibus(void *p)
{
    uintptr_t a = (uintptr_t)p;
    if (a >= SRAM1_DBUS_LOW && a < SRAM1_DBUS_HIGH) {
        return (void *)(a + SRAM1_I_D_OFFSET);
    }
    if (a >= PSRAM_DBUS_LOW && a < PSRAM_DBUS_HIGH) {
        return (void *)(a + PSRAM_I_D_OFFSET);
    }
    return p;
}

static inline bool addr_in_psram_dbus(uint32_t a)
{
    return a >= PSRAM_DBUS_LOW && a < PSRAM_DBUS_HIGH;
}

/* Write a 32-bit word directly to PSRAM via SPI1/CS1 using the ROM OPI
   exec helper. Used when the destination address is in the PSRAM
   virtual range — the cache-mediated path is not yet working on this
   chip, but the SPI1-direct path is proven reliable across the full
   capacity by psram.c's connectivity probes. */
static void psram_spi1_write32(uint32_t phys_off, uint32_t value)
{
    esp_rom_opiflash_exec_cmd(1, ESP_ROM_SPIFLASH_OPI_DTR_MODE,
                              OPI_PSRAM_SYNC_WRITE, OCT_PSRAM_CMD_BITLEN,
                              phys_off, OCT_PSRAM_ADDR_BITLEN,
                              OCT_PSRAM_WR_DUMMY_BITLEN,
                              (uint8_t *)&value, 32,
                              NULL, 0,
                              1u << 1, false);
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
    /* When the destination is PSRAM, we bypass the cache/MMU write
       path and stream the payload to the chip through SPI1 using the
       OPI SYNC_WRITE command. This mirror-copies the entire image
       (header + payload) to PSRAM physical offset `psram_off` so that
       CPU instruction fetch via the IBUS MMU mapping lands on valid
       bytes. */
    bool     dest_is_psram = false;
    uint32_t psram_off     = 0;
    /* Cache the header fields locally. For PSRAM destinations we
       must NOT read back through app_hdr->* since that goes through
       the (still-suspended) cache and hangs. */
    uint32_t app_size      = 0;
    void    *app_entry     = NULL;

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
                    dest_is_psram = addr_in_psram_dbus(tmp->dest_addr);
                    psram_off = dest_is_psram
                                ? (tmp->dest_addr - PSRAM_DBUS_LOW) : 0;
                    app_size  = tmp->size;
                    app_entry = (void *)tmp->entry;

                    if (dest_is_psram) {
                        /* Header-sized mirror at PSRAM physical offset
                           via SPI1, one 32-bit word at a time. The
                           struct is 16 bytes so exactly 4 words. */
                        const uint32_t *hw = (const uint32_t *)hdr_buf;
                        for (unsigned w = 0; w < 4; ++w) {
                            psram_spi1_write32(psram_off + w * 4, hw[w]);
                        }
                    } else {
                        /* Internal-SRAM destination: struct copy via
                           CPU store (cache-less internal SRAM). */
                        *app_hdr = *tmp;
                    }
                } else {
                    ets_printf("bad magic=0x%x size=%u dest=0x%x entry=0x%x\r\n",
                               (unsigned)tmp->magic, (unsigned)tmp->size,
                               (unsigned)tmp->dest_addr,
                               (unsigned)(uintptr_t)tmp->entry);
                    bytes_read = 0;
                }
            }
        } else if (app_hdr && bytes_read < app_size) {
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
                    if (dest_is_psram) {
                        /* PSRAM target: SPI1-direct write at the
                           physical offset (header is at psram_off..+16,
                           so payload[0] lands at psram_off + 16). */
                        psram_spi1_write32(
                            psram_off + sizeof(*app_hdr) +
                                (payload_idx & ~3u),
                            stage.w);
                    } else {
                        uint32_t *dst = (uint32_t *)(void *)
                            ((uint8_t *)app_hdr->bytes + (payload_idx & ~3u));
                        *dst = stage.w;
                    }
                    stage.w = 0;
                }
            }
            if (bytes_read == app_size) {
                if (dest_is_psram) {
                    extern void psram_enable_cache(void);
                    psram_enable_cache();
                }

                /* Translate DBUS entry to IBUS for code fetch. Use the
                   locally-cached entry pointer — never read through
                   app_hdr->entry for PSRAM destinations. */
                void (*entry)(app_context_t *) =
                    (void (*)(app_context_t *))dbus_to_ibus(app_entry);

                ets_printf("jump 0x%x\r\n", (unsigned)(uintptr_t)entry);
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
