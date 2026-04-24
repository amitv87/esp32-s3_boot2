/*
 * boot2 PSRAM init, following IDF v5.4.2's bootloader_flash_hardware_init
 * (for RAM-loaded apps, our case) before calling esp_psram_impl_enable.
 *
 * Reference: components/bootloader_support/bootloader_flash/src/
 *            bootloader_flash_config_esp32s3.c  line ~333
 *
 * That function is guarded by #if CONFIG_APP_BUILD_TYPE_RAM — i.e. it
 * exists precisely for code that's loaded into RAM (no bootloader
 * runs) and needs to bring SPI0 / cache / MMU up to the same state
 * the full IDF bootloader would have produced. Before we mirrored it,
 * our boot2 was doing a subset, and our PSRAM init saw inconsistent
 * MSPI state — SPI1 mode-register reads came back byte-misaligned
 * and cache→SPI0→PSRAM transfers hung.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "psram.h"
#include "clk_init.h"
#include "rom_symbols.h"

/* IDF private APIs (brought in by CMakeLists include dirs) */
#include "esp_private/esp_psram_impl.h"
#include "esp_private/mspi_timing_tuning.h"
#include "hal/cache_hal.h"
#include "hal/cache_types.h"
#include "hal/mmu_hal.h"

/* ROM function — declared manually to avoid pulling in esp32s3/rom/opi_flash.h
   which transitively includes spi_flash.h and conflicts with our extern decls. */
extern void esp_rom_opiflash_pin_config(void);

#define BIT(n)                      (1u << (n))
#define SOC_MMU_ACCESS_SPIRAM       BIT(15)
#define PSRAM_DBUS_VADDR            0x3C000000u
#define PSRAM_IBUS_VADDR            0x42000000u
#define MMU_PAGE_SIZE_KB            64

/* ---- SPI0/SPI1 register helpers (direct offsets) ---- */
#define SPI0                        0x60003000u
#define SPI1                        0x60002000u
#define SPI_MEM_CTRL_REG(i)         ((i) ? SPI1 : SPI0)                 /* +0x08 */
#define SPI_MEM_CTRL2_REG(i)        (((i) ? SPI1 : SPI0) + 0x10)
#define SPI_MEM_USER_REG(i)         (((i) ? SPI1 : SPI0) + 0x18)
#define SPI_MEM_FDUMMY_OUT          BIT(17)
#define SPI_MEM_D_POL               BIT(19)
#define SPI_MEM_Q_POL               BIT(18)
#define SPI_MEM_CS_HOLD_M           BIT(6)
#define SPI_MEM_CS_SETUP_M          BIT(7)
#define SPI_MEM_CS_HOLD_TIME_V      0x1F
#define SPI_MEM_CS_HOLD_TIME_S      0
#define SPI_MEM_CS_SETUP_TIME_V     0x1F
#define SPI_MEM_CS_SETUP_TIME_S     5

static inline uint32_t RD32(uintptr_t a) { return *(volatile uint32_t *)a; }
static inline void WR32(uintptr_t a, uint32_t v) { *(volatile uint32_t *)a = v; }
static inline void R32_OR(uintptr_t a, uint32_t m) { *(volatile uint32_t *)a |= m; }
static inline void R32_FIELD(uintptr_t a, uint32_t mask, uint32_t shift, uint32_t v)
{
    uint32_t t = *(volatile uint32_t *)a;
    t &= ~(mask << shift);
    t |= (v & mask) << shift;
    *(volatile uint32_t *)a = t;
}

/* ---- ROM functions we call from IDF's RAM-app init path ---- */
extern void esp_rom_spiflash_attach(uint32_t spiconfig, bool legacy);
extern int  esp_rom_spiflash_config_readmode(int mode);
extern int  esp_rom_spiflash_config_clk(uint8_t div, uint8_t spi_num);
extern int  esp_rom_spiflash_unlock(void);

/* ESP_ROM_SPIFLASH_*_MODE values (from rom/spi_flash.h) */
#define ESP_ROM_SPIFLASH_QIO_MODE   0
#define ESP_ROM_SPIFLASH_QOUT_MODE  1
#define ESP_ROM_SPIFLASH_DIO_MODE   2
#define ESP_ROM_SPIFLASH_DOUT_MODE  3
#define ESP_ROM_SPIFLASH_FASTRD_MODE 4

/* IDF v5.4.2 bootloader_flash_set_dummy_out — enable flash dummy output
   on SPI0 and SPI1. Same for OPI and non-OPI flashes. */
static void bl_flash_set_dummy_out(void)
{
    R32_OR(SPI_MEM_CTRL_REG(0), SPI_MEM_FDUMMY_OUT | SPI_MEM_D_POL | SPI_MEM_Q_POL);
    R32_OR(SPI_MEM_CTRL_REG(1), SPI_MEM_FDUMMY_OUT | SPI_MEM_D_POL | SPI_MEM_Q_POL);
}

/* IDF v5.4.2 bootloader_flash_cs_timing_config — non-OPI branch
   (QIO/DIO/etc. all use this branch). Sets CS_HOLD_TIME=0,
   CS_SETUP_TIME=0, enables CS_HOLD + CS_SETUP in USER_REG. */
static void bl_flash_cs_timing_config_non_opi(void)
{
    R32_FIELD(SPI_MEM_CTRL2_REG(0), SPI_MEM_CS_HOLD_TIME_V,  SPI_MEM_CS_HOLD_TIME_S,  0);
    R32_FIELD(SPI_MEM_CTRL2_REG(0), SPI_MEM_CS_SETUP_TIME_V, SPI_MEM_CS_SETUP_TIME_S, 0);
    R32_OR(SPI_MEM_USER_REG(0), SPI_MEM_CS_HOLD_M | SPI_MEM_CS_SETUP_M);
}

int psram_init(uint32_t *out_size_bytes)
{
    /* === RAM-app init sequence.
         Ordering follows IDF v5.4.2 call_start_cpu0 / bootloader_init. === */

    /* 1. Recalibrate BBPLL and switch CPU to 80 MHz from PLL.
       ROM leaves BBPLL poorly calibrated; without this, MSPI timing is off. */
    bbpll_recalib_and_set_cpu_240mhz();

    /* 2. Cache and MMU init BEFORE spiflash_attach.
       mmu_hal_init calls ROM_Boot_Cache_Init (ESP_ROM_RAM_APP_NEEDS_MMU_INIT=1
       on S3), which wipes MMU state and resets MSPI SPI1 registers. It MUST
       run before spiflash_attach, not after, or it clobbers MSPI config and
       leaves SPI1's SRAM clock in an undefined state. This matches IDF's
       bootloader_init_ext_mem → bootloader_flash_hardware_init order. */
    cache_hal_config_t cache_cfg = { .core_nums = 1 };
    cache_hal_init(&cache_cfg);
    mmu_hal_config_t mmu_cfg = { .core_nums = 1, .mmu_page_size = 0x10000 };
    mmu_hal_init(&mmu_cfg);

    /* 3. Attach MSPI pins via ROM. spiconfig=0 means "use efuse default".
       Now safe: ROM_Boot_Cache_Init has already run above. */
    esp_rom_spiflash_attach(0, false);

    /* 4. Set flash read mode to QIO — matches vendor firmware. */
    esp_rom_spiflash_config_readmode(ESP_ROM_SPIFLASH_QIO_MODE);

    /* 5. Flash clock: divider 2 (80 MHz from 160 MHz MSPI core), on
       both SPI0 (cache) and SPI1 (direct). */
    esp_rom_spiflash_config_clk(2, 0);
    esp_rom_spiflash_config_clk(2, 1);

    /* 6. Flash dummy-out bits and CS timing. */
    bl_flash_set_dummy_out();
    bl_flash_cs_timing_config_non_opi();

    /* 7. Unlock flash (clear volatile WP bits). */
    esp_rom_spiflash_unlock();

    /* 8. Second cache/MMU init (ext_mem_init pattern): re-init cache AFTER
       flash is configured so the controller picks up the new clock/mode.
       Use mmu_hal_ctx_init (software context only) — no second ROM call. */
    cache_hal_init(&cache_cfg);
    mmu_hal_ctx_init(&mmu_cfg);

    /* 8b. Cache mode config (ext_mem_init): 16 KB icache, 32 KB dcache,
       8-way, 32-byte lines. Matches vendor CONFIG_ESP32S3_*_CACHE_*. */
    rom_config_instruction_cache_mode(16 * 1024, 8, 32);
    rom_Cache_Suspend_DCache();
    rom_config_data_cache_mode(32 * 1024, 8, 32);
    Cache_Resume_DCache(0);
    /* 8c. MMU partition: irom_size=0 for RAM-app, drom_size = full table. */
    Cache_Set_IDROM_MMU_Size(0, 512 * 4);

    /* 9. Cache disable/enable dance — flush stale state before PSRAM init. */
    cache_hal_disable(1 /* EXT_MEM */, CACHE_TYPE_ALL);
    cache_hal_enable(1 /* EXT_MEM */, CACHE_TYPE_ALL);

    /* 10. OPI MSPI pin init — MUST run before esp_psram_impl_enable().
       esp_rom_opiflash_pin_config() configures GPIO26 as SPICS1 (PSRAM chip
       select) and GPIO33-40 as the 8-bit OPI data bus for both SPI0 and SPI1.
       Without this, esp_rom_opiflash_exec_cmd(1, OPI_DTR_MODE, ...) in the
       PSRAM MR read path asserts a CS pin that isn't routed to PSRAM — the
       mode register reads return flash data instead (vendor_id=0x1C, 16 MB).
       mspi_timing_set_pin_drive_strength() sets drive strength=3 on all MSPI
       pins; needed at 80 MHz OPI to meet setup/hold margins. */
    esp_rom_opiflash_pin_config();
    mspi_timing_set_pin_drive_strength();

    ets_printf("psram: RAM-app flash init complete, running esp_psram_impl_enable...\r\n");

    /* 11. Run IDF's PSRAM init verbatim. SPI0/1, cache, and MMU are now
       in the same state as they would be at the start of a v5.4.2 bootloader,
       which is exactly the precondition esp_psram_impl_enable expects. */
    esp_err_t rc = esp_psram_impl_enable();
    if (rc != 0) {
        ets_printf("psram: IDF bring-up failed, rc=%d\r\n", (int)rc);
        return -1;
    }

    uint32_t size_bytes = 0;
    esp_psram_impl_get_physical_size(&size_bytes);
    ets_printf("psram: IDF reports size=%u KB\r\n", (unsigned)(size_bytes >> 10));

    /* --- Map PSRAM into DBUS + IBUS address spaces. --- */
    uint32_t pages = size_bytes / (MMU_PAGE_SIZE_KB * 1024u);

    rom_Cache_Suspend_DCache();
    int r1 = Cache_Dbus_MMU_Set(SOC_MMU_ACCESS_SPIRAM, PSRAM_DBUS_VADDR,
                                0, MMU_PAGE_SIZE_KB, pages, 0);
    *(volatile uint32_t *)0x600C400Cu &= ~0x3u;   /* unshut DBUS */
    Cache_Resume_DCache(0);

    int r2 = Cache_Ibus_MMU_Set(SOC_MMU_ACCESS_SPIRAM, PSRAM_IBUS_VADDR,
                                0, MMU_PAGE_SIZE_KB, pages, 0);
    Cache_Invalidate_Addr(PSRAM_DBUS_VADDR, size_bytes);
    Cache_Invalidate_Addr(PSRAM_IBUS_VADDR, size_bytes);

    ets_printf("psram: MMU dbus=0x%x rc=%d  ibus=0x%x rc=%d  (%u pages)\r\n",
               (unsigned)PSRAM_DBUS_VADDR, r1,
               (unsigned)PSRAM_IBUS_VADDR, r2, (unsigned)pages);

    /* --- Cache-mediated probe --- */
    volatile uint32_t *p = (volatile uint32_t *)PSRAM_DBUS_VADDR;
    ets_printf("psram: cache probe W1 ...\r\n");
    p[0] = 0xCAFEBABEu;
    ets_printf("psram: cache probe W2 ...\r\n");
    p[1] = 0xDEADBEEFu;
    ets_printf("psram: cache probe R1 ...\r\n");
    uint32_t v0 = p[0];
    ets_printf("psram: cache probe R2 ...\r\n");
    uint32_t v1 = p[1];
    ets_printf("psram: cache probe result [0]=0x%x [1]=0x%x %s\r\n",
               (unsigned)v0, (unsigned)v1,
               (v0 == 0xCAFEBABEu && v1 == 0xDEADBEEFu) ? "PASS" : "FAIL");

    if (out_size_bytes) *out_size_bytes = size_bytes;
    return 0;
}

void psram_enable_cache(void) { /* IDF enabled the cache already */ }
