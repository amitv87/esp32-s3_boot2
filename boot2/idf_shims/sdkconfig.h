/*
 * Hand-crafted sdkconfig.h for boot2's port of IDF PSRAM init.
 * Mirrors the subset of CONFIG_* macros that the copied IDF sources
 * reference, with values appropriate for:
 *   - ESP32-S3 (single-core variant flow)
 *   - QSPI flash (GigaDevice GD25Q256, 32 MB, 4-line / quad per eFuse)
 *   - Octal PSRAM (AP-compatible 8 MB observed)
 *   - Flash 80 MHz QIO/STR, PSRAM 120 MHz OPI/DTR (DDR) — PSRAM needs
 *     tuning at this speed; flash does not. IDF's mspi_timing layer
 *     picks core_clk=240 (PSRAM expected) and sets flash divider 3,
 *     PSRAM divider 2. Tuning sweep runs in esp_psram_impl_enable().
 *   - NOTE: do not bump flash to 120M without checking GD25Q256 spec;
 *     part typically caps at 104 MHz QIO STR.
 */
#pragma once

#define CONFIG_IDF_TARGET_ESP32S3               1
#define CONFIG_IDF_TARGET_ARCH_XTENSA           1
#define CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE      1
#define CONFIG_FREERTOS_UNICORE                 1

/* Flash mode & speed: QIO @ 80 MHz @ 16 MB, matches vendor firmware
   (09_LVGL_V8_Test's sdkconfig) which reports "flash io: qio". */
#define CONFIG_ESPTOOLPY_FLASHMODE_QIO          1
#define CONFIG_ESPTOOLPY_FLASH_SAMPLE_MODE_STR  1
#define CONFIG_ESPTOOLPY_FLASHFREQ_80M          1
#define CONFIG_ESPTOOLPY_FLASHSIZE_16MB         1

/* PSRAM */
#define CONFIG_SPIRAM                           1
#define CONFIG_SPIRAM_MODE_OCT                  1
#define CONFIG_SPIRAM_TYPE_AUTO                 1
#define CONFIG_SPIRAM_SPEED_120M                1
#define CONFIG_SPIRAM_SPEED                     120
#define CONFIG_SPIRAM_FETCH_INSTRUCTIONS        0
#define CONFIG_SPIRAM_RODATA                    0
#define CONFIG_SPIRAM_ECC_ENABLE                0
#define CONFIG_SPIRAM_MEMTEST                   0

/* Cache / MMU */
#define CONFIG_MMU_PAGE_SIZE                    0x10000  /* 64 KB */
#define CONFIG_MMU_PAGE_MODE                    "64KB"

/* Referenced by mspi_timing_by_mspi_delay.c::get_working_pll_freq when
   PSRAM is DTR and core clock is 240 MHz. We don't execute the flash
   branch of that scan (flash doesn't need tuning), but the function
   body is still compiled. S3 boots from flash offset 0x0. */
#define CONFIG_BOOTLOADER_OFFSET_IN_FLASH       0x0

/* IDF logging: compile the log macros to no-ops so we don't need the
   full esp_log runtime. We leave ESP_EARLY_LOGE etc. resolvable but
   printing-inert. */
#define CONFIG_LOG_DEFAULT_LEVEL                0
#define CONFIG_LOG_MAXIMUM_LEVEL                0

/* Forward-declare runtime primitives some IDF headers use inline
   without including stdlib.h themselves. In a bare-metal build we
   want these to exist as weak symbols; stdlib.h shim provides an
   inline abort() for anyone who #includes <stdlib.h>, and we declare
   the bare symbol here so headers that call abort() without an
   include still compile. */
#ifdef __cplusplus
extern "C" {
#endif
void abort(void) __attribute__((noreturn));
#ifdef __cplusplus
}
#endif
