/*
 * Hand-crafted sdkconfig.h for boot2's port of IDF PSRAM init.
 * Mirrors the subset of CONFIG_* macros that the copied IDF sources
 * reference, with values appropriate for:
 *   - ESP32-S3 (single-core variant flow)
 *   - OPI flash (Micron 128 Mbit observed)
 *   - Octal PSRAM (AP-compatible 8 MB observed)
 *   - 80 MHz MSPI (no DDR tuning mandatory beyond IDF's default calibration)
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
#define CONFIG_SPIRAM_SPEED_80M                 1
#define CONFIG_SPIRAM_SPEED                     80
#define CONFIG_SPIRAM_FETCH_INSTRUCTIONS        0
#define CONFIG_SPIRAM_RODATA                    0
#define CONFIG_SPIRAM_ECC_ENABLE                0
#define CONFIG_SPIRAM_MEMTEST                   0

/* Cache / MMU */
#define CONFIG_MMU_PAGE_SIZE                    0x10000  /* 64 KB */
#define CONFIG_MMU_PAGE_MODE                    "64KB"

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
