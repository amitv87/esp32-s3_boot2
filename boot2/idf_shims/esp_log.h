/*
 * Minimal esp_log.h shim: turn ESP_LOG* / ESP_EARLY_LOG* into no-ops
 * so the copied IDF sources compile without the full logging runtime.
 */
#pragma once

#include <stdint.h>

typedef enum { ESP_LOG_NONE, ESP_LOG_ERROR, ESP_LOG_WARN, ESP_LOG_INFO,
               ESP_LOG_DEBUG, ESP_LOG_VERBOSE } esp_log_level_t;

#define ESP_LOG_ATTR_TAG(name, str)      static const char name[] = str
#define ESP_LOG_ATTR_STR(x)              x

#define ESP_EARLY_LOGE(tag, fmt, ...)    do { (void)tag; } while (0)
#define ESP_EARLY_LOGW(tag, fmt, ...)    do { (void)tag; } while (0)
#define ESP_EARLY_LOGI(tag, fmt, ...)    do { (void)tag; } while (0)
#define ESP_EARLY_LOGD(tag, fmt, ...)    do { (void)tag; } while (0)
#define ESP_EARLY_LOGV(tag, fmt, ...)    do { (void)tag; } while (0)

#define ESP_LOGE(tag, fmt, ...)          do { (void)tag; } while (0)
#define ESP_LOGW(tag, fmt, ...)          do { (void)tag; } while (0)
#define ESP_LOGI(tag, fmt, ...)          do { (void)tag; } while (0)
#define ESP_LOGD(tag, fmt, ...)          do { (void)tag; } while (0)
#define ESP_LOGV(tag, fmt, ...)          do { (void)tag; } while (0)

#define ESP_DRAM_LOGE(tag, fmt, ...)     do { (void)tag; } while (0)
#define ESP_DRAM_LOGW(tag, fmt, ...)     do { (void)tag; } while (0)
#define ESP_DRAM_LOGI(tag, fmt, ...)     do { (void)tag; } while (0)
#define ESP_DRAM_LOGD(tag, fmt, ...)     do { (void)tag; } while (0)
#define ESP_DRAM_LOGV(tag, fmt, ...)     do { (void)tag; } while (0)

#include <inttypes.h>    /* for PRIu32 etc., referenced in IDF log strings */
