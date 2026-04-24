#pragma once
#include "esp_err.h"
/* External memory cache freeze/unfreeze — we don't freeze caches since
   we never run concurrent DMA or IPC that would need them. Stubs. */
static inline esp_err_t esp_cache_freeze_ext_mem_cache(void)   { return 0; }
static inline esp_err_t esp_cache_unfreeze_ext_mem_cache(void) { return 0; }
