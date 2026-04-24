#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Brings up octal PSRAM and maps it into DBUS at 0x3C000000.
   Returns 0 on success, negative on error. If `out_size_bytes`
   is non-NULL and init succeeds, it receives the detected chip
   capacity in bytes. */
int psram_init(uint32_t *out_size_bytes);

/* Enable I/DCache after all PSRAM writes complete (cache enable
   kicks off SPI0 activity that blocks SPI1-direct PSRAM writes, so
   we defer it until the loader has finished filling PSRAM). */
void psram_enable_cache(void);

#define PSRAM_DBUS_BASE     0x3C000000u

#ifdef __cplusplus
}
#endif
