#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "rom_symbols.h"   /* exposes ets_printf for callers */

#ifdef __cplusplus
extern "C" {
#endif

void    io_init(void);
bool    io_rxavailable(void);
uint8_t io_getchar(void);
void    io_put_bytes(const uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif
