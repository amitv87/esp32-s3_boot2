/*
 * Thin wrappers over ROM cache symbols whose non-prefixed names IDF
 * expects (cache_hal.c / cache_ll.h call e.g. Cache_Suspend_ICache)
 * but the ROM only exports with the `rom_` prefix on ESP32-S3.
 *
 * IDF normally provides these as patches in esp_rom/patches/; here we
 * just forward to the ROM ones directly.
 */

#include <stdint.h>

extern uint32_t rom_Cache_Suspend_ICache(void);
extern uint32_t rom_Cache_Suspend_DCache(void);
extern void     rom_Cache_WriteBack_Addr(uint32_t addr, uint32_t size);
extern void     rom_Cache_Freeze_ICache_Enable(int mode);
extern void     rom_Cache_Freeze_DCache_Enable(int mode);

uint32_t Cache_Suspend_ICache(void) { return rom_Cache_Suspend_ICache(); }
uint32_t Cache_Suspend_DCache(void) { return rom_Cache_Suspend_DCache(); }
void Cache_WriteBack_Addr(uint32_t addr, uint32_t size) {
    rom_Cache_WriteBack_Addr(addr, size);
}
void Cache_Freeze_ICache_Enable(int mode) { rom_Cache_Freeze_ICache_Enable(mode); }
void Cache_Freeze_DCache_Enable(int mode) { rom_Cache_Freeze_DCache_Enable(mode); }
