#pragma once
#include <stdbool.h>
/* Board uses Winbond W25Q128JVSI — Quad SPI flash, not OPI. This
   affects IDF's PSRAM init paths (spi_flash_set_rom_required_regs /
   spi_flash_set_vendor_required_regs are conditional on this). */
static inline bool bootloader_flash_is_octal_mode_enabled(void) { return false; }
