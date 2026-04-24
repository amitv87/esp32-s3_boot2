/*
 * Prototypes for ESP32-S3 ROM functions we call from boot2.
 * Symbol addresses are resolved by the linker via the copied
 *   vendor/esp_rom/esp32s3/ld/esp32s3.rom.ld
 *   vendor/esp_rom/esp32s3/ld/esp32s3.rom.api.ld
 * linker scripts. We only declare here what we actually use.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- UART (ROM group "uart"). ROM already initialized UART0 at 115200 8N1
   using XTAL clock on entry, so these work immediately. --- */
int  uart_tx_one_char(uint8_t c);         /* returns 0 (ETS_OK) on success  */
int  uart_rx_one_char(uint8_t *out);      /* returns 0 if byte read, nonzero if FIFO empty */

/* --- USB-Serial-JTAG (ROM group "usb_uart"). Peripheral is live after
   host enumerates; no Uart_Init_USB required for this variant. --- */
int  usb_uart_device_rx_one_char(uint8_t *out);
int  usb_uart_device_tx_one_char(uint8_t c);
void usb_uart_device_tx_flush(void);

/* --- ROM printf. Active UART channel is UART0 by default. For USB-JTAG
   we install a putc hook via ets_install_putc1 / ets_install_putc2 so
   ets_printf output is routed to the USB endpoint instead. --- */
int  ets_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void ets_install_putc1(void (*putc)(char c));
void ets_install_putc2(void (*putc)(char c));

/* --- Busy-wait delay (calibrated by ROM against XTAL). --- */
void ets_delay_us(uint32_t us);

/* --- Cache control — used before jumping into a freshly-loaded app. --- */
void Cache_Invalidate_ICache_All(void);
void Cache_Invalidate_DCache_All(void);
void Cache_Invalidate_Addr(uint32_t addr, uint32_t size);
/* The non-prefixed Cache_Suspend_DCache is an IDF wrapper; the
   underlying ROM symbol is rom_Cache_Suspend_DCache. */
uint32_t rom_Cache_Suspend_DCache(void);
void Cache_Resume_DCache(uint32_t autoload);

/* Full ROM cache init — sets default sizes/ways/line-size, turns caches
   on. ROM normally runs this during early boot; for embedded OPI
   flash variants on S3 it appears to be skipped, leaving DCACHE_CTRL
   master-disabled. We call it ourselves before programming the MMU. */
void ROM_Boot_Cache_Init(void);
/* Partitions the MMU table between ICache and DCache ownership. */
void Cache_Owner_Init(void);
/* Canonical ROM enable functions — do more than just flip the ENABLE
   bit (internal state machine init, invalidate all, etc.). The
   autoload arg is a bitmask; passing 0 disables autoload. */
void Cache_Enable_ICache(uint32_t autoload);
void Cache_Enable_DCache(uint32_t autoload);
void Cache_Disable_ICache(void);
void Cache_Disable_DCache(void);

/* Returns 1 if the given virtual address is routed through the cache
   for read/instruction fetch, 0 if it bypasses cache (in which case
   cache setup for that range is a no-op and transfers never reach
   SPI0). Useful to confirm the cache actually sees our PSRAM VA. */
uint32_t Cache_Address_Through_ICache(uint32_t addr);
uint32_t Cache_Address_Through_DCache(uint32_t addr);
int  Cache_Dbus_MMU_Set(uint32_t ext_ram, uint32_t vaddr, uint32_t paddr,
                         uint32_t psize_kb, uint32_t num_pages, uint32_t fixed);
int  Cache_Ibus_MMU_Set(uint32_t ext_ram, uint32_t vaddr, uint32_t paddr,
                         uint32_t psize_kb, uint32_t num_pages, uint32_t fixed);

/* IDF's cpu_start.c ext_mem_init() calls these to configure the
   ICache and DCache modes (size / ways / line size) before first
   use. Without them, the caches are in an undefined mode that may
   not service external-memory transactions correctly. */
void rom_config_instruction_cache_mode(uint32_t cache_size,
                                        uint8_t ways,
                                        uint8_t line_size);
void rom_config_data_cache_mode(uint32_t cache_size,
                                 uint8_t ways,
                                 uint8_t line_size);
/* Partitions the MMU table between IROM and DROM regions. */
void Cache_Set_IDROM_MMU_Size(uint32_t irom_size, uint32_t drom_size);

/* --- OPI flash/PSRAM helper (used to program PSRAM mode registers). --- */
void esp_rom_opiflash_exec_cmd(int spi_num, int mode,
                                uint32_t cmd, int cmd_bit_len,
                                uint32_t addr, int addr_bit_len,
                                int dummy_bits,
                                uint8_t *mosi_data, int mosi_bit_len,
                                uint8_t *miso_data, int miso_bit_len,
                                uint32_t cs_mask,
                                bool is_write_erase_operation);
void esp_rom_spi_set_dtr_swap_mode(int spi_num, bool wr_swap, bool rd_swap);

/* ROM helpers for the OPI flash chip — first 3 bytes of out_id are
   {vendor, memory_type, capacity_code}. */
int esp_rom_opiflash_read_id(uint8_t *out_id);
int esp_rom_opiflash_read(uint32_t flash_addr, void *data_addr, int len);

#ifdef __cplusplus
}
#endif
