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

/* --- Cache control — used before jumping into a freshly-loaded app. --- */
void Cache_Invalidate_ICache_All(void);
void Cache_Invalidate_DCache_All(void);

#ifdef __cplusplus
}
#endif
