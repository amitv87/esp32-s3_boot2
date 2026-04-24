#include "io.h"

/* Peek cache: ROM usb_uart_device_rx_one_char consumes on read, same as UART. */
static uint8_t s_peek;
static bool    s_has_peek;

static void usb_putc_shim(char c)
{
    /* ets_printf emits chars one at a time; batch not possible here.
       After each char we strobe tx_flush so the host sees output promptly;
       cost is small since ROM only flushes when the endpoint has space. */
    usb_uart_device_tx_one_char((uint8_t)c);
    usb_uart_device_tx_flush();
}

static void null_putc(char c)
{
    (void)c;
}

void io_init(void)
{
    /* Route ets_printf away from UART0 onto USB-Serial-JTAG. Channel 1 is
       the primary; channel 2 is a secondary that ROM code sometimes uses
       for debug output — null it so we don't dual-print to UART0. */
    ets_install_putc1(usb_putc_shim);
    ets_install_putc2(null_putc);
}

bool io_rxavailable(void)
{
    if (s_has_peek) {
        return true;
    }
    uint8_t c;
    if (usb_uart_device_rx_one_char(&c) == 0) {
        s_peek = c;
        s_has_peek = true;
        return true;
    }
    return false;
}

uint8_t io_getchar(void)
{
    if (s_has_peek) {
        s_has_peek = false;
        return s_peek;
    }
    uint8_t c;
    while (usb_uart_device_rx_one_char(&c) != 0) {
        /* spin */
    }
    return c;
}

void io_put_bytes(const uint8_t *buf, size_t len)
{
    while (len--) {
        usb_uart_device_tx_one_char(*buf++);
    }
    usb_uart_device_tx_flush();
}
