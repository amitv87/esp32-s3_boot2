#include "io.h"

/* ROM's uart_rx_one_char consumes the byte from the FIFO when it reads, so
   implementing a peek-style io_rxavailable requires a 1-byte cache. */
static uint8_t s_peek;
static bool    s_has_peek;

void io_init(void)
{
    /* ROM bootloader already configured UART0 at 115200 8N1 on entry.
       ets_printf also routes to UART0 by default. Nothing to do. */
}

bool io_rxavailable(void)
{
    if (s_has_peek) {
        return true;
    }
    uint8_t c;
    if (uart_rx_one_char(&c) == 0) {
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
    while (uart_rx_one_char(&c) != 0) {
        /* spin */
    }
    return c;
}

void io_put_bytes(const uint8_t *buf, size_t len)
{
    while (len--) {
        uart_tx_one_char(*buf++);
    }
}
