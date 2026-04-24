#include "io.h"

extern void loader_poll(void);
extern void wdt_disable_all(void);

#if BOOT2_PSRAM_OCTAL
#  include "psram.h"
#endif

__attribute__((noreturn))
void boot2_main(void)
{
    /* ROM hands us control with MWDT0, RWDT, and the super-WDT all armed
       with ~1 s "flashboot protection" timeouts. Turn them off before we
       do anything else — the idle poll loop has no reason to feed them. */
    wdt_disable_all();

    io_init();

    /* Give the USB-CDC host time to re-enumerate after reset before we
       emit the banner — otherwise the first line or two gets swallowed
       by terminal programs still attaching. */
    ets_delay_us(2000000);

    ets_printf("boot2 " __DATE__ " " __TIME__ "\r\n");

#if BOOT2_PSRAM_OCTAL
    uint32_t psram_bytes = 0;
    if (psram_init(&psram_bytes) != 0) {
        ets_printf("psram: init failed, continuing without it\r\n");
    }
#endif

    loader_poll();

    /* loader_poll is an infinite loop; if it ever returns, spin. */
    for (;;) { }
}
