#include "io.h"

extern void loader_poll(void);
extern void wdt_disable_all(void);

__attribute__((noreturn))
void boot2_main(void)
{
    /* ROM hands us control with MWDT0, RWDT, and the super-WDT all armed
       with ~1 s "flashboot protection" timeouts. Turn them off before we
       do anything else — the idle poll loop has no reason to feed them. */
    wdt_disable_all();

    io_init();
    ets_printf("boot2 " __DATE__ " " __TIME__ "\r\n");

    loader_poll();

    /* loader_poll is an infinite loop; if it ever returns, spin. */
    for (;;) { }
}
