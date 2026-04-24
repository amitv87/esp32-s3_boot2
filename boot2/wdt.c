#include <stdint.h>

/*
 * Disable every watchdog ROM leaves enabled:
 *   - MWDT0 (Timer Group 0 watchdog) — fires after ~1 s of no-feed
 *     ("flashboot protection")
 *   - RWDT (RTC watchdog) — second-tier, same purpose
 *   - SWD (super watchdog in RTC_CNTL) — final backstop
 *
 * This mirrors what ESP-IDF's bootloader_config_wdt() +
 * bootloader_super_wdt_auto_feed() do, but done with direct register
 * writes so we avoid pulling in wdt_hal / bootloader_support.
 *
 * Register offsets from components/soc/esp32s3/register/soc/{rtc_cntl,
 * timer_group}_reg.h. Write-protect keys are documented in the TRM.
 */

#define REG32(a) (*(volatile uint32_t *)(uintptr_t)(a))

#define RTCCNTL_BASE              0x60008000u
#define RTC_WDTCONFIG0            (RTCCNTL_BASE + 0x098)
#define RTC_WDTWPROTECT           (RTCCNTL_BASE + 0x0B0)
#define RTC_SWD_CONF              (RTCCNTL_BASE + 0x0B4)
#define RTC_SWD_WPROTECT          (RTCCNTL_BASE + 0x0B8)
#define RTC_WDT_WKEY              0x50D83AA1u
#define RTC_SWD_WKEY              0x8F1D312Au
#define RTC_SWD_AUTO_FEED_EN_BIT  (1u << 31)

#define TIMG0_BASE                0x6001F000u
#define TIMG0_WDTCONFIG0          (TIMG0_BASE + 0x048)
#define TIMG0_WDTWPROTECT         (TIMG0_BASE + 0x064)
#define TIMG_WDT_WKEY             0x50D83AA1u

void wdt_disable_all(void)
{
    /* MWDT0 (TIMG0): unlock, clear config (disables all 4 stages), relock. */
    REG32(TIMG0_WDTWPROTECT) = TIMG_WDT_WKEY;
    REG32(TIMG0_WDTCONFIG0)  = 0;
    REG32(TIMG0_WDTWPROTECT) = 0;

    /* RWDT: same pattern. */
    REG32(RTC_WDTWPROTECT) = RTC_WDT_WKEY;
    REG32(RTC_WDTCONFIG0)  = 0;
    REG32(RTC_WDTWPROTECT) = 0;

    /* SWD can't be disabled on most chips — put it into auto-feed so it
       keeps getting stroked by hardware and never fires. */
    REG32(RTC_SWD_WPROTECT) = RTC_SWD_WKEY;
    REG32(RTC_SWD_CONF)    |= RTC_SWD_AUTO_FEED_EN_BIT;
    REG32(RTC_SWD_WPROTECT) = 0;
}
