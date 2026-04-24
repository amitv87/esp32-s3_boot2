#pragma once

/*
 * Recalibrate BBPLL and switch CPU to PLL 240 MHz.
 * Must be called from IRAM before any MSPI / PSRAM / cache operations.
 */
void bbpll_recalib_and_set_cpu_240mhz(void);
