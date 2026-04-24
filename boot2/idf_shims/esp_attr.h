#pragma once
#define IRAM_ATTR
#define DRAM_ATTR
#define RTC_IRAM_ATTR
#define RTC_DATA_ATTR
#define RTC_RODATA_ATTR
#define RTC_NOINIT_ATTR
#define COREDUMP_DRAM_ATTR
#define NOINLINE_ATTR   __attribute__((noinline))
#define FORCE_INLINE_ATTR __attribute__((always_inline)) inline
