/*
 * BBPLL recalibration + CPU clock init for boot2.
 *
 * ROM leaves the BBPLL in an imprecisely-calibrated state. Without
 * recalibration, MSPI timing is off and cache-mediated PSRAM access
 * either returns wrong data or hangs forever.
 *
 * Mirrors IDF v5.4.2:
 *   recalib_bbpll()            (esp_system/port/soc/esp32s3/clk.c:362)
 *   rtc_clk_bbpll_configure()  (esp_hw_support/port/esp32s3/rtc_clk.c:185)
 *   rtc_clk_cpu_freq_to_pll_mhz() (rtc_clk.c:207)
 *   regi2c_ctrl_ll_i2c_*()    (esp_hal_regi2c/esp32s3/include/hal/regi2c_ctrl_ll.h)
 *   clk_ll_bbpll_*()           (esp_hal_clock/esp32s3/include/hal/clk_tree_ll.h)
 */

#include <stdint.h>
#include "clk_init.h"
#include "esp_rom_regi2c.h"    /* esp_rom_regi2c_write / esp_rom_regi2c_write_mask */

/* ---- Register addresses (ESP32-S3 TRM, confirmed against IDF register/soc/) ---- */

/* SYSTEM peripheral — CPU and PLL clock mux */
#define SYSTEM_CPU_PER_CONF_REG     0x600C0010u   /* DR_REG_SYSTEM_BASE + 0x10 */
#define SYSTEM_SYSCLK_CONF_REG      0x600C0060u   /* DR_REG_SYSTEM_BASE + 0x60 */

/* RTC_CNTL — BBPLL power gating */
#define RTC_CNTL_OPTIONS0_REG       0x60008000u   /* DR_REG_RTCCNTL_BASE + 0x0 */
#define RTC_CNTL_BB_I2C_FORCE_PD   (1u << 6)
#define RTC_CNTL_BBPLL_I2C_FORCE_PD (1u << 8)
#define RTC_CNTL_BBPLL_FORCE_PD    (1u << 10)
#define BBPLL_PD_MASK               (RTC_CNTL_BB_I2C_FORCE_PD | \
                                     RTC_CNTL_BBPLL_I2C_FORCE_PD | \
                                     RTC_CNTL_BBPLL_FORCE_PD)

/* RTC_CNTL_DATE_REG — LDO slave power-down control */
#define RTC_CNTL_DATE_REG           0x600081FCu   /* DR_REG_RTCCNTL_BASE + 0x1FC */
#define RTC_CNTL_SLAVE_PD_S         13
#define RTC_CNTL_SLAVE_PD_MASK      (0x3Fu << RTC_CNTL_SLAVE_PD_S)
#define DEFAULT_LDO_SLAVE           0x7u          /* all 6 slaves on */

/* I2C master analog config — BBPLL calibration control */
#define I2C_MST_ANA_CONF0_REG       0x6000E040u
#define BBPLL_STOP_FORCE_HIGH       (1u << 2)
#define BBPLL_STOP_FORCE_LOW        (1u << 3)
#define BBPLL_CAL_DONE              (1u << 24)

/* Analog config — regi2c bus enable/reset */
#define ANA_CONFIG_REG              0x6000E044u
#define ANA_I2C_ALL_RST_MASK        (0x3FFu << 8)  /* bits [17:8]: reset all i2c channels */
#define ANA_I2C_BBPLL_EN_BIT        (1u << 17)     /* 0 = BBPLL i2c enabled */

/* SYSTEM_CPU_PER_CONF_REG fields */
#define SYS_PLL_FREQ_SEL_BIT        (1u << 2)      /* 1 = 480 MHz PLL */
#define SYS_CPUPERIOD_SEL_S         0
#define SYS_CPUPERIOD_SEL_MASK      (0x3u << SYS_CPUPERIOD_SEL_S)
#define SYS_CPUPERIOD_SEL_80M       0u
#define SYS_CPUPERIOD_SEL_160M      1u
#define SYS_CPUPERIOD_SEL_240M      2u

/* SYSTEM_SYSCLK_CONF_REG fields */
#define SYS_PRE_DIV_CNT_S           0
#define SYS_PRE_DIV_CNT_MASK        (0x3FFu << SYS_PRE_DIV_CNT_S)
#define SYS_SOC_CLK_SEL_S           10
#define SYS_SOC_CLK_SEL_MASK        (0x3u << SYS_SOC_CLK_SEL_S)
#define SYS_SOC_CLK_SEL_XTAL        0u
#define SYS_SOC_CLK_SEL_PLL         1u

/* I2C_DIG_REG block — digital/RTC LDO bias (soc/esp32s3/include/soc/regi2c_dig_reg.h) */
#define I2C_DIG_REG                 0x6D
#define I2C_DIG_REG_HOSTID          1
#define I2C_DIG_REG_EXT_RTC_DREG   4    /* reg 4, bits[4:0] */
#define I2C_DIG_REG_EXT_DIG_DREG   6    /* reg 6, bits[4:0] */
#define DIG_DBIAS_240M              28   /* safe default; IDF reads this from efuse at runtime */

/* I2C_BBPLL block constants (soc/esp32s3/include/soc/regi2c_bbpll.h) */
#define I2C_BBPLL                   0x66
#define I2C_BBPLL_HOSTID            1
#define I2C_BBPLL_MODE_HF           4    /* reg 4, full byte */
#define I2C_BBPLL_OC_REF_DIV        2    /* reg 2, full byte: (dchgp<<4)|div_ref */
#define I2C_BBPLL_OC_DIV_7_0        3    /* reg 3, full byte: div7_0 */
#define I2C_BBPLL_OC_DR1_REG        5    /* reg 5, bits[2:0] */
#define I2C_BBPLL_OC_DR3_REG        5    /* reg 5, bits[6:4] */
#define I2C_BBPLL_OC_DCUR           6    /* reg 6, full byte: (dlref<<6)|(dhref<<4)|dcur */
#define I2C_BBPLL_OC_VCO_DBIAS_REG  9   /* reg 9, bits[1:0] */

/* ROM delay + clock frequency update */
extern void ets_delay_us(uint32_t us);
extern void ets_update_cpu_frequency(uint32_t mhz);

/* ---- Register helpers ---- */
static inline uint32_t RD(uintptr_t a)
{
    return *(volatile uint32_t *)a;
}
static inline void WR(uintptr_t a, uint32_t v)
{
    *(volatile uint32_t *)a = v;
}
static inline void R_SET(uintptr_t a, uint32_t m)
{
    *(volatile uint32_t *)a |= m;
}
static inline void R_CLR(uintptr_t a, uint32_t m)
{
    *(volatile uint32_t *)a &= ~m;
}
static inline void R_FIELD(uintptr_t a, uint32_t mask, uint32_t v)
{
    uint32_t t = *(volatile uint32_t *)a;
    *(volatile uint32_t *)a = (t & ~mask) | (v & mask);
}

/*
 * bbpll_recalib_and_set_cpu_80mhz — recalibrate BBPLL and switch CPU to
 * 480 MHz PLL / 80 MHz CPU.
 *
 * Must be called from IRAM (no flash access is safe while CPU clock
 * transitions through XTAL or while BBPLL is briefly powered down).
 * boot2 runs entirely from IRAM, so this is safe unconditionally.
 *
 * Call this before any MSPI/PSRAM/cache operations.
 */
void bbpll_recalib_and_set_cpu_240mhz(void)
{
    /* 1. Reset all regi2c analog bus channels, then re-enable just BBPLL.
     *    mirrors: regi2c_ctrl_ll_i2c_reset() + regi2c_ctrl_ll_i2c_bbpll_enable()
     *    (rtc_clk_init.c calls these before rtc_clk_cpu_freq_set_config) */
    R_SET(ANA_CONFIG_REG, ANA_I2C_ALL_RST_MASK);  /* set bits[17:8] → reset all */
    R_CLR(ANA_CONFIG_REG, ANA_I2C_BBPLL_EN_BIT);  /* clear bit 17 → BBPLL i2c on */

    /* 2. Switch CPU source to XTAL 40 MHz (PRE_DIV_CNT=0 → div=1).
     *    mirrors: rtc_clk_cpu_freq_to_xtal(40, 1) */
    R_FIELD(SYSTEM_SYSCLK_CONF_REG,
            SYS_PRE_DIV_CNT_MASK | SYS_SOC_CLK_SEL_MASK,
            (0u << SYS_PRE_DIV_CNT_S) | (SYS_SOC_CLK_SEL_XTAL << SYS_SOC_CLK_SEL_S));
    ets_update_cpu_frequency(40);

    /* 3. Power down BBPLL (force-PD bits).
     *    mirrors: rtc_clk_bbpll_disable() */
    R_SET(RTC_CNTL_OPTIONS0_REG, BBPLL_PD_MASK);

    /* 4. Power up BBPLL.
     *    mirrors: rtc_clk_bbpll_enable() */
    R_CLR(RTC_CNTL_OPTIONS0_REG, BBPLL_PD_MASK);

    /* 5. Set BBPLL digital frequency select to 480 MHz.
     *    mirrors: clk_ll_bbpll_set_freq_mhz(480) */
    R_SET(SYSTEM_CPU_PER_CONF_REG, SYS_PLL_FREQ_SEL_BIT);   /* PLL_FREQ_SEL=1 */

    /* 6. Start BBPLL self-calibration.
     *    mirrors: clk_ll_bbpll_calibration_start() */
    R_CLR(I2C_MST_ANA_CONF0_REG, BBPLL_STOP_FORCE_HIGH);
    R_SET(I2C_MST_ANA_CONF0_REG, BBPLL_STOP_FORCE_LOW);

    /* 7. Write BBPLL analog config for 480 MHz from 40 MHz XTAL.
     *    mirrors: clk_ll_bbpll_set_config(480, 40)
     *
     *    Parameters (from clk_tree_ll.h, SOC_XTAL_FREQ_40M case):
     *      div_ref=0, div7_0=8, dr1=0, dr3=0, dchgp=5, dcur=3, dbias=3
     *    Derived:
     *      OC_REF_DIV byte = (dchgp=5 << 4) | div_ref=0 = 0x50
     *      OC_DIV_7_0 byte = 8 = 0x08
     *      OC_DCUR    byte = (dlref_sel=1<<6)|(dhref_sel=3<<4)|dcur=3 = 0x73
     */
    esp_rom_regi2c_write(I2C_BBPLL, I2C_BBPLL_HOSTID, I2C_BBPLL_MODE_HF,    0x6B);
    esp_rom_regi2c_write(I2C_BBPLL, I2C_BBPLL_HOSTID, I2C_BBPLL_OC_REF_DIV, 0x50);
    esp_rom_regi2c_write(I2C_BBPLL, I2C_BBPLL_HOSTID, I2C_BBPLL_OC_DIV_7_0, 0x08);
    esp_rom_regi2c_write_mask(I2C_BBPLL, I2C_BBPLL_HOSTID, I2C_BBPLL_OC_DR1_REG, 2, 0, 0);
    esp_rom_regi2c_write_mask(I2C_BBPLL, I2C_BBPLL_HOSTID, I2C_BBPLL_OC_DR3_REG, 6, 4, 0);
    esp_rom_regi2c_write(I2C_BBPLL, I2C_BBPLL_HOSTID, I2C_BBPLL_OC_DCUR,    0x73);
    esp_rom_regi2c_write_mask(I2C_BBPLL, I2C_BBPLL_HOSTID, I2C_BBPLL_OC_VCO_DBIAS_REG, 1, 0, 3);

    /* 8. Wait for calibration done, then delay 10 µs.
     *    mirrors: while(!clk_ll_bbpll_calibration_is_done()); esp_rom_delay_us(10); */
    while (!(RD(I2C_MST_ANA_CONF0_REG) & BBPLL_CAL_DONE))
        ;
    ets_delay_us(10);

    /* 9. Stop BBPLL calibration.
     *    mirrors: clk_ll_bbpll_calibration_stop() */
    R_CLR(I2C_MST_ANA_CONF0_REG, BBPLL_STOP_FORCE_LOW);
    R_SET(I2C_MST_ANA_CONF0_REG, BBPLL_STOP_FORCE_HIGH);

    /* 10. Switch CPU to PLL at 240 MHz.
     *     mirrors: rtc_clk_cpu_freq_to_pll_mhz(240)
     *
     *     Going from 40 MHz → 240 MHz (upward):
     *       a. Raise LDO bias first (CPU needs more voltage at 240 MHz).
     *          IDF reads per-chip calibrated values from efuse; we use the
     *          safe chip-level default (28), which is what IDF ships before
     *          efuse calibration runs. Delay 40 µs for the regulator to settle.
     *       b. Open all 6 LDO slaves: pd_slave = 240/80 = 3,
     *          SLAVE_PD = DEFAULT_LDO_SLAVE >> 3 = 0x7 >> 3 = 0.
     *       c. Set CPUPERIOD_SEL = 2 (480 MHz PLL / 2 = 240 MHz). */
    esp_rom_regi2c_write_mask(I2C_DIG_REG, I2C_DIG_REG_HOSTID,
                              I2C_DIG_REG_EXT_RTC_DREG, 4, 0, DIG_DBIAS_240M);
    esp_rom_regi2c_write_mask(I2C_DIG_REG, I2C_DIG_REG_HOSTID,
                              I2C_DIG_REG_EXT_DIG_DREG, 4, 0, DIG_DBIAS_240M);
    ets_delay_us(40);

    R_FIELD(RTC_CNTL_DATE_REG, RTC_CNTL_SLAVE_PD_MASK, 0u << RTC_CNTL_SLAVE_PD_S);

    R_FIELD(SYSTEM_CPU_PER_CONF_REG, SYS_CPUPERIOD_SEL_MASK,
            SYS_CPUPERIOD_SEL_240M << SYS_CPUPERIOD_SEL_S);

    R_FIELD(SYSTEM_SYSCLK_CONF_REG,
            SYS_PRE_DIV_CNT_MASK | SYS_SOC_CLK_SEL_MASK,
            (0u << SYS_PRE_DIV_CNT_S) | (SYS_SOC_CLK_SEL_PLL << SYS_SOC_CLK_SEL_S));
    ets_update_cpu_frequency(240);
}
