// SPDX-License-Identifier: GPL-2.0+
/*
 * ESP32-S31 SPL clock-tree bring-up: MPLL (EMAC ref path), CPU→CPLL
 * (320 MHz), and the systimer peripheral clock. First-stage bring-up, so it
 * lives in the SPL (which runs from uncached HP-RAM), not in the OpenSBI
 * runtime. Ported from the OpenSBI port's esp32s31_generic.c; register/ROM
 * addresses are chip constants.
 */

#include <stdio.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/types.h>

#include "spl_clk.h"

#define ESP32S31_PMU_BASE                 0x20704000U
#define ESP32S31_LP_AONCLKRST_BASE        0x20701000U
#define ESP32S31_HP_ALIVE_SYS_BASE        0x20589000U
#define ESP32S31_HP_SYS_CLKRST_BASE       0x20587000U
#define ESP32S31_MODEM_LPCON_BASE         0x2010F000U
#define ESP32S31_MODEM_SYSCON_BASE        0x20109C00U

/* Read-modify-write one register field (clear `mask`, OR in `val`). */
static void reg_set_field(void *reg, u32 mask, u32 val)
{
	writel((readl(reg) & ~mask) | val, reg);
}

/*
 * Bring up the MPLL (500 MHz) feeding MSPI/PSRAM and the EMAC reference
 * clock. psram_init() already brings MPLL up for PSRAM; this additionally
 * does the MODEM-I2C-master calibration path that the EMAC ref clock
 * depends on (power-up, configure, calibrate — same minimum sequence).
 */
#define PMU_IMM_HP_CK_POWER_1_OFF         0xF4U
#define PMU_HP_ACTIVE_HP_CK_POWER_OFF     0x1CU
#define PMU_PSRAM_CFG_OFF                 0x1E8U
#define HP_ALIVE_SYS_HP_CLK_CTRL_OFF      0x00U
#define LP_AONCLKRST_MSPI_DIV_OFF         0x54U
#define HP_SYS_CLKRST_ANA_PLL_CTRL0_OFF   0x174U
#define HP_SYS_CLKRST_MODEM_CTRL0_OFF     0x40U
#define MODEM_LPCON_CLK_CONF_OFF          0x18U
#define MODEM_LPCON_CLK_CONF_FORCE_ON_OFF 0x1CU
#define MODEM_SYSCON_CLK_CONF_OFF         0x04U

#define PMU_TIE_HIGH_GLOBAL_MPLL_ICG      BIT(22)
#define PMU_TIE_HIGH_XPD_MPLL_I2C         BIT(26)
#define PMU_TIE_HIGH_XPD_MPLL             BIT(30)
#define PMU_HP_ACTIVE_XPD_MPLL_I2C        BIT(26)
#define PMU_HP_ACTIVE_XPD_MPLL            BIT(30)
#define PMU_PSRAM_XPD                     BIT(31)
#define HP_ALIVE_SYS_HP_MPLL_500M_CLK_EN  BIT(31)
#define MSPI_FB_DIV_MASK                  (0x1FU << 3)
#define MSPI_FB_DIV_500MHZ                (24U << 3)
#define MSPI_CAL_END                      BIT(8)
#define MSPI_CAL_STOP                     BIT(9)
#define HP_SYS_CLKRST_MODEM_CLK_EN        BIT(0)
#define MODEM_LPCON_CLK_I2C_MST_EN        BIT(2)
#define MODEM_LPCON_CLK_I2C_MST_FO        BIT(2)
#define MODEM_SYSCON_CLK_I2C_MST_SEL_160M BIT(12)

void s31_spl_mpll_enable(void)
{
	void *pmu_imm    = (void *)(ESP32S31_PMU_BASE + PMU_IMM_HP_CK_POWER_1_OFF);
	void *pmu_active = (void *)(ESP32S31_PMU_BASE + PMU_HP_ACTIVE_HP_CK_POWER_OFF);
	void *pmu_psram  = (void *)(ESP32S31_PMU_BASE + PMU_PSRAM_CFG_OFF);
	void *hp_alive   = (void *)(ESP32S31_HP_ALIVE_SYS_BASE + HP_ALIVE_SYS_HP_CLK_CTRL_OFF);
	void *mspi_div   = (void *)(ESP32S31_LP_AONCLKRST_BASE + LP_AONCLKRST_MSPI_DIV_OFF);
	void *ana_pll    = (void *)(ESP32S31_HP_SYS_CLKRST_BASE + HP_SYS_CLKRST_ANA_PLL_CTRL0_OFF);
	void *modem_bus  = (void *)(ESP32S31_HP_SYS_CLKRST_BASE + HP_SYS_CLKRST_MODEM_CTRL0_OFF);
	void *lpcon_clk  = (void *)(ESP32S31_MODEM_LPCON_BASE + MODEM_LPCON_CLK_CONF_OFF);
	void *lpcon_fo   = (void *)(ESP32S31_MODEM_LPCON_BASE + MODEM_LPCON_CLK_CONF_FORCE_ON_OFF);
	void *syscon_clk = (void *)(ESP32S31_MODEM_SYSCON_BASE + MODEM_SYSCON_CLK_CONF_OFF);
	u32 v;

	/* MPLL calibration uses the analog I2C master (MODEM domain). Bring up
	 * its bus + clocks first, mirroring IDF bootloader_hardware_init().
	 */
	v = readl(modem_bus);
	writel(v | HP_SYS_CLKRST_MODEM_CLK_EN, modem_bus);
	v = readl(lpcon_clk);
	writel(v | MODEM_LPCON_CLK_I2C_MST_EN, lpcon_clk);
	v = readl(lpcon_fo);
	writel(v | MODEM_LPCON_CLK_I2C_MST_FO, lpcon_fo);
	v = readl(syscon_clk);
	writel(v | MODEM_SYSCON_CLK_I2C_MST_SEL_160M, syscon_clk);

	/* Power on the MPLL analog core + I2C interface, ungate its clock. */
	v = readl(pmu_imm);
	writel(v | PMU_TIE_HIGH_GLOBAL_MPLL_ICG |
		   PMU_TIE_HIGH_XPD_MPLL_I2C |
		   PMU_TIE_HIGH_XPD_MPLL, pmu_imm);

	v = readl(pmu_active);
	writel(v | PMU_HP_ACTIVE_XPD_MPLL |
		   PMU_HP_ACTIVE_XPD_MPLL_I2C, pmu_active);

	v = readl(pmu_psram);
	writel(v | PMU_PSRAM_XPD, pmu_psram);

	v = readl(hp_alive);
	writel(v | HP_ALIVE_SYS_HP_MPLL_500M_CLK_EN, hp_alive);

	/* Start calibration, set fb_div (500 MHz), wait, stop. */
	v = readl(ana_pll);
	writel(v & ~MSPI_CAL_STOP, ana_pll);
	v = readl(mspi_div);
	writel((v & ~MSPI_FB_DIV_MASK) | MSPI_FB_DIV_500MHZ, mspi_div);
	for (u32 i = 0; i < 1000000U; i++) {
		if (readl(ana_pll) & MSPI_CAL_END)
			break;
	}
	v = readl(ana_pll);
	writel(v | MSPI_CAL_STOP, ana_pll);
}

/*
 * Switch CPU from XTAL (40 MHz) to CPLL (320 MHz): CPU=320, MEM=160, SYS~107,
 * APB~53 MHz. The CLINT MTIME is CPU-clocked, so it moves to 320 MHz too —
 * the dts timebase-frequency (320M) and esp_systimer .timer_freq follow.
 */
#define HP_CLKRST_SOC_CLK_SEL             0x00U
#define HP_CLKRST_CPU_FREQ_CTRL0          0x04U
#define HP_CLKRST_MEM_FREQ_CTRL0          0x08U
#define HP_CLKRST_SYS_FREQ_CTRL0          0x0CU
#define HP_CLKRST_APB_FREQ_CTRL0          0x10U
#define HP_CLKRST_ROOT_CLK_CTRL0          0x14U
#define LP_AONCLKRST_CPLL_DIV             0x4CU	/* b[3:0] ref_div, b[11:4] fb_div */
#define ANA_PLL_CPU_CAL_END               BIT(2)	/* RO */
#define ANA_PLL_CPU_CAL_STOP              BIT(3)

void s31_spl_cpu_switch_to_cpll(void)
{
	void *hc = (void *)ESP32S31_HP_SYS_CLKRST_BASE;
	void *cpll_div = (void *)(ESP32S31_LP_AONCLKRST_BASE + LP_AONCLKRST_CPLL_DIV);
	void *cal = (void *)(ESP32S31_HP_SYS_CLKRST_BASE + HP_SYS_CLKRST_ANA_PLL_CTRL0_OFF);
	int guard;

	/* Program the CPLL to 320 MHz (40 MHz XTAL * fb_div/ref_div = 40*8/1).
	 * The ROM leaves it at fb_div=15/ref_div=2 = 300 MHz, so without this the
	 * CPU runs at 300, not 320. CPU is still on XTAL here, so the idle CPLL
	 * can be re-tuned + recalibrated safely.
	 */
	reg_set_field(cpll_div, 0xFU, 1U);		/* ref_div = 1 */
	reg_set_field(cpll_div, 0xFF0U, 8U << 4);	/* fb_div = 8 */
	writel(readl(cal) & ~ANA_PLL_CPU_CAL_STOP, cal);	/* start calibration */
	for (guard = 1000000; !(readl(cal) & ANA_PLL_CPU_CAL_END) && guard; guard--)
		;
	if (!guard)
		printf("SPL: WARN: CPLL calibration timed out; CPU clock may be unstable\n");
	writel(readl(cal) | ANA_PLL_CPU_CAL_STOP, cal);		/* stop calibration */

	/* Set dividers BEFORE switching source (IDF sequence). */
	reg_set_field(hc + HP_CLKRST_CPU_FREQ_CTRL0, 0xFFU, 0);   /* /1 */
	reg_set_field(hc + HP_CLKRST_MEM_FREQ_CTRL0, 0x1U,  1);   /* /2 */
	reg_set_field(hc + HP_CLKRST_SYS_FREQ_CTRL0, 0xFFU, 2);   /* /3 */
	reg_set_field(hc + HP_CLKRST_APB_FREQ_CTRL0, 0xFFU, 1);   /* /2 */

	/* Switch XTAL -> CPLL (SOC_CLK_SEL = 1 = CPLL). */
	reg_set_field(hc + HP_CLKRST_SOC_CLK_SEL, 0x3U, 1U);

	/* Commit (SOC_CLK_UPDATE) and wait for self-clear (bounded). */
	writel(readl(hc + HP_CLKRST_ROOT_CLK_CTRL0) | BIT(0),
	       hc + HP_CLKRST_ROOT_CLK_CTRL0);
	for (guard = 1000000;
	     (readl(hc + HP_CLKRST_ROOT_CLK_CTRL0) & BIT(0)) && guard; guard--)
		;
}

/*
 * Bring up the systimer peripheral so Linux can use it as its always-on
 * clocksource/clockevent. XTAL-fed (40 MHz / 2.5 = 16 MHz), independent of
 * CPU_CLK, free-running through WFI (unlike the CPU-clocked CLINT MTIME).
 * HP_SYS_CLKRST systimer_ctrl0 @ +0x120; SYSTIMER CONF @ base.
 */
#define ESP32S31_SYSTIMER_BASE            0x20399000U
#define HP_CLKRST_SYSTIMER_CTRL0          0x120U

void s31_spl_systimer_clk_init(void)
{
	void *clk = (void *)(ESP32S31_HP_SYS_CLKRST_BASE + HP_CLKRST_SYSTIMER_CTRL0);
	void *conf = (void *)ESP32S31_SYSTIMER_BASE;

	reg_set_field(clk, BIT(3), 0);			/* clk_src_sel = XTAL */
	writel(readl(clk) | BIT(0) | BIT(4), clk);	/* apb_clk_en + clk_en */
	writel(readl(clk) | BIT(1), clk);		/* pulse reset */
	writel(readl(clk) & ~BIT(1), clk);
	/* UNIT0 free-running: work_en set, not stalled by either core. */
	writel((readl(conf) | BIT(30)) & ~(BIT(27) | BIT(28)), conf);
}
