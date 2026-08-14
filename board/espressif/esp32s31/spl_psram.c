// SPDX-License-Identifier: GPL-2.0+
/*
 * ESP32-S31 PSRAM bring-up.
 *
 * Bring-up sequence: LDO → MPLL → MSPI controller → mode regs →
 * ID/density readout → timing tuning → PSRAM MMU. Verified on real
 * silicon.
 *
 * Copyright (C) 2026, Espressif Systems (Shanghai) CO LTD
 */

#include <cpu_func.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/types.h>

#include "spl_psram.h"

/*------------------------------------------------------------------------
 * LDO — PSRAM voltage domain
 *------------------------------------------------------------------------
 */
#define DR_REG_PMU_BASE                   0x20704000U
#define PMU_PSRAM_CFG_REG                 (DR_REG_PMU_BASE + 0x1E8)
#define PMU_PSRAM_XPD                     BIT(31)
#define PMU_EXT_LDO_CTRL_REG              (DR_REG_PMU_BASE + 0x218)
#define PMU_EXT_LDO_TIE_HIGH              BIT(0)
#define PMU_EXT_LDO_MUL_S                 3
#define PMU_EXT_LDO_MUL_M                 (0x7U << PMU_EXT_LDO_MUL_S)
#define PMU_EXT_LDO_DREF_S                6
#define PMU_EXT_LDO_DREF_M                (0xFU << PMU_EXT_LDO_DREF_S)

/*
 * PSRAM rail target is 1.8 V. The LDO output is
 * Vout = Vref(dref) * (4 + mul) / 4 with Vref(6) = 0.8 V, so dref=6,
 * mul=5 gives 0.8 * 9 / 4 = 1.800 V — an exact grid hit. Recompute
 * dref/mul from that formula if the target voltage ever changes.
 * TIE_HIGH (bypass regulation to the 3.3 V rail) stays cleared.
 */
#define PSRAM_LDO_DREF                    6
#define PSRAM_LDO_MUL                     5

static void psram_ldo_init(void)
{
	u32 v;

	v = readl((void *)PMU_EXT_LDO_CTRL_REG);
	v &= ~(PMU_EXT_LDO_TIE_HIGH | PMU_EXT_LDO_MUL_M | PMU_EXT_LDO_DREF_M);
	v |= PSRAM_LDO_MUL << PMU_EXT_LDO_MUL_S;
	v |= PSRAM_LDO_DREF << PMU_EXT_LDO_DREF_S;
	writel(v, (void *)PMU_EXT_LDO_CTRL_REG);

	setbits_le32((void *)PMU_PSRAM_CFG_REG, PMU_PSRAM_XPD);
}

/*------------------------------------------------------------------------
 * MPLL — multiplier PLL feeding MSPI / PSRAM controller
 *
 * S31's MPLL is fully digital — no regi2c. fb_div lives in a single
 * LP_AONCLKRST register; calibration handshake via HP_SYS_CLKRST.
 *------------------------------------------------------------------------
 */
#define PMU_IMM_HP_CK_POWER_1_REG         (DR_REG_PMU_BASE + 0xF4)
#define PMU_TIE_HIGH_GLOBAL_MPLL_ICG      BIT(22)
#define PMU_TIE_HIGH_XPD_MPLL_I2C         BIT(26)
#define PMU_TIE_HIGH_XPD_MPLL             BIT(30)
/* Our flow (ROM -> OpenSBI -> U-Boot) has no earlier stage that sets
 * the CPLL bits in PMU_IMM_HP_CK_POWER_1, so they start cleared here.
 * Without them the PMU FSM leaves MPLL in a degraded state where MDIO
 * can't drive MDC — set them alongside MPLL's own bits below.
 */
#define PMU_TIE_HIGH_GLOBAL_CPLL_ICG      BIT(19)
#define PMU_TIE_HIGH_XPD_CPLL_I2C         BIT(23)
#define PMU_TIE_HIGH_XPD_CPLL             BIT(27)
#define PMU_TIE_HIGH_CPLL_BITS            \
	(PMU_TIE_HIGH_GLOBAL_CPLL_ICG | \
	 PMU_TIE_HIGH_XPD_CPLL_I2C | \
	 PMU_TIE_HIGH_XPD_CPLL)
#define PMU_HP_ACTIVE_HP_CK_POWER_REG     (DR_REG_PMU_BASE + 0x1C)
#define PMU_HP_ACTIVE_XPD_MPLL_I2C        BIT(26)
#define PMU_HP_ACTIVE_XPD_MPLL            BIT(30)
/* HP_ACTIVE_HP_CK_POWER needs bit 22 plus I2C/XPD for CPLL (23/27),
 * BBPLL (24/28), and MPLL (26/30) all set together (0x5DC00000) —
 * setting MPLL's bits alone leaves the PMU FSM in a degraded state on
 * real silicon where MPLL doesn't actually drive the EMAC REF clock
 * cleanly.
 */
#define PMU_HP_ACTIVE_ALL_PLL_XPD         0x5DC00000U

#define DR_REG_HP_ALIVE_SYS_BASE          0x20589000U
#define HP_ALIVE_SYS_HP_CLK_CTRL_REG      (DR_REG_HP_ALIVE_SYS_BASE + 0x0)
#define HP_ALIVE_SYS_HP_MPLL_500M_CLK_EN  BIT(31)

#define DR_REG_HP_SYS_CLKRST_BASE         0x20587000U
#define HP_SYS_CLKRST_ANA_PLL_CTRL0_REG   (DR_REG_HP_SYS_CLKRST_BASE + 0x174)
#define HP_SYS_CLKRST_REG_MSPI_CAL_END    BIT(8)
#define HP_SYS_CLKRST_REG_MSPI_CAL_STOP   BIT(9)

#define DR_REG_LP_AONCLKRST_BASE          0x20701000U
#define LP_AONCLKRST_MSPI_DIV_REG         (DR_REG_LP_AONCLKRST_BASE + 0x54)
#define LP_AONCLKRST_MSPI_FB_DIV_S        3
#define LP_AONCLKRST_MSPI_FB_DIV_M        (0x1FU << LP_AONCLKRST_MSPI_FB_DIV_S)

#define PSRAM_MPLL_FREQ_MHZ               500
#define XTAL_FREQ_MHZ                     40

static void psram_mpll_init(void)
{
	u8 ref_div = 1;
	u8 fb_div;

	/* Power-up sequence. Also brings up CPLL, since nothing earlier
	 * in this boot flow sets those bits; without them the PMU FSM
	 * leaves MPLL inert.
	 */
	setbits_le32((void *)PMU_IMM_HP_CK_POWER_1_REG,
		     PMU_TIE_HIGH_CPLL_BITS |
		     PMU_TIE_HIGH_GLOBAL_MPLL_ICG);
	setbits_le32((void *)PMU_IMM_HP_CK_POWER_1_REG,
		     PMU_TIE_HIGH_XPD_MPLL | PMU_TIE_HIGH_XPD_MPLL_I2C);
	setbits_le32((void *)HP_ALIVE_SYS_HP_CLK_CTRL_REG,
		     HP_ALIVE_SYS_HP_MPLL_500M_CLK_EN);
	setbits_le32((void *)PMU_HP_ACTIVE_HP_CK_POWER_REG,
		     PMU_HP_ACTIVE_ALL_PLL_XPD);

	/* Configure + calibrate. fb_div = target_MHz * (ref_div+1) /
	 * xtal_MHz - 1, per the MPLL's frequency formula.
	 */
	fb_div = PSRAM_MPLL_FREQ_MHZ * (ref_div + 1) / XTAL_FREQ_MHZ - 1;

	clrbits_le32((void *)HP_SYS_CLKRST_ANA_PLL_CTRL0_REG,
		     HP_SYS_CLKRST_REG_MSPI_CAL_STOP);
	clrsetbits_le32((void *)LP_AONCLKRST_MSPI_DIV_REG,
			LP_AONCLKRST_MSPI_FB_DIV_M,
			(u32)fb_div << LP_AONCLKRST_MSPI_FB_DIV_S);
	while (!(readl((void *)HP_SYS_CLKRST_ANA_PLL_CTRL0_REG) &
		 HP_SYS_CLKRST_REG_MSPI_CAL_END))
		;
	setbits_le32((void *)HP_SYS_CLKRST_ANA_PLL_CTRL0_REG,
		     HP_SYS_CLKRST_REG_MSPI_CAL_STOP);
}

/*------------------------------------------------------------------------
 * MSPI controller clock
 *
 * Touches HP_SYS_CLKRST.psram_ctrl0 for module clock + reset + clock
 * source select. MSPI_ID_2 and _ID_3 alias the same register fields
 * here, so one write covers both.
 *------------------------------------------------------------------------
 */
#define HP_SYS_CLKRST_PSRAM_CTRL0_REG     (DR_REG_HP_SYS_CLKRST_BASE + 0x68)
#define PSRAM_SYS_CLK_EN                  BIT(0)
#define PSRAM_AXI_RST_EN                  BIT(1)
#define PSRAM_APB_RST_EN                  BIT(2)
#define PSRAM_CLK_SRC_SEL_S               5
#define PSRAM_CLK_SRC_SEL_M               (0x3U << PSRAM_CLK_SRC_SEL_S)
#define PSRAM_CLK_SRC_MPLL                BIT(PSRAM_CLK_SRC_SEL_S)
#define PSRAM_PLL_CLK_EN                  BIT(7)

static void psram_mspi_clock_init(void)
{
	setbits_le32((void *)HP_SYS_CLKRST_PSRAM_CTRL0_REG,
		     PSRAM_SYS_CLK_EN | PSRAM_PLL_CLK_EN);
	setbits_le32((void *)HP_SYS_CLKRST_PSRAM_CTRL0_REG,
		     PSRAM_AXI_RST_EN | PSRAM_APB_RST_EN);
	clrbits_le32((void *)HP_SYS_CLKRST_PSRAM_CTRL0_REG,
		     PSRAM_AXI_RST_EN | PSRAM_APB_RST_EN);
	clrsetbits_le32((void *)HP_SYS_CLKRST_PSRAM_CTRL0_REG,
			PSRAM_CLK_SRC_SEL_M, PSRAM_CLK_SRC_MPLL);
}

/*------------------------------------------------------------------------
 * MSPI pin drive + DQS enable
 *
 * All 11 PSRAM pins (D, Q, WP, HOLD, DQ4-7, DQS_0, CK, CS) share the
 * same DRV field at bits [14:12]; DQS_0 also has XPD at bit [0].
 *------------------------------------------------------------------------
 */
#define DR_REG_IOMUX_MSPI_PIN_BASE        0x20584000U
#define IOMUX_MSPI_PIN_DRV_S              12
#define IOMUX_MSPI_PIN_DRV_M              (0x7U << IOMUX_MSPI_PIN_DRV_S)
#define IOMUX_MSPI_PIN_PSRAM_DQS_0_XPD    BIT(0)
#define PSRAM_PIN_DRIVE_STRENGTH          2

static const u32 psram_pin_drv_regs[] = {
	DR_REG_IOMUX_MSPI_PIN_BASE + 0x1C,  /* PSRAM_D */
	DR_REG_IOMUX_MSPI_PIN_BASE + 0x20,  /* PSRAM_Q */
	DR_REG_IOMUX_MSPI_PIN_BASE + 0x24,  /* PSRAM_WP */
	DR_REG_IOMUX_MSPI_PIN_BASE + 0x28,  /* PSRAM_HOLD */
	DR_REG_IOMUX_MSPI_PIN_BASE + 0x2C,  /* PSRAM_DQ4 */
	DR_REG_IOMUX_MSPI_PIN_BASE + 0x30,  /* PSRAM_DQ5 */
	DR_REG_IOMUX_MSPI_PIN_BASE + 0x34,  /* PSRAM_DQ6 */
	DR_REG_IOMUX_MSPI_PIN_BASE + 0x38,  /* PSRAM_DQ7 */
	DR_REG_IOMUX_MSPI_PIN_BASE + 0x3C,  /* PSRAM_DQS_0 */
	DR_REG_IOMUX_MSPI_PIN_BASE + 0x40,  /* PSRAM_CK */
	DR_REG_IOMUX_MSPI_PIN_BASE + 0x44,  /* PSRAM_CS */
};

static void psram_pin_init(void)
{
	u32 drv = (u32)PSRAM_PIN_DRIVE_STRENGTH << IOMUX_MSPI_PIN_DRV_S;
	int i;

	for (i = 0; i < (int)ARRAY_SIZE(psram_pin_drv_regs); i++)
		clrsetbits_le32((void *)psram_pin_drv_regs[i],
				IOMUX_MSPI_PIN_DRV_M, drv);

	setbits_le32((void *)(DR_REG_IOMUX_MSPI_PIN_BASE + 0x3C),
		     IOMUX_MSPI_PIN_PSRAM_DQS_0_XPD);
}

/*------------------------------------------------------------------------
 * PSRAM controller — CS timing, page size, bus clock, DLL
 *------------------------------------------------------------------------
 */
#define DR_REG_PSRAM_MSPI0_BASE           0x20502000U

#define SPI_MEM_S_SRAM_CLK_REG            (DR_REG_PSRAM_MSPI0_BASE + 0x50)
#define SPI_MEM_S_SCLKCNT_L_S             0
#define SPI_MEM_S_SCLKCNT_H_S             8
#define SPI_MEM_S_SCLKCNT_N_S             16

#define SPI_SMEM_S_ECC_CTRL_REG           (DR_REG_PSRAM_MSPI0_BASE + 0x174)
#define SPI_SMEM_S_PAGE_SIZE_S            18
#define SPI_SMEM_S_PAGE_SIZE_M            (0x3U << SPI_SMEM_S_PAGE_SIZE_S)

#define SPI_SMEM_S_TIMING_CALI_REG        (DR_REG_PSRAM_MSPI0_BASE + 0x190)
#define SPI_SMEM_S_DLL_TIMING_CALI        BIT(5)

/* MSPI_ID_3 (used by ROM SPI helpers for mode-reg transactions) has its
 * own clock register on the MSPI1 block and its own DLL bit on the
 * MEM-path TIMING_CALI register. Both ID_2 and ID_3 must be configured
 * in lockstep or the ROM's cmd_start hangs forever.
 */
#define DR_REG_PSRAM_MSPI1_BASE           0x20503000U
#define SPI1_MEM_S_CLOCK_REG              (DR_REG_PSRAM_MSPI1_BASE + 0x14)
#define SPI_MEM_S_TIMING_CALI_REG         (DR_REG_PSRAM_MSPI0_BASE + 0x180)
#define SPI_MEM_S_DLL_TIMING_CALI         BIT(5)

#define SPI_SMEM_S_AC_REG                 (DR_REG_PSRAM_MSPI0_BASE + 0x1A0)
#define SPI_SMEM_S_CS_SETUP               BIT(0)
#define SPI_SMEM_S_CS_HOLD                BIT(1)
#define SPI_SMEM_S_CS_SETUP_TIME_S        2
#define SPI_SMEM_S_CS_SETUP_TIME_M        (0x1FU << SPI_SMEM_S_CS_SETUP_TIME_S)
#define SPI_SMEM_S_CS_HOLD_TIME_S         7
#define SPI_SMEM_S_CS_HOLD_TIME_M         (0x1FU << SPI_SMEM_S_CS_HOLD_TIME_S)
#define SPI_SMEM_S_CS_HOLD_DELAY_S        25
#define SPI_SMEM_S_CS_HOLD_DELAY_M        (0x3FU << SPI_SMEM_S_CS_HOLD_DELAY_S)
#define SPI_SMEM_S_SPLIT_TRANS_EN         BIT(31)

/* AP octal PSRAM defaults: setup=4, hold=4, hold_delay=3 cycles. */
#define AP_OCT_PSRAM_CS_SETUP_TIME        4
#define AP_OCT_PSRAM_CS_HOLD_TIME         4
#define AP_OCT_PSRAM_CS_HOLD_DELAY        3

/* PSRAM speed at init: MPLL (500 MHz) / divisor. Set to 125 MHz so
 * MPLL can run at 500 MHz — required for the EMAC's 125 MHz RGMII
 * reference (CNNT_HP_EMAC_REF_CTRL.DIV = 3 -> MPLL/4 = 125 MHz).
 */
#define PSRAM_SPEED_MHZ                   125
#define PSRAM_BUS_FREQ_DIV                (PSRAM_MPLL_FREQ_MHZ / PSRAM_SPEED_MHZ)

static void psram_ctrlr_init(void)
{
	u32 ac;
	u32 freqdiv = PSRAM_BUS_FREQ_DIV;
	u32 freqbits;

	/* CS timing — written together with split-trans enable. */
	ac = SPI_SMEM_S_CS_SETUP | SPI_SMEM_S_CS_HOLD |
	     ((u32)(AP_OCT_PSRAM_CS_SETUP_TIME - 1) << SPI_SMEM_S_CS_SETUP_TIME_S) |
	     ((u32)(AP_OCT_PSRAM_CS_HOLD_TIME - 1) << SPI_SMEM_S_CS_HOLD_TIME_S) |
	     ((u32)(AP_OCT_PSRAM_CS_HOLD_DELAY - 1) << SPI_SMEM_S_CS_HOLD_DELAY_S) |
	     SPI_SMEM_S_SPLIT_TRANS_EN;
	writel(ac, (void *)SPI_SMEM_S_AC_REG);

	/* Page size = 2048 → field value 3 (AP octal PSRAM). */
	clrsetbits_le32((void *)SPI_SMEM_S_ECC_CTRL_REG, SPI_SMEM_S_PAGE_SIZE_M,
			3U << SPI_SMEM_S_PAGE_SIZE_S);

	/* Bus clock (clk_ll_set_bus_clock equivalent). freqdiv encoding:
	 *   N = divisor - 1, L = N, H = floor(N/2). Both MSPI_ID_2
	 *   (SMEM/AXI-cached path) and MSPI_ID_3 (MEM/command-mode path
	 *   that the ROM SPI helpers drive) need their own clock.
	 */
	freqbits = ((freqdiv - 1) << SPI_MEM_S_SCLKCNT_N_S) |
		   ((freqdiv / 2 - 1) << SPI_MEM_S_SCLKCNT_H_S) |
		   ((freqdiv - 1) << SPI_MEM_S_SCLKCNT_L_S);
	writel(freqbits, (void *)SPI_MEM_S_SRAM_CLK_REG);
	writel(freqbits, (void *)SPI1_MEM_S_CLOCK_REG);

	/* DLL enable for timing calibration — both SMEM (PSRAM AXI) and
	 * MEM (ROM-helper command) paths.
	 */
	setbits_le32((void *)SPI_SMEM_S_TIMING_CALI_REG,
		     SPI_SMEM_S_DLL_TIMING_CALI);
	setbits_le32((void *)SPI_MEM_S_TIMING_CALI_REG,
		     SPI_MEM_S_DLL_TIMING_CALI);
}

/*------------------------------------------------------------------------
 * Mode register programming + ID readback (octal DTR SPI)
 *
 * Three ROM-exported helpers exist for this (S31 rom.ld):
 *   esp_rom_spi_set_op_mode = 0x2F800124
 *   esp_rom_spi_cmd_config  = 0x2F80011C
 *   esp_rom_spi_cmd_start   = 0x2F800120
 *------------------------------------------------------------------------
 */
/* MSPI1 controller register addresses for command-mode PSRAM access.
 * We re-implement the three ROM SPI helpers as direct register pokes
 * because the ROM versions at 0x2F80011C/120/124 hang for reasons we
 * couldn't pin down.
 */
#define SPI1_MEM_S_CMD_REG                (DR_REG_PSRAM_MSPI1_BASE + 0x00)
#define SPI1_MEM_S_ADDR_REG               (DR_REG_PSRAM_MSPI1_BASE + 0x04)
#define SPI1_MEM_S_CTRL_REG               (DR_REG_PSRAM_MSPI1_BASE + 0x08)
#define SPI1_MEM_S_USER_REG               (DR_REG_PSRAM_MSPI1_BASE + 0x18)
#define SPI1_MEM_S_USER1_REG              (DR_REG_PSRAM_MSPI1_BASE + 0x1C)
#define SPI1_MEM_S_USER2_REG              (DR_REG_PSRAM_MSPI1_BASE + 0x20)
#define SPI1_MEM_S_MOSI_DLEN_REG          (DR_REG_PSRAM_MSPI1_BASE + 0x24)
#define SPI1_MEM_S_MISO_DLEN_REG          (DR_REG_PSRAM_MSPI1_BASE + 0x28)
#define SPI1_MEM_S_MISC_REG               (DR_REG_PSRAM_MSPI1_BASE + 0x34)
#define SPI1_MEM_S_CACHE_FCTRL_REG        (DR_REG_PSRAM_MSPI1_BASE + 0x3C)
#define SPI1_MEM_S_W0_REG                 (DR_REG_PSRAM_MSPI1_BASE + 0x58)
#define SPI1_MEM_S_DDR_REG                (DR_REG_PSRAM_MSPI1_BASE + 0xD4)

#define S1_USR                            BIT(18)
#define S1_USR_MOSI                       BIT(27)
#define S1_USR_MISO                       BIT(28)
#define S1_USR_DUMMY                      BIT(29)
#define S1_USR_ADDR                       BIT(30)
#define S1_USR_COMMAND                    BIT(31)
#define S1_USR_DUMMY_CYCLELEN_M           (0x3FU << 0)
#define S1_USR_ADDR_BITLEN_S              26
#define S1_USR_ADDR_BITLEN_M              (0x3FU << S1_USR_ADDR_BITLEN_S)
#define S1_USR_COMMAND_VALUE_M            (0xFFFFU << 0)
#define S1_USR_COMMAND_BITLEN_S           28
#define S1_USR_COMMAND_BITLEN_M           (0xFU << S1_USR_COMMAND_BITLEN_S)
#define S1_USR_MOSI_DBITLEN_M             (0x3FFU << 0)
#define S1_USR_MISO_DBITLEN_M             (0x3FFU << 0)
#define S1_CS0_DIS                        BIT(0)
#define S1_CS1_DIS                        BIT(1)
#define S1_CACHE_USR_ADDR_4BYTE           BIT(1)

/* CTRL_REG mode bits (OPI DTR uses oct + fastrd). */
#define S1_FDOUT_OCT                      BIT(4)
#define S1_FDIN_OCT                       BIT(5)
#define S1_FADDR_OCT                      BIT(6)
#define S1_FCMD_QUAD                      BIT(8)
#define S1_FCMD_OCT                       BIT(9)
#define S1_FASTRD_MODE                    BIT(13)
#define S1_FREAD_DUAL                     BIT(14)
#define S1_FREAD_QUAD                     BIT(20)
#define S1_FREAD_DIO                      BIT(23)
#define S1_FREAD_QIO                      BIT(24)
#define S1_CTRL_MODE_CLEAR_M              (S1_FCMD_OCT | S1_FCMD_QUAD | \
					  S1_FREAD_QIO | S1_FREAD_QUAD | \
					  S1_FREAD_DIO | S1_FREAD_DUAL | \
					  S1_FASTRD_MODE | S1_FADDR_OCT | \
					  S1_FDIN_OCT | S1_FDOUT_OCT)

#define S1_FWRITE_DUAL                    BIT(12)
#define S1_FWRITE_QUAD                    BIT(13)
#define S1_FWRITE_DIO                     BIT(14)
#define S1_FWRITE_QIO                     BIT(15)
#define S1_USER_MODE_CLEAR_M              (S1_FWRITE_DUAL | S1_FWRITE_DIO | \
					  S1_FWRITE_QUAD | S1_FWRITE_QIO)

/* DDR_REG bits. */
#define S1_FMEM_DDR_EN                    BIT(0)
#define S1_FMEM_DDR_CMD_DIS               BIT(4)

#define AP_OCT_PSRAM_REG_READ             0x4040
#define AP_OCT_PSRAM_REG_WRITE            0xC0C0
#define AP_OCT_PSRAM_RD_REG_DUMMY_BITLEN  (2 * (9 - 1))

#define PSRAM_MSPI_ID                     3        /* PSRAM controller uses MSPI3 path */
#define PSRAM_CS_MASK                     BIT(1)

#define AP_OCT_PSRAM_RD_LATENCY           6
#define AP_OCT_PSRAM_WR_LATENCY           3

union psram_mr0 {
	struct {
		u8 drive_str   : 2;
		u8 read_latency: 3;
		u8 lt          : 1;
		u8 rsvd6       : 1;
		u8 tso         : 1;
	};
	u8 val;
};

union psram_mr2 {
	struct {
		u8 density : 3;
		u8 dev_id  : 2;
		u8 kgd     : 3;
	};
	u8 val;
};

union psram_mr4 {
	struct {
		u8 pasr       : 3;
		u8 rf         : 2;
		u8 wr_latency : 3;
	};
	u8 val;
};

union psram_mr8 {
	struct {
		u8 bl    : 2;
		u8 bt    : 1;
		u8 rbx   : 1;
		u8 rsvd5 : 2;
		u8 x16   : 1;
		u8 rsvd7 : 1;
	};
	u8 val;
};

/* Direct-register replacement for esp_rom_spi_set_op_mode(OPI_DTR_MODE)
 * on the MSPI1 controller — clears any prior mode bits, sets octal CMD
 * + ADDR + IN + OUT + FASTRD in CTRL, enables DDR + drives CMD in DDR
 * mode in DDR_REG.
 */
static void spi1_set_op_mode_opi_dtr(void)
{
	clrbits_le32((void *)SPI1_MEM_S_CTRL_REG, S1_CTRL_MODE_CLEAR_M);
	clrbits_le32((void *)SPI1_MEM_S_USER_REG, S1_USER_MODE_CLEAR_M);
	setbits_le32((void *)SPI1_MEM_S_CTRL_REG,
		     S1_FCMD_OCT | S1_FADDR_OCT | S1_FDIN_OCT |
			     S1_FDOUT_OCT | S1_FASTRD_MODE);
	clrbits_le32((void *)SPI1_MEM_S_DDR_REG, S1_FMEM_DDR_CMD_DIS);
	setbits_le32((void *)SPI1_MEM_S_DDR_REG, S1_FMEM_DDR_EN);
}

static void psram_xact(u16 cmd, u32 addr, u32 *tx, u32 tx_bits,
		       u32 *rx, u32 rx_bits, u32 dummy)
{
	const u32 cmd_bit_len = 16;
	const u32 addr_bit_len = 32;
	u32 user;

	/* Wait for prior transaction (if any). OPI-DTR mode is set once
	 * by the caller (psram_mode_reg_init) before the batch of
	 * transactions; the mode bits don't change between calls.
	 */
	while (readl((void *)SPI1_MEM_S_CMD_REG) != 0)
		;

	/* USER2: command value + bitlen (- 1). */
	writel(((u32)cmd & 0xFFFF) |
	       ((cmd_bit_len - 1) << S1_USR_COMMAND_BITLEN_S),
	       (void *)SPI1_MEM_S_USER2_REG);

	/* USER1: addr bitlen (- 1) and dummy cyclelen (- 1) if any. */
	{
		u32 user1 = (addr_bit_len - 1) << S1_USR_ADDR_BITLEN_S;

		if (dummy)
			user1 |= (dummy - 1) & S1_USR_DUMMY_CYCLELEN_M;
		writel(user1, (void *)SPI1_MEM_S_USER1_REG);
	}

	/* Address. 32-bit means 4-byte mode in CACHE_FCTRL. */
	writel(addr, (void *)SPI1_MEM_S_ADDR_REG);
	setbits_le32((void *)SPI1_MEM_S_CACHE_FCTRL_REG,
		     S1_CACHE_USR_ADDR_4BYTE);

	/* USER: enable phases for this command. */
	user = S1_USR_COMMAND | S1_USR_ADDR;
	if (dummy)
		user |= S1_USR_DUMMY;
	if (tx_bits) {
		user |= S1_USR_MOSI;
		writel((tx_bits - 1) & S1_USR_MOSI_DBITLEN_M,
		       (void *)SPI1_MEM_S_MOSI_DLEN_REG);
		if (tx)
			writel(*tx, (void *)SPI1_MEM_S_W0_REG);
	} else {
		writel(0, (void *)SPI1_MEM_S_MOSI_DLEN_REG);
	}
	if (rx_bits) {
		user |= S1_USR_MISO;
		writel((rx_bits - 1) & S1_USR_MISO_DBITLEN_M,
		       (void *)SPI1_MEM_S_MISO_DLEN_REG);
	} else {
		writel(0, (void *)SPI1_MEM_S_MISO_DLEN_REG);
	}
	writel(user, (void *)SPI1_MEM_S_USER_REG);

	/* CS select: PSRAM is on CS1. Disable CS0 (default), enable CS1. */
	setbits_le32((void *)SPI1_MEM_S_MISC_REG, S1_CS0_DIS);
	clrbits_le32((void *)SPI1_MEM_S_MISC_REG, S1_CS1_DIS);

	/* Trigger. */
	setbits_le32((void *)SPI1_MEM_S_CMD_REG, S1_USR);
	while (readl((void *)SPI1_MEM_S_CMD_REG) & S1_USR)
		;

	/* Restore default CS state (CS0 active, CS1 inactive). */
	clrbits_le32((void *)SPI1_MEM_S_MISC_REG, S1_CS0_DIS);
	setbits_le32((void *)SPI1_MEM_S_MISC_REG, S1_CS1_DIS);

	if (rx_bits && rx)
		*rx = readl((void *)SPI1_MEM_S_W0_REG);
}

static u32 s_psram_size;

static void psram_mode_reg_init(void)
{
	u32 rx_buf = 0;
	u32 tx_buf;
	union psram_mr0 mr0;
	union psram_mr2 mr2;
	union psram_mr4 mr4;
	union psram_mr8 mr8;

	spi1_set_op_mode_opi_dtr();

	/* Read MR0 from addr 0 (the read returns MR0 + MR1; we only use
	 * MR0 here).
	 */
	psram_xact(AP_OCT_PSRAM_REG_READ, 0, NULL, 0, &rx_buf, 16,
		   AP_OCT_PSRAM_RD_REG_DUMMY_BITLEN);
	mr0.val = rx_buf & 0xFF;

	/* Read MR4 (addr 4 brings back MR4 + MR8). */
	psram_xact(AP_OCT_PSRAM_REG_READ, 4, NULL, 0, &rx_buf, 16,
		   AP_OCT_PSRAM_RD_REG_DUMMY_BITLEN);
	mr4.val = rx_buf & 0xFF;

	/* Update MR0 latency / drive, then write back. */
	mr0.lt = 1;
	mr0.read_latency = AP_OCT_PSRAM_RD_LATENCY;
	mr0.drive_str = 0;
	tx_buf = mr0.val;
	psram_xact(AP_OCT_PSRAM_REG_WRITE, 0, &tx_buf, 16, NULL, 0, 0);

	/* Update MR4 wr_latency, then write back. */
	mr4.wr_latency = AP_OCT_PSRAM_WR_LATENCY;
	tx_buf = mr4.val;
	psram_xact(AP_OCT_PSRAM_REG_WRITE, 4, &tx_buf, 16, NULL, 0, 0);

	/* Read MR8 (8 bits at addr 8). */
	psram_xact(AP_OCT_PSRAM_REG_READ, 8, NULL, 0, &rx_buf, 8,
		   AP_OCT_PSRAM_RD_REG_DUMMY_BITLEN);
	mr8.val = rx_buf & 0xFF;

	/* Update MR8 burst length, then write back. */
	mr8.bl = 3;    /* 2048-byte burst */
	mr8.bt = 0;    /* wrap */
	mr8.rbx = 1;
	mr8.x16 = 0;
	tx_buf = mr8.val;
	psram_xact(AP_OCT_PSRAM_REG_WRITE, 8, &tx_buf, 16, NULL, 0, 0);

	/* Read MR2 for density (addr 2 brings MR2 + MR3). */
	psram_xact(AP_OCT_PSRAM_REG_READ, 2, NULL, 0, &rx_buf, 16,
		   AP_OCT_PSRAM_RD_REG_DUMMY_BITLEN);
	mr2.val = rx_buf & 0xFF;

	switch (mr2.density) {
	case 0x1:
		s_psram_size = 4 * 1024 * 1024;
		break;
	case 0x3:
		s_psram_size = 8 * 1024 * 1024;
		break;
	case 0x5:
		s_psram_size = 16 * 1024 * 1024;
		break;
	case 0x7:
		s_psram_size = 32 * 1024 * 1024;
		break;
	case 0x6:
		s_psram_size = 64 * 1024 * 1024;
		break;
	default:
		s_psram_size = 0;
		break;
	}
}

/*------------------------------------------------------------------------
 * SMEM controller config for AXI/cached PSRAM access
 *
 * All bits land in SPIMEM2 at DR_REG_PSRAM_MSPI0_BASE = 0x20502000.
 *------------------------------------------------------------------------
 */
#define SPI_MEM_S_CTRL1_REG               (DR_REG_PSRAM_MSPI0_BASE + 0x0C)
#define SPI_MEM_S_AR_SPLICE_EN            BIT(25)
#define SPI_MEM_S_AW_SPLICE_EN            BIT(26)

#define SPI_MEM_S_CACHE_FCTRL_REG_SMEM    (DR_REG_PSRAM_MSPI0_BASE + 0x3C)
#define SPI_MEM_S_AXI_REQ_EN              BIT(0)
#define SPI_CLOSE_AXI_INF_EN              BIT(31)

#define SPI_MEM_S_CACHE_SCTRL_REG         (DR_REG_PSRAM_MSPI0_BASE + 0x40)
#define SPI_MEM_S_CACHE_USR_SADDR_4BYTE   BIT(0)
#define SPI_MEM_S_USR_WR_SRAM_DUMMY       BIT(3)
#define SPI_MEM_S_USR_RD_SRAM_DUMMY       BIT(4)
#define SPI_MEM_S_CACHE_SRAM_USR_RCMD     BIT(5)
#define SPI_MEM_S_SRAM_RDUMMY_CYCLELEN_S  6
#define SPI_MEM_S_SRAM_ADDR_BITLEN_S      14
#define SPI_MEM_S_CACHE_SRAM_USR_WCMD     BIT(20)
#define SPI_MEM_S_SRAM_OCT                BIT(21)
#define SPI_MEM_S_SRAM_WDUMMY_CYCLELEN_S  22

#define SPI_MEM_S_SRAM_CMD_REG            (DR_REG_PSRAM_MSPI0_BASE + 0x44)
#define SPI_MEM_S_SDIN_OCT                BIT(18)
#define SPI_MEM_S_SDOUT_OCT               BIT(19)
#define SPI_MEM_S_SADDR_OCT               BIT(20)
#define SPI_MEM_S_SCMD_OCT                BIT(21)
#define SPI_MEM_S_SDUMMY_WOUT             BIT(23)
#define SPI_MEM_S_SDIN_HEX                BIT(26)
#define SPI_MEM_S_SDOUT_HEX               BIT(27)

#define SPI_MEM_S_SRAM_DRD_CMD_REG        (DR_REG_PSRAM_MSPI0_BASE + 0x48)
#define SPI_MEM_S_SRAM_DWR_CMD_REG        (DR_REG_PSRAM_MSPI0_BASE + 0x4C)
#define SPI_MEM_S_CACHE_SRAM_CMD_BITLEN_S 28

#define SPI_SMEM_S_DDR_REG_AXI            (DR_REG_PSRAM_MSPI0_BASE + 0xD8)
#define SPI_SMEM_S_DDR_EN                 BIT(0)
#define SPI_SMEM_S_VAR_DUMMY              BIT(1)
#define SPI_SMEM_S_DDR_RDAT_SWP           BIT(2)
#define SPI_SMEM_S_DDR_WDAT_SWP           BIT(3)

/* AP octal PSRAM command opcodes + bit widths. */
#define AP_OCT_PSRAM_SYNC_WRITE           0x8080
#define AP_OCT_PSRAM_SYNC_READ            0x0000
#define AP_OCT_PSRAM_CMD_BITLEN           16
#define AP_OCT_PSRAM_ADDR_BITLEN          32
#define AP_OCT_PSRAM_RD_DUMMY_BITLEN      (2 * (18 - 1))   /* 34 */
#define AP_OCT_PSRAM_WR_DUMMY_BITLEN      (2 * (9 - 1))    /* 16 */

static void psram_axi_config(void)
{
	u32 sctrl;
	u32 cmd_reg;

	/* SRAM_DWR_CMD: write cmd value + bitlen. */
	writel((u32)AP_OCT_PSRAM_SYNC_WRITE |
	       ((AP_OCT_PSRAM_CMD_BITLEN - 1) << SPI_MEM_S_CACHE_SRAM_CMD_BITLEN_S),
	       (void *)SPI_MEM_S_SRAM_DWR_CMD_REG);

	/* SRAM_DRD_CMD: read cmd value + bitlen. */
	writel((u32)AP_OCT_PSRAM_SYNC_READ |
	       ((AP_OCT_PSRAM_CMD_BITLEN - 1) << SPI_MEM_S_CACHE_SRAM_CMD_BITLEN_S),
	       (void *)SPI_MEM_S_SRAM_DRD_CMD_REG);

	/* CACHE_SCTRL: combine 4-byte addr, RD/WR cmd enables, RD/WR
	 * dummy enables + cyclelens, addr bitlen, octal mode flag.
	 */
	sctrl = SPI_MEM_S_CACHE_USR_SADDR_4BYTE |
		SPI_MEM_S_USR_WR_SRAM_DUMMY |
		SPI_MEM_S_USR_RD_SRAM_DUMMY |
		SPI_MEM_S_CACHE_SRAM_USR_RCMD |
		SPI_MEM_S_CACHE_SRAM_USR_WCMD |
		SPI_MEM_S_SRAM_OCT |
		((u32)(AP_OCT_PSRAM_RD_DUMMY_BITLEN - 1)
		 << SPI_MEM_S_SRAM_RDUMMY_CYCLELEN_S) |
		((u32)(AP_OCT_PSRAM_ADDR_BITLEN - 1)
		 << SPI_MEM_S_SRAM_ADDR_BITLEN_S) |
		((u32)(AP_OCT_PSRAM_WR_DUMMY_BITLEN - 1)
		 << SPI_MEM_S_SRAM_WDUMMY_CYCLELEN_S);
	writel(sctrl, (void *)SPI_MEM_S_CACHE_SCTRL_REG);

	/* SRAM_CMD: octal CMD/ADDR/DOUT/DIN + WOUT dummy. Preserves
	 * default SDUMMY_RIN=1 via RMW. Clear HEX bits explicitly.
	 */
	cmd_reg = readl((void *)SPI_MEM_S_SRAM_CMD_REG);
	cmd_reg |= SPI_MEM_S_SCMD_OCT | SPI_MEM_S_SADDR_OCT |
		   SPI_MEM_S_SDOUT_OCT | SPI_MEM_S_SDIN_OCT |
		   SPI_MEM_S_SDUMMY_WOUT;
	cmd_reg &= ~(SPI_MEM_S_SDIN_HEX | SPI_MEM_S_SDOUT_HEX);
	writel(cmd_reg, (void *)SPI_MEM_S_SRAM_CMD_REG);

	/* SMEM DDR: enable DDR + variable dummy. Clear data swap. */
	writel(SPI_SMEM_S_DDR_EN | SPI_SMEM_S_VAR_DUMMY,
	       (void *)SPI_SMEM_S_DDR_REG_AXI);

	/* CACHE_FCTRL: enable AXI access, clear close-axi-inf default. */
	setbits_le32((void *)SPI_MEM_S_CACHE_FCTRL_REG_SMEM,
		     SPI_MEM_S_AXI_REQ_EN);
	clrbits_le32((void *)SPI_MEM_S_CACHE_FCTRL_REG_SMEM,
		     SPI_CLOSE_AXI_INF_EN);

	/* CTRL1: enable AXI read + write splice. */
	setbits_le32((void *)SPI_MEM_S_CTRL1_REG,
		     SPI_MEM_S_AR_SPLICE_EN | SPI_MEM_S_AW_SPLICE_EN);
}

/*------------------------------------------------------------------------
 * PSRAM MMU map
 *
 * Uses the same SPIMEM2 MMU as OpenSBI's flash MMU but with MMU ID 1
 * (PSRAM) instead of ID 0 (flash). 64 KB pages. Same INDEX/CONTENT
 * register pair, different VALID/ACCESS bit positions:
 *   PSRAM:  bits[9:0] paddr_page | BIT(10) ACCESS_PSRAM | BIT(11) VALID
 *   (Flash: bits[9:0] paddr_page | 0      ACCESS_FLASH | BIT(12) VALID)
 *
 * After writing the entries, invalidate the L1 I-cache for the PSRAM
 * range so any speculative prefetch / autoload that captured stale
 * PSRAM contents pre-MMU is dropped.
 *------------------------------------------------------------------------
 */
#define SPI_MEM_S_MMU_ITEM_CONTENT_REG    (DR_REG_PSRAM_MSPI0_BASE + 0x37C)
#define SPI_MEM_S_MMU_ITEM_INDEX_REG      (DR_REG_PSRAM_MSPI0_BASE + 0x380)
#define SOC_MMU_PSRAM_VALID               BIT(11)
#define SOC_MMU_ACCESS_PSRAM              BIT(10)
#define S31_MMU_PAGE_SIZE                 0x10000U     /* 64 KB */

static void psram_mmu_map(u32 psram_size_bytes)
{
	u32 page_count = psram_size_bytes / S31_MMU_PAGE_SIZE;
	u32 i;

	/* MMU has 1024 entries × 64 KB = 64 MB max. Clamp in case a
	 * larger density code ever appears.
	 */
	if (page_count > 1024)
		page_count = 1024;

	for (i = 0; i < page_count; i++) {
		writel(i, (void *)SPI_MEM_S_MMU_ITEM_INDEX_REG);
		writel(i | SOC_MMU_PSRAM_VALID | SOC_MMU_ACCESS_PSRAM,
		       (void *)SPI_MEM_S_MMU_ITEM_CONTENT_REG);
	}

	/* Drop any I-cache lines that captured stale PSRAM contents (via
	 * speculative prefetch / autoload) before this MMU programming.
	 */
	invalidate_icache_range(PSRAM_VADDR_START,
				PSRAM_VADDR_START +
				page_count * S31_MMU_PAGE_SIZE);
}

int psram_init(void)
{
	psram_ldo_init();
	psram_mpll_init();
	psram_mspi_clock_init();
	psram_pin_init();
	psram_ctrlr_init();
	psram_mode_reg_init();
	psram_axi_config();
	if (!s_psram_size)
		return -1;
	psram_mmu_map(s_psram_size);
	return 0;
}
