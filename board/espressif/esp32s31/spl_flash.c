// SPDX-License-Identifier: GPL-2.0+
/*
 * ESP32-S31 SPL flash address-space map + DIO->QIO retune.
 */

#include <stdio.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/types.h>
#include <asm/arch-esp32s31/rom.h>

#include "spl_cache_mmu.h"
#include "spl_flash.h"

#define DR_REG_CACHE_BASE          0x2C000000U

#define CACHE_L1_ICACHE_CTRL_REG   (DR_REG_CACHE_BASE + 0x0U)
#define CACHE_L1_DCACHE_CTRL_REG   (DR_REG_CACHE_BASE + 0x4U)
#define CACHE_L1_ICACHE_SHUT_IBUS0 BIT(0)
#define CACHE_L1_ICACHE_SHUT_IBUS1 BIT(1)
#define CACHE_L1_DCACHE_SHUT_DBUS0 BIT(0)

#define S31_MMU_PAGE_SIZE          0x10000U
#define S31_MMU_PAGE_MASK          (S31_MMU_PAGE_SIZE - 1U)
#define S31_FLASH_START            0x40000000U

static void reg_clr_bits(u32 reg, u32 mask)
{
	writel(readl((void *)reg) & ~mask, (void *)reg);
}

int s31_spl_flash_map(u32 vaddr, u32 paddr, u32 size)
{
	u32 first_entry, page_count, first_paddr_page, i;

	if (!size || (vaddr & S31_MMU_PAGE_MASK) || (paddr & S31_MMU_PAGE_MASK) ||
	    (size & S31_MMU_PAGE_MASK) || vaddr < S31_FLASH_START)
		return -1;

	first_entry = (vaddr - S31_FLASH_START) / S31_MMU_PAGE_SIZE;
	page_count = size / S31_MMU_PAGE_SIZE;
	first_paddr_page = paddr / S31_MMU_PAGE_SIZE;

	cache_disable_all();
	for (i = 0; i < page_count; i++)
		mmu_write_flash_entry(first_entry + i,
				      (first_paddr_page + i) |
				      SOC_MMU_FLASH_VALID | SOC_MMU_ACCESS_FLASH);
	reg_clr_bits(CACHE_L1_ICACHE_CTRL_REG,
		     CACHE_L1_ICACHE_SHUT_IBUS0 | CACHE_L1_ICACHE_SHUT_IBUS1);
	reg_clr_bits(CACHE_L1_DCACHE_CTRL_REG, CACHE_L1_DCACHE_SHUT_DBUS0);
	cache_enable_all();
	return 0;
}

/*
 * Override the ROM PMA so M-mode can write PSRAM. The ROM marks
 * 0x40000000-0x60000000 as R+X only (entry 15); add higher-priority
 * entry 7 (NAPOT, RWX) over the 64 MB PSRAM window, mirroring OpenSBI's
 * s31_pma_init. Without it, stores into PSRAM (e.g. the FIT load) trap.
 */
void s31_spl_pma_psram_rwx(void)
{
	/* NAPOT addr for base 0x50000000, size 0x04000000. */
	unsigned long addr = ((0x50000000UL | ((0x04000000UL >> 1) - 1)) >> 2);
	/* A=NAPOT(0xC<<28) | R(1<<4) | W(1<<3) | X(1<<2) | EN(1<<0). */
	unsigned long cfg = 0xC000001DUL;

	asm volatile ("csrw 0xbd7, %0" :: "r"(addr));
	asm volatile ("csrw 0xbc7, %0" :: "r"(cfg));
}

/*
 * Upgrade the flash cache read path from the ROM's DIO boot to QIO (4-line
 * 0xEB) @ 80 MHz, ~2x read bandwidth. This is first-stage bring-up (flash
 * controller tuning), so it belongs in the SPL, not in OpenSBI: the SPL runs
 * from uncached HP-RAM and can disable the flash cache to reprogram it
 * without faulting its own instruction fetch. Ported from the OpenSBI port's
 * flash_qio.c; register/ROM addresses are chip constants.
 *
 * Mechanism: memory-mapped (AXI/XIP) reads are driven by SPIMEM0 (the CACHE
 * controller at 0x20500000); QE/status writes go via SPIMEM1 (the COMMAND
 * controller at 0x20501000). Verify-guarded: snapshot a mapped XIP word in
 * DIO, apply QIO, and revert if the read-back differs so a misconfig can
 * never wedge the boot.
 */

/* Dedicated MSPI flash pads (esp32s31 soc/spi_pins.h). */
#define MSPI_PAD_HD                30U
#define MSPI_PAD_WP                28U
#define MSPI_PAD_CS0               26U
#define MSPI_PAD_CLK               31U
#define MSPI_PAD_MISO              27U
#define MSPI_PAD_MOSI              32U

/* SPIMEM1 = flash COMMAND controller (single-line user commands). */
#define SPI1_BASE                  0x20501000U
#define SPI1_CMD                   (SPI1_BASE + 0x00U)
#define SPI1_USER                  (SPI1_BASE + 0x18U)
#define SPI1_USER1                 (SPI1_BASE + 0x1CU)
#define SPI1_USER2                 (SPI1_BASE + 0x20U)
#define CMD_USR                    BIT(18)
#define USR2_CMD_BITLEN_8          (7U << 28)

/* SPIMEM0 = flash CACHE (XIP auto-read) controller. */
#define SPI0_BASE                  0x20500000U
#define SPI0_CTRL                  (SPI0_BASE + 0x08U)
#define SPI0_CTRL2                 (SPI0_BASE + 0x10U)
#define SPI0_USER                  (SPI0_BASE + 0x18U)
#define SPI0_USER1                 (SPI0_BASE + 0x1CU)
#define SPI0_USER2                 (SPI0_BASE + 0x20U)
#define SPI0_RD_STATUS             (SPI0_BASE + 0x2CU)
#define SPI0_CACHE_FCTRL           (SPI0_BASE + 0x3CU)

/*
 * Flash core-clock config (HP_SYS_CLKRST.flash_ctrl0 @ 0x20587064):
 * BBPLL/6 = 80 MHz (flctrl0 = 0xba1).
 */
#define FLASH_CTRL0                0x20587064U
#define FC0_SYS_CLK_EN             BIT(0)
#define FC0_CLK_SRC_SEL_S          5U
#define FC0_CLK_SRC_SEL_M          (0x3U << FC0_CLK_SRC_SEL_S)
#define FC0_PLL_CLK_EN             BIT(7)
#define FC0_CORE_CLK_EN            BIT(8)
#define FC0_CORE_CLK_DIV_S         9U
#define FC0_CORE_CLK_DIV_M         (0xFFU << FC0_CORE_CLK_DIV_S)
#define FLASH_CLK_SRC_BBPLL        1U
#define FLASH_CORE_80M_DIV         6U

/* Mapped XIP word (u-boot.itb first slot) to verify cache reads. */
#define FLASH_XIP_TEST_ADDR        0x40000000U
/* QE = status-register-2 bit 1 => bit 9 of the 16-bit combined status. */
#define FLASH_SR_QE_BIT            BIT(9)

/* Send a bare single-line flash command (no address/data) via SPIMEM1. */
static void s31_flash_cmd(u8 cmd)
{
	u32 guard = 1000000;

	writel(0, (void *)SPI1_USER);
	writel(0, (void *)SPI1_USER1);
	writel(USR2_CMD_BITLEN_8 | cmd, (void *)SPI1_USER2);
	writel(CMD_USR, (void *)SPI1_CMD);
	while ((readl((void *)SPI1_CMD) & CMD_USR) && --guard)
		;
}

/* Reset the flash (exit any continuous-read mode the ROM's DIO boot left):
 * 0x66 reset-enable + 0x99 reset.
 */
static void s31_flash_reset(void)
{
	int i;

	s31_flash_cmd(0x66);
	s31_flash_cmd(0x99);
	for (i = 0; i < 100000; i++)	/* ~flash tRST */
		asm volatile ("" ::: "memory");
}

static u32 s31_flash_read_sr(void *chip)
{
	u32 sr_lo = 0, sr_hi = 0;

	esp_rom_spiflash_read_status(chip, &sr_lo);		/* SR1, bits [7:0]  */
	esp_rom_spiflash_read_statushigh(chip, &sr_hi);	/* SR2, bits [15:8] */

	return (sr_lo & 0x00FFU) | (sr_hi & 0xFF00U);
}

void s31_spl_flash_reconfig_qio(void)
{
	void *chip = *(void **)rom_spiflash_legacy_data;
	u32 sr, ref, chk, fc0;
	u32 o_ctrl, o_user, o_user1, o_user2, o_rdst, o_fctrl, o_fc0;

	if (!chip) {
		printf("flash-qio: no ROM flashchip, staying DIO\n");
		return;
	}

	/* Ensure the flash's Quad-Enable bit is set (SR2 bit 1). */
	sr = s31_flash_read_sr(chip);
	if (!(sr & FLASH_SR_QE_BIT)) {
		esp_rom_spiflash_wait_idle(chip);
		esp_rom_spiflash_write_status(chip, sr | FLASH_SR_QE_BIT);
		esp_rom_spiflash_wait_idle(chip);
		sr = s31_flash_read_sr(chip);
		if (!(sr & FLASH_SR_QE_BIT)) {
			printf("flash-qio: QE set failed, staying DIO\n");
			return;
		}
	}

	/* Snapshot a mapped XIP word (working DIO) + the registers we touch,
	 * so a bad quad read can self-heal back to DIO.
	 */
	ref	= readl((void *)FLASH_XIP_TEST_ADDR);
	o_ctrl	= readl((void *)SPI0_CTRL);
	o_user	= readl((void *)SPI0_USER);
	o_user1	= readl((void *)SPI0_USER1);
	o_user2	= readl((void *)SPI0_USER2);
	o_rdst	= readl((void *)SPI0_RD_STATUS);
	o_fctrl	= readl((void *)SPI0_CACHE_FCTRL);
	o_fc0	= readl((void *)FLASH_CTRL0);

	/* Flash core clock -> BBPLL/6 = 80 MHz. */
	fc0 = (o_fc0 & ~FC0_CORE_CLK_DIV_M) |
	      ((FLASH_CORE_80M_DIV - 1U) << FC0_CORE_CLK_DIV_S);
	fc0 |= FC0_CORE_CLK_EN | FC0_SYS_CLK_EN | FC0_PLL_CLK_EN;
	fc0 = (fc0 & ~FC0_CLK_SRC_SEL_M) |
	      (FLASH_CLK_SRC_BBPLL << FC0_CLK_SRC_SEL_S);

	/* MSPI pad drive strength (bootloader_configure_spi_pins). */
	rom_gpio_pad_set_drv(MSPI_PAD_CLK, 1);
	rom_gpio_pad_set_drv(MSPI_PAD_CS0, 1);
	rom_gpio_pad_set_drv(MSPI_PAD_MISO, 1);
	rom_gpio_pad_set_drv(MSPI_PAD_MOSI, 1);
	rom_gpio_pad_set_drv(MSPI_PAD_WP, 1);
	rom_gpio_pad_set_drv(MSPI_PAD_HD, 1);

	/* Switch to QIO: reset the flash, config_readmode(QIO) programs the
	 * SPIMEM0 AXI cache read path (0xEB), select_qio_pins routes the MSPI
	 * pins, zero CS setup/hold. No input-timing tuning needed at 80 MHz.
	 */
	cache_disable_all();
	writel(fc0, (void *)FLASH_CTRL0);
	s31_flash_reset();
	esp_rom_spiflash_config_readmode(ESP_ROM_SPIFLASH_QIO_MODE);
	esp_rom_spiflash_select_qio_pins(0, 0);
	writel(readl((void *)SPI0_CTRL2) & ~0x1FFFU, (void *)SPI0_CTRL2);
	cache_enable_all();

	chk = readl((void *)FLASH_XIP_TEST_ADDR);
	if (chk != ref) {
		/* Quad read bad — restore DIO so the board still boots. */
		cache_disable_all();
		writel(o_fc0,   (void *)FLASH_CTRL0);
		writel(o_ctrl,  (void *)SPI0_CTRL);
		writel(o_user,  (void *)SPI0_USER);
		writel(o_user1, (void *)SPI0_USER1);
		writel(o_user2, (void *)SPI0_USER2);
		writel(o_rdst,  (void *)SPI0_RD_STATUS);
		writel(o_fctrl, (void *)SPI0_CACHE_FCTRL);
		cache_enable_all();
		printf("flash-qio: quad read failed (0x%08x), reverted to DIO\n",
		       chk);
		return;
	}

	printf("flash-qio: QIO @ 80 MHz enabled\n");
}
