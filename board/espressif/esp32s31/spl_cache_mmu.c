// SPDX-License-Identifier: GPL-2.0+
/*
 * ESP32-S31 SPL cache/flash-MMU bring-up. Ported from the OpenSBI port's
 * cache_mmu.c so the SPL can map flash into the XIP window (0x40000000)
 * and read the FIT (OpenSBI + dtb) from it. Register/ROM addresses are
 * chip constants.
 */

#include <asm/io.h>
#include <linux/types.h>
#include <asm/arch-esp32s31/rom.h>

#include "spl_cache_mmu.h"

#define DR_REG_FLASH_SPI0_BASE         0x20500000U
#define DR_REG_PSRAM_MSPI0_BASE        0x20502000U
#define DR_REG_HP_SYS_CLKRST_BASE      0x20587000U

#define SPI_MEM_C_MMU_ITEM_CONTENT_REG (DR_REG_FLASH_SPI0_BASE + 0x37CU)
#define SPI_MEM_C_MMU_ITEM_INDEX_REG   (DR_REG_FLASH_SPI0_BASE + 0x380U)
#define SPI_MEM_C_MMU_POWER_CTRL_REG   (DR_REG_FLASH_SPI0_BASE + 0x384U)
#define SPI_MEM_S_MMU_POWER_CTRL_REG   (DR_REG_PSRAM_MSPI0_BASE + 0x384U)

#define SPI_MMU_PAGE_SIZE_M            (0x3U << 3)
#define SPI_MMU_PAGE_SIZE_FLASH_64K    (2U << 3)
#define SPI_MMU_PAGE_SIZE_PSRAM_64K    (0U << 3)

#define SOC_MMU_FLASH_INVALID          0U

#define HP_SYS_CLKRST_CACHE_CTRL0_REG  (DR_REG_HP_SYS_CLKRST_BASE + 0x38U)
#define CACHE_CTRL0_FORCE_ON_MASK      ((1U << 1) | (1U << 4) | (1U << 7) | (1U << 10))

#define S31_MMU_ENTRY_NUM              1024U

static void reg_set_bits(u32 reg, u32 mask)
{
	writel(readl((void *)reg) | mask, (void *)reg);
}

static void reg_write_field(u32 reg, u32 mask, u32 val)
{
	writel((readl((void *)reg) & ~mask) | (val & mask), (void *)reg);
}

void cache_disable_all(void)
{
	Cache_Disable_L1_CORE0_ICache();
	Cache_Disable_L1_CORE1_ICache();
	Cache_Disable_L1_DCache();
	Cache_Invalidate_All(CACHE_MAP_L1_ALL);
}

void cache_enable_all(void)
{
	Cache_Enable_L1_DCache(0);
	Cache_Enable_L1_CORE0_ICache(0);
	Cache_Enable_L1_CORE1_ICache(0);
}

void mmu_write_flash_entry(u32 entry_id, u32 content)
{
	writel(entry_id, (void *)SPI_MEM_C_MMU_ITEM_INDEX_REG);
	writel(content, (void *)SPI_MEM_C_MMU_ITEM_CONTENT_REG);
}

void s31_spl_cache_mmu_init(void)
{
	u32 i;

	reg_set_bits(HP_SYS_CLKRST_CACHE_CTRL0_REG, CACHE_CTRL0_FORCE_ON_MASK);
	cache_disable_all();
	reg_write_field(SPI_MEM_C_MMU_POWER_CTRL_REG, SPI_MMU_PAGE_SIZE_M,
			SPI_MMU_PAGE_SIZE_FLASH_64K);
	reg_write_field(SPI_MEM_S_MMU_POWER_CTRL_REG, SPI_MMU_PAGE_SIZE_M,
			SPI_MMU_PAGE_SIZE_PSRAM_64K);
	for (i = 0; i < S31_MMU_ENTRY_NUM; i++)
		mmu_write_flash_entry(i, SOC_MMU_FLASH_INVALID);
	cache_enable_all();
}
