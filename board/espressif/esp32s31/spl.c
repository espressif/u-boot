// SPDX-License-Identifier: GPL-2.0+
/*
 * ESP32-S31 U-Boot SPL: the mask ROM's second-stage. Does first-stage SoC
 * bring-up (WDT, cache/flash MMU, PSRAM, flash DIO->QIO, clock tree), then
 * the SPL framework loads a FIT (OpenSBI fw_dynamic + U-Boot proper + dtb)
 * from NOR and hands off to OpenSBI.
 *
 * Copyright (C) 2026, Espressif Systems (Shanghai) CO LTD
 */

#include <spl.h>
#include <stdio.h>
#include <asm/io.h>
#include <asm/sections.h>
#include <linux/bitops.h>

#include "spl_cache_mmu.h"
#include "spl_flash.h"
#include "spl_clk.h"
#include "spl_psram.h"

/* ROM-armed watchdogs (RTC WDT + super-WDT + MWDT0). Disabled from
 * harts_early_init (start.S, earliest C hook) so the chip doesn't reset a
 * few seconds into SPL.
 */
#define S31_RTC_WDT_BASE 0x20801000
#define S31_TIMG0_BASE   0x20580000
#define S31_WDT_WKEY     0x50D83AA1

#define S31_RTC_WDT_SWD_WPROTECT 0x20
#define S31_RTC_WDT_SWD_CONF     0x1C
#define S31_RTC_WDT_WPROTECT     0x18
#define S31_RTC_WDT_CONFIG0      0x00
#define S31_RTC_WDT_SWD_AUTO_FEED_EN BIT(18)

#define S31_TIMG0_WDT_WPROTECT   0x64
#define S31_TIMG0_WDT_CONFIG0    0x48

void harts_early_init(void)
{
	void *rtc = (void *)S31_RTC_WDT_BASE;
	void *tg0 = (void *)S31_TIMG0_BASE;

	/* super-WDT: auto-feed */
	writel(S31_WDT_WKEY, rtc + S31_RTC_WDT_SWD_WPROTECT);
	writel(readl(rtc + S31_RTC_WDT_SWD_CONF) | S31_RTC_WDT_SWD_AUTO_FEED_EN,
	       rtc + S31_RTC_WDT_SWD_CONF);
	writel(0, rtc + S31_RTC_WDT_SWD_WPROTECT);
	/* RTC WDT: unlock, clear config, re-lock */
	writel(S31_WDT_WKEY, rtc + S31_RTC_WDT_WPROTECT);
	writel(0, rtc + S31_RTC_WDT_CONFIG0);
	writel(0, rtc + S31_RTC_WDT_WPROTECT);
	/* MWDT0 (TIMG0) */
	writel(S31_WDT_WKEY, tg0 + S31_TIMG0_WDT_WPROTECT);
	writel(0, tg0 + S31_TIMG0_WDT_CONFIG0);
	writel(0, tg0 + S31_TIMG0_WDT_WPROTECT);
}

void spl_board_init(void)
{
	puts("ESP32-S31 SPL active\n");

	/* Map the flash into the XIP window so SPL can read the FIT and, later,
	 * u-boot's booti bootcmd can reach the kernel Image (0x40400000) and dtb
	 * (0x40200000). Base flash 0x100000 -> vaddr 0x40000000; map 16 MB so
	 * the whole u-boot.itb + kernel-slot span is visible.
	 */
	s31_spl_cache_mmu_init();
	s31_spl_flash_map(0x40000000, 0x100000, 0x1000000);
	s31_spl_pma_psram_rwx();
	/* Upgrade the flash read path to QIO now that it's mapped, so the FIT
	 * load (and later the XIP kernel) fetch at ~2x. First-stage bring-up:
	 * safe here because the SPL runs from uncached HP-RAM.
	 */
	s31_spl_flash_reconfig_qio();
	if (psram_init())
		puts("spl: psram_init failed\n");

	/* Clock-tree bring-up. Order: MPLL (EMAC ref path; psram_init already
	 * did the PSRAM MPLL) -> CPU 40->320 MHz CPLL -> systimer peripheral
	 * clock for Linux's clocksource.
	 */
	s31_spl_mpll_enable();
	s31_spl_cpu_switch_to_cpll();
	s31_spl_systimer_clk_init();
}

u32 spl_boot_device(void)
{
	return BOOT_DEVICE_NOR;
}

/* SPL's control dtb is appended to the SPL image at _end. */
void *board_fdt_blob_setup(int *err)
{
	*err = 0;
	return (void *)_end;
}
