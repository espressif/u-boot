// SPDX-License-Identifier: GPL-2.0+
/*
 * ESP32-S31 cache maintenance for U-Boot proper (S-mode).
 *
 * No Zicbom, no fence.i reach to the SoC-external flash/PSRAM cache, so
 * S-mode delegates to OpenSBI's DMA cache-maintenance vendor ecall.
 * OpenSBI rejects ranges outside the flash/PSRAM windows as a no-op
 * (e.g. HP-SRAM DMA buffers need none), so the return value is ignored.
 *
 * Copyright (C) 2026, Espressif Systems (Shanghai) CO LTD
 */

#include <cpu_func.h>
#include <linux/types.h>
#include <asm/sbi.h>

/* Funcids match esp32s31_vendor_ext_provider: a0 = vaddr, a1 = size. */
#define S31_SBI_CACHE_WBACK       0
#define S31_SBI_CACHE_INVAL       1
#define S31_SBI_ICACHE_SYNC       3
#define S31_SBI_ICACHE_SYNC_RANGE 4
#define S31_SBI_DCACHE_WBACK_ALL  5

/* Vendor extension ID mirrors OpenSBI's dispatch:
 * SBI_EXT_VENDOR_START + (mvendorid & window). Cached after one
 * SBI base-extension call.
 */
#define SBI_EXT_VENDOR_START      0x09000000UL
#define SBI_EXT_VENDOR_END        0x09FFFFFFUL

static long s31_cache_extid(void)
{
	static long extid;
	long mvendorid = 0;

	if (!extid) {
		sbi_get_mvendorid(&mvendorid);
		extid = SBI_EXT_VENDOR_START +
			(mvendorid & (SBI_EXT_VENDOR_END -
				      SBI_EXT_VENDOR_START));
	}
	return extid;
}

static void s31_cache_op(int fid, unsigned long start, unsigned long end)
{
	sbi_ecall(s31_cache_extid(), fid, start, end - start, 0, 0, 0, 0);
}

void flush_dcache_range(unsigned long start, unsigned long end)
{
	s31_cache_op(S31_SBI_CACHE_WBACK, start, end);
}

void invalidate_dcache_range(unsigned long start, unsigned long end)
{
	s31_cache_op(S31_SBI_CACHE_INVAL, start, end);
}

void invalidate_icache_range(unsigned long start, unsigned long end)
{
	s31_cache_op(S31_SBI_ICACHE_SYNC_RANGE, start, end);
	asm volatile("fence.i" ::: "memory");
}

void flush_dcache_all(void)
{
	sbi_ecall(s31_cache_extid(), S31_SBI_DCACHE_WBACK_ALL, 0, 0, 0, 0, 0,
		  0);
}
