// SPDX-License-Identifier: GPL-2.0+
/*
 * ESP32-S31 SPL cache MMU function wrappers for SPL M-mode.
 *
 * Copyright (C) 2026, Espressif Systems (Shanghai) CO LTD
 */

#include <cpu_func.h>
#include <linux/bitops.h>
#include <linux/types.h>
#include <asm/arch-esp32s31/rom.h>

void flush_dcache_range(unsigned long start, unsigned long end)
{
	if (start < end)
		Cache_WriteBack_Addr(CACHE_MAP_L1_DCACHE, start, end - start);
}

void invalidate_icache_range(unsigned long start, unsigned long end)
{
	if (start < end)
		Cache_Invalidate_Addr(CACHE_MAP_L1_ICACHE_0, start, end - start);
	asm volatile("fence.i" ::: "memory");
}
