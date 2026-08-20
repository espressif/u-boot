// SPDX-License-Identifier: GPL-2.0+
/*
 * ESP32-S31 board support.
 *
 * Copyright (C) 2026, Espressif Systems (Shanghai) CO LTD
 */

#include <init.h>
#include <errno.h>
#include <stdio.h>
#include <linux/libfdt.h>
#include <asm/global_data.h>

DECLARE_GLOBAL_DATA_PTR;

void *board_fdt_blob_setup(int *err)
{
	void *fdt = (void *)(uintptr_t)gd->arch.firmware_fdt_addr;

	*err = 0;
	if (!fdt || fdt_check_header(fdt) != 0) {
		*err = -ENXIO;
		return NULL;
	}
	return fdt;
}

/* Cap RAM top below chip-ROM interface data at 0x2F07FFAC. */
phys_addr_t board_get_usable_ram_top(phys_size_t total_size)
{
	return 0x2f07e600;
}

int board_init(void)
{
	return 0;
}

int board_late_init(void)
{
	puts("ESP32-S31 U-Boot active\n");

	return 0;
}
