/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Board-specific U-Boot config header for ESP32-S31.
 *
 * Copyright (C) 2026, Espressif Systems (Shanghai) CO LTD
 */

#ifndef __ESP32S31_CONFIG_H
#define __ESP32S31_CONFIG_H

/*
 * SPL reads the FIT from the flash XIP window (mapped by the SPL
 * before load; see board/espressif/esp32s31/spl_flash.c).
 */
#define CFG_SYS_UBOOT_BASE     0x40000000

/*
 * booti allocates below board_get_usable_ram_top() by default, which is
 * the tiny HP-RAM bank (316 KB, mostly consumed by U-Boot's own
 * relocation). Point the lmb allocator at PSRAM instead.
 */
#define CFG_EXTRA_ENV_SETTINGS \
	"bootm_low=0x50000000\0" \
	"bootm_size=0x1000000\0"

#endif /* __ESP32S31_CONFIG_H */
