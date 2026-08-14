/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * ESP32-S31 mask-ROM entry points.
 *
 * The chip ROM exports a fixed function table; addresses are chip
 * constants from the ROM's own build, fixed for a given mask-ROM
 * revision. The cache helpers dispatch via rom_cache_internal_table_ptr
 * at 0x2F07FFBC, which the SPL keeps intact by living below it.
 *
 * M-mode only.
 *
 * Copyright (C) 2026, Espressif Systems (Shanghai) CO LTD
 */

#ifndef __ESP32S31_ROM_H
#define __ESP32S31_ROM_H

#include <linux/bitops.h>
#include <linux/types.h>

/* Cache maintenance. The map argument selects the L1 caches to act on. */
#define CACHE_MAP_L1_ICACHE_0            BIT(0)
#define CACHE_MAP_L1_ICACHE_1            BIT(1)
#define CACHE_MAP_L1_ICACHE              (CACHE_MAP_L1_ICACHE_0 | CACHE_MAP_L1_ICACHE_1)
#define CACHE_MAP_L1_DCACHE              BIT(4)
#define CACHE_MAP_L1_ALL                 (CACHE_MAP_L1_ICACHE | CACHE_MAP_L1_DCACHE)

typedef void (*rom_void_fn_t)(void);
typedef void (*rom_autoload_fn_t)(u32 autoload);
typedef int (*rom_cache_all_fn_t)(u32 map);
typedef int (*rom_cache_range_fn_t)(u32 map, u32 addr, u32 size);

#define Cache_Invalidate_Addr            ((rom_cache_range_fn_t)0x2F8005E8U)
#define Cache_WriteBack_Addr             ((rom_cache_range_fn_t)0x2F8005F0U)
#define Cache_Invalidate_All             ((rom_cache_all_fn_t)0x2F8005F8U)

#define Cache_Disable_L1_CORE0_ICache    ((rom_void_fn_t)0x2F80068CU)
#define Cache_Enable_L1_CORE0_ICache     ((rom_autoload_fn_t)0x2F800690U)
#define Cache_Disable_L1_CORE1_ICache    ((rom_void_fn_t)0x2F80069CU)
#define Cache_Enable_L1_CORE1_ICache     ((rom_autoload_fn_t)0x2F8006A0U)
#define Cache_Disable_L1_DCache          ((rom_void_fn_t)0x2F8006ACU)
#define Cache_Enable_L1_DCache           ((rom_autoload_fn_t)0x2F8006B0U)

/*
 * SPI flash helpers (legacy esp_rom_spiflash API). The chip argument is
 * the ROM's own state block, reachable via the legacy-data pointer.
 */
#define rom_spiflash_legacy_data         0x2F07FFE0U

typedef int (*rom_sf_status_fn_t)(void *chip, u32 *status);
typedef int (*rom_sf_wstatus_fn_t)(void *chip, u32 status);
typedef int (*rom_sf_waitidle_fn_t)(void *chip);
typedef int (*rom_sf_readmode_fn_t)(u32 mode);
typedef void (*rom_sf_qiopins_fn_t)(u32 wp_gpio, u32 spiconfig);
typedef void (*rom_pad_drv_fn_t)(u32 gpio, u32 drv);

#define esp_rom_spiflash_read_status     ((rom_sf_status_fn_t)0x2F8001A0U)
#define esp_rom_spiflash_read_statushigh ((rom_sf_status_fn_t)0x2F8001A4U)
#define esp_rom_spiflash_write_status    ((rom_sf_wstatus_fn_t)0x2F8001A8U)
#define esp_rom_spiflash_wait_idle       ((rom_sf_waitidle_fn_t)0x2F80012CU)
#define esp_rom_spiflash_config_readmode ((rom_sf_readmode_fn_t)0x2F80019CU)
#define esp_rom_spiflash_select_qio_pins ((rom_sf_qiopins_fn_t)0x2F800184U)
#define rom_gpio_pad_set_drv             ((rom_pad_drv_fn_t)0x2F800740U)

/* readmode argument to esp_rom_spiflash_config_readmode */
#define ESP_ROM_SPIFLASH_QIO_MODE        0U

#endif /* __ESP32S31_ROM_H */
