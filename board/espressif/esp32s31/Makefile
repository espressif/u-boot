# SPDX-License-Identifier: GPL-2.0+
#
# Copyright (C) 2026, Espressif Systems (Shanghai) CO LTD

# The SPL (M-mode, pre-OpenSBI) and U-Boot proper (S-mode) share no
# board code: each stage has its own cache and fdt hooks.
ifdef CONFIG_SPL_BUILD
obj-y	+= spl.o
obj-y	+= spl_cache.o
obj-y	+= spl_cache_mmu.o
obj-y	+= spl_flash.o
obj-y	+= spl_clk.o
obj-y	+= spl_psram.o
else
obj-y	+= board.o
obj-y	+= cache.o
endif
