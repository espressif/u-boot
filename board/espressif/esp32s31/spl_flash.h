/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef _S31_SPL_FLASH_H_
#define _S31_SPL_FLASH_H_

#include <linux/types.h>

/* SPL flash-map + PSRAM PMA + DIO->QIO bring-up. */
int s31_spl_flash_map(u32 vaddr, u32 paddr, u32 size);
void s31_spl_pma_psram_rwx(void);
void s31_spl_flash_reconfig_qio(void);

#endif /* _S31_SPL_FLASH_H_ */
