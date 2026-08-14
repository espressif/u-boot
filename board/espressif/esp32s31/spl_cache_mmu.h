/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef _S31_SPL_CACHE_MMU_H_
#define _S31_SPL_CACHE_MMU_H_

#include <linux/types.h>

/* Flash-side flash MMU entry bits, also used directly by spl_flash.c's
 * s31_spl_flash_map() when it writes its own entries.
 */
#define SOC_MMU_FLASH_VALID  BIT(12)
#define SOC_MMU_ACCESS_FLASH 0U

/* Cache/flash-MMU bring-up, run from the SPL before the flash map. */
void s31_spl_cache_mmu_init(void);

void cache_disable_all(void);
void cache_enable_all(void);
void mmu_write_flash_entry(u32 entry_id, u32 content);

#endif /* _S31_SPL_CACHE_MMU_H_ */
