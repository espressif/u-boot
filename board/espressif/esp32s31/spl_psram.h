/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * ESP32-S31 PSRAM bring-up. Brings up an Octal PSRAM end-to-end:
 * LDO + MPLL + MSPI controller + mode regs + AXI config + MMU map at
 * 0x50000000. Verified on real silicon to be R/W via `md.l` / `mw.l` /
 * `mtest`. Returns 0 on success, -1 if the chip didn't respond to
 * mode-register readback.
 */

#ifndef _S31_SPL_PSRAM_H_
#define _S31_SPL_PSRAM_H_

#include <linux/types.h>

/*
 * PSRAM XIP window. 16 MB of physical PSRAM is mapped at vaddr
 * [START, END) by psram_init's PSRAM MMU programming.
 */
#define PSRAM_VADDR_START 0x50000000UL
#define PSRAM_VADDR_END   0x51000000UL

int psram_init(void);

#endif /* _S31_SPL_PSRAM_H_ */
