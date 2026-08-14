/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef _S31_SPL_CLK_H_
#define _S31_SPL_CLK_H_

/* First-stage clock-tree bring-up, run from the SPL. */
void s31_spl_mpll_enable(void);
void s31_spl_cpu_switch_to_cpll(void);
void s31_spl_systimer_clk_init(void);

#endif
