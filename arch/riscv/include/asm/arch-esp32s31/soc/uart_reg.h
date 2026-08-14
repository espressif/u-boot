/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * ESP32-S31 UART register definitions.
 *
 * Copyright (C) 2026, Espressif Systems (Shanghai) CO LTD
 */

#ifndef __ESP32S31_SOC_UART_REG_H
#define __ESP32S31_SOC_UART_REG_H

#include <linux/bitops.h>

#define UART_RXFIFO_RD_BYTE_M	0x000000ffU

#define UART_CLKDIV_M		0x00000fffU
#define UART_CLKDIV_FRAG_V	0x0000000fU
#define UART_CLKDIV_FRAG_S	20

#define UART_RXFIFO_CNT_M	0x000000ffU
#define UART_TXFIFO_CNT_M	0x00ff0000U
#define UART_TXFIFO_CNT_S	16

#define UART_RXFIFO_RST		BIT(22)
#define UART_TXFIFO_RST		BIT(23)

#endif /* __ESP32S31_SOC_UART_REG_H */
