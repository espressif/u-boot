// SPDX-License-Identifier: GPL-2.0
/*
 * ESP SoC Uart Driver
 *
 * Copyright (C) 2025 Espressif Systems (Shanghai) CO LTD
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <common.h>
#include <clk.h>
#include <debug_uart.h>
#include <dm.h>
#include <errno.h>
#include <fdtdec.h>
#include <log.h>
#include <watchdog.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <linux/compiler.h>
#include <serial.h>
#include <linux/err.h>
#include <asm/arch-esp32s31/soc/uart_reg.h>
DECLARE_GLOBAL_DATA_PTR;

/**
 * The UART_REG_UPDATE_REG register offset is far from the uart base
 * we do not list it in the "struct uart_reg"
 */
#define UART_UPDATE_OFF		0x98

struct uart_esp {
	union
	{
		u32 txfifo;
		u32 rxfifo;
	};
	u32 int_raw;
	u32 int_st;
	u32 int_en;
	u32 int_clr;
	u32 div;
	u32 rx_filt;
	u32 status;
	u32 sync;
};

struct esp_uart_plat {
	unsigned long clock;
	unsigned long base;
	struct uart_esp *regs;
};

/* Set up the baud rate in gd struct */
static void _esp_serial_setbrg(struct uart_esp *regs,
				  unsigned long clock, unsigned long baud)
{
	unsigned long div_value = (clock << 4) / baud;
	unsigned long div = ((div_value >> 4) & UART_CLKDIV_M);
	unsigned long div_frag = (div_value & UART_CLKDIV_FRAG_V) << UART_CLKDIV_FRAG_S;
	writel(div | div_frag, &regs->div);
	writel(1, (void *)((uintptr_t)regs + UART_UPDATE_OFF));
	while (readl((void *)((uintptr_t)regs + UART_UPDATE_OFF)) & 1);
}

static void _esp_serial_init(struct uart_esp *regs)
{
	u32 sync;
	// wait tx idle first
	while((readl(&regs->status) >> UART_TXFIFO_CNT_S) & UART_TXFIFO_CNT_M);
	// set rst fifo
	sync = readl(&regs->sync);
	sync |= (UART_TXFIFO_RST | UART_RXFIFO_RST);
	writel(sync, &regs->sync);
	// write update to valid the rst
	writel(1, (void *)((uintptr_t)regs + UART_UPDATE_OFF));
	while (readl((void *)((uintptr_t)regs + UART_UPDATE_OFF)) & 1);
	// clr rst fifo
	sync &= ~(UART_TXFIFO_RST | UART_RXFIFO_RST);
	writel(sync, &regs->sync);
	// write update to valid the rst clr
	writel(1, (void *)((uintptr_t)regs + UART_UPDATE_OFF));
	while (readl((void *)((uintptr_t)regs + UART_UPDATE_OFF)) & 1);
}

static int _esp_serial_putc(struct uart_esp *regs, const char c)
{
	if (((readl(&regs->status) & UART_TXFIFO_CNT_M) >> UART_TXFIFO_CNT_S) >= 128 )
		return -EAGAIN;

	writel(c, &regs->txfifo);

	return 0;
}

static int _esp_serial_getc(struct uart_esp *regs)
{
	int ch;
	int cnt = (readl(&regs->status) & UART_RXFIFO_CNT_M);
	if (!cnt)
		return -EAGAIN;
	ch = readl(&regs->rxfifo) & UART_RXFIFO_RD_BYTE_M;
	return ch;
}

static int esp_serial_setbrg(struct udevice *dev, int baudrate)
{
	int ret;
	struct clk clk;
	struct esp_uart_plat *plat = dev_get_plat(dev);
	u32 clock = 0;

	ret = clk_get_by_index(dev, 0, &clk);
	if (IS_ERR_VALUE(ret)) {
		debug("ESP UART failed to get clock\n");
		ret = dev_read_u32(dev, "clock-frequency", &clock);
		if (IS_ERR_VALUE(ret)) {
			debug("ESP UART clock not defined\n");
			return 0;
		}
	} else {
		clock = clk_get_rate(&clk);
		if (IS_ERR_VALUE(clock)) {
			debug("ESP UART clock get rate failed\n");
			return 0;
		}
	}
	plat->clock = clock;
	_esp_serial_setbrg(plat->regs, plat->clock, baudrate);

	return 0;
}

static int esp_serial_probe(struct udevice *dev)
{
	struct esp_uart_plat *plat = dev_get_plat(dev);

	/* No need to reinitialize the UART after relocation */
	if (gd->flags & GD_FLG_RELOC)
		return 0;

	_esp_serial_init(plat->regs);

	return 0;
}

static int esp_serial_getc(struct udevice *dev)
{
	int c;
	struct esp_uart_plat *plat = dev_get_plat(dev);
	struct uart_esp *regs = plat->regs;

	while ((c = _esp_serial_getc(regs)) == -EAGAIN) ;

	return c;
}

static int esp_serial_putc(struct udevice *dev, const char ch)
{
	int rc;
	struct esp_uart_plat *plat = dev_get_plat(dev);

	while ((rc = _esp_serial_putc(plat->regs, ch)) == -EAGAIN) ;

	return rc;
}

static int esp_serial_pending(struct udevice *dev, bool input)
{
	struct esp_uart_plat *plat = dev_get_plat(dev);
	struct uart_esp *regs = plat->regs;

	if (input)
		return (readl(&regs->status) & UART_RXFIFO_CNT_M);
	else
		return (readl(&regs->status) & UART_TXFIFO_CNT_M);


	return 0;
}

static int esp_serial_of_to_plat(struct udevice *dev)
{
	struct esp_uart_plat *plat = dev_get_plat(dev);

	plat->base = (uintptr_t)dev_read_addr(dev);
	plat->regs = (struct uart_esp *)(uintptr_t)dev_read_addr(dev);
	if (IS_ERR(plat->regs))
		return PTR_ERR(plat->regs);

	return 0;
}

static const struct dm_serial_ops esp_serial_ops = {
	.putc = esp_serial_putc,
	.getc = esp_serial_getc,
	.pending = esp_serial_pending,
	.setbrg = esp_serial_setbrg,
};

static const struct udevice_id esp_serial_ids[] = {
	{ .compatible = "esp,uart0" },
	{ }
};

U_BOOT_DRIVER(serial_esp) = {
	.name	= "serial_esp",
	.id	= UCLASS_SERIAL,
	.of_match = esp_serial_ids,
	.of_to_plat = esp_serial_of_to_plat,
	.plat_auto	= sizeof(struct esp_uart_plat),
	.probe = esp_serial_probe,
	.ops	= &esp_serial_ops,
};
