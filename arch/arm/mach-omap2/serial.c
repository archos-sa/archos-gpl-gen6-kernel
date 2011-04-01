/*
 * arch/arm/mach-omap2/serial.c
 *
 * OMAP2 serial support.
 *
 * Copyright (C) 2005 Nokia Corporation
 * Author: Paul Mundt <paul.mundt@nokia.com>
 *
 * Based off of arch/arm/mach-omap/omap1/serial.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/serial_8250.h>
#include <linux/serial_reg.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/console.h>
#ifdef CONFIG_TRACK_RESOURCES
#include <linux/device.h>
#endif
#include <asm/io.h>

#include <asm/arch/common.h>
#include <asm/arch/board.h>

#include "prcm-regs.h"

static struct clk * uart1_ick = NULL;
static struct clk * uart1_fck = NULL;
static struct clk * uart2_ick = NULL;
static struct clk * uart2_fck = NULL;
static struct clk * uart3_ick = NULL;
static struct clk * uart3_fck = NULL;

#ifdef CONFIG_ARCH_OMAP34XX
struct omap_uart_regs {
	u16 dll;
	u16 dlh;
	u16 ier;
	u16 sysc;
	u16 scr;
	u16 wer;
};
static struct omap_uart_regs uart_ctx[OMAP_MAX_NR_PORTS];
#endif /* #ifdef CONFIG_ARCH_OMAP34XX */

static struct plat_serial8250_port serial_platform_data[] = {
	{
		.membase	= (char *)IO_ADDRESS(OMAP_UART1_BASE),
		.mapbase	= (unsigned long)OMAP_UART1_BASE,
		.irq		= 72,
		.flags		= UPF_BOOT_AUTOCONF,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= OMAP16XX_BASE_BAUD * 16,
	}, {
		.membase	= (char *)IO_ADDRESS(OMAP_UART2_BASE),
		.mapbase	= (unsigned long)OMAP_UART2_BASE,
		.irq		= 73,
		.flags		= UPF_BOOT_AUTOCONF,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= OMAP16XX_BASE_BAUD * 16,
	}, {
		.membase	= (char *)IO_ADDRESS(OMAP_UART3_BASE),
		.mapbase	= (unsigned long)OMAP_UART3_BASE,
		.irq		= 74,
		.flags		= UPF_BOOT_AUTOCONF,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= OMAP16XX_BASE_BAUD * 16,
	}, {
		.flags		= 0
	}
};

struct uart_params {
	volatile u32	*wken_regaddr;
	u8		wken_bitpos;
	volatile u32	*sysc_regaddr;
	volatile u32	*wer_regaddr;
};

struct uart_params uart_wkup_params[OMAP_MAX_NR_PORTS];

static inline unsigned int serial_read_reg(struct plat_serial8250_port *up,
					   int offset)
{
	offset <<= up->regshift;
	return (unsigned int)__raw_readb(up->membase + offset);
}

static inline void serial_write_reg(struct plat_serial8250_port *p, int offset,
				    int value)
{
	offset <<= p->regshift;
	__raw_writeb(value, (unsigned long)(p->membase + offset));
}

/*
 * Internal UARTs need to be initialized for the 8250 autoconfig to work
 * properly. Note that the TX watermark initialization may not be needed
 * once the 8250.c watermark handling code is merged.
 */
static void __init omap_serial_reset(struct plat_serial8250_port *p)
{
#if 0
	unsigned long timeout;
	int lcr = 0;
#ifdef CONFIG_DEBUG_LL
	int dll, dlh;
	
	lcr = serial_read_reg(p, UART_LCR);
	serial_write_reg(p, UART_LCR, UART_LCR_DLAB | lcr);
	dll = serial_read_reg(p, UART_DLL);
	dlh = serial_read_reg(p, UART_DLM);
#endif

	serial_write_reg(p, UART_OMAP_SYSC, 0x2);
	timeout = jiffies + msecs_to_jiffies(10);
	while (!(serial_read_reg(p, UART_OMAP_SYSS) & 1)
		&& time_before(jiffies, timeout)) {
		udelay(10);
	}
	if (unlikely(!(serial_read_reg(p, UART_OMAP_SYSS) & 1)))
		printk(KERN_WARNING ": timeout waiting for uart reset\n");

	serial_write_reg(p, UART_LCR, 0xBF);
	serial_write_reg(p, UART_EFR, UART_EFR_ECB);
#ifdef CONFIG_DEBUG_LL
	serial_write_reg(p, UART_DLL, dll);
	serial_write_reg(p, UART_DLM, dlh);
#endif
	serial_write_reg(p, UART_LCR, lcr & ~UART_LCR_DLAB);
	serial_write_reg(p, UART_IER, 0x00);
	serial_write_reg(p, UART_MCR, 0x00);
	serial_write_reg(p, UART_FCR, 7);
#endif
	serial_write_reg(p, UART_OMAP_MDR1, 0x07);
	serial_write_reg(p, UART_OMAP_SCR, 0x08);
	serial_write_reg(p, UART_OMAP_MDR1, 0x00);
#if 1
	serial_write_reg(p, UART_OMAP_SYSC,  (0x02 << 3) | (1 << 2) | (1 << 0));
#else
	serial_write_reg(p, UART_OMAP_SYSC, 0x01);
#endif
}

#ifdef CONFIG_TRACK_RESOURCES
/* device name needed for resource tracking layer */
struct device_driver serial_drv = {
	.name =  "uart",
};

struct device serial_dev = {
	.driver = &serial_drv,
};
#endif

#ifdef CONFIG_ARCH_OMAP3
void omap_serial_wken(void)
{
	int i;
	u32 uart_sysc, pm_wken;

	uart_wkup_params[0].wken_regaddr = &PM_WKEN1_CORE;
	uart_wkup_params[0].wken_bitpos = 13;
	uart_wkup_params[0].sysc_regaddr = &PRCM_UART1_SYSCONFIG;
	uart_wkup_params[0].wer_regaddr = &PRCM_UART1_WER;

	uart_wkup_params[1].wken_regaddr = &PM_WKEN1_CORE;
	uart_wkup_params[1].wken_bitpos = 14;
	uart_wkup_params[1].sysc_regaddr = &PRCM_UART2_SYSCONFIG;
	uart_wkup_params[1].wer_regaddr = &PRCM_UART2_WER;

	uart_wkup_params[2].wken_regaddr = &PM_WKEN_PER;
	uart_wkup_params[2].wken_bitpos = 11;
	uart_wkup_params[2].sysc_regaddr = &PRCM_UART3_SYSCONFIG;
	uart_wkup_params[2].wer_regaddr = &PRCM_UART3_WER;

	for (i = 0; i < OMAP_MAX_NR_PORTS; i++) {
		struct plat_serial8250_port *p = serial_platform_data + i;
		
		/* skip disabled ports */
		if (p->mapbase == 0)
			continue;

		/* Enable module wakeup capability */
		pm_wken = *uart_wkup_params[i].wken_regaddr;
		pm_wken |= 1<<(uart_wkup_params[i].wken_bitpos);
		*uart_wkup_params[i].wken_regaddr = pm_wken;

		/* Enable smart idle in UART SYSC */
		uart_sysc = *uart_wkup_params[i].sysc_regaddr;
		uart_sysc |= ((2<<3)|(1<<2)|(1));
		*uart_wkup_params[i].sysc_regaddr = uart_sysc;

		/* Enabling all wakeup bits in WER_REG */
		*uart_wkup_params[i].wer_regaddr = 0x3F;
	}
}
#endif

void __init omap_serial_init()
{
	int i;
	const struct omap_uart_config *info;
#ifdef CONFIG_TRACK_RESOURCES
	struct device *dev = &serial_dev;
#else
	struct device *dev = NULL;
#endif

	/*
	 * Make sure the serial ports are muxed on at this point.
	 * You have to mux them off in device drivers later on
	 * if not needed.
	 */

	info = omap_get_config(OMAP_TAG_UART,
			       struct omap_uart_config);

	if (info == NULL)
		return;

	for (i = 0; i < OMAP_MAX_NR_PORTS; i++) {
		struct plat_serial8250_port *p = serial_platform_data + i;

		if (!(info->enabled_uarts & (1 << i))) {
			p->membase = 0;
			p->mapbase = 0;
			continue;
		}

		switch (i) {
		case 0:
			uart1_ick = clk_get(dev, "uart1_ick");
			if (IS_ERR(uart1_ick))
				printk("Could not get uart1_ick\n");
			else {
				clk_enable(uart1_ick);
			}

			uart1_fck = clk_get(dev, "uart1_fck");
			if (IS_ERR(uart1_fck))
				printk("Could not get uart1_fck\n");
			else {
				clk_enable(uart1_fck);
			}
			break;
		case 1:
			uart2_ick = clk_get(dev, "uart2_ick");
			if (IS_ERR(uart2_ick))
				printk("Could not get uart2_ick\n");
			else {
				clk_enable(uart2_ick);
			}

			uart2_fck = clk_get(dev, "uart2_fck");
			if (IS_ERR(uart2_fck))
				printk("Could not get uart2_fck\n");
			else {
				clk_enable(uart2_fck);
			}
			break;
		case 2:
			uart3_ick = clk_get(dev, "uart3_ick");
			if (IS_ERR(uart3_ick))
				printk("Could not get uart3_ick\n");
			else {
				clk_enable(uart3_ick);
			}

			uart3_fck = clk_get(dev, "uart3_fck");
			if (IS_ERR(uart3_fck))
				printk("Could not get uart3_fck\n");
			else {
				clk_enable(uart3_fck);
			}
			break;
		}
	
		omap_serial_reset(p);
	}
#ifdef CONFIG_ARCH_OMAP34XX
#ifndef CONFIG_OMAP_SERIAL_WAKE
	omap_serial_wken();
#endif
#endif
}

#ifdef CONFIG_ARCH_OMAP34XX
void omap_uart_save_ctx(int unum)
{
 	u16 lcr = 0;
	struct plat_serial8250_port *p = serial_platform_data + unum;

	if (unum >= OMAP_MAX_NR_PORTS)
		return;
	if (p->mapbase == 0)
		return;

	lcr = serial_read_reg(p, UART_LCR);
	serial_write_reg(p, UART_LCR, 0xBF);
	uart_ctx[unum].dll = serial_read_reg(p, UART_DLL);
	uart_ctx[unum].dlh = serial_read_reg(p, UART_DLM);
	serial_write_reg(p, UART_LCR, lcr);
	uart_ctx[unum].ier = serial_read_reg(p, UART_IER);
	uart_ctx[unum].sysc = serial_read_reg(p, UART_OMAP_SYSC);
	uart_ctx[unum].scr = serial_read_reg(p, UART_OMAP_SCR);
	uart_ctx[unum].wer = serial_read_reg(p, UART_OMAP_WER);
}
EXPORT_SYMBOL(omap_uart_save_ctx);

void omap_uart_restore_ctx(int unum)
{
 	u16 efr = 0;
 
	struct plat_serial8250_port *p = serial_platform_data + unum;

	if (unum >= OMAP_MAX_NR_PORTS)
		return;
	if (p->mapbase == 0)
		return;

	serial_write_reg(p, UART_OMAP_MDR1, 0x7);
	serial_write_reg(p, UART_LCR, 0xBF); /* Config B mode */
	efr = serial_read_reg(p, UART_EFR);
	serial_write_reg(p, UART_EFR, UART_EFR_ECB);
	serial_write_reg(p, UART_LCR, 0x0); /* Operational mode */
	serial_write_reg(p, UART_IER, 0x0);
	serial_write_reg(p, UART_LCR, 0xBF); /* Config B mode */
	serial_write_reg(p, UART_DLL, uart_ctx[unum].dll);
	serial_write_reg(p, UART_DLM, uart_ctx[unum].dlh);
	serial_write_reg(p, UART_LCR, 0x0); /* Operational mode */
	serial_write_reg(p, UART_IER, uart_ctx[unum].ier);
	serial_write_reg(p, UART_FCR, 0xA1);
	serial_write_reg(p, UART_LCR, 0xBF); /* Config B mode */
	serial_write_reg(p, UART_EFR, efr);
	serial_write_reg(p, UART_LCR, UART_LCR_WLEN8);
	serial_write_reg(p, UART_OMAP_SCR, uart_ctx[unum].scr);
	serial_write_reg(p, UART_OMAP_WER, uart_ctx[unum].wer);
	serial_write_reg(p, UART_OMAP_SYSC, uart_ctx[unum].sysc);
	serial_write_reg(p, UART_OMAP_MDR1, 0x00); /* UART 16x mode */
}
EXPORT_SYMBOL(omap_uart_restore_ctx);
#endif

static struct platform_device serial_device = {
	.name			= "serial8250",
	.id			= PLAT8250_DEV_PLATFORM,
	.dev			= {
		.platform_data	= serial_platform_data,
	},
};

static int __init omap_init(void)
{
	return platform_device_register(&serial_device);
}
arch_initcall(omap_init);
