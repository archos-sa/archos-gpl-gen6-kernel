/*
 * linux/arch/arm/mach-omap2/irq.c
 *
 * Interrupt handler for OMAP2 boards.
 *
 * Copyright (C) 2005 Nokia Corporation
 * Author: Paul Mundt <paul.mundt@nokia.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <asm/hardware.h>
#include <asm/mach/irq.h>
#include <asm/irq.h>
#include <asm/io.h>

#define INTC_REVISION	0x0000
#define INTC_SYSCONFIG	0x0010
#define INTC_SYSSTATUS	0x0014
#define INTC_CONTROL	0x0048
#define INTC_IDLE	0x0050

#define INTC_MIR_CLEAR0	0x0088
#define INTC_MIR_SET0	0x008c

#define sync_p_port_write()	\
	do { 			\
		int dv = 0;	\
		__asm__ __volatile__ ("mcr p15, 0, %0, c7, c10, 4" : : "r" (dv));	\
	} while(0)
/*
 * OMAP2 has a number of different interrupt controllers, each interrupt
 * controller is identified as its own "bank". Register definitions are
 * fairly consistent for each bank, but not all registers are implemented
 * for each bank.. when in doubt, consult the TRM.
 */
static struct omap_irq_bank {
	unsigned long base_reg;
	unsigned int nr_irqs;
} __attribute__ ((aligned(4))) irq_banks[] = {
	{
		/* MPU INTC */
		.base_reg	= 0,
		.nr_irqs	= 96,
	},
};

/* XXX: FIQ and additional INTC support (only MPU at the moment) */
static void omap_ack_irq(unsigned int irq)
{
	sync_p_port_write();
	__raw_writel(0x1, irq_banks[0].base_reg + INTC_CONTROL);
	sync_p_port_write();
}

static void omap_mask_irq(unsigned int irq)
{
	int offset = (irq >> 5) << 5;

	if (irq >= 64) {
		irq %= 64;
	} else if (irq >= 32) {
		irq %= 32;
	}
	sync_p_port_write();
	__raw_writel(1 << irq, irq_banks[0].base_reg + INTC_MIR_SET0 + offset);
	sync_p_port_write();
}

static void omap_unmask_irq(unsigned int irq)
{
	int offset = (irq >> 5) << 5;

	if (irq >= 64) {
		irq %= 64;
	} else if (irq >= 32) {
		irq %= 32;
	}
	sync_p_port_write();
	__raw_writel(1 << irq, irq_banks[0].base_reg + INTC_MIR_CLEAR0 + offset);
	sync_p_port_write();
}

static void omap_mask_ack_irq(unsigned int irq)
{
	omap_mask_irq(irq);
	omap_ack_irq(irq);
}

static struct irq_chip omap_irq_chip = {
	.name	= "INTC",
	.ack	= omap_mask_ack_irq,
	.mask	= omap_mask_irq,
	.unmask	= omap_unmask_irq,
};

static void __init omap_irq_bank_init_one(struct omap_irq_bank *bank)
{
	unsigned long tmp;

	tmp = __raw_readl(bank->base_reg + INTC_REVISION) & 0xff;
	printk(KERN_INFO "IRQ: Found an INTC at 0x%08lx "
			 "(revision %ld.%ld) with %d interrupts\n",
			 bank->base_reg, tmp >> 4, tmp & 0xf, bank->nr_irqs);

	tmp = __raw_readl(bank->base_reg + INTC_SYSCONFIG);
	tmp |= 1 << 1;	/* soft reset */
	__raw_writel(tmp, bank->base_reg + INTC_SYSCONFIG);

	while (!(__raw_readl(bank->base_reg + INTC_SYSSTATUS) & 0x1))
		/* Wait for reset to complete */;

	/* Enable autoidle */
	__raw_writel(1 << 0, bank->base_reg + INTC_SYSCONFIG);
}

#if defined (CONFIG_FIQ)
void move_irq_to_fiq(int controller_no, int irq_no) {
	unsigned long tmp;
	int offset;
	struct omap_irq_bank *bank = irq_banks + controller_no;

	offset = INTC_ILR_BASE + 4*irq_no;
	tmp = __raw_readl(bank->base_reg + offset) & 0xff;
	tmp |= 1;
	__raw_writel(tmp, bank->base_reg + offset);
}
EXPORT_SYMBOL(move_irq_to_fiq);
#endif

#if defined (CONFIG_PRIORITY_ON_IRQ)
#define SET_PRIO(n)	(n<<2)
#define DEFAULT_PRIO	SET_PRIO(5)

static void _set_irq_default_priority(int irq_no) {
	unsigned long tmp;
	int offset;

	offset = INTC_ILR_BASE + 4*irq_no;
	tmp = __raw_readl(irq_banks[0].base_reg + offset) & 0xffffff03;
	
	switch ( irq_no ) {
		case INT_24XX_GPTIMER2:
			tmp |= (SET_PRIO(1));
			printk("writing at 0x%08x: 0x%08x\n",(irq_banks[0].base_reg + offset),tmp);

		break;
		default:
			tmp |= (DEFAULT_PRIO);
		break;
	}

	__raw_writel(tmp, irq_banks[0].base_reg + offset);
}
#endif

void __init omap_init_irq(void)
{
	unsigned long nr_irqs = 0;
	unsigned int nr_banks = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(irq_banks); i++) {
		struct omap_irq_bank *bank = irq_banks + i;

		if (cpu_is_omap24xx()) {
			bank->base_reg = IO_ADDRESS(OMAP24XX_IC_BASE);
		}
		if (cpu_is_omap34xx()) {
			bank->base_reg = VA_IC_BASE;
		}
		omap_irq_bank_init_one(bank);

		nr_irqs += bank->nr_irqs;
		nr_banks++;
	}

	printk(KERN_INFO "Total of %ld interrupts on %d active controller%s\n",
	       nr_irqs, nr_banks, nr_banks > 1 ? "s" : "");

	for (i = 0; i < nr_irqs; i++) {
		set_irq_chip(i, &omap_irq_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
#if defined (CONFIG_PRIORITY_ON_IRQ)
		_set_irq_default_priority(i);
#endif
	}

}

