/*
 * linux/arch/arm/mach-omap2/id.c
 *
 * OMAP2 CPU identification code
 *
 * Copyright (C) 2005 Nokia Corporation
 * Written by Tony Lindgren <tony@atomide.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <asm/io.h>

unsigned int scalability_status;


#if defined(CONFIG_ARCH_OMAP2420)
#define TAP_BASE	io_p2v(0x48014000)
#elif defined(CONFIG_ARCH_OMAP2430)
#define TAP_BASE	io_p2v(0x4900A000)
#elif defined(CONFIG_ARCH_OMAP34XX)
#define TAP_BASE	io_p2v(0x4830A000)
#endif

#define OMAP_TAP_IDCODE		0x0204
#if defined(CONFIG_ARCH_OMAP34XX)
#define OMAP_TAP_PROD_ID	0x0210
#else
#define OMAP_TAP_PROD_ID	0x0208
#endif

#define OMAP_TAP_DIE_ID_0	0x0218
#define OMAP_TAP_DIE_ID_1	0x021C
#define OMAP_TAP_DIE_ID_2	0x0220
#define OMAP_TAP_DIE_ID_3	0x0224

/* system_rev fields for OMAP2 processors:
 *   CPU id bits     [31:16],
 *   CPU device type [15:12], (unprg,normal,POP)
 *   CPU revision    [11:08]
 *   CPU class bits  [07:00]
 */

struct omap_id {
	u16	hawkeye;	/* Silicon type (Hawkeye id) */
	u8	dev;		/* Device type from production_id reg */
	u32	type;		/* combined type id copied to system_rev */
};

/* Register values to detect the OMAP version */
static struct omap_id omap_ids[] __initdata = {
	{ .hawkeye = 0xb5d9, .dev = 0x0, .type = 0x24200000 },
	{ .hawkeye = 0xb5d9, .dev = 0x1, .type = 0x24201000 },
	{ .hawkeye = 0xb5d9, .dev = 0x2, .type = 0x24202000 },
	{ .hawkeye = 0xb5d9, .dev = 0x4, .type = 0x24220000 },
	{ .hawkeye = 0xb5d9, .dev = 0x8, .type = 0x24230000 },
	{ .hawkeye = 0xb68a, .dev = 0x0, .type = 0x24300000 },
	{ .hawkeye = 0xb7ae, .dev = 0xf, .type = 0x34300000 },
	{ .hawkeye = 0xb7ae, .dev = 0x0, .type = 0x34301000 },
};

static u32 __init read_tap_reg(int reg)
{
	unsigned int regval = 0;
	u32 cpuid;

	/* Reading the IDCODE register on 3430 ES1 results in a
	 * data abort as the register is not exposed on the OCP
	 * Hence reading the Cortex Rev
	 */
	cpuid = read_cpuid(CPUID_ID);

	/* If the processor type is Cortex-A8 and the revision is 0x0
	 * it means its Cortex r0p0 which is 3430 ES1
	 */
	if ((((cpuid >> 4) & 0xFFF) == 0xC08) && ((cpuid & 0xF) == 0x0)) {
		switch (reg) {
		case OMAP_TAP_IDCODE  : regval = 0x0B7AE02F; break;
		/* Making DevType as 0xF in ES1 to differ from ES2 */
		case OMAP_TAP_PROD_ID : regval = 0x000F00F0; break;
		case OMAP_TAP_DIE_ID_0: regval = 0x01000000; break;
		case OMAP_TAP_DIE_ID_1: regval = 0x1012d687; break;
		case OMAP_TAP_DIE_ID_2:	regval = 0x00000000; break;
		case OMAP_TAP_DIE_ID_3:	regval = 0x2d2c0000; break;
		}
	} else
		regval = __raw_readl(TAP_BASE + reg);

	return regval;

}

void __init omap2_check_revision(void)
{
	int ctrl_status = 0;
	int i, j;
	u32 idcode;
	u32 prod_id;
	u16 hawkeye;
	u8  dev_type;
	u8  rev;

	idcode = read_tap_reg(OMAP_TAP_IDCODE);
	prod_id = read_tap_reg(OMAP_TAP_PROD_ID);
	hawkeye = (idcode >> 12) & 0xffff;
	rev = (idcode >> 28) & 0x0f;
	dev_type = (prod_id >> 16) & 0x0f;

#ifdef DEBUG
	printk(KERN_DEBUG "OMAP_TAP_IDCODE 0x%08x REV %i HAWKEYE 0x%04x MANF %03x\n",
		idcode, rev, hawkeye, (idcode >> 1) & 0x7ff);
	printk(KERN_DEBUG "OMAP_TAP_DIE_ID_0: 0x%08x\n",
		read_tap_reg(OMAP_TAP_DIE_ID_0));
	printk(KERN_DEBUG "OMAP_TAP_DIE_ID_1: 0x%08x DEV_REV: %i\n",
		read_tap_reg(OMAP_TAP_DIE_ID_1),
	       (read_tap_reg(OMAP_TAP_DIE_ID_1) >> 28) & 0xf);
	printk(KERN_DEBUG "OMAP_TAP_DIE_ID_2: 0x%08x\n",
		read_tap_reg(OMAP_TAP_DIE_ID_2));
	printk(KERN_DEBUG "OMAP_TAP_DIE_ID_3: 0x%08x\n",
		read_tap_reg(OMAP_TAP_DIE_ID_3));
	printk(KERN_DEBUG "OMAP_TAP_PROD_ID_0: 0x%08x DEV_TYPE: %i\n",
		prod_id, dev_type);
#endif

	/* Check hawkeye ids */
	for (i = 0; i < ARRAY_SIZE(omap_ids); i++) {
		if (hawkeye == omap_ids[i].hawkeye)
			break;
	}

	if (i == ARRAY_SIZE(omap_ids)) {
		printk(KERN_ERR "Unknown OMAP CPU id\n");
		return;
	}

	for (j = i; j < ARRAY_SIZE(omap_ids); j++) {
		if (dev_type == omap_ids[j].dev)
			break;
	}

	if (j == ARRAY_SIZE(omap_ids)) {
		printk(KERN_ERR "Unknown OMAP device type. "
				"Handling it as OMAP%04x\n",
				omap_ids[i].type >> 16);
		j = i;
	}

	/*
	 * system_rev encoding is as follows
	 * system_rev & 0xff000000 -> Omap Class (24xx/34xx)
	 * system_rev & 0xfff00000 -> Omap Sub Class (242x/343x)
	 * system_rev & 0xffff0000 -> Omap type (2420/2422/2423/2430/3430)
	 * system_rev & 0x0000f000 -> Silicon revision (ES1, ES2 )
	 * system_rev & 0x00000700 -> Device Type ( EMU/HS/GP/BAD )
	 * system_rev & 0x0000003f -> sys_boot[0:5]
	 */
	/* Embedding the ES revision info in type field */
	system_rev = omap_ids[j].type;

	ctrl_status = omap_readl(OMAP2_CONTROL_STATUS);
	system_rev |= (ctrl_status & 0x3f);
	system_rev |= (ctrl_status & 0x700);

#ifdef CONFIG_ARCH_OMAP34XX
	/* Reading the scalability status register for versions
	 * greater than 3430 ES1 so as as to identify whether running
	 * in 3410 or 3420 mode
	 */
	if(is_sil_rev_greater_than(OMAP3430_REV_ES1_0)) {
		scalability_status = CONTROL_SCALABLE_OMAP_STATUS & 0xFFFF;
		if ((scalability_status & 0x7FFF) == 0x4C00)
			scalability_status |= OMAP3525;
		else if (((scalability_status >> 8) & 0x3) == 0x2)
			scalability_status |= OMAP3410;
		else if (((scalability_status >> 8) & 0x3) == 0x1)
			scalability_status |= OMAP3420;
	}
#endif
}

EXPORT_SYMBOL(scalability_status);
