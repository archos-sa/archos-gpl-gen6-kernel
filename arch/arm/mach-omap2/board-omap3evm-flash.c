/*
 * linux/arch/arm/mach-omap2/board-omap3evm-flash.c
 * 
 * Copyright (c) 2008 Mistralsolutions pvt Ltd.,
 *
 * Derived from mach-omap2/board-3430sdp-flash.c
 *
 * Maintainer:
 * 	Naveen		anaveen@mistralsolutions.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/onenand_regs.h>
#include <linux/types.h>
#include <linux/io.h>

#include <asm/mach/flash.h>
#include <asm/arch/onenand.h>
#include <asm/arch/board.h>
#include <asm/arch/gpmc.h>
#include <asm/arch/nand.h>

static int omap3evm_onenand_setup(void __iomem *);

static struct mtd_partition omap3evm_onenand_partitions[] = {
        {
                .name           = "X-Loader",
                .offset         = 0,
		.size		= 4 * (64 * 2048),
                .mask_flags     = MTD_WRITEABLE /* force read-only */
        },
        {
                .name           = "U-Boot",
                .offset         = MTDPART_OFS_APPEND,
                .size           =  15*(64*2048),
                .mask_flags     = MTD_WRITEABLE /* force read-only */
        },
        {
                .name           = "params",
                .offset         = MTDPART_OFS_APPEND,
                .size           = 1*(64*2048),
        },
        {
                .name           = "linux",
                .offset         = MTDPART_OFS_APPEND,
                .size           = 40*(64*2048),
        },
        {
                .name           = "jffs2",
                .offset         = MTDPART_OFS_APPEND,
                .size           = MTDPART_SIZ_FULL,
        },
};

static struct omap_onenand_platform_data omap3evm_onenand_data = {
        .parts = omap3evm_onenand_partitions,
        .nr_parts = ARRAY_SIZE(omap3evm_onenand_partitions),
        .onenand_setup = omap3evm_onenand_setup,
	.dma_channel	= -1,	/* disable DMA in OMAP OneNAND driver */
};

static struct platform_device omap3evm_onenand_device = {
	.name		= "omap2-onenand",
	.id		= -1,
	.dev = {
		.platform_data = &omap3evm_onenand_data,
	},
};

/**
 *      omap3evm_onenand_setup - Set the onenand sync mode
 *      @onenand_base:  The onenand base address in GPMC memory map
 *
 */

static int omap3evm_onenand_setup(void __iomem *onenand_base)
	{
      /* nothing is required to be setup for onenand as of now */  
        return 0;
}

void __init omap3evm_flash_init(void)
{
        unsigned long gpmc_base_add, gpmc_cs_base_add;
	u8		cs = 0;
	unsigned char	onenandcs = GPMC_CS_NUM + 1;

        gpmc_base_add = GPMC_VIRT;

        while (cs < GPMC_CS_NUM) {
                int ret = 0;
		gpmc_cs_base_add =
			(gpmc_base_add + GPMC_CS0_BASE + (cs*GPMC_CS_SIZE));

		/* xloader/Uboot would have programmed the NAND/oneNAND
		 * base address for us This is a ugly hack. The proper
		 * way of doing this is to pass the setup of u-boot up
		 * to kernel using kernel params - something on the
		 * lines of machineID. Check if oneNAND is
		 * lines of machineID. Check if Nand/oneNAND is
		 * configured */
			ret = __raw_readl(gpmc_cs_base_add + GPMC_CS_CONFIG7);
			if ((ret & 0x3F) == (ONENAND_MAP >> 24)) {
				/* Found it!! */
				if(onenandcs > GPMC_CS_NUM)
					onenandcs = cs;
			}
		cs++;
	}
	if (onenandcs > GPMC_CS_NUM) {
		printk("OneNAND: Unable to find configuration in GPMC\n ");
		printk("MTD: Unable to find MTD configuration in GPMC "
		       " - not registering.\n");
		return;
	}

	if (onenandcs < GPMC_CS_NUM) {
		omap3evm_onenand_data.cs = onenandcs;
		if (platform_device_register(&omap3evm_onenand_device) < 0)
			printk(KERN_ERR "Unable to register OneNAND device\n");
		}
	}

