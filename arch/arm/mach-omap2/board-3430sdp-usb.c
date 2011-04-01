/*
 * linux/arch/arm/mach-omap2/board-3430sdp-usb.c
 *
 * Copyright (C) 2007 Texas Instruments
 * Author: Vikram Pandita
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/usb/musb.h>

#include <asm/arch/hardware.h>
#include <asm/arch/usb.h>

/***************************************************/
/* MUSB Controller Platform data                   */
/***************************************************/
#if	defined(CONFIG_USB_MUSB_HDRC) || defined(CONFIG_USB_MUSB_HDRC_MODULE)
static struct resource musb_resources[] = {
	[0] = {
		.start	= HS_BASE,
		.end	= HS_BASE + SZ_8K,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {	/* general IRQ */
		.start	= INT_243X_HS_USB_MC,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {	/* DMA IRQ */
		.start	= INT_243X_HS_USB_DMA,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct musb_hdrc_platform_data musb_plat = {
#ifdef CONFIG_USB_MUSB_OTG
	.mode		= MUSB_OTG,
#elif CONFIG_USB_MUSB_HDRC_HCD
	.mode		= MUSB_HOST,
#elif CONFIG_USB_GADGET_MUSB_HDRC
	.mode		= MUSB_PERIPHERAL,
#endif
	.multipoint	= 1,
	.clock 		= "hsusb_ick",
	.aux_clock	= NULL,
	.set_clock	= NULL,
};

static u64 musb_dmamask = ~(u32)0;

static struct platform_device musb_device = {
	.name		= "musb_hdrc",
	.id		= 0,
	.dev = {
		.dma_mask		= &musb_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.platform_data		= &musb_plat,
	},
	.num_resources	= ARRAY_SIZE(musb_resources),
	.resource	= musb_resources,
};
#endif
/***************************************************/


/***************************************************/
/* EHCI DEVICE for OMAP3430 ES2.0                  */
/***************************************************/
#if     defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)
static struct resource ehci_resources[] = {
	[0] = {
		.start   = L4_BASE+0x64800,
		.end     = L4_BASE+0x64800 + SZ_1K,
		.flags   = IORESOURCE_MEM,
	},
	[1] = {         /* general IRQ */
		.start   = 77,
		.flags   = IORESOURCE_IRQ,
	}
};

static u64 ehci_dmamask = ~(u32)0;
static struct platform_device ehci_device = {
	.name           = "tiomap3430-ehci",
	.id             = 0,
	.dev = {
		.dma_mask               = &ehci_dmamask,
		.coherent_dma_mask      = 0xffffffff,
		.platform_data          = 0x0,
	},
	.num_resources  = ARRAY_SIZE(ehci_resources),
	.resource       = ehci_resources,
};
#endif
/***************************************************/

/***************************************************/
/* OHCI DEVICE for OMAP3430 ES2.0                  */
/***************************************************/
#if     defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
static struct resource ohci_resources[] = {
	[0] = {
		.start   = L4_BASE+0x64400,
		.end     = L4_BASE+0x64400 + SZ_1K,
		.flags   = IORESOURCE_MEM,
	},
	[1] = {         /* general IRQ */
		.start   = 76,
		.flags   = IORESOURCE_IRQ,
	}
};

/* The dmamask must be set for OHCI to work */
static u64 ohci_dmamask = ~(u32)0;

static void usb_release(struct device *dev)
{
        /* normally not freed */
}

static struct platform_device ohci_device = {
	.name           = "tiomap3430-ohci",
	.id             = 0,
	.dev = {
		.release		= usb_release,
		.dma_mask               = &ohci_dmamask,
		.coherent_dma_mask      = 0xffffffff,
		.platform_data          = 0x0,
	},
	.num_resources  = ARRAY_SIZE(ohci_resources),
	.resource       = ohci_resources,
};
/***************************************************/
#endif

void __init sdp_usb_init(void)
{

#if     defined(CONFIG_USB_MUSB_HDRC) || defined(CONFIG_USB_MUSB_HDRC_MODULE)
	if (platform_device_register(&musb_device) < 0) {
		printk(KERN_ERR "Unable to register HS-USB (MUSB) device\n");
		return;
	}
#endif

#if     defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)
	if (platform_device_register(&ehci_device) < 0) {
		printk(KERN_ERR "Unable to register HS-USB (EHCI) device\n");
		return;
	}
#endif

#if     defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
	if (platform_device_register(&ohci_device) < 0) {
		printk(KERN_ERR "Unable to register FS-USB (OHCI) device\n");
		return;
	}
#endif
}

