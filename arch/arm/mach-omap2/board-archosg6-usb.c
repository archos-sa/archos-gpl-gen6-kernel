/*
 * linux/arch/arm/mach-omap2/board-omap3evm-usb.c
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
#include <linux/usb/ehci.h>
#include <linux/err.h>
#include <linux/clk.h>

#include <asm/arch/hardware.h>
#include <asm/arch/usb.h>
#include <asm/arch/gpio.h>
#include <asm/mach-types.h>
#include <asm/arch/mux.h>
#include <asm/arch/board.h>


/***************************************************/
/* MUSB Controller Platform data                   */
/***************************************************/
#if	defined(CONFIG_USB_MUSB_HDRC) || defined(CONFIG_USB_MUSB_HDRC_MODULE)
static struct resource musb_resources[] = {
	[0] = {
		.start	= HS_BASE,
		.end	= HS_BASE + SZ_4K,
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
#if   defined(CONFIG_USB_MUSB_OTG)
	.mode		= MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)  || defined (CONFIG_USB_MUSB_DUAL_ROLE)
	.mode		= MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode		= MUSB_PERIPHERAL,
#endif
	.multipoint	= 1,
	.clock		= "hsusb_ick",
	.aux_clock 	= "sys_clkout1",
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
static struct ehci_platform_data ehci_plat;

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
	.name           = "ehci-omap",
	.id             = 0,
	.dev = {
		.dma_mask               = &ehci_dmamask,
		.coherent_dma_mask      = 0xffffffff,
		.platform_data          = &ehci_plat,
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

static struct g6_gpio gio_usb_id ;
static struct g6_gpio gio_musb_enable ;
static struct g6_gpio gio_ehci_enable ;

int archosg6_set_usb_id( int enable )
{
	if ( _PIN_NB( gio_usb_id ) != 0 ) {
		if ( enable ) 
			omap_set_gpio_dataout( _PIN_NB( gio_usb_id ), 1);
		else
			omap_set_gpio_dataout( _PIN_NB( gio_usb_id ), 0);
	}

	return 0;
} 
EXPORT_SYMBOL(archosg6_set_usb_id);

int archosg6_enable_musb( int enable )
{
	if ( _PIN_NB( gio_musb_enable ) != 0 ) {
		if ( enable ) 
			omap_set_gpio_dataout( _PIN_NB( gio_musb_enable ), 1);
		else
			omap_set_gpio_dataout( _PIN_NB( gio_musb_enable ), 0);
	}

	return 0;
} 
EXPORT_SYMBOL(archosg6_enable_musb);

int archosg6_enable_ehci( int enable )
{
	if ( _PIN_NB( gio_ehci_enable ) != 0 ) {
		if ( enable ) 
			omap_set_gpio_dataout( _PIN_NB( gio_ehci_enable ), 1);
		else
			omap_set_gpio_dataout( _PIN_NB( gio_ehci_enable ), 0);
	}

	return 0;
} 
EXPORT_SYMBOL(archosg6_enable_ehci);


void __init archosg6_usb_init(void)
{
	const struct archosg6_usb_config *usb_cfg;
	usb_cfg = omap_get_config( ARCHOS_TAG_USB, struct archosg6_usb_config );
	if (usb_cfg == NULL) {
		printk(KERN_DEBUG "archosg6_usb_init: no board configuration found\n");
		return;
	}
	if ( hardware_rev >= usb_cfg->nrev ) {
		printk(KERN_DEBUG "archosg6_usb_init: hardware_rev (%i) >= nrev (%i)\n",
			hardware_rev, usb_cfg->nrev);
		return;
	}

	if ( _PIN_NB (usb_cfg->rev[hardware_rev].enable_usb_ehci) != 0 ) {

		gio_ehci_enable = usb_cfg->rev[hardware_rev].enable_usb_ehci;

		_INIT_OUTPUT( gio_ehci_enable );

#if     defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)
		ehci_plat.usb_enable_pin = _PIN_NB( gio_ehci_enable );
		ehci_plat.usb_enable_delay = 10;
#endif
		/* reset the PHY */
		archosg6_enable_ehci( 0 );
		udelay(2);
		archosg6_enable_ehci( 1 );

		/* pin mux for EHCI Port 2 */
		omap_cfg_reg(AE7_3430_USB2HS_PHY_CLK);
		omap_cfg_reg(AF7_3430_USB2HS_PHY_STP);
		omap_cfg_reg(AG7_3430_USB2HS_PHY_DIR);
		omap_cfg_reg(AH7_3430_USB2HS_PHY_NXT);
		omap_cfg_reg(AG8_3430_USB2HS_PHY_DATA0);
		omap_cfg_reg(AH8_3430_USB2HS_PHY_DATA1);
		omap_cfg_reg(AB2_3430_USB2HS_PHY_DATA2);
		omap_cfg_reg(V3_3430_USB2HS_PHY_DATA3);
		omap_cfg_reg(Y2_3430_USB2HS_PHY_DATA4);
		omap_cfg_reg(Y3_3430_USB2HS_PHY_DATA5);
		omap_cfg_reg(Y4_3430_USB2HS_PHY_DATA6);
		omap_cfg_reg(AA3_3430_USB2HS_PHY_DATA7);
	}

	/* Inventra USB */
	gio_usb_id = usb_cfg->rev[hardware_rev].usb_id;

	_INIT_OUTPUT( gio_usb_id );
	omap_set_gpio_dataout( _PIN_NB( gio_usb_id ), 0 );

	/* keep PHY in reset to save power... */	
	gio_musb_enable = usb_cfg->rev[hardware_rev].enable_usb_musb;

	_INIT_OUTPUT( gio_musb_enable );
	omap_set_gpio_dataout( _PIN_NB ( gio_musb_enable ), 0);
	
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

