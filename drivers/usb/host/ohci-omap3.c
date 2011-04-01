/*
 * OHCI HCD (Host Controller Driver) for USB.
 *
 * (C) Copyright 1999 Roman Weissgaerber <weissg@vienna.at>
 * (C) Copyright 2000-2005 David Brownell
 * (C) Copyright 2002 Hewlett-Packard Company
 * (C) Copyright 2007 Texas Instruments
 * (C) Copyright 2007 Vikram Pandita <vikram.pandita@ti.com>
 *
 * OMAP Bus Glue
 *
 * Modified for OMAP by Tony Lindgren <tony@atomide.com>
 * Based on the 2.4 OMAP OHCI driver originally done by MontaVista Software Inc.
 * and on ohci-sa1111.c by Christopher Hoover <ch@hpl.hp.com>
 *
 * This file is licenced under the GPL.
 */

#include <linux/signal.h>	/* IRQF_DISABLED */
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/mach-types.h>

#include <asm/arch/mux.h>
#include <asm/arch/irqs.h>
#include <asm/arch/gpio.h>
#include <asm/arch/fpga.h>
#include <asm/arch/usb.h>


//Debugging Only
//#define KERN_DEBUG KERN_INFO

#ifndef CONFIG_ARCH_OMAP
#error "This file is OMAP bus glue.  CONFIG_OMAP must be defined."
#endif

extern int usb_disabled(void);
extern int ocpi_enable(void);

//static struct clk *usb_host_ck;
//static struct clk *usb_dc_ck;
static int host_enabled;
static int host_initialized;

#if 0
static void omap_ohci_clock_power(int on)
{
	if (on) {
		clk_enable(usb_dc_ck);
		clk_enable(usb_host_ck);
		/* guesstimate for T5 == 1x 32K clock + APLL lock time */
		udelay(100);
	} else {
		clk_disable(usb_host_ck);
		clk_disable(usb_dc_ck);
	}
}
#endif

/*
 * Board specific gang-switched transceiver power on/off.
 * NOTE:  OSK supplies power from DC, not battery.
 */
static int omap_ohci_transceiver_power(int on)
{
#if 0
	if (on) {
		if (machine_is_omap_innovator() && cpu_is_omap1510())
			fpga_write(fpga_read(INNOVATOR_FPGA_CAM_USB_CONTROL)
				| ((1 << 5/*usb1*/) | (1 << 3/*usb2*/)),
			       INNOVATOR_FPGA_CAM_USB_CONTROL);
		else if (machine_is_omap_osk())
			tps65010_set_gpio_out_value(GPIO1, LOW);
	} else {
		if (machine_is_omap_innovator() && cpu_is_omap1510())
			fpga_write(fpga_read(INNOVATOR_FPGA_CAM_USB_CONTROL)
				& ~((1 << 5/*usb1*/) | (1 << 3/*usb2*/)),
			       INNOVATOR_FPGA_CAM_USB_CONTROL);
		else if (machine_is_omap_osk())
			tps65010_set_gpio_out_value(GPIO1, HIGH);
	}
#endif
	return 0;
}

#ifdef CONFIG_ARCH_OMAP15XX
	//deleted for cleanup of code
#else
#define omap_1510_local_bus_power(x)	{}
#define omap_1510_local_bus_init()	{}
#endif

/*-------------------------------------------------------------------------*/

static int ohci_omap_init(struct usb_hcd *hcd)
{
	struct ohci_hcd		*ohci = hcd_to_ohci(hcd);
	//struct omap_usb_config	*config = hcd->self.controller->platform_data;
	int			need_transceiver = 1; //(config->otg != 0);
	int			ret;

	dev_dbg(hcd->self.controller, "starting USB Controller\n");

	/* boards can use OTG transceivers in non-OTG modes */
	need_transceiver = need_transceiver
			|| machine_is_omap_h2() || machine_is_omap_h3();

	if (cpu_is_omap16xx())
		ocpi_enable();

//#ifdef	CONFIG_ARCH_OMAP_OTG
	if (need_transceiver) {
		ohci->transceiver = otg_get_transceiver();
		if (ohci->transceiver) {
			int	status = otg_set_host(ohci->transceiver,
						&ohci_to_hcd(ohci)->self);
			dev_dbg(hcd->self.controller, "init %s transceiver, status %d\n",
					ohci->transceiver->label, status);
			if (status) {
				if (ohci->transceiver)
					put_device(ohci->transceiver->dev);
				return status;
			}
		} else {
			dev_err(hcd->self.controller, "can't find transceiver\n");
			return -ENODEV;
		}
	}
//#endif

	//omap_ohci_clock_power(1);

	if (cpu_is_omap15xx()) {
		omap_1510_local_bus_power(1);
		omap_1510_local_bus_init();
	}

	if ((ret = ohci_init(ohci)) < 0)
		return ret;

	/* board-specific power switching and overcurrent support */
	if (machine_is_omap_osk() || machine_is_omap_innovator()) {
		u32	rh = roothub_a (ohci);

		/* power switching (ganged by default) */
		rh &= ~RH_A_NPS;

		/* TPS2045 switch for internal transceiver (port 1) */
		if (machine_is_omap_osk()) {
			ohci_to_hcd(ohci)->power_budget = 250;

			rh &= ~RH_A_NOCP;

			/* gpio9 for overcurrent detction */
			omap_cfg_reg(W8_1610_GPIO9);
			omap_request_gpio(9);
			omap_set_gpio_direction(9, 1 /* IN */);

			/* for paranoia's sake:  disable USB.PUEN */
			omap_cfg_reg(W4_USB_HIGHZ);
		}
		ohci_writel(ohci, rh, &ohci->regs->roothub.a);
		distrust_firmware = 0;
	} else if (machine_is_nokia770()) {
		/* We require a self-powered hub, which should have
		 * plenty of power. */
		ohci_to_hcd(ohci)->power_budget = 0;
	}

	/* FIXME khubd hub requests should manage power switching */
	omap_ohci_transceiver_power(1);

	/* board init will have already handled HMC and mux setup.
	 * any external transceiver should already be initialized
	 * too, so all configured ports use the right signaling now.
	 */

	return 0;
}

static void ohci_omap_stop(struct usb_hcd *hcd)
{
	dev_dbg(hcd->self.controller, "stopping USB Controller\n");
	//omap_ohci_clock_power(0);
}


/*-------------------------------------------------------------------------*/
/* PRCM: USBHOST_CM */
#define CM_FCLKEN_USBHOST       (L4_BASE+0x5400)
#define CM_ICLKEN_USBHOST       (L4_BASE+0x5410)
#define CM_IDLEST_USBHOST       (L4_BASE+0x5420)
#define CM_AUTOIDLE_USBHOST     (L4_BASE+0x5430)
#define CM_SLEEPDEP_USBHOST     (L4_BASE+0x5444)
#define CM_CLKSTCTRL_USBHOST    (L4_BASE+0x5448)
#define CM_CLKSTST_USBHOST      (L4_BASE+0x544C)

#define OMAP_USBHOST_BASE       (L4_BASE+0x60000)
#define OMAP_USBHOST_OHCI_BASE  (OMAP_USBHOST_BASE + 0x4400)

/* USBHOST: TLL, UUH, OHCI, EHCI                                           */
#define OMAP_USBHOST_BASE       (L4_BASE+0x60000)

#define OMAP_USBHOST_TLL_BASE   (OMAP_USBHOST_BASE + 0x2000)
#define OMAP_USBHOST_UHH_BASE   (OMAP_USBHOST_BASE + 0x4000)
#define OMAP_USBHOST_OHCI_BASE  (OMAP_USBHOST_BASE + 0x4400)
#define OMAP_USBHOST_EHCI_BASE  (OMAP_USBHOST_BASE + 0x4800)

/* TLL Register Set */
#define OMAP_USBTLL_REVISION    (OMAP_USBHOST_TLL_BASE + (0x00000000))
#define OMAP_USBTLL_SYSCONFIG   (OMAP_USBHOST_TLL_BASE + (0x00000010))
#define OMAP_USBTLL_SYSSTATUS   (OMAP_USBHOST_TLL_BASE + (0x00000014))
#define OMAP_USBTLL_IRQSTATUS   (OMAP_USBHOST_TLL_BASE + (0x00000018))
#define OMAP_USBTLL_IRQENABLE   (OMAP_USBHOST_TLL_BASE + (0x0000001C))
#define OMAP_TLL_SHARED_CONF    (OMAP_USBHOST_TLL_BASE + (0x00000030))
#define OMAP_TLL_CHANNEL_CONF(num) (OMAP_USBHOST_TLL_BASE+(0x00000040+0x04*num))

/* UHH Register Set */
#define OMAP_UHH_REVISION       (OMAP_USBHOST_UHH_BASE + (0x00000000))
#define OMAP_UHH_SYSCONFIG      (OMAP_USBHOST_UHH_BASE + (0x00000010))
#define OMAP_UHH_SYSSTATUS      (OMAP_USBHOST_UHH_BASE + (0x00000014))
#define OMAP_UHH_HOSTCONFIG     (OMAP_USBHOST_UHH_BASE + (0x00000040))
#define OMAP_UHH_DEBUG_CSR      (OMAP_USBHOST_UHH_BASE + (0x00000044))


#if defined(CONFIG_OMAP_OHCI_PHY_MODE) && defined(CONFIG_OMAP_OHCI_PHY_MODE_6PIN_DAT_SE0)
static void setup_func_mux_fs_6pin_phy_mode(void);
#elif defined(CONFIG_OMAP_OHCI_TLL_MODE)
static void setup_func_mux_fs_tll_mode(void);
#endif

/**
 * usb_hcd_omap_probe - initialize OMAP-based HCDs
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller, and
 * then invokes the start() method for the HCD associated with it
 * through the hotplug entry's driver_data.
 */
static int usb_hcd_omap_probe (const struct hc_driver *driver,
			  struct platform_device *pdev)
{
	int retval;
	struct usb_hcd *hcd = 0;
	struct ohci_hcd *ohci;
	struct clk *clock = 0;

	if (pdev->num_resources != 2) {
		printk(KERN_ERR "hcd probe: invalid num_resources: %i\n",
		       pdev->num_resources);
		return -ENODEV;
	}

	if (pdev->resource[0].flags != IORESOURCE_MEM
			|| pdev->resource[1].flags != IORESOURCE_IRQ) {
		printk(KERN_ERR "hcd probe: invalid resource type\n");
		return -ENODEV;
	}

	/* Enable Func Mux for 4-pin mode with ISP1301 on Port1 and Port3*/
#if defined(CONFIG_OMAP_OHCI_PHY_MODE) && defined(CONFIG_OMAP_OHCI_PHY_MODE_6PIN_DAT_SE0)
	setup_func_mux_fs_6pin_phy_mode();
#elif defined(CONFIG_OMAP_OHCI_TLL_MODE)
	setup_func_mux_fs_tll_mode();
#else
#error "WARNING: Missing Func Mux setting for OHCI"
#endif

	//ENABLE OHCI CLOCKS: iclk, fclk

	//Enable DPLL4: uboot does this no need

#if 0
	//Enable Fclk of OHCI
	omap_writel((1<<1) | (1<<0), CM_FCLKEN_USBHOST);	

	usb_host_ck = clk_get(0, "usb_hhc_ck");
	if (IS_ERR(usb_host_ck))
		return PTR_ERR(usb_host_ck);

	if (!cpu_is_omap15xx())
		usb_dc_ck = clk_get(0, "usb_dc_ck");
	else
		usb_dc_ck = clk_get(0, "lb_ck");

	if (IS_ERR(usb_dc_ck)) {
		clk_put(usb_host_ck);
		return PTR_ERR(usb_dc_ck);
	}
#endif


////////////////////////////////////////////////////////////////////////////////
//ENABLE_CLK: START-  ALL CLOCKS: HS/FS for now

#if defined(CONFIG_USB_EHCI_HCD) && defined(CONFIG_MACH_OMAP_3430SDP)
	//Clk and TLL configuration is already done in EHCI
	// No need to configure clocks and TLL again
#else
        /* Enable DPLL 5 : Based on Input of 13Mhz*/
        //omap_writel( (12 << 0) | (120 << 8), 0x48004d4c); /*CM_CLKSEL4_PLL*/
        //omap_writel( 1,     0x48004d50);
        //omap_writel( (7 << 0) | (7 << 4),    0x48004d04);
        //while(!(omap_readl(0x48004d24) & 1))
        //        pr_debug("idlest2 = 0x%x\n", omap_readl(0x48004d24));

	omap_writel(0, CM_AUTOIDLE_USBHOST);
	/* Disable sleep dependency with MPU and IVA */
	omap_writel((0<<2)| (1<<1), CM_SLEEPDEP_USBHOST);
	omap_writel(0, CM_CLKSTCTRL_USBHOST);

	clock = clk_get(&pdev->dev, "usbhost_ick");
	if(IS_ERR(clock))
		return PTR_ERR(clock);
	clk_enable(clock);
	omap_writel((1<<0), CM_ICLKEN_USBHOST);

	clock = clk_get(&pdev->dev, "usbhost2_fck");
	if(IS_ERR(clock))
		return PTR_ERR(clock);
	clk_enable(clock);
	omap_writel((1<<1) | (1<<0), CM_FCLKEN_USBHOST);


	clock = clk_get(&pdev->dev, "usbhost1_fck");
	if(IS_ERR(clock))
		return PTR_ERR(clock);
	clk_enable(clock);
	omap_writel((1<<1) | (1<<0), CM_FCLKEN_USBHOST);

	/* Configure TLL for 60Mhz clk for ULPI */
	clock = clk_get(&pdev->dev, "usbtll_host_sar_fck");
	if(IS_ERR(clock))
		return PTR_ERR(clock);
	clk_enable(clock);

	//TLL-CLK
	omap_writel( (1<<2), 0x48004a08); /*CM_FCLKEN3_CORE*/ //TLL_FCLK

	clock = clk_get(&pdev->dev, "usbtll_ick");
	if(IS_ERR(clock))
		return PTR_ERR(clock);
	clk_enable(clock);
	omap_writel( (1<<2), 0x48004a18); /*CM_ICLKEN3_CORE*/

//ENABLE_CLK : END

//TLL SETUP
        //USBTLL_SYSCONFIG: start soft reset
        omap_writel( (1<<1)|(1<<2)|(1<<8), 0x48062010); /* USBTLL_SYSCONFIG */
        //wait for TLL reset done
        while( !omap_readl(0x48062014) ); /* USBTLL_SYSSTATUS */
        pr_debug("\n TLL RESET DONE");

        omap_writel( (0<<2), 0x48004a38); /* CM_AUTOIDLE3_CORE */

	if( omap_writel( (0<<0), 0x48004a38) )
		 printk("\n->TLL: cannot access");
	else
		 printk("\n->TLL: Accessible");

        omap_writel( 0xBEEF, 0x48062816 );
        pr_debug("ULPI_SCRATCH_REG %x", omap_readl( 0x48062816 ));

        omap_writel( 0x7, 0x4806201C); /*USBTLL_IRQENABLE*/


	//UUH-CONFIGURATION 
	/* ULPI_BYPASS=1 */
        omap_writel( (1<<0), OMAP_UHH_HOSTCONFIG);
	pr_debug("Entered UTMI MODE: %s", omap_readl(OMAP_UHH_HOSTCONFIG)?"no":"yes");
	omap_writel( 0x1108, OMAP_UHH_SYSCONFIG); //0x48064010); //UUH_SYSCONFIG

        printk("\n->UUH waiting for reset to complete....");
	while( (omap_readl(OMAP_UHH_SYSSTATUS) & 1) != 1 );
        printk("\n->UUH reset complete!!");

        printk("\n->UUH waiting for OHCI reset to complete....");
	while( (omap_readl(OMAP_UHH_SYSSTATUS) & 2) != 2 );
        printk("\n->UUH OHCI reset complete!!\n");

        printk("\n->UUH-dbg-csr 0x%x\n", omap_readl(OMAP_UHH_DEBUG_CSR));

#endif

        //TLL_SHARED_CONF: Set divider ratio as 1
        omap_writel( 0x60 , 0x48062030);

        //TLL_CHANNEL_CONF_i[] -->
                //FSLSMODE = 0
                //CHANMODE = 1
        omap_writel( 0x00000EDB, 0x48062040 );
        omap_writel( 0x00000EDB, 0x48062044 );
        omap_writel( 0x00000EDB, 0x48062048 );


        omap_writel( 0x1108, 0x48064010);


	hcd = usb_create_hcd (driver, &pdev->dev, pdev->dev.bus_id);
	if (!hcd) {
		retval = -ENOMEM;
		goto err0;
	}
	hcd->rsrc_start = pdev->resource[0].start;
	hcd->rsrc_len = pdev->resource[0].end - pdev->resource[0].start + 1;

	//if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
	//	dev_dbg(&pdev->dev, "request_mem_region failed\n");
	//	retval = -EBUSY;
	//	goto err1;
	//}

	hcd->regs = (void __iomem *) (int) IO_ADDRESS(hcd->rsrc_start);
	pr_debug("\n\n-->VIRT-OHCI-BASE [0x%x], [0x%x] irq[%d]\n\n",hcd->regs, (unsigned int)io_p2v( 0x48064400 ), pdev->resource[1].start);

	ohci = hcd_to_ohci(hcd);
	ohci_hcd_init(ohci);

	host_initialized = 0;
	host_enabled = 1;

	//irq = platform_get_irq(pdev, 0);
	//if (irq < 0) {
	//	retval = -ENXIO;
	//	goto err2;
	//}
	retval = usb_add_hcd(hcd, pdev->resource[1].start, IRQF_DISABLED);
	if (retval)
		goto err2;

	host_initialized = 1;

	//if (!host_enabled)
		//omap_ohci_clock_power(0);

	return 0;
err2:
//	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
//err1:
	usb_put_hcd(hcd);
err0:
//	clk_put(usb_dc_ck);
//	clk_put(usb_host_ck);
	return retval;
}

#if defined(CONFIG_OMAP_OHCI_PHY_MODE) && defined(CONFIG_OMAP_OHCI_PHY_MODE_6PIN_DAT_SE0)
static void setup_func_mux_fs_6pin_phy_mode(void)
{
        /* PHY mode of operation for board: 750-2083-001 */
        /* ISP1301 connected to Port1 and Port2 and Port3 */
        /* Do Func Mux setting for 4-pin bi-dir mode */

        /* Port1 */
	omap_cfg_reg(AF10_3430_USB1FS_PHY_MM1_RXDP);
	omap_cfg_reg(AG9_3430_USB1FS_PHY_MM1_RXDM);
        omap_cfg_reg(W13_3430_USB1FS_PHY_MM1_RXRCV);
        omap_cfg_reg(W12_3430_USB1FS_PHY_MM1_TXSE0);
        omap_cfg_reg(W11_3430_USB1FS_PHY_MM1_TXDAT);
        omap_cfg_reg(Y11_3430_USB1FS_PHY_MM1_TXEN_N);


        /* Port2 */
        omap_cfg_reg(AF7_3430_USB2FS_PHY_MM2_RXDP);
        omap_cfg_reg(AH7_3430_USB2FS_PHY_MM2_RXDM);
        omap_cfg_reg(AB10_3430_USB2FS_PHY_MM2_RXRCV);
        omap_cfg_reg(AB9_3430_USB2FS_PHY_MM2_TXSE0);
        omap_cfg_reg(W3_3430_USB2FS_PHY_MM2_TXDAT);
        omap_cfg_reg(T4_3430_USB2FS_PHY_MM2_TXEN_N);

        /* Port3 */
        omap_cfg_reg(AH3_3430_USB3FS_PHY_MM3_RXDP);
        omap_cfg_reg(AE3_3430_USB3FS_PHY_MM3_RXDM);
        omap_cfg_reg(AD1_3430_USB3FS_PHY_MM3_RXRCV);
        omap_cfg_reg(AE1_3430_USB3FS_PHY_MM3_TXSE0);
        omap_cfg_reg(AD2_3430_USB3FS_PHY_MM3_TXDAT);
        omap_cfg_reg(AC1_3430_USB3FS_PHY_MM3_TXEN_N);

        return;
}
#elif defined(CONFIG_OMAP_OHCI_TLL_MODE)
static void setup_func_mux_fs_tll_mode(void)
{
	pr_debug("\n OHCI-TLL mode not implemented yet\n");
        /* Set Func mux for : */
        /* Port1 */
        /* Port2 */
        /* Port3 */
}
	return;
}
#endif

/* may be called with controller, bus, and devices active */

/**
 * usb_hcd_omap_remove - shutdown processing for OMAP-based HCDs
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_omap_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 */
static inline void
usb_hcd_omap_remove (struct usb_hcd *hcd, struct platform_device *pdev)
{
	struct ohci_hcd		*ohci = hcd_to_ohci (hcd);

	usb_remove_hcd(hcd);
	if (ohci->transceiver) {
		(void) otg_set_host(ohci->transceiver, 0);
		put_device(ohci->transceiver->dev);
	}
	if (machine_is_omap_osk())
		omap_free_gpio(9);
	//release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
	//clk_put(usb_dc_ck);
	//clk_put(usb_host_ck);
}
/*-------------------------------------------------------------------------*/

static int
ohci_omap_start (struct usb_hcd *hcd)
{
	struct omap_usb_config *config;
	struct ohci_hcd	*ohci = hcd_to_ohci (hcd);
	int		ret;

	printk("\n [%s] \n", __FUNCTION__);
	if (!host_enabled)
		return 0;
	config = hcd->self.controller->platform_data;
#ifdef HACK_OMAP_OHCI //todo
	if (config->otg || config->rwc) {
		ohci->hc_control = OHCI_CTRL_RWC;
		writel(OHCI_CTRL_RWC, &ohci->regs->control);
	}
#endif

	ohci->hc_control = OHCI_CTRL_RWC;
	writel(OHCI_CTRL_RWC, &ohci->regs->control);
	if ((ret = ohci_run (ohci)) < 0) {
		dev_err(hcd->self.controller, "can't start\n");
		ohci_stop (hcd);
		return ret;
	}
	return 0;
}

/*-------------------------------------------------------------------------*/

static const struct hc_driver ohci_omap_hc_driver = {
	.description =		hcd_name,
	.product_desc =		"OMAP OHCI",
	.hcd_priv_size =	sizeof(struct ohci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ohci_irq,
	.flags =		HCD_USB11 | HCD_MEMORY,

	/*
	 * basic lifecycle operations
	 */
	.reset =		ohci_omap_init,
	.start =		ohci_omap_start,
	.stop =			ohci_omap_stop,
	.shutdown = 		ohci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ohci_urb_enqueue,
	.urb_dequeue =		ohci_urb_dequeue,
	.endpoint_disable =	ohci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ohci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ohci_hub_status_data,
	.hub_control =		ohci_hub_control,
	.hub_irq_enable =	ohci_rhsc_enable,
#ifdef	CONFIG_PM
	.bus_suspend =		ohci_bus_suspend,
	.bus_resume =		ohci_bus_resume,
#endif
	.start_port_reset =	ohci_start_port_reset,
};

/*-------------------------------------------------------------------------*/

static int ohci_hcd_omap_drv_probe(struct platform_device *dev)
{
	return usb_hcd_omap_probe(&ohci_omap_hc_driver, dev);
}

static int ohci_hcd_omap_drv_remove(struct platform_device *dev)
{
	struct usb_hcd		*hcd = platform_get_drvdata(dev);

	usb_hcd_omap_remove(hcd, dev);
	platform_set_drvdata(dev, NULL);

	return 0;
}

/*-------------------------------------------------------------------------*/

#ifdef	CONFIG_PM

static int ohci_omap_suspend(struct platform_device *dev, pm_message_t message)
{
	return 0;
#if 0
	struct ohci_hcd	*ohci = hcd_to_ohci(platform_get_drvdata(dev));

	if (time_before(jiffies, ohci->next_statechange))
		msleep(5);
	ohci->next_statechange = jiffies;

	omap_ohci_clock_power(0);
	ohci_to_hcd(ohci)->state = HC_STATE_SUSPENDED;
	dev->dev.power.power_state = PMSG_SUSPEND;
	return 0;
#endif
}

static int ohci_omap_resume(struct platform_device *dev)
{
	return 0;
#if 0
	struct ohci_hcd	*ohci = hcd_to_ohci(platform_get_drvdata(dev));

	if (time_before(jiffies, ohci->next_statechange))
		msleep(5);
	ohci->next_statechange = jiffies;

	omap_ohci_clock_power(1);
	dev->dev.power.power_state = PMSG_ON;
	usb_hcd_resume_root_hub(platform_get_drvdata(dev));
	return 0;
#endif
}

#endif

/*-------------------------------------------------------------------------*/

/*
 * Driver definition to register with the OMAP bus
 */
static struct platform_driver ohci_hcd_omap_driver = {
	.probe		= ohci_hcd_omap_drv_probe,
	.remove		= ohci_hcd_omap_drv_remove,
	.shutdown 	= usb_hcd_platform_shutdown,
#ifdef	CONFIG_PM
	.suspend	= ohci_omap_suspend,
	.resume		= ohci_omap_resume,
#endif
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "tiomap3430-ohci",
	},
};

#if 0
static int __init ohci_hcd_omap_init (void)
{
	printk (KERN_DEBUG "%s: " DRIVER_INFO " (OMAP)\n", hcd_name);
	if (usb_disabled())
		return -ENODEV;

	pr_debug("%s: block sizes: ed %Zd td %Zd\n", hcd_name,
		sizeof (struct ed), sizeof (struct td));

	return platform_driver_register(&ohci_hcd_omap_driver);
}

static void __exit ohci_hcd_omap_cleanup (void)
{
	platform_driver_unregister(&ohci_hcd_omap_driver);
}

module_init (ohci_hcd_omap_init);
module_exit (ohci_hcd_omap_cleanup);
#endif
