/*
 * ehci-omap.c - driver for USBHOST on OMAP 34xx processor
 *
 * Bus Glue for OMAP34xx USBHOST 3 port EHCI controller
 * Tested on OMAP3430 ES2.0 SDP
 *
 * Copyright (C) 2007-2008 Texas Instruments, Inc.
 * Author: Vikram Pandita <vikram.pandita@ti.com>
 *
 * Based on "ehci-fsl.c" and "ehci-au1xxx.c" ehci glue layers
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/platform_device.h>
#include <linux/usb/ehci.h>
#include <linux/clk.h>

#include <asm/arch/gpio.h>
#include <asm/mach-types.h>


#include "ehci-omap.h"


struct usb_hcd *ghcd;
#ifdef CONFIG_OMAP_EHCI_PHY_MODE
/* EHCI connected to External PHY */

/* External USB connectivity board: 750-2099-001(C)
 * Connected to OMAP3430 SDP
 * The board has Port1 and Port2 connected to ISP1504 in 12-pin ULPI mode
 */

/* ISSUE1:
 *      ISP1504 for input clocking mode needs special reset handling
 * 	Hold the PHY in reset by asserting CHIP_SEL_N signal
 * 	Then start the 60Mhz clock input to PHY
 * 	Release the reset after a delay -
 * 		to get the PHY state machine in working state
 */
#undef EXTERNAL_PHY_RESET
	#define EXT_PHY_RESET_GPIO_PORT1 (57)
	#define EXT_PHY_RESET_GPIO_PORT2 (61)
#define	EXT_PHY_RESET_DELAY		(500)

#endif /* CONFIG_OMAP_EHCI_PHY_MODE */

/*-------------------------------------------------------------------------*/

/* Define USBHOST clocks for clock management */
struct ehci_omap_clock_defs {
	struct clk	*usbhost_ick_clk;
	struct clk	*usbhost2_120m_fck_clk;
	struct clk	*usbhost1_48m_fck_clk;
	struct clk	*usbtll_fck_clk;
	struct clk	*usbtll_ick_clk;
	unsigned	suspended:1;
};

/* Clock names as per clock framework: May change so keep as #defs */
#define USBHOST_ICKL		"usbhost_ick"
#define USBHOST_120M_FCLK	"usbhost2_fck"
#define USBHOST_48M_FCLK	"usbhost1_fck"
#define USBHOST_TLL_ICKL	"usbtll_ick"
#define USBHOST_TLL_FCLK	"usbtll_host_sar_fck"
/*-------------------------------------------------------------------------*/
/* Debug support
 *
 */

/*
 * Support reading/writing to SMSC PHY connected to port 2
 */
 
static int smsc_read(int addr)
{
	unsigned int reg = 0;

	omap_writel(( ( addr & 0x3F ) << EHCI_INSNREG05_ULPI_REGADD_SHIFT) |
 		  (3 << EHCI_INSNREG05_ULPI_OPSEL_SHIFT) |  /* Read */
 		  (2 << EHCI_INSNREG05_ULPI_PORTSEL_SHIFT) |/* Port2 */
 		  (1 << EHCI_INSNREG05_ULPI_CONTROL_SHIFT), /* Start */
 		  EHCI_INSNREG05_ULPI);

	while( ( reg = omap_readl(EHCI_INSNREG05_ULPI)) & (1ul << EHCI_INSNREG05_ULPI_CONTROL_SHIFT) ){
		udelay(2);
	}

	return reg & 0xFF;
}

static void smsc_write(int addr, u8 value)
{
	omap_writel(( ( addr & 0x3F ) << EHCI_INSNREG05_ULPI_REGADD_SHIFT) |
 		  (2 << EHCI_INSNREG05_ULPI_OPSEL_SHIFT) |  /* Write */
 		  (2 << EHCI_INSNREG05_ULPI_PORTSEL_SHIFT) |/* Port2 */
 		  (1 << EHCI_INSNREG05_ULPI_CONTROL_SHIFT) | /* Start */
		  value, EHCI_INSNREG05_ULPI);

	while( omap_readl(EHCI_INSNREG05_ULPI) & (1ul << EHCI_INSNREG05_ULPI_CONTROL_SHIFT) ){
		udelay(2);
	}
}

static void host_write_port(u8 port, const char *buf)
{
	struct usb_bus *bus;
	struct ehci_hcd *ehci = hcd_to_ehci(ghcd);
	u32 port_status;
	u32 cmd;

	/* Reset Device */
	if (!strncmp(buf, "reset", 5)){
		printk("\n RESET PORT \n");
		bus = hcd_to_bus(ghcd);
		if (bus->root_hub->children[port])
			usb_reset_device(bus->root_hub->children[port]);
	}

	if (!strncmp(buf, "t-j", 3)){
		printk("\n TEST_J \n");

		/* Suspend bus first */
		ehci_bus_suspend(ghcd);
		port_status = ehci_readl(ehci, &ehci->regs->port_status[port]);
		cmd = ehci_readl(ehci, &ehci->regs->command);

		port_status |= 1<<16; /* Test_Packet on Port2 */
		ehci_writel(ehci, port_status, &ehci->regs->port_status[port]);

		cmd |= CMD_RUN;
		ehci_writel(ehci, cmd, &ehci->regs->command);
	}

	if (!strncmp(buf, "t-k", 3)){
		printk("\n TEST_K \n");

		/* Suspend bus first */
		ehci_bus_suspend(ghcd);
		port_status = ehci_readl(ehci, &ehci->regs->port_status[port]);
		cmd = ehci_readl(ehci, &ehci->regs->command);

		port_status |= 2<<16; /* Test_Packet on Port2 */
		ehci_writel(ehci, port_status, &ehci->regs->port_status[port]);

		cmd |= CMD_RUN;
		ehci_writel(ehci, cmd, &ehci->regs->command);
	}

	if (!strncmp(buf, "t-se0", 5)){
		printk("\n TEST_SE0_NAK \n");

		/* Suspend bus first */
		ehci_bus_suspend(ghcd);
		port_status = ehci_readl(ehci, &ehci->regs->port_status[port]);
		cmd = ehci_readl(ehci, &ehci->regs->command);

		port_status |= 3<<16; /* Test_Packet on Port2 */
		ehci_writel(ehci, port_status, &ehci->regs->port_status[port]);

		cmd |= CMD_RUN;
		ehci_writel(ehci, cmd, &ehci->regs->command);
	}

	/* Send test packet on suspended port */
	if (!strncmp(buf, "t-pkt", 5)){
		printk("\n TEST_PACKET \n");

		/* Suspend bus first */
		ehci_bus_suspend(ghcd);
		port_status = ehci_readl(ehci, &ehci->regs->port_status[port]);
		cmd = ehci_readl(ehci, &ehci->regs->command);

		/* Set Test packet bit */
		port_status |= 4<<16; /* Test_Packet on Port2 */
		ehci_writel(ehci, port_status, &ehci->regs->port_status[port]);

		cmd |= CMD_RUN;
		ehci_writel(ehci, cmd, &ehci->regs->command);
	}

	if (!strncmp(buf, "t-force", 7)){
		printk("\n TEST_FORCE \n");

		/* Suspend bus first */
		ehci_bus_suspend(ghcd);
		port_status = ehci_readl(ehci, &ehci->regs->port_status[port]);
		cmd = ehci_readl(ehci, &ehci->regs->command);

		port_status |= 5<<16; /* Test_Packet on Port2 */
		ehci_writel(ehci, port_status, &ehci->regs->port_status[port]);

		cmd |= CMD_RUN;
		ehci_writel(ehci, cmd, &ehci->regs->command);
	}
	if (!strncmp(buf, "t-read", 6)){
		printk("SMSC Vendor ID: %0x Product ID:%0x\n", smsc_read(0) + (smsc_read(1)<<8), smsc_read(2) + (smsc_read(3)<<8));
	}

	if (!strncmp(buf, "t-reset", 7)){
		printk("\n SMSC-RESET \n");
			       
		smsc_write( 4, smsc_read( 4 ) | (1<<5));
		printk("SMSC function control: %x\n", smsc_read( 4 ));
	}
}

static ssize_t
host_show_port(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "\nOptions\t--> Description\n"
			"\nreset\t--> Reset Device"
			"\nt-j\t--> Send TEST_J on suspended port"
			"\nt-k\t--> Send TEST_K on suspended port"
			"\nt-pkt\t--> Send TEST_PACKET[53] on suspended port"
			"\nt-force\t--> Send TEST_FORCE_ENABLE on suspended port"
			"\nt-se0\t--> Send TEST_SE0_NAK on suspended port\n\n"
			"\nt-read\t--> Read some data from SMSC PHY\n\n"
);
}

/* Port 1 */
static ssize_t
host_write_port1(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t n)
{
	struct usb_device       *udev = to_usb_device(dev);

	usb_lock_device(udev);
	host_write_port(0, buf);
	usb_unlock_device(udev);
	return n;
}
static DEVICE_ATTR(port1, S_IRUGO | S_IWUSR, host_show_port, host_write_port1);

/* Port 2 */
static ssize_t
host_write_port2(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t n)
{
	struct usb_device       *udev = to_usb_device(dev);

	usb_lock_device(udev);
	host_write_port(1, buf);
	usb_unlock_device(udev);
	return n;
}
static DEVICE_ATTR(port2, S_IRUGO | S_IWUSR, host_show_port, host_write_port2);

#if defined(CONFIG_OMAP_EHCI_TLL_MODE)
/* Port 3 */
static ssize_t
host_write_port3(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t n)
{
	struct usb_device       *udev = to_usb_device(dev);

	usb_lock_device(udev);
	host_write_port(2, buf);
	usb_unlock_device(udev);
	return n;
}
static DEVICE_ATTR(port3, S_IRUGO | S_IWUSR, host_show_port, host_write_port3);
#endif
/*-------------------------------------------------------------------------*/
#ifdef CONFIG_OMAP_EHCI_TLL_MODE

static void omap_usb_utmi_init(struct usb_hcd *hcd, u8 tll_channel_mask)
{
	int i;

	/* Use UTMI Ports of TLL */
	omap_writel((1 << OMAP_UHH_HOSTCONFIG_ULPI_BYPASS_SHIFT)|
			(1<<OMAP_UHH_HOSTCONFIG_INCR4_BURST_EN_SHIFT)|
			(1<<OMAP_UHH_HOSTCONFIG_INCR8_BURST_EN_SHIFT)|
			(1<<OMAP_UHH_HOSTCONFIG_INCR16_BURST_EN_SHIFT)|
			(0<<OMAP_UHH_HOSTCONFIG_INCRX_ALIGN_EN_SHIFT),
						OMAP_UHH_HOSTCONFIG);
	/* Ensure bit is set */
	while (!(omap_readl(OMAP_UHH_HOSTCONFIG) &
		(1 << OMAP_UHH_HOSTCONFIG_ULPI_BYPASS_SHIFT)));

	dev_dbg(hcd->self.controller, "\nEntered UTMI MODE: success\n");

	/* Program the 3 TLL channels upfront */

	for (i = 0; i < OMAP_TLL_CHANNEL_COUNT; i++) {

		/* Disable AutoIdle */
		omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) &
			    ~(1<<OMAP_TLL_CHANNEL_CONF_UTMIAUTOIDLE_SHIFT),
			    OMAP_TLL_CHANNEL_CONF(i));

		/* Disable BitStuffing */
		omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) |
			(1<<OMAP_TLL_CHANNEL_CONF_ULPINOBITSTUFF_SHIFT),
			OMAP_TLL_CHANNEL_CONF(i));

		/* SDR Mode */
		omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) &
			    ~(1<<OMAP_TLL_CHANNEL_CONF_ULPIDDRMODE_SHIFT),
			    OMAP_TLL_CHANNEL_CONF(i));

	}

	/* Program Common TLL register */
	omap_writel((1 << OMAP_TLL_SHARED_CONF_FCLK_IS_ON_SHIFT) |
			(1 << OMAP_TLL_SHARED_CONF_USB_DIVRATION_SHIFT) |
			(0 << OMAP_TLL_SHARED_CONF_USB_180D_SDR_EN_SHIFT) |
			(0 << OMAP_TLL_SHARED_CONF_USB_90D_DDR_EN_SHFT),
				OMAP_TLL_SHARED_CONF);

	/* Enable channels now */
	for (i = 0; i < OMAP_TLL_CHANNEL_COUNT; i++) {

		/* Enable only the channel that is needed */
		if (!(tll_channel_mask & 1<<i))
			continue;

		omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) |
			    (1<<OMAP_TLL_CHANNEL_CONF_CHANEN_SHIFT),
			    OMAP_TLL_CHANNEL_CONF(i));

		omap_writeb(0xBE, OMAP_TLL_ULPI_SCRATCH_REGISTER(i));
		dev_dbg(hcd->self.controller, "\nULPI_SCRATCH_REG[ch=%d]"
			"= 0x%02x\n",
			i+1, omap_readb(OMAP_TLL_ULPI_SCRATCH_REGISTER(i)));
	}
}

#else
# define omap_usb_utmi_init(x, y)	0
#endif


/* omap_start_ehc
 * 	- Start the TI USBHOST controller
 */
static int omap_start_ehc(struct platform_device *dev, struct usb_hcd *hcd)
{
	struct ehci_platform_data *ehci_pdata = dev->dev.platform_data;
	struct ehci_omap_clock_defs *ehci_clocks;
	u32 val;

	dev_dbg(hcd->self.controller, ": starting TI EHCI USB Controller\n");

	ehci_clocks = (struct ehci_omap_clock_defs *)(
				((char *)hcd_to_ehci(hcd)) +
					sizeof(struct ehci_hcd));
	ehci_clocks->suspended = 0;

#if 0 /* DPLL5 programming taken care by clock framework */
	/* Start DPLL5 Programming:
	 * Clock Framework is not doing this now:
	 * This will be done in clock framework later
	 */
	/* Enable DPLL 5 : Based on Input of 13Mhz*/
	cm_write_mod_reg((12 << OMAP3430ES2_PERIPH2_DPLL_DIV_SHIFT)|
			(120 << OMAP3430ES2_PERIPH2_DPLL_MULT_SHIFT),
			PLL_MOD, OMAP3430ES2_CM_CLKSEL4);

	cm_write_mod_reg(1 << OMAP3430ES2_DIV_120M_SHIFT,
			PLL_MOD, OMAP3430ES2_CM_CLKSEL5);

	cm_write_mod_reg((7 << OMAP3430ES2_PERIPH2_DPLL_FREQSEL_SHIFT) |
			(7 << OMAP3430ES2_EN_PERIPH2_DPLL_SHIFT),
			PLL_MOD, OMAP3430ES2_CM_CLKEN2);

	while (!(cm_read_mod_reg(PLL_MOD, CM_IDLEST2) &
				OMAP3430_ST_PERIPH2_CLK))
		dev_dbg(hcd->self.controller,
			"idlest2 = 0x%x\n",
			cm_read_mod_reg(PLL_MOD, CM_IDLEST2));
	/* End DPLL5 programming */

	/* PRCM settings for USBHOST:
	 * Interface clk un-related to domain transition
	 */
	cm_write_mod_reg(0 << OMAP3430ES2_AUTO_USBHOST_SHIFT,
				OMAP3430ES2_USBHOST_MOD, CM_AUTOIDLE);

	/* Disable sleep dependency with MPU and IVA */
	cm_write_mod_reg((0 << OMAP3430ES2_EN_MPU_SHIFT) |
				(0 << OMAP3430ES2_EN_IVA2_SHIFT),
				OMAP3430ES2_USBHOST_MOD, OMAP3430_CM_SLEEPDEP);

	/* Disable Automatic transition of clock */
	cm_write_mod_reg(0 << OMAP3430ES2_CLKTRCTRL_USBHOST_SHIFT,
				OMAP3430ES2_USBHOST_MOD, CM_CLKSTCTRL);
#endif

	/* Enable Clocks for USBHOST */
	ehci_clocks->usbhost_ick_clk = clk_get(&dev->dev,
						USBHOST_ICKL);
	if (IS_ERR(ehci_clocks->usbhost_ick_clk))
		return PTR_ERR(ehci_clocks->usbhost_ick_clk);
	clk_enable(ehci_clocks->usbhost_ick_clk);


	ehci_clocks->usbhost2_120m_fck_clk = clk_get(&dev->dev,
							USBHOST_120M_FCLK);
	if (IS_ERR(ehci_clocks->usbhost2_120m_fck_clk))
		return PTR_ERR(ehci_clocks->usbhost2_120m_fck_clk);
	clk_enable(ehci_clocks->usbhost2_120m_fck_clk);

	ehci_clocks->usbhost1_48m_fck_clk = clk_get(&dev->dev,
						USBHOST_48M_FCLK);
	if (IS_ERR(ehci_clocks->usbhost1_48m_fck_clk))
		return PTR_ERR(ehci_clocks->usbhost1_48m_fck_clk);
	clk_enable(ehci_clocks->usbhost1_48m_fck_clk);


#ifdef EXTERNAL_PHY_RESET
	/* Refer: ISSUE1 */
	omap_request_gpio(EXT_PHY_RESET_GPIO_PORT1);
	omap_set_gpio_direction(EXT_PHY_RESET_GPIO_PORT1, 0);
	omap_request_gpio(EXT_PHY_RESET_GPIO_PORT2);
	omap_set_gpio_direction(EXT_PHY_RESET_GPIO_PORT2, 0);
	omap_set_gpio_dataout(EXT_PHY_RESET_GPIO_PORT1, 1);
	omap_set_gpio_dataout(EXT_PHY_RESET_GPIO_PORT2, 1);
	/* Hold the PHY in RESET for enough time till DIR is high */
	udelay(EXT_PHY_RESET_DELAY);
#endif

	if (ehci_pdata && ehci_pdata->usb_enable_pin ) {
		omap_set_gpio_dataout(ehci_pdata->usb_enable_pin, 0);
		udelay(EXT_PHY_RESET_DELAY);
	}

	/* Configure TLL for 60Mhz clk for ULPI */
	ehci_clocks->usbtll_fck_clk = clk_get(&dev->dev, USBHOST_TLL_FCLK);
	if (IS_ERR(ehci_clocks->usbtll_fck_clk))
		return PTR_ERR(ehci_clocks->usbtll_fck_clk);
	clk_enable(ehci_clocks->usbtll_fck_clk);

	ehci_clocks->usbtll_ick_clk = clk_get(&dev->dev, USBHOST_TLL_ICKL);
	if (IS_ERR(ehci_clocks->usbtll_ick_clk))
		return PTR_ERR(ehci_clocks->usbtll_ick_clk);
	clk_enable(ehci_clocks->usbtll_ick_clk);

	/* Disable Auto Idle of USBTLL */
	cm_write_mod_reg((0 << OMAP3430_AUTO_USBTLL_SHIFT),
				CORE_MOD, OMAP3430_CM_AUTOIDLE3_CORE);

	/* Wait for TLL to be Active */
	while ((cm_read_mod_reg(CORE_MOD, OMAP3430_CM_IDLEST3_CORE) &
		(1 << OMAP3430_ST_USBTLL_SHIFT)));

#ifdef CONFIG_OMAP_USBHOST_SAR_SUPPORT
	/* Automatic context SAR */
	val = prm_read_mod_reg(CORE_MOD, PM_PWSTCTRL)
				| OMAP3430_SAVEANDRESTORE;
	prm_write_mod_reg(val, CORE_MOD, PM_PWSTCTRL);
#endif	
	/* perform TLL soft reset, and wait until reset is complete */
	omap_writel(1 << OMAP_USBTLL_SYSCONFIG_SOFTRESET_SHIFT,
			OMAP_USBTLL_SYSCONFIG);
	/* Wait for TLL reset to complete */
	while (!(omap_readl(OMAP_USBTLL_SYSSTATUS) &
		(1 << OMAP_USBTLL_SYSSTATUS_RESETDONE_SHIFT)));

	dev_dbg(hcd->self.controller, "\n TLL RESET DONE\n");

	/* Smart Idle mode */
	omap_writel((1 << OMAP_USBTLL_SYSCONFIG_ENAWAKEUP_SHIFT) |
			(2 << OMAP_USBTLL_SYSCONFIG_SIDLEMODE_SHIFT) |
			(0 << OMAP_USBTLL_SYSCONFIG_CACTIVITY_SHIFT) |
			(1 << OMAP_USBTLL_SYSCONFIG_AUTOIDLE_SHIFT),
			OMAP_USBTLL_SYSCONFIG);

	/* Put UHH in SmartIdle/SmartStandby mode */
	omap_writel((1 << OMAP_UHH_SYSCONFIG_AUTOIDLE_SHIFT) |
			(1 << OMAP_UHH_SYSCONFIG_ENAWAKEUP_SHIFT) |
			(2 << OMAP_UHH_SYSCONFIG_SIDLEMODE_SHIFT) |
			(0 << OMAP_UHH_SYSCONFIG_CACTIVITY_SHIFT) |
			(2 << OMAP_UHH_SYSCONFIG_MIDLEMODE_SHIFT),
			OMAP_UHH_SYSCONFIG);

#ifdef CONFIG_OMAP_EHCI_PHY_MODE
	/* Bypass the TLL module for PHY mode operation */
	omap_writel((0 << OMAP_UHH_HOSTCONFIG_ULPI_BYPASS_SHIFT)|
			(1<<OMAP_UHH_HOSTCONFIG_INCR4_BURST_EN_SHIFT)|
			(1<<OMAP_UHH_HOSTCONFIG_INCR8_BURST_EN_SHIFT)|
			(1<<OMAP_UHH_HOSTCONFIG_INCR16_BURST_EN_SHIFT)|
			(0<<OMAP_UHH_HOSTCONFIG_INCRX_ALIGN_EN_SHIFT),
			OMAP_UHH_HOSTCONFIG);
	/* Ensure that BYPASS is set */
	while (omap_readl(OMAP_UHH_HOSTCONFIG) &
		(1 << OMAP_UHH_HOSTCONFIG_ULPI_BYPASS_SHIFT));

	dev_dbg(hcd->self.controller, "Entered ULPI PHY MODE: success");
	printk("\n UHH_HOSTCONFIG = %x\n", omap_readl(OMAP_UHH_HOSTCONFIG));

#else
	/* Enable UTMI mode for all 3 TLL channels */
	omap_usb_utmi_init(hcd,
		OMAP_TLL_CHANNEL_1_EN_MASK |
		OMAP_TLL_CHANNEL_2_EN_MASK |
		OMAP_TLL_CHANNEL_3_EN_MASK
		);
#endif

#ifdef EXTERNAL_PHY_RESET
	/* Refer ISSUE1:
	 * Hold the PHY in RESET for enough time till PHY is settled and ready
	 */
	udelay(EXT_PHY_RESET_DELAY);
	omap_set_gpio_dataout(EXT_PHY_RESET_GPIO_PORT1, 0);
	omap_set_gpio_dataout(EXT_PHY_RESET_GPIO_PORT2, 0);
#endif

	if (ehci_pdata && ehci_pdata->usb_enable_pin) {
		udelay(ehci_pdata->usb_enable_delay);
		omap_set_gpio_dataout(ehci_pdata->usb_enable_pin, 1);
	}

	return 0;
}

/*-------------------------------------------------------------------------*/

static void omap_stop_ehc(struct platform_device *dev, struct usb_hcd *hcd)
{
	struct ehci_platform_data *ehci_pdata = dev->dev.platform_data;
	struct ehci_omap_clock_defs *ehci_clocks;

	ehci_clocks = (struct ehci_omap_clock_defs *)
			(((char *)hcd_to_ehci(hcd)) + sizeof(struct ehci_hcd));

	dev_dbg(hcd->self.controller, ": stopping TI EHCI USB Controller\n");

	/* Reset OMAP modules for insmod/rmmod to work */
	omap_writel( (1<<1), OMAP_UHH_SYSCONFIG);
	while( !(omap_readl(OMAP_UHH_SYSSTATUS) & (1<<0)) );
	while( !(omap_readl(OMAP_UHH_SYSSTATUS) & (1<<1)) );
	while( !(omap_readl(OMAP_UHH_SYSSTATUS) & (1<<2)) );
	dev_dbg(hcd->self.controller,
		"UHH RESET DONE OMAP_UHH_SYSSTATUS %x !!\n",
			omap_readl(OMAP_UHH_SYSSTATUS));

	omap_writel( (1<<1), OMAP_USBTLL_SYSCONFIG);
	while( !(omap_readl(OMAP_USBTLL_SYSSTATUS) & (1<<0)) );
	dev_dbg(hcd->self.controller, ":TLL RESEET DONE");

	if (ehci_clocks->usbtll_fck_clk != NULL) {
		clk_disable(ehci_clocks->usbtll_fck_clk);
		clk_put(ehci_clocks->usbtll_fck_clk);
		ehci_clocks->usbtll_fck_clk = NULL;
	}

	if (ehci_clocks->usbhost_ick_clk != NULL) {
		clk_disable(ehci_clocks->usbhost_ick_clk);
		clk_put(ehci_clocks->usbhost_ick_clk);
		ehci_clocks->usbhost_ick_clk = NULL;
	}

	if (ehci_clocks->usbhost1_48m_fck_clk != NULL) {
		clk_disable(ehci_clocks->usbhost1_48m_fck_clk);
		clk_put(ehci_clocks->usbhost1_48m_fck_clk);
		ehci_clocks->usbhost1_48m_fck_clk = NULL;
	}

	if (ehci_clocks->usbhost2_120m_fck_clk != NULL) {
		clk_disable(ehci_clocks->usbhost2_120m_fck_clk);
		clk_put(ehci_clocks->usbhost2_120m_fck_clk);
		ehci_clocks->usbhost2_120m_fck_clk = NULL;
	}

	if (ehci_clocks->usbtll_ick_clk != NULL) {
		clk_disable(ehci_clocks->usbtll_ick_clk);
		clk_put(ehci_clocks->usbtll_ick_clk);
		ehci_clocks->usbtll_ick_clk = NULL;
	}


#ifdef EXTERNAL_PHY_RESET
	omap_free_gpio(EXT_PHY_RESET_GPIO_PORT1);
	omap_free_gpio(EXT_PHY_RESET_GPIO_PORT2);
#endif

	if (ehci_pdata && ehci_pdata->usb_enable_pin ) {
		omap_set_gpio_dataout(ehci_pdata->usb_enable_pin, 0);
	}

	dev_dbg(hcd->self.controller,
		": Clock to USB host has been disabled\n");
}

static const struct hc_driver ehci_omap_hc_driver;

/*-------------------------------------------------------------------------*/

/* configure so an HC device and id are always provided */
/* always called with process context; sleeping is OK */

/**
 * ehci_hcd_omap_drv_probe - initialize TI-based HCDs
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller, and
 * then invokes the start() method for the HCD associated with it
 * through the hotplug entry's driver_data.
 *
 */
static int ehci_hcd_omap_drv_probe(struct platform_device *dev)
{
	int retval=0;
	struct usb_hcd *hcd;
	struct ehci_hcd *ehci;

	dev_dbg(&dev->dev, "ehci_hcd_omap_drv_probe()");

	if (usb_disabled())
		return -ENODEV;

	/* Port level support */
	/* /sys/devices/platform/ehci-omap.0/portn */
	retval = device_create_file(&dev->dev, &dev_attr_port1);
	retval = device_create_file(&dev->dev, &dev_attr_port2);
#if defined(CONFIG_OMAP_EHCI_TLL_MODE)
	retval = device_create_file(&dev->dev, &dev_attr_port3);
#endif
	if (dev->resource[1].flags != IORESOURCE_IRQ) {
		dev_dbg(&dev->dev, "resource[1] is not IORESOURCE_IRQ");
		retval = -ENOMEM;
	}

	ghcd = hcd = usb_create_hcd(&ehci_omap_hc_driver, &dev->dev, dev->dev.bus_id);
	if (!hcd)
		return -ENOMEM;

	retval = omap_start_ehc(dev, hcd);
	if (retval)
		return retval;

	hcd->rsrc_start = 0;
	hcd->rsrc_len = 0;
	hcd->rsrc_start = dev->resource[0].start;
	hcd->rsrc_len = dev->resource[0].end - dev->resource[0].start + 1;

	hcd->regs = (void __iomem *) (int) IO_ADDRESS(hcd->rsrc_start);

	ehci = hcd_to_ehci(hcd);
	ehci->caps = hcd->regs;

	ehci->regs = hcd->regs + HC_LENGTH(readl(&ehci->caps->hc_capbase));
	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = readl(&ehci->caps->hcs_params);

	/* SET 1 micro-frame Interrupt interval */
	writel (readl (&ehci->regs->command) | (1<<16), &ehci->regs->command);

	retval = usb_add_hcd(hcd, dev->resource[1].start,
				IRQF_DISABLED | IRQF_SHARED);
	if (retval == 0)
		return retval;

	dev_dbg(hcd->self.controller, "ERR: add_hcd");
	omap_stop_ehc(dev, hcd);

	usb_put_hcd(hcd);
	return retval;
}

/*-------------------------------------------------------------------------*/

/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * ehci_hcd_omap_drv_remove - shutdown processing for EHCI HCDs
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_ehci_hcd_omap_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
static int ehci_hcd_omap_drv_remove(struct platform_device *dev)
{
	struct usb_hcd *hcd = platform_get_drvdata(dev);
        struct ehci_omap_clock_defs *ehci_clocks;

        ehci_clocks = (struct ehci_omap_clock_defs *)
                        (((char *)hcd_to_ehci(hcd)) + sizeof(struct ehci_hcd));

	dev_dbg(&dev->dev, "ehci_hcd_omap_drv_remove()\n");

       if (ehci_clocks->suspended) {
		clk_enable(ehci_clocks->usbhost_ick_clk);
		clk_enable(ehci_clocks->usbtll_ick_clk);
                /* Enable All F-clks now before accessing controller */
                clk_enable(ehci_clocks->usbtll_fck_clk);
                clk_enable(ehci_clocks->usbhost1_48m_fck_clk);
                clk_enable(ehci_clocks->usbhost2_120m_fck_clk);
                ehci_clocks->suspended = 0;
        }

	/* Remove Debug support */
	device_remove_file(&dev->dev, &dev_attr_port1);
	device_remove_file(&dev->dev, &dev_attr_port2);
#if defined(CONFIG_OMAP_EHCI_TLL_MODE)
	device_remove_file(&dev->dev, &dev_attr_port3);
#endif
	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);
	omap_stop_ehc(dev, hcd);

	return 0;
}

/*-------------------------------------------------------------------------*/
#ifdef CONFIG_PM
static int omap_ehci_bus_suspend(struct usb_hcd *hcd)
{
        struct ehci_omap_clock_defs *ehci_clocks;
	int ret = 0;

        ehci_clocks = (struct ehci_omap_clock_defs *)
                        (((char *)hcd_to_ehci(hcd)) + sizeof(struct ehci_hcd));

	if (!ehci_clocks->suspended)
	{
		ret = ehci_bus_suspend(hcd);
		/* Write OHCI.HCCONTROL.HCFS = Suspend */
		omap_writel(3<<6, 0x48064404);
		mdelay(5); /* MSTANDBY assertion is delayed by ~5ms */
		/* Ports suspended: Stop All Clks */
		clk_disable(ehci_clocks->usbhost_ick_clk);
		clk_disable(ehci_clocks->usbtll_ick_clk);
		clk_disable(ehci_clocks->usbtll_fck_clk);
		clk_disable(ehci_clocks->usbhost1_48m_fck_clk);
		clk_disable(ehci_clocks->usbhost2_120m_fck_clk);
		ehci_clocks->suspended = 1;
	}

	return ret;
}

static int omap_ehci_bus_resume(struct usb_hcd *hcd)
{
	struct ehci_omap_clock_defs *ehci_clocks;
	int ret = 0;

        ehci_clocks = (struct ehci_omap_clock_defs *)
                        (((char *)hcd_to_ehci(hcd)) + sizeof(struct ehci_hcd));

	if (ehci_clocks->suspended) {
		/* Enable clks before accessing the controller */
		clk_enable(ehci_clocks->usbhost_ick_clk);
		clk_enable(ehci_clocks->usbtll_ick_clk);
        	clk_enable(ehci_clocks->usbtll_fck_clk);
        	clk_enable(ehci_clocks->usbhost1_48m_fck_clk);
        	clk_enable(ehci_clocks->usbhost2_120m_fck_clk);
		ehci_clocks->suspended = 0;
		/* Wakeup ports by resume */
		ret = ehci_bus_resume(hcd);
	}

	return ret;
}

static int ehci_hcd_omap_drv_suspend(struct platform_device *pdev, pm_message_t level)
{
	int rc = ehci_hcd_omap_drv_remove(pdev);
	/* Important!!! waiting for suspend */
	msleep(50);
	return rc;
}

static int ehci_hcd_omap_drv_resume(struct platform_device *pdev)
{
	return ehci_hcd_omap_drv_probe(pdev);
}
#endif
/*-------------------------------------------------------------------------*/

static const struct hc_driver ehci_omap_hc_driver = {
	.description = hcd_name,
	.product_desc = "OMAP-EHCI Host Controller",
	.hcd_priv_size = sizeof(struct ehci_hcd)
				+ sizeof(struct ehci_omap_clock_defs),

	/*
	 * generic hardware linkage
	 */
	.irq = ehci_irq,
	.flags = HCD_MEMORY | HCD_USB2,

	/*
	 * basic lifecycle operations
	 */
	.reset = ehci_init,
	.start = ehci_run,
	.stop = ehci_stop,
	.shutdown = ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue = ehci_urb_enqueue,
	.urb_dequeue = ehci_urb_dequeue,
	.endpoint_disable = ehci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number = ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data = ehci_hub_status_data,
	.hub_control = ehci_hub_control,
#ifdef	CONFIG_PM
	.bus_suspend = ehci_bus_suspend,
	.bus_resume = ehci_bus_resume,
#endif
};

/*-------------------------------------------------------------------------*/
MODULE_ALIAS("omap-ehci");
static struct platform_driver ehci_hcd_omap_driver = {
	.probe = ehci_hcd_omap_drv_probe,
	.remove = ehci_hcd_omap_drv_remove,
	.shutdown = usb_hcd_platform_shutdown,
	.suspend      = ehci_hcd_omap_drv_suspend,
	.resume       = ehci_hcd_omap_drv_resume,
	.driver = {
		.name = "ehci-omap",
		.bus = &platform_bus_type
	}
};
