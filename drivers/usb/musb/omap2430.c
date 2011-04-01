/*
 * Copyright (C) 2005-2006 by Texas Instruments
 *
 * This file is part of the Inventra Controller Driver for Linux.
 *
 * The Inventra Controller Driver for Linux is free software; you
 * can redistribute it and/or modify it under the terms of the GNU
 * General Public License version 2 as published by the Free Software
 * Foundation.
 *
 * The Inventra Controller Driver for Linux is distributed in
 * the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with The Inventra Controller Driver for Linux ; if not,
 * write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/clk.h>

#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/arch/hardware.h>
#include <asm/arch/mux.h>
#include <asm/arch/gpio.h>

#include "musbdefs.h"
#include "omap2430.h"
#include "smsc.h"
#include <asm/arch/battery.h>

#ifdef CONFIG_ARCH_OMAP3430
#define	get_cpu_rev()	2
#endif

#if defined(CONFIG_TWL4030_USB)
extern void twl4030_phy_suspend(int controller_off);
extern void twl4030_phy_resume(void);
#else
#define twl4030_phy_suspend(x)	/* not defined */
#define twl4030_phy_resume()	/* not defined */
#endif


void musb_platform_enable(struct musb *musb)
{
}
void musb_platform_disable(struct musb *musb)
{
}
static void omap_vbus_power(struct musb *musb, int is_on, int sleeping)
{
}

static void omap_set_vbus(struct musb *musb, int is_on)
{
	u8		devctl;
	/* HDRC controls CPEN, but beware current surges during device
	 * connect.  They can trigger transient overcurrent conditions
	 * that must be ignored.
	 */

	devctl = musb_readb(musb->pRegs, MGC_O_HDRC_DEVCTL);

	if (is_on) {
		musb->is_active = 1;
		musb->xceiv.default_a = 1;
		musb->xceiv.state = OTG_STATE_A_WAIT_VRISE;
		devctl |= MGC_M_DEVCTL_SESSION;

		MUSB_HST_MODE(musb);
	} else {
		musb->is_active = 0;

		/* NOTE:  we're skipping A_WAIT_VFALL -> A_IDLE and
		 * jumping right to B_IDLE...
		 */

		musb->xceiv.default_a = 0;
		musb->xceiv.state = OTG_STATE_B_IDLE;
		devctl &= ~MGC_M_DEVCTL_SESSION;

		MUSB_DEV_MODE(musb);
	}
	musb_writeb(musb->pRegs, MGC_O_HDRC_DEVCTL, devctl);

	DBG(1, "VBUS %s, devctl %02x "
		/* otg %3x conf %08x prcm %08x */ "\n",
		otg_state_string(musb),
		musb_readb(musb->pRegs, MGC_O_HDRC_DEVCTL));
}
static int omap_set_power(struct otg_transceiver *x, unsigned mA)
{
	return 0;
}

static int smsc_read(struct musb *musb, int addr)
{
	int reg = 0;

//	printk(KERN_ERR "SMSC read  addr %08X ", addr);

	musb_writeb(musb->pRegs, MGC_O_HDRC_REGADDR, addr);
	musb_writeb(musb->pRegs, MGC_O_HDRC_REGCONTROL, MGC_M_REGCONTROL_REGREQ | MGC_M_REGCONTROL_REGRDNWR);

	while(! (musb_readb(musb->pRegs, MGC_O_HDRC_REGCONTROL) & MGC_M_REGCONTROL_REGCMPLT) ){
		udelay(2);
//		printk(".");
	}

	reg = musb_readb(musb->pRegs, MGC_O_HDRC_REGDATA);

//	printk("%02X\n", reg);

	musb_writeb(musb->pRegs, MGC_O_HDRC_REGCONTROL, 0);

	return reg;
}

static void smsc_write(struct musb *musb, int addr, u8 value)
{
// 	printk(KERN_ERR "SMSC write addr %08X ", addr);

	musb_writeb(musb->pRegs, MGC_O_HDRC_REGADDR, addr);
	musb_writeb(musb->pRegs, MGC_O_HDRC_REGDATA, value);
	musb_writeb(musb->pRegs, MGC_O_HDRC_REGCONTROL, MGC_M_REGCONTROL_REGREQ);

	while(! (musb_readb(musb->pRegs, MGC_O_HDRC_REGCONTROL) & MGC_M_REGCONTROL_REGCMPLT) ){
		udelay(2);
//		printk(".");
	}

// 	printk("%02X\n", value);

	musb_writeb(musb->pRegs, MGC_O_HDRC_REGCONTROL, 0);
}


int musb_platform_resume(struct musb *musb);

extern int archosg6_set_usb_id( int enable );
extern int archosg6_enable_musb( int enable );

static void platform_test_charger( struct musb *musb )
{
	int reg = 0;

	printk( KERN_DEBUG "Detecting host type...\n");
	reg  = smsc_read(musb, SMSC_FUNCTIONCTRL);
	reg |= SMSC_FUNCTIONCTRL_OPMODE_L;
	reg &= ~SMSC_FUNCTIONCTRL_OPMODE_H;	
	smsc_write(musb, SMSC_FUNCTIONCTRL, reg);
	reg = smsc_read(musb, SMSC_POWERIO);
	reg |= SMSC_POWERIO_PULLUPDP;
	smsc_write(musb, SMSC_POWERIO, reg);

	msleep(100);

	reg = smsc_read(musb, SMSC_DEBUG);

	if( (reg & SMSC_DEBUG_LINESTATE0) && (reg & SMSC_DEBUG_LINESTATE1) ){
		// D+ and D- shorted
		printk( KERN_DEBUG "Single-Ended 1 --> Charger\n");
#ifdef CONFIG_MACH_ARCHOS_G6
		archos_usb_high_charge( 1 );
#endif
	} else {
		// Normal case
		printk( KERN_DEBUG "Single-Ended 0 --> Host or other\n");
	}

	// restore good register values.
	reg  = smsc_read(musb, SMSC_POWERIO);
	reg &= ~SMSC_POWERIO_PULLUPDP;
	reg &= ~SMSC_POWERIO_PULLUPDM;
	smsc_write(musb, SMSC_POWERIO, reg);

	reg  = smsc_read(musb, SMSC_FUNCTIONCTRL);
	reg &= ~SMSC_FUNCTIONCTRL_OPMODE_L;
	reg &= ~SMSC_FUNCTIONCTRL_OPMODE_H;	
	smsc_write(musb, SMSC_FUNCTIONCTRL, reg);
}

int __init musb_platform_init(struct musb *musb)
{
	int reg = 0;

#ifdef CONFIG_MACH_ARCHOS_G6
	// set the id pin.
	if( is_host_enabled(musb) ){
		pr_debug("host mode, id is grounded\n");
		//archosg6_set_usb_id( 1 );
	} else {
		pr_debug("device mode, id is high impedance\n");
		archosg6_set_usb_id( 0 );
	}

	// Archos S.A.
	// we have to reset the PHY else switching host/device on the fly does not work.
	archosg6_enable_musb( 0 );
	udelay(10);
	archosg6_enable_musb( 1 );
	udelay(10);

#endif
#if defined(CONFIG_ARCH_OMAP2430)
	omap_cfg_reg(AE5_2430_USB0HS_STP);
	/* get the clock */
	musb->clock = clk_get((struct device *)musb->controller, "usbhs_ick");
	if(IS_ERR(musb->clock))
		return PTR_ERR(musb->clock);
#endif

	musb->asleep = 1;
	clk_enable(musb->clock);
	musb_platform_resume(musb);

	OTG_SYSCONFIG_REG |= ENABLEWAKEUP;
	OTG_INTERFSEL_REG |= ULPI_12PIN;

	/* Archos S.A.
	 * wait some time to allow the controller to initialize
	 * the PHY before we mess with its internals
	 */
	msleep(10);

	// Archos S.A.
	// disable the vbus comparators, we do not want an error if there is a glitch on vbus
	reg = smsc_read(musb, SMSC_INTENFALLING);
	reg &= ~SMSC_INTENFALLING_VBUSVALID;
	smsc_write(musb, SMSC_INTENFALLING, reg);

	if( is_host_enabled(musb) ){
		pr_debug("Ground ID inside the phy.\n");
		reg = smsc_read(musb, SMSC_CARKITCTRL);
		reg |= SMSC_CARKITCTRL_IDGND;
		smsc_write(musb, SMSC_CARKITCTRL, reg);
	} else {
		pr_debug("Put ID high impedance.\n");
		reg = smsc_read(musb, SMSC_CARKITCTRL);
		reg &= ~SMSC_CARKITCTRL_IDGND;
		smsc_write(musb, SMSC_CARKITCTRL, reg);

		platform_test_charger(musb);
	}
	
	pr_debug("HS USB OTG: revision 0x%x, sysconfig 0x%02x, "
			"sysstatus 0x%x, intrfsel 0x%x, simenable  0x%x\n",
			OTG_REVISION_REG, OTG_SYSCONFIG_REG, OTG_SYSSTATUS_REG,
			OTG_INTERFSEL_REG, OTG_SIMENABLE_REG);

	omap_vbus_power(musb, musb->board_mode == MUSB_HOST, 1);

	if (is_host_enabled(musb))
		musb->board_set_vbus = omap_set_vbus;
	if (is_peripheral_enabled(musb))
		musb->xceiv.set_power = omap_set_power;

	return 0;
}

int musb_platform_suspend(struct musb *musb)
{
	DBG(1, "\n3430-suspend()\n");

	if(!musb->asleep) {
		OTG_FORCESTDBY_REG &= ~ENABLEFORCE; /* disable MSTANDBY */
		OTG_SYSCONFIG_REG &= FORCESTDBY;	/* enable force standby */
		OTG_SYSCONFIG_REG &= ~AUTOIDLE;		/* disable auto idle */
		OTG_SYSCONFIG_REG |= FORCEIDLE;		/* enable force idle */
		OTG_FORCESTDBY_REG |= ENABLEFORCE; /* enable MSTANDBY */
		OTG_SYSCONFIG_REG |= AUTOIDLE;		/* enable auto idle */
#if 0
#if defined(CONFIG_OMAP34XX_OFFMODE) && !defined(CONFIG_USB_MUSB_HDRC_MODULE)
		/* Do nothing */
#else
		twl4030_phy_suspend(1);
#endif
#endif
		musb->asleep = 1;
	}

	return 0;
}

int musb_platform_resume(struct musb *musb)
{
	DBG(1, "\n3430-resume()\n");
#if 0
#if defined(CONFIG_OMAP34XX_OFFMODE) && !defined(CONFIG_USB_MUSB_HDRC_MODULE)
	/* Do nothing */
#else
	twl4030_phy_resume();
#endif
#endif

	OTG_FORCESTDBY_REG &= ~ENABLEFORCE; /* disable MSTANDBY */
	OTG_SYSCONFIG_REG |= SMARTSTDBY;	/* enable smart standby */
	OTG_SYSCONFIG_REG &= ~AUTOIDLE;		/* disable auto idle */
	OTG_SYSCONFIG_REG |= SMARTIDLE;		/* enable smart idle */
	OTG_SYSCONFIG_REG |= AUTOIDLE;		/* enable auto idle */

	musb->asleep = 0;

	return 0;
}

int musb_platform_exit(struct musb *musb)
{

	omap_vbus_power(musb, 0 /*off*/, 1);

	musb_platform_suspend(musb);

	/* make sure to stop all timers */
	del_timer_sync( &musb_otg_timer );
	
#ifdef CONFIG_MACH_ARCHOS_G6
	/* turn off USB PHY */
	archosg6_enable_musb( 0 );
#endif
	OTG_SYSCONFIG_REG &= ~ENABLEWAKEUP; /* Disable Wakeup */

	clk_disable(musb->clock);
	return 0;
}

#if defined(CONFIG_OMAP34XX_OFFMODE) && !defined(CONFIG_USB_MUSB_HDRC_MODULE)
typedef struct {
	/* common registers */
	u8 	faddr;
	u8	power;
	u16 	intrtx;
	u16 	intrrx;
	u16 	intrtxe;
	u16 	intrrxe;
	u8 	intrusbe;
	u8 	intrusb;
	u8	devctl;
} musb_common_regs_t;

typedef struct {
	/* Fifo registers */
	u8	rxfifosz;
	u8	txfifosz;
	u16	txfifoadd;
	u16	rxfifoadd;
} musb_index_regs_t;

typedef struct {
	/* MUSB init context */
	musb_common_regs_t	common_regs;
	musb_index_regs_t 	index_regs[MUSB_C_NUM_EPS];
} musb_context_t;


/* Global: MUSB Context data pointer */
musb_context_t *context_ptr = NULL;
struct musb *musb_ptr = NULL;
bool do_cold_plugging = 0;

/* Context Save/Restore for OFF mode */
int musb_context_store_and_suspend( struct musb *musb, int overwrite )
{
	u8 i;
	DBG(1, "\nMUSB-Context-SAVE (Off mode support)\n");

#if defined(CONFIG_USB_MUSB_HDRC_MODULE)
	/* Keep system suspended and wakeup through T2 pres INT */
	musb_platform_suspend(musb);
	return 0;
#endif

	/* Save MUSB Context only once */
	if (!context_ptr || overwrite) {
		if (!context_ptr) {
			context_ptr = kzalloc(sizeof(musb_context_t), GFP_KERNEL);
			if (!context_ptr)
				return -ENOMEM;
		}

		/* Save musb ptr */
		musb_ptr = musb;

		/* Save Common registers */
		context_ptr->common_regs.faddr = musb_readb(musb_ptr->pRegs, MGC_O_HDRC_FADDR);
		context_ptr->common_regs.power = musb_readb(musb_ptr->pRegs, MGC_O_HDRC_POWER);
		context_ptr->common_regs.intrtx = musb_readw(musb_ptr->pRegs, MGC_O_HDRC_INTRTX);
		context_ptr->common_regs.intrrx = musb_readw(musb_ptr->pRegs, MGC_O_HDRC_INTRRX);
		context_ptr->common_regs.intrtxe = musb_readw(musb_ptr->pRegs, MGC_O_HDRC_INTRTXE);
		context_ptr->common_regs.intrrxe = musb_readw(musb_ptr->pRegs, MGC_O_HDRC_INTRRXE);
		context_ptr->common_regs.intrusbe = musb_readb(musb_ptr->pRegs, MGC_O_HDRC_INTRUSBE);
		context_ptr->common_regs.intrusb = musb_readb(musb_ptr->pRegs, MGC_O_HDRC_INTRUSB);
		context_ptr->common_regs.devctl = musb_readb(musb_ptr->pRegs, MGC_O_HDRC_DEVCTL);

		DBG(4, "MUSB ContextStore: Common regs: \nfaddr(%x) \npower(%x) \ninttx(%x) \nintrx(%x)"
			" \ninttxe(%x) \nintrxe(%x) \nintusbe(%x) \nintusb(%x) \ndevctl(%x)\n",
				context_ptr->common_regs.faddr,
				context_ptr->common_regs.power,
				context_ptr->common_regs.intrtx,
				context_ptr->common_regs.intrrx,
				context_ptr->common_regs.intrtxe,
				context_ptr->common_regs.intrrxe,
				context_ptr->common_regs.intrusbe,
				context_ptr->common_regs.intrusb,
				context_ptr->common_regs.devctl);
	
		/* Save FIFO setup details */
		for (i=0; i<MUSB_C_NUM_EPS; i++){
			musb_writeb(musb_ptr->pRegs, MGC_O_HDRC_INDEX, i);
			context_ptr->index_regs[i].rxfifosz = musb_readb(musb_ptr->pRegs, MGC_O_HDRC_RXFIFOSZ);
			context_ptr->index_regs[i].txfifosz = musb_readb(musb_ptr->pRegs, MGC_O_HDRC_TXFIFOSZ);
			context_ptr->index_regs[i].txfifoadd = musb_readw(musb_ptr->pRegs, MGC_O_HDRC_TXFIFOADD);
			context_ptr->index_regs[i].rxfifoadd = musb_readw(musb_ptr->pRegs, MGC_O_HDRC_RXFIFOADD);

			DBG(4, "\n EP(%d) rxfifosz(%x) txfifosz(%x) txfifoaddr(%x) rxfifoadd(%x)",
					i,
					context_ptr->index_regs[i].rxfifosz,
					context_ptr->index_regs[i].txfifosz,
					context_ptr->index_regs[i].txfifoadd,
					context_ptr->index_regs[i].rxfifoadd);
		}
		DBG(4, "\nMUSB Context: FIFO END\n\n");
	}

	/* Keep system suspended and wakeup through T2 pres INT */
	/* For cold plugging case: do not suspend controller */
	if( do_cold_plugging ) {
		do_cold_plugging = 0; /* For once only */
	}
	else {
		/* Reset the controller and keep in default state on power-up */
		OTG_SYSCONFIG_REG |= SOFTRST;

		/* Keep USB suspended till cable is attached */
		musb_platform_suspend(musb_ptr);
	}

	return 0;
}
EXPORT_SYMBOL(musb_context_store_and_suspend);

void musb_context_restore_and_wakeup( void )
{
	u8 i;

	/* For Module no need to restore context */
#ifdef CONFIG_USB_MUSB_HDRC_MODULE
	//Ensure that I-CLK is on after OFF mode
	musb_platform_resume(musb_ptr);
	return;
#endif

	if (!context_ptr) {
		/* T2 called us as a device was found connected */
		/* Its cold plugging, so remember the state when MUSB is up */
		do_cold_plugging = 1;
		return;
	}
	
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
	if ((is_otg_enabled(musb_ptr) || is_peripheral_enabled(musb_ptr)) &&
			(musb_ptr->pGadgetDriver == NULL)) {
		/* The gadget driver is not yet loaded. Treat this as a case
		 * of cold_plugging
		 */
		do_cold_plugging = 1;
		return;
	}
#endif

	DBG(1, "\nMUSB Restore Context: Start\n");

	/* Ensure that I-CLK is on after OFF mode */
	musb_platform_resume(musb_ptr);

	/* Set System specific registers 
	 * Init controller
	 */
	OTG_SYSCONFIG_REG |= ENABLEWAKEUP;
	OTG_INTERFSEL_REG |= ULPI_12PIN;

	/* Restore MUSB specific registers */

	/* Restoer: MUSB Common regs */
	musb_writeb(musb_ptr->pRegs, MGC_O_HDRC_FADDR, context_ptr->common_regs.faddr);
	musb_writeb(musb_ptr->pRegs, MGC_O_HDRC_POWER, context_ptr->common_regs.power);
	musb_writew(musb_ptr->pRegs, MGC_O_HDRC_INTRTX, context_ptr->common_regs.intrtx);
	musb_writew(musb_ptr->pRegs, MGC_O_HDRC_INTRRX, context_ptr->common_regs.intrrx);
	musb_writew(musb_ptr->pRegs, MGC_O_HDRC_INTRTXE, context_ptr->common_regs.intrtxe);
	musb_writew(musb_ptr->pRegs, MGC_O_HDRC_INTRRXE, context_ptr->common_regs.intrrxe);
	musb_writeb(musb_ptr->pRegs, MGC_O_HDRC_INTRUSBE, context_ptr->common_regs.intrusbe);
	musb_writeb(musb_ptr->pRegs, MGC_O_HDRC_INTRUSB, context_ptr->common_regs.intrusb);
	musb_writeb(musb_ptr->pRegs, MGC_O_HDRC_DEVCTL, context_ptr->common_regs.devctl);

	DBG(4, "MUSB ContextStore: Common regs: \nfaddr(%x) \npower(%x) \ninttx(%x) \nintrx(%x)"
			" \ninttxe(%x) \nintrxe(%x) \nintusbe(%x) \nintusb(%x) \ndevctl(%x)\n",
				context_ptr->common_regs.faddr,
				context_ptr->common_regs.power,
				context_ptr->common_regs.intrtx,
				context_ptr->common_regs.intrrx,
				context_ptr->common_regs.intrtxe,
				context_ptr->common_regs.intrrxe,
				context_ptr->common_regs.intrusbe,
				context_ptr->common_regs.intrusb,
				context_ptr->common_regs.devctl);

        /* Restore: FIFO setup details */
	for (i=0; i<MUSB_C_NUM_EPS; i++){
		musb_writeb(musb_ptr->pRegs, MGC_O_HDRC_INDEX, i);
		musb_writeb(musb_ptr->pRegs, MGC_O_HDRC_RXFIFOSZ, context_ptr->index_regs[i].rxfifosz);
		musb_writeb(musb_ptr->pRegs, MGC_O_HDRC_TXFIFOSZ, context_ptr->index_regs[i].txfifosz);
		musb_writew(musb_ptr->pRegs, MGC_O_HDRC_TXFIFOADD, context_ptr->index_regs[i].txfifoadd);
		musb_writew(musb_ptr->pRegs, MGC_O_HDRC_RXFIFOADD, context_ptr->index_regs[i].rxfifoadd);
		DBG(4, "\n EP(%d) rxfifosz(%x) txfifosz(%x) txfifoaddr(%x) rxfifoadd(%x)",
					i,
					context_ptr->index_regs[i].rxfifosz,
					context_ptr->index_regs[i].txfifosz,
					context_ptr->index_regs[i].txfifoadd,
					context_ptr->index_regs[i].rxfifoadd);
	}

	DBG(1, "\n\tMUSB Restore Context: Done\n");
	return;
}
EXPORT_SYMBOL(musb_context_restore_and_wakeup);
#endif /*#if defined(CONFIG_OMAP34XX_OFFMODE)  && !defined(CONFIG_USB_MUSB_HDRC_MODULE) */
