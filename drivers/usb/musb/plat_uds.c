/*****************************************************************
 * Copyright 2005 Mentor Graphics Corporation
 * Copyright (C) 2005-2006 by Texas Instruments
 * Copyright (C) 2006 by Nokia Corporation
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
 * ANY DOWNLOAD, USE, REPRODUCTION, MODIFICATION OR DISTRIBUTION
 * OF THIS DRIVER INDICATES YOUR COMPLETE AND UNCONDITIONAL ACCEPTANCE
 * OF THOSE TERMS.THIS DRIVER IS PROVIDED "AS IS" AND MENTOR GRAPHICS
 * MAKES NO WARRANTIES, EXPRESS OR IMPLIED, RELATED TO THIS DRIVER.
 * MENTOR GRAPHICS SPECIFICALLY DISCLAIMS ALL IMPLIED WARRANTIES
 * OF MERCHANTABILITY; FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT.  MENTOR GRAPHICS DOES NOT PROVIDE SUPPORT
 * SERVICES OR UPDATES FOR THIS DRIVER, EVEN IF YOU ARE A MENTOR
 * GRAPHICS SUPPORT CUSTOMER.
 ******************************************************************/

/*
 * Inventra (Multipoint) Dual-Role Controller Driver for Linux.
 *
 * This consists of a Host Controller Driver (HCD) and a peripheral
 * controller driver implementing the "Gadget" API; OTG support is
 * in the works.  These are normal Linux-USB controller drivers which
 * use IRQs and have no dedicated thread.
 *
 * This version of the driver has only been used with products from
 * Texas Instruments.  Those products integrate the Inventra logic
 * with other DMA, IRQ, and bus modules, as well as other logic that
 * needs to be reflected in this driver.
 *
 *
 * NOTE:  the original Mentor code here was pretty much a collection
 * of mechanisms that don't seem to have been fully integrated/working
 * for *any* Linux kernel version.  This version aims at Linux 2.6.now,
 * Key open issues include:
 *
 *  - Lack of host-side transaction scheduling, for all transfer types.
 *    The hardware doesn't do it; instead, software must.
 *
 *    This is not an issue for OTG devices that don't support external
 *    hubs, but for more "normal" USB hosts it's a user issue that the
 *    "multipoint" support doesn't scale in the expected ways.  That
 *    includes DaVinci EVM in a common non-OTG mode.
 *
 *      * Control and bulk use dedicated endpoints, and there's as
 *        yet no mechanism to either (a) reclaim the hardware when
 *        peripherals are NAKing, which gets complicated with bulk
 *        endpoints, or (b) use more than a single bulk endpoint in
 *        each direction.
 *
 *        RESULT:  one device may be perceived as blocking another one.
 *
 *      * Interrupt and isochronous will dynamically allocate endpoint
 *        hardware, but (a) there's no record keeping for bandwidth;
 *        (b) in the common case that few endpoints are available, there
 *        is no mechanism to reuse endpoints to talk to multiple devices.
 *
 *        RESULT:  At one extreme, bandwidth can be overcommitted in
 *        some hardware configurations, no faults will be reported.
 *        At the other extreme, the bandwidth capabilities which do
 *        exist tend to be severely undercommitted.  You can't yet hook
 *        up both a keyboard and a mouse to an external USB hub.
 */

/*
 * This gets many kinds of configuration information:
 *	- Kconfig for everything user-configurable
 *	- <asm/arch/hdrc_cnf.h> for SOC or family details
 *	- platform_device for addressing, irq, and platform_data
 *	- platform_data is mostly for board-specific informarion
 *
 * Most of the conditional compilation will (someday) vanish.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>

#include <asm/io.h>

#ifdef	CONFIG_ARM
#include <asm/arch/hardware.h>
#include <asm/arch/memory.h>
#include <asm/mach-types.h>
#include <asm/arch/gpio.h>
#endif

#include "musbdefs.h"


#ifdef CONFIG_ARCH_DAVINCI
#include "davinci.h"
#endif



#if MUSB_DEBUG > 0
unsigned debug = MUSB_DEBUG;
module_param(debug, uint, 0);
MODULE_PARM_DESC(debug, "initial debug message level");

#define MUSB_VERSION_SUFFIX	"/dbg"
#else
//Deviation from Open Source - 
//musb_procfs.c has the definition for now.
//we need the procfs interface even in non-debug mode
//const char *otg_state_string(struct musb *musb)
//{
//	static char buf[8];
//
//	snprintf(buf, sizeof buf, "otg-%d", musb->xceiv.state);
//	return buf;
//}
#endif

#define DRIVER_AUTHOR "Mentor Graphics, Texas Instruments, Nokia"
#define DRIVER_DESC "Inventra Dual-Role USB Controller Driver"

#define MUSB_VERSION_BASE "2.2a/db-0.5.2"

#ifndef MUSB_VERSION_SUFFIX
#define MUSB_VERSION_SUFFIX	""
#endif
#define MUSB_VERSION	MUSB_VERSION_BASE MUSB_VERSION_SUFFIX

#define DRIVER_INFO DRIVER_DESC ", v" MUSB_VERSION

const char musb_driver_name[] = "musb_hdrc";

MODULE_DESCRIPTION(DRIVER_INFO);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");


/*-------------------------------------------------------------------------*/

static inline struct musb *dev_to_musb(struct device *dev)
{
#ifdef CONFIG_USB_MUSB_HDRC_HCD
        if( mode_default == MUSB_HOST ){
	       return hcd_to_musb(dev_get_drvdata(dev));
        } else {
		return dev_get_drvdata(dev);
        } 
#else
        return dev_get_drvdata(dev);
#endif
}

/*-------------------------------------------------------------------------*/

#ifndef CONFIG_USB_TUSB6010
/*
 * Load an endpoint's FIFO
 */
void musb_write_fifo(struct musb_hw_ep *hw_ep, u16 wCount, const u8 *pSource)
{
	void __iomem *fifo = hw_ep->fifo;

	prefetch((u8 *)pSource);

	DBG(4, "%cX ep%d fifo %p count %d buf %p\n",
			'T', hw_ep->bLocalEnd, fifo, wCount, pSource);

	/* we can't assume unaligned reads work */
	if (likely((0x01 & (unsigned long) pSource) == 0)) {
		u16	index = 0;

		/* best case is 32bit-aligned source address */
		if ((0x02 & (unsigned long) pSource) == 0) {
			if (wCount >= 4) {
				writesl(fifo, pSource + index, wCount >> 2);
				index += wCount & ~0x03;
			}
			if (wCount & 0x02) {
				musb_writew(fifo, 0, *(u16*)&pSource[index]);
				index += 2;
			}
		} else {
			if (wCount >= 2) {
				writesw(fifo, pSource + index, wCount >> 1);
				index += wCount & ~0x01;
			}
		}
		if (wCount & 0x01)
			musb_writeb(fifo, 0, pSource[index]);
	} else  {
		/* byte aligned */
		writesb(fifo, pSource, wCount);
	}
}

/*
 * Unload an endpoint's FIFO
 */
void musb_read_fifo(struct musb_hw_ep *hw_ep, u16 wCount, u8 *pDest)
{
	void __iomem *fifo = hw_ep->fifo;

	DBG(4, "%cX ep%d fifo %p count %d buf %p\n",
			'R', hw_ep->bLocalEnd, fifo, wCount, pDest);

	/* we can't assume unaligned writes work */
	if (likely((0x01 & (unsigned long) pDest) == 0)) {
		u16	index = 0;

		/* best case is 32bit-aligned destination address */
		if ((0x02 & (unsigned long) pDest) == 0) {
			if (wCount >= 4) {
				readsl(fifo, pDest, wCount >> 2);
				index = wCount & ~0x03;
			}
			if (wCount & 0x02) {
				*(u16*)&pDest[index] = musb_readw(fifo, 0);
				index += 2;
			}
		} else {
			if (wCount >= 2) {
				readsw(fifo, pDest, wCount >> 1);
				index = wCount & ~0x01;
			}
		}
		if (wCount & 0x01)
			pDest[index] = musb_readb(fifo, 0);
	} else  {
		/* byte aligned */
		readsb(fifo, pDest, wCount);
	}
}

#endif	/* normal PIO */


/*-------------------------------------------------------------------------*/
void musb_load_testpacket(struct musb *musb)
{
	void __iomem	*regs = musb->aLocalEnd[0].regs;

	MGC_SelectEnd(musb->pRegs, 0);
	musb_write_fifo(musb->control_ep,
			sizeof(musb_test_packet), musb_test_packet);
	musb_writew(regs, MGC_O_HDRC_CSR0, MGC_M_CSR0_TXPKTRDY);
}

/*-------------------------------------------------------------------------*/

//#ifdef	CONFIG_USB_MUSB_OTG
/*
 * See also USB_OTG_1-3.pdf 6.6.5 Timers
 * REVISIT: Are the other timers done in the hardware?
 */
#define TB_ASE0_BRST		100	/* Min 3.125 ms */

/*
 * Handles OTG hnp timeouts, such as b_ase0_brst
 */
void musb_otg_timer_func(unsigned long data)
{
	struct musb	*musb = (struct musb *)data;
	unsigned long	flags;
	u8	power;
	u8	devctl;

	devctl = musb_readb(musb->pRegs, MGC_O_HDRC_DEVCTL);

	spin_lock_irqsave(&musb->Lock, flags);
	switch (musb->xceiv.state) {

	case OTG_STATE_B_PERIPHERAL:
		/* Clear RESUME if it is set for REMOTE-WAKEUP/SRP*/
		if( MGC_M_POWER_RESUME & (power = musb_readb(musb->pRegs, MGC_O_HDRC_POWER)) ){
			musb_writeb(musb->pRegs, MGC_O_HDRC_POWER, power & ~MGC_M_POWER_RESUME);
		}
                break;

	case OTG_STATE_B_WAIT_ACON:
		DBG(1, "HNP: B_WAIT_ACON timeout, going back to B_PERIPHERAL\n");
		musb_g_disconnect(musb);
		musb->xceiv.state = OTG_STATE_B_PERIPHERAL;
		musb->is_active = 0;
		break;

	case OTG_STATE_A_WAIT_BCON:
		del_timer(&musb_otg_timer);

		INFO("No response from B-device\n");
		// FIXME: this caused musb as host only to fail on device disconnect
#if 0
		devctl &= ~MGC_M_DEVCTL_SESSION;
		musb_writeb(musb->pRegs, MGC_O_HDRC_DEVCTL, devctl);

		devctl = musb_readb(musb->pRegs, MGC_O_HDRC_DEVCTL);
		if (devctl & MGC_M_DEVCTL_BDEVICE) {
			musb->xceiv.state = OTG_STATE_B_IDLE;
			MUSB_DEV_MODE(musb);
		}
		else {
			musb->xceiv.state = OTG_STATE_A_IDLE;
			MUSB_HST_MODE(musb);
		}
#endif
#if 0
#if defined(CONFIG_OMAP34XX_OFFMODE) && !defined(CONFIG_USB_MUSB_HDRC_MODULE)
		/* Keep MUSB suspended on Cable Detach */
		musb_platform_suspend(musb);
#endif
#endif
                break;

#ifdef CONFIG_USB_MUSB_HDRC_HCD
	case OTG_STATE_A_SUSPEND:

		if( is_host_enabled(musb) ){

			/* finish RESUME signaling? */
			if (musb->port1_status & MUSB_PORT_STAT_RESUME){
				power = musb_readb(musb->pRegs, MGC_O_HDRC_POWER);
				power &= ~MGC_M_POWER_RESUME;
				DBG(1, "root port resume stopped, power %02x\n",
										power);
				musb_writeb(musb->pRegs, MGC_O_HDRC_POWER, power);
				musb->is_active = 1;
				musb->port1_status &= ~(USB_PORT_STAT_SUSPEND
							| MUSB_PORT_STAT_RESUME);
				musb->port1_status |= USB_PORT_STAT_C_SUSPEND << 16;
				usb_hcd_poll_rh_status(musb_to_hcd(musb));
				/* NOTE: it might really be A_WAIT_BCON ... */
				musb->xceiv.state = OTG_STATE_A_HOST;
			}
			break;
		}
#endif


	case OTG_STATE_A_WAIT_VFALL:
#if 0
                /* Wait till VBUS falls below SessionEnd (~0.2V); the 1.3 RTL
                 * seems to mis-handle session "start" otherwise (or in our
                 * case "recover"), in routine "VBUS was valid by the time
                 * VBUSERR got reported during enumeration" cases.
                 */
                if (devctl & MGC_M_DEVCTL_VBUS) {
                        mod_timer(&otg_timer, jiffies + POLL_SECONDS * HZ);
                        break;
                }
                musb->xceiv.state = OTG_STATE_A_WAIT_VRISE;
                musb_writel(musb->ctrl_base, DAVINCI_USB_INT_SET_REG,
                        MGC_M_INTR_VBUSERROR << DAVINCI_USB_USBINT_SHIFT);
#endif
		break;

	case OTG_STATE_B_IDLE:
#if 0
                if (!is_peripheral_enabled(musb))
                        break;
                /* There's no ID-changed IRQ, so we have no good way to tell
                 * when to switch to the A-Default state machine (by setting
                 * the DEVCTL.SESSION flag).
                 * Workaround:  whenever we're in B_IDLE, try setting the
                 * session flag every few seconds.  If it works, ID was
                 * grounded and we're now in the A-Default state machine.
                 * NOTE setting the session flag is _supposed_ to trigger
                 * SRP, but clearly it doesn't.
                 */
                musb_writeb(mregs, MGC_O_HDRC_DEVCTL,
                                devctl | MGC_M_DEVCTL_SESSION);
                devctl = musb_readb(musb->pRegs, MGC_O_HDRC_DEVCTL);
                if (devctl & MGC_M_DEVCTL_BDEVICE)
                        mod_timer(&otg_timer, jiffies + POLL_SECONDS * HZ);
                else
                        musb->xceiv.state = OTG_STATE_A_IDLE;
#endif
		break;
	default:
		break;
	}
	spin_unlock_irqrestore(&musb->Lock, flags);

	return;
}

DEFINE_TIMER(musb_otg_timer, musb_otg_timer_func, 0, 0);

/*
 * Stops the B-device HNP state. Caller must take care of locking.
 */
void musb_hnp_stop(struct musb *musb)
{
	struct usb_hcd	*hcd = musb_to_hcd(musb);
	void __iomem	*pBase = musb->pRegs;
	u8	reg;

	switch (musb->xceiv.state) {
	case OTG_STATE_A_PERIPHERAL:
	case OTG_STATE_A_WAIT_VFALL:
		DBG(1, "HNP: Switching back to A-host\n");
		musb_g_disconnect(musb);
#ifdef CONFIG_USB_MUSB_HDRC_HCD
		if( is_host_enabled(musb) )
			musb_root_disconnect(musb);
#endif
		musb->xceiv.state = OTG_STATE_A_IDLE;
		musb->is_active = 0;
		break;
	case OTG_STATE_B_HOST:
		DBG(1, "HNP: Disabling HR\n");
#ifdef CONFIG_USB_MUSB_HDRC_HCD
		if( is_host_enabled(musb) )
			musb_root_disconnect(musb);
#endif
		hcd->self.is_b_host = 0;
		musb->xceiv.state = OTG_STATE_B_PERIPHERAL;
		reg = musb_readb(pBase, MGC_O_HDRC_POWER);
		reg |= MGC_M_POWER_SUSPENDM;
		musb_writeb(pBase, MGC_O_HDRC_POWER, reg);
		/* REVISIT: Start SESSION_REQUEST here? */
		break;
	default:
		DBG(1, "HNP: Stopping in unknown state %s\n",
			otg_state_string(musb));
	}
}

//#endif

/*
 * Interrupt Service Routine to record USB "global" interrupts.
 * Since these do not happen often and signify things of
 * paramount importance, it seems OK to check them individually;
 * the order of the tests is specified in the manual
 *
 * @param pThis instance pointer
 * @param bIntrUSB register contents
 * @param devctl
 * @param power
 */

#define STAGE0_MASK (MGC_M_INTR_RESUME | MGC_M_INTR_SESSREQ \
		| MGC_M_INTR_VBUSERROR | MGC_M_INTR_CONNECT \
		| MGC_M_INTR_RESET )

static irqreturn_t musb_stage0_irq(struct musb * pThis, u8 bIntrUSB,
				u8 devctl, u8 power)
{
	irqreturn_t handled = IRQ_NONE;
//#ifdef CONFIG_USB_MUSB_HDRC_HCD
	void __iomem *pBase = pThis->pRegs;
//#endif

	DBG(3, "<== Power=%02x, DevCtl=%02x, bIntrUSB=0x%x\n", power, devctl,
		bIntrUSB);

	/* in host mode, the peripheral may issue remote wakeup.
	 * in peripheral mode, the host may resume the link.
	 * spurious RESUME irqs happen too, paired with SUSPEND.
	 */
	if (bIntrUSB & MGC_M_INTR_RESUME) {
		handled = IRQ_HANDLED;
		DBG(3, "RESUME (%s)\n", otg_state_string(pThis));

		if (devctl & MGC_M_DEVCTL_HM) {
#ifdef CONFIG_USB_MUSB_HDRC_HCD
			if( is_host_enabled(pThis) ){
				switch (pThis->xceiv.state) {
				case OTG_STATE_A_SUSPEND:
					/* remote wakeup?  later, GetPortStatus
					* will stop RESUME signaling
					*/
					if (power & MGC_M_POWER_RESUME) {
						power &= ~MGC_M_POWER_SUSPENDM;
						musb_writeb(pBase, MGC_O_HDRC_POWER,
							power | MGC_M_POWER_RESUME);
	
						pThis->port1_status |=
							(USB_PORT_STAT_C_SUSPEND << 16)
							| MUSB_PORT_STAT_RESUME;
	
						/* Start RESUME for 50ms for OPT to pass
						*/
						musb_otg_timer.data = (unsigned long)pThis;
						mod_timer( &musb_otg_timer, jiffies +
							msecs_to_jiffies(50));
#if 0
						pThis->rh_timer = jiffies
							+ msecs_to_jiffies(20);
	
						pThis->xceiv.state = OTG_STATE_A_HOST;
						pThis->is_active = 1;
						usb_hcd_resume_root_hub(
								musb_to_hcd(pThis));
#endif
	
					} else if (power & MGC_M_POWER_SUSPENDM) {
						/* spurious */
						pThis->int_usb &= ~MGC_M_INTR_SUSPEND;
					}
					break;
				case OTG_STATE_B_WAIT_ACON:
					pThis->xceiv.state = OTG_STATE_B_PERIPHERAL;
					pThis->is_active = 1;
					MUSB_DEV_MODE(pThis);
					break;
				default:
					WARN("bogus %s RESUME (%s)\n",
						"host",
						otg_state_string(pThis));
				}
			}
#endif
		} else {
			switch (pThis->xceiv.state) {
#ifdef CONFIG_USB_MUSB_HDRC_HCD
			case OTG_STATE_A_SUSPEND:
				if( is_host_enabled(pThis) ){
					/* possibly DISCONNECT is upcoming */
					pThis->xceiv.state = OTG_STATE_A_HOST;
					usb_hcd_resume_root_hub(musb_to_hcd(pThis));
					break;
				}
#endif
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
			case OTG_STATE_B_WAIT_ACON:
			case OTG_STATE_B_PERIPHERAL:
				if (!is_peripheral_enabled(pThis)){
					WARN("bogus %s RESUME (%s)\n",
					"peripheral",
					otg_state_string(pThis));
					break;
				}

				/* disconnect while suspended?  we may
				 * not get a disconnect irq...
				 */
				if ((devctl & MGC_M_DEVCTL_VBUS)
						!= (3 << MGC_S_DEVCTL_VBUS)) {
					pThis->int_usb |= MGC_M_INTR_DISCONNECT;
					pThis->int_usb &= ~MGC_M_INTR_SUSPEND;
					break;
				}
				musb_g_resume(pThis);
				break;
			case OTG_STATE_B_IDLE:
				if (!is_peripheral_enabled(pThis)){
					WARN("bogus %s RESUME (%s)\n",
					"peripheral",
					otg_state_string(pThis));
					break;
				}

				pThis->int_usb &= ~MGC_M_INTR_SUSPEND;
				break;
#endif
			default:
				WARN("bogus %s RESUME (%s)\n",
					"peripheral",
					otg_state_string(pThis));
			}
		}
	}

#ifdef CONFIG_USB_MUSB_HDRC_HCD
	/* see manual for the order of the tests */
	if( is_host_enabled(pThis) ){
		if (bIntrUSB & MGC_M_INTR_SESSREQ) {
			DBG(1, "SESSION_REQUEST (%s)\n", otg_state_string(pThis));
	
			/* IRQ arrives from ID pin sense or (later, if VBUS power
			* is removed) SRP.  responses are time critical:
			*  - turn on VBUS (with silicon-specific mechanism)
			*  - go through A_WAIT_VRISE
			*  - ... to A_WAIT_BCON.
			* a_wait_vrise_tmout triggers VBUS_ERROR transitions
			*/
			musb_writeb(pBase, MGC_O_HDRC_DEVCTL, MGC_M_DEVCTL_SESSION);
			pThis->bEnd0Stage = MGC_END0_START;
			pThis->xceiv.state = OTG_STATE_A_IDLE;
			MUSB_HST_MODE(pThis);
			musb_set_vbus(pThis, 1);
	
			handled = IRQ_HANDLED;
		}
	
		if (bIntrUSB & MGC_M_INTR_VBUSERROR) {
			int	ignore = 0;
	
			/* During connection as an A-Device, we may see a short
			* current spikes causing voltage drop, because of cable
			* and peripheral capacitance combined with vbus draw.
			* (So: less common with truly self-powered devices, where
			* vbus doesn't act like a power supply.)
			*
			* Such spikes are short; usually less than ~500 usec, max
			* of ~2 msec.  That is, they're not sustained overcurrent
			* errors, though they're reported using VBUSERROR irqs.
			*
			* Workarounds:  (a) hardware: use self powered devices.
			* (b) software:  ignore non-repeated VBUS errors.
			*
			* REVISIT:  do delays from lots of DEBUG_KERNEL checks
			* make trouble here, keeping VBUS < 4.4V ?
			*/
			switch (pThis->xceiv.state) {
			case OTG_STATE_B_IDLE:
					DBG(1, "OTG STATE: B_idle\n");
			case OTG_STATE_A_IDLE:
					DBG(1, "OTG STATE: A_idle\n");
			case OTG_STATE_A_HOST:
				/* recovery is dicey once we've gotten past the
				* initial stages of enumeration, but if VBUS
				* stayed ok at the other end of the link, and
				* another reset is due (at least for high speed,
				* to redo the chirp etc), it might work OK...
				*/
			case OTG_STATE_A_WAIT_BCON:
			case OTG_STATE_A_WAIT_VRISE:
				if (pThis->vbuserr_retry) {
					pThis->vbuserr_retry--;
					ignore = 1;
					devctl |= MGC_M_DEVCTL_SESSION;
					musb_writeb(pBase, MGC_O_HDRC_DEVCTL, devctl);
				} else {
					pThis->port1_status |=
						(1 << USB_PORT_FEAT_OVER_CURRENT)
						| (1 << USB_PORT_FEAT_C_OVER_CURRENT);
				}
				break;
			default:
				break;
			}
	
			DBG(1, "VBUS_ERROR in %s (%02x, %s), retry #%d, port1 %08x\n",
					otg_state_string(pThis),
					devctl,
					({ char *s;
					switch (devctl & MGC_M_DEVCTL_VBUS) {
					case 0 << MGC_S_DEVCTL_VBUS:
						s = "<SessEnd"; break;
					case 1 << MGC_S_DEVCTL_VBUS:
						s = "<AValid"; break;
					case 2 << MGC_S_DEVCTL_VBUS:
						s = "<VBusValid"; break;
					//case 3 << MGC_S_DEVCTL_VBUS:
					default:
						s = "VALID"; break;
					}; s; }),
					VBUSERR_RETRY_COUNT - pThis->vbuserr_retry,
					pThis->port1_status);
	
			/* go through A_WAIT_VFALL then start a new session */
			if (!ignore)
				musb_set_vbus(pThis, 0);
			handled = IRQ_HANDLED;
		}
	
		if (bIntrUSB & MGC_M_INTR_CONNECT) {
			struct usb_hcd *hcd = musb_to_hcd(pThis);
	
			handled = IRQ_HANDLED;
			pThis->is_active = 1;
			set_bit(HCD_FLAG_SAW_IRQ, &hcd->flags);
	
			pThis->bEnd0Stage = MGC_END0_START;
	
	#ifdef CONFIG_USB_MUSB_OTG
			/* flush endpoints when transitioning from Device Mode */
			if (is_peripheral_active(pThis)) {
				// REVISIT HNP; just force disconnect
			}
			pThis->bDelayPortPowerOff = FALSE;
			musb_writew(pBase, MGC_O_HDRC_INTRTXE, pThis->wEndMask);
			musb_writew(pBase, MGC_O_HDRC_INTRRXE, pThis->wEndMask & 0xfffe);
			musb_writeb(pBase, MGC_O_HDRC_INTRUSBE, 0xf7);
	#endif
			pThis->port1_status &= ~(USB_PORT_STAT_LOW_SPEED
						|USB_PORT_STAT_HIGH_SPEED
						|USB_PORT_STAT_ENABLE
						);
			pThis->port1_status |= USB_PORT_STAT_CONNECTION
						|(USB_PORT_STAT_C_CONNECTION << 16);
	
			/* high vs full speed is just a guess until after reset */
			if (devctl & MGC_M_DEVCTL_LSDEV)
				pThis->port1_status |= USB_PORT_STAT_LOW_SPEED;
	
			if (hcd->status_urb)
				usb_hcd_poll_rh_status(hcd);
			else
				usb_hcd_resume_root_hub(hcd);
	
			MUSB_HST_MODE(pThis);
	
			/* indicate new connection to OTG machine */
			switch (pThis->xceiv.state) {
			case OTG_STATE_B_WAIT_ACON:
				DBG(1, "HNP: Waiting to switch to b_host state\n");
				pThis->xceiv.state = OTG_STATE_B_HOST;
				hcd->self.is_b_host = 1;
				break;
			default:
				if ((devctl & MGC_M_DEVCTL_VBUS)
						== (3 << MGC_S_DEVCTL_VBUS)) {
					pThis->xceiv.state = OTG_STATE_A_HOST;
					hcd->self.is_b_host = 0;
				}
				break;
			}
			DBG(1, "CONNECT (%s) devctl %02x\n",
					otg_state_string(pThis), devctl);
		}
	}
#endif	/* CONFIG_USB_MUSB_HDRC_HCD */

	/* mentor saves a bit: bus reset and babble share the same irq.
	 * only host sees babble; only peripheral sees bus reset.
	 */
	if (bIntrUSB & MGC_M_INTR_RESET) {
		if (devctl & MGC_M_DEVCTL_HM) {
			/*
			 * Looks like non-HS BABBLE can be ignored, but
			 * HS BABBLE is an error condition. For HS the solution
			 * is to avoid babble in the first place and fix whatever
			 * causes BABBLE. When HS BABBLE happens we can only stop
			 * the session.
			 */
			if (devctl & (MGC_M_DEVCTL_FSDEV | MGC_M_DEVCTL_LSDEV))
				DBG(1, "BABBLE devctl: %02x\n", devctl);
			else {
				ERR("Stopping host session because of babble\n");
				musb_writeb(pBase, MGC_O_HDRC_DEVCTL, 0);
			}
		} else {
			DBG(1, "BUS RESET\n");

			musb_g_reset(pThis);
			schedule_work(&pThis->irq_work);
		}

		handled = IRQ_HANDLED;
	}

	return handled;
}

/*
 * Interrupt Service Routine to record USB "global" interrupts.
 * Since these do not happen often and signify things of
 * paramount importance, it seems OK to check them individually;
 * the order of the tests is specified in the manual
 *
 * @param pThis instance pointer
 * @param bIntrUSB register contents
 * @param devctl
 * @param power
 */
static irqreturn_t musb_stage2_irq(struct musb * pThis, u8 bIntrUSB,
				u8 devctl, u8 power)
{
	irqreturn_t handled = IRQ_NONE;

#if 0
/* REVISIT ... this would be for multiplexing periodic endpoints, or
 * supporting transfer phasing to prevent exceeding ISO bandwidth
 * limits of a given frame or microframe.
 *
 * It's not needed for peripheral side, which dedicates endpoints;
 * though it _might_ use SOF irqs for other purposes.
 *
 * And it's not currently needed for host side, which also dedicates
 * endpoints, relies on TX/RX interval registers, and isn't claimed
 * to support ISO transfers yet.
 */
	if (bIntrUSB & MGC_M_INTR_SOF) {
		void __iomem *pBase = pThis->pRegs;
		struct musb_hw_ep	*ep;
		u8 bEnd;
		u16 wFrame;

		DBG(6, "START_OF_FRAME\n");
		handled = IRQ_HANDLED;

		/* start any periodic Tx transfers waiting for current frame */
		wFrame = musb_readw(pBase, MGC_O_HDRC_FRAME);
		ep = pThis->aLocalEnd;
		for (bEnd = 1; (bEnd < pThis->bEndCount)
					&& (pThis->wEndMask >= (1 << bEnd));
				bEnd++, ep++) {
			// FIXME handle framecounter wraps (12 bits)
			// eliminate duplicated StartUrb logic
			if (ep->dwWaitFrame >= wFrame) {
				ep->dwWaitFrame = 0;
				printk("SOF --> periodic TX%s on %d\n",
					ep->tx_channel ? " DMA" : "",
					bEnd);
				if (!ep->tx_channel)
					musb_h_tx_start(pThis, bEnd);
				else
					cppi_hostdma_start(pThis, bEnd);
			}
		}		/* end of for loop */
	}
#endif

	if ((bIntrUSB & MGC_M_INTR_DISCONNECT) && !pThis->bIgnoreDisconnect) {
		DBG(1, "DISCONNECT (%s) as %s, devctl %02x\n",
				otg_state_string(pThis),
				MUSB_MODE(pThis), devctl);
		handled = IRQ_HANDLED;

		switch (pThis->xceiv.state) {
#ifdef CONFIG_USB_MUSB_HDRC_HCD
		case OTG_STATE_A_HOST:
			if( is_host_enabled(pThis) ){
				musb_root_disconnect(pThis);
				
				devctl = musb_readb(pThis->pRegs, MGC_O_HDRC_DEVCTL);
				if( (devctl & MGC_M_DEVCTL_BDEVICE) ) {
					pThis->xceiv.state = OTG_STATE_B_IDLE;
#if 0
	#if defined(CONFIG_OMAP34XX_OFFMODE) && !defined(CONFIG_USB_MUSB_HDRC_MODULE)
					/* Keep MUSB suspended on cable detach */
					musb_platform_suspend(pThis); 
	#endif
#endif
				}
				else {
					/*Start Timer*/
					pThis->xceiv.state = OTG_STATE_A_WAIT_BCON;
	
					musb_otg_timer.data = (unsigned long)pThis;
					mod_timer( &musb_otg_timer, jiffies +
						msecs_to_jiffies(MGC_OTG_T_A_WAIT_BCON));
				}
					
				break;
			}
		case OTG_STATE_A_SUSPEND:
			if( is_host_enabled(pThis) ){
				musb_root_disconnect(pThis);
				if (pThis->a_wait_bcon != 0)
					musb_platform_try_idle(pThis, jiffies
						+ msecs_to_jiffies(pThis->a_wait_bcon));
				break;
			}
#endif	/* HOST */
#ifdef CONFIG_USB_MUSB_OTG
		case OTG_STATE_A_WAIT_VFALL:
		case OTG_STATE_B_HOST:
			musb_hnp_stop(pThis);
			break;
			/* FALLTHROUGH */
		case OTG_STATE_A_PERIPHERAL:
			musb_root_disconnect(pThis);
			/* FALLTHROUGH */
		case OTG_STATE_B_WAIT_ACON:
#endif	/* OTG */
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
		case OTG_STATE_B_PERIPHERAL:
			if (is_peripheral_enabled(pThis)){
				musb_g_disconnect(pThis);
#if 0
#if defined(CONFIG_OMAP34XX_OFFMODE) && !defined(CONFIG_USB_MUSB_HDRC_MODULE)
			musb_platform_suspend(pThis);
#endif
#endif
				break;
			}
#endif	/* GADGET */
		default:
			WARN("unhandled DISCONNECT transition (%s)\n",
				otg_state_string(pThis));
			break;
		}

		schedule_work(&pThis->irq_work);
	}

	if (bIntrUSB & MGC_M_INTR_SUSPEND) {
		DBG(1, "SUSPEND (%s) devctl %02x power %02x\n",
				otg_state_string(pThis), devctl, power);
		handled = IRQ_HANDLED;

		switch (pThis->xceiv.state) {
		case OTG_STATE_A_PERIPHERAL:
#ifdef CONFIG_USB_MUSB_OTG
			musb_hnp_stop(pThis);
#endif
			break;
		case OTG_STATE_B_PERIPHERAL:
			musb_g_suspend(pThis);
			pThis->is_active = is_otg_enabled(pThis)
					&& pThis->xceiv.gadget->b_hnp_enable;
			if (pThis->is_active) {
				pThis->xceiv.state = OTG_STATE_B_WAIT_ACON;
#ifdef	CONFIG_USB_MUSB_OTG
				DBG(1, "HNP: Setting timer for b_ase0_brst\n");
				musb_otg_timer.data = (unsigned long)pThis;
				mod_timer(&musb_otg_timer, jiffies
					+ msecs_to_jiffies(TB_ASE0_BRST));
#endif
			}
			break;
		case OTG_STATE_A_WAIT_BCON:
			if (pThis->a_wait_bcon != 0)
				musb_platform_try_idle(pThis, jiffies
					+ msecs_to_jiffies(pThis->a_wait_bcon));
			break;
		case OTG_STATE_A_HOST:
			pThis->xceiv.state = OTG_STATE_A_SUSPEND;
			pThis->is_active = is_otg_enabled(pThis)
					&& pThis->xceiv.host->b_hnp_enable;
			break;
		case OTG_STATE_B_HOST:
			/* Transition to B_PERIPHERAL, see 6.8.2.6 p 44 */
			DBG(1, "REVISIT: SUSPEND as B_HOST\n");
			break;
		default:
			/* "should not happen" */
			pThis->is_active = 0;
			break;
		}
	}


	return handled;
}

/*-------------------------------------------------------------------------*/

/*
* Program the HDRC to start (enable interrupts, dma, etc.).
*/
void musb_start(struct musb *musb)
{
	void __iomem	*regs = musb->pRegs;
	u8		devctl = musb_readb(regs, MGC_O_HDRC_DEVCTL);

	DBG(2, "<== devctl %02x\n", devctl);

	/*  Set INT enable registers, enable interrupts */
	musb_writew(regs, MGC_O_HDRC_INTRTXE, musb->wEndMask);
	musb_writew(regs, MGC_O_HDRC_INTRRXE, musb->wEndMask & 0xfffe);
	musb_writeb(regs, MGC_O_HDRC_INTRUSBE, 0xf7);

	musb_writeb(regs, MGC_O_HDRC_TESTMODE, 0);

	/* put into basic highspeed mode and start session */
	musb_writeb(regs, MGC_O_HDRC_POWER, MGC_M_POWER_ISOUPDATE
						| MGC_M_POWER_SOFTCONN
						| MGC_M_POWER_HSENAB
						/* ENSUSPEND wedges tusb */
						// | MGC_M_POWER_ENSUSPEND
						);

	musb->is_active = 0;
	devctl = musb_readb(regs, MGC_O_HDRC_DEVCTL);
	devctl &= ~MGC_M_DEVCTL_SESSION;

	if (is_otg_enabled(musb)) {
		/* session started after:
		 * (a) ID-grounded irq, host mode;
		 * (b) vbus present/connect IRQ, peripheral mode;
		 * (c) peripheral initiates, using SRP
		 */
		if ((devctl & MGC_M_DEVCTL_VBUS) == MGC_M_DEVCTL_VBUS)
			musb->is_active = 1;
		else
			devctl |= MGC_M_DEVCTL_SESSION;

	} else if (is_host_enabled(musb)) {
		/* assume ID pin is hard-wired to ground */
		devctl |= MGC_M_DEVCTL_SESSION;

	} else /* peripheral is enabled */ {
		if ((devctl & MGC_M_DEVCTL_VBUS) == MGC_M_DEVCTL_VBUS)
			musb->is_active = 1;
	}
	musb_platform_enable(musb);
	musb_writeb(regs, MGC_O_HDRC_DEVCTL, devctl);
}


static void musb_generic_disable(struct musb *pThis)
{
	void __iomem	*pBase = pThis->pRegs;
	u16	temp;

	/* disable interrupts */
	musb_writeb(pBase, MGC_O_HDRC_INTRUSBE, 0);
	musb_writew(pBase, MGC_O_HDRC_INTRTXE, 0);
	musb_writew(pBase, MGC_O_HDRC_INTRRXE, 0);

	/* off */
	musb_writeb(pBase, MGC_O_HDRC_DEVCTL, 0);

	/*  flush pending interrupts */
	temp = musb_readb(pBase, MGC_O_HDRC_INTRUSB);
	temp = musb_readw(pBase, MGC_O_HDRC_INTRTX);
	temp = musb_readw(pBase, MGC_O_HDRC_INTRRX);

}

/*
 * Make the HDRC stop (disable interrupts, etc.);
 * reversible by musb_start
 * called on gadget driver unregister
 * with controller locked, irqs blocked
 * acts as a NOP unless some role activated the hardware
 */
void musb_stop(struct musb *musb)
{
	/* stop IRQs, timers, ... */
	musb_platform_disable(musb);
	musb_generic_disable(musb);
	DBG(3, "HDRC disabled\n");

	/* FIXME
	 *  - mark host and/or peripheral drivers unusable/inactive
	 *  - disable DMA (and enable it in HdrcStart)
	 *  - make sure we can musb_start() after musb_stop(); with
	 *    OTG mode, gadget driver module rmmod/modprobe cycles that
	 *  - ...
	 */
	musb_platform_try_idle(musb, 0);
}

static void musb_shutdown(struct platform_device *pdev)
{
	struct musb	*musb = dev_to_musb(&pdev->dev);
	unsigned long	flags;

	spin_lock_irqsave(&musb->Lock, flags);
	musb_platform_disable(musb);
	musb_generic_disable(musb);
	if (musb->clock) {
		clk_put(musb->clock);
		musb->clock = NULL;
	}
	if (musb->aux_clock) {
		clk_disable(musb->aux_clock);
		clk_put(musb->aux_clock);
		musb->aux_clock = NULL;
	}
	
	spin_unlock_irqrestore(&musb->Lock, flags);

	/* FIXME power down */
}


/*-------------------------------------------------------------------------*/

/*
 * The silicon either has hard-wired endpoint configurations, or else
 * "dynamic fifo" sizing.  The driver has support for both, though at this
 * writing only the dynamic sizing is very well tested.   We use normal
 * idioms to so both modes are compile-tested, but dead code elimination
 * leaves only the relevant one in the object file.
 *
 * We don't currently use dynamic fifo setup capability to do anything
 * more than selecting one of a bunch of predefined configurations.
 */
#ifdef MUSB_C_DYNFIFO_DEF
#define	can_dynfifo()	1
#else
#define	can_dynfifo()	0
#endif

#if defined(CONFIG_USB_TUSB6010) || \
    defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3430)
static ushort __initdata fifo_mode = 4;
#else
static ushort __initdata fifo_mode = 2;
#endif

/* "modprobe ... fifo_mode=1" etc */
module_param(fifo_mode, ushort, 0);
MODULE_PARM_DESC(fifo_mode, "initial endpoint configuration");


/*
 * USB role is a module parameter now. Default is the platform-specific
 * data if no parameter is given
 */
int mode_default = MUSB_UNDEFINED;
module_param (mode_default, int, 0);
MODULE_PARM_DESC(mode_default, "default usb role: 1 - host, 2 - peripheral");

#define DYN_FIFO_SIZE (1<<(MUSB_C_RAM_BITS+2))

enum fifo_style { FIFO_RXTX, FIFO_TX, FIFO_RX } __attribute__ ((packed));
enum buf_mode { BUF_SINGLE, BUF_DOUBLE } __attribute__ ((packed));

struct fifo_cfg {
	u8		hw_ep_num;
	enum fifo_style	style;
	enum buf_mode	mode;
	u16		maxpacket;
};

/*
 * tables defining fifo_mode values.  define more if you like.
 * for host side, make sure both halves of ep1 are set up.
 */

/* mode 0 - fits in 2KB */
static struct fifo_cfg __initdata mode_0_cfg[] = {
{ .hw_ep_num = 1, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num = 1, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num = 2, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num = 2, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num = 3, .style = FIFO_RXTX, .maxpacket = 256, },
{ .hw_ep_num = 4, .style = FIFO_RXTX, .maxpacket = 256, },
};

/* mode 1 - fits in 4KB */
static struct fifo_cfg __initdata mode_1_cfg[] = {
{ .hw_ep_num = 1, .style = FIFO_TX,   .maxpacket = 512, .mode = BUF_DOUBLE, },
{ .hw_ep_num = 1, .style = FIFO_RX,   .maxpacket = 512, .mode = BUF_DOUBLE, },
{ .hw_ep_num = 2, .style = FIFO_RXTX, .maxpacket = 512, .mode = BUF_DOUBLE, },
{ .hw_ep_num = 3, .style = FIFO_RXTX, .maxpacket = 256, },
{ .hw_ep_num = 4, .style = FIFO_RXTX, .maxpacket = 256, },
};

/* mode 2 - fits in 4KB */
static struct fifo_cfg __initdata mode_2_cfg[] = {
{ .hw_ep_num = 1, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num = 1, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num = 2, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num = 2, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num = 3, .style = FIFO_RXTX, .maxpacket = 256, },
{ .hw_ep_num = 4, .style = FIFO_RXTX, .maxpacket = 256, },
};

/* mode 3 - fits in 4KB */
static struct fifo_cfg __initdata mode_3_cfg[] = {
{ .hw_ep_num = 1, .style = FIFO_TX,   .maxpacket = 512, .mode = BUF_DOUBLE, },
{ .hw_ep_num = 1, .style = FIFO_RX,   .maxpacket = 512, .mode = BUF_DOUBLE, },
{ .hw_ep_num = 2, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num = 2, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num = 3, .style = FIFO_RXTX, .maxpacket = 256, },
{ .hw_ep_num = 4, .style = FIFO_RXTX, .maxpacket = 256, },
};

/* mode 4 - fits in 16KB */
static struct fifo_cfg __initdata mode_4_cfg[] = {
{ .hw_ep_num =  1, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  1, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  2, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  2, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  3, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  3, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  4, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  4, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  5, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  5, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  6, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  6, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  7, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  7, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  8, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  8, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num =  9, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num =  9, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num = 10, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num = 10, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num = 11, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num = 11, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num = 12, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num = 12, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num = 13, .style = FIFO_TX,   .maxpacket = 512, },
{ .hw_ep_num = 13, .style = FIFO_RX,   .maxpacket = 512, },
{ .hw_ep_num = 14, .style = FIFO_RXTX, .maxpacket = 1024, },
{ .hw_ep_num = 15, .style = FIFO_RXTX, .maxpacket = 1024, },
};


/*
 * configure a fifo; for non-shared endpoints, this may be called
 * once for a tx fifo and once for an rx fifo.
 *
 * returns negative errno or offset for next fifo.
 */
static int __init
fifo_setup(struct musb *musb, struct musb_hw_ep  *hw_ep,
		const struct fifo_cfg *cfg, u16 offset)
{
	void __iomem	*mbase = musb->pRegs;
	int	size = 0;
	u16	maxpacket = cfg->maxpacket;
	u16	c_off = offset >> 3;
	u8	c_size;

	/* expect hw_ep has already been zero-initialized */

	size = ffs(max(maxpacket, (u16) 8)) - 1;
	maxpacket = 1 << size;

	c_size = size - 3;
	if (cfg->mode == BUF_DOUBLE) {
		if ((offset + (maxpacket << 1)) > DYN_FIFO_SIZE)
			return -EMSGSIZE;
		c_size |= MGC_M_FIFOSZ_DPB;
	} else {
		if ((offset + maxpacket) > DYN_FIFO_SIZE)
			return -EMSGSIZE;
	}

	/* configure the FIFO */
	musb_writeb(mbase, MGC_O_HDRC_INDEX, hw_ep->bLocalEnd);

#ifdef CONFIG_USB_MUSB_HDRC_HCD
	/* EP0 reserved endpoint for control, bidirectional;
	 * EP1 reserved for bulk, two unidirection halves.
	 */
	if( is_host_enabled(musb) ){
		if (hw_ep->bLocalEnd == 1)
			musb->bulk_ep = hw_ep;
	}
	/* REVISIT error check:  be sure ep0 can both rx and tx ... */
#endif
	switch (cfg->style) {
	case FIFO_TX:
		musb_writeb(mbase, MGC_O_HDRC_TXFIFOSZ, c_size);
		musb_writew(mbase, MGC_O_HDRC_TXFIFOADD, c_off);
		hw_ep->tx_double_buffered = !!(c_size & MGC_M_FIFOSZ_DPB);
		hw_ep->wMaxPacketSizeTx = maxpacket;
		break;
	case FIFO_RX:
		musb_writeb(mbase, MGC_O_HDRC_RXFIFOSZ, c_size);
		musb_writew(mbase, MGC_O_HDRC_RXFIFOADD, c_off);
		hw_ep->rx_double_buffered = !!(c_size & MGC_M_FIFOSZ_DPB);
		hw_ep->wMaxPacketSizeRx = maxpacket;
		break;
	case FIFO_RXTX:
		musb_writeb(mbase, MGC_O_HDRC_TXFIFOSZ, c_size);
		musb_writew(mbase, MGC_O_HDRC_TXFIFOADD, c_off);
		hw_ep->rx_double_buffered = !!(c_size & MGC_M_FIFOSZ_DPB);
		hw_ep->wMaxPacketSizeRx = maxpacket;

		musb_writeb(mbase, MGC_O_HDRC_RXFIFOSZ, c_size);
		musb_writew(mbase, MGC_O_HDRC_RXFIFOADD, c_off);
		hw_ep->tx_double_buffered = hw_ep->rx_double_buffered;
		hw_ep->wMaxPacketSizeTx = maxpacket;

		hw_ep->bIsSharedFifo = TRUE;
		break;
	}

	/* NOTE rx and tx endpoint irqs aren't managed separately,
	 * which happens to be ok
	 */
	musb->wEndMask |= (1 << hw_ep->bLocalEnd);

	return offset + (maxpacket << ((c_size & MGC_M_FIFOSZ_DPB) ? 1 : 0));
}

static struct fifo_cfg __initdata ep0_cfg = {
	.style = FIFO_RXTX, .maxpacket = 64,
};

static int __init ep_config_from_table(struct musb *musb)
{
	const struct fifo_cfg	*cfg;
	unsigned		i, n;
	int			offset;
	struct musb_hw_ep	*hw_ep = musb->aLocalEnd;

	switch (fifo_mode) {
	default:
		fifo_mode = 0;
		/* FALLTHROUGH */
	case 0:
		cfg = mode_0_cfg;
		n = ARRAY_SIZE(mode_0_cfg);
		break;
	case 1:
		cfg = mode_1_cfg;
		n = ARRAY_SIZE(mode_1_cfg);
		break;
	case 2:
		cfg = mode_2_cfg;
		n = ARRAY_SIZE(mode_2_cfg);
		break;
	case 3:
		cfg = mode_3_cfg;
		n = ARRAY_SIZE(mode_3_cfg);
		break;
	case 4:
		cfg = mode_4_cfg;
		n = ARRAY_SIZE(mode_4_cfg);
		break;
	}

	pr_debug("%s: setup fifo_mode %d\n",
			musb_driver_name, fifo_mode);


	offset = fifo_setup(musb, hw_ep, &ep0_cfg, 0);
	// assert(offset > 0)

	/* NOTE:  for RTL versions >= 1.400 EPINFO and RAMINFO would
	 * be better than static MUSB_C_NUM_EPS and DYN_FIFO_SIZE...
	 */

	for (i = 0; i < n; i++) {
		u8	epn = cfg->hw_ep_num;

		if (epn >= MUSB_C_NUM_EPS) {
			pr_debug( "%s: invalid ep %d\n",
					musb_driver_name, epn);
			continue;
		}
		offset = fifo_setup(musb, hw_ep + epn, cfg++, offset);
		if (offset < 0) {
			pr_debug( "%s: mem overrun, ep %d\n",
					musb_driver_name, epn);
			return -EINVAL;
		}
		epn++;
		musb->bEndCount = max(epn, musb->bEndCount);
	}

	pr_debug("%s: %d/%d max ep, %d/%d memory\n",
			musb_driver_name,
			n + 1, MUSB_C_NUM_EPS * 2 + 1,
			offset, DYN_FIFO_SIZE);
	
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	if( is_host_enabled(musb) ){
		if (!musb->bulk_ep) {
			pr_debug( "%s: missing bulk\n", musb_driver_name);
			return -EINVAL;
		}
	}
#endif

	return 0;
}


/*
 * ep_config_from_hw - when MUSB_C_DYNFIFO_DEF is false
 * @param pThis the controller
 */
static int __init ep_config_from_hw(struct musb *musb)
{
	u8 bEnd = 0, reg;
	struct musb_hw_ep *pEnd;
	void *pBase = musb->pRegs;

	DBG(2, "<== static silicon ep config\n");

	/* FIXME pick up ep0 maxpacket size */

	for (bEnd = 1; bEnd < MUSB_C_NUM_EPS; bEnd++) {
		MGC_SelectEnd(pBase, bEnd);
		pEnd = musb->aLocalEnd + bEnd;

		/* read from core using indexed model */
		reg = musb_readb(pEnd->regs, 0x10 + MGC_O_HDRC_FIFOSIZE);
		if (!reg) {
			/* 0's returned when no more endpoints */
			break;
		}
		musb->bEndCount++;
		musb->wEndMask |= (1 << bEnd);

		pEnd->wMaxPacketSizeTx = 1 << (reg & 0x0f);

		/* shared TX/RX FIFO? */
		if ((reg & 0xf0) == 0xf0) {
			pEnd->wMaxPacketSizeRx = pEnd->wMaxPacketSizeTx;
			pEnd->bIsSharedFifo = TRUE;
			continue;
		} else {
			pEnd->wMaxPacketSizeRx = 1 << ((reg & 0xf0) >> 4);
			pEnd->bIsSharedFifo = FALSE;
		}

		/* FIXME set up pEnd->{rx,tx}_double_buffered */

#ifdef CONFIG_USB_MUSB_HDRC_HCD
		if( is_host_enabled(musb) ){
			/* pick an RX/TX endpoint for bulk */
			if (pEnd->wMaxPacketSizeTx < 512
					|| pEnd->wMaxPacketSizeRx < 512)
				continue;
	
			/* REVISIT:  this algorithm is lazy, we should at least
			* try to pick a double buffered endpoint.
			*/
			if (musb->bulk_ep)
				continue;
			musb->bulk_ep = pEnd;
		}
#endif
	}

#ifdef CONFIG_USB_MUSB_HDRC_HCD
	if( is_host_enabled(musb) ){
		if (!musb->bulk_ep) {
			pr_debug( "%s: missing bulk\n", musb_driver_name);
			return -EINVAL;
		}
	}
#endif

	return 0;
}

enum { MUSB_CONTROLLER_MHDRC, MUSB_CONTROLLER_HDRC, };

/* Initialize MUSB (M)HDRC part of the USB hardware subsystem;
 * configure endpoints, or take their config from silicon
 */
static int __init musb_core_init(u16 wType, struct musb *pThis)
{
#ifdef MUSB_AHB_ID
	u32 dwData;
#endif
	u8 reg;
	char *type;
	u16 wRelease, wRelMajor, wRelMinor;
	char aInfo[100], aRevision[32], aDate[12];
	void __iomem	*pBase = pThis->pRegs;
	int		status = 0;
	int		i;

	/* log core options (read using indexed model) */
	//MGC_SelectEnd(pBase, 0);
	musb_writeb(pBase, MGC_O_HDRC_INDEX, 0);
	reg = musb_readb(pBase, 0x10 + MGC_O_HDRC_CONFIGDATA);

	strcpy(aInfo, (reg & MGC_M_CONFIGDATA_UTMIDW) ? "UTMI-16" : "UTMI-8");
	if (reg & MGC_M_CONFIGDATA_DYNFIFO) {
		strcat(aInfo, ", dyn FIFOs");
	}
	if (reg & MGC_M_CONFIGDATA_MPRXE) {
		strcat(aInfo, ", bulk combine");
#ifdef C_MP_RX
		pThis->bBulkCombine = TRUE;
#else
		strcat(aInfo, " (X)");		/* no driver support */
#endif
	}
	if (reg & MGC_M_CONFIGDATA_MPTXE) {
		strcat(aInfo, ", bulk split");
#ifdef C_MP_TX
		pThis->bBulkSplit = TRUE;
#else
		strcat(aInfo, " (X)");		/* no driver support */
#endif
	}
	if (reg & MGC_M_CONFIGDATA_HBRXE) {
		strcat(aInfo, ", HB-ISO Rx");
		strcat(aInfo, " (X)");		/* no driver support */
	}
	if (reg & MGC_M_CONFIGDATA_HBTXE) {
		strcat(aInfo, ", HB-ISO Tx");
		strcat(aInfo, " (X)");		/* no driver support */
	}
	if (reg & MGC_M_CONFIGDATA_SOFTCONE) {
		strcat(aInfo, ", SoftConn");
	}

	printk(KERN_DEBUG "%s: ConfigData=0x%02x (%s)\n",
			musb_driver_name, reg, aInfo);

#ifdef MUSB_AHB_ID
	dwData = musb_readl(pBase, 0x404);
	sprintf(aDate, "%04d-%02x-%02x", (dwData & 0xffff),
		(dwData >> 16) & 0xff, (dwData >> 24) & 0xff);
	/* FIXME ID2 and ID3 are unused */
	dwData = musb_readl(pBase, 0x408);
	printk("ID2=%lx\n", (long unsigned)dwData);
	dwData = musb_readl(pBase, 0x40c);
	printk("ID3=%lx\n", (long unsigned)dwData);
	reg = musb_readb(pBase, 0x400);
	wType = ('M' == reg) ? MUSB_CONTROLLER_MHDRC : MUSB_CONTROLLER_HDRC;
#else
	aDate[0] = 0;
#endif
	if (MUSB_CONTROLLER_MHDRC == wType) {
		pThis->bIsMultipoint = 1;
		type = "M";
	} else {
		pThis->bIsMultipoint = 0;
		type = "";
#ifdef CONFIG_USB_MUSB_HDRC_HCD
#ifndef	CONFIG_USB_OTG_BLACKLIST_HUB
		if( is_host_enabled(pThis) ){
			printk(KERN_ERR
				"%s: kernel must blacklist external hubs\n",
				musb_driver_name);
		}
#endif
#endif
	}

	/* log release info */
	wRelease = musb_readw(pBase, MGC_O_HDRC_HWVERS);
	wRelMajor = (wRelease >> 10) & 0x1f;
	wRelMinor = wRelease & 0x3ff;
	snprintf(aRevision, 32, "%d.%d%s", wRelMajor,
		wRelMinor, (wRelease & 0x8000) ? "RC" : "");
	printk(KERN_DEBUG "%s: %sHDRC RTL version %s %s\n",
			musb_driver_name, type, aRevision, aDate);

	/* configure ep0 */
	pThis->aLocalEnd[0].wMaxPacketSizeTx = MGC_END0_FIFOSIZE;
	pThis->aLocalEnd[0].wMaxPacketSizeRx = MGC_END0_FIFOSIZE;

	/* discover endpoint configuration */
	pThis->bEndCount = 1;
	pThis->wEndMask = 1;

	if (reg & MGC_M_CONFIGDATA_DYNFIFO) {
		if (can_dynfifo())
			status = ep_config_from_table(pThis);
		else {
			ERR("reconfigure software for Dynamic FIFOs\n");
			status = -ENODEV;
		}
	} else {
		if (!can_dynfifo())
			status = ep_config_from_hw(pThis);
		else {
			ERR("reconfigure software for static FIFOs\n");
			return -ENODEV;
		}
	}

	if (status < 0)
		return status;

	/* finish init, and print endpoint config */
	for (i = 0; i < pThis->bEndCount; i++) {
		struct musb_hw_ep	*hw_ep = pThis->aLocalEnd + i;

		hw_ep->fifo = MUSB_FIFO_OFFSET(i) + pBase;
#ifdef CONFIG_USB_TUSB6010
		hw_ep->fifo_async = pThis->async + 0x400 + MUSB_FIFO_OFFSET(i);
		hw_ep->fifo_sync = pThis->sync + 0x400 + MUSB_FIFO_OFFSET(i);
		hw_ep->fifo_sync_va =
			pThis->sync_va + 0x400 + MUSB_FIFO_OFFSET(i);

		if (i == 0)
			hw_ep->conf = pBase - 0x400 + TUSB_EP0_CONF;
		else
			hw_ep->conf = pBase + 0x400 + (((i - 1) & 0xf) << 2);
#endif

		hw_ep->regs = MGC_END_OFFSET(i, 0) + pBase;
#ifdef CONFIG_USB_MUSB_HDRC_HCD
		if( is_host_enabled(pThis) ){
			hw_ep->target_regs = MGC_BUSCTL_OFFSET(i, 0) + pBase;
			hw_ep->rx_reinit = 1;
			hw_ep->tx_reinit = 1;
		}
#endif

		if (hw_ep->wMaxPacketSizeTx) {
			pr_debug("%s: hw_ep %d%s, %smax %d\n",
				musb_driver_name, i,
				hw_ep->bIsSharedFifo ? "shared" : "tx",
				hw_ep->tx_double_buffered
					? "doublebuffer, " : "",
				hw_ep->wMaxPacketSizeTx);
		}
		if (hw_ep->wMaxPacketSizeRx && !hw_ep->bIsSharedFifo) {
			pr_debug("%s: hw_ep %d%s, %smax %d\n",
				musb_driver_name, i,
				"rx",
				hw_ep->rx_double_buffered
					? "doublebuffer, " : "",
				hw_ep->wMaxPacketSizeRx);
		}
		if (!(hw_ep->wMaxPacketSizeTx || hw_ep->wMaxPacketSizeRx))
			DBG(1, "hw_ep %d not configured\n", i);
	}

	return 0;
}

/*-------------------------------------------------------------------------*/

#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3430)

static irqreturn_t generic_interrupt(int irq, void *__hci)
{
	unsigned long	flags;
	irqreturn_t	retval = IRQ_NONE;
	struct musb	*musb = __hci;

	spin_lock_irqsave(&musb->Lock, flags);

	musb->int_usb = musb_readb(musb->pRegs, MGC_O_HDRC_INTRUSB);
	musb->int_tx = musb_readw(musb->pRegs, MGC_O_HDRC_INTRTX);
	musb->int_rx = musb_readw(musb->pRegs, MGC_O_HDRC_INTRRX);

	if (musb->int_usb || musb->int_tx || musb->int_rx)
		retval = musb_interrupt(musb);

	spin_unlock_irqrestore(&musb->Lock, flags);

	/* REVISIT we sometimes get spurious IRQs on g_ep0
	 * not clear why...
	 */
	if (retval != IRQ_HANDLED)
		DBG(5, "spurious?\n");

	return IRQ_HANDLED;
}

#else
#define generic_interrupt	NULL
#endif

/*
 * handle all the irqs defined by the HDRC core. for now we expect:  other
 * irq sources (phy, dma, etc) will be handled first, musb->int_* values
 * will be assigned, and the irq will already have been acked.
 *
 * called in irq context with spinlock held, irqs blocked
 */
irqreturn_t musb_interrupt(struct musb *musb)
{
	irqreturn_t	retval = IRQ_NONE;
	u8		devctl, power;
	int		ep_num;
	u32		reg;

	devctl = musb_readb(musb->pRegs, MGC_O_HDRC_DEVCTL);
	power = musb_readb(musb->pRegs, MGC_O_HDRC_POWER);

// 	printk(KERN_DEBUG "MUSB IRQ usb%04x tx%04x rx%04x\n",
// 		musb->int_usb, musb->int_tx, musb->int_rx);

	/* the core can interrupt us for multiple reasons; docs have
	 * a generic interrupt flowchart to follow
	 */
	if (musb->int_usb & STAGE0_MASK)
		retval |= musb_stage0_irq(musb, musb->int_usb,
				devctl, power);

	/* "stage 1" is handling endpoint irqs */

	/* handle endpoint 0 first */
	if (musb->int_tx & 1) {
		if (devctl & MGC_M_DEVCTL_HM)
			retval |= musb_h_ep0_irq(musb);
		else
			retval |= musb_g_ep0_irq(musb);
	}

	/* RX on endpoints 1-15 */
	reg = musb->int_rx >> 1;
	ep_num = 1;
	while (reg) {
		if (reg & 1) {
			// MGC_SelectEnd(musb->pRegs, ep_num);
			/* REVISIT just retval = ep->rx_irq(...) */
			retval = IRQ_HANDLED;
			if (devctl & MGC_M_DEVCTL_HM) {
				if (is_host_capable())
					musb_host_rx(musb, ep_num);
			} else {
				if (is_peripheral_capable())
					musb_g_rx(musb, ep_num);
			}
		}

		reg >>= 1;
		ep_num++;
	}

	/* TX on endpoints 1-15 */
	reg = musb->int_tx >> 1;
	ep_num = 1;
	while (reg) {
		if (reg & 1) {
			// MGC_SelectEnd(musb->pRegs, ep_num);
			/* REVISIT just retval |= ep->tx_irq(...) */
			retval = IRQ_HANDLED;
			if (devctl & MGC_M_DEVCTL_HM) {
				if (is_host_capable())
					musb_host_tx(musb, ep_num);
			} else {
				if (is_peripheral_capable())
					musb_g_tx(musb, ep_num);
			}
		}
		reg >>= 1;
		ep_num++;
	}

	/* finish handling "global" interrupts after handling fifos */
	if (musb->int_usb)
		retval |= musb_stage2_irq(musb,
				musb->int_usb, devctl, power);

	return retval;
}


#ifndef CONFIG_USB_INVENTRA_FIFO
static int __initdata use_dma = 1;

/* "modprobe ... use_dma=0" etc */
module_param(use_dma, bool, 0);
MODULE_PARM_DESC(use_dma, "enable/disable use of DMA");

void musb_dma_completion(struct musb *musb, u8 bLocalEnd, u8 bTransmit)
{
	u8	devctl = musb_readb(musb->pRegs, MGC_O_HDRC_DEVCTL);

	/* called with controller lock already held */

	if (!bLocalEnd) {
#ifndef CONFIG_USB_TUSB_OMAP_DMA
		if (!is_cppi_enabled()) {
			/* endpoint 0 */
			if (devctl & MGC_M_DEVCTL_HM)
				musb_h_ep0_irq(musb);
			else
				musb_g_ep0_irq(musb);
		}
#endif
	} else {
		/* endpoints 1..15 */
		if (bTransmit) {
			if (devctl & MGC_M_DEVCTL_HM) {
				if (is_host_capable())
					musb_host_tx(musb, bLocalEnd);
			} else {
				if (is_peripheral_capable())
					musb_g_tx(musb, bLocalEnd);
			}
		} else {
			/* receive */
			if (devctl & MGC_M_DEVCTL_HM) {
				if (is_host_capable())
					musb_host_rx(musb, bLocalEnd);
			} else {
				if (is_peripheral_capable())
					musb_g_rx(musb, bLocalEnd);
			}
		}
	}
}

#else
#define use_dma			0
#endif

/*-------------------------------------------------------------------------*/

#ifdef CONFIG_SYSFS

static ssize_t
musb_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct musb *musb = dev_to_musb(dev);
	unsigned long flags;
	int ret = -EINVAL;

	spin_lock_irqsave(&musb->Lock, flags);
	ret = sprintf(buf, "%s\n", otg_state_string(musb));
	spin_unlock_irqrestore(&musb->Lock, flags);

	return ret;
}

static ssize_t
musb_mode_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct musb	*musb = dev_to_musb(dev);
	unsigned long	flags;

	spin_lock_irqsave(&musb->Lock, flags);
	if (!strncmp(buf, "host", 4))
		musb_platform_set_mode(musb, MUSB_HOST);
	if (!strncmp(buf, "peripheral", 10))
		musb_platform_set_mode(musb, MUSB_PERIPHERAL);
	if (!strncmp(buf, "otg", 3))
		musb_platform_set_mode(musb, MUSB_OTG);
	spin_unlock_irqrestore(&musb->Lock, flags);

	return n;
}
static DEVICE_ATTR(mode, 0644, musb_mode_show, musb_mode_store);

static ssize_t
musb_cable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct musb *musb = dev_to_musb(dev);
	char *v1= "", *v2 = "?";
	unsigned long flags;
	int vbus;

	spin_lock_irqsave(&musb->Lock, flags);
#if defined(CONFIG_USB_TUSB6010) && !defined(CONFIG_USB_MUSB_OTG)
	/* REVISIT: connect-A != connect-B ... */
	vbus = musb_platform_get_vbus_status(musb);
	if (vbus)
		v2 = "connected";
	else
		v2 = "disconnected";
#else
	/* NOTE: board-specific issues, like too-big capacitors keeping
	 * VBUS high for a long time after power has been removed, can
	 * cause temporary false indications of a connection.
	 */
	vbus = musb_readb(musb->pRegs, MGC_O_HDRC_DEVCTL);
	if (vbus & 0x10) {
		/* REVISIT retest on real OTG hardware */
		switch (musb->board_mode) {
		case MUSB_HOST:
			v2 = "A";
			break;
		case MUSB_PERIPHERAL:
			v2 = "B";
			break;
		case MUSB_OTG:
			v1 = "Mini-";
			v2 = (vbus & MGC_M_DEVCTL_BDEVICE) ? "B" : "A";
			break;
		}
	} else	/* VBUS level below A-Valid */
		v2 = "disconnected";
#endif
	musb_platform_try_idle(musb, 0);
	spin_unlock_irqrestore(&musb->Lock, flags);

	return sprintf(buf, "%s%s\n", v1, v2);
}
static DEVICE_ATTR(cable, S_IRUGO, musb_cable_show, NULL);

static ssize_t
musb_vbus_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct musb	*musb = dev_to_musb(dev);
	unsigned long	flags;
	unsigned long	val;

	spin_lock_irqsave(&musb->Lock, flags);
	if (sscanf(buf, "%lu", &val) < 1) {
		printk(KERN_ERR "Invalid VBUS timeout ms value\n");
		return -EINVAL;
	}
	musb->a_wait_bcon = val;
	if (musb->xceiv.state == OTG_STATE_A_WAIT_BCON)
		musb->is_active = 0;
	musb_platform_try_idle(musb, jiffies + msecs_to_jiffies(val));
	spin_unlock_irqrestore(&musb->Lock, flags);

	return n;
}

static ssize_t
musb_vbus_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct musb	*musb = dev_to_musb(dev);
	unsigned long	flags;
	unsigned long	val;

	spin_lock_irqsave(&musb->Lock, flags);
	val = musb->a_wait_bcon;
	spin_unlock_irqrestore(&musb->Lock, flags);

	return sprintf(buf, "%lu\n", val);
}
static DEVICE_ATTR(vbus, 0644, musb_vbus_show, musb_vbus_store);

#ifdef CONFIG_USB_MUSB_OTG
static ssize_t
musb_srp_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct musb	*musb=dev_to_musb(dev);
	unsigned long	flags;
	unsigned short	srp;

	if (sscanf(buf, "%hu", &srp) != 1
			|| (srp != 1)) {
		printk (KERN_ERR "SRP: Value must be 1\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&musb->Lock, flags);
	if (srp == 1)
		musb_g_wakeup(musb);
	spin_unlock_irqrestore(&musb->Lock, flags);

	return n;
}
static DEVICE_ATTR(srp, 0644, NULL, musb_srp_store);
#endif
#endif

/* Only used to provide cable state change events */
static void musb_irq_work(struct work_struct *data)
{
	struct musb *musb = container_of(data, struct musb, irq_work);

	sysfs_notify(&musb->controller->kobj, NULL, "cable");
}

/* --------------------------------------------------------------------------
 * Init support
 */

static struct musb *__init
allocate_instance(struct device *dev, void __iomem *mbase)
{
	struct musb		*musb;
	struct musb_hw_ep	*ep;
	int			epnum;

#ifdef CONFIG_USB_MUSB_HDRC_HCD
	if( mode_default == MUSB_HOST ){
		struct usb_hcd	*hcd;
	
		hcd = usb_create_hcd(&musb_hc_driver, dev, dev->bus_id);
		if (!hcd)
			return NULL;
		
		/* usbcore sets dev->driver_data to hcd, and sometimes uses that... */
	
		musb = hcd_to_musb(hcd);
		INIT_LIST_HEAD(&musb->control);
		INIT_LIST_HEAD(&musb->in_bulk);
		INIT_LIST_HEAD(&musb->out_bulk);
	
		hcd->uses_new_polling = 1;
			
		musb->vbuserr_retry = VBUSERR_RETRY_COUNT;
		

	} else 
#endif /* CONFIG_USB_MUSB_HDRC_HCD */
	{
		musb = kzalloc(sizeof *musb, GFP_KERNEL);
		if (!musb)
			return NULL;
		dev_set_drvdata(dev, musb);	
	}

	musb->pRegs = mbase;
	musb->ctrl_base = mbase;
	musb->nIrq = -ENODEV;
	for (epnum = 0, ep = musb->aLocalEnd;
			epnum < MUSB_C_NUM_EPS;
			epnum++, ep++) {

		ep->musb = musb;
		ep->bLocalEnd = epnum;
	}

	musb->controller = dev;
	return musb;
}

static void musb_free(struct musb *musb)
{
	/* this has multiple entry modes. it handles fault cleanup after
	 * probe(), where things may be partially set up, as well as rmmod
	 * cleanup after everything's been de-activated.
	 */

#ifdef CONFIG_SYSFS
	device_remove_file(musb->controller, &dev_attr_mode);
	device_remove_file(musb->controller, &dev_attr_cable);
	device_remove_file(musb->controller, &dev_attr_vbus);
#ifdef CONFIG_USB_MUSB_OTG
	device_remove_file(musb->controller, &dev_attr_srp);
#endif
#endif

#ifdef CONFIG_USB_GADGET_MUSB_HDRC
	if (is_peripheral_enabled(musb))
		musb_gadget_cleanup(musb);
#endif

	if (musb->nIrq >= 0) {
		disable_irq_wake(musb->nIrq);
		free_irq(musb->nIrq, musb);
	}
	if (is_dma_capable() && musb->pDmaController) {
		struct dma_controller	*c = musb->pDmaController;

		(void) c->stop(c->pPrivateData);
		dma_controller_destroy(c);
	}

	musb_writeb(musb->pRegs, MGC_O_HDRC_DEVCTL, 0);
	musb_platform_exit(musb);
	musb_writeb(musb->pRegs, MGC_O_HDRC_DEVCTL, 0);

	if( is_host_enabled(musb) ){
		usb_put_hcd(musb_to_hcd(musb));
	} else {
		kfree(musb);
	}
}

#if 0
#if defined(CONFIG_OMAP34XX_OFFMODE)
extern int musb_context_store_and_suspend(struct musb *musb, int overwrite);
#endif
#endif
/*
 * Perform generic per-controller initialization.
 *
 * @pDevice: the controller (already clocked, etc)
 * @nIrq: irq
 * @pRegs: virtual address of controller registers,
 *	not yet corrected for platform-specific offsets
 */
static int __init
musb_init_controller(struct device *dev, int nIrq, void __iomem *ctrl)
{
	int			status;
	struct musb		*pThis;
	struct musb_hdrc_platform_data *plat = dev->platform_data;

	/* The driver might handle more features than the board; OK.
	 * Fail when the board needs a feature that's not enabled.
	 */
	if (!plat) {
		dev_dbg(dev, "no platform_data?\n");
		return -ENODEV;
	}

	if (mode_default == MUSB_UNDEFINED)
		mode_default = plat->mode;

	switch (plat->mode) {
	case MUSB_HOST:
#ifdef CONFIG_USB_MUSB_HDRC_HCD
		break;
#else
		goto bad_config;
#endif
	case MUSB_PERIPHERAL:
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
		break;
#else
		goto bad_config;
#endif
	case MUSB_OTG:
#ifdef CONFIG_USB_MUSB_OTG
		break;
#else
	bad_config:
#endif
	default:
		dev_dbg(dev, "incompatible Kconfig role setting, mode %i unsupported\n", mode_default);
		return -EINVAL;
	}

	/* allocate */
	pThis = allocate_instance(dev, ctrl);
	if (!pThis)
		return -ENOMEM;

	spin_lock_init(&pThis->Lock);
	pThis->board_mode = mode_default;
	pThis->board_set_power = plat->set_power;
	pThis->set_clock = plat->set_clock;
	pThis->min_power = plat->min_power;

	/* Clock usage is chip-specific ... functional clock (DaVinci,
	 * OMAP2430), or PHY ref (some TUSB6010 boards).  All this core
	 * code does is make sure a clock handle is available; platform
	 * code manages it during start/stop and suspend/resume.
	 */
	if (plat->clock) {
		pThis->clock = clk_get(dev, plat->clock);
		if (IS_ERR(pThis->clock)) {
			status = PTR_ERR(pThis->clock);
			pThis->clock = NULL;
			goto fail;
		}
	}
	
	/* Archos S.A.
	 * add an additional auxillary clock, used on Archos' Gen6 to
	 * supply the external PHY
	 */
	if (plat->aux_clock) {
		pThis->aux_clock = clk_get(dev, plat->aux_clock);
		if (IS_ERR(pThis->aux_clock)) {
			status = PTR_ERR(pThis->aux_clock);
			pThis->aux_clock = NULL;
			goto fail;
		}
		clk_enable(pThis->aux_clock);
	}

	/* assume vbus is off */

	/* platform adjusts pThis->pRegs and pThis->isr if needed,
	 * and activates clocks
	 */
	pThis->isr = generic_interrupt;
	status = musb_platform_init(pThis);

	if (status < 0)
		goto fail;
	if (!pThis->isr) {
		status = -ENODEV;
		goto fail2;
	}

#ifndef CONFIG_USB_INVENTRA_FIFO
	if (use_dma && dev->dma_mask) {
		struct dma_controller	*c;

		c = dma_controller_create(pThis, pThis->pRegs);
		pThis->pDmaController = c;
		if (c)
			(void) c->start(c->pPrivateData);
	}
#endif
	/* ideally this would be abstracted in platform setup */
	if (!is_dma_capable() || !pThis->pDmaController)
		dev->dma_mask = NULL;

	/* be sure interrupts are disabled before connecting ISR */
	musb_platform_disable(pThis);
	musb_generic_disable(pThis);

	/* setup musb parts of the core (especially endpoints) */
	status = musb_core_init(plat->multipoint
			? MUSB_CONTROLLER_MHDRC
			: MUSB_CONTROLLER_HDRC, pThis);
	if (status < 0)
		goto fail2;

	INIT_WORK(&pThis->irq_work, musb_irq_work);

	/* attach to the IRQ */
	if (request_irq (nIrq, pThis->isr, IRQF_DISABLED, dev->bus_id, pThis)) {
		dev_err(dev, "request_irq %d failed!\n", nIrq);
		status = -ENODEV;
		goto fail2;
	}
	pThis->nIrq = nIrq;
// FIXME this handles wakeup irqs wrong
	if (enable_irq_wake(nIrq) == 0)
		device_init_wakeup(dev, 1);

	pr_info("%s: USB %s mode controller at %p using %s + fifo %d, IRQ %d\n",
			musb_driver_name,
			({char *s;
			switch (pThis->board_mode) {
			case MUSB_HOST:		s = "Host"; break;
			case MUSB_PERIPHERAL:	s = "Peripheral"; break;
			default:		s = "OTG"; break;
			}; s; }),
			ctrl,
			(is_dma_capable() && pThis->pDmaController)
				? "DMA" : "PIO",
			fifo_mode,
			pThis->nIrq);

#ifdef CONFIG_USB_MUSB_HDRC_HCD
	/* host side needs more setup, except for no-host modes */
	if( is_host_enabled(pThis) ){
		struct usb_hcd	*hcd = musb_to_hcd(pThis);

		if (pThis->board_mode == MUSB_OTG)
			hcd->self.otg_port = 1;
		pThis->xceiv.host = &hcd->self;
		hcd->power_budget = 2 * (plat->power ? : 250);
	}
#endif				/* CONFIG_USB_MUSB_HDRC_HCD */

	/* For the host-only role, we can activate right away.
	 * (We expect the ID pin to be forcibly grounded!!)
	 * Otherwise, wait till the gadget driver hooks up.
	 */
	if (!is_otg_enabled(pThis) && is_host_enabled(pThis)) {
		MUSB_HST_MODE(pThis);
		pThis->xceiv.default_a = 1;
		pThis->xceiv.state = OTG_STATE_A_IDLE;

		status = usb_add_hcd(musb_to_hcd(pThis), -1, 0);

		DBG(1, "%s mode, status %d, devctl %02x %c\n",
			"HOST", status,
			musb_readb(pThis->pRegs, MGC_O_HDRC_DEVCTL),
			(musb_readb(pThis->pRegs, MGC_O_HDRC_DEVCTL)
					& MGC_M_DEVCTL_BDEVICE
				? 'B' : 'A'));

	} else /* peripheral is enabled */ {
		MUSB_DEV_MODE(pThis);
		pThis->xceiv.default_a = 0;
		pThis->xceiv.state = OTG_STATE_B_IDLE;

		status = musb_gadget_setup(pThis);

		DBG(1, "%s mode, status %d, dev%02x\n",
			is_otg_enabled(pThis) ? "OTG" : "PERIPHERAL",
			status,
			musb_readb(pThis->pRegs, MGC_O_HDRC_DEVCTL));

	}
#if 0
#if defined(CONFIG_OMAP34XX_OFFMODE) && !defined(CONFIG_USB_MUSB_HDRC_MODULE)
		/* Save Context of MUSB to recover from OFF mode */
		musb_context_store_and_suspend(pThis, 0);
#endif
#endif

	if (status == 0) {
		musb_debug_create("driver/musb_hdrc", pThis);
	}
	else {
fail:
		if (pThis->clock)
			clk_put(pThis->clock);
		if (pThis->aux_clock) {
			clk_disable(pThis->aux_clock);
			clk_put(pThis->aux_clock);
		}
			
		device_init_wakeup(dev, 0);
		musb_free(pThis);
		return status;
	}

#ifdef CONFIG_SYSFS
	status = device_create_file(dev, &dev_attr_mode);
	status = device_create_file(dev, &dev_attr_cable);
	status = device_create_file(dev, &dev_attr_vbus);
#ifdef CONFIG_USB_MUSB_OTG
	status = device_create_file(dev, &dev_attr_srp);
#endif /* CONFIG_USB_MUSB_OTG */
	status = 0;
#endif

	return status;

fail2:
	musb_platform_exit(pThis);
	goto fail;
}

/*-------------------------------------------------------------------------*/

/* all implementations (PCI bridge to FPGA, VLYNQ, etc) should just
 * bridge to a platform device; this driver then suffices.
 */

#ifndef CONFIG_USB_INVENTRA_FIFO
static u64	*orig_dma_mask;
#endif

static int __init musb_probe(struct platform_device *pdev)
{
	struct device	*dev = &pdev->dev;
	int		irq = platform_get_irq(pdev, 0);
	struct resource	*iomem;
	void __iomem	*base;

	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!iomem || irq == 0)
		return -ENODEV;

	base = ioremap(iomem->start, iomem->end - iomem->start + 1);
	if (!base) {
		dev_err(dev, "ioremap failed\n");
		return -ENOMEM;
	}

#ifndef CONFIG_USB_INVENTRA_FIFO
	/* clobbered by use_dma=n */
	orig_dma_mask = dev->dma_mask;
#endif
	return musb_init_controller(dev, irq, base);
}

static int __devexit musb_remove(struct platform_device *pdev)
{
	struct musb	*musb = dev_to_musb(&pdev->dev);
	void __iomem	*ctrl_base = musb->ctrl_base;

	/* this gets called on rmmod.
	 *  - Host mode: host may still be active
	 *  - Peripheral mode: peripheral is deactivated (or never-activated)
	 *  - OTG mode: both roles are deactivated (or never-activated)
	 */
	musb_shutdown(pdev);
	musb_debug_delete("driver/musb_hdrc", musb);
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	if( is_host_enabled(musb) ){
		if (musb->board_mode == MUSB_HOST)
			usb_remove_hcd(musb_to_hcd(musb));
	}
#endif
	musb_free(musb);
	iounmap(ctrl_base);
	device_init_wakeup(&pdev->dev, 0);
#ifndef CONFIG_USB_INVENTRA_FIFO
	pdev->dev.dma_mask = orig_dma_mask;
#endif
	return 0;
}

#ifdef	CONFIG_PM

static int musb_suspend(struct platform_device *pdev, pm_message_t message)
{
	struct musb	*musb = dev_to_musb(&pdev->dev);
	unsigned long	flags;
	int		state =	message.event;
#ifdef CONFIG_USB_SUSPEND
	u8	devctl;
#endif

	if (!musb->clock)
		return 0;
#if 0	
#if defined(CONFIG_OMAP34XX_OFFMODE) && !defined(CONFIG_USB_MUSB_HDRC_MODULE)
	/* No USB device attached case */
	/* May have gone to Off mode already */
	/* In that case dont touch registers */
	if(musb->asleep)
		return 0;	
#endif
#endif

	DBG(1, "suspend from %d -> %d\n", pdev->dev.power.power_state.event, state);
	if (state <= pdev->dev.power.power_state.event)
		return 0;

	//if (platform_off_state(state))
	//	state = OFF_STATE;

	//if (state != OFF_STATE)
		//goto idle_standby;

	spin_lock_irqsave(&musb->Lock, flags);

#ifdef CONFIG_USB_GADGET_MUSB_HDRC
	if (is_peripheral_active(musb)) {
		/* FIXME force disconnect unless we know USB will wake
		 * the system up quickly enough to respond ...
		 */
		if (musb->pGadgetDriver)
			musb_g_stop_activity(musb, musb->pGadgetDriver);

	}
#endif

	if (is_host_active(musb)) {
		/* we know all the children are suspended; sometimes
		 * they will even be wakeup-enabled.
		 */
#ifdef CONFIG_USB_SUSPEND
		spin_unlock_irqrestore(&musb->Lock, flags);
//#ifdef USB_INTERNAL_TEST
		musb->is_active = 0;
	
		//Clear the session
		devctl = musb_readb(musb->pRegs, MGC_O_HDRC_DEVCTL);
		devctl &= ~MGC_M_DEVCTL_SESSION;
		musb_writeb(musb->pRegs, MGC_O_HDRC_DEVCTL, devctl);
//#endif /* USB_INTERNAL_TEST */

		spin_lock_irqsave(&musb->Lock, flags);
#else
		musb_platform_disable(musb);
		musb_generic_disable(musb);
#endif
		//hcd->self.root_hub->state = USB_STATE_SUSPENDED;
		//hcd->state = HC_STATE_SUSPENDED
	}

	spin_unlock_irqrestore(&musb->Lock, flags);

//idle_standby:
	DBG(1, "idle, standby etc.\n");
	/* unlock irq before stopping clocks, because abort of DMA channels in 
	 * Mentor DMA controller logs an interrupt that needs to be cleared
	 */  
	musb_platform_suspend(musb); //, state);

	/* asleep = 1 if clocks are OFF */
	//if (state == OFF_STATE)
	musb->asleep = 1;

	pdev->dev.power.power_state.event = state;

	return 0;
}

#if 0
#if defined(CONFIG_OMAP34XX_OFFMODE) && !defined(CONFIG_USB_MUSB_HDRC_MODULE)
extern void musb_context_restore_and_wakeup(void);
#endif
#endif

static int musb_resume(struct platform_device *pdev)
{
	unsigned long	flags;
	struct musb	*musb = dev_to_musb(&pdev->dev);
#ifdef CONFIG_USB_SUSPEND
//	struct usb_hcd *hcd = musb_to_hcd(musb);
	u8	devctl;
#endif

	if (!musb->clock)
		return 0;

	DBG(1, "resume %d \n", pdev->dev.power.power_state.event);
	spin_lock_irqsave(&musb->Lock, flags);
#if 0
#if defined(CONFIG_OMAP34XX_OFFMODE) && !defined(CONFIG_USB_MUSB_HDRC_MODULE)
	musb_context_restore_and_wakeup();
#else
#endif
#endif
	musb_platform_resume(musb); //, dev->power.power_state.event);
	/* asleep = 0 if clocks are ON */
	musb->asleep = 0;

//	if (!platform_off_state(dev->power.power_state.event))
	//	goto done;
	
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
	if (is_peripheral_active(musb) && musb->pGadgetDriver) {
		musb->softconnect = 1;
		musb_start(musb);
	}
#endif

	if (is_host_active(musb)) {
#ifdef	CONFIG_USB_SUSPEND
		DBG(1, "usb resume\n");
		/* get extra cleanup even if remote wakeup isn't in use */
		spin_unlock_irqrestore(&musb->Lock, flags);

//#ifdef USB_INTERNAL_TEST
		devctl = musb_readb(musb->pRegs, MGC_O_HDRC_DEVCTL);
		devctl |= MGC_M_DEVCTL_SESSION;
		musb_writeb(musb->pRegs, MGC_O_HDRC_DEVCTL, devctl);
#ifdef USB_INTERNAL_TEST
#ifdef CONFIG_TWL4030_USB_HS_ULPI
		/* following is a workaround for Triton2 */
		mdelay(100);
		if (!(musb_readb(musb->pRegs, MGC_O_HDRC_DEVCTL) &
						MGC_M_DEVCTL_SESSION))
			musb_writeb(musb->pRegs, MGC_O_HDRC_DEVCTL, devctl);
#endif
		musb->port1_status &= ~(USB_PORT_STAT_LOW_SPEED
						|USB_PORT_STAT_HIGH_SPEED
						|USB_PORT_STAT_ENABLE);
		musb->port1_status |= USB_PORT_STAT_CONNECTION
					|(USB_PORT_STAT_C_CONNECTION << 16);
		usb_hcd_resume_root_hub(hcd);
#endif /* USB_INTERNAL_TEST */
#else
		musb_start(musb);
#endif
	}	
	/* for static cmos like DaVinci, register values were preserved
	 * unless for some reason the whole soc powered down and we're
	 * not treating that as a whole-system restart (e.g. swsusp)
	 */
//done:
	spin_unlock_irqrestore(&musb->Lock, flags);
	pdev->dev.power.power_state.event = 0; //DEV_SUSPEND_ON;
	return 0;
}

#else
#define	musb_suspend	NULL
#define	musb_resume	NULL
#endif

static struct platform_driver musb_driver = {
	.driver = {
		.name		= (char *)musb_driver_name,
		.bus		= &platform_bus_type,
		.owner		= THIS_MODULE,
	},
	.remove		= __devexit_p(musb_remove),
	.shutdown	= musb_shutdown,
	.suspend	= musb_suspend,
	.resume		= musb_resume,
};

/*-------------------------------------------------------------------------*/

static int __init musb_init(void)
{
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	if( mode_default == MUSB_HOST ){
		if (usb_disabled())
			return 0;
	}
#endif

	pr_info("%s: version " MUSB_VERSION ", "
#ifdef CONFIG_USB_INVENTRA_FIFO
		"pio"
#elif defined(CONFIG_USB_TI_CPPI_DMA)
		"cppi-dma"
#elif defined(CONFIG_USB_INVENTRA_DMA)
		"musb-dma"
#elif defined(CONFIG_USB_TUSB_OMAP_DMA)
		"tusb-omap-dma"
#else
		"?dma?"
#endif
		", "
#ifdef CONFIG_USB_MUSB_OTG
		"otg (peripheral+host)"
#endif
#if defined(CONFIG_USB_GADGET_MUSB_HDRC)
		"peripheral"
#endif
#if defined(CONFIG_USB_MUSB_HDRC_HCD)
		"host"
#endif
		", debug=%d\n",
		musb_driver_name, debug);
	return platform_driver_probe(&musb_driver, musb_probe);
}

/* make us init after usbcore and before usb
 * gadget and host-side drivers start to register
 */
subsys_initcall(musb_init);

static void __exit musb_cleanup(void)
{
	platform_driver_unregister(&musb_driver);
}
module_exit(musb_cleanup);
