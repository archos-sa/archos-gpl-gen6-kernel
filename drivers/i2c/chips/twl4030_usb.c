/*
 * twl4030_usb - TWL4030 USB transceiver, talking to OMAP OTG controller
 *
 * Copyright (C) 2004-2007 Texas Instruments
 *
 * FS USB suport is based heavily on the isp1301_omap.c OTG transceiver driver.
 * Copyright (C) 2004 David Brownell
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Current status:
 *	- FS USB Gadget and Host modes work independently. No OTG support.
 *	- HS USB ULPI mode works.
 */


#include <linux/module.h>
#include <linux/kernel_stat.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/usb/ch9.h>
#include <linux/usb_gadget.h>
#include <linux/usb.h>
#include <linux/usb/otg.h>

#include <asm/arch/twl4030.h>
#include <asm/arch/usb.h>
#include <asm/arch/irqs.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/arch/mux.h>
#include <asm/arch/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/clock.h>

#ifdef CONFIG_OMAP3430_ES2 
#include <asm/arch/resource.h>
#endif

/* Register defines */

#define	VENDOR_ID_LO		(0x00)
#define	VENDOR_ID_HI		(0x01)
#define	PRODUCT_ID_LO		(0x02)
#define	PRODUCT_ID_HI		(0x03)

#define	FUNC_CTRL		(0x04)
#define	FUNC_CTRL_SET		(0x05)
#define	FUNC_CTRL_CLR		(0x06)
#	define	SUSPENDM		(1 << 6)
#	define	RESET			(1 << 5)
#	define	OPMODE_MASK		(3 << 3) /* bits 3 and 4 */
#	define	OPMODE_NORMAL		(0 << 3)
#	define	OPMODE_NONDRIVING	(1 << 3)
#	define	OPMODE_DISABLE_BIT_NRZI	(2 << 3)
#	define	TERMSELECT		(1 << 2)
#	define	XCVRSELECT_MASK		(3 << 0) /* bits 0 and 1 */
#	define	XCVRSELECT_HS		(0 << 0)
#	define	XCVRSELECT_FS		(1 << 0)
#	define	XCVRSELECT_LS		(2 << 0)
#	define	XCVRSELECT_FS4LS	(3 << 0)

#define	IFC_CTRL     		(0x07)
#define	IFC_CTRL_SET		(0x08)
#define	IFC_CTRL_CLR		(0x09)
#	define	INTERFACE_PROTECT_DISABLE (1 << 7)
#	define	AUTORESUME		(1 << 4)
#	define	CLOCKSUSPENDM		(1 << 3)
#	define	CARKITMODE		(1 << 2)
#	define	FSLSSERIALMODE_3PIN	(1 << 1)

#define	OTG_CTRL		(0x0A)
#define	OTG_CTRL_SET 		(0x0B)
#define	OTG_CTRL_CLR 	   	(0x0C)
#	define	DRVVBUS			(1 << 5)
#	define	CHRGVBUS		(1 << 4)
#	define	DISCHRGVBUS		(1 << 3)
#	define	DMPULLDOWN		(1 << 2)
#	define	DPPULLDOWN		(1 << 1)
#	define	IDPULLUP		(1 << 0)

#define	USB_INT_EN_RISE		(0x0D)
#define	USB_INT_EN_RISE_SET	(0x0E)
#define	USB_INT_EN_RISE_CLR	(0x0F)
#define	USB_INT_EN_FALL		(0x10)
#define	USB_INT_EN_FALL_SET	(0x11)
#define	USB_INT_EN_FALL_CLR	(0x12)
#define	USB_INT_STS		(0x13)
#define	USB_INT_LATCH		(0x14)
#	define	IDGND			(1 << 4)
#	define	SESSEND			(1 << 3)
#	define	SESSVALID		(1 << 2)
#	define	VBUSVALID		(1 << 1)
#	define	HOSTDISCONNECT		(1 << 0)

#define	CARKIT_CTRL		(0x19)
#define	CARKIT_CTRL_SET		(0x1A)
#define	CARKIT_CTRL_CLR		(0x1B)
#define		MICEN			(1 << 6)
#define		SPKRIGHTEN		(1 << 5)
#define		SPKLEFTEN		(1 << 4)
#define		RXDEN			(1 << 3)
#define		TXDEN			(1 << 2)
#define		IDGNDDRV		(1 << 1)
#define		CARKITPWR		(1 << 0)
#define	CARKIT_PLS_CTRL		(0x22)
#define	CARKIT_PLS_CTRL_SET	(0x23)
#define	CARKIT_PLS_CTRL_CLR	(0x24)
#	define	SPKRRIGHT_BIASEN	(1 << 3)
#	define	SPKRLEFT_BIASEN		(1 << 2)
#	define	RXPLSEN			(1 << 1)
#	define	TXPLSEN			(1 << 0)

#define	MCPC_CTRL		(0x30)
#define	MCPC_CTRL_SET		(0x31)
#define	MCPC_CTRL_CLR		(0x32)
#define		RTSOL			(1 << 7)
#define		EXTSWR			(1 << 6)
#define		EXTSWC			(1 << 5)
#define		VOICESW			(1 << 4)
#define		OUT64K			(1 << 3)
#define		RTSCTSSW		(1 << 2)
#define		HS_UART			(1 << 0)

#define	MCPC_IO_CTRL		(0x33)
#define	MCPC_IO_CTRL_SET	(0x34)
#define	MCPC_IO_CTRL_CLR	(0x35)
#	define	MICBIASEN		(1<< 5)
#	define	CTS_NPU			(1 << 4)
#	define	RXD_PU			(1 << 3)
#	define	TXDTYP			(1 << 2)
#	define	CTSTYP			(1 << 1)
#	define	RTSTYP			(1 << 0)

#define	MCPC_CTRL2		(0x36)
#define	MCPC_CTRL2_SET		(0x37)
#define	MCPC_CTRL2_CLR		(0x38)
#	define	MCPC_CK_EN		(1 << 0)

#define	OTHER_FUNC_CTRL		(0x80)
#define	OTHER_FUNC_CTRL_SET	(0x81)
#define	OTHER_FUNC_CTRL_CLR	(0x82)
#	define	BDIS_ACON_EN		(1<< 4)
#	define	FIVEWIRE_MODE		(1 << 2)

#define	OTHER_IFC_CTRL		(0x83)
#define	OTHER_IFC_CTRL_SET	(0x84)
#define	OTHER_IFC_CTRL_CLR	(0x85)
#	define	OE_INT_EN		(1 << 6)
#	define	CEA2011_MODE		(1 << 5)
#	define	FSLSSERIALMODE_4PIN	(1 << 4)
#	define	HIZ_ULPI_60MHZ_OUT	(1 << 3)
#	define	HIZ_ULPI		(1 << 2)
#	define	ALT_INT_REROUTE		(1 << 0)

#define	OTHER_INT_EN_RISE	(0x86)
#define	OTHER_INT_EN_RISE_SET	(0x87)
#define	OTHER_INT_EN_RISE_CLR	(0x88)
#define	OTHER_INT_EN_FALL	(0x89)
#define	OTHER_INT_EN_FALL_SET	(0x8A)
#define	OTHER_INT_EN_FALL_CLR	(0x8B)
#define	OTHER_INT_STS		(0x8C)
#define	OTHER_INT_LATCH		(0x8D)
#	define	VB_SESS_VLD		(1 << 7)
#	define	DM_HI			(1 << 6) /* not valid for "latch" reg */
#	define	DP_HI			(1 << 5) /* not valid for "latch" reg */
#	define	BDIS_ACON		(1 << 3) /* not valid for "fall" regs */
#	define	MANU			(1 << 1)
#	define	ABNORMAL_STRESS		(1 << 0)

#define	ID_STATUS		(0x96)
#	define	ID_RES_FLOAT		(1 << 4)
#	define	ID_RES_440K		(1 << 3)
#	define	ID_RES_200K		(1 << 2)
#	define	ID_RES_102K		(1 << 1)
#	define	ID_RES_GND		(1 << 0)

#define	POWER_CTRL		(0xAC)
#define	POWER_CTRL_SET		(0xAD)
#define	POWER_CTRL_CLR		(0xAE)
#	define	OTG_ENAB		(1 << 5)

#define	OTHER_IFC_CTRL2		(0xAF)
#define	OTHER_IFC_CTRL2_SET	(0xB0)
#define	OTHER_IFC_CTRL2_CLR	(0xB1)
#	define	ULPI_STP_LOW		(1 << 4)
#	define	ULPI_TXEN_POL		(1 << 3)
#	define	ULPI_4PIN_2430		(1 << 2)
#	define	USB_INT_OUTSEL_MASK	(3 << 0) /* bits 0 and 1 */
#	define	USB_INT_OUTSEL_INT1N	(0 << 0)
#	define	USB_INT_OUTSEL_INT2N	(1 << 0)

#define	REG_CTRL_EN		(0xB2)
#define	REG_CTRL_EN_SET		(0xB3)
#define	REG_CTRL_EN_CLR		(0xB4)
#define	REG_CTRL_ERROR		(0xB5)
#	define	ULPI_I2C_CONFLICT_INTEN	(1 << 0)

#define	OTHER_FUNC_CTRL2	(0xB8)
#define	OTHER_FUNC_CTRL2_SET	(0xB9)
#define	OTHER_FUNC_CTRL2_CLR	(0xBA)
#	define	VBAT_TIMER_EN		(1 << 0)

/* following registers do not have separate _clr and _set registers */
#define	VBUS_DEBOUNCE		(0xC0)
#define	ID_DEBOUNCE		(0xC1)
#define	VBAT_TIMER		(0xD3)
#define	PHY_PWR_CTRL		(0xFD)
#	define	PHYPWD			(1 << 0)
#define	PHY_CLK_CTRL		(0xFE)
#	define	CLOCKGATING_EN		(1 << 2)
#	define	CLK32K_EN      		(1 << 1)
#	define	REQ_PHY_DPLL_CLK	(1 << 0)
#define	PHY_CLK_CTRL_STS	(0xFF)
#	define	PHY_DPLL_CLK		(1 << 0)

/* In module TWL4030_MODULE_PM_MASTER */
#define	PROTECT_KEY		(0x0E)

/* In module TWL4030_MODULE_PM_RECIEVER */
#define	VUSB_DEDICATED1		(0x7D)
#define	VUSB_DEDICATED2		(0x7E)
#define	VUSB1V5_DEV_GRP		(0x71)
#define	VUSB1V5_TYPE		(0x72)
#define	VUSB1V5_REMAP		(0x73)
#define	VUSB1V8_DEV_GRP		(0x74)
#define	VUSB1V8_TYPE		(0x75)
#define	VUSB1V8_REMAP		(0x76)
#define	VUSB3V1_DEV_GRP		(0x77)
#define	VUSB3V1_TYPE		(0x78)
#define	VUSB3V1_REMAP		(0x79)

#define	ID_STATUS		(0x96)
#define		ID_RES_FLOAT		(1 << 4) /* mini-B */
#define		ID_RES_440K		(1 << 3) /* type 2 charger */
#define		ID_RES_200K		(1 << 2) /* 5-wire carkit or
						    type 1 charger */
#define		ID_RES_102K		(1 << 1) /* phone */
#define		ID_RES_GND		(1 << 0) /* mini-A */

/* In module TWL4030_MODULE_INTBR */
#define PMBR1			(0x0D)
#	define GPIO_USB_4PIN_ULPI_2430C	(3 << 0)

/* In module TWL4030_MODULE_INT */
#define REG_PWR_ISR1		 (0x00)
#define REG_PWR_IMR1		 (0x01)
#define		USB_PRES		(1 << 2)
#define REG_PWR_EDR1		 (0x05)
#define		USB_PRES_FALLING	(1 << 4)
#define		USB_PRES_RISING		(1 << 5)
#define	REG_PWR_SIH_CTRL	(0x07)
#	define	COR			(1 << 2)

/*-------------------------------------------------------------------------*/

#define twl4030_i2c_read_u8_verify(module, data, address)		\
do {									\
	if (twl4030_i2c_read_u8(module, data, address) < 0) {		\
		printk(KERN_ERR "twl4030_usb: i2c read failed,		\
			line %d\n", __LINE__);				\
		goto i2c_failed;					\
	}								\
} while (0)

#define twl4030_i2c_write_u8_verify(module, data, address)		  \
do {									  \
	u8 check;							  \
	if (!((twl4030_i2c_write_u8((module), (data), (address)) >= 0) && \
	      (twl4030_i2c_read_u8((module), &check, (address)) >= 0) &&  \
	      (check == data))	&&					  \
	    !((twl4030_i2c_write_u8((module), (data), (address)) >= 0) && \
	      (twl4030_i2c_read_u8((module), &check, (address)) >= 0) &&  \
	      (check == (data))))	{				  \
		printk(KERN_ERR "twl4030_usb: i2c write failed,		  \
			line %d\n", __LINE__);				  \
		goto i2c_failed;					  \
	}								  \
} while (0)

#define twl4030_usb_write_verify(address, data)	\
	twl4030_i2c_write_u8_verify(TWL4030_MODULE_USB, (data), (address))

static inline int twl4030_usb_write(u8 address, u8 data)
{
	int ret = 0;
	ret = twl4030_i2c_write_u8(TWL4030_MODULE_USB, data, address);
	if (ret >= 0) {
#if 0	/* debug */
		u8 data1;
		if (twl4030_i2c_read_u8(TWL4030_MODULE_USB, &data1,
					address) < 0)
			printk(KERN_ERR "re-read failed\n");
		else
			printk("Write %s wrote %x read %x from reg %x\n",
			       (data1 == data) ? "succeed" : "mismatch",
			       data, data1, address);
#endif
	} else {
		printk(KERN_WARNING
			"TWL4030:USB:Write[0x%x] Error %d\n", address, ret);
	}
	return ret;
}

static inline int twl4030_usb_read(u8 address)
{
	u8 data;
	int ret = 0;
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_USB, &data, address);
	if (ret >= 0) {
		ret = data;
	} else {
		printk(KERN_WARNING
			"TWL4030:USB:Read[0x%x] Error %d\n", address, ret);
	}
	return ret;
}

/*-------------------------------------------------------------------------*/

struct twl4030_usb {
	struct otg_transceiver	otg;
	int			irq;
	u8 			usb_mode;	/* pin configuration */
#		define T2_USB_MODE_ULPI		1
#		define T2_USB_MODE_CEA2011_3PIN	2
	u8			asleep;
#ifdef CONFIG_OMAP3430_ES2
	struct	constraint_handle	*usb_power_constraint;
#endif
};

#ifdef CONFIG_OMAP3430_ES2
static struct constraint_id cnstr_id = {
	.type = RES_LATENCY_CO,
	.data = (void *)"latency",
};
#endif

static struct twl4030_usb *the_transceiver;

/*-------------------------------------------------------------------------*/

static inline int
twl4030_usb_set_bits(struct twl4030_usb *twl, u8 reg, u8 bits)
{
	return twl4030_usb_write(reg + 1, bits);
}

static inline int
twl4030_usb_clear_bits(struct twl4030_usb *twl, u8 reg, u8 bits)
{
	return twl4030_usb_write(reg + 2, bits);
}

/*-------------------------------------------------------------------------*/

#ifdef CONFIG_TWL4030_USB_FS_3_PIN

/* bits in OTG_CTRL_REG */

#define	OTG_XCEIV_OUTPUTS \
	(OTG_ASESSVLD|OTG_BSESSEND|OTG_BSESSVLD|OTG_VBUSVLD|OTG_ID)
#define	OTG_XCEIV_INPUTS \
	(OTG_PULLDOWN|OTG_PULLUP|OTG_DRV_VBUS|OTG_PD_VBUS|OTG_PU_VBUS|OTG_PU_ID)
#define	OTG_CTRL_BITS \
	(OTG_A_BUSREQ|OTG_A_SETB_HNPEN|OTG_B_BUSREQ|OTG_B_HNPEN|OTG_BUSDROP)
	/* and OTG_PULLUP is sometimes written */

#define	OTG_CTRL_MASK	(OTG_DRIVER_SEL| \
	OTG_XCEIV_OUTPUTS|OTG_XCEIV_INPUTS| \
	OTG_CTRL_BITS)


#ifdef CONFIG_ARCH_OMAP24XX
static struct clk *fs_usb_ick;
static struct clk *fs_usb_fck;
static int clocks_in_use;

static int omap2_usb_clk(int on)
{
	if (!fs_usb_ick)
		fs_usb_ick = clk_get((struct device *)NULL, "usb_l4_ick");
	if (!fs_usb_fck)
		fs_usb_fck = clk_get((struct device *)NULL, "usb_fck");

	if (on && !clocks_in_use) {
		clk_use(fs_usb_ick);
		clk_use(fs_usb_fck);
		clk_safe(fs_usb_fck);
		clocks_in_use = 1;
	} else if (!on && clocks_in_use) {
		clk_unuse(fs_usb_ick);
		clk_unuse(fs_usb_fck);
		clocks_in_use = 0;
	}
	return 0;
}

#define get_usb_clk() omap2_usb_clk(1);
#define put_usb_clk() omap2_usb_clk(0);
#else
#define get_usb_clk()
#define put_usb_ckl()
#endif	

static void enable_vbus_draw(struct twl4030_usb *twl, unsigned mA)
{
	pr_debug("%s UNIMPL\n", __FUNCTION__);
}
#if 0
static void enable_vbus_source(struct twl4030_usb *twl)
{
	pr_debug("%s UNIMPL\n", __FUNCTION__);
}
#endif

/* products will deliver OTG messages with LEDs, GUI, etc */
static inline void notresponding(struct twl4030_usb *twl)
{
	printk(KERN_NOTICE "OTG device not responding.\n");
}


static inline void dp_pullup(struct twl4030_usb *twl, int enable)
{
	if (enable) {
		twl4030_usb_clear_bits(twl, OTG_CTRL, DPPULLDOWN);
		twl4030_usb_set_bits(twl, FUNC_CTRL, TERMSELECT);
	} else {
		twl4030_usb_clear_bits(twl, FUNC_CTRL, TERMSELECT);
		twl4030_usb_set_bits(twl, OTG_CTRL, DPPULLDOWN);
	}
}

static int twl4030_usb_set_power(struct otg_transceiver *dev, unsigned mA)
{
	if (!the_transceiver)
		return -ENODEV;
	if (dev->state == OTG_STATE_B_PERIPHERAL)
		enable_vbus_draw(the_transceiver, mA);
	return 0;
}

#if	defined(CONFIG_USB_GADGET_OMAP) //temporary until OTG support is added.
static void b_peripheral(struct twl4030_usb *twl)
{
	OTG_CTRL_REG = OTG_CTRL_REG & OTG_XCEIV_OUTPUTS;
	usb_gadget_vbus_connect(twl->otg.gadget);

#ifdef	CONFIG_USB_OTG
	enable_vbus_draw(twl, 8);
	otg_update_twl(twl);
#else
	enable_vbus_draw(twl, 100);
	if (twl4030_usb_read(OTHER_INT_STS) & VB_SESS_VLD) {
		OTG_CTRL_REG &= ~OTG_BSESSEND;
		OTG_CTRL_REG |= OTG_BSESSVLD;
	}
	/* UDC driver just set OTG_BSESSVLD */
	dp_pullup(twl, 1);

	twl->otg.state = OTG_STATE_B_PERIPHERAL;
#endif
}
#endif

static int twl4030_usb_set_peripheral(struct otg_transceiver *otg,
				      struct usb_gadget *gadget)
{
	struct twl4030_usb *twl = container_of(otg, struct twl4030_usb, otg);

	if (!otg || twl != the_transceiver)
		return -ENODEV;

	if (!gadget) {
		get_usb_clk();
		OTG_IRQ_EN_REG = 0;
		put_usb_clk();
		if (!twl->otg.default_a)
			enable_vbus_draw(twl, 0);
		usb_gadget_vbus_disconnect(twl->otg.gadget);
		twl->otg.gadget = 0;
		twl4030_phy_suspend(1);
		return 0;
	}

#ifdef	CONFIG_USB_OTG
	/* ... */

#elif	!defined(CONFIG_USB_OHCI_HCD) && !defined(CONFIG_USB_OHCI_HCD_MODULE)

	twl4030_phy_resume();

	get_usb_clk();

	twl->otg.gadget = gadget;
	// FIXME update its refcount

	OTG_CTRL_REG = (OTG_CTRL_REG & OTG_CTRL_MASK
				& ~(OTG_XCEIV_OUTPUTS|OTG_CTRL_BITS))
			| OTG_ID;
	twl->otg.state = OTG_STATE_B_IDLE;

	twl4030_usb_set_bits(twl, USB_INT_EN_RISE, SESSVALID | VBUSVALID);
	twl4030_usb_set_bits(twl, USB_INT_EN_FALL, SESSVALID | VBUSVALID);

	printk(KERN_INFO "B-Peripheral sessions ok\n");

	/* If this has a Mini-AB connector, this mode is highly
	 * nonstandard ... but can be handy for testing, so long
	 * as you don't plug a Mini-A cable into the jack.
	 */
//	if (twl4030_usb_read(USB_INT_STS) & VBUSVALID)
		b_peripheral(twl);
	put_usb_clk();
	return 0;

#else
	printk(KERN_DEBUG "peripheral sessions not allowed\n");
	return -EINVAL;
#endif
}

/* add or disable the host device+driver */
static int twl4030_usb_set_host(struct otg_transceiver *otg,
				struct usb_bus *host)
{
	struct twl4030_usb *twl = container_of(otg, struct twl4030_usb, otg);

	if (!otg || twl != the_transceiver)
		return -ENODEV;

	if (!host) {
		get_usb_clk();
		OTG_IRQ_EN_REG = 0;
		put_usb_clk();
		twl4030_phy_suspend(1);
		twl->otg.host = 0;
		return 0;
	}

#ifdef	CONFIG_USB_OTG
	/* ... */

#elif	!defined(CONFIG_USB_GADGET_OMAP) && !defined(CONFIG_USB_GADGET_OMAP_MODULE)

	twl->otg.host = host;
	twl4030_phy_resume();

	twl4030_usb_set_bits(twl, OTG_CTRL, DMPULLDOWN | DPPULLDOWN);
	twl4030_usb_set_bits(twl, USB_INT_EN_RISE, IDGND);
	twl4030_usb_set_bits(twl, USB_INT_EN_FALL, IDGND);
	twl4030_usb_set_bits(twl, FUNC_CTRL, SUSPENDM);

	/* If this has a Mini-AB connector, this mode is highly
	 * nonstandard ... but can be handy for testing, especially with
	 * the Mini-A end of an OTG cable.  (Or something nonstandard
	 * like MiniB-to-StandardB, maybe built with a gender mender.)
	 */
	twl4030_usb_set_bits(twl, OTG_CTRL, DRVVBUS);

	printk(KERN_INFO "A-Host sessions ok\n");
#else
	printk(KERN_DEBUG "host sessions not allowed\n");
	return -EINVAL;
#endif
	return 0;
}

static void twl4030_usb_reset(struct twl4030_usb *twl)
{
	/* make like power-on reset */

	twl4030_usb_clear_bits(twl, OTG_CTRL, DMPULLDOWN | DPPULLDOWN);
	twl4030_usb_clear_bits(twl, USB_INT_EN_RISE, ~0);
	twl4030_usb_clear_bits(twl, USB_INT_EN_FALL, ~0);
	twl4030_usb_clear_bits(twl, MCPC_IO_CTRL, ~TXDTYP);
	twl4030_usb_set_bits(twl, MCPC_IO_CTRL, TXDTYP);
	twl4030_usb_clear_bits(twl, OTHER_FUNC_CTRL,
			       (BDIS_ACON_EN | FIVEWIRE_MODE));
	twl4030_usb_clear_bits(twl, OTHER_IFC_CTRL, ~0);
	twl4030_usb_clear_bits(twl, OTHER_INT_EN_RISE, ~0);
	twl4030_usb_clear_bits(twl, OTHER_INT_EN_FALL, ~0);
	twl4030_usb_clear_bits(twl, OTHER_IFC_CTRL2, ~0);
	twl4030_usb_clear_bits(twl, REG_CTRL_EN, ULPI_I2C_CONFLICT_INTEN);
	twl4030_usb_clear_bits(twl, OTHER_FUNC_CTRL2, VBAT_TIMER_EN);
}

static int fs_usb_init(struct twl4030_usb *twl)
{
	int status = 0;

	twl->usb_mode = T2_USB_MODE_CEA2011_3PIN;

	twl->otg.set_host = twl4030_usb_set_host,
	twl->otg.set_peripheral = twl4030_usb_set_peripheral,
	twl->otg.set_power = twl4030_usb_set_power,
	twl->otg.label = "twl4030";

	status = otg_set_transceiver(&twl->otg);
	if (status < 0)
		printk(KERN_ERR "can't register TWL4030 USB transceiver, %d\n",
			status);
	return 0;
}
#endif	// CONFIG_TWL4030_USB_FS_3_PIN

/*-------------------------------------------------------------------------*/

static void twl4030_cea2011_3_pin_FS_setup(struct twl4030_usb *twl)
{
	u8 pmbr1;

	/* Important! - choose bet GPIO or USB */
	twl4030_i2c_read_u8(TWL4030_MODULE_INTBR, &pmbr1, PMBR1);
	twl4030_i2c_write_u8(TWL4030_MODULE_INTBR,
			     pmbr1 | GPIO_USB_4PIN_ULPI_2430C, PMBR1);

	/* Mux between UART and USB ULPI lines */

	twl4030_usb_clear_bits(twl, MCPC_CTRL2, MCPC_CK_EN);

	twl4030_usb_clear_bits(twl, CARKIT_CTRL,
					CARKITPWR |
					TXDEN |
					RXDEN |
					SPKLEFTEN |
					MICEN);

	twl4030_usb_clear_bits(twl, CARKIT_PLS_CTRL,
					TXPLSEN |
					RXPLSEN |
					SPKRLEFT_BIASEN);

	twl4030_usb_clear_bits(twl, MCPC_CTRL,
					HS_UART |
					RTSCTSSW |
					OUT64K |
					VOICESW |
					EXTSWC |
					EXTSWR);

	twl4030_usb_set_bits(twl, MCPC_IO_CTRL, TXDTYP);

	twl4030_usb_clear_bits(twl, MCPC_IO_CTRL,
					RTSTYP |
					CTSTYP |
					RXD_PU |
					CTS_NPU |
					MICBIASEN);

	twl4030_usb_set_bits(twl, POWER_CTRL, OTG_ENAB);

	/* setup transceiver mode for FS */

	twl4030_usb_clear_bits(twl, OTG_CTRL, DPPULLDOWN | DMPULLDOWN);
	twl4030_usb_clear_bits(twl, FUNC_CTRL, XCVRSELECT_MASK | OPMODE_MASK);
	twl4030_usb_set_bits(twl, FUNC_CTRL, OPMODE_NORMAL | XCVRSELECT_FS);

	twl4030_usb_clear_bits(twl, IFC_CTRL, CARKITMODE);
	twl4030_usb_set_bits(twl, IFC_CTRL, FSLSSERIALMODE_3PIN);

	twl4030_usb_clear_bits(twl, OTHER_IFC_CTRL,
					CEA2011_MODE | FSLSSERIALMODE_4PIN);

	twl4030_usb_clear_bits(twl, OTHER_IFC_CTRL2, ULPI_4PIN_2430);

	twl4030_usb_set_bits(twl, OTHER_IFC_CTRL2, ULPI_TXEN_POL);
}

static void twl4030_usb_set_mode(struct twl4030_usb *twl, int mode)
{
	twl->usb_mode = mode;

	switch (mode) {
	case T2_USB_MODE_ULPI:
		twl4030_usb_clear_bits(twl, IFC_CTRL, CARKITMODE);
		twl4030_usb_set_bits(twl, POWER_CTRL, OTG_ENAB);
		twl4030_usb_clear_bits(twl, FUNC_CTRL,
					XCVRSELECT_MASK | OPMODE_MASK);
		break;
	case T2_USB_MODE_CEA2011_3PIN:
		twl4030_cea2011_3_pin_FS_setup(twl);
		break;
	default:
		/* power on defaults */
		break;
	};
}

#ifdef CONFIG_TWL4030_USB_HS_ULPI
static void hs_usb_init(struct twl4030_usb *twl)
{
	twl->usb_mode = T2_USB_MODE_ULPI;
	return;
}

#endif

static void twl4030_i2c_access(int on)
{
	unsigned long timeout;
	int val;

	if ((val = twl4030_usb_read(PHY_CLK_CTRL)) >= 0) {
		if (on) {
			/* enable DPLL to access PHY registers over I2C */
			val |= REQ_PHY_DPLL_CLK;
			twl4030_usb_write_verify(PHY_CLK_CTRL, (u8)val);

			timeout = jiffies + HZ;
			while (!(twl4030_usb_read(PHY_CLK_CTRL_STS) &
							PHY_DPLL_CLK)
				&& time_before(jiffies, timeout)) {
				udelay(10);
			}
			if (!(twl4030_usb_read(PHY_CLK_CTRL_STS) &
							PHY_DPLL_CLK))
				printk(KERN_ERR "Timeout setting T2 HSUSB "
						"PHY DPLL clock\n");
		} else {
			/* let ULPI control the DPLL clock */
			val &= ~REQ_PHY_DPLL_CLK;
			twl4030_usb_write_verify(PHY_CLK_CTRL, (u8)val);
		}
	}
i2c_failed:
	return;
}

static void usb_irq_enable(int rising, int falling)
{
	u8 val;

	/* edge setup */
	twl4030_i2c_read_u8_verify(TWL4030_MODULE_INT, &val, REG_PWR_EDR1);
	val &= ~(USB_PRES_RISING | USB_PRES_FALLING);
	if (rising)
		val = val | USB_PRES_RISING;
	if (falling)
		val = val | USB_PRES_FALLING;
	twl4030_i2c_write_u8_verify(TWL4030_MODULE_INT, val, REG_PWR_EDR1);

	/* un-mask interrupt */
	twl4030_i2c_read_u8_verify(TWL4030_MODULE_INT, &val, REG_PWR_IMR1);
	val &= ~USB_PRES;
	twl4030_i2c_write_u8_verify(TWL4030_MODULE_INT, val, REG_PWR_IMR1);

i2c_failed:
	return;
}

static void usb_irq_disable(void)
{
	u8 val;

	/* undo edge setup */
	twl4030_i2c_read_u8_verify(TWL4030_MODULE_INT, &val, REG_PWR_EDR1);
	val &= ~(USB_PRES_RISING | USB_PRES_FALLING);
	twl4030_i2c_write_u8_verify(TWL4030_MODULE_INT, val, REG_PWR_EDR1);

	/* mask interrupt */
	twl4030_i2c_read_u8_verify(TWL4030_MODULE_INT, &val, REG_PWR_IMR1);
	val |= USB_PRES;
	twl4030_i2c_write_u8_verify(TWL4030_MODULE_INT, val, REG_PWR_IMR1);

i2c_failed:
	return;
}

void twl4030_phy_suspend(int controller_off);
void twl4030_phy_resume(void);

static irqreturn_t twl4030_usb_irq(int irq, void *_twl)
{
	int ret = IRQ_NONE;
	u8 val;
	u8 sih_ctrl;

	/* save previous value of SIH_CTRL and disable clear_on_read */
	twl4030_i2c_read_u8(TWL4030_MODULE_INT, &sih_ctrl, REG_PWR_SIH_CTRL);
	twl4030_i2c_write_u8(TWL4030_MODULE_INT, (sih_ctrl & ~COR),
			     REG_PWR_SIH_CTRL);

	twl4030_i2c_read_u8_verify(TWL4030_MODULE_INT, &val, REG_PWR_ISR1);

	/* this interrupt line may be shared */
	if(!(val & USB_PRES))
		goto done;

	/* clear the interrupt */
	twl4030_i2c_write_u8(TWL4030_MODULE_INT, USB_PRES, REG_PWR_ISR1);

	/* action based on cable attach or detach */
	twl4030_i2c_read_u8_verify(TWL4030_MODULE_INT, &val, REG_PWR_EDR1);


	if (val & USB_PRES_RISING)
		twl4030_phy_resume();
	else
		twl4030_phy_suspend(0);

	twl4030_i2c_read_u8(TWL4030_MODULE_PM_MASTER,
				&val,
				0x0F /*STS_HW_CONDITIONS*/);
	if( (val & (1<<2)) || (val & (1<<7)) ){ /* STS_USB or STS_VBUS */
		printk(KERN_DEBUG "USB Device ATTACHED to T2");
	}
	else {
		printk(KERN_DEBUG "USB Device Disconnected from T2");
	}

	ret = IRQ_HANDLED;

i2c_failed:
done:
	/* restore previous value of SIH_CTRL */
	twl4030_i2c_write_u8(TWL4030_MODULE_INT, sih_ctrl, REG_PWR_SIH_CTRL);
	return ret;
}

static void twl4030_phy_power(struct twl4030_usb *twl, int on)
{
	u8 pwr;
	
	pwr = twl4030_usb_read(PHY_PWR_CTRL);
	if (on) {
		pwr &= ~PHYPWD;
		twl4030_usb_write_verify(PHY_PWR_CTRL, pwr);
		twl4030_usb_write(PHY_CLK_CTRL,
				  twl4030_usb_read(PHY_CLK_CTRL) |
					(CLOCKGATING_EN | CLK32K_EN));
	} else  {
		pwr |= PHYPWD;
		twl4030_usb_write_verify(PHY_PWR_CTRL, pwr);
	}
i2c_failed:
	return;
}

static void twl4030_usb_ldo_init(struct twl4030_usb *twl)
{
	/* Enable writing to power configuration registers */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0xC0, PROTECT_KEY);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0x0C, PROTECT_KEY);

	/* put VUSB3V1 LDO in active state */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECIEVER, 0, VUSB_DEDICATED2);

	/* input to VUSB3V1 LDO is from VBAT, not VBUS */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECIEVER, 0x14, VUSB_DEDICATED1);

	/* turn on 3.1V regulator */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECIEVER, 0x20, VUSB3V1_DEV_GRP);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECIEVER, 0, VUSB3V1_TYPE);

	/* turn on 1.5V regulator */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECIEVER, 0x20, VUSB1V5_DEV_GRP);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECIEVER, 0, VUSB1V5_TYPE);

	/* turn on 1.8V regulator */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECIEVER, 0x20, VUSB1V8_DEV_GRP);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECIEVER, 0, VUSB1V8_TYPE);

	/* disable access to power configuration registers */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECIEVER, 0, PROTECT_KEY);
}

#define METHOD	3	/* Method 1 does not work */

#if METHOD == 2
static void twl4030_usb_set_ldo_state(struct twl4030_usb *twl, int active)
{
	u8 val = 0;

	val = (active == 1) ? 0x20 : 0x0;

	/* Enable writing to power configuration registers */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0xC0, PROTECT_KEY);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0x0C, PROTECT_KEY);

	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECIEVER, val, VUSB3V1_DEV_GRP);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECIEVER, val, VUSB1V5_DEV_GRP);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECIEVER, val, VUSB1V8_DEV_GRP);

	/* disable access to power configuration registers */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECIEVER, 0, PROTECT_KEY);
}

#endif

void twl4030_phy_suspend(int controller_off)
{
	struct twl4030_usb *twl = the_transceiver;

	if (controller_off)
		usb_irq_disable();

	if (twl->asleep)
		return;

	if (!controller_off)
		/* enable rising edge interrupt to detect cable attach */
		usb_irq_enable(1, 0);

#if METHOD == 1
	twl4030_i2c_access(1);
	twl4030_usb_clear_bits(twl, FUNC_CTRL, SUSPENDM);
	twl4030_i2c_access(0);

#elif METHOD == 2
	twl4030_usb_set_ldo_state(twl, 0);

#elif METHOD == 3
	twl4030_phy_power(twl, 0);

#endif
	twl->asleep = 1;

#ifdef CONFIG_OMAP3430_ES2
	/* Release USB constraint on OFF/RET */
	if (twl->usb_power_constraint){
		constraint_remove(twl->usb_power_constraint);
	}
#endif

	return;
}
EXPORT_SYMBOL(twl4030_phy_suspend);

#if defined(CONFIG_OMAP34XX_OFFMODE)

#if defined(CONFIG_USB_MUSB_HDRC_MODULE)
	/* For Module no need to store context */
void musb_context_restore_and_wakeup( void ){}
#else
extern void musb_context_restore_and_wakeup( void );
#endif

#endif

void twl4030_phy_resume(void)
{
	struct twl4030_usb *twl = the_transceiver;

	if (!twl->asleep)
		return;

#ifdef CONFIG_OMAP3430_ES2
	/* Acquire USB constraint on OFF/RET */
	if (twl->usb_power_constraint)
		constraint_set(twl->usb_power_constraint,
					CO_LATENCY_MPUOFF_COREON);
#endif

	/* enable falling edge interrupt to detect cable detach */
	usb_irq_enable(0, 1);

#if METHOD == 1
	twl4030_i2c_access(1);
	twl4030_usb_set_bits(twl, FUNC_CTRL, SUSPENDM);
	if (twl->usb_mode == T2_USB_MODE_ULPI)
		twl4030_i2c_access(0);

#elif METHOD == 2
	twl4030_usb_set_ldo_state(twl, 1);
	twl4030_i2c_access(1);
	twl4030_usb_set_mode(twl, twl->usb_mode);
	if (twl->usb_mode == T2_USB_MODE_ULPI)
		twl4030_i2c_access(0);

#elif METHOD == 3
	twl4030_phy_power(twl, 1);
	twl4030_i2c_access(1);
	twl4030_usb_set_mode(twl, twl->usb_mode);
	if (twl->usb_mode == T2_USB_MODE_ULPI)
		twl4030_i2c_access(0);

#endif

	twl->asleep = 0;

#if defined(CONFIG_OMAP34XX_OFFMODE)
        /* Restore context of MUSB from OFF mode */
        musb_context_restore_and_wakeup();
#endif

	return;
}
EXPORT_SYMBOL(twl4030_phy_resume);

static int __init twl4030_usb_init(void)
{
	struct twl4030_usb	*twl;
	int status;
	u8 val;

	if (the_transceiver)
		return 0;

	twl = kcalloc(1, sizeof *twl, GFP_KERNEL);
	if (!twl)
		return 0;

	the_transceiver = twl;

	twl->irq = TWL4030_MODIRQ_PWR;

	usb_irq_disable();
	status = request_irq(twl->irq, twl4030_usb_irq,
		IRQF_DISABLED | IRQF_SHARED, "twl4030_usb", twl);
	if (status < 0) {
		printk(KERN_DEBUG "can't get IRQ %d, err %d\n",
			twl->irq, status);
		kfree(twl);
		return -ENODEV;
	}

#if defined (CONFIG_TWL4030_USB_HS_ULPI)
	hs_usb_init(twl);

#elif defined (CONFIG_TWL4030_USB_FS_3_PIN)
	fs_usb_init(twl);
#endif
	twl4030_usb_ldo_init(twl);
	twl4030_phy_power(twl, 1);
	twl4030_i2c_access(1);
	twl4030_usb_set_mode(twl, twl->usb_mode);


	if (twl->usb_mode == T2_USB_MODE_ULPI)
		twl4030_i2c_access(0);

	twl->asleep = 0;
#ifdef CONFIG_OMAP3430_ES2
	twl->usb_power_constraint = NULL;
#endif

	/* Allow USB presense interrupt as wakeup event */
	if (twl->usb_mode == T2_USB_MODE_ULPI){
		/* Suspend PHY but keep PRES int enabled */
		twl4030_phy_suspend(0);

		/* Check for Cold plugging case: if device already connected */
		twl4030_i2c_read_u8(TWL4030_MODULE_PM_MASTER,
					&val,
					0x0F /*STS_HW_CONDITIONS*/);
		if( (val & (1<<2)) || (val & (1<<7)) ){ /* STS_USB/STS_VBUSi */

			//Device is already connected: Cold boot
			printk(KERN_DEBUG "\tDevice ATTACHED: Cold plugging\n");
			twl4030_phy_resume();
		} else
			printk(KERN_DEBUG "\tDevice NOT-ATTACHED at bootup\n");
	}

#ifdef CONFIG_OMAP3430_ES2
	/* Get handle to USB Constraint for OFF/RETENTION */
	twl->usb_power_constraint = constraint_get("usb", &cnstr_id);
#endif

	printk(KERN_INFO "Initialized TWL4030 USB module");

	return 0;
}


static void __exit twl4030_usb_exit(void)
{
	struct twl4030_usb *twl = the_transceiver;
	int val;

	usb_irq_disable();
	free_irq(twl->irq, twl);

	/* set transceiver mode to power on defaults */
	twl4030_usb_set_mode(twl, -1);

	/* autogate 60MHz ULPI clock,
	 * clear dpll clock request for i2c access,
	 * disable 32KHz
	 */
	if ((val = twl4030_usb_read(PHY_CLK_CTRL)) >= 0) {
		val |= CLOCKGATING_EN;
		val &= ~(CLK32K_EN | REQ_PHY_DPLL_CLK);
		twl4030_usb_write(PHY_CLK_CTRL, (u8)val);
	}

	/* disable complete OTG block */
	twl4030_usb_clear_bits(twl, POWER_CTRL, OTG_ENAB);

	twl4030_phy_power(twl, 0);

#ifdef CONFIG_OMAP3430_ES2
	if (twl->usb_power_constraint)
		constraint_put(twl->usb_power_constraint);
#endif

	kfree(twl);
}

subsys_initcall(twl4030_usb_init);
module_exit(twl4030_usb_exit);

MODULE_AUTHOR("Texas Instruments, Inc.");
MODULE_DESCRIPTION("TWL4030 USB transceiver driver");
MODULE_LICENSE("GPL");
