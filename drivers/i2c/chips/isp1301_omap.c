/*
 * isp1301_omap - ISP 1301 USB transceiver, talking to OMAP OTG controller
 *
 * Copyright (C) 2004 Texas Instruments
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
 */
#undef	DEBUG
#undef	VERBOSE

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/usb/ch9.h>
#include <linux/usb_gadget.h>
#include <linux/usb.h>
#include <linux/usb/otg.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>

#include <asm/irq.h>
#include <asm/mach-types.h>

#include <asm/arch/gpio.h>
#include <asm/arch/usb.h>
#include <asm/arch/mux.h>


#ifndef	DEBUG
#undef	VERBOSE
#endif

//#define VERBOSE
//#define DEBUG
/* For 3430: OHCI does not have OTG */
#undef CONFIG_USB_OTG

/* INT line for ISP1301 for Port1 */
/* Port1: GPIO_22: Input on pin ETK_D8: mode 4 */
#define OTG_XCEIV_INT	(22)
/* Port2: GPIO_138: Input on pin MMC2_dat6: mode 4 */
#define OTG_XCEIV_INT_3	(138)

#define	DRIVER_VERSION	"24 August 2004"
#define	DRIVER_NAME	(isp1301_driver.driver.name)

MODULE_DESCRIPTION("ISP1301 USB OTG Transceiver Driver");
MODULE_LICENSE("GPL");

struct isp1301 {
	struct otg_transceiver	otg;
	struct i2c_client	client;
	void			(*i2c_release)(struct device *dev);

	int			irq;
	int			irq_type;
	u8 			usb_mode;	/* pin configuration */
#		define	DAT_SE0_UNIDIR	0
#		define	DAT_SE0_BIDIR	1
#		define	VP_VM_UNIDIR	2
#		define	VP_VM_BIDIR	3

	u32			last_otg_ctrl;
	unsigned		working:1;

	struct timer_list	timer;

	/* use keventd context to change the state for us */
	struct work_struct	work;

	unsigned long		todo;
#		define WORK_UPDATE_ISP	0	/* update ISP from OTG */
#		define WORK_UPDATE_OTG	1	/* update OTG from ISP */
#		define WORK_HOST_RESUME	4	/* resume host */
#		define WORK_TIMER	6	/* timer fired */
#		define WORK_STOP	7	/* don't resubmit */
};

/*-------------------------------------------------------------------------*/

/* products will deliver OTG messages with LEDs, GUI, etc */
static inline void notresponding(struct isp1301 *isp)
{
	printk(KERN_NOTICE "OTG device not responding.\n");
}

/*-------------------------------------------------------------------------*/

/* only two addresses possible */
#define	ISP_BASE		0x2c
static unsigned short normal_i2c[] = {
	ISP_BASE, ISP_BASE + 1,
	I2C_CLIENT_END };

I2C_CLIENT_INSMOD;

static struct i2c_driver isp1301_driver;

/* smbus apis are used for portability */

static inline u8
isp1301_get_u8(struct isp1301 *isp, u8 reg)
{
	return i2c_smbus_read_byte_data(&isp->client, reg + 0);
}

static inline int
isp1301_get_u16(struct isp1301 *isp, u8 reg)
{
	return i2c_smbus_read_word_data(&isp->client, reg);
}

static inline int
isp1301_set_bits(struct isp1301 *isp, u8 reg, u8 bits)
{
	return i2c_smbus_write_byte_data(&isp->client, reg + 0, bits);
}

static inline int
isp1301_clear_bits(struct isp1301 *isp, u8 reg, u8 bits)
{
	return i2c_smbus_write_byte_data(&isp->client, reg + 1, bits);
}

/*-------------------------------------------------------------------------*/

/* identification */
#define	ISP1301_VENDOR_ID		0x00	/* u16 read */
#define	ISP1301_PRODUCT_ID		0x02	/* u16 read */
#define	ISP1301_BCD_DEVICE		0x14	/* u16 read */

#define	I2C_VENDOR_ID_PHILIPS		0x04cc
#define	I2C_PRODUCT_ID_PHILIPS_1301	0x1301

/* operational registers */
#define	ISP1301_MODE_CONTROL_1		0x04	/* u8 read, set, +1 clear */
#	define	MC1_SPEED_REG		(1 << 0)
#	define	MC1_SUSPEND_REG		(1 << 1)
#	define	MC1_DAT_SE0		(1 << 2)
#	define	MC1_TRANSPARENT		(1 << 3)
#	define	MC1_BDIS_ACON_EN	(1 << 4)
#	define	MC1_OE_INT_EN		(1 << 5)
#	define	MC1_UART_EN		(1 << 6)
#	define	MC1_MASK		0x7f
#define	ISP1301_MODE_CONTROL_2		0x12	/* u8 read, set, +1 clear */
#	define	MC2_GLOBAL_PWR_DN	(1 << 0)
#	define	MC2_SPD_SUSP_CTRL	(1 << 1)
#	define	MC2_BI_DI		(1 << 2)
#	define	MC2_TRANSP_BDIR0	(1 << 3)
#	define	MC2_TRANSP_BDIR1	(1 << 4)
#	define	MC2_AUDIO_EN		(1 << 5)
#	define	MC2_PSW_EN		(1 << 6)
#	define	MC2_EN2V7		(1 << 7)
#define	ISP1301_OTG_CONTROL_1		0x06	/* u8 read, set, +1 clear */
#	define	OTG1_DP_PULLUP		(1 << 0)
#	define	OTG1_DM_PULLUP		(1 << 1)
#	define	OTG1_DP_PULLDOWN	(1 << 2)
#	define	OTG1_DM_PULLDOWN	(1 << 3)
#	define	OTG1_ID_PULLDOWN	(1 << 4)
#	define	OTG1_VBUS_DRV		(1 << 5)
#	define	OTG1_VBUS_DISCHRG	(1 << 6)
#	define	OTG1_VBUS_CHRG		(1 << 7)
#define	ISP1301_OTG_STATUS		0x10	/* u8 readonly */
#	define	OTG_B_SESS_END		(1 << 6)
#	define	OTG_B_SESS_VLD		(1 << 7)

#define	ISP1301_INTERRUPT_SOURCE	0x08	/* u8 read */
#define	ISP1301_INTERRUPT_LATCH		0x0A	/* u8 read, set, +1 clear */

#define	ISP1301_INTERRUPT_FALLING	0x0C	/* u8 read, set, +1 clear */
#define	ISP1301_INTERRUPT_RISING	0x0E	/* u8 read, set, +1 clear */

/* same bitfields in all interrupt registers */
#	define	INTR_VBUS_VLD		(1 << 0)
#	define	INTR_SESS_VLD		(1 << 1)
#	define	INTR_DP_HI		(1 << 2)
#	define	INTR_ID_GND		(1 << 3)
#	define	INTR_DM_HI		(1 << 4)
#	define	INTR_ID_FLOAT		(1 << 5)
#	define	INTR_BDIS_ACON		(1 << 6)
#	define	INTR_CR_INT		(1 << 7)

/*-------------------------------------------------------------------------*/

static const char *state_string(enum usb_otg_state state)
{
	switch (state) {
	case OTG_STATE_A_IDLE:		return "a_idle";
	case OTG_STATE_A_WAIT_VRISE:	return "a_wait_vrise";
	case OTG_STATE_A_WAIT_BCON:	return "a_wait_bcon";
	case OTG_STATE_A_HOST:		return "a_host";
	case OTG_STATE_A_SUSPEND:	return "a_suspend";
	case OTG_STATE_A_PERIPHERAL:	return "a_peripheral";
	case OTG_STATE_A_WAIT_VFALL:	return "a_wait_vfall";
	case OTG_STATE_A_VBUS_ERR:	return "a_vbus_err";
	case OTG_STATE_B_IDLE:		return "b_idle";
	case OTG_STATE_B_SRP_INIT:	return "b_srp_init";
	case OTG_STATE_B_PERIPHERAL:	return "b_peripheral";
	case OTG_STATE_B_WAIT_ACON:	return "b_wait_acon";
	case OTG_STATE_B_HOST:		return "b_host";
	default:			return "UNDEFINED";
	}
}

static inline const char *state_name(struct isp1301 *isp)
{
	return state_string(isp->otg.state);
}

#ifdef	VERBOSE
#define	dev_vdbg			dev_dbg
#else
#define	dev_vdbg(dev, fmt, arg...)	do{}while(0)
#endif

/*-------------------------------------------------------------------------*/

#define	TIMER_MINUTES	10
#define	TIMER_JIFFIES	(TIMER_MINUTES * 60 * HZ)

/* Almost all our I2C messaging comes from a work queue's task context.
 * NOTE: guaranteeing certain response times might mean we shouldn't
 * share keventd's work queue; a realtime task might be safest.
 */
void
isp1301_defer_work(struct isp1301 *isp, int work)
{
	int status;

	if (isp && !test_and_set_bit(work, &isp->todo)) {
		(void) get_device(&isp->client.dev);
		status = schedule_work(&isp->work);
		if (!status && !isp->working)
			dev_vdbg(&isp->client.dev,
				"work item %d may be lost\n", work);
	}
}

static ssize_t show_registers(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 ctrl, status, src;
	struct isp1301 *isp; 
	
	isp=container_of(dev, struct isp1301, client.dev);

	ctrl = isp1301_get_u8(isp, ISP1301_OTG_CONTROL_1);
	status = isp1301_get_u8(isp, ISP1301_OTG_STATUS);
	src = isp1301_get_u8(isp, ISP1301_INTERRUPT_SOURCE);

        return sprintf(buf, "ctrl/%02x stat/%02x src/%02x \n",
			ctrl, status, src);
}
static DEVICE_ATTR(registers, S_IRUGO, show_registers, NULL);
 
static void
dump_regs(struct isp1301 *isp, const char *label)
{
#ifdef	DEBUG
	u8	ctrl = isp1301_get_u8(isp, ISP1301_OTG_CONTROL_1);
	u8	status = isp1301_get_u8(isp, ISP1301_OTG_STATUS);
	u8	src = isp1301_get_u8(isp, ISP1301_INTERRUPT_SOURCE);

	pr_debug("otg: %s %s, otg/%02x stat/%02x.%02x\n",
			label, state_name(isp),
		ctrl, status, src);
	/* mode control and irq enables don't change much */
#endif
}

/*-------------------------------------------------------------------------*/

static u8 isp1301_clear_latch(struct isp1301 *isp)
{
	u8 latch = isp1301_get_u8(isp, ISP1301_INTERRUPT_LATCH);
	isp1301_clear_bits(isp, ISP1301_INTERRUPT_LATCH, latch);
	return latch;
}

static void
isp1301_work(struct work_struct *work)
{
	struct isp1301	*isp = container_of(work, struct isp1301, work);
	u8 isp_stat=0;

	isp_stat=isp1301_get_u8(isp, ISP1301_INTERRUPT_LATCH);

	printk("\n ISR Latch[%x]!!!\n", isp_stat);
	isp1301_clear_latch(isp);

	if (isp_stat & INTR_ID_GND) {
		printk("\n [isp1301]: ID-PIN-GND\n");
		if (isp_stat & INTR_VBUS_VLD) {
			isp->otg.state = OTG_STATE_A_HOST;
			printk("\n[isp1301]: A_HOST now");
			isp1301_set_bits(isp, ISP1301_INTERRUPT_RISING,
							INTR_DP_HI|INTR_DM_HI);
			isp1301_set_bits(isp, ISP1301_INTERRUPT_FALLING,
							INTR_DP_HI|INTR_DM_HI);
		}
	}

	if( isp1301_get_u8(isp, ISP1301_INTERRUPT_LATCH) & (INTR_DP_HI|INTR_DM_HI) ){

		printk("\n[isp1301]: CABLE ATTACHED \n");

        //omap_writel( 0x301807b, 0x48062040 ); 
        //omap_writel( 0x301807b, 0x48062044 );
        //omap_writel( 0x301807b, 0x48062048 );

	return;
		
	}

	return;
}

static irqreturn_t isp1301_irq(int irq, void *isp)
{
	isp1301_defer_work(isp, WORK_UPDATE_OTG);
	return IRQ_HANDLED;
}

static void isp1301_timer(unsigned long _isp)
{
	isp1301_defer_work((void *)_isp, WORK_TIMER);
}

/*-------------------------------------------------------------------------*/

static void isp1301_release(struct device *dev)
{
	struct isp1301	*isp;

	isp = container_of(dev, struct isp1301, client.dev);

	/* ugly -- i2c hijacks our memory hook to wait_for_completion() */
	if (isp->i2c_release)
		isp->i2c_release(dev);
	kfree (isp);
}

static struct isp1301 *the_transceiver;

static int isp1301_detach_client(struct i2c_client *i2c)
{
	struct isp1301	*isp;

	isp = container_of(i2c, struct isp1301, client);

	isp1301_clear_bits(isp, ISP1301_INTERRUPT_FALLING, ~0);
	isp1301_clear_bits(isp, ISP1301_INTERRUPT_RISING, ~0);
	free_irq(isp->irq, isp);

	if (machine_is_omap_h4() || machine_is_omap_3430sdp())
		omap_free_gpio(OTG_XCEIV_INT);

	isp->timer.data = 0;
	set_bit(WORK_STOP, &isp->todo);
	del_timer_sync(&isp->timer);
	flush_scheduled_work();

	put_device(&i2c->dev);
	the_transceiver = 0;

	return i2c_detach_client(i2c);
}

/* add or disable the host device+driver */
static int
isp1301_set_host(struct otg_transceiver *otg, struct usb_bus *host)
{
	struct isp1301	*isp = container_of(otg, struct isp1301, otg);

	if (!otg || isp != the_transceiver)
		return -ENODEV;

	if (!host) {
		isp->otg.host = 0;
		return 0;
	}

	// FIXME update its refcount
	isp->otg.host = host;

	dev_info(&isp->client.dev, "A-Host sessions ok\n");
	printk("\nISP1301: A-Host sessions ok\n");
	isp1301_set_bits(isp, ISP1301_INTERRUPT_RISING,
		INTR_ID_GND);
	isp1301_set_bits(isp, ISP1301_INTERRUPT_FALLING,
		INTR_ID_GND);

	/* If this has a Mini-AB connector, this mode is highly
	 * nonstandard ... but can be handy for testing, especially with
	 * the Mini-A end of an OTG cable.  (Or something nonstandard
	 * like MiniB-to-StandardB, maybe built with a gender mender.)
	 */
	isp1301_set_bits(isp, ISP1301_OTG_CONTROL_1, OTG1_VBUS_DRV);

	dump_regs(isp, __FUNCTION__);

	return 0;
}

/*-------------------------------------------------------------------------*/

/* no error returns, they'd just make bus scanning stop */
static int isp1301_probe(struct i2c_adapter *bus, int address, int kind)
{
	int			status;
	struct isp1301		*isp;
	struct i2c_client	*i2c;

	if (the_transceiver)
		return 0;

	isp = kzalloc(sizeof *isp, GFP_KERNEL);
	if (!isp)
		return 0;

	memset(isp, 0, sizeof(*isp));

	INIT_WORK(&isp->work, isp1301_work);
	init_timer(&isp->timer);
	isp->timer.function = isp1301_timer;
	isp->timer.data = (unsigned long) isp;

	isp->irq = -1;
	isp->irq_type = IRQF_DISABLED;
	isp->client.addr = address;
	i2c_set_clientdata(&isp->client, isp);
	isp->client.adapter = bus;
	isp->client.driver = &isp1301_driver;
	strlcpy(isp->client.name, DRIVER_NAME, I2C_NAME_SIZE);
	i2c = &isp->client;

	/* if this is a true probe, verify the chip ... */
	if (kind < 0) {

printk( "\n-->ctrl %x stats %x src %x\n", 
        isp1301_get_u8(isp, ISP1301_OTG_CONTROL_1),
        isp1301_get_u8(isp, ISP1301_OTG_STATUS),
        isp1301_get_u8(isp, ISP1301_INTERRUPT_SOURCE));


		status = isp1301_get_u16(isp, ISP1301_VENDOR_ID);
		if (status != I2C_VENDOR_ID_PHILIPS) {
			dev_dbg(&bus->dev, "addr %d not philips id: %d\n",
				address, status);
			goto fail1;
		}
		status = isp1301_get_u16(isp, ISP1301_PRODUCT_ID);
		if (status != I2C_PRODUCT_ID_PHILIPS_1301) {
			dev_dbg(&bus->dev, "%d not isp1301, %d\n",
				address, status);
			goto fail1;
		}
	}

	status = i2c_attach_client(i2c);
	if (status < 0) {
		dev_dbg(&bus->dev, "can't attach %s to device %d, err %d\n",
				DRIVER_NAME, address, status);
fail1:
		kfree(isp);
		return 0;
	}
	isp->i2c_release = i2c->dev.release;
	i2c->dev.release = isp1301_release;

	/* initial development used chiprev 2.00 */
	status = i2c_smbus_read_word_data(i2c, ISP1301_BCD_DEVICE);
	dev_info(&i2c->dev, "chiprev %x.%02x, driver " DRIVER_VERSION "\n",
		status >> 8, status & 0xff);

	/* make like power-on reset */
	isp1301_clear_bits(isp, ISP1301_MODE_CONTROL_1, MC1_MASK);

	isp1301_set_bits(isp, ISP1301_MODE_CONTROL_2, MC2_BI_DI);
	isp1301_clear_bits(isp, ISP1301_MODE_CONTROL_2, ~MC2_BI_DI);

	isp1301_set_bits(isp, ISP1301_OTG_CONTROL_1,
				OTG1_DM_PULLDOWN | OTG1_DP_PULLDOWN);
	isp1301_clear_bits(isp, ISP1301_OTG_CONTROL_1,
				~(OTG1_DM_PULLDOWN | OTG1_DP_PULLDOWN));

	isp1301_clear_bits(isp, ISP1301_INTERRUPT_LATCH, ~0);
	isp1301_clear_bits(isp, ISP1301_INTERRUPT_FALLING, ~0);
	isp1301_clear_bits(isp, ISP1301_INTERRUPT_RISING, ~0);

	if (machine_is_omap_h2() || machine_is_omap_h3() ||
	    machine_is_omap_h4() || machine_is_omap_3430sdp())
		isp->usb_mode = DAT_SE0_BIDIR;


	/* For 3430 ES2.0 */
	/* TODO: MUX canges for ISP1301 Port1 */
	if (machine_is_omap_3430sdp()) {

		/* full speed signaling by default */
		isp1301_set_bits(isp, ISP1301_MODE_CONTROL_1,
			MC1_SPEED_REG);
		isp1301_set_bits(isp, ISP1301_MODE_CONTROL_2,
			MC2_SPD_SUSP_CTRL);


		// TODO : FUNC MUX for GPIO line, 
		// Where to do the mux for other pins !!
		omap_cfg_reg(AA14_3430_ISP1301_GPIO22);

		isp->irq = OMAP_GPIO_IRQ(OTG_XCEIV_INT);
		omap_request_gpio(OTG_XCEIV_INT);
		omap_set_gpio_direction(OTG_XCEIV_INT, 1);
		isp->irq_type = IRQF_TRIGGER_FALLING;
		//omap_set_gpio_edge_ctrl(OTG_XCEIV_INT, OMAP_GPIO_FALLING_EDGE);
	}
	status = request_irq(isp->irq, isp1301_irq,
			isp->irq_type, DRIVER_NAME, isp);
	if (status < 0) {
		dev_dbg(&i2c->dev, "can't get IRQ %d, err %d\n",
				isp->irq, status);
#ifdef	CONFIG_USB_OTG
fail2:
#endif
		i2c_detach_client(i2c);
		goto fail1;
	}

	isp->otg.dev = &isp->client.dev;
	isp->otg.label = DRIVER_NAME;

	isp->otg.set_host = isp1301_set_host,

	the_transceiver = isp;

	dump_regs(isp, __FUNCTION__);
	status = device_create_file(&isp->client.dev, &dev_attr_registers);
	if (status <0)
		return status;

#ifdef	VERBOSE
	mod_timer(&isp->timer, jiffies + TIMER_JIFFIES);
	dev_dbg(&i2c->dev, "scheduled timer, %d min\n", TIMER_MINUTES);
#endif

	status = otg_set_transceiver(&isp->otg);
	if (status < 0)
		dev_err(&i2c->dev, "can't register transceiver, %d\n",
			status);

	//Some defaults
	isp->otg.default_a = 1;
	isp->otg.state = OTG_STATE_A_IDLE;

	//Clear suspend	
        isp1301_clear_bits(isp, ISP1301_MODE_CONTROL_1, MC1_SUSPEND_REG);

	// DAT_SE0: Uni-dir
	isp1301_set_bits(isp, ISP1301_MODE_CONTROL_1, MC1_DAT_SE0);

        isp1301_set_bits(isp, ISP1301_INTERRUPT_RISING,
                INTR_ID_GND);
        isp1301_set_bits(isp, ISP1301_INTERRUPT_FALLING,
                INTR_ID_GND);

        isp1301_set_bits(isp, ISP1301_OTG_CONTROL_1, OTG1_VBUS_DRV);

	return 0;
}

static int isp1301_scan_bus(struct i2c_adapter *bus)
{
	if (!i2c_check_functionality(bus, I2C_FUNC_SMBUS_BYTE_DATA
			| I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EINVAL;
	if (cpu_is_omap2420() || cpu_is_omap3430())
		return isp1301_probe(bus, 0x2C, -1);
			// address = 0x2d ;ISP address on H4,
			// kind = -1 ; to look like a true probe

	return i2c_probe(bus, &addr_data, isp1301_probe);
}

static struct i2c_driver isp1301_driver = {
	.driver = {
		.name	= "isp1301_omap",
	},
	.attach_adapter	= isp1301_scan_bus,
	.detach_client	= isp1301_detach_client,
};

/*-------------------------------------------------------------------------*/

static int __init isp_init(void)
{
	return i2c_add_driver(&isp1301_driver);
}
module_init(isp_init);

static void __exit isp_exit(void)
{
	if (the_transceiver)
		otg_set_transceiver(0);
	i2c_del_driver(&isp1301_driver);
}
module_exit(isp_exit);

