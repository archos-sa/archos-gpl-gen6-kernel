#include <linux/types.h>
#include <linux/sysdev.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/module.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/arch/battery.h>
#include <asm/mach-types.h>
#include <asm/errno.h>
#include <asm/arch/mux.h>
#include <asm/arch/gpio.h>
#include <asm/arch/board.h>
#include <asm/arch/archosg6-gpio.h>

#include "linux/msp430-io.h"
#include "linux/msp430.h"

#define DBG if (1)

static int charge_state;
static int charge_level;

static int _usb;
static int _dcin;
static int _usb_level;
static struct g6_gpio charge_gpio;

static void update_charge_state(void);

static void set_stop_chg(int on_off)
{
	if ( _PIN_NB( charge_gpio) != 0)
		omap_set_gpio_dataout( _PIN_NB( charge_gpio ), !!on_off );
}

#ifdef CONFIG_PM
static int battery_suspend(struct sys_device* dev, pm_message_t state)
{
	set_stop_chg(1);
	return 0;
}

static int battery_resume(struct sys_device* dev)
{
	set_stop_chg(!charge_state);
	return 0;
}
#endif

static struct sysdev_class omap_battery_sysclass = {
	set_kset_name("battery"),
#ifdef CONFIG_PM
	.suspend	= battery_suspend,
	.resume		= battery_resume,
#endif
};

static struct sys_device omap_battery0_device = {
	.id		= 0,
	.cls		= &omap_battery_sysclass,
};

static ssize_t show_battery0_charge_state(struct sys_device* dev, char* buf)
{
	return sprintf(buf, "%d\n", _usb_level); 
}

static ssize_t show_battery0_charge_level(struct sys_device* dev, char* buf)
{
	return sprintf(buf, "%d\n", charge_level); 
}

static struct sysdev_attribute attr_battery0_charge_state = {
	.attr = { .name = "charge_state", .mode = S_IRUGO|S_IWUSR },
	.show = show_battery0_charge_state,
};

static struct sysdev_attribute attr_battery0_charge_level = {
	.attr = { .name = "charge_level", .mode = S_IRUGO|S_IWUSR },
	.show = show_battery0_charge_level,
};


static void enable_high_charging(void)
{
#if defined (CONFIG_MACH_ARCHOS_G6) && !defined (CONFIG_MACH_OMAP_3430SDP)
	if (!machine_is_archos_g6tv()) {
		set_stop_chg(0);
		atmega_io_set_charge_mode(CHARGER_ON_HIGH);
	}
#endif

	charge_level=2;
	charge_state=1;
	
	kobject_uevent( &omap_battery0_device.kobj, KOBJ_CHANGE);
}

static void enable_low_charging(void)
{
#if defined (CONFIG_MACH_ARCHOS_G6) && !defined (CONFIG_MACH_OMAP_3430SDP)
	if (!machine_is_archos_g6tv()) {
		set_stop_chg(1);
		atmega_io_set_charge_mode(CHARGER_OFF);
		atmega_io_set_charge_mode(CHARGER_ON_LOW);
		set_stop_chg(0);
	}
#endif

	charge_level=1;
	charge_state=1;
	kobject_uevent( &omap_battery0_device.kobj, KOBJ_CHANGE);
}

static void disable_charging(void)
{
	
#if defined (CONFIG_MACH_ARCHOS_G6) && !defined (CONFIG_MACH_OMAP_3430SDP)
	if (!machine_is_archos_g6tv()) {
		set_stop_chg(1);
		atmega_io_set_charge_mode(CHARGER_OFF);
	}
#endif

	charge_state = 0;
	charge_level = 0;
	kobject_uevent( &omap_battery0_device.kobj, KOBJ_CHANGE);
}

static int is_module_capable_high_charging( unsigned long usb_type )
{	
	switch (usb_type) {
		// List of modules supporting high charging
		case MODULE_ID_TV_CRADLE:
		case MODULE_ID_VRA_GEN6:
		case MODULE_ID_BATTERY_DOCK_GEN6:
		case MODULE_ID_MUSIC_DOCK:
		case MODULE_ID_MINI_DOCK:
		case MODULE_ID_DVBT_SNAP_ON:
		case MODULE_ID_SERIAL_ADAPTER:
		case MODULE_ID_GPS_WITHOUT_TMC:
		case MODULE_ID_GPS_WITH_TMC:
		case MODULE_ID_POWER_CABLE:
			return 1;

		default:
			break;
	}
	return 0;
}

// verification at the USB side for autorisation of the the charge
static int usb_charge_level(void)
{
	return _usb_level;
}

static void update_charge_state(void)
{
	int usb_type = 0;

	if( _usb && !_dcin ){
		// usb without DC-in 
		// no charge with only usb & can't happen when plugging the PC cable
		disable_charging();
		return;
	}

	if( _dcin && !_usb ){
		// only DC-in non usb; always high charging
		if ( machine_is_archos_g6l()) 
			enable_high_charging();
		else {
			// ! warning ! this case should appears with pc_cable during plug or with
			// not complete connection, don't enable high charge in this case
			// enable high charge only if an allowed module is identified
			usb_type = atmega_io_getUsbType();
DBG 			printk("usb_type=%d\n",usb_type);

			if ( is_module_capable_high_charging(usb_type) ) {
				enable_high_charging();
			}
		}
		return;
	}

	if( _usb && _dcin ){
		// Means normally that both the USB cable and DC-in are plugged.
		// But be careful that both flags are also enabled when plugging the PC cable
		usb_type = atmega_io_getUsbType();
DBG 		printk("usb_type=%d\n",usb_type);

		if ( is_module_capable_high_charging(usb_type) ) {
			enable_high_charging();
		} else {
			if ( usb_charge_level() ) {
				enable_high_charging();
			} else {
				enable_low_charging();
			}
		}	
		return;
	}

	if( charge_state && ( !_usb && !_dcin )){
		disable_charging();	
		return;
	}
}

/* when usb_need is not 0 or 1, we keep it's value */

void archos_needs_battery(int usb_need, int dcin_need)
{
DBG 	printk("archos_needs_battery usb: %d dcin:%d\n",usb_need,dcin_need);

	if ( usb_need == 1 || usb_need == 0 )
		_usb = usb_need;

	if ( dcin_need == 1 || dcin_need == 0 )
		_dcin = dcin_need;

// printk("[BATTERY], usb: %d, dc_in: %d\n",_usb,_dcin);

	_usb_level = 0;


DBG 	printk("[BATTERY], usb: %d, dc_in: %d\n",_usb,_dcin);

	update_charge_state();
}

void archos_usb_high_charge(int charge)
{
	_usb_level = charge;

DBG 	printk("[BATTERY], usb: %d, dc_in: %d _usb_level: %d\n",_usb,_dcin, _usb_level);

	update_charge_state();
}

static int __init omap_battery_init_devicefs(void)
{
	const struct archosg6_charge_config *charge_cfg;
	int ret;

	sysdev_class_register(&omap_battery_sysclass);
	
	ret = sysdev_register(&omap_battery0_device);
	if (ret < 0)
		return ret;

	sysdev_create_file(&omap_battery0_device, &attr_battery0_charge_state);
	sysdev_create_file(&omap_battery0_device, &attr_battery0_charge_level);

	/* charge pin */
	charge_cfg = omap_get_config( ARCHOS_TAG_CHARGE, struct archosg6_charge_config );
	if (charge_cfg && (hardware_rev < charge_cfg->nrev)) {
		charge_gpio = charge_cfg->rev[hardware_rev];
		_INIT_OUTPUT( charge_gpio );
		omap_set_gpio_dataout( _PIN_NB( charge_gpio ), 1);	
	} else
		printk(KERN_DEBUG "archosg6_batt_init: no board configuration found\n");
	
	return 0;
}

arch_initcall(omap_battery_init_devicefs);

EXPORT_SYMBOL(archos_needs_battery);
EXPORT_SYMBOL(archos_usb_high_charge);
