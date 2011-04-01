#include <linux/types.h>
#include <linux/sysdev.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/module.h>
#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#include <asm/mach-types.h>
#include <linux/hsdpa.h>
#include <linux/delay.h>
#include <asm/arch/archosg6-gpio.h>

static int power_on_gpio = 0;
static int reset_gpio = 0;
static int reset_cycle = 0;

struct board_hsdpa_gpio {
	struct g6_gpio hsdpa_power_on;
	struct g6_gpio hsdpa_reset;
};

static struct board_hsdpa_gpio g6plus_hsdpa_gpio = {
	.hsdpa_power_on = { 154, AD2_3430_GPIO154 },
	.hsdpa_reset = { 136, AE4_3430_GPIO136 }
};

static struct board_hsdpa_gpio *hsdpa_gpio = NULL;

static void _do_reset_cycle( void )
{
	if (hsdpa_gpio != NULL && _PIN_NB( hsdpa_gpio->hsdpa_reset )) {
		printk(KERN_INFO "HSDPA resetting\n");
		omap_set_gpio_dataout( _PIN_NB( hsdpa_gpio->hsdpa_reset ), 1 );
		msleep(200);
		omap_set_gpio_dataout( _PIN_NB( hsdpa_gpio->hsdpa_reset ), 0 );
	}
}

#ifdef CONFIG_PM
static int hsdpa_suspend(struct sys_device* dev, pm_message_t state)
{
	printk(KERN_DEBUG "hsdpa_suspend\n");
	if (hsdpa_gpio != NULL && _PIN_NB( hsdpa_gpio->hsdpa_reset )) {
		omap_set_gpio_dataout( _PIN_NB( hsdpa_gpio->hsdpa_reset ), 1 );
	}

 	if (machine_is_archos_g6plus() && _PIN_NB(hsdpa_gpio->hsdpa_power_on)) {
 		omap_set_gpio_dataout( _PIN_NB(hsdpa_gpio->hsdpa_power_on), 1 );
 	}

		
	return 0;
}

static int hsdpa_resume(struct sys_device* dev)
{
	printk(KERN_DEBUG "hsdpa_resume\n");
 	if (machine_is_archos_g6plus() && _PIN_NB(hsdpa_gpio->hsdpa_power_on)) {
 		omap_set_gpio_dataout( _PIN_NB(hsdpa_gpio->hsdpa_power_on), 0 );
 	}

	if (hsdpa_gpio != NULL && _PIN_NB( hsdpa_gpio->hsdpa_reset )) {
		omap_set_gpio_dataout( _PIN_NB( hsdpa_gpio->hsdpa_reset ), 0 );
	}		
	return 0;	
}
#endif

static struct sysdev_class hsdpa_sysclass = {
	set_kset_name("hsdpa"),
#ifdef CONFIG_PM
	.suspend	= hsdpa_suspend,
	.resume		= hsdpa_resume,
#endif
};

static struct sys_device hsdpa_device = {
	.id		= 0,
	.cls		= &hsdpa_sysclass,
};


static void _power_on( int high_level )
{
	if ( machine_is_archos_g6plus() &&  _PIN_NB( hsdpa_gpio->hsdpa_power_on ) ) {
		printk("%s value %d\n",__FUNCTION__, high_level);
		if ( high_level )
			omap_set_gpio_dataout( _PIN_NB( hsdpa_gpio->hsdpa_power_on ), 1 );
		else
			omap_set_gpio_dataout( _PIN_NB( hsdpa_gpio->hsdpa_power_on ), 0 );
	}

}

static void _reset( int high_level )
{
	if ( machine_is_archos_g6plus() && _PIN_NB( hsdpa_gpio->hsdpa_reset )) {
		printk("%s value %d\n",__FUNCTION__, high_level);
		if ( high_level )
			omap_set_gpio_dataout( _PIN_NB( hsdpa_gpio->hsdpa_reset ), 1 );
		else
			omap_set_gpio_dataout( _PIN_NB( hsdpa_gpio->hsdpa_reset ), 0 );
	}

}

static ssize_t show_hsdpa_power_on_gpio(struct sys_device* dev, char* buf)
{
	return sprintf(buf, "%d\n", power_on_gpio); 
}

static ssize_t set_hsdpa_power_on_gpio(struct sys_device* dev, const char* buf, size_t len)
{
	int on_off = simple_strtol(buf, NULL, 10);
	
	if( on_off > 1 || on_off < 0 )
		return -EINVAL;

	if (power_on_gpio == on_off)
		return len;

	power_on_gpio = on_off;
	printk(KERN_DEBUG  "set_hsdpa_power_on_gpio %d\n",on_off);
	_power_on(on_off);
	return len;
}

static struct sysdev_attribute attr_hsdpa_power_on_gpio = {
	.attr  = { .name = "powergpio", .mode = S_IRUGO|S_IWUSR },
	.show  = show_hsdpa_power_on_gpio,
	.store = set_hsdpa_power_on_gpio,
};

static ssize_t show_hsdpa_reset_gpio(struct sys_device* dev, char* buf)
{
	return sprintf(buf, "%d\n", reset_gpio); 
}

static ssize_t set_hsdpa_reset_gpio(struct sys_device* dev, const char* buf, size_t len)
{
	int on_off = simple_strtol(buf, NULL, 10);
	
	if( on_off > 1 || on_off < 0 )
		return -EINVAL;

	if (reset_gpio == on_off)
		return len;

	reset_gpio = on_off;
	printk(KERN_DEBUG  "set_hsdpa_reset_gpio %d\n",on_off);
	_reset(on_off);
	return len;
}

static struct sysdev_attribute attr_hsdpa_reset_gpio = {
	.attr  = { .name = "resetgpio", .mode = S_IRUGO|S_IWUSR },
	.show  = show_hsdpa_reset_gpio,
	.store = set_hsdpa_reset_gpio,
};

static ssize_t show_hsdpa_reset_cycle(struct sys_device* dev, char* buf)
{
	return sprintf(buf, "%d\n", reset_cycle); 
}

static ssize_t set_hsdpa_reset_cycle(struct sys_device* dev, const char* buf, size_t len)
{
	int on_off = simple_strtol(buf, NULL, 10);
	
	if( on_off > 1 || on_off < 0 )
		return -EINVAL;

	if (reset_cycle == on_off)
		return len;

	reset_cycle = on_off;
	printk(KERN_DEBUG "set_hsdpa_reset_gpio %d\n",on_off);
	if (reset_cycle) {
		_do_reset_cycle();
	}
	reset_cycle = 0; 
	return len;
}

static struct sysdev_attribute attr_hsdpa_reset_cycle = {
	.attr  = { .name = "resetcycle", .mode = S_IRUGO|S_IWUSR },
	.show  = show_hsdpa_reset_cycle,
	.store = set_hsdpa_reset_cycle,
};

int __init archosg6_hsdpa_init(void)
{
	int ret;
	printk(KERN_INFO "HSDPA start init\n");
// mux init for hsdpa will be move in boot
	if ( machine_is_archos_g6plus() ) {
		hsdpa_gpio = &g6plus_hsdpa_gpio;
	} else {
		printk(KERN_ERR "HSDPA GPIO init error, bad machine type\n");
		return 0;
	}

	/* sysfs setup */
	sysdev_class_register(&hsdpa_sysclass);
	ret = sysdev_register(&hsdpa_device);
	if (ret)
		return ret;	

	/* hsdpa reset */
	if ( _PIN_NB( hsdpa_gpio->hsdpa_reset ) ) {
		reset_gpio = 1;
		sysdev_create_file(&hsdpa_device, &attr_hsdpa_reset_gpio);
		_INIT_OUTPUT(hsdpa_gpio->hsdpa_reset);
		omap_set_gpio_dataout(_PIN_NB(hsdpa_gpio->hsdpa_reset), 1);
	}

	/* hsdpa power on */
	if ( _PIN_NB( hsdpa_gpio->hsdpa_power_on ) ) {
		power_on_gpio = 1;
		sysdev_create_file(&hsdpa_device, &attr_hsdpa_power_on_gpio);
		_INIT_OUTPUT(hsdpa_gpio->hsdpa_power_on);
		omap_set_gpio_dataout( _PIN_NB( hsdpa_gpio->hsdpa_power_on ), 1 );
	}

	sysdev_create_file(&hsdpa_device, &attr_hsdpa_reset_cycle);
	reset_cycle = 0;

	printk(KERN_INFO "HSDPA GPIO init done\n");

	return 0;
}
