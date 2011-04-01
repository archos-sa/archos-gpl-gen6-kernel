#include <linux/types.h>
#include <linux/sysdev.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <asm/arch/clock.h>
#include <asm/arch/gpio.h>
#include <asm/arch/prcm.h>
#include <asm/arch/mux.h>
#include <asm/arch/board.h>
#include <asm/mach-types.h>

#define USE_CLOCK24MHZ

static struct g6_gpio gpio_hdd_pwron;
static struct g6_gpio gpio_hub_pwron;
#ifdef USE_CLOCK24MHZ
static struct clk *clkout2;
#endif
static struct clk *func96m_clk;
static int hubvcc;
static int hubpower;
static int hddvcc;

#ifdef USE_CLOCK24MHZ
static int set_clocks(struct device *dev)
{
	clkout2 = clk_get(dev, "sys_clkout2");
	if (IS_ERR(clkout2)) {
		dev_err(dev, "Could not get sys_clkout2\n");
		return -ENODEV;
	}
	/* configure 24 MHz output on SYS_CLKOUT2.  */
	func96m_clk = clk_get(dev, "cm_96m_ck");
	if (IS_ERR(func96m_clk)) {
		dev_err(dev, "Could not get func coreck clock\n");
		clk_put(clkout2);
		return -ENODEV;
	}

	clk_set_parent(clkout2, func96m_clk);
	clk_set_rate(clkout2, 24000000);

	return 0;
}
#endif

#ifdef CONFIG_PM
static int usbhdd_suspend(struct sys_device* dev, pm_message_t state)
{
	if (hubpower) {
		if (machine_is_archos_g6plus() && _PIN_NB(gpio_hub_pwron))
			omap_set_gpio_dataout( _PIN_NB( gpio_hub_pwron ), 0 );

#ifdef USE_CLOCK24MHZ
		clk_disable(clkout2);
#endif		
	}
		
	return 0;
}

static int usbhdd_resume(struct sys_device* dev)
{
	if (hubpower) {
#ifdef USE_CLOCK24MHZ
		clk_enable(clkout2);
#endif		
		if (machine_is_archos_g6plus() && _PIN_NB(gpio_hub_pwron))
			omap_set_gpio_dataout( _PIN_NB( gpio_hub_pwron ), 1 );
			
	}	
	return 0;
}
#endif

static struct sysdev_class usbhdd_sysclass = {
	set_kset_name("usbhdd"),
#ifdef CONFIG_PM
	.suspend	= usbhdd_suspend,
	.resume		= usbhdd_resume,
#endif
};

static struct sys_device usbhdd_device = {
	.id		= 0,
	.cls		= &usbhdd_sysclass,
};


static ssize_t show_usbhdd_hubvcc(struct sys_device* dev, char* buf)
{
	return sprintf(buf, "%d\n", hubvcc); 
}

static void hub_power( void )
{
	int hub_needed = hddvcc | hubvcc;
	
	if (hub_needed == hubpower)
		return;
	
	hubpower = hub_needed;
printk( KERN_DEBUG "hub_power %d \n", hubpower);

	if (_PIN_NB(gpio_hub_pwron))
		omap_set_gpio_dataout( _PIN_NB( gpio_hub_pwron ), hubpower );

#ifdef USE_CLOCK24MHZ
	if ( hubpower ) {
		clk_enable( clkout2 );
	} else {
		clk_disable( clkout2 );
	}
#endif	

}

static ssize_t set_usbhdd_hubvcc(struct sys_device* dev, const char* buf, size_t len)
{
	int on_off = simple_strtol(buf, NULL, 10);
	
	if( on_off > 1 || on_off < 0 )
		return -EINVAL;

	if (hubvcc == on_off)
		return len;

	hubvcc = on_off;
	printk( KERN_DEBUG "set_usbhdd_hubvcc %d\n",on_off);

	hub_power();
	
	return len;
}

static struct sysdev_attribute attr_usbhdd_hubvcc = {
	.attr  = { .name = "hubvcc", .mode = S_IRUGO|S_IWUSR },
	.show  = show_usbhdd_hubvcc,
	.store = set_usbhdd_hubvcc,
};

static ssize_t show_usbhdd_hddvcc(struct sys_device* dev, char* buf)
{
	return sprintf(buf, "%d\n", hddvcc);
}

/* export usbhdd power switch to drivers */
void usbhdd_power(int on_off)
{
	if (hddvcc == on_off)
		return;
	printk( KERN_DEBUG "usbhdd_power %d\n", on_off);
	hddvcc = on_off;

	if (_PIN_NB(gpio_hdd_pwron))
		omap_set_gpio_dataout( _PIN_NB( gpio_hdd_pwron ), on_off );

	// set hub power if needed
	hub_power();

}
EXPORT_SYMBOL(usbhdd_power);

static ssize_t set_usbhdd_hddvcc(struct sys_device* dev, const char* buf, size_t len)
{
	int on_off = simple_strtol(buf, NULL, 10);
	
	if( on_off > 1 || on_off < 0 )
		return -EINVAL;
	printk( KERN_DEBUG "set_usbhdd_hddvcc %d\n", on_off);

	if (hddvcc == on_off)
		return len;

	usbhdd_power( on_off );

	return len;
}

static struct sysdev_attribute attr_usbhdd_hddvcc = {
	.attr  = { .name = "hddvcc", .mode = S_IRUGO|S_IWUSR },
	.show  = show_usbhdd_hddvcc,
	.store = set_usbhdd_hddvcc,
};


int __init archosg6_usbhdd_init(void)
{
	int ret;
	const struct archosg6_usbhdd_config *hdd_cfg;
	
	/* usb hdd */
	hdd_cfg = omap_get_config( ARCHOS_TAG_USBHDD, struct archosg6_usbhdd_config );
	if (hdd_cfg == NULL) {
		printk(KERN_DEBUG "archosg6_usbhdd_init: no board configuration found\n");
		return -ENODEV;
	}
	if ( hardware_rev >= hdd_cfg->nrev ) {
		printk(KERN_DEBUG "archosg6_usbhdd_init: hardware_rev (%i) >= nrev (%i)\n",
			hardware_rev, hdd_cfg->nrev);
		return -ENODEV;
	}
	
	printk(KERN_DEBUG "archosg6_usbhdd_init\n");

	/* sysfs setup */
	sysdev_class_register(&usbhdd_sysclass);
	ret = sysdev_register(&usbhdd_device);
	if (ret)
		return ret;	

	/* HDD power switch */
	gpio_hdd_pwron = hdd_cfg->rev[hardware_rev].hdd_power;
	if(_PIN_NB(gpio_hdd_pwron)) {
		sysdev_create_file(&usbhdd_device, &attr_usbhdd_hddvcc);
		_INIT_OUTPUT(gpio_hdd_pwron);
		omap_set_gpio_dataout(_PIN_NB(gpio_hdd_pwron), 0);
	}
	
	/* HUB power switch */
	gpio_hub_pwron = hdd_cfg->rev[hardware_rev].hub_power;
	if (_PIN_NB(gpio_hub_pwron)) {
		/* clock out config */
#ifdef USE_CLOCK24MHZ
		omap_cfg_reg( AE22_3430_CLKOUT2 );
		set_clocks(NULL);
#endif
		sysdev_create_file(&usbhdd_device, &attr_usbhdd_hubvcc);
		_INIT_OUTPUT(gpio_hub_pwron);
		omap_set_gpio_dataout(_PIN_NB(gpio_hub_pwron), 0);
	}

	return 0;
}
