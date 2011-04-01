#include <linux/types.h>
#include <linux/sysdev.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <asm/arch/gpio.h>
#include <asm/arch/prcm.h>
#include <asm/arch/mux.h>

static struct g6_gpio gpio_sata_pwron = {
	.nb = 57,
	.mux_cfg = P8_3430_GPIO57,
};

static struct g6_gpio gpio_hdd_pwron = {
	.nb = 18,
	.mux_cfg = AE11_3430_GPIO18,
};

static struct g6_gpio gpio_sata_rdy = {
	.nb = 52,
	.mux_cfg = H3_3430_GPIO52,
};

static int satavcc;
static struct clk *clkout1;

#ifdef CONFIG_PM
static int usb2sata_suspend(struct sys_device* dev, pm_message_t state)
{
	if (satavcc)
		clk_disable(clkout1);
		
	return 0;
}

static int usb2sata_resume(struct sys_device* dev)
{
	if (satavcc)
		clk_enable(clkout1);
		
	return 0;
}
#endif

extern int archosg6_enable_ehci( int enable );

/* export usbsata_power to drivers for power management */
void usbsata_power(int on_off)
{
	if (satavcc == on_off)
		return;
		
	satavcc = on_off;
	omap_set_gpio_dataout(_PIN_NB(gpio_hdd_pwron), on_off);
	msleep(100);
	omap_set_gpio_dataout(_PIN_NB(gpio_sata_pwron), on_off);

	if (on_off) {
		clk_enable(clkout1);
		archosg6_enable_ehci( 1 );
	} else {
		/* wait another 100ms to propagate the disconnect through
		* the phy, then switch if off */
		msleep(100);
		archosg6_enable_ehci( 0 );
		clk_disable(clkout1);
	}
		
}
EXPORT_SYMBOL(usbsata_power);

static struct sysdev_class usb2sata_sysclass = {
	set_kset_name("usb2sata"),
#ifdef CONFIG_PM
	.suspend	= usb2sata_suspend,
	.resume		= usb2sata_resume,
#endif
};

static struct sys_device usb2sata0_device = {
	.id		= 0,
	.cls		= &usb2sata_sysclass,
};

static ssize_t show_usb2sata_satardy(struct sys_device* dev, char* buf)
{
	int satardy = omap_get_gpio_datain(_PIN_NB(gpio_sata_rdy));
	return sprintf(buf, "%d\n", satardy); 
}

static ssize_t show_usb2sata_satavcc(struct sys_device* dev, char* buf)
{
	return sprintf(buf, "%d\n", satavcc); 
}

static ssize_t set_usb2sata_satavcc(struct sys_device* dev, const char* buf, size_t len)
{
	int on_off = simple_strtol(buf, NULL, 10);
	
	if( on_off > 1 || on_off < -1 )
		return -EINVAL;

	usbsata_power(on_off);
	return len;
}

static struct sysdev_attribute attr_usb2sata_satardy = {
	.attr = { .name = "satardy", .mode = S_IRUGO },
	.show = show_usb2sata_satardy,
};

static struct sysdev_attribute attr_usb2sata_satavcc = {
	.attr  = { .name = "satavcc", .mode = S_IRUGO|S_IWUSR },
	.show  = show_usb2sata_satavcc,
	.store = set_usb2sata_satavcc,
};

int __init archosg6h_usb2sata_init(void)
{
	int ret;

	clkout1 = clk_get(NULL, "sys_clkout1");
	if (IS_ERR(clkout1)) {
		printk(KERN_ERR "clk_get(sys_clkout1) failed\n");
		return PTR_ERR(clkout1);
	}

	/* SATA bridge */
	_INIT_OUTPUT(gpio_sata_pwron);
	omap_set_gpio_dataout(_PIN_NB(gpio_sata_pwron), 0);

	/* HDD power switch */
	_INIT_OUTPUT(gpio_hdd_pwron);
	omap_set_gpio_dataout(_PIN_NB(gpio_hdd_pwron), 0);
	
	/* SATA_RDY signal */
	_INIT_INPUT(gpio_sata_rdy);
	
	sysdev_class_register(&usb2sata_sysclass);
	
	ret = sysdev_register(&usb2sata0_device);
	if (ret < 0)
		return ret;

	sysdev_create_file(&usb2sata0_device, &attr_usb2sata_satardy);
	sysdev_create_file(&usb2sata0_device, &attr_usb2sata_satavcc);
	
	return 0;
}
