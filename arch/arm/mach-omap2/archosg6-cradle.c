#include <linux/types.h>
#include <linux/sysdev.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <asm/mach-types.h>

#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#include <asm/arch/archosg6-gpio.h>
#include <asm/arch/board.h>

#define NEED_UART	1<<0
#define NEED_IR		1<<1

static struct g6_gpio gpio_5Vusb;

static int current_ir_mode=NEED_UART;

static struct sysdev_class accessory_sysclass = {
	set_kset_name("accessory"),
};

static ssize_t show_5vusb(struct sysdev_class* cls, char* buf)
{
	int val_5vusb = omap_get_gpio_datain( _PIN_NB( gpio_5Vusb ) );
	return sprintf(buf, "%i\n", val_5vusb); 
}

static ssize_t store_5vusb(struct sysdev_class* cls, const char* buf, size_t len)
{
	int on_off = simple_strtol(buf, NULL, 10);
	
	if( on_off > 1 || on_off < -1 )
		return -EINVAL;

	omap_set_gpio_dataout( _PIN_NB( gpio_5Vusb ), on_off);	
	return len;
}

static SYSDEV_CLASS_ATTR(5vusb, S_IRUGO|S_IWUSR, show_5vusb, store_5vusb);

int archosg6_accessory_set_irmode(char *mode) {

	int need = 0;

	printk(KERN_DEBUG "archosg6_accessory_set_irmode: mode (%s)\n", mode);

	if ( strncmp(mode,"uart",5) == 0 ) {
		need = NEED_UART;
	} else if ( strncmp(mode,"ir",2) == 0 ) {
		need = NEED_IR;
	} else if ( strncmp(mode,"both",4) == 0 ) {
		need = NEED_UART | NEED_IR;
	} else  {
		/* default */
		need = current_ir_mode;
	}


	if ( need & NEED_UART ) {
		/* connect uart3 rx/tx*/
		omap_cfg_reg(H20_3430_UART3_RX_IRRX);
		omap_cfg_reg(H21_3430_UART3_TX_IRTX);
	} else {
		/* disconnect uart */
		/* dis uart3 rx */
	 	omap_set_gpio_direction( 165, GPIO_DIR_INPUT);
		omap_cfg_reg(H20_3430_GPIO165);
		/* dis uart3 tx */
	 	omap_set_gpio_direction( 166, GPIO_DIR_INPUT);
		omap_cfg_reg(H21_3430_GPIO166);
	}

	if ( need & NEED_IR ) {
		/* connect ir remote */
		omap_cfg_reg(AB26_3430_UART2_CTS);
		/* ir blaster connection is done by the driver itself */
	} else {
		/* disconnect irblaster */
		/* irb => gio as input */
		omap_cfg_reg(AD25_3430_GPIO147);
		omap_cfg_reg(AB26_3430_SAFE);
	}
	
	return need;
}

static ssize_t show_irmode(struct sysdev_class* cls, char* buf)
{
	int ret;
	switch(current_ir_mode) {
		case NEED_UART:
			ret = sprintf(buf, "uart\n"); 
		break;
		case NEED_IR:
			ret = sprintf(buf, "ir\n"); 
		break;
		case (NEED_IR|NEED_UART):
			ret = sprintf(buf, "both\n"); 
		break;
		default:
			ret = sprintf(buf, "unknown\n"); 
		break;
	}
	
	return ret; 
}

static ssize_t store_irmode(struct sysdev_class* cls, const char* buf, size_t len)
{

	current_ir_mode = archosg6_accessory_set_irmode(buf);

	return len;
}

static SYSDEV_CLASS_ATTR(ir_mode, S_IRUGO|S_IWUSR, show_irmode, store_irmode);



static int __init archosg6_accessory_init(void)
{
	int ret;
	/* USB0 PHY VBUS */
	const struct archosg6_vbus_config *vbus_cfg;
	vbus_cfg = omap_get_config( ARCHOS_TAG_VBUS0, struct archosg6_vbus_config );
	if (vbus_cfg == NULL) {
		printk(KERN_DEBUG "archosg6_accessory_init: no board configuration found\n");
		return -ENODEV;
	}
	if ( hardware_rev >= vbus_cfg->nrev ) {
		printk(KERN_DEBUG "archosg6_accessory_init: hardware_rev (%i) >= nrev (%i)\n",
			hardware_rev, vbus_cfg->nrev);
		return -ENODEV;
	}

	gpio_5Vusb = vbus_cfg->rev[hardware_rev];
	
	printk(KERN_DEBUG "archosg6_accessory_init: gpio %i\n", gpio_5Vusb.nb);

	_INIT_OUTPUT( gpio_5Vusb );

	omap_set_gpio_dataout( _PIN_NB( gpio_5Vusb ), 0);
	
	ret = sysdev_class_register(&accessory_sysclass);
	if (ret >= 0) {
		sysdev_class_create_file(&accessory_sysclass, &attr_5vusb);	
		sysdev_class_create_file(&accessory_sysclass, &attr_ir_mode);	
	}
	
	return ret;
}

late_initcall(archosg6_accessory_init);

EXPORT_SYMBOL(archosg6_accessory_set_irmode);