#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>

#define VERSION "0.01"

#define GPS_CARHOLDER_SET_POWER 0
#define GPS_CARHOLDER_FORWARD_USB 1

static int gps_carholder_open(struct inode *inode, struct file *file)
{
printk("gps_carholder_open\n");
	if (omap_request_gpio(184)) {
printk("could not get GPIO184!\n");
		return -1;
	}

	if (omap_request_gpio(185)) {
printk("could not get GPIO185!\n");
		gpio_free(184);
		return -1;
	}

	omap_cfg_reg(AF14_3430_GPIO184);
	omap_cfg_reg(AG14_3430_GPIO185);

	/* set them to input here, they will be configured to outputs when we have
	   the carholder detected */
	gpio_direction_input(184);
	gpio_direction_input(185);

	return 0;
}

static int gps_carholder_release(struct inode *inode, struct file *file)
{
printk("gps_carholder_release\n");
	omap_cfg_reg(AF14_3430_I2C3_SCL);
	omap_cfg_reg(AG14_3430_I2C3_SDA);

	gpio_free(184);
	gpio_free(185);

	return 0;
}

static int gps_carholder_ioctl(struct inode *inode, struct file *file,
			    unsigned int cmd, unsigned long arg)
{
	/*
	function sets the direction with every call, so we do not have
	to store if the direction has been set or if it is the first call. 
	I do not want to set the direction on opening but after the carholder
	has been detected.
	I2C_PLUG_SCL => GPS_POWER (pin 20 of connector)		-> GPIO184
	I2C_PLUG_SDA => USB_FORWARD (pin 21 of connector)	-> GPIO185
	*/
	switch(cmd) {
		case GPS_CARHOLDER_SET_POWER:
printk("GPS_CARHOLDER_SET_POWER: %d\n", (int)arg);
			gpio_direction_output(184, (int)arg);
		break;

 		case GPS_CARHOLDER_FORWARD_USB:
printk("GPS_CARHOLDER_FORWARD_USB: %d\n", (int)arg);
			if ((int)arg)
				gpio_direction_output(185, 0);
			else
				gpio_direction_output(185, 1);
		break;

		default:
printk("gps_carholder_ioctl -> default\n");
		return -ENOIOCTLCMD;
	}

	return 0;
}

static struct file_operations gps_carholder_fops = {
	.owner = THIS_MODULE,
	.ioctl = gps_carholder_ioctl,
	.open  = gps_carholder_open,
	.release = gps_carholder_release,
};

static struct miscdevice gps_carholder_miscdev =
{
	251,
	"gps_carholder",
	&gps_carholder_fops
};

int __init gps_carholder_init(void)
{
	misc_register(&gps_carholder_miscdev);
	printk("GPS carholder, version %s\n", VERSION);

	return 0;
}

void __exit gps_carholder_exit(void)
{
	misc_deregister(&gps_carholder_miscdev);
}

module_init(gps_carholder_init);
module_exit(gps_carholder_exit);

MODULE_AUTHOR("Archos S.A.");
MODULE_DESCRIPTION("GPS carholder support driver");
MODULE_LICENSE("GPL");
