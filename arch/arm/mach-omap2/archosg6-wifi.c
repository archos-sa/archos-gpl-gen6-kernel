#include <linux/err.h>
#include <linux/module.h>
#include <linux/delay.h>

#include <asm/hardware.h>
#include <asm/arch/board.h>
#include <asm/arch/board-archosg6.h>
#include <asm/io.h>
#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>

static int wifi_pwron = -1;
static int wifi_rst = -1;

void archosg6_wifi_reset(void)
{
	if (wifi_rst != -1)
	{
		omap_set_gpio_dataout(wifi_rst, 1);
		mdelay(1);
		omap_set_gpio_dataout(wifi_rst, 0);
		udelay(10);
		omap_set_gpio_dataout(wifi_rst, 1);
		mdelay(1);
	}
}

EXPORT_SYMBOL(archosg6_wifi_reset);

void archosg6_wifi_set_power(int enable)
{
#define  IEN	(1 << 8)
#define  IDIS	(0 << 8)
#define  PTU	(1 << 4)
#define  PTD	(0 << 4)
#define  EN	(1 << 3)
#define  DIS	(0 << 3)
	printk("archosg6_wifi_set_power %s (wifi), pwr: %d, rst: %d\n", enable ? "on":"off", wifi_pwron, wifi_rst);

	if (wifi_pwron == -1 || wifi_rst == -1)
		return;

	if (enable)
	{
		omap_set_gpio_dataout( wifi_pwron, 1);	

		msleep(200);
		omap_cfg_reg(AE2_3430_MMC2_CLK);
		omap_cfg_reg(AG5_3430_MMC2_CMD);
		omap_cfg_reg(AH5_3430_MMC2_DAT0);
		omap_cfg_reg(AH4_3430_MMC2_DAT1);
		omap_cfg_reg(AG4_3430_MMC2_DAT2);
		omap_cfg_reg(AF4_3430_MMC2_DAT3);

	}
	else
	{
		omap_set_gpio_dataout(wifi_rst, 0);

		omap_cfg_reg(AE2_3430_SAFE);
		omap_cfg_reg(AG5_3430_SAFE);
		omap_cfg_reg(AH5_3430_SAFE);
		omap_cfg_reg(AH4_3430_SAFE);
		omap_cfg_reg(AG4_3430_SAFE);
		omap_cfg_reg(AF4_3430_SAFE);

		omap_set_gpio_dataout( wifi_pwron, 0);
	}
}

EXPORT_SYMBOL(archosg6_wifi_set_power);

int __init archosg6_wifi_init(void)
{
	const struct archosg6_mmc_wifi_dev_conf	*wifi_dev_conf;
	wifi_dev_conf = omap_get_config(ARCHOS_TAG_WIFI, struct archosg6_mmc_wifi_dev_conf);
	if (!wifi_dev_conf)
		return -ENODEV;

	if (hardware_rev < wifi_dev_conf->nrev)
	{
		if (wifi_dev_conf->rev[hardware_rev].power_pin != -1) 
			wifi_pwron = wifi_dev_conf->rev[hardware_rev].power_pin;
		
		if (wifi_dev_conf->rev[hardware_rev].rst_pin != -1) 
			wifi_rst = wifi_dev_conf->rev[hardware_rev].rst_pin;				

		if (wifi_dev_conf->rev[hardware_rev].power_pin_cfg != -1)
			omap_cfg_reg( wifi_dev_conf->rev[hardware_rev].power_pin_cfg );
		
		if (wifi_dev_conf->rev[hardware_rev].rst_pin_cfg != -1)
			omap_cfg_reg( wifi_dev_conf->rev[hardware_rev].rst_pin_cfg );

		if (wifi_pwron != -1 && wifi_rst != -1)
		{
			if (omap_request_gpio( wifi_pwron ) < 0)
				printk(KERN_ERR "can't get WIFI_PWRON (GPIO%d)\n", wifi_pwron);
			if (omap_request_gpio(wifi_rst) < 0)
				printk(KERN_ERR "can't get GPIO RST_WIFI (GPIO%d)\n", wifi_rst);

			omap_set_gpio_dataout(wifi_pwron, 0);	
			omap_set_gpio_dataout(wifi_rst, 0);
			omap_set_gpio_direction(wifi_rst, GPIO_DIR_OUTPUT);
			omap_set_gpio_direction(wifi_pwron, GPIO_DIR_OUTPUT);
		}
	}
	
	return 0;
}
