/*
 * linux/arch/arm/mach-omap3/board-3430sdp-hsmmc.c
 *
 * Copyright (C) 2007 Texas Instruments
 * Author: Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <asm/hardware.h>
#include <asm/arch/board.h>
#include <asm/arch/board-archosg6.h>
#include <asm/io.h>
#include <asm/arch/board.h>
#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>

#define mmc_slot1		1
#define mmc_slot2		2
#define mmc_slot3		3

extern void archosg6_wifi_reset(void);
extern void archosg6_wifi_set_power(int enable);

static int power_pin_mmc1 = -1;

void ceatahdd_power(int on_off)
{
	if (power_pin_mmc1 != -1)
		omap_set_gpio_dataout(power_pin_mmc1, !!on_off);
	msleep(1000);
}
EXPORT_SYMBOL(ceatahdd_power);


/*
 * Enable power to MMC controller.
 * slot - MMC1 - CE-ATA
 * slot - MMC2 - Wifi
 */
int enable_mmc_power(int slot)
{
	#if 0
	if (slot == mmc_slot1) {
		if (power_pin_mmc1 != -1) {
			omap_set_gpio_dataout(power_pin_mmc1, 1);
			msleep(1000);
		}
	} else
	#endif
	if (slot == mmc_slot2) {
		archosg6_wifi_set_power(1);
		archosg6_wifi_reset();
	}
	return 0;
}

/*
 * Disable power to MMC controller.
 * slot - MMC1 - CE-ATA
 * slot - MMC2 - Wifi
 */
int disable_mmc_power(int slot)
{
	#if 0
	if (slot == mmc_slot1) {
		if (power_pin_mmc1 != -1) {
			omap_set_gpio_dataout(power_pin_mmc1, 0);
			msleep(1000);
		}
	} else
	#endif
	if (slot == mmc_slot2) {
		archosg6_wifi_set_power(0);
	}
	return 0;
}

/*
 * Configure the GPIO parameters for the MMC hotplug irq
 */
int setup_mmc_carddetect_irq(int irq)
{
	return 0;
}

#ifdef CONFIG_PM
int mask_carddetect_int(int slot)
{
	return 0;
}

int unmask_carddetect_int(int slot)
{
	return 0;
}

#endif

int switch_power_mode(int power_mode)
{
	return 0;
}

int __init archosg6_hsmmc_init(void)
{
	const struct omap_mmc_config	*mmc_conf;
	const struct omap_mmc_conf	*mmc;

	mmc_conf = omap_get_config(OMAP_TAG_MMC, struct omap_mmc_config);
	if (!mmc_conf)
		return -ENODEV;

#if defined(CONFIG_OMAP3430_MMC1)
	mmc = &mmc_conf->mmc[0];
	if (mmc->enabled) {
		printk(KERN_DEBUG "mmc1 enabled\n");
		if (mmc->power_pin != -1) {
			power_pin_mmc1 = mmc->power_pin;
			if (omap_request_gpio(power_pin_mmc1) < 0)
				printk(KERN_ERR "can't get GPIO GPIO_HDD_PWR_ON (GPIO%d)\n", power_pin_mmc1);

			omap_set_gpio_dataout(power_pin_mmc1, 1);
			omap_set_gpio_direction(power_pin_mmc1, GPIO_DIR_OUTPUT);
		}
	}
#endif

#if defined(CONFIG_OMAP3430_MMC3)
	mmc = &mmc_conf->mmc[2];
	if (mmc->enabled) {
		printk(KERN_DEBUG "mmc3 enabled\n");

		/* establish pin multiplexing */
		omap_cfg_reg(AB1_3430_MMC3_CLK);
		omap_cfg_reg(AC3_3430_MMC3_CMD);
		omap_cfg_reg(AE4_3430_MMC3_DAT0);
		omap_cfg_reg(AH3_3430_MMC3_DAT1);
		omap_cfg_reg(AF3_3430_MMC3_DAT2);
		omap_cfg_reg(AE3_3430_MMC3_DAT3);
		omap_cfg_reg(AF11_3430_MMC3_DAT4);
		omap_cfg_reg(AG9_3430_MMC3_DAT5);
		omap_cfg_reg(AF9_3430_MMC3_DAT6);
		omap_cfg_reg(AH14_3430_MMC3_DAT7);

		if (mmc->power_pin != -1) {
			power_pin_mmc1 = mmc->power_pin;
			if (omap_request_gpio(power_pin_mmc1) < 0)
				printk(KERN_ERR "can't get GPIO GPIO_HDD_PWR_ON (GPIO%d)\n", power_pin_mmc1);

			omap_set_gpio_dataout(power_pin_mmc1, 1);
			omap_set_gpio_direction(power_pin_mmc1, GPIO_DIR_OUTPUT);
		}
	}
#endif
	
	return 0;
}

EXPORT_SYMBOL(enable_mmc_power);
EXPORT_SYMBOL(disable_mmc_power);
EXPORT_SYMBOL(setup_mmc_carddetect_irq);
EXPORT_SYMBOL(switch_power_mode);
#ifdef CONFIG_PM
EXPORT_SYMBOL(mask_carddetect_int);
EXPORT_SYMBOL(unmask_carddetect_int);
#endif
