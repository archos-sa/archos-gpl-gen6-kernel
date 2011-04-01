/*
 * linux/arch/arm/mach-omap2/sr_core.c
 *
 * OMAP34XX SmartReflex Voltage Control core functions
 *
 * Copyright (C) 2008 Archos S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/sysfs.h>

#include <asm/arch/prcm.h>
#include <asm/arch/sr_core.h>
#include "smartreflex.h"

#define DBG if (0)

static struct smartreflex_alg *current_sralg;

static int srcore_start_vddautocomp(int srid, int target_opp_no)
{
	int ret = SR_FAIL;
	
	if (current_sralg)
		ret = current_sralg->autocomp_start(current_sralg, srid, target_opp_no);

	return ret;
}

static int srcore_stop_vddautocomp(int srid)
{
	int ret = SR_FAIL;
	
	if (current_sralg)
		ret = current_sralg->autocomp_stop(current_sralg, srid);

	return ret;
}

static int alg_set_voltage_level(struct smartreflex_alg* alg, u32 target_opp, int vsel)
{
	u32 vdd = get_vdd(target_opp);
	u32 target_opp_no = get_opp_no(target_opp);

DBG	printk("srcore_set_voltage_level: target_opp_no %u, vdd %u, vsel %i\n",
		target_opp_no, vdd, vsel);

#if defined(CONFIG_OMAP_VOLT_SR_BYPASS)
	if (set_voltage_level_vcbypass(vdd, vsel)) {
		printk(KERN_ERR "Unable to set the voltage level \n");
		return SR_FAIL;
	}
#elif defined(CONFIG_OMAP_VOLT_VSEL)
	if (set_voltage_level(vdd, vsel)) {
		printk(KERN_ERR "Unable to set the voltage level \n");
		return SR_FAIL;
	}
#endif
	return SR_PASS;
}

int srcore_set_voltage_level(u32 target_opp, int vsel)
{
	int sr_status = 0;
	u32 vdd, target_opp_no;

	vdd = get_vdd(target_opp);
	target_opp_no = get_opp_no(target_opp);

	if (vdd == PRCM_VDD1) {
		sr_status = srcore_stop_vddautocomp(SR1);
	} else if (vdd == PRCM_VDD2) {
		sr_status = srcore_stop_vddautocomp(SR2);
	}

	if (alg_set_voltage_level(current_sralg, target_opp, vsel) == SR_FAIL)
		return SR_FAIL;

	if (sr_status == SR_PASS) {
		if (vdd == PRCM_VDD1)
			srcore_start_vddautocomp(SR1, target_opp_no);
		else if (vdd == PRCM_VDD2)
			srcore_start_vddautocomp(SR2, target_opp_no);
	}

	return SR_PASS;
}
EXPORT_SYMBOL(srcore_set_voltage_level);

int srcore_enable_smartreflex(int srid)
{
	int ret = SR_FAIL;
	
	if (current_sralg)
		ret = current_sralg->enable(current_sralg, srid);

	return ret;
}
EXPORT_SYMBOL(srcore_enable_smartreflex);

int srcore_disable_smartreflex(int srid)
{
	int ret = SR_FAIL;
	
	if (current_sralg)
		ret = current_sralg->disable(current_sralg, srid);

	return ret;
}
EXPORT_SYMBOL(srcore_disable_smartreflex);

int srcore_register_alg(struct smartreflex_alg *alg)
{
	int ret;

	if (current_sralg != NULL)
		return -EBUSY;

	alg->set_voltage = alg_set_voltage_level;
	ret = alg->init(alg);
	if (ret == SR_PASS)
		current_sralg = alg;

	return 0;
}
EXPORT_SYMBOL(srcore_register_alg);

int srcore_unregister_alg(struct smartreflex_alg *alg)
{
	int ret;

	if (current_sralg == NULL)
		return -ENODEV;

	if (current_sralg != alg)
		return -ENODEV;

	ret = alg->release(alg);
	current_sralg = NULL;

	return ret;
}
EXPORT_SYMBOL(srcore_unregister_alg);

static int __init srcore_init(void)
{
	current_sralg = NULL;

	return 0;
}

arch_initcall(srcore_init);
