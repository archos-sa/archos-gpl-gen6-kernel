/*
 * linux/arch/arm/mach-omap3/prcm-debug.c
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <asm/arch/resource.h>
#include <asm/arch/prcm.h>
#include <asm/arch/prcm-debug.h>
#include <asm/arch/clock.h>
#include "prcm-regs.h"


/* #define RESOURCE_TEST  */

#define S600M   600000000
#define S550M   550000000
#define S500M   500000000
#define S250M   250000000
#define S125M   125000000
#define S19M    19200000
#define S120M   120000000
#define S477M   476000000
#define S381M   381000000
#define S191M   190000000
#define S96M    96000000
#define S166M   166000000
#define S83M    83000000
#define S66M    66500000
#define S133M   133000000
#define S266M   266000000
#define S293M   293000000
#define S320M   320000000

#ifdef CONFIG_OMAP3430_ES2
#define scale_voltage_then_frequency  (curr_opp_no < target_opp_no)
#ifdef CONFIG_ARCH_OMAP3410
unsigned int round_rate_vdd1[5] = {
	S66M, S133M, S266M, S293M, S320M
	};
#else
unsigned int round_rate_vdd1[5] = {
	S125M, S250M, S500M, S550M, S600M
	};
#endif
unsigned int round_rate_vdd2[3] = {
	0, S83M, S166M
	};
#define MAX_VDD1_OPP CO_VDD1_OPP5
#define MAX_VDD2_OPP CO_VDD2_OPP3
#else
#define scale_voltage_then_frequency  (curr_opp_no > target_opp_no)
unsigned int round_rate_vdd1[4] = {
	S477M, S381M, S191M, S96M
	};
unsigned int round_rate_vdd2[2] = {
	S166M, S83M
	};
#define MAX_VDD1_OPP CO_VDD1_OPP4
#define MAX_VDD2_OPP CO_VDD2_OPP2
#endif


#define DPRINTK1(fmt, args...) printk(KERN_INFO"%s: " fmt,	\
				__FUNCTION__ , ## args)

#define execute_idle() \
	{       \
		__asm__ __volatile__ ("wfi");   \
	}

#define REQUEST_CO 0x1
#define RELEASE_CO 0x2
struct constraint_handle *co_latency;
struct constraint_handle *co_opp;

int nb_pre_test_func(struct notifier_block *n, unsigned long event, void *ptr)
{
	printk(KERN_INFO"Pre Notifier function called for level %lu\n", event);
	return 0;
}

int nb_post_test_func(struct notifier_block *n, unsigned long event, void *ptr)
{
	printk(KERN_INFO"Post Notifier function called for level %lu\n", event);
	return 0;
}
static struct notifier_block nb_test_pre = {
	nb_pre_test_func,
	NULL,
};

static struct notifier_block nb_test_post = {
	nb_post_test_func,
	NULL,
};

static struct constraint_id cnstr_id_lat = {
	.type = RES_LATENCY_CO,
	.data = (void *)"latency",
};

static struct constraint_id cnstr_id_vdd1 = {
	.type = RES_OPP_CO,
	.data = (void *)"vdd1_opp",
};

static struct constraint_id cnstr_id_vdd2 = {
	.type = RES_OPP_CO,
	.data = (void *)"vdd2_opp",
};

static struct constraint_id cnstr_id_arm = {
	.type = RES_FREQ_CO,
	.data = (void *)"arm_freq",
};

static struct constraint_id cnstr_id_dsp = {
	.type = RES_FREQ_CO,
	.data = (void *)"dsp_freq",
};

int power_configuration_test(void)
{
	u32 ivaf, core1f, core1i, core2i, wkupf, wkupi;
	u32 dssf, dssi, camf, cami, perf, peri;
	u32 ssi_sysconfig, sdrc_sysconfig, sdma_sysconfig,
	omap_ctrl_sysconfig, mbox_sysconfig, mcbsp1_sysconfig,
	mcbsp2_sysconfig, gpt10_sysconfig, gpt11_sysconfig, uart1_sysconfig,
	uart2_sysconfig, i2c1_sysconfig, i2c2_sysconfig, i2c3_sysconfig,
	mcspi1_sysconfig, mcspi2_sysconfig, mcspi3_sysconfig, mcspi4_sysconfig,
	mmc1_sysconfig, mmc2_sysconfig, sms_sysconfig, gpmc_sysconfig;
#ifdef CONFIG_OMAP3430_ES2
#ifndef CONFIG_ARCH_OMAP3410
	u32 sgxf, sgxi;
#endif
#else
	u32 fac_sysconfig, d2d_sysconfig;
#endif
	int ret = 0;

	/* Turn on all power domains for test */
	prcm_force_power_domain_state(DOM_IVA2, PRCM_ON);
	prcm_force_power_domain_state(DOM_DSS, PRCM_ON);
	prcm_force_power_domain_state(DOM_CAM, PRCM_ON);
	prcm_force_power_domain_state(DOM_PER, PRCM_ON);

	/* Save clock registers */
	ivaf = CM_FCLKEN_IVA2;
	core1f = CM_FCLKEN1_CORE;
	core1i = CM_ICLKEN1_CORE;
	core2i = CM_ICLKEN2_CORE;
#if defined(CONFIG_OMAP3430_ES2) && !defined(CONFIG_ARCH_OMAP3410)
	sgxf = CM_FCLKEN_SGX;
	sgxi = CM_ICLKEN_SGX;
#endif
	wkupf = CM_FCLKEN_WKUP;
	wkupi = CM_ICLKEN_WKUP;
	dssf = CM_FCLKEN_DSS;
	dssi = CM_ICLKEN_DSS;
	camf = CM_FCLKEN_CAM;
	cami = CM_ICLKEN_CAM;
	perf = CM_FCLKEN_PER;
	peri = CM_ICLKEN_PER;

	/* Enable all clocks */
	CM_FCLKEN_IVA2 = 0xFFFFFFFF;
	CM_FCLKEN1_CORE = 0xFFFFFFFF;
	CM_ICLKEN1_CORE = 0xFFFFFFFF;
	CM_ICLKEN2_CORE = 0xFFFFFFFF;
#if defined(CONFIG_OMAP3430_ES2) && !defined(CONFIG_ARCH_OMAP3410)
	CM_FCLKEN_SGX = 0xFFFFFFFF;
	CM_ICLKEN_SGX = 0xFFFFFFFF;
#endif
	CM_FCLKEN_WKUP = 0xFFFFFFFF;
	CM_ICLKEN_WKUP = 0xFFFFFFFF;
	CM_FCLKEN_DSS = 0xFFFFFFFF;
	CM_ICLKEN_DSS = 0xFFFFFFFF;
	CM_FCLKEN_CAM = 0xFFFFFFFF;
	CM_ICLKEN_CAM = 0xFFFFFFFF;
	CM_FCLKEN_PER = 0xFFFFFFFF;
	CM_ICLKEN_PER = 0xFFFFFFFF;

	/* Wait for clocks to stabilize */
	mdelay(3000);

	ssi_sysconfig = PRCM_SSI_SYSCONFIG;
	sdrc_sysconfig = PRCM_SDRC_SYSCONFIG;
	sdma_sysconfig = PRCM_SDMA_SYSCONFIG;
	omap_ctrl_sysconfig = PRCM_OMAP_CTRL_SYSCONFIG;
	mbox_sysconfig = PRCM_MBOXES_SYSCONFIG;
	mcbsp1_sysconfig = PRCM_MCBSP1_SYSCONFIG;
	mcbsp2_sysconfig = PRCM_MCBSP2_SYSCONFIG;
	gpt10_sysconfig = PRCM_GPT10_SYSCONFIG;
	gpt11_sysconfig = PRCM_GPT11_SYSCONFIG;
	uart1_sysconfig = PRCM_UART1_SYSCONFIG;
	uart2_sysconfig = PRCM_UART2_SYSCONFIG;
	i2c1_sysconfig = PRCM_I2C1_SYSCONFIG;
	i2c2_sysconfig = PRCM_I2C2_SYSCONFIG;
	i2c3_sysconfig = PRCM_I2C3_SYSCONFIG;
	mcspi1_sysconfig = PRCM_MCSPI1_SYSCONFIG;
	mcspi2_sysconfig = PRCM_MCSPI2_SYSCONFIG;
	mcspi3_sysconfig = PRCM_MCSPI3_SYSCONFIG;
	mcspi4_sysconfig = PRCM_MCSPI4_SYSCONFIG;
	mmc1_sysconfig = PRCM_MMC1_SYSCONFIG;
	mmc2_sysconfig = PRCM_MMC2_SYSCONFIG;
	sms_sysconfig = PRCM_SMS_SYSCONFIG;
	gpmc_sysconfig = PRCM_GPMC_SYSCONFIG;
#ifndef CONFIG_OMAP3430_ES2
	d2d_sysconfig = PRCM_D2D_SYSCONFIG;
	fac_sysconfig = PRCM_FAC_SYSCONFIG;
#endif

	DPRINTK1 ("Before - SSI\t0x%x\n", PRCM_SSI_SYSCONFIG);
	DPRINTK1 ("Before - SDRC\t0x%x\n", PRCM_SDRC_SYSCONFIG);
	DPRINTK1 ("Before - SDMA\t0x%x\n", PRCM_SDMA_SYSCONFIG);
	DPRINTK1 ("Before - CTRL\t0x%x\n", PRCM_OMAP_CTRL_SYSCONFIG);
	DPRINTK1 ("Before - MBOXES\t0x%x\n", PRCM_MBOXES_SYSCONFIG);
	DPRINTK1 ("Before - MCBSP1\t0x%x\n", PRCM_MCBSP1_SYSCONFIG);
	DPRINTK1 ("Before - MCBSP2\t0x%x\n", PRCM_MCBSP2_SYSCONFIG);
	DPRINTK1 ("Before - GPT10\t0x%x\n", PRCM_GPT10_SYSCONFIG);
	DPRINTK1 ("Before - GPT11\t0x%x\n", PRCM_GPT11_SYSCONFIG);
	DPRINTK1 ("Before - UART1\t0x%x\n", PRCM_UART1_SYSCONFIG);
	DPRINTK1 ("Before - UART2\t0x%x\n", PRCM_UART2_SYSCONFIG);
	DPRINTK1 ("Before - I2C1\t0x%x\n", PRCM_I2C1_SYSCONFIG);
	DPRINTK1 ("Before - I2C2\t0x%x\n", PRCM_I2C2_SYSCONFIG);
	DPRINTK1 ("Before - I2C3\t0x%x\n", PRCM_I2C3_SYSCONFIG);
	DPRINTK1 ("Before - MCSPI1\t0x%x\n", PRCM_MCSPI1_SYSCONFIG);
	DPRINTK1 ("Before - MCSPI2\t0x%x\n", PRCM_MCSPI2_SYSCONFIG);
	DPRINTK1 ("Before - MCSPI3\t0x%x\n", PRCM_MCSPI3_SYSCONFIG);
	DPRINTK1 ("Before - MCSPI4\t0x%x\n", PRCM_MCSPI4_SYSCONFIG);
	DPRINTK1 ("Before - MMC1\t0x%x\n", PRCM_MMC1_SYSCONFIG);
	DPRINTK1 ("Before - MMC2\t0x%x\n", PRCM_MMC2_SYSCONFIG);
	DPRINTK1 ("Before - SMS\t0x%x\n", PRCM_SMS_SYSCONFIG);
#ifndef CONFIG_OMAP3430_ES2
	DPRINTK1("Before - D2D\t0x%x\n", PRCM_D2D_SYSCONFIG);
	DPRINTK1("Before - FAC\t0x%x\n", PRCM_FAC_SYSCONFIG);
#endif

	ret = prcm_set_domain_power_configuration(DOM_CORE1, 1, 1, 1);
	if (ret != PRCM_PASS) {
		DPRINTK1("Set domain power configuration, DOM_CORE1 failed\n");
		goto revert;
	}
	ret = prcm_set_domain_power_configuration(DOM_WKUP, 1, 1, 1);
	if (ret != PRCM_PASS) {
		DPRINTK1("Set domain power configuration, DOM_WKUP failed\n");
		goto revert;
	}
	ret = prcm_set_domain_power_configuration(DOM_DSS, 1, 1, 1);
	if (ret != PRCM_PASS) {
		DPRINTK1("Set domain power configuration, DOM_DSS failed\n");
		goto revert;
	}
	ret = prcm_set_domain_power_configuration(DOM_CAM, 1, 1, 1);
	if (ret != PRCM_PASS) {
		DPRINTK1("Set domain power configuration, DOM_CAM failed\n");
		goto revert;
	}
	ret = prcm_set_domain_power_configuration(DOM_PER, 1, 1, 1);
	if (ret != PRCM_PASS) {
		DPRINTK1("Set domain power configuration, DOM_PER failed\n");
		goto revert;
	}

	ret = prcm_set_device_power_configuration(PRCM_SSI, 1, 1, 1);
	if (ret != PRCM_PASS) {
		DPRINTK1("Set domain power configuration, PRCM_SSI failed\n");
		goto revert;
	}
#ifndef CONFIG_ARCH_OMAP3410
	ret = prcm_set_device_power_configuration(PRCM_CSIB, 1, 1, 1);
	if (ret != PRCM_PASS) {
		DPRINTK1("Set domain power configuration, PRCM_CSIB failed\n");
		goto revert;
	}
#endif
	ret = prcm_set_device_power_configuration(PRCM_SDRC, 2, 1, 1);
	if (ret != PRCM_PASS) {
		DPRINTK1("Set domain power configuration, PRCM_SDRC failed\n");
		goto revert;
	}
	ret = prcm_set_device_power_configuration(PRCM_SMS, 1, 1, 1);
	if (ret != PRCM_PASS) {
		DPRINTK1("Set domain power configuration, PRCM_SMS failed\n");
		goto revert;
	}
	ret = prcm_set_device_power_configuration(PRCM_GPMC, 1, 1, 1);
	if (ret != PRCM_PASS) {
		DPRINTK1("Set domain power configuration, PRCM_GPMC failed\n");
		goto revert;
	}
	ret = prcm_set_device_power_configuration(PRCM_MPU_INTC, 1, 1, 1);
	if (ret != PRCM_PASS) {
		DPRINTK1("Set domain power configuration, PRCM_MPU_INTC	failed"
				"\n");
		goto revert;
	}
	ret = prcm_set_device_power_configuration(PRCM_CAM, 1, 1, 1);
	if (ret != PRCM_PASS) {
		DPRINTK1("Set domain power configuration, PRCM_CAM failed\n");
		goto revert;
	}
	ret = prcm_set_device_power_configuration(PRCM_MMU, 1, 1, 1);
	if (ret != PRCM_PASS) {
		DPRINTK1("Set domain power configuration, PRCM_MMU failed\n");
		goto revert;
	}
	ret = prcm_set_device_power_configuration(PRCM_MCBSP2, 1, 1, 1);
	if (ret != PRCM_PASS) {
		DPRINTK1("Set domain power configuration, PRCM_MCBSP2 failed"
				"\n");
		goto revert;
	}
	ret = prcm_set_device_power_configuration(PRCM_MCBSP3, 1, 1, 1);
	if (ret != PRCM_PASS) {
		DPRINTK1("Set domain power configuration, PRCM_MCBSP3 failed"
				"\n");
		goto revert;
	}
	ret = prcm_set_device_power_configuration(PRCM_GPT2, 1, 1, 1);
	if (ret != PRCM_PASS) {
		DPRINTK1("Set domain power configuration, PRCM_GPT2 failed\n");
		goto revert;
	}
	ret = prcm_set_device_power_configuration(PRCM_GPT3, 1, 1, 1);
	if (ret != PRCM_PASS) {
		DPRINTK1("Set domain power configuration, PRCM_GPT3 failed\n");
		goto revert;
	}
	ret = prcm_set_device_power_configuration(PRCM_GPT4, 1, 1, 1);
	if (ret != PRCM_PASS) {
		DPRINTK1("Set domain power configuration, PRCM_GPT4 failed\n");
		goto revert;
	}
	ret = prcm_set_device_power_configuration(PRCM_GPIO4, 1, 1, 1);
	if (ret != PRCM_PASS) {
		DPRINTK1("Set domain power configuration, PRCM_GPIO4 failed\n");
		goto revert;
	}

	DPRINTK1 ("After - SSI\t0x%x\n", PRCM_SSI_SYSCONFIG);
	DPRINTK1 ("After - SDRC\t0x%x\n", PRCM_SDRC_SYSCONFIG);
	DPRINTK1 ("After - SDMA\t0x%x\n", PRCM_SDMA_SYSCONFIG);
	DPRINTK1 ("After - CTRL\t0x%x\n", PRCM_OMAP_CTRL_SYSCONFIG);
	DPRINTK1 ("After - MBOXES\t0x%x\n", PRCM_MBOXES_SYSCONFIG);
	DPRINTK1 ("After - MCBSP1\t0x%x\n", PRCM_MCBSP1_SYSCONFIG);
	DPRINTK1 ("After - MCBSP2\t0x%x\n", PRCM_MCBSP2_SYSCONFIG);
	DPRINTK1 ("After - GPT10\t0x%x\n", PRCM_GPT10_SYSCONFIG);
	DPRINTK1 ("After - GPT11\t0x%x\n", PRCM_GPT11_SYSCONFIG);
	DPRINTK1 ("After - UART1\t0x%x\n", PRCM_UART1_SYSCONFIG);
	DPRINTK1 ("After - UART2\t0x%x\n", PRCM_UART2_SYSCONFIG);
	DPRINTK1 ("After - I2C1\t0x%x\n", PRCM_I2C1_SYSCONFIG);
	DPRINTK1 ("After - I2C2\t0x%x\n", PRCM_I2C2_SYSCONFIG);
	DPRINTK1 ("After - I2C3\t0x%x\n", PRCM_I2C3_SYSCONFIG);
	DPRINTK1 ("After - MCSPI1\t0x%x\n", PRCM_MCSPI1_SYSCONFIG);
	DPRINTK1 ("After - MCSPI2\t0x%x\n", PRCM_MCSPI2_SYSCONFIG);
	DPRINTK1 ("After - MCSPI3\t0x%x\n", PRCM_MCSPI3_SYSCONFIG);
	DPRINTK1 ("After - MCSPI4\t0x%x\n", PRCM_MCSPI4_SYSCONFIG);
	DPRINTK1 ("After - MMC1\t0x%x\n", PRCM_MMC1_SYSCONFIG);
	DPRINTK1 ("After - MMC2\t0x%x\n", PRCM_MMC2_SYSCONFIG);
	DPRINTK1 ("After - SMS\t0x%x\n", PRCM_SMS_SYSCONFIG);
#ifndef CONFIG_OMAP3430_ES2
	DPRINTK1 ("After - D2D\t0x%x\n", PRCM_D2D_SYSCONFIG);
	DPRINTK1 ("After - FAC\t0x%x\n", PRCM_FAC_SYSCONFIG);
#endif

revert:
	PRCM_SSI_SYSCONFIG = ssi_sysconfig;
	PRCM_SDRC_SYSCONFIG = sdrc_sysconfig;
	PRCM_SDMA_SYSCONFIG = sdma_sysconfig;
	PRCM_OMAP_CTRL_SYSCONFIG = omap_ctrl_sysconfig;
	PRCM_MBOXES_SYSCONFIG = mbox_sysconfig;
	PRCM_MCBSP1_SYSCONFIG = mcbsp1_sysconfig;
	PRCM_MCBSP5_SYSCONFIG = mcbsp2_sysconfig;
	PRCM_GPT10_SYSCONFIG = gpt10_sysconfig;
	PRCM_GPT11_SYSCONFIG = gpt11_sysconfig;
	PRCM_UART1_SYSCONFIG = uart1_sysconfig;
	PRCM_UART2_SYSCONFIG = uart2_sysconfig;
	PRCM_I2C1_SYSCONFIG = i2c1_sysconfig;
	PRCM_I2C2_SYSCONFIG = i2c2_sysconfig;
	PRCM_I2C3_SYSCONFIG = i2c3_sysconfig;
	PRCM_MCSPI1_SYSCONFIG = mcspi1_sysconfig;
	PRCM_MCSPI2_SYSCONFIG = mcspi2_sysconfig;
	PRCM_MCSPI3_SYSCONFIG = mcspi3_sysconfig;
	PRCM_MCSPI4_SYSCONFIG = mcspi4_sysconfig;
	PRCM_MMC1_SYSCONFIG = mmc1_sysconfig;
	PRCM_MMC2_SYSCONFIG = mmc2_sysconfig;
	PRCM_SMS_SYSCONFIG = sms_sysconfig;
	PRCM_GPMC_SYSCONFIG = gpmc_sysconfig;
#ifndef CONFIG_OMAP3430_ES2
	PRCM_D2D_SYSCONFIG = d2d_sysconfig;
	PRCM_FAC_SYSCONFIG = fac_sysconfig;
#endif

	/* Restore clock registers */
	CM_FCLKEN_IVA2 = ivaf;
	CM_FCLKEN1_CORE = core1f;
	CM_ICLKEN1_CORE = core1i;
	CM_ICLKEN2_CORE = core2i;
#if defined(CONFIG_OMAP3430_ES2) && !defined(CONFIG_ARCH_OMAP3410)
	CM_FCLKEN_SGX = sgxf;
	CM_ICLKEN_SGX = sgxi;
#endif
	CM_FCLKEN_WKUP = wkupf;
	CM_ICLKEN_WKUP = wkupi;
	CM_FCLKEN_DSS = dssf;
	CM_ICLKEN_DSS = dssi;
	CM_FCLKEN_CAM = camf;
	CM_ICLKEN_CAM = cami;
	CM_FCLKEN_PER = perf;
	CM_ICLKEN_PER = peri;

	/* Condition check required during error scenario */
	if (ret != PRCM_PASS)
		return -1;

	/* Sleep/WaKeup dependency API test */
	if (prcm_set_wkup_dependency(DOM_CAM, PRCM_WKDEP_EN_MPU |\
		PRCM_WKDEP_EN_IVA2 | PRCM_WKDEP_EN_WKUP) != PRCM_PASS) {
		printk(KERN_ERR"Camera domain wakeup dependency could not be "
				"set\n");
		return -1;
	}
	if (prcm_set_sleep_dependency(DOM_CAM, PRCM_SLEEPDEP_EN_CORE |\
		PRCM_SLEEPDEP_EN_MPU | PRCM_SLEEPDEP_EN_IVA2) != PRCM_PASS) {
		printk(KERN_ERR"Camera domain sleep dependency could not be "
				"set\n");
		return -1;
	}

	return 0;
}
EXPORT_SYMBOL(power_configuration_test);

int prcm_set_power_state(u32 domain, u32 mode, u32 state)
{
	int ret = 0;
	u8 power_state;
	u32 iclken, fclken;

	prcm_get_domain_interface_clocks(domain , &iclken);
	prcm_get_domain_functional_clocks(domain , &fclken);

	if ((state == PRCM_OFF) || (state == PRCM_RET)) {
		prcm_set_domain_interface_clocks(domain, 0x0);
		prcm_set_domain_functional_clocks(domain, 0x0);
	}

	if (prcm_set_power_domain_state(domain, state, mode)
							!= PRCM_PASS) {
		DPRINTK1("prcm_set_power_domain_state for "
				"domain %d failed. err:%d\n", domain, ret);
		ret = -1;
		goto revert;
	}
	prcm_get_power_domain_state(domain, &power_state);
	DPRINTK1("Power state for domain %d is %d\n", domain, power_state);

revert:
	/* Restore values in iclken and fclken registers */
	prcm_set_domain_interface_clocks(domain, iclken);
	prcm_set_domain_functional_clocks(domain, fclken);

	if (ret)
		return -1;

	return 0;
}

int powerapi_test(void)
{
	u32 cm_sleepdep_dss, cm_sleepdep_cam, cm_sleepdep_per;
	u32 pm_wkdep_iva2, pm_wkdep_mpu, pm_wkdep_dss, pm_wkdep_cam;
#ifndef CONFIG_ARCH_OMAP3410
	u32 pm_wkdep_per, pm_wkdep_neon;
#else
	u32 pm_wkdep_per;
#endif
	u32 ivaf, core1f, core1i, core2i, wkupf, wkupi;
	u32 dssf, dssi, camf, cami, perf, peri;
#ifndef CONFIG_OMAP3430_ES2
	u32 dss_sysconfig, dispc_sysconfig, rfbi_sysconfig;
#endif
	int tries = 0;
	int ret = 0, ret1 = 0;
	u8 state;
#if defined(CONFIG_OMAP3430_ES2) && !defined(CONFIG_ARCH_OMAP3410)
	u32 usbi, usbf, sgxi, sgxf, pm_wkdep_usb, pm_wkdep_sgx,
		cm_sleepdep_usb, cm_sleepdep_sgx;
#else
	u32 usbi, usbf, pm_wkdep_usb, cm_sleepdep_usb;
#endif

	/* Save clock registers */
	ivaf = CM_FCLKEN_IVA2;
	core1f = CM_FCLKEN1_CORE;
	core1i = CM_ICLKEN1_CORE;
	core2i = CM_ICLKEN2_CORE;
#ifdef CONFIG_OMAP3430_ES2
#ifndef CONFIG_ARCH_OMAP3410
	sgxf = CM_FCLKEN_SGX;
	sgxi = CM_ICLKEN_SGX;
#endif
	usbf = CM_FCLKEN_USBHOST;
	usbi = CM_ICLKEN_USBHOST;
#endif
	wkupf = CM_FCLKEN_WKUP;
	wkupi = CM_ICLKEN_WKUP;
	dssf = CM_FCLKEN_DSS;
	dssi = CM_ICLKEN_DSS;
	camf = CM_FCLKEN_CAM;
	cami = CM_ICLKEN_CAM;
	perf = CM_FCLKEN_PER;
	peri = CM_ICLKEN_PER;

	/* Clear all sleep/wakeup dependencies */
#ifdef CONFIG_OMAP3430_ES2
#ifndef CONFIG_ARCH_OMAP3410
	cm_sleepdep_sgx = CM_SLEEPDEP_SGX;
	pm_wkdep_sgx = PM_WKDEP_SGX;
#endif
	cm_sleepdep_usb	= CM_SLEEPDEP_USBHOST;
	pm_wkdep_usb = PM_WKDEP_USBHOST;
#endif
	cm_sleepdep_dss = CM_SLEEPDEP_DSS;
	cm_sleepdep_cam = CM_SLEEPDEP_CAM;
	cm_sleepdep_per = CM_SLEEPDEP_PER;
	pm_wkdep_iva2 = PM_WKDEP_IVA2;
	pm_wkdep_mpu = PM_WKDEP_MPU;
	pm_wkdep_dss = PM_WKDEP_DSS;
	pm_wkdep_cam = PM_WKDEP_CAM;
	pm_wkdep_per = PM_WKDEP_PER;
#ifndef CONFIG_ARCH_OMAP3410
	pm_wkdep_neon = PM_WKDEP_NEON;
#endif

#ifndef CONFIG_OMAP3430_ES2
	/* For DSS tests */
	dss_sysconfig = PRCM_DSS_SYSCONFIG;
	PRCM_DSS_SYSCONFIG = 0x10;

	dispc_sysconfig = PRCM_DISPC_SYSCONFIG;
	PRCM_DISPC_SYSCONFIG = 0x2010;

	rfbi_sysconfig = PRCM_RFBI_SYSCONFIG;
	PRCM_RFBI_SYSCONFIG = 0x10;
#endif

#ifdef CONFIG_OMAP3430_ES2
#ifndef CONFIG_ARCH_OMAP3410
	CM_SLEEPDEP_SGX = 0;
	PM_WKDEP_SGX = 0;
#endif
	CM_SLEEPDEP_USBHOST = 0;
	PM_WKDEP_USBHOST = 0;
#endif
	CM_SLEEPDEP_DSS = 0;
	CM_SLEEPDEP_CAM = 0;
	CM_SLEEPDEP_PER = 0;
	PM_WKDEP_IVA2 = 0;
	PM_WKDEP_MPU = 0;
	PM_WKDEP_DSS = 0;
	PM_WKDEP_CAM = 0;
	PM_WKDEP_PER = 0;
#ifndef CONFIG_ARCH_OMAP3410
	PM_WKDEP_NEON = 0;
#endif

	DPRINTK1("Testing multiple iterations using force mode\n");

	/* Test Multiple transitions for CAM/DSS/SGX/NEON/USBHOST in
	Force mode */
	for (tries = 0; tries < 3; tries++) {
		/* IVA2 */
		DPRINTK1("IVA2: Trying OFF/ON/RET/ON\n");
		ret |= prcm_set_power_state(DOM_IVA2, PRCM_FORCE, PRCM_ON);
		ret |= prcm_set_power_state(DOM_IVA2, PRCM_FORCE, PRCM_OFF);
		ret |= prcm_set_power_state(DOM_IVA2, PRCM_FORCE, PRCM_ON);
		ret |= prcm_set_power_state(DOM_IVA2, PRCM_FORCE, PRCM_RET);
		ret |= prcm_set_power_state(DOM_IVA2, PRCM_FORCE, PRCM_ON);

		/* DSS */
		DPRINTK1("DSS: Trying OFF/ON/RET/ON\n");
		ret |= prcm_set_power_state(DOM_DSS, PRCM_FORCE, PRCM_ON);
		ret |= prcm_set_power_state(DOM_DSS, PRCM_FORCE, PRCM_OFF);
		ret |= prcm_set_power_state(DOM_DSS, PRCM_FORCE, PRCM_ON);
		ret |= prcm_set_power_state(DOM_DSS, PRCM_FORCE, PRCM_RET);
		ret |= prcm_set_power_state(DOM_DSS, PRCM_FORCE, PRCM_ON);

		/* CAM */
		DPRINTK1("CAM: Trying OFF/ON/RET/ON\n");
		ret |= prcm_set_power_state(DOM_CAM, PRCM_FORCE, PRCM_ON);
		ret |= prcm_set_power_state(DOM_CAM, PRCM_FORCE, PRCM_OFF);
		ret |= prcm_set_power_state(DOM_CAM, PRCM_FORCE, PRCM_ON);
		ret |= prcm_set_power_state(DOM_CAM, PRCM_FORCE, PRCM_RET);
		ret |= prcm_set_power_state(DOM_CAM, PRCM_FORCE, PRCM_ON);

#ifdef CONFIG_OMAP3430_ES2
#ifndef CONFIG_ARCH_OMAP3410
		/* SGX */
		DPRINTK1("SGX: Trying OFF/ON/RET/ON\n");
		ret |= prcm_set_power_state(DOM_SGX, PRCM_FORCE, PRCM_ON);
		ret |= prcm_set_power_state(DOM_SGX, PRCM_FORCE, PRCM_OFF);
		ret |= prcm_set_power_state(DOM_SGX, PRCM_FORCE, PRCM_ON);
		ret |= prcm_set_power_state(DOM_SGX, PRCM_FORCE, PRCM_RET);
		ret |= prcm_set_power_state(DOM_SGX, PRCM_FORCE, PRCM_ON);
#endif
		/* USBHOST */
		DPRINTK1("USBHOST: Trying OFF/ON/RET/ON\n");
		ret |= prcm_set_power_state(DOM_USBHOST, PRCM_FORCE, PRCM_ON);
		ret |= prcm_set_power_state(DOM_USBHOST, PRCM_FORCE, PRCM_OFF);
		ret |= prcm_set_power_state(DOM_USBHOST, PRCM_FORCE, PRCM_ON);
		ret |= prcm_set_power_state(DOM_USBHOST, PRCM_FORCE, PRCM_RET);
		ret |= prcm_set_power_state(DOM_USBHOST, PRCM_FORCE, PRCM_ON);
#endif
		/* PER */
		DPRINTK1("PER: Trying OFF/ON/RET/ON\n");
		ret |= prcm_set_power_state(DOM_PER, PRCM_FORCE, PRCM_ON);
		ret |= prcm_set_power_state(DOM_PER, PRCM_FORCE, PRCM_OFF);
		ret |= prcm_set_power_state(DOM_PER, PRCM_FORCE, PRCM_ON);
		ret |= prcm_set_power_state(DOM_PER, PRCM_FORCE, PRCM_RET);
		ret |= prcm_set_power_state(DOM_PER, PRCM_FORCE, PRCM_ON);

#ifndef CONFIG_ARCH_OMAP3410
		/* NEON */
		DPRINTK1("NEON: Trying RET/ON\n");
#ifdef CONFIG_OMAP3430_ES2
		ret |= prcm_set_power_state(DOM_NEON, PRCM_FORCE, PRCM_ON);
		ret |= prcm_set_power_state(DOM_NEON, PRCM_FORCE, PRCM_OFF);
		ret |= prcm_set_power_state(DOM_NEON, PRCM_FORCE, PRCM_ON);
#endif
		ret |= prcm_set_power_state(DOM_NEON, PRCM_FORCE, PRCM_RET);
		ret |= prcm_set_power_state(DOM_NEON, PRCM_FORCE, PRCM_ON);
#endif

		if (ret) {
			printk(KERN_ERR"Error in setting the power states \n");
			goto revert;
		}
	}

	DPRINTK1("Testing force power domains API\n");
	/* Test Force API (Level2 API) */
	/* MPU */
	DPRINTK1("Testing MPU\n");
	ret1 = prcm_force_power_domain_state(DOM_MPU, PRCM_RET);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("Force of MPU power domain failed for state PRCM_RET"
				"\n");
		goto revert;
	}
	prcm_get_pre_power_domain_state(DOM_PER, &state);
	DPRINTK1("Previous mpu state = %x\n", state);

#ifdef CONFIG_OMAP3430_ES2
#ifndef CONFIG_ARCH_OMAP3410
	/* SGX */
	DPRINTK1("Testing SGX\n");
	ret1 = prcm_force_power_domain_state(DOM_SGX, PRCM_OFF);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("Force of SGX power domain failed for state PRCM_OFF"
				"\n");
		goto revert;
	}
	prcm_get_power_domain_state(DOM_SGX, &state);
	DPRINTK1("Current SGX state = %x\n", state);
	ret1 = prcm_force_power_domain_state(DOM_SGX, PRCM_ON);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("Force of SGX power domain failed for state PRCM_ON"
				"\n");
		goto revert;
	}
	prcm_get_power_domain_state(DOM_SGX, &state);
	DPRINTK1("Current SGX state = %x\n", state);
	ret1 = prcm_force_power_domain_state(DOM_SGX, PRCM_RET);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("Force of SGX power domain failed for state PRCM_RET"
				"\n");
		goto revert;
	}
	prcm_get_power_domain_state(DOM_SGX, &state);
	DPRINTK1("Current SGX state = %x\n", state);
	ret1 = prcm_force_power_domain_state(DOM_SGX, PRCM_ON);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("Force of SGX power domain failed for state PRCM_ON"
				"\n");
		goto revert;
	}
	prcm_get_power_domain_state(DOM_SGX, &state);
	DPRINTK1("Current SGX state = %x\n", state);
#endif

	/* USBHOST */
	DPRINTK1("Testing USBHOST\n");
	ret1 = prcm_force_power_domain_state(DOM_USBHOST, PRCM_OFF);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("Force of USBHOST power domain failed for state "
				"PRCM_OFF\n");
		goto revert;
	}
	prcm_get_power_domain_state(DOM_USBHOST, &state);
	DPRINTK1("Current USBHOST state = %x\n", state);
	ret1 = prcm_force_power_domain_state(DOM_USBHOST, PRCM_ON);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("Force of USBHOST power domain failed for state "
				"PRCM_ON\n");
		goto revert;
	}
	prcm_get_power_domain_state(DOM_USBHOST, &state);
	DPRINTK1("Current USBHOST state = %x\n", state);
	ret1 = prcm_force_power_domain_state(DOM_USBHOST, PRCM_RET);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("Force of USBHOST power domain failed for state "
				"PRCM_RET\n");
		goto revert;
	}
	prcm_get_power_domain_state(DOM_USBHOST, &state);
	DPRINTK1("Current USBHOST state = %x\n", state);
	ret1 = prcm_force_power_domain_state(DOM_USBHOST, PRCM_ON);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("Force of USBHOST power domain failed for state "
				"PRCM_ON\n");
		goto revert;
	}
	prcm_get_power_domain_state(DOM_USBHOST, &state);
	DPRINTK1("Current USBHOST state = %x\n", state);
#endif

	/* IVA2 */
	DPRINTK1("Testing IVA2\n");
	ret1 = prcm_force_power_domain_state(DOM_IVA2, PRCM_OFF);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("Force of IVA2 power domain failed for state PRCM_OFF"
				"\n");
		goto revert;
	}
	prcm_get_power_domain_state(DOM_IVA2, &state);
	DPRINTK1("Current IVA2 state = %x\n", state);
	ret1 = prcm_force_power_domain_state(DOM_IVA2, PRCM_ON);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("Force of IVA2 power domain failed for state PRCM_ON"
				"\n");
		goto revert;
	}
	prcm_get_power_domain_state(DOM_IVA2, &state);
	DPRINTK1("Current IVA2 state = %x\n", state);
	ret1 = prcm_force_power_domain_state(DOM_IVA2, PRCM_RET);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("Force of IVA2 power domain failed for state PRCM_RET"
				"\n");
		goto revert;
	}
	prcm_get_power_domain_state(DOM_IVA2, &state);
	DPRINTK1("Current IVA2 state = %x\n", state);
	ret1 = prcm_force_power_domain_state(DOM_IVA2, PRCM_ON);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("Force of IVA2 power domain failed for state PRCM_ON"
				"\n");
		goto revert;
	}
	prcm_get_power_domain_state(DOM_IVA2, &state);
	DPRINTK1("Current IVA2 state = %x\n", state);

	/* CAM */
	DPRINTK1("Testing CAM\n");
	ret1 = prcm_force_power_domain_state(DOM_CAM, PRCM_OFF);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("Force of CAM power domain failed for state PRCM_OFF"
				"\n");
		goto revert;
	}
	prcm_get_power_domain_state(DOM_CAM, &state);
	DPRINTK1("Current CAM state = %x\n", state);
	ret1 = prcm_force_power_domain_state(DOM_CAM, PRCM_ON);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("Force of CAM power domain failed for state PRCM_ON"
				"\n");
		goto revert;
	}
	prcm_get_power_domain_state(DOM_CAM, &state);
	DPRINTK1("Current CAM state = %x\n", state);
	ret1 = prcm_force_power_domain_state(DOM_CAM, PRCM_RET);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("Force of CAM power domain failed for state PRCM_RET"
				"\n");
		goto revert;
	}
	prcm_get_power_domain_state(DOM_CAM, &state);
	DPRINTK1("Current CAM state = %x\n", state);
	ret1 = prcm_force_power_domain_state(DOM_CAM, PRCM_ON);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("Force of CAM power domain failed for state PRCM_ON"
				"\n");
		goto revert;
	}
	prcm_get_power_domain_state(DOM_CAM, &state);
	DPRINTK1("Current CAM state = %x\n", state);

	/* PER */
	DPRINTK1("Testing PER\n");
	ret1 = prcm_force_power_domain_state(DOM_PER, PRCM_OFF);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("Force of PER power domain failed for state PRCM_OFF"
				"\n");
		goto revert;
	}
	prcm_get_power_domain_state(DOM_PER, &state);
	DPRINTK1("Current PER state = %x\n", state);
	ret1 = prcm_force_power_domain_state(DOM_PER, PRCM_ON);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("Force of PER power domain failed for state PRCM_ON"
				"\n");
		goto revert;
	}
	prcm_get_power_domain_state(DOM_PER, &state);
	DPRINTK1("Current PER state = %x\n", state);
	ret1 = prcm_force_power_domain_state(DOM_PER, PRCM_RET);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("Force of PER power domain failed for state PRCM_RET"
				"\n");
		goto revert;
	}
	prcm_get_power_domain_state(DOM_PER, &state);
	DPRINTK1("Current PER state = %x\n", state);
	ret1 = prcm_force_power_domain_state(DOM_PER, PRCM_ON);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("Force of PER power domain failed for state PRCM_ON"
				"\n");
		goto revert;
	}
	prcm_get_power_domain_state(DOM_PER, &state);
	DPRINTK1("Current PER state = %x\n", state);

#ifndef CONFIG_ARCH_OMAP3410
	/* NEON */
	DPRINTK1("Testing NEON\n");
	/* Neon cannot come back from OFF on ES 1.0 becaue of
	 * silicon erratum. So dont test OFF */
	ret1 = prcm_force_power_domain_state(DOM_NEON, PRCM_RET);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("Force of NEON power domain failed for state PRCM_RET"
				"\n");
		goto revert;
	}
	prcm_get_power_domain_state(DOM_NEON, &state);
	DPRINTK1("Current NEON state = %x\n", state);
	ret1 = prcm_force_power_domain_state(DOM_NEON, PRCM_ON);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("Force of NEON power domain failed for state PRCM_ON"
				"\n");
		goto revert;
	}
	prcm_get_power_domain_state(DOM_NEON, &state);
	DPRINTK1("Current NEON state = %x\n", state);
#endif

	DPRINTK1("Testing multiple iterations with auto mode\n");
	DPRINTK1("First iteration goes through but for subsequent iterations, "
			"need to use sleep dependency\n");
	DPRINTK1("This is due to limitation 2.6\n");
	/* Test Multiple transitions for CAM/DSS/SGX/NEON in Auto mode */
	for (tries = 0; tries < 3; tries++) {
		DPRINTK1("\n\n");
		DPRINTK1("Iteration: %d\n", (tries + 1));
		/*Making PREPWSTST registers ON*/
		PM_PREPWSTST_CAM = 0xF;
#ifdef CONFIG_OMAP3430_ES2
#ifndef CONFIG_ARCH_OMAP3410
		PM_PREPWSTST_SGX = 0xF;
#endif
		PM_PREPWSTST_USBHOST = 0xF;
#endif
		PM_PREPWSTST_DSS = 0xF;
		PM_PREPWSTST_PER = 0xF;
#ifndef CONFIG_ARCH_OMAP3410
		PM_PREPWSTST_NEON = 0xF;
#endif

		if (tries > 0) {
			DPRINTK1("Programming sleep dependency with mpu\n");
			DPRINTK1("NEON already has sleep dependency with mpu"
					"\n");
#ifdef CONFIG_OMAP3430_ES2
#ifndef CONFIG_ARCH_OMAP3410
			CM_SLEEPDEP_SGX |= (1 << 1);
#endif
			CM_SLEEPDEP_USBHOST |= (1 << 1);
#endif
			CM_SLEEPDEP_DSS |= (1 << 1);
			CM_SLEEPDEP_CAM |= (1 << 1);
			CM_SLEEPDEP_PER |= (1 << 1);
			DPRINTK1("Printing previous power states:\n");
			prcm_get_pre_power_domain_state(DOM_CAM, &state);
			DPRINTK1("Previous  CAM state = %x\n", state);
#ifdef CONFIG_OMAP3430_ES2
#ifndef CONFIG_ARCH_OMAP3410
			prcm_get_pre_power_domain_state(DOM_SGX, &state);
			DPRINTK1("Previous  SGX state = %x\n", state);
#endif
			prcm_get_pre_power_domain_state(DOM_USBHOST, &state);
			DPRINTK1("Previous  USBHOST state = %x\n", state);
#endif
			prcm_get_pre_power_domain_state(DOM_DSS, &state);
			DPRINTK1("Previous  DSS state = %x\n", state);
			prcm_get_pre_power_domain_state(DOM_PER, &state);
			DPRINTK1("Previous  PER state = %x\n", state);
#ifndef CONFIG_ARCH_OMAP3410
			prcm_get_pre_power_domain_state(DOM_NEON, &state);
			DPRINTK1("Previous  NEON state = %x\n", state);
#endif
		}


		/* Put the domains in OFF */
#ifndef CONFIG_ARCH_OMAP3410
		DPRINTK1("Programming CAM,SGX,DSS,PER in OFF NEON in Ret\n");
#else
		DPRINTK1("Programming CAM,DSS,PER in OFF\n");
#endif
		ret |= prcm_set_power_state(DOM_CAM, PRCM_AUTO, PRCM_OFF);
#ifdef CONFIG_OMAP3430_ES2
#ifndef CONFIG_ARCH_OMAP3410
		ret |= prcm_set_power_state(DOM_SGX, PRCM_AUTO, PRCM_OFF);
#endif
		ret |= prcm_set_power_state(DOM_USBHOST, PRCM_AUTO, PRCM_OFF);
#endif
		ret |= prcm_set_power_state(DOM_DSS, PRCM_AUTO, PRCM_OFF);
#ifndef CONFIG_ARCH_OMAP3410
		ret |= prcm_set_power_state(DOM_NEON, PRCM_AUTO, PRCM_RET);
#endif
		ret |= prcm_set_power_state(DOM_PER, PRCM_AUTO, PRCM_OFF);
		if (ret) {
			printk(KERN_ERR"Error in setting the power states \n");
			goto revert;
		}

		/* Set wakeup dependencies with MPU*/
		PM_WKDEP_CAM |= (1 << 1);
#ifdef CONFIG_OMAP3430_ES2
#ifndef CONFIG_ARCH_OMAP3410
		PM_WKDEP_SGX |= (1 << 1);
#endif
		PM_WKDEP_USBHOST |= (1 << 1);
#endif
		PM_WKDEP_DSS |= (1 << 1);
		PM_WKDEP_PER |= (1 << 1);
#ifndef CONFIG_ARCH_OMAP3410
		PM_WKDEP_NEON |= (1 << 1);
#endif

		if (tries == 0) {
			/* Prog the domains to ON */
			DPRINTK1("Programming CAM/SGX/DSS/PER to ON using AUTO"
					" mode\n");
			ret |= prcm_set_power_state(DOM_CAM, PRCM_AUTO,
							PRCM_ON);
#ifdef CONFIG_OMAP3430_ES2
#ifndef CONFIG_ARCH_OMAP3410
			ret |= prcm_set_power_state(DOM_SGX, PRCM_AUTO,
							PRCM_ON);
#endif
			ret |= prcm_set_power_state(DOM_USBHOST, PRCM_AUTO,
							PRCM_ON);
#endif
			ret |= prcm_set_power_state(DOM_DSS, PRCM_AUTO,
							PRCM_ON);
			ret |= prcm_set_power_state(DOM_PER, PRCM_AUTO,
							PRCM_ON);
			if (ret) {
				printk(KERN_ERR"Error in setting the power "
					"states\n");
				goto revert;
			}
		}

		/* Program MPU for Ret and execute WFI */
		DPRINTK1("Programming MPU to go to retention using AUTO mode"
				"\n");
		prcm_set_power_state(DOM_MPU, PRCM_AUTO, PRCM_RET);
		/* wfi */
		execute_idle();

		if (tries > 0) {
			/* Prog the domains to ON */
			DPRINTK1("Programming CAM/SGX/DSS/PER to ON using AUTO"
					" mode\n");
			ret |= prcm_set_power_state(DOM_CAM, PRCM_AUTO,
							PRCM_ON);
#ifdef CONFIG_OMAP3430_ES2
#ifndef CONFIG_ARCH_OMAP3410
			ret |= prcm_set_power_state(DOM_SGX, PRCM_AUTO,
							PRCM_ON);
#endif
			ret |= prcm_set_power_state(DOM_USBHOST, PRCM_AUTO,
							PRCM_ON);
#endif
			ret |= prcm_set_power_state(DOM_DSS, PRCM_AUTO,
							PRCM_ON);
			ret |= prcm_set_power_state(DOM_PER, PRCM_AUTO,
							PRCM_ON);
			if (ret) {
				printk(KERN_ERR"Error in setting the power "
						"states \n");
				goto revert;
			}
		}

		/* Check for the previous MPU state */
		DPRINTK1("Previous MPU state = %x\n", PM_PREPWSTST_MPU);
		prcm_get_power_domain_state(DOM_CAM, &state);
		DPRINTK1("Current  CAM state = %x\n", state);
#ifdef CONFIG_OMAP3430_ES2
#ifndef CONFIG_ARCH_OMAP3410
		prcm_get_power_domain_state(DOM_SGX, &state);
		DPRINTK1("Current  SGX state = %x\n", state);
#endif
		prcm_get_power_domain_state(DOM_USBHOST, &state);
		DPRINTK1("Current  USBHOST state = %x\n", state);
#endif
		prcm_get_power_domain_state(DOM_DSS, &state);
		DPRINTK1("Current  DSS state = %x\n", state);
		prcm_get_power_domain_state(DOM_PER, &state);
		DPRINTK1("Current  PER state = %x\n", state);
#ifndef CONFIG_ARCH_OMAP3410
		prcm_get_power_domain_state(DOM_NEON, &state);
		DPRINTK1("Current  NEON state = %x\n", state);
#endif
		prcm_get_pre_power_domain_state(DOM_CAM, &state);
		DPRINTK1("Previous  CAM state = %x\n", state);
#ifdef CONFIG_OMAP3430_ES2
#ifndef CONFIG_ARCH_OMAP3410
		prcm_get_pre_power_domain_state(DOM_SGX, &state);
		DPRINTK1("Previous  SGX state = %x\n", state);
#endif
		prcm_get_pre_power_domain_state(DOM_USBHOST, &state);
		DPRINTK1("Previous  USBHOST state = %x\n", state);
#endif
		prcm_get_pre_power_domain_state(DOM_DSS, &state);
		DPRINTK1("Previous  DSS state = %x\n", state);
		prcm_get_pre_power_domain_state(DOM_PER, &state);
		DPRINTK1("Previous  PER state = %x\n", state);
#ifndef CONFIG_ARCH_OMAP3410
		prcm_get_pre_power_domain_state(DOM_NEON, &state);
		DPRINTK1("Previous  NEON state = %x\n", state);
#endif
	}

	/* Turn on all power domains */
	ret1 = prcm_force_power_domain_state(DOM_IVA2, PRCM_ON);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("prcm_force_power_domain_state DOM_IVA2 failed\n");
		goto revert;
	}
#ifdef CONFIG_OMAP3430_ES2
#ifndef CONFIG_ARCH_OMAP3410
	ret1 = prcm_force_power_domain_state(DOM_SGX, PRCM_ON);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("prcm_force_power_domain_state DOM_SGX failed\n");
		goto revert;
	}
#endif
	ret1 = prcm_force_power_domain_state(DOM_USBHOST, PRCM_ON);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("prcm_force_power_domain_state DOM_USBHOST failed\n");
		goto revert;
	}
#endif
	ret1 = prcm_force_power_domain_state(DOM_DSS, PRCM_ON);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("prcm_force_power_domain_state DOM_DSS failed\n");
		goto revert;
	}
	ret1 = prcm_force_power_domain_state(DOM_CAM, PRCM_ON);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("prcm_force_power_domain_state DOM_CAM failed\n");
		goto revert;
	}
	ret1 = prcm_force_power_domain_state(DOM_PER, PRCM_ON);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("prcm_force_power_domain_state DOM_PER failed\n");
		goto revert;
	}
#ifndef CONFIG_ARCH_OMAP3410
	ret1 = prcm_force_power_domain_state(DOM_NEON, PRCM_ON);
	if (ret1 != PRCM_PASS) {
		DPRINTK1("prcm_force_power_domain_state DOM_NEON failed\n");
		goto revert;
	}
#endif
revert:
#ifndef CONFIG_OMAP3430_ES2
	/* Restore DSS related registers */
	PRCM_DSS_SYSCONFIG = dss_sysconfig;

	PRCM_DISPC_SYSCONFIG = dispc_sysconfig;

	PRCM_RFBI_SYSCONFIG = rfbi_sysconfig;
#endif
	/* Restore sleep/wakeup dependencies */
#ifdef CONFIG_OMAP3430_ES2
#ifndef CONFIG_ARCH_OMAP3410
	CM_SLEEPDEP_SGX = cm_sleepdep_sgx;
	PM_WKDEP_SGX = pm_wkdep_sgx;
#endif
	CM_SLEEPDEP_USBHOST = cm_sleepdep_usb;
	PM_WKDEP_USBHOST = pm_wkdep_usb;
#endif
	CM_SLEEPDEP_DSS = cm_sleepdep_dss;
	CM_SLEEPDEP_CAM = cm_sleepdep_cam;
	CM_SLEEPDEP_PER = cm_sleepdep_per;
	PM_WKDEP_IVA2 = pm_wkdep_iva2;
	PM_WKDEP_MPU = pm_wkdep_mpu;
	PM_WKDEP_DSS = pm_wkdep_dss;
	PM_WKDEP_CAM = pm_wkdep_cam;
	PM_WKDEP_PER = pm_wkdep_per;
#ifndef CONFIG_ARCH_OMAP3410
	PM_WKDEP_NEON = pm_wkdep_neon;
#endif

	/* Restore clock registers */
	CM_FCLKEN_IVA2 = ivaf;
	CM_FCLKEN1_CORE = core1f;
	CM_ICLKEN1_CORE = core1i;
	CM_ICLKEN2_CORE = core2i;
#ifdef CONFIG_OMAP3430_ES2
#ifndef CONFIG_ARCH_OMAP3410
	CM_FCLKEN_SGX = sgxf;
	CM_ICLKEN_SGX = sgxi;
#endif
	CM_FCLKEN_USBHOST = usbf;
	CM_ICLKEN_USBHOST = usbi;
#endif
	CM_FCLKEN_WKUP = wkupf;
	CM_ICLKEN_WKUP = wkupi;
	CM_FCLKEN_DSS = dssf;
	CM_ICLKEN_DSS = dssi;
	CM_FCLKEN_CAM = camf;
	CM_ICLKEN_CAM = cami;
	CM_FCLKEN_PER = perf;
	CM_ICLKEN_PER = peri;

	if (ret || (ret1 != PRCM_PASS))
		return -1;

	return 0;
}
EXPORT_SYMBOL(powerapi_test);

int clkapi_test(void)
{
	u32 cm_clksel_core, cm_clksel_wkup, cm_clkout_ctrl;
	u32 cm_clksel_per, cm_clksel1_pll;
	u32 divider = 0, div, result;
	u32 proc_speed;
	u8 state;
	int ret = 0;
#if defined(CONFIG_OMAP3430_ES2) && !defined(CONFIG_ARCH_OMAP3410)
	u32 cm_clksel_sgx;
#endif

	/*baskup the clock registers*/
	cm_clksel_core = CM_CLKSEL_CORE;
	cm_clksel_wkup = CM_CLKSEL_WKUP;
	cm_clkout_ctrl = CM_CLKOUT_CTRL;
#if defined(CONFIG_OMAP3430_ES2) && !defined(CONFIG_ARCH_OMAP3410)
	cm_clksel_sgx = CM_CLKSEL_SGX;
#endif
	cm_clksel_per = CM_CLKSEL_PER;
	cm_clksel1_pll = CM_CLKSEL1_PLL;

	/*Test code for clock divisor setting and reading*/
	if (!prcm_clksel_set_divider(PRCM_48M_FCLK, 1))
		DPRINTK1("Divider for PRCM_48M_FCLK set to 1\n");
	if (!prcm_clksel_get_divider(PRCM_48M_FCLK, &divider))
		DPRINTK1("Divider for PRCM_48M_FCLK is %d\n", divider);
	if (!prcm_clksel_set_divider(PRCM_12M_FCLK, 2))
		DPRINTK1("Divider for PRCM_12M_FCLK set to 2\n");
	if (!prcm_clksel_get_divider(PRCM_12M_FCLK, &divider))
		DPRINTK1("Divider for PRCM_12M_FCLK is %d\n", divider);
	if (!prcm_clksel_set_divider(PRCM_SYS_CLKOUT2, 16))
		DPRINTK1("Divider for PRCM_SYS_CLKOUT2 set to 16\n");
	if (!prcm_clksel_get_divider(PRCM_SYS_CLKOUT2, &divider))
		DPRINTK1("Divider for PRCM_SYS_CLKOUT2 is %d\n", divider);
	if (!prcm_clksel_set_divider(PRCM_L3_ICLK, 2))
		DPRINTK1("Divider for PRCM_L3_ICLK set to 2\n");
	if (!prcm_clksel_get_divider(PRCM_L3_ICLK, &divider))
		DPRINTK1("Divider for PRCM_L3_ICLK %d\n", divider);
	if (!prcm_clksel_set_divider(PRCM_L4_ICLK, 1))
		DPRINTK1("Divider for PRCM_L4_ICLK set to 1\n");
	if (!prcm_clksel_get_divider(PRCM_L4_ICLK, &divider))
		DPRINTK1("Divider for PRCM_L4_ICLK is %d\n", divider);
#ifndef CONFIG_OMAP3430_ES2
	if (!prcm_clksel_set_divider(PRCM_FSHOST, 1))
		DPRINTK1("Divider for PRCM_FSHOST set to 1\n");
	if (!prcm_clksel_get_divider(PRCM_FSHOST, &divider))
		DPRINTK1("Divider for PRCM_FSHOST is %d\n", divider);
#endif
	if (!prcm_clksel_set_divider(PRCM_RM_ICLK, 1))
		DPRINTK1("Divider for PRCM_RM_ICLK set to 1\n");
	if (!prcm_clksel_get_divider(PRCM_RM_ICLK, &divider))
		DPRINTK1("Divider for PRCM_RM_ICLK is %d\n", divider);
#ifdef CONFIG_OMAP3430_ES2
#ifndef CONFIG_ARCH_OMAP3410
	if (!prcm_clksel_set_divider(PRCM_SGX_FCLK, 3))
		DPRINTK1("Divider for PRCM_SGX_FCLK set to 3\n");
	if (!prcm_clksel_get_divider(PRCM_SGX_FCLK, &divider))
		DPRINTK1("Divider for PRCM_SGX_FCLK is %d\n", divider);
#endif
	if (!prcm_clksel_set_divider(PRCM_USIM, 10))
		DPRINTK1("Divider for PRCM_USIM set to 10\n");
	if (!prcm_clksel_get_divider(PRCM_USIM, &divider))
		DPRINTK1("Divider for PRCM_USIM is %d\n", divider);

#endif
	if (!prcm_clksel_set_divider(PRCM_SSI, 1))
		DPRINTK1("Divider for PRCM_SSI set to 1\n");
	if (!prcm_clksel_get_divider(PRCM_SSI, &divider))
		DPRINTK1("Divider for PRCM_SSI is %d\n", divider);
	/* Test code for clock source setting */
	ret = prcm_clk_set_source(PRCM_MCBSP1, PRCM_SYS_32K_CLK);
	if (ret == PRCM_PASS)
		DPRINTK1("Source clock for PRCM_MCBSP1 set to PRCM_SYS_32K_CLK"
				"\n");
	ret = prcm_clk_set_source(PRCM_MCBSP2, PRCM_SYS_32K_CLK);
	if (ret == PRCM_PASS)
		DPRINTK1("Source clock for PRCM_MCBSP2 set to PRCM_SYS_32K_CLK"
				"\n");
	ret = prcm_clk_set_source(PRCM_MCBSP3, PRCM_SYS_32K_CLK);
	if (ret == PRCM_PASS)
		DPRINTK1("Source clock for PRCM_MCBSP3 set to PRCM_SYS_32K_CLK"
				"\n");
	ret = prcm_clk_set_source(PRCM_MCBSP4, PRCM_SYS_32K_CLK);
	if (ret == PRCM_PASS)
		DPRINTK1("Source clock for PRCM_MCBSP4 set to PRCM_SYS_32K_CLK"
				"\n");
	ret = prcm_clk_set_source(PRCM_MCBSP5, PRCM_SYS_32K_CLK);
	if (ret == PRCM_PASS)
		DPRINTK1("Source clock for PRCM_MCBSP5 set to PRCM_SYS_32K"
				"_CLK\n");
	ret = prcm_clk_set_source(PRCM_SYS_CLKOUT2, PRCM_SYS_CLK);
	if (ret == PRCM_PASS)
		DPRINTK1("Source clock for PRCM_SYS_CLKOUT2 set to PRCM_SYS"
				"_CLK\n");
	ret = prcm_clk_set_source(PRCM_GPT1, PRCM_SYS_32K_CLK);
	if (ret == PRCM_PASS)
		DPRINTK1("Source clock for PRCM_GPT1 set to PRCM_SYS_32K"
				"_CLK\n");
	ret = prcm_clk_set_source(PRCM_GPT2, PRCM_SYS_32K_CLK);
	if (ret == PRCM_PASS)
		DPRINTK1("Source clock for PRCM_GPT2 set to PRCM_SYS_32K"
				"_CLK\n");
	ret = prcm_clk_set_source(PRCM_GPT3, PRCM_SYS_32K_CLK);
	if (ret == PRCM_PASS)
		DPRINTK1("Source clock for PRCM_GPT3 set to PRCM_SYS_32K"
				"_CLK\n");
	ret = prcm_clk_set_source(PRCM_GPT4, PRCM_SYS_32K_CLK);
	if (ret == PRCM_PASS)
		DPRINTK1("Source clock for PRCM_GPT4 set to PRCM_SYS_32K"
				"_CLK\n");
	ret = prcm_clk_set_source(PRCM_GPT5, PRCM_SYS_32K_CLK);
	if (ret == PRCM_PASS)
		DPRINTK1("Source clock for PRCM_GPT5 set to PRCM_SYS_32K"
				"_CLK\n");
	ret = prcm_clk_set_source(PRCM_GPT6, PRCM_SYS_32K_CLK);
	if (ret == PRCM_PASS)
		DPRINTK1("Source clock for PRCM_GPT6 set to PRCM_SYS_32K_"
				"CLK\n");
	ret = prcm_clk_set_source(PRCM_GPT7, PRCM_SYS_32K_CLK);
	if (ret == PRCM_PASS)
		DPRINTK1("Source clock for PRCM_GPT7 set to PRCM_SYS_32K_"
				"CLK\n");
	ret = prcm_clk_set_source(PRCM_GPT8, PRCM_SYS_32K_CLK);
	if (ret == PRCM_PASS)
		DPRINTK1("Source clock for PRCM_GPT8 set to PRCM_SYS_32K_CLK"
				"\n");
	ret = prcm_clk_set_source(PRCM_GPT9, PRCM_SYS_32K_CLK);
	if (ret == PRCM_PASS)
		DPRINTK1("Source clock for PRCM_GPT9 set to PRCM_SYS_32K_CLK"
				"\n");
	ret = prcm_clk_set_source(PRCM_GPT10, PRCM_SYS_32K_CLK);
	if (ret == PRCM_PASS)
		DPRINTK1("Source clock for PRCM_GPT10 set to PRCM_SYS_32K_CLK"
				"\n");
	ret = prcm_clk_set_source(PRCM_GPT11, PRCM_SYS_32K_CLK);
	if (ret == PRCM_PASS)
		DPRINTK1("Source clock for PRCM_GPT11 set to PRCM_SYS_32K_CLK"
				"\n");
#ifdef CONFIG_OMAP3430_ES2
	ret = prcm_clk_set_source(PRCM_USIM, PRCM_SYS_CLK);
	if (ret == PRCM_PASS)
		 DPRINTK1("Source clock for PRCM_USIM set to PRCM_SYS_CLK\n");
	ret = prcm_clk_set_source(PRCM_USIM, PRCM_DPLL4_M2X2_CLK);
	if (ret == PRCM_PASS)
		 DPRINTK1("Source clock for PRCM_USIM set to PRCM_DPLL4_M2X2"
				"_CLK\n");
	ret = prcm_clk_set_source(PRCM_USIM, PRCM_DPLL5_M2_CLK);
	if (ret == PRCM_PASS)
		 DPRINTK1("Source clock for PRCM_USIM set to PRCM_DPLL5_M2_CLK"
				"\n");
	ret = prcm_clk_set_source(PRCM_96M_CLK, PRCM_DPLL4_M2X2_CLK);
	if (ret == PRCM_PASS)
		 DPRINTK1("Source clock for PRCM_96M_CLK set to PRCM_DPLL4_"
				"M2X2_CLK\n");
	ret = prcm_clk_set_source(PRCM_96M_CLK, PRCM_SYS_CLK);
	if (ret == PRCM_PASS)
		 DPRINTK1("Source clock for PRCM_96M_CLK set to PRCM_SYS_CLK"
				"\n");
#endif
	if (ret != PRCM_PASS) {
		DPRINTK1("Some prcm_clk_set_source was not successful \n");
		goto revert;
	}

	if (!prcm_get_processor_speed(DOM_MPU, &proc_speed))
		DPRINTK1("MPU speed = %d\n", proc_speed);
	if (!prcm_get_processor_speed(DOM_IVA2, &proc_speed))
		DPRINTK1("IVA2 speed = %d\n", proc_speed);

	if (!prcm_clksel_round_rate(PRCM_SSI, 38000, 38000, &div))
		DPRINTK1("P-rate 38M,T-rate 38M,DIV=%d\n", div);
	if (!prcm_clksel_round_rate(PRCM_SSI, 38000, 19000, &div))
		DPRINTK1("P-rate 38M,T-rate 19M,DIV=%d\n", div);
	if (!prcm_clksel_round_rate(PRCM_SSI, 38000, 13000, &div))
		DPRINTK1("P-rate 38M,T-rate 9M,DIV=%d\n", div);
	if (!prcm_clksel_round_rate(PRCM_SSI, 38000, 9000, &div))
		DPRINTK1("P-rate 38M,T-rate 5M,DIV=%d\n", div);
	if (!prcm_clksel_round_rate(PRCM_SSI, 38000, 6000, &div))
		DPRINTK1("P-rate 38M,T-rate 3M,DIV=%d\n", div);
	if (!prcm_clksel_round_rate(PRCM_SSI, 38000, 2000, &div))
		DPRINTK1("P-rate 38M,T-rate 2M,DIV=%d\n", div);
	if (!prcm_clksel_round_rate(PRCM_SSI, 38000, 39000, &div))
		DPRINTK1("P-rate 38M,T-rate 39M,DIV=%d\n", div);
	if (!prcm_clksel_round_rate(PRCM_DPLL3_M3X2_CLK, 96000, 24000, &div))
		DPRINTK1("P-rate 96M,T-rate 24M,DIV=%d\n", div);

	prcm_is_clock_domain_active(DOM_IVA2, &state);
	if (state)
		DPRINTK1("IVA2 domain is active\n");
	else
		DPRINTK1("IVA2 domain is inactive\n");

	prcm_get_devices_not_idle(DOM_CORE1, &result);
	DPRINTK1("Devices Not Idle Mask for CORE1 domain %x\n", result);

	prcm_get_initiators_not_standby(DOM_CORE1, &result);
	DPRINTK1("Devices Not Standby Mask for CORE1 domain %x\n", result);

	prcm_get_domain_functional_clocks(DOM_CORE1, &result);
	DPRINTK1("functional clock Mask for CORE1 domain %x\n", result);

	prcm_get_domain_interface_clocks(DOM_CORE1, &result);
	DPRINTK1("Interface clock Mask for CORE1 domain %x\n", result);

revert:
	/* Restore the clock registers */
	CM_CLKSEL_CORE = cm_clksel_core;
	CM_CLKSEL_WKUP = cm_clksel_wkup;
	CM_CLKOUT_CTRL = cm_clkout_ctrl;
#if defined(CONFIG_OMAP3430_ES2) && !defined(CONFIG_ARCH_OMAP3410)
	CM_CLKSEL_SGX = cm_clksel_sgx;
#endif
	CM_CLKSEL_PER = cm_clksel_per;
	CM_CLKSEL1_PLL = cm_clksel1_pll;

	if (ret != PRCM_PASS)
		return -1;

	return 0;
}
EXPORT_SYMBOL(clkapi_test);

int dpllapi_test(void)
{
	u32 dpll_rate;
	int ret = 0;

	/* Enable all DPLL's if not already done*/
	if (prcm_enable_dpll(DPLL1_MPU)) {
		DPRINTK1("DPLL1 enable failed\n");
		return -1;
	}
	if (prcm_enable_dpll(DPLL2_IVA2)) {
		DPRINTK1("DPLL2 enable failed\n");
		return -1;
	}
#ifdef CONFIG_OMAP3430_ES2
	/* Disable autoidle for DPLL5 during this test case is active */
	CM_AUTOIDLE2_PLL = 0x0;
	if (prcm_enable_dpll(DPLL5_PER2)) {
		DPRINTK1("DPLL5 enable failed\n");
		return -1;
	}
#endif
	/* Get all possible DPLL output rates */
	prcm_get_dpll_rate(PRCM_DPLL1_M2X2_CLK, &dpll_rate);
	DPRINTK1("PRCM_DPLL1_M2X2_CLK = %d\n", dpll_rate);
	prcm_get_dpll_rate(PRCM_DPLL2_M2X2_CLK, &dpll_rate);
	DPRINTK1("PRCM_DPLL2_M2X2_CLK = %d\n", dpll_rate);
	prcm_get_dpll_rate(PRCM_DPLL3_M2_CLK, &dpll_rate);
	DPRINTK1("PRCM_DPLL3_M2_CLK   = %d\n", dpll_rate);
	prcm_get_dpll_rate(PRCM_DPLL3_M2X2_CLK, &dpll_rate);
	DPRINTK1("PRCM_DPLL3_M2X2_CLK = %d\n", dpll_rate);
	prcm_get_dpll_rate(PRCM_DPLL3_M3X2_CLK, &dpll_rate);
	DPRINTK1("PRCM_DPLL3_M3X2_CLK = %d\n", dpll_rate);
#ifdef CONFIG_OMAP3430_ES2
	prcm_get_dpll_rate(PRCM_DPLL5_M2_CLK, &dpll_rate);
	DPRINTK1("PRCM_DPLL5_M2_CLK = %d\n", dpll_rate);
#endif
	prcm_get_dpll_rate(PRCM_DSS, &dpll_rate);
	DPRINTK1("PRCM_DSS = %d\n", dpll_rate);
	prcm_get_dpll_rate(PRCM_CAM, &dpll_rate);
	DPRINTK1("PRCM_CAM = %d\n", dpll_rate);

	/* Put DPLL1/DPLL2 in BYPASS to change the configurations */
	if (prcm_put_dpll_in_bypass(DPLL1_MPU, LOW_POWER_BYPASS)
							!= PRCM_PASS) {
		DPRINTK1("Unable to put DPLL1 in Bypass\n");
		return -1;
	}
	if (prcm_put_dpll_in_bypass(DPLL2_IVA2, LOW_POWER_BYPASS)
							!= PRCM_PASS) {
		DPRINTK1("Unable to put DPLL2 in Bypass\n");
		goto revert0;
	}
#ifdef CONFIG_OMAP3430_ES2
	if (prcm_put_dpll_in_bypass(DPLL5_PER2, LOW_POWER_STOP)
							!= PRCM_PASS) {
		DPRINTK1("Unable to put DPLL5 in Bypass\n");
		goto revert1;
	}
#endif
	/* Get the revised rates once they are in bypass  */
	prcm_get_dpll_rate(PRCM_DPLL1_M2X2_CLK, &dpll_rate);
	DPRINTK1("PRCM_DPLL1_M2X2_CLK = %d\n", dpll_rate);
	prcm_get_dpll_rate(PRCM_DPLL2_M2X2_CLK, &dpll_rate);
	DPRINTK1("PRCM_DPLL2_M2X2_CLK = %d\n", dpll_rate);
	prcm_get_dpll_rate(PRCM_DPLL3_M2_CLK, &dpll_rate);
	DPRINTK1("PRCM_DPLL3_M2_CLK   = %d\n", dpll_rate);
	prcm_get_dpll_rate(PRCM_DPLL3_M2X2_CLK, &dpll_rate);
	DPRINTK1("PRCM_DPLL3_M2X2_CLK = %d\n", dpll_rate);
	prcm_get_dpll_rate(PRCM_DPLL3_M3X2_CLK, &dpll_rate);
	DPRINTK1("PRCM_DPLL3_M3X2_CLK = %d\n", dpll_rate);
	prcm_get_dpll_rate(PRCM_DSS, &dpll_rate);
#ifdef CONFIG_OMAP3430_ES2
	prcm_get_dpll_rate(PRCM_DPLL5_M2_CLK, &dpll_rate);
	DPRINTK1("PRCM_DPLL5_M2_CLK = %d\n", dpll_rate);
#endif
	DPRINTK1("PRCM_DSS = %d\n", dpll_rate);
	prcm_get_dpll_rate(PRCM_CAM, &dpll_rate);
	DPRINTK1("PRCM_CAM = %d\n", dpll_rate);

#ifdef CONFIG_OMAP3430_ES2
	/* Configure M=651,N=19,FREQ=0x3,M2=2 for DPLL1*/
	if (prcm_configure_dpll(DPLL1_MPU, 0x28B, 0x13, 0x3) != PRCM_PASS) {
		printk("1 - Unable to configure DPLL1\n");
		goto revert1;
	}
	if (prcm_configure_dpll(DPLL2_IVA2, 0x203, 0x16, 0x3) != PRCM_PASS) {
		printk(KERN_ERR"2 - Unable to configure DPLL2\n");
		goto revert1;
	}
	if (prcm_configure_dpll(DPLL5_PER2, 0x4B, 0xB, 0x6) != PRCM_PASS) {
		printk(KERN_ERR"3 - Unable to configure DPLL5\n");
		goto revert1;
	}
#else
	/* Configure M=248,N=9,FREQ=0x7,M2=2 for DPLL1*/
	if (prcm_configure_dpll(DPLL1_MPU, 0xF8, 0x9, 0x7) != PRCM_PASS) {
		printk(KERN_ERR"1 - Unable to configure DPLL1\n");
		goto revert1;
	}
	if (prcm_configure_dpll(DPLL2_IVA2, 0xB3, 0xA, 0x6) != PRCM_PASS) {
		printk(KERN_ERR"2 - Unable to configure DPLL2\n");
		goto revert1;
	}
#endif
	if (prcm_configure_dpll_divider(PRCM_DPLL1_M2X2_CLK, 0x2)
				!= PRCM_PASS) {
		printk(KERN_ERR"Unable to configure PRCM_DPLL1_M2X2_CLK\n");
		goto revert1;
	}
	if (prcm_configure_dpll_divider(PRCM_DPLL2_M2X2_CLK, 0x2)
				!= PRCM_PASS) {
		printk(KERN_ERR"Unable to configure PRCM_DPLL2_M2X2_CLK\n");
		goto revert1;
	}

	/* Re-enable DPLL1/DPLL2/DPLL5 */
	if (prcm_enable_dpll(DPLL1_MPU) != PRCM_PASS) {
		DPRINTK1("DPLL1 enable failed\n");
		ret = -2;
		goto revert1;
	}
	if (prcm_enable_dpll(DPLL2_IVA2) != PRCM_PASS) {
		DPRINTK1("DPLL2 enable failed\n");
		return -1;
	}
#ifdef CONFIG_OMAP3430_ES2
	if (prcm_enable_dpll(DPLL5_PER2) != PRCM_PASS) {
		DPRINTK1("DPLL5 enable failed\n");
		return -1;
	}
#endif

	/* Get the new rates  */
	prcm_get_dpll_rate(PRCM_DPLL1_M2X2_CLK, &dpll_rate);
	DPRINTK1("PRCM_DPLL1_M2X2_CLK = %d\n", dpll_rate);
	prcm_get_dpll_rate(PRCM_DPLL2_M2X2_CLK, &dpll_rate);
	DPRINTK1("PRCM_DPLL2_M2X2_CLK = %d\n", dpll_rate);
	prcm_get_dpll_rate(PRCM_DPLL3_M2_CLK, &dpll_rate);
	DPRINTK1("PRCM_DPLL3_M2_CLK   = %d\n", dpll_rate);
	prcm_get_dpll_rate(PRCM_DPLL3_M2X2_CLK, &dpll_rate);
	DPRINTK1("PRCM_DPLL3_M2X2_CLK = %d\n", dpll_rate);
	prcm_get_dpll_rate(PRCM_DPLL3_M3X2_CLK, &dpll_rate);
	DPRINTK1("PRCM_DPLL3_M3X2_CLK = %d\n", dpll_rate);
	prcm_get_dpll_rate(PRCM_DSS, &dpll_rate);
#ifdef CONFIG_OMAP3430_ES2
	prcm_get_dpll_rate(PRCM_DPLL5_M2_CLK, &dpll_rate);
	DPRINTK1("PRCM_DPLL5_M2_CLK = %d\n", dpll_rate);
#endif
	DPRINTK1("PRCM_DSS = %d\n", dpll_rate);
	prcm_get_dpll_rate(PRCM_CAM, &dpll_rate);
	DPRINTK1("PRCM_CAM = %d\n", dpll_rate);

	if (prcm_put_dpll_in_bypass(DPLL2_IVA2, LOW_POWER_BYPASS)
							!= PRCM_PASS) {
		DPRINTK1("Unable to put DPLL2 in Bypass\n");
		return -1;
	}
	/* Configure M2=1 for DPLL2 */
	if (prcm_configure_dpll_divider(PRCM_DPLL2_M2X2_CLK, 0x1)
				!= PRCM_PASS) {
		printk(KERN_INFO"a - Unable to configure PRCM_DPLL2_M2X2_CLK"
				"\n");
		ret = -2;
		goto revert1;
	}
	/* Need to replace the old values back */
	if (prcm_put_dpll_in_bypass(DPLL1_MPU, LOW_POWER_BYPASS)
							!= PRCM_PASS) {
		DPRINTK1("Unable to put DPLL1 in Bypass\n");
		return -1;
	}
#ifndef CONFIG_OMAP3430_ES2
	/* Configure M=377,N=18,M2=1 for DPLL1 */
	if (prcm_configure_dpll(DPLL1_MPU,
				0x179, 0x12, 0x4) != PRCM_PASS) {
		printk(KERN_ERR"1a - Unable to configure PRCM_DPLL1_M2X2_CLK"
				"\n");
		goto revert0;
	}
#endif
	if (prcm_configure_dpll_divider(PRCM_DPLL1_M2X2_CLK, 0x1)
				!= PRCM_PASS) {
		printk(KERN_ERR"2a - Unable to configure PRCM_DPLL1_M2X2_CLK"
				"\n");
		goto revert0;
	}

	/* Re-enable DPLL1/DPLL2 */
	if (prcm_enable_dpll(DPLL1_MPU) != PRCM_PASS) {
		DPRINTK1("a - DPLL1 enable failed\n");
		ret = -2;
		goto revert1;
	}
	if (prcm_enable_dpll(DPLL2_IVA2) != PRCM_PASS) {
		DPRINTK1("a - DPLL2 enable failed\n");
		return -1;
	}
#ifdef CONFIG_OMAP3430_ES2
	if (prcm_enable_dpll(DPLL5_PER2) != PRCM_PASS) {
		DPRINTK1("DPLL5 enable failed\n");
		return -1;
	}
	/* Re-enable DPLL5 autoidle */
	CM_AUTOIDLE2_PLL = 0x1;
#endif
	return 0;

revert1:
	ret = prcm_enable_dpll(DPLL2_IVA2);
	if (ret != PRCM_PASS)
		DPRINTK1("a - DPLL2 enable failed\n");

	if (ret == -2)
		return -1;

revert0:
	if (prcm_enable_dpll(DPLL1_MPU))
		DPRINTK1("a - DPLL1 enable failed\n");

	return -1;
}
EXPORT_SYMBOL(dpllapi_test);

int voltage_scaling_tst(u32 target_opp_id, u32 current_opp_id)
{
	u32 curr_opp_no, target_opp_no;
	u32 vdd;
	struct clk *p_vdd1_clk;
	struct clk *p_vdd2_clk;
	u32 valid_rate;

	DPRINTK1("\n");

	vdd = get_vdd(target_opp_id);

	p_vdd1_clk = clk_get(NULL, "virt_vdd1_prcm_set");
	p_vdd2_clk = clk_get(NULL, "virt_vdd2_prcm_set");
	curr_opp_no = current_opp_id & OPP_NO_MASK;
	target_opp_no = target_opp_id & OPP_NO_MASK;

	if (vdd == PRCM_VDD1) {
		DPRINTK1("Before - VDD1 rate is %lu\n",
				clk_get_rate(p_vdd1_clk));
		DPRINTK1("Before - BogoMIPS\t: %lu.%02lu\n",
		   loops_per_jiffy / (500000/HZ),
		   (loops_per_jiffy / (5000/HZ)) % 100);
		if (scale_voltage_then_frequency) {
			prcm_do_voltage_scaling(target_opp_id, current_opp_id);
			valid_rate = clk_round_rate(p_vdd1_clk,
				round_rate_vdd1[target_opp_no-1]);
			p_vdd1_clk->set_rate(p_vdd1_clk, valid_rate);
		} else {
			valid_rate = clk_round_rate(p_vdd1_clk,
					round_rate_vdd1[target_opp_no-1]);
			p_vdd1_clk->set_rate(p_vdd1_clk, valid_rate);
			prcm_do_voltage_scaling(target_opp_id, current_opp_id);
		}
		DPRINTK1("After - VDD1 rate is %lu\n",
				clk_get_rate(p_vdd1_clk));
		DPRINTK1("After - BogoMIPS\t: %lu.%02lu\n",
		   loops_per_jiffy / (500000/HZ),
		   (loops_per_jiffy / (5000/HZ)) % 100);

	} else if (vdd == PRCM_VDD2) {
		DPRINTK1("Before - VDD2 rate is %lu\n",
				clk_get_rate(p_vdd2_clk));
		if (scale_voltage_then_frequency) {
			prcm_do_voltage_scaling(target_opp_id, current_opp_id);
			valid_rate = clk_round_rate(p_vdd2_clk,
					round_rate_vdd2[target_opp_no-1]);
			p_vdd2_clk->set_rate(p_vdd2_clk, valid_rate);
		} else {
			valid_rate = clk_round_rate(p_vdd2_clk,
					round_rate_vdd2[target_opp_no-1]);
			p_vdd2_clk->set_rate(p_vdd2_clk, valid_rate);
			prcm_do_voltage_scaling(target_opp_id, current_opp_id);
		}
		DPRINTK1("After - VDD2 rate is %lu\n",
				clk_get_rate(p_vdd2_clk));
	}
	return 0;
}

int voltage_scaling_set_of_test (void)
{
	int i, e = 0;

	DPRINTK1("#### Start of Voltage Scaling tests \n");

	ssleep(5);
	e |= voltage_scaling_tst(PRCM_VDD1_OPP1, current_vdd1_opp);

	ssleep(5);
	e |= voltage_scaling_tst(PRCM_VDD1_OPP2, PRCM_VDD1_OPP1);

	ssleep(5);
	e |= voltage_scaling_tst(PRCM_VDD1_OPP3, PRCM_VDD1_OPP2);

	ssleep(5);
	e |= voltage_scaling_tst(PRCM_VDD1_OPP4, PRCM_VDD1_OPP3);

#ifdef CONFIG_OMAP3430_ES2
	ssleep(5);
	e |= voltage_scaling_tst(PRCM_VDD1_OPP5, PRCM_VDD1_OPP4);
	if (current_vdd1_opp != PRCM_VDD1_OPP5) {
		ssleep(5);
		e |= voltage_scaling_tst(current_vdd1_opp, PRCM_VDD1_OPP5);
	}
#else
	if (current_vdd1_opp != PRCM_VDD1_OPP4) {
		ssleep(5);
		e |= voltage_scaling_tst(current_vdd1_opp, PRCM_VDD1_OPP4);
	}
#endif
	for (i = 0; i < 2; i++) {
		ssleep(5);
		e |= voltage_scaling_tst(PRCM_VDD2_OPP2, current_vdd2_opp);
		ssleep(5);
		e |= voltage_scaling_tst(current_vdd2_opp, PRCM_VDD2_OPP2);
		if (e)
			break;
	}

	DPRINTK1("#### End of Voltage Scaling tests \n");
	return e;
}
EXPORT_SYMBOL(voltage_scaling_set_of_test);

int frequency_scaling_test(void)
{
	u32 dpll_rate;

	prcm_get_dpll_rate(PRCM_DPLL3_M2_CLK, &dpll_rate);
	DPRINTK1("PRCM_DPLL3_M2_CLK   = %d\n", dpll_rate);

#ifdef CONFIG_OMAP3430_ES2
	if (prcm_do_frequency_scaling(PRCM_VDD2_OPP2, PRCM_VDD2_OPP3)
							!= PRCM_PASS) {
#else
	if (prcm_do_frequency_scaling(PRCM_VDD2_OPP2, PRCM_VDD2_OPP1)
							!= PRCM_PASS) {
#endif
		DPRINTK1("Error in prcm_do_frequency_scaling for VDD2\n");
		return -1;
	} else
		DPRINTK1("\nDone Freq scaling DONE VDD2.... \n\n");

	prcm_get_dpll_rate(PRCM_DPLL3_M2_CLK, &dpll_rate);
	DPRINTK1("PRCM_DPLL3_M2_CLK   = %d\n", dpll_rate);

#ifdef CONFIG_OMAP3430_ES2
	if (prcm_do_frequency_scaling(PRCM_VDD2_OPP3, PRCM_VDD2_OPP2)
							!= PRCM_PASS) {
#else
	if (prcm_do_frequency_scaling(PRCM_VDD2_OPP1, PRCM_VDD2_OPP2)
							!= PRCM_PASS) {
#endif
		DPRINTK1("Error in prcm_do_frequency_scaling for VDD2\n");
		return -1;
	} else
		DPRINTK1("\nDone Freq scaling DONE VDD2.... \n\n");

	prcm_get_dpll_rate(PRCM_DPLL3_M2_CLK, &dpll_rate);
	DPRINTK1("PRCM_DPLL3_M2_CLK   = %d\n", dpll_rate);

	return 0;
}
EXPORT_SYMBOL(frequency_scaling_test);

void constraint_test(unsigned int co_type, unsigned long target_value,
			int action)
{
	switch (co_type) {
	case RES_LATENCY_CO:
		if (action == REQUEST_CO) {
			co_latency = constraint_get("test",
					&cnstr_id_lat);
			constraint_set(co_latency, target_value);
		} else if (action == RELEASE_CO) {
			constraint_remove(co_latency);
			constraint_put(co_latency);
		}
		break;
	case PRCM_VDD1_CONSTRAINT:
		if (action == REQUEST_CO) {
			co_opp = constraint_get("test", &cnstr_id_vdd1);
			constraint_register_pre_notification(co_opp,
					&nb_test_pre, MAX_VDD1_OPP+1);
			constraint_register_post_notification(co_opp,
					&nb_test_post, MAX_VDD1_OPP+1);
			constraint_set(co_opp, target_value);
		} else if (action == RELEASE_CO) {
			constraint_remove(co_opp);
			constraint_unregister_pre_notification(co_opp,
					&nb_test_pre, MAX_VDD1_OPP+1);
			constraint_unregister_post_notification(co_opp,
					&nb_test_post, MAX_VDD1_OPP+1);
			constraint_put(co_opp);
		}
		break;
	case PRCM_VDD2_CONSTRAINT:
		if (action == REQUEST_CO) {
			co_opp = constraint_get("test", &cnstr_id_vdd2);
			constraint_register_pre_notification(co_opp,
					&nb_test_pre, MAX_VDD2_OPP+1);
			constraint_register_post_notification(co_opp,
					&nb_test_post, MAX_VDD2_OPP+1);
			constraint_set(co_opp, target_value);
		} else if (action == RELEASE_CO) {
			constraint_remove(co_opp);
			constraint_unregister_pre_notification(co_opp,
					&nb_test_pre, MAX_VDD2_OPP+1);
			constraint_unregister_post_notification(co_opp,
					&nb_test_post, MAX_VDD2_OPP+1);
			constraint_put(co_opp);
		}
		break;
	case PRCM_ARMFREQ_CONSTRAINT:
		if (action == REQUEST_CO) {
			co_opp = constraint_get("test", &cnstr_id_arm);
			constraint_register_pre_notification(co_opp,
					&nb_test_pre, -1);
			constraint_register_post_notification(co_opp,
					&nb_test_post, -1);
			constraint_set(co_opp, target_value);
		} else if (action == RELEASE_CO) {
			constraint_remove(co_opp);
			constraint_unregister_pre_notification(co_opp,
					&nb_test_pre, -1);
			constraint_unregister_post_notification(co_opp,
					&nb_test_post, -1);
			constraint_put(co_opp);
		}
		break;
	case PRCM_DSPFREQ_CONSTRAINT:
		if (action == REQUEST_CO) {
			co_opp = constraint_get("test", &cnstr_id_dsp);
			constraint_register_pre_notification(co_opp,
					&nb_test_pre, -1);
			constraint_register_post_notification(co_opp,
					&nb_test_post, -1);
			constraint_set(co_opp, target_value);
		} else if (action == RELEASE_CO) {
			constraint_remove(co_opp);
			constraint_unregister_pre_notification(co_opp,
					&nb_test_pre, -1);
			constraint_unregister_post_notification(co_opp,
					&nb_test_post, -1);
			constraint_put(co_opp);
		}
		break;
	default:
		break;
	}
	return;
}
EXPORT_SYMBOL(constraint_test);

int resource_test_1(const char *usr_name, char *name);
int resource_test_2(char *name);
int resource_test_3(char *name);
int resource_notification_test(const char *usr_name, char *name);
void turn_power_domains_on(u8 domainid);
int test_logical_resource(char *name);

struct device_driver cameraisp_drv = {
	.name =  "cameraisp",
};

struct device cameraisp_dev = {
	.driver = &cameraisp_drv,
};

#if defined(CONFIG_OMAP3430_ES2) && !defined(CONFIG_ARCH_OMAP3410)
struct device_driver sgx_drv = {
	.name =  "sgx",
};

struct device sgx_dev = {
	.driver = &sgx_drv,
};

struct device dev_1 = {
	.driver = &sgx_drv,
};

struct device dev_2 = {
	.driver = &sgx_drv,
};

struct device dev_3 = {
	.driver = &sgx_drv,
};

struct device dev_4 = {
	.driver = &sgx_drv,
};
#endif
struct device logical_res;

int resource_test(void)
{
	u32 camf, cami;
	int ret;
#if defined(CONFIG_OMAP3430_ES2) && !defined(CONFIG_ARCH_OMAP3410)
	u32 sgxf, sgxi;
	/* Save clock registers */
	sgxf = CM_FCLKEN_SGX;
	sgxi = CM_ICLKEN_SGX;
	CM_FCLKEN_SGX   = 0x0;
	CM_ICLKEN_SGX   = 0x0;
#endif
	camf = CM_FCLKEN_CAM;
	cami = CM_ICLKEN_CAM;
	/* Disable the clocks */
	CM_FCLKEN_CAM   = 0x0;
	CM_ICLKEN_CAM   = 0x0;

#if defined(CONFIG_OMAP3430_ES2) && !defined(CONFIG_ARCH_OMAP3410)
	ret = resource_test_1("graphics", "sgx");
	if (ret != 0)
		goto test_fail;
	ret = resource_test_2("sgx");
	if (ret != 0)
		goto test_fail;
	ret = resource_test_3("sgx");
	if (ret != 0)
		goto test_fail;
	ret = resource_notification_test("graphics", "sgx");
	if (ret != 0)
		goto test_fail;
#endif
	ret = resource_test_1("camera", "cam");
	if (ret != 0)
		goto test_fail;
	ret = resource_test_2("cam");
	if (ret != 0)
		goto test_fail;
	ret = resource_test_3("cam");
	if (ret != 0)
		goto test_fail;
	ret = resource_notification_test("camera", "cam");
	if (ret != 0)
		goto test_fail;

	ret = test_logical_resource("core");

test_fail:
	turn_power_domains_on(DOM_CAM);
#if defined(CONFIG_OMAP3430_ES2) && !defined(CONFIG_ARCH_OMAP3410)
	turn_power_domains_on(DOM_SGX);
	/* Restore clock registers */
	CM_FCLKEN_SGX = sgxf;
	CM_ICLKEN_SGX = sgxi;
#endif
	CM_FCLKEN_CAM = camf;
	CM_ICLKEN_CAM = cami;

	if (ret != 0)
		printk(KERN_ERR"Test Failed\n");
	else
		printk(KERN_INFO"Test Passed\n");

	return 0;
}
EXPORT_SYMBOL(resource_test);

int resource_test_1(const char *usr_name, char *name)
{
	struct res_handle *res;
	int ret = 0;
	u8 result, level;

	printk(KERN_INFO"Entry %s: %s\n", __FUNCTION__, name);

	res = resource_get(usr_name, name);

	ret = resource_request(res, POWER_DOMAIN_ON);
	if (!ret) {
		prcm_get_power_domain_state(res->res->prcm_id, &result);
		if ((result) != 0x3)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in ON"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}
	ret = resource_request(res, POWER_DOMAIN_RET);
	if (!ret) {
		prcm_get_power_domain_state(res->res->prcm_id, &result);
		if ((result) != 0x1)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in RET"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}
	ret = resource_request(res, POWER_DOMAIN_ON);
	if (!ret) {
		prcm_get_power_domain_state(res->res->prcm_id, &result);
		if ((result) != 0x3)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in ON"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}

	ret = resource_release(res);
	if (!ret) {
		prcm_get_power_domain_state(res->res->prcm_id, &result);
		if ((result) != 0x0)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in OFF"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}

	ret = resource_request(res, POWER_DOMAIN_ON);
	if (!ret) {
		prcm_get_power_domain_state(res->res->prcm_id, &result);
		if ((result) != 0x3)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in ON"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}

	ret = resource_request(res, POWER_DOMAIN_RET);
	if (!ret) {
		prcm_get_power_domain_state(res->res->prcm_id, &result);
		if ((result) != 0x1)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in RET"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}
	ret = resource_release(res);
	if (ret == 0) {
		prcm_get_power_domain_state(res->res->prcm_id, &result);
		printk(KERN_ERR"FAILED!!!! Power domain %s went to OFF after "
				"RET\n", name);
	}
	printk(KERN_INFO"(result) : %x\n", (result));

	ret = resource_request(res, POWER_DOMAIN_ON);
	if (!ret) {
		prcm_get_power_domain_state(res->res->prcm_id, &result);
		if ((result) != 0x3)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in ON"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}

	level = resource_get_level(res);
	if (level != POWER_DOMAIN_ON)
		printk(KERN_ERR"resource_get_level failed for domain %s"
				"\n", name);

	resource_release(res);
	resource_put(res);

	printk(KERN_INFO"Exit %s: %s\n", __FUNCTION__, name);

	return 0;
}

int resource_test_2(char *name)
{
	struct res_handle *res_1, *res_2, *res_3, *res_4;
	int ret = 0;
	u8 result, level;

	printk(KERN_INFO"Entry %s: %s\n", __FUNCTION__, name);

	res_1 = resource_get("Name_1", name);

	ret = resource_request(res_1, POWER_DOMAIN_ON);
	if (!ret) {
		prcm_get_power_domain_state(res_1->res->prcm_id, &result);
		if ((result) != 0x3)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in ON"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}

	res_2 = resource_get("Name_2", name);

	ret = resource_request(res_2, POWER_DOMAIN_ON);
	if (!ret) {
		prcm_get_power_domain_state(res_2->res->prcm_id, &result);
		if ((result) != 0x3)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in ON"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}

	res_3 = resource_get("Name_3", name);

	ret = resource_request(res_3, POWER_DOMAIN_ON);
	if (!ret) {
		prcm_get_power_domain_state(res_3->res->prcm_id, &result);
		if ((result) != 0x3)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in ON"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}

	res_4 = resource_get("Name_4", name);

	ret = resource_request(res_4, POWER_DOMAIN_ON);
	if (!ret) {
		prcm_get_power_domain_state(res_4->res->prcm_id, &result);
		if ((result) != 0x3)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in ON"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}

	ret = resource_release(res_4);
	if (!ret) {
		prcm_get_power_domain_state(res_4->res->prcm_id, &result);
		if ((result) != 0x3)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in ON"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}

	resource_put(res_4);

	ret = resource_release(res_3);
	if (!ret) {
		prcm_get_power_domain_state(res_3->res->prcm_id, &result);
		if ((result) != 0x3)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in ON"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}

	resource_put(res_3);

	ret = resource_release(res_2);
	if (!ret) {
		prcm_get_power_domain_state(res_2->res->prcm_id, &result);
		if ((result) != 0x3)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in ON"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}

	resource_put(res_2);

	ret = resource_release(res_1);
	if (!ret) {
		prcm_get_power_domain_state(res_1->res->prcm_id, &result);
		if ((result) != 0x0)
			printk(KERN_ERR"Last one to release: FAILED!!!! Power "
					"domain %s not in OFF\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}

	level = resource_get_level(res_1);
	if (level != POWER_DOMAIN_OFF)
		printk(KERN_ERR"resource_get_level failed for domain %s"
				"\n", name);

	resource_put(res_1);

	printk(KERN_INFO"Exit %s: %s\n", __FUNCTION__, name);

	return 0;
}

int resource_test_3(char *name)
{
	struct res_handle *res_1, *res_2, *res_3, *res_4;
	int ret = 0;
	u8 result, level;

	printk(KERN_INFO"Entry %s: %s\n", __FUNCTION__, name);

	/** One ON, three RET and then change ON to RET **/
	/** State should be RET **/

	res_1 = resource_get("Name_1", name);

	ret = resource_request(res_1, POWER_DOMAIN_ON);
	if (!ret) {
		prcm_get_power_domain_state(res_1->res->prcm_id, &result);
		if ((result) != 0x3)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in ON"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}

	res_2 = resource_get("Name_2", name);

	ret = resource_request(res_2, POWER_DOMAIN_ON);
	if (!ret) {
		prcm_get_power_domain_state(res_2->res->prcm_id, &result);
		if ((result) != 0x3)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in ON"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));

	}
	ret = resource_request(res_2, POWER_DOMAIN_RET);
	if (!ret) {
		prcm_get_power_domain_state(res_2->res->prcm_id, &result);
		if ((result) != 0x3)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in ON"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}

	res_3 = resource_get("Name_3", name);

	ret = resource_request(res_3, POWER_DOMAIN_ON);
	if (!ret) {
		prcm_get_power_domain_state(res_3->res->prcm_id, &result);
		if ((result) != 0x3)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in ON"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}

	ret = resource_request(res_3, POWER_DOMAIN_RET);
	if (!ret) {
		prcm_get_power_domain_state(res_3->res->prcm_id, &result);
		if ((result) != 0x3)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in ON"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}

	res_4 = resource_get("Name_4", name);

	ret = resource_request(res_4, POWER_DOMAIN_ON);
	if (!ret) {
		prcm_get_power_domain_state(res_4->res->prcm_id, &result);
		if ((result) != 0x3)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in ON"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}

	ret = resource_request(res_4, POWER_DOMAIN_RET);
	if (!ret) {
		prcm_get_power_domain_state(res_4->res->prcm_id, &result);
		if ((result) != 0x3)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in ON"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}

	ret = resource_request(res_1, POWER_DOMAIN_RET);
	if (!ret) {
		prcm_get_power_domain_state(res_1->res->prcm_id, &result);
		if ((result) != 0x1)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in RET"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}

	ret = resource_request(res_4, POWER_DOMAIN_ON);
	if (!ret) {
		prcm_get_power_domain_state(res_4->res->prcm_id, &result);
		if ((result) != 0x3)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in ON"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}

	resource_release(res_4);
	resource_put(res_4);

	ret = resource_request(res_3, POWER_DOMAIN_ON);
	if (!ret) {
		prcm_get_power_domain_state(res_3->res->prcm_id, &result);
		if ((result) != 0x3)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in ON"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}

	resource_release(res_3);
	resource_put(res_3);

	ret = resource_request(res_2, POWER_DOMAIN_ON);
	if (!ret) {
		prcm_get_power_domain_state(res_2->res->prcm_id, &result);
		if ((result) != 0x3)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in ON"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}

	resource_release(res_2);
	resource_put(res_2);

	ret = resource_request(res_1, POWER_DOMAIN_ON);
	if (!ret) {
		prcm_get_power_domain_state(res_1->res->prcm_id, &result);
		if ((result) != 0x3)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in ON"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}

	level = resource_get_level(res_1);
	if (level != POWER_DOMAIN_ON)
		printk(KERN_ERR"resource_get_level failed for domain %s"
				"\n", name);

	resource_release(res_1);
	resource_put(res_1);

	printk(KERN_INFO"Exit %s: %s\n", __FUNCTION__, name);

	return ret;
}

int resource_notification_test(const char *usr_name, char *name)
{
	struct res_handle *res;
	int ret = 0;
	u8 result;

	printk(KERN_INFO"Entry %s: %s\n", __FUNCTION__, name);

	res = resource_get(usr_name, name);

	resource_register_pre_notification(res, &nb_test_pre, POWER_DOMAIN_ON);
	resource_register_pre_notification(res, &nb_test_pre, POWER_DOMAIN_RET);

	ret = resource_request(res, POWER_DOMAIN_ON);
	if (!ret) {
		prcm_get_power_domain_state(res->res->prcm_id, &result);
		if ((result) != 0x3)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in ON"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}

	ret = resource_request(res, POWER_DOMAIN_RET);
	if (!ret) {
		prcm_get_power_domain_state(res->res->prcm_id, &result);
		if ((result) != 0x1)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in RET"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}

	ret = resource_request(res, POWER_DOMAIN_ON);
	if (!ret) {
		prcm_get_power_domain_state(res->res->prcm_id, &result);
		if ((result) != 0x3)
			printk(KERN_ERR"FAILED!!!! Power domain %s not in ON"
					"\n", name);
			printk(KERN_INFO"(result) : %x\n", (result));
	}

	resource_unregister_pre_notification(res, &nb_test_pre,
							POWER_DOMAIN_ON);
	resource_unregister_pre_notification(res, &nb_test_pre,
							POWER_DOMAIN_RET);

	resource_release(res);

	resource_put(res);

	printk(KERN_INFO"Exit %s: %s\n", __FUNCTION__, name);

	return 0;
}

void turn_power_domains_on(u8 domainid)
{
	prcm_force_power_domain_state(domainid, POWER_DOMAIN_ON);

	return;
}

int test_logical_resource(char *name)
{
	struct res_handle *res;
	int ret = 0;

	printk(KERN_INFO"Entry %s: %s\n", __FUNCTION__, name);

	res = resource_get("logical", name);

	ret = resource_request(res, LOGICAL_USED);
	if (ret != 0) {
		printk(KERN_ERR"%s: resource_request for domain %s failed"
				"\n", __FUNCTION__, name);
		return ret;
	}

	resource_release(res);

	resource_put(res);

	printk("Exit %s: %s\n", __FUNCTION__, name);

	return ret;
}

int check_core_memory_logical_resource(bool mem1_state, bool mem2state,
							bool logicstate) {
	int ret = 0;
	int res_state1, res_state2, res_state3;

	ret = resource_request(memret1, mem1_state);
	if (ret != 0) {
		printk(KERN_ERR"Resource_request for MEMORY1  failed \n");
		return ret;
	}
	ret = resource_request(memret2, mem2state);
	if (ret != 0) {
		printk(KERN_ERR"Resource_request for MEMORY2  failed \n");
		return ret;
	}
	ret = resource_request(logret1, logicstate);
	if (ret != 0) {
		printk(KERN_ERR"Resource_request for LOGIC  failed \n");
		return ret;
	}
	res_state1 = resource_get_level(memret1);
	res_state2 = resource_get_level(memret2);
	res_state3 = resource_get_level(logret1);
	if (res_state1 == 1)
		printk(KERN_ERR"::: MEMORY 1 resource of core domain is  "
								"ON :::\n");
	else
		printk(KERN_ERR"::: MEMORY 1 resource of core domain is  "
								"OFF :::\n");
	if (res_state2 == 1)
		printk(KERN_ERR"::: MEMORY 2 resource of core domain is  "
								"ON :::\n");
	else
		printk(KERN_ERR"::: MEMORY 2 resource of core domain  is "
								"OFF :::\n");
	if (res_state3 == 1)
		printk(KERN_ERR"::: LOGIC resource of core domain is ON  "
								"  :::\n");
	else
		printk(KERN_ERR"::: LOGIC resource of core domain is OFF  "
								"  :::\n");
	return 0;
}

int check_dpll_resource(bool dpll1, bool dpll2, unsigned int dpll3, bool dpll4,
								bool dpll5) {
	int ret = 0;
	int res_state1, res_state2, res_state3, res_state4;
	struct res_handle *mpudpll;
	struct res_handle *iva2dpll;
	struct res_handle *coredpll;
	struct res_handle *perdpll;
	#ifdef CONFIG_OMAP3430_ES2
	struct res_handle *per2dpll;
	int res_state5;
	#endif

	mpudpll = (struct res_handle *)resource_get("mpuautoidle",
						"mpu_autoidle_res");
	ret = resource_request(mpudpll, dpll1);
	if (ret != 0) {
		printk(KERN_ERR"Resource_request for DPLL1  failed \n");
		return ret;
	}

	iva2dpll = (struct res_handle *)resource_get("iva2autoidle",
						"iva2_autoidle_res");
	ret = resource_request(iva2dpll, dpll2);
	if (ret != 0) {
		printk(KERN_ERR"Resource_request for DPLL2  failed \n");
		return ret;
	}

	coredpll = (struct res_handle *)resource_get("coreautoidle",
						"core_autoidle_res");
	ret = resource_request(coredpll, dpll3);
	if (ret != 0) {
		printk(KERN_ERR"Resource_request for DPLL3  failed \n");
		return ret;
	}

	perdpll = (struct res_handle *)resource_get("perautoidle",
						"per_autoidle_res");
	ret = resource_request(perdpll, dpll4);
	if (ret != 0) {
		printk(KERN_ERR"Resource_request for DPLL4  failed \n");
		return ret;
	}
#ifdef CONFIG_OMAP3430_ES2
	per2dpll = (struct res_handle *)resource_get("per2autoidle",
						"per2_autoidle_res");

	ret = resource_request(per2dpll, dpll5);
	if (ret != 0) {
		printk(KERN_ERR"Resource_request for DPLL5  failed \n");
		return ret;
	}
#endif
	res_state1 = resource_get_level(mpudpll);
	res_state2 = resource_get_level(iva2dpll);
	res_state3 = resource_get_level(coredpll);
	res_state4 = resource_get_level(perdpll);
#ifdef CONFIG_OMAP3430_ES2
	res_state5 = resource_get_level(per2dpll);
#endif
	if (res_state1 == 1)
		printk(KERN_ERR"::: DPLL1 in AUTOIDLE state        "
		" CM_AUTOIDLE_PLL_MPU  =%x :::\n", CM_AUTOIDLE_PLL_MPU);
	else
		printk(KERN_ERR"::: DPLL1 in NOAUTOIDLE state      "
		" CM_AUTOIDLE_PLL_MPU  =%x :::\n", CM_AUTOIDLE_PLL_MPU);
	if (res_state2 == 1)
		printk(KERN_ERR"::: DPLL2 in AUTOIDLE state        "
		" CM_AUTOIDLE_PLL_IVA2 =%x :::\n", CM_AUTOIDLE_PLL_IVA2);
	else
		printk(KERN_ERR"::: DPLL2 in NOAUTOIDLE state      "
		" CM_AUTOIDLE_PLL_IVA2 =%x :::\n", CM_AUTOIDLE_PLL_IVA2);
	if (res_state3 == 1)
		printk(KERN_ERR"::: DPLL3 in AUTOIDLE state        "
		" CM_AUTOIDLE_PLL      =%x :::\n", CM_AUTOIDLE_PLL);
	else if (res_state3 == 5)
		printk(KERN_ERR"::: DPLL3 in AUTOIDLE_BYPASS  state"
		" CM_AUTOIDLE_PLL      =%x :::\n", CM_AUTOIDLE_PLL);
	else
		printk(KERN_ERR"::: DPLL3 in NOAUTOIDLE state      "
		" CM_AUTOIDLE_PLL      =%x :::\n", CM_AUTOIDLE_PLL);
	if (res_state4 == 1)
		printk(KERN_ERR"::: DPLL4 in AUTOIDLE state        "
		" CM_AUTOIDLE_PLL      =%x :::\n", CM_AUTOIDLE_PLL);
	else
		printk(KERN_ERR"::: DPLL4 in NOAUTOIDLE state      "
		" CM_AUTOIDLE_PLL      =%x :::\n", CM_AUTOIDLE_PLL);
#ifdef CONFIG_OMAP3430_ES2
	if (res_state5 == 1)
		printk(KERN_ERR"::: DPLL5 in AUTOIDLE state        "
		" CM_AUTOIDLE2_PLL     =%x :::\n", CM_AUTOIDLE2_PLL);
	else
		printk(KERN_ERR"::: DPLL5  in NOAUTOIDLE state     "
		" CM_AUTOIDLE2_PLL     =%x :::\n", CM_AUTOIDLE2_PLL);
#endif
	resource_release(mpudpll);
	resource_put(mpudpll);
	resource_release(iva2dpll);
	resource_put(iva2dpll);
	resource_release(coredpll);
	resource_put(coredpll);
	resource_release(perdpll);
	resource_put(perdpll);
#ifdef CONFIG_OMAP3430_ES2
	resource_release(per2dpll);
	resource_put(per2dpll);
#endif

	return 0;
}


int memory_resource_test(int opt2)
{
	int ret = 0;
	bool all = 0;
	switch (opt2) {
	case 0x3F:
		printk(KERN_INFO"ALL TEST\n");
		all = 1;
	case 0x1:
		printk(KERN_INFO "TEST 1\n");
		ret = check_core_memory_logical_resource(MEMORY_RET,
						MEMORY_RET, LOGIC_RET);
		if (all == 0 || ret != 0)
		break;
	case 0x2:
		printk(KERN_INFO "TEST 2\n");
		ret = check_core_memory_logical_resource(MEMORY_OFF,
						MEMORY_RET, LOGIC_RET);
		if (all == 0 || ret != 0)
		break;
	case 0x3:
		printk(KERN_INFO "TEST 3\n");
		ret = check_core_memory_logical_resource(MEMORY_RET,
						MEMORY_OFF, LOGIC_RET);
		if (all == 0 || ret != 0)
		break;
	case 0x4:
		printk(KERN_INFO "TEST 4\n");
		ret = check_core_memory_logical_resource(MEMORY_OFF,
						MEMORY_OFF, LOGIC_RET);
		if (all == 0 || ret != 0)
		break;
	case 0x5:
		printk(KERN_INFO "TEST 5\n");
		ret = check_core_memory_logical_resource(MEMORY_RET,
						MEMORY_RET, LOGIC_OFF);
		if (all == 0 || ret != 0)
		break;
	case 0x6:
		printk(KERN_INFO "TEST 6\n");
		ret = check_core_memory_logical_resource(MEMORY_OFF,
						MEMORY_RET, LOGIC_OFF);
		if (all == 0 || ret != 0)
		break;
	case 0x7:
		printk(KERN_INFO "TEST 7\n");
		ret = check_core_memory_logical_resource(MEMORY_RET,
						MEMORY_OFF, LOGIC_OFF);
		if (all == 0 || ret != 0)
		break;
	case 0x8:
		printk(KERN_INFO "TEST 8\n");
		ret = check_core_memory_logical_resource(MEMORY_OFF,
						MEMORY_OFF, LOGIC_OFF);
		break;
	default:
		printk(KERN_INFO"**Unsupported memory resource state**\n"
			"** Select input parameter  opt2 between 1-8 **\n");
		return -1;
	}

	if (ret == 0) {
		printk(KERN_INFO"::: Memory resource test : PASS :::\n");
		return 0;
	}
	  else {
		printk(KERN_INFO"::: Memory resource test : FAIL :::\n");
		return -1;
	}


}
EXPORT_SYMBOL(memory_resource_test);

int dpll_resource_test(int opt2)
{
	int ret = 0;
	bool all = 0;
	switch (opt2) {
	case 0x3F:
		printk(KERN_INFO"ALL TEST\n");
		all = 1;
	case 0x1:
		printk(KERN_INFO"TEST 1\n");
		ret = check_dpll_resource(DPLL_NO_AUTOIDLE, DPLL_NO_AUTOIDLE,
			DPLL_NO_AUTOIDLE, DPLL_NO_AUTOIDLE, DPLL_NO_AUTOIDLE);
		if (all == 0 || ret != 0)
		break;
	case 0x2:
		printk(KERN_INFO"TEST 2\n");
		ret = check_dpll_resource(DPLL_AUTOIDLE, DPLL_NO_AUTOIDLE,
			DPLL_NO_AUTOIDLE, DPLL_NO_AUTOIDLE, DPLL_NO_AUTOIDLE);
		if (all == 0 || ret != 0)
		break;
	case 0x3:
		printk(KERN_INFO"TEST 3\n");
		ret = check_dpll_resource(DPLL_NO_AUTOIDLE, DPLL_AUTOIDLE,
			DPLL_NO_AUTOIDLE, DPLL_NO_AUTOIDLE, DPLL_NO_AUTOIDLE);
		if (all == 0 || ret != 0)
		break;
	case 0x4:
		printk(KERN_INFO"TEST 4\n");
		ret = check_dpll_resource(DPLL_NO_AUTOIDLE, DPLL_NO_AUTOIDLE,
			DPLL_NO_AUTOIDLE, DPLL_NO_AUTOIDLE, DPLL_AUTOIDLE);
		if (all == 0 || ret != 0)
		break;
	case 0x5:
		printk(KERN_INFO"TEST 5\n");
		ret = check_dpll_resource(DPLL_NO_AUTOIDLE, DPLL_NO_AUTOIDLE,
			DPLL_AUTOIDLE_BYPASS, DPLL_NO_AUTOIDLE,
						DPLL_NO_AUTOIDLE);
		if (all == 0 || ret != 0)
		break;
	case 0x6:
		printk(KERN_INFO"TEST 6\n");
		ret = check_dpll_resource(DPLL_NO_AUTOIDLE, DPLL_NO_AUTOIDLE,
			DPLL_AUTOIDLE_BYPASS, DPLL_AUTOIDLE,
					DPLL_NO_AUTOIDLE);
		if (all == 0 || ret != 0)
		break;
	case 0x7:
		printk(KERN_INFO"TEST 7\n");
		ret = check_dpll_resource(DPLL_NO_AUTOIDLE, DPLL_NO_AUTOIDLE,
			DPLL_AUTOIDLE, DPLL_AUTOIDLE,
					DPLL_NO_AUTOIDLE);
		if (all == 0 || ret != 0)
		break;
	case 0x8:
		printk(KERN_INFO"TEST 8\n");
		ret = check_dpll_resource(DPLL_NO_AUTOIDLE, DPLL_NO_AUTOIDLE,
			DPLL_AUTOIDLE, DPLL_NO_AUTOIDLE,
					DPLL_NO_AUTOIDLE);
		if (all == 0 || ret != 0)
		break;
	case 0x9:
		printk(KERN_INFO"TEST 9\n");
		ret = check_dpll_resource(DPLL_NO_AUTOIDLE, DPLL_NO_AUTOIDLE,
			DPLL_NO_AUTOIDLE, DPLL_AUTOIDLE, DPLL_NO_AUTOIDLE);
		if (all == 0 || ret != 0)
		break;
	case 0xa:
		printk(KERN_INFO"TEST 10\n");
		ret = check_dpll_resource(DPLL_NO_AUTOIDLE, DPLL_NO_AUTOIDLE,
			DPLL_NO_AUTOIDLE, DPLL_NO_AUTOIDLE,
							DPLL_NO_AUTOIDLE);
		break;
	default:
		printk(KERN_INFO"** Unsupported DPLL resource state **\n"
			"** Select input parameter  opt2 between 1-7 **\n");
		return -1;
	}

	if (ret == 0) {
		printk(KERN_INFO"::: Dpll resource test : PASS :::\n");
		return 0;
	}
	  else {
		printk(KERN_INFO"::: Dpll resource test : FAIL :::\n");
		return -1;
	}

}
EXPORT_SYMBOL(dpll_resource_test);
#ifdef RESOURCE_TEST
__initcall(resource_test);
#endif

