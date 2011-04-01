/*
 * linux/arch/arm/mach-omap3/smartreflex.c
 *
 * OMAP34XX SmartReflex Voltage Control
 *  
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Lesly A M <x0080970@ti.com>
 *
 * Added Class2 support 
 * Pratheesh Gangadhar <pratheesh@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#ifdef CONFIG_OMAP_VOLT_VSEL
#include <linux/workqueue.h>
#endif
#include <asm/arch/prcm.h>
#include <asm/arch/power_companion.h>
#include <asm/io.h>
#include <asm/mach-types.h>

#include "prcm-regs.h"
#include "smartreflex.h"

/* #define DEBUG_SR 1 */
#ifdef DEBUG_SR
#  define DPRINTK(fmt, args...) printk(KERN_ERR "%s: " fmt, __FUNCTION__ ,\
									## args)
#else
#  define DPRINTK(fmt, args...)
#endif

struct omap_sr {
	int srid;
	int is_sr_reset;
	int is_autocomp_active;
	struct clk *fck;
	u32 req_opp_no;
	u32 opp1_nvalue, opp2_nvalue, opp3_nvalue, opp4_nvalue, opp5_nvalue;
	u32 senp_mod, senn_mod;
	u32 srbase_addr;
	u32 vpbase_addr;
};

static struct omap_sr sr1 = {
	.srid = SR1,
	.is_sr_reset = 1,
	.is_autocomp_active = 0,
	.srbase_addr = OMAP34XX_SR1_BASE,
};

static struct omap_sr sr2 = {
	.srid = SR2,
	.is_sr_reset = 1,
	.is_autocomp_active = 0,
	.srbase_addr = OMAP34XX_SR2_BASE,
};

static inline void sr_write_reg(struct omap_sr *sr, int offset, u32 value)
{
	omap_writel(value, sr->srbase_addr + offset);
}

static inline void sr_modify_reg(struct omap_sr *sr, int offset, u32 mask,
				 u32 value)
{
	u32 reg_val;

	reg_val = omap_readl(sr->srbase_addr + offset);
	reg_val &= ~mask;
	reg_val |= value;

	omap_writel(reg_val, sr->srbase_addr + offset);
}

static inline u32 sr_read_reg(struct omap_sr *sr, int offset)
{
	return omap_readl(sr->srbase_addr + offset);
}

#ifdef  CONFIG_ENABLE_SR_CLASS2
#define OFFSET 0
#define GAIN   32
#ifndef CONFIG_TPS65023
#define V_PER_STEP 102
#define MIN_VSEL   20 
#define MAX_VSEL   64
#else
#define V_PER_STEP 205
#define MIN_VSEL   0
#define MAX_VSEL   24
#endif
u8 current_voltage, last_vdd1_opp;
#ifdef CONFIG_OMAP_VOLT_VSEL
void sr_start_vddautocomap(int srid, u32 target_opp_no);
int sr_stop_vddautocomap(int srid);
struct wq_setvoltage {
	u8 vsel;
	struct work_struct work;
};

struct wq_setvoltage wq_sr;

static void set_voltage(struct work_struct *work)
{
	struct wq_setvoltage *ptrwq_sr =
	    container_of(work, struct wq_setvoltage, work);
	if (ptrwq_sr->vsel) {
		int sr_status = 0;
		u8 vdd = get_vdd(current_vdd1_opp);
		u32 target_opp_no = get_opp_no(current_vdd1_opp);

		sr_status = sr_stop_vddautocomap(SR1);

		set_voltage_level(vdd, ptrwq_sr->vsel);
		if (sr_status)
			sr_start_vddautocomap(SR1, target_opp_no);
		enable_irq(INT_34XX_SR1_IRQ);
	}
}

#define VSEL wq_sr.vsel
#elif CONFIG_OMAP_VOLT_SR_BYPASS
#define VSEL vsel
#endif
static irqreturn_t omap_sr1_class2_irq(int irq, void *dev_id)
{
#ifdef CONFIG_OMAP_VOLT_SR_BYPASS
	u8 vsel;
#endif
	s8 tmp_val;
	u32 senerror_reg, tmp, oppno;
	s32 error, steps, delta;
	disable_irq(INT_34XX_SR1_IRQ);
	/* Disable the interrupt */
	sr_modify_reg(&sr1, ERRCONFIG,
		      (ERRCONFIG_MCUBOUNDINTEN | ERRCONFIG_MCUBOUNDINTST),
		      (~ERRCONFIG_MCUBOUNDINTEN | ERRCONFIG_MCUBOUNDINTST));

	tmp_val = 0;
	senerror_reg = sr_read_reg(&sr1, SENERROR);
	tmp = (senerror_reg & 0x0000FF00);
	tmp = tmp >> 8;
	tmp_val = (tmp & 0x000000FF);
	error = (tmp_val * 25) << 8;
	delta = ((error + OFFSET) * GAIN) >> 13;
	steps = delta / V_PER_STEP;
	oppno = get_opp_no(current_vdd1_opp);
	if (last_vdd1_opp != oppno) {
		current_voltage = mpu_iva2_vdd1_volts[oppno - 1];
		last_vdd1_opp = oppno;
	}
	VSEL = 0;
	DPRINTK(KERN_DEBUG "delta steps = %d oppno = %d\n", steps, oppno);
	if (delta < 0) {
		delta = -delta;
		if (delta >= V_PER_STEP) {
			current_voltage = VSEL = current_voltage + steps;
			if ((oppno > 1) && (current_voltage <  mpu_iva2_vdd1_volts[oppno - 2]) ) {
				current_voltage = mpu_iva2_vdd1_volts[oppno - 2]+1;
			}			
		}
	} else {
		if (delta > 0) {
			if (delta >= V_PER_STEP) {
				current_voltage = VSEL =
				    current_voltage + steps;
			}
		}
	}
	if (VSEL) {
		if (VSEL <= MIN_VSEL)
			VSEL = MIN_VSEL;
		else {
			if (VSEL > MAX_VSEL)
				VSEL = MAX_VSEL;
	}
#ifdef CONFIG_OMAP_VOLT_SR_BYPASS

		sr_voltagescale_vcbypass(current_vdd1_opp, VSEL);
		}
	enable_irq(INT_34XX_SR1_IRQ);
#elif CONFIG_OMAP_VOLT_VSEL

		schedule_work(&wq_sr.work);

	}
#endif
	return IRQ_HANDLED;
}
#endif

#ifndef USE_EFUSE_VALUES
static void cal_reciprocal(u32 sensor, u32 * sengain, u32 * rnsen)
{
	u32 gn, rn, mul;

	for (gn = 0; gn < GAIN_MAXLIMIT; gn++) {
		mul = 1 << (gn + 8);
		rn = mul / sensor;
		if (rn < R_MAXLIMIT) {
			*sengain = gn;
			*rnsen = rn;
		}
	}
}
#endif

static int sr_clk_enable(struct omap_sr *sr)
{
	if (clk_enable(sr->fck) != 0) {
		printk(KERN_ERR "Could not enable sr%d_fck\n", sr->srid);
		goto clk_enable_err;
	}

	/* set fclk- active , iclk- idle */
	sr_modify_reg(sr, ERRCONFIG, SR_CLKACTIVITY_MASK,
		      SR_CLKACTIVITY_IOFF_FON);

	return 0;

      clk_enable_err:
	return -1;
}

static int sr_clk_disable(struct omap_sr *sr)
{
	/* set fclk, iclk- idle */
	sr_modify_reg(sr, ERRCONFIG, SR_CLKACTIVITY_MASK,
		      SR_CLKACTIVITY_IOFF_FOFF);

	clk_disable(sr->fck);
	sr->is_sr_reset = 1;

	return 0;
}

static void sr_set_nvalues(struct omap_sr *sr)
{
#ifdef USE_EFUSE_VALUES
	u32 n1, n2;
#else
	u32 senpval, sennval;
	u32 senpgain, senngain;
	u32 rnsenp, rnsenn;
#endif

	if (sr->srid == SR1) {
#ifdef USE_EFUSE_VALUES
		/* Read values for VDD1 from EFUSE */
#else
		/* since E-Fuse Values are not available, calculating the
		 * reciprocal of the SenN and SenP values for SR1
		 */
		sr->senp_mod = 0x03;	/* SenN-M5 enabled */
		sr->senn_mod = 0x03;

		/* for OPP5 */
		senpval = 0x847;
		sennval = 0xace;

		cal_reciprocal(senpval, &senpgain, &rnsenp);
		cal_reciprocal(sennval, &senngain, &rnsenn);

		sr->opp5_nvalue =
		    ((senpgain << NVALUERECIPROCAL_SENPGAIN_SHIFT) |
		     (senngain << NVALUERECIPROCAL_SENNGAIN_SHIFT) |
		     (rnsenp << NVALUERECIPROCAL_RNSENP_SHIFT) |
		     (rnsenn << NVALUERECIPROCAL_RNSENN_SHIFT));

		/* for OPP4 */
		senpval = 0x724;
		sennval = 0x963;

		cal_reciprocal(senpval, &senpgain, &rnsenp);
		cal_reciprocal(sennval, &senngain, &rnsenn);

		sr->opp4_nvalue =
		    ((senpgain << NVALUERECIPROCAL_SENPGAIN_SHIFT) |
		     (senngain << NVALUERECIPROCAL_SENNGAIN_SHIFT) |
		     (rnsenp << NVALUERECIPROCAL_RNSENP_SHIFT) |
		     (rnsenn << NVALUERECIPROCAL_RNSENN_SHIFT));

		/* for OPP3 */
		senpval = 0x651;
		sennval = 0x859;

		cal_reciprocal(senpval, &senpgain, &rnsenp);
		cal_reciprocal(sennval, &senngain, &rnsenn);

		sr->opp3_nvalue =
		    ((senpgain << NVALUERECIPROCAL_SENPGAIN_SHIFT) |
		     (senngain << NVALUERECIPROCAL_SENNGAIN_SHIFT) |
		     (rnsenp << NVALUERECIPROCAL_RNSENP_SHIFT) |
		     (rnsenn << NVALUERECIPROCAL_RNSENN_SHIFT));

		/* for OPP2 */
		senpval = 0x3b8;
		sennval = 0x501;

		cal_reciprocal(senpval, &senpgain, &rnsenp);
		cal_reciprocal(sennval, &senngain, &rnsenn);

		sr->opp2_nvalue =
		    ((senpgain << NVALUERECIPROCAL_SENPGAIN_SHIFT) |
		     (senngain << NVALUERECIPROCAL_SENNGAIN_SHIFT) |
		     (rnsenp << NVALUERECIPROCAL_RNSENP_SHIFT) |
		     (rnsenn << NVALUERECIPROCAL_RNSENN_SHIFT));

		/* for OPP1 */
		senpval = 0x285;
		sennval = 0x36d;

		cal_reciprocal(senpval, &senpgain, &rnsenp);
		cal_reciprocal(sennval, &senngain, &rnsenn);

		sr->opp1_nvalue =
		    ((senpgain << NVALUERECIPROCAL_SENPGAIN_SHIFT) |
		     (senngain << NVALUERECIPROCAL_SENNGAIN_SHIFT) |
		     (rnsenp << NVALUERECIPROCAL_RNSENP_SHIFT) |
		     (rnsenn << NVALUERECIPROCAL_RNSENN_SHIFT));

		sr_clk_enable(sr);
		sr_write_reg(sr, NVALUERECIPROCAL, sr->opp3_nvalue);
		sr_clk_disable(sr);

#endif
	} else if (sr->srid == SR2) {
#ifdef USE_EFUSE_VALUES
		/* Read values for VDD2 from EFUSE */
#else
		/* since E-Fuse Values are not available, calculating the
		 * reciprocal of the SenN and SenP values for SR2
		 */
		sr->senp_mod = 0x03;
		sr->senn_mod = 0x03;

		/* for OPP3 */
		senpval = 0x570;
		sennval = 0x768;

		cal_reciprocal(senpval, &senpgain, &rnsenp);
		cal_reciprocal(sennval, &senngain, &rnsenn);

		sr->opp3_nvalue =
		    ((senpgain << NVALUERECIPROCAL_SENPGAIN_SHIFT) |
		     (senngain << NVALUERECIPROCAL_SENNGAIN_SHIFT) |
		     (rnsenp << NVALUERECIPROCAL_RNSENP_SHIFT) |
		     (rnsenn << NVALUERECIPROCAL_RNSENN_SHIFT));

		/* for OPP2 */
		senpval = 0x388;
		sennval = 0x4eb;

		cal_reciprocal(senpval, &senpgain, &rnsenp);
		cal_reciprocal(sennval, &senngain, &rnsenn);

		sr->opp2_nvalue =
		    ((senpgain << NVALUERECIPROCAL_SENPGAIN_SHIFT) |
		     (senngain << NVALUERECIPROCAL_SENNGAIN_SHIFT) |
		     (rnsenp << NVALUERECIPROCAL_RNSENP_SHIFT) |
		     (rnsenn << NVALUERECIPROCAL_RNSENN_SHIFT));

		/* for OPP1 */
		senpval = 0x257;
		sennval = 0x353;

		cal_reciprocal(senpval, &senpgain, &rnsenp);
		cal_reciprocal(sennval, &senngain, &rnsenn);

		sr->opp1_nvalue =
		    ((senpgain << NVALUERECIPROCAL_SENPGAIN_SHIFT) |
		     (senngain << NVALUERECIPROCAL_SENNGAIN_SHIFT) |
		     (rnsenp << NVALUERECIPROCAL_RNSENP_SHIFT) |
		     (rnsenn << NVALUERECIPROCAL_RNSENN_SHIFT));

#endif
	}

}

static void sr_configure_vp(int srid)
{
	u32 vpconfig;

	if (srid == SR1) {
		vpconfig = PRM_VP1_CONFIG_ERROROFFSET | PRM_VP1_CONFIG_ERRORGAIN
		    | PRM_VP1_CONFIG_INITVOLTAGE | PRM_VP1_CONFIG_TIMEOUTEN;

		PRM_VP1_CONFIG = vpconfig;
		PRM_VP1_VSTEPMIN = PRM_VP1_VSTEPMIN_SMPSWAITTIMEMIN |
		    PRM_VP1_VSTEPMIN_VSTEPMIN;

		PRM_VP1_VSTEPMAX = PRM_VP1_VSTEPMAX_SMPSWAITTIMEMAX |
		    PRM_VP1_VSTEPMAX_VSTEPMAX;

		PRM_VP1_VLIMITTO = PRM_VP1_VLIMITTO_VDDMAX |
		    PRM_VP1_VLIMITTO_VDDMIN | PRM_VP1_VLIMITTO_TIMEOUT;

		PRM_VP1_CONFIG |= PRM_VP1_CONFIG_INITVDD;
		PRM_VP1_CONFIG &= ~PRM_VP1_CONFIG_INITVDD;

	} else if (srid == SR2) {
		vpconfig = PRM_VP2_CONFIG_ERROROFFSET | PRM_VP2_CONFIG_ERRORGAIN
		    | PRM_VP2_CONFIG_INITVOLTAGE | PRM_VP2_CONFIG_TIMEOUTEN;

		PRM_VP2_CONFIG = vpconfig;
		PRM_VP2_VSTEPMIN = PRM_VP2_VSTEPMIN_SMPSWAITTIMEMIN |
		    PRM_VP2_VSTEPMIN_VSTEPMIN;

		PRM_VP2_VSTEPMAX = PRM_VP2_VSTEPMAX_SMPSWAITTIMEMAX |
		    PRM_VP2_VSTEPMAX_VSTEPMAX;

		PRM_VP2_VLIMITTO = PRM_VP2_VLIMITTO_VDDMAX |
		    PRM_VP2_VLIMITTO_VDDMIN | PRM_VP2_VLIMITTO_TIMEOUT;

		PRM_VP2_CONFIG |= PRM_VP2_CONFIG_INITVDD;
		PRM_VP2_CONFIG &= ~PRM_VP2_CONFIG_INITVDD;

	}
}

static void sr_configure_vc(void)
{
	PRM_VC_SMPS_SA =
	    (R_SRI2C_SLAVE_ADDR << PRM_VC_SMPS_SA1_SHIFT) |
	    (R_SRI2C_SLAVE_ADDR << PRM_VC_SMPS_SA0_SHIFT);

	PRM_VC_SMPS_VOL_RA = (R_VDD2_SR_CONTROL << PRM_VC_SMPS_VOLRA1_SHIFT) |
	    (R_VDD1_SR_CONTROL << PRM_VC_SMPS_VOLRA0_SHIFT);

	PRM_VC_CMD_VAL_0 = (PRM_VC_CMD_VAL0_ON << PRM_VC_CMD_ON_SHIFT) |
	    (PRM_VC_CMD_VAL0_ONLP << PRM_VC_CMD_ONLP_SHIFT) |
	    (PRM_VC_CMD_VAL0_RET << PRM_VC_CMD_RET_SHIFT) |
	    (PRM_VC_CMD_VAL0_OFF << PRM_VC_CMD_OFF_SHIFT);

	PRM_VC_CMD_VAL_1 = (PRM_VC_CMD_VAL1_ON << PRM_VC_CMD_ON_SHIFT) |
	    (PRM_VC_CMD_VAL1_ONLP << PRM_VC_CMD_ONLP_SHIFT) |
	    (PRM_VC_CMD_VAL1_RET << PRM_VC_CMD_RET_SHIFT) |
	    (PRM_VC_CMD_VAL1_OFF << PRM_VC_CMD_OFF_SHIFT);

	PRM_VC_CH_CONF = PRM_VC_CH_CONF_CMD1 | PRM_VC_CH_CONF_RAV1;

	PRM_VC_I2C_CFG = PRM_VC_I2C_CFG_MCODE | PRM_VC_I2C_CFG_HSEN
	    | PRM_VC_I2C_CFG_SREN;
}

static void sr_configure(struct omap_sr *sr)
{
	u32 sys_clk, sr_clk_length = 0;
	u32 sr_config;
	u32 senp_en, senn_en;

	senp_en = sr->senp_mod;
	senn_en = sr->senn_mod;

	sys_clk = prcm_get_system_clock_speed();

	switch (sys_clk) {
	case 12000:
		sr_clk_length = SRCLKLENGTH_12MHZ_SYSCLK;
		break;
	case 13000:
		sr_clk_length = SRCLKLENGTH_13MHZ_SYSCLK;
		break;
	case 19200:
		sr_clk_length = SRCLKLENGTH_19MHZ_SYSCLK;
		break;
	case 26000:
		sr_clk_length = SRCLKLENGTH_26MHZ_SYSCLK;
		break;
	case 38400:
		sr_clk_length = SRCLKLENGTH_38MHZ_SYSCLK;
		break;
	default:
		printk(KERN_ERR "Invalid sysclk value\n");
		break;
	}

	DPRINTK("SR : sys clk %lu\n", sys_clk);
	if (sr->srid == SR1) {
		sr_config = SR1_SRCONFIG_ACCUMDATA |
		    (sr_clk_length << SRCONFIG_SRCLKLENGTH_SHIFT) |
		    SRCONFIG_SENENABLE | SRCONFIG_ERRGEN_EN |
		    SRCONFIG_MINMAXAVG_EN |
		    (senn_en << SRCONFIG_SENNENABLE_SHIFT) |
		    (senp_en << SRCONFIG_SENPENABLE_SHIFT) | SRCONFIG_DELAYCTRL;

		sr_write_reg(sr, SRCONFIG, sr_config);

		sr_write_reg(sr, AVGWEIGHT, SR1_AVGWEIGHT_SENPAVGWEIGHT |
			     SR1_AVGWEIGHT_SENNAVGWEIGHT);

		sr_modify_reg(sr, ERRCONFIG, (SR_ERRWEIGHT_MASK |
					      SR_ERRMAXLIMIT_MASK |
					      SR_ERRMINLIMIT_MASK),
			      (SR1_ERRWEIGHT | SR1_ERRMAXLIMIT |
			       SR1_ERRMINLIMIT));

	} else if (sr->srid == SR2) {
		sr_config = SR2_SRCONFIG_ACCUMDATA |
		    (sr_clk_length << SRCONFIG_SRCLKLENGTH_SHIFT) |
		    SRCONFIG_SENENABLE | SRCONFIG_ERRGEN_EN |
		    SRCONFIG_MINMAXAVG_EN |
		    (senn_en << SRCONFIG_SENNENABLE_SHIFT) |
		    (senp_en << SRCONFIG_SENPENABLE_SHIFT) | SRCONFIG_DELAYCTRL;

		sr_write_reg(sr, SRCONFIG, sr_config);

		sr_write_reg(sr, AVGWEIGHT, SR2_AVGWEIGHT_SENPAVGWEIGHT |
			     SR2_AVGWEIGHT_SENNAVGWEIGHT);

		sr_modify_reg(sr, ERRCONFIG, (SR_ERRWEIGHT_MASK |
					      SR_ERRMAXLIMIT_MASK |
					      SR_ERRMINLIMIT_MASK),
			      (SR2_ERRWEIGHT | SR2_ERRMAXLIMIT |
			       SR2_ERRMINLIMIT));

	}
	sr->is_sr_reset = 0;
}

static void sr_enable(struct omap_sr *sr, u32 target_opp_no)
{
	u32 nvalue_reciprocal, current_nvalue;
	sr->req_opp_no = target_opp_no;

	if (sr->srid == SR1) {
		switch (target_opp_no) {
		case 5:
			nvalue_reciprocal = sr->opp5_nvalue;
			break;
		case 4:
			nvalue_reciprocal = sr->opp4_nvalue;
			break;
		case 3:
			nvalue_reciprocal = sr->opp3_nvalue;
			break;
		case 2:
			nvalue_reciprocal = sr->opp2_nvalue;
			break;
		case 1:
			nvalue_reciprocal = sr->opp1_nvalue;
			break;
		default:
			nvalue_reciprocal = sr->opp3_nvalue;
			break;
		}
	} else {
		switch (target_opp_no) {
		case 3:
			nvalue_reciprocal = sr->opp3_nvalue;
			break;
		case 2:
			nvalue_reciprocal = sr->opp2_nvalue;
			break;
		case 1:
			nvalue_reciprocal = sr->opp1_nvalue;
			break;
		default:
			nvalue_reciprocal = sr->opp3_nvalue;
			break;
		}
	}

	current_nvalue = sr_read_reg(sr, NVALUERECIPROCAL);

	if (current_nvalue == nvalue_reciprocal) {
		DPRINTK("System is already at the desired voltage level\n");
		return;
	}

	sr_write_reg(sr, NVALUERECIPROCAL, nvalue_reciprocal);

#ifdef CONFIG_ENABLE_SR_CLASS2
	/* Enable the interrupt */
	sr_modify_reg(sr, ERRCONFIG,
		      (ERRCONFIG_MCUBOUNDINTEN | ERRCONFIG_MCUBOUNDINTEN),
		      (ERRCONFIG_MCUBOUNDINTEN | ERRCONFIG_MCUBOUNDINTEN));
#endif
	/* Enable the interrupt */
	sr_modify_reg(sr, ERRCONFIG,
		      (ERRCONFIG_VPBOUNDINTEN | ERRCONFIG_VPBOUNDINTST),
		      (ERRCONFIG_VPBOUNDINTEN | ERRCONFIG_VPBOUNDINTST));

	if (sr->srid == SR1) {
#ifndef CONFIG_ENABLE_SR_CLASS2
		/* Enable VP1 */
		PRM_VP1_CONFIG |= PRM_VP1_CONFIG_VPENABLE;
#endif
	} else if (sr->srid == SR2) {
		/* Enable VP2 */
		PRM_VP2_CONFIG |= PRM_VP2_CONFIG_VPENABLE;
	}

	/* SRCONFIG - enable SR */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE, SRCONFIG_SRENABLE);

}

static void sr_disable(struct omap_sr *sr)
{
	sr->is_sr_reset = 1;
	/* SRCONFIG - disable SR */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE, ~SRCONFIG_SRENABLE);
	if (sr->srid == SR1) {
#ifndef CONFIG_ENABLE_SR_CLASS2
		/* Enable VP1 */
		PRM_VP1_CONFIG &= ~PRM_VP1_CONFIG_VPENABLE;
#endif
	} else if (sr->srid == SR2) {
		/* Enable VP2 */
		PRM_VP2_CONFIG &= ~PRM_VP2_CONFIG_VPENABLE;
	}
}

void sr_start_vddautocomap(int srid, u32 target_opp_no)
{
	struct omap_sr *sr = NULL;

	if (srid == SR1) {
		sr = &sr1;
	} else if (srid == SR2) {
		sr = &sr2;
	}

	if (sr->is_sr_reset == 1) {
		sr_clk_enable(sr);
		sr_configure(sr);
	}

	if (sr->is_autocomp_active == 1)
		DPRINTK(KERN_WARNING "SR%d: VDD autocomp is already active\n",
			srid);

	sr->is_autocomp_active = 1;
	sr_enable(sr, target_opp_no);
}

EXPORT_SYMBOL(sr_start_vddautocomap);

int sr_stop_vddautocomap(int srid)
{
	struct omap_sr *sr = NULL;

	if (srid == SR1) {
		sr = &sr1;
	} else if (srid == SR2) {
		sr = &sr2;
	}

	if (sr->is_autocomp_active == 1) {
		sr_disable(sr);
		sr_clk_disable(sr);
		sr->is_autocomp_active = 0;
		return SR_TRUE;
	} else {
		DPRINTK(KERN_WARNING "SR%d: VDD autocomp is not active\n",
			srid);
		return SR_FALSE;
	}

}

EXPORT_SYMBOL(sr_stop_vddautocomap);

void enable_smartreflex(int srid)
{
	u32 target_opp_no = 0;
	struct omap_sr *sr = NULL;

	if (srid == SR1) {
		sr = &sr1;
	} else if (srid == SR2) {
		sr = &sr2;
	}

	if (sr->is_autocomp_active == 1) {
		if (sr->is_sr_reset == 1) {
			if (srid == SR1) {
				/* Enable SR clks */
				CM_FCLKEN_WKUP |= SR1_CLK_ENABLE;
				target_opp_no = get_opp_no(current_vdd1_opp);

			} else if (srid == SR2) {
				/* Enable SR clks */
				CM_FCLKEN_WKUP |= SR2_CLK_ENABLE;
				target_opp_no = get_opp_no(current_vdd2_opp);
			}

			sr_configure(sr);

			sr_enable(sr, target_opp_no);
		}
	}

}

void disable_smartreflex(int srid)
{
	struct omap_sr *sr = NULL;

	if (srid == SR1) {
		sr = &sr1;
	} else if (srid == SR2) {
		sr = &sr2;
	}

	if (sr->is_autocomp_active == 1) {
		if (srid == SR1) {
			/* Enable SR clk */
			CM_FCLKEN_WKUP |= SR1_CLK_ENABLE;

		} else if (srid == SR2) {
			/* Enable SR clk */
			CM_FCLKEN_WKUP |= SR2_CLK_ENABLE;
		}

		if (sr->is_sr_reset == 0) {

			sr->is_sr_reset = 1;
			/* SRCONFIG - disable SR */
			sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE,
				      ~SRCONFIG_SRENABLE);

			if (sr->srid == SR1) {
				/* Disable SR clk */
				CM_FCLKEN_WKUP &= ~SR1_CLK_ENABLE;
#ifndef CONFIG_ENABLE_SR_CLASS2
				/* Enable VP1 */
				PRM_VP1_CONFIG &= ~PRM_VP1_CONFIG_VPENABLE;
#endif

			} else if (sr->srid == SR2) {
				/* Disable SR clk */
				CM_FCLKEN_WKUP &= ~SR2_CLK_ENABLE;
				/* Enable VP2 */
				PRM_VP2_CONFIG &= ~PRM_VP2_CONFIG_VPENABLE;
			}
		}
	}
}

#ifdef CONFIG_OMAP_VOLT_SR_BYPASS

/* Voltage Scaling using SR VCBYPASS */
int sr_voltagescale_vcbypass(u32 target_opp, u8 vsel)
{
	int sr_status = 0;
	u32 vdd, target_opp_no;

	vdd = get_vdd(target_opp);
	target_opp_no = get_opp_no(target_opp);

	if (vdd == PRCM_VDD1) {
		sr_status = sr_stop_vddautocomap(SR1);

	} else if (vdd == PRCM_VDD2) {
		sr_status = sr_stop_vddautocomap(SR2);
	}

	if (set_voltage_level_vcbypass(vdd, vsel)) {
		printk(KERN_ERR "Unable to set the voltage level \n");
		return SR_FAIL;
	}
	if (sr_status) {
		if (vdd == PRCM_VDD1)
			sr_start_vddautocomap(SR1, target_opp_no);
		else if (vdd == PRCM_VDD2)
			sr_start_vddautocomap(SR2, target_opp_no);
	}

	return SR_PASS;
}
#elif CONFIG_OMAP_VOLT_VSEL
int sr_set_voltage_level(u32 target_opp, u8 vsel)
{
	int sr_status = 0;
	u32 vdd, target_opp_no;

	vdd = get_vdd(target_opp);
	target_opp_no = get_opp_no(target_opp);

	if (vdd == PRCM_VDD1) {
		sr_status = sr_stop_vddautocomap(SR1);

	} else if (vdd == PRCM_VDD2) {
		sr_status = sr_stop_vddautocomap(SR2);

	}
	if (set_voltage_level(vdd, vsel)) {
		printk(KERN_ERR "Unable to set the voltage level \n");
		return SR_FAIL;
	}
	if (sr_status) {
		if (vdd == PRCM_VDD1)
			sr_start_vddautocomap(SR1, target_opp_no);
		else if (vdd == PRCM_VDD2)
			sr_start_vddautocomap(SR2, target_opp_no);
	}

	return SR_PASS;

}
#endif
/* Sysfs interface to select SR VDD1 auto compensation */
static ssize_t omap_sr_vdd1_autocomp_show(struct kset *subsys, char *buf)
{
	return sprintf(buf, "%d\n", sr1.is_autocomp_active);
}

static ssize_t omap_sr_vdd1_autocomp_store(struct kset *subsys,
					   const char *buf, size_t n)
{
	u32 current_vdd1opp_no;
	unsigned short value;

	if (sscanf(buf, "%hu", &value) != 1 || (value > 1)) {
		printk(KERN_ERR "sr_vdd1_autocomp: Invalid value\n");
		return -EINVAL;
	}

	current_vdd1opp_no = get_opp_no(current_vdd1_opp);

	if (value == 0) {
		sr_stop_vddautocomap(SR1);
	} else {
		sr_start_vddautocomap(SR1, current_vdd1opp_no);
	}

	return n;
}

static struct subsys_attribute sr_vdd1_autocomp = {
	.attr = {
		 .name = __stringify(sr_vdd1_autocomp),
		 .mode = 0644,
		 },
	.show = omap_sr_vdd1_autocomp_show,
	.store = omap_sr_vdd1_autocomp_store,
};

/* Sysfs interface to select SR VDD2 auto compensation */
static ssize_t omap_sr_vdd2_autocomp_show(struct kset *subsys, char *buf)
{
	return sprintf(buf, "%d\n", sr2.is_autocomp_active);
}

static ssize_t omap_sr_vdd2_autocomp_store(struct kset *subsys,
					   const char *buf, size_t n)
{
	u32 current_vdd2opp_no;
	unsigned short value;

	if (sscanf(buf, "%hu", &value) != 1 || (value > 1)) {
		printk(KERN_ERR "sr_vdd2_autocomp: Invalid value\n");
		return -EINVAL;
	}

	current_vdd2opp_no = get_opp_no(current_vdd2_opp);

	if (value == 0) {
		sr_stop_vddautocomap(SR2);
	} else {
		sr_start_vddautocomap(SR2, current_vdd2opp_no);
	}

	return n;
}

static struct subsys_attribute sr_vdd2_autocomp = {
	.attr = {
		 .name = __stringify(sr_vdd2_autocomp),
		 .mode = 0644,
		 },
	.show = omap_sr_vdd2_autocomp_show,
	.store = omap_sr_vdd2_autocomp_store,
};

int subsys_create_file(struct kset *s, struct subsys_attribute *a);
/**
 *      subsystem_remove_file - remove sysfs attribute file.
 *      @s:     subsystem.
 *      @a:     attribute desciptor.
 */
 
int subsys_remove_file(struct kset  *s, struct subsys_attribute *a)
{
	if (!s || !a)
		return -EINVAL;

         if (subsys_get(s)) {
                 sysfs_remove_file(&s->kobj, &a->attr);
                 subsys_put(s);
         }
	 return 0;
}

static int __init omap3_sr_init(void)
{
	int ret = 0;
#ifdef CONFIG_OMAP_VOLT_SR_BYPASS
	u8 RdReg;
#elif CONFIG_OMAP_VOLT_VSEL
	wq_sr.vsel = 0;
	current_voltage = 0;
	last_vdd1_opp = -1;
	INIT_WORK(&wq_sr.work, set_voltage);
#endif

#ifdef CONFIG_ARCH_OMAP34XX
	sr1.fck = clk_get(NULL, "sr1_fck");
	if (IS_ERR(sr1.fck))
		printk(KERN_ERR "Could not get sr1_fck\n");

	sr2.fck = clk_get(NULL, "sr2_fck");
	if (IS_ERR(sr2.fck))
		printk(KERN_ERR "Could not get sr2_fck\n");
#endif				/* #ifdef CONFIG_ARCH_OMAP34XX */

	/* Call the VPConfig, VCConfig, set N Values. */
	sr_set_nvalues(&sr1);
	sr_configure_vp(SR1);

	sr_set_nvalues(&sr2);
	sr_configure_vp(SR2);

	sr_configure_vc();

#ifdef  CONFIG_ENABLE_SR_CLASS2
	if (request_irq(INT_34XX_SR1_IRQ, omap_sr1_class2_irq, 0, "SR1_irq", NULL)) {
		printk("Could not install ISR\n");
	}
#endif
#ifdef CONFIG_OMAP_VOLT_SR_BYPASS
	if (machine_is_omap_2430sdp() || machine_is_omap_3430sdp()
		|| machine_is_omap3_evm())
	{
		/* Enable SR on T2 */
		ret = t2_in(PM_RECEIVER, &RdReg, R_DCDC_GLOBAL_CFG);
		RdReg |= DCDC_GLOBAL_CFG_ENABLE_SRFLX;
		ret |= t2_out(PM_RECEIVER, RdReg, R_DCDC_GLOBAL_CFG);
	}
#endif
	printk(KERN_INFO "SmartReflex driver initialized\n");

	ret = subsys_create_file(&power_subsys, &sr_vdd1_autocomp);
	if (ret)
		printk(KERN_ERR "subsys_create_file failed: %d\n", ret);

	ret = subsys_create_file(&power_subsys, &sr_vdd2_autocomp);
	if (ret)
		printk(KERN_ERR "subsys_create_file failed: %d\n", ret);

	return 0;
}

static void __exit omap3_sr_exit(void)
{
#ifdef CONFIG_ARCH_OMAP34XX
	clk_put(sr1.fck);
	clk_put(sr2.fck);
#endif
	subsys_remove_file(&power_subsys, &sr_vdd1_autocomp);
	subsys_remove_file(&power_subsys, &sr_vdd2_autocomp);
}

MODULE_AUTHOR("Texas Instruments, Inc.");
MODULE_DESCRIPTION("SmartReflex Class2 driver");
MODULE_LICENSE("GPL");

module_init(omap3_sr_init);
module_exit(omap3_sr_exit);
