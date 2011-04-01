/*
 * arch/arm/plat-omap/power_companion.c
 *
 * This file contains code to control TWL4030 power compaion chip operation.
 *
 * Copyright (C) 2006 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */
#if defined (CONFIG_TWL4030_CORE) 
#include <linux/types.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <asm/arch/power_companion.h>
#include <asm/arch/clock.h>
#include <asm/arch/mux.h>
#include <asm/arch/prcm.h>
#include <asm/semaphore.h>

/* Note: H4 uses the Menelaus companion chip.  Currently it is assumed u-boot
 * has set up the chip.  For Triton2, some part of this is done by the loader.
 * However, it turns out that a polled mode u-boot I2C setup can take a long
 * time.  So major init sequences are better done here.  T2 needs to setup
 * sequences before it can effectively shut down its HF clock and handle more
 * complex power needs.
 */

int power_companion_initialized = 0;

struct semaphore pwr_reg_lock;

static int vaux2_ldo_use_count = 0;

int twl4030_vaux2_ldo_use(void)
{
	vaux2_ldo_use_count++;

	/* If first module to use the VAUX2 voltage, enable the voltage */
	if(vaux2_ldo_use_count == 1){
		if(0!= t2_out(PM_RECEIVER, ENABLE_VAUX2_DEDICATED,
					  TWL4030_VAUX2_DEDICATED)) {
			vaux2_ldo_use_count --;
			return -EIO;
		}
		if(0!= t2_out(PM_RECEIVER, ENABLE_VAUX2_DEV_GRP,
					  TWL4030_VAUX2_DEV_GRP)) {
			vaux2_ldo_use_count --;
			return -EIO;
		}
	}
	return 0;
}

int twl4030_vaux2_ldo_unuse(void)
{
	if (vaux2_ldo_use_count == 0)
		return 0;

	/* If VAUX2 voltage is not used by any modules, disable the voltage */
	vaux2_ldo_use_count--;
	if(vaux2_ldo_use_count == 0){
		/*
		 * The VAUX2 voltage is not used by any module
		 * Disable the voltage
		 */
		if( 0 != t2_out(PM_RECEIVER, 0x00, TWL4030_VAUX2_DEDICATED)) {
			vaux2_ldo_use_count ++;
			return -EIO;
		}
		if( 0 != t2_out(PM_RECEIVER, 0x00, TWL4030_VAUX2_DEV_GRP)) {
			vaux2_ldo_use_count ++;
			return -EIO;
		}
	}
	return 0;
}
EXPORT_SYMBOL(twl4030_vaux2_ldo_use);
EXPORT_SYMBOL(twl4030_vaux2_ldo_unuse);

static int vaux3_ldo_use_count = 0;
int twl4030_vaux3_ldo_use(void)
{
	vaux3_ldo_use_count++;

	/* If first module to use the VAUX2 voltage, enable the voltage */
	if(vaux3_ldo_use_count == 1){
		if(0!= t2_out(PM_RECEIVER, ENABLE_VAUX3_DEDICATED,
					  TWL4030_VAUX3_DEDICATED)) {
			vaux3_ldo_use_count --;
			return -EIO;
		}
		if(0!= t2_out(PM_RECEIVER, ENABLE_VAUX3_DEV_GRP,
					  TWL4030_VAUX3_DEV_GRP)) {
			vaux3_ldo_use_count --;
			return -EIO;
		}
	}
	return 0;
}
int twl4030_vaux3_ldo_unuse(void)
{
	if (vaux3_ldo_use_count == 0)
		return 0;

	/* If VAUX3 voltage is not used by any modules, disable the voltage */
	vaux3_ldo_use_count--;
	if(vaux3_ldo_use_count == 0){
		/*
		 * The VAUX3 voltage is not used by any module
		 * Disable the voltage
		 */
		if( 0 != t2_out(PM_RECEIVER, 0x00, TWL4030_VAUX3_DEDICATED)) {
			vaux3_ldo_use_count ++;
			return -EIO;
		}

		if( 0 != t2_out(PM_RECEIVER, 0x00, TWL4030_VAUX3_DEV_GRP)) {
			vaux3_ldo_use_count ++;
			return -EIO;
		}
	}
	return 0;
}
EXPORT_SYMBOL(twl4030_vaux3_ldo_use);
EXPORT_SYMBOL(twl4030_vaux3_ldo_unuse);

#ifdef CONFIG_MACH_OMAP3_EVM

static int vaux4_ldo_use_count = 0;
int twl4030_vaux4_ldo_use(void)
{
        vaux4_ldo_use_count++;

        /* If first module to use the VAUX4 voltage, enable the voltage */
        if(vaux4_ldo_use_count == 1){
                if(0!= t2_out(PM_RECEIVER, ENABLE_VAUX4_DEDICATED,
                                          TWL4030_VAUX4_DEDICATED)) {
                        vaux4_ldo_use_count --;
                        return -EIO;
                }
                if(0!= t2_out(PM_RECEIVER, ENABLE_VAUX4_DEV_GRP,
                                          TWL4030_VAUX4_DEV_GRP)) {
                        vaux4_ldo_use_count --;
                        return -EIO;
                }
        }
        return 0;
}

int twl4030_vaux4_ldo_unuse(void)
{
        if (vaux4_ldo_use_count == 0)
                return 0;

        /* If VAUX4 voltage is not used by any modules, disable the voltage */
        vaux4_ldo_use_count--;
        if(vaux4_ldo_use_count == 0){
                /*
                 * The VAUX4 voltage is not used by any module
                 * Disable the voltage
                 */
                if( 0 != t2_out(PM_RECEIVER, 0x00, TWL4030_VAUX4_DEDICATED)) {
                        vaux4_ldo_use_count ++;
                        return -EIO;
                }

                if( 0 != t2_out(PM_RECEIVER, 0x00, TWL4030_VAUX4_DEV_GRP)) {
                        vaux4_ldo_use_count ++;
                        return -EIO;
                }
        }
        return 0;
}
EXPORT_SYMBOL(twl4030_vaux4_ldo_use);
EXPORT_SYMBOL(twl4030_vaux4_ldo_unuse);
#endif

int enable_vmode_companion_voltage_scaling(int vdd, int floor, int roof,
								int mode)
{
	int e = 0;

	/* set floor, roof voltage, enable jump mode, enable scaling */
	if (vdd == PRCM_VDD1) {
		e |= t2_out(PM_RECEIVER, floor, R_VDD1_VFLOOR);
		e |= t2_out(PM_RECEIVER, roof, R_VDD1_VROOF);
		e |= t2_out(PM_RECEIVER, mode, R_VDD1_STEP);
		e |= t2_out(PM_RECEIVER, EN_SCALE, R_VDD1_VMODE_CFG);
	}
	else if (vdd == PRCM_VDD2) {
		e |= t2_out(PM_RECEIVER, floor, R_VDD2_VFLOOR);
		e |= t2_out(PM_RECEIVER, roof, R_VDD2_VROOF);
		e |= t2_out(PM_RECEIVER, mode, R_VDD2_STEP);
		e |= t2_out(PM_RECEIVER, EN_SCALE, R_VDD2_VMODE_CFG);
	}
	if(e){
		printk(KERN_ERR "Error enabling voltage scaling\n");
		e = -EIO;
	}
	return e;
}
EXPORT_SYMBOL(enable_vmode_companion_voltage_scaling);

int disable_vmode_companion_voltage_scaling(void)
{
	int e = 0;
	e = t2_out(PM_RECEIVER, DIS_SCALE, R_VDD1_VMODE_CFG);
	if(e)
		e = -EIO;
	return e;
}
EXPORT_SYMBOL(disable_vmode_companion_voltage_scaling);

int write_val_companion_reg(u8 mod_no, u8 value, u8 reg)
{
	int e = 0;

	down(&pwr_reg_lock);
	e |= t2_out(mod_no, value, reg);
	up(&pwr_reg_lock);
	if(e){
		printk(KERN_ERR "TWL4030 Power Companion Register Access Error\n");
		e = -EIO;
	}
	return e;
}
EXPORT_SYMBOL(write_val_companion_reg);

int set_bits_companion_reg(u8 mod_no, u8 value, u8 reg)
{
	int e = 0;
	u8 RdReg;

	down(&pwr_reg_lock);
	e |= t2_in(mod_no, &RdReg, reg);
	RdReg |= value;
	e |= t2_out(mod_no, RdReg, reg);
	up(&pwr_reg_lock);
	if(e){
		printk(KERN_ERR "TWL4030 Power Companion Register Access Error\n");
		e = -EIO;
	}
	return e;
}
EXPORT_SYMBOL(set_bits_companion_reg);

int clear_bits_companion_reg(u8 mod_no, u8 value, u8 reg)
{
	int e = 0;
	u8 RdReg;

	down(&pwr_reg_lock);
	e |= t2_in(mod_no, &RdReg, reg);
	RdReg &= ~value;
	e |= t2_out(mod_no, RdReg, reg);
	up(&pwr_reg_lock);
	if(e){
		printk(KERN_ERR "TWL4030 Power Companion Register Access Error\n");
		e = -EIO;
	}
	return e;
}
EXPORT_SYMBOL(clear_bits_companion_reg);

static int protect_pm_master(void)
{
	int e = 0;
	e = t2_out(PM_MASTER, KEY_LOCK, R_PROTECTKEY);
	return e;
}

static int unprotect_pm_master(void)
{
	int e = 0;
	e |= t2_out(PM_MASTER, KEY_UNLOCK1, R_PROTECTKEY);
	e |= t2_out(PM_MASTER, KEY_UNLOCK2, R_PROTECTKEY);
	return e;
}

static int init_pwr_intc(void)
{
	int e;

	/* Set interrupt to clear when we write it. Enable interrupt
	 * exclusively between INT1 and INT2
	 */
	e = t2_out(PM_INT, (SIH_COR|SIH_EXCLEN), R_PWR_SIH_CTRL);

	/* mask all PWR INT1/2 interrupts */
	e |= t2_out(PM_INT, 0xFF, R_PWR_IMR1);
	e |= t2_out(PM_INT, 0xFF, R_PWR_IMR2);

	/* disable falling and rising edge of all PWR sources */
	e |=t2_out(PM_INT, 0x00, R_PWR_EDR1);
	e |=t2_out(PM_INT, 0x00, R_PWR_EDR2);

	/* clear all PWR INT1/2 interrupts */
	e |= t2_out(PM_INT, 0xFF, R_PWR_ISR1);
	e |= t2_out(PM_INT, 0xFF, R_PWR_ISR2);

	return e;
}

/* Set up an action to shut down T2's clocks and wake them up
 * following P1.  Depending on usage this sequence might be
 * reduced to just a unicast to the HFCLKOUT resource.
 */
static int config_sleep_wake_sequence(void)
{
	#define A2S_I 2
	#define AS2_J 4
#if defined(CONFIG_MACH_OMAP_3430SDP) || defined(CONFIG_MACH_OMAP3_EVM)
	#define S2A_I 2
#else
	#define S2A_I 3
#endif
	#define S2A_J 4
#if defined(CONFIG_MACH_OMAP_3430SDP) || defined(CONFIG_MACH_OMAP3_EVM)
	u8 sleep_on_seq[A2S_I][AS2_J] = {{0x20, 0xF0, 0x04, 0x2C},
						{0x21, 0x00, 0x02, 0x3F} };

	u8 sleep_off_seq[S2A_I][S2A_J] = {{0x20, 0xFE, 0x30, 0x2E},
						{0x21, 0x0E, 0x37, 0x2F} };
#else
	u8 sleep_on_seq[A2S_I][AS2_J] = {{0x14, 0x78, 0x04, 0x2C},
						{0x1E, 0x78, 0x02, 0x3F} };

	u8 sleep_off_seq[S2A_I][S2A_J] = {{0x01, 0x7E, 0x30, 0x2E},
						{0x1A, 0x7E, 0x37, 0x2F},
						{0x1E, 0x7E, 0x02, 0x3F} };
#endif
	int e=0, i=0, j=0;
	u8 data;

	/* CLKREQ is pulled high on the 2430SDP, therefore, we need to take
	 * it out of the HFCLKOUT DEV_GRP for P1 else HFCLKOUT can't be stopped.
	 */
	e |=t2_out(PM_RECEIVER, 0x20, R_HFCLKOUT_DEV_GRP);

	/* Set ACTIVE to SLEEP SEQ address in T2 memory*/
	e |=t2_out(PM_MASTER, 0x2B, R_SEQ_ADD_A2S);
	/* Set SLEEP to ACTIVE SEQ address for P1 and P2 */
	e |=t2_out(PM_MASTER, 0x2D, R_SEQ_ADD_SA12);
	/* Set SLEEP to ACTIVE SEQ address for P3 */
	e |=t2_out(PM_MASTER, 0x2D, R_SEQ_ADD_S2A3);

	/* Install Active->Sleep (A2S) sequence */
	while (i < A2S_I){
		while (j < AS2_J){
			data = (((0x2B + i) << 2) + j );  /*set starting t2-addr*/
			e |= t2_out(PM_MASTER, data, R_MEMORY_ADDRESS); /*select t2-addr*/
			data = sleep_on_seq[i][j]; /* get data */
			e |= t2_out(PM_MASTER, data, R_MEMORY_DATA); /*data into t2-addr*/
			j++;
		}
		j=0;
		i++;
	}

	i = j = 0;

	/* Install Sleep->Active (S2A) sequence */
	while (i < S2A_I){
		while (j < S2A_J){
			data = (((0x2D + i) << 2) + j ); /* set starting t2-addr */
			e |= t2_out(PM_MASTER, data, R_MEMORY_ADDRESS); /* sel t2-addr */
			data = sleep_off_seq[i][j];       /* get data */
			e |= t2_out(PM_MASTER, data, R_MEMORY_DATA); /* put into t2-addr */
			j++;
		}
		j=0;
		i++;
	}
#if defined(CONFIG_MACH_OMAP_3430SDP) || defined(CONFIG_MACH_OMAP3_EVM)
	/* Disabling AC charger effect on sleep-active transitions */
	e |= t2_in(PM_MASTER, &data, R_CFG_P1_TRANSITION);
	data &= 0x0;
	e |= t2_out(PM_MASTER, data , R_CFG_P1_TRANSITION);
#endif
	/* P1/P2/P3 LVL_WAKEUP should be on LEVEL */
	e |= t2_out(PM_MASTER, LVL_WAKEUP, R_P1_SW_EVENTS);
	e |= t2_out(PM_MASTER, LVL_WAKEUP, R_P2_SW_EVENTS);
	e |= t2_out(PM_MASTER, LVL_WAKEUP, R_P3_SW_EVENTS);

	if(e)
		printk(KERN_ERR "TWL4030 Power Companion seq config error\n");

	return e;
}

#ifdef CONFIG_OMAP_VOLT_VSEL
/*
 * This function sets the VSEL values in the Triton Power IC. This is done in
 * the software configurable mode.
 */
int set_voltage_level (u8 vdd, u8 vsel)
{
	int e = 0;

	/* Disabling the SmartReflex */
	if ((e = clear_bits_companion_reg(PM_RECEIVER,
					 DCDC_GLOBAL_CFG_ENABLE_SRFLX,
					 R_DCDC_GLOBAL_CFG))) {
		printk (KERN_INFO "Unable to disable SR\n");
		return e;
	}

	if (vdd == PRCM_VDD1) {
		/* Enable in software configuration mode, by disabling the
		 * hardware mode.
		 */
		if ((e = clear_bits_companion_reg(PM_RECEIVER,
						 VDD1_VMODE_CFG_ENABLE_VMODE,
						 R_VDD1_VMODE_CFG))) {
			printk (KERN_INFO "Unable to set Power IC in software \
				controlled mode\n");
			return e;
		}

		/* Set the step mode to single step */
		if ((e = set_bits_companion_reg (PM_RECEIVER,
						VDD1_STEP_STEP_REG_0,
						R_VDD1_STEP))) {
			printk (KERN_INFO "Unable to set the Voltage Step to \
				zero\n");
			return e;
		}

		/* Set the VSEL value */
		if ((e = write_val_companion_reg (PM_RECEIVER,
						 vsel,
						 R_VDD1_VSEL))) {
			printk (KERN_INFO "Unable to set the Voltage level to \
				vsel\n");
			return e;
		}
	}
	else { /* VDD2 */
		/* Presently only 2 OPP levels (i.e. OPP1 and OPP2) are supported */

		/* Enable in software configuration mode, by disabling the
		 * hardware mode.
		 */
		if ((e = clear_bits_companion_reg(PM_RECEIVER,
						 VDD2_VMODE_CFG_ENABLE_VMODE,
						 R_VDD2_VMODE_CFG))) {
			printk (KERN_INFO "Unable to set Power IC in software \
				controlled mode\n");
			return e;
		}

		/* Set the step mode to single step */
		if ((e = set_bits_companion_reg (PM_RECEIVER,
						VDD2_STEP_STEP_REG_0,
						R_VDD2_STEP))) {
			printk (KERN_INFO "Unable to set the Voltage Step to \
				zero\n");
			return e;
		}

		/* Set the VSEL value */
		if ((e = write_val_companion_reg (PM_RECEIVER,
						 vsel,
						 R_VDD2_VSEL))) {
			printk (KERN_INFO "Unable to set the Voltage level to \
				vsel\n");
			return e;
		}
	}
	/* 10 microsecond delay for the single step voltage to stabilize */
	udelay (10);

	return e;
}
EXPORT_SYMBOL(set_voltage_level);
#endif /* CONFIG_OMAP_VOLT_VSEL */

int power_companion_init(void)
{
	struct clk *osc;
	u32 rate, ctrl = HFCLK_FREQ_26_MHZ;
	int e = 0;

	if(power_companion_initialized != 0)
		return 0;

	osc = clk_get(NULL,"osc_ck");
	rate = clk_get_rate(osc);
	clk_put(osc);

	switch(rate) {
	case 19200000 : ctrl = HFCLK_FREQ_19p2_MHZ; break;
	case 26000000 : ctrl = HFCLK_FREQ_26_MHZ; break;
	case 38400000 : ctrl = HFCLK_FREQ_38p4_MHZ; break;
	}
	ctrl |= HIGH_PERF_SQ;

	/* Semaphore lock to prevent concurrent access */
	init_MUTEX(&pwr_reg_lock);

	e |= unprotect_pm_master();
	e |= t2_out(PM_MASTER, ctrl, R_CFGBOOT); /* effect->MADC+USB ck en */
	e |= init_pwr_intc();
	e |= config_sleep_wake_sequence(); /* Set action for P1-SLEEP1 */
	e |= protect_pm_master();

#ifdef CONFIG_OMAP_VOLT_VMODE
	if (cpu_is_omap24xx())
		e |= enable_vmode_companion_voltage_scaling(PRCM_VDD1, V_1p05,
				V_1p3, EN_JMP);
	else if (cpu_is_omap3430()) {
		e |= enable_vmode_companion_voltage_scaling(PRCM_VDD1, V_1p05,
				V_1p2, EN_JMP);
		e |= enable_vmode_companion_voltage_scaling(PRCM_VDD2, V_1p15,
				V_1p15, EN_JMP);
	}
#elif defined(CONFIG_OMAP_VOLT_SR) || defined(CONFIG_OMAP_VOLT_SR_BYPASS)
	/* Enabling the SmartReflex */
	e = set_bits_companion_reg(PM_RECEIVER, DCDC_GLOBAL_CFG_ENABLE_SRFLX,
							R_DCDC_GLOBAL_CFG);
	if (e)
		printk(KERN_INFO "Unable to enable SR\n");
#endif

	if(e){
		printk(KERN_ERR "TWL4030 Power Companion Init Error\n");
		e = -EIO;
	} else {
		power_companion_initialized = 1;
		printk(KERN_INFO "TWL4030 Power Companion Active\n");
	}
	return e;
}
EXPORT_SYMBOL(power_companion_init);
#endif
