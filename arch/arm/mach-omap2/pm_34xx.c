/*
 * linux/arch/arm/mach-omap2/pm_34xx.c
 *
 * OMAP3 Power Management Routines
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Karthik Dasu <karthik-dp@ti.com>
 *
 * Copyright (C) 2006 Nokia Corporation
 * Tony Lindgren <tony@atomide.com>
 *
 * Copyright (C) 2005 Texas Instruments, Inc.
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * Based on pm.c for omap1
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/cpuidle.h>
#include <linux/tick.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/atomic.h>
#include <asm/mach/time.h>
#include <asm/mach/irq.h>
#include <asm/mach-types.h>
#include <linux/mm.h>
#include <asm/mmu.h>
#include <asm/tlbflush.h>

#include <asm/arch/irqs.h>
#include <asm/arch/clock.h>
#include <asm/arch/sram.h>
#include <asm/arch/pm.h>
#include <asm/arch/sr_core.h>

#ifdef CONFIG_OMAP34XX_OFFMODE
#include <asm/arch/io.h>
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */

#define SR1_ID 1
#define SR2_ID 2

#include "prcm-regs.h"

/*
 * Define DEBUG_HW_SUP for extended information during 'cat /proc/pm_prepwst'
 */
#define DEBUG_HW_SUP

/*
 * Define DEBUG_PREPWST for getting state history information during 
 * 'cat /proc/pm_prepwst'
 */
#define DEBUG_PREPWST

#define S600M   600000000
#define S550M	550000000
#define S500M	500000000
#define S250M	250000000
#define S125M	125000000
#define S19M	 19200000
#define S120M	120000000
#define S477M	476000000
#define S381M	381000000
#define S191M	190000000
#define S96M	 96000000
#define S166M	166000000
#define S83M	 83000000

#define PRCM_MPU_IRQ	0xB		/* MPU IRQ 11 */
#define CORE_FCLK_MASK	0x3FF9E29 	/* Mask of all functional clocks*/
					/* in core except UART */
#ifdef CONFIG_OMAP3430_ES2
#define CORE3_FCLK_MASK		0x7	/* Mask of all functional clocks */
#define USBHOST_FCLK_MASK	0x3	/* Mask of all functional clocks in USB */
#ifndef CONFIG_ARCH_OMAP3410
#define SGX_FCLK_MASK		0x2	/* Mask of all functional clocks in SGX */
#endif
#else
#define GFX_FCLK_MASK		0x6	/* Mask of all functional clocks in GFX */
#endif

#define DSS_FCLK_MASK		0x7	/* Mask of all functional clocks in DSS */
#define CAM_FCLK_MASK		0x1 	/* Mask of all functional clocks in CAM */
#define PER_FCLK_MASK		0x17FF	/* Mask of all functional clocks in PER */
					/* except UART and GPIO */

#define CORE1_ICLK_VALID	0x3FFFFFF9 /*Ignore SDRC_ICLK*/
#define CORE2_ICLK_VALID	0x1F

#ifdef CONFIG_OMAP3430_ES2
#define CORE3_ICLK_VALID	0x4
#define USBHOST_ICLK_VALID	0x1
#ifndef CONFIG_ARCH_OMAP3410
#define	SGX_ICLK_VALID		0x1
#endif
#else	/* ifdef CONFIG_OMAP3430_ES2 */
#define	GFX_ICLK_VALID		0x1
#endif	/* ifdef CONFIG_OMAP3430_ES2 */

#define DSS_ICLK_VALID		0x1
#define CAM_ICLK_VALID		0x1
#define PER_ICLK_VALID 		0x1
#define WKUP_ICLK_VALID 	0x3E /* Ignore GPT1 ICLK as it is handled explicitly*/

#define OMAP3_MAX_STATES	7
#define OMAP3_STATE_C0		0 /* MPU ACTIVE + CORE ACTIVE(only WFI)*/
#define OMAP3_STATE_C1 		1 /* MPU WFI + Dynamic tick + CORE ACTIVE*/
#define OMAP3_STATE_C2 		2 /* MPU CSWR + CORE ACTIVE*/
#define OMAP3_STATE_C3 		3 /* MPU OFF + CORE ACTIVE*/
#define OMAP3_STATE_C4 		4 /* MPU RET + CORE CSWR */
#define OMAP3_STATE_C5 		5 /* MPU OFF + CORE CSWR */
#define OMAP3_STATE_C6 		6 /* MPU OFF + CORE OFF */

#ifdef CONFIG_CORE_OFF_CPUIDLE
#define MAX_SUPPORTED_STATES	7
#else
#define MAX_SUPPORTED_STATES 6
#endif

#define IOPAD_WKUP	1

/*#define DEBUG_CPUIDLE 1*/
#ifdef DEBUG_CPUIDLE
#define DPRINTK(fmt, args...) printk(KERN_ERR "%s: " fmt, \
					__FUNCTION__ , ## args)
#else
#define DPRINTK(fmt, args...)
#endif

#if defined (CONFIG_PREVENT_MPU_RET)
int  cpuidle_deepest_st = 1;
#elif defined (CONFIG_PREVENT_CORE_RET)
int  cpuidle_deepest_st = 3;
#else
#if defined (CONFIG_MACH_OMAP3EVM)
/* On the OMAP3EVM, start with no/ minimal power management */
int cpuidle_deepest_st = 1;
#else
int cpuidle_deepest_st = (MAX_SUPPORTED_STATES - 1);
#endif	/* defined (CONFIG_MACH_OMAP3EVM) */
#endif

/* Macro for debugging */
#define RESTORE_TABLE_ENTRY 1

/*
 * Time-out value for frame buffer (in seconds).
 */
int fb_timeout_val = 30;

u32 target_opp_no;
int enable_off = 1;
u32 debug_domain = DOM_CORE1;

struct res_handle  *memret1;
struct res_handle  *memret2;
struct res_handle  *logret1;
int res1_level = -1, res2_level = -1, res3_level = -1;

static struct constraint_handle *vdd1_opp_co;
static struct constraint_handle *vdd2_opp_co;

static struct constraint_id cnstr_id1 = {
	.type = RES_FREQ_CO,
	.data = (void *)"vdd1_opp",
};
static struct constraint_id cnstr_id2 = {
	.type = RES_FREQ_CO,
	.data = (void *)"vdd2_opp",
};

/*
 * Time-out value for UART before cutting clock (in msecs).
 */
#define UART_TIME_OUT	6000

extern void omap_uart_save_ctx(int unum);
extern void omap_uart_restore_ctx(int unum);
extern void set_blank_interval(int fb_timeout_val);

#define SCRATCHPAD_ROM			0x48002860
#define SCRATCHPAD			0x48002910
#define SCRATCHPAD_ROM_OFFSET		0x19C
#define TABLE_ADDRESS_OFFSET		0x31
#define TABLE_VALUE_OFFSET		0x30
#define CONTROL_REG_VALUE_OFFSET	0x32

struct system_power_state target_state;

static u32 *scratchpad_restore_addr;
static u32 restore_pointer_address;

static u32 mpu_ret_cnt;
static u32 mpu_off_cnt;
static u32 core_ret_cnt;
static u32 core_off_cnt;
struct omap3_processor_cx {
	u8 valid;
	u8 type;
	u32 sleep_latency;
	u32 wakeup_latency;
	u32 mpu_state;
	u32 core_state;
	u32 threshold;
	u32 flags;
};

#define LAST_IDLE_STATE_ARR_SIZE 10

struct idle_state {
	u32 mpu_state;
	u32 core_state;
	u32 per_state;
	u32 fclks;
	u32 iclks;
	u8 iva_state;
};

static struct idle_state last_idle_state[LAST_IDLE_STATE_ARR_SIZE];
u32 arr_wrptr;

struct omap3_processor_cx omap3_power_states[OMAP3_MAX_STATES];

struct omap3_processor_cx current_cx_state;

static void omap_init_power_states(void);
static void (*saved_idle)(void);
static inline unsigned long omap_32k_ticks_to_usecs(unsigned long ticks_32k)
{
	return (ticks_32k * 5*5*5*5*5*5) >> 9;
}

#ifdef CONFIG_OMAP3430_ES2
u32 current_vdd1_opp = PRCM_VDD1_OPP3;
u32 current_vdd2_opp = PRCM_VDD2_OPP3;
EXPORT_SYMBOL(current_vdd1_opp);
EXPORT_SYMBOL(current_vdd2_opp);
EXPORT_SYMBOL(power_subsys);
#else
u32 current_vdd1_opp = PRCM_VDD1_OPP1;
u32 current_vdd2_opp = PRCM_VDD2_OPP1;
#endif

struct clk *p_vdd1_clk;
struct clk *p_vdd2_clk;

struct sram_mem {
	u32 i[32000];
};
struct sram_mem *sdram_mem;

u32 vdd1_opp_no;
u32 vdd2_opp_no;
static unsigned long awake_time_end;

#ifdef CONFIG_ENABLE_VOLTSCALE_IN_SUSPEND
static u32 valid_rate;
#endif

struct mpu_core_vdd_state target_sleep_state;

/*
 * Functions to save & restore core context.
 */
extern void prcm_save_core_context(u32 target_core_state);
extern void prcm_restore_core_context(u32 target_core_state);

extern void omap2_gp_timer_program_next_event(unsigned long cycles);


/* For dynamically disabling cpuidle governor and switching
 * back to default OS idle hadler */
static int cpuidle_running = 1;

/**
 * store_prepwst - Store the previous power state
 */
static void store_prepwst(void)
{
	last_idle_state[arr_wrptr].mpu_state  = PM_PREPWSTST_MPU;
	last_idle_state[arr_wrptr].core_state = PM_PREPWSTST_CORE;
	last_idle_state[arr_wrptr].per_state = PM_PREPWSTST_PER;

	if ((PM_PREPWSTST_MPU & 0x3) == 0x1)
		mpu_ret_cnt++;
	if ((PM_PREPWSTST_MPU & 0x3) == 0x0)
		mpu_off_cnt++;
	if ((PM_PREPWSTST_CORE & 0x3) == 0x1)
		core_ret_cnt++;
	if ((PM_PREPWSTST_CORE & 0x3) == 0x0)
		core_off_cnt++;

	last_idle_state[arr_wrptr].fclks =
		(  (CM_FCLKEN1_CORE & CORE_FCLK_MASK)
#ifdef CONFIG_OMAP3430_ES2
#ifdef CONFIG_ARCH_HAS_OMAP3_SGX
		 | (CM_FCLKEN_SGX      & SGX_FCLK_MASK)
		 | (CM_FCLKEN_DSS      & DSS_FCLK_MASK)
#else
		 | (CM_FCLKEN_DSS      & DSS_FCLK_MASK)
#endif
		 | (CM_FCLKEN_USBHOST  & USBHOST_FCLK_MASK)
		 | (CM_FCLKEN3_CORE    & CORE3_FCLK_MASK)
#else
		 | (CM_FCLKEN_GFX      & GFX_FCLK_MASK)
		 | (CM_FCLKEN_DSS      & DSS_FCLK_MASK)
#endif
		 | (CM_FCLKEN_CAM      & CAM_FCLK_MASK)
		 | (CM_FCLKEN_PER      & PER_FCLK_MASK));

	last_idle_state[arr_wrptr].iclks =
		(  (CORE1_ICLK_VALID & (CM_ICLKEN1_CORE & ~CM_AUTOIDLE1_CORE))
		 | (CORE2_ICLK_VALID & (CM_ICLKEN2_CORE & ~CM_AUTOIDLE2_CORE))
#ifdef CONFIG_OMAP3430_ES2
		 | (CORE3_ICLK_VALID   & (CM_ICLKEN3_CORE & ~CM_AUTOIDLE3_CORE))
#ifdef CONFIG_ARCH_HAS_OMAP3_SGX
		 | (SGX_ICLK_VALID     & (CM_ICLKEN_SGX))
#endif
		 | (USBHOST_ICLK_VALID & (CM_ICLKEN_USBHOST))
#else
		 | (GFX_ICLK_VALID     & (CM_ICLKEN_GFX))
#endif
		 | (DSS_ICLK_VALID     & (CM_ICLKEN_DSS & ~CM_AUTOIDLE_DSS))
		 | (CAM_ICLK_VALID     & (CM_ICLKEN_CAM & ~CM_AUTOIDLE_CAM))
		 | (PER_ICLK_VALID     & (CM_ICLKEN_PER & ~CM_AUTOIDLE_PER))
		 | (WKUP_ICLK_VALID    & (CM_ICLKEN_WKUP & ~CM_AUTOIDLE_WKUP)));

	prcm_get_power_domain_state(DOM_IVA2,
				&(last_idle_state[arr_wrptr].iva_state));

	arr_wrptr++;

	if (arr_wrptr == LAST_IDLE_STATE_ARR_SIZE)
		arr_wrptr = 0;
}


static void restore_control_register(u32 val)
{
	__asm__ __volatile__ ("mcr p15, 0, %0, c1, c0, 0" : : "r" (val));
}

/**
 * restore_table_entry - restore table entry that was modified for enabling MMU
 */
static void restore_table_entry(void)
{
	u32 *scratchpad_address;
	u32 previous_value, control_reg_value;
	u32 *address;

	/* Get virtual address of SCRATCHPAD */
	scratchpad_address = (u32 *) io_p2v(SCRATCHPAD);
	/* Get address of entry that was modified */
	address = (u32 *) *(scratchpad_address + TABLE_ADDRESS_OFFSET);
	/* Get the previous value which needs to be restored */
	previous_value = *(scratchpad_address + TABLE_VALUE_OFFSET);
#ifdef RESTORE_TABLE_ENTRY
	/* Convert address to virtual address */
	address = __va(address);
	/* Restore table entry */
	*address = previous_value;
	/* Flush TLB */
	flush_tlb_all();
#endif
	control_reg_value = *(scratchpad_address + CONTROL_REG_VALUE_OFFSET);
	/* Restore control register*/
	/* This will enable caches and prediction */
	restore_control_register(control_reg_value);
}

static void (*_omap_sram_idle)(u32 *addr, int save_state);
static void (*_omap_save_secure_sram)(u32 *addr);

void omap_sram_idle(void)
{
	/* Variable to tell what needs to be saved and restored
	 * in omap_sram_idle*/
	/* save_state = 0 => Nothing to save and restored */
	/* save_state = 1 => Only L1 and logic lost */
	/* save_state = 2 => Only L2 lost */
	/* save_state = 3 => L1, L2 and logic lost */
	int save_state = 0;

	if (!_omap_sram_idle)
		return;

	switch (target_state.mpu_state) {
	case PRCM_MPU_ACTIVE:
	case PRCM_MPU_INACTIVE:
	case PRCM_MPU_CSWR_L2RET:
		/* No need to save context */
		save_state = 0;
		break;
	case PRCM_MPU_OSWR_L2RET:
		/* L1 and Logic lost */
		save_state = 1;
		break;
	case PRCM_MPU_CSWR_L2OFF:
		/* Only L2 lost */
		save_state = 2;
		break;
	case PRCM_MPU_OSWR_L2OFF:
	case PRCM_MPU_OFF:
		/* L1, L2 and logic lost */
		save_state = 3;
		break;
	default:
		/* Invalid state */
		printk(KERN_ERR "Invalid mpu state in sram_idle\n");
		return;
	}

	_omap_sram_idle(context_mem, save_state);

	/* Restore table entry modified during MMU restoration */
	if ((PM_PREPWSTST_MPU & 0x3) == 0x0)
		restore_table_entry();
}

static int pre_uart_inactivity(void)
{
	int driver8250_managed = 0;

	if (are_driver8250_uarts_active(&driver8250_managed)) {
		awake_time_end = jiffies + msecs_to_jiffies(UART_TIME_OUT);
		return 0;
	}

	if (time_before(jiffies, awake_time_end)) {
		return 0;
	}
	return 1;
}

static void post_uart_inactivity(void)
{
	if (awake_time_end == 0)
		awake_time_end = jiffies;
}

static int omap3_pm_prepare(suspend_state_t state)
{
	int error = 0;

	/* We cannot sleep in idle until we have resumed */
	saved_idle = pm_idle;
	pm_idle = NULL;

	switch (state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		break;

	/*TODO: Need to fix this*/
	/*case PM_SUSPEND_DISK:
		return -ENOTSUPP;*/

	default:
		return -EINVAL;
	}
#ifdef CONFIG_ENABLE_VOLTSCALE_IN_SUSPEND
#ifdef CONFIG_OMAP3430_ES2
	if (current_vdd1_opp != PRCM_VDD1_OPP1) {
		valid_rate = clk_round_rate(p_vdd1_clk, S125M);
		p_vdd1_clk->set_rate(p_vdd1_clk, valid_rate);
		prcm_do_voltage_scaling(PRCM_VDD1_OPP1, current_vdd1_opp);
	}
	if (current_vdd2_opp != PRCM_VDD2_OPP2) {
		valid_rate = clk_round_rate(p_vdd2_clk, S83M);
		p_vdd1_clk->set_rate(p_vdd2_clk, valid_rate);
		prcm_do_voltage_scaling(PRCM_VDD2_OPP2, current_vdd2_opp);
	}
#else
	/* Scale to lowest OPP for VDD1 before executing suspend */
	if (current_vdd1_opp != PRCM_VDD1_OPP4) {
		valid_rate = clk_round_rate(p_vdd1_clk, S96M);
		p_vdd1_clk->set_rate(p_vdd1_clk, valid_rate);
		prcm_do_voltage_scaling(PRCM_VDD1_OPP4, current_vdd1_opp);
	}
	if (current_vdd2_opp != PRCM_VDD2_OPP2) {
		valid_rate = clk_round_rate(p_vdd2_clk, S83M);
		p_vdd1_clk->set_rate(p_vdd2_clk, valid_rate);
		prcm_do_voltage_scaling(PRCM_VDD2_OPP2, current_vdd2_opp);
	}
#endif /* CONFIG_OMAP3430_ES2 */
#endif /* CONFIG_ENABLE_VOLTSCALE_IN_SUSPEND */
	return error;
}

#ifndef CONFIG_ARCH_OMAP3410
/* Save and retore global configuration in NEON domain */
static void omap3_save_neon_context(void)
{
	/* Nothing to do here */
	return;
}

static void omap3_restore_neon_context(void)
{
#ifdef CONFIG_VFP
	vfp_enable();
#endif
}
#endif

/**
 * omap3_save_per_context - Save global configuration in PER domain
 *
 */
static void omap3_save_per_context(void)
{
	omap_gpio_save();
	omap_uart_save_ctx(2);
}

/**
 * omap3_restore_per_context - Restore global configuration in PER domain
 *
 */
static void omap3_restore_per_context(void)
{
	omap_gpio_restore();
	omap_uart_restore_ctx(2);
}

void omap3_save_secure_ram_context(u32 target_core_state)
{
	if ((target_core_state  > PRCM_CORE_CSWR_MEMRET) &&
		(target_core_state != PRCM_CORE_OSWR_MEMRET)) {
			if (!is_device_type_gp()) {
				/* Disable dma irq before calling
				* secure rom code API
				*/
				omap_dma_disable_irq(0);
				omap_dma_disable_irq(1);
				_omap_save_secure_sram((u32 *)__pa(sdram_mem));
			}
		}
}

void omap3_push_sram_functions(void)
{
	_omap_sram_idle = omap_sram_push(omap34xx_suspend,
				omap34xx_suspend_sz);
	if (!is_device_type_gp())
		_omap_save_secure_sram = omap_sram_push(save_secure_ram_context,
			save_secure_ram_context_sz);
}

/* Configuration that is OS specific is done here */
static void omap3_restore_core_settings(void)
{
	prcm_lock_iva_dpll(current_vdd1_opp);
}

static void memory_logic_res_seting(void)
{
	res1_level = resource_get_level(memret1);
	res2_level = resource_get_level(memret2);
	res3_level = resource_get_level(logret1);
	if (res3_level == LOGIC_RET) {
		if ((res1_level == MEMORY_RET) && (res2_level == MEMORY_RET))
			target_state.core_state = PRCM_CORE_CSWR_MEMRET;
		else if (res1_level == MEMORY_OFF && res2_level == MEMORY_RET)
			target_state.core_state = PRCM_CORE_CSWR_MEM1OFF;
		else if (res1_level == MEMORY_RET &&  res2_level == MEMORY_OFF)
			target_state.core_state = PRCM_CORE_CSWR_MEM2OFF;
		else
			target_state.core_state = PRCM_CORE_CSWR_MEMOFF;
	} else if (res3_level == LOGIC_OFF) {
		if ((res1_level && res2_level) == MEMORY_RET)
			target_state.core_state = PRCM_CORE_OSWR_MEMRET;
		else if (res1_level == MEMORY_OFF && res2_level == MEMORY_RET)
			target_state.core_state = PRCM_CORE_OSWR_MEM1OFF;
		else if (res1_level == MEMORY_RET &&  res2_level == MEMORY_OFF)
			target_state.core_state = PRCM_CORE_OSWR_MEM2OFF;
		else
			target_state.core_state = PRCM_CORE_OSWR_MEMOFF;
	} else
		DPRINTK("Current State not supported in Retention");

}

/**
 * omap3_pm_suspend
 */
static int omap3_pm_suspend(void)
{
	int ret;

#ifdef CONFIG_OMAP3430_ES2
	srcore_disable_smartreflex(SR1_ID);
	srcore_disable_smartreflex(SR2_ID);
#endif /* #ifdef CONFIG_OMAP3430_ES2 */

	local_irq_disable();
	local_fiq_disable();

#ifdef CONFIG_CORE_OFF
	if (enable_off){
		omap_uart_save_ctx(0);
		omap_uart_save_ctx(1);
	}
#endif
#ifdef CONFIG_MPU_OFF
	/* On ES 2.0, if scrathpad is populated with valid
	* pointer, warm reset does not work
	* So populate scrathpad restore address only in
	* cpuidle and suspend calls
	*/
	*(scratchpad_restore_addr) = restore_pointer_address;
#endif

#ifdef CONFIG_OMAP34XX_OFFMODE
	context_restore_update(DOM_PER);
	context_restore_update(DOM_CORE1);

	target_state.iva2_state		= PRCM_OFF;
#ifdef CONFIG_ARCH_HAS_OMAP3_SGX
	target_state.gfx_state		= PRCM_OFF;
#endif
	target_state.dss_state		= PRCM_OFF;
	target_state.cam_state		= PRCM_OFF;
	target_state.per_state		= PRCM_OFF;
#ifdef CONFIG_OMAP3430_ES2
	target_state.usbhost_state	= PRCM_OFF;
#ifndef CONFIG_ARCH_OMAP3410
	target_state.neon_state		= PRCM_OFF;
#endif
#else
	/* Errata 1.46: Neon cannot be put to OFF on ES1.0 */
	target_state.neon_state = PRCM_RET;
#endif /* ifdef CONFIG_OMAP3430_ES2 */

#else	/* ifdef CONFIG_OMAP34XX_OFFMODE */
	target_state.iva2_state = PRCM_RET;
#ifdef CONFIG_ARCH_HAS_OMAP3_SGX
	target_state.gfx_state  = PRCM_RET;
#endif
	target_state.dss_state  = PRCM_RET;
	target_state.cam_state  = PRCM_RET;
	target_state.per_state  = PRCM_RET;
#ifndef CONFIG_ARCH_OMAP3410
	target_state.neon_state = PRCM_RET;
#endif
#ifdef CONFIG_OMAP3430_ES2
	target_state.usbhost_state = PRCM_RET;
#endif
#endif	/* ifdef CONFIG_OMAP34XX_OFFMODE */

#ifdef CONFIG_MPU_OFF
	if (enable_off)
		target_state.mpu_state = PRCM_MPU_OFF;
	else
		target_state.mpu_state = PRCM_MPU_CSWR_L2RET;
#else
	target_state.mpu_state = PRCM_MPU_CSWR_L2RET;
#endif
#ifndef CONFIG_ARCH_OMAP3410
	if (target_state.neon_state == PRCM_OFF)
		omap3_save_neon_context();
#endif
	if (target_state.per_state == PRCM_OFF)
		omap3_save_per_context();

#ifdef CONFIG_CORE_OFF
	if (enable_off)
		target_state.core_state = PRCM_CORE_OFF;
	else
		target_state.core_state = PRCM_CORE_CSWR_MEMRET;
#else
	target_state.core_state = PRCM_CORE_CSWR_MEMRET;
#endif	/* ifdef CONFIG_CORE_OFF */
	if (target_state.core_state == PRCM_CORE_CSWR_MEMRET) {
		memory_logic_res_seting();
	}
	if (target_state.core_state >=  PRCM_CORE_OSWR_MEMRET) {
		prcm_save_core_context(target_state.core_state);
		omap_uart_save_ctx(0);
		omap_uart_save_ctx(1);	
	}

#ifdef CONFIG_MACH_ARCHOS_G6
	ret = prcm_set_chip_power_mode(&target_state, 0);
#else
	ret = prcm_set_chip_power_mode(&target_state,
					PRCM_WAKEUP_T2_KEYPAD |
					PRCM_WAKEUP_TOUCHSCREEN);
#endif

	printk ("PM_PWSTST_MPU = %X PM_PREPWSTST_MPU = %X\n", PM_PWSTST_MPU, PM_PREPWSTST_MPU);
	printk ("PM_PWSTST_CORE = %X PM_PREPWSTST_CORE = %X\n", PM_PWSTST_CORE, PM_PREPWSTST_CORE);
	printk ("PM_PWSTST_IVA2 = %X PM_PREPWSTST_IVA2 = %X\n", PM_PWSTST_IVA2, PM_PREPWSTST_IVA2);
	printk ("PM_PWSTST_NEON = %X PM_PREPWSTST_NEON = %X\n", PM_PWSTST_NEON, PM_PREPWSTST_NEON);
	printk ("PM_PWSTST_PER = %X PM_PREPWSTST_PER = %X\n", PM_PWSTST_PER, PM_PREPWSTST_PER);
	printk ("PM_PWSTST_SGX = %X PM_PREPWSTST_SGX = %X\n", PM_PWSTST_SGX, PM_PREPWSTST_SGX);
	printk ("PM_PWSTST_USBHOST = %X PM_PREPWSTST_USBHOST = %X\n", PM_PWSTST_USBHOST, PM_PREPWSTST_USBHOST);
	printk ("PM_PWSTST_DSS = %X PM_PREPWSTST_DSS = %X\n", PM_PWSTST_DSS, PM_PREPWSTST_DSS);
	printk ("PM_PWSTST_CAM = %X PM_PREPWSTST_CAM = %X\n", PM_PWSTST_CAM, PM_PREPWSTST_CAM);

#ifdef CONFIG_MPU_OFF
	*(scratchpad_restore_addr) = 0;
#endif
#ifndef CONFIG_ARCH_OMAP3410
	if (target_state.neon_state == PRCM_OFF)
		omap3_restore_neon_context();
#endif
	if (target_state.per_state == PRCM_OFF)
		omap3_restore_per_context();

#ifdef CONFIG_CORE_OFF
	if (enable_off) {
		omap3_restore_core_settings();
		omap_uart_restore_ctx(0);
		omap_uart_restore_ctx(1);	
		if ((target_state.core_state > PRCM_CORE_CSWR_MEMRET) &&
			(target_state.core_state != PRCM_CORE_OSWR_MEMRET)) {
			restore_sram_functions();
			omap3_push_sram_functions();		
		}
	}
#else
	if (target_state.core_state >= PRCM_CORE_OSWR_MEMRET) {
#ifdef CONFIG_OMAP34XX_OFFMODE
		context_restore_update(DOM_CORE1);
#endif
		prcm_restore_core_context(target_state.core_state);
		omap_uart_restore_ctx(0);
		omap_uart_restore_ctx(1);	
	}
#endif
	local_fiq_enable();
	local_irq_enable();
	if (ret != 0) {
		printk(KERN_ERR "pm_suspend: Unable to set target state\n");
	}
	else {
		printk(KERN_INFO "pm_suspend: Successfully set target state\n");
#ifdef CONFIG_MPU_OFF
#ifdef CONFIG_CORE_OFF
		if (enable_off)
			printk(KERN_INFO "Target MPU state: OFF, "
					"Target CORE state: OFF\n");
		else
			printk(KERN_INFO "Target MPU state: CSWR_L2RET, "
					"Target CORE state: CSWR_MEMRET\n");
#else
		printk(KERN_INFO "Target MPU state: OFF, "
				"Target CORE state: CSWR_MEMRET\n");
#endif
#else
		printk(KERN_INFO "Target MPU state: CSWR_L2RET, "
				"Target CORE state: CSWR_MEMRET\n");
#endif
	}
#if defined (CONFIG_OMAP3430_ES2)
	srcore_enable_smartreflex(SR1_ID);
	srcore_enable_smartreflex(SR2_ID);
#endif /* #ifdef CONFIG_OMAP3430_ES2 */

	return ret;
}

static int omap3_pm_enter(suspend_state_t state)
{
	int ret = 0;

	switch (state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		ret = omap3_pm_suspend();
		if (ret)
			ret = -EBUSY;
		break;
	/*
	case PM_SUSPEND_DISK:
		ret = -ENOTSUPP;
		break;
	*/
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int omap3_pm_valid(suspend_state_t state)
{
	int ret = 0;

	switch (state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		ret = 1;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int omap3_pm_finish(suspend_state_t state)
{
#ifdef CONFIG_ENABLE_VOLTSCALE_IN_SUSPEND
#ifdef CONFIG_OMAP3430_ES2
	/* Scale back to previous OPP for VDD2*/
	if (current_vdd2_opp != PRCM_VDD2_OPP2) {
		prcm_do_voltage_scaling(PRCM_VDD2_OPP3, PRCM_VDD2_OPP2);
		valid_rate = clk_round_rate(p_vdd2_clk, S166M);
		p_vdd2_clk->set_rate(p_vdd2_clk, valid_rate);
	}
	/* Scale back to previous OPP for VDD1 */
	prcm_do_voltage_scaling(current_vdd1_opp, PRCM_VDD1_OPP1);
	if (current_vdd1_opp != PRCM_VDD1_OPP1) {
		switch (current_vdd1_opp) {
		case PRCM_VDD1_OPP2:
			valid_rate = clk_round_rate(p_vdd1_clk, S250M);
			break;
		case PRCM_VDD1_OPP3:
			valid_rate = clk_round_rate(p_vdd1_clk, S500M);
			break;
		case PRCM_VDD1_OPP4:
			valid_rate = clk_round_rate(p_vdd1_clk, S550M);
			break;
		case PRCM_VDD1_OPP5:
			valid_rate = clk_round_rate(p_vdd1_clk, S600M);
			break;
		}
		p_vdd1_clk->set_rate(p_vdd1_clk, valid_rate);
	}
#else
	/* Scale back to previous OPP for VDD2*/
	if (current_vdd2_opp != PRCM_VDD2_OPP2) {
		prcm_do_voltage_scaling(PRCM_VDD2_OPP1, PRCM_VDD2_OPP2);
		valid_rate = clk_round_rate(p_vdd2_clk, S166M);
		p_vdd2_clk->set_rate(p_vdd2_clk, valid_rate);
	}

	/* Scale back to previous OPP for VDD1 */
	prcm_do_voltage_scaling(current_vdd1_opp, PRCM_VDD1_OPP4);
	if (current_vdd1_opp != PRCM_VDD1_OPP4) {
		switch (current_vdd1_opp) {
		case PRCM_VDD1_OPP1:
			valid_rate = clk_round_rate(p_vdd1_clk, S477M);
			break;
		case PRCM_VDD1_OPP2:
			valid_rate = clk_round_rate(p_vdd1_clk, S381M);
			break;
		case PRCM_VDD1_OPP3:
			valid_rate = clk_round_rate(p_vdd1_clk, S191M);
			break;
		}
		p_vdd1_clk->set_rate(p_vdd1_clk, valid_rate);
	}
#endif /* CONFIG_OMAP3430_ES2 */
#endif /* CONFIG_ENABLE_VOLTSCALE_IN_SUSPEND */
	pm_idle = saved_idle;
	return 0;
}

static struct pm_ops omap_pm_ops = {
	.valid		= omap3_pm_valid,
	.prepare	= omap3_pm_prepare,
	.enter		= omap3_pm_enter,
	.finish		= omap3_pm_finish,
};

#ifdef CONFIG_CPU_IDLE

static int omap3_idle_bm_check(void)
{
	u32 core_dep;
	u8 state;
	/* Check if any modules other than debug uart and gpios are active*/
#ifdef CONFIG_OMAP3430_ES2
	core_dep = (  (CM_FCLKEN1_CORE & CORE_FCLK_MASK)
#ifdef CONFIG_ARCH_HAS_OMAP3_SGX
		    | (CM_FCLKEN_SGX & SGX_FCLK_MASK)
#endif
		    | (CM_FCLKEN_CAM & CAM_FCLK_MASK)
		    | (CM_FCLKEN_PER & PER_FCLK_MASK)
		    | (CM_FCLKEN_USBHOST & USBHOST_FCLK_MASK)
		    | (CM_FCLKEN3_CORE & CORE3_FCLK_MASK));

	/* To allow core retention during LPR scenario */
	if (!lpr_enabled)
		core_dep |= (CM_FCLKEN_DSS & DSS_FCLK_MASK);
#if 0
	DPRINTK("FCLKS: %x,%x,%x,%x,%x,%x,%x\n",
		(CM_FCLKEN1_CORE & CORE_FCLK_MASK),
		(CM_FCLKEN3_CORE & CORE3_FCLK_MASK),
		(CM_FCLKEN_SGX & SGX_FCLK_MASK),
		(CM_FCLKEN_DSS & DSS_FCLK_MASK),
		(CM_FCLKEN_CAM & CAM_FCLK_MASK),
		(CM_FCLKEN_PER & PER_FCLK_MASK),
		(CM_FCLKEN_USBHOST & USBHOST_FCLK_MASK));
#endif
#else
	core_dep = (  (CM_FCLKEN1_CORE & CORE_FCLK_MASK)
		    | (CM_FCLKEN_GFX & GFX_FCLK_MASK)
		    | (CM_FCLKEN_DSS & DSS_FCLK_MASK)
		    | (CM_FCLKEN_CAM & CAM_FCLK_MASK)
		    | (CM_FCLKEN_PER & PER_FCLK_MASK);
#if 0
	DPRINTK("FCLKS: %x,%x,%x,%x,%x\n",
		(CM_FCLKEN1_CORE & CORE_FCLK_MASK),
		(CM_FCLKEN_GFX & GFX_FCLK_MASK),
		(CM_FCLKEN_DSS & DSS_FCLK_MASK),
		(CM_FCLKEN_CAM & CAM_FCLK_MASK),
		(CM_FCLKEN_PER & PER_FCLK_MASK));
#endif
#endif

	/* Check if any modules have ICLK bit enabled and interface clock */
	/* autoidle disabled						 */
	core_dep |= (CORE1_ICLK_VALID & (CM_ICLKEN1_CORE & ~CM_AUTOIDLE1_CORE));
	/* Check for secure modules which have only ICLK */
	/* Do not check for rng module.It has been ensured that
	 * if rng is active cpu idle will never be entered
	 */
	core_dep |= (CORE2_ICLK_VALID & CM_ICLKEN2_CORE & ~4);
	/* Enabling SGX ICLK will prevent CORE ret*/
#ifdef CONFIG_OMAP3430_ES2
#ifdef CONFIG_ARCH_HAS_OMAP3_SGX
	core_dep |= SGX_ICLK_VALID & (CM_ICLKEN_SGX);
#endif
	core_dep |= (CORE3_ICLK_VALID & (CM_ICLKEN3_CORE & ~CM_AUTOIDLE3_CORE));
	core_dep |= (USBHOST_ICLK_VALID & (CM_ICLKEN_USBHOST &
			~CM_AUTOIDLE_USBHOST));
#else
	core_dep |= (GFX_ICLK_VALID & (CM_ICLKEN_GFX));
#endif
	core_dep |= (DSS_ICLK_VALID & (CM_ICLKEN_DSS & ~CM_AUTOIDLE_DSS));
	core_dep |= (CAM_ICLK_VALID & (CM_ICLKEN_CAM & ~CM_AUTOIDLE_CAM));
	core_dep |= (PER_ICLK_VALID & (CM_ICLKEN_PER & ~CM_AUTOIDLE_PER));
	core_dep |= (WKUP_ICLK_VALID & (CM_ICLKEN_WKUP & ~CM_AUTOIDLE_WKUP));
#if 0
	DPRINTK("ICLKS: %x,%x,%x,%x,%x,%x,%x,%x,%x\n",
		(CORE1_ICLK_VALID & (CM_ICLKEN1_CORE & ~CM_AUTOIDLE1_CORE)),
		(CORE2_ICLK_VALID & (CM_ICLKEN2_CORE & ~CM_AUTOIDLE2_CORE)),
#ifdef CONFIG_OMAP3430_ES2
		(CORE3_ICLK_VALID & (CM_ICLKEN3_CORE & ~CM_AUTOIDLE3_CORE)),
#ifdef CONFIG_ARCH_HAS_OMAP3_SGX
		(SGX_ICLK_VALID & (CM_ICLKEN_SGX)),
#endif
		(USBHOST_ICLK_VALID & (CM_ICLKEN_USBHOST &
			~CM_AUTOIDLE_USBHOST)),
#else
		(GFX_ICLK_VALID & (CM_ICLKEN_GFX)),
#endif
		(DSS_ICLK_VALID & (CM_ICLKEN_DSS & ~CM_AUTOIDLE_DSS)),
		(CAM_ICLK_VALID & (CM_ICLKEN_CAM & ~CM_AUTOIDLE_CAM)),
		(PER_ICLK_VALID & (CM_ICLKEN_PER & ~CM_AUTOIDLE_PER)),
		(WKUP_ICLK_VALID & (CM_ICLKEN_WKUP & ~CM_AUTOIDLE_WKUP)));
#endif
	prcm_get_power_domain_state(DOM_IVA2, &state);
#if 0
	/*DPRINTK("IVA:%d\n", state);*/
#endif
	if (state == PRCM_ON)
		core_dep |= 1;

	/* Check if a DMA transfer is active */
	if(omap_dma_running())
		core_dep |= 1;
	/* Check if debug UART is active */
	if(!pre_uart_inactivity())
		core_dep |= 1;	

	if (core_dep)
		return 1;
	else
		return 0;
}

/* Correct target state based on inactivity timer expiry, etc */
static void correct_target_state(void)
{
	switch (target_state.mpu_state) {
	case PRCM_MPU_ACTIVE:
	case PRCM_MPU_INACTIVE:
#ifndef CONFIG_ARCH_OMAP3410
		target_state.neon_state = PRCM_ON;
#endif
		break;
	case PRCM_MPU_CSWR_L2RET:
	case PRCM_MPU_OSWR_L2RET:
	case PRCM_MPU_CSWR_L2OFF:
	case PRCM_MPU_OSWR_L2OFF:
#ifndef CONFIG_ARCH_OMAP3410
		target_state.neon_state = PRCM_RET;
#endif
		break;
	case PRCM_MPU_OFF:
#ifndef CONFIG_ARCH_OMAP3410
		target_state.neon_state = PRCM_OFF;
#endif
		if (!enable_off) {
			target_state.mpu_state = PRCM_MPU_CSWR_L2RET;
#ifndef CONFIG_ARCH_OMAP3410
			target_state.neon_state = PRCM_RET;
#endif
		}
		break;
	}
	if (target_state.core_state > PRCM_CORE_INACTIVE) {
#ifdef CONFIG_OMAP34XX_OFFMODE
		/* Core can be put to RET/OFF - This means
		 * PER can be put to off if its inactivity timer
		 * has expired
		 */
		if (perdomain_timer_pending())
			target_state.per_state = PRCM_RET;
		else
			target_state.per_state = PRCM_OFF;

		if (target_state.core_state == PRCM_CORE_OFF) {
			if (coredomain_timer_pending())
				target_state.core_state = PRCM_CORE_CSWR_MEMRET;
			if (CM_FCLKEN_DSS & DSS_FCLK_MASK)
				target_state.core_state = PRCM_CORE_CSWR_MEMRET;
			if (!enable_off) {
				target_state.core_state = PRCM_CORE_CSWR_MEMRET;
				target_state.per_state = PRCM_RET;
			}
		}
#else
		target_state.per_state = PRCM_RET;
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */
	} else
		target_state.per_state = PRCM_ON;
		if (target_state.core_state == PRCM_CORE_CSWR_MEMRET)
			memory_logic_res_seting();

}

static int omap3_enter_idle(struct cpuidle_device *dev,
			struct cpuidle_state *state)
{
	struct omap3_processor_cx *cx;
#ifndef CONFIG_ARCH_OMAP3410
	u8 cur_per_state, cur_neon_state, pre_neon_state, pre_per_state;
#else
	u8 cur_per_state, pre_per_state;
#endif
	unsigned long timer_32k_sleep_ticks;
	u32 fclken_core, iclken_core, fclken_per, iclken_per;
	int wakeup_latency;
	int core_sleep_flg = 0;
#ifdef CONFIG_HW_SUP_TRANS
	u32 sleepdep_per, wakedep_per;
#endif /* #ifdef CONFIG_HW_SUP_TRANS */

	local_irq_disable();
	local_fiq_disable();

	/* Reset previous power state registers for mpu and core*/
	PM_PREPWSTST_MPU = 0xFF;
	PM_PREPWSTST_CORE = 0xFF;
#ifndef CONFIG_ARCH_OMAP3410
	PM_PREPWSTST_NEON = 0xFF;
#endif
	PM_PREPWSTST_PER = 0xFF;

	cx = cpuidle_get_statedata(state);
	current_cx_state = *cx;

	target_state.mpu_state = cx->mpu_state;
	target_state.core_state = cx->core_state;

	/* take a time marker for residency */
	timer_32k_sleep_ticks = omap_32k_sync_timer_read();

	if (cx->type == OMAP3_STATE_C0) {
		omap_sram_idle();
		goto return_sleep_time;
	}

 	/* side effect! correct_target_state() modifies global target_state variable! */
	correct_target_state();

	wakeup_latency = cx->wakeup_latency;
	if (target_state.core_state != cx->core_state) {
		/* Currently, this can happen only for core_off */
		/* Adjust wakeup latency to that of core_cswr state */
		/* Hard coded now and needs to be made more generic */
		/* omap3_power_states[4] is CSWR for core */
		wakeup_latency = omap3_power_states[4].wakeup_latency;
	}

	/* Reprogram next wake up tick to adjust for wake latency */
	if(cx->wakeup_latency > 1000){
		struct tick_device *d = tick_get_device(smp_processor_id());
		ktime_t adjust, next, now = ktime_get();
		if (ktime_to_ns(ktime_sub(d->evtdev->next_event, now)) > 
			(cx->wakeup_latency * 1000 + NSEC_PER_MSEC)){	
			adjust = ktime_set(0, (cx->wakeup_latency * 1000));
			next = ktime_sub(d->evtdev->next_event, adjust);
			clockevents_program_event(d->evtdev, next, now);
		}
 	}

	/* Check for pending interrupts. If there is an interrupt, return */
	if (INTCPS_PENDING_IRQ0 | INTCPS_PENDING_IRQ1 | INTCPS_PENDING_IRQ2) {
#ifdef DEBUG_PREPWST		
		store_prepwst();
#endif		
		goto return_sleep_time;
	}

	prcm_get_power_domain_state(DOM_PER, &cur_per_state);
#ifndef CONFIG_ARCH_OMAP3410
	prcm_get_power_domain_state(DOM_NEON, &cur_neon_state);
#endif
	fclken_core = CM_FCLKEN1_CORE;
	iclken_core = CM_ICLKEN1_CORE;
	fclken_per = CM_FCLKEN_PER;
	iclken_per = CM_ICLKEN_PER;


#ifdef CONFIG_HW_SUP_TRANS
	/* Facilitating SWSUP RET, from HWSUP mode */
	sleepdep_per = CM_SLEEPDEP_PER;
	wakedep_per = PM_WKDEP_PER;
#endif /* #ifdef CONFIG_HW_SUP_TRANS */

	/* If target state if core_off, save registers
	 * before changing anything
	 */
	if (target_state.core_state >= PRCM_CORE_OSWR_MEMRET) {
		prcm_save_registers(&target_state);
		omap_uart_save_ctx(0);
		omap_uart_save_ctx(1);
	}

	/* Check for pending interrupts. If there is an interrupt, return */
	if (INTCPS_PENDING_IRQ0 | INTCPS_PENDING_IRQ1 | INTCPS_PENDING_IRQ2) {
#ifdef DEBUG_PREPWST		
		store_prepwst();
#endif		
		goto return_sleep_time;
	}

	/* Program MPU and NEON to target state */
	if (target_state.mpu_state > PRCM_MPU_ACTIVE) {
#ifndef CONFIG_ARCH_OMAP3410
		if ((cur_neon_state == PRCM_ON) &&
			(target_state.neon_state != PRCM_ON)) {
			if (target_state.neon_state == PRCM_OFF)
				omap3_save_neon_context();
#ifdef CONFIG_HW_SUP_TRANS
			/* Facilitating SWSUP RET, from HWSUP mode */
			prcm_set_clock_domain_state(DOM_NEON, PRCM_NO_AUTO,
							       PRCM_FALSE);
			prcm_set_power_domain_state(DOM_NEON, PRCM_ON,
								PRCM_FORCE);
#endif /* #ifdef CONFIG_HW_SUP_TRANS */
			prcm_force_power_domain_state(DOM_NEON,
				target_state.neon_state);
		}
#endif
#ifdef CONFIG_MPU_OFF
		/* Populate scratchpad restore address */
		*(scratchpad_restore_addr) = restore_pointer_address;
#endif
		omap3_save_secure_ram_context(target_state.core_state);
		prcm_set_mpu_domain_state(target_state.mpu_state);
	}

	/* Check for pending interrupts. If there is an interrupt, return */
	if (INTCPS_PENDING_IRQ0 | INTCPS_PENDING_IRQ1 | INTCPS_PENDING_IRQ2)
		goto restore;

	/* Program CORE and PER to target state */
	if (target_state.core_state > PRCM_CORE_ACTIVE) {

		/* Log core sleep attmept */
		core_sleep_flg = 1;

		if ((cur_per_state == PRCM_ON) &&
				(target_state.per_state != PRCM_ON)) {
			if (target_state.per_state == PRCM_OFF)
				omap3_save_per_context();
			CM_FCLKEN_PER = 0;
			CM_ICLKEN_PER = 0;
#ifdef CONFIG_HW_SUP_TRANS
			/* Facilitating SWSUP RET, from HWSUP mode */
			prcm_set_clock_domain_state(DOM_PER, PRCM_NO_AUTO,
								PRCM_FALSE);
			prcm_clear_sleep_dependency(DOM_PER,
							PRCM_SLEEPDEP_EN_MPU);
			prcm_clear_wkup_dependency(DOM_PER, PRCM_WKDEP_EN_MPU);

			prcm_set_power_domain_state(DOM_PER, PRCM_ON,
								PRCM_FORCE);
#endif /* #ifdef CONFIG_HW_SUP_TRANS */
			prcm_force_power_domain_state(DOM_PER,
							target_state.per_state);
		}

#ifdef CONFIG_OMAP3430_ES2
		srcore_disable_smartreflex(SR1_ID);
		srcore_disable_smartreflex(SR2_ID);
#endif /* #ifdef CONFIG_OMAP3430_ES2 */

		/* WORKAROUND FOR SILICON ERRATA 1.64*/
		if (is_sil_rev_equal_to(OMAP3430_REV_ES1_0)) {
			if (CM_CLKOUT_CTRL & 0x80)
				CM_CLKOUT_CTRL &= ~(0x80);
		}

		prcm_set_core_domain_state(target_state.core_state);
		/* Enable Autoidle for GPT1 explicitly - Errata 1.4 */
		CM_AUTOIDLE_WKUP |= 0x1;
		CM_FCLKEN1_CORE &= ~0x6000;
		/* Disable HSUSB OTG ICLK explicitly*/
		CM_ICLKEN1_CORE &= ~0x10;
#ifdef IOPAD_WKUP
		/* Enabling IO_PAD capabilities */
		PM_WKEN_WKUP |= 0x100;
#endif /* #ifdef IOPAD_WKUP */
	}

	/* Check for pending interrupts. If there is an interrupt, return */
	if (INTCPS_PENDING_IRQ0 | INTCPS_PENDING_IRQ1 | INTCPS_PENDING_IRQ2)
		goto restore;

	omap_sram_idle();

restore:
#ifdef IOPAD_WKUP
	/* Disabling IO_PAD capabilities */
	PM_WKEN_WKUP &= ~(0x100);
#endif /* #ifdef IOPAD_WKUP */

	CM_FCLKEN1_CORE = fclken_core;
	CM_ICLKEN1_CORE = iclken_core;

	if (target_state.mpu_state > PRCM_MPU_ACTIVE) {
#ifdef CONFIG_MPU_OFF
		/* On ES 2.0, if scratchpad is populated with valid
		* pointer, warm reset does not work
		* So populate scratchpad restore address only in
		* cpuidle and suspend calls
		*/
		*(scratchpad_restore_addr) = 0x0;
#endif
		prcm_set_mpu_domain_state(PRCM_MPU_ACTIVE);
#ifndef CONFIG_ARCH_OMAP3410
		if ((cur_neon_state == PRCM_ON) &&
			(target_state.mpu_state > PRCM_MPU_INACTIVE)) {
			prcm_force_power_domain_state(DOM_NEON, cur_neon_state);
			prcm_get_pre_power_domain_state(DOM_NEON,
					&pre_neon_state);
			if (pre_neon_state == PRCM_OFF)
				omap3_restore_neon_context();
#ifdef CONFIG_HW_SUP_TRANS
			prcm_set_power_domain_state(DOM_NEON, POWER_DOMAIN_ON,
								PRCM_AUTO);
#endif /* #ifdef CONFIG_HW_SUP_TRANS */
		}
#endif
	}

	/* Continue core restoration part, only if Core-Sleep is attempted */
	if ((target_state.core_state > PRCM_CORE_ACTIVE)
		&& core_sleep_flg ) {
		prcm_set_core_domain_state(PRCM_CORE_ACTIVE);
		post_uart_inactivity();
#if defined (CONFIG_OMAP3430_ES2)
		srcore_enable_smartreflex(SR1_ID);
		srcore_enable_smartreflex(SR2_ID);
#endif /* #ifdef CONFIG_OMAP3430_ES2 */

		if (cur_per_state == PRCM_ON) {
			prcm_force_power_domain_state(DOM_PER, cur_per_state);
			prcm_get_pre_power_domain_state(DOM_PER,
					&pre_per_state);
			CM_FCLKEN_PER = fclken_per;
			CM_ICLKEN_PER = iclken_per;
			if (pre_per_state == PRCM_OFF) {
				omap3_restore_per_context();
#ifdef CONFIG_OMAP34XX_OFFMODE
				context_restore_update(DOM_PER);
#endif
			}
#ifdef CONFIG_HW_SUP_TRANS
			/* Facilitating SWSUP RET, from HWSUP mode */
			CM_SLEEPDEP_PER = sleepdep_per;
			PM_WKDEP_PER = wakedep_per;
			prcm_set_power_domain_state(DOM_PER, PRCM_ON,
								PRCM_AUTO);
#endif /* #ifdef CONFIG_HW_SUP_TRANS */
		}
		if (target_state.core_state >= PRCM_CORE_OSWR_MEMRET) {
#ifdef CONFIG_OMAP34XX_OFFMODE
			context_restore_update(DOM_CORE1);
#endif
			prcm_restore_registers(&target_state);
			prcm_restore_core_context(target_state.core_state);
			omap3_restore_core_settings();
			omap_uart_restore_ctx(0);
			omap_uart_restore_ctx(1);
		}
		/* Errata 1.4
		* if the timer device gets idled which is when we
		* are cutting the timer ICLK which is when we try
		* to put Core to RET.
		* Wait Period = 2 timer interface clock cycles +
		* 1 timer functional clock cycle
		* Interface clock = L4 clock. For the computation L4
		* clock is assumed at 50MHz (worst case).
		* Functional clock = 32KHz
		* Wait Period = 2*10^-6/50 + 1/32768 = 0.000030557 = 30.557uSec
		* Roundingoff the delay value to a safer 50uSec
		*/
		omap_udelay(GPTIMER_WAIT_DELAY);
		CM_AUTOIDLE_WKUP &= ~(0x1);
		if ((target_state.core_state > PRCM_CORE_CSWR_MEMRET) &&
			(target_state.core_state != PRCM_CORE_OSWR_MEMRET)) {
			restore_sram_functions();
			omap3_push_sram_functions();
		}
	}

	DPRINTK("MPU state:%x,CORE state:%x\n", PM_PREPWSTST_MPU,
							PM_PREPWSTST_CORE);
#ifdef DEBUG_PREPWST	
	store_prepwst();
#endif	

/* function optimized/only good for timer32k, watch roll */
return_sleep_time:
	timer_32k_sleep_ticks = omap_32k_sync_timer_read() -
							timer_32k_sleep_ticks;

	local_irq_enable();
	local_fiq_enable();

	return omap_32k_ticks_to_usecs(timer_32k_sleep_ticks);
}

static int omap3_enter_idle_bm(struct cpuidle_device *dev,
				struct cpuidle_state *state)
{
	struct cpuidle_state *new_state = NULL;
	int i, j;

	if ((state->flags & CPUIDLE_FLAG_CHECK_BM) && omap3_idle_bm_check()) {

		/* Find current state in list */
		for (i = 0; i < OMAP3_MAX_STATES; i++)
			if (state == &dev->states[i])
				break;
		BUG_ON(i == OMAP3_MAX_STATES);

		/* Back up to non 'CHECK_BM' state */
		for (j = i - 1;  j > 0; j--) {
			struct cpuidle_state *s = &dev->states[j];

			if (!(s->flags & CPUIDLE_FLAG_CHECK_BM)) {
				new_state = s;
				break;
			}
		}

		pr_debug("%s: Bus activity: Entering %s (instead of %s)\n",
			__FUNCTION__, new_state->name, state->name);
	}

	return omap3_enter_idle(dev, new_state ? : state);
}

struct cpuidle_driver omap3_idle_driver = {
	.name = 	"omap3_idle",
	.owner = 	THIS_MODULE,
};

DEFINE_PER_CPU(struct cpuidle_device, omap3_idle_dev);

static int __init omap3_idle_init(void)
{
	int i, count = 0;
	struct omap3_processor_cx *cx;
	struct cpuidle_state *state;
	struct cpuidle_device *dev;

	omap_init_power_states();
	cpuidle_register_driver(&omap3_idle_driver);

	dev = &per_cpu(omap3_idle_dev, smp_processor_id());

	for (i = 0; i < OMAP3_MAX_STATES; i++) {
		cx = &omap3_power_states[i];
		state = &dev->states[count];

		if (!cx->valid)
			continue;
		cpuidle_set_statedata(state, cx);
		state->exit_latency = cx->sleep_latency + cx->wakeup_latency;
		state->target_residency = cx->threshold;
		state->flags = cx->flags;
		state->enter = (state->flags & CPUIDLE_FLAG_CHECK_BM) ?
			omap3_enter_idle_bm : omap3_enter_idle;
		sprintf(state->name, "C%d", count+1);
		count++;
	}

	if (!count)
		return -EINVAL;
	dev->state_count = count;
	if (cpuidle_register_device(dev)) {
		printk(KERN_ERR "%s: CPUidle register device failed\n",
			__FUNCTION__);
		return -EIO;
	}
	return 0;
}

#ifdef CONFIG_CPU_IDLE
__initcall(omap3_idle_init);
#endif

static void omap_init_power_states(void)
{
	omap3_power_states[0].valid		= 1;
	omap3_power_states[0].type		= OMAP3_STATE_C0;
	omap3_power_states[0].sleep_latency	= 0;
	omap3_power_states[0].wakeup_latency	= 0;
	omap3_power_states[0].threshold 	= 0;
	omap3_power_states[0].mpu_state		= PRCM_MPU_ACTIVE;
	omap3_power_states[0].core_state	= PRCM_CORE_ACTIVE;
	omap3_power_states[0].flags = CPUIDLE_FLAG_SHALLOW;

	omap3_power_states[1].valid		= 1;
	omap3_power_states[1].type		= OMAP3_STATE_C1;
	omap3_power_states[1].sleep_latency	= 10;
	omap3_power_states[1].wakeup_latency	= 10;
	omap3_power_states[1].threshold		= 30;
	omap3_power_states[1].mpu_state		= PRCM_MPU_ACTIVE;
	omap3_power_states[1].core_state	= PRCM_CORE_ACTIVE;
	omap3_power_states[1].flags		= CPUIDLE_FLAG_TIME_VALID |
							CPUIDLE_FLAG_SHALLOW;

	omap3_power_states[2].valid		= 1;
	omap3_power_states[2].type		= OMAP3_STATE_C2;
	omap3_power_states[2].sleep_latency	= 50;
	omap3_power_states[2].wakeup_latency	= 50;
	omap3_power_states[2].threshold		= 300;
	omap3_power_states[2].mpu_state		= PRCM_MPU_CSWR_L2RET;
	omap3_power_states[2].core_state	= PRCM_CORE_ACTIVE;
	omap3_power_states[2].flags		= CPUIDLE_FLAG_TIME_VALID |
							CPUIDLE_FLAG_BALANCED;

	omap3_power_states[3].valid		= 1;
	omap3_power_states[3].type		= OMAP3_STATE_C3;
	omap3_power_states[3].sleep_latency	= 1500;
	omap3_power_states[3].wakeup_latency	= 1800;
	omap3_power_states[3].threshold		= 4000;
#ifdef CONFIG_MPU_OFF
	omap3_power_states[3].mpu_state		= PRCM_MPU_OFF;
#else
	omap3_power_states[3].mpu_state		= PRCM_MPU_CSWR_L2RET;
#endif
	omap3_power_states[3].core_state	= PRCM_CORE_ACTIVE;
	omap3_power_states[3].flags		= CPUIDLE_FLAG_TIME_VALID |
							CPUIDLE_FLAG_BALANCED;

	omap3_power_states[4].valid		= 1;
	omap3_power_states[4].type		= OMAP3_STATE_C4;
	omap3_power_states[4].sleep_latency	= 2500;
	omap3_power_states[4].wakeup_latency	= 7500;
	omap3_power_states[4].threshold		= 12000;
	omap3_power_states[4].mpu_state		= PRCM_MPU_CSWR_L2RET;
	omap3_power_states[4].core_state	= PRCM_CORE_CSWR_MEMRET;
	omap3_power_states[4].flags		= CPUIDLE_FLAG_TIME_VALID |
							CPUIDLE_FLAG_BALANCED |
							CPUIDLE_FLAG_CHECK_BM;
	omap3_power_states[5].valid		= 1;
	omap3_power_states[5].type		= OMAP3_STATE_C5;
	omap3_power_states[5].sleep_latency	= 3000;
	omap3_power_states[5].wakeup_latency	= 8500;
	omap3_power_states[5].threshold		= 15000;
#ifdef CONFIG_MPU_OFF
	omap3_power_states[5].mpu_state		= PRCM_MPU_OFF;
#else
	omap3_power_states[5].mpu_state		= PRCM_MPU_CSWR_L2RET;
#endif
	omap3_power_states[5].core_state	= PRCM_CORE_CSWR_MEMRET;
	omap3_power_states[5].flags		= CPUIDLE_FLAG_TIME_VALID |
							CPUIDLE_FLAG_BALANCED |
							CPUIDLE_FLAG_CHECK_BM;

#ifdef CONFIG_CORE_OFF_CPUIDLE
	omap3_power_states[6].valid		= 1;
#else
	omap3_power_states[6].valid		= 0;
#endif
	omap3_power_states[6].type		= OMAP3_STATE_C6;
	omap3_power_states[6].sleep_latency	= 10000;
	omap3_power_states[6].wakeup_latency	= 30000;
	omap3_power_states[6].threshold		= 300000;
	omap3_power_states[6].mpu_state		= PRCM_MPU_OFF;
	omap3_power_states[6].core_state	= PRCM_CORE_OFF;
	omap3_power_states[6].flags		= CPUIDLE_FLAG_TIME_VALID |
							CPUIDLE_FLAG_DEEP |
							CPUIDLE_FLAG_CHECK_BM;
}

#endif

/* Sysfs interface to select the system sleep state */
static ssize_t omap_pm_sleep_idle_state_show(struct kset *subsys, char *buf)
{
	return sprintf(buf, "%hu\n", cpuidle_deepest_st);
}

static ssize_t omap_pm_sleep_idle_state_store(struct kset *subsys,
				const char *buf, size_t n)
{
	unsigned short value;

	if (sscanf(buf, "%hu", &value) != 1 ||
		(value > (MAX_SUPPORTED_STATES - 1))) {
		printk(KERN_ERR "\nError : Invalid value."
				" (Valid range for current config is 0 - %d)\n\n",
				(MAX_SUPPORTED_STATES - 1));
		return -EINVAL;
	}

	cpuidle_deepest_st = value;

	if ((value == 0) && cpuidle_running) {
		cpuidle_pause_and_lock ();
		cpuidle_running = 0;
	}
	if (!cpuidle_running && (value != 0)) {
		cpuidle_resume_and_unlock();
		cpuidle_running = 1;
	}

	return n;
}

static struct subsys_attribute sleep_idle_state = {
	.attr = {
		.name = __stringify(cpuidle_deepest_state),
		.mode = 0644,
		},
	.show  = omap_pm_sleep_idle_state_show,
	.store = omap_pm_sleep_idle_state_store,
};

static ssize_t omap_pm_off_show(struct kset *subsys, char *buf)
{
	return sprintf(buf, "%hu\n", enable_off);
}

static ssize_t omap_pm_off_store(struct kset *subsys,
				const char *buf, size_t n)
{
	unsigned short value;
	if (sscanf(buf, "%hu", &value) != 1 ||
		((value != 0) && (value != 1))) {
		printk(KERN_ERR "off_enable: Invalid value\n");
		return -EINVAL;
	}
	enable_off = value;
	return n;
}

static struct subsys_attribute off_enable = {
	.attr = {
		.name = __stringify(enable_mpucoreoff),
		.mode = 0644,
		},
	.show  = omap_pm_off_show,
	.store = omap_pm_off_store,
};

static ssize_t omap_pm_domain_show(struct kset *subsys, char *buf)
{
	prcm_printreg(debug_domain);
	return 0;
}

static ssize_t omap_pm_domain_store(struct kset *subsys,
				const char *buf, size_t n)
{
	unsigned short value;
	if (sscanf(buf, "%hu", &value) != 1 ||
		(value > PRCM_NUM_DOMAINS)
		 || (value == DOM_CORE2)
		 || (value == 12)) {
		printk(KERN_ERR "dump_domain: Invalid value\n");
		return -EINVAL;
	}
	debug_domain = value;
	return n;
}


static struct subsys_attribute domain_print = {
	.attr = {
		.name = __stringify(debug_domain),
		.mode = 0644,
		},
	.show  = omap_pm_domain_show,
	.store = omap_pm_domain_store,
};

/*sysfs interface to chg the frame buffer timer*/
static ssize_t omap_pm_fb_timeout_show(struct kset *subsys, char *buf)
{
	return sprintf(buf, "%hu\n", fb_timeout_val);
}

static ssize_t omap_pm_fb_timeout_store(struct kset *subsys,
					const char *buf, size_t n)
{
	unsigned short value;

	if (sscanf(buf, "%hu", &value) != 1) {
		printk(KERN_ERR "fb_timeout_store: Invalid value\n");
		return -EINVAL;
	}

	fb_timeout_val = value;
	set_blank_interval(fb_timeout_val);

	return n;
}

static struct subsys_attribute fb_timeout = {
	.attr = {
		.name = __stringify(fb_timeout_value),
		.mode = 0644,
		},
	.show  = omap_pm_fb_timeout_show,
	.store = omap_pm_fb_timeout_store,
};


/* Sysfs interface to set the opp for vdd1 and vdd2 */
static ssize_t omap_pm_vdd1_opp_show(struct kset *subsys, char *buf)
{
	return sprintf(buf, "%x\n", (unsigned int)get_opp_no(
							current_vdd1_opp));
}

static ssize_t omap_pm_vdd1_opp_store(struct kset *subsys,
						const char *buf, size_t n)
{
	sscanf(buf, "%u", &target_opp_no);

#ifdef CONFIG_OMAP3430_ES2
	if ((target_opp_no < 1) || (target_opp_no > 5))
#else
	if ((target_opp_no < 1) || (target_opp_no > 4))
#endif
	{
		printk(KERN_ERR "\nError : Invalid OPP value for VDD1."
#ifdef CONFIG_OMAP3430_ES2
				" (Valid range is 1 - 5).\n\n");
#else
				" (Valid range is 1 - 4).\n\n");
#endif
		return -EINVAL;
	}
	if (target_opp_no == get_opp_no(current_vdd1_opp)) {
		DPRINTK("Target and current opp values are same\n");
	} else {
		if (target_opp_no != 1) {
			if (vdd1_opp_co == NULL)
				vdd1_opp_co = constraint_get("pm_fwk",
								&cnstr_id1);
			constraint_set(vdd1_opp_co, target_opp_no);
		} else {
			if (vdd1_opp_co != NULL) {
				constraint_remove(vdd1_opp_co);
				constraint_put(vdd1_opp_co);
				vdd1_opp_co = NULL;
			}
		}
	}
	return n;
}

static struct subsys_attribute vdd1_opp = {
	.attr = {
		.name = __stringify(vdd1_opp_value),
		.mode = 0644,
		},
	.show  = omap_pm_vdd1_opp_show,
	.store = omap_pm_vdd1_opp_store,
};

static ssize_t omap_pm_vdd2_opp_show(struct kset *subsys, char *buf)
{
	return sprintf(buf, "%x\n", (unsigned int)get_opp_no(current_vdd2_opp));
}

static ssize_t omap_pm_vdd2_opp_store(struct kset *subsys,
						const char *buf, size_t n)
{
	sscanf(buf, "%u", &target_opp_no);
#ifdef CONFIG_OMAP3430_ES2
	if ((target_opp_no < 1) || (target_opp_no > 3))
#else
	if ((target_opp_no < 1) || (target_opp_no > 2))
#endif
	{
		printk(KERN_ERR "\nError : Invalid OPP value for VDD2."
#ifdef CONFIG_OMAP3430_ES2
				" (Valid range is 1 - 3).\n\n");
#else
				" (Valid range is 1 - 2).\n\n");
#endif
		return -EINVAL;
	}
	if (target_opp_no == get_opp_no(current_vdd2_opp)) {
		DPRINTK("Target and current opp values are same\n");
	} else {
		if (target_opp_no != 1) {
			if (vdd2_opp_co == NULL)
				vdd2_opp_co = constraint_get("pm_fwk",
								&cnstr_id2);
			constraint_set(vdd2_opp_co, target_opp_no);
		} else {
			if (vdd2_opp_co != NULL) {
				constraint_remove(vdd2_opp_co);
				constraint_put(vdd2_opp_co);
				vdd2_opp_co = NULL;
			}
		}
	}

	return n;
}
static struct subsys_attribute vdd2_opp = {
	.attr = {
		.name = __stringify(vdd2_opp_value),
		.mode = 0644,
		},
	.show  = omap_pm_vdd2_opp_show,
	.store = omap_pm_vdd2_opp_store,
};

/* PRCM Interrupt Handler */
irqreturn_t prcm_interrupt_handler (int irq, void *dev_id)
{
	u32 wkst_wkup = PM_WKST_WKUP;
	u32 wkst1_core = PM_WKST1_CORE;
#ifdef CONFIG_OMAP3430_ES2
	u32 wkst3_core = PM_WKST3_CORE;
	u32 wkst_usbhost = PM_WKST_USBHOST;
#endif
	u32 wkst_per = PM_WKST_PER;
	u32 errst_vc = PRM_VC_TIMEOUTERR_ST | PRM_VC_RAERR_ST |
							PRM_VC_SAERR_EN;
	u32 fclk = 0;
	u32 iclk = 0;

	if (PM_WKST_WKUP) {
#ifdef IOPAD_WKUP
		/* Resetting UART1 inactivity timeout, during IO_PAD wakeup */
		if (wkst_wkup & 0x100) {
			awake_time_end = jiffies +
					 msecs_to_jiffies(UART_TIME_OUT);
		}

#endif /* #ifdef IOPAD_WKUP */

		iclk = CM_ICLKEN_WKUP;
		fclk = CM_FCLKEN_WKUP;
		CM_ICLKEN_WKUP |= wkst_wkup;
		CM_FCLKEN_WKUP |= wkst_wkup;
		PM_WKST_WKUP = wkst_wkup;
		while (PM_WKST_WKUP);
		CM_ICLKEN_WKUP = iclk;
		CM_FCLKEN_WKUP = fclk;
	}
	if (PM_WKST1_CORE) {
		iclk = CM_ICLKEN1_CORE;
		fclk = CM_FCLKEN1_CORE;
		CM_ICLKEN1_CORE |= wkst1_core;
		CM_FCLKEN1_CORE |= wkst1_core;
		PM_WKST1_CORE = wkst1_core;
		while (PM_WKST1_CORE);
		CM_ICLKEN1_CORE = iclk;
		CM_FCLKEN1_CORE = fclk;
	}
#ifdef CONFIG_OMAP3430_ES2
	if (PM_WKST3_CORE) {
		iclk = CM_ICLKEN3_CORE;
		fclk = CM_FCLKEN3_CORE;
		CM_ICLKEN3_CORE |= wkst3_core;
		CM_FCLKEN3_CORE |= wkst3_core;
		PM_WKST3_CORE = wkst3_core;
		while (PM_WKST3_CORE);
		CM_ICLKEN3_CORE = iclk;
		CM_FCLKEN3_CORE = fclk;
	}
	if (PM_WKST_USBHOST) {

		iclk = CM_ICLKEN_USBHOST;
		fclk = CM_FCLKEN_USBHOST;
		CM_ICLKEN_USBHOST |= wkst_usbhost;
		CM_FCLKEN_USBHOST |= wkst_usbhost;
		PM_WKST_USBHOST = wkst_usbhost;
		while (PM_WKST_USBHOST);
		CM_ICLKEN_USBHOST = iclk;
		CM_FCLKEN_USBHOST = fclk;
	}
#endif
	if (PM_WKST_PER) {
		iclk = CM_ICLKEN_PER;
		fclk = CM_FCLKEN_PER;
		CM_ICLKEN_PER |= wkst_per;
		CM_FCLKEN_PER |= wkst_per;
		PM_WKST_PER = wkst_per;
		while (PM_WKST_PER);
		CM_ICLKEN_PER = iclk;
		CM_FCLKEN_PER = fclk;
	}
#ifdef CONFIG_OMAP3430_ES2
	if (!(wkst_wkup | wkst1_core | wkst3_core | wkst_usbhost | wkst_per)) {
#else
	if (!(wkst_wkup | wkst1_core | wkst_per)) {
#endif
		if (!(PRM_IRQSTATUS_MPU & errst_vc)) {
			printk(KERN_ERR "%x,%x,%x,%x\n", PRM_IRQSTATUS_MPU,
					wkst_wkup, wkst1_core, wkst_per);
			printk(KERN_ERR "Spurious PRCM interrupt\n");
		}
	}

#ifdef CONFIG_OMAP_VOLT_SR_BYPASS
	if (PRM_IRQSTATUS_MPU & PRM_VC_TIMEOUTERR_ST) {
		printk(KERN_ERR "PRCM : Voltage Controller timeout\n");
		printk(KERN_ERR "PRM_IRQSTATUS_MPU    = 0x%x\n", PRM_IRQSTATUS_MPU) ;
		printk(KERN_ERR "PRM_VC_TIMEOUTERR_ST = 0x%x\n", PRM_VC_TIMEOUTERR_ST) ;
	}
	if (PRM_IRQSTATUS_MPU & PRM_VC_RAERR_ST) {
		printk(KERN_ERR "PRCM : Voltage Controller register address "
							"acknowledge error\n");
	}
	if (PRM_IRQSTATUS_MPU & PRM_VC_SAERR_ST) {
		printk(KERN_ERR "PRCM : Voltage Controller slave address "
							"acknowledge error\n");
	}
#endif /* #ifdef CONFIG_OMAP_VOLT_SR_BYPASS */

	if (PRM_IRQSTATUS_MPU) {
		PRM_IRQSTATUS_MPU |= 0x3;
		while (PRM_IRQSTATUS_MPU);
	}
	return IRQ_HANDLED;
}

#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
static void *omap_pm_prepwst_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *omap_pm_prepwst_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

static void omap_pm_prepwst_stop(struct seq_file *m, void *v)
{
}

char *pstates [4] =	{
			 "OFF",
			 "RET",
			 "INACT",
			 "ON"
			};

int omap_pm_prepwst_show(struct seq_file *m, void *v)
{
	u32 arr_rdptr, arr_cnt;

	seq_printf(m, "\nPrevious power state for MPU+CORE\n\n");

	arr_rdptr = arr_wrptr;
	arr_cnt = LAST_IDLE_STATE_ARR_SIZE;

#ifdef DEBUG_HW_SUP
	seq_printf(m, "PM_PWSTST_CORE %x\n", PM_PWSTST_CORE);
	seq_printf(m, "PM_PREPWSTST_CORE %x\n", PM_PREPWSTST_CORE);
	seq_printf(m, "CM_CLKSTCTRL_CORE %x\n", CM_CLKSTCTRL_CORE);
	seq_printf(m, "CM_CLKSTST_CORE %x\n\n", CM_CLKSTST_CORE);

	seq_printf(m, "PM_PWSTST_IVA2 %x\n", PM_PWSTST_IVA2);
	seq_printf(m, "PM_PREPWSTST_IVA2 %x\n", PM_PREPWSTST_IVA2);
	seq_printf(m, "CM_CLKSTCTRL_IVA2 %x\n", CM_CLKSTCTRL_IVA2);
	seq_printf(m, "CM_CLKSTST_IVA2 %x\n\n", CM_CLKSTST_IVA2);

#ifndef CONFIG_ARCH_OMAP3410
	seq_printf(m, "PM_PWSTST_NEON %x\n", PM_PWSTST_NEON);
	seq_printf(m, "PM_PREPWSTST_NEON %x\n", PM_PREPWSTST_NEON);
	seq_printf(m, "CM_CLKSTCTRL_NEON %x\n\n", CM_CLKSTCTRL_NEON);
#endif
	seq_printf(m, "PM_PWSTST_PER %x\n", PM_PWSTST_PER);
	seq_printf(m, "PM_PREPWSTST_PER %x\n", PM_PREPWSTST_PER);
	seq_printf(m, "CM_CLKSTCTRL_PER %x\n", CM_CLKSTCTRL_PER);
	seq_printf(m, "CM_CLKSTST_PER %x\n\n", CM_CLKSTST_PER);

	seq_printf(m, "PM_PWSTST_DSS %x\n", PM_PWSTST_DSS);
	seq_printf(m, "PM_PREPWSTST_DSS %x\n", PM_PREPWSTST_DSS);
	seq_printf(m, "CM_CLKSTCTRL_DSS %x\n", CM_CLKSTCTRL_DSS);
	seq_printf(m, "CM_CLKSTST_DSS %x\n\n", CM_CLKSTST_DSS);

#ifdef CONFIG_OMAP3430_ES2
	seq_printf(m, "PM_PWSTST_USBHOST %x\n", PM_PWSTST_USBHOST);
	seq_printf(m, "PM_PREPWSTST_USBHOST %x\n", PM_PREPWSTST_USBHOST);
	seq_printf(m, "CM_CLKSTCTRL_USBHOST %x\n", CM_CLKSTCTRL_USBHOST);
	seq_printf(m, "CM_CLKSTST_USBHOST %x\n\n", CM_CLKSTST_USBHOST);

#ifdef CONFIG_ARCH_HAS_OMAP3_SGX
	seq_printf(m, "PM_PWSTST_SGX %x\n", PM_PWSTST_SGX);
	seq_printf(m, "PM_PREPWSTST_SGX %x\n", PM_PREPWSTST_SGX);
	seq_printf(m, "CM_CLKSTCTRL_SGX %x\n", CM_CLKSTCTRL_SGX);
	seq_printf(m, "CM_CLKSTST_SGX %x\n\n", CM_CLKSTST_SGX);
#endif
#else
	seq_printf(m, "PM_PWSTST_GFX %x\n", PM_PWSTST_GFX);
	seq_printf(m, "PM_PREPWSTST_GFX %x\n", PM_PREPWSTST_GFX);
	seq_printf(m, "CM_CLKSTCTRL_GFX %x\n", CM_CLKSTCTRL_GFX);
	seq_printf(m, "CM_CLKSTST_GFX %x\n\n", CM_CLKSTST_GFX);
#endif /* ifdef CONFIG_OMAP3430_ES2 */

	seq_printf(m, "PM_PWSTST_CAM %x\n", PM_PWSTST_CAM);
	seq_printf(m, "PM_PREPWSTST_CAM %x\n", PM_PREPWSTST_CAM);
	seq_printf(m, "CM_CLKSTCTRL_CAM %x\n", CM_CLKSTCTRL_CAM);
	seq_printf(m, "CM_CLKSTST_CAM %x\n\n", CM_CLKSTST_CAM);
#endif /* ifdef DEBUG_HW_SUP */

	while (arr_cnt--) {
		if (arr_rdptr == 0)
			arr_rdptr = LAST_IDLE_STATE_ARR_SIZE;

		arr_rdptr--;
		seq_printf(m,
			"MPU = %x - %s, CORE = %x - %s, IVA = %x - %s "
						"PER = %x - %s "
			"fclks: %x, iclks: %x\n",
			last_idle_state[arr_wrptr].mpu_state,
			pstates [last_idle_state[arr_wrptr].mpu_state & 0x3],
			last_idle_state[arr_wrptr].core_state,
			pstates [last_idle_state[arr_wrptr].core_state & 0x3],
			last_idle_state[arr_wrptr].iva_state,
			pstates [last_idle_state[arr_wrptr].iva_state & 0x3],
			last_idle_state[arr_wrptr].per_state,
			pstates [last_idle_state[arr_wrptr].per_state & 0x3],			
			last_idle_state[arr_wrptr].fclks,
			last_idle_state[arr_wrptr].iclks
		);
	}
	seq_printf(m, "\nMPU RET CNT = %d, MPU OFF CNT = %d,"
			" CORE RET CNT = %d, CORE OFF CNT = %d\n\n",
			mpu_ret_cnt, mpu_off_cnt, core_ret_cnt, core_off_cnt);

	return 0;
}

static struct seq_operations omap_pm_prepwst_op = {
	.start = omap_pm_prepwst_start,
	.next  = omap_pm_prepwst_next,
	.stop  = omap_pm_prepwst_stop,
	.show  = omap_pm_prepwst_show
};

static int omap_pm_prepwst_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &omap_pm_prepwst_op);
}

static struct file_operations proc_pm_prepwst_ops = {
	.open		= omap_pm_prepwst_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

/* This API creates a proc entry for shared resources. */
int create_pmproc_entry(void)
{
	struct proc_dir_entry *entry;

	/* Create a proc entry for shared resources */
	entry = create_proc_entry("pm_prepwst", 0, NULL);
	if (entry) {
		entry->proc_fops = &proc_pm_prepwst_ops;
		}
	else
		printk(KERN_ERR "create_proc_entry failed\n");

	return 0;
}

#endif  /* #ifdef CONFIG_PROC_FS */


static int __init omap3_pm_init(void)
{
	int l, m, ret;
	printk(KERN_DEBUG "Power Management for TI OMAP.\n");

	omap3_push_sram_functions();
	pm_set_ops(&omap_pm_ops);

	/* In case of cold boot, clear scratchpad */
	if (RM_RSTST_CORE & 0x1)
		clear_scratchpad_contents();

#ifdef CONFIG_MPU_OFF
	save_scratchpad_contents();
#endif
	prcm_init();
	if (!is_device_type_gp()) {
		sdram_mem = kmalloc(sizeof(struct sram_mem), GFP_KERNEL);
		if (!sdram_mem)
			printk(KERN_ERR "Memory allocation failed when"
				"allocating for secure sram context");
	}
	memret1 = (struct res_handle *)resource_get("corememresret1",
							"core_mem1ret");
	memret2 = (struct res_handle *)resource_get("corememresret2",
							"core_mem2ret");
	logret1 = (struct res_handle *)resource_get("corelogicret",
							"core_logicret");

	/* Registering PRCM Interrupts to MPU, for Wakeup events */
	ret = request_irq(PRCM_MPU_IRQ, (irq_handler_t)prcm_interrupt_handler,
				IRQF_DISABLED, "prcm", NULL);
	if (ret) {
		printk(KERN_ERR "request_irq failed to register for 0x%x\n",
			PRCM_MPU_IRQ);
		return -1;
	}

	/* Adjust the system OPP based during bootup using
	 * KConfig option
	 */
	if (p_vdd1_clk == NULL) {
		p_vdd1_clk = clk_get(NULL, "virt_vdd1_prcm_set");
		if (p_vdd1_clk == NULL) {
			printk(KERN_ERR "Unable to get the VDD1 clk\n");
			return -1;
		}
	}

	if (p_vdd2_clk == NULL) {
		p_vdd2_clk = clk_get(NULL, "virt_vdd2_prcm_set");
		if (p_vdd2_clk == NULL) {
			printk(KERN_ERR "Unable to get the VDD2 clk\n");
			return -1;
		}
	}
#ifdef CONFIG_MPU_OFF
	clear_scratchpad_contents();
	save_scratchpad_contents();
#endif
	/* Configure OPP for VDD1 based on compile time setting */
#ifdef CONFIG_OMAP3ES1_VDD1_OPP1
	vdd1_opp_no = PRCM_VDD1_OPP1;
#elif defined(CONFIG_OMAP3ES2_VDD1_OPP1) \
	|| defined(CONFIG_OMAP3ES2_3410_VDD1_OPP1)
	vdd1_opp_no = PRCM_VDD1_OPP1;
#elif defined(CONFIG_OMAP3ES1_VDD1_OPP2)
	vdd1_opp_no = PRCM_VDD1_OPP2;
#elif defined(CONFIG_OMAP3ES2_VDD1_OPP2) \
	|| defined(CONFIG_OMAP3ES2_3410_VDD1_OPP2)
	vdd1_opp_no = PRCM_VDD1_OPP2;
#elif defined(CONFIG_OMAP3ES1_VDD1_OPP3)
	vdd1_opp_no = PRCM_VDD1_OPP3;
#elif defined(CONFIG_OMAP3ES2_VDD1_OPP3) \
	|| defined(CONFIG_OMAP3ES2_3410_VDD1_OPP3)
	vdd1_opp_no = PRCM_VDD1_OPP3;
#elif defined(CONFIG_OMAP3ES1_VDD1_OPP4)
	vdd1_opp_no = PRCM_VDD1_OPP4;
#elif defined(CONFIG_OMAP3ES2_VDD1_OPP4) \
	|| defined(CONFIG_OMAP3ES2_3410_VDD1_OPP4)
	vdd1_opp_no = PRCM_VDD1_OPP4;
#elif defined(CONFIG_OMAP3ES2_VDD1_OPP5) \
	|| defined(CONFIG_OMAP3ES2_3410_VDD1_OPP5)
	vdd1_opp_no = PRCM_VDD1_OPP5;
#endif

#ifdef CONFIG_OMAP3ES2_VDD2_OPP2
	vdd2_opp_no = PRCM_VDD2_OPP2;
#elif defined(CONFIG_OMAP3ES2_VDD2_OPP3)
	vdd2_opp_no = PRCM_VDD2_OPP3;
	prcm_do_voltage_scaling(PRCM_VDD2_OPP3, PRCM_VDD2_OPP2);
#endif

	/* Request for VDD1 and VDD2 OPP constraints */
	/* Currently by default VDD2 OPP level 3 is requested */
	if (vdd1_opp_no != PRCM_VDD1_OPP1) {
		vdd1_opp_co = constraint_get("pm_fwk", &cnstr_id1);
		constraint_set(vdd1_opp_co, get_opp_no(vdd1_opp_no));
	}

	if (vdd2_opp_no != PRCM_VDD2_OPP1) {
		vdd2_opp_co = constraint_get("pm_fwk", &cnstr_id2);
		constraint_set(vdd2_opp_co, get_opp_no(vdd2_opp_no));
	}

	PRM_IRQSTATUS_MPU = 0x3FFFFFD;
	PRM_IRQENABLE_MPU = 0x1;
#ifdef IOPAD_WKUP
	/* Enabling the IO_PAD PRCM interrupts */
	PRM_IRQENABLE_MPU |= 0x200;
#endif /* #ifdef IOPAD_WKUP */

#ifdef CONFIG_OMAP_VOLT_SR_BYPASS
	/* Enabling the VOLTAGE CONTROLLER PRCM interrupts */
	PRM_IRQENABLE_MPU |= PRM_VC_TIMEOUTERR_EN | PRM_VC_RAERR_EN|
							PRM_VC_SAERR_EN;
#endif /* #ifdef CONFIG_OMAP_VOLT_SR_BYPASS */
	l = subsys_create_file(&power_subsys, &sleep_idle_state);
	if (l)
		printk(KERN_ERR "subsys_create_file failed: %d\n", l);
	l = subsys_create_file(&power_subsys, &off_enable);
	if (l)
		printk(KERN_ERR "subsys_create_file failed for off: %d\n", l);
	/*For register dump*/
	l = subsys_create_file(&power_subsys, &domain_print);
	if (l)
		printk(KERN_ERR "subsys_create_file failed for print: %d\n", l);

	/* Sysfs entry for setting Display timeout*/
	m = subsys_create_file(&power_subsys, &fb_timeout);
	if (m)
		printk(KERN_ERR "subsys_create_file failed: %d\n", m);
	 /* Sysfs entry for setting opp for vdd1 */
		m = subsys_create_file(&power_subsys, &vdd1_opp);
	if (m)
		printk(KERN_ERR "subsys_create_file failed: %d\n", m);
	 /* Sysfs entry for setting opp for vdd2 */
	m = subsys_create_file(&power_subsys, &vdd2_opp);
	if (m)
		printk(KERN_ERR "subsys_create_file failed: %d\n", m);

	/* Set the default value of frame buffer timeout*/
	set_blank_interval(fb_timeout_val);
#ifdef CONFIG_PROC_FS
	create_pmproc_entry();
#endif  /* #ifdef CONFIG_PROC_FS */

	return 0;
}

/**
 * clear_scratchpad_contents - Clears the scratchpad contents
 *
 * Clears the scratchpad contents in case of cold boot. Called during bootup.
 */
void clear_scratchpad_contents(void)
{
	u32 max_offset = SCRATCHPAD_ROM_OFFSET;
	u32 offset = 0;
	u32 v_addr = io_p2v(SCRATCHPAD_ROM);

	/* Check if it is a cold reboot */
	if (PRM_RSTST & 0x1) {
		for ( ; offset <= max_offset; offset += 0x4)
			__raw_writel(0x0, (v_addr + offset));
		PRM_RSTST |= 0x1;
	}
}

/**
 * save_scratchpad_contents
 *
 * Populate the scratchpad structure with restore structure.
 */
void save_scratchpad_contents(void)
{
	u32 *scratchpad_address;
	u32 *restore_address;
	u32 *sdram_context_address;
	/* Get virtual address of SCRATCHPAD */
	scratchpad_address = (u32 *) io_p2v(SCRATCHPAD);
	/* Get Restore pointer to jump to while waking up from OFF */
	restore_address = get_restore_pointer();
	/* Convert it to physical address */
	restore_address = (u32 *) io_v2p(restore_address);
	/* Get address where registers are saved in SDRAM */
	sdram_context_address = (u32 *) io_v2p(context_mem);
	/* Booting configuration pointer*/
	*(scratchpad_address++) = 0x0;
	/* Public restore pointer */
	/* On ES 2.0, if scratchpad is populated with valid
	 * pointer, warm reset does not work
	 * So populate scratchpad restore address only in
	 * cpuidle and suspend calls
	 */
	scratchpad_restore_addr = scratchpad_address;
	restore_pointer_address = (u32) restore_address;
	*(scratchpad_address++) = restore_pointer_address;
	/* Secure ram restore pointer */
	if (is_device_type_gp())
	*(scratchpad_address++) = 0x0;
	else
		*(scratchpad_address++) = (u32) __pa(sdram_mem);
	/* SDRC Module semaphore */
	*(scratchpad_address++) = 0x0;
	/* PRCM Block Offset */
	*(scratchpad_address++) = 0x2C;
	/* SDRC Block Offset */
	*(scratchpad_address++) = 0x64;
	/* Empty */
	/* Offset 0x8*/
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = 0x0;
	/* Offset 0xC*/
	*(scratchpad_address++) = 0x0;
	/* Offset 0x10*/
	*(scratchpad_address++) = 0x0;
	/* Offset 0x14*/
	*(scratchpad_address++) = 0x0;
	/* Offset 0x18*/
	/* PRCM Block */
	*(scratchpad_address++) = PRM_CLKSRC_CTRL;
	*(scratchpad_address++) = PRM_CLKSEL;
	*(scratchpad_address++) = CM_CLKSEL_CORE;
	*(scratchpad_address++) = CM_CLKSEL_WKUP;
	*(scratchpad_address++) = CM_CLKEN_PLL;
	*(scratchpad_address++) = CM_AUTOIDLE_PLL;
	*(scratchpad_address++) = CM_CLKSEL1_PLL;
	*(scratchpad_address++) = CM_CLKSEL2_PLL;
	*(scratchpad_address++) = CM_CLKSEL3_PLL;
	*(scratchpad_address++) = CM_CLKEN_PLL_MPU;
	*(scratchpad_address++) = CM_AUTOIDLE_PLL_MPU;
	*(scratchpad_address++) = CM_CLKSEL1_PLL_MPU;
	*(scratchpad_address++) = CM_CLKSEL2_PLL_MPU;
	*(scratchpad_address++) = 0x0;
	/* SDRC Block */
	*(scratchpad_address++) = ((SDRC_CS_CFG & 0xFFFF) << 16) |
					(SDRC_SYS_CONFIG & 0xFFFF);
	*(scratchpad_address++) = ((SDRC_ERR_TYPE & 0xFFFF) << 16) |
					(SDRC_SHARING & 0xFFFF);
	*(scratchpad_address++) = SDRC_DLL_A_CTRL;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = SDRC_PWR;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = SDRC_MCFG_0;
	*(scratchpad_address++) = SDRC_MR0 & 0xFFFF;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = SDRC_ACTIM_CTRL_A_0;
	*(scratchpad_address++) = SDRC_ACTIM_CTRL_B_0;
	*(scratchpad_address++) = SDRC_RFR_CTRL_0;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = SDRC_MCFG_1;
	*(scratchpad_address++) = SDRC_MR1 & 0xFFFF;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = SDRC_ACTIM_CTRL_A_1;
	*(scratchpad_address++) = SDRC_ACTIM_CTRL_B_1;
	*(scratchpad_address++) = SDRC_RFR_CTRL_1;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = (u32) sdram_context_address;
}

arch_initcall(omap3_pm_init);
