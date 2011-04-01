/*
 *  linux/include/asm-arm/arch-omap/clock.h
 *
 *  Copyright (C) 2004 - 2005 Nokia corporation
 *  Written by Tuukka Tikkanen <tuukka.tikkanen@elektrobit.com>
 *  Based on clocks.h by Tony Lindgren, Gordon McNutt and RidgeRun, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ARCH_ARM_OMAP_CLOCK_H
#define __ARCH_ARM_OMAP_CLOCK_H

#include <linux/clk.h>
#include <linux/cpufreq.h>
#if defined (CONFIG_ARCH_OMAP34XX)
#include <asm/arch/resource.h>
#endif /* #if defined (CONFIG_ARCH_OMAP34XX) */

struct module;

#ifdef CONFIG_TRACK_RESOURCES
#define NUM_RES_HANDLES 100

struct resource_handle {
        struct clk *clk;
        struct list_head node1;
        struct list_head node2;
	struct device *dev;
	short index;
};
#endif

struct clk {
#if defined (CONFIG_ARCH_OMAP34XX)
	struct res_handle *res;
#endif /* #if defined (CONFIG_ARCH_OMAP34XX) */
	struct list_head node;
#ifdef CONFIG_TRACK_RESOURCES
        struct list_head clk_got;
        struct list_head clk_enabled;
#endif
	struct module *owner;
	const char *name;
	int id;
#if defined (CONFIG_ARCH_OMAP34XX)
	__u32 prcmid;
#endif
	struct clk *parent;
	unsigned long rate;
	__u32 flags;
	void __iomem *enable_reg;
	__u8 enable_bit;
	__u8 rate_offset;
	__u8 src_offset;
	__s8 usecount;
	void (*recalc) (struct clk *);
	int (*set_rate) (struct clk *, unsigned long);
	long (*round_rate) (struct clk *, unsigned long);
	void (*init) (struct clk *);
	int (*enable) (struct clk *);
	void (*disable) (struct clk *);
};

struct clk_functions {
	int (*clk_enable) (struct clk * clk);
	void (*clk_disable) (struct clk * clk);
	long (*clk_round_rate) (struct clk * clk, unsigned long rate);
	int (*clk_set_rate) (struct clk * clk, unsigned long rate);
	int (*clk_set_parent) (struct clk * clk, struct clk * parent);
	struct clk *(*clk_get_parent) (struct clk * clk);
	void (*clk_allow_idle) (struct clk * clk);
	void (*clk_deny_idle) (struct clk * clk);
	void (*clk_disable_unused) (struct clk * clk);
#ifdef CONFIG_CPU_FREQ
	void (*clk_init_cpufreq_table)(struct cpufreq_frequency_table **table);
#endif
};

extern unsigned int mpurate;

extern int clk_init(struct clk_functions *custom_clocks);
extern int clk_register(struct clk *clk);
extern void clk_unregister(struct clk *clk);
extern void propagate_rate(struct clk *clk);
extern void followparent_recalc(struct clk *clk);
extern void clk_allow_idle(struct clk *clk);
extern void clk_deny_idle(struct clk *clk);
extern int clk_get_usecount(struct clk *clk);
#if defined (CONFIG_ARCH_OMAP34XX)
extern void clk_use(struct clk *clk);
extern void clk_unuse(struct clk *clk);
extern void clk_safe(struct clk *clk);
#endif /* #if defined (CONFIG_ARCH_OMAP34XX) */
#ifdef CONFIG_CPU_FREQ
extern void clk_init_cpufreq_table(struct cpufreq_frequency_table **table);
#endif
extern void omap_show_domain_clks (u32);

/* Clock flags */
#define RATE_CKCTL		(1 << 0)	/* Rate of clock can be changed based on CLKSEL registers */
#define RATE_FIXED		(1 << 1)	/* Fixed clock rate */
#define RATE_PROPAGATES		(1 << 2)	/* Program children too */
#define VIRTUAL_CLOCK		(1 << 3)	/* Composite clock from table */
#define ALWAYS_ENABLED		(1 << 4)	/* Clock cannot be disabled */
#define ENABLE_REG_32BIT	(1 << 5)	/* Use 32-bit access */
#define VIRTUAL_IO_ADDRESS	(1 << 6)	/* Clock in virtual address */
#define CLOCK_IDLE_CONTROL	(1 << 7)
#define CLOCK_NO_IDLE_PARENT	(1 << 8)

#if defined (CONFIG_ARCH_OMAP34XX)
#define SRC_SEL			(1 << 9)	/* Source of the clock can be changed */
#define VDD1_CONFIG_PARTICIPANT	(1 << 10)	/* Fundamental clock */
#define VDD2_CONFIG_PARTICIPANT	(1 << 11)	/* Fundamental clock */
#define F_CLK			(1 << 12)	/* Functional clock */
#define I_CLK			(1 << 13)	/* Interface clock */
#define DPLL_OUTPUT		(1 << 14)	/* DPLL output */

#else				/* else part of CONFIG_ARCH_OMAP34XX */
#define DELAYED_APP             (1 << 9)	/* Delay application of clock */
#define CONFIG_PARTICIPANT      (1 << 10)	/* Fundamental clock */
#define CM_MPU_SEL1             (1 << 11)	/* Domain divider/source */
#define CM_DSP_SEL1             (1 << 12)
#define CM_GFX_SEL1             (1 << 13)
#define CM_MODEM_SEL1           (1 << 14)
#define CM_CORE_SEL1            (1 << 15)	/* Sets divider for many */
#define CM_CORE_SEL2            (1 << 16)	/* sets parent for GPT */
#define CM_WKUP_SEL1            (1 << 17)
#define CM_PLL_SEL1             (1 << 18)
#define CM_PLL_SEL2             (1 << 19)
#define CM_SYSCLKOUT_SEL1       (1 << 20)
#endif				/*endif for CONFIG_ARCH_OMAP34XX */

#define CLOCK_IN_OMAP310	(1 << 21)
#define CLOCK_IN_OMAP730	(1 << 22)
#define CLOCK_IN_OMAP1510	(1 << 23)
#define CLOCK_IN_OMAP16XX	(1 << 24)
#define CLOCK_IN_OMAP242X	(1 << 25)
#define CLOCK_IN_OMAP243X	(1 << 26)
#define CLOCK_IN_OMAP343X	(1 << 27)

#if defined (CONFIG_ARCH_OMAP34XX)
#define POWER_ON_REQUIRED	(1 << 28)	/* For devices which need to be powered on */
#endif /* #if defined (CONFIG_ARCH_OMAP34XX) */
#endif
