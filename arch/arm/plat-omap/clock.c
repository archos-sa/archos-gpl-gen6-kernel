/*
 *  linux/arch/arm/plat-omap/clock.c
 *
 *  Copyright (C) 2004 - 2005 Nokia corporation
 *  Written by Tuukka Tikkanen <tuukka.tikkanen@elektrobit.com>
 *
 *  Modified for omap shared clock framework by Tony Lindgren <tony@atomide.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#ifdef CONFIG_ARCH_OMAP34XX
#include <linux/proc_fs.h>
#endif

#include <asm/io.h>
#include <asm/semaphore.h>

#include <asm/arch/clock.h>
#include <asm/arch/prcm.h>

#ifdef CONFIG_TRACK_RESOURCES
/*
 * Function clk_get() has been updated to return 'clk' instead of the
 * resource handle container "res_han[index]".
 *
 * The macro has been updated to minimize changes across the file.
 *
 * TBD: Clean extra code due to this macro.
 */
#define R(clk) (clk)
#else
#define R(clk) (clk)
#endif

static LIST_HEAD(clocks);
static DEFINE_MUTEX(clocks_mutex);
static DEFINE_SPINLOCK(clockfw_lock);

static struct clk_functions *arch_clock;

#ifdef CONFIG_TRACK_RESOURCES
static DEFINE_MUTEX(res_mutex);
struct resource_handle res_han[NUM_RES_HANDLES];
short a[NUM_RES_HANDLES] = {0};
#endif

#if defined (CONFIG_ARCH_OMAP34XX)
extern struct dpll_registers dpll_reg[NO_OF_DPLL];
#endif

#ifdef CONFIG_PM_DEBUG

static void print_parents(struct clk *clk)
{
	struct clk *p;
	int printed = 0;

	list_for_each_entry(p, &clocks, node) {
		if (p->parent == clk && p->usecount) {
			if (!clk->usecount && !printed) {
				printk("MISMATCH: %s\n", clk->name);
				printed = 1;
			}
			printk("\t%-15s\n", p->name);
		}
	}
}

void clk_print_usecounts(void)
{
	unsigned long flags;
	struct clk *p;

	spin_lock_irqsave(&clockfw_lock, flags);
	list_for_each_entry(p, &clocks, node) {
		if (p->usecount)
			printk("%-15s: %d\n", p->name, p->usecount);
		print_parents(p);

	}
	spin_unlock_irqrestore(&clockfw_lock, flags);
}

#endif

/*-------------------------------------------------------------------------
 * Standard clock functions defined in include/linux/clk.h
 *-------------------------------------------------------------------------*/

/*
 * Returns a clock. Note that we first try to use device id on the bus
 * and clock name. If this fails, we try to use clock name only.
 */
struct clk * clk_get(struct device *dev, const char *id)
{
	struct clk *p;
#ifdef CONFIG_TRACK_RESOURCES
	short index = 0;
#else
	struct clk *clk = ERR_PTR(-ENOENT);
#endif
	int idno;

	if ((dev == NULL) || (dev->bus != &platform_bus_type))
		idno = -1;
	else
		idno = to_platform_device(dev)->id;

	mutex_lock(&clocks_mutex);

	list_for_each_entry(p, &clocks, node) {
		if ((p->id == idno) &&
			(strcmp(id, p->name) == 0) &&
			try_module_get(p->owner)) {
#ifdef CONFIG_TRACK_RESOURCES
			mutex_lock(&res_mutex);

			while (index < NUM_RES_HANDLES) {
				if (a[index] == 0)
					break;
				else
					index++;
			}
			a[index] = 1;

			res_han[index].clk = p;
			res_han[index].index = index;

			if (dev != NULL)
				res_han[index].dev = dev;
			list_add(&res_han[index].node1,
						&res_han[index].clk->clk_got);
			mutex_unlock(&res_mutex);
#else
			clk = p;
#endif
			goto found;
		}
	}

	list_for_each_entry(p, &clocks, node) {
		if ((strcmp(id, p->name) == 0) && (try_module_get(p->owner))) {
#ifdef CONFIG_TRACK_RESOURCES
			mutex_lock(&res_mutex);

			while (index < NUM_RES_HANDLES) {
				if (a[index] == 0)
					break;
				else
					index++;
			}
			a[index] = 1;

			res_han[index].clk = p;
			res_han[index].index = index;

			if (dev != NULL)
				res_han[index].dev = dev;

			list_add(&res_han[index].node1,
						&res_han[index].clk->clk_got);
			mutex_unlock(&res_mutex);
#else
			clk = p;
#endif
			break;
		}
	}

found:
	mutex_unlock(&clocks_mutex);

#ifdef CONFIG_TRACK_RESOURCES
	/*
	 * Return the clock object instead of faux address of the container
	 * array "res_han". This ensures consistency across all clients who
	 * are unaware of CONFIG_TRACK_RESOURCES - causing runtime failures.
	 */
	return (struct clk *) (res_han[index].clk) ;
#else
	return clk;
#endif
}

EXPORT_SYMBOL(clk_get);

int clk_enable(struct clk *clk)
{
	unsigned long flags;
	int ret = 0;
#ifdef CONFIG_TRACK_RESOURCES
	struct resource_handle *res = (struct resource_handle *) clk;
#endif

	if (R(clk) == NULL || IS_ERR(R(clk)))
		return -EINVAL;

	spin_lock_irqsave(&clockfw_lock, flags);
	if (arch_clock->clk_enable) {
		ret = arch_clock->clk_enable(R(clk));
#ifdef CONFIG_TRACK_RESOURCES
		list_add(&res->node2, &res->clk->clk_enabled);
#endif
	}
	spin_unlock_irqrestore(&clockfw_lock, flags);

	return ret;
}

EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
	unsigned long flags;
#ifdef CONFIG_TRACK_RESOURCES
	struct resource_handle *res = (struct resource_handle *) clk;
#endif

	if (R(clk) == NULL || IS_ERR(R(clk)))
		return;

	spin_lock_irqsave(&clockfw_lock, flags);
	if (R(clk)->usecount == 0) {
		printk(KERN_ERR "Trying disable clock %s with 0 usecount\n",
		       R(clk)->name);
		WARN_ON(1);
		goto out;
	}

	if (arch_clock->clk_disable) {
		arch_clock->clk_disable(R(clk));
#ifdef CONFIG_TRACK_RESOURCES
		list_del(&res->node2);
#endif
	}
out:
	spin_unlock_irqrestore(&clockfw_lock, flags);
}

EXPORT_SYMBOL(clk_disable);

int clk_get_usecount(struct clk *clk)
{
#ifdef CONFIG_TRACK_RESOURCES
	struct resource_handle *res;
#endif
	unsigned long flags;
	int ret = 0;

#ifdef CONFIG_TRACK_RESOURCES
	res = (struct resource_handle *) clk;
#endif

	if (R(clk) == NULL || IS_ERR(R(clk)))
		return 0;

	spin_lock_irqsave(&clockfw_lock, flags);
	ret = R(clk)->usecount;
	spin_unlock_irqrestore(&clockfw_lock, flags);

	return ret;
}
EXPORT_SYMBOL(clk_get_usecount);

unsigned long clk_get_rate(struct clk *clk)
{
#ifdef CONFIG_TRACK_RESOURCES
	struct resource_handle *res;
#endif
	unsigned long flags;
	unsigned long ret = 0;

#ifdef CONFIG_TRACK_RESOURCES
	res = (struct resource_handle *) clk;
#endif

	if (R(clk) == NULL || IS_ERR(R(clk)))
		return 0;

	spin_lock_irqsave(&clockfw_lock, flags);
	ret = R(clk)->rate;
	spin_unlock_irqrestore(&clockfw_lock, flags);

	return ret;
}
EXPORT_SYMBOL(clk_get_rate);

void clk_put(struct clk *clk)
{
#ifdef CONFIG_TRACK_RESOURCES
	struct resource_handle *res = (struct resource_handle *) clk;
#endif

	if (R(clk) && !IS_ERR(R(clk)))
		module_put(R(clk)->owner);

#ifdef CONFIG_TRACK_RESOURCES
	mutex_lock(&res_mutex);
	list_del(&res->node1);
	a[res->index] = 0;
	memset((struct resource_handle *) res, 0,
					sizeof(struct resource_handle));
	mutex_unlock(&res_mutex);
#endif
}
EXPORT_SYMBOL(clk_put);

/*-------------------------------------------------------------------------
 * Optional clock functions defined in include/linux/clk.h
 *-------------------------------------------------------------------------*/

long clk_round_rate(struct clk *clk, unsigned long rate)
{
#ifdef CONFIG_TRACK_RESOURCES
	struct resource_handle *res;
#endif
	unsigned long flags;
	long ret = 0;

#ifdef CONFIG_TRACK_RESOURCES
	res = (struct resource_handle *) clk;
#endif

	if (R(clk) == NULL || IS_ERR(R(clk)))
		return ret;

	spin_lock_irqsave(&clockfw_lock, flags);
	if (arch_clock->clk_round_rate)
		ret = arch_clock->clk_round_rate(R(clk), rate);
	spin_unlock_irqrestore(&clockfw_lock, flags);

	return ret;
}
EXPORT_SYMBOL(clk_round_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
#ifdef CONFIG_TRACK_RESOURCES
	struct resource_handle *res;
#endif
	unsigned long flags;
	int ret = -EINVAL;

#ifdef CONFIG_TRACK_RESOURCES
	res = (struct resource_handle *) clk;
#endif

	if (R(clk) == NULL || IS_ERR(R(clk)))
		return ret;

	spin_lock_irqsave(&clockfw_lock, flags);
	if (arch_clock->clk_set_rate)
		ret = arch_clock->clk_set_rate(R(clk), rate);
	spin_unlock_irqrestore(&clockfw_lock, flags);

	return ret;
}
EXPORT_SYMBOL(clk_set_rate);

int clk_set_parent(struct clk *clk, struct clk *parent)
{
#ifdef CONFIG_TRACK_RESOURCES
	struct resource_handle *res;
	struct resource_handle *resp;
#endif

	unsigned long flags;
	int ret = -EINVAL;

#ifdef CONFIG_TRACK_RESOURCES
	res = (struct resource_handle *) clk;
	resp = (struct resource_handle *) parent;
#endif

#ifdef CONFIG_TRACK_RESOURCES
	if (res->clk == NULL || IS_ERR(res->clk) || (resp->clk  == NULL)
							|| IS_ERR(resp->clk))
#else
	if (clk == NULL || IS_ERR(clk) || parent == NULL || IS_ERR(parent))
#endif
		return ret;

	spin_lock_irqsave(&clockfw_lock, flags);
	if (arch_clock->clk_set_parent)
#ifdef CONFIG_TRACK_RESOURCES
		ret =  arch_clock->clk_set_parent(res->clk, resp->clk);
#else
		ret =  arch_clock->clk_set_parent(clk, parent);
#endif
	spin_unlock_irqrestore(&clockfw_lock, flags);

	return ret;
}
EXPORT_SYMBOL(clk_set_parent);

struct clk *clk_get_parent(struct clk *clk)
{
#ifdef CONFIG_TRACK_RESOURCES
	struct resource_handle *res;
#endif
	unsigned long flags;
	struct clk * ret = NULL;

#ifdef CONFIG_TRACK_RESOURCES
	res = (struct resource_handle *) clk;
#endif

	if (R(clk) == NULL || IS_ERR(R(clk)))
		return ret;

	spin_lock_irqsave(&clockfw_lock, flags);
	if (arch_clock->clk_get_parent)
		ret = arch_clock->clk_get_parent(R(clk));
	spin_unlock_irqrestore(&clockfw_lock, flags);

	return ret;
}
EXPORT_SYMBOL(clk_get_parent);

/*-------------------------------------------------------------------------
 * OMAP specific clock functions shared between omap1 and omap2
 *-------------------------------------------------------------------------*/

unsigned int __initdata mpurate;

/*
 * By default we use the rate set by the bootloader.
 * You can override this with mpurate= cmdline option.
 */
static int __init omap_clk_setup(char *str)
{
	get_option(&str, &mpurate);

	if (!mpurate)
		return 1;

	if (mpurate < 1000)
		mpurate *= 1000000;

	return 1;
}
__setup("mpurate=", omap_clk_setup);

/* Used for clocks that always have same value as the parent clock */
void followparent_recalc(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return;

	clk->rate = clk->parent->rate;
#ifndef CONFIG_ARCH_OMAP34XX
	if (unlikely(clk->flags & RATE_PROPAGATES))
		propagate_rate(clk);
#endif
}

/* Propagate rate to children */
void propagate_rate(struct clk * tclk)
{
	struct clk *clkp;

	if (tclk == NULL || IS_ERR(tclk))
		return;

	list_for_each_entry(clkp, &clocks, node) {
		if (likely(clkp->parent != tclk))
			continue;
		if (likely((u32)clkp->recalc))
			clkp->recalc(clkp);
	}
}

/**
 * recalculate_root_clocks - recalculate and propagate all root clocks
 *
 * Recalculates all root clocks (clocks with no parent), which if the
 * clock's .recalc is set correctly, should also propagate their rates.
 * Called at init.
 */
void recalculate_root_clocks(void)
{
	struct clk *clkp;

	list_for_each_entry(clkp, &clocks, node) {
		if (unlikely(!clkp->parent) && likely((u32)clkp->recalc))
			clkp->recalc(clkp);
	}
}

int clk_register(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return -EINVAL;

	mutex_lock(&clocks_mutex);
	list_add(&clk->node, &clocks);
	if (clk->init)
		clk->init(clk);
	mutex_unlock(&clocks_mutex);

	return 0;
}
EXPORT_SYMBOL(clk_register);

void clk_unregister(struct clk *clk)
{
	if (clk == NULL || IS_ERR(clk))
		return;

	mutex_lock(&clocks_mutex);
	list_del(&clk->node);
	mutex_unlock(&clocks_mutex);
}
EXPORT_SYMBOL(clk_unregister);

void clk_deny_idle(struct clk *clk)
{
	unsigned long flags;

	if (clk == NULL || IS_ERR(clk))
		return;

	spin_lock_irqsave(&clockfw_lock, flags);
	if (arch_clock->clk_deny_idle)
		arch_clock->clk_deny_idle(clk);
	spin_unlock_irqrestore(&clockfw_lock, flags);
}
EXPORT_SYMBOL(clk_deny_idle);

void clk_allow_idle(struct clk *clk)
{
	unsigned long flags;

	if (clk == NULL || IS_ERR(clk))
		return;

	spin_lock_irqsave(&clockfw_lock, flags);
	if (arch_clock->clk_allow_idle)
		arch_clock->clk_allow_idle(clk);
	spin_unlock_irqrestore(&clockfw_lock, flags);
}
EXPORT_SYMBOL(clk_allow_idle);

#ifdef CONFIG_CPU_FREQ
void clk_init_cpufreq_table(struct cpufreq_frequency_table **table)
{
	unsigned long flags;

	spin_lock_irqsave(&clockfw_lock, flags);
	if (arch_clock->clk_init_cpufreq_table)
		arch_clock->clk_init_cpufreq_table(table);
	spin_unlock_irqrestore(&clockfw_lock, flags);
}
EXPORT_SYMBOL(clk_init_cpufreq_table);
#endif

void omap_show_domain_clks (u32 domainid)
{
	struct clk *cp;

	printk(KERN_INFO "Clocks for power domain %d\n", domainid);

	list_for_each_entry(cp, &clocks, node) {
		if (domainid != DOMAIN_ID(cp->prcmid))
			continue;
		printk(KERN_INFO "%s %d %ld %d 0x%08x %s\n", 
				cp->name, 
				cp->id, 
				cp->rate, 
				cp->usecount,
				cp->flags,
				(cp->flags & ALWAYS_ENABLED) ? "ALWAYS_ENABLED" : "");
	}
	return;
}

/*-------------------------------------------------------------------------*/

#ifdef CONFIG_OMAP_RESET_CLOCKS
/*
 * Disable any unused clocks left on by the bootloader
 */
static int __init clk_disable_unused(void)
{
	struct clk *ck;
	unsigned long flags;

	list_for_each_entry(ck, &clocks, node) {
#ifndef CONFIG_ARCH_OMAP34XX
		if (ck->usecount > 0 || (ck->flags & ALWAYS_ENABLED) ||
			ck->enable_reg == 0)
#else
		if (ck->usecount > 0 || (ck->flags & ALWAYS_ENABLED))
#endif
			continue;

		spin_lock_irqsave(&clockfw_lock, flags);
		if (arch_clock->clk_disable_unused)
			arch_clock->clk_disable_unused(ck);
		spin_unlock_irqrestore(&clockfw_lock, flags);
	}

	return 0;
}
late_initcall(clk_disable_unused);
#endif

int __init clk_init(struct clk_functions * custom_clocks)
{
	if (!custom_clocks) {
		printk(KERN_ERR "No custom clock functions registered\n");
		BUG();
	}

	arch_clock = custom_clocks;

	return 0;
}

/*#if defined(CONFIG_PM_DEBUG) && defined(CONFIG_PROC_FS)*/
#if defined(CONFIG_PROC_FS)
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static void *omap_ck_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *omap_ck_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

static void omap_ck_stop(struct seq_file *m, void *v)
{
}

int omap_ck_show(struct seq_file *m, void *v)
{
	struct clk *cp;
#if defined (CONFIG_ARCH_OMAP34XX)
	u32 idlest_lock_bit;
	volatile u32 *addr_idlest;
#endif
#ifdef CONFIG_TRACK_RESOURCES
	struct resource_handle *res;
#endif

	seq_printf(m, "%-20s %4s %10s %8s\n",
			"Clock", "ID", "Rate", "UseCount");
	seq_printf(m, "%-20s %4s %10s %8s\n",
			"====================", "====",
			"==========", "========");

	list_for_each_entry(cp, &clocks, node) {
#ifdef CONFIG_TRACK_RESOURCES
		seq_printf(m, "%s %d %ld %d\n", cp->name, cp->id, cp->rate,
								cp->usecount);

		list_for_each_entry(res, &(cp->clk_got), node1) {
			if (res->dev) {
				seq_printf(m, "\t%s %s\n",
				"Driver with handle: ", res->dev->driver->name);
			} else {
				seq_printf(m, "\t%s %s\n",
				"Driver with handle: ", "NULL");
			}
		}

		list_for_each_entry(res, &(cp->clk_enabled), node2) {
			if (res->dev) {
				seq_printf(m, "\t%s %s\n",
				"Driver which has enabled: ",
							res->dev->driver->name);
			} else {
				seq_printf(m, "\t%s %s\n",
				"Driver which has enabled: ", "NULL");
			}
		}
#else
#if defined (CONFIG_ARCH_OMAP34XX)
	/* Currently only the IVA2 clock is being checked for bypass mode.
	 * The same can be done for other clocks as well...
	 */
	if (cp->prcmid == PRCM_IVA2) {
		addr_idlest     = get_addr_pll(DPLL2_IVA2, REG_IDLEST_PLL);
		idlest_lock_bit = get_idlest_lock_bit(DPLL2_IVA2);

		if (*addr_idlest & idlest_lock_bit) {
			seq_printf(m,"%-20s %4d %10ld %8d\n",
				cp->name, cp->id, cp->rate, cp->usecount);
		}
		else {
			seq_printf(m,"%-20s %4d %10s %8d\n",
				cp->name, cp->id, "BYPASS", cp->usecount);
		}
	}
	else {
		seq_printf(m,"%-20s %4d %10ld %8d\n",
			cp->name, cp->id, cp->rate, cp->usecount);
	}
#else /* defined (CONFIG_ARCH_OMAP34XX) */
		seq_printf(m,"%-20s %4d %10ld %8d\n",
			cp->name, cp->id, cp->rate, cp->usecount);
#endif /* defined (CONFIG_ARCH_OMAP34XX) */
#endif
	}

	return 0;
}

static struct seq_operations omap_ck_op = {
	.start	= omap_ck_start,
	.next	= omap_ck_next,
	.stop	= omap_ck_stop,
	.show	= omap_ck_show
};

static int omap_ck_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &omap_ck_op);
}

static struct file_operations proc_omap_ck_operations = {
	.open	= omap_ck_open,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.release= seq_release,
};

int __init omap_ck_init(void)
{
	struct proc_dir_entry *entry;

	entry = create_proc_entry("omap_clocks", 0, NULL);
	if (entry)
		entry->proc_fops = &proc_omap_ck_operations;

	return 0;

}

__initcall(omap_ck_init);
#endif
