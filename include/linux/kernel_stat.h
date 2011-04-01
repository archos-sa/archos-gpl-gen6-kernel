#ifndef _LINUX_KERNEL_STAT_H
#define _LINUX_KERNEL_STAT_H

#include <asm/irq.h>
#include <linux/smp.h>
#include <linux/threads.h>
#include <linux/percpu.h>
#include <linux/cpumask.h>
#include <asm/cputime.h>
#include <linux/hrtimer.h>

/*
 * 'kernel_stat.h' contains the definitions needed for doing
 * some kernel statistics (CPU usage, context switches ...),
 * used by rstatd/perfmeter
 */

struct cpu_usage_stat {
	cputime64_t user;
	cputime64_t nice;
	cputime64_t system;
	cputime64_t softirq;
	cputime64_t irq;
	cputime64_t idle;
	cputime64_t iowait;
	cputime64_t steal;
};

struct kernel_stat {
	struct cpu_usage_stat	cpustat;
	unsigned int irqs[NR_IRQS];
	unsigned long long total_irqs;
	ktime_t irq_now;
	ktime_t irq_last;
};

DECLARE_PER_CPU(struct kernel_stat, kstat);

#define kstat_cpu(cpu)	per_cpu(kstat, cpu)
/* Must have preemption disabled for this to be meaningful. */
#define kstat_this_cpu	__get_cpu_var(kstat)

#ifdef CONFIG_ARCH_OMAP
static inline void inc_total_irqs(int cpu) {kstat_cpu(cpu).total_irqs++;}
#else
#define inc_total_irqs(cpu) { }
#endif /* defined (ARCH_OMAP) */


extern unsigned long long nr_context_switches(void);

/*
 * Number of interrupts per specific IRQ source, since bootup
 */
static inline int kstat_irqs(int irq)
{
	int cpu, sum = 0;

	for_each_possible_cpu(cpu)
		sum += kstat_cpu(cpu).irqs[irq];

	return sum;
}

/*
 * Provide hook to try and understand interrupt rate on a processor
 */
static inline void kstat_irq_stamp(void)
{
	const unsigned int cpu = smp_processor_id();
	kstat_cpu(cpu).irq_last = kstat_cpu(cpu).irq_now;
	kstat_cpu(cpu).irq_now = ktime_get();
}

extern void account_user_time(struct task_struct *, cputime_t);
extern void account_system_time(struct task_struct *, int, cputime_t);
extern void account_steal_time(struct task_struct *, cputime_t);

#endif /* _LINUX_KERNEL_STAT_H */
