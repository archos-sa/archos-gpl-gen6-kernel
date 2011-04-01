/*
 * linux/arch/arm/mach-omap3/resource.c
 * OMAP34XX Shared Resource Framework
 *
 * Copyright (C) 2006-2007 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 * History:
 *
 */

#include <linux/errno.h>
#include <linux/err.h>
#include <linux/notifier.h>
#include <asm/arch/resource.h>
#include <linux/latency.h>
#include <asm/arch/clock.h>
#include "resource_34xx.h"
#include "prcm-regs.h"

#ifdef CONFIG_OMAP34XX_OFFMODE
#include <linux/timer.h>
#include <linux/sysfs.h>
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */

static	DECLARE_MUTEX(res_handle_mutex);
static	DECLARE_MUTEX(users_list_mutex);
/* Pool of resource handles */
short	handle_flag[MAX_HANDLES];
struct	res_handle handle_list[MAX_HANDLES];

/* Pool of users for a resource */
short	usr_flag[MAX_USERS];
struct	users_list usr_list[MAX_USERS];

/* Pool of linked resource handles */
struct res_handle_node linked_hlist[MAX_HANDLES];

/* global variables which can be used in idle_thread */
short	core_active = LOGICAL_UNUSED;
struct res_handle *vdd1_res;
struct res_handle *vdd1_opp_co;
struct res_handle *vdd2_opp_co;
unsigned int vdd1_users;
unsigned int vdd1_arm_prenotifications;
unsigned int vdd1_arm_postnotifications;
unsigned int vdd1_dsp_prenotifications;
unsigned int vdd1_dsp_postnotifications;
struct res_handle *vdd2_res;
struct atomic_notifier_head freq_arm_pre_notifier_list;
struct atomic_notifier_head freq_arm_post_notifier_list;
struct atomic_notifier_head freq_dsp_pre_notifier_list;
struct atomic_notifier_head freq_dsp_post_notifier_list;

u32 valid_rate;

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
#define S166M	166000000
#define S83M	83000000
#define S66M    66500000
#define S133M   133000000
#define S266M   266000000
#define S293M   293000000
#define S320M   320000000

#define down_srfmutex(x)	if(!(in_atomic() || irqs_disabled())) \
					down(x)
#define up_srfmutex(x)		if(!(in_atomic() || irqs_disabled())) \
					up(x)

#ifdef CONFIG_OMAP3430_ES2
#ifdef CONFIG_ARCH_OMAP3410
unsigned int vdd1_arm_dsp_freq[5][4] = {
	{66, 66, CO_VDD1_OPP1, PRCM_VDD1_OPP1},
	{133, 133, CO_VDD1_OPP2, PRCM_VDD1_OPP2},
	{266, 266, CO_VDD1_OPP3, PRCM_VDD1_OPP3},
	{293, 293, CO_VDD1_OPP4, PRCM_VDD1_OPP4},
	{320, 320, CO_VDD1_OPP5, PRCM_VDD1_OPP5},
};
#else
unsigned int vdd1_arm_dsp_freq[5][4] = {
	{125, 90, CO_VDD1_OPP1, PRCM_VDD1_OPP1},
	{250, 180, CO_VDD1_OPP2, PRCM_VDD1_OPP2},
	{500, 360, CO_VDD1_OPP3, PRCM_VDD1_OPP3},
	{550, 396, CO_VDD1_OPP4, PRCM_VDD1_OPP4},
	{600, 430, CO_VDD1_OPP5, PRCM_VDD1_OPP5},
};
#endif

unsigned int vdd2_core_freq[3][3] = {
	{0, CO_VDD2_OPP1, PRCM_VDD2_OPP1},
	{83, CO_VDD2_OPP2, PRCM_VDD2_OPP2},
	{166, CO_VDD2_OPP3, PRCM_VDD2_OPP3},
};

#define scale_volt_then_freq  (cur_opp_no < target_opp_no)
#ifdef CONFIG_ARCH_OMAP3410
unsigned int rnd_rate_vdd1[5] = {
	 S66M, S133M, S266M, S293M, S320M
};
#else
unsigned int rnd_rate_vdd1[5] = {
	S125M, S250M, S500M, S550M, S600M
};
#endif
unsigned int rnd_rate_vdd2[3] = {
	0, S83M, S166M
};

#define request_vdd2_co	(target_value > CO_VDD1_OPP2)
#define release_vdd2_co	(target_value <= CO_VDD1_OPP2)
#define vdd2_co_val	CO_VDD2_OPP3
#else
unsigned int vdd1_arm_dsp_freq[4][4] = {
	{95, 62, CO_VDD1_OPP4, PRCM_VDD1_OPP4},
	{190, 125, CO_VDD1_OPP3, PRCM_VDD1_OPP3},
	{381, 250, CO_VDD1_OPP2, PRCM_VDD1_OPP2},
	{477, 313, CO_VDD1_OPP1, PRCM_VDD1_OPP1},
};

unsigned int vdd2_core_freq[2][3] = {
	{83, CO_VDD2_OPP2, PRCM_VDD2_OPP2},
	{166, CO_VDD2_OPP1, PRCM_VDD2_OPP1},
};

#define scale_volt_then_freq  (cur_opp_no > target_opp_no)
unsigned int rnd_rate_vdd1[4] = {
	S477M, S381M, S191M, S96M
};
unsigned int rnd_rate_vdd2[2] = {
	S166M, S83M
};

#define request_vdd2_co	(target_value < CO_VDD1_OPP3)
#define release_vdd2_co	(target_value >= CO_VDD1_OPP3)
#define vdd2_co_val	CO_VDD2_OPP1
#endif

/* WARNING: id_to_* has to be in sync with domains numbering in prcm.h */
static char *id_to_lname[] = {"lat_iva2", "lat_mpu", "lat_core1", "lat_core1",
			     "lat_3d", NULL, "lat_dss", "lat_cam", "lat_per",
#ifndef CONFIG_ARCH_OMAP3410
							NULL, "lat_neon",
#else
				NULL, NULL,
#endif
#ifdef CONFIG_OMAP3430_ES2
			      "lat_core1", "lat_usb",
#endif
};

int nb_arm_freq_prenotify_func(struct notifier_block *n, unsigned long event,
					void *ptr)
{
	atomic_notifier_call_chain(&freq_arm_pre_notifier_list,
		vdd1_arm_dsp_freq[event-1][0], NULL);
	return 0;
}

int nb_arm_freq_postnotify_func(struct notifier_block *n, unsigned long event,
					void *ptr)
{
	atomic_notifier_call_chain(&freq_arm_post_notifier_list,
		vdd1_arm_dsp_freq[event-1][0], NULL);
	return 0;
}

int nb_dsp_freq_prenotify_func(struct notifier_block *n, unsigned long event,
					void *ptr)
{
	atomic_notifier_call_chain(&freq_dsp_pre_notifier_list,
		vdd1_arm_dsp_freq[event-1][1], NULL);
	return 0;
}

int nb_dsp_freq_postnotify_func(struct notifier_block *n, unsigned long event,
					void *ptr)
{
	atomic_notifier_call_chain(&freq_dsp_post_notifier_list,
		vdd1_arm_dsp_freq[event-1][1], NULL);
	return 0;
}

static struct notifier_block nb_arm_freq_prenotify = {
	nb_arm_freq_prenotify_func,
	NULL,
};

static struct notifier_block nb_arm_freq_postnotify = {
	nb_arm_freq_postnotify_func,
	NULL,
};

static struct notifier_block nb_dsp_freq_prenotify = {
	nb_dsp_freq_prenotify_func,
	NULL,
};

static struct notifier_block nb_dsp_freq_postnotify = {
	nb_dsp_freq_postnotify_func,
	NULL,
};

/* To set the opp for vdd1 */
unsigned int vdd1_opp_setting(u32 target_opp_no)
{
	unsigned int cur_opp_no, target_vdd1_opp;

	target_vdd1_opp = vdd1_arm_dsp_freq[target_opp_no-1][3];
	cur_opp_no = get_opp_no(current_vdd1_opp);

	if (p_vdd1_clk == NULL)
		p_vdd1_clk = clk_get(NULL, "virt_vdd1_prcm_set");

	if (scale_volt_then_freq) {
		prcm_do_voltage_scaling(target_vdd1_opp, current_vdd1_opp);
		valid_rate = clk_round_rate(p_vdd1_clk,
			rnd_rate_vdd1[target_opp_no-1]);
		p_vdd1_clk->set_rate(p_vdd1_clk, valid_rate);
	} else {
		valid_rate = clk_round_rate(p_vdd1_clk,
			rnd_rate_vdd1[target_opp_no-1]);
		p_vdd1_clk->set_rate(p_vdd1_clk, valid_rate);
		prcm_do_voltage_scaling(target_vdd1_opp, current_vdd1_opp);
	}
	return target_vdd1_opp;
}

/* To set the opp value for vdd2 */
unsigned int vdd2_opp_setting(u32 target_opp_no)
{
	unsigned int cur_opp_no, target_vdd2_opp;

	target_vdd2_opp = vdd2_core_freq[target_opp_no-1][2];
	cur_opp_no = get_opp_no(current_vdd2_opp);

	if (p_vdd2_clk == NULL)
		p_vdd2_clk = clk_get(NULL, "virt_vdd2_prcm_set");

	if (scale_volt_then_freq) {
		prcm_do_voltage_scaling(target_vdd2_opp, current_vdd2_opp);
		valid_rate = clk_round_rate(p_vdd2_clk,
			rnd_rate_vdd2[target_opp_no-1]);
		p_vdd2_clk->set_rate(p_vdd2_clk, valid_rate);
	} else {
		valid_rate = clk_round_rate(p_vdd2_clk,
			rnd_rate_vdd2[target_opp_no-1]);
		p_vdd2_clk->set_rate(p_vdd2_clk, valid_rate);
		prcm_do_voltage_scaling(target_vdd2_opp, current_vdd2_opp);
	}
	return target_vdd2_opp;
}

/* #define DEBUG_RES_FRWK 1 */
#ifdef DEBUG_RES_FRWK
#define DPRINTK(fmt, args...)\
 printk(KERN_ERR "%s: " fmt, __FUNCTION__ , ## args)
#else
#  define DPRINTK(fmt, args...)
#endif

#ifdef CONFIG_OMAP34XX_OFFMODE

static  spinlock_t svres_reg_lock;

/* An array of struct which contains the context attributes for each domain. */
static struct domain_ctxsvres_status domain_ctxsvres[PRCM_NUM_DOMAINS];

/*  This function is called by the DeviceDriver when it has done context save.
 */
void context_save_done(struct clk *clk)
{
	struct domain_ctxsvres_status *ctxsvres;
	u32 prcm_id, domain_id, device_id, device_bitpos;

	prcm_id = clk->prcmid;
	domain_id = DOMAIN_ID(prcm_id);
	device_id = DEV_BIT_POS(prcm_id);
	device_bitpos = 1 << device_id;

	spin_lock(&svres_reg_lock);
	ctxsvres = &domain_ctxsvres[domain_id];

	ctxsvres->context_saved |= device_bitpos;
	spin_unlock(&svres_reg_lock);
}

/*  This function is called by the DeviceDriver to check whether context restore
 *  is required.
 */
int context_restore_required(struct clk *clk)
{
	struct domain_ctxsvres_status *ctxsvres;
	u32 prcm_id, domain_id, device_id, device_bitpos;
	int ret;

	ret = 0;
#ifdef CONFIG_TRACK_RESOURCES
	struct resource_handle *res = (struct resource_handle *) clk;
	prcm_id = res->clk->prcmid;
#else
	prcm_id = clk->prcmid;
#endif
	domain_id = DOMAIN_ID(prcm_id);
	device_id = DEV_BIT_POS(prcm_id);
	device_bitpos = 1 << device_id;

	spin_lock(&svres_reg_lock);
	ctxsvres = &domain_ctxsvres[domain_id];

	if (ctxsvres->context_restore & device_bitpos) {
		ret = 1;
		ctxsvres->context_saved &= ~device_bitpos;
		ctxsvres->context_restore &= ~device_bitpos;
	}
	spin_unlock(&svres_reg_lock);

	return ret;
}
EXPORT_SYMBOL(context_restore_required);

/*  This function is called to update the context attributes when power domain
 *  is put to OFF.
 */
void context_restore_update(unsigned long domain_id)
{
	struct domain_ctxsvres_status *ctxsvres;

	spin_lock(&svres_reg_lock);
	ctxsvres = &domain_ctxsvres[domain_id];
	ctxsvres->context_restore = ctxsvres->context_saved;

	if (domain_id == DOM_CORE1) {
		ctxsvres = &domain_ctxsvres[DOM_CORE2];
		ctxsvres->context_restore = ctxsvres->context_saved;
#ifdef CONFIG_OMAP3430_ES2 
		ctxsvres = &domain_ctxsvres[DOM_CORE3];
		ctxsvres->context_restore = ctxsvres->context_saved;
#endif
	}
	spin_unlock(&svres_reg_lock);
}
EXPORT_SYMBOL(context_restore_update);

#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */

/* This is the initializtion function for Shared resource framework. */
/* Initializes all the members of res_list. Always returns 0.	     */
int resource_init(void)
{
	struct shared_resource **resp;
	int ind;
	int i;
	int linked_cnt = 0;

#ifdef CONFIG_OMAP34XX_OFFMODE
	spin_lock_init(&svres_reg_lock);
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */

	DPRINTK("Initializing Shared Resource Framework\n");

	/* Init the res_handle POOL */
	for (ind = 0; ind < MAX_HANDLES; ind++) {
		handle_flag[ind] = UNUSED;
		handle_list[ind].res = NULL;
		handle_list[ind].usr_name = NULL;
		handle_list[ind].res_index = ind;
		handle_list[ind].usr_index = -1;
	}

	/* Init the users_list POOL */
	for (ind = 0; ind < MAX_USERS; ind++) {
		usr_flag[ind] = UNUSED;
		usr_list[ind].usr_name = NULL;
		usr_list[ind].level = DEFAULT_LEVEL;
		usr_list[ind].index = ind;
	}

	/* Init all the resources in  res_list */
	for (resp = res_list; resp < res_list + ARRAY_SIZE(res_list);
	     resp++) {
		INIT_LIST_HEAD(&(*resp)->users_list);
		init_MUTEX(&(*resp)->res_action_sem);
		INIT_LIST_HEAD(&(*resp)->linked_res_handles);

		/*
		 * latency resources have to follow domain resources in
		 * res_list!
		 */
		if (strncmp((*resp)->name, "lat_", 4) == 0) {
			for (i = 0; i < (*resp)->linked_res_num; i++) {
				if (linked_cnt >= MAX_HANDLES)
					goto res_init_fail;
				linked_hlist[linked_cnt].handle =
						resource_get((*resp)->name,
						(*resp)->linked_res_names[i]);

				if (IS_ERR(linked_hlist[linked_cnt].handle))
						goto res_init_fail;

				list_add(&linked_hlist[linked_cnt].node,
						 &(*resp)->linked_res_handles);
				linked_cnt++;
			}
		}
	}
	
	/* Setup the correct polarity for sysoffmode */
	/* Should be done in prcm_init but being done here since
	* there are sequencing issues
	*/
#ifdef CONFIG_MACH_ARCHOS_G6
	/* Archos S.A. : on Gen6, sys_off_mode is asserted HIGH */
	PRM_POLCTRL |= PRM_POL_SYSOFFMODE;
#else
	PRM_POLCTRL &= ~PRM_POL_SYSOFFMODE;
#endif
	return 0;

res_init_fail:
	/* TODO: free all memory and resources */
	return -1;
}

/* Returns a res_handle structure to the caller, which has a resource*/
/* structure and a name which is the clock name embedded in it. The clock */
/* name is embedded to track futher calls by the same device.*/
struct res_handle *resource_get(const char *name, const char *id)
{
	struct shared_resource **resp;
	short index = 0;
	struct res_handle *res = ERR_PTR(-ENOENT);

	if (name == NULL || id == NULL) {
		printk(KERN_ERR "resource_get: Invalid pointer\n");
		return res;
		}
	down_srfmutex(&res_handle_mutex);
	DPRINTK("resource_get for %s Clock-name %s\n", id, name);
	for (resp = res_list; resp < res_list + ARRAY_SIZE(res_list);
		resp++) {
		if (strcmp((*resp)->name, id) == 0) {
			/* look for a free handle from the handle_list POOL */
			while (index < MAX_HANDLES) {
				if (handle_flag[index] == UNUSED)
					break;
				else
					index++;
			}
			if (index >= MAX_HANDLES) {
				/* No free handle available */
				panic("FATAL ERROR: res_handle POOL EMPTY\n");
			}
			handle_flag[index] = USED;
			handle_list[index].res = (*resp);
			handle_list[index].usr_name = name;
			handle_list[index].res_index = index;
			DPRINTK("Returning the handle for %s, index = %d\n",
				id, index);
			res = &handle_list[index];
		}
	}
	up_srfmutex(&res_handle_mutex);
	return res;
}

/* Adds the request to the list of requests for the given resource.*/
/*Recalulates the target level to be set for the resource and updates */
/*it if not same as the current level. Also calls notification functions */
/*for registered users to notify the target level change		*/
int resource_request(struct res_handle *res, unsigned short target_level)
{
	struct 	shared_resource *resp;
	struct 	users_list *user, *cur_user = NULL;
	short 	index = 0;
	int	ret = -1;

	if (res == ERR_PTR(-ENOENT)) {
		DPRINTK("Invalid resource handle passed to reource_request\n");
		return ret;
	}

	resp = res->res;

	DPRINTK("resource_request: Clock-name %s\n", res->usr_name);

	down_srfmutex(&users_list_mutex);

	if (res->usr_index != -1) {
		cur_user = &usr_list[res->usr_index];
		ret = resp->validate(resp, cur_user->level, target_level);
		if (ret) {
			DPRINTK("Validation failed\n");
			ret = -EINVAL;
			goto ret;
		}
		DPRINTK("Updating resource level for %s, to %d\n",
			resp->name, target_level);
		cur_user->level = target_level;
		}
	else {
		ret = resp->validate(resp, DEFAULT_LEVEL, target_level);
		if (ret) {
			DPRINTK("Validation failed\n");
			ret = -EINVAL;
			goto ret;
		}
		DPRINTK("Adding resource level for %s, to %d\n",
			resp->name, target_level);
		while (index < MAX_USERS) {
			if (usr_flag[index] == UNUSED)
				break;
			else
				index++;
		}
		if (index >= MAX_USERS)
			panic("FATAL ERROR: users_list POOL EMPTY\n");

		usr_flag[index] = USED;
		usr_list[index].usr_name = res->usr_name;
		usr_list[index].level = target_level;
		usr_list[index].index = index;
		res->usr_index = index;
		list_add(&usr_list[index].node, &resp->users_list);
		cur_user = &usr_list[index];
	}

	if (target_level == resp->curr_level)
		goto ret;

	/* Regenerate the target_level for the resource */
	target_level = DEFAULT_LEVEL;
	list_for_each_entry(user, &(resp->users_list), node) {
		if (user->level > target_level)
			target_level = user->level;
	}
	DPRINTK("New Target level is %d\n", target_level);
	up_srfmutex(&users_list_mutex);

	if (target_level != resp->curr_level) {
		DPRINTK("Changing Level for resource %s to %d\n",
			resp->name, target_level);
		if ((resp->res_type != RES_FREQ_CO) &&
				(target_level < MAX_LEVEL)) {
			/* Call the notification functions */
			/* Currently not supported for Frequency constraints */
			atomic_notifier_call_chain(
				&resp->pre_notifier_list[target_level],
							target_level, NULL);
		}
		down_srfmutex(&resp->res_action_sem);
		ret = resp->action(resp, resp->curr_level, target_level);
		up_srfmutex(&resp->res_action_sem);
		if (ret)
			panic("FATAL ERROR: Unable to Change\
						level for resource %s to %d\n",
				resp->name, target_level);
		else
			/* If successful, change the resource curr_level */
			resp->curr_level = target_level;
		if ((resp->res_type != RES_FREQ_CO) &&
				(target_level < MAX_LEVEL))
			atomic_notifier_call_chain(
				&resp->post_notifier_list[target_level],
						target_level, NULL);
		}

	else {
		/* return success */
		ret = 0;
	}

	/* Increment the number of users for this domain. */
	resp->no_of_users++;
	return ret;
ret:
	up_srfmutex(&users_list_mutex);
	return ret;
}

/* Deletes the request from the list of request for the given resource.*/
/* Recalulates the target level to be set for the resource and updates it */
/* if not same as the current level.Also calls notification functions for */
/* registered users to notify the target level change			*/
int resource_release(struct res_handle *res)
{
	struct  shared_resource *resp;
	struct  users_list *user, *cur_user = NULL;
	unsigned short target_level;
	int ret = 0;

	if (res == ERR_PTR(-ENOENT)) {
		DPRINTK("Invalid resource handle passed to reource_release\n");
		return ret;
	}

	resp = res->res;
	DPRINTK("resource_request: Clock-name %s\n", res->usr_name);
	down_srfmutex(&users_list_mutex);
	if (res->usr_index != -1)
		cur_user = &usr_list[res->usr_index];
	else
		goto ret;

	ret = resp->validate(resp, cur_user->level, DEFAULT_LEVEL);
	if (ret) {
		DPRINTK("Validation failed\n");
		ret = -EINVAL;
		goto ret;

	}
	/* Delete the resource */
	DPRINTK("Deleting resource for %s\n", resp->name);
	list_del(&cur_user->node);
	usr_flag[cur_user->index] = UNUSED;
	usr_list[cur_user->index].usr_name = NULL;
	usr_list[cur_user->index].level = DEFAULT_LEVEL;
	usr_list[cur_user->index].index = cur_user->index;
	res->usr_index = -1;

	/* Regenerate the target_level for the resource */
	target_level = DEFAULT_LEVEL;
	list_for_each_entry(user, &(resp->users_list), node) {
		if (user->level > target_level)
			target_level = user->level;
		}

	DPRINTK("New Target level is %d\n", target_level);
	up_srfmutex(&users_list_mutex);

	if ((target_level == DEFAULT_LEVEL) &&
			(resp->prcm_id == PRCM_VDD1_CONSTRAINT))
		target_level = CO_VDD1_OPP1;
	else if ((target_level == DEFAULT_LEVEL) &&
			 (resp->prcm_id == PRCM_VDD2_CONSTRAINT))
		target_level = CO_VDD2_OPP2;
	else if ((target_level == DEFAULT_LEVEL) &&
			(resp->prcm_id == PRCM_ARMFREQ_CONSTRAINT))
		target_level = min_arm_freq;
	else if ((target_level == DEFAULT_LEVEL) &&
			(resp->prcm_id == PRCM_DSPFREQ_CONSTRAINT))
		target_level = min_dsp_freq;

	if (target_level != resp->curr_level) {
		DPRINTK("Changing Level for resource %s to %d\n",
			resp->name, target_level);
		if ((resp->res_type != RES_FREQ_CO) &&\
			(target_level < MAX_LEVEL))
			atomic_notifier_call_chain(
				&resp->pre_notifier_list[target_level],
				target_level, NULL);
		down_srfmutex(&resp->res_action_sem);
		ret = resp->action(resp, resp->curr_level, target_level);
		up_srfmutex(&resp->res_action_sem);
		if (ret)
			printk(KERN_ERR "FATAL ERROR: Unable to Change "
					"level for resource %s to %d\n",
						resp->name, target_level);
		else
			/* If successful, change the resource curr_level */
			resp->curr_level = target_level;
		if ((resp->res_type != RES_FREQ_CO) &&
				(target_level < MAX_LEVEL))
			atomic_notifier_call_chain(
				&resp->post_notifier_list[target_level],
				target_level, NULL);
	}

	/* Decrement the number of users for this domain. */
	resp->no_of_users--;
	return ret;
ret:
	up_srfmutex(&users_list_mutex);
	return ret;
}

/* Frees the res_handle structure from the pool */
void resource_put(struct res_handle *res)
{
	if (res == ERR_PTR(-ENOENT)) {
		DPRINTK("Invalid resource handle passed to resource_put\n");
		return;
	}

	if (res->usr_index != -1) {
		printk(KERN_ERR "resource_put called before "
				"resource_release\n");
		return;
	}
	down_srfmutex(&res_handle_mutex);
	DPRINTK("resource_put for %s, index = %d\n",
			res->res->name, res->res_index);
	handle_flag[res->res_index] = UNUSED;
	handle_list[res->res_index].res = NULL;
	handle_list[res->res_index].usr_name = NULL;
	handle_list[res->res_index].res_index = res->res_index;
	handle_list[res->res_index].usr_index = -1;
	up_srfmutex(&res_handle_mutex);
}


/* Registers a notification function from a resource user for a specific  */
/* target level. The function is called before a level change for the     */
/* resource to the target level.					  */
void resource_register_pre_notification(struct res_handle *res,
			struct notifier_block *nb, unsigned short target_level)
{
	struct shared_resource *resp;
	int ind;

	resp = res->res;
	DPRINTK("Notification registered for %s, level %d\n",
		resp->name, target_level);
	if (target_level == resp->max_levels) {
		for (ind = DEFAULT_LEVEL; ind < resp->max_levels; ind++) {
			res->nb_internal_pre[ind] =
				(struct notifier_block *)kmalloc(sizeof(struct notifier_block), GFP_KERNEL);
			*(res->nb_internal_pre[ind]) = *nb;
			atomic_notifier_chain_register(
				&resp->pre_notifier_list[ind],
				res->nb_internal_pre[ind]);
		}
		}
	else {
		res->nb_internal_pre[target_level] =
			(struct notifier_block *)
			kmalloc(sizeof(struct notifier_block), GFP_KERNEL);
		*(res->nb_internal_pre[target_level]) = *nb;
		atomic_notifier_chain_register(
				&resp->pre_notifier_list[target_level],
					res->nb_internal_pre[target_level]);
	}
}


/* Registers a notification function from a resource user for a specific  */
/* target level. The function is called before a level change for the     */
/* resource to the target level.					  */
void resource_register_post_notification(struct res_handle *res,
			struct notifier_block *nb, unsigned short target_level)
{
	struct shared_resource *resp;
	int ind;

	resp = res->res;
	DPRINTK("Notification registered for %s, level %d\n",
		resp->name, target_level);
	if (target_level == resp->max_levels) {
		for (ind = DEFAULT_LEVEL; ind < resp->max_levels; ind++) {
			res->nb_internal_post[ind] =
				(struct notifier_block *)
				kmalloc(sizeof(struct notifier_block), GFP_KERNEL);
			*(res->nb_internal_post[ind]) = *nb;
			atomic_notifier_chain_register(
				&resp->post_notifier_list[ind],
				res->nb_internal_post[ind]);
		}
		}
	else {
		res->nb_internal_post[target_level] =
			(struct notifier_block *)
			kmalloc(sizeof(struct notifier_block), GFP_KERNEL);
		*(res->nb_internal_post[target_level]) = *nb;
		atomic_notifier_chain_register(
			&resp->post_notifier_list[target_level],
			res->nb_internal_post[target_level]);
	}
}


/* Unregisters the notification function from a resource user for a specific  */
/* target level.    							      */
void resource_unregister_pre_notification(struct res_handle *res,
			struct notifier_block *nb, unsigned short target_level)
{
	struct shared_resource *resp;
	int ind, ret;

	resp = res->res;
	DPRINTK("Notification unregistered for %s, level %d\n",
		resp->name, target_level);
	if (target_level == resp->max_levels) {
		for (ind = DEFAULT_LEVEL; ind < resp->max_levels; ind++) {
			ret = atomic_notifier_chain_unregister(
				&resp->pre_notifier_list[ind],
				res->nb_internal_pre[ind]);
			if (ret != -ENOENT)
				kfree(res->nb_internal_pre[ind]);
		}
	} else {
		ret = atomic_notifier_chain_unregister(
			&resp->pre_notifier_list[target_level],
			res->nb_internal_pre[target_level]);
		if (ret != -ENOENT)
			kfree(res->nb_internal_pre[target_level]);
	}
}


/* Unregisters the notification function from a resource user for a specific  */
/* target level.    							      */
void resource_unregister_post_notification(struct res_handle *res,
			struct notifier_block *nb, unsigned short target_level)
{
	struct shared_resource *resp;
	int ind, ret;

	resp = res->res;
	DPRINTK("Notification unregistered for %s, level %d\n",
		resp->name, target_level);
	if (target_level == resp->max_levels) {
		for (ind = DEFAULT_LEVEL; ind < resp->max_levels; ind++) {
			ret = atomic_notifier_chain_unregister(
				&resp->post_notifier_list[ind],
				res->nb_internal_post[ind]);
			if (ret != -ENOENT)
				kfree(res->nb_internal_post[ind]);
		}
	} else {
		ret = atomic_notifier_chain_unregister(
				&resp->post_notifier_list[target_level],
				res->nb_internal_post[target_level]);
		if (ret != -ENOENT)
			kfree(res->nb_internal_post[target_level]);
		}
}

/* Returns the current level for a resource */
int resource_get_level(struct res_handle *res)
{
	struct shared_resource *resp;
	int ret;

	resp = res->res;
	down_srfmutex(&res_handle_mutex);
	ret = resp->curr_level;
	up_srfmutex(&res_handle_mutex);
	return ret;
}

/* Activation function to change target level for a Power Domain resource */
/* Transtitions from RET to OFF and OFF to RET are not allowed 		  */
int activate_power_res(struct shared_resource *resp,
		       unsigned short current_level,
		       unsigned short target_level)
{
	int ret = 0;
	unsigned long prcm_id = resp->prcm_id;

	/* Common code for transitioning to RET/OFF. */
	switch (target_level) {
	case POWER_DOMAIN_RET:
	case POWER_DOMAIN_OFF:
#ifdef CONFIG_HW_SUP_TRANS
		ret = prcm_set_clock_domain_state(prcm_id, PRCM_NO_AUTO,
								PRCM_FALSE);
		if (ret != PRCM_PASS) {
			printk("FAILED in clock domain to NO_AUTO for"
						" domain %lu \n", prcm_id);
			return -1;
		}

		switch (prcm_id) {
		case DOM_DSS:
		case DOM_CAM:
#ifdef CONFIG_OMAP34XX_OFFMODE
		/*case DOM_PER: */
#else
		case DOM_PER:
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */
#ifdef CONFIG_OMAP3430_ES2
		case DOM_USBHOST:
#ifdef CONFIG_ARCH_HAS_OMAP3_SGX
		case DOM_SGX:
#endif
#else
		case DOM_GFX:
#endif /* #ifdef CONFIG_OMAP3430_ES2 */
			prcm_clear_sleep_dependency(prcm_id,
							PRCM_SLEEPDEP_EN_MPU);
			prcm_clear_wkup_dependency(prcm_id, PRCM_WKDEP_EN_MPU);
			break;
		}
#endif /* #ifdef CONFIG_HW_SUP_TRANS */
		if (current_level != POWER_DOMAIN_ON) {
			ret = prcm_set_power_domain_state(prcm_id,
						POWER_DOMAIN_ON, PRCM_FORCE);
			if (ret != PRCM_PASS)
				return ret;
		}
		break;
	}
	switch (target_level) {
	case POWER_DOMAIN_ON:

			ret = prcm_set_power_domain_state(prcm_id,
						POWER_DOMAIN_ON, PRCM_FORCE);
#ifdef CONFIG_HW_SUP_TRANS
		switch (prcm_id) {
		case DOM_DSS:
		case DOM_CAM:
		case DOM_PER:
#ifdef CONFIG_OMAP3430_ES2
		case DOM_USBHOST:
#ifdef CONFIG_ARCH_HAS_OMAP3_SGX
		case DOM_SGX:
#endif
#else
		case DOM_GFX:
#endif /* #ifdef CONFIG_OMAP3430_ES2 */

			if (prcm_set_wkup_dependency(prcm_id,
					PRCM_WKDEP_EN_MPU) != PRCM_PASS) {
				printk("Domain %lu : wakeup dependency could"
						 " not be set\n", prcm_id);
				return -1;
			}
			if (prcm_set_sleep_dependency(prcm_id,
					PRCM_SLEEPDEP_EN_MPU) != PRCM_PASS) {
				printk("Domain %lu : sleep dependency could"
					" not be set\n", prcm_id);
				return -1;
			}
			/* NO BREAK */
#ifndef CONFIG_ARCH_OMAP3410
		case DOM_NEON:
#endif
		case DOM_IVA2:
			ret = prcm_set_power_domain_state(prcm_id,
						POWER_DOMAIN_ON, PRCM_AUTO);
			break;
		}
#endif /* #ifdef CONFIG_HW_SUP_TRANS */
			break;

	case POWER_DOMAIN_RET:
#ifdef CONFIG_OMAP34XX_OFFMODE
		ret =  prcm_set_power_domain_state(prcm_id,
					POWER_DOMAIN_RET, PRCM_FORCE);
		break;
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */
	case POWER_DOMAIN_OFF:

#ifdef CONFIG_OMAP_ZEBU
		if (prcm_id == DOM_DSS)
		return PRCM_PASS;
#endif

#ifdef CONFIG_OMAP34XX_OFFMODE
		if (prcm_id == DOM_PER)
			ret = prcm_set_power_domain_state(prcm_id,
						POWER_DOMAIN_RET, PRCM_FORCE);
		else
			ret = prcm_set_power_domain_state(prcm_id,
						POWER_DOMAIN_OFF, PRCM_FORCE);

		context_restore_update(prcm_id);
#else
		ret = prcm_set_power_domain_state(prcm_id,
					POWER_DOMAIN_RET, PRCM_FORCE);
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */
		break;
	default:
		ret = PRCM_FAIL;
	}
	return ret;
}

/* Activation function to change target level for a Logical resource */
int activate_logical_res(struct shared_resource *resp,
			 unsigned short current_level,
			 unsigned short target_level)
{
	unsigned long prcm_id = resp->prcm_id;

	switch (prcm_id) {
	case DOM_CORE1:
		core_active = (target_level)?LOGICAL_USED:LOGICAL_UNUSED;
		break;
	default:
		return -1;
	}
	return 0;
}

/* Function to change target level for Memory or Logic resource */
int activate_memory_logic(struct shared_resource *resp,
			 unsigned short current_level,
			 unsigned short target_level)
{
	unsigned long domain_id;
	unsigned long prcm_id = resp->prcm_id;
	domain_id = DOMAIN_ID(prcm_id);
	switch (prcm_id) {
	case PRCM_CORE_MEM1ON:
		prcm_set_memory_resource_on_state(target_level);
		break;
	case PRCM_CORE_MEM2ON:
		prcm_set_memory_resource_on_state(target_level);
		break;
	/* Check to make sure that OSWR is supported only when
	 * CORE_OFF macro is enabled
	 */
#ifdef CONFIG_CORE_OFF
	case PRCM_CORE_MEM1RET:
		break;
	case PRCM_CORE_MEM2RET:
		break;
	case PRCM_CORE_LOGICRET:
		break;
#endif
	default :
		DPRINTK("Unsupported resource\n");
		return -1;
	}
	return 0;
}


/* Functio to validate the memory or logic resource transition */
int validate_memory_logic(struct shared_resource *resp,
			 unsigned short current_level,
			 unsigned short target_level)
{
	if (target_level >= resp->max_levels)
		return -1;
	return 0;
}


int activate_autoidle_resource(struct shared_resource *resp,
			unsigned short current_level,
			 unsigned short target_level)
{
	unsigned long prcm_id = resp->prcm_id;
	int ret;
	ret = prcm_dpll_clock_auto_control(prcm_id, target_level);
	if (ret == PRCM_FAIL) {
		DPRINTK("Invalid DPLL Autoidle resource state\n");
		return -1;
	}
	return 0;
}

int validate_autoidle_resource(struct shared_resource *resp,
			unsigned short current_level,
			unsigned short target_level)
{
	int invalidlevelmin = 2, invalidlevelmax = 4;
	if (target_level >= resp->max_levels) {
		printk(KERN_ERR"Invalid Target Level\n");
		return -1;
	}
	if (strncmp (resp->name,"core_autoidle_res", 17) == 0)
	if (target_level >= invalidlevelmin &&
			target_level <= invalidlevelmax) {
		printk(KERN_ERR"Invalid Target Level\n");
		return -1;
	}
	
	return 0;
}

int activate_constraint(struct shared_resource *resp,
			unsigned short current_value,
			unsigned short target_value)
{
	int ind;
	unsigned long type = resp->prcm_id;

	DPRINTK("CUR_VAL = %d, TAR_VAL = %d\n", current_value, target_value);
	if (type == RES_LATENCY_CO) {
		switch (target_value) {
		case CO_LATENCY_WFI:
		/* Allows only wfi + tick suppression. State C1*/
		set_acceptable_latency("omap_lt_co", 99);
		break;
		case CO_LATENCY_MPURET_COREON:
		/* Allows upto MPU RET. State C2*/
		set_acceptable_latency("omap_lt_co", 3299);
		break;
		case CO_LATENCY_MPUOFF_COREON:
		/* Allows upto MPU OFF. State C3*/
		set_acceptable_latency("omap_lt_co", 4999);
		break;
		case CO_LATENCY_MPUOFF_CORERET:
		/* Allows upto CORE RET. State C4*/
		set_acceptable_latency("omap_lt_co", 39999);
		break;
		case CO_UNUSED:
		/* Removing the constraints */
		remove_acceptable_latency("omap_lt_co");
		break;
		default:
		set_acceptable_latency("omap_lt_co", target_value);
		break;
		}
	} else if ((type == PRCM_ARMFREQ_CONSTRAINT)) {
		for (ind = 0; ind < max_vdd1_opp; ind++) {
			if (vdd1_arm_dsp_freq[ind][0] >= target_value) {
				resource_request(vdd1_res,
					vdd1_arm_dsp_freq[ind][2]);
				break;
			}
		}
	} else if ((type == PRCM_DSPFREQ_CONSTRAINT)) {
		for (ind = 0; ind < max_vdd1_opp; ind++) {
			if (vdd1_arm_dsp_freq[ind][1] >= target_value) {
				resource_request(vdd1_res,
					vdd1_arm_dsp_freq[ind][2]);
				break;
			}
		}
	} else if (type == PRCM_VDD1_CONSTRAINT) {
		if (request_vdd2_co) {
			if (vdd2_res == NULL) {
				vdd2_res = resource_get("co_fwk", "vdd2_opp");
				resource_request(vdd2_res, vdd2_co_val);
			}
		}
		current_vdd1_opp = vdd1_opp_setting(target_value);
		if (release_vdd2_co) {
			if (vdd2_res != NULL) {
				resource_release(vdd2_res);
				resource_put(vdd2_res);
				vdd2_res = NULL;
			}
		}
	} else if (type == PRCM_VDD2_CONSTRAINT) {
		current_vdd2_opp = vdd2_opp_setting(target_value);
	}
	return 0;
}

int activate_pd_constraint(struct shared_resource *resp,
			   unsigned short current_value,
			   unsigned short target_value)
{
	struct res_handle_node *node;

	DPRINTK("CUR_VAL = %d, TAR_VAL = %d\n", current_value, target_value);

	if (list_empty(&resp->linked_res_handles))
		return 0;

	node = list_first_entry(&resp->linked_res_handles,
				struct res_handle_node,
				node);

	switch (target_value) {
	case CO_LATENCY_ON:
		/* Allows only ON power doamin state */
		return resource_request(node->handle, POWER_DOMAIN_ON);
		break;
	case CO_LATENCY_RET:
		/* Allows ON and RET power domain states */
		return resource_request(node->handle,
					POWER_DOMAIN_RET);
		break;
	case CO_UNUSED:
		/* Removing the constraints */
		return resource_release(node->handle);
		break;
	default:
		/* do nothing - allows all power domain states */
		break;
	}

	return 0;
}

/* Validate if the power domain transition from current to target level is*/
/* valid							     	*/
int validate_power_res(struct shared_resource *res,
			unsigned short current_level,
			unsigned short target_level)
{
	DPRINTK("Current_level = %d, Target_level = %d\n",
			current_level, target_level);

	if ((target_level >= res->max_levels)) {
		DPRINTK("Invalid target_level requested\n");
		return -1;
	}

	/* RET to OFF and OFF to RET transitions not allowed for IVA*/
	if (res->prcm_id == DOM_IVA2) {
		if ((current_level < POWER_DOMAIN_ON) &&
				(target_level != POWER_DOMAIN_ON)) {
			DPRINTK("%d to %d transitions for"
					" Power Domain IVA are not allowed\n",
					current_level, target_level);
			return -1;
		}
	}

	return 0;
}

/* Validate if the logical resource transition is valid */
int validate_logical_res(struct shared_resource *res,
				unsigned short current_level,
				unsigned short target_level)
{
	if (target_level >= res->max_levels)
		return -1;
	return 0;
}

int validate_constraint(struct shared_resource *res,
				unsigned short current_value,
				unsigned short target_value)
{
	if (target_value == DEFAULT_LEVEL)
		return 0;
	if (res->prcm_id == PRCM_ARMFREQ_CONSTRAINT) {
		if ((target_value < min_arm_freq) ||
				(target_value > max_arm_freq)) {
			printk(KERN_ERR "Invalid ARM frequency requested\n");
			return -1;
		}
	} else if (res->prcm_id == PRCM_DSPFREQ_CONSTRAINT) {
		if ((target_value < min_dsp_freq) ||
				(target_value > max_dsp_freq)) {
			printk(KERN_ERR "Invalid DSP frequency requested\n");
			return -1;
		}
	} else if (res->prcm_id == PRCM_VDD1_CONSTRAINT) {
		if ((target_value < min_vdd1_opp) ||
				(target_value > max_vdd1_opp)) {
			printk(KERN_ERR "Invalid VDD1 OPP requested\n");
			return -1;
		}
	} else if (res->prcm_id == PRCM_VDD2_CONSTRAINT) {
		if ((target_value < min_vdd2_opp) ||
				(target_value > max_vdd2_opp)) {
			printk(KERN_ERR "Invalid VDD2 OPP requested\n");
			return -1;
		}
	}
	return 0;
}

/* Turn off unused power domains during bootup
 * If OFFMODE macro is enabled, all power domains are turned off during bootup
 * If OFFMODE macro is not enabled, all power domains except IVA and GFX
 * are put to retention and IVA,GFX are put to off
 * NEON is not put to off because it is required by VFP
 */
int turn_power_domains_off(void)
{
	struct shared_resource **resp;
	u8 state;
	u32 prcmid;
	
	printk("turn_power_domains_off\n");
	
	for (resp = res_list; resp < res_list + ARRAY_SIZE(res_list); resp++) {
		if (list_empty(&((*resp)->users_list))) {
			if ((*resp)->res_type == RES_POWER_DOMAIN) {
				prcmid = (*resp)->prcm_id;
				prcm_get_power_domain_state(prcmid, &state);
				if (state == PRCM_ON) {
#ifdef CONFIG_OMAP34XX_OFFMODE
					if (prcmid == DOM_PER)
						state = PRCM_ON;
					else
						state = PRCM_OFF;
#else
#ifdef CONFIG_OMAP3430_ES2
#if !defined(CONFIG_ARCH_OMAP3410) && defined(CONFIG_ARCH_HAS_OMAP3_SGX)
					if ((prcmid == DOM_IVA2) || (prcmid == DOM_SGX))
#else
					if (prcmid == DOM_IVA2)
#endif
#else
					if ((prcmid == DOM_IVA2) ||
						(prcmid == DOM_GFX))
#endif /* CONFIG_OMAP3430_ES2 */
						state = PRCM_OFF;
					else
						state = PRCM_RET;
#endif /* CONFIG_OMAP34XX_OFFMODE */
#ifndef CONFIG_ARCH_OMAP3410
					if (prcmid == DOM_NEON) {
						if (is_sil_rev_equal_to
							(OMAP3430_REV_ES1_0))
							state = PRCM_RET;
#ifdef CONFIG_VFP
						state = PRCM_ON;
#endif
					}
#endif
					prcm_force_power_domain_state
						((*resp)->prcm_id, state);
				}
			}
		}
	}
	return 0;
}

/* Wrappers to handle Constraints */
struct constraint_handle *constraint_get(const char *name,
					 struct constraint_id *cnstr_id)
{
	char *id;
	struct clk *clk;
	switch (cnstr_id->type) {
	case RES_CLOCK_RAMPUP_CO:
#ifdef CONFIG_ARCH_OMAP34XX
		clk = (struct clk *)cnstr_id->data;
			if ((clk == NULL) || IS_ERR(clk))
			return ERR_PTR(-ENOENT);

		id = id_to_lname[DOMAIN_ID(clk->prcmid) - 1];
#else
		printk(KERN_ERR "%s: CLOCK_RAMPUP constraints are not "
			"supported\n", __FUNCTION__);
#endif /* CONFIG_ARCH_OMAP34XX */
		break;

	case RES_FREQ_CO:
		DPRINTK("Freq constraint %s requested\n",
			(char *)cnstr_id->data);
		if (vdd1_users == 0)
			vdd1_res = resource_get("co_fwk", "vdd1_opp");
			vdd1_users++;


			id = (char *)cnstr_id->data;
		break;

	default:
		id = (char *)cnstr_id->data;
		break;
	}
	/* Just calls the shared resource api */
	return (struct constraint_handle *) resource_get(name, id);
}
EXPORT_SYMBOL(constraint_get);

int constraint_set(struct constraint_handle *constraint,
					unsigned short constraint_level)
{
	return resource_request((struct res_handle *)constraint,
							constraint_level);
}
EXPORT_SYMBOL(constraint_set);

int constraint_remove(struct constraint_handle *constraint)
{
	return resource_release((struct res_handle *)constraint);
}
EXPORT_SYMBOL(constraint_remove);

void constraint_put(struct constraint_handle *constraint)
{
	struct  res_handle *res_h;
	res_h = (struct res_handle *)constraint;
	if ((strcmp("arm_freq", res_h->res->name) == 0)
		|| (strcmp("dsp_freq", res_h->res->name) == 0)) {
		vdd1_users--;
		if (vdd1_users == 0) {
			resource_release(vdd1_res);
			resource_put(vdd1_res);
			vdd1_res = NULL;
		}
	}
	resource_put((struct res_handle *)constraint);
}
EXPORT_SYMBOL(constraint_put);

void constraint_register_pre_notification(struct constraint_handle *constraint,
			 struct notifier_block *nb, unsigned short target_level)
{
	struct  res_handle *res_h;
	res_h = (struct res_handle *)constraint;
	if (res_h->res->prcm_id == PRCM_ARMFREQ_CONSTRAINT) {
		if (!vdd1_arm_prenotifications)
			resource_register_pre_notification(vdd1_res,
				&nb_arm_freq_prenotify, max_vdd1_opp+1);
		atomic_notifier_chain_register(&freq_arm_pre_notifier_list, nb);
		vdd1_arm_prenotifications++;
	} else if (res_h->res->prcm_id == PRCM_DSPFREQ_CONSTRAINT) {
		if (!vdd1_dsp_prenotifications)
			resource_register_pre_notification(vdd1_res,
				&nb_dsp_freq_prenotify, max_vdd1_opp+1);

		atomic_notifier_chain_register(&freq_dsp_pre_notifier_list, nb);
		vdd1_dsp_prenotifications++;
	} else {
		resource_register_pre_notification(
			(struct res_handle *)constraint, nb, target_level);
	}
}
EXPORT_SYMBOL(constraint_register_pre_notification);

void constraint_register_post_notification(struct constraint_handle *constraint,
		struct notifier_block *nb, unsigned short target_level)
{
	struct  res_handle *res_h;
	res_h = (struct res_handle *)constraint;
	if (res_h->res->prcm_id == PRCM_ARMFREQ_CONSTRAINT) {
		if (!vdd1_arm_postnotifications)
			resource_register_post_notification(vdd1_res,
				&nb_arm_freq_postnotify, max_vdd1_opp+1);


		atomic_notifier_chain_register(&freq_arm_post_notifier_list,
									nb);
		vdd1_arm_postnotifications++;
	} else if (res_h->res->prcm_id == PRCM_DSPFREQ_CONSTRAINT) {
		if (!vdd1_dsp_postnotifications)
			resource_register_post_notification(vdd1_res,
				&nb_dsp_freq_postnotify, max_vdd1_opp+1);

		atomic_notifier_chain_register(&freq_dsp_post_notifier_list,
									nb);
		vdd1_dsp_postnotifications++;
	} else {
		resource_register_post_notification(
				(struct res_handle *)constraint,
				nb, target_level);
	}
}
EXPORT_SYMBOL(constraint_register_post_notification);

void constraint_unregister_pre_notification(
		struct constraint_handle *constraint,
		struct notifier_block *nb,
		 unsigned short target_level)
{
	struct  res_handle *res_h;
	res_h = (struct res_handle *)constraint;
	if (res_h->res->prcm_id == PRCM_ARMFREQ_CONSTRAINT) {
		vdd1_arm_prenotifications--;
		atomic_notifier_chain_unregister(&freq_arm_pre_notifier_list,
									nb);
		if (!vdd1_arm_prenotifications)
			resource_unregister_pre_notification(vdd1_res,
			&nb_arm_freq_prenotify, max_vdd1_opp+1);
	} else if (res_h->res->prcm_id == PRCM_DSPFREQ_CONSTRAINT) {
		vdd1_dsp_prenotifications--;
		atomic_notifier_chain_unregister(&freq_dsp_pre_notifier_list,
									nb);
		if (!vdd1_dsp_prenotifications)
			resource_unregister_pre_notification(vdd1_res,
				&nb_dsp_freq_prenotify, max_vdd1_opp+1);
	} else {
		resource_unregister_pre_notification(
				(struct res_handle *)constraint,
				nb, target_level);
	}
}
EXPORT_SYMBOL(constraint_unregister_pre_notification);

void constraint_unregister_post_notification(
		struct constraint_handle *constraint,
		struct notifier_block *nb, unsigned short target_level)
{
	struct  res_handle *res_h;
	res_h = (struct res_handle *)constraint;
	if (res_h->res->prcm_id == PRCM_ARMFREQ_CONSTRAINT) {
		vdd1_arm_postnotifications--;
		atomic_notifier_chain_unregister(&freq_arm_post_notifier_list,
									nb);
		if (!vdd1_arm_postnotifications)
			resource_unregister_post_notification(vdd1_res,
				&nb_arm_freq_postnotify, max_vdd1_opp+1);
	} else if (res_h->res->prcm_id == PRCM_DSPFREQ_CONSTRAINT) {
		vdd1_dsp_postnotifications--;
		atomic_notifier_chain_unregister(&freq_dsp_post_notifier_list,
									nb);
		if (!vdd1_dsp_postnotifications)
			resource_unregister_post_notification(vdd1_res,
				&nb_dsp_freq_postnotify, max_vdd1_opp+1);
		} else {
		resource_unregister_post_notification(
			(struct res_handle *)constraint,
							nb, target_level);
	}
}
EXPORT_SYMBOL(constraint_unregister_post_notification);

int constraint_get_level(struct constraint_handle *constraint)
{
	return resource_get_level((struct res_handle *)constraint);
}
EXPORT_SYMBOL(constraint_get_level);

#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
static void *omap_shared_res_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *omap_shared_res_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

static void omap_shared_res_stop(struct seq_file *m, void *v)
{
}

int omap_shared_res_show(struct seq_file *m, void *v)
{
	struct shared_resource **resp;
	struct users_list *user;

	for (resp = res_list; resp < res_list + ARRAY_SIZE(res_list); resp++) {
		seq_printf(m, "Number of users for domain %s: %lu\n",
				(*resp)->name, (*resp)->no_of_users);
		list_for_each_entry(user, &((*resp)->users_list), node) {
			seq_printf(m, "User: %s  Level: %lu\n",
					user->usr_name, user->level);
		}
	}
	return 0;
}

static struct seq_operations omap_shared_res_op = {
	.start = omap_shared_res_start,
	.next  = omap_shared_res_next,
	.stop  = omap_shared_res_stop,
	.show  = omap_shared_res_show
};

static int omap_ck_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &omap_shared_res_op);
}

static struct file_operations proc_shared_res_ops = {
	.open		= omap_ck_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

/* This API creates a proc entry for shared resources. */
int proc_entry_create(void)
{
	struct proc_dir_entry *entry;

	/* Create a proc entry for shared resources */
	entry = create_proc_entry("shared_resources", 0, NULL);
	if (entry) {
		entry->proc_fops = &proc_shared_res_ops;
		printk(KERN_ERR "create_proc_entry succeeded\n");
	}
	else
		printk(KERN_ERR "create_proc_entry failed\n");

	return 0;
}

__initcall(proc_entry_create);
#endif

#ifdef CONFIG_AUTO_POWER_DOMAIN_CTRL
late_initcall(turn_power_domains_off);
#endif
