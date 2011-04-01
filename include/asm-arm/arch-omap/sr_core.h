/*
 * linux/include/asm-arm/arch-omap/sr_core.h
 *
 * OMAP34XX SmartReflex Voltage Control core functions
 *
 * Copyright (C) 2008 Archos S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define SR1		1
#define SR2		2

#define SR_FAIL		1
#define SR_PASS		0

#define SR_TRUE		1
#define SR_FALSE	0

struct smartreflex_alg
{
	int (*init)(struct smartreflex_alg *alg);
	int (*release)(struct smartreflex_alg *alg);
	
	int (*enable)(struct smartreflex_alg *alg, int srid);
	int (*disable)(struct smartreflex_alg *alg, int srid);

	int (*autocomp_state)(struct smartreflex_alg *alg, int srid);
	int (*autocomp_start)(struct smartreflex_alg *alg, int srid, int target_opp_no);
	int (*autocomp_stop)(struct smartreflex_alg *alg, int srid);

	int (*set_voltage)(struct smartreflex_alg *alg, u32 target_opp, int vsel);
};

extern int srcore_register_alg(struct smartreflex_alg *alg);
extern int srcore_unregister_alg(struct smartreflex_alg *alg);

#define MAX_NUM_POWER_SUPPLY 2
struct power_supply_desc
{
	int min_mv;
	int max_mv;
	int step_mv;
	int cur_mv;	
};

struct voltage_controller
{
	int num_power_supply;
	struct power_supply_desc power_supply[MAX_NUM_POWER_SUPPLY];
	int (*set_voltage)(int vdd, int mv);
};

extern int srcore_register_vc(struct voltage_controller *vc);
extern int srcore_unregister_vc(struct voltage_controller *vc);

/* API for PRCM */
extern int srcore_set_voltage_level(u32 target_opp, int vsel);
extern int srcore_enable_smartreflex(int srid);
extern int srcore_disable_smartreflex(int srid);
