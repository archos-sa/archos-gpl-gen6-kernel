/*
 * linux/arch/arm/mach-omap2/io.c
 *
 * OMAP2 I/O mapping code
 *
 * Copyright (C) 2005 Nokia Corporation
 * Copyright (C) 2007 Texas Instruments
 *
 * Author:
 *	Juha Yrjola <juha.yrjola@nokia.com>
 *	Syed Khasim <x0khasim@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <asm/tlb.h>
#include <asm/io.h>

#include <asm/mach/map.h>

#include <asm/arch/mux.h>
#include <asm/arch/omapfb.h>

extern void omap_sram_init(void);
#if defined(CONFIG_ARCH_OMAP24XX)
extern int omap2_clk_init(void);
#elif defined(CONFIG_ARCH_OMAP34XX)
extern int omap3_clk_init(void);
#endif

extern void omap2_check_revision(void);
extern void omap2_init_memory(void);
extern void gpmc_init(void);
extern void omapfb_reserve_sdram(void);

/*
 * The machine specific code may provide the extra mapping besides the
 * default mapping provided here.
 */

#ifdef CONFIG_ARCH_OMAP24XX
static struct map_desc omap24xx_io_desc[] __initdata = {
	{
		.virtual	= L3_24XX_VIRT,
		.pfn		= __phys_to_pfn(L3_24XX_PHYS),
		.length		= L3_24XX_SIZE,
		.type		= MT_DEVICE
	},
	{
		.virtual	= L4_24XX_VIRT,
		.pfn		= __phys_to_pfn(L4_24XX_PHYS),
		.length		= L4_24XX_SIZE,
		.type		= MT_DEVICE
	},
};

#ifdef CONFIG_ARCH_OMAP2420
static struct map_desc omap242x_io_desc[] __initdata = {
	{
		.virtual	= DSP_MEM_24XX_VIRT,
		.pfn		= __phys_to_pfn(DSP_MEM_24XX_PHYS),
		.length		= DSP_MEM_24XX_SIZE,
		.type		= MT_DEVICE
	},
	{
		.virtual	= DSP_IPI_24XX_VIRT,
		.pfn		= __phys_to_pfn(DSP_IPI_24XX_PHYS),
		.length		= DSP_IPI_24XX_SIZE,
		.type		= MT_DEVICE
	},
	{
		.virtual	= DSP_MMU_24XX_VIRT,
		.pfn		= __phys_to_pfn(DSP_MMU_24XX_PHYS),
		.length		= DSP_MMU_24XX_SIZE,
		.type		= MT_DEVICE
	},
};

#endif

#ifdef CONFIG_ARCH_OMAP2430
static struct map_desc omap243x_io_desc[] __initdata = {
	{
		.virtual	= L4_WK_243X_VIRT,
		.pfn		= __phys_to_pfn(L4_WK_243X_PHYS),
		.length		= L4_WK_243X_SIZE,
		.type		= MT_DEVICE
	},
	{
		.virtual	= OMAP243X_GPMC_VIRT,
		.pfn		= __phys_to_pfn(OMAP243X_GPMC_PHYS),
		.length		= OMAP243X_GPMC_SIZE,
		.type		= MT_DEVICE
	},
	{
		.virtual	= OMAP243X_SDRC_VIRT,
		.pfn		= __phys_to_pfn(OMAP243X_SDRC_PHYS),
		.length		= OMAP243X_SDRC_SIZE,
		.type		= MT_DEVICE
	},
	{
		.virtual	= OMAP243X_SMS_VIRT,
		.pfn		= __phys_to_pfn(OMAP243X_SMS_PHYS),
		.length		= OMAP243X_SMS_SIZE,
		.type		= MT_DEVICE
	},
};
#endif
#endif

#ifdef	CONFIG_ARCH_OMAP34XX
static struct map_desc omap34xx_io_desc[] __initdata = {
        {
                .virtual        = L4_VIRT,
                .pfn            = __phys_to_pfn(L4_PHYS),
                .length         = L4_SIZE,
                .type           = MT_MEMORY_SO
        },
        {
                .virtual        = L4_WK_VIRT,
                .pfn            = __phys_to_pfn(L4_WK_PHYS),
                .length         = L4_WK_SIZE,
                .type           = MT_DEVICE
        },
        {
                .virtual        = L4_PER_VIRT,
                .pfn            = __phys_to_pfn(L4_PER_PHYS),
                .length         = L4_PER_SIZE,
                .type           = MT_DEVICE
        },
        {
                .virtual        = L4_EMU_VIRT,
                .pfn            = __phys_to_pfn(L4_EMU_PHYS),
                .length         = L4_EMU_SIZE,
                .type           = MT_MEMORY_SO
        },
        {
                .virtual        = GFX_VIRT,
                .pfn            = __phys_to_pfn(GFX_PHYS),
                .length         = GFX_SIZE,
                .type           = MT_MEMORY_SO
        },
        {
                .virtual        = L3_VIRT,
                .pfn            = __phys_to_pfn(L3_PHYS),
                .length         = L3_SIZE,
                .type           = MT_MEMORY_SO
        },
        {
                .virtual        = SMS_VIRT,
                .pfn            = __phys_to_pfn(SMS_PHYS),
                .length         = SMS_SIZE,
                .type           = MT_MEMORY_SO
        },
        {
                .virtual        = SDRC_VIRT,
                .pfn            = __phys_to_pfn(SDRC_PHYS),
                .length         = SDRC_SIZE,
                .type           = MT_MEMORY_SO
        },
        {
                .virtual        = GPMC_VIRT,
                .pfn            = __phys_to_pfn(GPMC_PHYS),
                .length         = GPMC_SIZE,
                .type           = MT_MEMORY_SO
        },
};
#endif

void __init omap2_map_common_io(void)
{
#if defined(CONFIG_ARCH_OMAP2420)
	iotable_init(omap24xx_io_desc, ARRAY_SIZE(omap24xx_io_desc));
	iotable_init(omap242x_io_desc, ARRAY_SIZE(omap242x_io_desc));
#elif defined(CONFIG_ARCH_OMAP2430)
	iotable_init(omap24xx_io_desc, ARRAY_SIZE(omap24xx_io_desc));
	iotable_init(omap243x_io_desc, ARRAY_SIZE(omap243x_io_desc));
#elif defined(CONFIG_ARCH_OMAP34XX)
	iotable_init(omap34xx_io_desc, ARRAY_SIZE(omap34xx_io_desc));
#endif
	/* Normally devicemaps_init() would flush caches and tlb after
	 * mdesc->map_io(), but we must also do it here because of the CPU
	 * revision check below.
	 */
	local_flush_tlb_all();
	flush_cache_all();
	omap2_check_revision();
	omap_sram_init();
	omapfb_reserve_sdram();
}

void __init omap2_init_common_hw(void)
{
#if defined(CONFIG_ARCH_OMAP24XX)
	omap2_mux_init();
	omap2_clk_init();
#elif defined(CONFIG_ARCH_OMAP34XX)
	omap3_mux_init();
	omap3_clk_init();
#endif	
	omap2_init_memory();
	gpmc_init();
}