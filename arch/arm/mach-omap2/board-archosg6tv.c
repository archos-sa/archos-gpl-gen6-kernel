/*
 * linux/arch/arm/mach-omap2/board-archosg6tv.c
 *
 * Copyright (C) 2008 Matthias Welwarsky, Archos S.A.,
 *
 * Derived from mach-omap2/board-omap3evm.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/clk.h>

#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#include <asm/arch/irda.h>
#include <asm/arch/board.h>
#include <asm/arch/common.h>
#include <asm/arch/dma.h>
#include <asm/arch/gpmc.h>
#include <asm/arch/pwm-gp.h>
#include <asm/arch/archosg6-irr.h>

#include <asm/io.h>
#include <asm/delay.h>

#define CONTROL_SYSC_SMARTIDLE	(0x2 << 3)
#define CONTROL_SYSC_AUTOIDLE	(0x1)

//#define CONFIG_WIFI_PWRON_GPIO_115

#ifdef CONFIG_WIFI_PWRON_GPIO_115
#define GPIO_WIFI_PWRON	115 // csi2_dy1
#else
#define GPIO_WIFI_PWRON	186 // sys_clkout2
#endif

static int wifi_pwron;

#define GPIO_RST_WIFI	16

#define CONFIG_ETH_SMSC9211

#ifdef CONFIG_ETH_SMSC9211
#define	G6_SMSC9211_CS	5
#define OMAP34XX_ETHR_GPIO_IRQ_GEN6	142
#endif

// #ifdef CONFIG_MACH_ARCHOS_G6
#define CONFIG_ETH_SMSC9211_GEN6
// #endif

static void scm_clk_init(void)
{
	struct clk *p_omap_ctrl_clk = NULL;

	p_omap_ctrl_clk = clk_get (NULL, "omapctrl_ick");
	if (p_omap_ctrl_clk != NULL) {
		if (clk_enable(p_omap_ctrl_clk)	!= 0) {
			printk(KERN_ERR "failed to enable scm clks\n");
			clk_put(p_omap_ctrl_clk);
		}
	}
	/* Sysconfig set to SMARTIDLE and AUTOIDLE */
	CONTROL_SYSCONFIG = (CONTROL_SYSC_SMARTIDLE | CONTROL_SYSC_AUTOIDLE);
}

#ifdef CONFIG_ETH_SMSC9211
static struct resource g6_smsc9211_resources[] = {
	[0] = {
// 		.start	= OMAP34XX_ETHR_START-0x1000000,
// 		.end	= OMAP34XX_ETHR_START-0x1000000 + SZ_4K,
		/* for gpmc, if the cs is not enabled while request_cs, the .start and .end have no sens. the .start will be a address found by allocate_resource()  */
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= OMAP_GPIO_IRQ(0),  // real value is initilized by g6_init_smsc9211
		.end	= 0,
		.flags	= IORESOURCE_IRQ,
	},
};
#endif

#ifdef CONFIG_ETH_SMSC9211
static struct platform_device g6_smsc9211_device = {
	.name		= "smsc9211",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(g6_smsc9211_resources),
	.resource	= g6_smsc9211_resources,
};
#endif

#ifdef CONFIG_ETH_SMSC9211

	#define CS3_PINCFG U8_3430_GPIO54
	#define CS2_PINCFG  V8_3430_GPIO53
	#define CS4_PINCFG T8_3430_GPIO55

	#define ETH_CS_PINCFG R8_3430_CS5

	#define ETH_CS5_GIONUM	56

#ifdef CONFIG_ETH_SMSC9211_GEN6
	#define ETH_SEL_PINCFG AF9_3430_GPIO22
	#define ETH_IRQ_PINCFG AF5_3430_GPIO142
	#define ETH_PME_PINCFG AE5_3430_GPIO143
	#define ETH_RST_PINCFG AE6_3430_GPIO141

	#define ETH_SEL_GIONUM 22
	#define ETH_IRQ_GIONUM 142
	#define ETH_PME_GIONUM 143
	#define ETH_RST_GIONUM 141
#else
	#define ETH_SEL_PINCFG N8_3430_GPIO58
	#define ETH_IRQ_PINCFG P8_3430_GPIO57
	#define ETH_PME_PINCFG T8_3430_GPIO55

	#define ETH_SEL_GIONUM 58
	#define ETH_IRQ_GIONUM 57
	#define ETH_PME_GIONUM 55
#endif

static void smsc9211_pinset(void)
{
	omap_cfg_reg(ETH_CS_PINCFG);

	omap_cfg_reg(ETH_SEL_PINCFG);
	omap_cfg_reg(ETH_IRQ_PINCFG);
	omap_cfg_reg(ETH_PME_PINCFG);

#ifdef CONFIG_ETH_SMSC9211_GEN6
	omap_cfg_reg(ETH_RST_PINCFG);
#endif

	omap_request_gpio(ETH_SEL_GIONUM);
	omap_request_gpio(ETH_IRQ_GIONUM);
	omap_request_gpio(ETH_PME_GIONUM);

#ifdef CONFIG_ETH_SMSC9211_GEN6
	omap_request_gpio(ETH_RST_GIONUM);
#endif

	omap_set_gpio_direction(ETH_SEL_GIONUM, GPIO_DIR_OUTPUT);
	omap_set_gpio_dataout(ETH_SEL_GIONUM,0);
	omap_set_gpio_direction(ETH_IRQ_GIONUM, GPIO_DIR_INPUT);
	omap_set_gpio_direction(ETH_PME_GIONUM, GPIO_DIR_INPUT);

#ifdef CONFIG_ETH_SMSC9211_GEN6
	omap_set_gpio_direction(ETH_RST_GIONUM, GPIO_DIR_OUTPUT);
#endif
 	mdelay(5);
}

static inline void __init g6_init_smsc9211(void)
{
	unsigned long cs_mem_base;
	unsigned int rate;
	struct clk *l3ck;
// 	int eth_gpio = 0;
	printk("g6_init_smsc9211\n");

	l3ck = clk_get(NULL, "l3_ck");
	if (IS_ERR(l3ck))
		rate = 100000000;
	else
		rate = clk_get_rate(l3ck);

	if (gpmc_cs_request(G6_SMSC9211_CS, SZ_16M, &cs_mem_base) < 0) {
		printk(KERN_ERR "Failed to request GPMC mem for smsc9211\n");
		return;
	}

	g6_smsc9211_resources[0].start = cs_mem_base + 0x0;
	g6_smsc9211_resources[0].end   = cs_mem_base + 0x1ff;

	printk("cs_mem_base of smsc9211: %x\n",g6_smsc9211_resources[0].start);

	udelay(100);

	gpmc_cs_write_reg(G6_SMSC9211_CS,0x4,0x060600);
	gpmc_cs_write_reg(G6_SMSC9211_CS,0xc,0x06000600);
	gpmc_cs_write_reg(G6_SMSC9211_CS,0x10,0x10060909);
	gpmc_cs_write_reg(G6_SMSC9211_CS,0x14,0x06030000);
#ifdef CONFIG_ETH_SMSC9211_GEN6
	g6_smsc9211_resources[1].start = 
			OMAP_GPIO_IRQ(OMAP34XX_ETHR_GPIO_IRQ_GEN6);
#else
	g6_smsc9211_resources[1].start = 
			OMAP_GPIO_IRQ(OMAP34XX_ETHR_GPIO_IRQ_EVM);
#endif


// 	eth_gpio = OMAP34XX_ETHR_GPIO_IRQ_GEN6;

// 	if (omap_request_gpio(eth_gpio) < 0) {
// 		printk(KERN_ERR "Failed to request GPIO%d for smsc9211 IRQ\n",
// 			eth_gpio);
// 		return;
// 	}
// 	omap_set_gpio_direction(eth_gpio, 1);

	smsc9211_pinset();
}
#endif 

static struct platform_device *archos_devices[] __initdata = {
#ifdef CONFIG_ETH_SMSC9211
	&g6_smsc9211_device,
#endif
};

static int __init omap3430_i2c_init(void)
{
	omap_cfg_reg(AE15_3430_I2C2_SDA);
	omap_cfg_reg(AF15_3430_I2C2_SCL);
	omap_register_i2c_bus(1, CONFIG_I2C_OMAP34XX_HS_BUS1, NULL, 0);
	omap_register_i2c_bus(2, CONFIG_I2C_OMAP34XX_HS_BUS2, NULL, 0);
	omap_register_i2c_bus(3, CONFIG_I2C_OMAP34XX_HS_BUS3, NULL, 0);
	return 0;
}

static void __init _init_irq(void)
{
	omap2_init_common_hw();
	omap_init_irq();
	omap_gpio_init();
	g6_init_smsc9211();
}

static struct omap_uart_config uart_config __initdata = {
#ifdef CONFIG_HS_SERIAL_SUPPORT
	.enabled_uarts	= ((1 << 0) | (1 << 1) | (1 << 2)),
#endif
};

static struct omap_mmc_config mmc_config __initdata = {
	.mmc [0] = {
		.enabled        = 0,
		.wire4		= 1,
		.wire8		= 1,
		.wp_pin         = -1,
		.power_pin      = -1,
		.switch_pin     = -1,
		.f_max		= 32000000,
	},
	.mmc [1] = {
		.enabled        = 1,
		.wire4          = 1,
		.wp_pin         = -1,
		.power_pin      = -1,
		.switch_pin     = -1,
	},
	.mmc [2] = {
		.enabled	= 0,
		.wire4		= 0,
		.wire8		= 1,
		.wp_pin		= -1,
		.power_pin	= -1,
		.switch_pin	= -1,
		.f_max		= 52000000,
	},
};
	
static struct archosg6_mmc_wifi_dev_conf wifi_dev_conf __initdata = {
	.nrev = 6,
	.rev[0] = { .power_pin = GPIO_WIFI_PWRON, .power_pin_cfg = -1, .rst_pin = GPIO_RST_WIFI, .rst_pin_cfg = -1 },
	.rev[1] = { .power_pin = GPIO_WIFI_PWRON, .power_pin_cfg = -1, .rst_pin = GPIO_RST_WIFI, .rst_pin_cfg = -1 },
	.rev[2] = { .power_pin = 54,              .power_pin_cfg = -1, .rst_pin = GPIO_RST_WIFI, .rst_pin_cfg = U8_3430_GPIO54 },
	.rev[3] = { .power_pin = 54,              .power_pin_cfg = -1, .rst_pin = GPIO_RST_WIFI, .rst_pin_cfg = U8_3430_GPIO54 },
	.rev[4] = { .power_pin = 54,              .power_pin_cfg = -1, .rst_pin = GPIO_RST_WIFI, .rst_pin_cfg = U8_3430_GPIO54 },
	.rev[5] = { .power_pin = 54,              .power_pin_cfg = -1, .rst_pin = GPIO_RST_WIFI, .rst_pin_cfg = U8_3430_GPIO54 },
	.rev[6] = { .power_pin = 54,              .power_pin_cfg = -1, .rst_pin = GPIO_RST_WIFI, .rst_pin_cfg = U8_3430_GPIO54 },
};

static struct archosg6_vbus_config vbus0_config __initdata = {
	.nrev = 4,
	.rev[0] = { .nb = 55, .mux_cfg = T8_3430_GPIO55 },
	.rev[1] = { .nb = 55, .mux_cfg = T8_3430_GPIO55 },
	.rev[2] = { .nb = 55, .mux_cfg = T8_3430_GPIO55 },
	.rev[3] = { .nb = 55, .mux_cfg = T8_3430_GPIO55 },
};


static struct archosg6_display_config display_config __initdata = {
	.nrev = 4, /* cover v0, v1 and v2 boards */
	.rev[0] = {
		.cpldreset = 	{ 140, AF6_3430_GPIO140 },
		.disp_select =	{ 0, 0 },
		.lcd_pwon = 	{ 0, 0 },
		.lcd_rst = 	{ 0, 0 },
		.bkl_pwon = 	{ 0, 0 },
		.lcd_pci = 	{ 0, 0 },
		.hdmi_dac = 	{ 152, AE1_3430_GPIO152 },
		.hdmi_it = 	{ 15, AG12_3430_GPIO15 },
		.cpld_aux = 	{ 21, AH14_3430_GPIO21 },
	},
	.rev[1] = {
		.cpldreset = 	{ 140, AF6_3430_GPIO140 },
		.disp_select =	{ 0, 0 },
		.lcd_pwon = 	{ 0, 0 },
		.lcd_rst = 	{ 0, 0 },
		.bkl_pwon = 	{ 0, 0 },
		.lcd_pci = 	{ 0, 0 },
		.hdmi_dac = 	{ 152, AE1_3430_GPIO152 },
		.hdmi_it = 	{ 15, AG12_3430_GPIO15 },
		.cpld_aux = 	{ 148, AA8_3430_GPIO148 },
	},
	.rev[2] = {
		.cpldreset = 	{ 140, AF6_3430_GPIO140 },
		.disp_select =	{ 0, 0 },
		.lcd_pwon = 	{ 0, 0 },
		.lcd_rst = 	{ 0, 0 },
		.bkl_pwon = 	{ 0, 0 },
		.lcd_pci = 	{ 0, 0 },
		.hdmi_dac = 	{ 152, AE1_3430_GPIO152 },
		.hdmi_it = 	{ 15, AG12_3430_GPIO15 },
		.cpld_aux = 	{ 14, AF11_3430_GPIO14 },
	},
	.rev[3] = {
		.cpldreset = 	{ 140, AF6_3430_GPIO140 },
		.disp_select =	{ 0, 0 },
		.lcd_pwon = 	{ 0, 0 },
		.lcd_rst = 	{ 0, 0 },
		.bkl_pwon = 	{ 0, 0 },
		.lcd_pci = 	{ 0, 0 },
		.hdmi_dac = 	{ 152, AE1_3430_GPIO152 },
		.hdmi_it = 	{ 15, AG12_3430_GPIO15 },
		.cpld_aux = 	{ 14, AF11_3430_GPIO14  },
	},
};

static struct archosg6_usb_config usb_config __initdata = {
	.nrev = 4, /* cover v0, v1 and v2 boards */
	.rev[0] = {
		.usb_id = { .nb = 151, .mux_cfg = Y8_3430_GPIO151    },
		.enable_usb_musb = { .nb = 20,  .mux_cfg = AF13_3430_GPIO20   },
		.enable_usb_ehci = { .nb = 58,  .mux_cfg = N8_3430_GPIO58   },
	},
	.rev[1] = {
		.usb_id = { .nb = 151, .mux_cfg = Y8_3430_GPIO151    },
		.enable_usb_musb = { .nb = 20,  .mux_cfg = AF13_3430_GPIO20   },
		.enable_usb_ehci = { .nb = 58,  .mux_cfg = N8_3430_GPIO58   },
	},
	.rev[2] = {
		.usb_id = { .nb = 156, .mux_cfg = Y21_3430_GPIO156    },
		.enable_usb_musb = { .nb = 20,  .mux_cfg = AF13_3430_GPIO20   },
		.enable_usb_ehci = { .nb = 58,  .mux_cfg = N8_3430_GPIO58   },
	},
	.rev[3] = {
		.usb_id = { .nb = 156, .mux_cfg = Y21_3430_GPIO156    },
		.enable_usb_musb = { .nb = 20,  .mux_cfg = AF13_3430_GPIO20   },
		.enable_usb_ehci = { .nb = 58,  .mux_cfg = N8_3430_GPIO58   },
	},
};

static struct archosg6_audio_config audio_config __initdata = {
	.nrev = 4,
	.rev[0] = {
		.spdif = { .nb = 23, .mux_cfg = AG9_3430_GPIO23 },
		.hp_on = { .nb = 0, .mux_cfg = 0 },
		.headphone_plugged = { .nb = 0, .mux_cfg = 0},
	},
	.rev[1] = {
		.spdif = { .nb = 23, .mux_cfg = AG9_3430_GPIO23 },
		.hp_on = { .nb = 0, .mux_cfg = 0 },
		.headphone_plugged = { .nb = 0, .mux_cfg = 0},
	},
	.rev[2] = {
		.spdif = { .nb = 23, .mux_cfg = AG9_3430_GPIO23 },
		.hp_on = { .nb = 0, .mux_cfg = 0 },
		.headphone_plugged = { .nb = 0, .mux_cfg = 0},
	},
	.rev[3] = {
		.spdif = { .nb = 23, .mux_cfg = AG9_3430_GPIO23 },
		.hp_on = { .nb = 0, .mux_cfg = 0 },
		.headphone_plugged = { .nb = 0, .mux_cfg = 0},
	},
};

static struct omap_board_config_kernel board_config[] __initdata = {
	{ OMAP_TAG_UART,	&uart_config },
	{ OMAP_TAG_MMC,         &mmc_config },
	{ ARCHOS_TAG_VBUS0,	&vbus0_config },
	{ ARCHOS_TAG_DISPLAY,	&display_config},
	{ ARCHOS_TAG_USB,	&usb_config},
	{ ARCHOS_TAG_AUDIO,     &audio_config},
	{ ARCHOS_TAG_WIFI,      &wifi_dev_conf},
};

extern void __init archosg6_usb_init(void);
extern void __init archosg6h_usb2sata_init(void);

static void __init _usb_init(void)
{
	archosg6_usb_init();
	archosg6h_usb2sata_init();
}

extern int __init archosg6_display_init(void);
extern int __init archosg6_wifi_init(void);
extern void archosg6_audio_gpio_init(void);
extern int __init archosg6_videoin_init(void);
extern void __init fixup_archos_g6(struct machine_desc *desc, struct tag *tags, char **cmdline, struct meminfo *mi);
extern void archosg6_power_off(void);

static void __init _board_init(void)
{
	/* System Control module clock initialization */
	scm_clk_init();
	
	omap_cfg_reg(H20_3430_UART3_RX_IRRX);

	/* I2C init */
	omap3430_i2c_init();
	
	platform_add_devices(archos_devices, ARRAY_SIZE(archos_devices));
	omap_board_config = board_config;
	omap_board_config_size = ARRAY_SIZE(board_config);

	omap_serial_init();
	archosg6_wifi_init();
	
	_usb_init();

	archosg6_audio_gpio_init();
	archosg6_display_init();
	archosg6_videoin_init();

	pm_power_off = archosg6_power_off;
}

static void __init _map_io(void)
{
	omap2_map_common_io();
}

MACHINE_START(ARCHOS_G6TV, "ARCHOS G6TV")
	/* Maintainer: Syed Khasim - Texas Instruments Inc */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
#ifdef CONFIG_ARCHOSG6_FIXUP
	.fixup		= fixup_archos_g6,
#endif
	.map_io		= _map_io,
	.init_irq	= _init_irq,
	.init_machine	= _board_init,
	.timer		= &omap_timer,
MACHINE_END
