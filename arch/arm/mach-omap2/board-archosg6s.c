/*
 * linux/arch/arm/mach-omap2/board-omap3evm.c
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
#include <linux/spi/spi.h>

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
#include <asm/arch/mcspi.h>
#include <asm/arch/pwm-gp.h>
#include <asm/arch/spi-gpio.h>


#include <asm/io.h>
#include <asm/delay.h>
#include <asm/arch/archosg6-gpio.h>
#include <asm/arch/archosg6-irr.h>
#include <asm/arch/archosg6-touchscreen.h>

#define CONTROL_SYSC_SMARTIDLE	(0x2 << 3)
#define CONTROL_SYSC_AUTOIDLE	(0x1)

//#define CONFIG_WIFI_PWRON_GPIO_115

#ifdef CONFIG_WIFI_PWRON_GPIO_115
#define GPIO_WIFI_PWRON	115 // csi2_dy1
#else
#define GPIO_WIFI_PWRON	186 // sys_clkout2
#endif

#define GPIO_RST_WIFI	16
#define GPIO_HDD_PWR_ON	18

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

static struct spi_board_info g6s_spi_board_info[] = {
	[0] = { },
	[1] = {
		.modalias	= "hx5091",
		.bus_num	= 5,
		.chip_select	= 0,
		.max_speed_hz   = 1500000,
	},
};

static struct spigpio_info lcd_spi_conf = {
	.pin_clk = 0,
	.pin_mosi = 0,
	.pin_miso = 0,
	.pin_cs = 0,
	.board_size = 1,
	.board_info = &g6s_spi_board_info[1],
};

static struct platform_device archosg6_spi_gpio = {
	.name		  = "omap3-spi-gpio",
	.id		  = 5,
	.dev = {
		.platform_data = &lcd_spi_conf,
	},
};

static struct archosg6_spi_conf spi_lcd_config[MAX_HWREVS] = {
{
{ .nb = 178, .mux_cfg = AA3_3430_GPIO178 },
{ .nb = 179, .mux_cfg = Y2_3430_GPIO179 },
{ .nb = 181, .mux_cfg = Y4_3430_GPIO181 },},
{
{ .nb = 178, .mux_cfg = AA3_3430_GPIO178 },
{ .nb = 179, .mux_cfg = Y2_3430_GPIO179 },
{ .nb = 181, .mux_cfg = Y4_3430_GPIO181 },},
{
{ .nb = 178, .mux_cfg = AA3_3430_GPIO178 },
{ .nb = 179, .mux_cfg = Y2_3430_GPIO179 },
{ .nb = 181, .mux_cfg = Y4_3430_GPIO181 },},
{
{ .nb = 178, .mux_cfg = AA3_3430_GPIO178 },
{ .nb = 179, .mux_cfg = Y2_3430_GPIO179 },
{ .nb = 181, .mux_cfg = Y4_3430_GPIO181 },},
{
{ .nb = 178, .mux_cfg = AA3_3430_GPIO178 },
{ .nb = 179, .mux_cfg = Y2_3430_GPIO179 },
{ .nb = 181, .mux_cfg = Y4_3430_GPIO181 },},
{
{ .nb = 159, .mux_cfg = U21_3430_GPIO159 },
{ .nb = 158, .mux_cfg = V21_3430_GPIO158 },
{ .nb = 154, .mux_cfg = AD2_3430_GPIO154 },},
{
{ .nb = 159, .mux_cfg = U21_3430_GPIO159 },
{ .nb = 158, .mux_cfg = V21_3430_GPIO158 },
{ .nb = 154, .mux_cfg = AD2_3430_GPIO154 },},
};

static void spi_lcd_init( void )
{
	// spi out config
	_INIT_OUTPUT( spi_lcd_config[hardware_rev].spi_clk );
	_INIT_OUTPUT( spi_lcd_config[hardware_rev].spi_data );
	_INIT_OUTPUT( spi_lcd_config[hardware_rev].spi_cs );
	lcd_spi_conf.pin_clk = spi_lcd_config[hardware_rev].spi_clk.nb;
	lcd_spi_conf.pin_mosi = spi_lcd_config[hardware_rev].spi_data.nb;
	lcd_spi_conf.pin_cs = spi_lcd_config[hardware_rev].spi_cs.nb;
}

static struct platform_device archos_g6_ceata_device = {
	.name		= "g6_ceata",
	.id		= -1,
};

static struct platform_device *archos_devices[] __initdata = {
	&archos_g6_ceata_device,
	&archosg6_spi_gpio,
};

static void __init _init_irq(void)
{
	omap2_init_common_hw();
	omap_init_irq();
	omap_gpio_init();
}

static struct omap_uart_config uart_config __initdata = {
#ifdef CONFIG_HS_SERIAL_SUPPORT
	.enabled_uarts	= ((1 << 0) | (1 << 1) | (1 << 2)),
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

static struct omap_mmc_config mmc_config __initdata = {
	.mmc [0] = {
		.enabled        = 1,
		.wire4		= 0,
		.wire8		= 1,
		.wp_pin         = -1,
		.power_pin      = GPIO_HDD_PWR_ON,
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
		.power_pin	= GPIO_HDD_PWR_ON,
		.switch_pin	= -1,
		.f_max		= 52000000,
	},
};
	
static struct archosg6_mmc_wifi_dev_conf wifi_dev_conf __initdata = {
	.nrev = 7,
	.rev[0] = { .power_pin = GPIO_WIFI_PWRON, .power_pin_cfg = -1, .rst_pin = GPIO_RST_WIFI, .rst_pin_cfg = -1 },
	.rev[1] = { .power_pin = GPIO_WIFI_PWRON, .power_pin_cfg = -1, .rst_pin = GPIO_RST_WIFI, .rst_pin_cfg = -1 },
	.rev[2] = { .power_pin = 54,              .power_pin_cfg = -1, .rst_pin = GPIO_RST_WIFI, .rst_pin_cfg = U8_3430_GPIO54 },
	.rev[3] = { .power_pin = 54,              .power_pin_cfg = -1, .rst_pin = GPIO_RST_WIFI, .rst_pin_cfg = U8_3430_GPIO54 },
	.rev[4] = { .power_pin = 54,              .power_pin_cfg = -1, .rst_pin = GPIO_RST_WIFI, .rst_pin_cfg = U8_3430_GPIO54 },
	.rev[5] = { .power_pin = 54,              .power_pin_cfg = -1, .rst_pin = GPIO_RST_WIFI, .rst_pin_cfg = U8_3430_GPIO54 },
	.rev[6] = { .power_pin = 54,              .power_pin_cfg = -1, .rst_pin = GPIO_RST_WIFI, .rst_pin_cfg = U8_3430_GPIO54 },
};

static struct archosg6_tsp_config tsp_config __initdata = {
	.nrev = 7,
	.rev[0] = {
		.irq_gpio = { .nb = 24, .mux_cfg = AE7_3430_GPIO24    },
		.pwr_gpio = { .nb = 148, .mux_cfg = AA8_3430_GPIO148 },
		.x_plate_ohms = 745,
		.pressure_max = 700,
	},
	.rev[1] = {
		.irq_gpio = { .nb = 24, .mux_cfg = AE7_3430_GPIO24    },
		.pwr_gpio = { .nb = 148, .mux_cfg = AA8_3430_GPIO148 },
		.x_plate_ohms = 745,
		.pressure_max = 700,
	},
	.rev[2] = {
		.irq_gpio = { .nb = 24, .mux_cfg = AE7_3430_GPIO24    },
		.pwr_gpio = { .nb = 56,  .mux_cfg = R8_3430_GPIO56   },
		.x_plate_ohms = 745,
		.pressure_max = 700,
	},
	.rev[3] = {
		.irq_gpio = { .nb = 24, .mux_cfg = AE7_3430_GPIO24    },
		.pwr_gpio = { .nb = 56,  .mux_cfg = R8_3430_GPIO56   },
		.x_plate_ohms = 745,
		.pressure_max = 700,
	},
	.rev[4] = {
		.irq_gpio = { .nb = 24, .mux_cfg = AE7_3430_GPIO24    },
		.pwr_gpio = { .nb = 56,  .mux_cfg = R8_3430_GPIO56   },
		.x_plate_ohms = 745,
		.pressure_max = 700,
	},
	.rev[5] = {
		.irq_gpio = { .nb = 142, .mux_cfg = AF5_3430_GPIO142 },
		.pwr_gpio = { .nb = 56,  .mux_cfg = R8_3430_GPIO56   },
		.x_plate_ohms = 745,
		.pressure_max = 700,
	},
	.rev[6] = {
		.irq_gpio = { .nb = 142, .mux_cfg = AF5_3430_GPIO142 },
		.pwr_gpio = { .nb = 56,  .mux_cfg = R8_3430_GPIO56   },
		.x_plate_ohms = 745,
		.pressure_max = 700,
	},
};

static struct archosg6_vbus_config vbus0_config __initdata = {
	.nrev = 7,
	.rev[0] = { .nb = 101, .mux_cfg = B24_3430_GPIO101 },
	.rev[1] = { .nb = 101, .mux_cfg = B24_3430_GPIO101 },
	.rev[2] = { .nb = 55, .mux_cfg = T8_3430_GPIO55 },
	.rev[3] = { .nb = 55, .mux_cfg = T8_3430_GPIO55 },
	.rev[4] = { .nb = 55, .mux_cfg = T8_3430_GPIO55 },
	.rev[5] = { .nb = 55, .mux_cfg = T8_3430_GPIO55 },
	.rev[6] = { .nb = 55, .mux_cfg = T8_3430_GPIO55 },
};

static struct archosg6_charge_config charge_config __initdata = {
	.nrev = 7,
	.rev[0] = { .nb = 0, .mux_cfg = 0 },
	.rev[1] = { .nb = 0, .mux_cfg = 0 },
	.rev[2] = { .nb = 0, .mux_cfg = 0 },
	.rev[3] = { .nb = 29, .mux_cfg = AH8_3430_GPIO29 },
	.rev[4] = { .nb = 29, .mux_cfg = AH8_3430_GPIO29 },
	.rev[5] = { .nb = 155, .mux_cfg = AC1_3430_GPIO155 },
	.rev[6] = { .nb = 155, .mux_cfg = AC1_3430_GPIO155 },
};

static struct archosg6_display_config display_config __initdata = {
	.nrev = 7, /* */
	.rev[0] = {
		.cpldreset = 	{ 27, AH7_3430_GPIO27 },
		.disp_select = 	{ 14, AF11_3430_GPIO14 },
		.lcd_pwon = 	{ 25, AF7_3430_GPIO25 },
		.lcd_rst = 	{ 26, AG7_3430_GPIO26 },
		.bkl_pwon = 	{ 151, Y8_3430_GPIO151 },
		.lcd_pci = 	{ 0, 0 },
		.hdmi_dac = 	{ 28, AG8_3430_GPIO28 },
		.hdmi_it = 	{ 15, AG12_3430_GPIO15 },
		.cpld_aux = 	{ 21, AH14_3430_GPIO21 },
	},
	.rev[1] = {
		.cpldreset = 	{ 27, AH7_3430_GPIO27 },
		.disp_select = 	{ 14, AF11_3430_GPIO14 },
		.lcd_pwon = 	{ 25, AF7_3430_GPIO25 },
		.lcd_rst = 	{ 26, AG7_3430_GPIO26 },
		.bkl_pwon = 	{ 151, Y8_3430_GPIO151 },
		.lcd_pci = 	{ 0, 0 },
		.hdmi_dac = 	{ 28, AG8_3430_GPIO28 },
		.hdmi_it = 	{ 15, AG12_3430_GPIO15 },
		.cpld_aux = 	{ 21, AH14_3430_GPIO21 },
	},
	.rev[2] = {
		.cpldreset = 	{ 27, AH7_3430_GPIO27 },
		.disp_select = 	{ 14, AF11_3430_GPIO14 },
		.lcd_pwon = 	{ 25, AF7_3430_GPIO25 },
		.lcd_rst = 	{ 26, AG7_3430_GPIO26 },
		.bkl_pwon = 	{ 53, V8_3430_GPIO53 }, 
		.lcd_pci = 	{ 0, 0 },
		.hdmi_dac = 	{ 28, AG8_3430_GPIO28 },
		.hdmi_it = 	{ 15, AG12_3430_GPIO15 },
		.cpld_aux = 	{ 21, AH14_3430_GPIO21 },
	},
	.rev[3] = {
		.cpldreset = 	{ 27, AH7_3430_GPIO27 },
		.disp_select = 	{ 14, AF11_3430_GPIO14 },
		.lcd_pwon = 	{ 25, AF7_3430_GPIO25 },
		.lcd_rst = 	{ 26, AG7_3430_GPIO26 },
		.bkl_pwon = 	{ 53, V8_3430_GPIO53 }, 
		.lcd_pci = 	{ 0, 0 },
		.hdmi_dac = 	{ 28, AG8_3430_GPIO28 },
		.hdmi_it = 	{ 15, AG12_3430_GPIO15 },
		.cpld_aux = 	{ 21, AH14_3430_GPIO21 },
	},
	.rev[4] = {
		.cpldreset = 	{ 27, AH7_3430_GPIO27 },
		.disp_select = 	{ 14, AF11_3430_GPIO14 },
		.lcd_pwon = 	{ 25, AF7_3430_GPIO25 },
		.lcd_rst = 	{ 26, AG7_3430_GPIO26 },
		.bkl_pwon = 	{ 53, V8_3430_GPIO53 }, 
		.lcd_pci = 	{ 0, 0 },
		.hdmi_dac = 	{ 28, AG8_3430_GPIO28 },
		.hdmi_it = 	{ 15, AG12_3430_GPIO15 },
		.cpld_aux = 	{ 21, AH14_3430_GPIO21 },
	},
	.rev[5] = {
		.cpldreset = 	{ 140, AF6_3430_GPIO140 },
		.disp_select = 	{ 160, T21_3430_GPIO160 },
		.lcd_pwon = 	{ 143, AE5_3430_GPIO143 },
		.lcd_rst = 	{ 141, AE6_3430_GPIO141 },
		.bkl_pwon = 	{  53, V8_3430_GPIO53   }, 
		.lcd_pci = 	{   0, 0 },
		.hdmi_dac = 	{ 152, AE1_3430_GPIO152 },
		.hdmi_it = 	{  15, AG12_3430_GPIO15 },
		.cpld_aux = 	{ 161, K26_3430_GPIO161 },
	},
	.rev[6] = {
		.cpldreset = 	{ 140, AF6_3430_GPIO140 },
		.disp_select = 	{ 160, T21_3430_GPIO160 },
		.lcd_pwon = 	{ 143, AE5_3430_GPIO143 },
		.lcd_rst = 	{ 141, AE6_3430_GPIO141 },
		.bkl_pwon = 	{  53, V8_3430_GPIO53   }, 
		.lcd_pci = 	{   0, 0 },
		.hdmi_dac = 	{ 152, AE1_3430_GPIO152 },
		.hdmi_it = 	{  15, AG12_3430_GPIO15 },
		.cpld_aux = 	{ 161, K26_3430_GPIO161 },
	},
};

static struct archosg6_keys_config keys_config __initdata = {
	.nrev = 7,
	.rev[0] = {
		.power = { .nb = 29, .mux_cfg = AH8_3430_GPIO29 },
		.vol_down = { .nb = 12, .mux_cfg = AF10_3430_GPIO12 },
		.vol_up = { .nb = 13, .mux_cfg = AE10_3430_GPIO13},
	},
	.rev[1] = {
		.power = { .nb = 29, .mux_cfg = AH8_3430_GPIO29 },
		.vol_down = { .nb = 12, .mux_cfg = AF10_3430_GPIO12 },
		.vol_up = { .nb = 13, .mux_cfg = AE10_3430_GPIO13},
	},
	.rev[2] = {
		.power = { .nb = 29, .mux_cfg = AH8_3430_GPIO29 },
		.vol_down = { .nb = 12, .mux_cfg = AF10_3430_GPIO12 },
		.vol_up = { .nb = 13, .mux_cfg = AE10_3430_GPIO13},
	},
	.rev[3] = {
		.power = { .nb = 0, .mux_cfg = 0 },
		.vol_down = { .nb = 12, .mux_cfg = AF10_3430_GPIO12 },
		.vol_up = { .nb = 13, .mux_cfg = AE10_3430_GPIO13},
	},
	.rev[4] = {
		.power = { .nb = 0, .mux_cfg = 0 },
		.vol_down = { .nb = 12, .mux_cfg = AF10_3430_GPIO12 },
		.vol_up = { .nb = 13, .mux_cfg = AE10_3430_GPIO13},
	},
	.rev[5] = {
		.power = { .nb = 0, .mux_cfg = 0 },
		.vol_down = { .nb = 12, .mux_cfg = AF10_3430_GPIO12 },
		.vol_up = { .nb = 13, .mux_cfg = AE10_3430_GPIO13},
	},
	.rev[6] = {
		.power = { .nb = 0, .mux_cfg = 0 },
		.vol_down = { .nb = 12, .mux_cfg = AF10_3430_GPIO12 },
		.vol_up = { .nb = 13, .mux_cfg = AE10_3430_GPIO13},
	},
};

static struct archosg6_usb_config usb_config __initdata = {
	.nrev = 7,
	.rev[0] = {
		.usb_id = { .nb = 102, .mux_cfg = C24_3430_GPIO102    },
		.enable_usb_musb = { .nb = 20,  .mux_cfg = AF13_3430_GPIO20   },
		.enable_usb_ehci = { .nb = 0, .mux_cfg = 0 },
	},
	.rev[1] = {
		.usb_id = { .nb = 102, .mux_cfg = C24_3430_GPIO102    },
		.enable_usb_musb = { .nb = 20,  .mux_cfg = AF13_3430_GPIO20   },
		.enable_usb_ehci = { .nb = 0, .mux_cfg = 0 },
	},
	.rev[2] = {
		.usb_id = { .nb = 156, .mux_cfg = Y21_3430_GPIO156    },
		.enable_usb_musb = { .nb = 20,  .mux_cfg = AF13_3430_GPIO20   },
		.enable_usb_ehci = { .nb = 0,  .mux_cfg = 0   },
	},
	.rev[3] = {
		.usb_id = { .nb = 156, .mux_cfg = Y21_3430_GPIO156    },
		.enable_usb_musb = { .nb = 20,  .mux_cfg = AF13_3430_GPIO20   },
		.enable_usb_ehci = { .nb = 0,  .mux_cfg = 0   },
	},
	.rev[4] = {
		.usb_id = { .nb = 156, .mux_cfg = Y21_3430_GPIO156    },
		.enable_usb_musb = { .nb = 20,  .mux_cfg = AF13_3430_GPIO20   },
		.enable_usb_ehci = { .nb = 0,  .mux_cfg = 0   },
	},
	.rev[5] = {
		.usb_id = { .nb = 156, .mux_cfg = Y21_3430_GPIO156    },
		.enable_usb_musb = { .nb = 20,  .mux_cfg = AF13_3430_GPIO20   },
		.enable_usb_ehci = { .nb = 153, .mux_cfg = AD1_3430_GPIO153},
	},
	.rev[6] = {
		.usb_id = { .nb = 156, .mux_cfg = Y21_3430_GPIO156    },
		.enable_usb_musb = { .nb = 20,  .mux_cfg = AF13_3430_GPIO20   },
		.enable_usb_ehci = { .nb = 153, .mux_cfg = AD1_3430_GPIO153},
	},
};

static struct archosg6_usbhdd_config usbhdd_config __initdata = {
	.nrev = 7,
	.rev[5] = {
		.hdd_power = { .nb = 18, .mux_cfg = AE11_3430_GPIO18 },
	},
	.rev[6] = {
		.hdd_power = { .nb = 18, .mux_cfg = AE11_3430_GPIO18 },
	},
};

static struct archosg6_audio_config audio_config __initdata = {
	.nrev = 7,
	.rev[0] = {
		.spdif = { .nb = 23, .mux_cfg = AG9_3430_GPIO23 },
		.hp_on = { .nb = 22, .mux_cfg = AF9_3430_GPIO22 },
		.headphone_plugged = { .nb = 17, .mux_cfg = AE13_3430_GPIO17},
	},
	.rev[1] = {
		.spdif = { .nb = 23, .mux_cfg = AG9_3430_GPIO23 },
		.hp_on = { .nb = 22, .mux_cfg = AF9_3430_GPIO22 },
		.headphone_plugged = { .nb = 17, .mux_cfg = AE13_3430_GPIO17},
	},
	.rev[2] = {
		.spdif = { .nb = 23, .mux_cfg = AG9_3430_GPIO23 },
		.hp_on = { .nb = 22, .mux_cfg = AF9_3430_GPIO22 },
		.headphone_plugged = { .nb = 17, .mux_cfg = AE13_3430_GPIO17},
	},
	.rev[3] = {
		.spdif = { .nb = 23, .mux_cfg = AG9_3430_GPIO23 },
		.hp_on = { .nb = 22, .mux_cfg = AF9_3430_GPIO22 },
		.headphone_plugged = { .nb = 17, .mux_cfg = AE13_3430_GPIO17},
	},
	.rev[4] = {
		.spdif = { .nb = 23, .mux_cfg = AG9_3430_GPIO23 },
		.hp_on = { .nb = 22, .mux_cfg = AF9_3430_GPIO22 },
		.headphone_plugged = { .nb = 17, .mux_cfg = AE13_3430_GPIO17},
	},
	.rev[5] = {
		.spdif = { .nb = 162, .mux_cfg = W21_3430_GPIO162 },
		.hp_on = { .nb = 157, .mux_cfg = AA21_3430_GPIO157 },
		.headphone_plugged = { .nb = 17, .mux_cfg = AE13_3430_GPIO17},
	},
	.rev[6] = {
		.spdif = { .nb = 162, .mux_cfg = W21_3430_GPIO162 },
		.hp_on = { .nb = 157, .mux_cfg = AA21_3430_GPIO157 },
		.headphone_plugged = { .nb = 17, .mux_cfg = AE13_3430_GPIO17},
	},
};

static struct g6_gpio hdd_id __initdata = { .nb = 57, .mux_cfg = P8_3430_GPIO57};

static struct omap_board_config_kernel board_config[] __initdata = {
	{ OMAP_TAG_UART,	&uart_config },
	{ OMAP_TAG_MMC,         &mmc_config },
	{ ARCHOS_TAG_TSP,	&tsp_config },
	{ ARCHOS_TAG_VBUS0,	&vbus0_config },
	{ ARCHOS_TAG_DISPLAY,	&display_config},
	{ ARCHOS_TAG_KEYS,	&keys_config},
	{ ARCHOS_TAG_USB,	&usb_config},
	{ ARCHOS_TAG_CHARGE,	&charge_config},
	{ ARCHOS_TAG_USBHDD,    &usbhdd_config},
	{ ARCHOS_TAG_AUDIO,     &audio_config},
	{ ARCHOS_TAG_WIFI,      &wifi_dev_conf},
};

extern void __init archosg6_usb_init(void);
extern int __init archosg6_display_init(void);
extern int __init archosg6_hsmmc_init(void);
extern int __init archosg6_wifi_init(void);
extern void archosg6_audio_gpio_init(void);
extern int __init archosg6_keys_init(void);
extern void __init fixup_archos_g6(struct machine_desc *desc, struct tag *tags, char **cmdline, struct meminfo *mi);
extern void archosg6_power_off(void);
extern int __init archosg6_usbhdd_init(void);

static void __init _board_init(void)
{
	int hdd_is_usb = 0;

	/* System Control module clock initialization */
	scm_clk_init();
	/* I2C init */
	omap3430_i2c_init();
	spi_lcd_init();

	if (hardware_rev >= 5) {
		/* on v1.6 pcb mmc1 is not in use */
		mmc_config.mmc[0].enabled = 0;
		
		_INIT_INPUT(hdd_id);
		hdd_is_usb = omap_get_gpio_datain(_PIN_NB(hdd_id)) == 0;
		/* if the HDD is connected via USB, disable MMC1 */
		if (hdd_is_usb) {
			printk(KERN_DEBUG "board_init: usb hdd found\n");
		} else {
			printk(KERN_DEBUG "board_init: ce-ata hdd found,"
						" enabling mmc3\n");
			mmc_config.mmc[2].enabled = 1;
		}
	}

	platform_add_devices(archos_devices, ARRAY_SIZE(archos_devices));
	omap_board_config = board_config;
	omap_board_config_size = ARRAY_SIZE(board_config);

	/* ads before spi */
	ads7846_dev_init(&g6s_spi_board_info[0]);

	spi_register_board_info(g6s_spi_board_info,
				ARRAY_SIZE(g6s_spi_board_info));

	omap_serial_init();
	archosg6_hsmmc_init();
	archosg6_wifi_init();
	archosg6_usb_init();
	if (hdd_is_usb)
		archosg6_usbhdd_init();
	archosg6_audio_gpio_init();
	archosg6_display_init();
	archosg6_keys_init();

	pm_power_off = archosg6_power_off;
}

static void __init _map_io(void)
{
	omap2_map_common_io();
}

MACHINE_START(ARCHOS_G6S, "ARCHOS G6S")
	/* Maintainer: Matthias Welwarsky, Archos S.A. */
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
