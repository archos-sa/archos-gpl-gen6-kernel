/*
 * linux/arch/arm/mach-omap2/board-omap3evm.c
 *
 * Copyright (C) 2007 Texas Instruments
 *
 * Modified from mach-omap2/board-3430sdp.c
 *
 * Initial code: Syed Mohammed Khasim
 * Modified for OMAP3EVM: Nagarjun (nagarjun@mistralsolutions.com)
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
#include <linux/spi/ads7846.h>

#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#include <asm/arch/irda.h>
#include <asm/arch/board.h>
#include <asm/arch/common.h>
#include <asm/arch/keypad.h>
#include <asm/arch/dma.h>
#include <asm/arch/gpmc.h>
#include <asm/arch/twl4030-rtc.h>
#include <asm/arch/mcspi.h>

#include <asm/io.h>
#include <asm/delay.h>


/* =============================================================================
 * System Control Module
 * =============================================================================
 */
#define CONTROL_SYSC_SMARTIDLE	(0x2 << 3)
#define CONTROL_SYSC_AUTOIDLE	(0x1)

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

	/* Enable SMARTIDLE and AUTOIDLE */
	CONTROL_SYSCONFIG = (CONTROL_SYSC_SMARTIDLE | CONTROL_SYSC_AUTOIDLE);
}


/* =============================================================================
 * TWL4030
 * =============================================================================
 */
#define TWL4030_MSECURE_GPIO    22

#ifdef CONFIG_RTC_DRV_TWL4030
static int twl4030_rtc_init(void)
{
	int ret = 0;

	/* 3430ES2.0 doesn't have msecure/gpio-22 line connected to T2 */
	if (is_device_type_gp() && is_sil_rev_less_than(OMAP3430_REV_ES2_0)) {
		u32 msecure_pad_config_reg = OMAP2_CTRL_BASE + 0xA3C;
		int mux_mask = 0x04;
		u16 tmp;

		ret = omap_request_gpio(TWL4030_MSECURE_GPIO);
		if (ret < 0) {
			printk(KERN_ERR "twl4030_rtc_init: can't"
				"reserve GPIO:%d !\n", TWL4030_MSECURE_GPIO);
			goto out;
		}

		/*
		* TWL4030 will be in secure mode if msecure line from OMAP
		* is low. Make msecure line high in order to change the
		* TWL4030 RTC time and calender registers.
		*/
		omap_set_gpio_direction(TWL4030_MSECURE_GPIO, 0);

		tmp = omap_readw(msecure_pad_config_reg);
		tmp &= 0xF8;	/* To enable mux mode 03/04 = GPIO_RTC */
		tmp |= mux_mask;/* To enable mux mode 03/04 = GPIO_RTC */
		omap_writew(tmp, msecure_pad_config_reg);

		omap_set_gpio_dataout(TWL4030_MSECURE_GPIO, 1);
	}
out:
	return ret;
}

static void twl4030_rtc_exit(void)
{
	omap_free_gpio(TWL4030_MSECURE_GPIO);
}

static struct twl4030rtc_platform_data omap3evm_twl4030rtc_data = {
	.init = &twl4030_rtc_init,
	.exit = &twl4030_rtc_exit,
};

static struct platform_device omap3evm_twl4030rtc_device = {
	.name		= "twl4030_rtc",
	.id		= -1,
	.dev            = {
				.platform_data	= &omap3evm_twl4030rtc_data,
			},
};
#endif


/* =============================================================================
 * Touchscreen
 *
 * The TSC2046 is a next-generation version to the ADS7846 4-wire touch screen
 * controller. The TSC2046 is 100% pin-compatible with the existing ADS7846,
 * drops into the same socket.
 *
 * Hence, ads7846 driver is used for touchscreen.
 * =============================================================================
 */
#ifdef CONFIG_OMAP3430_ES2
#define TS_GPIO                175 
#else
#define TS_GPIO                3 
#endif

extern int twl4030_vaux1_ldo_use(void);
extern int twl4030_vaux1_ldo_unuse(void);

extern int twl4030_vaux3_ldo_use(void);
extern int twl4030_vaux3_ldo_unuse(void);

static void ads7846_dev_init(void)
{
	if (omap_request_gpio(TS_GPIO) < 0)
		printk(KERN_ERR "can't get ads746 pen down GPIO\n");

	omap_set_gpio_direction(TS_GPIO, 1);

	omap_set_gpio_debounce(TS_GPIO, 1);
	omap_set_gpio_debounce_time(TS_GPIO, 0xa);
}

static int ads7846_get_pendown_state(void)
{
	return !omap_get_gpio_datain(TS_GPIO);
}

/* This enable(1)/disable(0) the voltage for TS: uses twl4030 calls */
static int ads7846_vaux_control(int vaux_cntrl)
{
	int ret = 0;
#if defined (CONFIG_TWL4030_POWER)
	/* check for return value of ldo_use: if success it returns 0*/
	if (vaux_cntrl == VAUX_ENABLE)
		ret = twl4030_vaux3_ldo_use();
	else if (vaux_cntrl == VAUX_DISABLE)
		ret = twl4030_vaux3_ldo_unuse();
#endif
	return ret;
}

struct ads7846_platform_data tsc2046_config = {
	.get_pendown_state = ads7846_get_pendown_state,
	.keep_vref_on	   = 1,
	.vaux_control		= ads7846_vaux_control,
};


static struct omap2_mcspi_device_config tsc2046_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel = 1,  /* 0: slave, 1: master */
};

struct spi_board_info omap3evm_spi_board_info[] = {
	[0] = {
		/* TSC2046 operates at a max freqency of 2MHz, so
		 * operate slightly below at 1.5MHz
		 */
		.modalias	= "ads7846",
		.bus_num	= 1,
		.chip_select	= 0,
		.max_speed_hz   = 1500000,
		.controller_data= &tsc2046_mcspi_config,
		.irq		 = OMAP_GPIO_IRQ(TS_GPIO),
		.platform_data  = &tsc2046_config,
	},
};


/* =============================================================================
 * Keypad
 * =============================================================================
 */

static int omap3evm_keymap[] = {
	KEY(0, 0, KEY_LEFT),
	KEY(0, 1, KEY_RIGHT),
	KEY(0, 2, KEY_A),
	KEY(0, 3, KEY_B),
	KEY(0, 4, KEY_C),
	KEY(1, 0, KEY_DOWN),
	KEY(1, 1, KEY_UP),
	KEY(1, 2, KEY_E),
	KEY(1, 3, KEY_F),
	KEY(1, 4, KEY_G),
	KEY(2, 0, KEY_ENTER),
	KEY(2, 1, KEY_I),
	KEY(2, 2, KEY_J),
	KEY(2, 3, KEY_K),
	KEY(2, 4, KEY_3),
	KEY(3, 0, KEY_M),
	KEY(3, 1, KEY_N),
	KEY(3, 2, KEY_O),
	KEY(3, 3, KEY_P),
	KEY(3, 4, KEY_Q),
	KEY(4, 0, KEY_R),
	KEY(4, 1, KEY_4),
	KEY(4, 2, KEY_T),
	KEY(4, 3, KEY_U),
	KEY(4, 4, KEY_D),
	KEY(5, 0, KEY_V),
	KEY(5, 1, KEY_W),
	KEY(5, 2, KEY_L),
	KEY(5, 3, KEY_S),
	KEY(5, 4, KEY_H),
	0
};

static struct omap_kp_platform_data omap3evm_kp_data = {
	.rows		= 4,
	.cols		= 4,
	.keymap 	= omap3evm_keymap,
	.keymapsize 	= ARRAY_SIZE(omap3evm_keymap),
	.rep		= 1,
};

static struct platform_device omap3evm_kp_device = {
	.name		= "omap_twl4030keypad",
	.id		= -1,
	.dev		= {
		.platform_data = &omap3evm_kp_data,
	},
};

/* =============================================================================
 * Ethernet
 * =============================================================================
 */
#define OMAP3EVM_ETHR_START  0x2c000000
#define OMAP3EVM_ETHR_SIZE   1024
#define OMAP3EVM_ETHR_GPIO_IRQ 176 

#define OMAP3EVM_SMC911X_CS        5

static struct resource omap3evm_smc911x_resources[] = {
    [0] = {
                .start  = OMAP3EVM_ETHR_START,
            .end    = (OMAP3EVM_ETHR_START + OMAP3EVM_ETHR_SIZE - 1),
                .flags  = IORESOURCE_MEM,
          },
    [1] = {
                .start  = OMAP_GPIO_IRQ(OMAP3EVM_ETHR_GPIO_IRQ),
            .end    = OMAP_GPIO_IRQ(OMAP3EVM_ETHR_GPIO_IRQ),
                .flags  = IORESOURCE_IRQ,
          },
};

static struct platform_device omap3evm_smc911x_device = {
    .name       = "smc911x",
    .id     = -1,
    .num_resources  = ARRAY_SIZE(omap3evm_smc911x_resources),
    .resource   = &omap3evm_smc911x_resources [0],
};

static inline void __init omap3evm_init_smc911x(void)
{
        int eth_cs;

        eth_cs = OMAP3EVM_SMC911X_CS;

        if (omap_request_gpio(OMAP3EVM_ETHR_GPIO_IRQ) < 0) {
                printk(KERN_ERR "Failed to request GPIO%d for smc911x IRQ\n",
                        OMAP3EVM_ETHR_GPIO_IRQ);
                return;
        }
        omap_set_gpio_direction(OMAP3EVM_ETHR_GPIO_IRQ, 1);
}

/* =============================================================================
 * UART
 * =============================================================================
		 */
static struct omap_uart_config omap3evm_uart_config __initdata = {
	.enabled_uarts	= ((1 << 0) | (1 << 1) | (1 << 2)),
};

#define UART1_INTERRUPT_MASK	(1 << 8)
#define UART2_INTERRUPT_MASK    (1 << 9)
#define UART3_INTERRUPT_MASK    (1 << 10)

unsigned int uart_interrupt_mask_value;
static u32 *uart_detect(void){
	char str[7];
	u32 *temp_ptr = 0;
	if (console_detect(str))
		printk(KERN_INFO"Invalid console paramter....\n");

	if (!strcmp(str, "ttyS0")) {
		temp_ptr = (u32 *)(&CONTROL_PADCONF_UART1_CTS);
		uart_interrupt_mask_value = UART1_INTERRUPT_MASK;
		}
	else if (!strcmp(str, "ttyS1")) {
		temp_ptr = (u32 *)(&CONTROL_PADCONF_UART2_TX);
		uart_interrupt_mask_value = UART2_INTERRUPT_MASK;
		}
	else if (!strcmp(str, "ttyS2")) {
		temp_ptr = (u32 *)(&CONTROL_PADCONF_UART3_RTS_SD);
		uart_interrupt_mask_value = UART3_INTERRUPT_MASK;
		}
	else
		printk(KERN_INFO
		"!!!!!!!!! Unable to recongnize Console UART........\n");
	return (u32 *)(temp_ptr);
}
void  init_wakeupconfig(void){
	u32 *ptr;
	ptr = (u32 *)uart_detect();
	*ptr |= (u32)((IO_PAD_WAKEUPENABLE | IO_PAD_OFFPULLUDENABLE |
			IO_PAD_OFFOUTENABLE | IO_PAD_OFFENABLE |
			IO_PAD_INPUTENABLE | IO_PAD_PULLUDENABLE)
			<<  IO_PAD_HIGH_SHIFT);

}

void uart_padconf_control(void){
	u32 *ptr;
	ptr = (u32 *)uart_detect();
		*ptr |= (u32)((IO_PAD_WAKEUPENABLE)
			 << IO_PAD_HIGH_SHIFT);
}

/* =============================================================================
 * I2C
 * =============================================================================
 */
static int __init omap3430_i2c_init(void)
{
	omap_register_i2c_bus(1, CONFIG_I2C_OMAP34XX_HS_BUS1, NULL, 0);
	omap_register_i2c_bus(2, CONFIG_I2C_OMAP34XX_HS_BUS2, NULL, 0);
	omap_register_i2c_bus(3, CONFIG_I2C_OMAP34XX_HS_BUS3, NULL, 0);
	return 0;
}

/* =============================================================================
 * MMC
 * =============================================================================
 */
static struct omap_mmc_config omap3evm_mmc_config __initdata = {
	.mmc [0] = {
		.enabled        = 1,
		.wire4          = 1,
		.wp_pin         = -1,
		.power_pin      = -1,
		.switch_pin     = 0,
	},
	.mmc [1] = {
		.enabled        = 1,
		.wire4          = 1,
		.wp_pin         = -1,
		.power_pin      = -1,
		.switch_pin     = 1,
	},
	.mmc [2] = {
		.enabled        = 1,
		.wire4          = 1,
		.wp_pin         = -1,
		.power_pin      = -1,
		.switch_pin     = 1,	
	},
};

/* =============================================================================
 * Board level initialization
 * =============================================================================
 */
#include "prcm-regs.h"
#define PRCM_INTERRUPT_MASK	(1 << 11)

extern void __init sdp_usb_init(void);
extern void __init omap3evm_flash_init(void);

void setup_board_wakeup_source(u32 wakeup_source)
{
	if (wakeup_source & PRCM_WAKEUP_T2_KEYPAD)
	{
		PRCM_GPIO1_SYSCONFIG = 0x15;
		PM_WKEN_WKUP |= 0x8;     /* en gpio block 1 wakeup */
		PM_MPUGRPSEL_WKUP = 0x8; /* route wake to MPU */
		INTC_MIR_0 = ~((1<<29)); /* unmask GPIO block 1 at MPU pic */
		/* mode switch from nIRQ to GPIO-block1-source0 */
		CONTROL_PADCONF_SYS_NIRQ &= 0xFFFFFFF8;
		CONTROL_PADCONF_SYS_NIRQ |= 0x4;
		/* Enable wake up trigger in GPIO1 */
		GPIO1_SETIRQENABLE1 |= 0x1;
		GPIO1_SETWKUENA |= 0x1;
		GPIO1_FALLINGDETECT |= 0x1;

	}
	if (wakeup_source & PRCM_WAKEUP_TOUCHSCREEN)
	{
		INTC_MIR_1 = ~((1<<3));//Unmask GPIO module 6 for TS
	}
	/* Unmasking the PRCM interrupts */
	INTC_MIR_0 &= ~PRCM_INTERRUPT_MASK;
	 /* Unmasking the UART interrupts */
	INTC_MIR_2 = ~uart_interrupt_mask_value;
}

static void __init omap3evm_init_irq(void)
{
	omap2_init_common_hw();
	omap_init_irq();
	omap_gpio_init();
	omap3evm_init_smc911x();
}

static struct omap_board_config_kernel omap3evm_config[] __initdata = {
	{ OMAP_TAG_UART,	&omap3evm_uart_config },
	{ OMAP_TAG_MMC,         &omap3evm_mmc_config },
};

static struct platform_device *omap3evm_devices[] __initdata = {
	&omap3evm_smc911x_device,
	&omap3evm_kp_device,
#ifdef CONFIG_RTC_DRV_TWL4030
	&omap3evm_twl4030rtc_device,
#endif
};

static void __init omap3evm_init(void)
{
	scm_clk_init();	
	platform_add_devices(omap3evm_devices, ARRAY_SIZE(omap3evm_devices));
	omap_board_config = omap3evm_config;
	omap_board_config_size = ARRAY_SIZE(omap3evm_config);
	spi_register_board_info(omap3evm_spi_board_info,
				ARRAY_SIZE(omap3evm_spi_board_info));
	omap3evm_flash_init();
	ads7846_dev_init();
	omap_serial_init();

	sdp_usb_init();
}
arch_initcall(omap3430_i2c_init);

static void __init omap3evm_map_io(void)
{
	omap2_map_common_io();
}

MACHINE_START(OMAP3_EVM, "OMAP3 omap3evm board")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap3evm_map_io,
	.init_irq	= omap3evm_init_irq,
	.init_machine	= omap3evm_init,
	.timer		= &omap_timer,
MACHINE_END
