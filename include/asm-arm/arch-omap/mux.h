/*
 * linux/include/asm-arm/arch-omap/mux.h
 *
 * Table of the Omap register configurations for the FUNC_MUX and
 * PULL_DWN combinations.
 *
 * Copyright (C) 2003 - 2005 Nokia Corporation
 *
 * Written by Tony Lindgren <tony.lindgren@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * NOTE: Please use the following naming style for new pin entries.
 *	 For example, W8_1610_MMC2_DAT0, where:
 *	 - W8	     = ball
 *	 - 1610	     = 1510 or 1610, none if common for both 1510 and 1610
 *	 - MMC2_DAT0 = function
 *
 * Change log:
 *   Added entry for the I2C interface. (02Feb 2004)
 *   Copyright (C) 2004 Texas Instruments
 *
 *   Added entry for the keypad and uwire CS1. (09Mar 2004)
 *   Copyright (C) 2004 Texas Instruments
 *
 */

#ifndef __ASM_ARCH_MUX_H
#define __ASM_ARCH_MUX_H

#define PU_PD_SEL_NA		0	/* No pu_pd reg available */
#define PULL_DWN_CTRL_NA	0	/* No pull-down control needed */

#ifdef	CONFIG_OMAP_MUX_DEBUG
#define MUX_REG(reg, mode_offset, mode) .mux_reg_name = "FUNC_MUX_CTRL_"#reg, \
					.mux_reg = FUNC_MUX_CTRL_##reg, \
					.mask_offset = mode_offset, \
					.mask = mode,

#define PULL_REG(reg, bit, status)	.pull_name = "PULL_DWN_CTRL_"#reg, \
					.pull_reg = PULL_DWN_CTRL_##reg, \
					.pull_bit = bit, \
					.pull_val = status,

#define PU_PD_REG(reg, status)		.pu_pd_name = "PU_PD_SEL_"#reg, \
					.pu_pd_reg = PU_PD_SEL_##reg, \
					.pu_pd_val = status,

#define MUX_REG_730(reg, mode_offset, mode) .mux_reg_name = "OMAP730_IO_CONF_"#reg, \
					.mux_reg = OMAP730_IO_CONF_##reg, \
					.mask_offset = mode_offset, \
					.mask = mode,

#define PULL_REG_730(reg, bit, status)	.pull_name = "OMAP730_IO_CONF_"#reg, \
					.pull_reg = OMAP730_IO_CONF_##reg, \
					.pull_bit = bit, \
					.pull_val = status,

#else

#define MUX_REG(reg, mode_offset, mode) .mux_reg = FUNC_MUX_CTRL_##reg, \
					.mask_offset = mode_offset, \
					.mask = mode,

#define PULL_REG(reg, bit, status)	.pull_reg = PULL_DWN_CTRL_##reg, \
					.pull_bit = bit, \
					.pull_val = status,

#define PU_PD_REG(reg, status)		.pu_pd_reg = PU_PD_SEL_##reg, \
					.pu_pd_val = status,

#define MUX_REG_730(reg, mode_offset, mode) \
					.mux_reg = OMAP730_IO_CONF_##reg, \
					.mask_offset = mode_offset, \
					.mask = mode,

#define PULL_REG_730(reg, bit, status)	.pull_reg = OMAP730_IO_CONF_##reg, \
					.pull_bit = bit, \
					.pull_val = status,

#endif /* CONFIG_OMAP_MUX_DEBUG */

#define MUX_CFG(desc, mux_reg, mode_offset, mode,	\
		pull_reg, pull_bit, pull_status,	\
		pu_pd_reg, pu_pd_status, debug_status)	\
{							\
	.name =	 desc,					\
	.debug = debug_status,				\
	MUX_REG(mux_reg, mode_offset, mode)		\
	PULL_REG(pull_reg, pull_bit, pull_status)	\
	PU_PD_REG(pu_pd_reg, pu_pd_status)		\
},


/*
 * OMAP730 has a slightly different config for the pin mux.
 * - config regs are the OMAP730_IO_CONF_x regs (see omap730.h) regs and
 *   not the FUNC_MUX_CTRL_x regs from hardware.h
 * - for pull-up/down, only has one enable bit which is is in the same register
 *   as mux config
 */
#define MUX_CFG_730(desc, mux_reg, mode_offset, mode,	\
		   pull_bit, pull_status, debug_status)	\
{							\
	.name =	 desc,					\
	.debug = debug_status,				\
	MUX_REG_730(mux_reg, mode_offset, mode)		\
	PULL_REG_730(mux_reg, pull_bit, pull_status)	\
	PU_PD_REG(NA, 0)				\
},

#define MUX_CFG_24XX(desc, reg_offset, mode,		\
		     pull_en, pull_mode, dbg)		\
{							\
	.name		= desc,				\
	.debug		= dbg,				\
	.mux_reg	= OMAP2_CTRL_BASE + reg_offset,	\
	.mask		= mode,				\
	.pull_val	= pull_en,			\
	.pu_pd_val	= pull_mode,			\
},

#define MUX_CFG_34XX(desc, reg_offset, mode,		\
			pull_en, pull_mode,		\
			input_en,			\
			offen,				\
			offouten,			\
			offoutval,			\
			offpulluden,			\
			offpulltypesel,			\
			wakeupen)			\
[desc] = {						\
	.name		= __stringify(desc),		\
	.debug		= 0,				\
	.mux_reg	= OMAP2_CTRL_BASE + reg_offset,	\
	.mask		= mode,				\
	.pull_val	= pull_en,			\
	.pu_pd_val	= pull_mode,			\
	.input_en_val	= input_en,			\
	.offenable	= offen,			\
	.offoutenable	= offouten,			\
	.offoutvalue	= offoutval,			\
	.offpulludenable	= offpulluden,		\
	.offpulltypeselect	= offpulltypesel,	\
	.wakeupenable	= wakeupen			\
},

#define PULL_DISABLED	0
#define PULL_ENABLED	1

#define PULL_DOWN	0
#define PULL_UP		1

struct pin_config {
	char *name;
	unsigned char busy;
	unsigned char debug;

	const char *mux_reg_name;
	const unsigned int mux_reg;
	const unsigned char mask_offset;
	const unsigned char mask;

	const char *pull_name;
	const unsigned int pull_reg;
	const unsigned char pull_val;
	const unsigned char pull_bit;

	const char *pu_pd_name;
	const unsigned int pu_pd_reg;
	const unsigned char pu_pd_val;

	const unsigned char input_en_val;
	const unsigned char offenable;
	const unsigned char offoutenable;
	const unsigned char offoutvalue;
	const unsigned char offpulludenable;
	const unsigned char offpulltypeselect;
	const unsigned char wakeupenable;
};

enum omap730_index {
	/* OMAP 730 keyboard */
	E2_730_KBR0,
	J7_730_KBR1,
	E1_730_KBR2,
	F3_730_KBR3,
	D2_730_KBR4,
	C2_730_KBC0,
	D3_730_KBC1,
	E4_730_KBC2,
	F4_730_KBC3,
	E3_730_KBC4,

	/* USB */
	AA17_730_USB_DM,
	W16_730_USB_PU_EN,
	W17_730_USB_VBUSI,
};

enum omap1xxx_index {
	/* UART1 (BT_UART_GATING)*/
	UART1_TX = 0,
	UART1_RTS,

	/* UART2 (COM_UART_GATING)*/
	UART2_TX,
	UART2_RX,
	UART2_CTS,
	UART2_RTS,

	/* UART3 (GIGA_UART_GATING) */
	UART3_TX,
	UART3_RX,
	UART3_CTS,
	UART3_RTS,
	UART3_CLKREQ,
	UART3_BCLK,	/* 12MHz clock out */
	Y15_1610_UART3_RTS,

	/* PWT & PWL */
	PWT,
	PWL,

	/* USB master generic */
	R18_USB_VBUS,
	R18_1510_USB_GPIO0,
	W4_USB_PUEN,
	W4_USB_CLKO,
	W4_USB_HIGHZ,
	W4_GPIO58,

	/* USB1 master */
	USB1_SUSP,
	USB1_SEO,
	W13_1610_USB1_SE0,
	USB1_TXEN,
	USB1_TXD,
	USB1_VP,
	USB1_VM,
	USB1_RCV,
	USB1_SPEED,
	R13_1610_USB1_SPEED,
	R13_1710_USB1_SE0,

	/* USB2 master */
	USB2_SUSP,
	USB2_VP,
	USB2_TXEN,
	USB2_VM,
	USB2_RCV,
	USB2_SEO,
	USB2_TXD,

	/* OMAP-1510 GPIO */
	R18_1510_GPIO0,
	R19_1510_GPIO1,
	M14_1510_GPIO2,

	/* OMAP1610 GPIO */
	P18_1610_GPIO3,
	Y15_1610_GPIO17,

	/* OMAP-1710 GPIO */
	R18_1710_GPIO0,
	V2_1710_GPIO10,
	N21_1710_GPIO14,
	W15_1710_GPIO40,

	/* MPUIO */
	MPUIO2,
	N15_1610_MPUIO2,
	MPUIO4,
	MPUIO5,
	T20_1610_MPUIO5,
	W11_1610_MPUIO6,
	V10_1610_MPUIO7,
	W11_1610_MPUIO9,
	V10_1610_MPUIO10,
	W10_1610_MPUIO11,
	E20_1610_MPUIO13,
	U20_1610_MPUIO14,
	E19_1610_MPUIO15,

	/* MCBSP2 */
	MCBSP2_CLKR,
	MCBSP2_CLKX,
	MCBSP2_DR,
	MCBSP2_DX,
	MCBSP2_FSR,
	MCBSP2_FSX,

	/* MCBSP3 */
	MCBSP3_CLKX,

	/* Misc ballouts */
	BALLOUT_V8_ARMIO3,
	N20_HDQ,

	/* OMAP-1610 MMC2 */
	W8_1610_MMC2_DAT0,
	V8_1610_MMC2_DAT1,
	W15_1610_MMC2_DAT2,
	R10_1610_MMC2_DAT3,
	Y10_1610_MMC2_CLK,
	Y8_1610_MMC2_CMD,
	V9_1610_MMC2_CMDDIR,
	V5_1610_MMC2_DATDIR0,
	W19_1610_MMC2_DATDIR1,
	R18_1610_MMC2_CLKIN,

	/* OMAP-1610 External Trace Interface */
	M19_1610_ETM_PSTAT0,
	L15_1610_ETM_PSTAT1,
	L18_1610_ETM_PSTAT2,
	L19_1610_ETM_D0,
	J19_1610_ETM_D6,
	J18_1610_ETM_D7,

	/* OMAP16XX GPIO */
	P20_1610_GPIO4,
	V9_1610_GPIO7,
	W8_1610_GPIO9,
	N20_1610_GPIO11,
	N19_1610_GPIO13,
	P10_1610_GPIO22,
	V5_1610_GPIO24,
	AA20_1610_GPIO_41,
	W19_1610_GPIO48,
	M7_1610_GPIO62,
	V14_16XX_GPIO37,
	R9_16XX_GPIO18,
	L14_16XX_GPIO49,

	/* OMAP-1610 uWire */
	V19_1610_UWIRE_SCLK,
	U18_1610_UWIRE_SDI,
	W21_1610_UWIRE_SDO,
	N14_1610_UWIRE_CS0,
	P15_1610_UWIRE_CS3,
	N15_1610_UWIRE_CS1,

	/* OMAP-1610 SPI */
	U19_1610_SPIF_SCK,
	U18_1610_SPIF_DIN,
	P20_1610_SPIF_DIN,
	W21_1610_SPIF_DOUT,
	R18_1610_SPIF_DOUT,
	N14_1610_SPIF_CS0,
	N15_1610_SPIF_CS1,
	T19_1610_SPIF_CS2,
	P15_1610_SPIF_CS3,

	/* OMAP-1610 Flash */
	L3_1610_FLASH_CS2B_OE,
	M8_1610_FLASH_CS2B_WE,

	/* First MMC */
	MMC_CMD,
	MMC_DAT1,
	MMC_DAT2,
	MMC_DAT0,
	MMC_CLK,
	MMC_DAT3,

	/* OMAP-1710 MMC CMDDIR and DATDIR0 */
	M15_1710_MMC_CLKI,
	P19_1710_MMC_CMDDIR,
	P20_1710_MMC_DATDIR0,

	/* OMAP-1610 USB0 alternate pin configuration */
	W9_USB0_TXEN,
	AA9_USB0_VP,
	Y5_USB0_RCV,
	R9_USB0_VM,
	V6_USB0_TXD,
	W5_USB0_SE0,
	V9_USB0_SPEED,
	V9_USB0_SUSP,

	/* USB2 */
	W9_USB2_TXEN,
	AA9_USB2_VP,
	Y5_USB2_RCV,
	R9_USB2_VM,
	V6_USB2_TXD,
	W5_USB2_SE0,

	/* 16XX UART */
	R13_1610_UART1_TX,
	V14_16XX_UART1_RX,
	R14_1610_UART1_CTS,
	AA15_1610_UART1_RTS,
	R9_16XX_UART2_RX,
	L14_16XX_UART3_RX,

	/* I2C OMAP-1610 */
	I2C_SCL,
	I2C_SDA,

	/* Keypad */
	F18_1610_KBC0,
	D20_1610_KBC1,
	D19_1610_KBC2,
	E18_1610_KBC3,
	C21_1610_KBC4,
	G18_1610_KBR0,
	F19_1610_KBR1,
	H14_1610_KBR2,
	E20_1610_KBR3,
	E19_1610_KBR4,
	N19_1610_KBR5,

	/* Power management */
	T20_1610_LOW_PWR,

	/* MCLK Settings */
	V5_1710_MCLK_ON,
	V5_1710_MCLK_OFF,
	R10_1610_MCLK_ON,
	R10_1610_MCLK_OFF,

	/* CompactFlash controller */
	P11_1610_CF_CD2,
	R11_1610_CF_IOIS16,
	V10_1610_CF_IREQ,
	W10_1610_CF_RESET,
	W11_1610_CF_CD1,

	/* parallel camera */
	J15_1610_CAM_LCLK,
	J18_1610_CAM_D7,
	J19_1610_CAM_D6,
	J14_1610_CAM_D5,
	K18_1610_CAM_D4,
	K19_1610_CAM_D3,
	K15_1610_CAM_D2,
	K14_1610_CAM_D1,
	L19_1610_CAM_D0,
	L18_1610_CAM_VS,
	L15_1610_CAM_HS,
	M19_1610_CAM_RSTZ,
	Y15_1610_CAM_OUTCLK,

	/* serial camera */
	H19_1610_CAM_EXCLK,
	Y12_1610_CCP_CLKP,
	W13_1610_CCP_CLKM,
	W14_1610_CCP_DATAP,
	Y14_1610_CCP_DATAM,

};

enum omap24xx_index {
	/* 24xx I2C */
	M19_24XX_I2C1_SCL,
	L15_24XX_I2C1_SDA,
	J15_24XX_I2C2_SCL,
	H19_24XX_I2C2_SDA,

	/* 24xx Menelaus interrupt */
	W19_24XX_SYS_NIRQ,

	/* 24xx clock */
	W14_24XX_SYS_CLKOUT,

	/* 24xx GPMC chipselects, wait pin monitoring */
	E2_GPMC_NCS2,
	L2_GPMC_NCS7,
	L3_GPMC_WAIT0,
	N7_GPMC_WAIT1,
	M1_GPMC_WAIT2,
	P1_GPMC_WAIT3,

	/* 242X McBSP */
	Y15_24XX_MCBSP2_CLKX,
	R14_24XX_MCBSP2_FSX,
	W15_24XX_MCBSP2_DR,
	V15_24XX_MCBSP2_DX,

	/* 24xx GPIO */
	M21_242X_GPIO11,
	P21_242X_GPIO12,
	AA10_242X_GPIO13,
	AA6_242X_GPIO14,
	AA4_242X_GPIO15,
	Y11_242X_GPIO16,
	AA12_242X_GPIO17,
	AA8_242X_GPIO58,
	Y20_24XX_GPIO60,
	W4__24XX_GPIO74,
	M15_24XX_GPIO92,
	P20_24XX_GPIO93,
	P18_24XX_GPIO95,
	M18_24XX_GPIO96,
	L14_24XX_GPIO97,
	J15_24XX_GPIO99,
	V14_24XX_GPIO117,
	P14_24XX_GPIO125,

	/* 242x DBG GPIO */
	V4_242X_GPIO49,
	W2_242X_GPIO50,
	U4_242X_GPIO51,
	V3_242X_GPIO52,
	V2_242X_GPIO53,
	V6_242X_GPIO53,
	T4_242X_GPIO54,
	Y4_242X_GPIO54,
	T3_242X_GPIO55,
	U2_242X_GPIO56,

	/* 24xx external DMA requests */
	AA10_242X_DMAREQ0,
	AA6_242X_DMAREQ1,
	E4_242X_DMAREQ2,
	G4_242X_DMAREQ3,
	D3_242X_DMAREQ4,
	E3_242X_DMAREQ5,

	P20_24XX_TSC_IRQ,

	/* UART3 */
	K15_24XX_UART3_TX,
	K14_24XX_UART3_RX,

	/* MMC/SDIO */
	G19_24XX_MMC_CLKO,
	H18_24XX_MMC_CMD,
	F20_24XX_MMC_DAT0,
	H14_24XX_MMC_DAT1,
	E19_24XX_MMC_DAT2,
	D19_24XX_MMC_DAT3,
	F19_24XX_MMC_DAT_DIR0,
	E20_24XX_MMC_DAT_DIR1,
	F18_24XX_MMC_DAT_DIR2,
	E18_24XX_MMC_DAT_DIR3,
	G18_24XX_MMC_CMD_DIR,
	H15_24XX_MMC_CLKI,

	/* Full speed USB */
	J20_24XX_USB0_PUEN,
	J19_24XX_USB0_VP,
	K20_24XX_USB0_VM,
	J18_24XX_USB0_RCV,
	K19_24XX_USB0_TXEN,
	J14_24XX_USB0_SE0,
	K18_24XX_USB0_DAT,

	N14_24XX_USB1_SE0,
	W12_24XX_USB1_SE0,
	P15_24XX_USB1_DAT,
	R13_24XX_USB1_DAT,
	W20_24XX_USB1_TXEN,
	P13_24XX_USB1_TXEN,
	V19_24XX_USB1_RCV,
	V12_24XX_USB1_RCV,

	AA10_24XX_USB2_SE0,
	Y11_24XX_USB2_DAT,
	AA12_24XX_USB2_TXEN,
	AA6_24XX_USB2_RCV,
	AA4_24XX_USB2_TLLSE0,

	/* Keypad GPIO*/
	T19_24XX_KBR0,
	R19_24XX_KBR1,
	V18_24XX_KBR2,
	M21_24XX_KBR3,
	E5__24XX_KBR4,
	M18_24XX_KBR5,
	R20_24XX_KBC0,
	M14_24XX_KBC1,
	H19_24XX_KBC2,
	V17_24XX_KBC3,
	P21_24XX_KBC4,
	L14_24XX_KBC5,
	N19_24XX_KBC6,

	/* 24xx Menelaus Keypad GPIO */
	B3__24XX_KBR5,
	AA4_24XX_KBC2,
	B13_24XX_KBC6,

	/* 2430 USB */
	AD9_2430_USB0_PUEN,
	Y11_2430_USB0_VP,
	AD7_2430_USB0_VM,
	AE7_2430_USB0_RCV,
	AD4_2430_USB0_TXEN,
	AF9_2430_USB0_SE0,
	AE6_2430_USB0_DAT,
	AD24_2430_USB1_SE0,
	AB24_2430_USB1_RCV,
	Y25_2430_USB1_TXEN,
	AA26_2430_USB1_DAT,

	/* 2430 HS-USB */
	AD9_2430_USB0HS_DATA3,
	Y11_2430_USB0HS_DATA4,
	AD7_2430_USB0HS_DATA5,
	AE7_2430_USB0HS_DATA6,
	AD4_2430_USB0HS_DATA2,
	AF9_2430_USB0HS_DATA0,
	AE6_2430_USB0HS_DATA1,
	AE8_2430_USB0HS_CLK,
	AD8_2430_USB0HS_DIR,
	AE5_2430_USB0HS_STP,
	AE9_2430_USB0HS_NXT,
	AC7_2430_USB0HS_DATA7,

	/* 2430 McBSP */
	AC10_2430_MCBSP2_FSX,
	AD16_2430_MCBSP2_CLX,
	AE13_2430_MCBSP2_DX,
	AD13_2430_MCBSP2_DR,
	AC10_2430_MCBSP2_FSX_OFF,
	AD16_2430_MCBSP2_CLX_OFF,
	AE13_2430_MCBSP2_DX_OFF,
	AD13_2430_MCBSP2_DR_OFF,

	/* 2430 UART3 & GPIO */
	L2_2430_UART_IRTX,
	K2_2430_GPIO_103,
	L2_2430_GPIO_104,
};

enum omap34xx_index {

	/* PHY - HSUSB: 12-pin ULPI PHY: Port 1*/
	Y9_3430_USB1HS_PHY_STP,
	Y8_3430_USB1HS_PHY_CLK,
	W13_3430_USB1HS_PHY_DATA0,
	W12_3430_USB1HS_PHY_DATA1,
	W11_3430_USB1HS_PHY_DATA2,
	Y13_3430_USB1HS_PHY_DATA7,
	W9_3430_USB1HS_PHY_DATA4,
	Y12_3430_USB1HS_PHY_DATA5,
	W8_3430_USB1HS_PHY_DATA6,
	Y11_3430_USB1HS_PHY_DATA3,
	AA14_3430_USB1HS_PHY_DIR,
	AA11_3430_USB1HS_PHY_NXT,

	/* PHY - HSUSB: 12-pin ULPI PHY: Port 2, different pins */
	AE7_3430_USB2HS_PHY_CLK,
	AF7_3430_USB2HS_PHY_STP,
	AG7_3430_USB2HS_PHY_DIR,
	AH7_3430_USB2HS_PHY_NXT,
	AG8_3430_USB2HS_PHY_DATA0,
	AH8_3430_USB2HS_PHY_DATA1,
	AB2_3430_USB2HS_PHY_DATA2,
	V3_3430_USB2HS_PHY_DATA3,
	Y2_3430_USB2HS_PHY_DATA4,
	Y3_3430_USB2HS_PHY_DATA5,
	Y4_3430_USB2HS_PHY_DATA6,
	AA3_3430_USB2HS_PHY_DATA7,

	/* TLL - HSUSB: 12-pin TLL Port 1*/
	Y9_3430_USB1HS_TLL_STP,
	Y8_3430_USB1HS_TLL_CLK,
	W13_3430_USB1HS_TLL_DATA0,
	W12_3430_USB1HS_TLL_DATA1,
	W11_3430_USB1HS_TLL_DATA2,
	Y13_3430_USB1HS_TLL_DATA7,
	W9_3430_USB1HS_TLL_DATA4,
	Y12_3430_USB1HS_TLL_DATA5,
	W8_3430_USB1HS_TLL_DATA6,
	Y11_3430_USB1HS_TLL_DATA3,
	AA14_3430_USB1HS_TLL_DIR,
	AA11_3430_USB1HS_TLL_NXT,

	/* TLL - HSUSB: 12-pin TLL Port 2*/
	AA8_3430_USB2HS_TLL_CLK,
	AA10_3430_USB2HS_TLL_STP,
	AA9_3430_USB2HS_TLL_DIR,
	AB11_3430_USB2HS_TLL_NXT,
	AB10_3430_USB2HS_TLL_DATA0,
	AB9_3430_USB2HS_TLL_DATA1,
	W3_3430_USB2HS_TLL_DATA2,
	T2_3430_USB2HS_TLL_DATA7,
	T3_3430_USB2HS_TLL_DATA4,
	R3_3430_USB2HS_TLL_DATA5,
	R4_3430_USB2HS_TLL_DATA6,
	T4_3430_USB2HS_TLL_DATA3,

	/* TLL - HSUSB: 12-pin TLL Port 3*/
	AB3_3430_USB3HS_TLL_STP,
	AA3_3430_USB3HS_TLL_DIR,
	Y3_3430_USB3HS_TLL_NXT,
	AB12_3430_USB3HS_TLL_DATA4,
	AB13_3430_USB3HS_TLL_DATA5,
	AA13_3430_USB3HS_TLL_DATA6,
	AA12_3430_USB3HS_TLL_DATA7,
	AA6_3430_USB3HS_TLL_CLK,
	Y4_3430_USB3HS_TLL_DATA1,
	AA5_3430_USB3HS_TLL_DATA0,
	Y5_3430_USB3HS_TLL_DATA2,
	W5_3430_USB3HS_TLL_DATA3,

        /* PHY FSUSB: FS Serial PHY 4-pin mode for Port 1*/
	AF10_3430_USB1FS_PHY_MM1_RXDP,
	AG9_3430_USB1FS_PHY_MM1_RXDM,
        W13_3430_USB1FS_PHY_MM1_RXRCV,
        W12_3430_USB1FS_PHY_MM1_TXSE0,
        W11_3430_USB1FS_PHY_MM1_TXDAT,
        Y11_3430_USB1FS_PHY_MM1_TXEN_N,

	AA14_3430_ISP1301_GPIO22,

        /* PHY FSUSB: FS Serial PHY 4-pin mode for Port 2*/
        AF7_3430_USB2FS_PHY_MM2_RXDP,
        AH7_3430_USB2FS_PHY_MM2_RXDM,
        AB10_3430_USB2FS_PHY_MM2_RXRCV,
        AB9_3430_USB2FS_PHY_MM2_TXSE0,
        W3_3430_USB2FS_PHY_MM2_TXDAT,
        T4_3430_USB2FS_PHY_MM2_TXEN_N,

        /* PHY FSUSB: FS Serial PHY 4-pin mode for Port 3*/
        AH3_3430_USB3FS_PHY_MM3_RXDP,
        AE3_3430_USB3FS_PHY_MM3_RXDM,
        AD1_3430_USB3FS_PHY_MM3_RXRCV,
        AE1_3430_USB3FS_PHY_MM3_TXSE0,
        AD2_3430_USB3FS_PHY_MM3_TXDAT,
        AC1_3430_USB3FS_PHY_MM3_TXEN_N,

	H18_3430_UART3_CTS_RCTX,
	H19_3430_UART3_RTS_SD,
	H20_3430_UART3_RX_IRRX,
	H21_3430_UART3_TX_IRTX,
	
	AD25_3430_UART2_RX,
	AB26_3430_UART2_CTS, /* IRRmote */
	
	H19_3430_GPIO164,
	V21_3430_GPIO158,
	AD25_3430_GPIO147,
	AD25_3430_GPT08,

	/* ATMEGA interrput */
	J25_3430_GPIO170,

	/* ethernet LAN9211 */
	/* and also speaker on */
	AF9_3430_GPIO22,
	AF5_3430_GPIO142,
	AE5_3430_GPIO143,
	AE6_3430_GPIO141,

	/* ethernet LAN9211 EVM*/
	R8_3430_CS5,
 	U8_3430_GPIO54,
	V8_3430_GPIO53,
	T8_3430_GPIO55,
	P8_3430_GPIO57,
	N8_3430_GPIO58,

	/* CPLD RESET */
	AF6_3430_GPIO140,

	AG12_3430_GPIO15 /* hdmi interrupt*/,

	/* TSC on SDP */
	AF15_3430_GPIO142,	/* IRQ */
	AF15_3430_GPIO113,	/* PWR ON */

	/* LCD BKL */
	AB25_3430_GPT10,

	/* HDMI on EVM , or DVB-T on G6TV */
	AA4_3430_GPIO173, /*PRWDN for HDMI, 3V3 on for DVB-T */
	
	AB1_3430_GPIO176, /*hdmi interrupt and tvp power down for g6tv */

	/* TSC on G6S/G6H */
	AE7_3430_GPIO24,	/* IRQ */
	AH19_3430_GPIO113,	/* PWRON */
	
	/* DSS */
	D28_3430_DSS_PCLK,
	D26_3430_DSS_HSYNC,
	D27_3430_DSS_VSYNC,
	E27_3430_DSS_ACBIAS,
	AG22_3430_DSS_DATA0,
	AH22_3430_DSS_DATA1,
	AG23_3430_DSS_DATA2,
	AH23_3430_DSS_DATA3,
	AG24_3430_DSS_DATA4,
	AH24_3430_DSS_DATA5,
	E26_3430_DSS_DATA6,
	F28_3430_DSS_DATA7,
	F27_3430_DSS_DATA8,
	G26_3430_DSS_DATA9,
	AD28_3430_DSS_DATA10,
	AD27_3430_DSS_DATA11,
	AB28_3430_DSS_DATA12,
	AB27_3430_DSS_DATA13,
	AA28_3430_DSS_DATA14,
	AA27_3430_DSS_DATA15,
	G25_3430_DSS_DATA16,
	H27_3430_DSS_DATA17,
	H26_3430_DSS_DATA18,
	H25_3430_DSS_DATA19,
	E28_3430_DSS_DATA20,
	J26_3430_DSS_DATA21,
	AC27_3430_DSS_DATA22,
	AC28_3430_DSS_DATA23,
	
	/* ENABLE_USB */
	AF13_3430_GPIO20,
	/* 5VUSB_ON */
	AE22_3430_GPIO186,

	/*  headphone plugged */
	AE13_3430_GPIO17,

	/*  spdif en */
	AG9_3430_GPIO23,

	/* lcd cmd */
	AH7_3430_GPIO27,
	AF11_3430_GPIO14,
	Y8_3430_GPIO151,
	AF7_3430_GPIO25,
	AG7_3430_GPIO26,
	AB25_3430_GPIO145,

	/* New TSC PWR ON */
	AA8_3430_GPIO148,

	/* HDMI/DAC for g6h */
	AE1_3430_GPIO152,

	/* HDMI/DAC select for g6s */
	AG8_3430_GPIO28,

	/* CPLD AUX for g6: for adv field control */
	AH14_3430_GPIO21,

	/* HSDPA for g6gplus */
	AE4_3430_GPIO136,
	AD1_3430_GPIO153,
	AD2_3430_GPIO154,

	/* valide I2C2 pins  */
	AE15_3430_I2C2_SDA,
	AF15_3430_I2C2_SCL,

/* DC-in detection of battry dock */
	AF14_3430_I2C3_SCL,
	AF14_3430_GPIO184,

	AG14_3430_I2C3_SDA,
	AG14_3430_GPIO185,

	/* DVB-T or TVP plugged on the I/O Camera port */
	AF10_3430_GPIO12, // DVBT_TVP switch or keyb vol+

	/* DVB-T pins */
	AB3_3430_GPIO171, // 1V8_DVBT
	AB4_3430_GPIO172, // 5V_DVBT
	AC2_3430_GPIO174, // RST_DVBT
	
	/* TVP pins */
	AC3_3430_GPIO175, // RST_TVP

	/*  CAM reset */
	C23_3430_GPIO98,
	/* usb id */
	R8_3430_GPIO56,
	B24_3430_GPIO101,
	C24_3430_GPIO102,
	Y21_3430_GPIO156,

	AE10_3430_GPIO13, // keyb vol-
	AC1_3430_GPIO155,  // keyb on
	AH8_3430_GPIO29,
	
	AE11_3430_GPIO18, /* HDD_PWR_ON */
	H3_3430_GPIO52,  /* SATA_RDY */

	N28_3430_GPIO120,  /* Hub usb gplus */
	AE22_3430_CLKOUT2,

	AH3_3430_GPIO137,
	AF3_3430_GPIO138,
	AE3_3430_GPIO139,
	AA3_3430_MCPSI2_CLK,
	Y2_3430_MCSPI2_SIMO,
	Y3_3430_MCSPI2_SOMI,
	Y4_3430_MCSPI2_CS0,
	H20_3430_GPIO165,

	K26_3430_GPIO161,
	W21_3430_GPIO162,
	AA21_3430_GPIO157,
	U21_3430_GPIO159,

	AA3_3430_GPIO178,
	Y2_3430_GPIO179,
	Y3_3430_GPIO180,
	Y4_3430_GPIO181,
	H21_3430_GPIO166,

	T21_3430_GPIO160,

	/* MMC3 on G6S v1.6 PCB */
	AB1_3430_MMC3_CLK,
	AC3_3430_MMC3_CMD,
	AE4_3430_MMC3_DAT0,
	AH3_3430_MMC3_DAT1,
	AF3_3430_MMC3_DAT2,
	AE3_3430_MMC3_DAT3,
	AF11_3430_MMC3_DAT4,
	AG9_3430_MMC3_DAT5,
	AF9_3430_MMC3_DAT6,
	AH14_3430_MMC3_DAT7,

	/* MMC2 on all units */
	AE2_3430_MMC2_CLK,
	AG5_3430_MMC2_CMD,
	AH5_3430_MMC2_DAT0,
	AH4_3430_MMC2_DAT1,
	AG4_3430_MMC2_DAT2,
	AF4_3430_MMC2_DAT3,
	AE2_3430_SAFE,
	AG5_3430_SAFE,
	AH5_3430_SAFE,
	AH4_3430_SAFE,
	AG4_3430_SAFE,
	AF4_3430_SAFE,

	AB26_3430_SAFE, /* IRRmote off */
};

#ifdef	CONFIG_OMAP_MUX
/* setup pin muxing in Linux */
extern int omap1_mux_init(void);
extern int omap2_mux_init(void);
extern int omap3_mux_init(void);
extern int omap_mux_register(struct pin_config * pins, unsigned long size);
extern int omap_cfg_reg(unsigned long reg_cfg);
#else
/* boot loader does it all (no warnings from CONFIG_OMAP_MUX_WARNINGS) */
static inline int omap1_mux_init(void) { return 0; }
static inline int omap2_mux_init(void) { return 0; }
static inline int omap3_mux_init(void) { return 0; }
static inline int omap_cfg_reg(unsigned long reg_cfg) { return 0; }
#endif

#endif
