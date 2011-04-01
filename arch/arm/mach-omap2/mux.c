/*
 * linux/arch/arm/mach-omap2/mux.c
 *
 * OMAP2 and OMAP3 pin multiplexing configurations
 *
 * Copyright (C) 2007 Texas Instruments Inc.
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
 */
#include <linux/module.h>
#include <linux/init.h>
#include <asm/system.h>
#include <asm/io.h>
#include <linux/spinlock.h>

#include <asm/arch/mux.h>

#ifdef CONFIG_OMAP_MUX

/* NOTE: See mux.h for the enumeration */

struct pin_config __initdata_or_module omap24xx_pins[] = {
/*
 *	description			mux	mux	pull	pull	debug
 *					offset	mode	ena	type
 */

/* 24xx I2C */
MUX_CFG_24XX("M19_24XX_I2C1_SCL",	0x111,	0,	0,	0,	1)
MUX_CFG_24XX("L15_24XX_I2C1_SDA",	0x112,	0,	0,	0,	1)
MUX_CFG_24XX("J15_24XX_I2C2_SCL",	0x113,	0,	0,	1,	1)
MUX_CFG_24XX("H19_24XX_I2C2_SDA",	0x114,	0,	0,	0,	1)

/* Menelaus interrupt */
MUX_CFG_24XX("W19_24XX_SYS_NIRQ",	0x12c,	0,	1,	1,	1)

/* 24xx clocks */
MUX_CFG_24XX("W14_24XX_SYS_CLKOUT",	0x137,	0,	1,	1,	1)

/* 24xx GPMC chipselects, wait pin monitoring */
MUX_CFG_24XX("E2_GPMC_NCS2",		0x08e,	0,	1,	1,	1)
MUX_CFG_24XX("L2_GPMC_NCS7",		0x093,	0,	1,	1,	1)
MUX_CFG_24XX("L3_GPMC_WAIT0",		0x09a,	0,	1,	1,	1)
MUX_CFG_24XX("N7_GPMC_WAIT1",		0x09b,	0,	1,	1,	1)
MUX_CFG_24XX("M1_GPMC_WAIT2",		0x09c,	0,	1,	1,	1)
MUX_CFG_24XX("P1_GPMC_WAIT3",		0x09d,	0,	1,	1,	1)

/* 24xx McBSP */
MUX_CFG_24XX("Y15_24XX_MCBSP2_CLKX",	0x124,	1,	1,	0,	1)
MUX_CFG_24XX("R14_24XX_MCBSP2_FSX",	0x125,	1,	1,	0,	1)
MUX_CFG_24XX("W15_24XX_MCBSP2_DR",	0x126,	1,	1,	0,	1)
MUX_CFG_24XX("V15_24XX_MCBSP2_DX",	0x127,	1,	1,	0,	1)

/* 24xx GPIO */
MUX_CFG_24XX("M21_242X_GPIO11",		0x0c9,	3,	1,	1,	1)
MUX_CFG_24XX("P21_242X_GPIO12",		0x0ca,	3,	0,	0,	1)
MUX_CFG_24XX("AA10_242X_GPIO13",	0x0e5,	3,	0,	0,	1)
MUX_CFG_24XX("AA6_242X_GPIO14",		0x0e6,	3,	0,	0,	1)
MUX_CFG_24XX("AA4_242X_GPIO15",		0x0e7,	3,	0,	0,	1)
MUX_CFG_24XX("Y11_242X_GPIO16",		0x0e8,	3,	0,	0,	1)
MUX_CFG_24XX("AA12_242X_GPIO17",	0x0e9,	3,	0,	0,	1)
MUX_CFG_24XX("AA8_242X_GPIO58",		0x0ea,	3,	0,	0,	1)
MUX_CFG_24XX("Y20_24XX_GPIO60",		0x12c,	3,	0,	0,	1)
MUX_CFG_24XX("W4__24XX_GPIO74",		0x0f2,	3,	0,	0,	1)
MUX_CFG_24XX("M15_24XX_GPIO92",		0x10a,	3,	0,	0,	1)
MUX_CFG_24XX("P20_24XX_GPIO93",		0x10b,	3,	0,	0,	1)
MUX_CFG_24XX("P18_24XX_GPIO95",		0x10d,	3,	0,	0,	1)
MUX_CFG_24XX("M18_24XX_GPIO96",		0x10e,	3,	0,	0,	1)
MUX_CFG_24XX("L14_24XX_GPIO97",		0x10f,	3,	0,	0,	1)
MUX_CFG_24XX("J15_24XX_GPIO99",		0x113,	3,	1,	1,	1)
MUX_CFG_24XX("V14_24XX_GPIO117",	0x128,	3,	1,	0,	1)
MUX_CFG_24XX("P14_24XX_GPIO125",	0x140,	3,	1,	1,	1)

/* 242x DBG GPIO */
MUX_CFG_24XX("V4_242X_GPIO49",		0xd3,	3,	0,	0,	1)
MUX_CFG_24XX("W2_242X_GPIO50",		0xd4,	3,	0,	0,	1)
MUX_CFG_24XX("U4_242X_GPIO51",		0xd5,	3,	0,	0,	1)
MUX_CFG_24XX("V3_242X_GPIO52",		0xd6,	3,	0,	0,	1)
MUX_CFG_24XX("V2_242X_GPIO53",		0xd7,	3,	0,	0,	1)
MUX_CFG_24XX("V6_242X_GPIO53",		0xcf,	3,	0,	0,	1)
MUX_CFG_24XX("T4_242X_GPIO54",		0xd8,	3,	0,	0,	1)
MUX_CFG_24XX("Y4_242X_GPIO54",		0xd0,	3,	0,	0,	1)
MUX_CFG_24XX("T3_242X_GPIO55",		0xd9,	3,	0,	0,	1)
MUX_CFG_24XX("U2_242X_GPIO56",		0xda,	3,	0,	0,	1)

/* 24xx external DMA requests */
MUX_CFG_24XX("AA10_242X_DMAREQ0",	0x0e5,	2,	0,	0,	1)
MUX_CFG_24XX("AA6_242X_DMAREQ1",	0x0e6,	2,	0,	0,	1)
MUX_CFG_24XX("E4_242X_DMAREQ2",		0x074,	2,	0,	0,	1)
MUX_CFG_24XX("G4_242X_DMAREQ3",		0x073,	2,	0,	0,	1)
MUX_CFG_24XX("D3_242X_DMAREQ4",		0x072,	2,	0,	0,	1)
MUX_CFG_24XX("E3_242X_DMAREQ5",		0x071,	2,	0,	0,	1)

/* TSC IRQ */
MUX_CFG_24XX("P20_24XX_TSC_IRQ",	0x108,	0,	0,	0,	1)

/* UART3 */
MUX_CFG_24XX("K15_24XX_UART3_TX",	0x118,	0,	0,	0,	1)
MUX_CFG_24XX("K14_24XX_UART3_RX",	0x119,	0,	0,	0,	1)

/* MMC/SDIO */
MUX_CFG_24XX("G19_24XX_MMC_CLKO",	0x0f3,	0,	0,	0,	1)
MUX_CFG_24XX("H18_24XX_MMC_CMD",	0x0f4,	0,	0,	0,	1)
MUX_CFG_24XX("F20_24XX_MMC_DAT0",	0x0f5,	0,	0,	0,	1)
MUX_CFG_24XX("H14_24XX_MMC_DAT1",	0x0f6,	0,	0,	0,	1)
MUX_CFG_24XX("E19_24XX_MMC_DAT2",	0x0f7,	0,	0,	0,	1)
MUX_CFG_24XX("D19_24XX_MMC_DAT3",	0x0f8,	0,	0,	0,	1)
MUX_CFG_24XX("F19_24XX_MMC_DAT_DIR0",	0x0f9,	0,	0,	0,	1)
MUX_CFG_24XX("E20_24XX_MMC_DAT_DIR1",	0x0fa,	0,	0,	0,	1)
MUX_CFG_24XX("F18_24XX_MMC_DAT_DIR2",	0x0fb,	0,	0,	0,	1)
MUX_CFG_24XX("E18_24XX_MMC_DAT_DIR3",	0x0fc,	0,	0,	0,	1)
MUX_CFG_24XX("G18_24XX_MMC_CMD_DIR",	0x0fd,	0,	0,	0,	1)
MUX_CFG_24XX("H15_24XX_MMC_CLKI",	0x0fe,	0,	0,	0,	1)

/* Full speed USB */
MUX_CFG_24XX("J20_24XX_USB0_PUEN",	0x11d,	0,	0,	0,	1)
MUX_CFG_24XX("J19_24XX_USB0_VP",	0x11e,	0,	0,	0,	1)
MUX_CFG_24XX("K20_24XX_USB0_VM",	0x11f,	0,	0,	0,	1)
MUX_CFG_24XX("J18_24XX_USB0_RCV",	0x120,	0,	0,	0,	1)
MUX_CFG_24XX("K19_24XX_USB0_TXEN",	0x121,	0,	0,	0,	1)
MUX_CFG_24XX("J14_24XX_USB0_SE0",	0x122,	0,	0,	0,	1)
MUX_CFG_24XX("K18_24XX_USB0_DAT",	0x123,	0,	0,	0,	1)

MUX_CFG_24XX("N14_24XX_USB1_SE0",	0x0ed,	2,	0,	0,	1)
MUX_CFG_24XX("W12_24XX_USB1_SE0",	0x0dd,	3,	0,	0,	1)
MUX_CFG_24XX("P15_24XX_USB1_DAT",	0x0ee,	2,	0,	0,	1)
MUX_CFG_24XX("R13_24XX_USB1_DAT",	0x0e0,	3,	0,	0,	1)
MUX_CFG_24XX("W20_24XX_USB1_TXEN",	0x0ec,	2,	0,	0,	1)
MUX_CFG_24XX("P13_24XX_USB1_TXEN",	0x0df,	3,	0,	0,	1)
MUX_CFG_24XX("V19_24XX_USB1_RCV",	0x0eb,	2,	0,	0,	1)
MUX_CFG_24XX("V12_24XX_USB1_RCV",	0x0de,	3,	0,	0,	1)

MUX_CFG_24XX("AA10_24XX_USB2_SE0",	0x0e5,	2,	0,	0,	1)
MUX_CFG_24XX("Y11_24XX_USB2_DAT",	0x0e8,	2,	0,	0,	1)
MUX_CFG_24XX("AA12_24XX_USB2_TXEN",	0x0e9,	2,	0,	0,	1)
MUX_CFG_24XX("AA6_24XX_USB2_RCV",	0x0e6,	2,	0,	0,	1)
MUX_CFG_24XX("AA4_24XX_USB2_TLLSE0",	0x0e7,	2,	0,	0,	1)

/* Keypad GPIO*/
MUX_CFG_24XX("T19_24XX_KBR0",		0x106,	3,	1,	1,	1)
MUX_CFG_24XX("R19_24XX_KBR1",		0x107,	3,	1,	1,	1)
MUX_CFG_24XX("V18_24XX_KBR2",		0x139,	3,	1,	1,	1)
MUX_CFG_24XX("M21_24XX_KBR3",		0xc9,	3,	1,	1,	1)
MUX_CFG_24XX("E5__24XX_KBR4",		0x138,	3,	1,	1,	1)
MUX_CFG_24XX("M18_24XX_KBR5",		0x10e,	3,	1,	1,	1)
MUX_CFG_24XX("R20_24XX_KBC0",		0x108,	3,	0,	0,	1)
MUX_CFG_24XX("M14_24XX_KBC1",		0x109,	3,	0,	0,	1)
MUX_CFG_24XX("H19_24XX_KBC2",		0x114,	3,	0,	0,	1)
MUX_CFG_24XX("V17_24XX_KBC3",		0x135,	3,	0,	0,	1)
MUX_CFG_24XX("P21_24XX_KBC4",		0xca,	3,	0,	0,	1)
MUX_CFG_24XX("L14_24XX_KBC5",		0x10f,	3,	0,	0,	1)
MUX_CFG_24XX("N19_24XX_KBC6",		0x110,	3,	0,	0,	1)

/* 24xx Menelaus Keypad GPIO */
MUX_CFG_24XX("B3__24XX_KBR5",		0x30,	3,	1,	1,	1)
MUX_CFG_24XX("AA4_24XX_KBC2",		0xe7,	3,	0,	0,	1)
MUX_CFG_24XX("B13_24XX_KBC6",		0x110,	3,	0,	0,	1)

/* 2430 USB */
MUX_CFG_24XX("AD9_2430_USB0_PUEN",	0x133,	4,	0,	0,	1)
MUX_CFG_24XX("Y11_2430_USB0_VP",	0x134,	4,	0,	0,	1)
MUX_CFG_24XX("AD7_2430_USB0_VM",	0x135,	4,	0,	0,	1)
MUX_CFG_24XX("AE7_2430_USB0_RCV",	0x136,	4,	0,	0,	1)
MUX_CFG_24XX("AD4_2430_USB0_TXEN",	0x137,	4,	0,	0,	1)
MUX_CFG_24XX("AF9_2430_USB0_SE0",	0x138,	4,	0,	0,	1)
MUX_CFG_24XX("AE6_2430_USB0_DAT",	0x139,	4,	0,	0,	1)
MUX_CFG_24XX("AD24_2430_USB1_SE0",	0x107,	2,	0,	0,	1)
MUX_CFG_24XX("AB24_2430_USB1_RCV",	0x108,	2,	0,	0,	1)
MUX_CFG_24XX("Y25_2430_USB1_TXEN",	0x109,	2,	0,	0,	1)
MUX_CFG_24XX("AA26_2430_USB1_DAT",	0x10A,	2,	0,	0,	1)

/* 2430 HS-USB */
MUX_CFG_24XX("AD9_2430_USB0HS_DATA3",	0x133,	0,	0,	0,	1)
MUX_CFG_24XX("Y11_2430_USB0HS_DATA4",	0x134,	0,	0,	0,	1)
MUX_CFG_24XX("AD7_2430_USB0HS_DATA5",	0x135,	0,	0,	0,	1)
MUX_CFG_24XX("AE7_2430_USB0HS_DATA6",	0x136,	0,	0,	0,	1)
MUX_CFG_24XX("AD4_2430_USB0HS_DATA2",	0x137,	0,	0,	0,	1)
MUX_CFG_24XX("AF9_2430_USB0HS_DATA0",	0x138,	0,	0,	0,	1)
MUX_CFG_24XX("AE6_2430_USB0HS_DATA1",	0x139,	0,	0,	0,	1)
MUX_CFG_24XX("AE8_2430_USB0HS_CLK",	0x13A,	0,	0,	0,	1)
MUX_CFG_24XX("AD8_2430_USB0HS_DIR",	0x13B,	0,	0,	0,	1)
MUX_CFG_24XX("AE5_2430_USB0HS_STP",	0x13c,	0,	1,	1,	1)
MUX_CFG_24XX("AE9_2430_USB0HS_NXT",	0x13D,	0,	0,	0,	1)
MUX_CFG_24XX("AC7_2430_USB0HS_DATA7",	0x13E,	0,	0,	0,	1)

/* 2430 McBSP */
MUX_CFG_24XX("AC10_2430_MCBSP2_FSX",	0x012E,	1,	0,	0,	1)
MUX_CFG_24XX("AD16_2430_MCBSP2_CLX",	0x012F,	1,	0,	0,	1)
MUX_CFG_24XX("AE13_2430_MCBSP2_DX",	0x0130,	1,	0,	0,	1)
MUX_CFG_24XX("AD13_2430_MCBSP2_DR",	0x0131,	1,	0,	0,	1)
MUX_CFG_24XX("AC10_2430_MCBSP2_FSX_OFF",0x012E,	0,	0,	0,	1)
MUX_CFG_24XX("AD16_2430_MCBSP2_CLX_OFF",0x012F,	0,	0,	0,	1)
MUX_CFG_24XX("AE13_2430_MCBSP2_DX_OFF",	0x0130,	0,	0,	0,	1)
MUX_CFG_24XX("AD13_2430_MCBSP2_DR_OFF",	0x0131,	0,	0,	0,	1)

/* 2430 UART3 & GPIO */
MUX_CFG_24XX("L2_2430_UART_IRTX",	0x0128,	0,	0,	0,	1)
MUX_CFG_24XX("K2_2430_GPIO_103",	0x0127,	3,	0,	0,	1)
MUX_CFG_24XX("L2_2430_GPIO_104",	0x0128,	3,	0,	0,	1)
};

int __init omap2_mux_init(void)
{
	omap_mux_register(omap24xx_pins, ARRAY_SIZE(omap24xx_pins));
	return 0;
}

struct pin_config __initdata_or_module omap34xx_pins[] = {
/*
 *	description			mux	mux	pull	pull	inp 
 *					offset	mode	ena	type	ena
 *					 					
 *				off	off	off	off	off	 wkup	
 * 				ena	/outen	outval	pullen	pulltyp	 en
 */

/* PHY - HSUSB: 12-pin ULPI PHY: Port 1*/
MUX_CFG_34XX(Y9_3430_USB1HS_PHY_STP,	0x5d8,	3,	0,	0,	0,
				1, 	0,	0, 	0, 	0,	0)
MUX_CFG_34XX(Y8_3430_USB1HS_PHY_CLK,	0x5da,	3,	0,	0,	0,
			   	1, 	0,	0, 	0, 	0,	0)
MUX_CFG_34XX(W13_3430_USB1HS_PHY_DATA0,	0x5dc,	3,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(W12_3430_USB1HS_PHY_DATA1,	0x5de,	3,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(W11_3430_USB1HS_PHY_DATA2,	0x5e0,	3,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(Y13_3430_USB1HS_PHY_DATA7,	0x5e2,	3,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(W9_3430_USB1HS_PHY_DATA4,	0x5e4,	3,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(Y12_3430_USB1HS_PHY_DATA5,	0x5e6,	3,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(W8_3430_USB1HS_PHY_DATA6,	0x5e8,	3,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(Y11_3430_USB1HS_PHY_DATA3,	0x5ea,	3,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AA14_3430_USB1HS_PHY_DIR,0x5ec,	3,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AA11_3430_USB1HS_PHY_NXT,0x5ee,	3,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)

/* PHY - HSUSB: 12-pin ULPI PHY: Port 2*/
MUX_CFG_34XX(AE7_3430_USB2HS_PHY_CLK,	0x5f0,  3,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(AF7_3430_USB2HS_PHY_STP,	0x5f2,  3,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(AG7_3430_USB2HS_PHY_DIR,	0x5f4,  3,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AH7_3430_USB2HS_PHY_NXT,	0x5f6,  3,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AG8_3430_USB2HS_PHY_DATA0,	0x5f8,  3,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AH8_3430_USB2HS_PHY_DATA1,	0x5fa,  3,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AB2_3430_USB2HS_PHY_DATA2,	0x1d4,  3,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(V3_3430_USB2HS_PHY_DATA3,	0x1de,  3,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(Y2_3430_USB2HS_PHY_DATA4,	0x1d8,  3,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(Y3_3430_USB2HS_PHY_DATA5,	0x1da,  3,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(Y4_3430_USB2HS_PHY_DATA6,	0x1dc,  3,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AA3_3430_USB2HS_PHY_DATA7,	0x1d6,  3,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)

/* TLL - HSUSB: 12-pin TLL Port 1*/
MUX_CFG_34XX(Y9_3430_USB1HS_TLL_STP,	0x5d8,	6,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(Y8_3430_USB1HS_TLL_CLK,	0x5da, 	6,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(W13_3430_USB1HS_TLL_DATA0,	0x5dc, 	6,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(W12_3430_USB1HS_TLL_DATA1,	0x5de, 	6,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(W11_3430_USB1HS_TLL_DATA2,	0x5e0, 	6,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(Y13_3430_USB1HS_TLL_DATA7,	0x5e2,  6,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(W9_3430_USB1HS_TLL_DATA4,	0x5e4,  6,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(Y12_3430_USB1HS_TLL_DATA5,	0x5e6,  6,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(W8_3430_USB1HS_TLL_DATA6,	0x5e8,  6,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(Y11_3430_USB1HS_TLL_DATA3,	0x5ea,  6,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AA14_3430_USB1HS_TLL_DIR,	0x5ec,  6,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AA11_3430_USB1HS_TLL_NXT,	0x5ee,  6,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)

/* TLL - HSUSB: 12-pin TLL Port 2*/
MUX_CFG_34XX(AA8_3430_USB2HS_TLL_CLK,	0x5f0,  6,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AA10_3430_USB2HS_TLL_STP,	0x5f2,  6,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AA9_3430_USB2HS_TLL_DIR,	0x5f4,  6,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AB11_3430_USB2HS_TLL_NXT,	0x5f6,  6,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AB10_3430_USB2HS_TLL_DATA0,0x5f8,  6,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AB9_3430_USB2HS_TLL_DATA1,	0x5fa,  6,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(W3_3430_USB2HS_TLL_DATA2,	0x1d4,  2,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(T2_3430_USB2HS_TLL_DATA7,	0x1d6,  2,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(T3_3430_USB2HS_TLL_DATA4,	0x1d8,  2,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(R3_3430_USB2HS_TLL_DATA5,	0x1da,  2,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(R4_3430_USB2HS_TLL_DATA6,	0x1dc,  2,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(T4_3430_USB2HS_TLL_DATA3,	0x1de,  2,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)

/* TLL - HSUSB: 12-pin TLL Port 3*/
MUX_CFG_34XX(AB3_3430_USB3HS_TLL_STP,	0x166,  5,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AA3_3430_USB3HS_TLL_DIR,	0x168,  5,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(Y3_3430_USB3HS_TLL_NXT,	0x16a,  5,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AB12_3430_USB3HS_TLL_DATA4,0x16c,  5,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AB13_3430_USB3HS_TLL_DATA5,0x16e,  5,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AA13_3430_USB3HS_TLL_DATA6,0x170,  5,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AA12_3430_USB3HS_TLL_DATA7,0x172,  5,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AA6_3430_USB3HS_TLL_CLK,	0x180,  5,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(Y4_3430_USB3HS_TLL_DATA1,	0x184,  5,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AA5_3430_USB3HS_TLL_DATA0,	0x186,  5,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(Y5_3430_USB3HS_TLL_DATA2,	0x188,  5,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(W5_3430_USB3HS_TLL_DATA3,	0x18a,  5,	1,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)

/* PHY FSUSB: FS Serial PHY 4-pin mode for Port 1*/
MUX_CFG_34XX(AF10_3430_USB1FS_PHY_MM1_RXDP,	0x5d8,  5,      0,      0,      1,
				   	1, 	1, 	0, 	1, 	0,	0)
MUX_CFG_34XX(AG9_3430_USB1FS_PHY_MM1_RXDM,	0x5ee,  5,      0,      0,      1,
				   	1, 	1, 	0, 	1, 	0,	0)
MUX_CFG_34XX(W13_3430_USB1FS_PHY_MM1_RXRCV,	0x5dc,  5,      0,      0,      1,
				   	1, 	1, 	0, 	1, 	0,	0)
MUX_CFG_34XX(W12_3430_USB1FS_PHY_MM1_TXSE0,	0x5de,  5,      0,      0,      1,
				   	1, 	1, 	0, 	1, 	0,	0)
MUX_CFG_34XX(W11_3430_USB1FS_PHY_MM1_TXDAT,	0x5e0,  5,      0,      0,      1,
				   	1,	1, 	0, 	1, 	0,	0)
MUX_CFG_34XX(Y11_3430_USB1FS_PHY_MM1_TXEN_N,	0x5ea,  5,      0,      0,      1,
				   	1, 	1, 	0, 	1, 	0,	0)
/* FSUSB: Serial PHY Port 1 - ISP1301: INT LINE */
MUX_CFG_34XX(AA14_3430_ISP1301_GPIO22,		0x5EC,	4,	0,	0,	1,
				   	1, 	1, 	0, 	1, 	0,	0)

/* PHY FSUSB: FS Serial PHY 4-pin mode for Port 2*/
MUX_CFG_34XX(AF7_3430_USB2FS_PHY_MM2_RXDP,	0x5f2,  5,      0,      0,      1,
				   	1,	1, 	0, 	1, 	0,	0)
MUX_CFG_34XX(AH7_3430_USB2FS_PHY_MM2_RXDM,	0x5f6,  5,      0,      0,      1,
				   	1,	1, 	0, 	1, 	0,	0)
MUX_CFG_34XX(AB10_3430_USB2FS_PHY_MM2_RXRCV,	0x5f8,  5,      0,      0,      1,
				   	1, 	1, 	0, 	1, 	0,	0)
MUX_CFG_34XX(AB9_3430_USB2FS_PHY_MM2_TXSE0,	0x5fa,  5,      0,      0,      1,
				   	1, 	1, 	0, 	1, 	0,	0)
MUX_CFG_34XX(W3_3430_USB2FS_PHY_MM2_TXDAT,	0x1d4,  5,      0,      0,      1,
				   	1, 	1, 	0, 	1, 	0,	0)
MUX_CFG_34XX(T4_3430_USB2FS_PHY_MM2_TXEN_N,	0x1de,  5,      0,      0,      0,
				   	1, 	1, 	0, 	1, 	0,	0)

/* PHY FSUSB: FS Serial PHY 4-pin mode for Port 3*/
MUX_CFG_34XX(AH3_3430_USB3FS_PHY_MM3_RXDP,	0x166,  6,      0,      0,      1,
				   	1, 	1, 	0, 	1, 	0,	0)
MUX_CFG_34XX(AE3_3430_USB3FS_PHY_MM3_RXDM,	0x16a,  6,      0,      0,      1,
				   	1,	1, 	0, 	1, 	0,	0)
MUX_CFG_34XX(AD1_3430_USB3FS_PHY_MM3_RXRCV,	0x186,  6,      0,      0,      1,
				   	1,	1, 	0, 	1, 	0,	0)
MUX_CFG_34XX(AE1_3430_USB3FS_PHY_MM3_TXSE0,	0x184,  6,      0,      0,      1,
				   	1,	1, 	0, 	1, 	0,	0)
MUX_CFG_34XX(AD2_3430_USB3FS_PHY_MM3_TXDAT,	0x188,  6,      0,      0,      1,
				   	1,	1, 	0, 	1, 	0,	0)
MUX_CFG_34XX(AC1_3430_USB3FS_PHY_MM3_TXEN_N,	0x18a,  6,      0,      0,      0,
				   	1,	0, 	0, 	0, 	0,	0)

/* UART3 */
MUX_CFG_34XX(H18_3430_UART3_CTS_RCTX,	0x19a,	0,	1,	0,	1,
				1, 	1, 	0, 	1, 	0,	0)
MUX_CFG_34XX(H19_3430_UART3_RTS_SD,	0x19c,	0,	0,	0,	1,
				1, 	1, 	0, 	1, 	0,	0)
MUX_CFG_34XX(H20_3430_UART3_RX_IRRX,	0x19e,	0,	0,	0,	1,
				1, 	1, 	0, 	1, 	1,	0)
MUX_CFG_34XX(H21_3430_UART3_TX_IRTX,	0x1a0,	0,	0,	0,	0,
				1, 	0, 	0, 	0, 	0,	0)

/* UART2 */
MUX_CFG_34XX(AD25_3430_UART2_RX,	0x17a,	0,	0,	0,	1,
				1,	1,	0,	1,	0,	0)
/* IRRmote */
MUX_CFG_34XX(AB26_3430_UART2_CTS,	0x174,	2,	0,	0,	1,
				1,	1,	0,	1,	1,	0)
/* GPIO */
MUX_CFG_34XX(H19_3430_GPIO164,		0x19c,	4,	0,	0,	1,
				1, 	1, 	0, 	1, 	0,	0)
MUX_CFG_34XX(V21_3430_GPIO158,		0x190,	4,	0,	0,	1,
				1, 	1, 	0, 	1, 	0,	0)
MUX_CFG_34XX(AD25_3430_GPIO147,		0x17a,	4,	0,	0,	1,
				1, 	1, 	0, 	1, 	0,	0)
/* GPT12 or 8 ?: according to spec it is 12, according to pop it is 8*/
MUX_CFG_34XX(AD25_3430_GPT08,		0x17a,	2,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
/* ATMEGA interrput */
MUX_CFG_34XX(J25_3430_GPIO170,		0x1c6,	4,	0,	0,	1,
				1,	1,	0,	0,	0,	1)
/* ethernet LAN9211 */
/* and speaker ON */
MUX_CFG_34XX(AF9_3430_GPIO22,		0x5ec,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(AF5_3430_GPIO142,		0x170,	4,	0,	0,	1,
				1,	1,	0,	1,	0,	0)
MUX_CFG_34XX(AE5_3430_GPIO143,		0x172,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(AE6_3430_GPIO141,		0x16e,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)

/* ethernet LAN9211 EVM*/
MUX_CFG_34XX(R8_3430_CS5,		0x0b8,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(U8_3430_GPIO54,		0x0b4,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(V8_3430_GPIO53,		0x0b2,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(T8_3430_GPIO55,		0x0b6,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(P8_3430_GPIO57,		0x0ba,	4,	0,	0,	1,
				1,	1,	0,	1,	0,	0)
MUX_CFG_34XX(N8_3430_GPIO58,		0x0bc,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)

/* CPLD RESET */
MUX_CFG_34XX(AF6_3430_GPIO140,		0x16c,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
/* hdmi interrupt */
MUX_CFG_34XX(AG12_3430_GPIO15,		0x5de,	4,	0,	0,	1,
				1,	1,	0,	1,	0,	0)

/* FIXME: Wrong! TSC on G6S and G6H */
/*	IRQ */
MUX_CFG_34XX(AF15_3430_GPIO142,		0x170,	4,	0,	0,	1,
				1,	1,	0,	1,	0,	0)
/*	PWR ON */

MUX_CFG_34XX(AF15_3430_GPIO113,		0x136,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)

/* LCD BKL */
MUX_CFG_34XX(AB25_3430_GPT10,		0x176,	2,	0,	0,	0,
				1,	0,	0,	0,	0,	0)

/* hdmi on EVM */
/*PRWDN*/
MUX_CFG_34XX(AA4_3430_GPIO173,		0x1cc,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
/* tvp power down for g6tv */
MUX_CFG_34XX(AB1_3430_GPIO176,		0x1d2,	4,	0,	0,	1,
				1,	1,	0,	1,	0,	0)

/* G6S Board config */
/* TSC IRQ: GPIO24, TSC PWRON: GPIO113 */
MUX_CFG_34XX(AE7_3430_GPIO24,		0x5f0,	4,	0,	0,	1,
				1,	1,	0,	1,	0,	0)
MUX_CFG_34XX(AH19_3430_GPIO113,		0x104,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
		
		
		/* DSS */
MUX_CFG_34XX(D28_3430_DSS_PCLK,		0xd4,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(D26_3430_DSS_HSYNC,	0xd6,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(D27_3430_DSS_VSYNC,	0xd8,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(E27_3430_DSS_ACBIAS,	0xda,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(AG22_3430_DSS_DATA0,	0xdc,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(AH22_3430_DSS_DATA1,	0xde,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(AG23_3430_DSS_DATA2,	0xe0,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(AH23_3430_DSS_DATA3,	0xe2,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(AG24_3430_DSS_DATA4,	0xe4,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(AH24_3430_DSS_DATA5,	0xe6,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(E26_3430_DSS_DATA6,	0xe8,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(F28_3430_DSS_DATA7,	0xea,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(F27_3430_DSS_DATA8,	0xec,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(G26_3430_DSS_DATA9,	0xee,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(AD28_3430_DSS_DATA10,	0xf0,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(AD27_3430_DSS_DATA11,	0xf2,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(AB28_3430_DSS_DATA12,	0xf4,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(AB27_3430_DSS_DATA13,	0xf6,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(AA28_3430_DSS_DATA14,	0xf8,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(AA27_3430_DSS_DATA15,	0xfa,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(G25_3430_DSS_DATA16,	0xfc,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(H27_3430_DSS_DATA17,	0xfe,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(H26_3430_DSS_DATA18,	0x100,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(H25_3430_DSS_DATA19,	0x102,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(E28_3430_DSS_DATA20,	0x104,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(J26_3430_DSS_DATA21,	0x106,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(AC27_3430_DSS_DATA22,	0x108,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(AC28_3430_DSS_DATA23,	0x10a,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)

/* ENABLE_USB */
MUX_CFG_34XX(AF13_3430_GPIO20,		0x5e8,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
/* 5VUSB_ON */
MUX_CFG_34XX(AE22_3430_GPIO186,		0x1e2,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)

/* Headphone Plugged */
MUX_CFG_34XX(AE13_3430_GPIO17,		0x5e2,	4,	0,	0,	1,
				1,	1,	0,	1,	0,	0)
/* Spdif enable */
MUX_CFG_34XX(AG9_3430_GPIO23	,	0x5ee,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)

/* lcd cmd debug */
MUX_CFG_34XX(AH7_3430_GPIO27,		0x5f6,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(AF11_3430_GPIO14,		0x5dc,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(Y8_3430_GPIO151,		0x182,	4,	1,	0,	1,
				1,	1,	0,	1,	0,	0)
MUX_CFG_34XX(AF7_3430_GPIO25,		0x5f2,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(AG7_3430_GPIO26,		0x5f4,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
/* for debug without pwm */
MUX_CFG_34XX(AB25_3430_GPIO145,		0x176,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)

/* New TSC power on: g6h tested */
MUX_CFG_34XX(AA8_3430_GPIO148,		0x17c,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)

/* HDMI/DAC select for g6h */
MUX_CFG_34XX(AE1_3430_GPIO152	,	0x184,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
/* HDMI/DAC select for g6s */
MUX_CFG_34XX(AG8_3430_GPIO28	,	0x5f8,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)


/* CPLD AUX for g6: for adv field control */
MUX_CFG_34XX(AH14_3430_GPIO21	,	0x5ea,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
/*  HSDPA for g6gplus */
MUX_CFG_34XX(AE4_3430_GPIO136	,	0x164,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(AD1_3430_GPIO153	,	0x186,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(AD2_3430_GPIO154	,	0x188,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)

/* reactive I2C2 pins set as GIO at avboot2*/
MUX_CFG_34XX(AE15_3430_I2C2_SDA,	0x1c0,	0,	0,	0,	1,
				1,	0,	1,	0,	0,	0)
MUX_CFG_34XX(AF15_3430_I2C2_SCL,	0x1be,	0,	0,	0,	1,
				1,	0,	1,	0,	0,	0)

/* DC-in detection of battry dock */
MUX_CFG_34XX(AF14_3430_I2C3_SCL,	0x1c2,	0,	0,	0,	1,
				1,	1,	0,	1,	0,	0)
MUX_CFG_34XX(AF14_3430_GPIO184,		0x1c2,	4,	0,	0,	1,
				1,	0,	0,	0,	0,	0)

MUX_CFG_34XX(AG14_3430_I2C3_SDA,	0x1c4,	0,	0,	0,	1,
				1,	1,	0,	1,	0,	0)
MUX_CFG_34XX(AG14_3430_GPIO185,	0x1c4,	4,	0,	0,	1,
				1,	0,	0,	0,	0,	0)

/* G6TV Board Config */

/* Select whether the DVB-T chipset is active ( =1) or the TVP (=0) */
MUX_CFG_34XX(AF10_3430_GPIO12,		0x5d8,	4,	0,	0,	1,
				1,	1,	0,	1,	0,	0)
/* DVB-T pins */
MUX_CFG_34XX(AB3_3430_GPIO171,		0x1c8,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(AB4_3430_GPIO172,		0x1ca,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(AC2_3430_GPIO174,		0x1ce,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)

/* TVP pins */
MUX_CFG_34XX(AC3_3430_GPIO175,		0x1d0,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)

/* cam reset for g6tv */
MUX_CFG_34XX(C23_3430_GPIO98,		0x114,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
// usb commands
MUX_CFG_34XX(R8_3430_GPIO56,		0x0b8,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(B24_3430_GPIO101,		0x11a,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(C24_3430_GPIO102,		0x11c,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(Y21_3430_GPIO156,		0x18c,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)

/* keyb */
MUX_CFG_34XX(AE10_3430_GPIO13,		0x5da,	4,	0,	0,	1,
				1,	1,	0,	1,	0,	0)
MUX_CFG_34XX(AC1_3430_GPIO155,		0x18a,	4,	0,	0,	1,
				1,	1,	0,	1,	1,	0)
MUX_CFG_34XX(AH8_3430_GPIO29,		0x5fa,	4,	0,	0,	1,
				1,	1,	0,	1,	1,	0)

/* HDD_PWR_ON */
MUX_CFG_34XX(AE11_3430_GPIO18,		0x5e4,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
/* SATA_RDY */
MUX_CFG_34XX(H3_3430_GPIO52,		0x0b0,	4,	0,	0,	1,
				1,	1,	0,	1,	0,	0)

/* hub usb gplus */
MUX_CFG_34XX(N28_3430_GPIO120,		0x144,	4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)

MUX_CFG_34XX(AE22_3430_CLKOUT2,		0x1e2,	0,	0,	0,	0,
				1,	0,	0,	0,	0,	0)

MUX_CFG_34XX(AH3_3430_GPIO137,   	0x166,  4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(AF3_3430_GPIO138,   	0x168,  4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)
MUX_CFG_34XX(AE3_3430_GPIO139,   	0x16a,  4,	0,	0,	0,
				1,	0,	0,	0,	0,	0)

MUX_CFG_34XX(AA3_3430_MCPSI2_CLK,	0x1d6,  0,	0,	0,	0,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(Y2_3430_MCSPI2_SIMO,	0x1d8,  0,	0,	0,	0,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(Y3_3430_MCSPI2_SOMI,	0x1da,  0,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(Y4_3430_MCSPI2_CS0,	0x1dc,  0,	0,	0,	0,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(H20_3430_GPIO165,		0x19e,	4,	0,	0,	1,
				1, 	1, 	1, 	1, 	1,	0)

MUX_CFG_34XX(K26_3430_GPIO161,		0x196,	4,	0,	0,	0,
				1, 	0, 	0, 	0, 	0,	0)
MUX_CFG_34XX(W21_3430_GPIO162,		0x198,	4,	0,	0,	0,
				1, 	0, 	0, 	0, 	0,	0)
MUX_CFG_34XX(AA21_3430_GPIO157,		0x18e,	4,	0,	0,	0,
				1, 	0, 	0, 	0, 	0,	0)
MUX_CFG_34XX(U21_3430_GPIO159,		0x192,	4,	0,	0,	0,
				1, 	0, 	0, 	0, 	0,	0)
MUX_CFG_34XX(AA3_3430_GPIO178,		0x1d6,  4,	0,	0,	0,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(Y2_3430_GPIO179,		0x1d8,  4,	0,	0,	0,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(Y3_3430_GPIO180,		0x1da,  4,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(Y4_3430_GPIO181,		0x1dc,  4,	0,	0,	0,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(T21_3430_GPIO160,		0x194,	4,	0,	0,	0,
				1,	1,	0,	1,	0,	0)
MUX_CFG_34XX(H21_3430_GPIO166,		0x1a0,	4,	0,	0,	1,
				1, 	1, 	0, 	1, 	0,	0)

/* MMC3 on G6S v1.6 PCB*/
MUX_CFG_34XX(AB1_3430_MMC3_CLK,		0x1d2,	3,	0,	1,	1,
				1,	1,	0,	1,	0,	0)
MUX_CFG_34XX(AC3_3430_MMC3_CMD,		0x1d0,	3,	0,	1,	1,
				1,	1,	0,	1,	0,	0)
MUX_CFG_34XX(AE4_3430_MMC3_DAT0,	0x164,	3,	0,	1,	1,
				1,	1,	0,	1,	0,	0)
MUX_CFG_34XX(AH3_3430_MMC3_DAT1,   	0x166,  3,	0,	1,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AF3_3430_MMC3_DAT2,   	0x168,  3,	0,	1,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AE3_3430_MMC3_DAT3,   	0x16a,  3,	0,	1,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AF11_3430_MMC3_DAT4,	0x5dc,  2,	0,	1,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AG9_3430_MMC3_DAT5,	0x5ee,  2,	0,	1,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AF9_3430_MMC3_DAT6,	0x5ec,  2,	0,	1,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AH14_3430_MMC3_DAT7,	0x5ea,  2,	0,	1,	1,
			   	1, 	1,	0, 	1, 	0,	0)

/* MMC2 on all units */
MUX_CFG_34XX(AE2_3430_MMC2_CLK,		0x158,	0,	0,	0,	1,
				1,	1,	0,	1,	0,	0)
MUX_CFG_34XX(AG5_3430_MMC2_CMD,		0x15a,	0,	0,	0,	1,
				1,	1,	0,	1,	0,	0)
MUX_CFG_34XX(AH5_3430_MMC2_DAT0,	0x15c,	0,	0,	0,	1,
				1,	1,	0,	1,	0,	0)
MUX_CFG_34XX(AH4_3430_MMC2_DAT1,   	0x15e,  0,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AG4_3430_MMC2_DAT2,   	0x160,  0,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AF4_3430_MMC2_DAT3,   	0x162,  0,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AE2_3430_SAFE,		0x158,	7,	0,	0,	1,
				1,	1,	0,	1,	0,	0)
MUX_CFG_34XX(AG5_3430_SAFE,		0x15a,	7,	0,	0,	1,
				1,	1,	0,	1,	0,	0)
MUX_CFG_34XX(AH5_3430_SAFE,		0x15c,	7,	0,	0,	1,
				1,	1,	0,	1,	0,	0)
MUX_CFG_34XX(AH4_3430_SAFE,   		0x15e,  7,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AG4_3430_SAFE,   		0x160,  7,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)
MUX_CFG_34XX(AF4_3430_SAFE,   		0x162,  7,	0,	0,	1,
			   	1, 	1,	0, 	1, 	0,	0)

/* IRRmote off */
MUX_CFG_34XX(AB26_3430_SAFE,		0x174,	7,	0,	0,	1,
				1,	1,	0,	1,	0,	0)

};

int __init omap3_mux_init(void)
{	
	omap_mux_register(omap34xx_pins, ARRAY_SIZE(omap34xx_pins));
	return 0;
}

#endif
