/*
 * linux/include/asm-arm/arch-omap/board-archosg6.h
 *
 * Hardware definitions for TI ARCHOS G6 board.
 *
 * Copyright (C) 2008 Niklas Schroeter, Archos S.A.,
 *
 * Derived from mach-omap2/board-omap3evm.h
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __ASM_ARCH_OMAP_ARCHOSG6_H
#define __ASM_ARCH_OMAP_ARCHOSG6_H

#include <asm/arch/archosg6-gpio.h>

#define FLASH_BASE_ARCHOS_G6	0x10000000  /* NOR flash (256 Meg aligned) */

/* various memory sizes */
#define FLASH_SIZE_ARCHOS_G6	SZ_128M

#define PRCM_WAKEUP_T2_KEYPAD	0x1
#define PRCM_WAKEUP_TOUCHSCREEN	0x2
#define PRCM_WAKEUP_UART1	0x4

/* WAKEUP*/
#define OMAP3_WAKEUP (PRCM_WAKEUP_T2_KEYPAD | PRCM_WAKEUP_TOUCHSCREEN)

/* max. number of hardware revisions (change if necessary) */
#define MAX_HWREVS	8
/*
 * board config tags, use with omap_get_config()
 * to distribute board configuration to init
 * code and drivers.
 */
struct archosg6_tsp_conf
{
	struct g6_gpio	irq_gpio;
	struct g6_gpio	pwr_gpio;
	u16 x_plate_ohms;
	u16 pressure_max;
};

struct archosg6_tsp_config
{
	int	nrev;	/* number of hardware revisions */
	struct archosg6_tsp_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_TSP	0x4e01

struct archosg6_vbus_config
{
	int	nrev;	/* number of hardware revisions */
	struct g6_gpio rev[MAX_HWREVS];
};
#define ARCHOS_TAG_VBUS0 0x4e02

struct archosg6_disp_conf
{
	struct g6_gpio cpldreset;
	struct g6_gpio disp_select;
	struct g6_gpio lcd_pwon;
	struct g6_gpio lcd_rst;
	struct g6_gpio bkl_pwon;
	struct g6_gpio lcd_pci;
	struct g6_gpio hdmi_dac;
	struct g6_gpio hdmi_it;
	struct g6_gpio cpld_aux;
};

struct archosg6_display_config
{
	int	nrev;	/* number of hardware revisions */
	struct archosg6_disp_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_DISPLAY	0x4e03

struct archosg6_keys_conf
{
	struct g6_gpio power;
	struct g6_gpio vol_up;
	struct g6_gpio vol_down;
};
struct archosg6_keys_config
{
	int	nrev;	/* number of hardware revisions */
	struct archosg6_keys_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_KEYS	0x4e04

struct archosg6_usb_conf
{
	struct g6_gpio	usb_id;
	struct g6_gpio	enable_usb_ehci;
	struct g6_gpio	enable_usb_musb;
};

struct archosg6_usb_config
{
	int	nrev;	/* number of hardware revisions */
	struct archosg6_usb_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_USB	0x4e05

struct archosg6_charge_config
{
	int	nrev;	/* number of hardware revisions */
	struct g6_gpio rev[MAX_HWREVS];
};
#define ARCHOS_TAG_CHARGE 0x4e07

struct archosg6_usbhdd_conf
{
	struct g6_gpio hub_power;
	struct g6_gpio hdd_power;
};

struct archosg6_usbhdd_config
{
	int	nrev;	/* number of hardware revisions */
	struct archosg6_usbhdd_conf rev[MAX_HWREVS];
};
#define ARCHOS_TAG_USBHDD 0x4e08

struct archosg6_audio_conf
{
	struct g6_gpio spdif;
	struct g6_gpio hp_on;
	struct g6_gpio headphone_plugged;
};

struct archosg6_audio_config
{
	int	nrev;	/* number of hardware revisions */
	struct archosg6_audio_conf rev[MAX_HWREVS];
};
#define ARCHOS_TAG_AUDIO 0x4e09

struct archosg6_spi_conf
{
	struct g6_gpio spi_clk;
	struct g6_gpio spi_data;
	struct g6_gpio spi_cs;
};

#define ARCHOS_TAG_WIFI	0x4e0a

struct archosg6_mmc_wifi_dev_conf {
	int nrev; /* number of hardware revisions */
	struct mmc_dev_pins {
		s16 power_pin;
		s32 power_pin_cfg;
		s16 rst_pin;
		s32 rst_pin_cfg;
	} rev[MAX_HWREVS];
};

extern void usbsata_power(int on_off);
extern void usbhdd_power(int on_off);

#endif /*  __ASM_ARCH_OMAP_ARCHOSG6_H */

