/*
 * Samsung LCD panel support for the TI OMAP board
 *
 * Copyright (C) 2008 Archos SA
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>

#ifdef CONFIG_TRACK_RESOURCES
#include <linux/device.h>
#endif
#include <asm/hardware.h>
#include <asm/uaccess.h>
#include <asm/delay.h>
#include <asm/arch/gpio.h>
#include <asm/arch/clock.h>
#include <linux/workqueue.h>
#include <asm/arch/power_companion.h>

#ifdef CONFIG_PM
#include <linux/notifier.h>
#include <linux/pm.h>
#endif

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <asm/arch/gpio.h>
#include <asm/arch/clock.h>
#include <linux/delay.h>
#include <linux/display_out.h>
#include <asm/arch/omapfb.h>
#include <asm/arch/display.h>
#include <linux/hx5091.h>
#include <asm/arch/pwm-gp.h>
#include <asm/mach-types.h>
#include <linux/tpo_lmj048.h>

//#define DEBUGMSG printk
#define DEBUGMSG(...)  do {} while(0)


#undef TEST_ON_EVM	// use 4"3 lcd from gen5
#undef GPIO_FIX

static int lcd_in_use;
static int lcd_backlight_state = 0;
static int lcd_backlight_level = 0;

static void lcd_backlight_off(void);
static void lcd_backlight_on(void);
static void (*lcd_spi_init)(int) = NULL;

static void lcd_enable(void);
static void lcd_disable(void);

#ifdef TEST_ON_EVM
#ifdef CONFIG_MACH_OMAP_3430SDP
//#define LCD_PANEL_ENABLE_GPIO 	28
//#define LCD_PANEL_BACKLIGHT_GPIO 	24
#define LCD_PANEL_ENABLE_GPIO 		137
#define LCD_PANEL_BACKLIGHT_GPIO 	42
#define LCD_PANEL_RESET_GPIO 		25
#endif
#endif

#define DRIVER			"samsung lcd driver"
#define SAMSUNG_LCD_DEVICE	"samsung_lcd"

#define SMG_LCD_XRES	 	800
#define SMG_LCD_YRES 		480
#define SMG_LCD_PIXCLOCK	37037	/* 27 MHz    val = 10e6/F(MHz)*/


extern int display_out_init_lcdinfo( struct lcd_panel *var, struct lcd_fops * fops );
extern struct lcd_io_fops *archosg6_lcd_get_io(void);
extern void spi_gpio_init_state( int state );

#ifdef TEST_ON_EVM
static int panel_power_gpio = LCD_PANEL_ENABLE_GPIO;
static int panel_reset_gpio = LCD_PANEL_RESET_GPIO;
static int backlight_power_gpio = LCD_PANEL_BACKLIGHT_GPIO;
#endif

static struct lcd_io_fops * lcd_io;

// ------------------------------------------
// LCD configuration for FB 
// lcd 4"8 rate to 50Hz
struct lcd_panel samsung_wvga_48panel = {
	.name		= "samsung_wvga_48",
	.config		= OMAP_LCDC_PANEL_TFT	/* active matrix lcd */,
	.bpp		= 24, 			/* = depth 24, padded in 32 bits */
	.data_lines	= LCD_DATA_LINE_24BIT,
	.x_res		= 800,
	.y_res		= 480,
	.hsw		= 2,		/* horizontal sync pulse width () - 1 */
	.hfp		= 212,		/* horizontal front porch () - 1 */
	.hbp		= 22,		/* horizontal back porch () - 1 */
	.vsw		= 2,		/* vertical sync pulse width () - 1 */
	.vfp		= 32,		/* front porch */
	.vbp		= 7,		/* back porch */

	.pixel_clock	= SMG_LCD_PIXCLOCK,	/* 27.027Mhz in picoseconds val = 10e6/F(MHz) */

	.init		= NULL /*sdp2430_panel_init*/,
	.cleanup	= NULL /*sdp2430_panel_cleanup*/,
	.enable		= NULL /*sdp2430_panel_enable*/,
	.disable	= NULL /*sdp2430_panel_disable*/,
	.get_caps	= NULL /*sdp2430_panel_get_caps*/,
};

struct lcd_panel tpo_wvga_48panel = {
	.name		= "tpo_wvga_48",
	.config		= OMAP_LCDC_PANEL_TFT	/* active matrix lcd */,
	.bpp		= 24, 			/* = depth 24, padded in 32 bits */
	.data_lines	= LCD_DATA_LINE_24BIT,
	.x_res		= 800,
	.y_res		= 480,
	.hsw		= 2,		/* horizontal sync pulse width () - 1 */
	.hfp		= 212,		/* horizontal front porch () - 1 */
	.hbp		= 42,		/* horizontal back porch () - 1 */
	.vsw		= 2,		/* vertical sync pulse width () - 1 */
	.vfp		= 18,		/* front porch */
	.vbp		= 11,		/* back porch */

	.pixel_clock	= SMG_LCD_PIXCLOCK,	/* 27.027Mhz in picoseconds val = 10e6/F(MHz) */

	.init		= NULL /*sdp2430_panel_init*/,
	.cleanup	= NULL /*sdp2430_panel_cleanup*/,
	.enable		= NULL /*sdp2430_panel_enable*/,
	.disable	= NULL /*sdp2430_panel_disable*/,
	.get_caps	= NULL /*sdp2430_panel_get_caps*/,
};

struct lcd_panel samsung_wvga_70panel = {
	.name		= "samsung_wvga_70",
	.config		= OMAP_LCDC_PANEL_TFT 	/* active matrix lcd */,
	.bpp		= 24, 			/* = depth 24, padded in 32 bits */
	.data_lines	= LCD_DATA_LINE_24BIT,
	.x_res		= 800,
	.y_res		= 480,
	.hsw		= 41,		/* horizontal sync pulse width () - 1 */
	.hfp		= 2, 		/* horizontal front porch () - 1 */
	.hbp		= 2,		/* horizontal back porch () - 1 */
	.vsw		= 8,		/* vertical sync pulse width () - 1 */
	.vfp		= 0, 		/* front porch */
	.vbp		= 45,		/* back porch */

	.pixel_clock	= SMG_LCD_PIXCLOCK,	/* 27.027Mhz in picoseconds val = 10e6/F(MHz) */

	.init		= NULL /*sdp2430_panel_init*/,
	.cleanup	= NULL /*sdp2430_panel_cleanup*/,
	.enable		= NULL /*sdp2430_panel_enable*/,
	.disable	= NULL /*sdp2430_panel_disable*/,
	.get_caps	= NULL /*sdp2430_panel_get_caps*/,
};

#define MONO_CONF

#ifdef MONO_CONF
// configuration table from Samsung
#define SMS_REG_TO_INIT	56

#if 0
// old table ( before 20/11/2008 )
// ref Samsung 20080521_4.8in_WVGA_Archos.pdf
static hx5091_t SMSWVGA48_init [SMS_REG_TO_INIT] = {
	{0x00, 0x63},	// stdby by software, normal mode
	{0x01, 0x55},	// default
	{0x02, 0x10},	// v posit archos value
	{0x03, 0x80},	// h posit archos value
	{0x04, 0x00},	// default
	{0x05, 0x11},	// default
	{0x06, 0x00},
	{0x07, 0x00},	// default
	{0x08, 0x40},	// default
	{0x09, 0x01},
	{0x0a, 0x30},
	{0x0b, 0x1a},
	{0x0c, 0x29},
	{0x0d, 0x3f},
	{0x0e, 0x20},	// default
	{0x0f, 0x20},	// default
	{0x10, 0xfa},
	{0x11, 0xfa},
	{0x12, 0x07},	// vcom default
	{0x13, 0x20},	// gain default
	{0x14, 0x20},	// gain default
	{0x15, 0x20},	// gain default
	{0x16, 0x80},	// offset default
	{0x17, 0x80},	// offset default
	{0x18, 0x80},	// offset default
	{0x20, 0x00},
	{0x21, 0x0c},
	{0x22, 0xd9},
	{0x23, 0x19},
	{0x24, 0x56},
	{0x25, 0x88},
	{0x26, 0xb9},
	{0x27, 0x4b},
	{0x28, 0xbe},
	{0x29, 0x21},
	{0x2a, 0x77},
	{0x2b, 0xff},
	{0x2c, 0x40},
	{0x2d, 0x95},
	{0x2e, 0xfe},
	{0x50, 0x00},
	{0x51, 0x0c},
	{0x52, 0xd9},
	{0x53, 0x19},
	{0x54, 0x56},
	{0x55, 0x88},
	{0x56, 0xb9},
	{0x57, 0x4b},
	{0x58, 0xbe},
	{0x59, 0x21},
	{0x5a, 0x77},
	{0x5b, 0xff},
	{0x5c, 0x40},
	{0x5d, 0x95},
	{0x5e, 0xfe},
	{0x2f, 0x21},
};

#else
// new table 20/11/2008
// ref Samsung "Archos LMS480KC01 Vertical line Issue_081119.pdf"
static hx5091_t SMSWVGA48_init [SMS_REG_TO_INIT] = {
	{0x00, 0x63},	// stdby by software, normal mode
	{0x01, 0x55},	// default
	{0x02, 0x10},	// v posit archos value
	{0x03, 0x80},	// h posit archos value
	{0x04, 0x00},	// default
	{0x05, 0x11},	// default
	{0x06, 0x00},
	{0x07, 0x00},	// default
	{0x08, 0x40},	// default
	{0x09, 0x01},
	{0x0a, 0x20},
	{0x0b, 0x29},
	{0x0c, 0x10},
	{0x0d, 0x30},
	{0x0e, 0x20},	// default
	{0x0f, 0x20},	// default
	{0x10, 0xca},
	{0x11, 0xca},
	{0x12, 0x0b},	// vcom default
	{0x13, 0x20},	// gain default
	{0x14, 0x20},	// gain default
	{0x15, 0x20},	// gain default
	{0x16, 0x80},	// offset default
	{0x17, 0x80},	// offset default
	{0x18, 0x80},	// offset default
	{0x20, 0x00},
	{0x21, 0x03},
	{0x22, 0xef},
	{0x23, 0x2d},
	{0x24, 0x6b},
	{0x25, 0xa1},
	{0x26, 0xb9},
	{0x27, 0x6d},
	{0x28, 0xeb},
	{0x29, 0x51},
	{0x2a, 0x97},
	{0x2b, 0xff},
	{0x2c, 0x40},
	{0x2d, 0x95},
	{0x2e, 0xfe},
	{0x50, 0x00},
	{0x51, 0x03},
	{0x52, 0xef},
	{0x53, 0x2d},
	{0x54, 0x6b},
	{0x55, 0xa1},
	{0x56, 0xd8},
	{0x57, 0x6d},
	{0x58, 0xeb},
	{0x59, 0x51},
	{0x5a, 0x97},
	{0x5b, 0xff},
	{0x5c, 0x40},
	{0x5d, 0x95},
	{0x5e, 0xfe},
	{0x2f, 0x21},
};
#endif

#else
// Multi config
// configuration tables from Samsung
// not defined yet
#endif

#ifdef MONO_CONF
// configuration table from TPO
// report 26 /05 /2008
#define TPO_REG_TO_INIT	27
static hx5091_t TPOWVGA48_init [TPO_REG_TO_INIT] = {
	{0x03, 0x86},	// stdby by software, normal mode
	{0x04, 0x0f},	// default 0x0f 
	{0x05, 0x33},	// default
	{0x09, 0xff},	// default 
	{0x3a, 0x99},	// vcom
	{0x3c, 0xe0},	// default
	{0x3d, 0xf4},	// default
	{0x3e, 0x21},
	{0x3f, 0x87},	// default
	{0x15, 0x55},	// default
	{0x16, 0xaf},
	{0x17, 0xfc},
	{0x18, 0x77},
	{0x19, 0xd3},
	{0x1a, 0xdf},
	{0x1b, 0xf0},	// default
	{0x1c, 0x10},	// default
	{0x1d, 0x45},
	{0x1e, 0x7b},
	{0x1f, 0xea},	//  default
	{0x20, 0x8d},	// gain default
	{0x21, 0xf0},	// gain default
	{0x22, 0x26},	// gain default
	{0x23, 0x53},	// offset default
	{0x24, 0x7c},	// offset default
	{0x25, 0xbb},	// offset default
	{0x26, 0xff},	// offset default

};
#if 0
// gamma curve adjustment 
// not activated
#define TPOgamma_REG_TO_INIT	31
static hx5091_t TPOWVGA48gamma_init [TPOgamma_REG_TO_INIT] = {
	{0x20, 0x00},
	{0x21, 0x0c},
	{0x22, 0xd9},
	{0x23, 0x19},
	{0x24, 0x56},
	{0x25, 0x88},
	{0x26, 0xb9},
	{0x27, 0x4b},
	{0x28, 0xbe},
	{0x29, 0x21},
	{0x2a, 0x77},
	{0x2b, 0xff},
	{0x2c, 0x40},
	{0x2d, 0x95},
	{0x2e, 0xfe},
	{0x50, 0x00},
	{0x51, 0x0c},
	{0x52, 0xd9},
	{0x53, 0x19},
	{0x54, 0x56},
	{0x55, 0x88},
	{0x56, 0xb9},
	{0x57, 0x4b},
	{0x58, 0xbe},
	{0x59, 0x21},
	{0x5a, 0x77},
	{0x5b, 0xff},
	{0x5c, 0x40},
	{0x5d, 0x95},
	{0x5e, 0xfe},
	{0x2f, 0x21},
};
#endif
#else
// Multi config
// configuration tables from TPO
// not defined yet
#endif

// ------------------------------------------
// GPIO interface

static void lcd_panel_enable(void * unused)
{
	DEBUGMSG("lcd_panel_enable \n" );
	if ( lcd_io->lcd_power )
		lcd_io->lcd_power( 1 );
}

static void lcd_panel_disable(void * unused)
{
	DEBUGMSG("lcd_panel_disable\n");
	if ( lcd_io->lcd_power )
		lcd_io->lcd_power( 0 );
}

static void lcd_backlight_on(void)
{
	if (lcd_backlight_state)
		return;

	DEBUGMSG("lcd_backlight_on \n");
	
	if ( lcd_io->backlight_power )
		lcd_io->backlight_power( 1 );
	pwm_gpt_start( LCD_BKL );
	lcd_backlight_state = 1;
}

static void lcd_backlight_off(void)
{
	if (!lcd_backlight_state)
		return;

	DEBUGMSG("lcd_backlight_off\n");
	
	pwm_gpt_stop( LCD_BKL );
	if ( lcd_io->backlight_power )
		lcd_io->backlight_power( 0 );
	lcd_backlight_state = 0;
}

static void lcd_reset_enable(void * unused)
{
	DEBUGMSG("lcd_reset_enable \n" );
	if ( lcd_io->lcd_reset )
		lcd_io->lcd_reset( 1 );
}

static void lcd_reset_disable(void * unused)
{
	DEBUGMSG("lcd_reset_disable\n");
	if ( lcd_io->lcd_reset )
		lcd_io->lcd_reset( 0 );
}

static void lcd_pci_enable(void * unused)
{
	DEBUGMSG("lcd_pci_enable \n" );
	if ( lcd_io->lcd_pci )
		lcd_io->lcd_pci( 1 );
}

static void lcd_pci_disable(void * unused)
{
	DEBUGMSG("lcd_pci_disable\n");
	if ( lcd_io->lcd_pci )
		lcd_io->lcd_pci( 0 );
}

static void lcd_spi_init_state( int state )
{
	if ( lcd_spi_init != NULL )
		lcd_spi_init( state );
}

#undef BLKMAX	// just on/off blklevel

static int set_backlight( int level )
{
#ifdef BLKMAX
// just for debug, 
	if (level)
		omap_set_gpio_dataout( 145, 0 );
	else
		omap_set_gpio_dataout( 145, 1 );

#else
	// frequency and level limits to define	
	if ( level > 255 )
		level = 255; 
	if (level < 0)
		level = 0;
	
	if (lcd_backlight_state == 0) {
		lcd_backlight_on();
		lcd_backlight_level = -1;
	}

	if (lcd_backlight_level == level)
		return 0;

	pwm_gpt_set_speed( LCD_BKL, 10000, level );

	if (level == 0)
		lcd_backlight_off();

	lcd_backlight_level = level;
#endif
	return 0;
}

// Apply correction to backlight level
// depending on lcd type 
// max: pwm value for maximum backlight value
// ref: max value from application 
// step: pwm steps for each 100 level input steps

static int backlight_correction( int level, int max, int ref, int step )
{
	int tmp;
	int corr_level =  max - ( ( ( ref - level) /100 ) * step );

DEBUGMSG("level: %d, corr_level:%d \n", level, corr_level);
		if ( ( level % 100 ) != 0 ) {
			tmp = level - ((level/100) * 100 );
			corr_level -= ( step * ( 100 - tmp )) /100 ;
			if ( corr_level < 0 ) 
				corr_level = 0;
DEBUGMSG(" correct: %d, then %d\n", tmp, corr_level);
		}
	return corr_level;

}

static int set_backlight_70( int level )
{
// lcd 7" for gen6 need inverted backlight value
	int smg_level = 0;

	if ( level > 0 ) {
		smg_level = backlight_correction( level, 255, 300, 90);
	}

	smg_level = ~smg_level & 0xff;
	if ( smg_level == 0 )
		smg_level = 1;	// we don't want to cut but max backlight

	return ( set_backlight( smg_level ) );
}

static int set_backlight_smg_48( int level )
{
	int smg_level = 0;

	if ( level > 0 ) {
		smg_level = backlight_correction( level, 200, 300, 60);
	}

	return ( set_backlight( smg_level & 0xff) );
}

static int set_backlight_tpo_48( int level )
{
	int tpo_level = 0;

	if ( level > 0 ) {
		if ( level > 120 )
			tpo_level = backlight_correction( level, 140, 300, 40);
		else
			tpo_level = backlight_correction( level, 140, 300, 35);
	}

	return ( set_backlight( tpo_level & 0xff) );
}

static void lcd_enable(void)
{
	lcd_panel_enable(NULL);
	// delay here to define
	msleep( 50);
	lcd_reset_enable(NULL);
	// delay here to define
	msleep( 100);
	lcd_pci_enable(NULL);
	lcd_spi_init_state(1);
	msleep( 100);
}

static void lcd_disable(void)
{
	lcd_spi_init_state(0);
	lcd_reset_disable(NULL);
	lcd_panel_disable(NULL);
	lcd_pci_disable(NULL);
}

static int power_lcd_panel( int enable )
{
	switch (enable) {
	case 0:
		lcd_disable();
		lcd_in_use = 0;
		break;
	
	default:
		if ( lcd_in_use )
			break;
		lcd_enable();
		lcd_in_use = 1;
		break;
	}
	return 0;
}

static int power_lcd_backlight( int enable )
{
	if ( enable ) {
		lcd_backlight_on();
	} else {
		lcd_backlight_off();
	}
	/*set_backlight( lcd_backlight_level );*/
	return 0;
}

static int power_lcd70_backlight( int enable )
{
	if ( enable ) {
		lcd_backlight_on();
	} else {
		lcd_backlight_off();
	}
	/*set_backlight( lcd_backlight_level );*/
	return 0;
}


// ------------------------------------------
// SPI interface for hx5091
extern int hx5091_spi_reg_write( int reg, int val );
extern int hx5091_spi_reg_read( int reg, int *val );
#ifdef GPIO_FIX
extern int hx5091_gpio_spi_reg_write( int reg, int val );
#endif
static int lcd_spi_reg_write ( int reg, int val ) {

	int ret = 0;
// to fix
#ifdef GPIO_FIX
	if ( machine_is_archos_g6s() )
		ret = hx5091_spi_reg_write( reg, val );
	else if ( machine_is_archos_g6h() ||  machine_is_archos_g6plus() )
		ret = hx5091_gpio_spi_reg_write( reg, val );
#else
	ret = hx5091_spi_reg_write( reg, val );
#endif
	return ret;
}	

static int set_contrast( int level )
{
	int ret;

	if ( level < 15 )
		level = 15;
	if ( level > 0x3f )
		level = 0x3f;
#ifdef TEST_ON_EVM
	ret = hx5091_spi_reg_write( HX5078_RGAIN, level);
	ret = hx5091_spi_reg_write( HX5078_GGAIN, level);
	ret = hx5091_spi_reg_write( HX5078_BGAIN, level);
#else
	ret = lcd_spi_reg_write( HX5091_RGAIN, level);
	ret = lcd_spi_reg_write( HX5091_GGAIN, level);
	ret = lcd_spi_reg_write( HX5091_BGAIN, level);

#endif
	return ret;
}

static int set_brightness( int level )
{
	int ret;

#ifdef TEST_ON_EVM
	if ( level < 5 )
		level = 5;
	if ( level > 0x30 )
		level = 0x30;

	ret = hx5091_spi_reg_write( HX5078_ROFFSET, level);
	ret = hx5091_spi_reg_write( HX5078_GOFFSET, level);
	ret = hx5091_spi_reg_write( HX5078_BOFFSET, level);
#else
	level *=4;
	if ( level < 50 )
		level = 50;
	if ( level > 0xff )
		level = 0xff;

	ret = lcd_spi_reg_write( HX5091_ROFFSET, level);
	ret = lcd_spi_reg_write( HX5091_GOFFSET, level);
	ret = lcd_spi_reg_write( HX5091_BOFFSET, level);

#endif
	return ret;
}

static int set_gamma( int level)
{
	// only default conf for now
	return 0;
}

static int set_conf( int level)
{
	int ret = 0;
	int i;

	if ( !lcd_in_use )
		return -1;

#ifdef MONO_CONF
	for ( i = 0; i < SMS_REG_TO_INIT; i++ ) {
		ret = lcd_spi_reg_write( (unsigned int)SMSWVGA48_init[i].reg, (unsigned int)SMSWVGA48_init[i].val);
		if (ret < 0)
			break;
		//DEBUGMSG( "reg: %x value: %x \n", SMSWVGA48_init[i].reg, SMSWVGA48_init[i].val );
	}
#else

	if ( level < 0 )
		level = 0;

	if ( level >= SMSG_NB_CONF)
		level = SMSG_NB_CONF;

	for ( i = 0; i < SMS_REG_TO_INIT; i++ ) {
		ret = lcd_spi_reg_write( (unsigned int)SMSWVGA48_init[level][i].reg, (unsigned int)SMSWVGA48_init[level][i].val);
		if (ret < 0)
			break;
		DEBUGMSG( "reg: %x value: %x \n", SMSWVGA48_init[level][i].reg, SMSWVGA48_init[level][i].val );
	}
#endif
	return ret;
}

static int set_sync( int level)
{
	return -ENODEV;

}

static int set_vcom ( int level)
{
	int ret;
	level /=2;
	if ( level < 0 )
		level = 0;
	if ( level > 0x1f )
		level = 0x1f;

	ret = lcd_spi_reg_write( HX5091_VCOM, level);
	
	return ret;
}

static int set_reg ( int reg, int val )
{
	int ret;

	if ( !lcd_in_use )
		return -1;

	ret = lcd_spi_reg_write( (unsigned int) reg, (unsigned int) val );
DEBUGMSG(" set_reg value: %x \n", val);
	return ret;
}

static int get_reg ( int reg , int *val )
{
	int ret;

	if ( !lcd_in_use )
		return -1;

	ret = hx5091_spi_reg_read( reg, val );
DEBUGMSG("value: %x \n", *val);
	return ret;
}

// ------------------------------------------
// SPI interface for tpo
extern int tpo_spi_reg_write( int reg, int val );
extern int tpo_spi_reg_read( int reg, int *val );

static int set_tpo_contrast( int level )
{
	int ret;

	if ( level < 10 )
		level = 10;
	if ( level > 0x3f )
		level = 0x3f;
	ret = tpo_spi_reg_write( TPO_RGAIN, level);
	ret = tpo_spi_reg_write( TPO_GGAIN, level);
	ret = tpo_spi_reg_write( TPO_BGAIN, level);
	return ret;
}

static int set_tpo_brightness( int level )
{
	int ret;

	if ( level < 5 )
		level = 5;
	if ( level > 0x3f )
		level = 0x3f;

	ret = tpo_spi_reg_write( TPO_ROFFSET, level);
	ret = tpo_spi_reg_write( TPO_GOFFSET, level);
	ret = tpo_spi_reg_write( TPO_BOFFSET, level);

	return ret;
}

static int set_tpo_gamma( int level)
{
	return -ENODEV;

}

static int set_tpo_conf( int level)
{
	int ret = 0;
	int i;

	if ( !lcd_in_use )
		return -1;
#ifdef MONO_CONF
	for ( i = 0; i < TPO_REG_TO_INIT; i++ ) {
		ret = tpo_spi_reg_write( (unsigned int)TPOWVGA48_init[i].reg, (unsigned int)TPOWVGA48_init[i].val);
		if (ret < 0)
			break;
//		DEBUGMSG( "reg: %x value: %x \n", TPOWVGA48_init[i].reg, TPOWVGA48_init[i].val );
	}
#else

	if ( level < 0 )
		level = 0;

	if ( level >= TPO_NB_CONF)
		level = TPO_NB_CONF;

	for ( i = 0; i < TPO_REG_TO_INIT; i++ ) {
		ret = tpo_spi_reg_write( (unsigned int)TPOWVGA48_init[level][i].reg, (unsigned int)TPOWVGA48_init[level][i].val);
		if (ret < 0)
			break;
//		DEBUGMSG( "reg: %x value: %x \n", TPOWVGA48_init[level][i].reg, TPOWVGA48_init[level][i].val );
	}
#endif
	return ret;
}

static int set_tpo_vcom ( int level)
{
	int ret;
	if ( level < 0 )
		level = 0;
	if ( level > 0x3f )
		level = 0x3f;

	level |= 0x80;	// vcom capability default

	ret = tpo_spi_reg_write( TPO_VCOM, level);
	
	return ret;
}

static int set_tpo_reg ( int reg, int val )
{
	int ret;

	if ( !lcd_in_use )
		return -1;
	
	ret = tpo_spi_reg_write( (unsigned int) reg, (unsigned int) val );
DEBUGMSG("value: %x \n", val);
	return ret;
}

static int get_tpo_reg ( int reg , int *val )
{
	int ret;

	if ( !lcd_in_use )
		return -1;

	ret = tpo_spi_reg_read( reg, val );
DEBUGMSG("value: %x \n", *val);
	return ret;
}

//------------------------------------------------------
static struct lcd_fops samsung_48_fops = {

	.enable_display = power_lcd_panel,
#ifdef TEST_ON_EVM
	.enable_backlight = 0,
#else
	.enable_backlight = power_lcd_backlight,
#endif
	.backlight_level = set_backlight_smg_48,
	.set_contrast = set_contrast,
	.set_brightness = set_brightness,
	.set_gamma = set_gamma,
	.set_reg = set_reg,
	.get_reg = get_reg,
	.set_conf = set_conf,
	.set_sync = set_sync,
	.set_vcom = set_vcom,
};

static struct lcd_fops tpo_48_fops = {

	.enable_display = power_lcd_panel,
#ifdef TEST_ON_EVM
	.enable_backlight = 0,
#else
	.enable_backlight = power_lcd_backlight,
#endif
	.backlight_level = set_backlight_tpo_48,
	.set_contrast = set_tpo_contrast,
	.set_brightness = set_tpo_brightness,
	.set_gamma = set_tpo_gamma,
	.set_reg = set_tpo_reg,
	.get_reg = get_tpo_reg,
	.set_conf = set_tpo_conf,
	.set_sync = set_sync,
	.set_vcom = set_tpo_vcom,
};

static struct lcd_fops samsung_70_fops = {

	.enable_display = power_lcd_panel,
#ifdef TEST_ON_EVM
	.enable_backlight = 0,
#else
	.enable_backlight = power_lcd70_backlight,
#endif
	.backlight_level = set_backlight_70,
	.set_contrast = 0,
	.set_brightness = 0,
	.set_gamma = 0,
	.set_reg = 0,
	.get_reg = 0,
	.set_conf = 0,
	.set_sync = 0,
	.set_vcom = 0,
};

// init lcd conf, init display struct for lcd,
// get spi client, get pwm backlight
static int lcd_init(void)
{
	DEBUGMSG ("init lcd samsung \n");

	// register lcd conf to display
	if ( machine_is_archos_g6l() )
		display_out_init_lcdinfo( &samsung_wvga_70panel, &samsung_70_fops );
	else {
		if( ( hardware_rev == 4 ) || ( hardware_rev == 6 ) ){
			printk("select TPO LCD for specific HW ID\n");
			display_out_init_lcdinfo( &tpo_wvga_48panel, &tpo_48_fops );
		} else	
			display_out_init_lcdinfo( &samsung_wvga_48panel, &samsung_48_fops );
		
		if ( spi_gpio_init_state != NULL)
			lcd_spi_init = spi_gpio_init_state;
	}
	// request platform gpio
	lcd_io = archosg6_lcd_get_io();

	DEBUGMSG("LCD panel %dx%d\n", SMG_LCD_XRES, SMG_LCD_YRES);
	return 0;
}


static int lcd_exit(void)
{
	omap2_disp_get_dss();
	omap2_disp_disable_output_dev( OMAP2_OUTPUT_LCD );
	omap2_disp_put_dss();
	pwm_gpt_stop( LCD_BKL );

	lcd_io = NULL;

	return 0;
}
/* ------------------------------------------------------------------------------ */
/* Power and device Management */

static int __init lcd_probe(struct platform_device *odev)
{
	DEBUGMSG("lcd_probe\n");
	return lcd_init();

}

static struct platform_driver g6_lcd_driver = {
	.driver = {
		.name   = "g6_lcd",
	},

	.probe          = lcd_probe,
};

static struct platform_device lcd_device = {
        .name     = SAMSUNG_LCD_DEVICE,
	.id    = 9,
};


static int __init g6_lcd_drv_init(void)
{
	DEBUGMSG(" init module samsung lcd driver\n");
	/* Register the driver with LDM */
	if ( platform_driver_register( &g6_lcd_driver )) {
		printk(KERN_ERR DRIVER ": failed to register g6_lcd driver\n");
		return -ENODEV;
	}
	
	/* Register the device with LDM */
	if (platform_device_register(&lcd_device)) {
		printk(KERN_ERR DRIVER ": failed to register lcd device\n");
		platform_driver_unregister(&g6_lcd_driver);
		return -ENODEV;
	}
	lcd_init();
	return 0;
}

static void __exit g6_lcd_drv_cleanup(void)
{
	DEBUGMSG(" exit samsung lcd driver\n");
	lcd_exit();
	platform_device_unregister(&lcd_device);
	platform_driver_unregister(&g6_lcd_driver);
}

MODULE_AUTHOR("ARCHOS");
MODULE_DESCRIPTION("Samsung LCD driver");
MODULE_LICENSE("GPL");

module_init(g6_lcd_drv_init);
module_exit(g6_lcd_drv_cleanup);

