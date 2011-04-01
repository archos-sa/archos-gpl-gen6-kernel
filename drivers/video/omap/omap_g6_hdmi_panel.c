/*
 * HDMI panel support for the TI OMAP board
 *
 * Copyright (C) 2008 Archos SA
 * Author: Sebastien LALAURETTE
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

#include <linux/display_out.h>
#include <asm/arch/omapfb.h>
#include <asm/arch/display.h>



#define DBG if(1)
#define DEBUGMSG DBG printk

#define MOD_INC_USE_COUNT
#define MOD_DEC_USE_COUNT



#ifdef CONFIG_MACH_OMAP_3430SDP

#endif
#ifdef CONFIG_MACH_ARCHOS_G6S

#endif

#define DRIVER		"hdmi tv output parameters"

#define HDMI_VGA_BASEX          160
#define HDMI_VGA_BASEY          35
#define HDMI_480P_BASEX         138
#define HDMI_480P_BASEY         42
#define HDMI_576P_BASEX         144
#define HDMI_576P_BASEY         44
#define HDMI_720P50_BASEX 	700
#define HDMI_720P50_BASEY 	25
#define HDMI_720P60_BASEX 	370
#define HDMI_720P60_BASEY 	25
#define HDMI_1080I_BASEX   	0x58
#define HDMI_1080I_BASEY   	0x5

#define FIX_CPLD_SYNC

#if defined (FIX_CPLD_SYNC)
	#define FIX_V_SYNC	1
#else
	#define FIX_V_SYNC	0
#endif


// #define BASEXMAX  HDMI_720P_BASEX
// #define BASEYMAX  HDMI_720P_BASEY
// #define DISP_MEMYMAX DISP_MEMY720P

static int panel_in_use = 0;
// ------------------------------------------
// FB configuration 
static struct fb_var_screeninfo hdmi_vga_4_3_60_var = {
	.xres		= HDMI_VGA_DISP_XRES,
	.yres		= HDMI_VGA_DISP_YRES,
	.xres_virtual	= HDMI_VGA_DISP_XRES,
	.yres_virtual	= HDMI_VGA_DISP_YRES*NB_FRAME_BUFFERS,
	.xoffset	= 0,
	.yoffset	= 0,
	.bits_per_pixel	= 32,
	.grayscale	= 0,
	.red		= {16, 8, 0},
	.green		= { 8, 8, 0},
	.blue		= { 0, 8, 0},
	.transp		= {24 ,8, 0},
	.nonstd		= 0,
	.activate	= FB_ACTIVATE_NOW,
	.height		= -1,
	.width		= -1,
	.accel_flags	= 0,
	.pixclock	= 39682,	/* 25.2Mhz in picoseconds val = 10e6/F(MHz) */
	.left_margin	= 80,		/* pixclocks NOTE 48 + 32 our cpld will by default add 32 pixclock to the h sync len */
	.right_margin	= 16,		/* pixclocks */
	.upper_margin	= (33+FIX_V_SYNC),		/* line clocks */
	.lower_margin	= (10-FIX_V_SYNC),		/* line clocks */
	.hsync_len	= 64,		/* horizontal sync pulse width (96) FIXME : reg is 5 bit width (max = 64) but NOTE 96 - 32 our cpld will by default add 32 pixclock here */
	.vsync_len	= 2,		/* line clocks */
	.sync		= 1,
	.vmode		= FB_VMODE_NONINTERLACED,
	.rotate		= 0,
	.reserved[0]	= 0,
};

static struct fb_var_screeninfo hdmi_480p_16_9_60_var = {
	.xres		= HDMI_480P_DISP_XRES,
	.yres		= HDMI_480P_DISP_YRES,
	.xres_virtual	= HDMI_480P_DISP_XRES,
	.yres_virtual	= HDMI_480P_DISP_YRES*NB_FRAME_BUFFERS,
	.xoffset	= 0,
	.yoffset	= 0,
	.bits_per_pixel	= 32,
	.grayscale	= 0,
	.red		= {16, 8, 0},
	.green		= { 8, 8, 0},
	.blue		= { 0, 8, 0},
	.transp		= {24 ,8, 0},
	.nonstd		= 0,
	.activate	= FB_ACTIVATE_NOW,
	.height		= -1,
	.width		= -1,
	.accel_flags	= 0,
	.pixclock	= 37000,	/* 27.027Mhz in picoseconds val = 10e6/F(MHz) */
	.left_margin	= 92,		/* pixclocks NOTE 60 + 32 our cpld will by default add 32 pixclock to the h sync len */
	.right_margin	= 16,		/* pixclocks */
	.upper_margin	= (30+FIX_V_SYNC),		/* line clocks front porch 42-12 */ 
	.lower_margin	= (9-FIX_V_SYNC),		/* line clocks back porch 3+6 */
	.hsync_len	= 30,		/* pixclocks 62-32 our cpld will by default add 32 pixclock here */
	.vsync_len	= 6,		/* line clocks */
	.sync		= 1,
	.vmode		= FB_VMODE_NONINTERLACED,
	.rotate		= 0,
	.reserved[0]	= 0,
};

static struct fb_var_screeninfo hdmi_576p_16_9_50_var = {
	.xres		= HDMI_576P_DISP_XRES,
	.yres		= HDMI_576P_DISP_YRES,
	.xres_virtual	= HDMI_576P_DISP_XRES,
	.yres_virtual	= HDMI_576P_DISP_YRES*NB_FRAME_BUFFERS,
	.xoffset	= 0,
	.yoffset	= 0,
	.bits_per_pixel	= 32,
	.grayscale	= 0,
	.red		= {16, 8, 0},
	.green		= { 8, 8, 0},
	.blue		= { 0, 8, 0},
	.transp		= {24 ,8, 0},
	.nonstd		= 0,
	.activate	= FB_ACTIVATE_NOW,
	.height		= -1,
	.width		= -1,
	.accel_flags	= 0,
	.pixclock	= 37037,	/* 27.027Mhz in picoseconds val = 10e6/F(MHz) */
	.left_margin	= 100,		/* pixclocks NOTE 68 + 32 our cpld will by default add 32 pixclock to the h sync len */
	.right_margin	= 12,		/* pixclocks */
	.upper_margin	= (39+FIX_V_SYNC),		/* line clocks front porch 44-5 */
	.lower_margin	= (5-FIX_V_SYNC),		/* line clocks */
	.hsync_len	= 32,		/* pixclocks 64-32 our cpld will by default add 32 pixclock here */
	.vsync_len	= 5,		/* line clocks */
	.sync		= 1,
	.vmode		= FB_VMODE_NONINTERLACED,
	.rotate		= 0,
	.reserved[0]	= 0,
};

static struct fb_var_screeninfo hdmi_720p_16_9_50_var = {
	.xres		= HDMI_720P_DISP_XRES,
	.yres		= HDMI_720P_DISP_YRES,
	.xres_virtual	= HDMI_720P_DISP_XRES,
	.yres_virtual	= HDMI_720P_DISP_YRES*NB_FRAME_BUFFERS,
	.xoffset	= 0,
	.yoffset	= 0,
	.bits_per_pixel	= 32,
	.grayscale	= 0,
	.red		= {16, 8, 0},
	.green		= { 8, 8, 0},
	.blue		= { 0, 8, 0},
	.transp		= {24 ,8, 0},
	.nonstd		= 0,
	.activate	= FB_ACTIVATE_NOW,
	.height		= -1,
	.width		= -1,
	.accel_flags	= 0,
	.pixclock	= 13468,	/* 74.25Mhz in picoseconds val = 10e6/F(MHz) */
	.left_margin	= 252,		/* pixclocks NOTE 220 + 32 our cpld will by default add 32 pixclock to the h sync len */
	.right_margin	= 440,		/* pixclocks but FIXME limit is  256 !!! */
	.upper_margin	= (20+FIX_V_SYNC),		/* line clocks front porch 25-5 */
	.lower_margin	= (5-FIX_V_SYNC),		/*pixclocks*/
	.hsync_len	= 8,		/* pixclocks 40-32 our cpld will by default add 32 pixclock here */
	.vsync_len	= 5,		/* line clocks */
	.sync		= 0,
	.vmode		= FB_VMODE_NONINTERLACED,
	.rotate		= 0,
	.reserved[0]	= 0,
};

static struct fb_var_screeninfo hdmi_720p_16_9_60_var = {
	.xres		= HDMI_720P_DISP_XRES,
	.yres		= HDMI_720P_DISP_YRES,
	.xres_virtual	= HDMI_720P_DISP_XRES,
	.yres_virtual	= HDMI_720P_DISP_YRES*NB_FRAME_BUFFERS,
	.xoffset	= 0,
	.yoffset	= 0,
	.bits_per_pixel	= 32,
	.grayscale	= 0,
	.red		= {16, 8, 0},
	.green		= { 8, 8, 0},
	.blue		= { 0, 8, 0},
	.transp		= {24 ,8, 0},
	.nonstd		= 0,
	.activate	= FB_ACTIVATE_NOW,
	.height		= -1,
	.width		= -1,
	.accel_flags	= 0,
	.pixclock	= 13468,	/* 74.25Mhz in picoseconds val = 10e6/F(MHz) */
	.left_margin	= 252,		/* pixclocks NOTE 220 + 32 our cpld will by default add 32 pixclock to the h sync len */
	.right_margin	= 110,		/* pixclocks */
	.upper_margin	= (20+FIX_V_SYNC),		/* line clocks front porch 25-5 */
	.lower_margin	= (5-FIX_V_SYNC),		/* line clocks */
	.hsync_len	= 8,		/* pixclocks 40-32 our cpld will by default add 32 pixclock here */
	.vsync_len	= 5,		/* line clocks */
	.sync		= 0,
	.vmode		= FB_VMODE_NONINTERLACED,
	.rotate		= 0,
	.reserved[0]	= 0,
};


//------------------------------------------------------
extern int display_out_init_hdmiinfo( struct fb_var_screeninfo *i_p_fb_var, u32 i_nb_resolutions );
static u32 hdmi_panel_nb_resolution = 0;
static struct fb_var_screeninfo hdmi_var_conf[NB_MAX_RESOLUTION_PER_PANEL];

// init lcd conf, init display struct for lcd,
// get spi client, get pwm backlight
static int hdmi_panel_init(void)
{

printk ("init hdmi panel \n");
	hdmi_panel_nb_resolution = 0;
	memcpy(&hdmi_var_conf[hdmi_panel_nb_resolution],&hdmi_vga_4_3_60_var,sizeof(struct fb_var_screeninfo));
	hdmi_panel_nb_resolution ++;
	memcpy(&hdmi_var_conf[hdmi_panel_nb_resolution],&hdmi_480p_16_9_60_var,sizeof(struct fb_var_screeninfo));
	hdmi_panel_nb_resolution ++;
	memcpy(&hdmi_var_conf[hdmi_panel_nb_resolution],&hdmi_576p_16_9_50_var,sizeof(struct fb_var_screeninfo));
	hdmi_panel_nb_resolution ++;
	memcpy(&hdmi_var_conf[hdmi_panel_nb_resolution],&hdmi_720p_16_9_50_var,sizeof(struct fb_var_screeninfo));
	hdmi_panel_nb_resolution ++;
	memcpy(&hdmi_var_conf[hdmi_panel_nb_resolution],&hdmi_720p_16_9_60_var,sizeof(struct fb_var_screeninfo));
	hdmi_panel_nb_resolution ++;
	// register hdmi panel conf to display out
	display_out_init_hdmiinfo( (struct fb_var_screeninfo*)hdmi_var_conf,hdmi_panel_nb_resolution );

// request platform gpio
// #if defined(CONFIG_MACH_OMAP_2430SDP) || defined(CONFIG_MACH_OMAP_3430SDP)
// 	omap_request_gpio( panel_power_gpio);  /* LCD panel */
// 	omap_set_gpio_direction( panel_power_gpio, 0); /* output */
// #endif
	return 0;
}


static int hdmi_panel_exit(void)
{
	omap2_disp_get_dss();
	omap2_disp_disable_output_dev( OMAP2_OUTPUT_LCD );
	omap2_disp_put_dss();

// #if defined(CONFIG_MACH_OMAP_2430SDP) || defined(CONFIG_MACH_OMAP_3430SDP)
// 	omap_free_gpio( panel_power_gpio);  /* LCD panel */
// #endif

	return 0;
}
/* ------------------------------------------------------------------------------ */
/* Power and device Management */

// static int __init hdmi_panel_probe(struct platform_device *odev)
// {
// 	printk("hdmi_panel_probe\n");
// 
// 	return hdmi_panel_init();
// 
// }

#ifdef CONFIG_PM
static int hdmi_panel_suspend(struct platform_device *odev, pm_message_t state)
{
	if (!panel_in_use)
		return 0;
	return 0;
}

static int hdmi_panel_resume(struct platform_device *odev)
{
	if (panel_in_use)
		return 0;
	return 0;
}

#endif /* CONFIG_PM */

static struct platform_driver g6_hdmi_panel_driver = {
	.driver = {
		.name   = "g6_hdmi_panel",
	},
	//.devid          = OMAP24xx_LCD_DEVID,
	//.busid          = OMAP_BUS_L3,
	//.clocks         = 0,
// 	.probe          = hdmi_panel_probe,
#ifdef CONFIG_PM
	.suspend        = hdmi_panel_suspend,
	.resume         = hdmi_panel_resume,
#endif
};

static struct platform_device hdmi_panel_device = {
        .name     = "hdmi_panel",
	.id    = 9,
	//.devid    = OMAP24xx_LCD_DEVID,
	// .busid    = OMAP_BUS_L3,
};


//static int __init g6_lcd_drv_init(void)
int __init g6_hdmi_panel_drv_init(void)
{
printk(" init module hdmi panel driver\n");
	/* Register the driver with LDM */
	if ( platform_driver_register( &g6_hdmi_panel_driver )) {
		printk(KERN_ERR DRIVER ": failed to register g6_hdmi_panel driver\n");
		return -ENODEV;
	}
	
	/* Register the device with LDM */
	if (platform_device_register(&hdmi_panel_device)) {
		printk(KERN_ERR DRIVER ": failed to register g6_hdmi_panel device\n");
		platform_driver_unregister(&g6_hdmi_panel_driver);
		return -ENODEV;
	}
	hdmi_panel_init();
	return 0;
}

static void __exit g6_hdmi_panel_drv_cleanup(void)
{
	printk(" exit hdmi panel driver\n");
	hdmi_panel_exit();
	platform_device_unregister(&hdmi_panel_device);
	platform_driver_unregister(&g6_hdmi_panel_driver);
}

MODULE_AUTHOR("ARCHOS S.A.");
MODULE_DESCRIPTION("hdmi panel driver");
MODULE_LICENSE("GPL");

module_init(g6_hdmi_panel_drv_init);
module_exit(g6_hdmi_panel_drv_cleanup);

