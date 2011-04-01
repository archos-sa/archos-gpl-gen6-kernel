/*
 * OMAP internal TV out panel support 
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
#include <linux/i2c-dev.h>

#define FIX_CPLD_SYNC

#if defined (FIX_CPLD_SYNC)
	#define FIX_V_SYNC	1
#else
	#define FIX_V_SYNC	0
#endif

#define DBG if(1)
#define DEBUGMSG DBG printk

#define MOD_INC_USE_COUNT
#define MOD_DEC_USE_COUNT

#define DRIVER		"external tv output parameters"

#define TIME_BASE_PAL 20
#define TIME_BASE_NTSC 16
#define TIME_BASE_HDPAL	-1

//#define TVEXT_DEBUG
#if defined TVEXT_DEBUG
extern int tv_irq_cnt;
	#define _inc_irq_cnt() tv_irq_cnt++;
#else
	#define _inc_irq_cnt() while(0);
#endif

extern int archosg6_display_get_extvideoenc_i2c_nbr(void);
extern int display_out_init_ext_videoenc_info( struct fb_var_screeninfo *i_p_fb_var, u32 i_nb_resolutions,
struct ext_videoenc_fops *fops );

#define ADV734X_I2C_NB	archosg6_display_get_extvideoenc_i2c_nbr()

static struct i2c_client *ext_videoenc_handle = NULL;
static struct ext_videoenc_fops fops;

static int time_base = TIME_BASE_PAL;
static struct display_io_fops *_display_io = NULL;

static void _set_gpio_field(int field) 
{
	if (_display_io == NULL)
		_display_io=archosg6_display_get_io();
	
	if (_display_io == NULL)
		return;

 	if (field == EVEN_FIELD) 
		 _display_io->display_set_aux(1);
 	else
		 _display_io->display_set_aux(0);
 
 	return;
}

static void videoenc_isr_handler( void *arg, struct pt_regs *regs )
{
	static int toggle = 0;

	if ( toggle ) {
		if ( time_base > 0 )
			omap2_disp_swap_dma_ba( EVEN_FIELD, time_base );
		_set_gpio_field( EVEN_FIELD );
	} else {
		if ( time_base > 0 )
			omap2_disp_swap_dma_ba( ODD_FIELD, time_base );
		_set_gpio_field( ODD_FIELD );
	}
	toggle ^= 1;

	return;
}

static int _check_i2_client(void) 
{
	if (ext_videoenc_handle == NULL)
		ext_videoenc_handle = i2c_get_client(I2C_DRIVERID_ADV734X,ADV734X_I2C_NB,NULL);
	
	return (ext_videoenc_handle==NULL?-1:0);
} 

static void _videoenc_reset(void)
{
	printk(KERN_INFO" videoenc reset\n");
	if ( _check_i2_client() < 0) {
		printk(KERN_ERR "error: cannot get i2c client for external video enc\n");
		return -1;
	}

	/* set  GIO COMPLENTE and  GIO RGB to 0 */
	ext_videoenc_handle->driver->command(ext_videoenc_handle,0,2);

	ext_videoenc_handle = 0;
}

/* analog out resolution */
static int _select_format(int format) 
{
	static int prev_format=-1;
	int arg = 0;

	printk(KERN_INFO"select format [%d]\n",format);
	if ( _check_i2_client() < 0) {
		printk(KERN_ERR "error: cannot get i2c client for external video enc\n");
		return -1;
	}

	if (format == RESOLUTION_VIDEOEXT_PAL) {
		/* setup pal */
		time_base = TIME_BASE_PAL;
	} else if (format == RESOLUTION_VIDEOEXT_NTSC) {
		/* setup pal */
		time_base = TIME_BASE_NTSC;
	} else if (format == RESOLUTION_VIDEOEXT_HDPAL) {
		/* setup pal */
		time_base = TIME_BASE_HDPAL;
	} else {
		printk(KERN_ERR "Unknown external video format [%d] !!\n",format);
		return 1;
	}

	ext_videoenc_handle->driver->command(ext_videoenc_handle,format,arg);
#ifdef DBG
	/* dump adv registers */
	ext_videoenc_handle->driver->command(ext_videoenc_handle,format,3);
#endif
	prev_format = format;

	return 0;

}

/* analog out connectors type */
static int _select_analog_out(int format) 
{
	static int prev_format=-1;
	int arg = 1;
// Even thought the analog out doesn't change
// After changing analog resolution,we have to reset the connector type, because adv734 has been reset
// 	if ( prev_format == format ) {
// 		printk(KERN_INFO "analog out [%d] already selected\n",format);
// 		return 0;
// 	}

	printk(KERN_INFO"select analog out [%d]\n",format);
	if ( _check_i2_client() < 0) {
		printk(KERN_ERR "error: cannot get i2c client for external video enc\n");
		return -1;
	}

	ext_videoenc_handle->driver->command(ext_videoenc_handle,format,arg);
#ifdef DBG
	ext_videoenc_handle->driver->command(ext_videoenc_handle,format,3);
#endif

	prev_format = format;

	return 0;

	return 0;
}

static int panel_in_use = 0;
// ------------------------------------------
// FB configuration 


static struct fb_var_screeninfo ext_tvout_pal_var = {
	/* PAL frame size is 720 * 576,
	* but due to overscan, about 640 x 520 is visible : be aware of this when drawing in the framebuffer
	*/
	.xres = PAL_DISP_XRES,
	.yres = PAL_DISP_YRES,
	.xres_virtual	= PAL_DISP_XRES,
	.yres_virtual	= PAL_DISP_YRES * NB_FRAME_BUFFERS,
	.xoffset	= 0,
	.yoffset	= 0,
	.bits_per_pixel	= 32,
	.grayscale	= 0,
	.red		= {16, 8, 0},
	.green		= { 8, 8, 0},
	.blue		= { 0, 8, 0},
	.transp		= { 24, 8, 0},
	.nonstd		= 0,
	.activate	= FB_ACTIVATE_NOW,
	.height		= -1,
	.width		= -1,
	.accel_flags	= 0,
	.pixclock	= 74074,		/* picoseconds =27/2 MHz */
	.left_margin	= 100,		/* pixclocks */
	.right_margin	= 12,		/* pixclocks */
	.upper_margin	= 19,		/* line clocks: modified every field */
	.lower_margin	= (2-FIX_V_SYNC),		/* line clocks */
	.hsync_len	= 32,		/* pixclocks */
	.vsync_len	= (3+FIX_V_SYNC),	/* line clocks */
						/* do the cpld fix here because upper_margin is modified every field to simulate interlace stream */
	.sync		= 1,
	.vmode		= FB_VMODE_IBUS,
	.rotate		= 0,
	.reserved[0]	= 0,
};

static struct fb_var_screeninfo ext_tvout_ntsc_var = {
	/* NTSC frame size is 720 * 480,
	* but due to overscan, about 640 x 430 is visible : be aware of this when drawing in the framebuffer
	*/
	.xres = NTSC_DISP_XRES,
	.yres = NTSC_DISP_YRES,
	.xres_virtual	= NTSC_DISP_XRES,
	.yres_virtual	= NTSC_DISP_YRES * NB_FRAME_BUFFERS,
	.xoffset	= 0,
	.yoffset	= 0,
	.bits_per_pixel	= 32,
	.grayscale	= 0,
	.red		= {16, 8, 0},
	.green		= { 8, 8, 0},
	.blue		= { 0, 8, 0},
	.transp		= { 24, 8, 0},
	.nonstd		= 0,
	.activate	= FB_ACTIVATE_NOW,
	.height		= -1,
	.width		= -1,
	.accel_flags	= 0,
	.pixclock	= 74000,	  /* picoseconds =27/2 MHz */
	.left_margin	= 89,		  /* 60+32 pixclocks */
	.right_margin	= 19,		  /* 20 pixclocks */
	.upper_margin	= 15,		  /* line clocks: modified every field */
	.lower_margin	= (4-FIX_V_SYNC), /* line clocks */
	.hsync_len	= 30,		  /* 62-32 pixclocks */
	.vsync_len	= (3+FIX_V_SYNC),	/* line clocks */
						/* do the cpld fix here because upper_margin is modified every field to simulate interlace stream */
	.sync		= 1,
	.vmode		= FB_VMODE_IBUS,
	.rotate		= 0,
	.reserved[0]	= 0,
};

static struct fb_var_screeninfo ext_tvout_hd_pal_var = {
	.xres		= HDMI_720P_DISP_XRES,
	.yres		= HDMI_720P_DISP_YRES,
	.xres_virtual	= HDMI_720P_DISP_XRES,
	.yres_virtual	= HDMI_720P_DISP_YRES * NB_FRAME_BUFFERS,
	.xoffset	= 0,
	.yoffset	= 0,
	.bits_per_pixel	= 32,
	.grayscale	= 0,
	.red		= {16, 8, 0},
	.green		= { 8, 8, 0},
	.blue		= { 0, 8, 0},
	.transp		= { 0, 0, 0},
	.nonstd		= 0,
	.activate	= FB_ACTIVATE_NOW,
	.height		= -1,
	.width		= -1,
	.accel_flags	= 0,
	.pixclock	= 13468,	/* 74.25Mhz in picoseconds val = 10e6/F(MHz) */
	.left_margin	= 292,		/* pixclocks NOTE 220 + 32 our cpld will by default add 32 pixclock to the h sync len */
	.right_margin	= 70,		/* pixclocks */
	.upper_margin	= (20+FIX_V_SYNC),		/* line clocks front porch 25-5 */
	.lower_margin	= (5-FIX_V_SYNC),		/* line clocks */
	.hsync_len	= 8,		/* pixclocks 40-32 our cpld will by default add 32 pixclock here */
	.vsync_len	= 5,		/* line clocks */
	.sync		= 1,
	.vmode		= FB_VMODE_NONINTERLACED,
	.rotate		= 0,
	.reserved[0]	= 0,
};

//------------------------------------------------------

static struct fb_var_screeninfo ext_tvout_var_conf[RESOLUTION_NB_VIDEOEXT];

// init lcd conf, init display struct for lcd,
// get spi client, get pwm backlight
static int ext_tvout_panel_init(void)
{
	fops.isr_handler              = videoenc_isr_handler;
	fops.vidout_select_format     = _select_format;
	fops.vidout_select_analog_out = _select_analog_out;
	fops.vidout_reset             = _videoenc_reset;

	printk (KERN_INFO "external tvout panel \n");
	memcpy(&ext_tvout_var_conf[RESOLUTION_VIDEOEXT_PAL],   &ext_tvout_pal_var,   sizeof(struct fb_var_screeninfo));
	memcpy(&ext_tvout_var_conf[RESOLUTION_VIDEOEXT_NTSC],  &ext_tvout_ntsc_var,  sizeof(struct fb_var_screeninfo));
	memcpy(&ext_tvout_var_conf[RESOLUTION_VIDEOEXT_HDPAL], &ext_tvout_hd_pal_var,sizeof(struct fb_var_screeninfo));

	// register int_tvout panel conf to display out
	display_out_init_ext_videoenc_info( (struct fb_var_screeninfo*)ext_tvout_var_conf,RESOLUTION_NB_VIDEOEXT,&fops );

// request platform gpio
// #if defined(CONFIG_MACH_OMAP_2430SDP) || defined(CONFIG_MACH_OMAP_3430SDP)
// 	omap_request_gpio( panel_power_gpio);  /* LCD panel */
// 	omap_set_gpio_direction( panel_power_gpio, 0); /* output */
// #endif
	return 0;
}

static int ext_tvout_panel_exit(void)
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

#ifdef CONFIG_PM
static int ext_tvout_panel_suspend(struct platform_device *dev, pm_message_t state)
{
	if (!panel_in_use)
		return 0;
	return 0;
}

static int ext_tvout_panel_resume(struct platform_device *dev)
{
	if (panel_in_use)
		return 0;
	return 0;
}

#endif /* CONFIG_PM */

static struct platform_driver g6_ext_tvout_panel_driver = {
	.driver = {
		.name   = "g6_ext_tvout_panel",
	},
#ifdef CONFIG_PM
	.suspend        = ext_tvout_panel_suspend,
	.resume         = ext_tvout_panel_resume,
#endif
};

static struct platform_device ext_tvout_panel_device = {
        .name     = "ext_tvout_panel",
	.id    = 12,
};

//static int __init g6_lcd_drv_init(void)
int __init g6_ext_tvout_panel_drv_init(void)
{
printk(" init module ext_tvout panel driver\n");
	/* Register the driver with LDM */
	if ( platform_driver_register( &g6_ext_tvout_panel_driver )) {
		printk(KERN_ERR DRIVER ": failed to register g6_ext_tvout_panel driver\n");
		return -ENODEV;
	}
	
	/* Register the device with LDM */
	if (platform_device_register(&ext_tvout_panel_device)) {
		printk(KERN_ERR DRIVER ": failed to register g6_ext_tvout_panel device\n");
		platform_driver_unregister(&g6_ext_tvout_panel_driver);
		return -ENODEV;
	}
	ext_tvout_panel_init();
	return 0;
}

static void __exit g6_ext_tvout_panel_drv_cleanup(void)
{
	printk(" exit ext_tvout panel driver\n");
	ext_tvout_panel_exit();
	platform_device_unregister(&ext_tvout_panel_device);
	platform_driver_unregister(&g6_ext_tvout_panel_driver);
}

MODULE_AUTHOR("ARCHOS S.A.");
MODULE_DESCRIPTION("ext_tvout panel driver");
MODULE_LICENSE("GPL");

module_init(g6_ext_tvout_panel_drv_init);
module_exit(g6_ext_tvout_panel_drv_cleanup);
