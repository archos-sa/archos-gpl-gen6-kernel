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



#define DBG if(1)
#define DEBUGMSG DBG printk

#define MOD_INC_USE_COUNT
#define MOD_DEC_USE_COUNT



#ifdef CONFIG_MACH_OMAP_3430SDP

#endif
#ifdef CONFIG_MACH_ARCHOS_G6S

#endif

#define DRIVER		"internal tv output parameters"

static int panel_in_use = 0;
// ------------------------------------------
// FB configuration 


static struct fb_var_screeninfo int_tvout_pal_bdghi_var = {
	/* PAL frame size is 720 * 546,
	* but due to overscan, about 640 x 520 is visible : be aware of this when drawing in the framebuffer
	*/
	.xres = PAL_DISP_XRES,
	.yres = PAL_DISP_YRES,
	.xres_virtual	= PAL_DISP_XRES,
	.yres_virtual	= PAL_DISP_YRES*NB_FRAME_BUFFERS,
	.xoffset	= 0,
	.yoffset	= 0,
	.bits_per_pixel	= 32,
	.grayscale	= 0,
// 	.red		= {16, 8, 0},
// 	.green		= { 8, 8, 0},
// 	.blue		= { 0, 8, 0},
// 	.transp		= { 0, 0, 0},
	.red		= {16, 8, 0},
	.green		= { 8, 8, 0},
	.blue		= { 0, 8, 0},
	.transp		= { 24, 8, 0},
	.nonstd		= 0,
	.activate	= FB_ACTIVATE_NOW,
	.height		= -1,
	.width		= -1,
	.accel_flags	= 0,
	.pixclock	= 37037,		/* picoseconds */
	.left_margin	= 0,		/* pixclocks */
	.right_margin	= 0,		/* pixclocks */
	.upper_margin	= 0,		/* line clocks */
	.lower_margin	= 0,		/* line clocks */
	.hsync_len	= 0,		/* pixclocks */
	.vsync_len	= 0,		/* line clocks */
	.sync		= 0,
	.vmode		= FB_VMODE_INTERLACED,
	.rotate		= 0,
	.reserved[0]	= 0,
};

static struct fb_var_screeninfo int_tvout_pal_nc_var = {
	/* PAL frame size is 720 * 546,
	* but due to overscan, about 640 x 520 is visible : be aware of this when drawing in the framebuffer
	*/
	.xres = PAL_DISP_XRES,
	.yres = PAL_DISP_YRES,
	.xres_virtual	= PAL_DISP_XRES,
	.yres_virtual	= PAL_DISP_YRES*NB_FRAME_BUFFERS,
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
	.pixclock	= 37037,		/* picoseconds */
	.left_margin	= 0,		/* pixclocks */
	.right_margin	= 0,		/* pixclocks */
	.upper_margin	= 0,		/* line clocks */
	.lower_margin	= 0,		/* line clocks */
	.hsync_len	= 0,		/* pixclocks */
	.vsync_len	= 0,		/* line clocks */
	.sync		= 0,
	.vmode		= FB_VMODE_INTERLACED,
	.rotate		= 0,
	.reserved[0]	= 0,
};

static struct fb_var_screeninfo int_tvout_pal_n_var = {
	/* PAL frame size is 720 * 546,
	* but due to overscan, about 640 x 520 is visible : be aware of this when drawing in the framebuffer
	*/
	.xres = PAL_DISP_XRES,
	.yres = PAL_DISP_YRES,
	.xres_virtual	= PAL_DISP_XRES,
	.yres_virtual	= PAL_DISP_YRES*NB_FRAME_BUFFERS,
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
	.pixclock	= 37037,		/* picoseconds */
	.left_margin	= 0,		/* pixclocks */
	.right_margin	= 0,		/* pixclocks */
	.upper_margin	= 0,		/* line clocks */
	.lower_margin	= 0,		/* line clocks */
	.hsync_len	= 0,		/* pixclocks */
	.vsync_len	= 0,		/* line clocks */
	.sync		= 0,
	.vmode		= FB_VMODE_INTERLACED,
	.rotate		= 0,
	.reserved[0]	= 0,
};

static struct fb_var_screeninfo int_tvout_pal_m_var = {
	/* PAL frame size is 720 * 546,
	* but due to overscan, about 640 x 520 is visible : be aware of this when drawing in the framebuffer
	*/
	.xres = PAL_DISP_XRES,
	.yres = PAL_DISP_YRES,
	.xres_virtual	= PAL_DISP_XRES,
	.yres_virtual	= PAL_DISP_YRES*NB_FRAME_BUFFERS,
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
	.pixclock	= 37037,		/* picoseconds */
	.left_margin	= 0,		/* pixclocks */
	.right_margin	= 0,		/* pixclocks */
	.upper_margin	= 0,		/* line clocks */
	.lower_margin	= 0,		/* line clocks */
	.hsync_len	= 0,		/* pixclocks */
	.vsync_len	= 0,		/* line clocks */
	.sync		= 0,
	.vmode		= FB_VMODE_INTERLACED,
	.rotate		= 0,
	.reserved[0]	= 0,
};

static struct fb_var_screeninfo int_tvout_pal_60_var = {
	/* PAL frame size is 720 * 546,
	* but due to overscan, about 640 x 520 is visible : be aware of this when drawing in the framebuffer
	*/
	.xres = PAL_DISP_XRES,
	.yres = PAL_DISP_YRES,
	.xres_virtual	= PAL_DISP_XRES,
	.yres_virtual	= PAL_DISP_YRES*NB_FRAME_BUFFERS,
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
	.pixclock	= 37037,		/* picoseconds */
	.left_margin	= 0,		/* pixclocks */
	.right_margin	= 0,		/* pixclocks */
	.upper_margin	= 0,		/* line clocks */
	.lower_margin	= 0,		/* line clocks */
	.hsync_len	= 0,		/* pixclocks */
	.vsync_len	= 0,		/* line clocks */
	.sync		= 0,
	.vmode		= FB_VMODE_INTERLACED,
	.rotate		= 0,
	.reserved[0]	= 0,
};

static struct fb_var_screeninfo int_tvout_ntsc_m_var = {
	/* NTSC frame size is 720 * 480,
	* but due to overscan, about 640 x 430 is visible : be aware of this when drawing in the framebuffer
	*/
	.xres = NTSC_DISP_XRES,
	.yres = NTSC_DISP_YRES,
	.xres_virtual	= NTSC_DISP_XRES,
	.yres_virtual	= NTSC_DISP_YRES*NB_FRAME_BUFFERS,
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
	.pixclock	= 37000,		/* picoseconds */
	.left_margin	= 0,		/* pixclocks */
	.right_margin	= 0,		/* pixclocks */
	.upper_margin	= 0,		/* line clocks */
	.lower_margin	= 0,		/* line clocks */
	.hsync_len	= 0,		/* pixclocks */
	.vsync_len	= 0,		/* line clocks */
	.sync		= 0,
	.vmode		= FB_VMODE_INTERLACED,
	.rotate		= 0,
	.reserved[0]	= 0,
};

static struct fb_var_screeninfo int_tvout_ntsc_j_var = {
	/* NTSC frame size is 720 * 480,
	* but due to overscan, about 640 x 430 is visible : be aware of this when drawing in the framebuffer
	*/
	.xres = NTSC_DISP_XRES,
	.yres = NTSC_DISP_YRES,
	.xres_virtual	= NTSC_DISP_XRES,
	.yres_virtual	= NTSC_DISP_YRES*NB_FRAME_BUFFERS,
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
	.pixclock	= 37000,		/* picoseconds */
	.left_margin	= 0,		/* pixclocks */
	.right_margin	= 0,		/* pixclocks */
	.upper_margin	= 0,		/* line clocks */
	.lower_margin	= 0,		/* line clocks */
	.hsync_len	= 0,		/* pixclocks */
	.vsync_len	= 0,		/* line clocks */
	.sync		= 0,
	.vmode		= FB_VMODE_INTERLACED,
	.rotate		= 0,
	.reserved[0]	= 0,
};

static struct fb_var_screeninfo int_tvout_ntsc_443_var = {
	/* NTSC frame size is 720 * 480,
	* but due to overscan, about 640 x 430 is visible : be aware of this when drawing in the framebuffer
	*/
	.xres = NTSC_DISP_XRES,
	.yres = NTSC_DISP_YRES,
	.xres_virtual	= NTSC_DISP_XRES,
	.yres_virtual	= NTSC_DISP_YRES*NB_FRAME_BUFFERS,
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
	.pixclock	= 37000,		/* picoseconds */
	.left_margin	= 0,		/* pixclocks */
	.right_margin	= 0,		/* pixclocks */
	.upper_margin	= 0,		/* line clocks */
	.lower_margin	= 0,		/* line clocks */
	.hsync_len	= 0,		/* pixclocks */
	.vsync_len	= 0,		/* line clocks */
	.sync		= 0,
	.vmode		= FB_VMODE_INTERLACED,
	.rotate		= 0,
	.reserved[0]	= 0,
};


//------------------------------------------------------
extern int display_out_init_tvinfo( struct fb_var_screeninfo *i_p_fb_var, u32 i_nb_resolutions );
static u32 int_tvout_panel_nb_resolution = 0;
static struct fb_var_screeninfo int_tvout_var_conf[NB_MAX_RESOLUTION_PER_PANEL];

// init lcd conf, init display struct for lcd,
// get spi client, get pwm backlight
static int int_tvout_panel_init(void)
{

printk ("init internal tvout panel \n");
	int_tvout_panel_nb_resolution = 0;
	memcpy(&int_tvout_var_conf[int_tvout_panel_nb_resolution],&int_tvout_pal_bdghi_var,sizeof(struct fb_var_screeninfo));
	int_tvout_panel_nb_resolution ++;
	memcpy(&int_tvout_var_conf[int_tvout_panel_nb_resolution],&int_tvout_pal_nc_var,sizeof(struct fb_var_screeninfo));
	int_tvout_panel_nb_resolution ++;
	memcpy(&int_tvout_var_conf[int_tvout_panel_nb_resolution],&int_tvout_pal_n_var,sizeof(struct fb_var_screeninfo));
	int_tvout_panel_nb_resolution ++;
	memcpy(&int_tvout_var_conf[int_tvout_panel_nb_resolution],&int_tvout_pal_m_var,sizeof(struct fb_var_screeninfo));
	int_tvout_panel_nb_resolution ++;
	memcpy(&int_tvout_var_conf[int_tvout_panel_nb_resolution],&int_tvout_pal_60_var,sizeof(struct fb_var_screeninfo));
	int_tvout_panel_nb_resolution ++;
	memcpy(&int_tvout_var_conf[int_tvout_panel_nb_resolution],&int_tvout_ntsc_m_var,sizeof(struct fb_var_screeninfo));
	int_tvout_panel_nb_resolution ++;
	memcpy(&int_tvout_var_conf[int_tvout_panel_nb_resolution],&int_tvout_ntsc_j_var,sizeof(struct fb_var_screeninfo));
	int_tvout_panel_nb_resolution ++;
	memcpy(&int_tvout_var_conf[int_tvout_panel_nb_resolution],&int_tvout_ntsc_443_var,sizeof(struct fb_var_screeninfo));
	int_tvout_panel_nb_resolution ++;
	// register int_tvout panel conf to display out
	display_out_init_tvinfo( (struct fb_var_screeninfo*)int_tvout_var_conf,int_tvout_panel_nb_resolution );

// request platform gpio
// #if defined(CONFIG_MACH_OMAP_2430SDP) || defined(CONFIG_MACH_OMAP_3430SDP)
// 	omap_request_gpio( panel_power_gpio);  /* LCD panel */
// 	omap_set_gpio_direction( panel_power_gpio, 0); /* output */
// #endif
	return 0;
}


static int int_tvout_panel_exit(void)
{
	omap2_disp_get_dss();
	omap2_disp_disable_output_dev( OMAP2_OUTPUT_TV );
	omap2_disp_put_dss();

// #if defined(CONFIG_MACH_OMAP_2430SDP) || defined(CONFIG_MACH_OMAP_3430SDP)
// 	omap_free_gpio( panel_power_gpio);  /* LCD panel */
// #endif

	return 0;
}
/* ------------------------------------------------------------------------------ */
/* Power and device Management */

static int __init int_tvout_panel_probe(struct platform_device *odev)
{
	printk("int_tvout_panel_probe\n");
	return int_tvout_panel_init();
}

#ifdef CONFIG_PM
static int int_tvout_panel_suspend(struct platform_device *odev, pm_message_t state)
{
	if (!panel_in_use)
		return 0;
	return 0;
}

static int int_tvout_panel_resume(struct platform_device *odev)
{
	if (panel_in_use)
		return 0;
	return 0;
}

#endif /* CONFIG_PM */

static struct platform_driver g6_int_tvout_panel_driver = {
	.driver = {
		.name   = "g6_int_tvout_panel",
	},
	.probe          = int_tvout_panel_probe,
#ifdef CONFIG_PM
	.suspend        = int_tvout_panel_suspend,
	.resume         = int_tvout_panel_resume,
#endif
};

static struct platform_device int_tvout_panel_device = {
        .name     = "int_tvout_panel",
	.id    = 10,
};


//static int __init g6_lcd_drv_init(void)
int __init g6_int_tvout_panel_drv_init(void)
{
printk(" init module int_tvout panel driver\n");
	/* Register the driver with LDM */
	if ( platform_driver_register( &g6_int_tvout_panel_driver )) {
		printk(KERN_ERR DRIVER ": failed to register g6_int_tvout_panel driver\n");
		return -ENODEV;
	}
	
	/* Register the device with LDM */
	if (platform_device_register(&int_tvout_panel_device)) {
		printk(KERN_ERR DRIVER ": failed to register g6_int_tvout_panel device\n");
		platform_driver_unregister(&g6_int_tvout_panel_driver);
		return -ENODEV;
	}
	int_tvout_panel_init();
	return 0;
}

static void __exit g6_int_tvout_panel_drv_cleanup(void)
{
	printk(" exit int_tvout panel driver\n");
	int_tvout_panel_exit();
	platform_device_unregister(&int_tvout_panel_device);
	platform_driver_unregister(&g6_int_tvout_panel_driver);
}

MODULE_AUTHOR("ARCHOS S.A.");
MODULE_DESCRIPTION("int_tvout panel driver");
MODULE_LICENSE("GPL");

module_init(g6_int_tvout_panel_drv_init);
module_exit(g6_int_tvout_panel_drv_cleanup);

