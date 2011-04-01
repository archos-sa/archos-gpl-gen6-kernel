/*
 *  Copyright (C) 2007 
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or (at
 *  your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#ifndef _LINUX_DISPLAY_OUT_H
#define _LINUX_DISPLAY_OUT_H

#define DISPLAY_OUT_MASK 	0x07
#define ANALOG_OUT_MASK 	0x70

#define DISPLAY_OUT_DEFAULT	0x0
#define DISPLAY_OUT_LCD		0x1
#define DISPLAY_OUT_TV		0x2
#define DISPLAY_OUT_HDMI	0x3
#define DISPLAY_OUT_TV_CRADLE	0x4
#define DISPLAY_OUT_LAST	DISPLAY_OUT_TV_CRADLE

#define ANALOG_OUT_DEFAULT	0x0
#define ANALOG_OUT_CVBS		0x10
#define ANALOG_OUT_SVIDEO	0x20
#define ANALOG_OUT_RGB		0x30
#define ANALOG_OUT_COMPONENT	0x40



#define RESOLUTION_VGA			0
#define RESOLUTION_480P			1
#define RESOLUTION_576P			2
#define RESOLUTION_720P50		3
#define RESOLUTION_720P60		4

//Those resolutions have to be declared in the same order as in file arch-omap/display.h
#define RESOLUTION_PAL_BDGHI		0
#define RESOLUTION_PAL_NC		1
#define RESOLUTION_PAL_N		2
#define RESOLUTION_PAL_M		3
#define RESOLUTION_PAL_60		4
#define RESOLUTION_NTSC_M		5	
#define RESOLUTION_NTSC_J		6
#define RESOLUTION_NTSC_443		7

#define NB_MAX_RESOLUTION_PER_PANEL	8

#define LCD_DISP_XRES      	800
#define LCD_DISP_YRES      	480
#define HDMI_VGA_DISP_XRES      640
#define HDMI_VGA_DISP_YRES      480
#define HDMI_480P_DISP_XRES     720
#define HDMI_480P_DISP_YRES     480
#define HDMI_576P_DISP_XRES     720
#define HDMI_576P_DISP_YRES     576
#define HDMI_720P_DISP_XRES    1280
#define HDMI_720P_DISP_YRES     720
#define NTSC_DISP_XRES		720
#define NTSC_DISP_YRES		480
#define PAL_DISP_XRES		720
#define PAL_DISP_YRES		576


#define DISPLAY_XRES_MAX		1280
#define DISPLAY_YRES_MAX		720

#define NB_FRAME_BUFFERS		1

#define VIDEO_EXT_ENC_ISR_REGISTERED	(1<<0)
#define VIDEO_EXT_ENC_ISR_ON		(1<<1)

#define RESOLUTION_VIDEOEXT_PAL		0
#define RESOLUTION_VIDEOEXT_NTSC	1
#define RESOLUTION_VIDEOEXT_HDPAL	2
#define RESOLUTION_NB_VIDEOEXT		3



struct lcd_fops {

	int (* enable_display)( int enable );
	int (* enable_backlight)( int enable );
	int (* backlight_level)( int level );
	int (* set_contrast)( int level );
	int (* set_brightness)( int level );
	int (* set_gamma)( int level );
	int (* set_reg)( int reg, int val );
	int (* get_reg)( int reg, int *val );
	int (* set_conf)( int level );
	int (* set_sync)( int level );
	int (* set_vcom)( int level );
};

struct tv_fops {

	int (*vidout_rgb)( int enable );
	int (*vidout_widescreen)( int enable );
	int (*vidout_ext)( int enable );
	int (*vidout_enable)( int enable );
};

struct ext_videoenc_fops {

	void (*isr_handler)(void *arg, struct pt_regs *regs);
	int (*vidout_select_format)( int type );
	int (*vidout_select_analog_out)( int type );
	int isr_state;
	void (*vidout_reset)( void );
};


typedef struct {
	int reg;
	int val;
	int par1;
	int par2;
	} lcd_disp_t;

	
struct omap_fb_panel_select{
	int interface_id;
	int format_id;
};
		
struct display_out{
	struct omap_fb_panel_select select;
	struct lcd_panel *lcd_panel_var;
	struct lcd_fops *lcd_panel_ops;
	int nb_int_tvout_format;
	struct fb_var_screeninfo int_tvout_panel_var[NB_MAX_RESOLUTION_PER_PANEL];
// 	struct tv_fops *tv_panel_ops;
	int nb_hdmi_format;
	struct fb_var_screeninfo hdmi_panel_var[NB_MAX_RESOLUTION_PER_PANEL];

	/* External Video Encoder: cradle */
	int nb_ext_tvout_format;
	struct fb_var_screeninfo ext_tvout_panel_var[NB_MAX_RESOLUTION_PER_PANEL];
	
	/* fops for video encoder */
	struct ext_videoenc_fops *ext_videoenc_panel_ops;

};

struct display_io_fops {
	void (* display_reset)( int enable );
	void (* display_select)( int enable );
	void (* display_hdmidac)( int enable );
	void (* display_set_aux)( int enable );
};

struct lcd_io_fops {
	void (* lcd_power)( int enable );
	void (* lcd_reset)( int enable );
	void (* backlight_power)( int enable );
	void (* lcd_pci)( int enable );
};

struct hdmi_io_fops {
	int (* hdmi_get_it_io)( void );
};

struct display_io_fops *archosg6_display_get_io(void);

/* IOCTL commands. */
#define  DISP_GET_ID			_IOWR ( 'D', 0, lcd_disp_t*)
#define  DISP_SET_CONTRAST		_IOWR ( 'D', 1, lcd_disp_t*)
#define  DISP_SET_BRIGHTNESS		_IOWR ( 'D', 2, lcd_disp_t*)
#define  DISP_SET_GAMMA			_IOWR ( 'D', 3, lcd_disp_t*)
#define  DISP_SET_SYNC			_IOWR ( 'D', 4, lcd_disp_t*)
#define  DISP_SET_CONFIG		_IOWR ( 'D', 5, lcd_disp_t*)
#define  DISP_SET_VCOM			_IOWR ( 'D', 6, lcd_disp_t*)
#define  DISP_GET_REG			_IOWR ( 'D', 7, lcd_disp_t*)
#define  DISP_SET_REG			_IOWR ( 'D', 8, lcd_disp_t*)
#define  DISP_SET_BCKL_LEV		_IOWR ( 'D', 9, lcd_disp_t*)
#define  DISP_ENABLE_DISP		_IOWR ( 'D', 10, lcd_disp_t*)
#define  DISP_ENABLE_BCKL		_IOWR ( 'D', 11, lcd_disp_t*)


#define DISP_LCD 0


#endif
