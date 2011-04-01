/* drivers/video/omap/omap_g6_disp_out.c */
/*----------------------------------------------------------------------------*/
/* Description :  Driver for LCD and TV output on OMAP34xx archos platform    */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*============================================================================*/
/*                             HISTORICAL                                     */
/*============================================================================*/
/* Authors   :  T.I / ROBIC / LALAURETTE                  Date : 01/01/08     */
/*                                                                            */
/* $Revision: 0.0 $                                                       */
/*----------------------------------------------------------------------------*/

/*============================================================================*/
/*                             FUNCTIONS LIST                                 */
/*============================================================================*/
/*          Functions                        Scope (API, INTERNAL , LOCAL)    */
/*----------------------------------------------------------------------------*/

/*============================================================================*/
/*                        DECLARATION SPACE                                   */
/*============================================================================*/
/*----------------------------------------------------------------------------*/
/*          Includes                                                          */
/*----------------------------------------------------------------------------*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <asm/arch/omapfb.h>
#ifdef CONFIG_TRACK_RESOURCES
#include <linux/device.h>
#endif
#include <asm/hardware.h>
#include <asm/uaccess.h>
#include <asm/delay.h>
#include <asm/arch/display.h>
#include <asm/arch/gpio.h>
#include <asm/arch/clock.h>

#include <linux/workqueue.h>
#include <asm/arch/power_companion.h>
#include <linux/display_out.h>
#ifdef CONFIG_PM
#include <linux/notifier.h>
#include <linux/pm.h>
#endif
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include "omap_fb.h"

/*----------------------------------------------------------------------------*/
/*          Definitions                                                       */
/*----------------------------------------------------------------------------*/
/* ******** Definitions/Consts ********************************************** */
#define DRIVER			"omap2_disp_out"
#define	DRIVER_DESC		"OMAP Display output driver"

#define DEFAULT_LCD_PIXCLOCK_MAX	40000 /* freq 25 MHz lim */
#define DEFAULT_LCD_PIXCLOCK_MIN	13000 /* freq 75 MHz lim */

#define DBG if(1)
#define DEBUGMSG DBG printk


/* ******** Definitions/Types *********************************************** */

/* ******** Definitions/Variables ******************************************* */
struct display_out disp_out;
static struct display_io_fops * display_io;

#ifndef CONFIG_FB_OMAP_BOOTLOADER_INIT
static int display_out_id_prev = -1;
#else
static int display_out_id_prev = DISPLAY_OUT_LCD;
#endif

/* ******** Definitions/Functions ******************************************* */
extern int fb_out_layer;
extern int omap24xxfb_set_output_layer(int layer);
extern int  omap_g6_dispout_ioctl_init(void);
extern void omap_g6_dispout_ioctl_exit(void);
extern int display_out_lcd_panel_to_fb_var(struct fb_var_screeninfo *o_p_fb_var, struct lcd_panel *i_p_lcd_var);
extern void omap_init_lcd_go( void);
extern int omap_init_lcd_interface( struct fb_var_screeninfo * var,int acb, int ipc, int onoff , int rf);
extern int omap_g6_dispout_lcd_stop(void);
extern int omap_g6_dispout_lcd_init(void);
extern void omap_g6_dispout_lcd_exit(void);
extern void omap_g6_dispout_lcd_get_panel_default_var(struct fb_var_screeninfo *var);
extern int omap_g6_dispout_digital_init(void);
extern void omap_g6_dispout_digital_exit(void);
extern void omap_g6_dispout_digital_get_panel_default_var(struct fb_var_screeninfo *var);
extern void omap_g6_dispout_digital_power_tv(int level);
extern int omap_init_digital_interface(const int analog_output,const int format_id);
extern void omap_init_digital_go(void);
extern struct display_io_fops *archosg6_display_get_io(void);


/*---------------------------------------------------------------------------*/
static void display_select( int interface )
{
#ifdef CONFIG_MACH_ARCHOS_G6
	if ( display_io->display_select ) {
		if ( interface == DISPLAY_OUT_LCD || interface == DISPLAY_OUT_TV ) {
			display_io->display_select( 1 );
			display_io->display_hdmidac( 0 );
		} else if ( interface == DISPLAY_OUT_TV_CRADLE) {
			display_io->display_select( 0 );
			display_io->display_hdmidac( 0 );
		}
		else  {
			display_io->display_select( 0 );
			display_io->display_hdmidac( 1 );
		}
	}
#endif
}

int get_panel_is_interlaced(void)
{
	return (display_out_id_prev == DISPLAY_OUT_TV_CRADLE);
}

static void display_io_enable( int enable )
{
#ifdef CONFIG_MACH_ARCHOS_G6
	if ( display_io->display_reset )
		display_io->display_reset( enable );
#endif
}

/*============================================================================*/
/**
 \brief Function called by omap_fb on FBIO_LCD_PUT_SCREENINFO private ioctl
 \param lcd_info pointer to omap_lcd_info structure
 \return 0 if ok, else -1
 */
/*============================================================================*/
int omap_lcd_init(struct omap_lcd_info *lcd_info)
{
	struct fb_var_screeninfo *var = NULL;	
	int acb=0,ipc=0,onoff=0;
	int ret = 0;
	
	omap_g6_dispout_lcd_get_panel_default_var(var);
	if (lcd_info) {
		var->pixclock     = lcd_info->pixclock,
		var->left_margin  = lcd_info->left_margin,
		var->right_margin = lcd_info->right_margin,
		var->upper_margin = lcd_info->upper_margin,
		var->lower_margin = lcd_info->lower_margin,
		var->hsync_len    = lcd_info->hsync_len,
		var->vsync_len    = lcd_info->vsync_len,
		var->sync         = lcd_info->sync,
		acb          = lcd_info->acb,
		ipc          = lcd_info->ipc,
		onoff        = lcd_info->onoff;
	}
	ret = omap_init_lcd_interface(var,acb,ipc,onoff,1);
	omap_init_lcd_go();
	return ret;
}

//-------------------------------------------------------------------------
/*============================================================================*/
/**
 \brief Init display out driver
 \return 0 if ok, else -1
 */
/*============================================================================*/
static int __init omap_g6_dispout_init(void)
{
	int ret = 0;

	printk(KERN_DEBUG "omap_g6_dispout_init\n");

	ret = omap_g6_dispout_lcd_init();
	ret = omap_g6_dispout_digital_init();
	ret = omap_g6_dispout_ioctl_init();
#ifdef CONFIG_MACH_ARCHOS_G6
	display_io = archosg6_display_get_io();
#endif

	memset ( &disp_out, 0, sizeof( disp_out) );
	printk( "Display out Driver register success\n" );
	// enable cpld here, must be move depending on config for power mgt
	display_io_enable( 1 );
	return ret;
}

device_initcall(omap_g6_dispout_init);


/*============================================================================*/
/**
 \brief Release display out driver
 */
/*============================================================================*/
static void __exit omap_g6_dispout_exit(void)
{
	omap_g6_dispout_digital_exit();
	omap_g6_dispout_lcd_exit();
	omap_g6_dispout_ioctl_exit();

	display_io_enable( 0 );
	display_io = NULL;

}
module_exit(omap_g6_dispout_exit);

/*============================================================================*/
/**
 \brief get default framebuffer screen info var, according to the selected output device 
 \param o_p_var  pointer to framebuffer screen info var struct
 \param output_dev  OMAP2_OUTPUT_LCD or OMAP2_OUTPUT_TV
  */
/*============================================================================*/
void get_panel_default_var(struct fb_var_screeninfo *o_p_var, const int i_output_dev)
{
	printk( KERN_DEBUG " get panel default info var\n");
	if (i_output_dev == OMAP2_OUTPUT_LCD) {
		omap_g6_dispout_lcd_get_panel_default_var(o_p_var);
	}else if (i_output_dev == OMAP2_OUTPUT_TV) {		
		omap_g6_dispout_digital_get_panel_default_var(o_p_var);
	}
}

/*============================================================================*/
/**
 \brief get framebuffer screen info var, according to the selected panel 
 \param o_pb_fb_var  pointer to framebuffer screen info var structure
 \param i_p_sel  pointer to a panel select structure
 \return 0 if ok, else -1
 */
/*============================================================================*/
int get_panel_var(struct fb_var_screeninfo *o_pb_fb_var, struct omap_fb_panel_select *i_p_sel)
{
	int ret = 0;
	int display_out_id = i_p_sel->interface_id & DISPLAY_OUT_MASK;
	
	printk( KERN_DEBUG " get panel info var for interface =%d format=%d\n",i_p_sel->interface_id,i_p_sel->format_id);

	/* init with default value */
	omap_g6_dispout_lcd_get_panel_default_var( o_pb_fb_var );
	
	if(display_out_id<= DISPLAY_OUT_LAST){
		switch( display_out_id & DISPLAY_OUT_MASK ){
			case DISPLAY_OUT_DEFAULT:
			case DISPLAY_OUT_LCD:
				printk(KERN_INFO "get_panel_var : LCD\n");
				display_out_lcd_panel_to_fb_var( o_pb_fb_var, disp_out.lcd_panel_var );
				break;
			case DISPLAY_OUT_TV:
				if(i_p_sel->format_id < disp_out.nb_int_tvout_format){
					printk(KERN_INFO "get_panel_var : INT TV OUT [%d]\n",i_p_sel->format_id);
					memcpy(o_pb_fb_var,
					       &(disp_out.int_tvout_panel_var[i_p_sel->format_id]), sizeof(struct fb_var_screeninfo));
					printk(KERN_INFO "get_panel_var : xres=%d yres=%d pixclock=%d\n",
					       disp_out.int_tvout_panel_var[i_p_sel->format_id].xres,
					       disp_out.int_tvout_panel_var[i_p_sel->format_id].yres,
					       disp_out.int_tvout_panel_var[i_p_sel->format_id].pixclock);
				} else {
					printk(KERN_INFO "get_panel_var : INT TV OUT [%d] error, format not supported.\n",i_p_sel->format_id);
				}
				break;
			case DISPLAY_OUT_HDMI:
				if(i_p_sel->format_id < disp_out.nb_hdmi_format){
					printk(KERN_INFO "get_panel_var : HDMI [%d]\n",i_p_sel->format_id);
					memcpy(o_pb_fb_var,
					       &(disp_out.hdmi_panel_var[i_p_sel->format_id]), sizeof(struct fb_var_screeninfo));
					printk(KERN_INFO "get_panel_var : xres=%d yres=%d pixclock=%d\n",
					       disp_out.hdmi_panel_var[i_p_sel->format_id].xres,
					       disp_out.hdmi_panel_var[i_p_sel->format_id].yres,
					       disp_out.hdmi_panel_var[i_p_sel->format_id].pixclock);
				} else {
					printk(KERN_INFO "get_panel_var : HDMI [%d] error, format not supported.\n",i_p_sel->format_id);
				}
				break;
			case DISPLAY_OUT_TV_CRADLE:
				if(i_p_sel->format_id < disp_out.nb_ext_tvout_format){
					printk(KERN_INFO "get_panel_var : EXTERNAL VIDEO ENC [%d]\n",i_p_sel->format_id);
					memcpy(o_pb_fb_var,
					       &(disp_out.ext_tvout_panel_var[i_p_sel->format_id]), sizeof(struct fb_var_screeninfo));
					printk(KERN_INFO "get_panel_var : xres=%d yres=%d pixclock=%d\n",
					       disp_out.ext_tvout_panel_var[i_p_sel->format_id].xres,
					       disp_out.ext_tvout_panel_var[i_p_sel->format_id].yres,
					       disp_out.ext_tvout_panel_var[i_p_sel->format_id].pixclock);
				} else {
					printk(KERN_INFO "get_panel_var : EXTERNAL VIDEO ENC [%d] error, format not supported.\n",i_p_sel->format_id);
				}
				break;
			default:
				printk(KERN_INFO "get_panel_var : display_out_id %d error, not supported.\n",display_out_id);
				ret = -EINVAL;
				break;
		}
	}else {
		printk(KERN_INFO "get_panel_var : display_out_id %d > DISPLAY_OUT_LAST=%d error.\n",display_out_id,DISPLAY_OUT_LAST);
		ret = -1;
	}
	return ret;
}

/*============================================================================*/
/**
 \brief get the maximal panel pixel clock / output device
 \param i_output_dev  output device
 \return pixel clock value
 */
/*============================================================================*/
u32 get_panel_pixclock_max(const int i_output_dev)
{
	if (i_output_dev == OMAP2_OUTPUT_LCD) {
		return DEFAULT_LCD_PIXCLOCK_MAX;
	}
	else if (i_output_dev == OMAP2_OUTPUT_TV) {
		return ~0;
	}

	return -EINVAL;
}

/*============================================================================*/
/**
 \brief get the minimal panel pixel clock / output device
 \param i_output_dev  output device
 \return pixel clock value
 */
/*============================================================================*/
u32 get_panel_pixclock_min(const int i_output_dev)
{
	if (i_output_dev == OMAP2_OUTPUT_LCD) {
		return DEFAULT_LCD_PIXCLOCK_MIN;
	}
	else if (i_output_dev == OMAP2_OUTPUT_TV) {
		return 0;
	}

	return -EINVAL;
}

/*============================================================================*/
/**
 \brief select a panel output device and its resolution
 \param i_p_sel  pointer to a panel select structure
 \return 0 if ok, else -1
 */
/*============================================================================*/
int set_panel_output(struct omap_fb_panel_select *i_p_sel)
{
	int ret = 0;

	struct fb_var_screeninfo fb_var;
	int display_out_id = i_p_sel->interface_id & DISPLAY_OUT_MASK;

	printk( KERN_DEBUG "set_panel_output %d \n", display_out_id);

	if ( (display_out_id_prev != display_out_id) && (display_out_id_prev == DISPLAY_OUT_TV_CRADLE) ) {
		/* disable cradle irq */
		if ( (disp_out.ext_videoenc_panel_ops->isr_state & VIDEO_EXT_ENC_ISR_ON) ) {
			omap2_disp_irqdisable((omap2_disp_isr_t)disp_out.ext_videoenc_panel_ops->isr_handler,DISPC_IRQSTATUS_PROGRAMMEDLINENUMBER);
			disp_out.ext_videoenc_panel_ops->isr_state &= ~ VIDEO_EXT_ENC_ISR_ON;
			
			omap2_disp_unregister_isr((omap2_disp_isr_t)disp_out.ext_videoenc_panel_ops->isr_handler);
			disp_out.ext_videoenc_panel_ops->isr_state &= ~ VIDEO_EXT_ENC_ISR_REGISTERED;
		}

		if ( disp_out.ext_videoenc_panel_ops->vidout_reset ) {
			disp_out.ext_videoenc_panel_ops->vidout_reset();
		} else
			printk("cannot set reset video out\n");
	}
	
	/* init with default value */
	omap_g6_dispout_lcd_get_panel_default_var(&fb_var);
	
	if(display_out_id <= DISPLAY_OUT_LAST){

		if ( display_out_id_prev != display_out_id) {
			omap2_disp_get_dss();
			omap2_disp_disable_output_dev(OMAP2_OUTPUT_LCD);
			omap2_disp_disable_output_dev(OMAP2_OUTPUT_TV);
			omap_g6_dispout_digital_power_tv(TV_OFF);
			omap2_disp_put_dss();
		}

		display_out_id_prev = display_out_id;

		switch(display_out_id){
			case DISPLAY_OUT_LCD:
				printk(KERN_INFO "set_panel_output : LCD\n");
				disp_out.select.interface_id = i_p_sel->interface_id; 
				disp_out.select.format_id = 0;
				display_out_lcd_panel_to_fb_var(&fb_var,disp_out.lcd_panel_var);
				omap_init_lcd_interface(&fb_var,0,0,1,1);
				mdelay(1);
				omap2_disp_get_dss();

				omap2_disp_set_output_dev(OMAP2_VIDEO1, OMAP2_OUTPUT_LCD);
				omap2_disp_set_output_dev(OMAP2_VIDEO2, OMAP2_OUTPUT_LCD);
				omap2_disp_set_output_dev(OMAP2_GRAPHICS, OMAP2_OUTPUT_LCD); /* last, set GO bit */

				omap2_disp_put_dss();
				break;
			case DISPLAY_OUT_TV:
				if (i_p_sel->format_id < disp_out.nb_int_tvout_format){
					printk(KERN_INFO "set_panel_output : INT TV OUT [%d]\n",i_p_sel->format_id);
					disp_out.select.interface_id = i_p_sel->interface_id; 
					disp_out.select.format_id = i_p_sel->format_id;
					memcpy(&fb_var ,&(disp_out.int_tvout_panel_var[i_p_sel->format_id]),sizeof(struct fb_var_screeninfo));
					omap_init_digital_interface(i_p_sel->interface_id & ANALOG_OUT_MASK, i_p_sel->format_id);
					mdelay(1);
					omap2_disp_get_dss();
					
					omap2_disp_set_output_dev(OMAP2_VIDEO1, OMAP2_OUTPUT_TV);
					omap2_disp_set_output_dev(OMAP2_VIDEO2, OMAP2_OUTPUT_TV);
					omap2_disp_set_output_dev(OMAP2_GRAPHICS, OMAP2_OUTPUT_TV); /* last, set GO bit */

					omap2_disp_put_dss();
				} else {
					printk(KERN_INFO "set_panel_output : INT TV OUT [%d] error, format not supported.\n",i_p_sel->format_id);
				}
				break;
			case DISPLAY_OUT_HDMI:
				if (i_p_sel->format_id < disp_out.nb_hdmi_format){
					printk(KERN_INFO "set_panel_output : HDMI [%d]\n",i_p_sel->format_id);
					disp_out.select.interface_id = i_p_sel->interface_id; 
					disp_out.select.format_id = i_p_sel->format_id;
					memcpy(&fb_var ,&(disp_out.hdmi_panel_var[i_p_sel->format_id]),sizeof(struct fb_var_screeninfo));
					omap_init_lcd_interface(&fb_var,0,0,1,1);
					mdelay(1);
					omap2_disp_get_dss();
					
					omap2_disp_set_output_dev(OMAP2_VIDEO1, OMAP2_OUTPUT_LCD);
					omap2_disp_set_output_dev(OMAP2_VIDEO2, OMAP2_OUTPUT_LCD);
					omap2_disp_set_output_dev(OMAP2_GRAPHICS, OMAP2_OUTPUT_LCD); /* last, set GO bit */
					
					omap2_disp_put_dss();
				} else {
					printk(KERN_INFO "set_panel_output : HDMI [%d] error, format not supported.\n",i_p_sel->format_id);
				}
				break;
			case DISPLAY_OUT_TV_CRADLE:
				if (i_p_sel->format_id < disp_out.nb_ext_tvout_format){

					printk(KERN_INFO "set_panel_output : EXT VIDEO ENC [%d]\n",i_p_sel->format_id);
					disp_out.select.interface_id = i_p_sel->interface_id; 
					disp_out.select.format_id = i_p_sel->format_id;

					if ( (disp_out.ext_videoenc_panel_ops)->vidout_select_format ) {
						printk("function add: %x\n",
								(unsigned int)(disp_out.ext_videoenc_panel_ops->vidout_select_format));
						(disp_out.ext_videoenc_panel_ops)->vidout_select_format(i_p_sel->format_id);
					} else
						printk("cannot set format\n");
						
					if ( disp_out.ext_videoenc_panel_ops->vidout_select_analog_out )
						(disp_out.ext_videoenc_panel_ops)->vidout_select_analog_out(i_p_sel->interface_id & ANALOG_OUT_MASK);
					else
						printk("cannot set analog out\n");
					
					printk(KERN_INFO "set_panel_output: analog out set\n");

					memcpy(&fb_var, &(disp_out.ext_tvout_panel_var[i_p_sel->format_id]),
						sizeof(struct fb_var_screeninfo));

					omap_init_lcd_interface(&fb_var,0,0,1,1);
					
					omap2_disp_get_dss();
					omap2_disp_set_output_dev(OMAP2_VIDEO1, OMAP2_OUTPUT_LCD);
					omap2_disp_set_output_dev(OMAP2_VIDEO2, OMAP2_OUTPUT_LCD);
					omap2_disp_set_output_dev(OMAP2_GRAPHICS, OMAP2_OUTPUT_LCD); /* last, set GO bit */

					/* setup irq on line number */
					/* this enable also the irq */

					if ( !(disp_out.ext_videoenc_panel_ops->isr_state & VIDEO_EXT_ENC_ISR_REGISTERED) ) {
						disp_out.ext_videoenc_panel_ops->isr_state |=
						( VIDEO_EXT_ENC_ISR_ON | VIDEO_EXT_ENC_ISR_REGISTERED);

						omap2_disp_register_isr((omap2_disp_isr_t) disp_out.ext_videoenc_panel_ops->isr_handler, &(disp_out.ext_videoenc_panel_ops->isr_state),
						DISPC_IRQSTATUS_PROGRAMMEDLINENUMBER);
	
	
						/* irq on line 25 */
						omap2_disp_set_irq_on_linenumber(25);
					}

					if ( !(disp_out.ext_videoenc_panel_ops->isr_state & VIDEO_EXT_ENC_ISR_ON) ) {
						omap2_disp_irqenable((omap2_disp_isr_t) disp_out.ext_videoenc_panel_ops->isr_handler,DISPC_IRQSTATUS_PROGRAMMEDLINENUMBER);
					}
					omap2_disp_put_dss();

				} else {
					printk(KERN_INFO "set_panel_output : EXTERNAL VIDEO ENC [%d] error, format not supported.\n",i_p_sel->format_id);
				}

				break;
				
			default:
				printk(KERN_INFO "set_panel_output : display_out_id %d error, not supported.\n",display_out_id);
				ret = -EINVAL;

			case DISPLAY_OUT_DEFAULT:
				disp_out.select.interface_id = DISPLAY_OUT_DEFAULT; 
				disp_out.select.format_id = 0;
				
				/* disable all the outputs */
				omap_g6_dispout_digital_power_tv(TV_OFF);
				omap_g6_dispout_lcd_stop();
				omap2_disp_get_dss();
				omap2_disp_disable_output_dev(OMAP2_OUTPUT_TV);
				omap2_disp_put_dss();
				break;
		}

		/* tell the cpld to switch between lcd and cradle */
		display_select( display_out_id );
	} else {
		printk(KERN_INFO "set_panel_output : display_out_id %d > DISPLAY_OUT_LAST = %d error\n",display_out_id,DISPLAY_OUT_LAST);
		ret = -1;
	}

	return ret;
}

EXPORT_SYMBOL(get_panel_default_var);
EXPORT_SYMBOL(get_panel_var);
EXPORT_SYMBOL(get_panel_pixclock_max);
EXPORT_SYMBOL(get_panel_pixclock_min);
EXPORT_SYMBOL(set_panel_output);
