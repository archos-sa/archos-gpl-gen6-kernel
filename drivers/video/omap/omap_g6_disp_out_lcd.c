/* drivers/video/omap/omap_g6_disp_out_lcd.c */
/*----------------------------------------------------------------------------*/
/* Description :  Driver for LCD output on OMAP34xx archos platform           */
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

#define DBG if(1)
#define DEBUGMSG DBG printk

#define DEFAULT_HDMI_RESOLUTION_INDEX		RESOLUTION_VGA

#define OMAP2_LCD_DRIVER	"omap_lcd"
#define OMAP24xx_LCD_DEVICE	"omap_lcd"


/* ******** Definitions/Types *********************************************** */

/* ******** Definitions/Variables ******************************************* */
extern struct display_out disp_out;

static struct fb_var_screeninfo default_lcd_panel_var = {
	.xres		= LCD_DISP_XRES,
	.yres		= LCD_DISP_YRES,
	.xres_virtual	= DISPLAY_XRES_MAX,
	.yres_virtual	= DISPLAY_YRES_MAX*NB_FRAME_BUFFERS,
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
	.upper_margin	= 33,		/* line clocks */
	.lower_margin	= 10,		/* line clocks */
	.hsync_len	= 64,		/* horizontal sync pulse width (96) FIXME : reg is 5 bit width (max = 64) but NOTE 96 - 32 our cpld will by default add 32 pixclock here */
	.vsync_len	= 2,		/* line clocks */
	.sync		= 1,
	.vmode		= FB_VMODE_NONINTERLACED,
	.rotate		= 0,
	.reserved[0]	= 0,
};

/* ******** Definitions/Functions ******************************************* */
extern int omap24xxfb_set_output_layer(int layer);
/*---------------------------------------------------------------------------*/
//        API functions for omap_g6_disp_out driver
/*---------------------------------------------------------------------------*/
void omap_init_lcd_go( void);
int omap_init_lcd_interface( struct fb_var_screeninfo * var,int acb, int ipc, int onoff, int rf );
int omap_g6_dispout_lcd_init(void);
void omap_g6_dispout_lcd_exit(void);
void omap_g6_dispout_lcd_get_panel_default_var(struct fb_var_screeninfo *var);

/*---------------------------------------------------------------------------*/
//        Static functions
/*---------------------------------------------------------------------------*/
/*============================================================================*/
/**
 \brief round a kHz value
 \param val value to round
 \return rounded value
 */
/*============================================================================*/
static u32 _my_round_clkdiv(u32 val)
{
	u32 a,b,c;
	a = val / 1000;
	b = val - (a * 1000);
	if(b<500){
		c = a;
	} else {
		c = a + 1;
	}
	return c;
}

/*============================================================================*/
/**
 \brief probe function for lcd platform device driver
 \param odev pointer to platform_device structure
 \return 0 if ok, else -1
 */
/*============================================================================*/
static int __init lcd_probe(struct platform_device *odev)
{
	int ret = 0;
	disp_out.select.interface_id = DISPLAY_OUT_LCD;
	disp_out.select.format_id = 0;
	disp_out.lcd_panel_var = NULL;
#ifndef CONFIG_FB_OMAP_BOOTLOADER_INIT
	ret = omap_init_lcd_interface( &default_lcd_panel_var, 0, 0, 1,1 );
	omap_init_lcd_go();
#endif
	return ret;
}

/*============================================================================*/
/**
 \brief release function for lcd platform device driver
 */
/*============================================================================*/
int omap_g6_dispout_lcd_stop(void)
{
	omap2_disp_get_dss();
#if 1
	// set hsync and vsync to 0
	omap2_disp_lcdcfg_polfreq(	0,   // horizontal sync active low
					0,   // vertical sync active low
					0,    // ACB
					0,    // IPC
					1,   // ONOFF
					1 //raising/falling edge
				);
#endif
	omap2_disp_disable_output_dev(OMAP2_OUTPUT_LCD);
	omap2_disp_put_dss();
	return 0;
}

static struct platform_driver omap2_lcd_driver = {
	.driver = {
		.name   = OMAP2_LCD_DRIVER,
	},
	.probe          = lcd_probe,
};

static struct platform_device lcd_device = {
	.name     = OMAP24xx_LCD_DEVICE,
	.id    = 9,
};

/*---------------------------------------------------------------------------*/
//        API functions for omag6_dispout driver
/*---------------------------------------------------------------------------*/
/*============================================================================*/
/**
 \brief activate lcd output
 */
/*============================================================================*/
void omap_init_lcd_go( void)
{
	omap2_disp_get_dss();
#if 1
	// restore hsync and vsync
	omap2_disp_lcdcfg_polfreq(	1,   // horizontal sync active low
					1,   // vertical sync active low
					0,    // ACB
					0,    // IPC
					1,   // ONOFF
					1 //raising/falling edge
				);
#endif
	omap2_disp_enable_output_dev( OMAP2_OUTPUT_LCD );
	omap2_disp_put_dss();
}

/*============================================================================*/
/**
 \brief configure the lcd output interface : pixclock, sync, dataline depth, line lenght, etc...
 \param var pointer to framebuffer variable screeninfo structure.
 \param acb
 \param ipc 
 \param onoff
 \param rf
 \return 0
 */
/*============================================================================*/
int omap_init_lcd_interface( struct fb_var_screeninfo * var,int acb, int ipc, int onoff, int rf )
{
	u32 clkdiv = 0;
	s32 pixclk = 1;
	
	printk(KERN_DEBUG "omap g6, init lcd interface\n");
	
	omap2_disp_set_clkrate_g6(
		omap2_disp_get_clkrate_g6_from_pixclock(PICOS2KHZ(var->pixclock)) );
	
	omap2_disp_get_dss();
	
	/* configuring rgba 32 */
// 	printk(KERN_DEBUG "omap g6, configure ARGB32 \n");
	omap2_disp_set_colorkey(OMAP2_OUTPUT_LCD,OMAP2_VIDEO_SOURCE,0xFE00FE); /*set an unusefull value for transparency color key */
	omap2_disp_disable_colorkey(OMAP2_OUTPUT_LCD); /* disable color key */
	omap2_disp_set_alphablend(OMAP2_OUTPUT_LCD,1); /* enable alpha blender */
	omap2_disp_set_global_alphablend_value(OMAP2_GRAPHICS,255);
	omap2_disp_set_global_alphablend_value(OMAP2_VIDEO2,255);

	omap2_disp_set_panel_size( OMAP2_OUTPUT_LCD, var->xres, var->yres);

	pixclk = PICOS2KHZ(var->pixclock)/*kHz*/;
	/*make sure pixclk>0 */
	if(pixclk<=0){
		printk("\n\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");	
		printk("omap g6, error, pixclk=%d set to 27000 to avoid problems\n",pixclk);	
		printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n\n");
		pixclk = 27000;	
	}
	clkdiv = (omap2_disp_get_dssfclk()/*Hz*/ ) / pixclk ;
	clkdiv = _my_round_clkdiv(clkdiv);
	
	printk("omap g6, clkdiv=%d,pixclock=%dkHz,dss1clock=%d\n",
		(int)clkdiv, (int)PICOS2KHZ(var->pixclock), (int)omap2_disp_get_dssfclk());

	omap2_disp_config_lcd(clkdiv,
			      var->left_margin-1,	// hbp
			      var->right_margin-1,	// hfp
			      var->hsync_len-1,		// hsw
			      var->upper_margin,	// vbp
			      var->lower_margin,	// vfp
			      var->vsync_len -1		// vsw
			      );

	omap2_disp_lcdcfg_polfreq(	var->sync,   // horizontal sync active low
					var->sync,   // vertical sync active low
					acb,    // ACB
					ipc,    // IPC
					onoff,   // ONOFF
					rf //raising/falling edge
				);

	// set the display physical datalines
	if ( disp_out.lcd_panel_var != NULL )
		omap2_disp_set_lcddatalines( disp_out.lcd_panel_var->data_lines );
	else
		omap2_disp_set_lcddatalines( LCD_DATA_LINE_24BIT );	// default value for G6

	//MW: no omap2_disp_put_dss();
	return 0;
}

/*============================================================================*/
/**
 \brief initialise display out lcd interface
 \return 0 if ok, else -1
 */
/*============================================================================*/
int omap_g6_dispout_lcd_init(void)
{
	int ret = 0;
	printk( "\n\n omap_g6_dispout_lcd_init\n");

	/* Register the driver with LDM */
	if (platform_driver_register(&omap2_lcd_driver)) {
		printk(KERN_ERR  ": failed to register omap2_lcd driver\n");
		return -ENODEV;
	}
	
	/* Register the device with LDM */
	if (platform_device_register(&lcd_device)) {
		printk(KERN_ERR  ": failed to register lcd device\n");
		platform_driver_unregister(&omap2_lcd_driver);
		return -ENODEV;
	}
	return ret;
}

/*============================================================================*/
/**
 \brief release display out lcd interface
 \return 0 if ok, else -1
 */
/*============================================================================*/
void omap_g6_dispout_lcd_exit(void)
{
	omap_g6_dispout_lcd_stop();
	platform_device_unregister(&lcd_device);
	platform_driver_unregister(&omap2_lcd_driver);
}

/*============================================================================*/
/**
 \brief return the default lcd variable screeninfo structure
 \param var pointer to framebuffer variable screeninfo structure.
 */
/*============================================================================*/
void omap_g6_dispout_lcd_get_panel_default_var(struct fb_var_screeninfo *var)
{
	printk(" get lcd panel default info var\n");
		memcpy((struct fb_var_screeninfo *) var,
			&default_lcd_panel_var, sizeof(*var));
}


/*---------------------------------------------------------------------------*/
//        API functions exported in kernel space
/*---------------------------------------------------------------------------*/
/*============================================================================*/
/**
 \brief convert lcd_panel structure to variable screeninfo structure
 \param o_p_fb_var output pointer to framebuffer variable screeninfo structure.
 \param i_p_lcd_var input pointer to lcd_panel structure
 \return 0 if ok, else -1
 */
/*============================================================================*/
int display_out_lcd_panel_to_fb_var(struct fb_var_screeninfo *o_p_fb_var, struct lcd_panel *i_p_lcd_var)
{
	int ret = 0;
	/* init default parameters */
	memcpy( o_p_fb_var, &default_lcd_panel_var, sizeof(struct fb_var_screeninfo ));
	/* convert specific parameters */
	if(i_p_lcd_var != NULL){
		o_p_fb_var->xres	 	= i_p_lcd_var->x_res;
		o_p_fb_var->yres	 	= i_p_lcd_var->y_res;
		o_p_fb_var->xres_virtual 	= i_p_lcd_var->x_res;
		o_p_fb_var->yres_virtual 	= i_p_lcd_var->y_res;
		o_p_fb_var->left_margin		= i_p_lcd_var->hfp + 1;	/* pixclocks */
		o_p_fb_var->right_margin	= i_p_lcd_var->hbp + 1;	/* pixclocks */
		o_p_fb_var->upper_margin	= i_p_lcd_var->vfp;	/* line clocks */
		o_p_fb_var->lower_margin	= i_p_lcd_var->vbp;	/* line clocks */
		o_p_fb_var->hsync_len		= i_p_lcd_var->hsw + 1;	/* pixclocks */
		o_p_fb_var->vsync_len		= i_p_lcd_var->vsw + 1;	/* line clocks */
		// 		o_p_fb_var->bits_per_pixel 	= i_p_lcd_var->bpp; /*NOTE it does not match bpp is the physical bus width and  bits_per_pixel the color depth */
		o_p_fb_var->pixclock		= i_p_lcd_var->pixel_clock;
	} else {
		ret = - 1;
	}
	return ret;
}

/*============================================================================*/
/**
 \brief register lcd panel info & apply the configuration
 \param i_p_lcd_var input pointer to lcd_panel structure
 \param i_p_lcd_var input pointer to lcd_fops structure
 \return 0
 */
/*============================================================================*/
int display_out_init_lcdinfo( struct lcd_panel *i_p_lcd_var, struct lcd_fops * i_p_lcd_fops )
{
	printk("init LCD module info and fops\n");
	disp_out.select.interface_id = DISPLAY_OUT_LCD;
	disp_out.select.format_id = 0;
	if(i_p_lcd_var != NULL){
		disp_out.lcd_panel_var = (struct lcd_panel *)i_p_lcd_var;
		if(i_p_lcd_fops != NULL){
			disp_out.lcd_panel_ops = (struct lcd_fops *)i_p_lcd_fops;
		}	
		display_out_lcd_panel_to_fb_var(&default_lcd_panel_var,i_p_lcd_var);		
#ifndef CONFIG_FB_OMAP_BOOTLOADER_INIT
		omap_init_lcd_interface((struct fb_var_screeninfo *)&default_lcd_panel_var,0,0,1,1);
		omap_init_lcd_go();
#endif
	}
	return 0;
}
EXPORT_SYMBOL( display_out_init_lcdinfo );

/*============================================================================*/
/**
 \brief register hdmi panel infos & apply the configuration
 \param i_p_fb_var input pointer to a list of framebuffer screeninfo var
 \param i_nb_resolutions number of framebuffer screeninfo var (each represent a specific resolution)
 \return 0 if ok, else -1
 */
/*============================================================================*/
int display_out_init_hdmiinfo( struct fb_var_screeninfo *i_p_fb_var, u32 i_nb_resolutions )
{
	int ret = 0;
	int i = 0;
	struct fb_var_screeninfo * default_var = (struct fb_var_screeninfo *)&(i_p_fb_var[DEFAULT_HDMI_RESOLUTION_INDEX]);
	printk("init HDMI module infos \n");
	disp_out.nb_hdmi_format = 0;
	disp_out.select.interface_id = DISPLAY_OUT_HDMI;
	disp_out.select.format_id = 0;
	for(i=0 ; i<i_nb_resolutions ; i++) {
		memcpy(&(disp_out.hdmi_panel_var[i]), &(i_p_fb_var[i]),sizeof( struct fb_var_screeninfo));
		printk("add HDMI format[%d] %dx%d,  pixel_clock@%ld kHz\n",
		       i,
		       disp_out.hdmi_panel_var[i].xres,
		       disp_out.hdmi_panel_var[i].yres,
		       PICOS2KHZ(disp_out.hdmi_panel_var[i].pixclock));
	}

	if(i_nb_resolutions > 0){
		disp_out.nb_hdmi_format = i_nb_resolutions;
	} else {
		printk(KERN_ERR "display_out_init_hdmiinfo : no resolution in conf :-( \n");
		ret = -1;
	}
	return (ret);
}
EXPORT_SYMBOL( display_out_init_hdmiinfo );

/*============================================================================*/
/**
 \brief register external video encoder panel infos & apply the configuration
 \param i_p_fb_var input pointer to a list of framebuffer screeninfo var
 \param i_nb_resolutions number of framebuffer screeninfo var (each represent a specific resolution)
 \return 0 if ok, else -1
 */
/*============================================================================*/
int display_out_init_ext_videoenc_info( struct fb_var_screeninfo *i_p_fb_var, u32 i_nb_resolutions,
struct ext_videoenc_fops *fops )
{
	int ret = 0;
	int i = 0;
// 	struct fb_var_screeninfo * default_var = (struct fb_var_screeninfo *)&(i_p_fb_var[0]);
	printk("init external video encoder module infos \n");
	disp_out.nb_ext_tvout_format = 0;
	for(i=0 ; i<i_nb_resolutions ; i++) {
		memcpy(&(disp_out.ext_tvout_panel_var[i]), &(i_p_fb_var[i]),sizeof( struct fb_var_screeninfo));
		printk("add external video encoder format[%d] %dx%d,  pixel_clock@%ld kHz\n",
		       i,
		       disp_out.ext_tvout_panel_var[i].xres,
		       disp_out.ext_tvout_panel_var[i].yres,
		       PICOS2KHZ(disp_out.ext_tvout_panel_var[i].pixclock));
	}

	if(i_nb_resolutions > 0) {
		disp_out.nb_ext_tvout_format = i_nb_resolutions;
	} else {
		printk(KERN_ERR "display_out_init_ext_videoenc_info : no resolution in conf :-( \n");
		ret = -1;
	}

	if ( fops ) {
		disp_out.ext_videoenc_panel_ops=fops;
	}

	return (ret);
}
EXPORT_SYMBOL( display_out_init_ext_videoenc_info );


int display_out_release_lcd( void )
{
	printk("release LCD info and fops\n");

	disp_out.lcd_panel_var = NULL;
	disp_out.lcd_panel_ops = NULL;
	return 0;
}
EXPORT_SYMBOL( display_out_release_lcd );
