/* drivers/video/omap/omap_g6_disp_out_digital.c */
/*----------------------------------------------------------------------------*/
/* Description :  Driver for DIGITAL output on OMAP34xx archos platform       */
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
#include <asm/arch/venc.h>
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

#define DEFAULT_TV_XRES		640
#define DEFAULT_TV_YRES		480


#define DBG if(1)
#define DEBUGMSG DBG printk

#define DEFAULT_ANALOGTV_RESOLUTION_INDEX	RESOLUTION_NTSC_M

#define OMAP2_TV_DRIVER		"omap_atv"
#define OMAP24xx_TV_DEVICE	"omap_atv"


/* ******** Definitions/Types *********************************************** */

/* ******** Definitions/Variables ******************************************* */
extern struct display_out disp_out;

static struct fb_var_screeninfo default_ntsc_panel_var = {
	.xres		= NTSC_DISP_XRES,
	.yres		= NTSC_DISP_YRES,
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

static struct fb_var_screeninfo default_pal_panel_var = {
	.xres		= PAL_DISP_XRES,
	.yres		= PAL_DISP_YRES,
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

/* ******** Definitions/Functions ******************************************* */
void omap_init_digital_go(void);
int omap_init_digital_interface(const int analog_output,const int format_id);
static void tvout_off(struct work_struct *work);
static void tvout_on(struct work_struct *work);

DECLARE_WORK(tvout_on_work, tvout_on);
DECLARE_WORK(tvout_off_work, tvout_off);

/*
 * DSS & VENC register I/O routines
 */
static __inline__ u32 dss_reg_in(u32 offset)
{
	return  omap_readl(DSS_REG_BASE + DSS_REG_OFFSET + offset);
}
static __inline__ u32 dss_reg_out(u32 offset, u32 val)
{
	omap_writel(val,DSS_REG_BASE + DSS_REG_OFFSET + offset);
	return val;
}
static __inline__ u32 venc_reg_in(u32 offset)
{
	return omap_readl(DSS_REG_BASE + VENC_REG_OFFSET + offset);
}
static __inline__ u32 venc_reg_out(u32 offset, u32 val)
{
	omap_writel(val, DSS_REG_BASE + VENC_REG_OFFSET + offset);
	return val;
}


void omap_g6_dispout_digital_power_tv(int level)
{
	switch(level) {
		case TV_OFF:
			if(!in_interrupt())
				tvout_off(NULL);
			else
				schedule_work(&tvout_off_work);
			break;
		default:
			if(!in_interrupt())
				tvout_on(NULL);
			else
				schedule_work(&tvout_on_work);
			break;
	}
}

static void tvout_off(struct work_struct *work)
{
	omap2_disp_get_all_clks();
	// put dac output off 
	venc_reg_out( VENC_DAC_OUTPUT, VENC_DAC_VIDEO_INVERT );
	omap2_disp_set_tvref(TVREF_OFF);
	omap2_disp_put_all_clks();
}

static void tvout_on(struct work_struct *work)
{
	omap2_disp_get_all_clks();
	omap2_disp_set_tvref(TVREF_ON);
	omap2_disp_put_all_clks();
}

/*============================================================================*/
/**
 \brief register tv out panel infos & apply the configuration
 \param i_p_fb_var input pointer to a list of framebuffer screeninfo var
 \param i_nb_resolutions number of framebuffer screeninfo var (each represent a specific resolution)
 \return 0 if ok, else -1
 */
/*============================================================================*/
int display_out_init_tvinfo( struct fb_var_screeninfo *i_p_fb_var, u32 i_nb_resolutions /*, struct tv_fops * fops*/ )
{
	int ret = 0;
	int i = 0;
// 	int tvstd = PAL_BDGHI + DEFAULT_ANALOGTV_RESOLUTION_INDEX;
// 	struct fb_var_screeninfo * default_var = (struct fb_var_screeninfo *)&(i_p_fb_var[DEFAULT_ANALOGTV_RESOLUTION_INDEX]);
	DEBUGMSG("init internal TV OUT module infos \n");
	disp_out.nb_int_tvout_format = 0;
	disp_out.select.interface_id = DISPLAY_OUT_TV | ANALOG_OUT_SVIDEO;
	disp_out.select.format_id = 0;
	for(i=0 ; i<i_nb_resolutions ; i++) {
		memcpy(&(disp_out.int_tvout_panel_var[i]), &(i_p_fb_var[i]),sizeof( struct fb_var_screeninfo));
		DEBUGMSG("add internal tv out format[%d] %dx%d,  pixel_clock@%ld kHz\n",
		       i,
		       disp_out.int_tvout_panel_var[i].xres,
		       disp_out.int_tvout_panel_var[i].yres,
		       PICOS2KHZ(disp_out.int_tvout_panel_var[i].pixclock));
	}
// 	disp_out.tv_panel_ops = fops;

	if(i_nb_resolutions > 0){
		disp_out.nb_int_tvout_format = i_nb_resolutions;
#ifndef CONFIG_FB_OMAP_BOOTLOADER_INIT
		omap_init_digital_interface(ANALOG_OUT_SVIDEO, DEFAULT_ANALOGTV_RESOLUTION_INDEX);
		omap_init_digital_go();
#endif
	} else {
		DEBUGMSG(KERN_ERR "display_out_init_tvinfo : no resolution in conf :-( \n");
		ret = -1;
	}
	return 0;
}
EXPORT_SYMBOL( display_out_init_tvinfo );



int display_out_release_tv( void )
{
DEBUGMSG("release TV info and fops\n");
	memset(disp_out.int_tvout_panel_var,0,sizeof(struct fb_var_screeninfo)*NB_MAX_RESOLUTION_PER_PANEL);
// 	disp_out.tv_panel_var = NULL;
// 	disp_out.tv_panel_ops = NULL;
	return 0;
}
EXPORT_SYMBOL( display_out_release_tv );



static int __init tv_probe(struct platform_device *odev)
{
	int ret = 0;
	DEBUGMSG(KERN_INFO "tv_probe\n\n");
	disp_out.select.interface_id = DISPLAY_OUT_TV | ANALOG_OUT_SVIDEO;
	disp_out.select.format_id = 0;
#ifndef CONFIG_FB_OMAP_BOOTLOADER_INIT
	omap2_disp_get_all_clks();
	omap_g6_dispout_digital_power_tv(TV_ON);
	omap2_disp_set_tvstandard(NTSC_M);
	omap2_disp_set_panel_size(OMAP2_OUTPUT_TV, 720, 480);
	omap2_disp_enable_output_dev(OMAP2_OUTPUT_TV);
	omap2_disp_put_all_clks();
#endif
	return ret;
}

static int tv_exit(void)
{
	omap2_disp_get_all_clks();
	omap2_disp_disable_output_dev(OMAP2_OUTPUT_TV);
	omap_g6_dispout_digital_power_tv(TV_OFF);
	omap2_disp_put_all_clks();
	return 0;
}

static struct platform_driver omap2_tv_driver = {
	.driver = {
		.name   = OMAP2_TV_DRIVER,
	},
	.probe          = tv_probe,
};

static struct platform_device tv_device = {
	.name     = OMAP24xx_TV_DEVICE,
	.id    = 10,
};

/*============================================================================*/
/**
 \brief activate lcd output
 */
/*============================================================================*/
void omap_init_digital_go( void)
{
	omap2_disp_get_all_clks();
	omap2_disp_enable_output_dev( OMAP2_OUTPUT_TV );
	omap2_disp_put_all_clks();
}

/*============================================================================*/
/**
 \brief activate the digital interface according to a panel select struct.
 \param analog_output ANALOG_OUT_DEFAULT, ANALOG_OUT_CVBS, ANALOG_OUT_SVIDEO, ANALOG_OUT_RGB
 \param format_id  tv out resolution id
 \return 0 if ok, else -1
 */
/*============================================================================*/
int omap_init_digital_interface(const int analog_output,const int format_id)
{
	int tvstd = PAL_BDGHI;
	int ret = 0;
	u32 dss_control;
	u32 venc_output_ctrl = VENC_DAC_LUMA_ENABLE | VENC_DAC_CHROMA_ENABLE | VENC_DAC_VIDEO_INVERT;
	
	
	DEBUGMSG("omap_init_digital_interface format %d\n",format_id );
	omap2_disp_get_all_clks();
	/*configuring ARGB32 */
	omap2_disp_set_colorkey(OMAP2_OUTPUT_TV,OMAP2_VIDEO_SOURCE,0xFE00FE); /*set an unusefull value for transparency color key */
	omap2_disp_disable_colorkey(OMAP2_OUTPUT_TV);/* disable color key */
	omap2_disp_set_alphablend(OMAP2_OUTPUT_TV,1);/* enable alpha blender */
	omap2_disp_set_global_alphablend_value(OMAP2_GRAPHICS,255);
	omap2_disp_set_global_alphablend_value(OMAP2_VIDEO2,255);
	
	dss_control = dss_reg_in(DSS_CONTROL);
	switch(analog_output){
		case ANALOG_OUT_CVBS:
			DEBUGMSG(KERN_INFO "ANALOG_OUT_CVBS(%d)\n\n",analog_output);
			/*set to composite */
			dss_control &= ~DSS_CONTROL_VENC_OUT;
			/*VENC_OUTPUT_CONTROL : just enable composite*/
			venc_output_ctrl = VENC_DAC_COMPOSITE_ENABLE | VENC_DAC_VIDEO_INVERT;
			break;
		case ANALOG_OUT_SVIDEO:
			DEBUGMSG(KERN_INFO "ANALOG_OUT_SVIDEO(%d)\n\n",analog_output);
			dss_control = dss_reg_in(DSS_CONTROL);
			/*set to svideo */
			dss_control |= DSS_CONTROL_VENC_OUT;
			/*VENC_OUTPUT_CONTROL : enable chroma and luma */
			venc_output_ctrl = VENC_DAC_LUMA_ENABLE | VENC_DAC_CHROMA_ENABLE | VENC_DAC_VIDEO_INVERT;
			break;
		case ANALOG_OUT_RGB:
			DEBUGMSG(KERN_INFO "ANALOG_OUT_RGB(%d)\n\n",analog_output);
			/* TODO */
			break;	
		default:
			DEBUGMSG(KERN_INFO "ANALOG_OUT_DEFAULT(%d)\n\n",analog_output);
			break;
	}
	dss_reg_out(DSS_CONTROL, dss_control);
	omap_g6_dispout_digital_power_tv(TV_ON);
	
	tvstd = PAL_BDGHI + format_id;
	omap2_disp_set_tvstandard(tvstd);
	venc_reg_out(VENC_DAC_OUTPUT, venc_output_ctrl);
	venc_reg_out(VENC_DAC_TST, venc_reg_in(VENC_DAC_TST));
	
	omap2_disp_set_panel_size(OMAP2_OUTPUT_TV, 
				  disp_out.int_tvout_panel_var[format_id].xres, 
				  disp_out.int_tvout_panel_var[format_id].yres);
	
	//omap2_disp_put_all_clks();

	return ret;
}

/*============================================================================*/
/**
 \brief initialise display out digital interface
 \return 0 if ok, else -1
 */
/*============================================================================*/
int omap_g6_dispout_digital_init(void)
{

	int ret = 0;
	DEBUGMSG( "\n\n omap_g6_dispout_digital_init\n");

	
	/* Register the driver with LDM */
	if (platform_driver_register(&omap2_tv_driver)) {
		DEBUGMSG(KERN_ERR ": failed to register omap2_tvdriver\n");
		return -ENODEV;
	}
	
	/* Register the device with LDM */
	if (platform_device_register(&tv_device)) {
		DEBUGMSG(KERN_ERR ": failed to register tv device\n");
		platform_driver_unregister(&omap2_tv_driver);
		return -ENODEV;
	}
	
	return ret;
}


/*============================================================================*/
/**
 \brief release display out digital interface
 \return 0 if ok, else -1
 */
/*============================================================================*/
void omap_g6_dispout_digital_exit(void)
{
	tv_exit();
	platform_device_unregister(&tv_device);
	platform_driver_unregister(&omap2_tv_driver);
}


/*============================================================================*/
/**
 \brief return the default digital variable screeninfo structure
 \param var pointer to framebuffer variable screeninfo structure.
 */
/*============================================================================*/
void omap_g6_dispout_digital_get_panel_default_var(struct fb_var_screeninfo *var)
{
	int tv = omap2_disp_get_tvstandard();
	
	DEBUGMSG(" get digital panel default info var\n");

	if(tv == PAL_BDGHI ||
		tv == PAL_NC    ||
		tv == PAL_N     ||
		tv == PAL_M     ||
		tv == PAL_60){
		memcpy((struct fb_var_screeninfo *) var,
			&default_pal_panel_var, sizeof(*var));
	}else {
		 memcpy((struct fb_var_screeninfo *) var,
			 &default_ntsc_panel_var, sizeof(*var));
	}
}

