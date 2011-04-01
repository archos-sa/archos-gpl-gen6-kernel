/* drivers/video/omap/omap_g6_disp_out_ioctl.c */
/*----------------------------------------------------------------------------*/
/* Description :  ioctl interface for LCD on OMAP34xx archos platform         */
/*                                                                            */ /*----------------------------------------------------------------------------*/

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
/*          Functions                        Scope (API, INTERNAL , LOCAL)
            
*/
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
#include <asm/hardware.h>
#include <asm/uaccess.h>
#include <asm/delay.h>
#include <asm/arch/display.h>
#include <asm/arch/gpio.h>
#include <asm/arch/clock.h>

#include <linux/display_out.h>
#include <linux/miscdevice.h>
#include "omap_fb.h"

/*----------------------------------------------------------------------------*/
/*          Definitions                                                       */
/*----------------------------------------------------------------------------*/
/* ******** Definitions/Consts ********************************************** */

//#define DEBUGMSG printk
#define DEBUGMSG(...) do {} while (0)

/* ******** Definitions/Types *********************************************** */

/* ******** Definitions/Variables ******************************************* */
extern struct display_out disp_out;
static int display_enable = DISP_LCD;
static unsigned char display_open_status;

/* ******** Definitions/Functions ******************************************* */
static int disp_lcd_getid( lcd_disp_t *param )
{
	int ret = 0;

	DEBUGMSG(" lcd get id \n" );
	return ret;
}

static int disp_lcd_getreg( lcd_disp_t *param )
{
	int ret = -ENODEV;

	if ( disp_out.lcd_panel_ops->get_reg ) {
		ret = disp_out.lcd_panel_ops->get_reg( param->reg, &param->val );
		DEBUGMSG("read reg: %x value: %x \n", param->reg, param->val );

	}
	return ret;
}

static int disp_lcd_setreg( lcd_disp_t *param )
{
	int ret = -ENODEV;

	if ( disp_out.lcd_panel_ops->set_reg ) {
		DEBUGMSG("set value: %x \n", param->val);
		ret = disp_out.lcd_panel_ops->set_reg( param->reg, param->val );
	}
	return ret;
}

static int disp_lcd_setconf( lcd_disp_t *param )
{
	int ret = -ENODEV;

	if ( disp_out.lcd_panel_ops->set_conf ) {
		DEBUGMSG("set conf: %x \n", param->par1);
		ret = disp_out.lcd_panel_ops->set_conf( param->par1 );
	}
	return ret;

}

static int disp_lcd_setbrightness( lcd_disp_t *param )
{
	int ret = -ENODEV;

	if ( disp_out.lcd_panel_ops->set_brightness ) {
		DEBUGMSG("set bright: %x \n", param->par1);
		ret = disp_out.lcd_panel_ops->set_brightness( param->par1 );
	}
	return ret;

}

static int disp_lcd_setgamma( lcd_disp_t *param )
{
	int ret = -ENODEV;

	if ( disp_out.lcd_panel_ops->set_gamma ) {
		DEBUGMSG("set gamma: %x \n", param->par1);
		ret = disp_out.lcd_panel_ops->set_gamma( param->par1 );
	}
	return ret;

}

static int disp_lcd_setcontrast( lcd_disp_t *param )
{
	int ret = -ENODEV;

	if ( disp_out.lcd_panel_ops->set_contrast ) {
		DEBUGMSG("set contrast: %x \n", param->par1);
		ret = disp_out.lcd_panel_ops->set_contrast( param->par1 );
	}
	return ret;

}

static int disp_lcd_setvcom( lcd_disp_t *param )
{
	int ret = -ENODEV;

	if ( disp_out.lcd_panel_ops->set_vcom ) {
		DEBUGMSG("set conf: %x \n", param->par1);
		ret = disp_out.lcd_panel_ops->set_vcom( param->par1 );
	}
	return ret;

}

static int disp_lcd_enable( lcd_disp_t *param )
{
	int ret = -ENODEV;
	
	if ( disp_out.lcd_panel_ops->enable_display ) {

		DEBUGMSG("lcd enable: %x \n", param->par1);
		ret = disp_out.lcd_panel_ops->enable_display( param->par1 );

		if ( disp_out.lcd_panel_ops->set_conf && param->par1 ) {
			DEBUGMSG("set conf \n" );
			// default conf
			ret = disp_out.lcd_panel_ops->set_conf( 3 );
		}
	}

	return ret;

}


static int disp_lcd_backlight( lcd_disp_t *param )
{
	int ret = -ENODEV;
	
	if ( disp_out.lcd_panel_ops->enable_backlight ) {
		DEBUGMSG("bckl enable: %x \n", param->par1);
		ret = disp_out.lcd_panel_ops->enable_backlight( param->par1 );
	}

	return ret;

}

static int disp_backlight_level( lcd_disp_t *param )
{
	int ret = -ENODEV;
	
	if ( disp_out.lcd_panel_ops->backlight_level ) {
		DEBUGMSG("bckl level: %x \n", param->par1);
		ret = disp_out.lcd_panel_ops->backlight_level( param->par1 );
	}

	return ret;

}

void enable_backlight(void)
{
#if 0
	lcd_disp_t param = { .par1 = 1 };
	disp_lcd_backlight(&param);
#endif
}
EXPORT_SYMBOL(enable_backlight);

void disable_backlight(void)
{
#if 0
	lcd_disp_t param = { .par1 = 0 };
	disp_lcd_backlight(&param);
#endif
}
EXPORT_SYMBOL(disable_backlight);

static int disp_lcd_setsync( lcd_disp_t *param )
{
	return -EINVAL;

}

static int disp_out_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = -EFAULT;
	void __user *argp = (void __user *)arg;
	lcd_disp_t param;
	
	if ( display_enable != DISP_LCD || disp_out.lcd_panel_ops == NULL )
		return ret;

	switch (cmd) {

	case DISP_GET_ID:
		DEBUGMSG( " : disp_getid\n" );
		if (!copy_from_user(&param, argp, sizeof(param)))
			return disp_lcd_getid( &param );
		break;
	case DISP_SET_CONTRAST:
		DEBUGMSG( " : disp_setcontrast\n" );
		if (!copy_from_user(&param, argp, sizeof(param)))
			return disp_lcd_setcontrast( &param );
		break;

	case DISP_SET_BRIGHTNESS:
		DEBUGMSG( " : disp_setbrightness\n" );
		if (!copy_from_user(&param, argp, sizeof(param)))
			return disp_lcd_setbrightness( &param );
		break;

	case DISP_SET_GAMMA:
		DEBUGMSG( " : disp_setgamma\n" );
		if (!copy_from_user(&param, argp, sizeof(param)))
			return disp_lcd_setgamma( &param );
		break;

	case DISP_GET_REG:
		DEBUGMSG( " : disp_getreg\n" );
		if (!copy_from_user(&param, argp, sizeof(param))) {
			ret=disp_lcd_getreg( &param );
			if (copy_to_user(argp, &param, sizeof(param)))
				return -EFAULT;
		}
		break;

	case DISP_SET_REG:
		DEBUGMSG( " : disp_setreg\n" );
		if (!copy_from_user(&param, argp, sizeof(param)))
			return disp_lcd_setreg( &param );
		break;

	case DISP_SET_SYNC:
		DEBUGMSG( " : disp_setsync\n" );
		if (!copy_from_user(&param, argp, sizeof(param)))
			return disp_lcd_setsync( &param );
		break;

	case DISP_SET_CONFIG:
		DEBUGMSG( " : disp_setconf\n" );
		if (!copy_from_user(&param, argp, sizeof(param)))
			return disp_lcd_setconf( &param );
		break;

	case DISP_SET_VCOM:
		DEBUGMSG( " : disp_setvcom\n" );
		if (!copy_from_user(&param, argp, sizeof(param)))
			return disp_lcd_setvcom( &param );
		break;

	case DISP_ENABLE_BCKL:
		DEBUGMSG( " : backlight_enable \n" );
		if (!copy_from_user(&param, argp, sizeof(param)))
			return disp_lcd_backlight( &param );
		break;

	case DISP_ENABLE_DISP:
		DEBUGMSG( " : disp_enable \n" );
		if (!copy_from_user(&param, argp, sizeof(param)))
			return disp_lcd_enable( &param );
		break;

	case DISP_SET_BCKL_LEV:
		DEBUGMSG( " : backlight level \n" );
		if (!copy_from_user(&param, argp, sizeof(param)))
			return disp_backlight_level( &param );
		break;

	default:
		ret = -ENOIOCTLCMD;

	}

	return ret;
}

static int disp_out_open( struct inode *inode, struct file *file )
{
	if ( display_open_status & 1 )
		return -EBUSY;

	display_open_status |= 1;

	return 0;
}

static int disp_out_release( struct inode *inode, struct file *file )
{
	display_open_status = 0;
	return 0;
}

static struct file_operations disp_out_fops = {
	.owner		= THIS_MODULE,
	.ioctl          = disp_out_ioctl,
	.open           = disp_out_open,
	.release        = disp_out_release,
};

static struct miscdevice disp_out_misc = {
        .minor  = DISP_OUT_MINOR,
        .name   = "disp",
        .fops	= &disp_out_fops,
};

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
int  omap_g6_dispout_ioctl_init(void)
{
	int ret;

	printk(KERN_DEBUG "omap_g6_dispout_ioctl_init\n");

	ret = misc_register( &disp_out_misc );
	if (ret) {
		printk(KERN_ERR "cannot register miscdev on minor=%d (err=%d)\n",
			DISP_OUT_MINOR, ret);
		return ret;
	}
	return 0;
}


void omap_g6_dispout_ioctl_exit(void)
{
	misc_deregister( &disp_out_misc );
}
