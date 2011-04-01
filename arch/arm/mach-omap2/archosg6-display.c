#include <linux/types.h>
#include <linux/sysdev.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/module.h>
#include <asm/arch/gpio.h>
#include <asm/arch/prcm.h>
#include <linux/fb.h>
#include <asm/mach-types.h>
#include <asm/arch/mux.h>
#include <linux/display_out.h>
#include <asm/arch/board.h>

#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
#define _INIT_STATE( a, b ) { }	// don't reset hardware if banner in boot
#else
#define _INIT_STATE( a, b ) omap_set_gpio_dataout( _PIN_NB(a), b );
#endif

static int board_disp_hdmi_i2c_nbr = 3;
static int board_disp_extvideoenc_i2c_nbr = 3;
static struct archosg6_disp_conf disp_gpio;	// local copy!

extern void omap2_disp_gfxformat ( int *format , int read_access);

static struct sysdev_class display_sysclass = {
	set_kset_name("display"),
};

static ssize_t show_gfxformat(struct sysdev_class* cls, char* buf)
{
	int gfx_format;
	omap2_disp_gfxformat(&gfx_format,1);
	return sprintf(buf, "%i\n", gfx_format); 
}

static ssize_t set_gfxformat(struct sysdev_class* cls, const char* buf, size_t len)
{
	int val = simple_strtol(buf, NULL, 10);
	
	if( val > 15 || val < -1 )
		return -EINVAL;

	omap2_disp_gfxformat(&val,0);

	return len;
}

static SYSDEV_CLASS_ATTR(gfxformat, S_IRUGO|S_IWUSR, show_gfxformat, set_gfxformat);

static void _lcd_backlight( int enable )
{
	if ( enable )
		omap_set_gpio_dataout( _PIN_NB( disp_gpio.bkl_pwon ), 1 );
	else
		omap_set_gpio_dataout( _PIN_NB( disp_gpio.bkl_pwon ), 0 );

}

static void _lcd_reset( int enable )
{
	if ( enable )
		omap_set_gpio_dataout( _PIN_NB( disp_gpio.lcd_rst ), 1 );
	else
		omap_set_gpio_dataout( _PIN_NB( disp_gpio.lcd_rst ), 0 );

}

static void _lcd_power( int enable )
{
	if ( enable )
		omap_set_gpio_dataout( _PIN_NB( disp_gpio.lcd_pwon ), 1 );
	else
		omap_set_gpio_dataout( _PIN_NB( disp_gpio.lcd_pwon ), 0 );

}

static void _lcd_pci( int enable )
{
	if ( enable )
		omap_set_gpio_dataout( _PIN_NB( disp_gpio.lcd_pci ), 1 );
	else
		omap_set_gpio_dataout( _PIN_NB( disp_gpio.lcd_pci ), 0 );
	
}

static struct lcd_io_fops lcd_io = {
	.lcd_power = _lcd_power,
	.lcd_reset = _lcd_reset,
	.backlight_power = _lcd_backlight,
	.lcd_pci = 0,
};

static struct lcd_io_fops lcd_g6l_io = {
	.lcd_power = _lcd_power,
	.lcd_reset = _lcd_reset,
	.backlight_power = 0,
	.lcd_pci = _lcd_pci,
};

struct lcd_io_fops *archosg6_lcd_get_io(void) {

	if ( machine_is_archos_g6l() )	
		return &lcd_g6l_io;
	else
		return &lcd_io;
} 

EXPORT_SYMBOL(archosg6_lcd_get_io);

static int _hdmi_get_it_io(void) {

	return (_PIN_NB( disp_gpio.hdmi_it ));
}

static struct hdmi_io_fops hdmi_io = {
	.hdmi_get_it_io = &_hdmi_get_it_io,
};

struct hdmi_io_fops *archosg6_hdmi_get_io(void) {
	return &hdmi_io;
} 

EXPORT_SYMBOL(archosg6_hdmi_get_io);


static void _display_select( int enable )
{
	printk("Display select: %s\n", (enable==1 ? "LCD":"HDMI"));
	if ( enable )
		omap_set_gpio_dataout( _PIN_NB( disp_gpio.disp_select ), 1 );
	else
		omap_set_gpio_dataout( _PIN_NB( disp_gpio.disp_select ), 0 );

}

static void _display_cpld_enable( int enable )
{
	printk("CPLD reset: %d\n",enable);
	if ( enable )
		omap_set_gpio_dataout( _PIN_NB( disp_gpio.cpldreset ), 1 );
	else
		omap_set_gpio_dataout( _PIN_NB( disp_gpio.cpldreset ), 0 );

}

static void _hdmi_set_dac(int enable) {

	printk("HDMI dac: %d\n",enable);

	if ( enable )
		omap_set_gpio_dataout( _PIN_NB( disp_gpio.hdmi_dac ), 1 );
	else
		omap_set_gpio_dataout( _PIN_NB( disp_gpio.hdmi_dac ), 0 );

}

static void _cpld_set_aux(int enable) {


	if ( enable )
		omap_set_gpio_dataout( _PIN_NB( disp_gpio.cpld_aux ), 1 );
	else
		omap_set_gpio_dataout( _PIN_NB( disp_gpio.cpld_aux ), 0 );

}

static struct display_io_fops display_io = {
	.display_reset = _display_cpld_enable,
	.display_select = _display_select,
	.display_hdmidac = _hdmi_set_dac,
	.display_set_aux = _cpld_set_aux,
};

struct display_io_fops *archosg6_display_get_io(void) {
	return &display_io;
} 

EXPORT_SYMBOL(archosg6_display_get_io);


int archosg6_display_get_hdmi_i2c_nbr(void) {
	return board_disp_hdmi_i2c_nbr;
} 
EXPORT_SYMBOL(archosg6_display_get_hdmi_i2c_nbr);

int archosg6_display_get_extvideoenc_i2c_nbr(void) {
	return board_disp_extvideoenc_i2c_nbr;
} 
EXPORT_SYMBOL(archosg6_display_get_extvideoenc_i2c_nbr);


int __init archosg6_display_init(void)
{
	int ret;
	const struct archosg6_display_config *disp_cfg;
	disp_cfg = omap_get_config( ARCHOS_TAG_DISPLAY, struct archosg6_display_config );
	
	if (disp_cfg == NULL) {
		printk(KERN_DEBUG "archosg6_display_init: no board configuration found\n");
		return 0;
	}
	if ( hardware_rev >= disp_cfg->nrev ) {
		printk(KERN_DEBUG "archosg6_display_init: hardware_rev (%i) >= nrev (%i)\n",
			hardware_rev, disp_cfg->nrev);
		return 0;
	}

	/*NOTE : SL make sure DSS datalines are configured to output 24bits rgb */
	omap_cfg_reg(D28_3430_DSS_PCLK);
	omap_cfg_reg(D26_3430_DSS_HSYNC);
	omap_cfg_reg(D27_3430_DSS_VSYNC);
	omap_cfg_reg(E27_3430_DSS_ACBIAS);
	omap_cfg_reg(AG22_3430_DSS_DATA0);
	omap_cfg_reg(AH22_3430_DSS_DATA1);
	omap_cfg_reg(AG23_3430_DSS_DATA2);
	omap_cfg_reg(AH23_3430_DSS_DATA3);
	omap_cfg_reg(AG24_3430_DSS_DATA4);
	omap_cfg_reg(AH24_3430_DSS_DATA5);
	omap_cfg_reg(E26_3430_DSS_DATA6);
	omap_cfg_reg(F28_3430_DSS_DATA7);
	omap_cfg_reg(F27_3430_DSS_DATA8);
	omap_cfg_reg(G26_3430_DSS_DATA9);
	omap_cfg_reg(AD28_3430_DSS_DATA10);
	omap_cfg_reg(AD27_3430_DSS_DATA11);
	omap_cfg_reg(AB28_3430_DSS_DATA12);
	omap_cfg_reg(AB27_3430_DSS_DATA13);
	omap_cfg_reg(AA28_3430_DSS_DATA14);
	omap_cfg_reg(AA27_3430_DSS_DATA15);
	omap_cfg_reg(G25_3430_DSS_DATA16);
	omap_cfg_reg(H27_3430_DSS_DATA17);
	omap_cfg_reg(H26_3430_DSS_DATA18);
	omap_cfg_reg(H25_3430_DSS_DATA19);
	omap_cfg_reg(E28_3430_DSS_DATA20);
	omap_cfg_reg(J26_3430_DSS_DATA21);
	omap_cfg_reg(AC27_3430_DSS_DATA22);
	omap_cfg_reg(AC28_3430_DSS_DATA23);

	disp_gpio = disp_cfg->rev[hardware_rev];

	if ( machine_is_archos_g6tv() ) {
		board_disp_hdmi_i2c_nbr = 2;
		board_disp_extvideoenc_i2c_nbr = 2;
	} else {
		board_disp_hdmi_i2c_nbr = 3;
		board_disp_extvideoenc_i2c_nbr = 3;
	}

	/* cpld reset */
	_INIT_OUTPUT( disp_gpio.cpldreset );
	_INIT_STATE( disp_gpio.cpldreset, 0 );

	if ( !machine_is_archos_g6tv() ) {	

		/* display out selection */
		_INIT_OUTPUT( disp_gpio.disp_select);
		_INIT_STATE( disp_gpio.disp_select , 0 )
		/* lcd power */
		_INIT_OUTPUT( disp_gpio.lcd_pwon);
		_INIT_STATE( disp_gpio.lcd_pwon , 0 )

		/* lcd reset */
		_INIT_OUTPUT( disp_gpio.lcd_rst);
		_INIT_STATE( disp_gpio.lcd_rst , 0 )

		if ( machine_is_archos_g6l() ) {	
			/* lcd pci */
			_INIT_OUTPUT( disp_gpio.lcd_pci);
			_INIT_STATE( disp_gpio.lcd_pci , 0 )

		} else {
			/* bkl power */
			_INIT_OUTPUT(disp_gpio.bkl_pwon);
			_INIT_STATE( disp_gpio.bkl_pwon , 0 )
		}
	}

	/* hdmi int selection */
	_INIT_INPUT(disp_gpio.hdmi_it);
	/* hdmi dac selection */
	_INIT_OUTPUT(disp_gpio.hdmi_dac);
	/* cpld aux selection */
	_INIT_OUTPUT(disp_gpio.cpld_aux);

	ret = sysdev_class_register(&display_sysclass);
	if (ret >= 0)
		sysdev_class_create_file(&display_sysclass, &attr_gfxformat);

	return 0;
}
