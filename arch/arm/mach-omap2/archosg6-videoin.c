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
#include <asm/arch/archosg6-gpio.h>
#include <asm/arch/archosg6-videoin.h>


struct board_video_in {
	struct g6_gpio rst_tvp;
	struct g6_gpio pwdn_tvp;
	struct g6_gpio dvbt_tvp;
	struct g6_gpio cam_reset;
	struct g6_gpio rst_dvbt;
	struct g6_gpio dvbt_1v8_on;
	struct g6_gpio dvbt_5v_on;
	struct g6_gpio dvbt_3v3_on;
};

static struct board_video_in g6tv_video_in = {
	.rst_tvp =  	{ 175, AC3_3430_GPIO175 },
	.pwdn_tvp = 	{ 176, AB1_3430_GPIO176 },
	.dvbt_tvp =	{ 12, AF10_3430_GPIO12 },
	.cam_reset =	{ 98, C23_3430_GPIO98 },
	.rst_dvbt =	{ 174, AC2_3430_GPIO174 },
	.dvbt_1v8_on =	{ 171, AB3_3430_GPIO171 },
	.dvbt_5v_on =	{ 172, AB4_3430_GPIO172 },
	.dvbt_3v3_on =	{ 173, AA4_3430_GPIO173 },
};

static struct board_video_in *video_in=NULL;


/* this function is mainly for g6tv which has to select */
/* between tvp and dvbt */
static void _select_tvp_vs_dvbt ( int sel ) {

	if ( video_in != NULL ) {

		if ( sel == SELECT_TVP ) {
			/* remove tvp and dvbt reset */
			omap_set_gpio_dataout( _PIN_NB(video_in->cam_reset) , 1 );
			/* select tvp */
			omap_set_gpio_dataout( _PIN_NB(video_in->dvbt_tvp) , 0 );
	
		} else if ( sel == SELECT_DVBT ) {
			/* remove tvp and dvbt reset */
			omap_set_gpio_dataout( _PIN_NB(video_in->cam_reset) , 1 );
			/* select dvbt */
			omap_set_gpio_dataout( _PIN_NB(video_in->dvbt_tvp) , 1 );
		}
	}

	return;
}

/* this function is mainly for g6tv which has to select */
/* between tvp and dvbt */
static void _tvp_power ( int sel ) {

	if ( video_in != NULL ) {

		if ( sel == POWER_ON_TVP ) {
			omap_set_gpio_dataout( _PIN_NB(video_in->pwdn_tvp) , 0 );
	
		} else if ( sel == POWER_OFF_TVP ) {
			omap_set_gpio_dataout( _PIN_NB(video_in->pwdn_tvp) , 1 );
		}
	}

	return;
}

static void _tvp_bus_width_select ( int sel ) {

	if ( video_in != NULL ) {

		if ( sel == TVP_8_BITS ) {
			omap_set_gpio_dataout( _PIN_NB(video_in->rst_tvp) , 1 );
	
		} else if ( sel == TVP_10_BITS ) {
			omap_set_gpio_dataout( _PIN_NB(video_in->rst_tvp) , 0 );
		}
	}


	return;
}

static int _get_tvp_i2c_nb(void) {

	if ( machine_is_archos_g6tv() ) 
		return 2;
	else
		return 3;
}

static void _dvbt_power ( int sel ) {

	if ( video_in != NULL ) {

		if ( sel == POWER_OFF_DVBT ) {
			omap_set_gpio_dataout( _PIN_NB(video_in->dvbt_1v8_on) , 0 );
			omap_set_gpio_dataout( _PIN_NB(video_in->dvbt_3v3_on) , 0 );
			omap_set_gpio_dataout( _PIN_NB(video_in->dvbt_5v_on) ,  0 );
	
		} else /* power-on */ {
			omap_set_gpio_dataout( _PIN_NB(video_in->dvbt_5v_on) ,  1 );
			omap_set_gpio_dataout( _PIN_NB(video_in->dvbt_3v3_on) , 1 );
			omap_set_gpio_dataout( _PIN_NB(video_in->dvbt_1v8_on) , 1 );
		}
	}
	return;
}

static void _dvbt_reset ( int sel ) {

	if ( video_in != NULL ) {

		if ( sel == RESET_RELEASE_DVBT ) {
			omap_set_gpio_dataout( _PIN_NB(video_in->rst_dvbt) , 1 );
	
		} else { /* reset active */
			omap_set_gpio_dataout( _PIN_NB(video_in->rst_dvbt) , 0 );
		}
	}
	return;
}


static struct video_in_io_fops video_in_io = {
	.select_chip = &_select_tvp_vs_dvbt,
	.tvp_onoff   = &_tvp_power,
	.tvp_bus_width_select  = &_tvp_bus_width_select,
	.get_tvp_i2c_nb = &_get_tvp_i2c_nb,
	.dvbt_onoff = & _dvbt_power,
	.dvbt_reset = & _dvbt_reset,
};

struct video_in_io_fops *archosg6_video_in_get_io(void) {
	return &video_in_io;
} 

EXPORT_SYMBOL(archosg6_video_in_get_io);

int __init archosg6_videoin_init(void)
{

	if ( machine_is_archos_g6tv() ) {
		video_in = &g6tv_video_in;
	}

	if ( video_in != NULL) {

		_INIT_OUTPUT(video_in->rst_tvp);
		_INIT_OUTPUT(video_in->pwdn_tvp);
		video_in_io.tvp_onoff(POWER_ON_TVP);
		_INIT_OUTPUT(video_in->dvbt_tvp);
		_INIT_OUTPUT(video_in->cam_reset);
		_INIT_OUTPUT(video_in->rst_dvbt);
		_INIT_OUTPUT(video_in->dvbt_1v8_on);
		_INIT_OUTPUT(video_in->dvbt_5v_on);
		_INIT_OUTPUT(video_in->dvbt_3v3_on);

		video_in_io.dvbt_reset(RESET_PULL_DVBT); /* Hold reset */
		video_in_io.dvbt_onoff(POWER_ON_DVBT);   /* Apply power to chip */
	}
	
	return 0;
}
