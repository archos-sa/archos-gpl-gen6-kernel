#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>

#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#include "asm/arch/archosg6-audio.h"
#include <asm/mach-types.h>
#include <asm/arch/board.h>

static struct archosg6_audio_conf audio_gpio;

static void _set_ampli(int onoff)
{
	if ( onoff ==  PLAT_ON)
		omap_set_gpio_dataout( _PIN_NB( audio_gpio.spdif), 0);
	else
		omap_set_gpio_dataout( _PIN_NB( audio_gpio.spdif), 1);
}

static void _set_hp(int onoff)
{
	if ( onoff ==  PLAT_ON)
		omap_set_gpio_dataout( _PIN_NB( audio_gpio.hp_on), 1);
	else
		omap_set_gpio_dataout( _PIN_NB( audio_gpio.hp_on), 0);
}

static int _get_headphone_plugged(void)
{
	return omap_get_gpio_datain( _PIN_NB( audio_gpio.headphone_plugged) );
}

static void _sys_clkout1_en(int en)
{
	struct clk *sys_clkout1;

	sys_clkout1 = clk_get(NULL, "sys_clkout1");
	if (!IS_ERR(sys_clkout1)) {
		if (en == PLAT_ON) {
			if ( clk_enable(sys_clkout1) != 0) {
				printk(KERN_ERR "failed to enable sys_clkout1\n");
			}
		} else
			clk_disable(sys_clkout1);
		
		clk_put(sys_clkout1);
	}
}

static struct audio_device_config audio_device_io = {
	.set_spdif = &_set_ampli,
	.get_headphone_plugged =&_get_headphone_plugged,
	.set_codec_master_clk_state = &_sys_clkout1_en,
	.set_speaker_state = &_set_hp,
};

static struct audio_device_config g6tv_audio_device_io = {
	.set_spdif = &_set_ampli,
	.get_headphone_plugged = 0,
	.set_codec_master_clk_state = &_sys_clkout1_en,
	.set_speaker_state = 0,
};

struct audio_device_config *archosg6_audio_get_io(void) {

	if ( machine_is_archos_g6tv() )
		return &g6tv_audio_device_io;
	else
		return &audio_device_io;
} 


int __init archosg6_audio_gpio_init(void)
{
	const struct archosg6_audio_config *audio_cfg;
	
	/* audio  */
	audio_cfg = omap_get_config( ARCHOS_TAG_AUDIO, struct archosg6_audio_config );
	if (audio_cfg == NULL) {
		printk(KERN_DEBUG "archosg6_audio_init: no board configuration found\n");
		return -ENODEV;
	}
	if ( hardware_rev >= audio_cfg->nrev ) {
		printk(KERN_DEBUG "archosg6_audio_init: hardware_rev (%i) >= nrev (%i)\n",
			hardware_rev, audio_cfg->nrev);
		return -ENODEV;
	}

	audio_gpio = audio_cfg->rev[hardware_rev];

	_INIT_OUTPUT( audio_gpio.spdif );

	_INIT_OUTPUT( audio_gpio.hp_on );

	_INIT_INPUT( audio_gpio.headphone_plugged );

	printk("Audio GPIO init done\n");
	return 0;
}

EXPORT_SYMBOL(archosg6_audio_get_io);

