#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>

#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#include <asm/mach-types.h>
#include <asm/arch/board.h>

static struct gpio_keys_button g6_gpio_keys_buttons[] = {

	[0] = {
		.code		= KEY_VOLUMEUP,
		.gpio		= 0,
		.desc		= "vol up sw",
		.active_low	= 1,
		.type		= EV_KEY,
	},
	[1] = {
		.code		= KEY_VOLUMEDOWN,
		.gpio		= 0,
		.desc		= "vol down sw",
		.active_low	= 1,
		.type		= EV_KEY,
	},
	[2] = {
		.code		= KEY_POWER,
		.gpio		= 0,
		.desc		= "power sw",
		.active_low	= 0,
		.type		= EV_KEY,
	},
};

static struct gpio_keys_platform_data g6_gpio_keys = {
	.buttons		= g6_gpio_keys_buttons,
	.nbuttons		= 0,
};

static struct platform_device g6_gpio_keys_device = {
	.name			= "gpio-keys",
	.id			= -1,
	.dev			= {
		.platform_data	= &g6_gpio_keys,
	},
};

int __init archosg6_keys_init(void)
{
	const struct archosg6_keys_config *keys_cfg;
	
	keys_cfg = omap_get_config( ARCHOS_TAG_KEYS, struct archosg6_keys_config );
	if (keys_cfg == NULL) {
		printk(KERN_DEBUG "archosg6_keys_init: no board configuration found\n");
		return -ENODEV;
	}

	if ( hardware_rev >= keys_cfg->nrev ) {
		printk(KERN_DEBUG "archosg6_keys_init: hardware_rev (%i) >= nrev (%i)\n",
			hardware_rev, keys_cfg->nrev);
		return 0;
	}

	/* Vol Up SW */
	_INIT_INPUT( keys_cfg->rev[hardware_rev].vol_up );
	g6_gpio_keys_buttons[0].gpio = _PIN_NB( keys_cfg->rev[hardware_rev].vol_up );
	g6_gpio_keys.nbuttons++;
	
	/* Vol Down SW */
	_INIT_INPUT( keys_cfg->rev[hardware_rev].vol_down );
	g6_gpio_keys_buttons[1].gpio = _PIN_NB(  keys_cfg->rev[hardware_rev].vol_down );
	g6_gpio_keys.nbuttons++;

	if (_PIN_NB( keys_cfg->rev[hardware_rev].power )) {
		_INIT_INPUT( keys_cfg->rev[hardware_rev].power );
		g6_gpio_keys_buttons[2].gpio = _PIN_NB( keys_cfg->rev[hardware_rev].power );
		g6_gpio_keys.nbuttons++;
	}

	return platform_device_register(&g6_gpio_keys_device);
}

