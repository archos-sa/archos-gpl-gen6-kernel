#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <asm/arch/mcspi.h>
#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#include <asm/mach-types.h>
#include <asm/arch/board.h>


/* GPIO used for TSC2046 (touchscreen)
 *
 * Also note that the tsc2046 is the same silicon as the ads7846, so
 * that driver is used for the touchscreen. */
static struct g6_gpio ts_pwron;
static struct g6_gpio ts_irq;

static int ads7846_get_pendown_state(void)
{
	return !omap_get_gpio_datain( _PIN_NB( ts_irq ) );
}

/* This enable(1)/disable(0) the voltage for TS */
static int ads7846_vaux_control(int vaux_cntrl)
{
	if (vaux_cntrl == VAUX_ENABLE)
		omap_set_gpio_dataout( _PIN_NB( ts_pwron ), 1);
	else if (vaux_cntrl == VAUX_DISABLE)
		omap_set_gpio_dataout( _PIN_NB( ts_pwron ), 0);

	return 0;
}

static struct omap2_mcspi_device_config tsc2046_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel = 1,  /* 0: slave, 1: master */
};

static struct ads7846_platform_data tsc2046_config = {
	.get_pendown_state = ads7846_get_pendown_state,
	.keep_vref_on	   = 1,
	.vaux_control	   = ads7846_vaux_control,
	.settle_delay_usecs = 100,
	.x_plate_ohms      = 745,
	.pressure_min	   = 100,
	.pressure_max	   = 700,
	.penirq_recheck_delay_usecs = 3000,	
};

struct spi_board_info g6_ts_spi_board_info = {
	/* TSC2046 operates at a max freqency of 2MHz, so
	 * operate slightly below at 1.5MHz */
	.modalias	= "ads7846",
	.bus_num	= 1,
	.chip_select	= 0,
	.max_speed_hz   = 400000,
	.controller_data= &tsc2046_mcspi_config,
	.platform_data  = &tsc2046_config,
};

void __init ads7846_dev_init(struct spi_board_info *pt_spi)
{
	const struct archosg6_tsp_config *tsp_cfg;
	tsp_cfg = omap_get_config( ARCHOS_TAG_TSP, struct archosg6_tsp_config );
	/* might be NULL, G6TV has not touchscreen */
	if (tsp_cfg == NULL) {
		printk(KERN_DEBUG "ads7846_dev_init: no board configuration found\n");
		return;
	}
	if ( hardware_rev >= tsp_cfg->nrev ) {
		printk(KERN_DEBUG "ads7846_dev_init: hardware_rev (%i) >= nrev (%i)\n",
			hardware_rev, tsp_cfg->nrev);
		return;
	}

	ts_irq = tsp_cfg->rev[hardware_rev].irq_gpio;
	ts_pwron = tsp_cfg->rev[hardware_rev].pwr_gpio;


	tsc2046_config.x_plate_ohms = tsp_cfg->rev[hardware_rev].x_plate_ohms;
	tsc2046_config.pressure_max = tsp_cfg->rev[hardware_rev].pressure_max;


	printk(KERN_DEBUG "ads7846_dev_init: irq_gpio %i, pwr_gpio %i\n",
			ts_irq.nb, ts_pwron.nb);

	_INIT_OUTPUT( ts_pwron );
	_INIT_INPUT( ts_irq );

	omap_set_gpio_debounce( _PIN_NB( ts_irq ) , 1);
	omap_set_gpio_debounce_time( _PIN_NB( ts_irq ) , 0xa);

	/* fix spi irq gio nb */
	g6_ts_spi_board_info.irq = OMAP_GPIO_IRQ(_PIN_NB( ts_irq ));
	memcpy(pt_spi, &g6_ts_spi_board_info, sizeof(struct spi_board_info));

}

