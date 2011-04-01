#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include "dib7000p.h"
#include "dib0070.h"

#include "demux.h"
#include "dmxdev.h"
#include "dvb_demux.h"
#include "dvb_frontend.h"
#include "dvb_net.h"
#include "dvbdev.h"
#include "dib0070.h"

#define DRIVER_NAME "ArchosDiB7070P"

#define ARCHOS_DIB7070P_I2C_ADAPTER_ID 1

#define NHWFILTERS 8

struct archos_dib7070p_t {
	u8 id;
	int state;

	// DVB
	struct dvb_adapter dvb_adapter;
	struct dvb_frontend *fe;
	struct dmx_frontend hw_frontend;
	struct dmx_frontend mem_frontend;
	struct dmxdev dmxdev;
	struct dvb_demux demux;
	
	unsigned int full_ts_users;
	unsigned int users;

	// I2C
	//struct i2c_algo_bit_data i2c_bit;
	struct i2c_adapter *i2c_adap;
	unsigned int i2cbug;
	
	int (*set_param_save) (struct dvb_frontend *, struct dvb_frontend_parameters *);
	
	void *priv;
};

static struct archos_dib7070p_t *driver_data = NULL;

/* DIB7070 generic */
static struct dibx000_agc_config dib7070_agc_config = {
	BAND_UHF | BAND_VHF | BAND_LBAND | BAND_SBAND,
	/* P_agc_use_sd_mod1=0, P_agc_use_sd_mod2=0, P_agc_freq_pwm_div=5, P_agc_inv_pwm1=0, P_agc_inv_pwm2=0,
	 * P_agc_inh_dc_rv_est=0, P_agc_time_est=3, P_agc_freeze=0, P_agc_nb_est=5, P_agc_write=0 */
	(0 << 15) | (0 << 14) | (5 << 11) | (0 << 10) | (0 << 9) | (0 << 8) | (3 << 5) | (0 << 4) | (5 << 1) | (0 << 0), // setup

	600, // inv_gain
	10,  // time_stabiliz

	0,  // alpha_level
	118,  // thlock

	0,     // wbd_inv
	3530,  // wbd_ref
	1,     // wbd_sel
	5,     // wbd_alpha

	65535,  // agc1_max
		0,  // agc1_min

	65535,  // agc2_max
	0,      // agc2_min

	0,      // agc1_pt1
	40,     // agc1_pt2
	183,    // agc1_pt3
	206,    // agc1_slope1
	255,    // agc1_slope2
	72,     // agc2_pt1
	152,    // agc2_pt2
	88,     // agc2_slope1
	90,     // agc2_slope2

	17,  // alpha_mant
	27,  // alpha_exp
	23,  // beta_mant
	51,  // beta_exp

	0,  // perform_agc_softsplit
};

static int dib7070_tuner_reset(struct dvb_frontend *fe, int onoff)
{
	return dib7000p_set_gpio(fe, 8, 0, !onoff);
}

static int dib7070_tuner_sleep(struct dvb_frontend *fe, int onoff)
{
	return dib7000p_set_gpio(fe, 9, 0, onoff);
}

static struct dib0070_config dib7070p_dib0070_config[2] = {
	{
		.i2c_address = DEFAULT_DIB0070_I2C_ADDRESS,
		.reset = dib7070_tuner_reset,
		.sleep = dib7070_tuner_sleep,
		.clock_khz = 12000,
		.clock_pad_drive = 4
	}, {
		.i2c_address = DEFAULT_DIB0070_I2C_ADDRESS,
		.reset = dib7070_tuner_reset,
		.sleep = dib7070_tuner_sleep,
		.clock_khz = 12000,

	}
};

static struct dibx000_bandwidth_config dib7070_bw_config_12_mhz = {
	60000, 15000, // internal, sampling
	1, 20, 3, 1, 0, // pll_cfg: prediv, ratio, range, reset, bypass
	0, 0, 1, 1, 2, // misc: refdiv, bypclk_div, IO_CLK_en_core, ADClkSrc, modulo
	(3 << 14) | (1 << 12) | (524 << 0), // sad_cfg: refsel, sel, freq_15k
	(0 << 25) | 0, // ifreq = 0.000000 MHz
	20452225, // timf
	12000000, // xtal_hz
};

static struct dib7000p_config dib7070p_dib7000p_config = {
	.output_mpeg2_in_188_bytes = 1,

	.agc_config_count = 1,
	.agc = &dib7070_agc_config,
	.bw  = &dib7070_bw_config_12_mhz,

	.gpio_dir = DIB7000P_GPIO_DEFAULT_DIRECTIONS,
	.gpio_val = DIB7000P_GPIO_DEFAULT_VALUES,
	.gpio_pwm_pos = DIB7000P_GPIO_DEFAULT_PWM_POS,

	.hostbus_diversity = 1,
};

static int archos_dib7070p_frontend_attach(struct archos_dib7070p_t *data)
{
	dib7000p_i2c_enumeration(data->i2c_adap, 1, 0x12, &dib7070p_dib7000p_config);

	data->fe = dvb_attach(dib7000p_attach, data->i2c_adap, 0x80, &dib7070p_dib7000p_config);
	if (data->fe == NULL)
		printk("%s: dib7000p_attach failed\n", __FUNCTION__);
	
	return data->fe == NULL ? -ENODEV : 0;
}


static int dib7070_set_param_override(struct dvb_frontend *fe, struct dvb_frontend_parameters *fep)
{
	struct archos_dib7070p_t *data = fe->dvb->priv;

	u16 offset;
	u8 band = BAND_OF_FREQUENCY(fep->frequency/1000);
	switch (band) {
		case BAND_VHF: offset = 950; break;
		case BAND_UHF:
		default: offset = 550; break;
	}
	
	//printk("WBD for DiB7000P: %d\n", offset + dib0070_wbd_offset(fe));
	dib7000p_set_wbd_ref(fe, offset + dib0070_wbd_offset(fe));
	
	return data->set_param_save(fe, fep);
}


static int dib7070p_tuner_attach(struct archos_dib7070p_t *data)
{
	struct i2c_adapter *tun_i2c = dib7000p_get_i2c_master(data->fe, DIBX000_I2C_INTERFACE_TUNER, 1);

	if (data->id == 0) {
		if (dvb_attach(dib0070_attach, data->fe, tun_i2c, &dib7070p_dib0070_config[0]) == NULL) {
			printk("%s: tuner0 attach failed\n", __FUNCTION__);
			return -ENODEV;
		}
	} else {
		if (dvb_attach(dib0070_attach, data->fe, tun_i2c, &dib7070p_dib0070_config[1]) == NULL) {
			printk("%s: tuner1 attach failed\n", __FUNCTION__);
			return -ENODEV;
		}
	}

	data->set_param_save = data->fe->ops.tuner_ops.set_params;
	data->fe->ops.tuner_ops.set_params = dib7070_set_param_override;
	
	return 0;
}

static int __devinit frontend_init(struct archos_dib7070p_t *data)
{
	int ret;

	ret = archos_dib7070p_frontend_attach(data);
	if (ret < 0)
		return ret;
		
	ret = dib7070p_tuner_attach(data);
	if (ret < 0)
		return ret;
	
	ret = dvb_register_frontend(&data->dvb_adapter, data->fe);
	if (ret < 0) {
		if (data->fe->ops.release)
			data->fe->ops.release(data->fe);
		return ret;
	}

	return 0;
}

static int start_feed(struct dvb_demux_feed *f)
{
	printk("%s called\n", __FUNCTION__);
	return 0;
}

static int stop_feed(struct dvb_demux_feed *f)
{
	printk("%s called\n", __FUNCTION__);
	return 0;
}


static int __devinit probe(void)
{
	struct dvb_adapter *dvb_adapter;
	struct dvb_demux *dvbdemux;
	struct dmx_demux *dmx;
	int ret = -ENOMEM;


	driver_data = kzalloc(sizeof(struct archos_dib7070p_t), GFP_KERNEL);
	if (!driver_data)
		goto out;
	
	/* TODO: turn on DVB-T hardware and reset it ... */
		
	/* DVB */
	ret = dvb_register_adapter(&driver_data->dvb_adapter, DRIVER_NAME, THIS_MODULE, NULL);
	if (ret < 0) {
		printk("dvd_register_adapter failed\n");
		goto err_exit;
	}
	
	dvb_adapter = &driver_data->dvb_adapter;
	
	if (dvb_adapter->priv == NULL) {
		dvb_adapter->priv = driver_data;
	}
	else {
		printk("priv is already used!\n");
	}
	
	// I2C
	driver_data->i2c_adap = i2c_get_adapter(ARCHOS_DIB7070P_I2C_ADAPTER_ID);
	if (!driver_data->i2c_adap) {
		printk("I2C get_adapter failed\n");
		goto err_exit;
	}

	dvbdemux = &driver_data->demux;
	dvbdemux->filternum = 256;
	dvbdemux->feednum = 256;
	
	dvbdemux->start_feed = start_feed;
	dvbdemux->stop_feed = stop_feed;
	
	dvbdemux->dmx.capabilities = DMX_TS_FILTERING | DMX_SECTION_FILTERING;
	ret = dvb_dmx_init(dvbdemux);
	if (ret < 0)
		goto err_dvb_unregister_adapter;

	dmx = &dvbdemux->dmx;
	
	//driver_data->hw_frontend.source = DMX_FRONTEND_0;
	//driver_data->mem_frontend.source = DMX_MEMORY_FE;
	driver_data->dmxdev.filternum = NHWFILTERS;
	driver_data->dmxdev.demux = dmx;
	driver_data->dmxdev.capabilities = 0;

	ret = dvb_dmxdev_init(&driver_data->dmxdev, dvb_adapter);
	if (ret < 0)
		goto err_dvb_dmx_release;

	ret = dmx->add_frontend(dmx, &driver_data->hw_frontend);
	if (ret < 0)
		goto err_dvb_dmxdev_release;

	ret = dmx->add_frontend(dmx, &driver_data->mem_frontend);
	if (ret < 0)
		goto err_remove_hw_frontend;

	ret = dmx->connect_frontend(dmx, &driver_data->hw_frontend);
	if (ret < 0)
		goto err_remove_mem_frontend;

	ret = frontend_init(driver_data);
	if (ret < 0)
		goto err_disconnect_frontend;

	//dvb_net_init(dvb_adapter, &driver_data->dvbnet, dmx);
	
out:
	return ret;

err_disconnect_frontend:
	dmx->disconnect_frontend(dmx);
err_remove_mem_frontend:
	dmx->remove_frontend(dmx, &driver_data->mem_frontend);
err_remove_hw_frontend:
	dmx->remove_frontend(dmx, &driver_data->hw_frontend);
err_dvb_dmxdev_release:
	dvb_dmxdev_release(&driver_data->dmxdev);
err_dvb_dmx_release:
	dvb_dmx_release(dvbdemux);
err_dvb_unregister_adapter:
	dvb_unregister_adapter(dvb_adapter);
err_exit:
	kfree(driver_data);
	goto out;
}

static void __devexit remove(void)
{
	struct dvb_adapter *dvb_adapter = &driver_data->dvb_adapter;
	struct dvb_demux *dvbdemux = &driver_data->demux;
	struct dmx_demux *dmx = &dvbdemux->dmx;

	dmx->close(dmx);
		
	if (driver_data->fe)
		dvb_unregister_frontend(driver_data->fe);

	dmx->disconnect_frontend(dmx);
	dmx->remove_frontend(dmx, &driver_data->mem_frontend);
	dmx->remove_frontend(dmx, &driver_data->hw_frontend);
	dvb_dmxdev_release(&driver_data->dmxdev);
	dvb_dmx_release(dvbdemux);
	dvb_unregister_adapter(dvb_adapter);
	
	/* TODO: turn off DVB-T hardware ... */

	kfree(driver_data);
	driver_data = NULL;
}

static int __init archos_dib7070p_init(void)
{
	probe();
	
	return 0;
}

static void __exit archos_dib7070p_exit(void)
{
	if (driver_data)
		remove();
}

module_init(archos_dib7070p_init);
module_exit(archos_dib7070p_exit);

MODULE_AUTHOR("Collin Mulliner , Archos S.A.");
MODULE_DESCRIPTION("Archos DiB7070P Driver");
MODULE_LICENSE("GPL");
