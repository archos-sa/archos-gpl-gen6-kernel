#include <linux/init.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/suspend.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/videodev.h>
#include <media/video-buf.h>
#include <media/video-buf-dvb.h>
#include <linux/i2c.h>
#include <asm/arch/hardware.h>
#include <asm/arch/gpio.h>
#include <asm/mach-types.h>
#include <asm/arch/mux.h>
#include <asm/arch/archosg6-videoin.h>

#include "dib3000mc.h"
#include "dib7000m.h"
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

#ifdef CONFIG_MACH_ARCHOS_G6
#define ARCHOS_DIB7070P_I2C_ADAPTER_ID 3
#endif
#ifdef CONFIG_MACH_OMAP_3430SDP
#define ARCHOS_DIB7070P_I2C_ADAPTER_ID 2
#endif

extern struct videobuf_queue_ops* omap34xxarcdvb_get_videobufqops(void);
extern spinlock_t* omap34xxarcdvb_get_videobufqlock(void);

struct archosdib7070p_t {

	struct videobuf_queue bufqueue;

	struct videobuf_dvb dvb;
	struct videobuf_queue_ops *dvb_qops;
	spinlock_t *qops_lock;
	struct video_in_io_fops * video_io_ops;
	struct i2c_adapter *i2c_adap;
	
	spinlock_t *irqlock;
};

static struct archosdib7070p_t *driver_data = NULL;
static char driver_name[] = { DRIVER_NAME };

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
	.output_mpeg2_in_188_bytes = 0,

	.agc_config_count = 1,
	.agc = &dib7070_agc_config,
	.bw  = &dib7070_bw_config_12_mhz,

	.gpio_dir = DIB7000P_GPIO_DEFAULT_DIRECTIONS,
	.gpio_val = DIB7000P_GPIO_DEFAULT_VALUES,
	.gpio_pwm_pos = DIB7000P_GPIO_DEFAULT_PWM_POS,

	.hostbus_diversity = 1,
	
	.output_mode = OUTMODE_MPEG2_PAR_GATED_CLK,
};

/*
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
*/

static int archosdib7070p_init(void)
{
	struct i2c_adapter *tun_i2c = NULL;
	int ret;

	driver_data = kmalloc(sizeof(struct archosdib7070p_t), GFP_KERNEL);
	if ( ! driver_data)
	{
		return -1;
	}
	memset(driver_data, 0, sizeof(struct archosdib7070p_t));

	driver_data->dvb.name = driver_name;
	driver_data->dvb.dvbq = driver_data->bufqueue;
		
	// fill videbuf_queue_ops with functions from omap34xxarcdvb.c
	driver_data->dvb_qops = omap34xxarcdvb_get_videobufqops();
	if (driver_data->dvb_qops == NULL)
	{
		printk(KERN_ERR "omap34xxarcdvb_get_videobufqops failed\n");
		goto out_error;
	}

	//spin_lock_init(&driver_data->irqlock);
	driver_data->irqlock = omap34xxarcdvb_get_videobufqlock();
	if (driver_data->irqlock == NULL)
	{
		printk(KERN_ERR "omap34xxarcdvb_get_videobufqlock failed\n");
		goto out_error;
	}

	driver_data->video_io_ops = archosg6_video_in_get_io();
	if (driver_data->video_io_ops == NULL)
	{
		printk(KERN_ERR "archosg6_video_in_get_io failed\n");
		goto out_error;
	}

	/* Enable the dvb-t chip */
	/* Temporary : Select I/O data path */
	driver_data->video_io_ops->select_chip(SELECT_DVBT);
	driver_data->video_io_ops->dvbt_reset(RESET_PULL_DVBT);
 	udelay(1000);
	driver_data->video_io_ops->dvbt_reset(RESET_RELEASE_DVBT);
	udelay(2000); /* Wait after reset for chip stabilization and clock start */

	videobuf_queue_init(&driver_data->dvb.dvbq, driver_data->dvb_qops,
		NULL, driver_data->irqlock,
		V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_FIELD_ALTERNATE,
		sizeof(struct videobuf_buffer), driver_data);

	//videobuf_set_buftype(&driver_data->dvb.dvbq, VIDEOBUF_BUF_LINEAR);
	//INIT_LIST_HEAD(&driver_data->dma_queue);

	/* Get the I2C adapter */
	driver_data->i2c_adap = i2c_get_adapter(ARCHOS_DIB7070P_I2C_ADAPTER_ID);
	if (!driver_data->i2c_adap) {
		printk(KERN_ERR "i2c_get_adapter(%d) failed\n", ARCHOS_DIB7070P_I2C_ADAPTER_ID);
		goto out_error;
	}

	if ( (driver_data->video_io_ops->dvbt_reset == NULL) || 
	     (driver_data->video_io_ops->dvbt_onoff == NULL) ||
	     (driver_data->video_io_ops->select_chip == NULL) )
	{
		printk(KERN_ERR "archosg6_video_in_get_io : i/o operations undefined\n");
		goto out_error;
	}

	ret = dib7000p_i2c_enumeration(driver_data->i2c_adap, 1, 0x12, &dib7070p_dib7000p_config);
	if ( ret != 0 ) {
		printk("KERN_ERR dib7000p_i2c_enumeration failed\n");
		goto out_error;
	}
	driver_data->dvb.frontend = dvb_attach(dib7000p_attach, driver_data->i2c_adap, 0x80, &dib7070p_dib7000p_config);
	if (!driver_data->dvb.frontend) {
		printk(KERN_ERR "dib7000p_attach failed\n");
		goto out_error;
	}

	tun_i2c = dib7000p_get_i2c_master(driver_data->dvb.frontend, DIBX000_I2C_INTERFACE_TUNER, 1);
	if (!tun_i2c) {
		printk(KERN_ERR "dib7000p_get_i2c_master failed\n");
		goto out_error;
	}

	dvb_attach(dib0070_attach, driver_data->dvb.frontend, tun_i2c, &dib7070p_dib0070_config[0]);
	
	//vpfe_dev->dvb.priv = vpfe_dev->dvb.frontend->ops.tuner_ops.set_params;
	//vpfe_dev->dvb.frontend->fe->ops.tuner_ops.set_params = dib7070_set_param_override;	

	if (driver_data->dvb.frontend == NULL) {
		printk(KERN_ERR "%s: frontend initialization failed\n", __FUNCTION__);
		goto out_error;
	}
	
	ret = videobuf_dvb_register(&driver_data->dvb, THIS_MODULE, driver_data, NULL);
	printk(KERN_INFO "%s: Initialization successful\n", __FUNCTION__);
	return ret;

out_error:
	if (driver_data)
		kfree(driver_data);
	driver_data = NULL;
	return -1;
}

static void archosdib7070p_exit(void)
{
	videobuf_dvb_unregister(&driver_data->dvb);
	
	if (driver_data)
		kfree(driver_data);
	driver_data = NULL;
}

module_init(archosdib7070p_init);
module_exit(archosdib7070p_exit);

MODULE_AUTHOR("Collin Mulliner Archos S.A.");
MODULE_DESCRIPTION("DIB7070P DVB-T Driver");
MODULE_LICENSE("GPL");
