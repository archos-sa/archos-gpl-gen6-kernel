/*
    tda18271-fe.c - driver for the Philips / NXP TDA18271 silicon tuner

    Copyright (C) 2007 Michael Krufky (mkrufky@linuxtv.org)

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#include <linux/delay.h>
#include "compat.h"
#include <linux/videodev2.h>
#include "tuner-driver.h"

#include "tda18271.h"
#include "tda18271-priv.h"

static int tda18271_debug;
module_param_named(debug, tda18271_debug, int, 0644);
MODULE_PARM_DESC(debug, "Turn on/off frontend debugging (default:off).");

#define dprintk(level, fmt, arg...) do {\
	if (tda18271_debug >= level) \
		printk(KERN_DEBUG "%s: " fmt, __FUNCTION__, ##arg); } while (0)

/*---------------------------------------------------------------------*/

#define TDA18271_ANALOG  0
#define TDA18271_DIGITAL 1

struct tda18271_priv {
	u8 i2c_addr;
	struct i2c_adapter *i2c_adap;
	unsigned char tda18271_regs[TDA18271_NUM_REGS];
	int mode;

	u32 frequency;
	u32 bandwidth;
};

static int tda18271_i2c_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	struct tda18271_priv *priv = fe->tuner_priv;
	struct analog_tuner_ops *ops = fe->ops.analog_demod_ops;
	int ret = 0;

	switch (priv->mode) {
	case TDA18271_ANALOG:
		if (ops && ops->i2c_gate_ctrl)
			ret = ops->i2c_gate_ctrl(fe, enable);
		break;
	case TDA18271_DIGITAL:
		if (fe->ops.i2c_gate_ctrl)
			ret = fe->ops.i2c_gate_ctrl(fe, enable);
		break;
	}

	return ret;
};

/*---------------------------------------------------------------------*/

static void tda18271_dump_regs(struct dvb_frontend *fe)
{
	struct tda18271_priv *priv = fe->tuner_priv;
	unsigned char *regs = priv->tda18271_regs;

	dprintk(1, "=== TDA18271 REG DUMP ===\n");
	dprintk(1, "ID_BYTE            = 0x%x\n", 0xff & regs[R_ID]);
	dprintk(1, "THERMO_BYTE        = 0x%x\n", 0xff & regs[R_TM]);
	dprintk(1, "POWER_LEVEL_BYTE   = 0x%x\n", 0xff & regs[R_PL]);
	dprintk(1, "EASY_PROG_BYTE_1   = 0x%x\n", 0xff & regs[R_EP1]);
	dprintk(1, "EASY_PROG_BYTE_2   = 0x%x\n", 0xff & regs[R_EP2]);
	dprintk(1, "EASY_PROG_BYTE_3   = 0x%x\n", 0xff & regs[R_EP3]);
	dprintk(1, "EASY_PROG_BYTE_4   = 0x%x\n", 0xff & regs[R_EP4]);
	dprintk(1, "EASY_PROG_BYTE_5   = 0x%x\n", 0xff & regs[R_EP5]);
	dprintk(1, "CAL_POST_DIV_BYTE  = 0x%x\n", 0xff & regs[R_CPD]);
	dprintk(1, "CAL_DIV_BYTE_1     = 0x%x\n", 0xff & regs[R_CD1]);
	dprintk(1, "CAL_DIV_BYTE_2     = 0x%x\n", 0xff & regs[R_CD2]);
	dprintk(1, "CAL_DIV_BYTE_3     = 0x%x\n", 0xff & regs[R_CD3]);
	dprintk(1, "MAIN_POST_DIV_BYTE = 0x%x\n", 0xff & regs[R_MPD]);
	dprintk(1, "MAIN_DIV_BYTE_1    = 0x%x\n", 0xff & regs[R_MD1]);
	dprintk(1, "MAIN_DIV_BYTE_2    = 0x%x\n", 0xff & regs[R_MD2]);
	dprintk(1, "MAIN_DIV_BYTE_3    = 0x%x\n", 0xff & regs[R_MD3]);
}

static void tda18271_read_regs(struct dvb_frontend *fe)
{
	struct tda18271_priv *priv = fe->tuner_priv;
	unsigned char *regs = priv->tda18271_regs;
	unsigned char buf = 0x00;
	int ret;
	struct i2c_msg msg[] = {
		{ .addr = priv->i2c_addr, .flags = 0,
		  .buf = &buf, .len = 1 },
		{ .addr = priv->i2c_addr, .flags = I2C_M_RD,
		  .buf = regs, .len = 16 }
	};

	tda18271_i2c_gate_ctrl(fe, 1);

	/* read all registers */
	ret = i2c_transfer(priv->i2c_adap, msg, 2);

	tda18271_i2c_gate_ctrl(fe, 0);

	if (ret != 2)
		printk("ERROR: %s: i2c_transfer returned: %d\n",
		       __FUNCTION__, ret);

	if (tda18271_debug > 2)
		tda18271_dump_regs(fe);
}

static void tda18271_write_regs(struct dvb_frontend *fe, int idx, int len)
{
	struct tda18271_priv *priv = fe->tuner_priv;
	unsigned char *regs = priv->tda18271_regs;
	unsigned char buf[TDA18271_NUM_REGS+1];
	struct i2c_msg msg = { .addr = priv->i2c_addr, .flags = 0,
			       .buf = buf, .len = len+1 };
	int i, ret;

	BUG_ON((len == 0) || (idx+len > sizeof(buf)));

	buf[0] = idx;
	for (i = 1; i <= len; i++) {
		buf[i] = regs[idx-1+i];
	}

	tda18271_i2c_gate_ctrl(fe, 1);

	/* write registers */
	ret = i2c_transfer(priv->i2c_adap, &msg, 1);

	tda18271_i2c_gate_ctrl(fe, 0);

	if (ret != 1)
		printk(KERN_WARNING "ERROR: %s: i2c_transfer returned: %d\n",
		       __FUNCTION__, ret);
}

/*---------------------------------------------------------------------*/

static int tda18271_init_regs(struct dvb_frontend *fe)
{
	struct tda18271_priv *priv = fe->tuner_priv;
	unsigned char *regs = priv->tda18271_regs;

	printk(KERN_INFO "tda18271: initializing registers\n");

	/* initialize registers */
	regs[R_ID]   = 0x83;
	regs[R_TM]   = 0x08;
	regs[R_PL]   = 0x80;
	regs[R_EP1]  = 0xc6;
	regs[R_EP2]  = 0xdf;
	regs[R_EP3]  = 0x16;
	regs[R_EP4]  = 0x60;
	regs[R_EP5]  = 0x80;
	regs[R_CPD]  = 0x80;
	regs[R_CD1]  = 0x00;
	regs[R_CD2]  = 0x00;
	regs[R_CD3]  = 0x00;
	regs[R_MPD]  = 0x00;
	regs[R_MD1]  = 0x00;
	regs[R_MD2]  = 0x00;
	regs[R_MD3]  = 0x00;
	regs[R_EB1]  = 0xff;
	regs[R_EB2]  = 0x01;
	regs[R_EB3]  = 0x84;
	regs[R_EB4]  = 0x41;
	regs[R_EB5]  = 0x01;
	regs[R_EB6]  = 0x84;
	regs[R_EB7]  = 0x40;
	regs[R_EB8]  = 0x07;
	regs[R_EB9]  = 0x00;
	regs[R_EB10] = 0x00;
	regs[R_EB11] = 0x96;
	regs[R_EB12] = 0x0f;
	regs[R_EB13] = 0xc1;
	regs[R_EB14] = 0x00;
	regs[R_EB15] = 0x8f;
	regs[R_EB16] = 0x00;
	regs[R_EB17] = 0x00;
	regs[R_EB18] = 0x00;
	regs[R_EB19] = 0x00;
	regs[R_EB20] = 0x20;
	regs[R_EB21] = 0x33;
	regs[R_EB22] = 0x48;
	regs[R_EB23] = 0xb0;

	tda18271_write_regs(fe, 0x00, TDA18271_NUM_REGS);
	/* setup AGC1 & AGC2 */
	regs[R_EB17] = 0x00;
	tda18271_write_regs(fe, R_EB17, 1);
	regs[R_EB17] = 0x03;
	tda18271_write_regs(fe, R_EB17, 1);
	regs[R_EB17] = 0x43;
	tda18271_write_regs(fe, R_EB17, 1);
	regs[R_EB17] = 0x4c;
	tda18271_write_regs(fe, R_EB17, 1);

	regs[R_EB20] = 0xa0;
	tda18271_write_regs(fe, R_EB20, 1);
	regs[R_EB20] = 0xa7;
	tda18271_write_regs(fe, R_EB20, 1);
	regs[R_EB20] = 0xe7;
	tda18271_write_regs(fe, R_EB20, 1);
	regs[R_EB20] = 0xec;
	tda18271_write_regs(fe, R_EB20, 1);

	/* image rejection calibration */

	/* low-band */
	regs[R_EP3] = 0x1f;
	regs[R_EP4] = 0x66;
	regs[R_EP5] = 0x81;
	regs[R_CPD] = 0xcc;
	regs[R_CD1] = 0x6c;
	regs[R_CD2] = 0x00;
	regs[R_CD3] = 0x00;
	regs[R_MPD] = 0xcd;
	regs[R_MD1] = 0x77;
	regs[R_MD2] = 0x08;
	regs[R_MD3] = 0x00;

	tda18271_write_regs(fe, R_EP3, 11);
	msleep(5); /* pll locking */

	regs[R_EP1] = 0xc6;
	tda18271_write_regs(fe, R_EP1, 1);
	msleep(5); /* wanted low measurement */

	regs[R_EP3] = 0x1f;
	regs[R_EP4] = 0x66;
	regs[R_EP5] = 0x85;
	regs[R_CPD] = 0xcb;
	regs[R_CD1] = 0x66;
	regs[R_CD2] = 0x70;
	regs[R_CD3] = 0x00;

	tda18271_write_regs(fe, R_EP3, 7);
	msleep(5); /* pll locking */

	regs[R_EP2] = 0xdf;
	tda18271_write_regs(fe, R_EP2, 1);
	msleep(30); /* image low optimization completion */

	/* mid-band */
	regs[R_EP3] = 0x1f;
	regs[R_EP4] = 0x66;
	regs[R_EP5] = 0x82;
	regs[R_CPD] = 0xa8;
	regs[R_CD1] = 0x66;
	regs[R_CD2] = 0x00;
	regs[R_CD3] = 0x00;
	regs[R_MPD] = 0xa9;
	regs[R_MD1] = 0x73;
	regs[R_MD2] = 0x1a;
	regs[R_MD3] = 0x00;

	tda18271_write_regs(fe, R_EP3, 11);
	msleep(5); /* pll locking */

	regs[R_EP1] = 0xc6;
	tda18271_write_regs(fe, R_EP1, 1);
	msleep(5); /* wanted mid measurement */

	regs[R_EP3] = 0x1f;
	regs[R_EP4] = 0x66;
	regs[R_EP5] = 0x86;
	regs[R_CPD] = 0xa8;
	regs[R_CD1] = 0x66;
	regs[R_CD2] = 0xa0;
	regs[R_CD3] = 0x00;

	tda18271_write_regs(fe, R_EP3, 7);
	msleep(5); /* pll locking */

	regs[R_EP2] = 0xdf;
	tda18271_write_regs(fe, R_EP2, 1);
	msleep(30); /* image mid optimization completion */

	/* high-band */
	regs[R_EP3] = 0x1f;
	regs[R_EP4] = 0x66;
	regs[R_EP5] = 0x83;
	regs[R_CPD] = 0x98;
	regs[R_CD1] = 0x65;
	regs[R_CD2] = 0x00;
	regs[R_CD3] = 0x00;
	regs[R_MPD] = 0x99;
	regs[R_MD1] = 0x71;
	regs[R_MD2] = 0xcd;
	regs[R_MD3] = 0x00;

	tda18271_write_regs(fe, R_EP3, 11);
	msleep(5); /* pll locking */

	regs[R_EP1] = 0xc6;
	tda18271_write_regs(fe, R_EP1, 1);
	msleep(5); /* wanted high measurement */

	regs[R_EP3] = 0x1f;
	regs[R_EP4] = 0x66;
	regs[R_EP5] = 0x87;
	regs[R_CPD] = 0x98;
	regs[R_CD1] = 0x65;
	regs[R_CD2] = 0x50;
	regs[R_CD3] = 0x00;

	tda18271_write_regs(fe, R_EP3, 7);
	msleep(5); /* pll locking */

	regs[R_EP2] = 0xdf;

	tda18271_write_regs(fe, R_EP2, 1);
	msleep(30); /* image high optimization completion */

	regs[R_EP4] = 0x64;
	tda18271_write_regs(fe, R_EP4, 1);

	regs[R_EP1] = 0xc6;
	tda18271_write_regs(fe, R_EP1, 1);

	return 0;
}

static int tda18271_tune(struct dvb_frontend *fe,
			 u32 ifc, u32 freq, u32 bw, u8 std)
{
	struct tda18271_priv *priv = fe->tuner_priv;
	unsigned char *regs = priv->tda18271_regs;
	u32 div, N = 0;
	int i;

	tda18271_read_regs(fe);

	/* test IR_CAL_OK to see if we need init */
	if ((regs[R_EP1] & 0x08) == 0)
		tda18271_init_regs(fe);

#if 0
	/* FIXME: FM Radio support */
	if (t->mode == V4L2_TUNER_RADIO)
		freq = freq / 1000;
#endif

	dprintk(1, "freq = %d, ifc = %d\n", freq, ifc);

	/* RF tracking filter calibration */

	/* calculate BP_Filter */
	i = 0;
	while ((tda18271_bp_filter[i].rfmax * 1000) < freq) {
		if (tda18271_bp_filter[i + 1].rfmax == 0)
			break;
		i++;
	}
	dprintk(2, "bp filter = 0x%x, i = %d\n", tda18271_bp_filter[i].val, i);

	regs[R_EP1]  &= ~0x07; /* clear bp filter bits */
	regs[R_EP1]  |= tda18271_bp_filter[i].val;
	tda18271_write_regs(fe, R_EP1, 1);

	regs[R_EB4]  &= 0x07;
	regs[R_EB4]  |= 0x60;
	tda18271_write_regs(fe, R_EB4, 1);

	regs[R_EB7]   = 0x60;
	tda18271_write_regs(fe, R_EB7, 1);

	regs[R_EB14]  = 0x00;
	tda18271_write_regs(fe, R_EB14, 1);

	regs[R_EB20]  = 0xcc;
	tda18271_write_regs(fe, R_EB20, 1);

	/* set CAL mode to RF tracking filter calibration */
	regs[R_EB4]  |= 0x03;

	/* calculate CAL PLL */

	switch (priv->mode) {
	case TDA18271_ANALOG:
		N = freq - 1250000;
		break;
	case TDA18271_DIGITAL:
		N = freq + bw / 2;
		break;
	}

	i = 0;
	while ((tda18271_cal_pll[i].lomax * 1000) < N) {
		if (tda18271_cal_pll[i + 1].lomax == 0)
			break;
		i++;
	}
	dprintk(2, "cal pll, pd = 0x%x, d = 0x%x, i = %d\n",
		tda18271_cal_pll[i].pd, tda18271_cal_pll[i].d, i);

	regs[R_CPD]   = tda18271_cal_pll[i].pd;

	div =  ((tda18271_cal_pll[i].d * (N / 1000)) << 7) / 125;
	regs[R_CD1]   = 0xff & (div >> 16);
	regs[R_CD2]   = 0xff & (div >> 8);
	regs[R_CD3]   = 0xff & div;

	/* calculate MAIN PLL */

	switch (priv->mode) {
	case TDA18271_ANALOG:
		N = freq - 250000;
		break;
	case TDA18271_DIGITAL:
		N = freq + bw / 2 + 1000000;
		break;
	}

	i = 0;
	while ((tda18271_main_pll[i].lomax * 1000) < N) {
		if (tda18271_main_pll[i + 1].lomax == 0)
			break;
		i++;
	}
	dprintk(2, "main pll, pd = 0x%x, d = 0x%x, i = %d\n",
		tda18271_main_pll[i].pd, tda18271_main_pll[i].d, i);

	regs[R_MPD]   = (0x7f & tda18271_main_pll[i].pd);

	switch (priv->mode) {
	case TDA18271_ANALOG:
		regs[R_MPD]  &= ~0x08;
		break;
	case TDA18271_DIGITAL:
		regs[R_MPD]  |=  0x08;
		break;
	}

	div =  ((tda18271_main_pll[i].d * (N / 1000)) << 7) / 125;
	regs[R_MD1]   = 0xff & (div >> 16);
	regs[R_MD2]   = 0xff & (div >> 8);
	regs[R_MD3]   = 0xff & div;

	tda18271_write_regs(fe, R_EP3, 11);
	msleep(5); /* RF tracking filter calibration initialization */

	/* search for K,M,CO for RF Calibration */
	i = 0;
	while ((tda18271_km[i].rfmax * 1000) < freq) {
		if (tda18271_km[i + 1].rfmax == 0)
			break;
		i++;
	}
	dprintk(2, "km = 0x%x, i = %d\n", tda18271_km[i].val, i);

	regs[R_EB13] &= 0x83;
	regs[R_EB13] |= tda18271_km[i].val;
	tda18271_write_regs(fe, R_EB13, 1);

	/* search for RF_BAND */
	i = 0;
	while ((tda18271_rf_band[i].rfmax * 1000) < freq) {
		if (tda18271_rf_band[i + 1].rfmax == 0)
			break;
		i++;
	}
	dprintk(2, "rf band = 0x%x, i = %d\n", tda18271_rf_band[i].val, i);

	regs[R_EP2]  &= ~0xe0; /* clear rf band bits */
	regs[R_EP2]  |= (tda18271_rf_band[i].val << 5);

	/* search for Gain_Taper */
	i = 0;
	while ((tda18271_gain_taper[i].rfmax * 1000) < freq) {
		if (tda18271_gain_taper[i + 1].rfmax == 0)
			break;
		i++;
	}
	dprintk(2, "gain taper = 0x%x, i = %d\n",
		tda18271_gain_taper[i].val, i);

	regs[R_EP2]  &= ~0x1f; /* clear gain taper bits */
	regs[R_EP2]  |= tda18271_gain_taper[i].val;

	tda18271_write_regs(fe, R_EP2, 1);
	tda18271_write_regs(fe, R_EP1, 1);
	tda18271_write_regs(fe, R_EP2, 1);
	tda18271_write_regs(fe, R_EP1, 1);

	regs[R_EB4]  &= 0x07;
	regs[R_EB4]  |= 0x40;
	tda18271_write_regs(fe, R_EB4, 1);

	regs[R_EB7]   = 0x40;
	tda18271_write_regs(fe, R_EB7, 1);
	msleep(10);

	regs[R_EB20]  = 0xec;
	tda18271_write_regs(fe, R_EB20, 1);
	msleep(60); /* RF tracking filter calibration completion */

	regs[R_EP4]  &= ~0x03; /* set cal mode to normal */
	tda18271_write_regs(fe, R_EP4, 1);

	tda18271_write_regs(fe, R_EP1, 1);

	/* RF tracking filer correction for VHF_Low band */
	i = 0;
	while ((tda18271_rf_cal[i].rfmax * 1000) < freq) {
		if (tda18271_rf_cal[i].rfmax == 0)
			break;
		i++;
	}
	dprintk(2, "rf cal = 0x%x, i = %d\n", tda18271_rf_cal[i].val, i);

	/* VHF_Low band only */
	if (tda18271_rf_cal[i].rfmax != 0) {
		regs[R_EB14]   = tda18271_rf_cal[i].val;
		tda18271_write_regs(fe, R_EB14, 1);
	}

	/* Channel Configuration */

	switch (priv->mode) {
	case TDA18271_ANALOG:
		regs[R_EB22]  = 0x2c;
		break;
	case TDA18271_DIGITAL:
		regs[R_EB22]  = 0x37;
		break;
	}
	tda18271_write_regs(fe, R_EB22, 1);

	regs[R_EP1]  |= 0x40; /* set dis power level on */

	/* set standard */
	regs[R_EP3]  &= ~0x1f; /* clear std bits */

	/* see table 22 */
	regs[R_EP3]  |= std;

	regs[R_EP4]  &= ~0x03; /* set cal mode to normal */

	regs[R_EP4]  &= ~0x1c; /* clear if level bits */
	switch (priv->mode) {
	case TDA18271_ANALOG:
		regs[R_MPD]  &= ~0x80; /* IF notch = 0 */
		break;
	case TDA18271_DIGITAL:
		regs[R_EP4]  |= 0x04;
		regs[R_MPD]  |= 0x80;
		break;
	}

	regs[R_EP4]  &= ~0x80; /* turn this bit on only for fm */

	/* FIXME: image rejection validity EP5[2:0] */

	/* calculate MAIN PLL */
	N = freq + ifc;

	i = 0;
	while ((tda18271_main_pll[i].lomax * 1000) < N) {
		if (tda18271_main_pll[i + 1].lomax == 0)
			break;
		i++;
	}
	dprintk(2, "main pll, pd = 0x%x, d = 0x%x, i = %d\n",
		tda18271_main_pll[i].pd, tda18271_main_pll[i].d, i);

	regs[R_MPD]   = (0x7f & tda18271_main_pll[i].pd);
	switch (priv->mode) {
	case TDA18271_ANALOG:
		regs[R_MPD]  &= ~0x08;
		break;
	case TDA18271_DIGITAL:
		regs[R_MPD]  |= 0x08;
		break;
	}

	div =  ((tda18271_main_pll[i].d * (N / 1000)) << 7) / 125;
	regs[R_MD1]   = 0xff & (div >> 16);
	regs[R_MD2]   = 0xff & (div >> 8);
	regs[R_MD3]   = 0xff & div;

	tda18271_write_regs(fe, R_TM, 15);
	msleep(5);

	return 0;
}

/* ------------------------------------------------------------------ */

static int tda18271_set_params(struct dvb_frontend *fe,
			       struct dvb_frontend_parameters *params)
{
	struct tda18271_priv *priv = fe->tuner_priv;
	u8 std;
	u32 bw, sgIF = 0;

	u32 freq = params->frequency;

	priv->mode = TDA18271_DIGITAL;

	/* see table 22 */
	if (fe->ops.info.type == FE_ATSC) {
		switch (params->u.vsb.modulation) {
		case VSB_8:
		case VSB_16:
			std = 0x1b; /* device-specific (spec says 0x1c) */
			sgIF = 5380000;
			break;
		case QAM_64:
		case QAM_256:
			std = 0x18; /* device-specific (spec says 0x1d) */
			sgIF = 4000000;
			break;
		default:
			printk(KERN_WARNING "%s: modulation not set!\n",
			       __FUNCTION__);
			return -EINVAL;
		}
		freq += 1750000; /* Adjust to center (+1.75MHZ) */
		bw = 6000000;
	} else if (fe->ops.info.type == FE_OFDM) {
		switch (params->u.ofdm.bandwidth) {
		case BANDWIDTH_6_MHZ:
			std = 0x1b; /* device-specific (spec says 0x1c) */
			bw = 6000000;
			sgIF = 3300000;
			break;
		case BANDWIDTH_7_MHZ:
			std = 0x19; /* device-specific (spec says 0x1d) */
			bw = 7000000;
			sgIF = 3800000;
			break;
		case BANDWIDTH_8_MHZ:
			std = 0x1a; /* device-specific (spec says 0x1e) */
			bw = 8000000;
			sgIF = 4300000;
			break;
		default:
			printk(KERN_WARNING "%s: bandwidth not set!\n",
			       __FUNCTION__);
			return -EINVAL;
		}
	} else {
		printk(KERN_WARNING "%s: modulation type not supported!\n",
		       __FUNCTION__);
		return -EINVAL;
	}

	return tda18271_tune(fe, sgIF, freq, bw, std);
}

static int tda18271_set_analog_params(struct dvb_frontend *fe,
				      struct analog_parameters *params)
{
	struct tda18271_priv *priv = fe->tuner_priv;
	u8 std;
	unsigned int sgIF;
	char *mode;

	priv->mode = TDA18271_ANALOG;

	/* see table 22 */
	if (params->std & V4L2_STD_MN) {
		std = 0x0d;
		sgIF =  92;
		mode = "MN";
	} else if (params->std & V4L2_STD_B) {
		std = 0x0e;
		sgIF =  108;
		mode = "B";
	} else if (params->std & V4L2_STD_GH) {
		std = 0x0f;
		sgIF =  124;
		mode = "GH";
	} else if (params->std & V4L2_STD_PAL_I) {
		std = 0x0f;
		sgIF =  124;
		mode = "I";
	} else if (params->std & V4L2_STD_DK) {
		std = 0x0f;
		sgIF =  124;
		mode = "DK";
	} else if (params->std & V4L2_STD_SECAM_L) {
		std = 0x0f;
		sgIF =  124;
		mode = "L";
	} else if (params->std & V4L2_STD_SECAM_LC) {
		std = 0x0f;
		sgIF =  20;
		mode = "LC";
	} else {
		std = 0x0f;
		sgIF =  124;
		mode = "xx";
	}

	if (params->mode == V4L2_TUNER_RADIO)
		sgIF =  88; /* if frequency is 5.5 MHz */

	dprintk(1, "setting tda18271 to system %s\n", mode);

	return tda18271_tune(fe, sgIF * 62500, params->frequency * 62500,
			     0, std);
}

static int tda18271_release(struct dvb_frontend *fe)
{
	kfree(fe->tuner_priv);
	fe->tuner_priv = NULL;
	return 0;
}

static int tda18271_get_frequency(struct dvb_frontend *fe, u32 *frequency)
{
	struct tda18271_priv *priv = fe->tuner_priv;
	*frequency = priv->frequency;
	return 0;
}

static int tda18271_get_bandwidth(struct dvb_frontend *fe, u32 *bandwidth)
{
	struct tda18271_priv *priv = fe->tuner_priv;
	*bandwidth = priv->bandwidth;
	return 0;
}

static struct dvb_tuner_ops tda18271_tuner_ops = {
	.info = {
		.name = "NXP TDA18271HD",
		.frequency_min  =  45000000,
		.frequency_max  = 864000000,
		.frequency_step =     62500
	},
	.init              = tda18271_init_regs,
	.set_params        = tda18271_set_params,
	.set_analog_params = tda18271_set_analog_params,
	.release           = tda18271_release,
	.get_frequency     = tda18271_get_frequency,
	.get_bandwidth     = tda18271_get_bandwidth,
};

struct dvb_frontend *tda18271_attach(struct dvb_frontend *fe, u8 addr,
				     struct i2c_adapter *i2c)
{
	struct tda18271_priv *priv = NULL;

	dprintk(1, "@ %d-%04x\n", i2c_adapter_id(i2c), addr);
	priv = kzalloc(sizeof(struct tda18271_priv), GFP_KERNEL);
	if (priv == NULL)
		return NULL;

	priv->i2c_addr = addr;
	priv->i2c_adap = i2c;

	memcpy(&fe->ops.tuner_ops, &tda18271_tuner_ops,
	       sizeof(struct dvb_tuner_ops));

	fe->tuner_priv = priv;

	return fe;
}
EXPORT_SYMBOL_GPL(tda18271_attach);
MODULE_DESCRIPTION("NXP TDA18271HD analog / digital tuner driver");
MODULE_AUTHOR("Michael Krufky <mkrufky@linuxtv.org>");
MODULE_LICENSE("GPL");

/*
 * Overrides for Emacs so that we follow Linus's tabbing style.
 * ---------------------------------------------------------------------------
 * Local variables:
 * c-basic-offset: 8
 * End:
 */
