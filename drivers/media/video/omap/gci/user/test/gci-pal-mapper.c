/*
 * drivers/media/video/omap/gci/user/test/gci-pal-mapper.c
 *
 * GCI-PAL-MAPPER kind of module for testing
 * TI's OMAP3430 Camera ISP - GCI solution
 *
 * Copyright (C) 2007 Texas Instruments.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published 
 * by the Free Software Foundation version 2.1 of the License.
 *
 * This program is distributed .as is. WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 */

//#define RAW_5MP_PARLL_GR2
//#undef SMART_3MP_PARLL_OV
//#undef RAW_3MP_PARLL_ES1

#include <stdio.h>
#include <stdlib.h>
#include <asm/arch/gci/isc_cmd.h>

#ifdef RAW_3MP_PARLL_ES1
#include "mt9t012_config.h"
#define SMART_SENSOR 0
#endif

#ifdef RAW_5MP_PARLL_GR2
#include "mt9p012_config.h"
#define RAW_5MP_PARLL_GR2_FOCUS
#define SMART_SENSOR 0
#endif

#ifdef SMART_3MP_PARLL_OV
#include "ov3640_config.h"
#define SMART_SENSOR 1
#endif


#include "../paluser.h"
#include "gci-pal-mapper.h"

#if 1
#define GCI_PRINT(format,...)\
	printf("GCI: " format, ## __VA_ARGS__)
#else
#define GCI_PRINT(format,...)
#endif



#ifdef SMART_3MP_PARLL_OV
#define SENSOR_MAX_WIDTH 2048
#define SENSOR_MAX_HEIGHT 1536
#endif


#ifdef RAW_5MP_PARLL_GR2
#define SENSOR_MAX_WIDTH 2048
#define SENSOR_MAX_HEIGHT 1536
#endif

#ifdef RAW_3MP_PARLL_ES1
#define SENSOR_MAX_WIDTH 2048
#define SENSOR_MAX_HEIGHT 1536
#endif

#define CAP_SHOTS 4
#define CAP_DELAY_1STSHOT 2
#define CAP_DELAY_INTERSHOT 3
int sensor_data_width;
int sensor_reg_width;

int detected_prim_sensor = 0;
int detected_sec_sensor;
ISE_PAL_REG_HANDLE_T pal_reg_handle;

static unsigned char sensor_fmt;
static unsigned int cip_start = 0;
static unsigned char prv_fmt_val = 0;
static ISE_USE_CASE_T use_case;
static ISE_CAPTURE_STATE_T cap_state;
static ISE_DIMENSION_T prv_size = {176,144};
static ISE_DIMENSION_T img_size = {640,480};
static ISE_DIMENSION_T sensor_prv_size;
static ISE_DIMENSION_T sensor_img_size;
static unsigned int color_effect;
static ISE_POWER_STATE_T pwr_st;
static unsigned short brightness_level;
static unsigned short contrast_level;
static ISE_COLOR_FMT_T prv_fmt = 0;
static ISE_COLOR_FMT_T cap_fmt = 0;

static ISE_IMAGER_CONFIG_CALLBACK_T *cip_callback;
static ISE_IMAGER_CONFIG_T cip_config_data;

/* Used to generate PAL transactions when there is 
 * change in cipdata */
static ISE_IMAGER_CONFIG_T curr_cip_data = {
	ISE_CIP_STATE_STOP, /* cip_state */
	0x0,/* clock_freq_khz */
	0x0,/* win_hoffset */
	0x0,/* win_voffset */
	176,/* win_width */
	144,/* win_height */
	-1,/* color_fmt */
	-1,/* hsync_prop */
	-1,/* vsync_prop */
	-1,/* pclk_prop */
	0x0/* drive_strength */
};

static unsigned int zoom_magnitude = 1;
static unsigned int sensor_fv_width = SENSOR_MAX_WIDTH;
static unsigned int sensor_fv_height = SENSOR_MAX_HEIGHT;
static unsigned int left = 0;
static unsigned int top = 0;

static ISE_PAL_REG_CONFIG_T palreg_config_imager0 = {
	0, 			/* prevent block */
	ISE_PAL_REG_TYPE_I2C, 	/* Register type */
	0 			/* i2c_config_t */
};

static ISE_PAL_REG_CONFIG_T palreg_config_imager0_focus = {
	0, 			/* prevent block */
	ISE_PAL_REG_TYPE_I2C, 	/* Register type */
	0 			/* i2c_config_t */
};

static ISE_PAL_REG_CONFIG_T palreg_config_camtrans = {
	0, /* prevent block */
	ISE_PAL_REG_TYPE_CAM, /* Register type */
	0 /* i2c_config_t */
};

/* Assumes that register address is always 8 bit 
 * for CAM register address and 16 bit for sensor address
 * Checks for the data width to fill up the data
*/
static
void make_palregtrans(ISE_PAL_REG_TRANS_T *regtrans,__u32 cmd_addr,
				__u8 cam, __u32 data, __u8 data_width,
				__u8 attr)
{
	__u8 i = 0;
	regtrans->data = (__u8*)malloc(sizeof(__u8) * (data_width + 1));
	if(cam){
		/* +1 for 8 bit CAM address */
		regtrans->size = data_width + 1; 
		regtrans->data[i++] = GET_ADDRESS(cmd_addr);
	}
	else{
		if (sensor_reg_width == SENSOR_WIDTH16) {
			/* +2 for 16 bit Sensor address  +1 for datawidth*/
			regtrans->size = data_width + 2 + 1; 
			/* data[0] should have the data width */
			regtrans->data[i++] = data_width; 
			regtrans->data[i++] = cmd_addr & 0xFF;
			regtrans->data[i++] = (cmd_addr >> 8) & 0xFF;
		}
		if (sensor_reg_width == SENSOR_WIDTH8) {
			/* +2 for 16 bit Sensor address  +1 for datawidth*/
			regtrans->size = data_width + 1 + 1; 
			/* data[0] should have the data width */
			regtrans->data[i++] = data_width; 
			regtrans->data[i++] = cmd_addr & 0xFF;
		}
	}
	
	if (data_width == 1) {
		regtrans->data[i++] = (__u8) (data & 0xFF);
	}
	else if (data_width == 2) {
		regtrans->data[i++] = (__u8) (data & 0xFF);
		regtrans->data[i++] = (__u8) (data >> 8);
	}
	else if (data_width == 4){
		regtrans->data[i++] = (__u8) (data & 0xFF);
		regtrans->data[i++] = (__u8) (data >> 8) & 0xFF;
		regtrans->data[i++] = (__u8) (data >>16) & 0xFF;
		regtrans->data[i++] = (__u8) (data >>24) & 0xFF;
	}
	regtrans->trans_type = attr;
}

static
int debug_sensor_trans(unsigned int data[],int size)
{
	int j = 0,i = 0,ret;
	ISE_PAL_REG_TRANS_T reg_trans[size];
	unsigned char reg_format;
	for (j=0;j<size;j+=4) {
		if ((data[j+2] == SENSOR_WIDTH8) && 
				(sensor_data_width !=SENSOR_WIDTH8)) {
			sensor_data_width = SENSOR_WIDTH8;
			reg_format = (sensor_reg_width <<
				       	CAM_PASST_FORMAT_ADDR_WIDTH_SHIFT) |
	       				(sensor_data_width <<
					 CAM_PASST_FORMAT_DATA_WIDTH_SHIFT);
			make_palregtrans(&reg_trans[i++],CAM_PASST_FORMAT,1,reg_format,1,PAL_WRITE);
		}
		if ((data[j+2] == SENSOR_WIDTH16) && 
				(sensor_data_width !=SENSOR_WIDTH16)) {
			sensor_data_width = SENSOR_WIDTH16;
			reg_format = (sensor_reg_width <<
				       	CAM_PASST_FORMAT_ADDR_WIDTH_SHIFT) |
	       				(sensor_data_width <<
					 CAM_PASST_FORMAT_DATA_WIDTH_SHIFT);
			make_palregtrans(&reg_trans[i++],CAM_PASST_FORMAT,1,reg_format,1,PAL_WRITE);
		}
		make_palregtrans(&reg_trans[i++],CAM_PASST_REG_ADDR_L,1,data[j],2,PAL_WRITE);
		make_palregtrans(&reg_trans[i++],CAM_PASST_REG_DATA_L,1,data[j+1],data[j+2],data[j + 3]);
	}
	ret = ISE_PAL_reg_trans(pal_reg_handle,reg_trans,i);
	return ret;
}

int issue_sensortrans_reg16_data16(unsigned int data[], int size)
{
	int j = 0,i = 0,ret = 0;
	ISE_PAL_REG_TRANS_T reg_trans[size];
	for (j=0;j<size;j+=4) {
		make_palregtrans(&reg_trans[i++],data[j],0,data[j+1],
				data[j+2],data[j+3]);
	}
	ret = ISE_PAL_reg_trans(pal_reg_handle,reg_trans,i);
	return ret;
}

static int
issue_camtrans_cip_syncsignals(unsigned int hsync, unsigned int vsync,
				unsigned int pclk)
{
	ISE_PAL_REG_TRANS_T data[10];
	int i =0,ret;
	unsigned int ccdc_sync_mode_val = 0, ispif_ctrl_val = 0;
	
	if(hsync == ISE_SIGNAL_PROP_ACTIVE_HIGH)
	ccdc_sync_mode_val &= CAM_CCDC_SYNCMODE_HDPOL;
	else
	ccdc_sync_mode_val |= CAM_CCDC_SYNCMODE_HDPOL;
	
	if(vsync == ISE_SIGNAL_PROP_ACTIVE_HIGH)
	ccdc_sync_mode_val &= CAM_CCDC_SYNCMODE_VDPOL;
	else
	ccdc_sync_mode_val |= CAM_CCDC_SYNCMODE_VDPOL;

	/* Only for Parallel interface */
	if(pclk == ISE_SIGNAL_PROP_RISING_EDGE)
	ispif_ctrl_val &= CAM_ISPIF_CTRL_PAR_CLK_POL;
	else
	ispif_ctrl_val |= CAM_ISPIF_CTRL_PAR_CLK_POL;

	make_palregtrans(&(data[i++]),PAGE_SWITCH,1,PAGEC,1,FALSE);
#ifdef SMART_3MP_PARLL_OV
	make_palregtrans(&data[i++],CAM_CCDC_SYNCMODE,1,0x31720,4,TRUE);
#else
	make_palregtrans(&data[i++],CAM_CCDC_SYNCMODE,1,
				ccdc_sync_mode_val,4,TRUE);
#endif
	make_palregtrans(&(data[i++]),PAGE_SWITCH,1,PAGEE,1,FALSE);
#ifdef SMART_3MP_PARLL_OV
	make_palregtrans(&data[i++],CAM_ISPIF_CTRL,1,0x29C18C,4,TRUE);
#else
	make_palregtrans(&data[i++],CAM_ISPIF_CTRL,1,
				ispif_ctrl_val,4,TRUE);
#endif
	ret = ISE_PAL_reg_trans(pal_reg_handle,data, i);
	return ret;
}

static int
issue_camtrans_cip_init(void)
{
	ISE_PAL_REG_TRANS_T data[20];
	int i =0,ret;
	/*Initialisation commands */
	make_palregtrans(&(data[i++]),PAGE_SWITCH,1,PAGE0,1,FALSE);
	make_palregtrans(&data[i++],CAM_SENSOR_SELECT,1,
				PRIMARY_SENSOR,1,FALSE);

	ret = ISE_PAL_reg_trans(pal_reg_handle,data, i);
	return ret;
}
static int
issue_camtrans_cip_cap_time(unsigned int cap_shots,
			unsigned int delay_1stshot,
			unsigned int delay_intershot)
{
	ISE_PAL_REG_TRANS_T data[10];
	int i =0 , ret;

	make_palregtrans(&(data[i++]),PAGE_SWITCH,1,PAGE1,1,FALSE);
	make_palregtrans(&data[i++],CAM_IMG_CAP_SHOTS,1,cap_shots,1,FALSE);
	make_palregtrans(&data[i++],CAM_IMG_CAP_DELAY_FRAME,1,
				delay_1stshot,1,FALSE);
	make_palregtrans(&data[i++],CAM_IMG_CAP_PERIOD,1,
				delay_intershot,1,FALSE);

	ret = ISE_PAL_reg_trans(pal_reg_handle,data, i);
	return ret;
}
static int
issue_camtrans_cip_cap_resln(unsigned int width,unsigned int height)
{
	ISE_PAL_REG_TRANS_T data[10];
	int i =0 , ret;

	make_palregtrans(&(data[i++]),PAGE_SWITCH,1,PAGE1,1,FALSE);
	make_palregtrans(&data[i++],CAM_IMG_CAP_WIDTH_H,1,
				width>>0x8,1,FALSE);
	make_palregtrans(&data[i++],CAM_IMG_CAP_WIDTH_L,1,
				(width&0xFF),1,FALSE);

	make_palregtrans(&data[i++],CAM_IMG_CAP_HEIGHT_H,1,
				height>>0x8,1,FALSE);
	make_palregtrans(&data[i++],CAM_IMG_CAP_HEIGHT_L,1,
				(height&0xFF),1,FALSE);
	/* TBD Don't do this and see if the prepared values are updated
	make_palregtrans(&data[i++],CMD_FE,1,0x01,1,FALSE); */
	img_size.width = width; img_size.height = height;
	ret = ISE_PAL_reg_trans(pal_reg_handle,data, i);
	return ret;
}

static int
issue_camtrans_cip_sensor_resln(unsigned int width,unsigned int height)
{
	ISE_PAL_REG_TRANS_T data[10];
	int i, ret;

	i =0;
	make_palregtrans(&data[i++],PAGE_SWITCH,1,PAGE1,1,FALSE);
	/*
	* Approx VIEWAREA_WIDTH = 1024;
	* 0x400 = 0x04 - H, 0x00 -L
	*/
	make_palregtrans(&data[i++],CAM_VIEWAREA_WIDTH_H,1,
				(width >> 8),1,FALSE);
	make_palregtrans(&data[i++],CAM_VIEWAREA_WIDTH_L,1,
				(width & 0xFF),1,FALSE);
	/*
	* Approx VIEWAREA_HEIGHT = 768
	* 0x300 = 0x03 - H, 0x00 - L
	*/
	make_palregtrans(&data[i++],CAM_VIEWAREA_HEIGHT_H,1,
				(height >> 8),1,FALSE);
	make_palregtrans(&data[i++],CAM_VIEWAREA_HEIGHT_L,1,
				(height & 0xFF),1,FALSE);

	ret = ISE_PAL_reg_trans(pal_reg_handle,data, i);
	return ret;
}

/* This calculation is for raw sensor 
 * The sensor output size is calculated limiting  to the binning 
 * and scaling that could be done in the sensor*/
static int
issue_camtrans_get_rawsensor_resln(ISE_DIMENSION_T *sensor_setsize,
			unsigned int *left, unsigned int *top)
{
	ISE_PAL_REG_TRANS_T data[10];
	int i =0 , ret;
	unsigned int width, height;

	if(cap_state == ISE_CAPTURE_STATE_CAPTURE){
		width = img_size.width;
		height = img_size.height;
	}
	else if(cap_state == ISE_CAPTURE_STATE_PREVIEW){
		width = prv_size.width;
		height = prv_size.height;
	}

	if(zoom_magnitude != 1)
		;
		/* TBD Have to back calculate sensor output size
		 * for the prv_size expected from resizer
		 */
	else{
#ifdef RAW_3MP_PARLL_ES1
	get_mt9t012_resln(width, height,&(sensor_setsize->width),
				&(sensor_setsize->height));
#endif
#ifdef RAW_5MP_PARLL_GR2
	get_mt9p012_resln(width, height,&(sensor_setsize->width),
				&(sensor_setsize->height));
#endif
	}
	GCI_PRINT("\nraw va_w = %d,va_h = %d, left = %d,top = %d",
				sensor_setsize->width,
				sensor_setsize->height,*left,*top);
	make_palregtrans(&data[i++],PAGE_SWITCH,1,PAGE1,1,FALSE);
	/*
	* Get exact VIEWAREA_WIDTH ;
	* 0x400 = 0x04 - H, 0x00 -L
	*/
	make_palregtrans(&data[i++],CAM_VIEWAREA_WIDTH_H,1,
				sensor_setsize->width >> 8,1,FALSE);
	make_palregtrans(&data[i++],CAM_VIEWAREA_WIDTH_L,1,
				sensor_setsize->width & 0xFF,1,FALSE);
	/*
	* Get exact VIEWAREA_HEIGHT
	* 0x300 = 0x03 - H, 0x00 - L
	*/
	make_palregtrans(&data[i++],CAM_VIEWAREA_HEIGHT_H,1,
				sensor_setsize->height >> 8,1,FALSE);
	make_palregtrans(&data[i++],CAM_VIEWAREA_HEIGHT_L,1,
				sensor_setsize->height & 0xFF,1,FALSE);
	/*
	* SENSOR_WIDTH ;
	* 0x200 = 0x20 - H, 0x00 -L
	*/
	make_palregtrans(&data[i++],CAM_SENSOR_WIDTH_H,1,
				sensor_setsize->width >> 8,1,FALSE);
	make_palregtrans(&data[i++],CAM_SENSOR_WIDTH_L,1,
				sensor_setsize->width & 0xFF,1,FALSE);
	/*
	* SENSOR_HEIGHT ;
	* 0x180 = 0x18 - H, 0x00 -L
	*/
	make_palregtrans(&data[i++],CAM_SENSOR_HEIGHT_H,1,
				sensor_setsize->height >> 8,1,FALSE);
	make_palregtrans(&data[i++],CAM_SENSOR_HEIGHT_L,1,
				sensor_setsize->height & 0xFF,1,FALSE);

	ret = ISE_PAL_reg_trans(pal_reg_handle,data, i);
}

/* The Zoom calculations are for Smart sensor support 
 * The view area size ,preview size, imagecapture size are all
 * kept same since CCDC is not supposed to crop anything.
 * Any changes needed for Zoom will be done in view area size itself.*/
static int
issue_camtrans_get_smartsensor_resln(ISE_DIMENSION_T *sensor_setsize,
			unsigned int *left, unsigned int *top)
{
	ISE_PAL_REG_TRANS_T data[10];
	int i =0 , ret;
	unsigned int va_width, va_height,width, height;

	/* We know that view area = prv/img size since no croppring
	 * done in the CIP or camcfg module */
	if(cap_state == ISE_CAPTURE_STATE_CAPTURE){
		va_width = img_size.width;
		va_height = img_size.height;
	}
	else if(cap_state == ISE_CAPTURE_STATE_PREVIEW){
		va_width = prv_size.width;
		va_height = prv_size.height;
	}

	if(((zoom_magnitude * va_width) <= sensor_fv_width)
		&&((zoom_magnitude * va_height) <= sensor_fv_height)){
		width = zoom_magnitude * va_width;
		height = zoom_magnitude * va_height;
	}else{
		width = sensor_fv_width;
		height = sensor_fv_height;
		va_width = sensor_fv_width/zoom_magnitude;
		va_height = sensor_fv_height/zoom_magnitude;
	}
	/* Crop w.r.t center zoom */
	*left = (width - va_width)/2;
	*top = (height - va_height)/2;

	GCI_PRINT("\n smart va_w = %d,va_h = %d, left = %d,top = %d",
				va_width,va_height,left,top);
	
	make_palregtrans(&data[i++],PAGE_SWITCH,1,PAGE1,1,FALSE);
	/*
	* Get exact VIEWAREA_WIDTH ;
	* 0x400 = 0x04 - H, 0x00 -L
	*/
	make_palregtrans(&data[i++],CAM_VIEWAREA_WIDTH_H,1,
				va_width >> 8,1,FALSE);
	make_palregtrans(&data[i++],CAM_VIEWAREA_WIDTH_L,1,
				va_width & 0xFF,1,FALSE);
	/*
	* Get exact VIEWAREA_HEIGHT
	* 0x300 = 0x03 - H, 0x00 - L
	*/
	make_palregtrans(&data[i++],CAM_VIEWAREA_HEIGHT_H,1,
				va_height >> 8,1,FALSE);
	make_palregtrans(&data[i++],CAM_VIEWAREA_HEIGHT_L,1,
				va_height & 0xFF,1,FALSE);
	/*
	* SENSOR_WIDTH ;
	* 0x200 = 0x20 - H, 0x00 -L
	*/
	make_palregtrans(&data[i++],CAM_SENSOR_WIDTH_H,1,
				va_width >> 8,1,FALSE);
	make_palregtrans(&data[i++],CAM_SENSOR_WIDTH_L,1,
				va_width & 0xFF,1,FALSE);
	/*
	* SENSOR_HEIGHT ;
	* 0x180 = 0x18 - H, 0x00 -L
	*/
	make_palregtrans(&data[i++],CAM_SENSOR_HEIGHT_H,1,
				va_height >> 8,1,FALSE);
	make_palregtrans(&data[i++],CAM_SENSOR_HEIGHT_L,1,
				va_height & 0xFF,1,FALSE);

	if( (va_width != prv_size.width) ||
		(va_height != prv_size.height) )
			;
		/* TBD intimate change in prv_size to camcfg and 
		 * also to OMX as Actual resolution */

	if( (va_width != img_size.width) ||
		(va_height != img_size.height) )
			;
		/* TBD intimate change in img_size to camcfg and 
		 * also to OMX as Actual resolution */
	ret = ISE_PAL_reg_trans(pal_reg_handle,data, i);
	
	sensor_setsize->width = va_width;
	sensor_setsize->height = va_height;

	return ret;
}

static int
issue_camtrans_cip_fmt(ISE_COLOR_FMT_T fmt,unsigned char isprv_fmt)
{
	ISE_PAL_REG_TRANS_T data[10];
	int i = 0,ret;
	int sensor_fmt_val = 0;

	switch(fmt)
	{
		case ISE_COLOR_FMT_422_YUYV:
			prv_fmt_val = 0x0 << CAM_PRV_FORMAT_L_PRV_YUV_FORMAT_SHIFT;
		break;
		case ISE_COLOR_FMT_422_YVYU:
		break;
		case ISE_COLOR_FMT_422_UYVY:
		break;
		case ISE_COLOR_FMT_422_VYUY:
		break;
		case ISE_COLOR_FMT_422_PLANAR:
		case ISE_COLOR_FMT_420_PLANAR:
		ret = -1; /* not Supported */
		break;
		case ISE_COLOR_FMT_RGB_565:
		break;
		case ISE_COLOR_FMT_BAYER_RGGB:
		break;
		case ISE_COLOR_FMT_BAYER_GRBG:
		break;
		case ISE_COLOR_FMT_BAYER_GBRG:
		break;
		case ISE_COLOR_FMT_BAYER_BGGR:
		break;
		case ISE_COLOR_FMT_ENC_JFIF:
		case ISE_COLOR_FMT_ENC_JPEG:
		break;
	};
	if(sensor_fmt == ISE_COLOR_FMT_BAYER_BGGR)
		 /* 0 - Bayer 10 */
		sensor_fmt_val = SENSOR_FORMAT_BAYER10 <<
				CAM_SENSOR_FORMAT_SENSOR_FORMAT_SHIFT;
	else if(sensor_fmt == ISE_COLOR_FMT_422_YUYV)
		sensor_fmt_val = SENSOR_FORMAT_YUV422 << 
				CAM_SENSOR_FORMAT_SENSOR_FORMAT_SHIFT;

	GCI_PRINT("sensor_fmt = %d, sensor_fmt_val = %d", 
				sensor_fmt, sensor_fmt_val);

	make_palregtrans(&data[i++],PAGE_SWITCH,1,PAGE1,1,FALSE);
	make_palregtrans(&data[i++],CAM_SENSOR_FORMAT,1,
				sensor_fmt_val,1,FALSE);

	GCI_PRINT("prv_fmt_val = 0x%x",prv_fmt_val);

	if(isprv_fmt){
	/*
	* PRV_FORMAT - YUV422, UYVY, Disable Preview
	* 0x00 -H, 0x00 -L
	*/
	prv_fmt = fmt;
	make_palregtrans(&data[i++],CAM_PRV_FORMAT_H,1,0x00,1,FALSE);
	make_palregtrans(&data[i++],CAM_PRV_FORMAT_L,1,
				0/*(prv_fmt_val & 0xFE)*/,1,FALSE);
	}
	else{
	cap_fmt = fmt;
	make_palregtrans(&data[i++],CAM_IMG_CAP_FORMAT_H,1,0x00,1,FALSE);
	make_palregtrans(&data[i++],CAM_IMG_CAP_FORMAT_L,1,0x00,1,FALSE);
	}
	make_palregtrans(&data[i++],CMD_FE,1,0x01,1,FALSE);
	ret = ISE_PAL_reg_trans(pal_reg_handle,data, i);
	return ret;
}

static int
issue_camtrans_sensor_page()
{
	ISE_PAL_REG_TRANS_T data[10];
	int ret,i=0, pma_val;
	unsigned char regaddr_width, regdata_width, reg_format;

#ifdef RAW_3MP_PARLL_ES1
	regaddr_width = SENSOR_WIDTH16;
	regdata_width = SENSOR_WIDTH16;
	sensor_data_width = SENSOR_WIDTH16;
	/* Assume 216MHz as Cam_mclk and 12Mhz to the imager0 */
	pma_val = 0x12;
#endif
#ifdef RAW_5MP_PARLL_GR2
	regaddr_width = SENSOR_WIDTH16;
	regdata_width = SENSOR_WIDTH16;
	sensor_data_width = SENSOR_WIDTH16;
	/* Assume 216MHz as Cam_mclk and 12Mhz to the imager0 */
	pma_val = 0x12;
#endif
#ifdef SMART_3MP_PARLL_OV
	regaddr_width = SENSOR_WIDTH16;
	regdata_width = SENSOR_WIDTH8;
	sensor_data_width = SENSOR_WIDTH8;
	/* Assume 216MHz as Cam_mclk and 12Mhz to the imager0 */
	pma_val = 0x12;
#endif

	reg_format = (regaddr_width << CAM_PASST_FORMAT_ADDR_WIDTH_SHIFT) |
	       		(regdata_width << CAM_PASST_FORMAT_DATA_WIDTH_SHIFT);
	make_palregtrans(&data[i++],PAGE_SWITCH,1,PAGE6,1,PAL_WRITE);
	make_palregtrans(&data[i++],CAM_PASST_FORMAT,1,reg_format,1,PAL_WRITE);
	make_palregtrans(&data[i++],PAGE_SWITCH,1,PAGE0,1,PAL_WRITE);
	make_palregtrans(&data[i++],CAM_PMA,1,pma_val,1,PAL_WRITE);

	ret = ISE_PAL_reg_trans(pal_reg_handle,data, i);
	sensor_reg_width = SENSOR_WIDTH16;
	return ret;
}

static int
i2c_change_reg_size(int regaddr_width)
{
	ISE_PAL_REG_TRANS_T data[2];
	int ret,i=0;
	unsigned char reg_format;
	reg_format = (regaddr_width << CAM_PASST_FORMAT_ADDR_WIDTH_SHIFT) |
	       	(sensor_data_width << CAM_PASST_FORMAT_DATA_WIDTH_SHIFT);
	make_palregtrans(&data[i++],PAGE_SWITCH,1,PAGE6,1,PAL_WRITE);
	make_palregtrans(&data[i++],CAM_PASST_FORMAT,1,reg_format,1,PAL_WRITE);
	ret = ISE_PAL_reg_trans(pal_reg_handle,data, i);
	sensor_reg_width = regaddr_width;
	return ret;
}

static int 
issue_sensortrans_reset()
{
	int ret = 0;
#ifdef RAW_5MP_PARLL_GR2
	ret = mt9p012_reset();
#endif
#ifdef RAW_3MP_PARLL_ES1
	ret = mt9t012_reset();
#endif
#ifdef SMART_3MP_PARLL_OV
	ret = ov3640_reset();
	ISE_PAL_udelay(5 * 1000);
#endif
	return ret;
}

static int
issue_sensortrans_initial_config()
{
	int ret = 0;
#ifdef RAW_5MP_PARLL_GR2
	ret = mt9p012_initial_config();
#endif
#ifdef RAW_3MP_PARLL_ES1
	ret = mt9t012_initial_config();
#endif
#ifdef SMART_3MP_PARLL_OV
	ret = ov3640_initial_config();
#endif
	return ret;
}

static int
issue_sensortrans_focus_initial_config() {
	int ret = 0;
#ifdef RAW_5MP_PARLL_GR2
	ret = mt9p012_focus_initial_config();
#endif
	return ret;
}

static int
issue_sensortrans_preview_config(unsigned int width, unsigned height,
			unsigned int left, unsigned int top)
{
	int ret = 0;
#ifdef RAW_5MP_PARLL_GR2
	ret = mt9p012_preview_config(width,height, left, top);
#endif
#ifdef RAW_3MP_PARLL_ES1
	ret = mt9t012_preview_config(width,height, left, top);
#endif
#ifdef SMART_3MP_PARLL_OV
	ret = ov3640_preview_config(width,height, left, top);
	ISE_PAL_udelay(200 * 1000);
#endif
	return ret;
}

static int
issue_sensortrans_brightness(unsigned short brightness_level)
{
	int ret = 0;
#ifdef SMART_3MP_PARLL_OV
	if(brightness_level)
	ret = ov3640_brightness_config(brightness_level);
#endif
	return ret;
}

static int
issue_sensortrans_contrast(unsigned short contrast_level)
{
	int ret = 0;
#ifdef SMART_3MP_PARLL_OV
	if(contrast_level)
	ret = ov3640_contrast_config(contrast_level);
#endif
	return ret;
}

static int
issue_sensortrans_coloreffect(unsigned short color)
{
	int ret = 0;
#ifdef SMART_3MP_PARLL_OV
	if(color)
	ret = ov3640_coloreffect_config(color);
#endif
	return ret;
}


static void 
config_gpio()
{
	unsigned char val;
#ifdef RAW_3MP_PARLL_ES1
	ISE_PAL_gpio_set(T2_VAUX_DEV_GRP_P1, TRUE, NULL);
	ISE_PAL_gpio_set(T2_VAUX_2_8_V, TRUE, NULL);
	ISE_PAL_gpio_set(ISE_PAL_GPIO_IMGR0_RST, TRUE, NULL);
#endif
#ifdef RAW_5MP_PARLL_GR2
	ISE_PAL_gpio_set(CAMERA_STANDBY_GPIO , FALSE, NULL);
	ISE_PAL_gpio_set(FPGA_BASE, TRUE, NULL);
	ISE_PAL_gpio_set(T2_VAUX_DEV_GRP_P1, TRUE, NULL);
	ISE_PAL_gpio_set(T2_VAUX_2_8_V, TRUE, NULL);
	ISE_PAL_gpio_set(ISE_PAL_GPIO_IMGR0_RST, FALSE, NULL);
	/* Delay hardcoded in the kernel */
	ISE_PAL_udelay(1000);
	ISE_PAL_gpio_set(ISE_PAL_GPIO_IMGR0_RST, TRUE, NULL);
	ISE_PAL_udelay(300);
#endif
#ifdef SMART_3MP_PARLL_OV
	ISE_PAL_gpio_set(T2_VAUX_DEV_GRP_P1, TRUE, NULL);
	ISE_PAL_gpio_set(T2_VAUX_2_8_V, TRUE, NULL);
	ISE_PAL_udelay(100);

	ISE_PAL_gpio_set(ISE_PAL_GPIO_IMGR0_RST, TRUE, NULL);
	ISE_PAL_gpio_set(CAMERA_STANDBY_GPIO , FALSE, NULL);
	ISE_PAL_udelay(100);
	ISE_PAL_gpio_set(ISE_PAL_GPIO_IMGR0_RST, FALSE, NULL);
	ISE_PAL_udelay(100);
	ISE_PAL_gpio_set(ISE_PAL_GPIO_IMGR0_RST, TRUE, NULL);
	ISE_PAL_udelay(10*100);
	ISE_PAL_gpio_set(FPGA_BASE, TRUE, NULL);
	ISE_PAL_udelay(100);
#endif
}


static int
issue_camtrans_cip_start(int start)
{
	ISE_PAL_REG_TRANS_T data[10];
	int i = 0,ret;
	
	make_palregtrans(&(data[i++]),PAGE_SWITCH,1,PAGE1,1,FALSE);
//	make_palregtrans(&(data[i++]),CMD_FE,1,0x01,1,FALSE);
	switch(cap_state)
	{
	case ISE_CAPTURE_STATE_PREVIEW:
		curr_cip_data.win_width = prv_size.width;
		curr_cip_data.win_height = prv_size.height;
		if(start){
		make_palregtrans(&data[i++],CAM_PRV_FORMAT_L,1,
				(prv_fmt_val | 0x1),1,FALSE);
		cip_start = 1;
		}
		else{
		make_palregtrans(&data[i++],CAM_PRV_FORMAT_L,1,
				(prv_fmt_val & 0xFE),1,FALSE);
		cip_start = 0;
		}
	break;
	case ISE_CAPTURE_STATE_CAPTURE:
		curr_cip_data.win_width = img_size.width;
		curr_cip_data.win_height = img_size.height;
		if(start){
		make_palregtrans(&(data[i++]),CAM_IMG_CAP_TRIGGER,1,
				0x1,1,FALSE);
		cip_start = 1;
		}
		else{
		make_palregtrans(&(data[i++]),CAM_IMG_CAP_TRIGGER,1,
				0x2,1,FALSE);
		cip_start = 0;
		}
	break;
	default:
		ret = -1;/* State Not supported */
		return ret;
	break;
	};
	make_palregtrans(&(data[i++]),CMD_FE,1,0x01,1,FALSE);
	ISE_PAL_reg_trans(pal_reg_handle,data, i);

	return ret;
}

static int
issue_sensortrans_change_size(unsigned int left, unsigned int top)
{
	if(cap_state == ISE_CAPTURE_STATE_CAPTURE)
	issue_sensortrans_preview_config(sensor_img_size.width,
			sensor_img_size.height, left, top);
	else if(cap_state == ISE_CAPTURE_STATE_PREVIEW)
	issue_sensortrans_preview_config(sensor_prv_size.width,
			sensor_prv_size.height,left, top);
}

/* Would be useful when only the CIP size to be changed 
 * while the sensor size remain the same 
 */
static int 
issue_camtrans_cip_prv_resln(int width,int height)
{
	ISE_PAL_REG_TRANS_T data[10];
	int i = 0,ret;

	make_palregtrans(&data[i++],PAGE_SWITCH,1,PAGE1,1,PAL_WRITE);
	make_palregtrans(&data[i++],CAM_PRV_WIDTH_H,1,
				(width>>0x8),1,FALSE);
	make_palregtrans(&data[i++],CAM_PRV_WIDTH_L,1,
				width&0xFF,1,FALSE);
	make_palregtrans(&data[i++],CAM_PRV_HEIGHT_H,1,
				(height>>0x8),1,FALSE);
	make_palregtrans(&data[i++],CAM_PRV_HEIGHT_L,1,
				height&0xFF,1,FALSE);
	prv_size.width = width; prv_size.height = height;
	ret = ISE_PAL_reg_trans(pal_reg_handle, data, i);
}

static int
issue_camtrans_cip_callback_changes(const ISE_IMAGER_CONFIG_T *cip_config_data)
{
	ISE_PAL_REG_TRANS_T data[10];
	int i =0,ret;
	static unsigned char polarity_set = 0;

	ISE_PAL_reg_open(&palreg_config_camtrans,&pal_reg_handle, NULL);
	
	if((cip_config_data->win_width != curr_cip_data.win_width)
		||(cip_config_data->win_height != curr_cip_data.win_height)){
		/* It might be entered here if the default values of prv_size 
		 * and curr_cip_data width,height are not same */
		/* Does not come here since PAL transactions would have 
		 * already happened before cipcallback to be called. */
		if(cap_state == ISE_CAPTURE_STATE_CAPTURE){
		issue_camtrans_cip_cap_resln(cip_config_data->win_width,
					cip_config_data->win_height);
		}
		else if(cap_state == ISE_CAPTURE_STATE_PREVIEW){
		issue_camtrans_cip_prv_resln(cip_config_data->win_width,
					cip_config_data->win_height);
		}
	}
	/* Do the polarity setting only once */
	if(!polarity_set){
		issue_camtrans_cip_syncsignals(cip_config_data->hsync_prop,
				cip_config_data->vsync_prop,
				cip_config_data->pclk_prop);
		polarity_set = 1;
	}
	ISE_PAL_reg_close(&pal_reg_handle);
}

/*
 * Sends sensor transaction with slave id to PAL to know 
 * if sensor is available.
 * phys_loc	: 0 for Primary sensor, 1 for Secondary Sensor
 * callback	: Callback not used as of now. Since we dont see
 *		 any use of this callback
 * cookie	: The argument to the callback when initiated
 * imager_info	: Information of the corresponding sensor details
 * 		 all are static information and does not need any 
 *		 sensor transaction.
 */
ISE_STATUS_T ISE_detect_imager(const ISE_IMAGER_PHYS_LOCATION_T phys_loc, 
		ISE_IMAGER_CONFIG_CALLBACK_T *const callback,
		void *cookie,
		ISE_IMAGER_DETAILS_T *imager_info)
{
	int ret;

	palreg_config_imager0.type_config.i2c_config.bus_speed_khz = 
			i2ccfg.bus_speed_khz;
	palreg_config_imager0.type_config.i2c_config.write_addr = 
			i2ccfg.write_addr;
	palreg_config_imager0.type_config.i2c_config.i2c_num = 
			i2ccfg.i2c_num;
	palreg_config_imager0.type_config.i2c_config.drive_strength = 
			i2ccfg.drive_strength;
	palreg_config_imager0.reg_type = ISE_PAL_REG_TYPE_I2C;
#ifdef RAW_5MP_PARLL_GR2_FOCUS
	palreg_config_imager0_focus.type_config.i2c_config.bus_speed_khz = 
			i2ccfg_focus.bus_speed_khz;
	palreg_config_imager0_focus.type_config.i2c_config.write_addr = 
			i2ccfg_focus.write_addr;
	palreg_config_imager0_focus.type_config.i2c_config.i2c_num = 
			i2ccfg_focus.i2c_num;
	palreg_config_imager0_focus.type_config.i2c_config.drive_strength = 
			i2ccfg_focus.drive_strength;
	palreg_config_imager0_focus.reg_type = ISE_PAL_REG_TYPE_I2C;
#endif
	palreg_config_camtrans.type_config.i2c_config.bus_speed_khz = 
			0;
	palreg_config_camtrans.type_config.i2c_config.write_addr = 
			0xFF;
	palreg_config_camtrans.type_config.i2c_config.i2c_num = 
			0;
	palreg_config_camtrans.type_config.i2c_config.drive_strength = 
			0;
	palreg_config_camtrans.reg_type = ISE_PAL_REG_TYPE_CAM;


	if(!detected_prim_sensor){
		config_gpio();
		ISE_PAL_reg_open(&palreg_config_camtrans,&pal_reg_handle,
				NULL);
		issue_camtrans_sensor_page();
		ISE_PAL_reg_close(&pal_reg_handle);
		ISE_PAL_reg_open(&palreg_config_imager0,&pal_reg_handle,
				NULL);
		if(issue_sensortrans_reset()){
			GCI_PRINT("\n sensorret failed = %d",ret);
			ret= ISE_STATUS_PHYS_LOC_NOT_AVAIL;
		}
		else{
#ifdef RAW_5MP_PARLL_GR2_FOCUS

		ISE_PAL_reg_open(&palreg_config_camtrans,&pal_reg_handle,
				NULL);
		ret  = i2c_change_reg_size(SENSOR_WIDTH8);
		ISE_PAL_reg_close(&pal_reg_handle);
		ISE_PAL_reg_open(&palreg_config_imager0_focus,&pal_reg_handle,
				NULL);
		issue_sensortrans_focus_initial_config();
		ISE_PAL_reg_close(&pal_reg_handle);
		ISE_PAL_reg_open(&palreg_config_camtrans,&pal_reg_handle,
				NULL);
		ret = i2c_change_reg_size(SENSOR_WIDTH16);
		ISE_PAL_reg_close(&pal_reg_handle);

#endif
			detected_prim_sensor = 1;
			/*TBD move the error messages to pal */
			ret = ISE_STATUS_OK;
		}
		ISE_PAL_reg_close(&pal_reg_handle);
	}
#ifdef RAW_5MP_PARLL_GR2
	strcpy(imager_info->cam_man[0],"MT9T012");
	strcpy(imager_info->cam_man[1], "PARALLEL" );
	strcpy(imager_info->cam_man[2], "RAW");
#endif
#ifdef SMART_3MP_PARLL_OV
	strcpy(imager_info->cam_man[0],"OV3640");
	strcpy(imager_info->cam_man[1], "PARALLEL" );
	strcpy(imager_info->cam_man[2], "SMART");
#endif
	imager_info->num_of_devices = 1;
	imager_info->phys_loc = ISE_IMAGER_PHYS_LOCATION_IMAGER0;
	imager_info->phys_orientation = ISE_IMAGER_PHYS_ORIENTATION_CCW_0;
	imager_info->capabilities = cap;
	imager_info->detect_info = 0; /* Nothing to fill */
	/* TBD move the error messages to pal */
	return ret;
}

ISE_STATUS_T
ISE_connect(ISE_IMAGER_DETAILS_T *img_info,
		const ISE_EVENTS_T *const events,
		void *cookie,
		const ISE_USE_CASE_T use_case,
		ISE_HANDLE_T *ise_hdl)
{
	int ret;
	ret = ISE_STATUS_OK;

	ISE_PAL_reg_open(&palreg_config_camtrans,&pal_reg_handle, NULL);
	/*imager_info not used since primary sensor is assumed */
	issue_camtrans_cip_init();
	ISE_PAL_reg_close(pal_reg_handle);
	ISE_PAL_reg_open(&palreg_config_imager0,&pal_reg_handle, NULL);
	issue_sensortrans_initial_config();
	ISE_PAL_reg_close(pal_reg_handle);

	/* TBD ISE spec does not have means to set the callback
	 * arguments.  So passing NULL for cookie and sending 
	 * ISE_IMAGER_CONFIG_T in cip_config_data variable */
	 /* Only cip callback used */
	cip_callback = events->cip_config;
	cip_config_data.cip_state = ISE_CIP_STATE_STOP;
	cip_config_data.clock_freq_khz = 0x12000;
#ifdef RAW_5MP_PARLL_GR2
	cip_config_data.color_fmt = ISE_COLOR_FMT_BAYER_BGGR;
#endif
#ifdef SMART_3MP_PARLL_OV
	cip_config_data.color_fmt = ISE_COLOR_FMT_422_YUYV;
#endif
	cip_config_data.drive_strength = 0x0;
#if defined (RAW_3MP_PARLL_ES1) || defined(RAW_5MP_PARLL_GR2)
	cip_config_data.hsync_prop = ISE_SIGNAL_PROP_RISING_EDGE;
	cip_config_data.vsync_prop = ISE_SIGNAL_PROP_RISING_EDGE;
	cip_config_data.pclk_prop = ISE_SIGNAL_PROP_ACTIVE_HIGH;
#endif
	cip_config_data.win_hoffset = 0;
	cip_config_data.win_voffset = 0;
	cip_config_data.win_width = prv_size.width;
	cip_config_data.win_height = prv_size.height;
	/* TBD Callback function declaration,definition in OMX */
	//cip_callback(NULL,&cip_config_data);
	/* Note: Even though cip_callback is called with cip_config
	 * parameters to indicate OMX of the cip changes,
	 * corresponding PAL transactions should also happen in the 
	 * kernel modules for the change to take effect on 
	 * camera configuation path*/
	issue_camtrans_cip_callback_changes(&cip_config_data);
	/* Return the same handle for both SI and VID capture */
	if(ret == ISE_STATUS_OK){
		*ise_hdl = (ISE_HANDLE_T)1;
	}
	else
		ret = ISE_STATUS_FAIL;
	ret = ISE_send_command(ise_hdl, ISE_CMD_SET_USE_CASE, &use_case, 
			sizeof(ISE_USE_CASE_T));
	return ret;
}

ISE_STATUS_T
ISE_get_attribute(const ISE_HANDLE_T ise_hdl,
			const ISE_ATTR_T attr_name,
			void *attr_val,
			/*const*/ unsigned int attr_size)
{
	int ret = ISE_STATUS_OK;
	switch(attr_name)
	{
	case ISE_ATTR_ACTUAL_RESOLUTION:
		ret = ISE_STATUS_INVAL_ATTR;
	break;
	case ISE_ATTR_PREV_RESOLUTION:
		attr_val = (void*)&prv_size;
		attr_size = sizeof(prv_size);
	break;
	case ISE_ATTR_CAPTURE_RESOLUTION:
		attr_val = (void*)&img_size;
		attr_size = sizeof(img_size);
	break;
	case ISE_ATTR_CAPTURE_STATE:
		attr_val = (void*)&cap_state;
		attr_size = sizeof( cap_state);
	break;
	case ISE_ATTR_FRAME_RATE_MODE:
		ret = ISE_STATUS_ATTR_NOT_SUPPORTED;
	break;
	case ISE_ATTR_FRAME_RATE:
		ret = ISE_STATUS_ATTR_NOT_SUPPORTED;
	break;
	case ISE_ATTR_SCENE_MODE:
		ret = ISE_STATUS_ATTR_NOT_SUPPORTED;
	break;
	case ISE_ATTR_STYLE:
		attr_val = (void*)&color_effect;
		attr_size = sizeof(ISE_STYLE_T);
	break;
	case ISE_ATTR_DIGITAL_ZOOM:
		attr_val = (void*)&zoom_magnitude;
		attr_size = sizeof(unsigned int);
	break;
	case ISE_ATTR_USE_CASE:
		attr_val = (void*)&use_case;
		attr_size = sizeof(ISE_USE_CASE_T);
	break;
	case ISE_ATTR_POWER_STATE:
		attr_val = (void*)&pwr_st;
		attr_size = sizeof(ISE_POWER_STATE_T);
	break;
	case ISE_ATTR_BRIGHTNESS:
		attr_val = (void*)&brightness_level;
		attr_size = sizeof(unsigned int);
	break;
	case ISE_ATTR_CONTRAST:
		attr_val = (void*)&contrast_level;
		attr_size = sizeof(unsigned int);
	break;
	case ISE_ATTR_CAPABILITIES:
		attr_val = (void*)&cap;
		attr_size = sizeof(ISE_IMAGER_CAPABILITIES_T);
	break;
	case ISE_ATTR_PREV_COLOR_FMT:
		attr_val = (void*)&prv_fmt;
		attr_size = sizeof(ISE_COLOR_FMT_T);
	break;
	case ISE_ATTR_CAPTURE_COLOR_FMT:
		attr_val = (void*)&cap_fmt;
		attr_size = sizeof(ISE_COLOR_FMT_T);
	break;
	case ISE_ATTR_GET_VERSION:
		ret = ISE_STATUS_ATTR_NOT_SUPPORTED;
	break;
	default:
		ret = ISE_STATUS_INVAL_ATTR;
	}
	return ret;
}

ISE_STATUS_T
ISE_send_command(const ISE_HANDLE_T ise_hdl,const ISE_CMD_T cmd,
		const void *const param_val,
		const unsigned int param_size)
{
	static unsigned char pwr_standby_cip_running;
	int ret;
	
	switch(cmd)
	{
	case ISE_CMD_SET_PREV_RESOLUTION:
	{
		ISE_CAPTURE_STATE_T old_capstate = cap_state;
		cap_state = ISE_CAPTURE_STATE_PREVIEW;
		memcpy(&prv_size, param_val, param_size);
		ISE_PAL_reg_open(&palreg_config_camtrans,&pal_reg_handle,
				NULL);
		ret = issue_camtrans_cip_prv_resln(prv_size.width,
				prv_size.height);
		if(zoom_magnitude && SMART_SENSOR)
		ret = issue_camtrans_get_smartsensor_resln(&sensor_prv_size,
				&left, &top);
		else
		ret = issue_camtrans_get_rawsensor_resln(&sensor_prv_size,
				&left, &top);
		ISE_PAL_reg_close(&pal_reg_handle);
		cap_state = old_capstate;
	}
	break;
	case ISE_CMD_SET_CAPTURE_RESOLUTION:
	{
		ISE_CAPTURE_STATE_T old_capstate = cap_state;
		cap_state = ISE_CAPTURE_STATE_CAPTURE;
		memcpy(&img_size, param_val, param_size);
		ISE_PAL_reg_open(&palreg_config_camtrans,&pal_reg_handle,
				NULL);
		ret = issue_camtrans_cip_cap_resln(img_size.width,
				img_size.height);
		if(zoom_magnitude && SMART_SENSOR)
		ret = issue_camtrans_get_smartsensor_resln(&sensor_img_size,
				&left, &top);
		else
		ret = issue_camtrans_get_rawsensor_resln(&sensor_img_size,
				&left, &top);
		ISE_PAL_reg_close(&pal_reg_handle);
		cap_state = old_capstate;
	}
	break;
	case ISE_CMD_SET_CAPTURE_STATE:
	{
		memcpy(&cap_state, param_val, param_size);
		/* cap_state is distinguished inside these calls 
		 * the CAM commands are different, 
		 * the effect is same ie change in size and format*/
		ISE_PAL_reg_open(&palreg_config_imager0,&pal_reg_handle,
				NULL);
		/* TBD keep a check to call if the sensor change configuration
		 * is really needed */
		ret = issue_sensortrans_change_size(left, top);
		ISE_PAL_reg_close(&pal_reg_handle);
		ISE_PAL_reg_open(&palreg_config_camtrans,&pal_reg_handle,
				NULL);
		ret = issue_camtrans_cip_start(1);
		ISE_PAL_reg_close(&pal_reg_handle);
	}
	break;
	case ISE_CMD_SET_FRAME_RATE:
	break;
	case ISE_CMD_SET_STYLE:
	{
		memcpy(&color_effect,param_val,param_size);
#ifdef SMART_3MP_PARLL_OV
		if(color_effect == ISE_STYLE_REDDISH)
			color_effect = 1;
		else if(color_effect == ISE_STYLE_SEPIA)
			color_effect = 2;
		else if(color_effect == ISE_STYLE_BLACK_WHITE)
			color_effect = 3;
		else color_effect = 0;
		ISE_PAL_reg_open(&palreg_config_imager0,&pal_reg_handle,
				NULL);
		ret = issue_sensortrans_coloreffect(color_effect);
		ISE_PAL_reg_close(&pal_reg_handle);
#endif
	}
	break;
	case ISE_CMD_SET_DIGITAL_ZOOM:
		memcpy(&zoom_magnitude,param_val,param_size);
		if(zoom_magnitude > cap.max_digital_zoom)
			zoom_magnitude = cap.max_digital_zoom;
	break;
	case ISE_CMD_SET_USE_CASE:
	{
		memcpy(&use_case,param_val,param_size);
		switch(use_case)
		{
		case ISE_USE_CASE_SI_CAPTURE:
		{
		ISE_PAL_reg_open(&palreg_config_camtrans,&pal_reg_handle,
				NULL);
		ret = issue_camtrans_cip_cap_time(CAP_SHOTS,
				CAP_DELAY_1STSHOT, CAP_DELAY_INTERSHOT);
		ISE_PAL_reg_close(&pal_reg_handle);
			/* TBD Set the image sizes supported */
		}
		break;
		case ISE_USE_CASE_VID_CAPTURE:
			/* TBD Set the video sizes supported */
		break;
		default:
			GCI_PRINT("ISE-ERR: Usecase Not supported");
			ret = ISE_STATUS_CMD_NOT_SUPPORTED;
		break;
		};
	}
	break;
	case ISE_CMD_SET_POWER_STATE:
	{
		memcpy(&pwr_st, param_val, param_size);
		GCI_PRINT("**Power %d 0x%x %d\n",pwr_st, param_val,
				param_size);
		ISE_PAL_reg_open(&palreg_config_camtrans,&pal_reg_handle,
				NULL);
		/* Start getting frames in the state that was previously in */
		if(pwr_st == ISE_POWER_STATE_OUT_OF_STANDBY){
			//TBD issue_camtrans_cip_clocks(1);
			/* Power standby had been called when cip was ON */
			if(pwr_standby_cip_running){
				pwr_standby_cip_running = 0;
				issue_camtrans_cip_start(1);
			}
		}
		else{
			/* Make sure that stop is called after dqbuf of
			all queued buffers is done. Because video-buf
			does not like the unfilled queued buffers */
			if(cip_start == 1){
				pwr_standby_cip_running = 1;
				issue_camtrans_cip_start(0);
			}
			/* TBD issue_camtrans_cip_clocks(0); */
			/* TBD Sensor in low power */
		}
		ISE_PAL_reg_close(&pal_reg_handle);
	}
	break;
	case ISE_CMD_SET_BRIGHTNESS:
	{
		memcpy(&brightness_level,param_val,param_size);
#ifdef SMART_3MP_PARLL_OV
		ISE_PAL_reg_open(&palreg_config_imager0,&pal_reg_handle,
				NULL);
		ret = issue_sensortrans_brightness(brightness_level);
		ISE_PAL_reg_close(&pal_reg_handle);
#endif
	}
	break;
	case ISE_CMD_SET_CONTRAST:
	{
		memcpy(&contrast_level,param_val,param_size);

#ifdef SMART_3MP_PARLL_OV
		ISE_PAL_reg_open(&palreg_config_imager0,&pal_reg_handle,
			NULL);
		ret = issue_sensortrans_contrast(contrast_level);
		ISE_PAL_reg_close(&pal_reg_handle);
#endif
	}
	break;
	case ISE_CMD_SET_PREV_COLOR_FMT:
	{
		ISE_CAPTURE_STATE_T old_capstate = cap_state;
		cap_state = ISE_CAPTURE_STATE_PREVIEW;
		/* TBD for Smart sensor Sensortrans if any */
#ifdef RAW_5MP_PARLL_GR2
	sensor_fmt = ISE_COLOR_FMT_BAYER_BGGR;
#endif
#ifdef SMART_3MP_PARLL_OV
	sensor_fmt = param_val;
	GCI_PRINT("sensor_fmt = %d", (ISE_COLOR_FMT_T)param_val);
	/* TBD for smart*/
	sensor_fmt = ISE_COLOR_FMT_422_YUYV;
#endif
		ISE_PAL_reg_open(&palreg_config_camtrans,&pal_reg_handle,
			NULL);
		ret = issue_camtrans_cip_fmt((ISE_COLOR_FMT_T)param_val,TRUE);
		ISE_PAL_reg_close(&pal_reg_handle);
		cap_state = old_capstate;
	}
	break;
	case ISE_CMD_SET_CAPTURE_COLOR_FMT:
	{
		ISE_CAPTURE_STATE_T old_capstate = cap_state;
		cap_state = ISE_CAPTURE_STATE_CAPTURE;
		/* TBD for Smart sensor Sensortrans if any */
#ifdef RAW_5MP_PARLL_GR2
	sensor_fmt = ISE_COLOR_FMT_BAYER_BGGR;
#endif
#ifdef SMART_3MP_PARLL_OV
	sensor_fmt = param_val;
	sensor_fmt = ISE_COLOR_FMT_422_YUYV;
#endif
		ISE_PAL_reg_open(&palreg_config_camtrans,&pal_reg_handle,
				NULL);
		ret = issue_camtrans_cip_fmt((ISE_COLOR_FMT_T)param_val,
				FALSE);
		ISE_PAL_reg_close(&pal_reg_handle);
		cap_state = old_capstate;
	}
	break;
	case ISE_CMD_SET_ACTIVE_DEVICE:
	break;
	default:
	{
		return ISE_STATUS_CMD_NOT_SUPPORTED;
	}
	break;
	}; /* End of switch ISE_CMD_T */
	return ISE_STATUS_OK;
}


ISE_STATUS_T
ISE_disconnect(ISE_HANDLE_T *ise_hdl)
{
	ISE_PAL_reg_open(&palreg_config_camtrans,&pal_reg_handle, NULL);
	if(cip_start == 1)
		issue_camtrans_cip_start(0);
	ISE_PAL_reg_close(&pal_reg_handle);
	/* TBD Pull off the power to the sensor or put in standby state*/
	*ise_hdl = (ISE_HANDLE_T)0;
	return ISE_STATUS_OK;
}

