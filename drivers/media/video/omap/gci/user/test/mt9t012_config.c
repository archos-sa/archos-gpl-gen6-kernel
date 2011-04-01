/*
 * drivers/media/video/omap/gci/user/test/mt9t012_config.c
 *
 * Sensor config file for testing
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

extern int issue_sensortrans_reg16_data16(unsigned int data[], int size);

#define SENSOR_WIDTH8	1
#define SENSOR_WIDTH16	2
#define SENSOR_WIDTH32	4

#define SENSOR_WRITE	1
#define SENSOR_READ	0



#define REG_MODEL_ID			0x0
#define REG_MODE_SELECT			0x0100 
#define REG_IMAGE_ORIENTATION		0x0101 
#define REG_SOFTWARE_RESET		0x0103
#define REG_GROUPED_PAR_HOLD		0x0104

#define REG_FINE_INT_TIME		0x0200
#define REG_COARSE_INT_TIME		0x0202

#define REG_ANALOG_GAIN_GLOBAL		0x0204
#define REG_ANALOG_GAIN_GREENR		0x0206
#define REG_ANALOG_GAIN_RED		0x0208
#define REG_ANALOG_GAIN_BLUE		0x020A
#define REG_ANALOG_GAIN_GREENB		0x020C
#define REG_DIGITAL_GAIN_GREENR		0x020E
#define REG_DIGITAL_GAIN_RED		0x0210
#define REG_DIGITAL_GAIN_BLUE		0x0212
#define REG_DIGITAL_GAIN_GREENB		0x0214

#define REG_VT_PIX_CLK_DIV		0x0300
#define REG_VT_SYS_CLK_DIV		0x0302
#define REG_PRE_PLL_CLK_DIV		0x0304
#define REG_PLL_MULTIPLIER		0x0306
#define REG_OP_PIX_CLK_DIV		0x0308
#define REG_OP_SYS_CLK_DIV		0x030A



#define REG_FRAME_LEN_LINES		0x0340
#define REG_LINE_LEN_PCK		0x0342

#define REG_X_ADDR_START		0x0344
#define REG_Y_ADDR_START		0x0346
#define REG_X_ADDR_END			0x0348
#define REG_Y_ADDR_END			0x034A
#define REG_X_OUTPUT_SIZE		0x034C
#define REG_Y_OUTPUT_SIZE		0x034E

#define REG_SCALING_MODE		0x0400
#define REG_SCALE_M			0x0404
#define REG_SCALE_N			0x0406

#define REG_ROW_SPEED			0x3016
#define REG_RESET_REGISTER		0x301A
#define REG_PIXEL_ORDER			0x3024
#define REG_READ_MODE			0x3040

#define REG_DATAPATH_STATUS		0x306A
#define REG_DATAPATH_SELECT		0x306E



static unsigned int sensor_revid[] = {
	REG_MODEL_ID,0x01,SENSOR_WIDTH16,SENSOR_READ
};

static unsigned int sensor_reset[] = {
	REG_SOFTWARE_RESET,0x01,SENSOR_WIDTH8,SENSOR_WRITE
};
#define SENSOR_RESET_SIZE	4

static unsigned int sensor_init[] = {
	REG_RESET_REGISTER,0x18c8,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_GROUPED_PAR_HOLD,0x01,SENSOR_WIDTH8,SENSOR_WRITE,
	REG_ANALOG_GAIN_GREENR,0x0008,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_ANALOG_GAIN_RED,0x0008,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_ANALOG_GAIN_BLUE,0x0008,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_ANALOG_GAIN_GREENB,0x0008,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_DIGITAL_GAIN_GREENR,0x0100,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_DIGITAL_GAIN_RED,0x0100,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_DIGITAL_GAIN_BLUE,0x0100,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_DIGITAL_GAIN_GREENB,0x0100,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_GROUPED_PAR_HOLD,0x00,SENSOR_WIDTH8,SENSOR_WRITE,
};
#define SENSOR_INIT_SIZE	44

static unsigned int sensor_streaming_on[] = {
	REG_MODE_SELECT,0x01,SENSOR_WIDTH8,SENSOR_WRITE
};
#define SENSOR_STREAMING_ON_SIZE	4

static unsigned int sensor_streaming_off[] = {
	REG_MODE_SELECT,0x00,SENSOR_WIDTH8,SENSOR_WRITE
};
#define SENSOR_STREAMING_OFF_SIZE	4

static unsigned int sensor_set_clock[] = {
	REG_GROUPED_PAR_HOLD,0x01,SENSOR_WIDTH8,SENSOR_WRITE,
	REG_VT_PIX_CLK_DIV,0x0005,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_PRE_PLL_CLK_DIV,0x0003,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_PLL_MULTIPLIER,0x0033,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_OP_PIX_CLK_DIV,0x000A,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_OP_SYS_CLK_DIV,0x0001,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_GROUPED_PAR_HOLD,0x00,SENSOR_WIDTH8,SENSOR_WRITE,
};
#define SENSOR_SET_CLOCK_SIZE	28

static unsigned int sensor_enter_video_mode[] = {
	REG_READ_MODE,(3<<2) | (3<<5),SENSOR_WIDTH16,SENSOR_WRITE,
	REG_GROUPED_PAR_HOLD,0x01,SENSOR_WIDTH8,SENSOR_WRITE,
	REG_FRAME_LEN_LINES,0x310,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_LINE_LEN_PCK,0x6AC,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_COARSE_INT_TIME,0x30F,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_FINE_INT_TIME,0x204,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_X_ADDR_START,0x0004,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_Y_ADDR_START,0x0004,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_X_ADDR_END,0x0804,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_Y_ADDR_END,0x0604,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_SCALE_M,0x0020,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_GROUPED_PAR_HOLD,0x00,SENSOR_WIDTH8,SENSOR_WRITE,
};
#define SENSOR_ENTER_VIDEO_SIZE	48

static unsigned int sensor_enter_capture_mode[] = {
};

static unsigned int sensor_scaler_on[] = {
	REG_GROUPED_PAR_HOLD,0x01,SENSOR_WIDTH8,SENSOR_WRITE,
	REG_RESET_REGISTER,0x10c8,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_DATAPATH_SELECT,(4<<10) | (4<<13) | 0x2,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_X_OUTPUT_SIZE,0x0200,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_Y_OUTPUT_SIZE,0x0180,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_SCALING_MODE,0x0002,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_GROUPED_PAR_HOLD,0x00,SENSOR_WIDTH8,SENSOR_WRITE,
};
#define SENSOR_SCALER_ON_SIZE	28

static unsigned int sensor_scaler_off[] = {
	REG_GROUPED_PAR_HOLD,0x01,SENSOR_WIDTH8,SENSOR_WRITE,
	REG_RESET_REGISTER,0x18c8,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_DATAPATH_SELECT,(4<<10) | (4<<13) | 0x00 ,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_X_OUTPUT_SIZE,0x0400,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_Y_OUTPUT_SIZE,0x0300,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_SCALING_MODE,0x0000,SENSOR_WIDTH16,SENSOR_WRITE,
	REG_GROUPED_PAR_HOLD,0x00,SENSOR_WIDTH8,SENSOR_WRITE,
};
#define SENSOR_SCALER_OFF_SIZE	28

static unsigned int sensor_dump[] = {
	REG_RESET_REGISTER,0x00,SENSOR_WIDTH16,SENSOR_READ,
	REG_DATAPATH_SELECT,0x00,SENSOR_WIDTH16,SENSOR_READ,
	REG_X_OUTPUT_SIZE,0x00,SENSOR_WIDTH16,SENSOR_READ,
	REG_Y_OUTPUT_SIZE,0x00,SENSOR_WIDTH16,SENSOR_READ,
	REG_SCALING_MODE,0x00,SENSOR_WIDTH16,SENSOR_READ,
};
#define SENSOR_DUMP_SIZE 20



int
mt9t012_reset()
{
	int ret = 0;
	ret = issue_sensortrans_reg16_data16(sensor_reset,SENSOR_RESET_SIZE);
	return ret;
}

int
mt9t012_initial_config()
{
	int ret = 0;
	ret = issue_sensortrans_reg16_data16(sensor_streaming_off,
				SENSOR_STREAMING_OFF_SIZE);
	ret = issue_sensortrans_reg16_data16(sensor_init,
				SENSOR_INIT_SIZE);
	ret = issue_sensortrans_reg16_data16(sensor_set_clock,
				SENSOR_SET_CLOCK_SIZE);
	/* TBD Issue trans for sensor standby mode */
	return ret;
}

int
mt9t012_preview_config(unsigned int width, unsigned height,
			unsigned int left, unsigned int top)
{
	int ret = 0;

	static int scaler_on = 0;
	ret = issue_sensortrans_reg16_data16(sensor_streaming_off,
				SENSOR_STREAMING_OFF_SIZE);
	ret = issue_sensortrans_reg16_data16(sensor_enter_video_mode,
				SENSOR_ENTER_VIDEO_SIZE);
	if((width <= 512) && (height <=384)){
		if(!scaler_on){
		 /* SCALAR ON */
		ret = issue_sensortrans_reg16_data16(sensor_scaler_on,
				SENSOR_SCALER_ON_SIZE);
		scaler_on = 1;
		}
	}
	else if((width > 512) && (height > 384)){
		/*SCALAR OFF */;
		ret = issue_sensortrans_reg16_data16(sensor_scaler_off,
				SENSOR_SCALER_OFF_SIZE);
		scaler_on = 0;
	}
	ret = issue_sensortrans_reg16_data16(sensor_streaming_on,
				SENSOR_STREAMING_ON_SIZE);
	return ret;
}

void 
get_mt9t012_resln(unsigned int width, unsigned int height,
				unsigned int* set_width,
				unsigned int* set_height)
{
		/* TBD 512, 1024 768, 384 has to be appended with
		 * "x" pixels that would be cropped in the ccdc,
		 * preview, resizer modules */
		if(((width > 512) && (width < 1024)) 
			&& ((height > 384) && (height < 768))){
			*set_width = 1024;
			*set_height = 768;
		}
		else if(((width < 512) && (width > 88)) /* > SQCIF size*/
			&& ((height < 384) && (height > 72))){
			*set_width = 512;
			*set_height = 384;
		}
		else if((width > 1024) && (height > 768)){
			*set_width = 2048;
			*set_height = 1536;
		}
	return;
}

