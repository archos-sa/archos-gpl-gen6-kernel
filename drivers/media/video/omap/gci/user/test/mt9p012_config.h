/*
 * drivers/media/video/omap/gci/user/test/mt9p012_config.h
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

#ifndef MT9P012_CONFIG_H
#define MT9P012_CONFIG_H


#include "gci-pal-mapper.h"

extern int mt9p012_initial_config();
extern int mt9p012_preview_config(unsigned int width, unsigned height,
			unsigned int left, unsigned int top);
extern void get_mt9p012_resln(unsigned int width, unsigned int height,
				unsigned int* set_width,
				unsigned int* set_height);
extern int mt9p012_reset();
extern int mt9p012_focus_initial_config();

#define T2_VAUX_2_8_V		0x09 << 8
#define T2_VAUX_DEV_GRP_P1	0x20 << 8
#define FPGA_BASE		0x08000000

#define SENSOR_RESET_GPIO  	98	/* for SDP ES2.0 */
#define CAMERA_STANDBY_GPIO	58	//sv126


#define PRIMARY_SENSOR_SLAVE_ADDR	0x36
#define D88_AF_I2C_ADDR			0x1A


ISE_PAL_REG_CONFIG_I2C_T i2ccfg = {
	400, 				/* bus_speed_khz; */
	PRIMARY_SENSOR_SLAVE_ADDR,	/* write_addr; */
	1,				/* drive_strength; */
	1				/* the sensor is on I2C-1 bus*/
	};

ISE_PAL_REG_CONFIG_I2C_T i2ccfg_focus = {
	400, 				/* bus_speed_khz; */
	D88_AF_I2C_ADDR,		/* write_addr; */
	1,				/* drive_strength; */
	1				/* the sensor is on I2C-1 bus*/
	};


ISE_IMAGER_CAPABILITIES_T cap = {
	0,	/* supported_flash_modes; */
	0,	/*  supported_flash_types;*/
	0,	/*  supported_flash_colors;*/
	0,	/*  supported_focus_modes;*/
	0,	/*  supported_metering_modes;*/
	0,	/*  supported_scene_modes;*/
	0,	/*  supported_exposure_compensation;*/
	4,	/*  supported_color_fmts;*/
	0,	/*  supported_mirror;*/
	0,	/*  supported_wb;*/
	0,	/*  supported_lighting_freq;*/
	0,	/*  supported_styles;*/
	0,	/*  supported_rotations;*/
	0,	/*  supported_test_patterns;*/
	0,	/*  supported_thumbnail_color_fmts;*/
	0,	/*  supported_dli_devices;*/
	0,	/*  supported_anti_shake;*/
	0,	/*  max_brightness;*/
	0,	/*  max_contrast;*/
	30,	/*  ISE_Q16_T max_fps; */
	2,	/*  ISE_Q16_T max_digital_zoom;*/
	0,	/* ISE_Q16_T max_optical_zoom;*/
	1,	/*ISE_ZOOM_STEP_T digital_zoom_step; */
	0,	/*ISE_ZOOM_STEP_T optical_zoom_step; */
	0,	/* ISE_BOOLEAN_T support_arbitrary_resolutions;*/
	{{176,144},{352,288},{640,480},{320,240},{720,480}, {640,576}}
	};

#endif /* ifndef MT9P012_CONFIG_H */
