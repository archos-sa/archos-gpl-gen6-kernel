
/*
 * drivers/media/video/omap/video_decoder_if.h
 *
 * Copyright (C) 2008 Archos S.A.
 * 
 * Video decoder interface to OMAP video capture drivers
 * Video decoder driver should implement this interface
 *
 * This package is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation. 
 * 
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR 
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED 
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE. 
 */
 
#ifndef OMAP_VIDEO_DECODER_IF_H
#define OMAP_VIDEO_DECODER_IF_H
 
#define LEN_SENSOR_NAME		31

#define PAR_MODE_NOBT8			0
#define PAR_MODE_NOBT10		1
#define PAR_MODE_NOBT12		2
#define PAR_MODE_BT8			4
#define PAR_MODE_BT10			5

#define SENSOR_PARALLEL			0
#define SENSOR_SERIAL1			1
#define SENSOR_SERIAL2			2

#define SENSOR_ISP			0
#define SENSOR_RAW			1

#define SYNC_ACTIVE_HIGH		0
#define SYNC_ACTIVE_LOW		1

#define V4L2_BUF_TYPE_STILL_CAPTURE 	V4L2_BUF_TYPE_PRIVATE

struct video_decoder {
	unsigned int version;
	char name[LEN_SENSOR_NAME + 1];

	int decoder_type;   /* isp or raw sensor */
	int decoder_interface;   /* parallel or serial sensor */

	int parallel_mode;  /* parallel I/F mode */
	int hs_polarity;    /* horizontal sync polarity */ 	
	int vs_polarity;    /* vertical sync polarity */
	int image_swap;	    /* image swap or not */
	int bt_correction;  /* BT correction enabled or not */
	
	/* init the sensor with the passed pix format. A none zero private
	   pointer must be returned on success. The same pointer is passed
	   back for all other functions. This gives a sensor driver the
	   chance to handle multiple sensor. */ 
	void *(*init)(struct v4l2_pix_format *);
	/* clean up the sensor */
	int (*cleanup)(void *);

	/* These are for power management */
	int (*power_on)(void *);
	int (*power_off)(void *);

	/* Handle V4L2 fmt IOCTLs.*/ 
	/* Raw sensor driver doesn't implement enum_pixformat. camera driver should
	   handle it */
	int (*enum_pixformat)(struct v4l2_fmtdesc *, void *);
	/* Raw sensor driver needs to implement try_format. But this is filtered by
	   camera driver who konws ISP */
	int (*try_format) (struct v4l2_pix_format *, void *);

	/* Configure the sensor to generate the passed pix format at the
	   passed capture rate with the passed xclk */   
	int (*configure) (struct v4l2_pix_format *, struct v4l2_fract *, void *);
	/* These handle V4L2 control IOCTLs */
	int (*query_control) (struct v4l2_queryctrl *, void *);
	int (*get_control) (struct  v4l2_control *, void *);
	int (*set_control) (struct  v4l2_control *, void *);
	int (*get_standard) (v4l2_std_id *std, void *);
	int (*set_standard) (v4l2_std_id *std, void *);
	int (*enum_input) (struct v4l2_input *, void *);
	int (*set_input) (unsigned int *, void *);
	int (*get_input) (unsigned int *, void *);
	
	int (*stream_start) (void *);
	int (*stream_stop) (void *);
	
	/* Linux' video_decoder ioctl */
	int (*ioctl) (unsigned int, void *, void *);

	/* Optionally. These handle V4L2 crop IOCTLs with 
	   V4L2_BUF_TYPE_VIDEO_CAPTURE buffer type */
	int (*cropcap) (struct v4l2_cropcap *, void *);
	int (*get_crop) (struct  v4l2_crop *, void *);
	int (*set_crop) (struct  v4l2_crop *, void *);
};


extern int omap_dvr_register_decoder(struct video_decoder *dvr_decoder); 
extern int omap_dvr_unregister_decoder(struct video_decoder *dvr_decoder); 

#endif
