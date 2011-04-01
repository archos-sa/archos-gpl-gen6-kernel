/*
 * drivers/media/video/omap/gci/user/test/read_save.c
 *
 * Image capture and store test case case for
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

#include <stdio.h>
#include <stdlib.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/types.h>
#include <linux/videodev.h>
#include <sys/mman.h>
#include <linux/ioctl.h>
#include <sys/time.h>
#include "gci-pal-mapper.h"


#define SAVE_TO_FILE 
#define SAVE_TO_FILE_NAME "video_out"
#define SAVE_TO_FILE_N_FRAMES 10

//#define SMART_3MP_PARLL_OV 1

#define NUM_BUFS 4

ISE_HANDLE_T ise_hdl;

int main (int argc, char *argv[])
{
	struct {
		void *start;
		size_t length;
	} *cbuffers;
	void *src_start;
	struct v4l2_capability capability;
	struct v4l2_format cformat;
	struct v4l2_requestbuffers creqbuf;
	struct v4l2_buffer cfilledbuffer[5];
	int cfd, i, ret, count = -1, memtype = V4L2_MEMORY_USERPTR;
	int fd_save = 0;
	int index = 1;
	struct timeval time;
	
	ISE_IMAGER_DETAILS_T *imager_info;
	ISE_EVENTS_T events;
	ISE_HANDLE_T ise_hdl;
	ISE_DIMENSION_T prv_size;
	ISE_COLOR_FMT_T col_fmt;
	ISE_POWER_STATE_T pwr_st;
	ISE_CAPTURE_STATE_T cap_state;

	count = 500;
	
	cfd = open("/dev/video0", O_RDWR);
	if (cfd == -1) {
		perror("Error...!!! /dev/camera not present.");
	}

	GEN_CAM_PAL_init();
	
	ret = ioctl(cfd, VIDIOC_G_FMT, &cformat);
		if (ret < 0) {
			perror("Camera VIDIOC_G_FMT");
			return -1;
		}

	ret = ioctl (cfd, VIDIOC_S_FMT, &cformat);
		if (ret < 0) {
			perror ("Camera VIDIOC_S_FMT");
			return -1;
		}

	if ((fd_save = creat(SAVE_TO_FILE_NAME, O_RDWR | O_APPEND)) <= 0) 
	{
		printf("Can't create file %s\n", SAVE_TO_FILE_NAME);
		fd_save = 0;
	}

	creqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	creqbuf.memory = memtype;
	creqbuf.count = NUM_BUFS;
	if (ioctl(cfd, VIDIOC_REQBUFS, &creqbuf) < 0) {
		perror ("VIDEO_REQBUFS");
		return -1;
	}
	printf("Camera Driver allowed buffers reqbuf.count = %d\n", creqbuf.count);

	cbuffers = calloc(creqbuf.count, sizeof(*cbuffers));
	/* Allocate user memory, and queue each buffer */
	for (i = 0; i < creqbuf.count; ++i) {
		struct v4l2_buffer buffer;  
		buffer.type = creqbuf.type;
		buffer.memory = creqbuf.memory;
		buffer.index = i;
		if(ioctl(cfd, VIDIOC_QUERYBUF, &buffer) < 0){
			perror("VIDIOC_QUERYBUF");
			return -1;
		}
		cbuffers[i].length = buffer.length;
		buffer.flags = 0;
		/* round to 4KB page */
		if (cbuffers[i].length & 0xfff) {
			cbuffers[i].length = (cbuffers[i].length 
				& 0xfffff000) + 0x1000;
		}
		buffer.length =  cbuffers[i].length;
		cbuffers[i].start = malloc(cbuffers[i].length);
		buffer.m.userptr = ((unsigned int)cbuffers[i].start 
				& 0xffffffe0) +0x20;
		memset(cbuffers[i].start,0,cbuffers[i].length);
		if (ioctl(cfd, VIDIOC_QBUF, &buffer) < 0) {
			perror("CAMERA VIDIOC_QBUF");
			return -1;
		}
	}

	imager_info = (ISE_IMAGER_DETAILS_T*)malloc(sizeof(ISE_IMAGER_DETAILS_T));
	ISE_detect_imager(ISE_IMAGER_PHYS_LOCATION_IMAGER0, NULL, NULL,imager_info);
	ise_hdl = (ISE_HANDLE_T)malloc(sizeof(ISE_HANDLE_T));
	events.cip_config = NULL; /* TBD by OMX to plugin proper callback */
	ISE_connect(imager_info,&events,NULL, ISE_USE_CASE_VID_CAPTURE,ise_hdl);
	printf("Done with Connect now the sensor and CIP has to be in \
				standby mode\n");
	ISE_send_command(ise_hdl,ISE_CMD_SET_PREV_COLOR_FMT,&col_fmt,
				sizeof(ISE_COLOR_FMT_T));
	prv_size.height = cformat.fmt.pix.height;
	prv_size.width = cformat.fmt.pix.width;
	ISE_send_command(ise_hdl,ISE_CMD_SET_PREV_RESOLUTION, &prv_size,
				sizeof(ISE_DIMENSION_T));


	if (ioctl(cfd, VIDIOC_STREAMON, &creqbuf.type) < 0 ) {
		perror("VIDIOC_STREAMON");
		return -1;
	}

	pwr_st = ISE_POWER_STATE_OUT_OF_STANDBY;
	ISE_send_command(ise_hdl,ISE_CMD_SET_POWER_STATE, &pwr_st,
				sizeof(ISE_POWER_STATE_T));
	cap_state = ISE_CAPTURE_STATE_PREVIEW;
	ISE_send_command(ise_hdl, ISE_CMD_SET_CAPTURE_STATE, &cap_state,
				sizeof(ISE_CAPTURE_STATE_T));
	sleep(1);

	i = 0;

	while (i < NUM_BUFS) {
		(cfilledbuffer[i]).type = creqbuf.type;
		/* De-queue the next avaliable buffer */
		while (ioctl(cfd, VIDIOC_DQBUF, &cfilledbuffer[i]) < 0) {
			perror ("VIDIOC_DQBUF");
		}
		i++;
	}
	if (ioctl(cfd, VIDIOC_STREAMOFF, &creqbuf.type) < 0){
		perror("VIDIOC_STREAMOFF");
		return -1;
	}

	pwr_st = ISE_POWER_STATE_IN_STANDBY;
	ISE_send_command(ise_hdl,ISE_CMD_SET_POWER_STATE, 
			&pwr_st, sizeof(ISE_POWER_STATE_T));
	
	i = 0;

	while (i < NUM_BUFS) {

		src_start = cbuffers[(cfilledbuffer[i]).index].start; 
		if (memtype == V4L2_MEMORY_USERPTR) {
			src_start = (char *)(((unsigned int)src_start 
				& 0xffffffe0) + 0x20);
		}

		if (fd_save > 0) {
			if(i == 3){
			/* this compile option allow us to write video to a file */
			 /* we only save some frames */
			write(fd_save, src_start, cformat.fmt.pix.width 
				*cformat.fmt.pix.height*2);
			printf("written %d frame, fd=%d %x\n", i, fd_save,
				(cfilledbuffer[i]).index);
			}
		}

		i++;
	}

	ISE_disconnect(ise_hdl);

	for (i = 0; i < creqbuf.count; i++) {
		if (cbuffers[i].start) {
			if (memtype == V4L2_MEMORY_USERPTR)
				free(cbuffers[i].start);
		}
	}
	free(cbuffers);
	GEN_CAM_PAL_exit();
	close(cfd);
	if (fd_save > 0)
		close(fd_save);
}

