/*
 * drivers/media/video/omap/gci/user/test/streaming.c
 *
 * Streaming test case case for
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

#include "gci-pal-mapper.h"

#define VIDEO_DEVICE1 "/dev/video1"
#define VIDEO_DEVICE2 "/dev/video2"

#undef SAVE_TO_FILE 
#define SAVE_TO_FILE_NAME "video_out"
#define SAVE_TO_FILE_N_FRAMES 100

#define NUM_BUF 4

//#define SMART_3MP_PARLL_OV 1


static void usage(void)
{
	printf("streaming [brightness][level 1-7] [contrast]");
	printf(" [level 1-7] [color] [1-3]\n");
}


int main (int argc, char *argv[])
{
	struct screen_info_struct {
		int fd ;
		char *data ;
		int width ;
		int height ;
	} screen_info ;
	struct {
		void *start;
		size_t length;
	} *vbuffers,*cbuffers;
	void *src_start;
	struct v4l2_capability capability;
	struct v4l2_format cformat, vformat;
	struct v4l2_requestbuffers creqbuf, vreqbuf;
	struct v4l2_buffer cfilledbuffer, vfilledbuffer;
	int cfd, i, ret, count = -1, memtype = V4L2_MEMORY_MMAP;
	int fd_save = 0;
	int index = 1, vid = 1, set_video_img = 0;
	unsigned int brightness_level = 0, contrast_level = 0;
	unsigned int color = 0;

	ISE_IMAGER_DETAILS_T *imager_info;
	ISE_EVENTS_T events;
	ISE_HANDLE_T ise_hdl;
	ISE_DIMENSION_T prv_size;
	ISE_COLOR_FMT_T col_fmt;
	ISE_POWER_STATE_T pwr_st;
	ISE_CAPTURE_STATE_T cap_state;
	vid =1;
	count = 500;
	

	if ((argc > 1) && (!strcmp(argv[1], "?"))) {
		usage();
		return 0;
	}

	if ((argc > 1) && (!strcmp(argv[1], "brightness"))) {
		if (argc > 2)
			brightness_level = atoi(argv[2]);
	} 
	if ((argc > 3) && (!strcmp(argv[3], "contrast"))){
		if(argc > 4)
			contrast_level = atoi(argv[4]);
	}
	if ((argc > 5) && (!strcmp(argv[5], "color"))){
		if(argc > 6)
			color = atoi(argv[6]);
	}

	cfd = open("/dev/video0", O_RDWR);
	if (cfd == -1) {
		perror("Error...!!! /dev/camera not present.");
	}
	GEN_CAM_PAL_init();

	screen_info.fd = open((vid==1)?VIDEO_DEVICE1:VIDEO_DEVICE2, O_RDWR);
	if (screen_info.fd <= 0) {
		printf("Could no open the device %s\n",(vid==1)?VIDEO_DEVICE1:
					VIDEO_DEVICE2);
		vid = 0;
	}

	if (ioctl(screen_info.fd, VIDIOC_QUERYCAP, &capability) == -1) {
		perror("video VIDIOC_QUERYCAP");
		return -1;
	}
	if ( capability.capabilities & V4L2_CAP_STREAMING)
		;
	else {
		return -1;
	}

	ret = ioctl(cfd, VIDIOC_G_FMT, &cformat);
		if (ret < 0) {
		    perror("Camera VIDIOC_G_FMT");
		    return -1;
		}
//	cformat.fmt.pix.width = 176;
//	cformat.fmt.pix.height = 144;
	printf("Cformat Wd = %d, Ht = %d",cformat.fmt.pix.width,
			cformat.fmt.pix.height);

	ret = ioctl (cfd, VIDIOC_S_FMT, &cformat);
		if (ret < 0) {
			perror ("Camera VIDIOC_S_FMT");
			return -1;
		}

#ifdef SAVE_TO_FILE
	if ((fd_save = creat(SAVE_TO_FILE_NAME, O_RDWR)) <= 0) 
	{
		printf("Can't create file %s\n", SAVE_TO_FILE_NAME);
		fd_save = 0;
	}
#endif
	if (vid != 0) {
		vformat.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
		ret = ioctl(screen_info.fd, VIDIOC_G_FMT, &vformat);
		if (ret < 0) {
		    perror("video VIDIOC_G_FMT");
		    return -1;
		}
		vformat.fmt.pix.width = cformat.fmt.pix.width;
		vformat.fmt.pix.height = cformat.fmt.pix.height;
		vformat.fmt.pix.sizeimage = vformat.fmt.pix.width *
				vformat.fmt.pix.height * 2;
#ifdef SMART_3MP_PARLL_OV
		vformat.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
#else
		vformat.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
#endif
		ret = ioctl (screen_info.fd, VIDIOC_S_FMT, &vformat);
		if (ret < 0) {
			perror ("video VIDIOC_S_FMT");
			return -1;
		}

		vreqbuf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
		vreqbuf.memory = V4L2_MEMORY_MMAP;
		vreqbuf.count = NUM_BUF;
		if (ioctl(screen_info.fd, VIDIOC_REQBUFS, &vreqbuf) == -1) {
			perror ("video VIDEO_REQBUFS");
			return;
		}

		vbuffers = calloc(vreqbuf.count, sizeof(*vbuffers));
		for (i = 0; i < vreqbuf.count; ++i) {
			struct v4l2_buffer buffer;
			buffer.type = vreqbuf.type;
			buffer.index = i;
			if (ioctl(screen_info.fd, VIDIOC_QUERYBUF, &buffer) == -1){
				perror("video VIDIOC_QUERYBUF");
				return;
			}
			vbuffers[i].length= buffer.length;
			vbuffers[i].start = mmap(NULL, buffer.length, PROT_READ|
						PROT_WRITE, MAP_SHARED,
						screen_info.fd, buffer.m.offset);
			if (vbuffers[i].start == MAP_FAILED ){
				perror ("video mmap");
				return;
			}

		}
	}
	
	creqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	creqbuf.memory = memtype;
	creqbuf.count = NUM_BUF;
	if (ioctl(cfd, VIDIOC_REQBUFS, &creqbuf) < 0) {
		perror ("VIDEO_REQBUFS");
		return -1;
	}

	cbuffers = calloc(creqbuf.count, sizeof(*cbuffers));
	/* mmap driver memory or allocate user memory, and queue each buffer */
	for (i = 0; i < creqbuf.count; ++i) {
	struct v4l2_buffer buffer;  
		buffer.type = creqbuf.type;
		buffer.memory = creqbuf.memory;
		buffer.index = i;
		if(ioctl(cfd, VIDIOC_QUERYBUF, &buffer) < 0){
			perror("VIDIOC_QUERYBUF");
			return -1;
		}
	if (memtype == V4L2_MEMORY_USERPTR) {
		buffer.flags = 0;
		buffer.m.userptr = vbuffers[i].start ;
		buffer.length = vbuffers[i].length;
	}
	else {
		cbuffers[i].length= buffer.length;
		cbuffers[i].start = mmap(NULL, buffer.length, PROT_READ |
					PROT_WRITE, MAP_SHARED, cfd,
					buffer.m.offset);
		if (cbuffers[i].start == MAP_FAILED){
			perror("mmap");
			return -1;
		}
		buffer.m.userptr = ((unsigned int)cbuffers[i].start 
				& 0xffffffe0) + 0x20;
		buffer.length = cbuffers[i].length;
	}
		
	if (ioctl(cfd, VIDIOC_QBUF, &buffer) < 0) {
		perror("CAMERA VIDIOC_QBUF");
		return -1;
	}		
	}
	imager_info = (ISE_IMAGER_DETAILS_T*)malloc(sizeof(ISE_IMAGER_DETAILS_T));
	ISE_detect_imager(ISE_IMAGER_PHYS_LOCATION_IMAGER0, NULL, NULL,
				imager_info);
	ise_hdl = (ISE_HANDLE_T)malloc(sizeof(ISE_HANDLE_T));
	events.cip_config = NULL; /* TBD by OMX to plugin proper callback */
	ISE_connect(imager_info,&events,NULL, ISE_USE_CASE_VID_CAPTURE,ise_hdl);
	//printf("Done with Connect now the sensor and CIP has to be in standby mode\n");

#ifdef SMART_3MP_PARLL_OV
	/* TBD for smart*/
	col_fmt = ISE_COLOR_FMT_422_YUYV;
#endif
	ISE_send_command(ise_hdl,ISE_CMD_SET_PREV_COLOR_FMT,&col_fmt,
				sizeof(ISE_COLOR_FMT_T));

	prv_size.height = cformat.fmt.pix.height;
	prv_size.width = cformat.fmt.pix.width;
	ISE_send_command(ise_hdl,ISE_CMD_SET_PREV_RESOLUTION,
				&prv_size, sizeof(ISE_DIMENSION_T));
	
	/* turn on streaming */
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
	if (ioctl(screen_info.fd, VIDIOC_STREAMON, &vreqbuf.type) < 0 ) {
		perror("video VIDIOC_STREAMON");
		return -1;
	}

	/* caputure 1000 frames or when we hit the passed nmuber of frames */
	cfilledbuffer.type = creqbuf.type;
	vfilledbuffer.type = vreqbuf.type;
	i = 0;
	vfilledbuffer.index = -1;
	while (i <= 1000) {
		/* De-queue the next avaliable buffer */
		while (ioctl(cfd, VIDIOC_DQBUF, &cfilledbuffer) < 0) {
			perror ("VIDIOC_DQBUF");
			if (vid != 0) {
				while (ioctl(screen_info.fd, VIDIOC_QBUF,
					&vfilledbuffer) < 0) {
					perror ("VIDIOC_QBUF***");
				}
			}
			else {
				while (ioctl(cfd, VIDIOC_QBUF, &cfilledbuffer)
					< 0) {
					perror("VIDIOC_QBUF***");
				}
			}
		}
		i++;

		if (memtype == V4L2_MEMORY_MMAP) { 
			memcpy(vbuffers[cfilledbuffer.index].start,
			cbuffers[cfilledbuffer.index].start,
			cfilledbuffer.length);
		}

		src_start = cbuffers[cfilledbuffer.index].start; 
		if (memtype == V4L2_MEMORY_USERPTR) {
			src_start = (char *)(((unsigned int)src_start 
				& 0xffffffe0) + 0x20);
		}

		if (fd_save > 0) {
			/* this compile option allow us to write video to a file */
			if (i <= 50) { /* we only save some frames */
			write(fd_save, src_start, cformat.fmt.pix.width 
				*cformat.fmt.pix.height*2);
			printf("written %d frame, fd=%d\n", i, fd_save);
			}
			if ((fd_save > 0) && (i == 50))
				close(fd_save);
		}

		if (vfilledbuffer.index != -1) {
		/* De-queue the previous buffer from video driver */
			if (ioctl(screen_info.fd, VIDIOC_DQBUF, &vfilledbuffer) < 0) {
				perror ("cam VIDIOC_DQBUF");
				return;
			}
		}
		vfilledbuffer.index = cfilledbuffer.index; 
		//if(i>=25)
		/* Queue the new buffer to video driver for rendering */
		if (ioctl(screen_info.fd, VIDIOC_QBUF, &vfilledbuffer) == -1){
			perror ("video VIDIOC_QBUF");
			return;
		}
#ifdef SMART_3MP_PARLL_OV
		if (i == 200) {
			ISE_send_command(ise_hdl,ISE_CMD_SET_BRIGHTNESS, 
				&brightness_level, sizeof(unsigned int));
		}
		if (i == 400) {
			ISE_send_command(ise_hdl,ISE_CMD_SET_CONTRAST, 
				&contrast_level, sizeof(unsigned int));
		}
		if (i == 600) {
			if(color == 1)
				color = ISE_STYLE_REDDISH;
			else if(color == 2)
				color = ISE_STYLE_SEPIA;
			else if(color == 3)
				color = ISE_STYLE_BLACK_WHITE;
			ISE_send_command(ise_hdl,ISE_CMD_SET_STYLE, 
				&color, sizeof(ISE_STYLE_T));
		}
#endif
		if (i == 800) {
			creqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			if (ioctl(cfd, VIDIOC_STREAMOFF, &creqbuf.type) < 0){
				perror("VIDIOC_STREAMOFF");
				return -1;
			}
			pwr_st = ISE_POWER_STATE_IN_STANDBY;
			ISE_send_command(ise_hdl,ISE_CMD_SET_POWER_STATE, 
				&pwr_st, sizeof(ISE_POWER_STATE_T));
		//	printf("Done\n");
			break;
		}

		while (ioctl(cfd, VIDIOC_QBUF, &cfilledbuffer) < 0) {
			perror ("CAM VIDIOC_QBUF");
		}
	}
	/* we didn't trun off streaming yet */
	if (count == -1) {
		pwr_st = ISE_POWER_STATE_IN_STANDBY;
		ISE_send_command(ise_hdl,ISE_CMD_SET_POWER_STATE, 
				&pwr_st, sizeof(ISE_POWER_STATE_T));
		if (vid != 0) {
			if (ioctl(screen_info.fd, VIDIOC_STREAMOFF, &vreqbuf.type) == -1) {
				perror("video VIDIOC_STREAMOFF");
				return -1;
			}

		}
	}
	ISE_disconnect(ise_hdl);
	for (i = 0; i < vreqbuf.count; i++) {
		if (vbuffers[i].start) {
			munmap(vbuffers[i].start, vbuffers[i].length);
		}
	}

	free(vbuffers);


	for (i = 0; i < creqbuf.count; i++) {
		if (cbuffers[i].start) {
			if (memtype == V4L2_MEMORY_USERPTR)
				free(cbuffers[i].start);
			else
				munmap(cbuffers[i].start, cbuffers[i].length);
		}
	}
	free(cbuffers);
	GEN_CAM_PAL_exit();
	close(screen_info.fd);
	close(cfd);
	if (fd_save > 0)
		close(fd_save);
}
