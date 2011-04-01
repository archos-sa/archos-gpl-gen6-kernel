/*
 * drivers/media/video/omap/gci/user/pal_user.c
 *
 * PAL user implementation for TI's OMAP3430 Camera ISP. 
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
#include <linux/ioctl.h>
#include "paluser.h"

int fd;

/*Utility Interface APIs */
/* - Delay a given number of microseconds */
void
ISE_PAL_udelay(unsigned int usec)
{
	int ret;
	ret = ioctl(fd,GEN_CAM_PAL_UDELAY,usec);
	/* Not using usleep since it seems not to 
	 * be effective for delays between i2c transactions */
#if 0 
	usleep(usec);
#endif
}

/* Memory allocation in heap */
void *
ISE_PAL_mem_alloc(const unsigned int size)
{
	return (void*)malloc(size);
}

/* Free the memory */
void 
ISE_PAL_mem_free(void *mem)
{
	free(mem);
}

/* Sets the memory to a defined value */
void *
ISE_PAL_mem_set(void *mem,unsigned char b,unsigned int n)
{
	return (void*)memset(mem, b, n);
}

/* Copy block to block */
void *
ISE_PAL_mem_copy(void *mem1,void *mem2,unsigned int n)
{
	return (void*)memcpy(mem1, mem2, n);
}


/* Reads the value of a gpio pin*/
ISE_BOOLEAN_T
ISE_PAL_gpio_get(const ISE_PAL_GPIO_T gpio,ISE_BOOLEAN_T *val,
		const void *cookie)
{
	int ret;
	struct gpio_data gpiodata;
	gpiodata.gpio_num = gpio;
	ret = ioctl(fd,GEN_CAM_PAL_GPIO_GET,gpiodata);
	return ret;
}

/* Acquires and writes data to a gpio pin*/
ISE_BOOLEAN_T
ISE_PAL_gpio_set(const ISE_PAL_GPIO_T gpio,ISE_BOOLEAN_T val,
		const void *cookie)
{
	int ret;
	unsigned int t2_vaux;
	struct gpio_data gpiodata;
	
	t2_vaux = gpio >> 8;
	if(t2_vaux){
		if ((t2_vaux >> 8))
			/* 32bit value for FPGA vio enable */
			ret = ret = ioctl(fd, GEN_CAM_PAL_T2_VAUX_SET,
					gpio);
		else
			ret = ioctl(fd, GEN_CAM_PAL_T2_VAUX_SET,
					t2_vaux);
	}
	else {
		/* Gpio is configured as Output for Camera*/
		gpiodata.is_input = 0;
		gpiodata.gpio_num = gpio;
		gpiodata.value = val;
		ret = ioctl(fd,GEN_CAM_PAL_GPIO_SET,&gpiodata);
	}
	return ret;
}


/* Register transaction */
ISE_PAL_REG_STATUS_T
ISE_PAL_reg_trans(ISE_PAL_REG_HANDLE_T handle,
		ISE_PAL_REG_TRANS_T *transactions,
		unsigned int num_of_trans)
{
	int ret;
	struct pal_nregtrans regtrans;
	regtrans.alltrans = transactions;
	regtrans.nregtrans = num_of_trans;
	ret = ioctl(fd,GEN_CAM_PAL_REG_TRANSFER,&regtrans);
	return ret;
}

/* Invokes reg_open ioctl which initialises psuedoi2c
 * for the first time. Distinguishes if it is a i2c transaction
 * or the cam_isc transaction.
 */
ISE_PAL_REG_STATUS_T
ISE_PAL_reg_open(const ISE_PAL_REG_CONFIG_T *const config,
		ISE_PAL_REG_HANDLE_T *handle,
		const void* cookie)
{
	int ret;
	/* Send the i2c info to PAL kern which is then
	 * handled by psuedo i2c 
	 */
	ret = ioctl(fd,GEN_CAM_PAL_REG_OPEN,config);
	handle = (ISE_PAL_REG_HANDLE_T*)fd;
	if(ret == -1)
		return ISE_PAL_REG_STATUS_FAIL;
	if(ret == -2) //EINVAL
		return ISE_PAL_REG_STATUS_INVAL_PARAM;
	else 
		return ISE_PAL_REG_STATUS_OK;
}

ISE_PAL_REG_STATUS_T
ISE_PAL_reg_close(ISE_PAL_REG_HANDLE_T *handle)
{
	handle = NULL;
}

/* Linux OS related APIs to keep the
 * ISC and PAL kern modules ready to recieve
 * the ISE APIs
 */

/* GEN_CAM_Pal driver cleanup*/
void GEN_CAM_PAL_exit()
{
	close(fd);
}

/* GEN_CAM_Pal driver initilisation*/
int GEN_CAM_PAL_init()
{
	int ret;

	fd = open("/dev/omap_palkern", O_RDWR);
	if (fd == -1) {
		perror("Error...!!! /dev/omap_palkern not present.");
	}
	return 0;
}

