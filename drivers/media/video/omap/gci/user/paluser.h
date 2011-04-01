/*
 * drivers/media/video/omap/gci/user/pal_user.h
 *
 * Top level public header file for PAL user interface in
 * TI's OMAP3430 Camera ISP
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

#ifndef _PALUSER_H
#define _PALUSER_H

#include <asm/arch/gci/omap34xxcam_palkern.h>

#include <linux/types.h>

typedef enum
{
	ISE_PAL_GPIO_IMGR0_PWR = VAUX_2_8_V,
	ISE_PAL_GPIO_IMGR1_PWR = 0,
	ISE_PAL_GPIO_IMGR0_RST = SENSOR_RESET_GPIO,
	ISE_PAL_GPIO_IMGR1_RST = 0,
	ISE_PAL_GPIO_FLASH_ENABLE = 0,
	ISE_PAL_GPIO_FLASH_STROBE = 0,
	ISE_PAL_GPIO_IMGR0_MAIN_PWR = VAUX_DEV_GRP_P1,
	ISE_PAL_GPIO_IMGR1_MAIN_PWR = 0
} ISE_PAL_GPIO_T;

/* A register access handle 
 * that was created by ISE_PAL_reg_open.
 */
typedef unsigned int * ISE_PAL_REG_HANDLE_T;

/* - Delay a given number of microseconds */
void
ISE_PAL_udelay(unsigned int usec);

/* Memory related APIs */

/* Memory allocation in heap */
void *ISE_PAL_mem_alloc(const unsigned int size);

/* Free the memory */
void ISE_PAL_mem_free(void *mem);

/* Sets the memory to a defined value */
void *ISE_PAL_mem_set(void *mem,unsigned char b,unsigned int n);

/* Copy block to block */
void *ISE_PAL_mem_copy(void *mem1,void *mem2,unsigned int n);


ISE_BOOLEAN_T
ISE_PAL_gpio_get(const ISE_PAL_GPIO_T gpio,ISE_BOOLEAN_T *val,
		const void *cookie);


ISE_BOOLEAN_T
ISE_PAL_gpio_set(const ISE_PAL_GPIO_T gpio,ISE_BOOLEAN_T val,
		const void *cookie);

ISE_PAL_REG_STATUS_T
ISE_PAL_reg_open(const ISE_PAL_REG_CONFIG_T *const config,
		ISE_PAL_REG_HANDLE_T *handle,
		const void* cookie);

ISE_PAL_REG_STATUS_T
ISE_PAL_reg_close(ISE_PAL_REG_HANDLE_T *handle);

ISE_PAL_REG_STATUS_T
ISE_PAL_reg_trans(ISE_PAL_REG_HANDLE_T handle,
		ISE_PAL_REG_TRANS_T *transactions,
		unsigned int num_of_trans);

#endif /* _PALUSER_H */
