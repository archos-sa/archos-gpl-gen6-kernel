/*
 * drivers/media/video/omap/isp/omap_previewer.c
 *
 * Wrapper for Preview module in TI's OMAP3430 ISP
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/scatterlist.h>
#include <asm/arch/io.h>
#include "isp.h"
#include "ispmmu.h"
#include "ispreg.h"
#include "omap_previewer.h"

#define OMAP_PREV_NAME		"omap-previewer"

/* global variable of type cdev to register driver to the kernel */
static struct cdev cdev;
/* global variable which keeps major and minor number of the driver in it */
static dev_t devt;
struct device *prev_dev;
static struct class *prev_class = NULL;
/* global object of prev_device structure */
struct prev_device prevdevice;
static struct platform_driver omap_previewer_driver;

/* inline function to free reserver pages  */
void inline prev_free_pages(unsigned long addr, unsigned long bufsize)
{
	unsigned long size, ad = addr;
	size = PAGE_SIZE << (get_order(bufsize));
	if (!addr)
		return;
	while (size > 0) {
		ClearPageReserved(virt_to_page(addr));
		addr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
	free_pages(ad, get_order(bufsize));
}


/* prev_calculate_crop: This function is used to calculate frame size 
   reduction depending on the features enabled by the application. */
void prev_calculate_crop(struct prev_params *config, struct prev_cropsize *crop)
{
	dev_dbg(prev_dev, "prev_calculate_crop E\n");

	if (!config || !crop) {

		dev_err(prev_dev, "\nErron in argument");
		return;
	}

	crop->hcrop = crop->vcrop = 0;

	/* Horizontal medial filter reduces image width by 4 pixels 
	   2 right and 2 left */
	if (config->features & PREV_HORZ_MEDIAN_FILTER) {
		crop->hcrop += 4;
	}
	/* Noise filter reduces image height and width 2 pixels from 
	   top, left, right and bottom */
	if (config->features & PREV_NOISE_FILTER) {
		crop->hcrop += 4;
		crop->vcrop += 4;
	}
	/* CFA Interpolation reduces image height and width 2 pixels 
	   from top, left, right and bottom */
	if (config->features & PREV_CFA) {
		crop->hcrop += 4;
		crop->vcrop += 4;
	}
	/* Luma enhancement reduces image width 1 pixels from left, right */
	if (config->features & (PREV_LUMA_ENHANCE | PREV_CHROMA_SUPPRESS)) {
		crop->hcrop += 2;
	}
	dev_dbg(prev_dev, "prev_calculate_crop L\n");
}

int get_status(struct prev_status *status)
{
	int ret = 0;
	if (!status) {
		dev_err(prev_dev, "get_status:invalid parameter\n");
		return -EINVAL;
	}

	status->hw_busy = (char)isppreview_busy();

	return ret;
}


int request_buffer(struct prev_device *device, struct prev_reqbufs *reqbufs)
{
	struct prev_buffer *buffer = NULL;
	int count = 0;
	unsigned long adr;
	u32 size;

	if (!reqbufs || !device) {
		dev_err(prev_dev, "request_buffer: error in argument\n");
		return -EINVAL;
	}

	/* if number of buffers requested is more then support return error */
	if (reqbufs->count > MAX_BUFFER) {
		dev_err(prev_dev, "request_buffer: invalid buffer count\n");
		return -EINVAL;
	}

	/* if buf_type is input then allocate buffers for input */
	if (reqbufs->buf_type == PREV_BUF_IN) {
		/*if buffer count is zero, free all the buffers */
		if (reqbufs->count == 0) {
			/* free all the buffers */
			for (count = 0; count < device->in_numbuffers; count++) {
				/* free memory allocate for the image */
				if (device->in_buff[count]) {
					adr =
					    (unsigned long)device->
					    in_buff[count]->offset;
					if (adr)
						prev_free_pages((unsigned long)
								phys_to_virt
								(adr),
								device->in_buff
								[count]->size);

					/* free the memory allocated 
					   to prev_buffer */
					kfree(device->in_buff[count]);

					device->in_buff[count] = NULL;
				}
			}
			device->in_numbuffers = 0;
			return 0;
		}

		/* free the extra buffers */
		if (device->in_numbuffers > reqbufs->count &&
		    reqbufs->size == device->in_buff[0]->size) {
			for (count = reqbufs->count;
			     count < device->in_numbuffers; count++) {
				/* free memory allocate for the image */
				if (device->in_buff[count]) {
					adr = device->in_buff[count]->offset;
					if (adr)
						prev_free_pages((unsigned long)
								phys_to_virt
								(adr),
								device->in_buff
								[count]->size);

					/* free the memory allocated 
					   to prev_buffer */
					kfree(device->in_buff[count]);

					device->in_buff[count] = NULL;
				}
			}
			device->in_numbuffers = reqbufs->count;
			return 0;
		}
		/* if size requested is different from already allocated, 
		   free memory of all already allocated buffers */
		if (device->in_numbuffers) {
			if (reqbufs->size != device->in_buff[0]->size) {
				for (count = 0;
				     count < device->in_numbuffers; count++) {
					if (device->in_buff[count]) {
						adr =
						    device->
						    in_buff[count]->offset;
						if (adr)
							prev_free_pages((unsigned long)
									phys_to_virt
									(adr),
									device->
									in_buff
									[count]->
									size);

						kfree(device->in_buff[count]);

						device->in_buff[count] = NULL;
					}
				}
				device->in_numbuffers = 0;
			}
		}

		/* allocate the buffer */
		for (count = device->in_numbuffers; count < reqbufs->count;
		     count++) {
			/* Allocate memory for struct prev_buffer */
			buffer =
			    kmalloc(sizeof(struct prev_buffer), GFP_KERNEL);

			/* if memory allocation fails then return error */
			if (!buffer) {
				/* free all the buffers */
				while (--count >= device->in_numbuffers) {
					adr = device->in_buff[count]->offset;
					if (adr)
						prev_free_pages((unsigned long)
								phys_to_virt
								(adr),
								device->in_buff
								[count]->size);
					kfree(device->in_buff[count]);
					device->in_buff[count] = NULL;
				}
				dev_err(prev_dev, "request_buffer:not \
					enough memory\n");
				return -ENOMEM;
			}

			/* assign buffer's address in configuration */
			device->in_buff[count] = buffer;

			/* set buffers index and buf_type,size parameters */
			buffer->index = count;
			buffer->buf_type = PREV_BUF_IN;
			buffer->size = reqbufs->size;
			/* allocate memory for buffer of size passed 
			   in reqbufs */
			buffer->offset =
			    (unsigned long)__get_free_pages(GFP_KERNEL |
							    GFP_DMA,
							    get_order
							    (reqbufs->size));

			/* if memory allocation fails, return error */
			if (!(buffer->offset)) {
				/* free all the buffer's space */
				kfree(buffer);
				device->in_buff[count] = NULL;
				while (--count >= device->in_numbuffers) {
					adr = device->in_buff[count]->offset;
					if (adr)
						prev_free_pages((unsigned long)
								phys_to_virt
								(adr),
								device->in_buff
								[count]->size);
					kfree(device->in_buff[count]);
					device->in_buff[count] = NULL;
				}
				dev_err(prev_dev, "request_buffer:not "
					"enough memory\n");

				return -ENOMEM;
			}

			adr = (unsigned long)buffer->offset;
			size = PAGE_SIZE << (get_order(reqbufs->size));
			while (size > 0) {
				/* make sure the frame buffers 
				   are never swapped out of memory */
				SetPageReserved(virt_to_page(adr));
				adr += PAGE_SIZE;
				size -= PAGE_SIZE;
			}
			/* convert vertual address to physical */
			buffer->offset = (unsigned long)
			    virt_to_phys((void *)(buffer->offset));
		}
		device->in_numbuffers = reqbufs->count;
	}
	/* if buf_type is output then allocate buffers for output */
	else if (reqbufs->buf_type == PREV_BUF_OUT) {
		if (reqbufs->count == 0) {
			/* free all the buffers */
			for (count = 0;
			     count < device->out_numbuffers; count++) {
				/* free memory allocate for the image */
				if (device->out_buff[count]) {
					adr = device->out_buff[count]->offset;
					if (adr)
						prev_free_pages((unsigned long)
								phys_to_virt
								(adr),
								device->out_buff
								[count]->size);

					/* free the memory allocated to 
					   prev_buffer */
					kfree(device->out_buff[count]);

					device->out_buff[count] = NULL;
				}
			}
			device->out_numbuffers = 0;

			return 0;
		}
		/* free the buffers */
		if (device->out_numbuffers > reqbufs->count &&
		    reqbufs->size == device->out_buff[0]->size) {
			for (count = reqbufs->count;
			     count < device->out_numbuffers; count++) {
				/* free memory allocate for the image */
				if (device->out_buff[count]) {
					adr = device->out_buff[count]->offset;
					if (adr)
						prev_free_pages((unsigned long)
								phys_to_virt
								(adr),
								device->out_buff
								[count]->size);

					/* free the memory allocated to 
					   prev_buffer */
					kfree(device->out_buff[count]);

					device->out_buff[count] = NULL;
				}
			}
			device->out_numbuffers = reqbufs->count;

			return 0;
		}
		/* if size requested is different from already allocated, 
		   free memory of all already allocated buffers */
		if (device->out_numbuffers) {
			if (reqbufs->size != device->out_buff[0]->size) {
				for (count = 0;
				     count < device->out_numbuffers; count++) {
					if (device->out_buff[count]) {
						adr =
						    device->
						    out_buff[count]->offset;

						if (adr)
							prev_free_pages((unsigned long)
									phys_to_virt
									(adr),
									device->
									out_buff
									[count]->
									size);

						kfree(device->out_buff[count]);

						device->out_buff[count] = NULL;
					}
				}
				device->out_numbuffers = 0;
			}
		}

		/* allocate the buffer */
		for (count = device->out_numbuffers;
		     count < reqbufs->count; count++) {
			/* Allocate memory for struct prev_buffer */
			buffer =
			    kmalloc(sizeof(struct prev_buffer), GFP_KERNEL);

			/* if memory allocation fails then return error */
			if (!buffer) {
				/* free all the buffers */
				while (--count >= device->out_numbuffers) {
					adr = device->out_buff[count]->offset;
					if (adr)
						prev_free_pages((unsigned long)
								phys_to_virt
								(adr),
								device->out_buff
								[count]->size);
					kfree(device->out_buff[count]);
					device->out_buff[count] = NULL;
				}

				dev_err(prev_dev, "request_buffer:not enough "
					"memory\n");

				return -ENOMEM;
			}

			/* assign buffer's address out configuration */
			device->out_buff[count] = buffer;

			/* set buffers outdex and buf_type,size parameters */
			buffer->index = count;
			buffer->buf_type = PREV_BUF_OUT;
			buffer->size = reqbufs->size;
			/* allocate memory for buffer of size passed 
			   in reqbufs */
			buffer->offset =
			    (unsigned long)__get_free_pages(GFP_KERNEL |
							    GFP_DMA,
							    get_order
							    (reqbufs->size));

			/* if memory allocation fails, return error */
			if (!(buffer->offset)) {
				/* free all the buffer's space */
				kfree(buffer);
				device->out_buff[count] = NULL;
				while (--count >= device->out_numbuffers) {
					adr = device->out_buff[count]->offset;
					if (adr)
						prev_free_pages((unsigned long)
								phys_to_virt
								(adr),
								device->out_buff
								[count]->size);
					kfree(device->out_buff[count]);
					device->out_buff[count] = NULL;
				}
				dev_err(prev_dev, "request_buffer:not "
					"enough memory\n");

				return -ENOMEM;
			}

			adr = (unsigned long)buffer->offset;
			size = PAGE_SIZE << (get_order(reqbufs->size));
			while (size > 0) {
				/* make sure the frame buffers 
				   are never swapped out of memory */
				SetPageReserved(virt_to_page(adr));
				adr += PAGE_SIZE;
				size -= PAGE_SIZE;
			}
			/* convert vertual address to physical */
			buffer->offset = (unsigned long)
			    virt_to_phys((void *)(buffer->offset));
		}
		device->out_numbuffers = reqbufs->count;
	} else {
		dev_err(prev_dev, "request_buffer: invalid buffer type\n");
		return -EINVAL;
	}

	return 0;
}

/* querybuffer: This function will query the buffers physical address
     whose index is passed in prev_buffer. it will store that address 
	in prev_buffer. */
int query_buffer(struct prev_device *device, struct prev_buffer *buffer)
{

	if (!buffer || !device) {
		dev_err(prev_dev, "query_buffer: error in argument\n");
		return -EINVAL;
	}

	/* if buf_type is input buffer then get offset of input buffer */
	if (buffer->buf_type == PREV_BUF_IN) {
		/* error checking for wrong index number */
		if (buffer->index >= device->in_numbuffers) {
			dev_err(prev_dev, "query_buffer: invalid index");
			return -EINVAL;
		}

		/* get the offset and size of the buffer and store 
		   it in buffer */
		buffer->offset = device->in_buff[buffer->index]->offset;
		buffer->size = device->in_buff[buffer->index]->size;
	}
	/* if buf_type is output buffer then get offset of output buffer */
	else if (buffer->buf_type == PREV_BUF_OUT) {
		/* error checking for wrong index number */
		if (buffer->index >= device->out_numbuffers) {
			dev_err(prev_dev, "query_buffer: invalid index\n");
			return -EINVAL;
		}
		/* get the offset and size of the buffer and store 
		   it in buffer */
		buffer->offset = device->out_buff[buffer->index]->offset;
		buffer->size = device->out_buff[buffer->index]->size;
	} else {
		dev_err(prev_dev, "query_buffer: invalid buffer type\n");
		return -EINVAL;
	}

	return 0;
}

int prev_hw_setup(struct prev_params *config)
{
	if (config->features & PREV_AVERAGER) {
		isppreview_config_averager(config->average);
	} else
		isppreview_config_averager(0);

	if (config->features & PREV_INVERSE_ALAW)
		isppreview_enable_invalaw(1);
	else
		isppreview_enable_invalaw(0);

	if (config->features & PREV_HORZ_MEDIAN_FILTER) {
		isppreview_config_hmed(config->hmf_params);
		isppreview_enable_hmed(1);
	} else
		isppreview_enable_hmed(0);

	if (config->features & PREV_DARK_FRAME_SUBTRACT) {
		isppreview_set_darkaddr(config->drkf_params.addr);
		isppreview_config_darklineoffset(config->drkf_params.offset);
		isppreview_enable_drkframe(1);
	} else
		isppreview_enable_drkframe(0);

	if (config->features & PREV_LENS_SHADING) {
		isppreview_config_drkf_shadcomp(config->lens_shading_shift);
		isppreview_enable_shadcomp(1);
	} else
		isppreview_enable_shadcomp(0);
	
	return 0;
}

int validate_params(struct prev_params *params)
{
	struct prev_cropsize crop;

	if (!params) {
		printk("validate_params: error in argument");
		return -EINVAL;
	}

	prev_calculate_crop(params, &crop);

	/* check whether down sampling rate is one of the supported */
	if ((params->features & PREV_AVERAGER) == PREV_AVERAGER) {
		if ((params->average != NO_AVE)
		    && (params->average != AVE_2_PIX)
		    && (params->average != AVE_4_PIX)
		    && (params->average != AVE_8_PIX)) {
			/* if not return error */
			printk("validate_params: wrong pix average\n");
			return -EINVAL;
		} else if (((params->average == AVE_2_PIX)
				&& (params->size_params.hsize % 2)) ||
				((params->average == AVE_4_PIX)
			    	&& (params->size_params.hsize % 4)) ||
				((params->average == AVE_8_PIX)
			    	&& (params->size_params.hsize % 8))) {
			printk("validate_params: "
				"wrong pix average for input size\n");
			return -EINVAL;	
		}
	}

	/* check for valid values of pixel size */
	if (params->size_params.pixsize != PREV_INWIDTH_8BIT
	    && params->size_params.pixsize != PREV_INWIDTH_10BIT) {
		printk("validate_params: wrong pixsize\n");
		return -EINVAL;
	}

	/* check whether size of the image is within limit */
	if ((params->size_params.hsize) > MAX_IMAGE_WIDTH + crop.hcrop
	    || (params->size_params.hsize) < 0) {
		printk("validate_params: wrong hsize\n");
		return -EINVAL;
	}

	if ((params->size_params.vsize) > MAX_IMAGE_HEIGHT + crop.vcrop
	    || (params->size_params.vsize) < 0) {
		printk("validate_params: wrong vsize\n");
		return -EINVAL;
	}

	/* check for valid values output pixel format */
	if (params->pix_fmt != YCPOS_YCrYCb &&
	    YCPOS_YCbYCr != params->pix_fmt &&
	    YCPOS_CbYCrY != params->pix_fmt &&
	    YCPOS_CrYCbY != params->pix_fmt) {
		printk("validate_params: wrong pix_fmt");
		return -EINVAL;
	}

	/* dark frame capture and subtract should not be enabled 
	   at the same time */
	if ((params->features & PREV_DARK_FRAME_SUBTRACT) &&
	    (params->features & PREV_DARK_FRAME_CAPTURE)) {
	    	printk("validate_params: DARK FRAME CAPTURE and "
		       "SUBSTRACT cannot be enabled at same time\n");
		return -EINVAL;
	}

	/* check to see dark frame address should not be null */
	if (params->features & PREV_DARK_FRAME_SUBTRACT)
		if (!(params->drkf_params.addr)
		    || (params->drkf_params.offset % 32)) {
			printk("validate_params: dark frame address\n");
			return -EINVAL;
		}

	/* check to see lens shading shift value should not be greater 
	   than 7 */
	if (params->features & PREV_LENS_SHADING)
		if ((params->lens_shading_shift > 7)
		    || !(params->drkf_params.addr)
		    || (params->drkf_params.offset % 32)) {
		    	printk("validate_params: lens shading shift\n");
			return -EINVAL;
		}

	/* if pitch is zero assign it to the width of the image */
	if (params->size_params.in_pitch <= 0
	    || params->size_params.in_pitch % 32) {
		params->size_params.in_pitch = 
				(params->size_params.hsize * 2) & 0xFFE0;
		printk("\nError in in_pitch; new value = %d",
			params->size_params.in_pitch);
	}

	if (params->size_params.out_pitch <= 0
	    || params->size_params.out_pitch % 32) {
		params->size_params.out_pitch = 
				((params->size_params.hsize - crop.hcrop) * 2)
				& 0xFFE0;
		printk("\nError in out_pitch; new value = %d",
			params->size_params.in_pitch);
	}

	return 0;
}

/* This function is used to free memory allocated to buffers */
int free_buffers(struct prev_device *device)
{
	int i;
	unsigned long adr;

	if (!device) {
		printk("\nfree_buffers: error in argument");
		return -EINVAL;
	}
	/* free memory allocated to in buffers */
	for (i = 0; i < device->in_numbuffers; i++) {
		if (device->in_buff[i]) {
			adr = device->in_buff[i]->offset;
			if (adr)
				prev_free_pages((unsigned long)
						phys_to_virt(adr),
						device->in_buff[i]->size);

			kfree(device->in_buff[i]);
			device->in_buff[i] = NULL;
		}
	}
	device->in_numbuffers = 0;
	/* free memory allocated to out buffers */
	for (i = 0; i < device->out_numbuffers; i++) {
		if (device->out_buff[i]) {
			adr = device->out_buff[i]->offset;
			if (adr)
				prev_free_pages((unsigned long)
						phys_to_virt(adr),
						device->out_buff[i]->size);

			kfree(device->out_buff[i]);
			device->out_buff[i] = NULL;
		}
	}

	device->out_numbuffers = 0;

	return 0;
}

/* Helper functions */
static inline void set_size(int hstart, int vstart, int width, int height)
{
	u32 horz_info = (width - 1 + hstart) & 0x3fff;
	u32 vert_info = (height - 1 + vstart) & 0x3fff;
	
	horz_info |= ((hstart & 0x3fff) << 16);
	vert_info |= ((vstart & 0x3fff) << 16);
	omap_writel(horz_info, ISPPRV_HORZ_INFO);
	omap_writel(vert_info, ISPPRV_VERT_INFO);
}


/*
 * Callback from ISP driver for ISP Preview Interrupt
 * status 	: IRQ0STATUS
 * arg1		: Not used as of now.
 * arg2		: Not used as of now.
 */
static void
preview_isr(unsigned long status, void *arg1, void *arg2)
{
	struct prev_device *prevdevice = (struct prev_device *)arg1;

	if ((PREV_DONE & status) != PREV_DONE)
		return;

	if (prevdevice)
		complete(&(prevdevice->wfc));

}

/*
 * Performs the Preview process
 */ 
int preview(struct prev_device *device, struct prev_convert *convert)
{
	int bpp, size, cropsize;
	unsigned long in_addr, out_addr;
	unsigned long in_addr_mmu, out_addr_mmu;
	struct prev_cropsize crop;
	int ret = 0;
	u32 out_hsize, out_vsize, out_line_offset;

	/* error checking */
	if (!convert || !device) {
		dev_err(prev_dev, "preview: invalid convert parameters\n");
		return -EINVAL;
	}

	/* Call prev_calculate_crop to calculate size reduction in 
	   input image */
	prev_calculate_crop(device->params, &crop);
	/* Set bytes per pixel */
	if (device->params->size_params.pixsize == PREV_INWIDTH_8BIT)
		bpp = 1;
	else
		bpp = 2;

	size = device->params->size_params.hsize *
	    device->params->size_params.vsize * bpp;
	cropsize =
	    2 * (crop.vcrop * device->params->size_params.hsize +
		 crop.hcrop * (device->params->size_params.vsize - crop.vcrop));
	/* configure input buffer's address */
	/* If index member of in_buff of arg is less than 0 then */
	if (convert->in_buff.index < 0) {
		/* If size member of in_buff of arg is less than the size 
		   specified in size_params member of prev_params */
		if (convert->in_buff.size < size)
			return -EINVAL;

		/* Check for 32 byte aligned address */
		if (convert->in_buff.offset % 32 || !convert->in_buff.offset)
			return -EINVAL;

		/* Set address in RSDR_ADDR */
		in_addr = convert->in_buff.offset;
	} else {
		/* Check for valid index */
		if (convert->in_buff.index > device->in_numbuffers) {
			dev_err(prev_dev, "\ninvalid index");
			return -EINVAL;
		}

		/* check for size validity */
		if (size > device->in_buff[convert->in_buff.index]->size) {
			dev_err(prev_dev, "\nsize incorrect size = %d", size);
			return -EINVAL;
		}
		in_addr =
		    (unsigned long)device->in_buff[convert->in_buff.
						   index]->offset;
	}

	if (convert->out_buff.index < 0) {
		/* If size member of in_buff of arg is less than the size 
		   specified in size_params member of prev_params */
		if (convert->out_buff.size < (2 * size / bpp - cropsize))
			return -EINVAL;

		/* Check for 32 byte aligned address */
		if (convert->out_buff.offset % 32 || !convert->out_buff.offset)
			return -EINVAL;

		/* Set address in WSDR_ADDR */
		out_addr = convert->out_buff.offset;
	} else {
		/* Check for valid index */
		if (convert->out_buff.index > device->out_numbuffers) {
			dev_err(prev_dev, "\ninvalid index");
			return -EINVAL;
		}

		/* check for size validity */
		if ((2 * size / bpp - cropsize) >
		    device->out_buff[convert->out_buff.index]->size) {
			dev_err(prev_dev, "\nsize incorrect size");
			return -EINVAL;
		}
		out_addr =
		    (unsigned long)device->out_buff[convert->out_buff.
						    index]->offset;
	}
	in_addr_mmu = ispmmu_map(in_addr, size);
	out_addr_mmu = ispmmu_map(out_addr, size);

	/* Set input - output addresses */
	ret = isppreview_set_inaddr(in_addr_mmu);
	if (ret)
		return ret;

	ret = isppreview_set_outaddr(out_addr_mmu);
	if (ret)
		return ret;

	isppreview_try_size(device->params->size_params.hsize,
			    device->params->size_params.vsize,
			    &out_hsize, &out_vsize);

	ret = isppreview_config_inlineoffset(device->params->size_params.hsize
						* bpp);
	if (ret)
		return ret;

	out_line_offset = out_hsize * bpp;
	while ((out_line_offset & PREV_32BYTES_ALIGN_MASK)!= out_line_offset)
		out_line_offset++;

	ret = isppreview_config_outlineoffset(out_line_offset);
	if (ret)
		return ret;

	isppreview_config_size(device->params->size_params.hsize,
			       device->params->size_params.vsize,
			       out_hsize, out_vsize);

	/* Set input source to DDRAM */
	isppreview_config_datapath(PRV_RAW_MEM, PREVIEW_MEM);

	ret = isp_set_callback(CBK_PREV_DONE, preview_isr, (void *)&prevdevice,
		       (void *)NULL);
	if (ret) {
		printk("ERROR while setting Previewer callback!\n");
		return ret;
	}
	/* enable previewer which starts previewing */
	isppreview_enable(1);

	/* wait untill processing is not completed */
	wait_for_completion_interruptible(&(device->wfc));

	if (in_addr_mmu) {
		ispmmu_unmap(in_addr_mmu);
		in_addr_mmu = 0;
	}
	if (out_addr_mmu) {
		ispmmu_unmap(out_addr_mmu);
		out_addr_mmu = 0;
	}
	ret = isp_unset_callback(CBK_PREV_DONE);
	return ret;
}


int previewer_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct prev_device *device = &prevdevice;
	struct prev_params *config = isppreview_get_config();

	if (config == NULL) {
		printk("Unable to initialize default config "
			"from isppreviewer!!!\n\n");
		return -EACCES;
	}

	if (device->opened || filp->f_flags & O_NONBLOCK) {
		dev_err
		    (prev_dev, "previewer_open: device is already openend\n");
		return -EBUSY;
	}

	isp_get();
	ret = isppreview_request();
	if (ret) {
		isp_put();
		printk("Can't acquire isppreview\n");
		return ret;
	}

	/* initialize mutex to 0 */
	prevdevice.params = config;
	prevdevice.opened = 1;
	prevdevice.in_numbuffers = 0;
	prevdevice.out_numbuffers = 0;

	init_completion(&(prevdevice.wfc));
	prevdevice.wfc.done = 0;
	init_MUTEX(&(prevdevice.sem));
	
	return 0;
}


int previewer_release(struct inode *inode, struct file *filp)
{
	struct prev_device *device = &prevdevice;

	/* call free_buffers to free memory allocated to buffers */
	free_buffers(device);

	/* change the device status to available */
	device->opened = 0;
	prevdevice.params = NULL;
	isppreview_free();
	isp_put();
	return 0;
}


int previewer_mmap(struct file *file, struct vm_area_struct *vma)
{
	/* get the address of global object of prev_device structure */
	struct prev_device *device = &prevdevice;
	int i, flag = 0;
	/* get the page offset */
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	/* page offset passed in mmap should one from input buffers */
	for (i = 0; i < device->in_numbuffers; i++) {
		if (device->in_buff[i]->offset == offset) {
			flag = 1;
			break;
		}
	}

	/* page offset passed in mmap should one from output buffers */
	if (flag == 0) {
		for (i = 0; i < device->out_numbuffers; i++) {
			if (device->out_buff[i]->offset == offset) {
				flag = 1;
				break;
			}
		}
	}

	/* if it is not set offset is not available in input/output buffers */
	if (flag == 0)
		return -EAGAIN;

	/* map buffers address space from kernel space to user space */
	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		return -EAGAIN;
	}
	return 0;
}

int previewer_ioctl(struct inode *inode, struct file *file,
		    unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct prev_params params;
	struct prev_convert conv;
	/* get the address of global object of prev_device structure */
	struct prev_device *device = &prevdevice;

	dev_dbg(prev_dev,"Entering previewer_ioctl()\n");

	/* Before decoding check for correctness of cmd */
	if (_IOC_TYPE(cmd) != PREV_IOC_BASE) {
		dev_err(prev_dev, "Bad command Value \n");
		return -1;
	}
	if (_IOC_NR(cmd) > PREV_IOC_MAXNR) {
		dev_err(prev_dev, "Bad command Value\n");
		return -1;
	}

	/* Verify accesses       */
	if (_IOC_DIR(cmd) & _IOC_READ)
		ret = !access_ok(VERIFY_WRITE, (void *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		ret = !access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd));
	if (ret) {
		dev_err(prev_dev, "access denied\n");
		return -1;	/*error in access */
	}

	/* switch according value of cmd */
	switch (cmd) {
		/* if case is to request buffers */
	case PREV_REQBUF:
		/* call request buffer to allocate buffers */
		down_interruptible(&(device->sem));
		ret = request_buffer(device, (struct prev_reqbufs *)arg);
		up(&(device->sem));
		break;

		/* if case is to query for buffer address */
	case PREV_QUERYBUF:
		/* call query buffer which will return buffer address */
		down_interruptible(&(device->sem));
		ret = query_buffer(device, (struct prev_buffer *)arg);
		up(&(device->sem));
		break;

		/* if case is to set configuration parameters */
	case PREV_SET_PARAM:
		down_interruptible(&(device->sem));
		/* copy the parameters to the configuration */
		if (copy_from_user
		    (&params, (struct prev_params *)arg,
		     sizeof(struct prev_params))) {
			/* if it fails return error */
			up(&(device->sem));
			return -EFAULT;
		}
		/* check for errors */
		ret = validate_params(&params);
		if (ret < 0) {
			printk("Error validating parameters!\n");
			up(&(device->sem));
			return ret;
		}
		/* copy the values to device params */
		if (device->params)
			memcpy(device->params, &params,
			       sizeof(struct prev_params));
		else {
			up(&(device->sem));
			return -EINVAL;
		}

		ret = prev_hw_setup(device->params);
		up(&(device->sem));
		break;

		/* if case is to get configuration parameters */
	case PREV_GET_PARAM:
		/* copy the parameters from the configuration */
		if (copy_to_user
		    ((struct prev_params *)arg, (device->params),
		     sizeof(struct prev_params)))
			/* if copying fails return error */
			ret = -EFAULT;
		break;

		/* if the case is to get status */
	case PREV_GET_STATUS:
		/* call getstatus function to get the status in arg */
		ret = get_status((struct prev_status *)arg);
		break;

		/* if the case is to do previewing */
	case PREV_PREVIEW:
		/* call preview function to do preview */
		if (copy_from_user(&conv, (struct prev_convert *)arg,
				   sizeof(struct prev_convert)))
			return -EFAULT;

		down_interruptible(&(device->sem));
		ret = preview(device, &conv);
		up(&(device->sem));
		break;
	case PREV_GET_CROPSIZE:
		prev_calculate_crop(device->params,
				    (struct prev_cropsize *)arg);
		break;

	default:
		dev_err(prev_dev, "previewer_ioctl: Invalid Command Value\n");
		ret = -EINVAL;
	}
	return ret;
}

static void previewer_platform_release(struct device *device)
{
	/* This is called when the reference count goes to zero */
}

static struct file_operations prev_fops = {
	.owner = THIS_MODULE,
	.open = previewer_open,
	.release = previewer_release,
	.mmap = previewer_mmap,
	.ioctl = previewer_ioctl,
};

static struct platform_device omap_previewer_device = {
	.name = OMAP_PREV_NAME,
	.id = -1,
	.dev = {
		/* we may add later */
		.release = previewer_platform_release,
	}
};

static int __init previewer_probe(struct platform_device *pdev)
{
	return 0;
}

static int previewer_remove(struct platform_device *pdev)
{
	platform_device_unregister(&omap_previewer_device); 
	platform_driver_unregister(&omap_previewer_driver);
	cdev_del(&cdev);
	unregister_chrdev_region(devt, 1);
	return 0;
}

static struct platform_driver omap_previewer_driver = {
	.probe = previewer_probe,
	.remove = previewer_remove,
//	.shutdown = previewer_shutdown,
//	.suspend = previewer_suspend,
//	.resume = previewer_resume,
	.driver = {
		.owner	= THIS_MODULE,
		.name = OMAP_PREV_NAME,
	},
};


static int __init omap_previewer_init(void)
{
	int ret;
	int major;
	/* Register the driver in the kernel */
	/* dynamically get the major number for the driver using 
	   alloc_chrdev_region function */
	major = register_chrdev(0, OMAP_PREV_NAME, &prev_fops);

	if (major < 0) {
		printk(OMAP_PREV_NAME ": initialization "
                	"failed. could not register character device\n");
		return -ENODEV;
	}

	ret = platform_driver_register(&omap_previewer_driver);
	if (ret) {
		printk(OMAP_PREV_NAME
			": failed to register platform driver!\n");
		goto fail2;
	}
	/* Register the drive as a platform device */
	ret = platform_device_register(&omap_previewer_device); 
	if (ret) {
		printk(OMAP_PREV_NAME
			": failed to register platform device!\n");
		goto fail3;
	}

	prev_class = class_create(THIS_MODULE, OMAP_PREV_NAME);
	if (!prev_class)
		goto fail4;

	prev_dev = device_create(prev_class, prev_dev, (MKDEV(major, 0)),
			       OMAP_PREV_NAME);
	printk(OMAP_PREV_NAME ": Registered Previewer Wrapper\n");
	prevdevice.opened = 0;
	return 0;

fail4:
	platform_device_unregister(&omap_previewer_device);
fail3:
	platform_driver_unregister(&omap_previewer_driver);
fail2:
	unregister_chrdev(major, OMAP_PREV_NAME);

	return ret;
}

static void __exit omap_previewer_exit(void)
{
	previewer_remove(&omap_previewer_device);
}

module_init(omap_previewer_init);
module_exit(omap_previewer_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("OMAP ISP Previewer");
MODULE_LICENSE("GPL");
