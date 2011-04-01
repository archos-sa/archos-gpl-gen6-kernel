/*
 * drivers/media/video/omap/omap34xxdvr.c
 *
 * Video-for-Linux (Version 2) video capture driver for OMAP34xx ISP.
 *
 * Copyright (C) 2008 Archos S.A.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Leverage omap34xx camera driver
 * Copyright (C) 2004 MontaVista Software, Inc.
 * Copyright (C) 2006 Texas Instruments.
 * Copyright (C) 2008 Archos S.A.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/videodev.h>
#include <media/video-buf.h>
#include <media/v4l2-dev.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <asm/arch/display.h>
#include <asm/arch/clock.h>
#include <linux/platform_device.h>

#include <asm/cacheflush.h>
#include <asm/io.h>
#include <asm/byteorder.h>
#include <asm/scatterlist.h>
#include <asm/irq.h>
#include <asm/semaphore.h>

#ifdef CONFIG_PM
#include <linux/notifier.h>
#include <linux/pm.h>
#endif

#include <asm/arch/resource.h>

#include "isp/isp.h"
#include "isp/ispreg.h"
#include "isp/ispccdc.h"
#include "isp/isppreview.h"
#include "isp/ispresizer.h"
#include "isp/ispmmu.h"
#include "omap24xxlib.h"
#include "omap24xxcam_user.h"

#include "video_decoder_if.h"
#include "omap34xxdvr.h"


#undef OMAP_DVR_DEBUG

#ifdef OMAP_DVR_DEBUG
#define DPRINTK_DVR(format,...)\
	printk("DVR: " format, ## __VA_ARGS__)
#else
#define DPRINTK_DVR(format,...)
#endif

#define DEBUG_RECORD

#define DBG if(1)

/* OMAP3430 has 12 rotation Contexts(settings).
 * Camera driver uses VRFB Context 1 if video 1 pipeline is chosen for Preview
 * or Context 2 if video 2 pipeline is chosen.
 * sms_rot_phy is recalculated each time before preview starts.
 */
static unsigned long sms_rot_phy[4];
#define VRFB_BASE		0x70000000
#define VRFB_CONTEXT_SIZE	0x04000000
#define VRFB_ANGLE_SIZE	0x01000000

#define QVGA_SIZE 	320*240*2	/* memory needed for QVGA image */
#define VGA_SIZE 	QVGA_SIZE*4	/* memory needed for VGA image */
#define SXGA_SIZE 	1280*960*2	/* memory needed for SXGA image */
#define D1_SIZE         768*572*2	/* memory needed for D1 image */

#define VGA_WIDTH	640
#define VGA_HEIGHT	480
#define QXGA_WIDTH	2048
#define QXGA_HEIGHT	1536
#define QCIF_WIDTH	176
#define QCIF_HEIGHT	144
#define MAX_RESIZER_INPUTSIZE	1280
#define DVR_NAME "omap34xxdvr"

int sensor_init = 0;
/* Need these for streaming case temporary buffer*/
static unsigned long isp_temp_virt_addr;
static unsigned long isp_temp_phy_addr;
static unsigned long isp_temp_ispmmu_addr;

/* Internal states to maintain buffer synchronization on startup
ISP_BUF_INIT     - ISP is running and new buffers have been queued but not written to. 
ISP_FREE_RUNNING - ISP is running but no buffers have been queued
ISP_BUF_TRAN     - CCDC bit is written to but not latched on till the next VSYNC. Data
		   will be written to the buffer only when state transitions to ISP_BUF_INIT
*/

#define ISP_BUF_INIT     0
#define ISP_FREE_RUNNING 1
#define ISP_BUF_TRAN     2

spinlock_t isp_temp_buf_lock;

static int isp_temp_state = ISP_BUF_INIT;

/* this is the sensor ops implemented by the associated sesnor driver */
extern struct camera_sensor camera_sensor_if;

/* global variables */
static struct omap34xxdvr_device *this_dvr;

#ifdef CONFIG_OMAP3430_ES2	
struct constraint_handle *co_opp_dvr_vdd1;
struct constraint_handle *co_opp_dvr_vdd2;
struct constraint_handle *co_opp_dvr_latency;
#endif

/* module parameters */
static int video_nr = 0;	/* video device minor (-1 ==> auto assign) */

/* Size of video overlay framebuffer.  This determines the maximum image size
 * that can be previewed.  Default is 600KB, enough for VGA. */
/* TODO: very big now enough for ISP Preview output. Will make it VGA */
static int overlay_mem = 0;//2048*1536*2/4;

/* Crop capabilities */
static struct v4l2_rect ispcroprect_a;

#ifdef CONFIG_OMAP3430_ES2
static struct constraint_id cnstr_id_vdd1 = {
	.type = RES_OPP_CO,
	.data = (void *)"vdd1_opp",
};

static struct constraint_id cnstr_id_vdd2 = {
	.type = RES_OPP_CO,
	.data = (void *)"vdd2_opp",
};

static struct constraint_id cnstr_id_latency = {
	.type = RES_LATENCY_CO,
	.data = (void *)"latency",
};
#endif

/* -------------------------------------------------------------------------- */
#ifdef CONFIG_PM
#define omap34xxdvr_suspend_lockout(s,f) \
	if ((s)->suspended) {\
		int err;\
		if ((f)->f_flags & O_NONBLOCK)\
			return -EBUSY;\
		err = wait_event_interruptible((s)->suspend_wq,\
					(s)->suspended == 0);\
		if (err < 0)\
			return err;\
	}
#else
#define omap34xxdvr_suspend_lockout(s,f) s=s
#endif


/* -------------------------------------------------------------------------- */

static void omap34xxdvr_sg_dma_timeout( unsigned long data );

static void
omap34xxdvr_set_isp_buf(struct omap34xxdvr_device *dvr, struct sgdma_state *sgdma)
{
	ispccdc_set_outaddr(sgdma->isp_addr);		
}

static void
omap34xxdvr_set_isp_callback(struct omap34xxdvr_device *dvr, struct sgdma_state *sgdma)
{
	omap_writel(omap_readl(ISP_IRQ0STATUS)| ISP_INT_CLR, ISP_IRQ0STATUS);	
	isp_set_callback(CBK_CCDC_VD0, sgdma->callback, dvr, sgdma);

	init_timer(&sgdma->wdt);
	sgdma->wdt.function = omap34xxdvr_sg_dma_timeout;
	sgdma->wdt.data = (unsigned long)sgdma->arg;
	sgdma->wdt.expires = jiffies + msecs_to_jiffies(200);
	add_timer(&sgdma->wdt);
}

static void
omap34xxdvr_unset_callback(struct omap34xxdvr_device *dvr)
{
	isp_unset_callback(CBK_CCDC_VD0);
	omap_writel(omap_readl(ISP_IRQ0STATUS)| ISP_INT_CLR, ISP_IRQ0STATUS);
}
static int
omap34xxdvr_start_isp(struct omap34xxdvr_device *dvr)
{
	/* start the needed isp components assuming these components
	 * are configured correctly.
	 */
	return 0;
}

static inline int
omap34xxdvr_stop_isp(struct omap34xxdvr_device *dvr)
{
	typeof(jiffies) timeout;
	
	omap34xxdvr_unset_callback(dvr);

	ispccdc_enable(0);
	
	timeout = jiffies + msecs_to_jiffies(100);
	while (ispccdc_busy() && time_before(jiffies, timeout)) {
		mdelay(10);
	}

	if (jiffies >= timeout)
	{
printk("%s: ispccdc_busy timed out\n", __FUNCTION__);
		return -1;
	} 
	
	return 0;
}

/* -------------------------------------------------------------------------- */

static void
omap34xxdvr_sg_dma_process(struct omap34xxdvr_device *dvr, int irq);

static void omap34xxdvr_sg_dma_timeout( unsigned long data )
{
	unsigned long flags, vbq_lock_flags;
	struct omap34xxdvr_device *dvr = this_dvr;
	struct videobuf_buffer *vb = (struct videobuf_buffer *)data;
	
	printk("%s: %d timed out\n", __FUNCTION__, vb->i);

	// we are done, return the buffer
	spin_lock_irqsave(&dvr->vbq_lock, vbq_lock_flags);
	
	spin_lock_irqsave(&dvr->sg_lock, flags);
	dvr->free_sgdma++;
	if(dvr->free_sgdma > NUM_SG_DMA)
		dvr->free_sgdma = NUM_SG_DMA;
	spin_unlock_irqrestore(&dvr->sg_lock, flags);
	
	do_gettimeofday(&vb->ts);
	vb->field_count = dvr->field_count;
	dvr->field_count += 2;

	vb->state = STATE_DONE;

	if (dvr->streaming) {
		omap34xxdvr_sg_dma_process(dvr, 1);	
	}

	wake_up(&vb->done);

	if (dvr->streaming == NULL) {
		omap34xxdvr_stop_isp(dvr);
	}
		
	spin_unlock_irqrestore(&dvr->vbq_lock, vbq_lock_flags);
}


/*
 * Process the scatter-gather DMA queue by starting queued transfers.
 * This can be called from either a process context or an IRQ context.
 * 	case 1: from a process, ISP not started.
 * 	case 2: from a process, ISP started.
 * 	case 3: from IRQ, ISP started.
 * We make sure updating buffer pointer happens only once on the
 * shadow register,
 */
static void
omap34xxdvr_sg_dma_process(struct omap34xxdvr_device *dvr, int irq)
{
	struct sgdma_state *sgdma;
	unsigned long irqflags;
	
	DPRINTK_DVR("omap34xxdvr_sg_dma_process dma_notify=%i irq=%i\n",
		dvr->dma_notify, irq);

	spin_lock_irqsave(&dvr->sg_lock, irqflags);
	/* any pending sgdma ?
	 * we can at most start or queue one sgdma */
	if ((NUM_SG_DMA - dvr->free_sgdma) > 0) {
		/* get the next sgdma */
		sgdma = dvr->sgdma + (dvr->next_sgdma + dvr->free_sgdma) % NUM_SG_DMA;
	  	if (!irq) {
			if (dvr->dma_notify) {
				/* case 1: queue & start. */
				omap34xxdvr_set_isp_callback(dvr,sgdma);
				omap34xxdvr_set_isp_buf(dvr, sgdma);
				ispccdc_enable(1);
				omap34xxdvr_start_isp(dvr);
				dvr->dma_notify = 0;
				spin_lock(&isp_temp_buf_lock);		  
				isp_temp_state = ISP_BUF_TRAN;
				spin_unlock(&isp_temp_buf_lock);
			} else {
				/* case 3:only need to queue (update buf ptr). */
				spin_lock(&isp_temp_buf_lock);
				if(isp_temp_state == ISP_FREE_RUNNING)
				{
					omap34xxdvr_set_isp_callback(dvr,sgdma);			 
					omap34xxdvr_set_isp_buf(dvr, sgdma);
					/* Non startup case */	
					ispccdc_enable(1);
					isp_temp_state = ISP_BUF_TRAN;		  
				}
				spin_unlock(&isp_temp_buf_lock);
			}
	  	} else {
			/* case 3:only need to queue (update buf ptr). */
			spin_lock(&isp_temp_buf_lock);
			omap34xxdvr_set_isp_callback(dvr,sgdma);			 
			omap34xxdvr_set_isp_buf(dvr, sgdma);
			/* Non startup case */		
			ispccdc_enable(1);
			isp_temp_state = ISP_BUF_INIT;		
			spin_unlock(&isp_temp_buf_lock);		
			/* TODO: clear irq. old interrupt can come first. OK for preview.*/
			
			if (dvr->dma_notify) {
				omap34xxdvr_start_isp(dvr);
				dvr->dma_notify = 0;
			}
		}
	} else {
		spin_lock(&isp_temp_buf_lock);
		/* Disable VD0 and CCDC here before next VSYNC */
		omap_writel((omap_readl(ISP_IRQ0ENABLE))&
				(~IRQ0ENABLE_CCDC_VD0_IRQ),ISP_IRQ0ENABLE);
		ispccdc_enable(0);
		isp_temp_state = ISP_FREE_RUNNING;
		spin_unlock(&isp_temp_buf_lock);
	}

	spin_unlock_irqrestore(&dvr->sg_lock, irqflags);
}

/* Queue a scatter-gather DMA transfer from the camera to memory.
 * Returns zero if the transfer was successfully queued, or
 * non-zero if all of the scatter-gather slots are already in use.
 */
static int
omap34xxdvr_sg_dma_queue(struct omap34xxdvr_device *dvr, dma_addr_t isp_addr,
			 isp_callback_t callback, void *arg, int irq)
{
	unsigned long irqflags;
	struct sgdma_state *sgdma;

	spin_lock_irqsave(&dvr->sg_lock, irqflags);

	if (!dvr->free_sgdma) {
		spin_unlock_irqrestore(&dvr->sg_lock, irqflags);
		return -EBUSY;
	}

	sgdma = dvr->sgdma + dvr->next_sgdma;

	sgdma->isp_addr = isp_addr;
	sgdma->status = 0;
	sgdma->callback = callback;
	sgdma->arg = arg;

	dvr->next_sgdma = (dvr->next_sgdma + 1) % NUM_SG_DMA;
	dvr->free_sgdma--;

	spin_unlock_irqrestore(&dvr->sg_lock, irqflags);

	omap34xxdvr_sg_dma_process(dvr, irq);

	return 0;
}

static void
omap34xxdvr_sgdma_init(struct omap34xxdvr_device *dvr)
{
	int sg;

	dvr->free_sgdma = NUM_SG_DMA;
	dvr->next_sgdma = 0;
	for (sg = 0; sg < NUM_SG_DMA; sg++) {
		dvr->sgdma[sg].status = 0;
		dvr->sgdma[sg].callback = NULL;
		dvr->sgdma[sg].arg = NULL;
	}
}

/* -------------------------------------------------------------------------- */

/* Callback routine for overlay DMA completion.  We just start another DMA
 * transfer unless overlay has been turned off.
 */
static void
omap34xxdvr_overlay_callback(unsigned long status, void *arg1, void *arg2)
{
	struct omap34xxdvr_device *dvr = (struct omap34xxdvr_device *) arg1;
	unsigned long irqflags;
	int err;

	spin_lock(&dvr->sg_lock);
	dvr->free_sgdma++;
	if(dvr->free_sgdma > NUM_SG_DMA)
		dvr->free_sgdma = NUM_SG_DMA;
	spin_unlock(&dvr->sg_lock);
	
	spin_lock_irqsave(&dvr->overlay_lock, irqflags);

	switch (status) {
	case CCDC_VD0:
		/* Program shadow registers for CCDC */
		ispccdc_config_shadow_registers();
		spin_unlock_irqrestore(&dvr->overlay_lock, irqflags);
		return;

	default:
		break;
	}
	
	if (dvr->overlay_cnt > 0)
		--dvr->overlay_cnt;

	if (!dvr->previewing) {
		omap34xxdvr_stop_isp(dvr);
		spin_unlock_irqrestore(&dvr->overlay_lock, irqflags);
		return;
	}

	
	while (dvr->overlay_cnt < 2) {
		err = omap34xxdvr_sg_dma_queue(dvr, dvr->isp_addr_overlay,
			omap34xxdvr_overlay_callback, NULL, 1);
		if (err)
			break;			
		++dvr->overlay_cnt;
	}

	spin_unlock_irqrestore(&dvr->overlay_lock, irqflags);
}

/* Begin DMA'ing into the camera overlay framebuffer.  We queue up two DMA
 * transfers so that all frames will be captured.
 */
static void
omap34xxdvr_start_overlay_dma(struct omap34xxdvr_device *dvr)
{
	int err;
	unsigned long irqflags;

	if (!dvr->previewing)
		return;

	if (dvr->pix.sizeimage > dvr->overlay_size)
		return;

	spin_lock_irqsave(&dvr->overlay_lock, irqflags);
	while (dvr->overlay_cnt < 2) {
		err = omap34xxdvr_sg_dma_queue(dvr, dvr->isp_addr_overlay,
			omap34xxdvr_overlay_callback, NULL, 0);
		if (err)
			break;
		++dvr->overlay_cnt;
	}

	spin_unlock_irqrestore(&dvr->overlay_lock, irqflags);
}

/* -------------------------------------------------------------------------- */


/* This routine is called from interrupt context when a scatter-gather DMA
 * transfer of a videobuf_buffer completes.
 */
static void
omap34xxdvr_vbq_complete(unsigned long status, void *arg1, void *arg2)
{
	struct omap34xxdvr_device *dvr = (struct omap34xxdvr_device *) arg1;
	struct sgdma_state *sgdma = (struct sgdma_state *)arg2;
	struct videobuf_buffer *vb = (struct videobuf_buffer *) sgdma->arg;

	int field = ispccdc_get_field() ? V4L2_FIELD_TOP : V4L2_FIELD_BOTTOM;

	spin_lock(&dvr->vbq_lock);

	del_timer_sync(&sgdma->wdt);

	switch (status) {
	case CCDC_VD0:
		/* Program shadow registers for CCDC */
		ispccdc_config_shadow_registers();
		spin_unlock(&dvr->vbq_lock);
	
		spin_lock(&isp_temp_buf_lock);
		/* Either in ISP_BUF_TRAN or ISP_FREE_RUNNING then return */
#if !defined DEBUG_RECORD
 		if(isp_temp_state != ISP_BUF_INIT) {
 			spin_unlock(&isp_temp_buf_lock);
 			return;
 		}
#else
		/* For debub purpose only */
		/* re-inint buf state at the end of dma */
		isp_temp_state = ISP_BUF_INIT;
#endif
		spin_unlock(&isp_temp_buf_lock);
		break;

	default:
		spin_unlock(&dvr->vbq_lock);
		break;
	}

	spin_lock(&dvr->vbq_lock);

	if( field == V4L2_FIELD_TOP ) {
		// we wait for the bottom field, but we have to reenable the capture stuff
		if ((status & MMU_ERR) == MMU_ERR){
			vb->state = STATE_ERROR;
			printk("\tRECD ERROR!!!!!\n");
		} else {
			if (dvr->streaming) {
				omap34xxdvr_sg_dma_process(dvr, 1);	
			}
		}

	} else {
		// we are done, return the buffer
		spin_lock(&dvr->sg_lock);
		dvr->free_sgdma++;
        	if(dvr->free_sgdma > NUM_SG_DMA)
			dvr->free_sgdma = NUM_SG_DMA;
		spin_unlock(&dvr->sg_lock);

		do_gettimeofday(&vb->ts);
		vb->field_count = dvr->field_count;
		dvr->field_count += 2;
		if ((status & MMU_ERR) == MMU_ERR){
			vb->state = STATE_ERROR;
			printk("\tRECD ERROR!!!!!\n");
			//omap34xxdvr_error_handler(dvr);
		} else {
			vb->state = STATE_DONE;
			if (dvr->streaming) {
				omap34xxdvr_sg_dma_process(dvr, 1);	
			}
		}
		wake_up(&vb->done);
	}
	
	if (dvr->streaming == NULL) {
		omap34xxdvr_stop_isp(dvr);
	}
	spin_unlock(&dvr->vbq_lock);
}

static void
omap34xxdvr_vbq_release(struct videobuf_queue *q, struct videobuf_buffer *vb)
{
	struct omap34xxdvr_fh *fh = q->priv_data;
	struct omap34xxdvr_device *dvr = fh->dvr;

	omap34xxdvr_stop_isp(dvr);
	/* we already drain the queue so videobuf_waiton is no longer needed */
	videobuf_dma_unmap(q, &vb->dma);
	videobuf_dma_free(&vb->dma);

	/* video-buf calls us multiple times. need to make sure that we only
	 * unmap once */
	if (dvr->isp_addr_capture[vb->i]) {
		ispmmu_unmap(dvr->isp_addr_capture[vb->i]);
		dvr->isp_addr_capture[vb->i] = 0;
	}

	vb->state = STATE_NEEDS_INIT;

}

/* Limit the number of available kernel image capture buffers based on the
 * number requested, the currently selected image size, and the maximum
 * amount of memory permitted for kernel capture buffers.
 */
static int
omap34xxdvr_vbq_setup(struct videobuf_queue *q, unsigned int *cnt, unsigned int *size)
{
	struct omap34xxdvr_fh *fh = q->priv_data;
	struct omap34xxdvr_device *dvr = fh->dvr;

	if (*cnt <= 0)
		*cnt = VIDEO_MAX_FRAME;	/* supply a default number of buffers */

	if (*cnt > VIDEO_MAX_FRAME)
		*cnt = VIDEO_MAX_FRAME;

	spin_lock(&dvr->img_lock);
	*size = dvr->pix.sizeimage;
	spin_unlock(&dvr->img_lock);

	dvr->count = *cnt;
	
	return 0;
}

static int 
omap34xxdvr_dma_map(struct videobuf_queue* q, struct videobuf_buffer *vb)
{
	if (vb->memory == V4L2_MEMORY_USERPTR && vb->baddr)
	{
		struct videobuf_dmabuf *dma = &vb->dma;		
		struct vm_area_struct *vma;
		
		dma->bus_addr = 0;
		dma->vmalloc = 0;
		dma->pages = 0;

		/* For kernel direct-mapped memory, take the easy way */
		if (vb->baddr >= PAGE_OFFSET) {
			dma->bus_addr = virt_to_phys((void *)vb->baddr);
		} else if ((vma = find_vma(current->mm, vb->baddr)) && (vma->vm_flags & VM_IO) && (vma->vm_pgoff)) {
			/* this will catch, kernel-allocated,
				mmaped-to-usermode addresses */
			dma->bus_addr = (vma->vm_pgoff << PAGE_SHIFT) + (vb->baddr - vma->vm_start);
		} 
		
		if (dma->bus_addr)
		{				
			unsigned long first = (dma->bus_addr & PAGE_MASK)              >> PAGE_SHIFT;
			unsigned long last  = ((dma->bus_addr+vb->size-1) & PAGE_MASK) >> PAGE_SHIFT;

			dma->direction = DMA_FROM_DEVICE;
			dma->offset    = dma->bus_addr & ~PAGE_MASK;
			dma->nr_pages  = last-first+1;
			
//printk("call videobuf_dma_map, baddr: %08x, bus_addr: %08x, nr_pages: %d, offset: %d\n", vb->baddr, dma->bus_addr, dma->nr_pages, dma->offset);
			return videobuf_dma_map(q, dma);
		}
	}

	return videobuf_iolock(q, vb, NULL);
}

static int
omap34xxdvr_vbq_prepare(struct videobuf_queue *q, struct videobuf_buffer *vb,
	enum v4l2_field field)
{
	struct omap34xxdvr_fh *fh = q->priv_data;
	struct omap34xxdvr_device *dvr = fh->dvr;
	unsigned int size, isp_addr;


	int err = 0;

	spin_lock(&dvr->img_lock);
	size = dvr->pix.sizeimage;

	if (vb->baddr) {
		/* This is a userspace buffer. */
#if 0
		if (size > vb->bsize) {
printk("omap34xxdvr_vbq_prepare: size > vb->bsize  %d > %d\r\n", size, vb->bsize );
			/* The buffer isn't big enough. */
			err = -EINVAL;
		} else 
#endif
		{
			vb->size = size;
			/* video-buf uses bsize (buffer size) instead of
			 * size (image size) to generate sg slist. Work
			 * around this bug by making bsize the same as
			 * size.
			 */
			vb->bsize = size;
		}
	} else {
		if (vb->state != STATE_NEEDS_INIT) {
			/* We have a kernel bounce buffer that has already been
			 * allocated.
			 */
			if (size > vb->size) {
				/* The image size has been changed to a larger
				 * size since this buffer was allocated, so we
				 * need to free and reallocate it.
				 */
				spin_unlock(&dvr->img_lock);
				omap34xxdvr_vbq_release(q, vb);
				spin_lock(&dvr->img_lock);
				vb->size = size;
			}
		} else {
			/* We need to allocate a new kernel bounce buffer. */
			vb->size = size;
		}
	}

	vb->width = dvr->pix.width;
	vb->height = dvr->pix.height;
	vb->field = field;

	spin_unlock(&dvr->img_lock);

	if (err)
		return err;

	if (vb->state == STATE_NEEDS_INIT) {
		err = omap34xxdvr_dma_map(q, vb);
		if (!err) {
			if (vb->dma.bus_addr)
				isp_addr = ispmmu_map(vb->dma.bus_addr, vb->dma.nr_pages << PAGE_SHIFT);
			else
				isp_addr = ispmmu_map_sg(vb->dma.sglist, vb->dma.sglen);
			if (!isp_addr)
				err = -EIO;
			else
				dvr->isp_addr_capture[vb->i]= isp_addr;
		}
	}

	if (!err) {
		vb->state = STATE_PREPARED;
		if (vb->baddr) {
		    flush_cache_user_range(NULL, vb->baddr, (vb->baddr + vb->bsize));
		} else {
			/* sync a kernel buffer */
			dmac_flush_range(vb->dma.vmalloc, (vb->dma.vmalloc + (vb->dma.nr_pages << PAGE_SHIFT)));
			outer_flush_range(__pa(vb->dma.vmalloc), __pa(vb->dma.vmalloc + (vb->dma.nr_pages << PAGE_SHIFT)));
		}
	} else
		omap34xxdvr_vbq_release(q, vb);

	return err;
}

static void
omap34xxdvr_vbq_queue(struct videobuf_queue *q, struct videobuf_buffer *vb)
{
	struct omap34xxdvr_fh *fh = q->priv_data;
	struct omap34xxdvr_device *dvr = fh->dvr;
	enum videobuf_state state = vb->state;
	int err;

//printk("%s: %d %d, %d\n", __FUNCTION__, vb->i, ( vb->i + 1 )%dvr->count, dvr->count);

	vb->state = STATE_QUEUED;
	//err = omap34xxdvr_sg_dma_queue(dvr, dvr->isp_addr_capture[vb->i], omap34xxdvr_vbq_complete, vb, 0);
	err = omap34xxdvr_sg_dma_queue(dvr, dvr->isp_addr_capture[( vb->i + 1 )%dvr->count], omap34xxdvr_vbq_complete, vb, 0);
	if (err) {
		/* Oops.  We're not supposed to get any errors here.  The only
		 * way we could get an error is if we ran out of scatter-gather
		 * DMA slots, but we are supposed to have at least as many
		 * scatter-gather DMA slots as video buffers so that can't
		 * happen.
		 */
		printk(KERN_DEBUG DVR_NAME
		       ": Failed to queue a video buffer for DMA!\n");
		vb->state = state;
	}
}

/* -------------------------------------------------------------------------- */
/*
 * This function is used to set up a default preview window for the current
 * image. It also sets up a proper preview crop.
 * It is called when a new image size is set or a new rotation is set.
 */
void static
omap34xxdvr_preview(struct omap34xxdvr_device *dvr)
{
	/* Get the panel parameters.
	 * They can change from LCD to TV
	 * or TV to LCD
	 */
	omap2_disp_get_dss();
	if (dvr->overlay_rotation == PREVIEW_ROTATION_90 ||
	    dvr->overlay_rotation == PREVIEW_ROTATION_270)
		omap2_disp_get_panel_size(
		 omap2_disp_get_output_dev(dvr->vid_preview),
			  	 &(dvr->fbuf.fmt.height),
			  	 &(dvr->fbuf.fmt.width));
	else
		omap2_disp_get_panel_size(
		 omap2_disp_get_output_dev(dvr->vid_preview),
			  	 &(dvr->fbuf.fmt.width),
			  	 &(dvr->fbuf.fmt.height));
		
	/* intialize the preview parameters */
	omap24xxvout_new_format(&dvr->pix, &dvr->fbuf,
				&dvr->preview_crop, &dvr->win);
	omap2_disp_put_dss();
}

/* list of image formats supported via OMAP ISP */
const static struct v4l2_fmtdesc isp_formats[] = {
	{
		.description	= "UYVY, packed",
		.pixelformat	= V4L2_PIX_FMT_UYVY,
	},
	{
		.description	= "YUYV (YUV 4:2:2), packed",
		.pixelformat	= V4L2_PIX_FMT_YUYV,
	},
};
#define NUM_CAPTURE_FORMATS (sizeof(isp_formats)/sizeof(isp_formats[0]))
#define NUM_OVERLAY_FORMATS 2

/* 
 * Config ISP pipeline for ISP sensor. Only CCDC is needed
 * context: 1 video
 */
static void
omap34xxdvr_config_pipeline(struct omap34xxdvr_device *dvr, int context)
{
	u32 w, h;
	struct v4l2_pix_format *pix = &dvr->pix;

	w = pix->width;
	if (pix->field == V4L2_FIELD_INTERLACED)
		h = pix->height / 2;
	else
		h = pix->height;
	
	dvr->ccdc_width  = w*2; // with*2 like gen5
	dvr->ccdc_height = h;

	ispccdc_config_datapath(CCDC_YUV_BT, CCDC_OTHERS_MEM);
	ispccdc_try_size(w*2, h, &dvr->ccdc_width, &dvr->ccdc_height);
	ispccdc_config_size(w*2, h, dvr->ccdc_width, dvr->ccdc_height);
	ispccdc_config_outlineoffset(dvr->ccdc_width, EVENEVEN, 1);
	ispccdc_config_outlineoffset(dvr->ccdc_width, ODDEVEN, 1);
	ispccdc_config_outlineoffset(dvr->ccdc_width, EVENODD, 1);
	ispccdc_config_outlineoffset(dvr->ccdc_width, ODDODD, 1);
	
}


static void
omap34xxdvrisp_configure_interface(struct omap34xxdvr_device *dvr, int context)
{
	struct isp_interface_config config;
	
	/* FIXME: configure decoder and interface correctly */
		
	config.ccdc_par_ser = 0;
  	config.dataline_shift = 2;
	config.hsvs_syncdetect = 0;
	config.para_clk_pol = 0;
	
	/* bridge disabled for REC656 mode */
	config.par_bridge = 0;
	
	config.shutter = 0;
	config.prestrobe = 0;
	config.strobe = 0;
	config.vdint0_timing = 0;
	config.vdint1_timing = 0;
	isp_configure_interface(&config);
}


static int
omap34xxdvr_do_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		     void *arg)
{
	struct omap34xxdvr_fh *fh = file->private_data;
	struct omap34xxdvr_device *dvr = fh->dvr;
	
	int err = 0;

	switch (cmd) {
		/* for time being below IOCTL cmd is here */

	case VIDIOC_ENUMINPUT:
	{
		struct v4l2_input *input = (struct v4l2_input *) arg;
		int index = input->index;

		memset(input, 0, sizeof (*input));
		input->index = index;

		return dvr->dvr_decoder->enum_input(input, dvr->decoder);
	}

	case VIDIOC_G_INPUT:
	{
		unsigned int *input = arg;
		return dvr->dvr_decoder->get_input(input, dvr->decoder);
	}

	case VIDIOC_S_INPUT:
	{
		unsigned int *input = arg;
		return dvr->dvr_decoder->set_input(input, dvr->decoder);
	}

	case VIDIOC_QUERYCAP:
	{
		struct v4l2_capability *cap = (struct v4l2_capability *) arg;

		memset(cap, 0, sizeof (*cap));
		strlcpy(cap->driver, DVR_NAME, sizeof (cap->driver));
		strlcpy(cap->card, dvr->vfd->name, sizeof (cap->card));
		cap->bus_info[0] = '\0';
		cap->capabilities = V4L2_CAP_VIDEO_CAPTURE |
		    V4L2_CAP_VIDEO_OVERLAY | V4L2_CAP_READWRITE |
		    V4L2_CAP_STREAMING | V4L2_CAP_SLICED_VBI_CAPTURE;
		return 0;
	}

	case VIDIOC_ENUM_FMT:
	{
		struct v4l2_fmtdesc *fmt = arg;
		return dvr->dvr_decoder->enum_pixformat(fmt, dvr->decoder);
	}

	case VIDIOC_G_FMT:
	{
		struct v4l2_format *f = (struct v4l2_format *) arg;
		switch (f->type) {
		case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		{
			struct v4l2_pix_format *pix = &f->fmt.pix;
			memset(pix, 0, sizeof (*pix));
			spin_lock(&dvr->img_lock);
			*pix = dvr->pix;
			spin_unlock(&dvr->img_lock);
			return 0;
		}

		case V4L2_BUF_TYPE_VIDEO_OVERLAY:
		{
			struct v4l2_window *win = &f->fmt.win;
			memset(win, 0, sizeof (*win));
			/* The API has a bit of a problem here.
			 * We're returning a v4l2_window
			 * structure, but that structure
			 * contains pointers to variable-sized
			 * objects for clipping rectangles and
			 * clipping bitmaps.  We will just
			 * return NULLs for those pointers.
			 */
			spin_lock(&dvr->img_lock);
			win->w = dvr->win.w;
			win->field = dvr->win.field;
			win->chromakey = dvr->win.chromakey;
			spin_unlock(&dvr->img_lock);
			return 0;
		}

		default:
			return -EINVAL;
		}
	}

	case VIDIOC_TRY_FMT:
	{
		struct v4l2_format *f = (struct v4l2_format *) arg;

		switch (f->type) {
		case V4L2_BUF_TYPE_VIDEO_OVERLAY:
		{
			struct v4l2_window *win = &f->fmt.win;

			spin_lock(&dvr->img_lock);
			err =
			    omap24xxvout_try_window(&dvr->fbuf,
						    win);
			spin_unlock(&dvr->img_lock);
			return err;
		}

		case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		{
			int ret = dvr->dvr_decoder->try_format(&f->fmt.pix, 
					dvr->decoder);
		
			/* make image size page aligned */
			f->fmt.pix.sizeimage = PAGE_ALIGN(f->fmt.pix.sizeimage);
		
			return ret;
		}

		default:
			return -EINVAL;
		}
	}

	case VIDIOC_S_FMT:
	{
		struct v4l2_format *f = (struct v4l2_format *) arg;
		struct v4l2_pix_format *pixfmt;

		switch (f->type) {
		case V4L2_BUF_TYPE_VIDEO_OVERLAY:
		{
			struct v4l2_window *win = &f->fmt.win;

			spin_lock(&dvr->img_lock);
			if (dvr->previewing || dvr->streaming) {
				spin_unlock(&dvr->img_lock);
				return -EBUSY;
			}
			/*
			 * Reset Crop values since image size might have changed
			 */
			ispcroprect_a.left   = 0;
			ispcroprect_a.top    = 0;
			ispcroprect_a.width  = f->fmt.pix.width;
			ispcroprect_a.height = f->fmt.pix.height;
			
			/* Get the panel parameters.
			 * They can change from LCD to TV
			 * or TV to LCD
			 */
			omap2_disp_get_dss();
			if (dvr->overlay_rotation == PREVIEW_ROTATION_90 ||
			    dvr->overlay_rotation == PREVIEW_ROTATION_270)
				omap2_disp_get_panel_size(
					omap2_disp_get_output_dev(dvr->vid_preview),
						&(dvr->fbuf.fmt.height),
						&(dvr->fbuf.fmt.width));
			else
				omap2_disp_get_panel_size(
					omap2_disp_get_output_dev(dvr->vid_preview),
						&(dvr->fbuf.fmt.width),
						&(dvr->fbuf.fmt.height));

			err =
			    omap24xxvout_new_window(&dvr->preview_crop,
						    &dvr->win,
						    &dvr->fbuf,
						    win);
			omap2_disp_put_dss();
			spin_unlock(&dvr->img_lock);
			return err;
		}

		case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		{
#if 0 // Need format setting during stream is on in avos. I have not seen conflicts yet
			spin_lock(&dvr->img_lock);
			if (dvr->streaming || dvr->previewing) {
				spin_unlock(&dvr->img_lock);
				return -EBUSY;
			}
#endif
			dvr->dvr_decoder->try_format(&f->fmt.pix, dvr->decoder);
			pixfmt = &f->fmt.pix;

			/* make image size page aligned */
			pixfmt->sizeimage = PAGE_ALIGN(pixfmt->sizeimage);
			
			/* set the new user capture format */
			dvr->pix = f->fmt.pix;

			/* adjust the capture frame rate */
			/* FIXME: do we need nominal_timeperframe?
			dvr->xclk =
			    dvr->dvr_decoder->calc_xclk(pixfmt,
					&dvr->nominal_timeperframe, dvr->decoder);
			*/
			dvr->cparm.timeperframe = dvr->nominal_timeperframe;

			/* set a default display window and preview crop */
			omap34xxdvr_preview(dvr);
			spin_unlock(&dvr->img_lock);

			#if 0
			/* FIXME: anything to do for us? */
			/* negotiate xclk with isp */
			dvr->xclk = isp_negotiate_xclka(dvr->xclk);
			/* program the agreed new xclk frequency */
			dvr->xclk = isp_set_xclka(dvr->xclk);

			/* program the sensor */
			err = dvr->dvr_decoder->configure(pixfmt,
				dvr->xclk, &dvr->cparm.timeperframe, dvr->decoder);
			#endif

			omap34xxdvr_config_pipeline(dvr, 1);
			return err;
		}

		case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
			return dvr->dvr_decoder->ioctl(cmd, arg, dvr->decoder);
			
		default:
			return -EINVAL;
		}
	}

	case VIDIOC_G_FBUF:
	{
		struct v4l2_framebuffer *fbuf =
		    (struct v4l2_framebuffer *) arg;

		spin_lock(&dvr->img_lock);
		*fbuf = dvr->fbuf;
		spin_unlock(&dvr->img_lock);
		return 0;
	}

	case VIDIOC_S_FBUF:
	{
		struct v4l2_framebuffer *fbuf =
		    (struct v4l2_framebuffer *) arg;
		unsigned int flags = fbuf->flags;

		/* The only field the user is allowed to change is
		 * fbuf->flags.
		 */
		spin_lock(&dvr->img_lock);
		if (dvr->previewing) {
			spin_unlock(&dvr->img_lock);
			return -EBUSY;
		}
		if (flags & V4L2_FBUF_FLAG_CHROMAKEY)
			dvr->fbuf.flags |= V4L2_FBUF_FLAG_CHROMAKEY;
		else
			dvr->fbuf.flags &= ~V4L2_FBUF_FLAG_CHROMAKEY;
		spin_unlock(&dvr->img_lock);
		return 0;
	}

	case VIDIOC_CROPCAP:
	{
		struct v4l2_cropcap *cropcap =
		    (struct v4l2_cropcap *) arg;
		enum v4l2_buf_type type = cropcap->type;

		memset(cropcap, 0, sizeof (*cropcap));
		cropcap->type = type;
		switch (type) {
		case V4L2_BUF_TYPE_VIDEO_OVERLAY:
		{
			spin_lock(&dvr->img_lock);
			cropcap->bounds.width = dvr->pix.width & ~1;
			cropcap->bounds.height = dvr->pix.height & ~1;
			omap24xxvout_default_crop(&dvr->pix, &dvr->fbuf,
							&cropcap->defrect);
			spin_unlock(&dvr->img_lock);
			cropcap->pixelaspect.numerator = 1;
			cropcap->pixelaspect.denominator = 1;
			return 0;
		}

		case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		{
			if (dvr->dvr_decoder->cropcap) {
				return dvr->dvr_decoder->cropcap(cropcap,
						dvr->decoder);
			}
			
			spin_lock(&dvr->img_lock);
			cropcap->bounds.width = dvr->pix.width;
			cropcap->bounds.height = dvr->pix.height;
			spin_unlock(&dvr->img_lock);
			cropcap->defrect.width = cropcap->bounds.width;
			cropcap->defrect.height = cropcap->bounds.height;
			cropcap->pixelaspect.numerator = 1;
			cropcap->pixelaspect.denominator = 1;
			return 0;
		}

		default:
			return -EINVAL;
		}
	}

	case VIDIOC_G_CROP:
	{
		struct v4l2_crop *crop = (struct v4l2_crop *) arg;

		switch (crop->type) {
		case V4L2_BUF_TYPE_VIDEO_OVERLAY:
		{
			spin_lock(&dvr->img_lock);
			crop->c = dvr->preview_crop;
			spin_unlock(&dvr->img_lock);
			return 0;
		}

		case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		{
			if (dvr->dvr_decoder->get_crop) {
				return dvr->dvr_decoder->get_crop(crop,
					  dvr->decoder);
			}
			/* The sensor doesn't support cropping.
			 * We don't support it either.
			 */
			return -EINVAL;
		}

		default:
			return -EINVAL;
		}
	}

	case VIDIOC_S_CROP:
	{
		struct v4l2_crop *crop = (struct v4l2_crop *) arg;
		switch (crop->type) {
		case V4L2_BUF_TYPE_VIDEO_OVERLAY:
		{
			spin_lock(&dvr->img_lock);
			if (dvr->previewing) {
				spin_unlock(&dvr->img_lock);
				return -EBUSY;
			}
			err = omap24xxvout_new_crop(&dvr->pix, 
				&dvr->preview_crop, &dvr->win, 
				&dvr->fbuf, &crop->c);
			spin_unlock(&dvr->img_lock);
			return err;
		}

		case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		{
			if (dvr->dvr_decoder->set_crop) {
				return dvr->dvr_decoder->set_crop(crop, 
								dvr->decoder);
			}
			/* The sensor doesn't support cropping.
			 * We don't support it either.
			 */
			return -EINVAL;
		}

		default:
			return -EINVAL;
		}
	}

	case VIDIOC_G_PARM:
	{
		struct v4l2_streamparm *parm =
		    (struct v4l2_streamparm *) arg;
		enum v4l2_buf_type type = parm->type;

		memset(parm, 0, sizeof (*parm));
		parm->type = type;
		if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
			return -EINVAL;
		spin_lock(&dvr->img_lock);
		parm->parm.capture = dvr->cparm;
		spin_unlock(&dvr->img_lock);
		return 0;
	}

	case VIDIOC_S_PARM:
	{
		struct v4l2_streamparm *parm =
		    (struct v4l2_streamparm *) arg;
		struct v4l2_captureparm *cparm = &parm->parm.capture;

		if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
			return -EINVAL;
		spin_lock(&dvr->img_lock);
		if (dvr->streaming || dvr->previewing) {
			spin_unlock(&dvr->img_lock);
			return -EBUSY;
		}

		dvr->cparm.capturemode = cparm->capturemode;
		if (cparm->timeperframe.numerator
		    && cparm->timeperframe.denominator) 
		{
			dvr->nominal_timeperframe = cparm->timeperframe;
			
			#if 0
			/* FIXME: adjust the capture frame rate? */
			dvr->xclk =
			    dvr->dvr_decoder->calc_xclk(&dvr->pix,
				&dvr->nominal_timeperframe, dvr->decoder);
			dvr->xclk = isp_negotiate_xclka(dvr->xclk);
			
			dvr->cparm.timeperframe = dvr->nominal_timeperframe;
			#endif

			spin_unlock(&dvr->img_lock);

			/* program the sensor */
			err = dvr->dvr_decoder->configure(&dvr->pix,
				&dvr->cparm.timeperframe, dvr->decoder);
			cparm->timeperframe = dvr->cparm.timeperframe;
		} else {
			spin_unlock(&dvr->img_lock);
		}
		return 0;
	}

	case VIDIOC_G_OVERLAY_ROT:
	{
		int *rotation = arg;
		
		spin_lock(&dvr->img_lock);
		*rotation = dvr->overlay_rotation;
		spin_unlock(&dvr->img_lock);
		return 0;
	}
	
	case VIDIOC_S_OVERLAY_ROT:
	{
		int *rotation = arg;
		
		if (*rotation < PREVIEW_ROTATION_NO ||
		    *rotation > PREVIEW_ROTATION_270)
		    return -EINVAL;

		spin_lock(&dvr->img_lock);
		if (*rotation == dvr->overlay_rotation) {
			spin_unlock(&dvr->img_lock);
			return 0;
		}
		if (dvr->streaming || dvr->previewing) {
			spin_unlock(&dvr->img_lock);
			return -EBUSY;
		}

		/* set the new rotation mode */
		dvr->overlay_rotation = *rotation;

		/* we have to adjust the preview crop and window */
		/* set a default display window and preview crop */
		omap34xxdvr_preview(dvr);

		spin_unlock(&dvr->img_lock);
		return 0;
	}
	
	case VIDIOC_OVERLAY:
	{
		int *on = arg;
		int vrfb_pixelsize = 2;
		int rotation, dss_dma_start;
		int outputoffset;

		if(*on) {
			if(isp_temp_ispmmu_addr){
				ispmmu_unmap(isp_temp_ispmmu_addr);
				isp_temp_ispmmu_addr = 0;
			}
		} else {
			if(!isp_temp_ispmmu_addr)
				isp_temp_ispmmu_addr = ispmmu_map(isp_temp_phy_addr,D1_SIZE);
		}
		spin_lock(&dvr->img_lock);
		/*
		 * We do not allow previewing any format that is not
		 * supported by OMAP DSS.
		 */
		if (dvr->pix.pixelformat != V4L2_PIX_FMT_YUYV &&
		    dvr->pix.pixelformat != V4L2_PIX_FMT_UYVY &&
		    dvr->pix.pixelformat != V4L2_PIX_FMT_RGB565 &&
		    dvr->pix.pixelformat != V4L2_PIX_FMT_RGB565X){
			spin_unlock(&dvr->img_lock);
			return -EINVAL;
		}
		/*
		 * We design the driver in such a way that video preview
		 * and video capture are mutually exclusive.
		 */
		if (!(dvr->previewing || dvr->previewing) && *on) {
			if (dvr->overlay_size > 0 && dvr->pix.sizeimage <= dvr->overlay_size) {
				/* YUV images require twice as much memory as RGB
				 * per VRFB requirement. So the max preview YUV
				 * image is smaller than RGB image.
				 */
				if ((V4L2_PIX_FMT_YUYV == dvr->pix.pixelformat ||
				    V4L2_PIX_FMT_UYVY == dvr->pix.pixelformat) &&
				    dvr->overlay_rotation) {
				    if (dvr->pix.sizeimage*2 > dvr->overlay_size) {
					spin_unlock(&dvr->img_lock);
					return -EINVAL;
				    }
				    vrfb_pixelsize <<= 1;
				}
				/* V1 is the default for preview */
				dvr->vid_preview = (*on == 2) ?
				    OMAP2_VIDEO2 : OMAP2_VIDEO1;
				if (!omap2_disp_request_layer
				    (dvr->vid_preview)) {
					spin_unlock(&dvr->img_lock);
					return -EBUSY;
				}
				omap2_disp_get_dss();
				
			#ifdef CONFIG_OMAP3430_ES2
				DPRINTK_DVR("Setting constraint for VDD2\n");
				constraint_set(co_opp_dvr_vdd2,
					CO_VDD2_OPP3);
				constraint_set(co_opp_dvr_latency,
					CO_LATENCY_MPURET_COREON);
			#endif 
				
				
				if (dvr->overlay_rotation) {
					/* re-calculate sms_rot_phy[] */
					int i, base = VRFB_BASE +
						dvr->vid_preview*VRFB_CONTEXT_SIZE;
					for (i = 0; i < 4; i++)
						sms_rot_phy[i] = base +
							i*VRFB_ANGLE_SIZE;

					rotation = (dvr->overlay_rotation-1)*90;
					if (rotation == 90 || rotation == 270) {
						if (rotation == 90)
							rotation = 270;
						else
							rotation = 90;

						omap2_disp_set_vrfb(dvr->vid_preview,
						dvr->overlay_base_phys, dvr->pix.height,
						dvr->pix.width,vrfb_pixelsize);

					}
					else
						omap2_disp_set_vrfb(dvr->vid_preview,
						dvr->overlay_base_phys, dvr->pix.width,
						dvr->pix.height,vrfb_pixelsize);

					dvr->overlay_base_dma =
						sms_rot_phy[rotation/90];
					
					dss_dma_start = sms_rot_phy[0];
					
				}
				else {
					dvr->overlay_base_dma = dvr->overlay_base_phys;
					dss_dma_start = dvr->overlay_base_phys;
					rotation = -1;
				}

				omap34xxdvrisp_configure_interface(dvr, 1);
				omap34xxdvr_config_pipeline(dvr, 1);

				/* does mapping for overlay buffer */
				sg_dma_address(&dvr->overlay_sglist) = dvr->overlay_base_dma;

				/* Configure the ISP MMU to map enough 
				memory for VRFB memory space for rotation*/					

				if(dvr->overlay_rotation)
					sg_dma_len(&dvr->overlay_sglist) = 2048*2048*vrfb_pixelsize;
				else
					sg_dma_len(&dvr->overlay_sglist) = dvr->overlay_size;						

				dvr->isp_addr_overlay = ispmmu_map(
					sg_dma_address(&dvr->overlay_sglist), 
					sg_dma_len(&dvr->overlay_sglist));


				/* Set output offset to maximum number of bytes per line 
				in VRFB inorder to allow 2D addressing */

				outputoffset = 2048*vrfb_pixelsize;
				
				/* Lets configure offset here */
				
				if(dvr->overlay_rotation)
					ispccdc_config_outlineoffset(outputoffset, 0, 0);

				/* Turn on the overlay window */
				omap2_disp_config_vlayer(dvr->vid_preview,
							 &dvr->pix,
							 &dvr->preview_crop,
							 &dvr->win,
							 rotation,
							 0);
				omap2_disp_start_vlayer(dvr->vid_preview,
							&dvr->pix,
							&dvr->preview_crop,
							&dvr->win,
							dss_dma_start,
							rotation,
							0);
				dvr->previewing = fh;
				spin_unlock(&dvr->img_lock);

#if 0	//TODO: revisit
				ispccdc_config_datapath(CCDC_RAW, CCDC_OTHERS_VP);
#if 1
				isppreview_config_datapath(PRV_RAW_CCDC,PREVIEW_MEM);
#else //To be enabled
				isppreview_config_datapath(PRV_RAW_CCDC,PREVIEW_RSZ);
				ispreszier_config_datapath(RSZ_OTFLY_YUV);
#endif
#endif //0

				/* start the camera interface */
				dvr->dma_notify = 1;
				omap34xxdvr_start_overlay_dma(dvr);
			} else {
				/* Image size is bigger than overlay
				 * buffer.
				 */
				spin_unlock(&dvr->img_lock);
				return -EINVAL;
			}
		} else if (dvr->previewing && !*on) {
			/* turn overlay off */
			omap2_disp_disable_layer(dvr->vid_preview);
			omap2_disp_release_layer(dvr->vid_preview);
			omap2_disp_put_dss();
			dvr->overlay_cnt = 0;
			dvr->dma_notify = 0;
			dvr->previewing = NULL;
			omap34xxdvr_stop_isp(dvr);
		#ifdef CONFIG_OMAP3430_ES2
			DPRINTK_DVR("Removing constraint for VDD2\n");
			constraint_remove(co_opp_dvr_vdd2);
			constraint_remove(co_opp_dvr_latency);
		#endif
			spin_unlock(&dvr->img_lock);
			ispmmu_unmap(dvr->isp_addr_overlay);
		} else
			spin_unlock(&dvr->img_lock);

		return 0;
	}

	case VIDIOC_REQBUFS:
		return videobuf_reqbufs(&fh->vbq, arg);

	case VIDIOC_QUERYBUF:
		return videobuf_querybuf(&fh->vbq, arg);

	case VIDIOC_QBUF:
		return videobuf_qbuf(&fh->vbq, arg);

	case VIDIOC_DQBUF:
		return videobuf_dqbuf(&fh->vbq, arg, file->f_flags & O_NONBLOCK);

	case VIDIOC_STREAMON:
	{
		spin_lock(&dvr->img_lock);
		if (dvr->streaming || dvr->previewing) {
			spin_unlock(&dvr->img_lock);
			return -EBUSY;
		} else
			dvr->streaming = fh;
		spin_unlock(&dvr->img_lock);
	#ifdef CONFIG_OMAP3430_ES2
		
		if (dvr->pix.width >= 640 && dvr->pix.height >= 480 ) {
			DPRINTK_DVR("Setting constraint for VDD1\n");
			constraint_set(co_opp_dvr_vdd1, CO_VDD1_OPP3);
			
		}
		DPRINTK_DVR("Setting constraint for VDD2\n");
		constraint_set(co_opp_dvr_vdd2, CO_VDD2_OPP3);
		constraint_set(co_opp_dvr_latency, CO_LATENCY_MPURET_COREON);
	#endif 
		
		omap34xxdvrisp_configure_interface(dvr, 1);
		omap34xxdvr_config_pipeline(dvr, 1);
		dvr->dvr_decoder->stream_start(dvr->decoder);
		dvr->dma_notify = 1;
		return videobuf_streamon(&fh->vbq);
	}

	case VIDIOC_STREAMOFF:
	{
		struct videobuf_queue *q = &fh->vbq;
		int i, err;

printk("VIDIOC_STREAMOFF\r\n");
		/* video-buf lib has trouble to turn off streaming while
		   any buffer is still in QUEUED state. Let's wait until
		   all queued buffers are filled.
		 */
		for (i = 0; i < VIDEO_MAX_FRAME; i++) {
			if (NULL == q->bufs[i])
				continue;
			if (q->bufs[i]->state == STATE_QUEUED) {
printk("wait for buffer: %d\r\n", i);
				err = videobuf_waiton(q->bufs[i], 0, 0);
				if (err)
					return err;
			} 
		}
		omap34xxdvr_stop_isp(dvr);

		isp_temp_state = 0;
		spin_lock(&dvr->img_lock);
		if (dvr->streaming == fh)
			dvr->streaming = NULL;
		spin_unlock(&dvr->img_lock);

		for (i = 0; i < VIDEO_MAX_FRAME; i++) {
			if (NULL == q->bufs[i])
				continue;
			if (q->bufs[i]->memory == V4L2_MEMORY_USERPTR)
				omap34xxdvr_vbq_release(q, q->bufs[i]);
		}
		dvr->dma_notify = 0;
		videobuf_streamoff(q);
	#ifdef CONFIG_OMAP3430_ES2
		if (dvr->pix.width >= 640 && dvr->pix.height >= 480 ) {
			DPRINTK_DVR("Removing constraint for VDD1 \n");
			constraint_remove(co_opp_dvr_vdd1);
		}
		DPRINTK_DVR("Removing constraint for VDD2 \n");
		constraint_remove(co_opp_dvr_vdd2);
		constraint_remove(co_opp_dvr_latency);
	#endif 
		dvr->dvr_decoder->stream_stop(dvr->decoder);
		return 0;
	}

	case VIDIOC_G_CTRL:
	{
		struct v4l2_control *vc = arg;
		return dvr->dvr_decoder->get_control(vc, dvr->decoder);
	}
	
	case VIDIOC_S_CTRL:
	{
		struct v4l2_control *vc = arg;
		return dvr->dvr_decoder->set_control(vc, dvr->decoder);
	}
	case VIDIOC_QUERYCTRL:
	{
		struct v4l2_queryctrl *qc = arg;
		return dvr->dvr_decoder->query_control(qc, dvr->decoder);
	}
	
	case VIDIOC_QUERYMENU:
		return -EINVAL;

	case VIDIOC_ENUMSTD:
	case VIDIOC_G_STD:
	case VIDIOC_S_STD:
		/* FIXME: implement this ioctls? */
		return -EINVAL;

	case VIDIOC_G_AUDIO:
	case VIDIOC_S_AUDIO:
	case VIDIOC_G_AUDOUT:
	case VIDIOC_S_AUDOUT:
		/* we don't have any audio inputs or outputs */
		return -EINVAL;

	case VIDIOC_G_JPEGCOMP:
	case VIDIOC_S_JPEGCOMP:
		/* JPEG compression is not supported */
		return -EINVAL;

	case VIDIOC_G_TUNER:
	case VIDIOC_S_TUNER:
	case VIDIOC_G_MODULATOR:
	case VIDIOC_S_MODULATOR:
	case VIDIOC_G_FREQUENCY:
	case VIDIOC_S_FREQUENCY:
		/* we don't have a tuner or modulator */
		return -EINVAL;

	case VIDIOC_ENUMOUTPUT:
	case VIDIOC_G_OUTPUT:
	case VIDIOC_S_OUTPUT:
		/* FIXME: implement these ioctls if appropriate */
		return -EINVAL;

	default:
		/* unrecognized ioctl, just pass to the decoder */
		return dvr->dvr_decoder->ioctl(cmd, arg, dvr->decoder);
	}
	
	return 0;
}

/* -------------------------------------------------------------------------- */

	/*
	 *  file operations
	 */

static unsigned int
omap34xxdvr_poll(struct file *file, struct poll_table_struct *wait)
{
	struct omap34xxdvr_fh *fh = file->private_data;
	struct omap34xxdvr_device *dvr = fh->dvr;
	struct videobuf_buffer *vb;
	enum v4l2_field field;

	omap34xxdvr_suspend_lockout(dvr, file);

	spin_lock(&dvr->img_lock);

	if (dvr->streaming == fh) {
		spin_unlock(&dvr->img_lock);
		/* streaming capture */
		if (list_empty(&fh->vbq.stream))
			return POLLERR;
		vb = list_entry(fh->vbq.stream.next, struct videobuf_buffer,
				stream);
	} else if (dvr->streaming) {
		/* streaming I/O is in progress on another file descriptor */
		spin_unlock(&dvr->img_lock);
		return POLLERR;
	} else {
		/* read() capture */
		spin_unlock(&dvr->img_lock);
		mutex_lock(&fh->vbq.lock);
		dvr->still_capture = 1;
		if (fh->vbq.read_buf == NULL) {
			/* need to capture a new image */
			fh->vbq.read_buf = videobuf_alloc(fh->vbq.msize);
			if (fh->vbq.read_buf == NULL) {
				mutex_unlock(&fh->vbq.lock);
				return POLLERR;
			}
			fh->vbq.read_buf->memory = V4L2_MEMORY_USERPTR;
			field = videobuf_next_field(&fh->vbq);
			if (fh->vbq.ops->buf_prepare(&fh->vbq, fh->vbq.read_buf,
						     field) != 0) {
				mutex_unlock(&fh->vbq.lock);
				return POLLERR;
			}

			dvr->dma_notify = 1;
			fh->vbq.ops->buf_queue(&fh->vbq, fh->vbq.read_buf);
			fh->vbq.read_off = 0;
		}
		mutex_unlock(&fh->vbq.lock);
		vb = (struct videobuf_buffer *) fh->vbq.read_buf;
	}

	poll_wait(file, &vb->done, wait);
	if (vb->state == STATE_DONE)
		return POLLIN | POLLRDNORM;
	else if (vb->state == STATE_ERROR)
		return POLLERR;
		 
	return 0;
}

static ssize_t
omap34xxdvr_read(struct file *file, char *data, size_t count, loff_t * ppos)
{
	struct omap34xxdvr_fh *fh = file->private_data;
	struct omap34xxdvr_device *dvr = fh->dvr;
	struct omap34xxdvr_fh *preview_fh;
	unsigned long irqflags;
	int free_sgdma, err;

	omap34xxdvr_suspend_lockout(dvr, file);

	/* user buffer has to be word aligned */
	if (((unsigned int) data & 0x3) != 0)
		return -EIO;

	spin_lock(&dvr->img_lock);
	if (dvr->streaming) {
		spin_unlock(&dvr->img_lock);
		return -EBUSY;
	}

      	omap34xxdvrisp_configure_interface(dvr, 2);
	preview_fh = NULL;
	if (dvr->previewing) {
		preview_fh = dvr->previewing;
		/* stop preview */
		dvr->previewing = NULL;
		spin_unlock(&dvr->img_lock);
		/* We need wait until sgdmas used by preview are freed.
		 * To minimize the shot-to-shot delay, we don't want to
		 * yield. Just want to start one-shot capture as soon as
		 * possible. An alternative is to stop the dma but the
		 * sync error would require a reset of the camera system.
		 */
		do {
			/* prevent the race with dma handler */
			spin_lock_irqsave(&dvr->sg_lock, irqflags);
			free_sgdma = dvr->free_sgdma;
			spin_unlock_irqrestore(&dvr->sg_lock, irqflags);
		} while (NUM_SG_DMA != free_sgdma);

		spin_lock(&dvr->img_lock);
	}
	dvr->still_capture = 1;
	spin_unlock(&dvr->img_lock);

#ifdef CONFIG_OMAP3430_ES2
	DPRINTK_DVR("Setting constraint for VDD2\n");
	constraint_set(co_opp_dvr_vdd2, CO_VDD2_OPP3);
	constraint_set(co_opp_dvr_latency, CO_LATENCY_MPURET_COREON);

#endif 

	omap34xxdvr_config_pipeline(dvr, 2);

	dvr->dma_notify = 1;
	if (((unsigned int) data & (32 - 1)) == 0) {
		/* use zero-copy if user buffer is aligned at 32-byte */
		err = videobuf_read_one(&fh->vbq, data, count, ppos, file->f_flags & O_NONBLOCK);
	} else {
		/* if user buffer is not aligned at 32-byte, we will use kernel
		   bounce buffer to capture the image pretending that we want to
		   read one pixel less than the actual image size.
		 */
		err = videobuf_read_one(&fh->vbq, data, count - 2, ppos, file->f_flags & O_NONBLOCK);
	}
	spin_lock(&dvr->img_lock);
	dvr->still_capture = 0;
	spin_unlock(&dvr->img_lock);

	/* if previwing was on, re-start it after the read */
	if (preview_fh) { /* was previewing */
		spin_lock(&dvr->img_lock);
		dvr->previewing = preview_fh;
		spin_unlock(&dvr->img_lock);
      		omap34xxdvrisp_configure_interface(dvr, 1);
		omap34xxdvr_config_pipeline(dvr, 1);
		
		dvr->dma_notify = 1;
		omap34xxdvr_start_overlay_dma(dvr);
	}
#ifdef CONFIG_OMAP3430_ES2
	DPRINTK_DVR("Removing constraint for VDD2\n");
	constraint_remove(co_opp_dvr_vdd2);
	constraint_remove(co_opp_dvr_latency);
#endif 
	return err;
}

static int
omap34xxdvr_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct omap34xxdvr_fh *fh = file->private_data;
	struct omap34xxdvr_device *dvr = fh->dvr;

	omap34xxdvr_suspend_lockout(dvr, file);
	return videobuf_mmap_mapper(&fh->vbq, vma);
}

static int
omap34xxdvr_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		  unsigned long arg)
{
	struct omap34xxdvr_fh *fh = file->private_data;
	struct omap34xxdvr_device *dvr = fh->dvr;

	omap34xxdvr_suspend_lockout(dvr, file);
	return video_usercopy(inode, file, cmd, arg, omap34xxdvr_do_ioctl);
}

static int
omap34xxdvr_release(struct inode *inode, struct file *file)
{
	struct omap34xxdvr_fh *fh = file->private_data;
	struct omap34xxdvr_device *dvr = fh->dvr;
	struct videobuf_queue *q = &fh->vbq;
	int i;

	omap34xxdvr_suspend_lockout(dvr, file);

	spin_lock(&dvr->img_lock);
	/* turn off overlay */
	if (dvr->previewing == fh) {
		dvr->previewing = NULL;
		omap34xxdvr_stop_isp(dvr);
		dvr->overlay_cnt = 0;
		spin_unlock(&dvr->img_lock);
		omap2_disp_disable_layer(dvr->vid_preview);
		omap2_disp_release_layer(dvr->vid_preview);
		omap2_disp_put_dss();
		spin_lock(&dvr->img_lock);
		ispmmu_unmap(dvr->isp_addr_overlay);
	}

	/* stop streaming capture */
	if (dvr->streaming == fh) {
		dvr->streaming = NULL;
		spin_unlock(&dvr->img_lock);
		for (i = 0; i < VIDEO_MAX_FRAME; i++) {
			if (NULL == q->bufs[i])
				continue;
			if (q->bufs[i]->memory == V4L2_MEMORY_USERPTR)
				omap34xxdvr_vbq_release(q, q->bufs[i]);
		}
		videobuf_streamoff(q);
		spin_lock(&dvr->img_lock);
	}
	dvr->free_sgdma = NUM_SG_DMA;
	spin_unlock(&dvr->img_lock);

	/* release read_buf videobuf_buffer struct */
	if (fh->vbq.read_buf) {
		omap34xxdvr_vbq_release(q, fh->vbq.read_buf);
		kfree(fh->vbq.read_buf);
	}

	/* free video_buffer objects */
	videobuf_mmap_free(q);

	file->private_data = NULL;
	kfree(fh);
	ispccdc_free();

	isp_put();
	isp_temp_state = ISP_BUF_INIT;
	return 0;
}

void omap34xxdvr_decoder_restore(void)
{
	if (sensor_init) {
		/* FIXME: add code for generic video decoder */
		/* mt9p012sensor_restore(dvr->decoder); */
	}
}
EXPORT_SYMBOL(omap34xxdvr_decoder_restore);

static int
omap34xxdvr_open(struct inode *inode, struct file *file)
{
	int minor = iminor(inode);
	struct omap34xxdvr_device *dvr = this_dvr;
	struct omap34xxdvr_fh *fh;

	if (!dvr || !dvr->vfd || (dvr->vfd->minor != minor))
		return -ENODEV;

	isp_temp_state = ISP_BUF_INIT;
	omap34xxdvr_suspend_lockout(dvr, file);

	isp_get();

	/* we request the needed ISP resources in one place */
	if (ispccdc_request()) {
		isp_put();
		return -EBUSY;
	}

	/* allocate per-filehandle data */
	fh = kmalloc(sizeof (*fh), GFP_KERNEL);
	if (NULL == fh) {
		ispccdc_free();
		isp_put();
		return -ENOMEM;
	}
	file->private_data = fh;
	fh->dvr = dvr;
	fh->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	videobuf_queue_init(&fh->vbq, &dvr->vbq_ops, NULL, &dvr->vbq_lock,
			    fh->type, V4L2_FIELD_NONE,
			    sizeof (struct videobuf_buffer), fh);

	if(!isp_temp_ispmmu_addr)
		isp_temp_ispmmu_addr = ispmmu_map(isp_temp_phy_addr,D1_SIZE);

	return 0;
}

static struct file_operations omap34xxdvr_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = omap34xxdvr_read,
	.poll = omap34xxdvr_poll,
	.ioctl = omap34xxdvr_ioctl,
	.mmap = omap34xxdvr_mmap,
	.open = omap34xxdvr_open,
	.release = omap34xxdvr_release,
};

/* -------------------------------------------------------------------------- */
#ifdef USE_PM
#ifdef CONFIG_PM
static int
omap34xxdvr_suspend(struct platform_device *dev, pm_message_t state)
{
	struct omap34xxdvr_device *dvr = platform_get_drvdata(dev);
	if (dvr->suspended)
		return 0;
	/*So as to turn off the xclka if the camera is not running.*/
	if(!(dvr->previewing || dvr->streaming || dvr->still_capture))
		isp_get();
	/* lock-out applications during suspend */
	dvr->suspended = 1;
	/* stall previewing */
	spin_lock(&dvr->img_lock);
	if (dvr->previewing) {
		omap2_disp_disable_layer(dvr->vid_preview);
		/* we still hold the video layer */
		omap2_disp_put_dss();
	}
	spin_unlock(&dvr->img_lock);
	/* ???what else is needed to suspend isp??? */
	if(dvr->streaming || dvr->previewing || dvr->still_capture){
		omap34xxdvr_stop_isp(dvr);
	}

	/* power down the sensor */
	if (dvr->decoder)
		dvr->dvr_decoder->power_off(dvr->decoder);
	
	isp_put();
	isp_temp_state = ISP_BUF_INIT;
	return 0;
}
static int
omap34xxdvr_resume(struct platform_device *dev)
{
	struct omap34xxdvr_device *dvr = platform_get_drvdata(dev);
	if (!dvr->suspended)
		return 0;

	isp_get();

	/* power up the sensor */
	if (dvr->decoder)
		dvr->dvr_decoder->power_on(dvr->decoder);

	if(dvr->streaming || dvr->previewing || dvr->still_capture){
		/* ???what else is needed to resume isp??? */
		if (dvr->free_sgdma > NUM_SG_DMA)
			dvr->free_sgdma = NUM_SG_DMA;

		if (dvr->streaming || dvr->previewing) {
			/* capture or previewing was in progress, so we need to register
			 * our routine to restart the camera interface the next time a
			 * DMA transfer is queued.
			 */
			omap34xxdvr_config_pipeline(dvr, 1);

			dvr->dma_notify = 1;
		}
		if (dvr->previewing) {
			if (dvr->overlay_cnt > 0)
				--dvr->overlay_cnt;
			omap2_disp_get_dss();
			omap2_disp_enable_layer(dvr->vid_preview);
			omap34xxdvr_start_overlay_dma(dvr);
		}
		if (dvr->streaming) {
			omap34xxdvr_sg_dma_process(dvr,0);
		}
	}
	else{
		isp_put();
	}
	/* camera interface will be enabled through dma_notify function
	 ** automatically when new dma starts
	 */

	/* wake up applications waiting on suspend queue */
	dvr->suspended = 0;
	wake_up(&dvr->suspend_wq);
	return 0;
}
#endif				/* PM */

static int
omap34xxdvr_probe(struct platform_device *dev)
{
	return 0;
}
static void
omap34xxdvr_dev_release(struct device *dev)
{
}

static struct platform_device omap34xxdvr_dev = {
	.name = DVR_NAME,
	.id   = 100,
	.dev = {
		.release = omap34xxdvr_dev_release,
		},
};

static struct platform_driver omap34xxdvr_driver = {
	.driver = {
		.name = DVR_NAME,
		},
	.probe = omap34xxdvr_probe,
#ifdef CONFIG_PM
	.suspend = omap34xxdvr_suspend,
	.resume = omap34xxdvr_resume,
#endif
};
#endif

static void
omap34xxdvrisp_config_ispsensor(struct omap34xxdvr_device *dvr)
{
	u32 w = dvr->pix.width;
	u32 h = dvr->pix.height;

	ispccdc_config_datapath(CCDC_YUV_BT, CCDC_OTHERS_MEM);
	ispccdc_try_size(w, h, &dvr->pix.width, &dvr->pix.height);
	printk("The agreed initial preview size (ISP sensor):\n");
	printk("\tsensor ouput (%u, %u)\n", w, h);
	printk("\tCCDC output (%u, %u)\n", dvr->pix.width, dvr->pix.height);
	/* set a new default display window and preview crop */
	omap34xxdvr_preview(dvr);
	ispccdc_config_size(w, h, dvr->pix.width, dvr->pix.height);
}

/*
 * API to be used by sensor drivers to register with camera driver
 */ 
int omap_dvr_register_decoder(struct video_decoder *decoder)
{
	struct omap34xxdvr_device *dvr = this_dvr;

	isp_get();

	dvr->dvr_decoder = decoder;
	dvr->if_type = ISP_PARLL;
	
	if (dvr->dvr_decoder->decoder_interface == SENSOR_SERIAL1)
		dvr->if_type = ISP_CSIA;
	else if (dvr->dvr_decoder->decoder_interface == SENSOR_SERIAL2)
		dvr->if_type = ISP_CSIB;
	if (isp_request_interface(dvr->if_type)) {
		printk(KERN_ERR DVR_NAME ": cannot get isp interface\n");
		isp_put();
		return -EINVAL;
	}
	
	/* initialize the decoder and define a default capture format dvr->pix */
	dvr->decoder = dvr->dvr_decoder->init(&dvr->pix);
	if (!dvr->decoder) {
		printk(KERN_ERR DVR_NAME ": cannot initialize decoder\n");
		isp_put();
		return -EINVAL;
	}
	printk(KERN_INFO "Decoder is %s\n", dvr->dvr_decoder->name);

	/* select an arbitrary default capture frame rate of 25fps */
	dvr->nominal_timeperframe.numerator = 1;
	dvr->nominal_timeperframe.denominator = 25;
	
	#if 0
	/* FIXME: something for us?
	 * calculate xclk based on the default capture format and default
	 * frame rate
	 */
	dvr->xclk = dvr->dvr_decoder->calc_xclk(&dvr->pix,
					&dvr->nominal_timeperframe,
					dvr->decoder);
	#endif

	dvr->cparm.timeperframe = dvr->nominal_timeperframe;

	/* initialize the image preview parameters based on the default capture
	 * format
	 */
	omap24xxvout_new_format(&dvr->pix, &dvr->fbuf,
			&dvr->preview_crop, &dvr->win);

	/* program the sensor for the default capture format and rate */
	dvr->dvr_decoder->configure(&dvr->pix, &dvr->cparm.timeperframe, 
				dvr->decoder);

	/* configue ISP for the default image */
	ispccdc_request();
	omap34xxdvrisp_config_ispsensor(dvr);

	ispccdc_free();
	isp_put();
	return 0;
}

/*
 * API to unregister sensor from camera driver
 */ 
int omap_dvr_unregister_decoder(struct video_decoder *decoder)
{
	struct omap34xxdvr_device *dvr = this_dvr;

	if (!decoder)
		return 0;
	isp_free_interface(dvr->if_type);

	/* TO DO: unregister just specified sensor */
	if (dvr->decoder)
		dvr->dvr_decoder->cleanup(dvr->decoder);
	/* sensor allocated private data is gone */

	dvr->decoder = NULL;
	return 0;
}

static void
omap34xxdvr_cleanup(void)
{
	struct omap34xxdvr_device *dvr = this_dvr;
	struct video_device *vfd;

	if (!dvr)
		return;

	vfd = dvr->vfd;

	if (vfd) {
		if (vfd->minor == -1) {
			/* The device was never registered, so release the
			 * video_device struct directly.
			 */
			video_device_release(vfd);
		} else {
			/* The unregister function will release the video_device
			 * struct as well as unregistering it.
			 */
			video_unregister_device(vfd);
		}
		dvr->vfd = NULL;
	}

	isp_get();
	
	if (dvr->isp_addr_overlay) {
		ispmmu_unmap(dvr->isp_addr_overlay);
		dvr->isp_addr_overlay = 0;
	}

	if (dvr->overlay_size > 0 && dvr->overlay_base) {
		dma_free_coherent(NULL, dvr->overlay_size,
			(void *)dvr->overlay_base, dvr->overlay_base_phys);
		dvr->overlay_base = 0;
	}
	dvr->overlay_base = dvr->overlay_base_phys = 0;

	if(isp_temp_ispmmu_addr){
		ispmmu_unmap(isp_temp_ispmmu_addr);
		isp_temp_ispmmu_addr = 0;
		isp_temp_phy_addr = 0;
	}
	
	isp_put();
	
#ifdef CONFIG_OMAP3430_ES2
	DPRINTK_DVR("Releasing constraint for VDD1 and VDD2\n");
	constraint_remove(co_opp_dvr_vdd1);
	constraint_remove(co_opp_dvr_vdd2);
	constraint_remove(co_opp_dvr_latency);

	constraint_put(co_opp_dvr_vdd1);
	constraint_put(co_opp_dvr_vdd2);
	constraint_put(co_opp_dvr_latency);
#endif 	

#ifdef USE_PM
	platform_device_unregister(&omap34xxdvr_dev);
	platform_driver_unregister(&omap34xxdvr_driver);
#endif
	kfree(dvr);
	this_dvr = NULL;
}

/*TODO: to support 2 sensors, move device handing code in init & cleanup to the new
omap_register/unregister_sensor functions */
static int __init
omap34xxdvr_init(void)
{
	struct omap34xxdvr_device *dvr;
	struct video_device *vfd;
	struct isp_sysc isp_sysconfig;
#ifdef USE_PM
	int ret;

	ret = platform_driver_register(&omap34xxdvr_driver);
	if (ret != 0)
		return ret;
	ret = platform_device_register(&omap34xxdvr_dev);
	if (ret != 0) {
		platform_driver_unregister(&omap34xxdvr_driver);
		return ret;
	}
#endif
	dvr = kmalloc(sizeof (struct omap34xxdvr_device), GFP_KERNEL);
	if (!dvr) {
		printk(KERN_ERR DVR_NAME ": could not allocate memory\n");
#ifdef USE_PM
		platform_device_unregister(&omap34xxdvr_dev);
		platform_driver_unregister(&omap34xxdvr_driver);
#endif
		return -ENOMEM;
	}
	memset(dvr, 0, sizeof (struct omap34xxdvr_device));
	this_dvr = dvr;

#ifdef USE_PM
	/* set driver specific data to use in power mgmt functions */
	//	omap_set_drvdata(&omap34xxdvr_dev, dvr);
	platform_set_drvdata(&omap34xxdvr_dev, dvr);
#endif

	dvr->suspended = 0;
	init_waitqueue_head(&dvr->suspend_wq);

	/* initialize the video_device struct */
	vfd = dvr->vfd = video_device_alloc();
	if (!vfd) {
		printk(KERN_ERR DVR_NAME
		       ": could not allocate video device struct\n");
		kfree(dvr);
#ifdef USE_PM
		platform_device_unregister(&omap34xxdvr_dev);
		platform_driver_unregister(&omap34xxdvr_driver);
#endif
		this_dvr = NULL;
		return -ENODEV;
	}
	vfd->release = video_device_release;

	strncpy(vfd->name, DVR_NAME, sizeof (vfd->name));
	vfd->type = VID_TYPE_CAPTURE | VID_TYPE_OVERLAY | VID_TYPE_CHROMAKEY;
	/* need to register for a VID_HARDWARE_* ID in videodev.h */
	vfd->hardware = 0;
	vfd->fops = &omap34xxdvr_fops;
#ifdef USE_PM
	video_set_drvdata(vfd, dvr);
#endif
	vfd->minor = -1;

	/* initialize the videobuf queue ops */
	dvr->vbq_ops.buf_setup = omap34xxdvr_vbq_setup;
	dvr->vbq_ops.buf_prepare = omap34xxdvr_vbq_prepare;
	dvr->vbq_ops.buf_queue = omap34xxdvr_vbq_queue;
	dvr->vbq_ops.buf_release = omap34xxdvr_vbq_release;
	spin_lock_init(&dvr->vbq_lock);

	/* allocate coherent memory for the overlay framebuffer */
	dvr->overlay_size = overlay_mem;
	if (dvr->overlay_size > 0) {
		dvr->overlay_base = (unsigned long) dma_alloc_coherent(NULL,
			dvr->overlay_size,
			(dma_addr_t *) &dvr->overlay_base_phys,
			GFP_KERNEL | GFP_DMA);
		if (!dvr->overlay_base) {
			printk(KERN_ERR DVR_NAME
			       "\n\n\n\n\n\n: cannot allocate overlay framebuffer\n");
			goto init_error;
		}
		
		memset((void *) dvr->overlay_base, 0, dvr->overlay_size);

		/* this is a VRFB address in the case of rotation */
		dvr->overlay_base_dma = dvr->overlay_base_phys;
		printk(KERN_INFO "overlay buffer at 0x%x\n", (unsigned int)dvr->overlay_base_dma);
	}
	
	isp_get();
	isp_sysconfig.reset = 0;
	isp_sysconfig.idle_mode = 1;
	isp_power_settings(isp_sysconfig);

	/* initialize the overlay spinlock  */
	spin_lock_init(&dvr->overlay_lock);

	/* initialize the spinlock used to serialize access to the image
	 * parameters
	 */

	spin_lock_init(&dvr->img_lock);
	spin_lock_init(&isp_temp_buf_lock);

	isp_temp_virt_addr = dvr->overlay_base;
	isp_temp_phy_addr = dvr->overlay_base_phys;
       	isp_temp_ispmmu_addr = ispmmu_map(isp_temp_phy_addr,D1_SIZE);

  	/* initialize the streaming capture parameters */
	dvr->cparm.readbuffers = 1;
	dvr->cparm.capability = V4L2_CAP_TIMEPERFRAME;

	/* get the framebuffer parameters in case the sensor init routine
	 * needs them
	 */
	/* the display controller video layer used for camera preview */
	dvr->vid_preview = OMAP2_VIDEO1;
	omap2_disp_get_dss();
	omap2_disp_get_panel_size(omap2_disp_get_output_dev(dvr->vid_preview),
				  &(dvr->fbuf.fmt.width),
				  &(dvr->fbuf.fmt.height));
	omap2_disp_put_dss();

	/* initialize the SGDMA data */
	omap34xxdvr_sgdma_init(dvr);

	if (video_register_device(vfd, VFL_TYPE_GRABBER, video_nr) < 0) {
		printk(KERN_ERR DVR_NAME
		       ": could not register Video for Linux device\n");
		isp_put();
		goto init_error;
	}
	else
		vfd->minor = video_nr;

	printk(KERN_INFO DVR_NAME
	       ": registered device video%d [v4l2]\n", vfd->minor);
	isp_put();
#ifdef CONFIG_OMAP3430_ES2	
	DPRINTK_DVR("Getting constraint for VDD1 and VDD2\n");
	co_opp_dvr_latency = constraint_get("omap34xxdvr", &cnstr_id_latency);
	co_opp_dvr_vdd1 = constraint_get("omap34xxdvr", &cnstr_id_vdd1);
	co_opp_dvr_vdd2 = constraint_get("omap34xxdvr", &cnstr_id_vdd2);
#endif	

#undef DVR_TEST
#ifdef DVR_TEST
	if (dvr->overlay_size > 0 && dvr->pix.sizeimage <= dvr->overlay_size) {
		printk(KERN_INFO DVR_NAME ": DVR test--enabling overlay\n");
		isp_get();
		/* does one-time mapping for overlay buffer */
		sg_dma_address(&dvr->overlay_sglist) = dvr->overlay_base_dma;
		sg_dma_len(&dvr->overlay_sglist) = dvr->overlay_size;
		dvr->isp_addr_overlay =
			ispmmu_map(sg_dma_address(&dvr->overlay_sglist), 
				sg_dma_len(&dvr->overlay_sglist));

		ispccdc_request();
		omap34xxdvrisp_configure_interface(dvr, 1);
		omap34xxdvrisp_config_pipeline(dvr, 1);

		omap2_disp_get_dss();
		/* turn on the video overlay */
		omap2_disp_config_vlayer(dvr->vid_preview, &dvr->pix,
					 &dvr->preview_crop, &dvr->win, -1, 0);
		omap2_disp_start_vlayer(dvr->vid_preview, &dvr->pix, 
					&dvr->preview_crop, &dvr->win,
					dvr->overlay_base_dma, -1, 0);
		dvr->previewing = (struct omap34xxdvr_fh *) 1;
		
		/* start the camera interface */
		dvr->dma_notify = 1;
		omap34xxdvr_start_overlay_dma(dvr);

		/* don't free resource since preview runs forever */
	} else {
		printk(KERN_INFO DVR_NAME
		       ": Can't start camera test--overlay buffer too"
		       " small\n");
	}
#endif
	return 0;

init_error:
	omap34xxdvr_cleanup();
	return -ENODEV;
}


MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("OMAP34xx Video for Linux camera driver");
MODULE_LICENSE("GPL");

EXPORT_SYMBOL_GPL(omap_dvr_register_decoder);
EXPORT_SYMBOL_GPL(omap_dvr_unregister_decoder);

module_param(video_nr, int, 0);
MODULE_PARM_DESC(video_nr,
		"Minor number for video device (-1 ==> auto assign)");

module_param(overlay_mem, int, 0);
MODULE_PARM_DESC(overlay_mem,
	"Preview overlay framebuffer size (default 600KB)");

module_init(omap34xxdvr_init);
module_exit(omap34xxdvr_cleanup);

