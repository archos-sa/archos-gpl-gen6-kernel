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

#include "isp/isp.h"
#include "isp/ispccdc.h"
#include "isp/ispmmu.h"

#include "isp/ispreg.h"

#include "omap24xxlib.h"
#include "omap24xxcam_user.h"
#include "sensor_if.h"


#define TS_HLINE_SIZE 192
#define TS_PACKET_SIZE 188
#define TS_PACKETS_PER_LINE 1

// for normal transfer mode
#define LINES_XFER 256

// during channel scanning
#define LINES_SCAN 1

/* number of complete "frames" (we can decide this) */
#define XFERS_NUM 4

/* NUM_SG_DMA is the number of scatter-gather DMA transfers that can be queued.
 * We need it to be 2 greater than the maximum number of frames so that 
 * we can use 2 slots for overlay while still having XFERS_NUM slots left 
 * for streaming.
 */
#define NUM_SG_DMA (XFERS_NUM+3)


static unsigned int debug = 0;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug,"enable debug messages");

#define dprintk(fmt, arg...)	if (debug)			\
	printk("%s: " fmt, "omap34xxarcdvb" , ## arg)

struct sgdma_state
{
	dma_addr_t isp_addr;     /* ISP space addr */ 
	unsigned long status;    /* DMA return code */
	isp_callback_t callback;
	void *arg;
};

struct driver_data_t
{
	struct videobuf_queue_ops vbq_ops; /* videobuf queue operations */
	spinlock_t vbq_lock;               /* spinlock for videobuf queues */
	
	/* scatter-gather DMA management */
	spinlock_t sg_lock;
	int free_sgdma;    /* number of free sg dma slots */
	int next_sgdma;    /* index of next sg dma slot to use */
	struct sgdma_state sgdma[NUM_SG_DMA];
	/* dma_notify is a flag to indicate a DMA transfer has been started. */
	int dma_notify;
	enum { STOP=0, RUNNING } state;
	
	dma_addr_t isp_addrs[NUM_SG_DMA];
	
	int xfer_mode; /* 0 = transfer, 1 = scan */
};

static struct driver_data_t *global_data = NULL;

/* wrapper functions for ISP related calls */
static void omap34xxarcdvb_isp_lock(void)
{
	isp_get();
}

static void omap34xxarcdvb_isp_unlock(void)
{
	isp_put();
}

static void omap34xxarcdvb_ispccdc_lock(void)
{
	ispccdc_request();
}

static void omap34xxarcdvb_ispccdc_unlock(void)
{
	ispccdc_free();
}

static void omap34xxarcdvb_isp_set_buf(struct sgdma_state *sgdma)
{
	ispccdc_set_outaddr(sgdma->isp_addr);		
}

static void omap34xxarcdvb_isp_set_callback(struct driver_data_t *glob, struct sgdma_state *sgdma)
{
	isp_set_callback(CBK_CCDC_VD0, sgdma->callback, glob, sgdma->arg);
}

static void omap34xxarcdvb_isp_unset_callback(void)
{
	isp_unset_callback(CBK_CCDC_VD0);
}

static void omap34xxarcdvb_isp_start(void)
{
	ispccdc_enable(1);
}

static int omap34xxarcdvb_isp_stop(void)
{
dprintk("omap34xxarcdvb_isp_stop\n");
	omap34xxarcdvb_isp_unset_callback();

	ispccdc_enable(0);
	while (ispccdc_busy()) {
dprintk("ISP_stop ");
		mdelay(10);
	}
dprintk("\n");
	return 0;
}

/* SG DMA Stuff */

/*
 * Process the scatter-gather DMA queue by starting queued transfers.
 * This can be called from either a process context or an IRQ context.
 * 	case 1: from a process, ISP not started.
 * 	case 2: from a process, ISP started.
 * 	case 3: from IRQ, ISP started.
 * We make sure updating buffer pointer happens only once on 
 * the shadow register
 *
 */
static int omap34xxarcdvb_sg_dma_process(struct driver_data_t *glob, int irq)
{
	struct sgdma_state *sgdma;
	unsigned long irqflags;
	
dprintk("P");
	
	spin_lock_irqsave(&glob->sg_lock, irqflags);
	
	// any pending sgdma? we can at most start or queue one sgdma 
	if ((NUM_SG_DMA - glob->free_sgdma) > 0) {
		// get the next sgdma
		sgdma = glob->sgdma + (glob->next_sgdma + glob->free_sgdma) % NUM_SG_DMA;
		if (!irq) {
			if (glob->dma_notify) {
dprintk("X%d\n", (glob->next_sgdma + glob->free_sgdma) % NUM_SG_DMA);
				
				// case 1: queue & start.
				omap34xxarcdvb_isp_set_callback(glob, sgdma);
				omap34xxarcdvb_isp_set_buf(sgdma);
				omap34xxarcdvb_isp_start();
				glob->dma_notify = 0;
			}
			else {
				// case 2:
dprintk(".");
				// defer queuing until the current frame done
				// assuming we don't miss the vsync of the
				// next frame.
			}
		}
		// from IRQ
		else {
dprintk("I");
			// case 3: only need to queue (update buf ptr).
			
			// queue next dma transfer
dprintk("%x %d\n", sgdma->isp_addr >> 24, (glob->next_sgdma + glob->free_sgdma) % NUM_SG_DMA );
dprintk("%d\n", (glob->next_sgdma + glob->free_sgdma) % NUM_SG_DMA );

			omap34xxarcdvb_isp_set_callback(glob, sgdma);
			omap34xxarcdvb_isp_set_buf(sgdma);
			
			// TODO: clear irq. old interrupt can come first. OK for preview.
	    
	    		// make sure to start the isp again
			if (glob->dma_notify) {
				omap34xxarcdvb_isp_start();
				glob->dma_notify = 0;
			}
		}
	}
	else {
		dprintk("%s: no SGs\n", __FUNCTION__);
		// we don't have any SGs to process
		spin_unlock_irqrestore(&glob->sg_lock, irqflags);
		return 0;
	}

	spin_unlock_irqrestore(&glob->sg_lock, irqflags);
	
	return 1;
}

/* 
 *  Queue a scatter-gather DMA transfer from the CCDC interface to memory.
 *  Returns zero if the transfer was successfully queued, or
 *   non-zero if all of the scatter-gather slots are already in use.
 */
static int omap34xxarcdvb_sg_dma_queue(struct driver_data_t *glob, dma_addr_t isp_addr, isp_callback_t callback, void *arg, int irq)
{
	unsigned long irqflags;
	struct sgdma_state *sgdma;
	int ret = 0;

	dprintk("S%d", glob->next_sgdma );

	spin_lock_irqsave(&glob->sg_lock, irqflags);

	// return error is all SGs are used
	if (!glob->free_sgdma) {
		dprintk("%s: no empty SGs\n", __FUNCTION__);
		spin_unlock_irqrestore(&glob->sg_lock, irqflags);
		return -EBUSY;
	}

	sgdma = glob->sgdma + glob->next_sgdma;

	sgdma->isp_addr = isp_addr;
	sgdma->status = 0;
	sgdma->callback = callback;
	sgdma->arg = arg;

	glob->next_sgdma = (glob->next_sgdma + 1) % NUM_SG_DMA;
	glob->free_sgdma--;

	spin_unlock_irqrestore(&glob->sg_lock, irqflags);
	dprintk("fr: %d\n", glob->free_sgdma);
	ret = omap34xxarcdvb_sg_dma_process(glob, irq);

	return ret;
}

static void omap34xxarcdvb_sgdma_init(struct driver_data_t *glob)
{
	int sg;

	glob->free_sgdma = NUM_SG_DMA;
	glob->next_sgdma = 0      ;
	for (sg = 0; sg < NUM_SG_DMA; sg++) {
		glob->sgdma[sg].status = 0;
		glob->sgdma[sg].callback = NULL;
		glob->sgdma[sg].arg = NULL;
	}
}

/* 
 *  This is an IRQ callback!
 *
 *  This routine is called from interrupt context when a scatter-gather DMA
 *   transfer of a videobuf_buffer completes.
 *
 */
static void omap34xxarcdvb_vbq_complete(unsigned long status, void *arg1, void *arg2)
{
	struct driver_data_t *glob = (struct driver_data_t*) arg1;
	struct videobuf_buffer *vb = (struct videobuf_buffer*) arg2;

	spin_lock(&glob->vbq_lock);

	switch (status) {
	case CCDC_VD0:
		/* Program shadow registers for CCDC */
		ispccdc_config_shadow_registers();
#if 0
static int t=0;
struct timeval tv;
do_gettimeofday( &tv );
int ms=tv.tv_usec/1000 + (tv.tv_sec % 1048576)*1000;
dprintk("V%d", ms-t);
t=ms;
#endif
		if ( global_data->state == RUNNING )
			omap34xxarcdvb_sg_dma_process(glob, 1);
		break;
	case PREV_DONE:
		dprintk("%s: PREV_DONE shouldn't be called\n", __FUNCTION__);
		spin_unlock(&glob->vbq_lock);
		return;
		break;
	case RESZ_DONE:
		dprintk("%s: RESZ_DONE shouldn't be called\n", __FUNCTION__);
		spin_unlock(&glob->vbq_lock);
		return;
		break;
	default:
		dprintk("%s: default shouldn't be called\n", __FUNCTION__);
		spin_unlock(&glob->vbq_lock);
		return;
		break;
	}

	// free completed sg element
	spin_lock(&glob->sg_lock);
	glob->free_sgdma++;
	if (glob->free_sgdma > NUM_SG_DMA)
		glob->free_sgdma = NUM_SG_DMA;
	spin_unlock(&glob->sg_lock);
	
	do_gettimeofday(&vb->ts);
	vb->state = STATE_DONE;
	if ( global_data->state == RUNNING )
		omap34xxarcdvb_sg_dma_process(glob, 1);
	
	// sync cache

	dmac_flush_range(vb->dma.vmalloc, (vb->dma.vmalloc + (vb->dma.nr_pages << PAGE_SHIFT)));
	outer_flush_range(__pa(vb->dma.vmalloc), __pa(vb->dma.vmalloc + (vb->dma.nr_pages << PAGE_SHIFT)));

if (1)	{	
	unsigned char *p = vb->dma.vmalloc;
	dprintk("W%02x%02x %x ", p[0], p[192], (uint32_t)(p)>>12 );
}

	wake_up(&vb->done);
	spin_unlock(&glob->vbq_lock);
}

/* video-buf-queue ops */
static void omap34xxarcdvb_vbq_release(struct videobuf_queue *q, struct videobuf_buffer *vb)
{
	unsigned long irqflags;
	dprintk("%s called\n", __FUNCTION__);
	
	omap34xxarcdvb_isp_stop();
	
	// we already drain the queue so videobuf_waiton is no longer needed
	videobuf_dma_unmap(q, &vb->dma);
	videobuf_dma_free(&vb->dma);

	if (global_data->isp_addrs[vb->i]) {
		ispmmu_unmap(global_data->isp_addrs[vb->i]);
		global_data->isp_addrs[vb->i] = 0;
	}

	vb->state = STATE_NEEDS_INIT;
	
	/* re-init sgdma */
	spin_lock_irqsave(&global_data->sg_lock, irqflags);
	global_data->state = STOP;
	omap34xxarcdvb_sgdma_init(global_data);
	global_data->dma_notify = 1;
	spin_unlock_irqrestore(&global_data->sg_lock, irqflags);
}

static int omap34xxarcdvb_vbq_setup(struct videobuf_queue *q, unsigned int *cnt, unsigned int *size)
{
	if ((*cnt) <= 0)
		*cnt = XFERS_NUM;

	if ((*cnt) > XFERS_NUM)
		*cnt = XFERS_NUM;
		
	*size = TS_HLINE_SIZE * LINES_XFER;

	return 0;
}

static int omap34xxarcdvb_vbq_prepare(struct videobuf_queue *q, struct videobuf_buffer *vb, enum v4l2_field field)
{
	dma_addr_t isp_addr;
	unsigned int lines, llength, size;
	int err = 0;

	dprintk("%s called\n", __FUNCTION__);

	llength = TS_HLINE_SIZE * TS_PACKETS_PER_LINE + 16; // some margin for the MMU...
	lines = LINES_XFER;
	size = lines * llength;
	dprintk("frame size: %d\n", size);

	if (vb->state != STATE_NEEDS_INIT) {
		dprintk("%s: vb->stat != INIT\n", __FUNCTION__);
		if (size > vb->size) {
			omap34xxarcdvb_vbq_release(q, vb);
			vb->size = size;
		}
	}

	if (vb->state == STATE_NEEDS_INIT) {
		vb->size = size;
		vb->width = llength;
		vb->height = lines;
		vb->field = field;
	
		dprintk("omap34xxarcdvb_vbq_prepare: callinf videobuf_iolock\n");
		err = videobuf_iolock(q, vb, NULL);		
		if (!err) {
			dprintk("%s sglen = %d\n", __FUNCTION__, vb->dma.sglen);
			isp_addr = ispmmu_map_sg(vb->dma.sglist, vb->dma.sglen);
			dprintk("%d isp_addr: %08x %08x\n", vb->i, isp_addr, vb->dma.vmalloc);
			if (!isp_addr)
				err = -EIO;
			else {
				global_data->isp_addrs[vb->i]= isp_addr;
				memset(vb->dma.vmalloc, 0, size);
			}
		}
		else {
			dprintk("omap34xxarcdvb_vbq_prepare: ret from videobuf_iolock: %d\n", err);
		}
		INIT_LIST_HEAD(&vb->queue);
	}

	if (!err) {
		vb->state = STATE_PREPARED;
		// sync a kernel buffer
		dmac_flush_range(vb->dma.vmalloc, (vb->dma.vmalloc + (vb->dma.nr_pages << PAGE_SHIFT)));
		outer_flush_range(__pa(vb->dma.vmalloc), __pa(vb->dma.vmalloc + (vb->dma.nr_pages << PAGE_SHIFT)));
		
		global_data->state = RUNNING;
	} else
		omap34xxarcdvb_vbq_release(q, vb);

	return err;
}

static void omap34xxarcdvb_vbq_queue(struct videobuf_queue *q, struct videobuf_buffer *vb)
{
	enum videobuf_state state = vb->state;
	int err;

dprintk("Q%d %d", vb->i, ( vb->i + 1 )%XFERS_NUM);

	vb->state = STATE_QUEUED;
	err = omap34xxarcdvb_sg_dma_queue(global_data, global_data->isp_addrs[( vb->i + 1 )%XFERS_NUM], omap34xxarcdvb_vbq_complete, vb, 0);
	if (!err) {
		/* Oops.  We're not supposed to get any errors here.  The only
		 * way we could get an error is if we ran out of scatter-gather
		 * DMA slots, but we are supposed to have at least as many
		 * scatter-gather DMA slots as video buffers so that can't
		 * happen.
		 */
		dprintk(KERN_DEBUG ": Failed to queue a video buffer for DMA!\n");
		vb->state = state;
	}
	else {
		//vb->state = STATE_ACTIVE;
	}

}


/*
 *  configure ccdc pipeline to mode (C)
 *  copy data from ccdc directly to memory
 *
 */
 
static int omap34xxarcdvb_isp_config_interface(void)
{
	struct isp_interface_config config;
	u32 outw, outh;
	
	if (isp_request_interface(ISP_PARLL)) {
		dprintk("%s isp_request_interface(PARALLEL) failed\n", __FUNCTION__);
		return 0;
	}
		
	// this is a RAW sensor	
	
	config.ccdc_par_ser = SENSOR_PARALLEL;
	config.dataline_shift = 0;
	config.hsvs_syncdetect = 3; // VS and HS on rising edge
	config.para_clk_pol = 1; // clock on falling edge
	config.par_bridge = 0;
	config.shutter = 0;
	config.prestrobe = 0;
	config.strobe = 0;
	config.vdint0_timing = 0;
	config.vdint1_timing = 0;
	isp_configure_interface(&config);
		
	ispccdc_config_datapath(CCDC_RAW_DVB, CCDC_OTHERS_MEM);
		
	ispccdc_enable_black_clamp( 0 );
	ispccdc_config_black_comp( (struct ispccdc_blcomp){ 0, 0, 0, 0 } );

	outw = TS_HLINE_SIZE;
	outh = LINES_XFER;
	ispccdc_try_size( TS_HLINE_SIZE, LINES_XFER, &outw, &outh );
	ispccdc_config_size(TS_HLINE_SIZE, LINES_XFER, outw, outh);
	ispccdc_config_outlineoffset(TS_HLINE_SIZE, 0, 0);

//	omap_writel((( LINES_XFER-1 ) << ISPCCDC_VDINT_0_SHIFT) , ISPCCDC_VDINT);
	omap_writel((( 0 ) << ISPCCDC_VDINT_0_SHIFT) , ISPCCDC_VDINT);

	ispccdc_config_culling( (struct ispccdc_culling){ 0xFF, 0xFF, 0xFF });
	ispccdc_enable_vp( 0 );

	ispccdc_print_status();
	
	return 1;
}

static void omap34xxarcdvb_isp_power_on(void)
{
	struct isp_sysc isp_sysconfig;
	isp_sysconfig.reset = 0;
	isp_sysconfig.idle_mode = 1;
	isp_power_settings(isp_sysconfig);
}

/* --- exported symbols --- */
struct videobuf_queue_ops* omap34xxarcdvb_get_videobufqops(void)
{
	return &(global_data->vbq_ops);
}
EXPORT_SYMBOL(omap34xxarcdvb_get_videobufqops);

spinlock_t* omap34xxarcdvb_get_videobufqlock(void)
{
	return &(global_data->vbq_lock);
}
EXPORT_SYMBOL(omap34xxarcdvb_get_videobufqlock);

/* --- END --- exported symbols --- */

static int __init omap34xxarcdvb_init(void)
{
	global_data = kmalloc(sizeof(struct driver_data_t), GFP_KERNEL | GFP_DMA);
	memset(global_data, 0, sizeof(struct driver_data_t));
	
	global_data->dma_notify = 1;
	global_data->state = STOP;
	omap34xxarcdvb_sgdma_init(global_data);
	
	/* initialize the videobuf queue ops */
	global_data->vbq_ops.buf_setup = omap34xxarcdvb_vbq_setup;
	global_data->vbq_ops.buf_prepare = omap34xxarcdvb_vbq_prepare;
	global_data->vbq_ops.buf_queue = omap34xxarcdvb_vbq_queue;
	global_data->vbq_ops.buf_release = omap34xxarcdvb_vbq_release;
	spin_lock_init(&global_data->vbq_lock);
	
	omap34xxarcdvb_isp_lock();
	
	/* TODO: GIO configuration to put the FPGA into the right mode... */
	
	omap34xxarcdvb_isp_power_on();
	omap34xxarcdvb_ispccdc_lock();
	omap34xxarcdvb_isp_config_interface();
	
	return 0;
}

static void omap34xxarcdvb_exit(void)
{
	printk("omap34xxarcdvb_exit\n");
	// isp_register_interface
	isp_free_interface(ISP_PARLL);
	
	omap34xxarcdvb_ispccdc_unlock();
	omap34xxarcdvb_isp_unlock();

	kfree(global_data);
	global_data = NULL;
}

module_init(omap34xxarcdvb_init);
module_exit(omap34xxarcdvb_exit);

MODULE_AUTHOR("Collin Mulliner , Archos S.A.");
MODULE_DESCRIPTION("OMAP34xx CCDC-DVB Driver");
MODULE_LICENSE("GPL");
