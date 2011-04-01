#include <linux/moduleparam.h>
#include <linux/module.h>
#include <linux/init.h>

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/highmem.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/blkdev.h>
#include <scsi/scsi_device.h>

#include <linux/mmc/mmc.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>

#include <linux/libata.h>
#include <linux/ata.h>

#include <linux/freezer.h>

#include "libata-ceata.h"

#define DRV_NAME "ce-ata"

#define CE_ATA_TASKFILE_EXP_FEATURES		1
#define CE_ATA_TASKFILE_EXP_SECTOR_COUNT 	2
#define CE_ATA_TASKFILE_EXP_LBA_LOW		3
#define CE_ATA_TASKFILE_EXP_LBA_MID		4
#define CE_ATA_TASKFILE_EXP_LBA_HIGH		5
#define CE_ATA_TASKFILE_CONTROL			6
#define CE_ATA_TASKFILE_ERROR			9
#define CE_ATA_TASKFILE_FEATURES		9
#define CE_ATA_TASKFILE_SECTOR_COUNT		10
#define CE_ATA_TASKFILE_LBA_LOW			11
#define CE_ATA_TASKFILE_LBA_MID			12
#define CE_ATA_TASKFILE_LBA_HIGH		13
#define CE_ATA_TASKFILE_DEVICE_HEAD		14
#define CE_ATA_TASKFILE_STATUS			15
#define CE_ATA_TASKFILE_COMMAND			15
	
#define CONTROL_MASK	0x06

#define CE_ATA_TIMEOUT_MS    10000

#define FIO		0x02
#define NBR		0x01

extern int ceata_rw_multiple_register(struct mmc_card *card, int rwFlag, unsigned char addr, u8 *buf, unsigned char count);
extern int ceata_rw_multiple_block(struct mmc_card *card, int rwFlag, u8 *buf, u16 data_unit_count);
extern int ceata_rw_sg(struct mmc_card *card, int rw, struct ata_queued_cmd *qc);
extern int mmc_fast_io(struct mmc_card *card, int rw, u8 addr, u8 *data);
extern int ce_ata_stop_transmission(struct mmc_card *card, int rw);
extern int ceata_set_clock(struct mmc_card* card, int dsor);

static inline void hd_access_enter(struct ceata_port *cp)
{
	if (cp->wake_up_hook)
		cp->wake_up_hook(cp);	
	cp->hd_access++;
}

static inline void hd_access_leave(struct ceata_port *cp)
{
	cp->io_count++;
	cp->hd_access--;
}

static u8 read_reg8(struct ata_port *ap, void __iomem* _addr)
{
	unsigned long addr = (unsigned long)_addr;
	struct ceata_port *cp = (struct ceata_port *)ap->private_data;
	int err;
	u8 val;
	
	hd_access_enter(cp);
	
	mmc_claim_host(cp->mmc);

	err = mmc_fast_io(cp->mmc->card, 0, (u8)addr, &val);			
	if (err != MMC_ERR_NONE) {
		printk("read_reg8 failed\n");
		val = 0xff;
	}
	
	mmc_release_host(cp->mmc);
	
	hd_access_leave(cp);
		
	return val;
}

static void write_reg8(struct ata_port *ap, u8 val, void __iomem* _addr)
{
	unsigned long addr = (unsigned long)_addr;
	struct ceata_port *cp = (struct ceata_port *)ap->private_data;
	int err;

	hd_access_enter(cp);

	mmc_claim_host(cp->mmc);
	
	err = mmc_fast_io(cp->mmc->card, 1, (u8)addr, &val);			
	if (err != MMC_ERR_NONE)
		printk("write_reg8: mmc_fast_io failed\n");
	
	mmc_release_host(cp->mmc);

	hd_access_leave(cp);
}

static unsigned int ceata_dev_classify(const struct ata_taskfile *tf)
{
	/* Apple's open source Darwin code hints that some devices only
	 * put a proper signature into the LBA mid/high registers,
	 * So, we only check those.  It's sufficient for uniqueness.
	 */

	if ((tf->lbam == 0xce) && (tf->lbah == 0xaa)) {
		DPRINTK("found CE-ATA device by sig\n");
		return ATA_DEV_CEATA;
	}

	DPRINTK("unknown device\n");
	return ATA_DEV_UNKNOWN;
}

static unsigned int ceata_dev_try_classify(struct ata_port *ap, unsigned int device, u8 *r_err)
{
	struct ceata_port *cp = (struct ceata_port *)ap->private_data;
	struct ata_taskfile tf;
	unsigned int class;
	struct ata_ioports *ioaddr = &ap->ioaddr;
	
	ap->ops->dev_select(ap, device);

	memset(&tf, 0, sizeof(tf));

	tf.lbam = read_reg8(ap, ioaddr->lbam_addr);
	tf.lbah = read_reg8(ap, ioaddr->lbah_addr);
	tf.device = read_reg8(ap, ioaddr->device_addr);
	
	/* determine if device is ATA or ATAPI */
	class = ceata_dev_classify(&tf);

	if (tf.device & FIO)
		cp->fio = 1;
	else
		cp->fio = 0;
		
	DPRINTK("Issuing ATA command with FAST_IO %ssupported\n", cp->fio ? "" : "not ");
	
	if (tf.device & NBR)
		cp->nbr = 1;
	else
		cp->nbr = 0;

	DPRINTK("Device does %sautomatically reallocate degraded blocks\n", cp->nbr ? "not " : "");
		
	if (class == ATA_DEV_UNKNOWN)
		return ATA_DEV_NONE;
	if ((class == ATA_DEV_CEATA) && (ata_chk_status(ap) == 0))
		return ATA_DEV_NONE;
	return class;
}

static int ceata_set_mode(struct ata_port *ap, struct ata_device **unused)
{
	return 0;
}

static void ceata_port_disable(struct ata_port *ap)
{
	printk(KERN_DEBUG "ceata_port_disable\n");
}

static inline u64 tf_lbanum(struct ata_taskfile *tf)
{
	return (u64)((u64)tf->lbal | ((u64)tf->lbam<<8) | ((u64)tf->lbah<<16) |
		((u64)tf->hob_lbal<<24) | ((u64)tf->hob_lbam<<32) |
		((u64)tf->hob_lbah<<40));
}

static inline u32 tf_nsect(struct ata_taskfile *tf)
{
	return ((u32)tf->nsect)|((u32)tf->hob_nsect<<8);
}

static void tf_read_lbanum(struct ata_port *ap, u64 *lbanum)
{
	struct ata_taskfile tf;

	if (lbanum == NULL)
		return;

	tf.lbal = read_reg8(ap, (void __iomem*)CE_ATA_TASKFILE_LBA_LOW);
	tf.lbam = read_reg8(ap, (void __iomem*)CE_ATA_TASKFILE_LBA_MID);
	tf.lbah = read_reg8(ap, (void __iomem*)CE_ATA_TASKFILE_LBA_HIGH);

	tf.hob_lbal = read_reg8(ap, (void __iomem*)CE_ATA_TASKFILE_EXP_LBA_LOW);
	tf.hob_lbam = read_reg8(ap, (void __iomem*)CE_ATA_TASKFILE_EXP_LBA_MID);
	tf.hob_lbah = read_reg8(ap, (void __iomem*)CE_ATA_TASKFILE_EXP_LBA_HIGH);

	*lbanum = tf_lbanum(&tf);
}

static void ceata_tf_read(struct ata_port *ap, struct ata_taskfile *tf)
{
	tf->command = read_reg8(ap, (void __iomem*)CE_ATA_TASKFILE_COMMAND);
	tf->feature = read_reg8(ap, (void __iomem*)CE_ATA_TASKFILE_FEATURES);
	tf->nsect = read_reg8(ap, (void __iomem*)CE_ATA_TASKFILE_SECTOR_COUNT);
	tf->lbal = read_reg8(ap, (void __iomem*)CE_ATA_TASKFILE_LBA_LOW);
	tf->lbam = read_reg8(ap, (void __iomem*)CE_ATA_TASKFILE_LBA_MID);
	tf->lbah = read_reg8(ap, (void __iomem*)CE_ATA_TASKFILE_LBA_HIGH);
	tf->device = read_reg8(ap, (void __iomem*)CE_ATA_TASKFILE_DEVICE_HEAD);

	tf->hob_feature = read_reg8(ap, (void __iomem*)CE_ATA_TASKFILE_EXP_FEATURES);
	tf->hob_nsect = read_reg8(ap, (void __iomem*)CE_ATA_TASKFILE_EXP_SECTOR_COUNT);
	tf->hob_lbal = read_reg8(ap, (void __iomem*)CE_ATA_TASKFILE_EXP_LBA_LOW);
	tf->hob_lbam = read_reg8(ap, (void __iomem*)CE_ATA_TASKFILE_EXP_LBA_MID);
	tf->hob_lbah = read_reg8(ap, (void __iomem*)CE_ATA_TASKFILE_EXP_LBA_HIGH);
}

static void xlat_tf(u8 *buf, const struct ata_taskfile *tf)
{
	buf[0] = 0;
	buf[1] = 0;//tf->hob_feature;
	buf[2] = tf->hob_nsect;
	buf[3] = tf->hob_lbal;
	buf[4] = tf->hob_lbam;
	buf[5] = tf->hob_lbah;

	buf[6] = 0x02;//(tf->ctl /*| 0x02*/) & CONTROL_MASK;
	buf[7] = 0;
	buf[8] = 0;
	buf[9] = 0;//tf->feature;
	buf[10] = tf->nsect;
	buf[11] = tf->lbal;
	buf[12] = tf->lbam;
	buf[13] = tf->lbah;
	buf[14] = /*tf->device &*/ 0x40;

	buf[15] = tf->command;
}

static void ceata_tf_load(struct ata_port *ap, const struct ata_taskfile *tf)
{
	struct ceata_port *cp = (struct ceata_port *)ap->private_data;

	xlat_tf(cp->tf_buf, tf);
}

static u8 ceata_check_error(struct ata_port *ap)
{
	struct ceata_port *cp = (struct ceata_port *)ap->private_data;

	struct ata_ioports *ioaddr = &ap->ioaddr;
	
	if (!cp || !cp->mmc || !cp->mmc->card)
		return 0xff;

	return read_reg8(ap, ioaddr->error_addr);		
}

static u8 ceata_check_status(struct ata_port *ap)
{
	struct ata_ioports *ioaddr = &ap->ioaddr;
	struct ceata_port *cp = (struct ceata_port *)ap->private_data;
	
	if (!cp || !cp->mmc || !cp->mmc->card)
		return 0xff;
	
	return read_reg8(ap, ioaddr->status_addr);		
}

static u8 ceata_check_altstatus(struct ata_port *ap)
{
	struct ceata_port *cp = (struct ceata_port *)ap->private_data;
	struct ata_ioports *ioaddr = &ap->ioaddr;
	
	if (!cp || !cp->mmc || !cp->mmc->card)
		return 0xff;

	return read_reg8(ap, ioaddr->altstatus_addr);		
}

static void ceata_exec_command(struct ata_port *ap, const struct ata_taskfile *tf)
{
	struct ceata_port *cp = (struct ceata_port *)ap->private_data;

	hd_access_enter(cp);
	ceata_rw_multiple_register(cp->mmc->card, 1, 0, cp->tf_buf, 16);
	hd_access_leave(cp);
}

static void ceata_dev_select (struct ata_port *ap, unsigned int device)
{
	struct ceata_port *cp = (struct ceata_port *)ap->private_data;

	if (!cp || !cp->mmc || !cp->mmc->card)
		return;
	
	if (device != 0)
		printk("CE_ATA_DEV_SELECT: only device 0 supported\n"); 
}

static void ceata_handle_result(struct ceata_port *cp, struct ata_queued_cmd *qc, int err)
{
	struct ata_port *ap = qc->ap;
	struct ata_eh_info *ehi = &ap->eh_info;
	typeof(jiffies) timeout = jiffies + msecs_to_jiffies(CE_ATA_TIMEOUT_MS);
	u8 error;
	u8 status;

	qc->err_mask = 0;

	if (err != MMC_ERR_NONE) {
		
		status = ata_chk_status(ap);
		if (status & ATA_DRQ) {
			int i;
			u64 qc_lba, drive_lba;
			u32 qc_nsect;
			int unsafe;

			qc_lba = tf_lbanum(&qc->tf);
			qc_nsect = tf_nsect(&qc->tf);
			
			tf_read_lbanum(ap, &drive_lba);
			do {
				tf_read_lbanum(ap, &drive_lba);
				#if 0
				unsafe = 15-((u32)(drive_lba-qc_lba)&15) < 3;
				#else
				unsafe = drive_lba - qc_lba != qc_nsect;
				#endif
				cond_resched();
			} while ( unsafe );

			status = ata_chk_status(ap);
		}
		
		if (status & (ATA_BUSY|ATA_DRQ))
			ce_ata_stop_transmission(cp->mmc->card,
				qc->tf.flags & ATA_TFLAG_WRITE ? 1:0);

		/* must signal some error, anything that forces libata to
		 * discard the data is OK */
		qc->err_mask |= AC_ERR_DEV;
		ehi->flags |= ATA_EHI_NO_AUTOPSY|ATA_EHI_QUIET;
	}

	while (time_before(jiffies, timeout)) {

		status = ata_chk_status(ap);
		if (status & ATA_ERR) {
			error = ceata_check_error(ap);
			ata_port_printk(ap, KERN_DEBUG,
				"status %02x, error %02x\n", status, error);
			qc->err_mask |= ac_err_mask(status);
			break;
		}

		/* at the end of a data transfer, drive should have
		 * no ATA_BSY or ATA_DRY asserted */
		if ((status & (ATA_BUSY|ATA_DRQ)) == 0)
			break;

		// printk(KERN_DEBUG "ata status: %02x\n", status);
		cond_resched();
	}

	if (!time_before(jiffies, timeout))
		ata_port_printk(ap, KERN_DEBUG, "wait !BUSY time out\n");

}

#if 0
static int __rw_sg(struct mmc_card *card, int rw, struct ata_queued_cmd *qc)
{
	struct scatterlist *sg;
	int err = MMC_ERR_NONE;

	ata_for_each_sg(sg, qc) {
		u8 *buffer;
		unsigned int nsect, nbytes;

		nbytes = qc->nbytes > sg_dma_len(sg) ? sg_dma_len(sg) : qc->nbytes;
		nsect = nbytes / qc->sect_size;
		buffer = (u8*)(page_address(sg->page) + sg->offset);
		
		err = ceata_rw_multiple_block(card, rw, buffer, nsect);
		if (err != MMC_ERR_NONE)
			break;
	}

	return err;
}
#endif

static int ceata_rw_do(struct ata_queued_cmd *qc)
{
	struct ata_port *ap = qc->ap;
	struct ceata_port *cp = (struct ceata_port *)ap->private_data;
	int rw = (qc->tf.flags & ATA_TFLAG_WRITE) ? 1:0;
	int retry = 0;
	int err;

	hd_access_enter(cp);

	do {
		typeof(jiffies) timeout = jiffies + msecs_to_jiffies(CE_ATA_TIMEOUT_MS);
		
		/* write transfer file including command */
		xlat_tf(cp->tf_buf, &qc->tf);
		err = ceata_rw_multiple_register(cp->mmc->card, 1, 0, cp->tf_buf, 16);
		if (err != MMC_ERR_NONE) {
			ata_port_printk(ap, KERN_DEBUG,
				"ceata_rw_do: failed to write taskfile\n");
			qc->err_mask = AC_ERR_DEV;
			goto out_ok;
		}

		/* wait until the drive signals !BSY and DRQ */
		while (time_before(jiffies, timeout)) {

			u8 status = ata_chk_status(ap);
			if (status & ATA_ERR) {
				u8 error = ceata_check_error(ap);
				ata_port_printk(ap, KERN_DEBUG,
					"status: %02x, error: %02x\n", status, error);
				qc->err_mask = ac_err_mask(status);
				goto out_ok;
			}

			if ( (status & ATA_BUSY) == 0) {
				/* at the beginning of a data tranfer, drive should
				 * not assert BUSY and should assert DRQ */
				if (status & ATA_DRQ)
					break;
			}

			//printk(KERN_DEBUG "ata status: %02x\n", status);
			cond_resched();
		}

		if (!time_before(jiffies, timeout))
			ata_port_printk(ap, KERN_DEBUG, "wait DRQ timeout\n");

	#if 1
		err = ceata_rw_sg(cp->mmc->card, rw, qc);
	#else
		err = __rw_sg(cp->mmc->card, rw, qc);
	#endif

		if (likely(err == MMC_ERR_NONE))
			qc->curbytes = qc->nbytes;
		else {
			if (retry)
				msleep(retry*100);
			retry++;
			ata_port_printk(ap, KERN_DEBUG,
				"ceata_rw: err=%d retry no. %d\n", err, retry);
		}

		/* error handling */
		ceata_handle_result(cp, qc, err);

		if (err != MMC_ERR_NONE && retry >= 1)
			ceata_set_clock(cp->mmc->card, retry+1);

	} while (err != MMC_ERR_NONE && retry < 3);

 out_ok:
	if (retry)
		ceata_set_clock(cp->mmc->card, 1);
	hd_access_leave(cp);
	ata_qc_complete(qc);
	
	return 0;
}

static int ceata_rw_thread(void *_cp)
{
	struct ceata_port *cp = (struct ceata_port *)_cp;
	int ret = 0;
	wait_queue_t wait;
	unsigned long flags;
	struct rw_queue_entry *qe;

	set_task_freezable(current);
	
	for (;;) {
		init_waitqueue_entry(&wait, current);
		add_wait_queue(&cp->wait_queue, &wait);
		for (;;) {
			if (list_empty(&cp->q))
				try_to_freeze();
			
			set_task_state(current, TASK_INTERRUPTIBLE);
			spin_lock_irqsave(&cp->rw_lock, flags);
			if (!list_empty(&cp->q) || kthread_should_stop())
				break;
			spin_unlock_irqrestore(&cp->rw_lock, flags);
			schedule();
		}
		
		set_task_state(current, TASK_RUNNING);
		remove_wait_queue(&cp->wait_queue, &wait);
	
		if (kthread_should_stop()) {
			spin_unlock_irqrestore(&cp->rw_lock, flags);
			break;
		}
		
		qe = container_of(cp->q.next, struct rw_queue_entry, list);
		list_del_init(&qe->list);
		spin_unlock_irqrestore(&cp->rw_lock, flags);
	
		ceata_rw_do(qe->qc);
	} 

	return ret;
}

static inline struct rw_queue_entry *find_queue_entry(struct ata_queued_cmd *qc)
{
	int i;
	struct ata_port *ap = qc->ap;
	struct ceata_port *cp = (struct ceata_port *)ap->private_data;
	
	for (i=0; i<ATA_MAX_QUEUE; i++)
		if (qc == &ap->qcmd[i])
			return &cp->qe[i];
			
	printk("no qe entry found\n");
	return NULL;	
}

static int ceata_rw(struct ata_queued_cmd *qc)
{
	struct ata_port *ap = qc->ap;
	struct ceata_port *cp = (struct ceata_port *)ap->private_data;
	struct rw_queue_entry *qe;

	qe = find_queue_entry(qc);
	INIT_LIST_HEAD(&qe->list);
	qe->qc = qc;
	spin_lock(&cp->rw_lock);
	list_add_tail(&qe->list, &cp->q);
	spin_unlock(&cp->rw_lock);
	wake_up(&cp->wait_queue);

	return 0;
}

static unsigned int ceata_qc_issue_prot(struct ata_queued_cmd *qc)
{
	struct ata_port *ap = qc->ap;
	struct ceata_port *cp = (struct ceata_port *)ap->private_data;
	
	cp->err_flags = 0;

	switch (qc->tf.protocol) {
	case ATA_PROT_PIO:
	case ATA_PROT_DMA:
		if (ceata_rw(qc) != 0)
			goto err;
		break;

	case ATA_PROT_NODATA:
		if ( qc->tf.command == 0xe7 /* FLUSH_CACHE */
		  || qc->tf.command == 0xea /* FLUSH_CACHE_EXT */
		  || qc->tf.command == 0xe0 /* STANDBY */)
		{
			if (qc->tf.command == 0xe7) {
				/* translate FLUSH_CACHE */
				qc->tf.command = 0xea; /* FLUSH_CACHE_EXT; */
			}

			ata_port_printk(ap, KERN_DEBUG, "nondata cmd 0x%02x\n",
				qc->tf.command);

			ap->ops->tf_load(ap, &qc->tf);
			ap->ops->exec_command(ap, &qc->tf);

			while(1) {
				u8 status = ata_busy_wait(ap, ATA_BUSY, 10);
				if (!(status & ATA_BUSY)) {
					break;
				}
				msleep(2);
			}

			ap->hsm_task_state = HSM_ST_IDLE;

			/* complete taskfile transaction */
			ata_qc_complete(qc);

			break;
		} else {
			ata_port_printk(ap, KERN_DEBUG, "cmd 0x%02x not supported\n",
				qc->tf.command);
			ap->hsm_task_state = HSM_ST_IDLE;
			ata_qc_complete(qc);	
			break;
		}

	case ATA_PROT_ATAPI_DMA:
	case ATA_PROT_ATAPI:
	case ATA_PROT_ATAPI_NODATA:
	default:
		printk("cmd 0x%02x not supported\n", qc->tf.command);
		goto err;
	}

	return 0;

err:
	return AC_ERR_SYSTEM;
}

static void ceata_data_xfer(struct ata_device *adev, unsigned char *buf,
		   unsigned int buflen, int write_data)
{
	struct ata_port *ap = adev->ap;
	struct ceata_port *cp = (struct ceata_port *)ap->private_data;
	
	int err;
	int data_unit_count = buflen / 512;

	hd_access_enter(cp);
	
	err = ceata_rw_multiple_block(cp->mmc->card, write_data ? 1:0, buf, data_unit_count);
	if (err != MMC_ERR_NONE) {
		printk("ceata_data_xfer failed\n");
		
		if (err == MMC_ERR_BADCRC) {
			printk("status: %02x\n", ceata_check_status(ap));
		}
	}
	
	hd_access_leave(cp);
}

static int ceata_hardreset(struct ata_port *ap, unsigned int *class,
		       unsigned long deadline)
{
	return 0;
}

static int ceata_softreset(struct ata_port *ap, unsigned int *classes,
		      unsigned long deadline)
{
	struct ceata_port *cp = (struct ceata_port *)ap->private_data;
	u8 err;
	struct ata_ioports *ioaddr = &ap->ioaddr;
	u8 status;
	
	if (cp == NULL || cp->mmc == NULL || cp->mmc->card == NULL)
		return -ENODEV;

	ata_port_printk(ap, KERN_DEBUG, "status 0x%02x before softreset\n",
		ata_chk_status(ap));

	/* select device 0 again */
	ap->ops->dev_select(ap, 0);

	/* issue bus reset */
	if (ap->flags & ATA_FLAG_SRST) {
		int rc;

		/* software reset.  causes dev0 to be selected */
		write_reg8(ap, ap->ctl & CONTROL_MASK, ioaddr->ctl_addr);
		udelay(20);	/* FIXME: flush */
		write_reg8(ap, (ap->ctl | ATA_SRST) & CONTROL_MASK, ioaddr->ctl_addr);
		udelay(20);	/* FIXME: flush */
		write_reg8(ap, ap->ctl & CONTROL_MASK, ioaddr->ctl_addr);

		do {
			/* wait before checking status after softreset,
			 * some drives lock up hard if you're to quick.
			 * also wait between checks to give the poor drive
			 * software enough air to complete the reset
			 *
			 * NOTE: 20ms is not enough. Increased to 40ms.
			 */
			msleep(40);

			status = ata_chk_status(ap);
			if (status == 0xFF)
				return -ENODEV;
		} while ( (status & ATA_BUSY) && time_before(jiffies, deadline));

		if (status & ATA_BUSY) {
			ata_port_printk(ap, KERN_WARNING,
				"BUSY after softreset, status 0x%02x\n", ata_chk_status(ap));
			return -ENODEV;
		}
	}

	/* determine by signature whether we have ATA or ATAPI devices */
	classes[0] = ceata_dev_try_classify(ap, 0, &err);
	classes[1] = ATA_DEV_NONE;

	return 0;
}

static int ceata_prereset(struct ata_port *ap, unsigned long deadline)
{
	struct ceata_port *cp = (struct ceata_port *)ap->private_data;
	struct ata_eh_context *ehc = &ap->eh_context;
	int rc;

	if (cp == NULL || cp->mmc == NULL || cp->mmc->card == NULL)
		return -ENOENT;
	
	/* handle link resume */
	if ((ehc->i.flags & ATA_EHI_RESUME_LINK) &&
	    (ap->flags & ATA_FLAG_HRST_TO_RESUME))
		ehc->i.action |= ATA_EH_HARDRESET;

	/* if we're about to do hardreset, nothing more to do */
	if (ehc->i.action & ATA_EH_HARDRESET)
		return 0;

	/* Wait for !BSY if the controller can wait for the first D2H
	 * Reg FIS and we don't know that no device is attached.
	 */

	if (!(ap->flags & ATA_FLAG_SKIP_D2H_BSY) && !ata_port_offline(ap)) {
		rc = ata_wait_ready(ap, deadline);
		if (rc && rc != -ENODEV) {
			ata_port_printk(ap, KERN_WARNING, "device not ready "
					"(errno=%d), forcing hardreset\n", rc);
			ehc->i.action |= ATA_EH_HARDRESET;
		}
	}

	return 0;
}

static void ceata_postreset(struct ata_port *ap, unsigned int *classes)
{
	/* bail out if no device is present */
	if (classes[0] == ATA_DEV_NONE) {
		DPRINTK("EXIT, no device\n");
		return;
	}

	/* set up device control */
	if (ap->ioaddr.ctl_addr)
		write_reg8(ap,  ap->ctl & CONTROL_MASK, ap->ioaddr.ctl_addr);
	
}

static void ceata_error_handler(struct ata_port *ap)
{
	struct ceata_port *cp = (struct ceata_port *)ap->private_data;
	struct ata_queued_cmd *qc;
	unsigned long flags;

	qc = __ata_qc_from_tag(ap, ap->active_tag);
	if (qc && !(qc->flags & ATA_QCFLAG_FAILED))
		qc = NULL;

	if (cp && cp->mmc && cp->mmc->card != NULL) {
		spin_lock_irqsave(ap->lock, flags);

		ap->hsm_task_state = HSM_ST_IDLE;
		ata_altstatus(ap);
		ata_chk_status(ap);
		ap->ops->irq_clear(ap);

		spin_unlock_irqrestore(ap->lock, flags);
	}

	/* PIO and DMA engines have been stopped, perform recovery */
	ata_do_eh(ap, ceata_prereset, ceata_softreset, ceata_hardreset, ceata_postreset);
}

static void ceata_bmdma_post_internal_cmd(struct ata_queued_cmd *qc)
{
}

static int ceata_cable_unknown(struct ata_port *ap)
{
	return ATA_CBL_PATA_UNK;
}

static void ceata_bmdma_irq_clear(struct ata_port *ap)
{
}

static int ceata_port_start(struct ata_port *ap)
{
	int ret;

	ret = ata_port_start(ap);
	if (ret < 0)
		return ret;
		
	return 0;
}

static int ceata_slave_config(struct scsi_device *sdev)
{
	int ret;
	
	ret = ata_scsi_slave_config(sdev);
	//sdev->manage_start_stop = 0;
	blk_queue_max_hw_segments(sdev->request_queue, 16);

	return ret;
}

static struct scsi_host_template ceata_sht = {
	.module			= THIS_MODULE,
	.name			= DRV_NAME,
	.ioctl			= ata_scsi_ioctl,
	.queuecommand		= ata_scsi_queuecmd,
	.can_queue		= ATA_DEF_QUEUE,
	.this_id		= ATA_SHT_THIS_ID,
	.sg_tablesize		= LIBATA_MAX_PRD,
	.cmd_per_lun		= ATA_SHT_CMD_PER_LUN,
	.emulated		= ATA_SHT_EMULATED,
	.use_clustering		= ATA_SHT_USE_CLUSTERING,
	.proc_name		= DRV_NAME,
	.dma_boundary		= ATA_DMA_BOUNDARY,
	.slave_configure	= ceata_slave_config,
	.slave_destroy		= ata_scsi_slave_destroy,
	.bios_param		= ata_std_bios_param,
};

static struct ata_port_operations ceata_port_ops = {
	.set_mode	= ceata_set_mode,

	.port_disable	= ceata_port_disable,
	.tf_load	= ceata_tf_load,
	.tf_read	= ceata_tf_read,
	.check_status 	= ceata_check_status,
	.check_altstatus = ceata_check_altstatus,
	.exec_command	= ceata_exec_command,
	.dev_select 	= ceata_dev_select,

	.data_xfer	= ceata_data_xfer,

	.error_handler	= ceata_error_handler,
	.post_internal_cmd = ceata_bmdma_post_internal_cmd,
	.cable_detect	= ceata_cable_unknown,

	.qc_prep 	= ata_noop_qc_prep,
	.qc_issue	= ceata_qc_issue_prot,

	.irq_handler	= NULL,
	.irq_clear	= ceata_bmdma_irq_clear,

	.port_start	= ceata_port_start,
};

static void ceata_ioports(struct ata_ioports *ioaddr)
{
	ioaddr->data_addr    = 0;
	ioaddr->error_addr   = (void __iomem*)CE_ATA_TASKFILE_ERROR;
	ioaddr->feature_addr = (void __iomem*)CE_ATA_TASKFILE_FEATURES;
	ioaddr->nsect_addr   = (void __iomem*)CE_ATA_TASKFILE_SECTOR_COUNT;
	ioaddr->lbal_addr    = (void __iomem*)CE_ATA_TASKFILE_LBA_LOW;
	ioaddr->lbam_addr    = (void __iomem*)CE_ATA_TASKFILE_LBA_MID;
	ioaddr->lbah_addr    = (void __iomem*)CE_ATA_TASKFILE_LBA_HIGH;
	ioaddr->device_addr  = (void __iomem*)CE_ATA_TASKFILE_DEVICE_HEAD;
	ioaddr->status_addr  = (void __iomem*)CE_ATA_TASKFILE_STATUS;
	ioaddr->command_addr = (void __iomem*)CE_ATA_TASKFILE_COMMAND;
//ioaddr->cmd_addr     = (void __iomem*)CE_ATA_TASKFILE_STATUS;
	ioaddr->ctl_addr     = (void __iomem*)CE_ATA_TASKFILE_CONTROL;
	ioaddr->altstatus_addr = (void __iomem*)CE_ATA_TASKFILE_STATUS;
}

static int ceata_init_one(struct ceata_host *ceata_host)
{
	struct ata_host *host = NULL;	
	int err = 0, i;
	
	static const struct ata_port_info info = {
		.sht = &ceata_sht,
		.flags = ATA_FLAG_NO_ATAPI | ATA_FLAG_SRST,
		.pio_mask = 0,
		.mwdma_mask = 0,
		.udma_mask = 0,
		.port_ops = &ceata_port_ops
	};
	const struct ata_port_info *ppi[] = { &info, NULL };

	ceata_host->dev->coherent_dma_mask = DMA_32BIT_MASK;
	
	ceata_host->n_ports = 1;
	
	/* alloc and init host */
	host = ata_host_alloc_pinfo(ceata_host->dev, ppi, ceata_host->n_ports);
	if (!host) {
		dev_printk(KERN_ERR, ceata_host->dev,
			   "failed to allocate ATA host\n");
		err = -ENOMEM;
		goto err_out;
	}

	for (i = 0; i < host->n_ports; i++) {
		ceata_ioports(&host->ports[i]->ioaddr);
		host->ports[i]->private_data = &ceata_host->ports[i];
	}

	ceata_host->ata = host;
	
	/* start host */
	err = ata_host_start(host);
	if (err)
		goto err_out;

	/* register */
	err = ata_host_register(host, ppi[0]->sht);
	if (err)
		goto err_out;

	return 0;

err_out:
	printk("ce_ata init failed\n");
	return err;
}

static int ceata_attach_port(struct ceata_port *cp)
{
	struct ata_port *ap = cp->ap;
	unsigned long flags;
	struct ata_eh_info *ehi = &ap->eh_info;
	
	ata_port_probe(ap);

	spin_lock_irqsave(ap->lock, flags);

	ehi->probe_mask = (1 << 1) - 1;
	ehi->action |= ATA_EH_SOFTRESET;
	ehi->flags |= ATA_EHI_NO_AUTOPSY | ATA_EHI_QUIET;

	ap->pflags &= ~ATA_PFLAG_INITIALIZING;
	//ap->pflags |= ATA_PFLAG_LOADING;
	ata_port_schedule_eh(ap);

	spin_unlock_irqrestore(ap->lock, flags);

	/* wait for EH to finish */
	ata_port_wait_eh(ap);
		
	return 0;
}

int ceata_add_port(struct ceata_host *chost, struct mmc_host *mmc, int init) 
{
	int i;
	int err;
		
	for (i=0; i<chost->n_ports; i++) {
		struct ceata_port *cp = &chost->ports[i];
		if (cp != NULL) {
			if (cp->mmc != NULL && cp->mmc != mmc)
				continue;
				
			cp->mmc = mmc;	
			cp->ap = chost->ata->ports[i];
			cp->ap->private_data = (void*)cp;
			cp->internal_data = chost->internal_data;

			if (init) {
				cp->tf_buf = kmalloc(16, GFP_KERNEL | GFP_DMA);
				if (cp->tf_buf == NULL)
					return -ENOMEM;

				init_waitqueue_head(&cp->wait_queue);
				cp->rw_thread = kthread_run(ceata_rw_thread, cp, "ceata_rw");
				if (IS_ERR(cp->rw_thread)) {
					err = PTR_ERR(cp->rw_thread);
					kfree(cp->tf_buf);
					return err;
				}

				spin_lock_init(&cp->rw_lock);
				INIT_LIST_HEAD(&cp->q);
	
				if ((err = ceata_attach_port(cp)) != 0) {
					cp->mmc = NULL;
					cp->ap = NULL;
					
					kthread_stop(cp->rw_thread);
					kfree(cp->tf_buf);
					return err;
				}			
			}
			return 0;
		}
	}

	return 0;
}

EXPORT_SYMBOL(ceata_add_port);

void ceata_del_port(struct ceata_host *chost, struct mmc_host *mmc, int release) 
{
	int i;
	struct ceata_port *cp;
		
	for (i=0; i<chost->n_ports; i++) {
		cp = &chost->ports[i];
		if (cp != NULL) {
			if (cp->mmc != NULL || cp->mmc != mmc)
				continue;
				
			if (release) {
				ata_port_detach(cp->ap);
				kthread_stop(cp->rw_thread);
				kfree(cp->tf_buf);
			}
			
			cp->mmc = NULL;	
			cp->ap = NULL;
			return;
		}
	}	
}

EXPORT_SYMBOL(ceata_del_port);

int ceata_register(struct ceata_host *host)
{
	int err;
	
	if ((err = ceata_init_one(host)) != 0)
		return err;

	return 0;
}

EXPORT_SYMBOL(ceata_register);

void ceata_unregister(struct ceata_host *host)
{
	ata_host_detach(host->ata);	
}

EXPORT_SYMBOL(ceata_unregister);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CE-ATA drive device driver");



