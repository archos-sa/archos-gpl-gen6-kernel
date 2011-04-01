/*
 * drivers/mmc/omap_hsmmc.c
 *
 * Driver for OMAP2430/3430 MMC controller.
 *
 * Copyright (C) 2006-2007 Texas Instruments, Inc
 * Author: Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/timer.h>

#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/ceata.h>
#include <linux/mmc/host.h>
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/dma.h>
#include <asm/mach-types.h>

#include <asm/hardware.h>
#include <asm/arch/board.h>
#include <asm/arch/cpu.h>
#include <asm/arch/clock.h>
#include <asm/semaphore.h>
#include "omap_hsmmc.h"

#ifdef CONFIG_PM
#include <linux/notifier.h>
#include <linux/pm.h>
#endif
#ifdef CONFIG_DPM
#include <linux/dpm.h>
#endif


#define mmc_clk_enable_aggressive(host)		/* NULL */
#define mmc_clk_disable_aggressive(host)	/* NULL */


struct mmc_omap_host *saved_host1, *saved_host2;

struct mmc_omap_host {
	int		suspended;
	struct		mmc_host *mmc;
	struct		mmc_request *mrq;
	struct		mmc_command *cmd;
	struct		mmc_data *data;
	struct		mmc_data *sdiodata;
	struct		timer_list detect_timer;
	struct		resource *mem_res;
	void		__iomem *base;
	void		*mapbase;
	struct		clk *fclk, *iclk, *dbclk;
	/* Required for a 3430 ES1.0 Sil errata fix */
	struct		clk *gptfck;
	unsigned int	id;
	int		irq;
	int		card_detect_irq;
	unsigned char	bus_mode;
	struct		semaphore sem;
	unsigned char	datadir;
	u32		*buffer;
	u32		bytesleft;
	int		use_dma, dma_ch;
	unsigned int	dma_len;
	unsigned int	dma_dir;
	struct		work_struct mmc_carddetect_work;
	int		initstream;
	int		tc;
	int		cc;
	int		ccs;
	int 		sdio_enable;
};

#ifdef CONFIG_OMAP34XX_OFFMODE
struct omap_hsmmc_regs {
	u32 hctl;
	u32 capa;
	u32 sysconfig;
	u32 ise;
	u32 ie;
	u32 con;
	u32 sysctl;
};
static struct omap_hsmmc_regs hsmmc_ctx[2];
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */


static int blkmode_bytecount[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
static void mmc_omap_polling_command(struct mmc_omap_host *host,
				struct mmc_command *cmd, u32 cmdreg);


static spinlock_t mmc_gpt_lock;
static int gptfclk_counter;

static int mmc_clk_counter [NO_OF_MMC_HOSTS];

#ifdef CONFIG_OMAP34XX_OFFMODE
static void omap2_hsmmc_save_ctx(struct mmc_omap_host *host)
{
	/* MMC : context save */
	hsmmc_ctx[host->id - 1].hctl = OMAP_HSMMC_READ(host->base, HCTL);
	hsmmc_ctx[host->id - 1].capa = OMAP_HSMMC_READ(host->base, CAPA);
	hsmmc_ctx[host->id - 1].sysconfig = OMAP_HSMMC_READ(host->base,
								SYSCONFIG);
	hsmmc_ctx[host->id - 1].ise = OMAP_HSMMC_READ(host->base, ISE);
	hsmmc_ctx[host->id - 1].ie = OMAP_HSMMC_READ(host->base, IE);
	hsmmc_ctx[host->id - 1].con = OMAP_HSMMC_READ(host->base, CON);
	hsmmc_ctx[host->id - 1].sysctl = OMAP_HSMMC_READ(host->base, SYSCTL);
}

static void omap2_hsmmc_restore_ctx(struct mmc_omap_host *host)
{
	/* MMC : context restore */
	OMAP_HSMMC_WRITE(host->base, HCTL, hsmmc_ctx[host->id - 1].hctl);
	OMAP_HSMMC_WRITE(host->base, CAPA, hsmmc_ctx[host->id - 1].capa);
	OMAP_HSMMC_WRITE(host->base, CON, hsmmc_ctx[host->id - 1].con);
	OMAP_HSMMC_WRITE(host->base, SYSCONFIG,
					hsmmc_ctx[host->id - 1].sysconfig);
	OMAP_HSMMC_WRITE(host->base, ISE, hsmmc_ctx[host->id - 1].ise);
	OMAP_HSMMC_WRITE(host->base, IE, hsmmc_ctx[host->id - 1].ie);
	OMAP_HSMMC_WRITE(host->base, SYSCTL, hsmmc_ctx[host->id - 1].sysctl);
	OMAP_HSMMC_WRITE(host->base, HCTL, OMAP_HSMMC_READ(host->base,
								HCTL) | SDBP);
}
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */

static int mmc_clk_enable (struct mmc_omap_host *host)
{
	int hostid = host->id - 1;


	if (!mmc_clk_counter[hostid]) {
		if (clk_enable(host->iclk) != 0)
			goto clk_en_err1;
		if (clk_enable(host->fclk) != 0)
			goto clk_en_err2;

#ifdef CONFIG_OMAP34XX_OFFMODE
		if (context_restore_required(host->fclk))
			omap2_hsmmc_restore_ctx(host);
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */
	}
	mmc_clk_counter[hostid] ++;
	return 0;

clk_en_err2:
	/* On fclk failure */
	clk_disable(host->iclk);

clk_en_err1:
	/* On iclk failure */

clk_en_err:
	dev_dbg(mmc_dev(host->mmc),
		"Unable to enable MMC clocks \n");
#if defined(CONFIG_MMC_OMAP3430) || defined(CONFIG_MMC_OMAP3430_MODULE) 
	spin_unlock(&mmc_gpt_lock);
#endif /* #if defined(CONFIG_MMC_OMAP3430) || defined(CONFIG_MMC_OMAP3430_MODULE) */
	return -1;
}

static int mmc_clk_disable (struct mmc_omap_host *host)
{
	int hostid = host->id - 1;
printk("mmc_clk_disable\n");

	mmc_clk_counter[hostid] --;
	if (!mmc_clk_counter[hostid]) {
#ifdef CONFIG_OMAP34XX_OFFMODE
		omap2_hsmmc_save_ctx(host);
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */
		clk_disable(host->fclk);
		clk_disable(host->iclk);
	}

	return 0;
}

/*
 * Stop clock to the card
 */
static void omap_mmc_stop_clock(struct mmc_omap_host *host)
{
	/* Stop clock to the card */
	OMAP_HSMMC_WRITE(host->base, SYSCTL,
			OMAP_HSMMC_READ(host->base, SYSCTL) & ~CEN);
	if ((OMAP_HSMMC_READ(host->base, SYSCTL) & CEN) != 0x0)
		dev_dbg(mmc_dev(host->mmc), "MMC clock not stoped,"
					"clock freq can not be altered\n");
}

/*
 * Send init stream sequence to the card before sending IDLE command
 */
static void send_init_stream(struct mmc_omap_host *host)
{
	int reg = 0, status;
	typeof(jiffies) timeout;

	OMAP_HSMMC_WRITE(host->base, ISE, INT_CLEAR);
	OMAP_HSMMC_WRITE(host->base, IE, INT_CLEAR);

	OMAP_HSMMC_WRITE(host->base, CON,
			OMAP_HSMMC_READ(host->base, CON) | INIT_STREAM);
	OMAP_HSMMC_WRITE(host->base, CMD, INIT_STREAM_CMD);

	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	while ((reg != CC) && time_before(jiffies, timeout)) {
		reg = OMAP_HSMMC_READ(host->base, STAT) & CC;
	}

	OMAP_HSMMC_WRITE(host->base, CON,
			OMAP_HSMMC_READ(host->base, CON) & ~INIT_STREAM);

	status = OMAP_HSMMC_READ(host->base, STAT);
	OMAP_HSMMC_WRITE(host->base, STAT, status);

}

/*
 * Configure the resptype, cmdtype and send the given command to the card
 */
static void
mmc_omap_start_command(struct mmc_omap_host *host, struct mmc_command *cmd)
{
	int  cmdreg = 0, resptype = 0, cmdtype = 0, status = 0;
	int func = 0, new_size = 0;

	printk(KERN_INFO "%s: CMD%d, argument 0x%08x\n",
			mmc_hostname(host->mmc), cmd->opcode, cmd->arg);

	host->cmd = cmd;

	if (cmd->opcode == SD_IO_RW_DIRECT) {
		if ((cmd->arg & IO_RW_DIRECT_MASK) == IO_RW_DIRECT_ARG_MASK)
			cmdtype = 0x2;
	}

	if (cmd->opcode == MMC_STOP_TRANSMISSION) {
		cmdtype = 0x3;
	}

	mmc_clk_enable_aggressive(host);

	/* Clear status bits and enable interrupts */
	OMAP_HSMMC_WRITE(host->base, STAT, OMAP_HSMMC_STAT_CLEAR);

	switch (RSP_TYPE(mmc_resp_type(cmd))) {
	case RSP_TYPE(MMC_RSP_R1):
		/* resp 1, resp 1b */
		if (MMC_RSP_R1 & MMC_RSP_BUSY)
			resptype = 3;
		else
			resptype = 2;
		break;
	case RSP_TYPE(MMC_RSP_R3):
			resptype = 2;
		break;
	case RSP_TYPE(MMC_RSP_R2):
		resptype = 1;
		break;
	default:
		break;
	}

	cmdreg = (cmd->opcode << 24) | (resptype << 16) | (cmdtype << 22);

	

	if (cmd->opcode == MMC_READ_SINGLE_BLOCK
		|| cmd->opcode == MMC_READ_MULTIPLE_BLOCK
		|| cmd->opcode == SD_APP_SEND_SCR
		|| (cmd->opcode == SD_SWITCH && cmd->arg == 0xfffff1)
		|| (cmd->opcode == SD_SWITCH && cmd->arg == 0x80fffff1)
		|| (cmd->opcode == MMC_SEND_EXT_CSD && cmd->arg == 0)) {

			cmdreg |= DP_SELECT | DDIR | MSBS | BCE;

	} else if (cmd->opcode == MMC_WRITE_BLOCK
		|| cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK) {
			cmdreg |= DP_SELECT | MSBS | BCE;
		cmdreg &= ~(DDIR);
	}
	
	if (cmd->polling) {
		mmc_omap_polling_command(host, cmd, cmdreg);
		return;
	}
	
	if (cmd->opcode == SD_IO_RW_DIRECT) {
		if (cmd->arg & OMAP_SDIO_READ) {
			cmdreg &= ~(DDIR);
			if ((cmd->arg & sdio_blkmode_mask) ==
					sdio_blkmode_regaddr1) {
				func = ((cmd->arg & sdio_rw_function_mask)
					>> 17);
				new_size = (cmd->arg & 0xFF);
				blkmode_bytecount[func] =
					blkmode_bytecount[func] & 0xFF00;
				blkmode_bytecount[func] =
					blkmode_bytecount[func] | new_size;
			} else if ((cmd->arg & sdio_blkmode_mask) ==
					sdio_blkmode_regaddr2) {
				func = ((cmd->arg & sdio_rw_function_mask)
					>> 17);
				new_size = ((cmd->arg & 0xFF) << 8);
				blkmode_bytecount[func] =
					blkmode_bytecount[func] & 0x00FF;
				blkmode_bytecount[func] =
					blkmode_bytecount[func] | new_size;
			} else
				cmdreg |= DDIR;
		}
	}
	if (cmd->opcode == MMC_GO_IDLE_STATE || cmd->opcode == MMC_SEND_OP_COND
		|| cmd->opcode == MMC_ALL_SEND_CID)
		OMAP_HSMMC_WRITE(host->base, CON,
				OMAP_HSMMC_READ(host->base, CON) | OD);

	if (cmd->opcode == MMC_GO_IDLE_STATE) {
		if (host->initstream == 0) {
			send_init_stream(host);
			host->initstream = 1;
		}
	}
	
	host->cc = 0;

	if (host->data || host->sdiodata)
		host->tc = 0;
	else
		host->tc = 1;

	
	OMAP_HSMMC_WRITE(host->base, ISE, INT_CLEAR);
	OMAP_HSMMC_WRITE(host->base, IE, INT_CLEAR);

	OMAP_HSMMC_WRITE(host->base, ARG, cmd->arg);
	OMAP_HSMMC_WRITE(host->base, CMD, cmdreg);
}


/*
 * Notify the xfer done on SDIO card to the core
 */
static void sdio_omap_xfer_done(struct mmc_omap_host *host, struct mmc_data *sdiodata)
{
	if (!sdiodata)
		return;

	host->data = NULL;
	host->sdiodata = NULL;
	host->datadir = OMAP_MMC_DATADIR_NONE;

	mmc_clk_disable_aggressive(host);

	if (!host->cmd) {
		host->mrq = NULL;
		mmc_request_done(host->mmc, sdiodata->mrq);
	}
	return;
}



/*
 * Notify the core about command completion
 */
static void mmc_omap_cmd_done(struct mmc_omap_host *host, struct mmc_command *cmd)
{
	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136) {
			/* response type 2 */
			cmd->resp[3] = OMAP_HSMMC_READ(host->base, RSP10);
			cmd->resp[2] = OMAP_HSMMC_READ(host->base, RSP32);
			cmd->resp[1] = OMAP_HSMMC_READ(host->base, RSP54);
			cmd->resp[0] = OMAP_HSMMC_READ(host->base, RSP76);
		} else {
			/* response types 1, 1b, 3, 4, 5, 6 */
			cmd->resp[0] = OMAP_HSMMC_READ(host->base, RSP10);
		}
	}
	if (host->mmc->mode == MMC_MODE_SDIO) {
		if (host->sdiodata == NULL || cmd->error != MMC_ERR_NONE) {
			dev_dbg(mmc_dev(host->mmc), "%s: End request, err %x\n",
				mmc_hostname(host->mmc), cmd->error);
			host->mrq = NULL;
			
			mmc_clk_disable_aggressive(host);
			
			mmc_request_done(host->mmc, cmd->mrq);
		}
	} else {
		if (host->data == NULL || cmd->error != MMC_ERR_NONE) {
			dev_dbg(mmc_dev(host->mmc), "%s: End request, err %x\n",
				mmc_hostname(host->mmc), cmd->error);
			host->mrq = NULL;

			mmc_clk_disable_aggressive(host);

			mmc_request_done(host->mmc, cmd->mrq);
		}
	}
}



/*
 * Functions for polling
 */
static void mmc_omap_polling_command(struct mmc_omap_host *host,
				     struct mmc_command *cmd, u32 cmdreg)
{
	int i, readCnt, bytec, status = 0;
	typeof(jiffies) timeout;
	
	if (cmd->opcode == SD_IO_RW_EXTENDED)
	{
		printk(KERN_INFO "mmc_omap_polling_command SD_IO_RW_EXTENDED\n");
	}

	if (cmd->arg & OMAP_SDIO_READ) {
		cmdreg |= DP_SELECT;
		cmdreg &= ~(DDIR);
	} else {
		cmdreg |= DP_SELECT | DDIR;
	}

	OMAP_HSMMC_WRITE(host->base, STAT, OMAP_HSMMC_STAT_CLEAR);
	OMAP_HSMMC_WRITE(host->base, ISE, INT_CLEAR);
	OMAP_HSMMC_WRITE(host->base, IE, INT_EN_MASK);

	OMAP_HSMMC_WRITE(host->base, ARG, cmd->arg);
	OMAP_HSMMC_WRITE(host->base, CMD, cmdreg);

	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	while (time_before(jiffies, timeout)) {
		status = OMAP_HSMMC_READ(host->base, STAT);
		if ((status & CC))
			break;
	}
	if (!(status & CC)) {
		dev_dbg(mmc_dev(host->mmc),
			"SDIO Command error CMD IO_RW_extd\n");
		host->cmd->error |= MMC_ERR_TIMEOUT;
		mmc_omap_cmd_done(host, host->cmd);
		return;
	}

	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	bytec = (host->cmd->arg & 0x1FF);
	readCnt = (bytec / 4);

	if (bytec % 4)
		readCnt++;

	if (host->cmd->arg & OMAP_SDIO_READ) {
		while (((OMAP_HSMMC_READ(host->base, PSTATE) 
				       & BWE) != BRW)
				       && time_before(jiffies, timeout)) ;

		for (i = 0; i < readCnt; i++)
			OMAP_HSMMC_WRITE(host->base, DATA, *host->buffer++);
	} else {
		while (((OMAP_HSMMC_READ(host->base, PSTATE) & BRE) != BRR)
				       && time_before(jiffies, timeout)) ;

		for (i = 0; i < readCnt; i++)
			*host->buffer++ = OMAP_HSMMC_READ(host->base, DATA);
	}

	status = 0;
	mmc_omap_cmd_done(host, host->cmd);
	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);

	while (time_before(jiffies, timeout)) {
		status = OMAP_HSMMC_READ(host->base, STAT);
		if (status & TC)
			break;
	}

	if (!(status & TC)) {
		dev_dbg(mmc_dev(host->mmc), "SDIO data sending error \n");
		host->data->error = MMC_ERR_TIMEOUT;
		return;
	}

	sdio_omap_xfer_done(host, host->sdiodata);
	return;
	
	
}

/*
 * Turn the socket power ON/OFF
 */
static int mmc_omap_power(struct mmc_omap_host *host, int on)
{
	int ret = 0;

	return ret;
}
/*
 * power switching module for mmc slot 1
 * power_mode=0 switches to 1.8V
 * power_mode=1 switches to 3V
 * Caller makes sure that it calls on  slot 1 with correct cpu revision
 */
static int omap_mmc_switch_opcond(struct mmc_omap_host *host, int power_mode)
{
	int ret = 0;

	mmc_clk_disable(host);

/* 3.0V not supported on Gen 6 */
	power_mode = 0;

printk("power mode: %d\n", power_mode);

	ret = mmc_omap_power(host,0);
	if (ret != 0)
		dev_dbg(mmc_dev(host->mmc),"Unable to disable power to MMC1\n");

	mmc_clk_enable(host);

	OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) & SDVSCLR);
	if (power_mode == 0) {
		OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) | SDVS18);
	} else {
		OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) | SDVS30);
		host->initstream = 0;
	}
	OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) | SDBP);

	return 0;
}

/*
 * Work Item to notify the core about card insertion/removal
 */




static ssize_t mmc_omap_detect_change(struct device *dev, struct device_attribute
				*attr, const char *buf, size_t count)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct mmc_omap_host *host = platform_get_drvdata(pdev);
	char cmd[25];
	int i = 0;

	while (buf[i] != ' ' && buf[i] != '\n' && i < count) {
		cmd[i] = buf[i];
		i++;
	}

	cmd[i] = '\0';
	i++;

	if (strcmp(cmd, "1")) {
		return count;	
	} 

	if (cpu_is_omap34xx()) {
		if (host->id == OMAP_MMC1_DEVID) {
			if (!(OMAP_HSMMC_READ(host->base, HCTL)
				& SDVSDET)) {
				if (omap_mmc_switch_opcond(host, 1) != 0)
					dev_dbg(mmc_dev(host->mmc),
					"mmc_omap_detect:switch"
					"command operation failed\n");
				host->mmc->ios.vdd =
					fls(host->mmc->ocr_avail) - 1;
			}
		}
	}
	
	mmc_detect_change(host->mmc, (HZ * 200) / 1000);
	
	return count;
}

static DEVICE_ATTR(detect_change, S_IWUSR, NULL, mmc_omap_detect_change);


/*
 * Routine to configure block leangth for MMC/SD/SDIO cards
 * and intiate the transfer.
 */
static int
mmc_omap_prepare_data(struct mmc_omap_host *host, struct mmc_request *req)
{
	int ret = 0;
	int byte_count = 0, func = 0;
	host->data = req->data;
	host->sdiodata = req->data;
	if (req->cmd->opcode == SD_IO_RW_EXTENDED) {
		if (req->cmd->arg & OMAP_SDIO_READ)
			host->datadir = OMAP_MMC_DATADIR_WRITE;
		else
			host->datadir = OMAP_MMC_DATADIR_READ;

		if ((req->cmd->arg & 0x1FF) == 0)
			byte_count = 0x200;
		else
			byte_count = req->cmd->arg & 0x1FF;

		func = ((req->cmd->arg & sdio_function_mask) >> 28);

		if (req->cmd->arg & SDIO_BLKMODE) {
			OMAP_HSMMC_WRITE(host->base, BLK,
					blkmode_bytecount[func]);
			OMAP_HSMMC_WRITE(host->base, BLK,
					OMAP_HSMMC_READ(host->base,
					BLK) | (byte_count << 16));
		} else {
			OMAP_HSMMC_WRITE(host->base, BLK, byte_count);
			OMAP_HSMMC_WRITE(host->base, BLK,
					OMAP_HSMMC_READ(host->base,BLK)
					| (1 << 16));
		}

		host->buffer = (u32 *) req->data->sdio_buffer_virt;

		return 0;
	}
	if (req->data == NULL) {
		host->datadir = OMAP_MMC_DATADIR_NONE;
		OMAP_HSMMC_WRITE(host->base, BLK, BLK_CLEAR);
		return 0;
	}

	OMAP_HSMMC_WRITE(host->base, BLK, (req->data->blksz));
	OMAP_HSMMC_WRITE(host->base, BLK,
					OMAP_HSMMC_READ(host->base,
					BLK) | (req->data->blocks << 16));
	host->datadir = (req->data->flags & MMC_DATA_WRITE) ?
			OMAP_MMC_DATADIR_WRITE : OMAP_MMC_DATADIR_READ;

	/* CPU copy */
	host->buffer =	(u32 *) (page_address(req->data->sg->page) + req->data->sg->offset);
	host->bytesleft = req->data->blocks * (req->data->blksz);
	host->dma_ch = -1;
	
	return 0;
}

/*
 * Request function. Exposed API to core for read/write operation
 */
static void omap_mmc_request(struct mmc_host *mmc, struct mmc_request *req)
{
	struct mmc_omap_host *host = mmc_priv(mmc);

	WARN_ON(host->mrq != NULL);
	host->mrq = req;

	mmc_clk_enable_aggressive(host);

	/* Reset MMC Controller's Data FSM */
	if (req->cmd->opcode == MMC_GO_IDLE_STATE) {
		OMAP_HSMMC_WRITE(host->base, SYSCTL,
				OMAP_HSMMC_READ(host->base, SYSCTL) | (1 << 25));
		while (OMAP_HSMMC_READ(host->base, SYSCTL) & (1 << 25)) ;

		OMAP_HSMMC_WRITE(host->base, SYSCTL,
				OMAP_HSMMC_READ(host->base, SYSCTL) | (1 << 26));
		while (OMAP_HSMMC_READ(host->base, SYSCTL) & (1 << 26)) ;
	}

	if ((req->cmd->opcode == SD_APP_SEND_SCR
		|| (req->cmd->opcode == MMC_SEND_EXT_CSD)) )
		mmc->ios.bus_width = MMC_BUS_WIDTH_1;

	if (mmc_omap_prepare_data(host, req))
		dev_dbg(mmc_dev(host->mmc),
			"MMC host %s failed to initiate data transfer\n",
			mmc_hostname(host->mmc));

	mmc_clk_disable_aggressive(host);

	if(req->cmd->opcode == SD_IO_RW_EXTENDED){
		printk(KERN_INFO "omap_mmc_request SD_IO_RW_EXTENDED\n");
	}
	if(req->cmd->opcode == SD_IO_RW_DIRECT){
		printk(KERN_INFO "omap_mmc_request SD_IO_RW_DIRECT\n");
	}
	if(req->cmd->opcode == SD_IO_SEND_OP_COND){
		printk(KERN_INFO "omap_mmc_request SD_IO_SEND_OP_COND\n");
	}
	mmc_omap_start_command(host, req->cmd);

}


/*
 * Routine to configure clock values. Exposed API to core
 */
static void omap_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct mmc_omap_host *host = mmc_priv(mmc);
	u16 dsor = 0;
	unsigned long regVal;
	typeof(jiffies) timeout;
	int *addr;

	printk(KERN_INFO "%s: set_ios: clock %dHz busmode %d"
			"powermode %d Vdd %x Bus Width %d\n",
			mmc_hostname(host->mmc), ios->clock, ios->bus_mode,
			ios->power_mode, ios->vdd, ios->bus_width);

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		host->initstream = 0;
#if defined(CONFIG_MMC_OMAP3430) || defined(CONFIG_MMC_OMAP3430_MODULE) 
		if (host->id == OMAP_MMC1_DEVID) {
			addr = (int *)&OMAP2_CONTROL_PBIAS_1;
			*addr &= ~(1 << 1);
			if (is_sil_rev_greater_than(OMAP3430_REV_ES1_0))
				*addr &= ~(1 << 9);
		}
#endif
		if (mmc_omap_power(host, 0))
			dev_dbg(mmc_dev(host->mmc),
				"Could not disable power to MMC%d\n",host->id);

		break;
	case MMC_POWER_UP:
		if (mmc_omap_power(host, 1))
			dev_dbg(mmc_dev(host->mmc),
				"Could not enable power to MMC%d\n",host->id);

#if defined(CONFIG_MMC_OMAP3430) || defined(CONFIG_MMC_OMAP3430_MODULE) 
		if (host->id == OMAP_MMC1_DEVID) {
			addr = (int *)&OMAP2_CONTROL_PBIAS_1;
			*addr |= (1 << 1);
			if (is_sil_rev_greater_than(OMAP3430_REV_ES1_0))
				*addr |= (1 << 9);
		}
#endif
		break;
	}

	mmc_clk_enable_aggressive(host);

	switch (mmc->ios.bus_width) {
	case MMC_BUS_WIDTH_8:
               	OMAP_HSMMC_WRITE(host->base, CON,
			OMAP_HSMMC_READ(host->base,CON)
			| EIGHT_BIT);
		break;
	case MMC_BUS_WIDTH_4:
		OMAP_HSMMC_WRITE(host->base, CON,
			OMAP_HSMMC_READ(host->base,CON) & ~EIGHT_BIT);
		OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base,HCTL)
			| FOUR_BIT);
		break;
	case MMC_BUS_WIDTH_1:
		OMAP_HSMMC_WRITE(host->base, CON,
			OMAP_HSMMC_READ(host->base,CON) & ~EIGHT_BIT);
		OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base,HCTL) & ~FOUR_BIT);
		break;
	}

	if (ios->clock) {
		/* Enable MMC_SD_CLK */
		dsor = OMAP_MMC_MASTER_CLOCK / ios->clock;
		if (dsor < 1)
			dsor = 1;

		if (OMAP_MMC_MASTER_CLOCK / dsor > ios->clock)
			dsor++;

		if (dsor > 250)
			dsor = 250;
	}

if (dsor)
	omap_mmc_stop_clock(host);
	regVal = OMAP_HSMMC_READ(host->base, SYSCTL);
	regVal = regVal & ~(CLKD_MASK);
	regVal = regVal | (dsor << 6);
	regVal = regVal | (DTO << 16);
	OMAP_HSMMC_WRITE(host->base, SYSCTL, regVal);
	OMAP_HSMMC_WRITE(host->base, SYSCTL,
			OMAP_HSMMC_READ(host->base, SYSCTL) | ICE);

	/* wait till the ICS bit is set */
	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	while ((OMAP_HSMMC_READ(host->base, SYSCTL) & ICS) != 0x2
		&& time_before(jiffies, timeout)) ;
	/* Enable clock to the card */
	OMAP_HSMMC_WRITE(host->base, SYSCTL,
			OMAP_HSMMC_READ(host->base, SYSCTL) | CEN);

	if (1/*host->mmc->mode == MMC_MODE_SDIO*/)
		OMAP_HSMMC_WRITE(host->base, CON,
			OMAP_HSMMC_READ(host->base, CON) | CLKEXTFREE);


	mmc_clk_disable_aggressive(host);
}


static struct mmc_host_ops mmc_omap_ops = {
	.request = omap_mmc_request,
	.set_ios = omap_mmc_set_ios,
	.enable_sdio_irq = NULL,
};

/*
 * Routine implementing the driver probe method
 */
static int __init omap_mmc_probe(struct platform_device *pdev)
{
	struct omap_mmc_conf *minfo = pdev->dev.platform_data;
	struct mmc_host *mmc;
	struct mmc_omap_host *host = NULL;
	struct resource *res;
	int ret = 0, *addr;
	unsigned long tmp;
	
	if (minfo == NULL) {
		dev_err(&pdev->dev, "platform data missing\n");
		return -ENXIO;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	
	if (res == NULL )
		return -ENXIO;

	res = request_mem_region(res->start, res->end - res->start + 1,
							pdev->dev.bus_id);
	if (res == NULL)
		return -EBUSY;

	mmc = mmc_alloc_host(sizeof(struct mmc_omap_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto mmc_alloc_err;
	}

	host = mmc_priv(mmc);
	host->mmc = mmc;

	sema_init(&host->sem, 1);

	host->use_dma = 0;
	host->dma_ch = -1;
	host->initstream = 0;
	host->mem_res = res;
	host->irq = NULL;

	host->id = pdev->id;
	host->mapbase = (void *)host->mem_res->start;
	host->base = (void __iomem *)IO_ADDRESS(host->mapbase);
	mmc->ops = &mmc_omap_ops;
	mmc->f_min = 400000;
	mmc->f_max = 52000000;

#ifdef CONFIG_MMC_BLOCK_BOUNCE
	mmc->max_phys_segs = 1;
	mmc->max_hw_segs = 1;
#else	
	mmc->max_phys_segs = 32;
	mmc->max_hw_segs = 32;
#endif
	mmc->max_blk_size = 2048;       /* BLEN is 11 bits (+1) */
	mmc->max_blk_count = 2048;      /* NBLK is 11 bits (+1) */
	mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;
	mmc->max_seg_size = mmc->max_req_size;

	if (cpu_is_omap34xx()) {

		host->fclk = clk_get(&pdev->dev, "mmchs_fck");
		if (IS_ERR(host->fclk)) {
			ret = PTR_ERR(host->fclk);
			host->fclk = NULL;
			goto clk_get_err;
		}

		host->iclk = clk_get(&pdev->dev, "mmchs_ick");
		if (IS_ERR(host->iclk)) {
			ret = PTR_ERR(host->iclk);
			clk_put(host->fclk);
			host->iclk = NULL;
			goto clk_get_err;
		}
	}

#ifdef CONFIG_OMAP34XX_OFFMODE
	modify_timeout_value(host->fclk, 500);
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */

	mmc_clk_enable(host);

	if (host->id == OMAP_MMC1_DEVID) {
		if (cpu_is_omap34xx()) {
#if defined(CONFIG_MMC_OMAP3430) || defined(CONFIG_MMC_OMAP3430_MODULE) 
			addr = (int *)&OMAP2_CONTROL_PBIAS_1;
			*addr |= (1 << 2);

			addr = (int *)&OMAP2_CONTROL_DEVCONF0;
			*addr |= (1 << 24);
#endif
			/* There is no 8-bit field in the structure yet */
			if (minfo->wire4) {
				mmc->caps = MMC_CAP_4_BIT_DATA;
			}

			OMAP_HSMMC_WRITE(host->base, HCTL,
					OMAP_HSMMC_READ(host->base,
					HCTL) | SDVS18);

		}
#if 0 /* GEN6 need omly 1.8V */
		mmc->ocr_avail = MMC_VDD_165_195;
#else
		mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_165_195;
#endif
		OMAP_HSMMC_WRITE(host->base, CAPA,OMAP_HSMMC_READ(host->base,
							CAPA) | VS30 | VS18);
	} else if (host->id == OMAP_MMC2_DEVID) {
		if (cpu_is_omap34xx()) {
#if defined(CONFIG_MMC_OMAP3430) || defined(CONFIG_MMC_OMAP3430_MODULE) 
			addr = (int *)&OMAP2_CONTROL_PBIAS_1;
			*addr |= (1 << 10);

			addr = (int *)&OMAP2_CONTROL_DEVCONF1;
			*addr |= (1 << 6);
#endif
			if (minfo->wire4)
				mmc->caps = MMC_CAP_4_BIT_DATA;
		}
		OMAP_HSMMC_WRITE(host->base, HCTL,
				OMAP_HSMMC_READ(host->base,
						HCTL) | SDVS18);

		/* Workaround for wlan chip sd8686 !!! 3.3V caps needed  */
		mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_165_195;

		OMAP_HSMMC_WRITE(host->base, CAPA,OMAP_HSMMC_READ(host->base, CAPA) | VS18);
	}

	mmc->caps |= MMC_CAP_MULTIWRITE | MMC_CAP_BYTEBLOCK | MMC_CAP_MMC_HIGHSPEED |
			MMC_CAP_SD_HIGHSPEED;

// #if 1 // AUTOIDLE
// 
// 	tmp = OMAP_HSMMC_READ(host->base, SYSCONFIG);
// 	
// 	tmp &= ~(AUTOIDLE | (3<<3));
// 	tmp |= (0<<8) | (2<<3) | (1<<2) | AUTOIDLE;
// 	
// 	OMAP_HSMMC_WRITE(host->base, SYSCONFIG, tmp);
// #endif

//printk("Highspeed: %s, (%08x)\n", (OMAP_HSMMC_READ(host->base, CAPA) >> 21) & 1 ? "yes" : "No", OMAP_HSMMC_READ(host->base, CAPA));
	/* Set SD bus power bit */
	OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) | SDBP | IWE);

	if (device_create_file(&pdev->dev, &dev_attr_detect_change) < 0) {
		dev_dbg(mmc_dev(host->mmc),
			"Unable to create sysfs"
			"attribute for mmc detect change\n");
	}



	if (host->id == OMAP_MMC1_DEVID)
		saved_host1 = host;
	else
		saved_host2 = host;

	platform_set_drvdata(pdev, host);

	mmc_clk_disable_aggressive(host);

	mmc_add_host(mmc);
	return 0;

clk_get_err:
	dev_dbg(mmc_dev(host->mmc),
		"Error getting clock for MMC\n");
	if (host) {
		mmc_free_host(mmc);
	}
	return ret;

mmc_alloc_err:
	if (host)
		mmc_free_host(mmc);
	return ret;

irq_err:
	mmc_clk_disable(host);

	clk_put(host->fclk);
	clk_put(host->iclk);

	if (host)
		mmc_free_host(mmc);
	return ret;
}

/*
 * Routine implementing the driver remove method
 */
static int omap_mmc_remove(struct platform_device *pdev)
{
	struct mmc_omap_host *host = platform_get_drvdata(pdev);
	struct resource *res;
	
	platform_set_drvdata(pdev, NULL);

	if (host) {

		flush_scheduled_work();

		/* Free the clks */
		clk_put(host->fclk);
		clk_put(host->iclk);

		device_remove_file(&pdev->dev,
					&dev_attr_detect_change);

		mmc_free_host(host->mmc);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, res->end - res->start + 1);

	return 0;
}

#define omap_mmc_suspend	NULL
#define omap_mmc_resume		NULL

#ifdef CONFIG_DPM
static int
omap_mmc_pre_scale(int slot, struct notifier_block *op, unsigned long level,
		   void *ptr)
{
	int i = 0, timeout = 20;
	struct mmc_omap_host *host = (slot == MMC1) ? saved_host1 : saved_host2;

	switch (level) {
	case SCALE_PRECHANGE:
		break;
	}
	return 0;
}

static int
omap_mmc_post_scale(int slot, struct notifier_block *op, unsigned long level,
		    void *ptr)
{
	struct mmc_omap_host *host = (slot == MMC1) ? saved_host1 : saved_host2;

	switch (level) {
	case SCALE_POSTCHANGE:
		if (host->dma_ch == -1) {
		/*
		 * Reset the stop at block gap event before re-starting the
		 * transmission
		 */
		OMAP_HSMMC_WRITE(host->base, HCTL,
				OMAP_HSMMC_READ(host->base,HCTL) & ~(SBGR));
		/* Restart the transmision from the previously left block */
		OMAP_HSMMC_WRITE(host->base, HCTL,
				OMAP_HSMMC_READ(host->base,HCTL) | CT);
		/* 1ms delay reuired after re-starting transfer */
		mdelay(1);
		}
		break;
	}
	return 0;
}

#if defined(CONFIG_OMAP3430_MMC1)
/*
 * Prescale function for MMC1 controller
 */
static int
omap_mmc1_scale_prechange(struct notifier_block *op, unsigned long level,
			  void *ptr)
{
	return omap_mmc_pre_scale(MMC1, op, level, ptr);
}

/*
 * Post scale  function for MMC1 controller
 */
static int
omap_mmc1_scale_postchange(struct notifier_block *op, unsigned long level,
			   void *ptr)
{
	return omap_mmc_post_scale(MMC1, op, level, ptr);
}

static struct notifier_block omap_mmc1_pre_scale = {
	.notifier_call = omap_mmc1_scale_prechange,
};

static struct notifier_block omap_mmc1_post_scale = {
	.notifier_call = omap_mmc1_scale_postchange,
};
#endif
#if defined(CONFIG_OMAP3430_MMC2)
/*
 * Prescale function for MMC2 controller
 */
static int
omap_mmc2_scale_prechange(struct notifier_block *op, unsigned long level,
			  void *ptr)
{
	return omap_mmc_pre_scale(MMC2, op, level, ptr);
}

/*
 * Post scale  function for MMC2 controller
 */
static int
omap_mmc2_scale_postchange(struct notifier_block *op, unsigned long level,
			   void *ptr)
{
	return omap_mmc_post_scale(MMC2, op, level, ptr);
}

static struct notifier_block omap_mmc2_pre_scale = {
	.notifier_call = omap_mmc2_scale_prechange,
};

static struct notifier_block omap_mmc2_post_scale = {
	.notifier_call = omap_mmc2_scale_postchange,
};
#endif
#endif

static struct platform_driver omap_mmc_driver = {
	.probe = omap_mmc_probe,
	.remove = omap_mmc_remove,
	.suspend = omap_mmc_suspend,
	.resume = omap_mmc_resume,
	.driver = {
		   .name = "hsmmc-omap",
		   .owner = THIS_MODULE,
	},
};

/*
 * Driver init method
 */
static int __init omap_mmc_init(void)
{
	/* Register the MMC driver */
	if (platform_driver_register(&omap_mmc_driver)) {
		printk(KERN_ERR ":failed to register MMC driver\n");
		return -ENODEV;
	}
#ifdef CONFIG_DPM
#if defined(CONFIG_OMAP3430_MMC1)
	/* DPM scale registration for MMC1 controller */
	dpm_register_scale(&omap_mmc1_pre_scale, SCALE_PRECHANGE);
	dpm_register_scale(&omap_mmc1_post_scale, SCALE_POSTCHANGE);
#endif
#if defined(CONFIG_OMAP3430_MMC2)
	/* DPM scale registration for MMC2 controller */
	dpm_register_scale(&omap_mmc2_pre_scale, SCALE_PRECHANGE);
	dpm_register_scale(&omap_mmc2_post_scale, SCALE_POSTCHANGE);
#endif
#endif
	printk(KERN_INFO "omap_mmc_init done\n");
	return 0;
}

/*
 * Driver exit method
 */
static void __exit omap_mmc_cleanup(void)
{
	/* Unregister MMC driver */
	platform_driver_unregister(&omap_mmc_driver);

#ifdef CONFIG_DPM
#if defined(CONFIG_OMAP3430_MMC1)
	/* Unregister DPM scale functions for MMC1 controller */
	dpm_unregister_scale(&omap_mmc1_pre_scale, SCALE_PRECHANGE);
	dpm_unregister_scale(&omap_mmc1_post_scale, SCALE_POSTCHANGE);
#endif
#if defined(CONFIG_OMAP3430_MMC2)
	/* Unregister DPM scale functions for MMC2 controller */
	dpm_unregister_scale(&omap_mmc2_pre_scale, SCALE_PRECHANGE);
	dpm_unregister_scale(&omap_mmc2_post_scale, SCALE_POSTCHANGE);
#endif
#endif
}

module_init(omap_mmc_init);
module_exit(omap_mmc_cleanup);

MODULE_DESCRIPTION("OMAP 3430 Multimedia Card driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments");
