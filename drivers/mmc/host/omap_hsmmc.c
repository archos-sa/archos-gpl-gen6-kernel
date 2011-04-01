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
#include <linux/i2c.h>
#include <linux/scatterlist.h>

#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/ceata.h>
#include <linux/mmc/host.h>
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/dma.h>
#include <asm/mach-types.h>
#if defined(CONFIG_MACH_OMAP_3430SDP) || defined(CONFIG_MACH_OMAP3_EVM)
#include <asm/arch/twl4030.h>
#endif
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

/* TAG for Aggressive Power changes in MMC */
//#define AGGR_PM_CAP 1
#undef AGGR_PM_CAP
#ifdef AGGR_PM_CAP
#define mmc_clk_enable_aggressive(host)		mmc_clk_enable(host)
#define mmc_clk_disable_aggressive(host)	mmc_clk_disable(host)
#else
#define mmc_clk_enable_aggressive(host)		/* NULL */
#define mmc_clk_disable_aggressive(host)	/* NULL */
#endif


#ifdef AGGR_PM_CAP
/* SYSCONFIG bit values */
#define OMAP_MMC_SYSCONFIG_CLKACT_IOFF_FOFF	0x0
#define OMAP_MMC_SYSCONFIG_CLKACT_ION_FOFF	0x1
#define OMAP_MMC_SYSCONFIG_CLKACT_IOFF_FON	0x2
#define OMAP_MMC_SYSCONFIG_CLKACT_ION_FON	0x3

#define OMAP_MMC_SYSCONFIG_SIDLE_FORCEIDLE	0x0
#define OMAP_MMC_SYSCONFIG_SIDLE_NOIDLE		0x0
#define OMAP_MMC_SYSCONFIG_SIDLE_SMARTIDLE	0x2

#define OMAP_MMC_SYSCONFIG_ENAWAKEUP	0x1

#define OMAP_MMC_SYSCONFIG_AUTOIDLE	0x0

/* SYSCONFIG bit Masks */
#define OMAP_MMC_SYSCONFIG_CLKACT_SHIFT		0x8
#define OMAP_MMC_SYSCONFIG_SIDLE_SHIFT		0x3
#define OMAP_MMC_SYSCONFIG_ENAWAKEUP_SHIFT	0x2

#define OMAP_MMC_SYSCONFIG_LVL1	0x1
#define OMAP_MMC_SYSCONFIG_LVL2	0x2
#endif /* #ifdef AGGR_PM_CAP */

#ifdef DEBUG
#define DPRINTK printk
#else
#define DPRINTK(x...) do {} while (0)
#endif

#define CONFIG_OMAP_SDIO_INTERRUPT
#define CONFIG_OMAP_CLOCK_MAX 52000000

#if defined(CONFIG_MACH_OMAP_2430SDP) || defined(CONFIG_MACH_OMAP_3430SDP) || defined(CONFIG_MACH_OMAP3_EVM) || defined(CONFIG_MACH_ARCHOS_G6)
extern int enable_mmc_power(int slot);
extern int disable_mmc_power(int slot);
extern int mask_carddetect_int(int slot);
extern int unmask_carddetect_int(int slot);
extern int setup_mmc_carddetect_irq(int irq);
extern int switch_power_mode(int power_mode);

extern ssize_t mmc_omap_show_cover_switch(struct device *dev, struct
					device_attribute *attr, char *buf);
extern ssize_t set_mmc_carddetect(struct device *dev, struct device_attribute
				*attr, const char *buf, size_t count);
#endif
#if defined(CONFIG_MACH_OMAP_2430SDP) || defined(CONFIG_MACH_OMAP_3430SDP) || defined(CONFIG_MACH_OMAP3_EVM) 
DEVICE_ATTR(mmc_cover_switch, S_IRUGO, mmc_omap_show_cover_switch, NULL);
DEVICE_ATTR(mmc_card_detect, S_IWUSR, NULL, set_mmc_carddetect);
#endif

struct mmc_omap_host *saved_host1, *saved_host2, *saved_host3;

struct mmc_omap_host {
	int		suspended;
	struct		mmc_host *mmc;
	struct		mmc_request *mrq;
	struct		mmc_command *cmd;
	struct		mmc_data *data;
#ifdef CONFIG_OMAP_SDIO
	struct		mmc_data *sdiodata;
	spinlock_t 	sdio_reg_lock;
#endif
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
	unsigned int	sg_dma_len;
	unsigned int	dma_dir;
	int 		chain_id;
	struct 		omap_dma_channel_params params;
	u32 		chains_requested;/* Number of chains to be requested */
	u32 		extra_chain_reqd;/* if there is a need of last chaining*/
	u32 		no_of_chain_reqd;/*No of times callback called*/
	u32 		current_cb_cnt;
	int 		brs_received;
	int 		dma_done;
	int 		dma_is_read;
	spinlock_t 	dma_lock;
	unsigned int	sg_len;
	int 		sg_idx;
	u32 		buffer_bytes_left;
	u32 		total_bytes_left;	    
	struct		work_struct mmc_carddetect_work;
	int		initstream;
	int		tc;
	int		cc;
	int 		sdio_enable;
	int		last_sg_nbytes;

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
static struct omap_hsmmc_regs hsmmc_ctx[NO_OF_MMC_HOSTS];
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */

static int blkmode_bytecount[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
static void mmc_omap_polling_command(struct mmc_omap_host *host,
				struct mmc_command *cmd, u32 cmdreg);
static void mmc_omap_dma_done(struct mmc_omap_host *host, struct mmc_data *data);
static void mmc_omap_end_of_data(struct mmc_omap_host *host, struct mmc_data *data);
static void mmc_chain_dma(struct mmc_omap_host *host, struct mmc_data *data);
static int mmc_omap_get_dma_channel(struct mmc_omap_host *host, struct mmc_data *data);

#if defined(CONFIG_MMC_OMAP3430) || defined(CONFIG_MMC_OMAP3430_MODULE) 
static spinlock_t mmc_gpt_lock;
static int gptfclk_counter;
#endif /* #if defined(CONFIG_MMC_OMAP3430) || defined(CONFIG_MMC_OMAP3430_MODULE) */

static int mmc_clk_counter [NO_OF_MMC_HOSTS];

static int dma_chains = 16;
module_param(dma_chains, int, 0644);
MODULE_PARM_DESC(dma_chains, "number of DMA chains used (default 16)");

#ifdef AGGR_PM_CAP
static int omap_mmc_sysconfig (struct mmc_omap_host *host, int level)
{
	u32 sysconfig_val;

	switch (level) {
	case OMAP_MMC_SYSCONFIG_LVL1:
		/*
		 * All features of SYSCONFIG enabled
		 * Note: MMC has Wakeup capability enabled at reset.
		 * If this capability is required then care needs to be taken
		 * for other wakeup related register such as HCTL(IWE bit) and
		 * interrupts in IE and ISE registers
		 *
		 * Clock activity has only FCLK ON
		 *
		 * Enabling SmartIdle in ES1.0, pervents CORE from going to
		 * retention. Hence even Wakeup capability is disabled.
		 */
		sysconfig_val = (
					(OMAP_MMC_SYSCONFIG_CLKACT_IOFF_FON <<
					OMAP_MMC_SYSCONFIG_CLKACT_SHIFT) |
					(OMAP_MMC_SYSCONFIG_SIDLE_SMARTIDLE <<
					OMAP_MMC_SYSCONFIG_SIDLE_SHIFT) |
					OMAP_MMC_SYSCONFIG_AUTOIDLE
				);

		OMAP_HSMMC_WRITE(host->base, SYSCONFIG, sysconfig_val);
		break;

	case OMAP_MMC_SYSCONFIG_LVL2:
		/*
		 * Clock activity has ICLK and FCLK OFF
		 */
		sysconfig_val = (
					(OMAP_MMC_SYSCONFIG_CLKACT_IOFF_FOFF <<
					OMAP_MMC_SYSCONFIG_CLKACT_SHIFT) |
					OMAP_MMC_SYSCONFIG_AUTOIDLE
				);

		OMAP_HSMMC_WRITE(host->base, SYSCONFIG, sysconfig_val);
		break;
	}
	return 0;
}
#endif /* #ifdef AGGR_PM_CAP */

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

	printk("ID: %d\n", host->id);
	printk("HCTL:      %08x\n", hsmmc_ctx[host->id - 1].hctl);
	printk("CAPA:      %08x\n", hsmmc_ctx[host->id - 1].capa);
	printk("SYSCONFIG: %08x\n", hsmmc_ctx[host->id - 1].sysconfig);
	printk("ISE:       %08x\n", hsmmc_ctx[host->id - 1].ise);
	printk("IE:        %08x\n", hsmmc_ctx[host->id - 1].ie);
	printk("CON:       %08x\n", hsmmc_ctx[host->id - 1].con);
	printk("SYSCTL:    %08x\n", hsmmc_ctx[host->id - 1].sysctl);
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
	/* switch on bus power again */
	OMAP_HSMMC_WRITE(host->base, HCTL,
		OMAP_HSMMC_READ(host->base, HCTL) | SDBP);
}
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */

static int mmc_clk_enable (struct mmc_omap_host *host)
{
	int hostid = host->id - 1;

#if defined(CONFIG_MMC_OMAP3430) || defined(CONFIG_MMC_OMAP3430_MODULE) 
	/* 3430-ES1.0  Sil errata fix */
	if (is_sil_rev_less_than(OMAP3430_REV_ES2_0)) {
		spin_lock(&mmc_gpt_lock);
		if (!gptfclk_counter) {
			if (clk_enable(host->gptfck) != 0) {
				dev_dbg(mmc_dev(host->mmc),
					"Unable to enable gptfck clock \n");
				goto clk_en_err;
			}
		}
		gptfclk_counter ++;
		spin_unlock(&mmc_gpt_lock);
	}
#endif /* #ifdef CONFIG_MMC_OMAP3430 */

	if (!mmc_clk_counter[hostid]) {
		if (clk_enable(host->iclk) != 0)
			goto clk_en_err1;
		if (clk_enable(host->fclk) != 0)
			goto clk_en_err2;

#ifdef AGGR_PM_CAP
		omap_mmc_sysconfig (host, OMAP_MMC_SYSCONFIG_LVL1);
#endif /* #ifdef AGGR_PM_CAP */
//#ifdef CONFIG_OMAP34XX_OFFMODE
//		if (context_restore_required(host->fclk))
//			omap2_hsmmc_restore_ctx(host);
//#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */
	}
	mmc_clk_counter[hostid] ++;
	return 0;

clk_en_err2:
	/* On fclk failure */
	clk_disable(host->iclk);

clk_en_err1:
	/* On iclk failure */
#if defined(CONFIG_MMC_OMAP3430) || defined(CONFIG_MMC_OMAP3430_MODULE) 
	if (is_sil_rev_less_than(OMAP3430_REV_ES2_0)) {
		spin_lock(&mmc_gpt_lock);
		gptfclk_counter --;
		if (!gptfclk_counter)
			clk_disable(host->gptfck);
		spin_unlock(&mmc_gpt_lock);
	}
#endif /* #if defined(CONFIG_MMC_OMAP3430) || defined(CONFIG_MMC_OMAP3430_MODULE) 
 */

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

	mmc_clk_counter[hostid] --;
	if (!mmc_clk_counter[hostid]) {
//#ifdef CONFIG_OMAP34XX_OFFMODE
//		omap2_hsmmc_save_ctx(host);
//#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */
#ifdef AGGR_PM_CAP
		omap_mmc_sysconfig (host, OMAP_MMC_SYSCONFIG_LVL2);
#endif /* #ifdef AGGR_PM_CAP */
		clk_disable(host->fclk);
		clk_disable(host->iclk);
	}

#if defined(CONFIG_MMC_OMAP3430) || defined(CONFIG_MMC_OMAP3430_MODULE) 
	/* 3430-ES1.0  Sil errata fix */
	if (is_sil_rev_less_than(OMAP3430_REV_ES2_0)) {
		spin_lock(&mmc_gpt_lock);
		gptfclk_counter --;
		if (!gptfclk_counter)
			clk_disable(host->gptfck);
		spin_unlock(&mmc_gpt_lock);
	}
#endif /* #if defined(CONFIG_MMC_OMAP3430) || defined(CONFIG_MMC_OMAP3430_MODULE) */
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
	unsigned int flags;

	disable_irq(host->irq);
	spin_lock_irqsave( &host->sdio_reg_lock, flags );
#ifdef CONFIG_OMAP_SDIO_INTERRUPT
	OMAP_HSMMC_WRITE(host->base, ISE, INT_CLEAR | CIRQ);
#else
	OMAP_HSMMC_WRITE(host->base, ISE, INT_CLEAR );
#endif
	OMAP_HSMMC_WRITE(host->base, IE, INT_CLEAR);
	spin_unlock_irqrestore( &host->sdio_reg_lock, flags );

	OMAP_HSMMC_WRITE(host->base, CON,
			OMAP_HSMMC_READ(host->base, CON) | INIT_STREAM);
	OMAP_HSMMC_WRITE(host->base, CMD, INIT_STREAM_CMD);

	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	while (time_before(jiffies, timeout)) {
		reg = OMAP_HSMMC_READ(host->base, STAT) & CC;
		if (reg == CC)
			break;
		msleep(10);
	}

	OMAP_HSMMC_WRITE(host->base, CON,
			OMAP_HSMMC_READ(host->base, CON) & ~INIT_STREAM);

	status = OMAP_HSMMC_READ(host->base, STAT);
	OMAP_HSMMC_WRITE(host->base, STAT, status);

	enable_irq(host->irq);
}

/*
 * Configure the resptype, cmdtype and send the given command to the card
 */
static void
mmc_omap_start_command(struct mmc_omap_host *host, struct mmc_command *cmd)
{
	int  cmdreg = 0, resptype = 0, cmdtype = 0;
	unsigned int ie_mask = INT_EN_MASK;
	unsigned int ise_mask = INT_EN_MASK;

#ifdef CONFIG_OMAP_SDIO
	unsigned int flags;
#endif	/* ifdef CONFIG_OMAP_SDIO */

	dev_dbg(mmc_dev(host->mmc), "%s: CMD%d, argument 0x%08x\n",
			mmc_hostname(host->mmc), cmd->opcode, cmd->arg);

	host->cmd = cmd;

#ifdef CONFIG_OMAP_SDIO
	if (cmd->opcode == SD_IO_RW_DIRECT) {
		if ((cmd->arg & IO_RW_DIRECT_MASK) == IO_RW_DIRECT_ARG_MASK)
			cmdtype = 0x2;
	}
#endif	/* ifdef CONFIG_OMAP_SDIO */

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

	if (cmd->polling) {
		mmc_omap_polling_command(host, cmd, cmdreg);
		return;
	}

	if (cmd->opcode == MMC_READ_SINGLE_BLOCK
		|| cmd->opcode == MMC_READ_MULTIPLE_BLOCK
		|| cmd->opcode == SD_APP_SEND_SCR
		|| (cmd->opcode == SD_SWITCH && cmd->arg == 0xfffff1)
		|| (cmd->opcode == SD_SWITCH && cmd->arg == 0x80fffff1)
		|| (cmd->opcode == MMC_SEND_EXT_CSD && cmd->arg == 0)) {

		if (host->use_dma)
			cmdreg |= DP_SELECT | DDIR | MSBS | BCE | DMA_EN;
		else
			cmdreg |= DP_SELECT | DDIR | MSBS | BCE;

	} else if (cmd->opcode == MMC_WRITE_BLOCK
		|| cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK) {

		if (host->use_dma)
			cmdreg |= DP_SELECT | MSBS | BCE | DMA_EN;
		else
			cmdreg |= DP_SELECT | MSBS | BCE;

		cmdreg &= ~(DDIR);
	}

#ifdef CONFIG_OMAP_SDIO
	/* Handle I/O related command settings */
	if (cmd->opcode == SD_IO_RW_EXTENDED) {

		if (cmd->arg & OMAP_SDIO_READ) {
#ifdef  CONFIG_OMAP_SDIO_NON_DMA_MODE
			cmdreg |= DP_SELECT;
#else
			cmdreg |= DP_SELECT | DMA_EN | BCE | MSBS;
#endif
			cmdreg &= ~(DDIR);
		} else {
#ifdef  CONFIG_OMAP_SDIO_NON_DMA_MODE
			cmdreg |= DP_SELECT | DDIR;
#else
			cmdreg |= DP_SELECT | DDIR | DMA_EN | BCE | MSBS;
#endif
		}
	}

	if (cmd->opcode == SD_IO_RW_DIRECT) {
		if (cmd->arg & OMAP_SDIO_READ) {
			/* if there is a write access to the I/O block size in the function base registers (FBR)
			   at 0x00n10/0x00n11 in CIA (function 0), store the new value also in our struct.
			   See SDIO standard 6.10 
			   AK: What happens if these are written by SD_IO_RW_EXTENDED?!? */
			if ((cmd->arg & SDIO_BLKMODE_MASK) ==
					SDIO_BLKMODE_REGADDR1) {
				int func = ((cmd->arg & SDIO_RW_FUNCTION_MASK)
					>> 17);
				blkmode_bytecount[func] = 
					( blkmode_bytecount[func] & 0xFF00 ) | (cmd->arg & 0xFF);
				dev_dbg(mmc_dev(host->mmc), "blocksize[%d] now %x CMD(%d) ARG: %08x\n", func, blkmode_bytecount[func], cmd->opcode, cmd->arg );
			} else if ((cmd->arg & SDIO_BLKMODE_MASK) ==
					SDIO_BLKMODE_REGADDR2) {
				int func = ((cmd->arg & SDIO_RW_FUNCTION_MASK)
					>> 17);
				blkmode_bytecount[func] =
					( blkmode_bytecount[func] & 0x00FF ) | ((cmd->arg & 0xFF) << 8);
				dev_dbg(mmc_dev(host->mmc), "blocksize[%d] now %x CMD(%d) ARG: %08x\n", func, blkmode_bytecount[func], cmd->opcode, cmd->arg );
			}
		}
	}

#endif	/* ifdef CONFIG_OMAP_SDIO */

#ifdef CONFIG_OMAP_CE_ATA
	if (cmd->opcode == CE_ATA_RW_MULTIPLE_REGISTER) {
		if (cmd->arg & (1 << 31)) {
			cmdreg |= DP_SELECT | DMA_EN | BCE | MSBS;
			cmdreg &= ~(DDIR);
		} else {
			cmdreg |= DP_SELECT | DDIR | DMA_EN | BCE | MSBS;
		}
	}
	else if (cmd->opcode == CE_ATA_RW_MULTIPLE_BLOCK) {
		if (cmd->arg & (1 << 31)) {
			if (host->use_dma)
				cmdreg |= DP_SELECT | MSBS | BCE | DMA_EN;
			else
				cmdreg |= DP_SELECT | MSBS | BCE;

			cmdreg &= ~(DDIR);
		} else {
			if (host->use_dma)
				cmdreg |= DP_SELECT | DDIR | MSBS | BCE | DMA_EN;
			else
				cmdreg |= DP_SELECT | DDIR | MSBS | BCE;
		}
	}
#endif

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
	
	spin_lock_irqsave( &host->sdio_reg_lock, flags );

	host->cc = 0;

	if (host->data || host->sdiodata)
		host->tc = 0;
	else
		host->tc = 1;

	OMAP_HSMMC_WRITE(host->base, ISE, OMAP_HSMMC_READ(host->base, ISE) | ise_mask);
	OMAP_HSMMC_WRITE(host->base, IE, OMAP_HSMMC_READ(host->base, IE) | ie_mask);
	spin_unlock_irqrestore( &host->sdio_reg_lock, flags );

	OMAP_HSMMC_WRITE(host->base, ARG, cmd->arg);
	OMAP_HSMMC_WRITE(host->base, CMD, cmdreg);
}

static void mmc_omap_request_done(struct mmc_omap_host *host, struct mmc_request *mrq)
{
	if (host->data && host->data->stop) {
		mmc_omap_start_command(host, host->data->stop);
		return;
	}
	
	host->mrq = NULL;
	mmc_request_done(host->mmc, mrq);
}

#ifdef CONFIG_OMAP_SDIO
/*
 * Notify the xfer done on SDIO card to the core
 */
static void sdio_omap_xfer_done(struct mmc_omap_host *host, struct mmc_data *sdiodata, int polling_mode)
{
	if (!sdiodata)
		return;
#ifndef  CONFIG_OMAP_SDIO_NON_DMA_MODE
	if (!polling_mode)
		dma_unmap_sg(mmc_dev(host->mmc), host->mrq->data->sg,
			host->dma_len, host->dma_dir);
#endif
	host->datadir = OMAP_MMC_DATADIR_NONE;

	mmc_clk_disable_aggressive(host);

	return;
}

static void sdio_omap_dma_done(struct mmc_omap_host *host, struct mmc_data *data)
{
	unsigned long flags;
	int done;

	done = 0;
	spin_lock_irqsave(&host->dma_lock, flags);
	if (host->brs_received)
		done = 1;
	else
		host->dma_done = 1;
	if (done) {
		sdio_omap_xfer_done(host, data, 0);
		host->tc = 1;
	}
	spin_unlock_irqrestore(&host->dma_lock, flags);
	
	if (host->cc && host->tc ) {
		if ( host->mrq )
			mmc_omap_request_done(host, host->mrq);
		else {
//printk("host->mrq = NULL, done %d\n", done);
		}
	}
}

#endif	/* ifdef CONFIG_OMAP_SDIO */

/*
 * Notify the xfer done on MMC/SD cards to the core
 */
static void mmc_omap_xfer_done(struct mmc_omap_host *host, struct mmc_data *data)
{
	BUG_ON(!data);

	if (host->use_dma) {
		/* Un-map the memory required for DMA */
		dma_unmap_sg(mmc_dev(host->mmc), data->sg, host->sg_len,
			host->dma_dir);
	}

	/* Reset the variables as transfer is complete */
	host->data = NULL;
	host->sg_len = 0;
	host->sg_dma_len = 0;
    
	host->datadir = OMAP_MMC_DATADIR_NONE;

	if (data->error == MMC_ERR_NONE)
		data->bytes_xfered += data->blocks * (data->blksz);
	else
		data->bytes_xfered = 0;
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
}

/*
 * Dma cleaning in case of command errors
 */
static void mmc_dma_cleanup(struct mmc_omap_host *host)
{
	int dma_ch;

	if(host->mmc->mode == MMC_MODE_SDIO) {
		if (host->use_dma && host->dma_ch != -1) {
			omap_stop_dma(host->dma_ch);
			dma_unmap_sg(mmc_dev(host->mmc), host->data->sg, host->dma_len,
			   		host->dma_dir);
			dma_ch = host->dma_ch;
			host->dma_ch = -1;
			omap_free_dma(dma_ch);
			sdio_omap_dma_done(host, host->data);
			up(&host->sem);
		}
	} else {
		if (host->use_dma) {
			omap_stop_dma_chain_transfers(host->chain_id);
			dma_unmap_sg(mmc_dev(host->mmc), host->data->sg, host->sg_len,
					host->dma_dir);
			omap_free_dma_chain(host->chain_id);
			mmc_omap_dma_done(host, host->data);
			host->chain_id = -1;
		}
	}
	
#ifdef CONFIG_OMAP_SDIO
	host->sdiodata = NULL;
#endif
	host->datadir = OMAP_MMC_DATADIR_NONE;
}

#if defined(CONFIG_OMAP_SDIO) && defined(CONFIG_OMAP_SDIO_NON_DMA_MODE)
/*
 * Sdio non dma mode data transfer function
 */
static void sdio_non_dma_xfer(struct mmc_omap_host *host)
{
	int i, readCnt, bytec;
	typeof(jiffies) timeout;

	if (host->cmd) {
		timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);

		if (host->cmd->opcode == SD_IO_RW_EXTENDED) {
			bytec = (host->cmd->arg & 0x1FF);

			if (bytec == 0)
				bytec = 0x200;

			readCnt = (bytec / 4);

			if (bytec % 4)
				readCnt++;

			if (host->cmd->arg & OMAP_SDIO_READ) {
				while (((OMAP_HSMMC_READ(host->base, PSTATE)
					& BWE) != BWE)
					&& time_before(jiffies, timeout));

				for (i = 0; i < readCnt; i++)
					OMAP_HSMMC_WRITE(host->base, DATA,
							*host->buffer++);
			} else {
				while (((OMAP_HSMMC_READ(host->base, PSTATE)
					& BRE) != BRE)
					&& time_before(jiffies, timeout));

				for (i = 0; i < readCnt; i++)
					*host->buffer++ =
					OMAP_HSMMC_READ(host->base, DATA);
			}
		}
	}
}
#endif

static inline u32* mmc_omap_get_buf_from_sg(struct mmc_omap_host * host)
{
	return sg_virt(host->data->sg);
}


/* PIO only */
static void mmc_omap_sg_to_buf(struct mmc_omap_host *host)
{
	struct scatterlist *sg;

	sg = host->data->sg + host->sg_idx;
	host->buffer_bytes_left = sg->length;
	host->buffer = page_address(sg->page) + sg->offset;
	if (host->buffer_bytes_left > host->total_bytes_left)
		host->buffer_bytes_left = host->total_bytes_left;
}


/* PIO only */
static void mmc_omap_xfer_data(struct mmc_omap_host *host, int write)
{
	int n;

	if (host->buffer_bytes_left == 0) {
		host->sg_idx++;
		BUG_ON(host->sg_idx == host->sg_len);
		mmc_omap_sg_to_buf(host);
	}
	n = 64;
	if (n > host->buffer_bytes_left)
		n = host->buffer_bytes_left;
	host->buffer_bytes_left -= n;
	host->total_bytes_left -= n;
	host->data->bytes_xfered += n;

	if (write) {
		__raw_writesw(host->base + OMAP_HSMMC_DATA, host->buffer, n);
	} else {
		__raw_readsw(host->base + OMAP_HSMMC_DATA, host->buffer, n);
	}
}

/*
 * Notify the xfer done on MMC/SD cards to the core
 */
static void mmc_omap_err_done(struct mmc_omap_host *host, struct mmc_data *data)
{
	if (host->use_dma && host->chain_id != -1) {
		omap_stop_dma_chain_transfers(host->chain_id);
		dma_unmap_sg(mmc_dev(host->mmc), data->sg, host->sg_len,
				host->dma_dir);
		omap_free_dma_chain(host->chain_id);
		host->chain_id = -1;
	}

	host->datadir = OMAP_MMC_DATADIR_NONE;
	host->sg_dma_len = 0;
	host->data = NULL;	
	host->tc = 1;

	mmc_clk_disable_aggressive(host);

	mmc_omap_request_done(host, host->mrq);
}

/*
 * The MMC controller IRQ handler
 */
static irqreturn_t mmc_omap_irq(int irq, void *dev_id)
{
	struct mmc_omap_host *host = (struct mmc_omap_host *)dev_id;
	int end_command = 0, end_transfer = 0;
	int  status;
	typeof(jiffies) timeout;

	if (unlikely(host == NULL))
		return IRQ_HANDLED;

	status = OMAP_HSMMC_READ(host->base, STAT);
	
	dev_dbg(mmc_dev(host->mmc), "Status in IRQ %08x\n", status);

	if (status & CIRQ) {
		if (host->mmc->mode == MMC_MODE_SDIO) {
#ifdef CONFIG_OMAP_SDIO_INTERRUPT
			struct mmc_host *mmc_host = host->mmc;

			if ( mmc_host->card ) {
				struct sdio_func *func = mmc_host->card->sdio_func[0];
				if (!func) {
					printk(KERN_WARNING "%s: pending IRQ for "
						"non-existant function\n",
						mmc_card_id(mmc_host->card));
				} else if (func->irq_handler) {
					mmc_host->ops->enable_sdio_irq(mmc_host, 0); // disable interrupts...
					func->irq_handler(func);
				}
			}
#endif
			return IRQ_HANDLED;
		}
		status &= ~(CIRQ);		
	}
	
	if (host->cmd && host->cmd->polling ) {
		printk("%s: Polling and IRQ status=%x mode=%d\n", mmc_hostname(host->mmc), status, host->mmc->mode);
		return IRQ_HANDLED;
	}
		
	if (host->cc && host->tc) {
		/* diagnostic only: OMAP3 sometimes
		 * signals TC after DCRC */
		if (host->data == NULL && (status & TC))
			DPRINTK(KERN_DEBUG "%s: TC IRQ with no data, status %08x\n",
				mmc_card_id(host->mmc->card), status);

		if (status != 0)
			OMAP_HSMMC_WRITE(host->base, STAT, status);

		mmc_clk_disable_aggressive(host);
		return IRQ_HANDLED;
	}

	if (host->cmd) {
		if (host->cmd->opcode == MMC_SELECT_CARD
			|| host->cmd->opcode == MMC_SWITCH) {
			timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
			while (time_before(jiffies, timeout)) {
				if ((OMAP_HSMMC_READ(host->base, STAT)
					& CC) == CC)
					break;
			}
		}
	}

	if (unlikely(host->use_dma == 0)) {
		if (host->total_bytes_left != 0) {
			if ((status & OMAP_MMC_STAT_BRR) || (status & TC))
				mmc_omap_xfer_data(host, 0);
			if (status & OMAP_MMC_STAT_BWR)
				mmc_omap_xfer_data(host, 1);
		}
	}

	if (status & OMAP_HSMMC_ERR) {
		if ((status & OMAP_HSMMC_CMD_TIMEOUT) ||
			(status & OMAP_HSMMC_CMD_CRC))
		{
			if (host->cmd && !(status & CC)) {
				if (status & (OMAP_HSMMC_CMD_TIMEOUT)) {
					/*
					 * Timeouts are normal in case of
					 * MMC_SEND_STATUS
					 */
					if (host->cmd->opcode !=
						MMC_ALL_SEND_CID)
						dev_err(mmc_dev(host->mmc),
							"CMD Timeout CMD%d\n",
							host->cmd->opcode);
					host->cmd->error = MMC_ERR_TIMEOUT;

					if (host->data)
						mmc_dma_cleanup(host);
					end_command = 1;
				} else {
					dev_err(mmc_dev(host->mmc),
						"%s: Command CRC error CMD%d\n",
						mmc_hostname(host->mmc),
						host->cmd->opcode);
					
					host->cmd->error = MMC_ERR_BADCRC;
					if (host->data)
						mmc_dma_cleanup(host);
					end_command = 1;
				}
				#if 1
				/* command line reset */
				OMAP_HSMMC_WRITE(host->base, SYSCTL,
					OMAP_HSMMC_READ(host->base, SYSCTL) | SRC);
				do { udelay(20); }
				while (OMAP_HSMMC_READ(host->base, SYSCTL) & SRC);
				#endif
			}
		}

		if ((status & OMAP_HSMMC_DATA_TIMEOUT)
			|| (status & OMAP_HSMMC_DATA_CRC)
			|| (status & OMAP_HSMMC_DATA_ENDBITERR))
		{
			if (host->data) {
				if (status & OMAP_HSMMC_DATA_TIMEOUT) {
					dev_err(mmc_dev(host->mmc),
						"%s: Data timeout, status: %08x, CMD(%d) ARG: %08x\n",
						mmc_hostname(host->mmc), status, host->cmd->opcode,
						host->cmd->arg );

					host->data->error = MMC_ERR_TIMEOUT;
					end_transfer = 1;
				} else {
					dev_err(mmc_dev(host->mmc),
							"%s: Data CRC error,"
							" status: %08x,"
							" CMD(%d) ARG: %08x,"
							" bytes left %d\n",
							mmc_hostname(host->mmc),
							status,
							host->cmd->opcode, host->cmd->arg,
							host->bytesleft);

					host->data->error = MMC_ERR_BADCRC;
					end_transfer = 1;
				}
				#if 1
				/* reset host data bus */
				OMAP_HSMMC_WRITE(host->base, SYSCTL,
				OMAP_HSMMC_READ(host->base, SYSCTL) | SRD);
				do { udelay(20); }
				while (OMAP_HSMMC_READ(host->base, SYSCTL) & SRD);
				#endif
			} else if ( host->cmd ) {
				dev_err(mmc_dev(host->mmc),
					"%s:Data timeout, but no host->data  status: %08x, CMD(%d) ARG: %08x\n",
					mmc_hostname(host->mmc), status, host->cmd->opcode,
					host->cmd->arg );
				host->cmd->error = MMC_ERR_TIMEOUT;
			}
		}

		if (status & OMAP_HSMMC_CARD_ERR) {
			dev_dbg(mmc_dev(host->mmc),
				"MMC%d: Card status error (CMD%d)\n",
				host->id, host->cmd->opcode);
			if (host->cmd) {
				host->cmd->error = MMC_ERR_FAILED;
				end_command = 1;
			}
			if (host->data) {
				host->data->error = MMC_ERR_FAILED;
				end_transfer = 1;
			}
		}
	}

#if defined(CONFIG_OMAP_SDIO) && defined(CONFIG_OMAP_SDIO_NON_DMA_MODE)
	sdio_non_dma_xfer(host);
#endif /* ifdef CONFIG_OMAP_SDIO */

	OMAP_HSMMC_WRITE(host->base, STAT, status);

	if (end_command || (status & CC)) {
		host->cc = 1;
		mmc_omap_cmd_done(host, host->cmd);
	}

	if (host->mmc->mode == MMC_MODE_MMC
		|| host->mmc->mode == MMC_MODE_SD
#ifdef CONFIG_OMAP_CE_ATA
		|| host->mmc->mode == MMC_MODE_CE_ATA
#endif
		)
	{
		if (end_transfer) {
			// mmc cmd request done immediately
			mmc_omap_err_done(host, host->data);
			return IRQ_HANDLED;
		}
		else if (status & TC)
			mmc_omap_end_of_data(host, host->data);
	}
#ifdef CONFIG_OMAP_SDIO
	else if (host->mmc->mode == MMC_MODE_SDIO) {
		if (end_transfer) {
			host->tc = 1;
			mmc_dma_cleanup(host);
		} else if (!host->use_dma && (status & TC)) {
			host->tc = 1;
			sdio_omap_xfer_done(host, host->sdiodata, 0);
		} else if (status & TC) {
			unsigned long flags;
			int done = 0;
			spin_lock_irqsave(&host->dma_lock, flags);
			if (host->dma_done)
				done = 1;
			else
				host->brs_received = 1;
			if (done) {
				sdio_omap_xfer_done(host, host->sdiodata, 0);
				host->tc = 1;
			} else
				DPRINTK("tc before DMA complete\n");
			spin_unlock_irqrestore(&host->dma_lock, flags);
		}
	}
#endif	/* ifdef CONFIG_OMAP_SDIO */
	
	if (host->cc && host->tc) {
		if ( host->mrq )
			mmc_omap_request_done(host, host->mrq);
		else {
			//printk("host->mrq = NULL, brs_received %d mode %d\n",
			//	host->brs_received, host->mmc->mode);
		}
	}
	return IRQ_HANDLED;
}

/*
 * Functions for polling
 */
static void mmc_omap_polling_command(struct mmc_omap_host *host,
				     struct mmc_command *cmd, u32 cmdreg)
{
	int i, readCnt, bytec, status = 0;
	typeof(jiffies) timeout;
	unsigned int flags;

	if (cmd->opcode == SD_IO_RW_EXTENDED) {
		if (cmd->arg & OMAP_SDIO_READ) {
			cmdreg |= DP_SELECT;
			cmdreg &= ~(DDIR);
		} else {
			cmdreg |= DP_SELECT | DDIR;
		}
	}
#ifdef CONFIG_OMAP_CE_ATA
	else if (cmd->opcode == CE_ATA_RW_MULTIPLE_REGISTER) {
		if (host->datadir == OMAP_MMC_DATADIR_WRITE) {
			cmdreg |= DP_SELECT;
			cmdreg &= ~(DDIR);
		}
		else {
			cmdreg |= DP_SELECT | DDIR;
		}
	}
#endif
	else if (cmd->opcode == SD_IO_RW_DIRECT) {
		if (cmd->arg & OMAP_SDIO_READ) {
			/* if there is a write access to the I/O block size in the function base registers (FBR)
			   at 0x00n10/0x00n11 in CIA (function 0), store the new value also in our struct.
			   See SDIO standard 6.10 */
			if ((cmd->arg & SDIO_BLKMODE_MASK) ==
					SDIO_BLKMODE_REGADDR1) {
				int func = ((cmd->arg & SDIO_RW_FUNCTION_MASK)
					>> 17);
				blkmode_bytecount[func] = 
					( blkmode_bytecount[func] & 0xFF00 ) | (cmd->arg & 0xFF);
				dev_dbg(mmc_dev(host->mmc), "blocksize[%d] now %x CMD(%d) ARG: %08x\n", func, blkmode_bytecount[func], cmd->opcode, cmd->arg );
			} else if ((cmd->arg & SDIO_BLKMODE_MASK) ==
					SDIO_BLKMODE_REGADDR2) {
				int func = ((cmd->arg & SDIO_RW_FUNCTION_MASK)
					>> 17);
				blkmode_bytecount[func] =
					( blkmode_bytecount[func] & 0x00FF ) | ((cmd->arg & 0xFF) << 8);
				dev_dbg(mmc_dev(host->mmc), "blocksize[%d] now %x CMD(%d) ARG: %08x\n", func, blkmode_bytecount[func], cmd->opcode, cmd->arg );
			}
		}
	}
	else if (cmd->opcode != MMC_FAST_IO) {
		printk("polling mode of command %d not supported\n", cmd->opcode);
		host->mrq = NULL;
		return;
	}
		
	spin_lock_irqsave( &host->sdio_reg_lock, flags );
	OMAP_HSMMC_WRITE(host->base, STAT, (OMAP_HSMMC_STAT_CLEAR) & ~(CIRQ));
	OMAP_HSMMC_WRITE(host->base, ISE, INT_CLEAR | (CIRQ & OMAP_HSMMC_READ(host->base, ISE)));
	OMAP_HSMMC_WRITE(host->base, IE, INT_EN_MASK | (CIRQ & OMAP_HSMMC_READ(host->base, IE)));
	spin_unlock_irqrestore( &host->sdio_reg_lock, flags );

	OMAP_HSMMC_WRITE(host->base, ARG, cmd->arg);
	OMAP_HSMMC_WRITE(host->base, CMD, cmdreg);

	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	while (time_before(jiffies, timeout)) {
		status = OMAP_HSMMC_READ(host->base, STAT);
		if ((status & CC))
			break;
		cond_resched();
	}
	if (!(status & CC)) {
		dev_err(mmc_dev(host->mmc),
			"Command error CMD %d, %08x\n", cmd->opcode, status);
		host->cmd->error |= MMC_ERR_TIMEOUT;
		host->mrq = NULL;
		mmc_omap_cmd_done(host, host->cmd);
		return;
	}

	if (cmd->opcode == SD_IO_RW_EXTENDED) {
		timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
		bytec = (host->cmd->arg & 0x1FF);
		readCnt = (bytec / 4);

		if (bytec % 4)
			readCnt++;

		if (host->cmd->arg & OMAP_SDIO_READ) {
			while (((OMAP_HSMMC_READ(host->base, PSTATE) & BWE) != BWE)
				&& time_before(jiffies, timeout))
					;

			for (i = 0; i < readCnt; i++)
				OMAP_HSMMC_WRITE(host->base, DATA, *host->buffer++);
		} else {
			while (((OMAP_HSMMC_READ(host->base, PSTATE) & BRE) != BRE)
				&& time_before(jiffies, timeout))
					;

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
			cond_resched();
		}

		if (!(status & TC)) {
			dev_dbg(mmc_dev(host->mmc), "SDIO data sending error \n");
			host->data->error = MMC_ERR_TIMEOUT;
			host->mrq = NULL;
			return;
		}

		host->mrq = NULL;
		sdio_omap_xfer_done(host, host->sdiodata, 1);
		return;
	}
#ifdef CONFIG_OMAP_CE_ATA
	else if (cmd->opcode == CE_ATA_RW_MULTIPLE_REGISTER) {
		timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
		bytec = host->bytesleft;
		readCnt = (bytec / 4);

		if (bytec % 4)
			readCnt++;

		if (host->datadir == OMAP_MMC_DATADIR_WRITE) {
			while (((OMAP_HSMMC_READ(host->base, PSTATE)
				& BWE) != BWE)
				&& time_before(jiffies, timeout))
				cond_resched();

			for (i = 0; i < readCnt; i++)
				OMAP_HSMMC_WRITE(host->base, DATA, *host->buffer++);
		} else {
			while (((OMAP_HSMMC_READ(host->base, PSTATE) & BRE) != BRE)
				&& time_before(jiffies, timeout))
					cond_resched();

			for (i = 0; i < readCnt; i++) {
				*host->buffer++ = OMAP_HSMMC_READ(host->base, DATA);
			}
		}

		status = 0;
		mmc_omap_cmd_done(host, host->cmd);
		timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);

		while (time_before(jiffies, timeout)) {
			status = OMAP_HSMMC_READ(host->base, STAT);
			if (status & TC)
				break;
			cond_resched();
		}

		if (!(status & TC)) {
			dev_err(mmc_dev(host->mmc), "data sending error\n");
			host->data->error = MMC_ERR_TIMEOUT;
			host->mrq = NULL;
			return;
		}

		host->mrq = NULL;
		mmc_omap_xfer_done(host, host->data);
		return;
	}
#endif
	else {
		host->mrq = NULL;
		mmc_omap_cmd_done(host, host->cmd);
	}
}

/*
 * Turn the socket power ON/OFF
 */
static int mmc_omap_power(struct mmc_omap_host *host, int on)
{
	int ret = 0;
#if defined(CONFIG_MACH_OMAP_3430SDP) || defined(CONFIG_MACH_OMAP3_EVM) || defined(CONFIG_MACH_ARCHOS_G6)
	if (on) {
			ret = enable_mmc_power(host->id);
	} else {
			ret = disable_mmc_power(host->id);
	}
#endif

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
	typeof(jiffies) timeout;

	mmc_clk_disable(host);

/* on Gen 6 devices VDDS = 3.0V supported only */
#if !defined(CONFIG_MACH_OMAP_3430SDP) && !defined(CONFIG_MACH_OMAP3_EVM)
	if (!power_mode) {
		printk("power mode is not supported from Archos G6 devices\n");
		return 0;
	}
#endif

	ret = mmc_omap_power(host,0);
	if (ret != 0)
		dev_dbg(mmc_dev(host->mmc), "Unable to disable power to MMC1\n");

#if defined(CONFIG_MACH_OMAP_3430SDP) || defined(CONFIG_MACH_OMAP3_EVM) || defined(CONFIG_MACH_OMAP3_EVM) || defined(CONFIG_MACH_ARCHOS_G6)
	if (switch_power_mode(power_mode))
		dev_dbg(mmc_dev(host->mmc), "Unable to switch operating"
			"voltage to the card\n");
#endif
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

	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	while (time_before(jiffies, timeout)) {
		if (OMAP_HSMMC_READ(host->base, HCTL) & SDBP)
			break;
		cond_resched();
	}
	if ((OMAP_HSMMC_READ(host->base, HCTL) & SDBP) == 0)
		dev_err(mmc_dev(host->mmc),
			"Invalid bus power configuration\n");

	return 0;
}

/*
 * Work Item to notify the core about card insertion/removal
 */
#if defined(CONFIG_MACH_OMAP_3430SDP) || defined(CONFIG_MACH_OMAP3_EVM)
static void mmc_omap_detect(struct work_struct *work)
{
	struct mmc_omap_host *host = container_of(work, struct mmc_omap_host,
		mmc_carddetect_work);

	mmc_clk_enable_aggressive(host);

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

	mmc_clk_disable_aggressive(host);
}
#endif

/*
 * Interrupt service routine for handling card insertion and removal
 */
static irqreturn_t mmc_omap_irq_cd(int irq, void *dev_id)
{
	struct mmc_omap_host *host = (struct mmc_omap_host *)dev_id;

	if (machine_is_omap_2430sdp() || machine_is_omap_3430sdp() || machine_is_omap3_evm())
		schedule_work(&host->mmc_carddetect_work);
	else
		dev_dbg(mmc_dev(host->mmc), "Place to implement MMC hotplug"
			"implementation based on what the other"
			"board can support\n");

	return IRQ_HANDLED;
}

static ssize_t mmc_omap_card_states_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "scan eject\n");
}

static ssize_t mmc_omap_change_card_state(struct device *dev, struct device_attribute
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

	if (!strcmp(cmd, "scan")) {

		if (cpu_is_omap34xx()) {
			if (host->id == OMAP_MMC1_DEVID) {
				if (!(OMAP_HSMMC_READ(host->base, HCTL) & SDVSDET)) {
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
	if (!strcmp(cmd, "eject")) {

		mmc_eject_card(host->mmc, (HZ * 200) / 1000);

		return count;	
	} 
	
	return count;
}

static DEVICE_ATTR(change_card_state, S_IWUSR | S_IRUSR, mmc_omap_card_states_show, mmc_omap_change_card_state);

static int mmc_omap_get_xfer_len(struct mmc_omap_host *host, struct mmc_data *data, int idx)
{
	return idx == host->sg_len-1 ? host->last_sg_nbytes : sg_dma_len(&data->sg[idx]);
}

/*
 * DMA call back function
 * lch is chain id in case of chaining
 */
static void mmc_omap_dma_cb(int lch, u16 ch_status, void *data)
{
	struct mmc_omap_host *host = (struct mmc_omap_host *)data;
	int chainid = host->chain_id;
	int dma_ch;

	if(host->mmc->mode == MMC_MODE_SDIO) {
		/*
		 * Only report the error for the time being, until the error handl
		 * for these type of errors is supported from the core
		 */
		if (ch_status & (1 << 11))
			dev_dbg(mmc_dev(host->mmc), " %s :MISALIGNED_ADRS_ERR\n",
				mmc_hostname(host->mmc));

		if (host->dma_ch < 0) {
			dev_dbg(mmc_dev(host->mmc), "%s:"
				"DMA callback while DMA not enabled?\n",
				mmc_hostname(host->mmc));
			return;
		}

		dma_ch = host->dma_ch;
		host->dma_ch = -1;
		omap_free_dma(dma_ch);
		sdio_omap_dma_done(host, host->data);
		up(&host->sem);
		return;
	}
	
	/* If we are at the last transfer, Shut down the receiver */
	if (omap_dma_chain_status(chainid) == OMAP_DMA_CHAIN_INACTIVE) {
		if(host->no_of_chain_reqd > host->current_cb_cnt) {
			mmc_chain_dma(host, host->data);
			omap_dma_set_interrupt_ch(host->chain_id, OMAP_DMA_DROP_IRQ |
				OMAP2_DMA_MISALIGNED_ERR_IRQ |
				OMAP2_DMA_TRANS_ERR_IRQ,
				OMAP_DMA_DROP_IRQ |
				OMAP_DMA_BLOCK_IRQ |
				OMAP2_DMA_MISALIGNED_ERR_IRQ |
				OMAP2_DMA_TRANS_ERR_IRQ);
			omap_start_dma_chain_transfers(host->chain_id);
			host->current_cb_cnt++;
		}
		else if(host->no_of_chain_reqd == host->current_cb_cnt) {
			if(host->extra_chain_reqd == 0) {
				omap_stop_dma_chain_transfers(chainid);
				omap_free_dma_chain(chainid);
				mmc_omap_dma_done(host, host->data);
				host->chain_id = -1;
			} else {
				omap_stop_dma_chain_transfers(chainid);
				omap_free_dma_chain(chainid);
				host->chain_id = -1;
				host->chains_requested = host->extra_chain_reqd;
				mmc_omap_get_dma_channel(host, host->data);
				mmc_chain_dma(host, host->data);
				omap_start_dma_chain_transfers(host->chain_id);
				host->extra_chain_reqd = 0;
			}
		} else {
			dev_dbg(mmc_dev(host->mmc), "%s:"
			"DMA callback ERROR\n",
			mmc_hostname(host->mmc));
		}
	} else {
		dev_dbg(mmc_dev(host->mmc), "%s:"
		  "DMA callback Channel active?\n",
		  mmc_hostname(host->mmc));
	}
}

#ifdef CONFIG_OMAP_SDIO
#ifndef CONFIG_OMAP_SDIO_NON_DMA_MODE

/*
 * Configure dma src and destination parameters
 */
static int mmc_omap_config_dma_param(int sync_dir, struct mmc_omap_host *host,
				     struct mmc_data *data)
{
	if (sync_dir == OMAP_DMA_DST_SYNC) {
		omap_set_dma_dest_params(host->dma_ch,
			0,	// dest_port required only for OMAP1
			OMAP_DMA_AMODE_CONSTANT,
			(dma_addr_t) (host->mapbase + OMAP_HSMMC_DATA),0, 0);
		omap_set_dma_src_params(host->dma_ch,
			0,	// src_port required only for OMAP1
			OMAP_DMA_AMODE_POST_INC,
			sg_dma_address(&data-> sg[0]), 0, 0);
		omap_set_dma_src_burst_mode(host->dma_ch, OMAP_DMA_DATA_BURST_16);
	} else {
		omap_set_dma_src_params(host->dma_ch,
			0,	// src_port required only for OMAP1
			OMAP_DMA_AMODE_CONSTANT,
			(dma_addr_t) (host->mapbase + OMAP_HSMMC_DATA),0, 0);
		omap_set_dma_dest_params(host->dma_ch,
			0,	// dest_port required only for OMAP1
			OMAP_DMA_AMODE_POST_INC,
			sg_dma_address(&data->sg[0]), 0,0);
		omap_set_dma_dest_burst_mode(host->dma_ch, OMAP_DMA_DATA_BURST_16);
	}
	return 0;
}

/*
 * Routine to configure and start dma for SDIO card
 */
static int
sdio_omap_start_dma_transfer(struct mmc_omap_host *host,
			     struct mmc_request *req)
{
	int sync_dev, sync_dir, dma_ch, ret, readCnt, bytecount;
	int nob = 1, func = 0;
	struct mmc_data *data = req->data;

	/*
	 * If for some reason the DMA transfer is still active,
	 * we wait for timeout period and free the dma
	 */

	if (host->dma_ch != -1) {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(100);
		if (down_trylock(&host->sem)) {

			dma_ch = host->dma_ch;
			host->dma_ch = -1;
			omap_free_dma(dma_ch);
			up(&host->sem);
			return 1;
		}
	} else {
		if (down_trylock(&host->sem)) {
			dev_dbg(mmc_dev(host->mmc),
				"Semaphore was not initialized \n");
			BUG();
		}
	}

	if (req->cmd->opcode == SD_IO_RW_EXTENDED) {
		if (req->cmd->arg & OMAP_SDIO_READ) {
			sync_dir = OMAP_DMA_DST_SYNC;
			
			if (host->id == OMAP_MMC1_DEVID)
				sync_dev = OMAP24XX_DMA_MMC1_TX;
			else if (host->id == OMAP_MMC2_DEVID)
				sync_dev = OMAP24XX_DMA_MMC2_TX;
			else
				sync_dev = OMAP34XX_DMA_MMC3_TX;
		} else {
			sync_dir = OMAP_DMA_SRC_SYNC;
			
			if (host->id == OMAP_MMC1_DEVID)
				sync_dev = OMAP24XX_DMA_MMC1_RX;
			else if (host->id == OMAP_MMC2_DEVID)
				sync_dev = OMAP24XX_DMA_MMC2_RX;
			else
				sync_dev = OMAP34XX_DMA_MMC3_RX;
		}

		ret = omap_request_dma(sync_dev, "SDIO", mmc_omap_dma_cb,
						host,&dma_ch);
		if (ret != 0) {
			dev_dbg(mmc_dev(host->mmc),
				"%s: omap_request_dma() failed with %d\n",
				mmc_hostname(host->mmc), ret);
			return ret;
		}

		host->dma_len =  dma_map_sg(mmc_dev(host->mmc), data->sg,
						data->sg_len, host->dma_dir);
		host->dma_ch = dma_ch;

		mmc_omap_config_dma_param(sync_dir, host, data);

		if (req->cmd->arg & SDIO_BLKMODE) {
			nob = req->cmd->arg & 0x1FF;

			if (nob == 0)
				nob = 0x200;

			func = ((req->cmd->arg & SDIO_FUNCTION_MASK) >> 28);
			bytecount = blkmode_bytecount[func];
			readCnt = (bytecount / 4);

			if (bytecount % 4)
				readCnt++;

		} else {
			bytecount = req->cmd->arg & 0x1FF;

			if (bytecount == 0)
				bytecount = 0x200;

			readCnt = (bytecount / 4);

			if (bytecount % 4)
				readCnt++;
		}

		omap_set_dma_transfer_params(dma_ch,
				OMAP_DMA_DATA_TYPE_S32, readCnt,
				nob, OMAP_DMA_SYNC_FRAME, sync_dev, sync_dir);

		omap_start_dma(dma_ch);
	}
	return 0;
}
#endif
#endif	/* ifdef CONFIG_OMAP_SDIO */

static int mmc_omap_get_dma_channel(struct mmc_omap_host *host, struct mmc_data *data)
{
	int ret = 0;
	int dma_chid;
	u16 frame;
	u32 count;
	int sync_dev = 0;

	frame = data->blksz;/*blk size*/
	count = mmc_omap_get_xfer_len(host, data, host->sg_idx)/frame;/*No of blocks*/

	/*
	 * If for some reason the DMA transfer is still active,
	 * we wait for timeout period and free the dma
	 */
	 if(host->chain_id != -1)
		dev_dbg(mmc_dev(host->mmc), "%s: chain is not free\n",
			mmc_hostname(host->mmc));

	/*Common params*/
	host->params.data_type = OMAP_DMA_DATA_TYPE_S32;
	host->params.dst_ei = 0;
	host->params.dst_fi = 0;
	host->params.dst_port = 0;
	host->params.elem_count = (data->blksz / 4);
	host->params.read_prio = DMA_CH_PRIO_HIGH;
	host->params.src_ei = 0;
	host->params.src_fi = 0;
	host->params.src_port = 0;
	host->params.sync_mode = OMAP_DMA_SYNC_FRAME;
	host->params.write_prio = DMA_CH_PRIO_HIGH;

	if (!(data->flags & MMC_DATA_WRITE)) {
		host->dma_dir = DMA_FROM_DEVICE;
		if (host->id == OMAP_MMC1_DEVID)
			sync_dev = OMAP24XX_DMA_MMC1_RX;
		else if (host->id == OMAP_MMC2_DEVID)
			sync_dev = OMAP24XX_DMA_MMC2_RX;
		else
			sync_dev = OMAP34XX_DMA_MMC3_RX;

		host->params.dst_amode = OMAP_DMA_AMODE_POST_INC;
		host->params.dst_start = sg_dma_address(&data->sg[host->sg_idx]);

		host->params.frame_count = count;

		host->params.src_amode = OMAP_DMA_AMODE_CONSTANT;
		host->params.src_or_dst_synch = OMAP_DMA_SRC_SYNC;
		host->params.src_start = (dma_addr_t) (host->mapbase +OMAP_HSMMC_DATA);

		host->params.trigger = sync_dev;

		host->params.dst_burst_mode = OMAP_DMA_DATA_BURST_16;
	} else {
		host->dma_dir = DMA_TO_DEVICE;
		if (host->id == OMAP_MMC1_DEVID)
			sync_dev = OMAP24XX_DMA_MMC1_TX;
		else if (host->id == OMAP_MMC2_DEVID)
			sync_dev = OMAP24XX_DMA_MMC2_TX;
		else
			sync_dev = OMAP34XX_DMA_MMC3_TX;

		host->params.dst_amode = OMAP_DMA_AMODE_CONSTANT;
		host->params.dst_start = (dma_addr_t) (host->mapbase + OMAP_HSMMC_DATA);

		host->params.frame_count = count;

		host->params.src_amode = OMAP_DMA_AMODE_POST_INC;
		host->params.src_or_dst_synch = OMAP_DMA_DST_SYNC;
		host->params.src_start = sg_dma_address(&data->sg[host->sg_idx]);

		host->params.trigger = sync_dev;

		host->params.src_burst_mode = OMAP_DMA_DATA_BURST_16;
	}

	/* Request a DMA chain for transfer
	 * A chain is requested before each transfer to avoid
	 * locking of DMA resources
	 */
	ret = omap_request_dma_chain(sync_dev, "MMC/SD", mmc_omap_dma_cb,
			    &dma_chid, host->chains_requested,
			    OMAP_DMA_DYNAMIC_CHAIN, &host->params);	
	if (ret != 0) {
		dev_dbg(mmc_dev(host->mmc),
			"%s: omap_request_dma_chain() failed with %d\n",
			mmc_hostname(host->mmc), ret);
		return ret;
	}

	if (host->chains_requested > 1)
		omap_dma_set_interrupt_ch(dma_chid, OMAP_DMA_DROP_IRQ |
				OMAP2_DMA_MISALIGNED_ERR_IRQ |
				OMAP2_DMA_TRANS_ERR_IRQ,
				OMAP_DMA_DROP_IRQ |
				OMAP_DMA_BLOCK_IRQ |
				OMAP2_DMA_MISALIGNED_ERR_IRQ |
				OMAP2_DMA_TRANS_ERR_IRQ);
	
	host->chain_id = dma_chid;
	return 0;
}

static void mmc_chain_dma(struct mmc_omap_host *host, struct mmc_data *data)
{
	u16 frame;
	u32 count,i,dma_chain_status, sg_idx = host->sg_idx;
	struct scatterlist *sg;

	frame = data->blksz;
	for (i = host->sg_idx ;i < (host->chains_requested + sg_idx); i++) {
	
		sg = &data->sg[i];
		count = mmc_omap_get_xfer_len(host, data, host->sg_idx)/frame;
		host->sg_dma_len += (frame * count);

		if(!(data->flags & MMC_DATA_WRITE)) {
			dma_chain_status = omap_dma_chain_a_transfer(host->chain_id,
					(dma_addr_t)(host->mapbase + OMAP_HSMMC_DATA),
					sg_dma_address(&data->sg[i]), (data->blksz / 4),
					count, host);
			if (dma_chain_status != 0)
				dev_err(mmc_dev(host->mmc),
				  "%s: omap_dma_chain_a_transfer() failed during read with %d\n",
				  mmc_hostname(host->mmc), dma_chain_status);
		} else {
			dma_chain_status = omap_dma_chain_a_transfer(host->chain_id,
					sg_dma_address(&data->sg[i]),
					(dma_addr_t)(host->mapbase + OMAP_HSMMC_DATA),
					(data->blksz / 4), count, host);
			if (dma_chain_status != 0)
				dev_err(mmc_dev(host->mmc),
				  "%s: omap_dma_chain_a_transfer() failed during write with %d\n",
				  mmc_hostname(host->mmc), dma_chain_status);
		}
		host->sg_idx++;
	}
}

/*
 * Routine to configure block leangth for MMC/SD/SDIO cards
 * and intiate the transfer.
 */
static int mmc_omap_prepare_data(struct mmc_omap_host *host, struct mmc_request *req)
{
	int use_dma;
	int i, block_size;
	unsigned int sg_len;
	struct mmc_data *data = req->data;
#ifdef CONFIG_OMAP_SDIO
	int byte_count = 0, func = 0;
	int ret = 0;
#endif

	if(unlikely(host == NULL))
		return -1;

	/* Store the pointer for request */
	host->data = req->data;
	/* make use of DMA by default */
	host->use_dma = 1;

#ifdef CONFIG_OMAP_SDIO
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

		func = ((req->cmd->arg & SDIO_FUNCTION_MASK) >> 28);

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

#ifdef  CONFIG_OMAP_SDIO_NON_DMA_MODE
		host->buffer = mmc_omap_get_buf_from_sg(host);
#else
		if (!req->cmd->polling) {
			ret = sdio_omap_start_dma_transfer(host, req);
			if (ret != 0) {
				dev_dbg(mmc_dev(host->mmc),
					"Sdio start dma failure\n");
				return ret;
			} else {
				host->buffer = NULL;
				host->bytesleft = 0;
			}
		} else {
			host->buffer = mmc_omap_get_buf_from_sg(host);
		}
#endif
		return 0;
	}
#endif	/* ifdef CONFIG_OMAP_SDIO */

	/* Enable DMA */
	if (req->data == NULL) {
		host->datadir = OMAP_MMC_DATADIR_NONE;
		OMAP_HSMMC_WRITE(host->base, BLK, BLK_CLEAR);
		/* Since there is nothing to DMA, clear the flag */
		host->use_dma = 0;
		return 0;
	}

	OMAP_HSMMC_WRITE(host->base, BLK, (req->data->blksz));
	OMAP_HSMMC_WRITE(host->base, BLK,
		OMAP_HSMMC_READ(host->base, BLK) | (req->data->blocks << 16));

	/* Copy the Block Size information */
	block_size = data->blksz;

	/* Cope with calling layer confusion; it issues "single
	 * block" writes using multi-block scatterlists.
	 */
	sg_len = (data->blocks == 1) ? 1 : data->sg_len;

	if(sg_len > dma_chains) {
		host->extra_chain_reqd = sg_len % dma_chains;
		host->no_of_chain_reqd = sg_len / dma_chains;
		host->chains_requested = dma_chains;
		host->current_cb_cnt = 1;
	} else {
		host->extra_chain_reqd = 0;
		host->no_of_chain_reqd = 0;
		host->chains_requested = data->sg_len;
		host->current_cb_cnt = 0;
	}

	host->last_sg_nbytes = req->data->blksz * req->data->blocks;
	
	/* Only do DMA for entire blocks */
	use_dma = host->use_dma;
	if (use_dma) {
		for (i = 0; i < sg_len; i++) {
			if ((data->sg[i].length % block_size) != 0) {
				use_dma = 0;
				break;
	    		}
			
			if (i != sg_len-1)
				host->last_sg_nbytes -= sg_dma_len(&data->sg[i]);
		}
	}

	host->datadir = (req->data->flags & MMC_DATA_WRITE) ?
			OMAP_MMC_DATADIR_WRITE : OMAP_MMC_DATADIR_READ;

	/* Initialize the internal scatter list count */
	host->sg_idx = 0;
	if (use_dma && !req->cmd->polling) {
		if (mmc_omap_get_dma_channel(host, data) == 0) {
			enum dma_data_direction dma_data_dir;

			if (data->flags & MMC_DATA_WRITE)
				dma_data_dir = DMA_TO_DEVICE;
			else
				dma_data_dir = DMA_FROM_DEVICE;

			host->sg_len = dma_map_sg(mmc_dev(host->mmc), data->sg,
					sg_len, dma_data_dir);

			host->total_bytes_left = 0;
			mmc_chain_dma(host, req->data);
			host->brs_received = 0;
			host->dma_done = 0;

			/* Enable DMA */
			host->use_dma = 1;
		} else {
			host->use_dma = 0;
		}
	} else {
		/* Revert to CPU copy */
		host->buffer =
			(u32 *) (page_address(req->data->sg->page) +
				req->data->sg->offset);
		host->bytesleft = req->data->blocks * (req->data->blksz);
		host->dma_ch = -1;
		host->use_dma = 0;
	}

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
			OMAP_HSMMC_READ(host->base, SYSCTL) | (1 << 26));
		while (OMAP_HSMMC_READ(host->base, SYSCTL) & (1 << 26))
			cond_resched();
	}

	if ((req->cmd->opcode == SD_APP_SEND_SCR
		|| (req->cmd->opcode == MMC_SEND_EXT_CSD)) && mmc->mode != MMC_MODE_CE_ATA)
		mmc->ios.bus_width = MMC_BUS_WIDTH_1;

	if (mmc_omap_prepare_data(host, req))
		dev_dbg(mmc_dev(host->mmc),
			"MMC host %s failed to initiate data transfer\n",
			mmc_hostname(host->mmc));

	/* Start the DMA if DMA is needed */
	if (host->use_dma && !req->cmd->polling
		&& (host->mmc->mode == MMC_MODE_MMC
			|| host->mmc->mode == MMC_MODE_SD
			|| host->mmc->mode == MMC_MODE_CE_ATA))
	{
		omap_start_dma_chain_transfers(host->chain_id);
	}

	mmc_clk_disable_aggressive(host);

	/* Send the stop command. Remember FCLK is not stopped before this call */
	mmc_omap_start_command(host, req->cmd);
}

static void mmc_omap_end_of_data(struct mmc_omap_host *host, struct mmc_data *data)
{
	unsigned long flags;
	int done;

	if (!host->use_dma) {
		mmc_omap_xfer_done(host, data);
		host->tc = 1;
		return;
	}
	done = 0;
	spin_lock_irqsave(&host->dma_lock, flags);
	if (host->dma_done)
		done = 1;
	else
		host->brs_received = 1;
	spin_unlock_irqrestore(&host->dma_lock, flags);

	if (done) {
		mmc_omap_xfer_done(host, data);
		host->tc = 1;
	}
}

static void mmc_omap_dma_done(struct mmc_omap_host *host, struct mmc_data *data)
{
	unsigned long flags;
	int done;

	done = 0;
	spin_lock_irqsave(&host->dma_lock, flags);
	if (host->brs_received)
		done = 1;
	else
		host->dma_done = 1;
	spin_unlock_irqrestore(&host->dma_lock, flags);
	if (done) {
		mmc_omap_xfer_done(host, data);
		host->tc = 1;
	}
	
	if (host->cc && host->tc)
		mmc_omap_request_done(host, host->mrq);
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

	dev_dbg(mmc_dev(host->mmc), "%s: set_ios: clock %dHz busmode %d"
			"powermode %d Vdd %x Bus Width %d\n",
			mmc_hostname(host->mmc), ios->clock, ios->bus_mode,
			ios->power_mode, ios->vdd, ios->bus_width);

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		host->initstream = 0;
#if 0 && defined(CONFIG_MMC_OMAP3430) || defined(CONFIG_MMC_OMAP3430_MODULE) 
		if (host->id == OMAP_MMC1_DEVID) {
			addr = (int *)&OMAP2_CONTROL_PBIAS_1;
			*addr &= ~(1 << 1); // VDDS is unstable !!!
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

#if 0 && defined(CONFIG_MMC_OMAP3430) || defined(CONFIG_MMC_OMAP3430_MODULE) 
#ifndef CONFIG_MMC_OMAP3430_18V_ONLY
		if (host->id == OMAP_MMC1_DEVID) {
			addr = (int *)&OMAP2_CONTROL_PBIAS_1;
			*addr |= (1 << 0); // VDDS = 3.0V
			*addr |= (1 << 1); // VDDS is stable !!!
			if (is_sil_rev_greater_than(OMAP3430_REV_ES1_0))
				*addr |= (1 << 9);
		}
#endif
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

#if defined(CONFIG_MMC_OMAP3430) || defined(CONFIG_MMC_OMAP3430_MODULE) 
#ifndef CONFIG_MMC_OMAP3430_18V_ONLY
	if (host->id == OMAP_MMC1_DEVID) {
		if (cpu_is_omap34xx() && is_sil_rev_less_than(OMAP3430_REV_ES2_0)) {
			if ((OMAP_HSMMC_READ(host->base, HCTL) & SDVSDET) &&
		    		((ios->vdd == 7) || (ios->vdd == 8))) {
				if (omap_mmc_switch_opcond(host, 0) != 0)
					dev_dbg(mmc_dev(host->mmc),
				       		"omap_mmc_set_ios:"
						"switch operation failed\n");
				host->initstream = 0;
			}
		}
	}
#endif
#endif
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
		&& time_before(jiffies, timeout))
		cond_resched();

	/* Enable clock to the card */
	OMAP_HSMMC_WRITE(host->base, SYSCTL,
			OMAP_HSMMC_READ(host->base, SYSCTL) | CEN);

#ifdef CONFIG_OMAP_SDIO
	/* TI recommends this work-around against spurious SDIO interrupt
	 * generation */
	if (host->mmc->mode == MMC_MODE_SDIO)
		OMAP_HSMMC_WRITE(host->base, CON,
			OMAP_HSMMC_READ(host->base, CON) | (CTPL | CLKEXTFREE));
#endif

	mmc_clk_disable_aggressive(host);
}

static void omap_mmc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct mmc_omap_host *host = mmc_priv(mmc);
	unsigned long flags;

	if (host->mmc->mode != MMC_MODE_SDIO || host->sdio_enable == enable)
		return;

	host->sdio_enable = enable;
	spin_lock_irqsave( &host->sdio_reg_lock, flags );
#ifdef CONFIG_OMAP_SDIO_INTERRUPT
	if (enable){
		OMAP_HSMMC_WRITE(host->base, ISE, OMAP_HSMMC_READ(host->base, ISE) | CIRQ);
		OMAP_HSMMC_WRITE(host->base, IE, OMAP_HSMMC_READ(host->base, IE) | CIRQ);
	} else 
#endif
	{
		OMAP_HSMMC_WRITE(host->base, ISE, OMAP_HSMMC_READ(host->base, ISE) & ~CIRQ);
		OMAP_HSMMC_WRITE(host->base, IE, OMAP_HSMMC_READ(host->base, IE) & ~CIRQ);
	}
	spin_unlock_irqrestore( &host->sdio_reg_lock, flags );
}

static struct mmc_host_ops mmc_omap_ops = {
	.request = omap_mmc_request,
	.set_ios = omap_mmc_set_ios,
	.enable_sdio_irq = omap_mmc_enable_sdio_irq,
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
	int ret = 0, irq, *addr;
	unsigned long tmp;
	
	if (minfo == NULL) {
		dev_err(&pdev->dev, "platform data missing\n");
		return -ENXIO;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (res == NULL || irq < 0)
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

	host->use_dma = OMAP_USE_DMA;
	host->dma_ch = -1;
	host->initstream = 0;
	host->mem_res = res;
	host->irq = irq;
	host->id = pdev->id;
	host->mapbase = (void *)host->mem_res->start;
	host->base = (void __iomem *)IO_ADDRESS(host->mapbase);
	mmc->ops = &mmc_omap_ops;
	mmc->f_min = 400000;
	if (minfo->f_max)
		mmc->f_max = minfo->f_max;
	else
		mmc->f_max = CONFIG_OMAP_CLOCK_MAX;
	spin_lock_init(&host->dma_lock);
	host->chain_id = -1;
	host->sg_dma_len = 0;

	mmc->max_phys_segs = 32;
	mmc->max_hw_segs = 32;
	mmc->max_blk_size = 512;       /* Block Length at max can be 1024 */
	mmc->max_blk_count = 2048;
	mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;
	mmc->max_seg_size = mmc->max_req_size;

	if (cpu_is_omap34xx()) {
		/* 3430-ES1.0  Sil errata fix */
		if (is_sil_rev_less_than(OMAP3430_REV_ES2_0)) {
			host->gptfck = clk_get(&pdev->dev, "gpt10_fck");
			if (IS_ERR(host->gptfck)) {
				ret = PTR_ERR(host->gptfck);
				host->gptfck = NULL;
				goto clk_get_err;
			}
		}

		host->fclk = clk_get(&pdev->dev, "mmc_fck");
		if (IS_ERR(host->fclk)) {
			ret = PTR_ERR(host->fclk);
			host->fclk = NULL;
			goto clk_get_err;
		}

		host->iclk = clk_get(&pdev->dev, "mmc_ick");
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
			*addr |= (1 << 2)|(1<<10);

			addr = (int *)&OMAP2_CONTROL_DEVCONF0;
			*addr |= (1 << 24);
#endif
			/* There is no 8-bit field in the structure yet */
			if (minfo->wire4)
				mmc->caps = MMC_CAP_4_BIT_DATA;
			if (minfo->wire8)
				mmc->caps = MMC_CAP_8_BIT_DATA;

			OMAP_HSMMC_WRITE(host->base, HCTL,
					OMAP_HSMMC_READ(host->base,
					HCTL) | SDVS30);

		}
		mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
		OMAP_HSMMC_WRITE(host->base, CAPA,OMAP_HSMMC_READ(host->base,
							CAPA) | VS30 | VS18);
	} else if (host->id == OMAP_MMC2_DEVID) {
		if (cpu_is_omap34xx()) {
#if defined(CONFIG_MMC_OMAP3430) || defined(CONFIG_MMC_OMAP3430_MODULE) 
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

#if 0 // settings used for sdio interrupt ???
		tmp = OMAP_HSMMC_READ(host->base, SYSCONFIG);
		tmp &= ~(AUTOIDLE | (3<<3));
		tmp |= (2<<3) | (1<<2) | AUTOIDLE;
		OMAP_HSMMC_WRITE(host->base, SYSCONFIG, tmp);
#endif
	} else if (host->id == OMAP_MMC3_DEVID) {
		if (minfo->wire4)
			mmc->caps = MMC_CAP_4_BIT_DATA;
		if (minfo->wire8)
			mmc->caps = MMC_CAP_8_BIT_DATA;

		OMAP_HSMMC_WRITE(host->base, HCTL,
				OMAP_HSMMC_READ(host->base,
						HCTL) | SDVS18);

		mmc->ocr_avail = MMC_VDD_165_195;

		/* only 1.8V I/O capabilities for MMC3 */
		OMAP_HSMMC_WRITE(host->base, CAPA, OMAP_HSMMC_READ(host->base, CAPA) | VS18);
	}

	mmc->caps |= MMC_CAP_MULTIWRITE | MMC_CAP_BYTEBLOCK | MMC_CAP_MMC_HIGHSPEED |
			MMC_CAP_SD_HIGHSPEED;

	/* Set SD bus power bit */
	OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) | SDBP | IWE);

#if defined(CONFIG_MACH_OMAP_3430SDP) || defined(CONFIG_MACH_OMAP3_EVM)
	if (machine_is_omap_2430sdp() || machine_is_omap_3430sdp() || machine_is_omap3_evm()) {
		/*
		 * Create sysfs entries for enabling/disabling hotplug
		 * support for MMC cards
		 */
		if (device_create_file(&pdev->dev,
				&dev_attr_mmc_cover_switch) < 0) {
			dev_dbg(mmc_dev(host->mmc),
		       		"Unable to create sysfs"
				"attribute for MMC1 cover switch\n");
		}
		if (device_create_file(&pdev->dev,
			&dev_attr_mmc_card_detect) < 0) {
			dev_dbg(mmc_dev(host->mmc),
				"Unable to create sysfs"
				"attribute for MMC1 card detect\n");
		}
	}
#endif

#ifdef CONFIG_OMAP_SDIO 
#ifdef CONFIG_OMAP_SDIO_INTERRUPT
	mmc->caps |= MMC_CAP_SDIO_IRQ;
#endif
	spin_lock_init(&host->sdio_reg_lock);
	host->sdio_enable = 0;
#endif

	if (device_create_file(&pdev->dev, &dev_attr_change_card_state) < 0) {
		dev_dbg(mmc_dev(host->mmc),
			"Unable to create sysfs"
			"attribute for mmc detect change\n");
	}

	/* Request IRQ for MMC operations */
	ret = request_irq(host->irq, mmc_omap_irq, 0, pdev->name,
				host);
	if (ret) {
		dev_dbg(mmc_dev(host->mmc), "Unable to grab HSMMC IRQ");
		goto irq_err;
	}

	host->card_detect_irq = minfo->switch_pin;
	if (minfo->switch_pin >= 0) {
#if defined(CONFIG_MACH_OMAP_3430SDP) || defined(CONFIG_MACH_OMAP3_EVM)
		if (machine_is_omap_2430sdp() || machine_is_omap_3430sdp() || machine_is_omap3_evm()) {
			host->card_detect_irq =
				TWL4030_GPIO_IRQ_NO(minfo->switch_pin);
			INIT_WORK(&host->mmc_carddetect_work, mmc_omap_detect);


			if (setup_mmc_carddetect_irq(minfo->switch_pin)) {
				free_irq(host->irq, host);
				goto irq_err;
			}

		}
#endif
	}

	if (minfo->switch_pin >= 0) {
		ret = request_irq(host->card_detect_irq,
			mmc_omap_irq_cd, IRQF_DISABLED, pdev->name,host);
		if (ret < 0) {
			dev_dbg(mmc_dev(host->mmc),
				"Unable to grab T2 GPIO IRQ");
			free_irq(host->irq, host);
			goto irq_err;
		}
	}

	if (host->id == OMAP_MMC1_DEVID)
		saved_host1 = host;
	else if (host->id == OMAP_MMC2_DEVID)
		saved_host2 = host;
	else
		saved_host3 = host;

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
		free_irq(host->irq, host);
		free_irq(host->card_detect_irq, host);
		flush_scheduled_work();

		/* Free the clks */
#ifndef AGGR_PM_CAP
		mmc_clk_disable(host);
#endif /* #ifndef AGGR_PM_CAP */

		clk_put(host->fclk);
		clk_put(host->iclk);

		device_remove_file(&pdev->dev,
					&dev_attr_change_card_state);

		mmc_free_host(host->mmc);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, res->end - res->start + 1);

	return 0;
}

#ifdef CONFIG_PM
/*
 * Routine to suspend the MMC device
 */
static int omap_mmc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mmc_omap_host *host = platform_get_drvdata(pdev);
	int ret = 0;
	int status;
	int *addr;

	DPRINTK("omap_mmc_suspend\n");

	if (!host)
		return 0;
		
	if (host->suspended)
		return 0;

	/* Notify the core to suspend the host */
	ret = mmc_suspend_host(host->mmc, state);
	if (ret)
		return ret;

	host->suspended = 1;

	/* Temporarily enabling the clocks for configuration */
	mmc_clk_enable_aggressive(host);

#if defined(CONFIG_MACH_OMAP_3430SDP) || defined(CONFIG_MACH_OMAP3_EVM)
	if (machine_is_omap_2430sdp() || machine_is_omap_3430sdp() || machine_is_omap3_evm()) {
		disable_irq(host->card_detect_irq);
		ret = mask_carddetect_int(host->id);
		if (ret)
			dev_dbg(mmc_dev(host->mmc),
				"Unable to mask the card detect"
				"interrupt in suspend\n");
	}
#endif

#ifdef CONFIG_OMAP34XX_OFFMODE
	omap2_hsmmc_save_ctx(host);
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */

	if (cpu_is_omap34xx()) {
		if (!(OMAP_HSMMC_READ(host->base, HCTL) & SDVSDET)) {
			OMAP_HSMMC_WRITE(host->base, HCTL,
				OMAP_HSMMC_READ(host->base, HCTL) & SDVSCLR);
				
			if (host->id == OMAP_MMC1_DEVID)
				OMAP_HSMMC_WRITE(host->base, HCTL,
					OMAP_HSMMC_READ(host->base, HCTL) | SDVS30);
			else
			if (host->id == OMAP_MMC2_DEVID)
				OMAP_HSMMC_WRITE(host->base, HCTL,
					OMAP_HSMMC_READ(host->base, HCTL) | SDVS18);

			OMAP_HSMMC_WRITE(host->base, HCTL,
				OMAP_HSMMC_READ(host->base, HCTL) | SDBP);
		}
	}

	/* Disable Interrupts */
	OMAP_HSMMC_WRITE(host->base, ISE, INT_CLEAR);
	OMAP_HSMMC_WRITE(host->base, IE, INT_CLEAR);

	/* Clearing the STAT register*/
	status = OMAP_HSMMC_READ(host->base, STAT);
	OMAP_HSMMC_WRITE(host->base, STAT, status);
	/* disable clks for MMC1 */
	mmc_clk_disable(host);

#if defined (CONFIG_MACH_ARCHOS_G6)
	if (host->id == OMAP_MMC1_DEVID) {
		addr = (int *)&OMAP2_CONTROL_PBIAS_1;
		*addr &= ~(1 << 1); // VDDS is unstable !!!
		if (is_sil_rev_greater_than(OMAP3430_REV_ES1_0))
			*addr &= ~(1 << 9);
	}
#endif

	ret = mmc_omap_power(host,0);
	if (ret != 0)
		dev_dbg(mmc_dev(host->mmc),
			"Unable to disable power\n");
	host->initstream = 0;
	
	return ret;
}

/*
 * Routine to resume the MMC device
 */
static int omap_mmc_resume(struct platform_device *pdev)
{
	struct mmc_omap_host *host = platform_get_drvdata(pdev);
	int ret = 0;
	int *addr;

	DPRINTK("omap_mmc_resume\n");

	if (!host)
		return 0;
		
	if (!host->suspended)
		return 0;

	ret = mmc_omap_power(host,1);
	if (ret != 0) {
		dev_dbg(mmc_dev(host->mmc),
		       "Unable to enable power to MMC1\n");
		return ret;
	}

#if defined (CONFIG_MACH_ARCHOS_G6)
	if (host->id == OMAP_MMC1_DEVID) {
		addr = (int *)&OMAP2_CONTROL_PBIAS_1;
		*addr |= (1 << 0); // VDDS = 3.0V
		*addr |= (1 << 1); // VDDS is stable !!!
		if (is_sil_rev_greater_than(OMAP3430_REV_ES1_0))
			*addr |= (1 << 9);
	} else
	if (host->id == OMAP_MMC2_DEVID) {
		addr = (int *)&OMAP2_CONTROL_DEVCONF1;
		*addr |= (1 << 6);
	}
#endif

#ifndef AGGR_PM_CAP
	mmc_clk_enable(host);
#endif /* #ifndef AGGR_PM_CAP */

#ifdef CONFIG_OMAP34XX_OFFMODE
	omap2_hsmmc_restore_ctx(host);
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */

#if defined(CONFIG_MACH_OMAP_3430SDP) || defined(CONFIG_MACH_OMAP3_EVM)
	if (machine_is_omap_3430sdp() || machine_is_omap3_evm()) {
		enable_irq(host->card_detect_irq);
		ret = unmask_carddetect_int(host->id);
		if (ret)
			dev_dbg(mmc_dev(host->mmc),
				"Unable to unmask the card"
				"detect interrupt\n");
	}
#endif
	/* Notify the core to resume the host */
	ret = mmc_resume_host(host->mmc);
	if (ret == 0)
		host->suspended = 0;
	
	return ret;
}

#else
#define omap_mmc_suspend	NULL
#define omap_mmc_resume		NULL
#endif

#ifdef CONFIG_DPM
static int
omap_mmc_pre_scale(int slot, struct notifier_block *op, unsigned long level,
		   void *ptr)
{
	int i = 0, timeout = 20;
	struct mmc_omap_host *host = (slot == MMC1) ? saved_host1 : (slot == MMC2) ? saved_host3 : saved_host2;

	printk("omap_mmc_pre_scale\n");

	switch (level) {
	case SCALE_PRECHANGE:
		/* If DMA is active then enable the stop at block gap event */
		if (host->dma_ch) {
			OMAP_HSMMC_WRITE(host->base, HCTL,
					OMAP_HSMMC_READ(host->base,HCTL) | SBGR);
			while (((OMAP_HSMMC_READ(host->base, STAT) & TC) != 0x2)
				|| (i < timeout)) {
				i++;
				/*
				 * Wait for 5 Micro seconds before reading the
				 * Block gap status
				 */
				udelay(5);
			}
			host->dma_ch = -1;
		}
		break;
	}
	return 0;
}

static int
omap_mmc_post_scale(int slot, struct notifier_block *op, unsigned long level,
		    void *ptr)
{
	struct mmc_omap_host *host = (slot == MMC1) ? saved_host1 : (slot == MMC2) ? saved_host3 : saved_host2;

	printk("omap_mmc_post_scale\n");

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
