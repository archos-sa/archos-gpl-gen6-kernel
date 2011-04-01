/** @file if_sdio.c
 *  @brief This file contains SDIO IF (interface) module
 *  related functions.
 *  
 * Copyright 2007 Intel Corporation and its suppliers. All rights reserved
 * 
 * (c) Copyright � 2003-2007, Marvell International Ltd. 
 *
 * This software file (the "File") is distributed by Marvell International 
 * Ltd. under the terms of the GNU General Public License Version 2, June 1991 
 * (the "License").  You may use, redistribute and/or modify this File in 
 * accordance with the terms and conditions of the License, a copy of which 
 * is available along with the File in the gpl.txt file or by writing to 
 * the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 
 * 02111-1307 or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE 
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about 
 * this warranty disclaimer.
 *
 */

#include "if_sdio.h"
#include <asm/scatterlist.h>

#include <linux/firmware.h>

#define DEFAULT_HELPER_NAME "mrvl/helper_sd.bin"
#define DEFAULT_FW_NAME "mrvl/sd8686.bin"

extern u8 *helper_name;
extern u8 *fw_name;

/* 
 * Define the SD Block Size
 * for SD8381-B0/B1, SD8385/B0 chip, it could be 32,64,128,256,512
 * for all other chips including SD8381-A0, it must be 32
 */

/* define SD block size for firmware download */
#define SD_BLOCK_SIZE_FW_DL     64
#define SDIO_HEADER_LEN		4

/* define SD block size for data Tx/Rx */
#define SD_BLOCK_SIZE           512

/* manually set the CIS addr for 8686 */
#define CIS_8686_PTR 0x8000

#define ALLOC_BUF_SIZE          (((MAX(MRVDRV_ETH_RX_PACKET_BUFFER_SIZE, \
                                        MRVDRV_SIZE_OF_CMD_BUFFER) + SDIO_HEADER_LEN \
                                        + SD_BLOCK_SIZE - 1) / SD_BLOCK_SIZE) * SD_BLOCK_SIZE)

#define GPIO_PORT_NUM
#define GPIO_PORT_INIT()
#define GPIO_PORT_TO_HIGH()
#define GPIO_PORT_TO_LOW()

/*
 * Poll the Card Status register until the bits specified in the argument
 * are turned on.
 */

#define BLOCK_MODE 1
#define FIXED_ADDRESS 0

// feng: temply add some notes here
#define BUS_IF_CTRL 0x0

struct net_device *the_netdev = NULL;

int sbi_mmc_disable_host_int(wlan_private *priv);


/* some sdio wrapper API */
static int sdio_read_byte(struct sdio_func *func, u32 addr, u8 *pdata)
{
	int ret;
	sdio_claim_host(func);
	*pdata = sdio_readb(func, addr, &ret);
	sdio_release_host(func);
	return ret;
}

static int sdio_write_byte(struct sdio_func *func, u32 addr, u8 data)
{
	int ret;
	sdio_claim_host(func);
	sdio_writeb(func, data, addr, &ret);
	sdio_release_host(func);
	return ret;
}

static int sdio_card_readb(struct sdio_func *func, u32 addr, u8 *data)
{
	int ret;

	sdio_claim_host(func);
	*data = sdio_f0_readb(func, addr, &ret);
	sdio_release_host(func);

	return ret;
}

static int sdio_card_writeb(struct sdio_func *func, u32 addr, u8 data)
{
	int ret;

	sdio_claim_host(func);
	sdio_f0_writeb(func, data, addr, &ret);
	sdio_release_host(func);

	return ret;
}


static wlan_private *pwlanpriv;
static wlan_private *(*wlan_add_callback) (void *dev_id);
static int (*wlan_remove_callback) (void *dev_id);
// static int cmd_result = 0;

int mv_sdio_read_event_cause(wlan_private * priv);

void sbi_interrupt(struct sdio_func  *func)
{
	umd_dbg("w8686 : sbi_interrupt\n");
	wlan_interrupt(the_netdev);
}

int sbi_probe_card(void *val)
{
	return WLAN_STATUS_SUCCESS;
}

static void sbi_io_request_done(struct mmc_request *req)
{
    ENTER();
    LEAVE();
}



static int sbi_dev_probe_func(struct sdio_func *func, const struct sdio_device_id *id)
{
	int result, ret = WLAN_STATUS_SUCCESS;
    	u8 chiprev, bic;   

	umd_dbg("enter, func = 0x%08x", (unsigned int) func);
    
	result = sdio_read_byte(func, CARD_REVISION_REG, &chiprev);

	/* read Revision Register to get the hw revision number */
	if (result) {
		printk("cannot read CARD_REVISION_REG\n");
	} else {
		umd_dbg("revision=0x%x\n", chiprev);
		switch (chiprev) {
			default:

			/* enable async interrupt mode */
			sdio_card_readb(func, BUS_IF_CTRL, &bic);
			bic |= ASYNC_INT_MODE;
			sdio_card_writeb(func, BUS_IF_CTRL, bic);
			break;
		}
    	}

	if (!wlan_add_callback)
		goto done;

	umd_dbg("will call sdio_enable_func");
	sdio_claim_host(func);
	ret = sdio_enable_func(func);
	sdio_release_host(func);
	if (ret) {
		printk(KERN_ALERT "Fail to enable func\n");
		ret = WLAN_STATUS_FAILURE;
		goto done;
	}

	/* feng: here we need add the irq hook */
	sdio_claim_host(func);
	ret =  sdio_claim_irq(func, sbi_interrupt);
	umd_dbg("sdio_claim_irq, ret = %d", ret);
	sdio_release_host(func);
	
        
	umd_dbg("Will call the wlan_add_callback");
	pwlanpriv = wlan_add_callback(func);

	if (pwlanpriv)
		ret = WLAN_STATUS_SUCCESS;
	else
		ret = WLAN_STATUS_FAILURE;

	//sdio_claim_host(func);
       //sdio_dump_cccr(func);
	//sdio_release_host(func);
	umd_dbg("exit, ret = %d", ret);
done:
	LEAVE();
	return ret;
}

static void sbi_dev_remove_func(struct sdio_func *func)
{
	if (!wlan_remove_callback)
		return /*WLAN_STATUS_FAILURE*/;
	pwlanpriv = NULL;

	wlan_remove_callback(func);

	sdio_claim_host(func);
	sdio_release_irq(func);
	sdio_disable_func(func);
	sdio_release_host(func);
}

static const struct sdio_device_id sdio_8686_ids[] = {
	{ SDIO_DEVICE_CLASS(SDIO_CLASS_WLAN)		},
	{ /* end: all zeroes */				},
};

MODULE_DEVICE_TABLE(sdio, sdio_8686_ids);

static struct sdio_driver sdio8686_driver = {
	.probe		= sbi_dev_probe_func,
	.remove		= sbi_dev_remove_func,
	.name 		= "sdio_8686",
	.id_table	= sdio_8686_ids,
};

int *sbi_register(wlan_notifier_fn_add add, wlan_notifier_fn_remove remove,
             void *arg)
{
	umd_enter();
	wlan_add_callback = add;
	wlan_remove_callback = remove;

	sdio_register_driver(&sdio8686_driver);
	umd_exit();
	return (int *) wlan_add_callback;
}

void sbi_unregister(void)
{
	sdio_unregister_driver(&sdio8686_driver);

	wlan_add_callback = NULL;
	wlan_remove_callback = NULL;	
}

int sbi_read_ioreg(wlan_private * priv, u32 reg, u8 *pdata)
{
	struct sdio_func *func = (struct sdio_func*)(priv->wlan_dev.card);
	int ret;

	ret = sdio_read_byte(func, reg, pdata);
	return ret;
}

int sbi_write_ioreg(wlan_private * priv, u32 reg, u8 data)
{
    	struct sdio_func *func = (struct sdio_func*)(priv->wlan_dev.card);
	int ret;

	ret = sdio_write_byte(func, reg, data);
	return ret;
}

int sbi_write_iomem(wlan_private * priv, u32 reg, u8 blockmode,
                u8 opcode, ssize_t cnt, ssize_t blksz, u8 * dat)
{
	struct sdio_func *func = (struct sdio_func*)(priv->wlan_dev.card);
	int ret;
	
	/* feng: need set the block size here */
	sdio_claim_host(func);
	if (blksz != func->cur_blksize) {
		umd_dbg( "sbi_write_iomem, sdio_set_block_size new = %d (cnt=%d)\n",blksz,cnt);
		ret = sdio_set_block_size(func, blksz);
		if (ret) {
			sdio_release_host(func);
			return ret;
		}
	}

// 	ret = mmc_io_rw_extended(func->card, 1, func->num, reg, 0, 
// 		dat, cnt, blksz);
// 	ret = sdio_memcpy_toio(func, reg,
// 			       dat, cnt);
	/*SL ! cnt*blksz !! */
	umd_dbg("sbi_write_iomem, sdio_memcpy_toio %d bytes\n",cnt*blksz);
	ret = sdio_memcpy_toio(func, reg,
			       dat, cnt*blksz);
	sdio_release_host(func);
	return ret;
}

int sbi_read_iomem(wlan_private * priv, u32 reg, u8 blockmode, u8 opcode,
               ssize_t cnt, ssize_t blksz, u8 * dat)
{
	struct sdio_func *func = (struct sdio_func*)(priv->wlan_dev.card);
	int ret;

	sdio_claim_host(func);
	if (blksz != func->cur_blksize) { 
		ret = sdio_set_block_size(func, blksz);
		if (ret) {
			sdio_release_host(func);
			return ret;
		}
	}

// 	ret = mmc_io_rw_extended(func->card, 0, func->num, reg, 0, 
// 		dat, cnt, blksz);
// 	ret = sdio_memcpy_fromio(func, dat,
// 			   reg, cnt);
	/*SL ! cnt*blksz !! */
	umd_dbg("sbi_read_iomem, sdio_memcpy_fromio %d bytes\n",cnt*blksz);
	ret = sdio_memcpy_fromio(func, dat,
				 reg, cnt*blksz);
	sdio_release_host(func);
	return ret;
}

int sbi_read_intr_reg(wlan_private * priv, u8 *ireg)
{
    return sbi_read_ioreg(priv, HOST_INTSTATUS_REG, ireg);
}

int sbi_read_card_reg(wlan_private * priv, u8 *cs)
{
    return sbi_read_ioreg(priv, CARD_STATUS_REG, cs);
}

int sbi_clear_int_status(wlan_private * priv, u8 mask)
{
	return sbi_write_ioreg(priv, HOST_INTSTATUS_REG, 0x0);
}

int
sbi_get_int_status(wlan_private * priv, u8 * ireg)
{
	int ret = WLAN_STATUS_SUCCESS;
	u8 cs, *cmdBuf;
	wlan_dev_t *wlan_dev = &priv->wlan_dev;
	struct sk_buff *skb;
// 	struct sdio_func *func = (struct sdio_func *) (priv->wlan_dev.card);

	//umd_enter();

	// feng: we mask it here, but this may be an issue
	//mss_set_sdio_int(slot->host, MSS_SDIO_INT_DIS);
	sbi_mmc_disable_host_int(priv);

	if ((ret = sbi_read_ioreg(priv, HOST_INTSTATUS_REG, ireg)) < 0) {
		PRINTM(WARN, "Reading interrupt status register failed\n");
		umd_dbg("Reading interrupt status register failed\n");
		ret = WLAN_STATUS_FAILURE;
		goto end;
	}

	//umd_dbg("int status ireg = 0x%08x", *ireg);

	if (*ireg != 0) {           /* DN_LD_HOST_INT_STATUS and/or UP_LD_HOST_INT_STATUS */
		/* Clear the interrupt status register */
		if ((ret = sbi_write_ioreg(priv, HOST_INTSTATUS_REG,
		                           ~(*ireg) & (DN_LD_HOST_INT_STATUS |
		                                       UP_LD_HOST_INT_STATUS))) < 0) {
		    PRINTM(WARN, "sdio_write_ioreg: clear interrupt status"
		           " register failed\n");
		    ret = WLAN_STATUS_FAILURE;
		    goto end;
		}
	}

	if (*ireg & DN_LD_HOST_INT_STATUS) {
		*ireg |= HIS_TxDnLdRdy;
		if (!priv->wlan_dev.dnld_sent) {        // tx_done already received
			umd_dbg("warning: tx_done already received:"
			   " dnld_sent=0x%x ireg=0x%x cs=0x%x",
			   priv->wlan_dev.dnld_sent, *ireg, cs);
		} else {
			//umd_dbg("tx_done received: dnld_sent=0x%x ireg=0x%x"
			 //  " cs=0x%x", priv->wlan_dev.dnld_sent, *ireg, cs);
			if (priv->wlan_dev.dnld_sent == DNLD_DATA_SENT)
			os_start_queue(priv);
			priv->wlan_dev.dnld_sent = DNLD_RES_RECEIVED;
		}
	}

	if (*ireg & UP_LD_HOST_INT_STATUS) {
		/*
		 * DMA read data is by block alignment,so we need alloc extra block
		 * to avoid wrong memory access.
		 */
		if (!(skb = dev_alloc_skb(ALLOC_BUF_SIZE))) {
		    PRINTM(WARN, "No free skb\n");
		    priv->stats.rx_dropped++;
		    return WLAN_STATUS_FAILURE;
		}

		/* Transfer data from card */
		/* TODO: Check for error return on the read */
		/* skb->tail is passed as we are calling skb_put after we
		 * are reading the data */
		if (mv_sdio_card_to_host(priv, &wlan_dev->upld_typ,
		                         (int *) &wlan_dev->upld_len, skb->tail,
		                         ALLOC_BUF_SIZE) < 0) {
		    PRINTM(WARN, "Card to host failed: ireg=0x%x cs=0x%x\n",
		           *ireg, cs);
		    if (sbi_read_ioreg(priv, CONFIGURATION_REG, &cs) < 0)
		        PRINTM(WARN, "sdio_read_ioreg failed\n");

		    //umd_dbg("Config Reg val = %d", cs);
		    if (sbi_write_ioreg(priv, CONFIGURATION_REG, (cs | 0x04)) <
		        0)
		        PRINTM(WARN, "write ioreg failed\n");

		    //umd_dbg("write success");
		    if (sbi_read_ioreg(priv, CONFIGURATION_REG, &cs) < 0)
		        PRINTM(WARN, "sdio_read_ioreg failed\n");

		    //umd_dbg("Config reg val =%x", cs);
		    ret = WLAN_STATUS_FAILURE;
		    kfree_skb(skb);
		    goto end;
		}

		//umd_dbg("Reading data in to skb size=%d upld_type=%d",
		//	wlan_dev->upld_len, wlan_dev->upld_typ);
		if (wlan_dev->upld_typ == MVSD_DAT) {
		    //umd_dbg("Up load type is Data");
		    *ireg |= HIS_RxUpLdRdy;
		    skb_put(skb, priv->wlan_dev.upld_len);
		    skb_pull(skb, SDIO_HEADER_LEN);
		    list_add_tail((struct list_head *) skb,
		                  (struct list_head *) &priv->adapter->RxSkbQ);
		} else if (wlan_dev->upld_typ == MVSD_CMD) {
		    *ireg &= ~(HIS_RxUpLdRdy);
		    *ireg |= HIS_CmdUpLdRdy;

		    //umd_dbg("Up load type is Cmd Response");
		    /* take care of CurCmd = NULL case by reading the 
		     * data to clear the interrupt */
		    if (!priv->adapter->CurCmd) {
		        cmdBuf = priv->wlan_dev.upld_buf;
		        priv->adapter->HisRegCpy &= ~HIS_CmdUpLdRdy;
			//                              *ireg &= ~HIS_RxUpLdRdy;
		    } else {
		        cmdBuf = priv->adapter->CurCmd->BufVirtualAddr;
		    }

		    priv->wlan_dev.upld_len -= SDIO_HEADER_LEN;
		    memcpy(cmdBuf, skb->data + SDIO_HEADER_LEN,
		           MIN(MRVDRV_SIZE_OF_CMD_BUFFER, priv->wlan_dev.upld_len));
		    kfree_skb(skb);
		} else if (wlan_dev->upld_typ == MVSD_EVENT) {
		    *ireg |= HIS_CardEvent;
		    kfree_skb(skb);
		}

		*ireg |= HIS_CmdDnLdRdy;
	}

	ret = WLAN_STATUS_SUCCESS;
end:
	//umd_exit();
	// feng: mask here, may cause some issue	
	sbi_reenable_host_interrupt(priv, 0x00);
	return ret;
}

int
sbi_poll_cmd_dnld_rdy(wlan_private * priv)
{
	return mv_sdio_poll_card_status(priv, CARD_IO_READY | UP_LD_CARD_RDY);
}

int sbi_card_to_host(wlan_private * priv, u32 type, u32 * nb, u8 * payload,
                 u16 npayload)
{
	return WLAN_STATUS_SUCCESS;
}

int sbi_read_event_cause(wlan_private * priv)
{
	return WLAN_STATUS_SUCCESS;
}

int mv_sdio_read_event_cause(wlan_private * priv)
{
	struct sdio_func *func = (struct sdio_func*)(priv->wlan_dev.card);
	int ret;
	u8 scr2;

	ENTER();
	/* the SCRATCH_REG @ 0x8fc tells the cause for the Mac Event */
	if ((ret = sdio_card_readb(func, 0x80fc, &scr2)) < 0) {
		umd_dbg("Unable to read Event cause\n");
		return ret;
	}
	priv->adapter->EventCause = scr2;

	LEAVE();
	return 0;
}

int sbi_retrigger(wlan_private * priv)
{

    ENTER();
    if (sbi_write_ioreg(priv, CONFIGURATION_REG, 0x4) < 0) {
        return -1;
    }
    LEAVE();
    return 0;
}

int if_dnld_ready(wlan_private * priv)
{
    int rval;
    u8 cs;
    ENTER();
    rval = sbi_read_ioreg(priv, CARD_STATUS_REG, &cs);
    if (rval < 0)
        return -EBUSY;
    LEAVE();
    return (cs & DN_LD_CARD_RDY) && (cs & CARD_IO_READY);
}

int sbi_is_tx_download_ready(wlan_private * priv)
{
	int rval;
	ENTER();
	rval = if_dnld_ready(priv);
	LEAVE();
	return (rval < 0) ? -1 : (rval == 1) ? 0 : -EBUSY;  //Check again
}

int sbi_reenable_host_interrupt(wlan_private *priv, u8 bits)
{
	struct sdio_func *func = (struct sdio_func*)(priv->wlan_dev.card);
	struct mmc_host	*host;

	host = func->card->host;
	host->ops->enable_sdio_irq(host, 1);
#if 0
    struct mss_slot *slot = (struct mss_slot *) (priv->wlan_dev.card);
    ENTER();
    //mss_set_sdio_int(slot->host, MSS_SDIO_INT_EN);
    LEAVE();
#endif
    return WLAN_STATUS_SUCCESS;

}

int sbi_mmc_disable_host_int(wlan_private *priv) 
{
	struct sdio_func *func = (struct sdio_func*)(priv->wlan_dev.card);
	struct mmc_host	*host;

	host = func->card->host;
	host->ops->enable_sdio_irq(host, 0);
	return 0;
}


/**  @brief This function disables the host interrupts mask.
 *
 *  @param priv    A pointer to wlan_private structure
 *  @param mask    the interrupt mask
 *  @return        WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int disable_host_int_mask(wlan_private * priv, u8 mask)
{
	int ret = WLAN_STATUS_SUCCESS;
	u8 host_int_mask;

	ENTER();
	/* Read back the host_int_mask register */
	ret = sbi_read_ioreg(priv, HOST_INT_MASK_REG, &host_int_mask);
	if (ret < 0) {
		ret = WLAN_STATUS_FAILURE;
		goto done;
	}

	/* Update with the mask and write back to the register */
	host_int_mask &= ~mask;
	ret = sbi_write_ioreg(priv, HOST_INT_MASK_REG, host_int_mask);
	if (ret < 0) {
		PRINTM(WARN, "Unable to diable the host interrupt!\n");
		ret = WLAN_STATUS_FAILURE;
    	}
  done:
	LEAVE();
	return ret;
}

/**
 *  @brief This function disables the host interrupts.
 *
 *  @param priv    A pointer to wlan_private structure
 *  @return        WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
int sbi_disable_host_int(wlan_private * priv)
{
	return disable_host_int_mask(priv, HIM_DISABLE);
}

/**
 *  @brief This function enables the host interrupts mask
 *
 *  @param priv    A pointer to wlan_private structure
 *  @param mask    the interrupt mask
 *  @return        WLAN_STATUS_SUCCESS
 */
static int enable_host_int_mask(wlan_private * priv, u8 mask)
{
	int ret = WLAN_STATUS_SUCCESS;

	ENTER();
	/* Simply write the mask to the register */
	ret = sbi_write_ioreg(priv, HOST_INT_MASK_REG, mask);

	if (ret < 0) {
		PRINTM(WARN, "ret = %d\n", ret);
		ret = WLAN_STATUS_FAILURE;
	}
	priv->adapter->HisRegCpy = 1;

	LEAVE();
	return ret;
}

/**
 *  @brief This function enables the host interrupts.
 *
 *  @param priv    A pointer to wlan_private structure
 *  @return        WLAN_STATUS_SUCCESS
 */
int sbi_enable_host_int(wlan_private * priv)
{
	return enable_host_int_mask(priv, HIM_ENABLE);
}

int sbi_unregister_dev(wlan_private * priv)
{
	ENTER();

	if (priv->wlan_dev.card != NULL) {
		/* Release the SDIO IRQ */
		//sdio_free_irq(priv->wlan_dev.card, priv->wlan_dev.netdev);
		PRINTM(WARN, "Making the sdio dev card as NULL\n");
	}
	LEAVE();
	return WLAN_STATUS_SUCCESS;
}

int mv_sdio_poll_card_status(wlan_private * priv, u8 bits)
{
    int tries;
    int rval;
    u8 cs;

    ENTER();
    for (tries = 0; tries < MAX_POLL_TRIES; tries++) {
        rval = sbi_read_ioreg(priv, CARD_STATUS_REG, &cs);
        //PRINTM(INFO, "rval = %x\n cs&bits =%x\n", rval, (cs & bits));
        if (rval == 0 && (cs & bits) == bits) {
            return 0;
        }

        udelay(100);
    }

    umd_dbg("mv_sdio_poll_card_status: FAILED!\n");
    LEAVE();
    return -EBUSY;
}

int sbi_register_dev(wlan_private * priv)
{
    int ret = WLAN_STATUS_SUCCESS;
    u8 reg;
    ENTER();

    the_netdev = priv->wlan_dev.netdev;

    /* Initialize the private structure */
    strncpy(priv->wlan_dev.name, "sdio0", sizeof(priv->wlan_dev.name));
    priv->wlan_dev.ioport = 0;
    priv->wlan_dev.upld_rcv = 0;
    priv->wlan_dev.upld_typ = 0;
    priv->wlan_dev.upld_len = 0;

    /* Read the IO port */
    ret = sbi_read_ioreg(priv, IO_PORT_0_REG, &reg);
    if (ret)
        goto failed;
    else
        priv->wlan_dev.ioport |= reg;

    ret = sbi_read_ioreg(priv, IO_PORT_1_REG, &reg);
    if (ret)
        goto failed;
    else
        priv->wlan_dev.ioport |= (reg << 8);

    ret = sbi_read_ioreg(priv, IO_PORT_2_REG, &reg);
    if (ret)
        goto failed;
    else
        priv->wlan_dev.ioport |= (reg << 16);

    umd_dbg("SDIO FUNC1 IO port: 0x%x\n", priv->wlan_dev.ioport);

    /* Disable host interrupt first. */
    if ((ret = disable_host_int_mask(priv, 0xff)) < 0) {
        PRINTM(WARN, "Warning: unable to disable host interrupt!\n");
    }
#if 0                           //marked by xc, we have init it
    /* Request the SDIO IRQ */
    umd_dbg("Before request_irq Address is if==>%p\n", isr_function);
    ret = sdio_request_irq(priv->wlan_dev.card,
                           (handler_fn_t) isr_function, 0,
                           "sdio_irq", priv->wlan_dev.netdev);

    umd_dbg("IrqLine: %d\n", card->ctrlr->tmpl->irq_line);

    if (ret < 0) {
        umd_dbg("Failed to request IRQ on SDIO bus (%d)\n", ret);
        goto failed;
    }
#endif
    //mod by xc
#if 0
    priv->wlan_dev.netdev->irq = card->ctrlr->tmpl->irq_line;
    priv->adapter->irq = priv->wlan_dev.netdev->irq;
    priv->adapter->chip_rev = card->chiprev;
#endif

    priv->hotplug_device =
        &((struct sdio_func *) (priv->wlan_dev.card))->card->dev;
    if (helper_name == NULL) {
        helper_name = DEFAULT_HELPER_NAME;
    }
    if (fw_name == NULL) {
        fw_name = DEFAULT_FW_NAME;
    }
    pwlanpriv = priv;
    return WLAN_STATUS_SUCCESS; /* success */

  failed:
    umd_dbg("register device fail\n");
    priv->wlan_dev.card = NULL;

    LEAVE();
    return WLAN_STATUS_FAILURE;
}

/*
 * This fuction is used for sending data to the SDIO card.
 */

int sbi_host_to_card(wlan_private * priv, u8 type, u8 * payload, u16 nb)
{
    int ret = WLAN_STATUS_SUCCESS;
    int buf_block_len;
    int blksz;

    ENTER();

    priv->adapter->HisRegCpy = 0;

    blksz = SD_BLOCK_SIZE;
    buf_block_len = (nb + SDIO_HEADER_LEN + blksz - 1) / blksz;

    priv->adapter->TmpTxBuf[0] = (nb + SDIO_HEADER_LEN) & 0xff;
    priv->adapter->TmpTxBuf[1] = ((nb + SDIO_HEADER_LEN) >> 8) & 0xff;
    priv->adapter->TmpTxBuf[2] = type;
    priv->adapter->TmpTxBuf[3] = 0x0;

    if (payload != NULL && nb > 0) {
        if (type == MVMS_CMD)
            memcpy(&priv->adapter->TmpTxBuf[SDIO_HEADER_LEN], payload, nb);
    } else {
        umd_dbg("Error: payload=%p, nb=%d\n", payload, nb);
    }

    /* The host polls for the IO_READY bit */
    ret = mv_sdio_poll_card_status(priv, CARD_IO_READY);
    if (ret < 0) {
        umd_dbg("<1> Poll failed in host_to_card : %d\n", ret);
        ret = WLAN_STATUS_FAILURE;
        goto exit;
    }

    /* Transfer data to card */
    ret = sbi_write_iomem(priv, priv->wlan_dev.ioport,
                          BLOCK_MODE, FIXED_ADDRESS, buf_block_len,
                          blksz, priv->adapter->TmpTxBuf);

    if (ret < 0) {
        PRINTM(WARN, "sdio_write_iomem failed: ret=%d\n", ret);
        umd_dbg("sdio_write_iomem failed: ret=%d !!!!!!!\n", ret);
        ret = WLAN_STATUS_FAILURE;
        goto exit;
    } else {
        //umd_dbg("sdio write -dnld val =>%d\n", ret);
    }

    if (type == MVSD_DAT)
        priv->wlan_dev.dnld_sent = DNLD_DATA_SENT;
    else
        priv->wlan_dev.dnld_sent = DNLD_CMD_SENT;
  exit:
    LEAVE();
    return ret;
}

/*
 * This function is used to read data from the card.
 */

int
mv_sdio_card_to_host(wlan_private * priv,
                     u32 * type, int *nb, u8 * payload, int npayload)
{
    int ret = WLAN_STATUS_SUCCESS;
    u16 buf_len = 0;
    int buf_block_len;
    int blksz;
    u32 *pevent;

    ENTER();
    if (!payload) {
        PRINTM(WARN, "payload NULL pointer received!\n");
        ret = WLAN_STATUS_FAILURE;
        goto exit;
    }

    /* Read the length of data to be transferred */
    ret = mv_sdio_read_scratch(priv, &buf_len);
    if (ret < 0) {
        PRINTM(WARN, "Failed to read the scratch reg\n");
        umd_dbg("Failed to read the scratch reg\n");
        ret = WLAN_STATUS_FAILURE;
        goto exit;
    }

    PRINTM(INFO, "Receiving %d bytes from card at scratch reg value\n",
           buf_len);
    if (buf_len - SDIO_HEADER_LEN <= 0 || buf_len > npayload) {
        PRINTM(WARN, "Invalid packet size from firmware, size = %d\n",
               buf_len);
        ret = WLAN_STATUS_FAILURE;
        goto exit;
//              if (buf_len > npayload + 4)
//                      buf_len = npayload + 4;
    }

    /* Allocate buffer */
    blksz = SD_BLOCK_SIZE;
    buf_block_len = (buf_len + blksz - 1) / blksz;

    /* The host polls for the IO_READY bit */
    ret = mv_sdio_poll_card_status(priv, CARD_IO_READY);
    if (ret < 0) {
        PRINTM(WARN, "<1> Poll failed in card_to_host : %d\n", ret);
        umd_dbg("Error!! Poll failed in card_to_host : %d\n", ret);
        ret = WLAN_STATUS_FAILURE;
        goto exit;
    }

    ret = sbi_read_iomem(priv, priv->wlan_dev.ioport,
                         BLOCK_MODE, FIXED_ADDRESS, buf_block_len,
                         blksz, payload);

    if (ret < 0) {
        umd_dbg("Error!! sdio_read_iomem failed - mv_sdio_card_to_host\n");
        ret = WLAN_STATUS_FAILURE;
        goto exit;
    }
    *nb = buf_len;

    if (*nb <= 0) {
        PRINTM(INFO, "Null packet recieved \n");
        umd_dbg("Null packet recieved \n");
        ret = WLAN_STATUS_FAILURE;
        goto exit;
    }

    *type = (payload[2] | (payload[3] << 8));
    if (*type == MVSD_EVENT) {
        pevent = (u32 *) & payload[4];
        priv->adapter->EventCause = MVSD_EVENT | (((u16) (*pevent)) << 3);
    }
  exit:
    LEAVE();
    return ret;
}

/*
 * Read from the special scratch 'port'.
 */

int
mv_sdio_read_scratch(wlan_private * priv, u16 * dat)
{
    int ret = WLAN_STATUS_SUCCESS;
    u8 scr0;
    u8 scr1;

    ENTER();
    ret = sbi_read_ioreg(priv, CARD_OCR_0_REG, &scr0);
    if (ret < 0)
        return WLAN_STATUS_FAILURE;
    PRINTM(INFO, "SCRATCH_0_REG = %x\n", scr0);

    ret = sbi_read_ioreg(priv, CARD_OCR_1_REG, &scr1);
    if (ret < 0)
        return WLAN_STATUS_FAILURE;
    PRINTM(INFO, "SCRATCH_1_REG = %x\n", scr1);

    *dat = (((u16) scr1) << 8) | scr0;
    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/*
 * 	Get the CIS Table 
 */
int
sbi_get_cis_info(wlan_private * priv)
{
    wlan_adapter *Adapter = priv->adapter;
    u8 tupledata[255];
    char cisbuf[512];
    int ofs = 0, i;
    u32 ret = WLAN_STATUS_SUCCESS;
	u32 cis_addr = CIS_8686_PTR;
    struct sdio_func *func = (struct sdio_func*)(priv->wlan_dev.card);

    //card_info = (struct sdio_card *) slot->card->prot_card;
    //ret = card_info->ccis.manufacturer;
	// feng: need figure out how to get the basic card info for SDIO card

    /* Read the Tuple Data */
    for (i = 0; i < sizeof(tupledata); i++) {
        ret = sdio_card_readb(func, cis_addr + i, &tupledata[i]);
        if (ret < 0) {
            PRINTM(WARN, "sbi_get_cis_info failed!!!\n");
            return WLAN_STATUS_FAILURE;
        }
    }

    memset(cisbuf, 0x0, sizeof(cisbuf));
    memcpy(cisbuf + ofs, tupledata, sizeof(cisbuf));

    /* Copy the CIS Table to Adapter */
    memset(Adapter->CisInfoBuf, 0x0, sizeof(cisbuf));
    memcpy(Adapter->CisInfoBuf, &cisbuf, sizeof(cisbuf));
    Adapter->CisInfoLen = sizeof(cisbuf);

    LEAVE();
    return WLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function downloads firmware image to the card.
 *
 *  @param priv         A pointer to wlan_private structure
 *  @param firmware     A pointer to firmware image buffer
 *  @param firmwarelen  the length of firmware image
 *  @return             WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int
sbi_download_wlan_fw_image(wlan_private * priv,
                           const u8 * firmware, int firmwarelen)
{
    u8 base0;
    u8 base1;
    int ret = WLAN_STATUS_SUCCESS;
    int offset;
    u8 *fwbuf = priv->adapter->TmpTxBuf;
    int timeout = 5000;
    u16 len;
    int txlen = 0;
    int tx_blocks = 0;
#ifdef FW_DOWNLOAD_SPEED
    u32 tv1, tv2;
#endif

    ENTER();

    PRINTM(INFO, "WLAN_FW: Downloading firmware of size %d bytes\n",
           firmwarelen);
#ifdef FW_DOWNLOAD_SPEED
    tv1 = get_utimeofday();
#endif

    /* Wait initially for the first non-zero value */
    do {
        if ((ret = sbi_read_ioreg(priv, HOST_F1_RD_BASE_0, &base0)) < 0) {
            PRINTM(WARN, "Dev BASE0 register read failed:"
                   " base0=0x%04X(%d)\n", base0, base0);
            ret = WLAN_STATUS_FAILURE;
            goto done;
        }
        if ((ret = sbi_read_ioreg(priv, HOST_F1_RD_BASE_1, &base1)) < 0) {
            PRINTM(WARN, "Dev BASE1 register read failed:"
                   " base1=0x%04X(%d)\n", base1, base1);
            ret = WLAN_STATUS_FAILURE;
            goto done;
        }
        len = (((u16) base1) << 8) | base0;
        mdelay(1);
    } while (!len && --timeout);

    if (!timeout) {
        PRINTM(MSG, "Helper downloading finished.\n");
        PRINTM(MSG, "Timeout for firmware downloading!\n");
        ret = WLAN_STATUS_FAILURE;
        goto done;
    }

    /* The host polls for the DN_LD_CARD_RDY and IO_READY bits */
    ret = mv_sdio_poll_card_status(priv, CARD_IO_READY | DN_LD_CARD_RDY);
    if (ret < 0) {
        PRINTM(FATAL, "Firmware download died @ the end\n");
        ret = WLAN_STATUS_FAILURE;
        goto done;
    }

    PRINTM(INFO, "WLAN_FW: Len got from firmware = 0x%04X(%d)\n", len, len);
    len &= ~B_BIT_0;

    /* Perform firmware data transfer */
    for (offset = 0; offset < firmwarelen; offset += txlen) {
        txlen = len;

        /* Set blocksize to transfer - checking for last block */
        if (firmwarelen - offset < txlen) {
            txlen = firmwarelen - offset;
	    PRINTM(INFO, "\n\nWLAN_FW: last block !!! txlen = %d\n\n\n",txlen);
        }
//         PRINTM(INFO, "WLAN_FW: offset=%d, txlen = 0x%04X(%d)\n",
//                offset, txlen, txlen);

        /* The host polls for the DN_LD_CARD_RDY and IO_READY bits */
        ret = mv_sdio_poll_card_status(priv, CARD_IO_READY | DN_LD_CARD_RDY);
        if (ret < 0) {
            PRINTM(FATAL, "Firmware download died @ %d\n", offset);
            goto done;
        }

        tx_blocks = (txlen + SD_BLOCK_SIZE_FW_DL - 1) / SD_BLOCK_SIZE_FW_DL;
//         PRINTM(INFO, "WLAN_FW:     tx_blocks = 0x%04X(%d)\n",
//                tx_blocks, tx_blocks);

        /* Copy payload to buffer */
        memcpy(fwbuf, &firmware[offset], txlen);

        /* Send data */
        ret = sbi_write_iomem(priv, priv->wlan_dev.ioport, BLOCK_MODE,
                              FIXED_ADDRESS, tx_blocks, SD_BLOCK_SIZE_FW_DL,
                              fwbuf);

        if (ret < 0) {
            PRINTM(FATAL, "IO error:transferring @ %d\n", offset);
            goto done;
        }

        do {
            udelay(10);
            if ((ret = sbi_read_ioreg(priv, HOST_F1_CARD_RDY, &base0)) < 0) {
                PRINTM(WARN, "Dev CARD_RDY register read failed:"
                       " base0=0x%04X(%d)\n", base0, base0);
                ret = WLAN_STATUS_FAILURE;
                goto done;
            }
            PRINTM(INFO, "offset=0x%08X len=0x%04X: "
                   "HOST_F1_CARD_RDY: 0x%04X\n", offset, txlen, base0);
        } while (!(base0 & 0x08) || !(base0 & 0x01));

        if ((ret = sbi_read_ioreg(priv, HOST_F1_RD_BASE_0, &base0)) < 0) {
            PRINTM(WARN, "Dev BASE0 register read failed:"
                   " base0=0x%04X(%d)\n", base0, base0);
            ret = WLAN_STATUS_FAILURE;
            goto done;
        }
        if ((ret = sbi_read_ioreg(priv, HOST_F1_RD_BASE_1, &base1)) < 0) {
            PRINTM(WARN, "Dev BASE1 register read failed:"
                   " base1=0x%04X(%d)\n", base1, base1);
            ret = WLAN_STATUS_FAILURE;
            goto done;
        }
        len = (((u16) base1) << 8) | base0;

        if (!len) {
            PRINTM(INFO, "WLAN Firmware Download Over\n");
            break;
        }

        if (len & B_BIT_0) {
            PRINTM(INFO, "CRC32 Error indicated by the helper:"
                   " len=0x%04X(%d)\n", len, len);
            len &= ~B_BIT_0;
            /* Setting this to 0 to resend from same offset */
            txlen = 0;
        } else {
            PRINTM(INFO, "%d,%d bytes block of firmware downloaded, new len from helper=%d\n",
                   offset, txlen,len);
        }
    }

    PRINTM(INFO, "Firmware Image of Size %d bytes downloaded\n", firmwarelen);

    ret = WLAN_STATUS_SUCCESS;
  done:
#ifdef FW_DOWNLOAD_SPEED
    tv2 = get_utimeofday();
    PRINTM(INFO, "firmware: %ld.%03ld.%03ld ", tv1 / 1000000,
           (tv1 % 1000000) / 1000, tv1 % 1000);
    PRINTM(INFO, " -> %ld.%03ld.%03ld ", tv2 / 1000000,
           (tv2 % 1000000) / 1000, tv2 % 1000);
    tv2 -= tv1;
    PRINTM(INFO, " == %ld.%03ld.%03ld\n", tv2 / 1000000,
           (tv2 % 1000000) / 1000, tv2 % 1000);
#endif
    LEAVE();
    return ret;
}

static int
sbi_prog_firmware_image(wlan_private * priv, const u8 * firmware,
                        int firmwarelen)
{
    int ret = WLAN_STATUS_SUCCESS;
    u16 firmwarestat;
    u8 *fwbuf = priv->adapter->TmpTxBuf;
    int fwblknow;
    u32 tx_len;
#ifdef FW_DOWNLOAD_SPEED
    unsigned long tv1, tv2;
#endif

//         struct sdio_func *func = (struct sdio_func*)(priv->wlan_dev.card);
//         u8 bic;

	//sdio_claim_host(func);
       //sdio_dump_cccr(func);
	//sdio_release_host(func);

        /* feng: here we need add the irq hook */

    /* Check if the firmware is already downloaded */
    if ((ret = mv_sdio_read_scratch(priv, &firmwarestat)) < 0) {
        PRINTM(INFO, "read scratch returned <0\n");
        goto done;
    }

    if (firmwarestat == FIRMWARE_READY) {
        PRINTM(INFO, "Firmware already downloaded!\n");
        /* TODO: We should be returning success over here */
        ret = WLAN_STATUS_SUCCESS;
        goto done;
    }

    PRINTM(INFO, "Downloading helper image (%d bytes), block size %d bytes ",
           firmwarelen, SD_BLOCK_SIZE_FW_DL);

    umd_dbg("Downloading helper image (%d bytes), block size %d bytes ",
           firmwarelen, SD_BLOCK_SIZE_FW_DL);

#ifdef FW_DOWNLOAD_SPEED
    tv1 = get_utimeofday();
#endif
    /* Perform firmware data transfer */
    tx_len =
        (FIRMWARE_TRANSFER_NBLOCK * SD_BLOCK_SIZE_FW_DL) - SDIO_HEADER_LEN;
    PRINTM(INFO, "tx len = %d\n ", tx_len);
    for (fwblknow = 0; fwblknow < firmwarelen; fwblknow += tx_len) {

        /* The host polls for the DN_LD_CARD_RDY and IO_READY bits */
        ret = mv_sdio_poll_card_status(priv, CARD_IO_READY | DN_LD_CARD_RDY);
        if (ret < 0) {
            PRINTM(INFO, "Firmware download died @ %d\n", fwblknow);
            goto done;
        }

        /* Set blocksize to transfer - checking for last block */
        if (firmwarelen - fwblknow < tx_len)
            tx_len = firmwarelen - fwblknow;

        fwbuf[0] = ((tx_len & 0x000000ff) >> 0);        /* Little-endian */
        fwbuf[1] = ((tx_len & 0x0000ff00) >> 8);
        fwbuf[2] = ((tx_len & 0x00ff0000) >> 16);
        fwbuf[3] = ((tx_len & 0xff000000) >> 24);

        /* Copy payload to buffer */
        memcpy(&fwbuf[SDIO_HEADER_LEN], &firmware[fwblknow], tx_len);

        PRINTM(INFO, ".");

        /* Send data */
        ret = sbi_write_iomem(priv, priv->wlan_dev.ioport, BLOCK_MODE,
                              FIXED_ADDRESS, FIRMWARE_TRANSFER_NBLOCK,
                              SD_BLOCK_SIZE_FW_DL, fwbuf);

        if (ret) {
            PRINTM(INFO, "IO error: transferring block @ %d\n", fwblknow);
            goto done;
        }
    }

    PRINTM(INFO, "\ndone (%d/%d bytes)\n", fwblknow, firmwarelen);
#ifdef FW_DOWNLOAD_SPEED
    tv2 = get_utimeofday();
    PRINTM(INFO, "helper: %ld.%03ld.%03ld ", tv1 / 1000000,
           (tv1 % 1000000) / 1000, tv1 % 1000);
    PRINTM(INFO, " -> %ld.%03ld.%03ld ", tv2 / 1000000,
           (tv2 % 1000000) / 1000, tv2 % 1000);
    tv2 -= tv1;
    PRINTM(INFO, " == %ld.%03ld.%03ld\n", tv2 / 1000000,
           (tv2 % 1000000) / 1000, tv2 % 1000);
#endif

    /* Write last EOF data */
    PRINTM(INFO, "Transferring EOF block\n");
    memset(fwbuf, 0x0, SD_BLOCK_SIZE_FW_DL);
    ret = sbi_write_iomem(priv, priv->wlan_dev.ioport, BLOCK_MODE,
                          FIXED_ADDRESS, 1, SD_BLOCK_SIZE_FW_DL, fwbuf);
    if (ret < 0) {
        PRINTM(INFO, "IO error in writing EOF firmware block\n");
        goto done;
    }

    ret = WLAN_STATUS_SUCCESS;

  done:
    LEAVE();
    return ret;
}

int
sbi_prog_helper(wlan_private * priv)
{
#ifdef USE_FIRMWARE_KERNEL_MODULE
    if (priv->fw_helper) {
        return sbi_prog_firmware_image(priv,
                                       priv->fw_helper->data,
                                       priv->fw_helper->size);
    } else {
        PRINTM(MSG, "No hotplug helper image\n");
        return WLAN_STATUS_FAILURE;
    }
#else
    return sbi_prog_firmware_image(priv,
    priv->myfw_helper.data,
    priv->myfw_helper.size);
#endif
}

int
sbi_prog_firmware_w_helper(wlan_private * priv)
{
#ifdef USE_FIRMWARE_KERNEL_MODULE
    if (priv->firmware) {
        return sbi_download_wlan_fw_image(priv,
					  priv->firmware->data,
					  priv->firmware->size);
                                        
    } else {
        PRINTM(MSG, "No hotplug firmware image\n");
        return WLAN_STATUS_FAILURE;
    }
#else
    return sbi_download_wlan_fw_image(priv,
    priv->myfirmware.data,
    priv->myfirmware.size);
#endif
}

int
sbi_prog_firmware(wlan_private * priv)
{
#ifdef USE_FIRMWARE_KERNEL_MODULE
	if (priv->firmware) {
        return sbi_download_wlan_fw_image(priv,
					  priv->firmware->data,
					  priv->firmware->size);
    } else {
        PRINTM(MSG, "No hotplug firmware image\n");
        return WLAN_STATUS_FAILURE;
    }
#else
    return sbi_download_wlan_fw_image(priv,
    priv->myfirmware.data,
    priv->myfirmware.size);
#endif
}

int
sbi_set_bus_clock(wlan_private * priv, u8 option)
{
    return WLAN_STATUS_SUCCESS;
}

int
sbi_exit_deep_sleep(wlan_private * priv)
{
    int ret = WLAN_STATUS_SUCCESS;
    PRINTM(INFO,
           "Trying to wakeup device... Conn=%d IntC=%d PS_Mode=%d PS_State=%d\n",
           priv->adapter->MediaConnectStatus, priv->adapter->IntCounter,
           priv->adapter->PSMode, priv->adapter->PSState);
//      sbi_set_bus_clock(priv, TRUE);

    if (priv->adapter->fwWakeupMethod == WAKEUP_FW_THRU_GPIO) {
//              GPIO_PORT_TO_LOW();
    } else
        ret = sbi_write_ioreg(priv, CONFIGURATION_REG, HOST_POWER_UP);

    return ret;
}

int
sbi_reset_deepsleep_wakeup(wlan_private * priv)
{

    int ret = WLAN_STATUS_SUCCESS;

    ENTER();

    if (priv->adapter->fwWakeupMethod == WAKEUP_FW_THRU_GPIO) {
        GPIO_PORT_TO_HIGH();
    } else
        ret = sbi_write_ioreg(priv, CONFIGURATION_REG, 0);

    LEAVE();

    return ret;
}

int
sbi_verify_fw_download(wlan_private * priv)
{
    int ret;
    u16 firmwarestat;
    int tries;
    u8 rsr;

    ENTER();
    /* Wait for firmware initialization event */
    for (tries = 0; tries < MAX_FIRMWARE_POLL_TRIES; tries++) {
        if ((ret = mv_sdio_read_scratch(priv, &firmwarestat)) < 0)
            continue;

        if (firmwarestat == FIRMWARE_READY) {
            ret = 0;
            PRINTM(INFO, "Firmware successfully downloaded\n");
            break;
        } else {
            mdelay(10);
            ret = -ETIMEDOUT;
        }
    }

    if (ret < 0) {
        PRINTM(INFO, "Timeout waiting for firmware to become active\n");
        goto done;
    }

    ret = sbi_read_ioreg(priv, HOST_INT_RSR_REG, &rsr);
    if (ret < 0) {
        PRINTM(INFO, "sdio_read_ioreg: reading INT RSR register failed\n");
        return -1;
    } else
        PRINTM(INFO, "sdio_read_ioreg: RSR register 0x%x\n", rsr);

    ret = 0;
  done:
    LEAVE();
    return ret;
}

#ifdef DEEP_SLEEP_XC

//mod by xc

/*extern int start_bus_clock(mmc_controller_t);
extern int stop_bus_clock_2(mmc_controller_t);
*/
int
sbi_enter_deep_sleep(wlan_private * priv)
{
    int ret;

    sbi_write_ioreg(priv, CONFIGURATION_REG, 0);
    mdelay(2);
    ret = sbi_write_ioreg(priv, CONFIGURATION_REG, HOST_POWER_DOWN);
//mod by xc
    //stop_bus_clock_2(((mmc_card_t)((priv->wlan_dev).card))->ctrlr);
    mdelay(2);

    return ret;
}

int
sbi_exit_deep_sleep(wlan_private * priv)
{
    int ret = 0;

    PRINTM(INFO,
           "Trying to wakeup device... Conn=%d IntC=%d PS_Mode=%d PS_State=%d\n",
           priv->adapter->MediaConnectStatus, priv->adapter->IntCounter,
           priv->adapter->PSMode, priv->adapter->PSState);
    //mod by xc
    //start_bus_clock(((mmc_card_t)((priv->wlan_dev).card))->ctrlr);

    if (priv->adapter->fwWakeupMethod == WAKEUP_FW_THRU_GPIO) {
        GPIO_PORT_TO_LOW();
    } else                      // SDIO method 
        ret = sbi_write_ioreg(priv, CONFIGURATION_REG, HOST_POWER_UP);

    return ret;
}

int
sbi_reset_deepsleep_wakeup(wlan_private * priv)
{
    ENTER();

    int ret = 0;

    if (priv->adapter->fwWakeupMethod == WAKEUP_FW_THRU_GPIO) {
        GPIO_PORT_TO_HIGH();
    } else                      // SDIO method 
        ret = sbi_write_ioreg(priv, CONFIGURATION_REG, 0);

    LEAVE();

    return ret;
}
#endif /* DEEP_SLEEP */

#ifdef CONFIG_MARVELL_PM
inline int
sbi_suspend(wlan_private * priv)
{
    int ret;

    ENTER();

    ret = sdio_suspend(priv->wlan_dev.card);

    LEAVE();
    return ret;
}

inline int
sbi_resume(wlan_private * priv)
{
    int ret;

    ENTER();

    ret = sdio_resume(priv->wlan_dev.card);

    LEAVE();
    return ret;
}
#endif

MODULE_LICENSE("GPL");
