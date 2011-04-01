#include <linux/module.h>
#include <linux/types.h>

#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/ceata.h>
#include <linux/mmc/core.h>
#include <linux/libata.h>
#include <linux/scatterlist.h>

#include "core.h"
#include "mmc_ops.h"

static int ccs_disable(struct mmc_card *card)
{
	int err;
	struct mmc_command cmd;

	BUG_ON(!card);

printk("ccs_disable\n");
	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = 0;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_NONE | MMC_CMD_BC;

	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err != MMC_ERR_NONE)
	{
		printk("ccs disable failed\n");
	}
printk("ccs_disable done\n");
	
	return err;
}

int ce_ata_stop_transmission(struct mmc_card *card, int rw)
{
	int err;
	struct mmc_command cmd;
	int retries = 100;
	
printk("stop_transmission\n");
	BUG_ON(!card);

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = MMC_STOP_TRANSMISSION;
	cmd.arg = 0;
	
	if (rw)
		cmd.flags = MMC_RSP_R1B | MMC_CMD_AC;
	else
		cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
	
	mmc_claim_host(card->host);

	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err != MMC_ERR_NONE)
		goto err;

	while(retries) {
		u32 status;

		err = mmc_send_status(card, &status);
		if (err != MMC_ERR_NONE)
			goto err;

		printk("cmd13 -> status: %08x, %04x\n", status, (status >> 9) & 0xffff);

		if (((status >> 9) & 0xffff) == 0x0004) // tran state
			break;
		
		msleep(1);
		retries--;
	}

err:
	mmc_release_host(card->host);
	return err;
} 

EXPORT_SYMBOL(ce_ata_stop_transmission);

int ceata_rw_multiple_register(struct mmc_card *card, int rwFlag, unsigned char addr, u8 *buf, unsigned char count)
{
	struct mmc_request mrq;
	struct mmc_command cmd;
	struct mmc_data data;
	struct scatterlist sg;

//printk("ce_ata_rw_multiple_register\n");
	BUG_ON(!card);
	BUG_ON(!card->host);
	BUG_ON(!buf);

	memset(&mrq, 0, sizeof(struct mmc_request));
	memset(&cmd, 0, sizeof(struct mmc_command));
	memset(&data, 0, sizeof(struct mmc_data));

	mrq.cmd = &cmd;
	mrq.data = &data;

	cmd.opcode = CE_ATA_RW_MULTIPLE_REGISTER;
	cmd.arg = ((rwFlag<<31) | (((unsigned long)addr & ~0x3UL) << 16) | ((unsigned long)count & ~0x3UL) << 0);
	if (rwFlag)
		cmd.flags = MMC_RSP_R1B;
	else
		cmd.flags = MMC_RSP_R1;
	data.blksz = count;
	data.blocks = 1;
	if (rwFlag)
		data.flags = MMC_DATA_WRITE;
	else
		data.flags = MMC_DATA_READ;
	data.sg = &sg;
	data.sg_len = 1;

	mmc_claim_host(card->host);
	
	sg_init_one(&sg, buf, count);

	mmc_set_data_timeout(&data, card);

	mmc_polling_request(card->host, &mrq);
	//mmc_wait_for_req(card->host, &mrq);
	
	mmc_release_host(card->host);

	if (cmd.error != MMC_ERR_NONE)
		return cmd.error;
	if (data.error != MMC_ERR_NONE)
		return data.error;

	return MMC_ERR_NONE;
}

EXPORT_SYMBOL(ceata_rw_multiple_register);

int ceata_rw_multiple_block(struct mmc_card *card, int rwFlag, u8 *buf, u16 data_unit_count)
{
	struct mmc_request mrq;
	struct mmc_command cmd;
	struct mmc_data data;
	struct scatterlist sg;

//printk("ce_ata_rw_multiple_block\n");
	BUG_ON(!card);
	BUG_ON(!card->host);
	BUG_ON(!buf);

	memset(&mrq, 0, sizeof(struct mmc_request));
	memset(&cmd, 0, sizeof(struct mmc_command));
	memset(&data, 0, sizeof(struct mmc_data));

	mrq.cmd = &cmd;
	mrq.data = &data;

	cmd.opcode = CE_ATA_RW_MULTIPLE_BLOCK;
	cmd.arg = ((rwFlag<<31) | data_unit_count);
	if (rwFlag)
		cmd.flags = MMC_RSP_R1B;
	else
		cmd.flags = MMC_RSP_R1;
	
	data.blksz = 512;
	data.blocks = data_unit_count;
	if (rwFlag)
		data.flags = MMC_DATA_WRITE;
	else
		data.flags = MMC_DATA_READ;
	data.sg = &sg;
	data.sg_len = 1;

	//cmd.flags |= MMC_RSP_CEATA_CCS_EN;
	
	mmc_claim_host(card->host);
	
	sg_init_one(&sg, buf, data_unit_count * 512);
	mmc_set_data_timeout(&data, card);
	mmc_wait_for_req(card->host, &mrq);

	mmc_release_host(card->host);

	if (cmd.error != MMC_ERR_NONE)
		return cmd.error;
	if (data.error != MMC_ERR_NONE)
		return data.error;

	return MMC_ERR_NONE;
}

EXPORT_SYMBOL(ceata_rw_multiple_block);

int ceata_rw_sg(struct mmc_card *card, int rw, struct ata_queued_cmd *qc)
{
	struct mmc_request mrq;
	struct mmc_command cmd;
	struct mmc_data data;
	unsigned int data_unit_count;

	int err = 0;

	memset(&mrq, 0, sizeof(struct mmc_request));
	memset(&cmd, 0, sizeof(struct mmc_command));
	memset(&data, 0, sizeof(struct mmc_data));

	mrq.cmd = &cmd;
	mrq.data = &data;

	data_unit_count = qc->nbytes / qc->sect_size;

	cmd.opcode = CE_ATA_RW_MULTIPLE_BLOCK;
	cmd.arg = (rw ? (1UL<<31):0) | data_unit_count;
	if (rw)
		cmd.flags = MMC_RSP_R1B;
	else
		cmd.flags = MMC_RSP_R1;

	data.blksz = qc->sect_size;
	data.blocks = data_unit_count;
	if (rw)
		data.flags = MMC_DATA_WRITE;
	else
		data.flags = MMC_DATA_READ;

	data.sg = qc->__sg;
	data.sg_len = qc->n_elem;

	//cmd.flags |= MMC_RSP_CEATA_CCS_EN;

	err = MMC_ERR_NONE;

	mmc_claim_host(card->host);

	mmc_set_data_timeout(&data, card);
	mmc_wait_for_req(card->host, &mrq);

	mmc_release_host(card->host);

	if (cmd.error != MMC_ERR_NONE) {
		err = cmd.error;
		printk("cmd error");
		return err;
	}

	if (data.error != MMC_ERR_NONE)
		err = data.error;

	return err;
}
EXPORT_SYMBOL(ceata_rw_sg);

int ceata_set_clock(struct mmc_card *card, int dsor)
{
	unsigned int max_dtr = (unsigned int)-1;

	if (dsor == 0)
		return -EINVAL;

	if (mmc_card_highspeed(card)) {
		if (max_dtr > card->ext_csd.hs_max_dtr)
			max_dtr = card->ext_csd.hs_max_dtr;
	} else if (max_dtr > card->csd.max_dtr) {
		max_dtr = card->csd.max_dtr;
	}

	max_dtr = max_dtr / dsor;
	mmc_set_clock(card->host, max_dtr);

	return 0;
}
EXPORT_SYMBOL(ceata_set_clock);
