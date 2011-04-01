#ifndef _MMC_CE_ATA_OPS_H
#define _MMC_CE_ATA_OPS_H

int ce_ata_rw_multiple_register(struct mmc_card *card, int rwFlag, unsigned char addr, u8 *buf, unsigned char count);
int ce_ata_rw_multiple_block(struct mmc_card *card, int rwFlag, u8 *buf, u16 data_unit_count);
int ce_ata_stop_transmission(struct mmc_card *card, int rw);

#endif
