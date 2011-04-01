#include <linux/moduleparam.h>
#include <linux/module.h>
#include <linux/init.h>

#include <linux/platform_device.h>
#include <asm/arch/board.h>

#include <linux/mmc/mmc.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/hdd_dpm.h>
#include <linux/completion.h>

#include "libata-ceata.h"

#define DRV_MMC_NAME "mmc_ceata"

struct ceata_archosg6 {
	struct ceata_host host;
	struct hdd_dpm_ops dpm_ops;
	int init;
	struct completion *wait;
};

static struct ceata_archosg6 ceata_archosg6;

#define to_archos_ce_ata(p) container_of(p, struct ceata_archosg6, dpm_ops)

extern int ceata_rw_multiple_register(struct mmc_card *card, int rwFlag, unsigned char addr, u8 *buf, unsigned char count);
extern int mmc_fast_io(struct mmc_card *card, int rw, u8 addr, u8 *data);
extern void ceatahdd_power(int on_off);

static u8 ceata_read_reg8(struct ceata_port *cp, u8 addr)
{
	int err;
	u8 val;
	
	if (cp == NULL || cp->mmc == NULL || cp->mmc->card == NULL)
		return 0xff;
		
	mmc_claim_host(cp->mmc);

	err = mmc_fast_io(cp->mmc->card, 0, addr, &val);			
	if (err != MMC_ERR_NONE)
		val = 0xff;
	
	mmc_release_host(cp->mmc);
	return val;
}

static int ceata_standby(struct ceata_port *cp)
{
	int err;
	u8 *tf;
	
	if (cp->mmc == NULL)
		return 0;

	tf = kzalloc(16, GFP_KERNEL);
	if (tf == NULL)
		return -1;
		
	tf[6] = 0x02;
	tf[15] = ATA_CMD_STANDBYNOW1;
	
	err = ceata_rw_multiple_register(cp->mmc->card, 1, 0, tf, 16);

	kfree(tf);
	
	if (err == MMC_ERR_BADCRC)
		return -1;
	
	while(1) {
		if (cp->ap) {
			u8 status = ceata_read_reg8(cp, 0x0f);
			if (status != 0xff && !(status & 0x80))
				break;
		}
		msleep(10);
	}
	
	return 0;
}

static int ceata_dpm_suspend(struct hdd_dpm_ops *dpm_ops)
{
	struct ceata_archosg6 *priv = to_archos_ce_ata(dpm_ops);
	struct ceata_port *cp = &priv->host.ports[0];
	
	if (dpm_ops->pm_state == PM_SUSPEND_STANDBY)
		return 0;		

	printk("ceata_dpm_suspend\n");

	if (cp->hd_access > 0)
		return -EAGAIN;

	dpm_ops->pm_state = -1;

	if (ceata_standby(cp) != 0)
		printk(KERN_ERR "ceata_standby failed\n");
	
	mmc_suspend_host(cp->mmc, PMSG_SUSPEND);

	ceatahdd_power(0);

	dpm_ops->pm_state = PM_SUSPEND_STANDBY;
	return 0;
}

static int ceata_dpm_resume(struct hdd_dpm_ops *dpm_ops)
{
	struct ceata_archosg6 *priv = to_archos_ce_ata(dpm_ops);
	struct ceata_port *cp = &priv->host.ports[0];
	DECLARE_COMPLETION_ONSTACK(wait);

	printk("ceata_dpm_resume\n");

	if (cp->mmc == NULL)
		return 0;
	
	dpm_ops->pm_state = -1;
		
	priv->wait = &wait;

	ceatahdd_power(1);
	mmc_resume_host(cp->mmc);
	wait_for_completion(&wait);
	
	dpm_ops->pm_state = PM_SUSPEND_ON;
	priv->wait = NULL;

	return 0;
}

static unsigned long ceata_dpm_iocount(struct hdd_dpm_ops *dpm_ops)
{
	struct ceata_archosg6 *priv = to_archos_ce_ata(dpm_ops);
	struct ceata_port *cp = &priv->host.ports[0];
	return cp->io_count;
}

static void ceata_dpm_wakeup_hook(struct ceata_port *cp)
{
	struct ceata_archosg6 *priv = (struct ceata_archosg6 *)cp->private_data;
	
	while(priv->dpm_ops.pm_state == -1)
		msleep(10);
	
	if (priv->dpm_ops.pm_state != PM_SUSPEND_ON)
		ceata_dpm_resume(&priv->dpm_ops);
}

static struct ceata_port ceata_port = {
	.mmc_id = "mmc0",
	.wake_up_hook = ceata_dpm_wakeup_hook,
};

static int ceata_dev_probe(struct mmc_card *card)
{
	int err;
	struct ceata_archosg6 *priv = &ceata_archosg6;
	struct mmc_host *mmc = card->host;

	printk("ceata_dev_probe\n");
	/*
	 * Check that the card supports the command class(es) we need.
	 */
	if (card->type != MMC_TYPE_CE_ATA)
		return -ENODEV;
		
	if ((err = ceata_add_port(&priv->host, mmc, priv->init)) != 0) {
		printk(KERN_ERR "CE-ATA drive port not added\n");
		return -ENODEV;
	}		

	if (priv->init) {
		priv->dpm_ops.suspend = ceata_dpm_suspend;
		priv->dpm_ops.resume  = ceata_dpm_resume;
		priv->dpm_ops.get_iocount = ceata_dpm_iocount;
		priv->dpm_ops.pm_state = PM_SUSPEND_ON;
		strlcpy(priv->dpm_ops.name, "sda", sizeof(priv->dpm_ops.name));
		hdd_dpm_register_dev(&priv->dpm_ops);
	}
	else {
		/* if somebody is waiting for us to finish probing,
		 * wake him up.
		 */
		if (priv->wait)
			complete(priv->wait);
	}
	
	priv->init = 0;
	
	return 0;
}

static void ceata_dev_remove(struct mmc_card *card)
{
	struct ceata_archosg6 *priv = &ceata_archosg6;
	struct mmc_host *mmc = card->host;
	
	if (priv->init) {
		hdd_dpm_unregister_dev(&priv->dpm_ops);
		priv->dpm_ops.suspend = NULL;
	}
	
	ceata_del_port(&priv->host, mmc, priv->init);
	
	priv->init = 0;
}

#ifdef CONFIG_PM
static int ceata_dev_suspend(struct mmc_card *card, pm_message_t state)
{
	printk("ceata_dev_suspend\n");
	ceatahdd_power(0);
	return 0;
}

static int ceata_dev_resume(struct mmc_card *card)
{
	printk("ceata_dev_resume\n");
	ceatahdd_power(1);
	return 0;
}
#else
#define	ceata_dev_suspend	NULL
#define ceata_dev_resume	NULL
#endif

static struct mmc_driver ceata_mmc_device_driver = {
	.drv	= {
		.name	= 	DRV_MMC_NAME,
	},
	.probe		= ceata_dev_probe,
	.remove		= ceata_dev_remove,
};

static int __init ceata_archosg6_probe(struct platform_device *pdev)
{
	int err;

	ceata_archosg6.host.dev = &pdev->dev;
	ceata_archosg6.host.ports = &ceata_port;
	ceata_archosg6.host.n_ports = 1;
	ceata_archosg6.init = 1;
	ceata_port.private_data = (void*)&ceata_archosg6;

	ceatahdd_power(1);

	if ((err = ceata_register(&ceata_archosg6.host)) != 0)
		return err;

	if ((err = mmc_register_driver(&ceata_mmc_device_driver)) != 0) {
		ceata_unregister(&ceata_archosg6.host);
		return err;
	}
	
	return 0;
}

static int ceata_archosg6_remove(struct platform_device *pdev)
{
	ceata_dpm_wakeup_hook(&ceata_port);

	ceata_archosg6.init = 1;

	if (ceata_standby(&ceata_port) != 0)
		printk("ceata_standby failed\n");

	mmc_unregister_driver(&ceata_mmc_device_driver);
	ceata_unregister(&ceata_archosg6.host);
	return 0;
}

static struct platform_driver ceata_archosg6_driver = {
	.probe = ceata_archosg6_probe,
	.remove = ceata_archosg6_remove,
	.suspend	= ceata_dev_suspend,
	.resume		= ceata_dev_resume,
	.driver = {
		   .name = "g6_ceata",
		   .owner = THIS_MODULE,
	},
};

static int __init ceata_drive_init(void)
{
	/* Register the CE-ATA driver */
	if (platform_driver_register(&ceata_archosg6_driver)) {
	  	printk(KERN_ERR ":failed to register MMC driver\n");
	  	return -ENODEV;
	}
	return 0;
}

static void __exit ceata_drive_exit(void)
{
	platform_driver_unregister(&ceata_archosg6_driver);
}

module_init(ceata_drive_init);
module_exit(ceata_drive_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Archos G6 CE-ATA device driver");
