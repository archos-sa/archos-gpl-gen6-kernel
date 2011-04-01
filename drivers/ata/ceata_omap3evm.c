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

struct ceata_omap3evm {
	struct ceata_host host;
	struct hdd_dpm_ops dpm_ops;
	int init;
	struct completion *wait;
};

static struct ceata_omap3evm ceata_omap3evm;

#define to_omap3evm_ce_ata(p) container_of(p, struct ceata_omap3evm, dpm_ops)

extern int ceata_rw_multiple_register(struct mmc_card *card, int rwFlag, unsigned char addr, u8 *buf, unsigned char count);
extern int mmc_fast_io(struct mmc_card *card, int rw, u8 addr, u8 *data);

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

static inline int ceata_standby(struct ceata_port *cp)
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
	if (err != MMC_ERR_NONE)
	{
		if (err == MMC_ERR_BADCRC)
			return -1;
	}
	
	kfree(tf);

	while(1)
	{
		if (cp->ap)
		{
			u8 status = ceata_read_reg8(cp, 0x0f);
//printk("status: %02x\n", status);
			if (status != 0xff && !(status & 0x80))
				break;
		}
		msleep(10);
	}
	
	return 0;
}

static int ceata_dpm_suspend(struct hdd_dpm_ops *dpm_ops)
{
	struct ceata_omap3evm *priv = to_omap3evm_ce_ata(dpm_ops);
	struct ceata_port *cp = &priv->host.ports[0];
	
//printk("%s called\n", __FUNCTION__);
	
	if (dpm_ops->pm_state == PM_SUSPEND_STANDBY)
		return 0;		
	else if (cp->hd_access > 0)
		return -EAGAIN;

	dpm_ops->pm_state = -1;

	if (ceata_standby(cp) != 0)
		printk("ceata_standby failed\n");
	
	msleep(1000);

	mmc_suspend_host(cp->mmc, PMSG_SUSPEND);

	dpm_ops->pm_state = PM_SUSPEND_STANDBY;
	return 0;
}

static int ceata_dpm_resume(struct hdd_dpm_ops *dpm_ops)
{
	struct ceata_omap3evm *priv = to_omap3evm_ce_ata(dpm_ops);
	struct ceata_port *cp = &priv->host.ports[0];
	DECLARE_COMPLETION_ONSTACK(wait);
	
//printk("%s called\n", __FUNCTION__);
	
	if (cp->mmc == NULL)
		return 0;
	
	dpm_ops->pm_state = -1;
		
	msleep(100);
	
	priv->wait = &wait;
	
	mmc_resume_host(cp->mmc);

#if 0
	int err = wait_for_completion_timeout(&wait, 10*HZ);
	
	if (!err) {
		printk("resume timeout\n");
	}
#else
	wait_for_completion(&wait);
#endif
	
	while(1)
	{
		if (cp->ap)
		{
			u8 status = ceata_read_reg8(cp, 0x0f);
//printk("status: %02x\n", status);
			if (status != 0xff && !(status & 0x80))
				break;
		}
		msleep(10);
	}
	
	dpm_ops->pm_state = PM_SUSPEND_ON;

	return 0;
}

static unsigned long ceata_dpm_iocount(struct hdd_dpm_ops *dpm_ops)
{
	struct ceata_omap3evm *priv = to_omap3evm_ce_ata(dpm_ops);
	struct ceata_port *cp = &priv->host.ports[0];
	return cp->io_count;
}

static void ceata_dpm_wakeup_hook(struct ceata_port *cp)
{
	struct ceata_omap3evm *priv = (struct ceata_omap3evm *)cp->private_data;
	
	while(priv->dpm_ops.pm_state == -1) {
		msleep(10);
	}
	
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
	struct ceata_omap3evm *priv = &ceata_omap3evm;
	struct mmc_host *mmc = card->host;

	/*
	 * Check that the card supports the command class(es) we need.
	 */
	if (card->type != MMC_TYPE_CE_ATA)
		return -ENODEV;
		
	if ((err = ceata_add_port(&priv->host, mmc, priv->init)) != 0)
	{
printk("CE-ATA drive port not added\n");
		return -ENODEV;
	}		

	if (priv->init)
	{
		priv->dpm_ops.suspend = ceata_dpm_suspend;
		priv->dpm_ops.resume  = ceata_dpm_resume;
		priv->dpm_ops.get_iocount = ceata_dpm_iocount;
		priv->dpm_ops.pm_state = PM_SUSPEND_ON;
		strlcpy(priv->dpm_ops.name, "sda", sizeof(priv->dpm_ops.name));
		hdd_dpm_register_dev(&priv->dpm_ops);
printk("CE-ATA drive power management registered\n");
	}
	else
		complete(priv->wait); // resume

	priv->init = 0;
	
	return 0;
}

static void ceata_dev_remove(struct mmc_card *card)
{
	struct ceata_omap3evm *priv = &ceata_omap3evm;
	struct mmc_host *mmc = card->host;
	
	if (priv->init)
	{
		hdd_dpm_unregister_dev(&priv->dpm_ops);
		priv->dpm_ops.suspend = NULL;
	}
	
	ceata_del_port(&priv->host, mmc, priv->init);
	
	priv->init = 0;
}

#ifdef CONFIG_PM
static int ceata_dev_suspend(struct mmc_card *card, pm_message_t state)
{
	return 0;
}

static int ceata_dev_resume(struct mmc_card *card)
{
	return 0;
}
#else
#define	ceata_dev_suspend	NULL
#define ceata_dev_resume	NULL
#endif

static struct mmc_driver ceata_mmc_device_driver = {
	.drv		= {
	.name	= 	DRV_MMC_NAME,
	},
	.probe		= ceata_dev_probe,
	.remove		= ceata_dev_remove,
	.suspend	= ceata_dev_suspend,
	.resume		= ceata_dev_resume,
};

static int __init ceata_omap3evm_probe(struct platform_device *pdev)
{
	int err;

	ceata_omap3evm.host.dev = &pdev->dev;
	ceata_omap3evm.host.ports = &ceata_port;
	ceata_omap3evm.host.n_ports = 1;
	ceata_omap3evm.init = 1;
	ceata_port.private_data = (void*)&ceata_omap3evm;
	
	if ((err = ceata_register(&ceata_omap3evm.host)) != 0)
		return err;

	if ((err = mmc_register_driver(&ceata_mmc_device_driver)) != 0)
	{
		ceata_unregister(&ceata_omap3evm.host);
		return err;
	}
	
	return 0;
}

static int ceata_omap3evm_remove(struct platform_device *pdev)
{
	ceata_dpm_wakeup_hook(&ceata_port);

	ceata_omap3evm.init = 1;

	if (ceata_standby(&ceata_port) != 0)
		printk("ceata_standby failed\n");

	mmc_unregister_driver(&ceata_mmc_device_driver);
	ceata_unregister(&ceata_omap3evm.host);
	return 0;
}

static struct platform_driver ceata_omap3evm_driver = {
	.probe = ceata_omap3evm_probe,
	.remove = ceata_omap3evm_remove,
	.suspend = NULL, 
	.resume = NULL,
	.driver = {
		   .name = "g6_ceata",
		   .owner = THIS_MODULE,
	},
};

static int __init ceata_drive_init(void)
{
	/* Register the wlan driver */
	if (platform_driver_register(&ceata_omap3evm_driver)) {
	  	printk(KERN_ERR ":failed to register MMC driver\n");
	  	return -ENODEV;
	}
	return 0;
}

static void __exit ceata_drive_exit(void)
{
	platform_driver_unregister(&ceata_omap3evm_driver);
}

module_init(ceata_drive_init);
module_exit(ceata_drive_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("OMAP3EVM CE-ATA device driver");
