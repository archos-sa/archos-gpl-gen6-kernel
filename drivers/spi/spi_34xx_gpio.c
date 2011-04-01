/* linux/drivers/spi/spi_34xx_gpio.c
 *
 * Copyright (c) 2008 ARCHOS
 *
 * 34XX GPIO based SPI driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <asm/arch/gpio.h>
#include <asm/arch/spi-gpio.h>
#include <asm/hardware.h>

#define DBG if(0)

struct omap3_spigpio {
	struct spi_bitbang		 bitbang;

	struct spigpio_info		*info;
	struct platform_device		*dev;
};
static	struct omap3_spigpio  *sp;

static inline struct omap3_spigpio *spidev_to_sg(struct spi_device *spi)
{
	return spi->controller_data;
}

static inline void setsck(struct spi_device *dev, int on)
{
	struct omap3_spigpio *sg = spidev_to_sg(dev);
	omap_set_gpio_dataout(sg->info->pin_clk, on ? 1 : 0);
}

static inline void setmosi(struct spi_device *dev, int on)
{
	struct omap3_spigpio *sg = spidev_to_sg(dev);
	omap_set_gpio_dataout(sg->info->pin_mosi, on ? 1 : 0);
}

static inline u32 getmiso(struct spi_device *dev)
{
	struct omap3_spigpio *sg = spidev_to_sg(dev);
	return omap_get_gpio_datain(sg->info->pin_miso) ? 1 : 0;
}

static void omap3_spigpio_chipselect(struct spi_device *dev, int value)
{
	struct omap3_spigpio *sg = spidev_to_sg(dev);
	omap_set_gpio_dataout(sg->info->pin_cs, value ? 0 : 1);
}

#define spidelay(x) ndelay(x)

#define	EXPAND_BITBANG_TXRX
#include <linux/spi/spi_bitbang.h>


static u32 omap3_spigpio_txrx_mode0(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha0(spi, nsecs, 0, word, bits);
}

static u32 omap3_spigpio_txrx_mode1(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha1(spi, nsecs, 0, word, bits);
}

static u32 omap3_spigpio_txrx_mode2(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha0(spi, nsecs, 1, word, bits);
}

static u32 omap3_spigpio_txrx_mode3(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha1(spi, nsecs, 1, word, bits);
}

/* set state of spi pins */
void spi_gpio_init_state( int state )
{
DBG printk("spi_gpio_init_state  %d \n", state);
	if (state) {
		omap_set_gpio_dataout(sp->info->pin_clk, 1);
		omap_set_gpio_dataout(sp->info->pin_mosi, 0);
		omap_set_gpio_dataout(sp->info->pin_cs, 1);
	} else {
		// set all pins to 0
		omap_set_gpio_dataout(sp->info->pin_clk, 0);
		omap_set_gpio_dataout(sp->info->pin_mosi, 0);
		omap_set_gpio_dataout(sp->info->pin_cs, 0);
	}
}

static int omap3_spigpio_probe(struct platform_device *dev)
{
	struct spi	*master;

	int ret;
	int i;
DBG printk("omap3_spigpio_probe  %x \n", dev->id);
	master = spi_alloc_master(&dev->dev, sizeof(struct omap3_spigpio));
	if (master == NULL) {
		dev_err(&dev->dev, "failed to allocate spi master\n");
		ret = -ENOMEM;
		goto err;
	}

	sp = spi_master_get_devdata(master);

	platform_set_drvdata(dev, sp);

	/* copy in the plkatform data */
	sp->info = dev->dev.platform_data;

	/* setup spi bitbang adaptor */
	sp->bitbang.master = spi_master_get(master);
	sp->bitbang.chipselect = omap3_spigpio_chipselect;

	sp->bitbang.txrx_word[SPI_MODE_0] = omap3_spigpio_txrx_mode0;
	sp->bitbang.txrx_word[SPI_MODE_1] = omap3_spigpio_txrx_mode1;
	sp->bitbang.txrx_word[SPI_MODE_2] = omap3_spigpio_txrx_mode2;
	sp->bitbang.txrx_word[SPI_MODE_3] = omap3_spigpio_txrx_mode3;

DBG printk(" clk: %d, mosi: %d, cs:%d \n",(int)sp->info->pin_clk, (int)sp->info->pin_mosi, (int)sp->info->pin_cs);
	/* set state of spi pins */
	spi_gpio_init_state(0);

	ret = spi_bitbang_start(&sp->bitbang);

	if (ret)
		goto err_no_bitbang;

	/* register the chips to go with the board */

	for (i = 0; i < sp->info->board_size; i++) {
		dev_info(&dev->dev, "registering %p: %s\n",
			 &sp->info->board_info[i],
			 sp->info->board_info[i].modalias);

		sp->info->board_info[i].controller_data = sp;
		spi_new_device(master, sp->info->board_info + i);
DBG printk("registering %p: %s\n",
			 &sp->info->board_info[i],
			 sp->info->board_info[i].modalias);
	}

	return 0;

 err_no_bitbang:
	spi_master_put(sp->bitbang.master);
 err:
	return ret;

}

static int omap3_spigpio_remove(struct platform_device *dev)
{
	struct omap3_spigpio *sp = platform_get_drvdata(dev);

	spi_bitbang_stop(&sp->bitbang);
	spi_master_put(sp->bitbang.master);

	return 0;
}

/* all gpio should be held over suspend/resume, so we should
 * not need to deal with this
*/

#define omap3_spigpio_suspend NULL
#define omap3_spigpio_resume NULL


static struct platform_driver omap3_spigpio_drv = {
	.probe		= omap3_spigpio_probe,
        .remove		= omap3_spigpio_remove,
        .suspend	= omap3_spigpio_suspend,
        .resume		= omap3_spigpio_resume,
        .driver		= {
		.name	= "omap3-spi-gpio",
		.owner	= THIS_MODULE,
        },
};

static int __init omap3_spigpio_init(void)
{
        return platform_driver_register(&omap3_spigpio_drv);
}

static void __exit omap3_spigpio_exit(void)
{
        platform_driver_unregister(&omap3_spigpio_drv);
}

module_init(omap3_spigpio_init);
module_exit(omap3_spigpio_exit);

EXPORT_SYMBOL(spi_gpio_init_state);

MODULE_DESCRIPTION("OMAP34XX SPI Driver");
MODULE_AUTHOR("ARCHOS");
MODULE_LICENSE("GPL");
