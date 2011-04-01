/*
 * hx5091_spi.c
 *
 * Copyright (C) 2008 Archos
 *
 * spi interface for TFT LCD chip HIMAX HX5091-A 
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <asm/mach-types.h>
#include <asm/arch/gpio.h>

#define SPI_HX5091_ADDR	1

#undef GPIO_FIX

#define DBG if(0)
#define debug2 DBG printk

struct hx5091_spi_t {
	struct			spi_device *spi;
	struct			spi_driver driver;
	struct 			spi_transfer t[2];
	struct 			spi_message msg;
	unsigned short dummy;
};

static struct hx5091_spi_t *hx5091_spi;

int hx5091_spi_reg_write( unsigned int addr, unsigned int value) 
{
	unsigned char data[2];
DBG	printk("spi_reg_write: addr=%d, val=%02X\n", addr, value);

	/* Now we prepare the command for transferring */
	data[0] = addr & 0x7f;
	data[1] = value;

	spi_message_init(&hx5091_spi->msg);

	memset(&hx5091_spi->t, 0, sizeof(hx5091_spi->t));
	hx5091_spi->t[0].tx_buf = data;
	hx5091_spi->t[0].rx_buf = NULL;
	hx5091_spi->t[0].len = 2;
	spi_message_add_tail(&hx5091_spi->t[0], &hx5091_spi->msg);
DBG printk("send sync\n");
	spi_sync( hx5091_spi->spi, &hx5091_spi->msg);

	return 0;
}

int hx5091_spi_reg_read( unsigned int addr, unsigned int *value) 
{

	unsigned char data, cmd;

	cmd = addr | 0x80;	// set read bit

	memset( hx5091_spi->t, 0, sizeof(hx5091_spi->t));
	spi_message_init(&hx5091_spi->msg);

	hx5091_spi->t[0].tx_buf = &cmd;
	hx5091_spi->t[0].rx_buf = &data;
	hx5091_spi->t[0].len = 2;
	spi_message_add_tail( &hx5091_spi->t[0], &hx5091_spi->msg);
	spi_sync( hx5091_spi->spi, &hx5091_spi->msg);

DBG	printk("spi_reg_read: addr=%d, val=%02X\n", addr, data);
	*value = data;	
	return 0;

}


int tpo_spi_reg_write( unsigned int addr, unsigned int value) 
{
	unsigned char data[2];
DBG printk("spitpo_reg_write: addr=%d, val=%02X\n", addr, value);

	/* Now we prepare the command for transferring */
	data[0] = (addr << 1 ) & 0xfe;
	data[1] = value;

	spi_message_init(&hx5091_spi->msg);

	memset(&hx5091_spi->t, 0, sizeof(hx5091_spi->t));
	hx5091_spi->t[0].tx_buf = data;
	hx5091_spi->t[0].rx_buf = NULL;
	hx5091_spi->t[0].len = 2;
	spi_message_add_tail(&hx5091_spi->t[0], &hx5091_spi->msg);
DBG printk("send sync\n");
	spi_sync( hx5091_spi->spi, &hx5091_spi->msg);

	return 0;
}


int tpo_spi_reg_read( unsigned int addr, unsigned int *value) 
{

	unsigned char data, cmd;

	cmd = (addr << 1) | 0x01;	// set read bit

	memset( hx5091_spi->t, 0, sizeof(hx5091_spi->t));
	spi_message_init(&hx5091_spi->msg);

	hx5091_spi->t[0].tx_buf = &cmd;
	hx5091_spi->t[0].rx_buf = &data;
	hx5091_spi->t[0].len = 2;
	spi_message_add_tail( &hx5091_spi->t[0], &hx5091_spi->msg);
	spi_sync( hx5091_spi->spi, &hx5091_spi->msg);

DBG	printk("spi_reg_read: addr=%d, val=%02X\n", addr, data);
	*value = data;	
	return 0;

}


#ifdef GPIO_FIX

// spi via gpios
static int spi_clk;
static int spi_data;
static int spi_cs;

static void spi_setscl(int state)
{
	omap_set_gpio_dataout( spi_clk, state );
}

static void spi_setsda(int state)
{
	omap_set_gpio_dataout( spi_data, state );
}

static void spi_enable(int state)
{
	omap_set_gpio_dataout( spi_cs, state ? 0 : 1 );
}

static void spi_wait(void)
{
    volatile int i;
    for (i=0;i<100;i++);
}

int hx5091_gpio_spi_reg_write( unsigned int addr, unsigned int value) 
{
	unsigned short buffer=0;
	int i;

	// samsung interface for now
	buffer = ( (unsigned short) addr & 0x7f ) << 8;	
	buffer |= (unsigned short) value;
DBG	printk("spi_gpio_write: %x %x %x\n", addr, value, buffer);
	
	spi_enable(1);
	spi_wait();
	for ( i=15 ; i >= 0 ; i-- ) {
		spi_setscl(0);
		spi_setsda( (buffer >> i) & 1 );	
		spi_wait();
		spi_setscl(1);
		spi_wait();
	}
	spi_wait();
	spi_enable(0);
	spi_setsda(0);
	return 0;
}

void spi_init_port()
{
	// spi port in gio mode
	// pins depending of boards -> to fix
	spi_clk = 137;	// ah3
	spi_data = 138;	// af3
	spi_cs = 139;	// ae3

	omap_set_gpio_dataout( spi_cs, 1 );
	omap_set_gpio_dataout( spi_clk, 1 );
	omap_set_gpio_dataout( spi_data, 0 );
}
#endif

static int  __devinit hx5091_probe( struct spi_device *spi )
{

	int r;
DBG printk("hx5091_probe bus %x\n", spi->master->bus_num);

	hx5091_spi = kzalloc( sizeof(*hx5091_spi), GFP_KERNEL );
	if (hx5091_spi == NULL) {
		dev_err(&spi->dev, "out of mem\n");
		return -ENOMEM;
	}
#ifdef GPIO_FIX
	if ( machine_is_archos_g6h() ||  machine_is_archos_g6plus() )
		spi_init_port();
	else
#endif
 	{

		spi_set_drvdata( spi, &hx5091_spi);
		hx5091_spi->spi = spi;

		hx5091_spi->spi->mode = SPI_MODE_3;
		hx5091_spi->spi->bits_per_word = 8;
		if ((r = spi_setup( hx5091_spi->spi )) < 0) {
			dev_err(&spi->dev, "SPI setup failed\n");
			goto err;
		}
	}	
	dev_info(&spi->dev, "initialized\n");

	return 0;
err:
	kfree( hx5091_spi );
	return r;

}
static int hx5091_suspend(struct spi_device *spi, pm_message_t message)
{
	return 0;
}

static int hx5091_resume(struct spi_device *spi)
{
	return 0;
}

static int hx5091_remove(struct spi_device *spi)
{
	hx5091_spi = NULL;
	return 0;
}

static struct spi_driver hx5091_driver = {
	.driver = {
		.name	= "hx5091",
		.owner	= THIS_MODULE,
	},
	.probe =	hx5091_probe,
	.remove =	__devexit_p(hx5091_remove),
	.suspend	= hx5091_suspend,
	.resume		= hx5091_resume,
};

static int __init hx5091_spi_init(void)
{
        int err = 0;

DBG printk ("hx5091_spi_init\n");
        err = spi_register_driver( &hx5091_driver );
        if (err) {
                printk("Failed to register HX5091 client.\n");
		return err;
        }

	return 0;
}

static void __exit hx5091_spi_exit(void)
{
        spi_unregister_driver(&hx5091_driver);
}

module_init(hx5091_spi_init);
module_exit(hx5091_spi_exit);

EXPORT_SYMBOL(hx5091_spi_reg_read);
EXPORT_SYMBOL(hx5091_spi_reg_write);
EXPORT_SYMBOL(tpo_spi_reg_read);
EXPORT_SYMBOL(tpo_spi_reg_write);

#ifdef GPIO_FIX
EXPORT_SYMBOL(hx5091_gpio_spi_reg_write);
#endif
MODULE_AUTHOR("Archos S.A.");
MODULE_DESCRIPTION("HX5091 TFT LCD Driver for the Archos G6 series");
MODULE_LICENSE("GPL");
