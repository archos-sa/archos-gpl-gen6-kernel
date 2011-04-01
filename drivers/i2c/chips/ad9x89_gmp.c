/*
 *   Analog device HDMI chip i2c interface.
 *
 *   Copyright (C) 2007  <archos.com> 
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <asm/arch/gpio.h>
#include <asm/io.h>
#include <asm/arch/mux.h>

#define DEBUG
#ifdef DEBUG
#define DEBUGMSG(ARGS...)  printk("<%s>: ",__FUNCTION__);printk(ARGS)
#define DEBUGI2C(ARGS...)  printk(KERN_INFO ARGS)
#else
#define DEBUGMSG( x... )
#define DEBUGI2C( x... )
#endif

#define AD9X89_GMP_NB_RETRY_I2C	3
#define AD9X89_GMP_ID1		(0x70>>1)
#define AD9X89_GMP_VERSION	"0.1"
#define AD9X89_GMP_DATE	"09-January-2008"

/*TV6*/
// #define AD9X89_GMP_I2C_NB	2
/*G6*/
#define AD9X89_GMP_I2C_NB	3
extern int archosg6_display_get_hdmi_i2c_nbr(void);/* to know the i2c number */

/* I2C Addresses to scan */
static unsigned short normal_i2c[] = { AD9X89_GMP_ID1, I2C_CLIENT_END };

/* This makes all addr_data:s */
I2C_CLIENT_INSMOD;

struct ad9x89_gmp_regval {
	int reg;
	int val;
};

static struct i2c_driver ad9x89_gmp_driver; 
static struct i2c_client *new_client_ad9x89_gmp;

struct i2c_client * i2c_get_ad9x89_gmp_client(void) {

	return new_client_ad9x89_gmp;
}

static inline int ad9x89_gmp_read_value(struct i2c_client *client, int reg, int *value)
{
	unsigned int data;
	int cnt = 0;
	
	do{
		cnt++;
		*value = 0;
		data = i2c_smbus_read_byte_data( client, reg );
	} while((cnt != AD9X89_GMP_NB_RETRY_I2C) && (data == -1));
	if((cnt == AD9X89_GMP_NB_RETRY_I2C) || (data == -1)){
		*value = 0;
		return -1;
	} else {
		*value = data;
		DEBUGI2C( "ad9x89_gmp_read_value[%#x]=%#x(%d)\n",reg,data,data);
		return 0;
	}
}

int ad9x89_gmp_i2c_read(int reg, int *value)
{
	return ad9x89_gmp_read_value(new_client_ad9x89_gmp, reg, value );	
}
EXPORT_SYMBOL(ad9x89_gmp_i2c_read);

static inline int ad9x89_gmp_write_value(struct i2c_client *client, int reg, int value)
{
	int cnt = 0;
	int ret = 0;
	
	do{
		cnt++;
		ret = i2c_smbus_write_byte_data(client, reg, value);
	} while((cnt != AD9X89_GMP_NB_RETRY_I2C) && (ret == -1));
	if(ret != -1)
		DEBUGI2C( "ad9x89_gmp_write_value[%#x]=%#x(%d)\n",reg,value,value);
	return ret;
}

int ad9x89_gmp_i2c_write(int reg, int value)
{
	return ad9x89_gmp_write_value(new_client_ad9x89_gmp, reg, value );	
}
EXPORT_SYMBOL(ad9x89_gmp_i2c_write);


static int ad9x89_gmp_detect_client(struct i2c_adapter *adapter, int address, 
				     int kind)
{
	int err;
      	const char *client_name = "AD9X89_I2C_GMP";
	int val = 0;
   
	DEBUGI2C("ad9x89_gmp_detect_client on bus %d. \n",adapter->nr );
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA | 
				     I2C_FUNC_SMBUS_WRITE_BYTE)) {
		printk(KERN_WARNING "%s functinality check failed\n", client_name);
		err = -ENODEV;
		return err;
	}
	
	if (!(new_client_ad9x89_gmp = kzalloc(sizeof(struct i2c_client),
				   GFP_KERNEL))) {
		err = -ENOMEM;
		printk(KERN_WARNING "Couldn't allocate memory for %s\n", client_name);
		return err;
	}
	
	new_client_ad9x89_gmp->addr = address;
	new_client_ad9x89_gmp->adapter = adapter;
	new_client_ad9x89_gmp->driver = &ad9x89_gmp_driver;
	new_client_ad9x89_gmp->flags = 0;
	strlcpy(new_client_ad9x89_gmp->name, client_name, I2C_NAME_SIZE);
       	
	if ((err = i2c_attach_client(new_client_ad9x89_gmp))) {
	       	printk(KERN_INFO "Couldn't attach %s\n", client_name);
		return err;
	}
		
	val = i2c_smbus_read_byte_data(new_client_ad9x89_gmp, 0x0);
	if(val >= 0){
		printk("client %s (%#x) attached to %s [%d]\n",
		       new_client_ad9x89_gmp->name,AD9X89_GMP_ID1<<1, adapter->name,(adapter->nr ));
		return 0;
	} else{
		err = -1;
		return err;
	}
}
	
static int ad9x89_gmp_detach_client(struct i2c_client *client)
{
	int err;
	
	if ((err = i2c_detach_client(client))) {
		printk("AD9X89_HDMI_I2C_GMP: Client deregistration failed, client not detached.\n");
		return err;
	}
	
	kfree(client);
	return 0;
}

static int ad9x89_gmp_attach_adapter(struct i2c_adapter *adapter)
{
#ifdef CONFIG_MACH_ARCHOS_G6
	if ( adapter->nr == archosg6_display_get_hdmi_i2c_nbr() ){
#else
	if ( adapter->nr == AD9X89_GMP_I2C_NB ){
#endif
	DEBUGI2C( "ad9x89_gmp_attach_adapter %d\n",adapter->nr);
		return i2c_probe(adapter, &addr_data, ad9x89_gmp_detect_client);
	} else {
 		return 1;
	}
}

/*-----------------------------------------------------------------------*/

static struct i2c_driver ad9x89_gmp_driver = {

	.driver = {
		.owner	= THIS_MODULE,
		.name	= "ad9x89 hdmi i2c gmp",
	},
	.id		= I2C_DRIVERID_AD9X89_GMP,
        .attach_adapter	= ad9x89_gmp_attach_adapter,
        .detach_client	= ad9x89_gmp_detach_client,
        .command	= NULL,
};

/*
 *  INIT part
 */

static int __init ad9x89_gmp_init(void)
{
	int res;
	
	if ((res = i2c_add_driver(&ad9x89_gmp_driver))) {
		printk("ad9x89 i2c gmp: Driver registration failed, module not inserted.\n");
		memset(new_client_ad9x89_gmp, 0x00, sizeof(struct i2c_client));
		return res;
	}

	printk("AD9X89 gmp I2C version %s (%s)\n", AD9X89_GMP_VERSION, AD9X89_GMP_DATE);
	
	return 0;
}

static void __exit ad9x89_gmp_exit(void)
{
	i2c_del_driver(&ad9x89_gmp_driver);
}

MODULE_AUTHOR("ARCHOS S.A.");
MODULE_DESCRIPTION("I2C interface for AD9X89 hdmi gmp.");
MODULE_LICENSE("GPL");

EXPORT_SYMBOL(i2c_get_ad9x89_gmp_client);

module_init(ad9x89_gmp_init)
module_exit(ad9x89_gmp_exit)
