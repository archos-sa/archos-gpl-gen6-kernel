/*
 *   Texas Instrumens WM87xx audio codec's i2c interface.
 *   
 *   Copyright (c) by Matthias Welwarsky  Archos S.A.
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

#define DBG if(1)

#define WM87xxID1		0x1a
#define WM87xx_VERSION		"0.1"
#define WM87xx_DATE		"06-November-2006"
#define WM87xx_I2C_NB		2

/* I2C Addresses to scan */
static unsigned short normal_i2c[] = { WM87xxID1, I2C_CLIENT_END };
/* This makes all addr_data:s */
I2C_CLIENT_INSMOD;

struct wm87xx_regval {
	int reg;
	int val;
};

static struct i2c_driver wm87xx_driver; 
static struct i2c_client *new_client;

struct i2c_client * i2c_get_wm87xx_client(void) {

	return new_client;
}

static inline int wm87xx_write_value(struct i2c_client *client, int reg, int value)
{
	int reg2send,val2send;

 	reg2send = (reg<<1) | ((value & 0x100)>>8);
	val2send = value & 0xFF;

	return i2c_smbus_write_byte_data(client, reg2send, val2send);
}

static int wm87xx_command(struct i2c_client* client, unsigned int cmd, void* arg)
{
	struct wm87xx_regval *regval = (struct wm87xx_regval*)arg;
	int res = -EIO;
	
	switch (cmd) {
	case 0:
		res = wm87xx_write_value(client, regval->reg, regval->val);
		break;
	
	default:
		break;
	}

	return res;
}

static int wm87xx_detect_client(struct i2c_adapter *adapter, int address, 
				     int kind)
{
	int err;
      	const char *client_name = "WM87xx Audio Codec";
   
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA | 
				     I2C_FUNC_SMBUS_WRITE_BYTE)) {
		printk(KERN_WARNING "%s functinality check failed\n", client_name);
		err = -ENODEV;
		return err;
	}
	
	if (!(new_client = kmalloc(sizeof(struct i2c_client),
				   GFP_KERNEL))) {
		err = -ENOMEM;
		printk(KERN_WARNING "Couldn't allocate memory for %s\n", client_name);
		return err;
	}
	
	memset(new_client, 0x00, sizeof(struct i2c_client));
	new_client->addr = address;
	new_client->adapter = adapter;
	new_client->driver = &wm87xx_driver;
	new_client->flags = 0;
	strlcpy(new_client->name, client_name, I2C_NAME_SIZE);
       	
	if ((err = i2c_attach_client(new_client))) {
	       	printk(KERN_WARNING "Couldn't attach %s\n", client_name);
		kfree(new_client);
		return err;
	}
	
	printk("client %s attached to %s [%d]\n", new_client->name, adapter->name,(adapter->nr + 1));
	return 0;
}
	
static int wm87xx_detach_client(struct i2c_client *client)
{
	int err;
	
	if ((err = i2c_detach_client(client))) {
		printk("wm87xx.o: Client deregistration failed, client not detached.\n");
		return err;
	}
	
	kfree(client);
	return 0;
}

static int wm87xx_attach_adapter(struct i2c_adapter *adapter)
{
 	if ( adapter->nr == WM87xx_I2C_NB )
		return i2c_probe(adapter, &addr_data, wm87xx_detect_client);
 	else
 		return 1;
}

/*-----------------------------------------------------------------------*/

static struct i2c_driver wm87xx_driver = {

	.driver = {
		.owner	= THIS_MODULE,
		.name	= "wm87xx i2c",
	},
        .id		= I2C_DRIVERID_WM8731,
        .attach_adapter	= wm87xx_attach_adapter,
        .detach_client	= wm87xx_detach_client,
        .command	= wm87xx_command,
};

/*
 *  INIT part
 */

static int __init wm87xx_init(void)
{
	int res;
	
	if ((res = i2c_add_driver(&wm87xx_driver))) {
		printk("wm87xx i2c: Driver registration failed, module not inserted.\n");
		return res;
	}

	printk("WM87xx I2C version %s (%s)\n", WM87xx_VERSION, WM87xx_DATE);
	
	return 0;
}

static void __exit wm87xx_exit(void)
{
	i2c_del_driver(&wm87xx_driver);
}

MODULE_AUTHOR("Matthias Welwarsky , Archos S.A.");
MODULE_DESCRIPTION("I2C interface for WM87xx codec.");
MODULE_LICENSE("GPL");

EXPORT_SYMBOL(i2c_get_wm87xx_client);

module_init(wm87xx_init)
module_exit(wm87xx_exit)
