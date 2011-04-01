/*
 *   Analog device I2c video encoder Interface.
 *
 *   Copyright (C) 2008 Archos SA
 *   Author: Archos
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
#include <linux/fb.h>
#include <linux/display_out.h>
#include "adv734x.h"

#define DEBUG
#ifdef DEBUG
#define DEBUGMSG(ARGS...)  printk("<%s>: ",__FUNCTION__);printk(ARGS)
#define DEBUGI2C(ARGS...)  printk(KERN_INFO ARGS)
#else
#define DEBUGMSG( x... )
#define DEBUGI2C( x... )
#endif

#define ADV734X_NB_RETRY_I2C	3
#define ADV734X_ID1		(0x54>>1)
#define ADV734X_VERSION	"0.0"
#define ADV734X_DATE	"March-2008"

static int ready=0;
module_param(ready, int, S_IRUGO|S_IWUSR);

extern int archosg6_display_get_extvideoenc_i2c_nbr(void);

#define ADV734X_I2C_NB	archosg6_display_get_extvideoenc_i2c_nbr()

#define MAX_ADV_CONFIG_SIZE 50
#define CONFIG_STOP 0xff


/* I2C Addresses to scan */
static unsigned short normal_i2c[] = { ADV734X_ID1, I2C_CLIENT_END };
 
/* This makes all addr_data:s */
I2C_CLIENT_INSMOD;

struct adv734x_regval {
	int reg;
	int val;
};

static struct adv734x_regval _pal_config[]={
	{0x17,0x2},
	{0x80,0x11},
	{0x82,0x40},
	{0x87,0x80},
	{0x88,0x00},
// 	{0x8a,0x0c},
 	{0x8a,0x0a},
	{0x8c,0xcb},
	{0x8d,0x8a},
	{0x8e,0x09},
	{0x8f,0x2a},
	{CONFIG_STOP,CONFIG_STOP}, /* end of config */
};

static struct adv734x_regval _ntsc_config[]={
	{0x17,0x2},
	{0x80,0x10},
	{0x82,0x40},
	{0x87,0x80},
	{0x88,0x00},
// 	{0x8a,0x0c},
 	{0x8a,0x0a},  /* mode 1 slave */
 	{0x8c,0x1f},
	{0x8d,0x7c},
	{0x8e,0xf0},
	{0x8f,0x21},
	{CONFIG_STOP,CONFIG_STOP}, /* end of config */
};

static struct adv734x_regval _hdpal_config[]={
	{0x17,0x2},
	{0x01,0x10},
	{0x30,0x28}, /* format 720p 50 Hz */
	{0x31,0x01}, /* pixel data valid */
	{0x33,0x28}, /*  */
	{0x34,0x40},
// 	{0x35,0x02}, /* RGB input */
 	{0x35,0x00}, /* YPrPb input */
	{CONFIG_STOP,CONFIG_STOP}, /* end of config */
};

static struct adv734x_regval _composite_config[]={
	{0x00,0x00}, /* all dac off */
	{0x02,0x00}, /* rgb out */	
	{0x00,(PLL_VAL|COMPOSITE_DAC)}, /* dac on*/
	{CONFIG_STOP,CONFIG_STOP}, /* end of config */
};

static struct adv734x_regval _component_config[]={
	{0x00,0x00}, /* all dac off */
	{0x02,0x20}, /* YPrPb out */	
	{0x00,(PLL_VAL|COMPONENT_DAC)}, /* dac on*/
	{CONFIG_STOP,CONFIG_STOP}, /* end of config */
};

static struct adv734x_regval _rgb_config[]={
	{0x00,0x00}, /* all dac off */
	{0x02,0x00}, /* rgb out */	
	{0x00,(PLL_VAL|RGB_DAC)}, /* dac on*/
	{CONFIG_STOP,CONFIG_STOP}, /* end of config */
};

static struct adv734x_regval _svideo_config[]={
	{0x00,0x00}, /* all dac off */
	{0x02,0x00}, /* rgb out */	
	{0x00,(PLL_VAL|SVIDEO_DAC)}, /* dac on*/
	{CONFIG_STOP,CONFIG_STOP}, /* end of config */
};

static struct adv734x_regval _dump_config[]={
	{0x00,0},
	{0x02,0},
	{0x17,0},
	{0x80,0},
	{0x82,0},
 	{0x84,0},
	{0x87,0},
	{0x88,0},
	{0x8a,0},
	{0x8c,0},
	{0x8d,0},
	{0x8e,0},
	{0x8f,0},
	{CONFIG_STOP,CONFIG_STOP}, /* end of config */
};

static struct adv734x_regval _sleepon_config[]={
	{0x17,0x2},
	{0x00,0x01}, /* set on sleep mode */
	{CONFIG_STOP,CONFIG_STOP}, /* end of config */
};

static struct i2c_driver adv734x_driver; 
static struct i2c_client *new_client_adv734x;

static void _local_tvp_gio_complente(struct i2c_client *c, int output) {

	void (*callback)(struct i2c_client *c, int output);

	callback = __symbol_get("tvp_gio_complente");

	if (callback) {
		(*callback)(c,output);
		__symbol_put("tvp_gio_complente");
	}
}

static void _local_tvp_gio_rgb(struct i2c_client *c, int output) {

	void (*callback)(struct i2c_client *c, int output);

	callback = __symbol_get("tvp_gio_rgb");

	if (callback) {
		(*callback)(c,output);
		__symbol_put("tvp_gio_rgb");
	}
}


struct i2c_client * i2c_get_adv734x_client(void) {

	return new_client_adv734x;
}

static inline int adv734x_read_value(struct i2c_client *client, int reg, int *value)
{
	unsigned int data;
	int cnt = -1;
	
	do{
		cnt++;
		*value = 0;
		data = i2c_smbus_read_byte_data( client, reg );
	} while((cnt != ADV734X_NB_RETRY_I2C) && (data == -1));
	if((cnt == ADV734X_NB_RETRY_I2C) || (data == -1)){
		DEBUGI2C( "adv734x bad read [%#x]=%#x(%d)\n",reg,data,data);
		*value = 0;
		return -1;
	} else {
		*value = data;
		DEBUGI2C( "adv734x_read_value[%#x]=%#x(%d)\n",reg,data,data);
		return 0;
	}
}

int adv734x_i2c_read(int reg, int *value)
{	
	return adv734x_read_value(new_client_adv734x, reg, value );	
}
EXPORT_SYMBOL(adv734x_i2c_read);

static inline int adv734x_write_value(struct i2c_client *client, int reg, int value)
{
	int cnt = 0;
	int ret = 0;
	
	do{
		cnt++;
		ret = i2c_smbus_write_byte_data(client, reg, value);
	} while((cnt != ADV734X_NB_RETRY_I2C) && (ret == -1));
	if(ret != -1)
		DEBUGI2C( "adv734x_write_value[%#x]=%#x(%d)\n",reg,value,value);
	return ret;
}

int adv734x_i2c_write(int reg, int value)
{
	return adv734x_write_value(new_client_adv734x, reg, value );	
}
EXPORT_SYMBOL(adv734x_i2c_write);

static int _read_config(struct adv734x_regval *pt) {

	int i=0;
 
	int val;
	int error=0;

	while ( (i<MAX_ADV_CONFIG_SIZE) && (pt->reg != CONFIG_STOP) && (error != -1) ) {
		error = adv734x_i2c_read(pt->reg,&val);
		pt++;
		i++;
	}

	if(error != -1)
		DEBUGI2C( "adv734x read %d value(s)\n",i);
	return error;

}

static int _setup_config(struct adv734x_regval *pt) {

	int i=0;
	int error=0;
	
	while ( (i<MAX_ADV_CONFIG_SIZE) && (pt->reg != CONFIG_STOP) && (error != -1) ) {
		error = adv734x_i2c_write(pt->reg,pt->val);
		pt++;
		i++;
	}

	if(error != -1)
		DEBUGI2C( "adv734x write %d value(s)\n",i);
	return error;

}	

static int adv734x_detect_client(struct i2c_adapter *adapter, int address, 
				     int kind)
{
	int err;
      	const char *client_name = "ADV734X_I2C";
	int val =0;
	
	DEBUGI2C("adv734x_detect_client on bus %d. \n",adapter->nr );
			
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA | 
				     I2C_FUNC_SMBUS_WRITE_BYTE)) {
		printk(KERN_INFO "%s functinality check failed\n", client_name);
		err = -ENODEV;
		return err;
	}

	if (!(new_client_adv734x = kmalloc(sizeof(struct i2c_client),
				   GFP_KERNEL))) {
		err = -ENOMEM;
		printk(KERN_WARNING "Couldn't allocate memory for %s\n", client_name);
		return err;
	}
	
	memset(new_client_adv734x,0, sizeof(struct i2c_client));

	new_client_adv734x->addr = address;
	new_client_adv734x->adapter = adapter;
	new_client_adv734x->driver = &adv734x_driver;
	new_client_adv734x->flags = 0;
	strlcpy(new_client_adv734x->name, client_name, I2C_NAME_SIZE);
       	
	if ((err = i2c_attach_client(new_client_adv734x))) {
	       	printk(KERN_WARNING "Couldn't attach %s\n", client_name);
		return err;
	}
	val = i2c_smbus_read_byte_data(new_client_adv734x, 0x0);
	if(val >= 0){
		DEBUGI2C( "ADV734X power mode = %#x\n",val);
	}
	else{
		err = -1;
		return err;
	}

	if ( val != 0x12 ) {
		ready = 0;
		printk(KERN_WARNING "adv734x i2C not ready wrong value read from, write 0x12 to register 0\n");
		val = i2c_smbus_write_byte_data(new_client_adv734x, 0x0, 0x12);
		if(val <= 0){
			DEBUGI2C( "ADV734X write register 0 error\n");
		}

		val = i2c_smbus_read_byte_data(new_client_adv734x, 0x0);
		if(val >= 0){
			DEBUGI2C( "ADV734X power mode = %#x\n",val);
		}

// 		err = -1;
// 		return err;
	} else
		ready = 1;

	_setup_config(_sleepon_config);

	return 0;
}
	
static int adv734x_detach_client(struct i2c_client *client)
{
	int err;

	if ((err = i2c_detach_client(client))) {
		printk("ADV734X_I2C: Client deregistration failed, client not detached.\n");
		return err;
	}

	kfree(client);
	return 0;
}

static int adv734x_attach_adapter(struct i2c_adapter *adapter)
{
 	if ( adapter->nr == (ADV734X_I2C_NB) ){		
		DEBUGI2C( "adv734x_attach_adapter %d\n",adapter->nr);
		return i2c_probe(adapter, &addr_data, adv734x_detect_client);
	} else {
 		return 1;
	}
}

static int adv734x_command(struct i2c_client* client, unsigned int cmd, int arg)
{
	int res = -EIO;

	if (arg == 0) {
		/* format select */
		switch (cmd) {
		case RESOLUTION_VIDEOEXT_PAL:
			/* Set Up PAL */
			res = _setup_config(_pal_config);
		break;
		case RESOLUTION_VIDEOEXT_NTSC:
			/* Set Up NTSC */
			res = _setup_config(_ntsc_config);
		break;
		case RESOLUTION_VIDEOEXT_HDPAL:
			/* Set Up NTSC */
			res = _setup_config(_hdpal_config);
		break;
		
		default:
			break;
		}
	}
	else if (arg == 1) {
		/* analog out select */
		switch (cmd) {
		case ANALOG_OUT_DEFAULT:
		case ANALOG_OUT_CVBS:
			/* Set Up composite */
 			res = _setup_config(_composite_config);
			_local_tvp_gio_complente(0,1);
			_local_tvp_gio_rgb(0,0);
			printk(KERN_INFO "[[TVP]] ANALOG_OUT_CVBS\n");
		break;
		case ANALOG_OUT_SVIDEO:
			/* Set Up svideo */
			res = _setup_config(_svideo_config);
			_local_tvp_gio_complente(0,1);
			_local_tvp_gio_rgb(0,0);
			printk(KERN_INFO "[[TVP]] ANALOG_OUT_SVIDEO\n");
		break;
		case ANALOG_OUT_RGB:
			/* Set Up rgb */
			res = _setup_config(_rgb_config);
			_local_tvp_gio_complente(0,1);
			_local_tvp_gio_rgb(0,1);
			printk(KERN_INFO "[[TVP]] ANALOG_OUT_RGB\n");
		break;
		case ANALOG_OUT_COMPONENT:
			/* Set Up rgb */
			res = _setup_config(_component_config);
			_local_tvp_gio_complente(0,0);
			_local_tvp_gio_rgb(0,0);
			printk(KERN_INFO "[[TVP]] ANALOG_OUT_COMPONENT\n");
		break;
		
		default:
			break;
		}
	} else if (arg == 2 ) {
			printk(KERN_INFO "[[TVP]] Reset ADV\n");
			res = _setup_config(_sleepon_config);
			_local_tvp_gio_complente(0,0);
			_local_tvp_gio_rgb(0,0);
	} else if (arg == 3 ) {
		/* dump */
		_read_config(_dump_config);
		
	} else {
	}

	return res;
}

/*-----------------------------------------------------------------------*/

static struct i2c_driver adv734x_driver = {

	.driver = {
		.owner	= THIS_MODULE,
		.name	= "adv734x i2c",
	},
	.id		= I2C_DRIVERID_ADV734X,
        .attach_adapter	= adv734x_attach_adapter,
        .detach_client	= adv734x_detach_client,
        .command	= adv734x_command,
};

/*
 *  INIT part
 */

static int __init adv734x_init(void)
{
	int res;
	
	if ((res = i2c_add_driver(&adv734x_driver))) {
		printk("adv734x i2c: Driver registration failed, module not inserted.\n");
		memset(new_client_adv734x,0, sizeof(struct i2c_client));
		return res;
	}

	printk("ADV734X I2C version %s (%s)\n", ADV734X_VERSION, ADV734X_DATE);
	
	return 0;
}

static void __exit adv734x_exit(void)
{
	i2c_del_driver(&adv734x_driver);
}

MODULE_AUTHOR("ARCHOS S.A.");
MODULE_DESCRIPTION("I2C interface for ADV7343.");
MODULE_LICENSE("GPL");

EXPORT_SYMBOL(i2c_get_adv734x_client);

module_init(adv734x_init)
module_exit(adv734x_exit)
