/*
 *   PCF8575 on AX05 Remote FM i2c interface.
 *
 * Copyright (c) 2007 Archos
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

#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>

#include <linux/pcf8575.h>

#define DEBUG
#ifdef DEBUG
#define DEBUGMSG(ARGS...)  printk("<%s>: ",__FUNCTION__);printk(ARGS)
#else
#define DEBUGMSG( x... )
#endif

#define ACCESSORY_I2C_NB	3



// Standard i2c insmod options
static unsigned short normal_i2c[]		= { (PCF8575_ADDR >> 1), I2C_CLIENT_END };
 
static unsigned short probe[2] 			= { I2C_CLIENT_END, I2C_CLIENT_END };
 
static unsigned short ignore[2]			= { I2C_CLIENT_END, I2C_CLIENT_END };
 
static unsigned short force[2]			= { I2C_CLIENT_END, I2C_CLIENT_END };
static unsigned short *forces[] = { force, NULL};
static struct i2c_client_address_data addr_data_pcf8575 = {
	.normal_i2c		= normal_i2c,
	.probe			= probe,
	.ignore			= ignore,
	.forces			= forces,
};


//  ----------------------------------------------------------------------------
//			I2C Commands
//  ----------------------------------------------------------------------------


static int i2c_pcf8575_write( struct i2c_client *c, uint16_t *regs )
{
	int err;
	u_char buffer[PCF8575_WRITE_BYTES];
 

	if ( !c || !c->adapter )
		return -ENODEV;

	buffer[0] = (u_char)((*regs) & 0xff);
	buffer[1] = (u_char)((*regs) >> 8);

	err = i2c_master_send( c, buffer, PCF8575_WRITE_BYTES );
	if ( err < 0 ) {
		printk( "i2c_pcf8575_write : error = %d\r\n", err );
		return 1;
	}

	// DEBUG
// 	DEBUGMSG( "i2c_pcf8575_write : %d bytes written\r\n", err );
// 	for ( i=0 ; i<PCF8575_WRITE_BYTES ; i++ ) {
// 		DEBUGMSG( "i2c_pcf8575_write : %02x\r\n", buffer[i] );
// 	}

	return 0;
}


//
// i2c_pcf8575_read()
//
// Reads the pins of the PCF8575
//	INPUT:	regs => pins 15-0
//		Set the corresponding bit to 1 if a pin is to be read
//	OUTPUT:	regs => pins 15-0
//
static int i2c_pcf8575_read( struct i2c_client *c, uint16_t *regs )
{
	int err;
	u_char buffer[PCF8575_READ_BYTES];
 

	if ( !c || !c->adapter )
		return -ENODEV;

	memset(buffer, 0, sizeof(buffer));

	// Set the pins we want to read to 1
	err = i2c_pcf8575_write( c, regs );
	if ( err > 0 ) {
		printk( "i2c_pcf8575_read : i2c_pcf8575_write failed\r\n" );
		return 1;
	}

	// Read the state of the pins
	err = i2c_master_recv( c, buffer, PCF8575_READ_BYTES );
	if ( err < 0 ) {
		printk( "i2c_pcf8575_read : recv error = %d\r\n", err );
		return 1;
	}
	
	*regs = ((buffer[1] << 8) & 0xff00) | (buffer[0] & 0x00ff);
	// Now *regs = state of the pins that have been read

	// DEBUG
// 	DEBUGMSG( "i2c_pcf8575_read : %d bytes read\r\n", err );
// 	for ( i=0 ; i<PCF8575_READ_BYTES ; i++ ) {
// 		DEBUGMSG( "i2c_pcf8575_read : %02x\r\n", buffer[i] );
// 	}

	return 0;
}


static int i2c_pcf8575_command( struct i2c_client *client, unsigned int cmd, void *arg )
{
	switch ( cmd )
	{
	case PCF8575_READ:
		return i2c_pcf8575_read( client, (uint16_t *)arg );

	case PCF8575_WRITE:
		return i2c_pcf8575_write( client, (uint16_t *)arg );

	default:
		return -EINVAL;
	}
}

//  ----------------------------------------------------------------------------
//			I2C Client & Driver
//  ----------------------------------------------------------------------------

static struct i2c_driver i2c_pcf8575_driver;
 


static int i2c_pcf8575_detect_client( struct i2c_adapter *adapter, int address, int kind )
{
	int err;
	struct i2c_client *i2c_pcf8575_client;
	const char *client_name = "pcf8575";

	if ( !i2c_check_functionality( adapter, I2C_FUNC_SMBUS_WORD_DATA | 
				     I2C_FUNC_SMBUS_WRITE_BYTE ) ) {
		printk( "i2c_pcf8575_detect_client : %s functionality check at 0x%x failed\r\n", client_name, address );
		err = -ENODEV;
		return err;
	}

	if ( !( i2c_pcf8575_client = kmalloc( sizeof(struct i2c_client), GFP_KERNEL ) ) ) {
		err = -ENOMEM;
		printk( "i2c_pcf8575_detect_client : Couldn't allocate memory for %s\r\n", client_name );
		return err;
	}

	memset( i2c_pcf8575_client, 0x00, sizeof(struct i2c_client) );
	i2c_pcf8575_client->addr = address;
	i2c_pcf8575_client->adapter = adapter;
	i2c_pcf8575_client->driver = &i2c_pcf8575_driver;
	i2c_pcf8575_client->flags = 0;
	strlcpy( i2c_pcf8575_client->name, client_name, I2C_NAME_SIZE );
	
	if ( ( err = i2c_attach_client( i2c_pcf8575_client ) ) ) {
		printk( "i2c_pcf8575_detect_client : Couldn't attach %s\r\n", client_name );
		kfree( i2c_pcf8575_client );
		return err;
	}

	printk( "i2c_pcf8575_detect_client : client %s attached to adapter %s\r\n", i2c_pcf8575_client->name, adapter->name );

	return 0;
}

static int i2c_pcf8575_attach_adapter( struct i2c_adapter *adapter )
{
	int err;

 	if ( adapter->nr == 3 ) {
		if ( ( err = i2c_probe( adapter, &addr_data_pcf8575, i2c_pcf8575_detect_client ) ) ) {
			printk("i2c_pcf8575_attach_adapter : Adapter attach failed.\r\n");
		}
		return err;
	} else
		return 1;
}

static int i2c_pcf8575_detach_client( struct i2c_client *client )
{
	int err;

	if ( ( err = i2c_detach_client( client ) ) ) {
		printk("i2c_pcf8575_detach_client : Client deregistration failed, client not detached.\r\n");
		return err;
	}

	kfree(client);
	return 0;
}




// -----------------------------------------------------------------------

static struct i2c_driver i2c_pcf8575_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "pcf8575_i2cdriver",
	},
	.id = I2C_DRIVERID_PCF8575,
	.attach_adapter = i2c_pcf8575_attach_adapter,
	.detach_client = i2c_pcf8575_detach_client,
	.command = i2c_pcf8575_command,
};


static int __init i2c_pcf8575_init( void )
{
	int res;
	
	if ( ( res = i2c_add_driver( &i2c_pcf8575_driver ) ) ) {
		printk("i2c_pcf8575_init : Driver registration failed, module not inserted.\r\n");
		return res;
	}
	printk( "i2c_pcf8575_init : Driver registered\r\n");
	
	return 0;
}

static void __exit i2c_pcf8575_exit( void)
{
 

	i2c_del_driver( &i2c_pcf8575_driver );

	printk( "i2c_pcf8575_exit: done\r\n");
}

module_init( i2c_pcf8575_init );
module_exit( i2c_pcf8575_exit );

// Module information
MODULE_AUTHOR("Archos S.A.");
MODULE_DESCRIPTION("I2C interface for PCF8575 on AX05 FM remote");
MODULE_LICENSE("GPL");
