/*
 *   Texas Instrumens MSP430 micro controller's i2c interface.
 *
 *   Copyright (c) by Matthias Welwarsky <Archos S.A.>
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
#include <asm/io.h>

#include <linux/msp430.h>

#undef DEBUG
#ifdef DEBUG
#define DEBUGMSG(ARGS...)  printk("<%s>: ",__FUNCTION__);printk(ARGS)
#else
#define DEBUGMSG( x... )
#endif

#define ATMEGA_I2C_ADDR		0x2d
#define ATMEGA_VERSION		"0.1"
#define ATMEGA_DATE		"06-January-2006"
/* on EVM 1 ; on GS06   2  */
#ifdef CONFIG_MACH_ARCHOS_G6
#define ATMEGA_I2C_BUS_NUM	2
#else
#define ATMEGA_I2C_BUS_NUM	1
#endif

#define ATMG_I2C_WRITE_BLOCK
#define ATMG_I2C_READ_BLOCK

#ifdef CONFIG_MACH_ARCHOS_G6
#undef ATMG_I2C_CHECKSUM
#endif

/* I2C Addresses to scan */
static unsigned short normal_i2c[] = { ATMEGA_I2C_ADDR, I2C_CLIENT_END };
// static unsigned short normal_i2c_range[] = { I2C_CLIENT_END };

/* This makes all addr_data:s */
I2C_CLIENT_INSMOD;

struct atmega_regval {
	int reg;
	int val;
};

static struct semaphore atmg_i2c_lock;

static struct i2c_driver atmg_i2c_driver;
// static int atmg_i2c_id = 0;
static struct i2c_client *new_client;

static int atmg_i2c_detect_client(struct i2c_adapter *adapter, int address, 
				     int kind)
{
	int err = 0;
	const char *client_name = "MSP430 uC";

	DEBUGMSG(" atmg_probe %s id %d addr %x\n", adapter->name, adapter->id,address);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA | 
				     I2C_FUNC_SMBUS_WRITE_BYTE)) {
		printk(KERN_WARNING "%s functionality check failed\n", client_name);
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
	new_client->driver = &atmg_i2c_driver;
// 	new_client->flags = I2C_CLIENT_ALLOW_USE;
// 	new_client->id = atmg_i2c_id++;
// 	new_client->id =  2;
	strlcpy(new_client->name, client_name, I2C_NAME_SIZE);

	if ((err = i2c_attach_client(new_client))) {
	       	printk(KERN_WARNING "Couldn't attach %s\n", client_name);
		kfree(new_client);
		return err;
	}
	
	printk("client %s attached to adapter %s\n", new_client->name, adapter->name);

	/* FIXME: for debugging only! */

	{
// 		int i,j;
		int data;
// 		for (j=0;j<10;j++) {

// 			for (i=ATMEGA_I2C_READ_REG_BASE; i < ATMEGA_I2C_READ_REG_BASE+30; i++) {
// 			for (i=ATMEGA_I2C_READ_REG_BASE; i < ATMEGA_I2C_READ_REG_BASE+30; i++) {
				data = i2c_smbus_read_byte_data(new_client, 7);
				DEBUGMSG("msp reg[%i]: %02X\n", 7, data & 0xFF);
// 			}
		printk("\n");
// 		}
/*
		data = 10;
		for (i=ATMEGA_I2C_WRITE_REG_BASE; i < ATMEGA_I2C_WRITE_REG_BASE+0x0a; i++) {
			i2c_smbus_write_byte_data(new_client, i, data);
			data++;
		}

		for (i=ATMEGA_I2C_REREAD_REG_BASE; i < ATMEGA_I2C_REREAD_REG_BASE+0x0a; i++) {
			data = i2c_smbus_read_byte_data(new_client, i);
			DEBUGMSG("msp reg[%i]: %02X\n", i, data & 0xFF);
		}*/
	}

// 	data = i2c_smbus_read_byte_data(new_client, ATMEGA_I2C_READ_REG_BASE+0x04):
// 	if data ==
	return 0;
}
	
static int atmg_i2c_detach_client(struct i2c_client *client)
{
	int err;
	
	if ((err = i2c_detach_client(client))) {
		printk("msp i2c : Client deregistration failed, client not detached.\n");
		return err;
	}
	
	kfree(client);
	return 0;
}

static int atmg_i2c_attach_adapter(struct i2c_adapter *adapter)
{
	int err=0;
	printk("atmg_i2c_attach_adapter %d\n",adapter->nr);

 	if ( adapter->nr == ATMEGA_I2C_BUS_NUM ){	
		printk("adapter->nr == %d? %d\n",ATMEGA_I2C_BUS_NUM,adapter->nr);	
		err = i2c_probe( adapter, &addr_data, atmg_i2c_detect_client );
		if (err) {
			printk("msp i2c : Adapter attach failed.\n");
		}
		return err;
	} else {
 		return 1;
	}

	return i2c_probe( adapter, &addr_data, atmg_i2c_detect_client );
}

static int atmg_i2c_write_reg( unsigned int reg, unsigned char *value, int size )
{
	int ret = 0;
	int err;
#ifdef ATMG_I2C_WRITE_BLOCK
	char *tmp;
#endif
	int i;

	if ( !new_client || !new_client->adapter )
		return -ENODEV;

#ifdef ATMG_I2C_WRITE_BLOCK
	tmp = kmalloc( size + 1, GFP_KERNEL );
	if ( tmp == NULL )
		return -ENOMEM;

	tmp[0] = reg;
	for ( i=0 ; i<size ; i++ ) {
		tmp[i+1] = value[i];
DEBUGMSG( "msp i2c write: reg = %02x data = %02x\n", reg + i, tmp[i+1] );
	}
	err = i2c_master_send( new_client, tmp , size + 1 );
	if ( err < 0 ) {
		printk( "msp i2c write: error = %d\n", err );
		ret = 1;
	}

	kfree( tmp );
#else
	for ( i=0 ; i<size ; i++ ) {
		err = i2c_smbus_write_byte_data( new_client, reg++, *value++ );
		if ( err < 0 ) {
			printk( "msp i2c write: error = %d\n", err );
			ret = 1;
			break;
		}
	}
#endif

	return ret;
}

static int atmg_i2c_table_available( void )
{
	int ret = 0;
	int err;
#ifdef ATMG_I2C_WRITE_BLOCK
	char buf[3] = { ATMEGA_I2C_AVAILABLE_REG, 0 , 1 };
#else
	unsigned int reg = ATMEGA_I2C_AVAILABLE_REG;
	char buf[2] = { 0 , 1 };
#endif

	if ( !new_client || !new_client->adapter )
		return -ENODEV;

#ifdef ATMG_I2C_WRITE_BLOCK
	err = i2c_master_send( new_client, buf , 3 );
#else
	err = i2c_smbus_write_byte_data( new_client, reg++, buf[0] );
	if ( err >= 0 ) {
		err = i2c_smbus_write_byte_data( new_client, reg++, buf[1] );
	}
#endif

	if ( err < 0 ) {
		printk( "msp i2c write: error = %d\n", err );
		ret = 1;
	}
	return ret;
}

static int atmg_i2c_write_table( struct atmega_exchange_table *table )
{
	int ret = 0;
	int err;
	int i;
#ifdef ATMG_I2C_WRITE_BLOCK
	char *tmp;
#else
	unsigned int reg;
	unsigned char val;
#endif
	unsigned char *data;
//	struct atmega_exchange_table *table = (struct atmega_exchange_table *) arg;

	if ( !new_client || !new_client->adapter )
		return -ENODEV;

	// Fill exchange table
	table->magic = ATMEGA_I2C_COMMAND_MAGIC;
	
#ifdef ATMG_I2C_WRITE_BLOCK
	tmp = kmalloc( ATMEGA_I2C_EXCHANGE_SIZE + 1, GFP_KERNEL );
	if ( tmp == NULL )
		return -ENOMEM;

	tmp[0] = ATMEGA_I2C_EXCHANGE_REG;

	for ( i=0 ; i<4 ; i++ ) {
		tmp[i+1] = (unsigned char)( ( table->value >> ( i * 8 ) ) & 0x000000ff );
DEBUGMSG( "msp i2c write: reg = %02x data = %02x\n", ATMEGA_I2C_EXCHANGE_REG + i, tmp[i+1] );
	}
	data = (unsigned char *)&table->control_byte;
	for ( i=4 ; i<ATMEGA_I2C_EXCHANGE_SIZE ; i++ ) {
		tmp[i+1] = *data++;
DEBUGMSG( "msp i2c write: reg = %02x data = %02x\n", ATMEGA_I2C_EXCHANGE_REG + i, tmp[i+1] );
	}
	err = i2c_master_send( new_client, tmp , ATMEGA_I2C_EXCHANGE_SIZE + 1 );
	if ( err < 0 ) {
		printk( "msp i2c write: error = %d\n", err );
		ret = 1;
	}

	kfree( tmp );
#else
	reg = ATMEGA_I2C_EXCHANGE_REG;

	for ( i=0 ; i<4 ; i++ ) {
		val = (unsigned char)( ( table->value >> ( i * 8 ) ) & 0x000000ff );
DEBUGMSG( "msp i2c write: reg = %d data = %d\n", reg, val );
		err = i2c_smbus_write_byte_data( new_client, reg++, val );
		if ( err < 0 ) {
			printk( "msp i2c write: error = %d\n", err );
			ret = 1;
			break;
		}
	}

	data = (unsigned char *)&table->control_byte;
	for ( i=4 ; i<ATMEGA_I2C_EXCHANGE_SIZE ; i++ ) {
DEBUGMSG( "msp i2c write: reg = %d data = %d\n", reg, *data );
		err = i2c_smbus_write_byte_data( new_client, reg++, *data++ );
		if ( err < 0 ) {
			printk( "msp i2c write: error = %d\n", err );
			ret = 1;
			break;
		}
	}
#endif

	return ret;
}

static int atmg_i2c_read_reg( unsigned int reg, unsigned long *value, int size )
{
#ifdef ATMG_I2C_READ_BLOCK
	int ret = 0;
	int err;
	char *tmp;
	int i;

	if ( !new_client || !new_client->adapter )
		return -ENODEV;

	tmp = kmalloc( size, GFP_KERNEL );
	if ( tmp == NULL )
		return -ENOMEM;

	tmp[0] = reg;
	err = i2c_master_send( new_client, tmp, 1 );
	if ( err < 0 ) {
		printk( "msp i2c read: send error = %d\n", err );
		ret = 1;
		goto exit;
	}

	err = i2c_master_recv( new_client, tmp, size );
	if ( err < 0 ) {
		printk( "msp i2c read: recv error = %d\n", err );
		ret = 1;
		goto exit;
	}

	*value = 0;
	for ( i=0 ; i<size ; i++ ) {
		*value |= ( tmp[i] << ( 8 * i ) );
DEBUGMSG( "msp i2c read: reg = %02x data = %02x\n", reg + i, tmp[i] );
	}

exit:
	kfree( tmp );
	return ret;
#else
	unsigned int data;
	int i;

	if ( !new_client || !new_client->adapter )
		return -ENODEV;

	*value = 0;
	for ( i=0 ; i<size ; i++ ) {
		data = i2c_smbus_read_byte_data( new_client, reg + i );
		if ( data < 0 ) {
			printk( "msp i2c read: error = %d\n", data );
			return -EINVAL;
		}
		*value |= ( ( data & 0xff ) << ( 8 * i ) );
DEBUGMSG( "msp i2c read: reg = %02x data = %02x\n", reg + i, data );
	}

	return 0;
#endif
}

/*-----------------------------------------------------------------------*/

static struct i2c_driver atmg_i2c_driver = {
	.driver = {
// 	 	.owner  = THIS_MODULE, 
		.name	= "msp430",
	},

// 	.id		= I2C_DRIVERID_MSP430,
//         .flags		= I2C_DF_NOTIFY,
        .attach_adapter	= atmg_i2c_attach_adapter,
        .detach_client	= atmg_i2c_detach_client,
//         .command	= 0,
};



/*
 *  INIT part
 */

static int __init atmg_i2c_init(void)
{
	int res;
	
	if ((res = i2c_add_driver(&atmg_i2c_driver))) {
		printk("atmg_i2c: Driver registration failed, module not inserted.\n");
		return res;
	}

        init_MUTEX( &atmg_i2c_lock );

	printk("MSP430 I2C version %s (%s)\n", ATMEGA_VERSION, ATMEGA_DATE);

	return 0;
}

static void __exit atmg_i2c_exit(void)
{
printk(" atmg_i2c_exit\n");
	i2c_del_driver( &atmg_i2c_driver );
}

int atmega_write_value( unsigned int reg, unsigned char *value, int size )
{
	down( &atmg_i2c_lock );
	if ( atmg_i2c_write_reg( reg, value, size ) ) {
		up( &atmg_i2c_lock );
		printk( "atmega_write_value: msp430 write failed\n");
		return -1;
	}
	up( &atmg_i2c_lock );
DEBUGMSG("atmega_write_value reg = %d\n", reg );
	return 0;
}

int atmega_write_table( struct atmega_exchange_table *table )
{
	down( &atmg_i2c_lock );

	if ( atmg_i2c_write_table( table ) ) {
		// Retry one time
		if ( atmg_i2c_write_table( table ) ) {
			up( &atmg_i2c_lock );
			printk( "atmega_write_table: msp430 write failed\n");
			return -1;
		} else {
			printk( "atmega_write_table: msp430 write retry OK\n");
		}
	}
	if ( atmg_i2c_table_available() ) {
		// Retry one time
		if ( atmg_i2c_table_available() ) {
			up( &atmg_i2c_lock );
			printk( "atmega_write_table: msp430 validation failed\n");
			return -1;
		} else {
			printk( "atmega_write_table: msp430 validation retry OK\n");
		}
	}
	up( &atmg_i2c_lock );
DEBUGMSG("atmega_write_table cmd = %d\n", table->control_byte );
	return 0;
}

int atmega_read_value( unsigned int reg, unsigned long *value, int size )
{
#ifdef ATMG_I2C_CHECKSUM

	unsigned long atmg_checksum;
	unsigned short reg_checksum = 1;
	int retry = 2;

	do {
		down( &atmg_i2c_lock );
		if ( atmg_i2c_read_reg( reg, value, size ) ) {
			// Retry one time
			if ( atmg_i2c_read_reg( reg, value, size ) ) {
				up( &atmg_i2c_lock );
				printk( "atmega_read_value: msp430 read failed\n");
				return -1;
			} else {
				DEBUGMSG( "atmega_read_value: msp430 read retry OK\n");
			}
		}

		retry--;

		// read msp checksum register
		if ( atmg_i2c_read_reg( ATMEGA_I2C_CS_REG, &atmg_checksum, ATMEGA_I2C_CS_SIZE ) ) {
			printk( "atmega_read_CS failed\n");
		} else {
			int i;
			for ( i = 0; i < size; i++)
				reg_checksum += (unsigned short) ( ( *value >> (8 * i)) & 0xff);

			if ( (unsigned short) atmg_checksum == reg_checksum) {
				retry--;
			} else if ( retry < 1 ){
				up( &atmg_i2c_lock );
				printk( "atmega_read_value: msp430 cs error\n");
				return -1;
			}
		}
		up( &atmg_i2c_lock );
	} while( retry > 0);

#else	// ATMG_I2C_CHECKSUM
	down( &atmg_i2c_lock );
	if ( atmg_i2c_read_reg( reg, value, size ) ) {
		// Retry one time
		if ( atmg_i2c_read_reg( reg, value, size ) ) {
			up( &atmg_i2c_lock );
			printk( "atmega_read_value: msp430 read failed\n");
			return -1;
		} else {
			printk( "atmega_read_value: msp430 read retry OK\n");
		}
	}
	up( &atmg_i2c_lock );
#endif

DEBUGMSG("atmega_read_value reg = %d\n", reg );
	return 0;
}


MODULE_AUTHOR("Matthias Welwarsky , Archos S.A.");
MODULE_DESCRIPTION("I2C interface for MSP430 uC.");
MODULE_LICENSE("GPL");

EXPORT_SYMBOL(atmega_write_value);
EXPORT_SYMBOL(atmega_write_table);
EXPORT_SYMBOL(atmega_read_value);

module_init(atmg_i2c_init)
module_exit(atmg_i2c_exit)
