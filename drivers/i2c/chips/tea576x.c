/*
 *   NXP Semiconductor TEA5766 on AX04 Remote FM i2c interface.
 *
 * Copyright (c) 2006 Archos
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

#include <linux/tea576x.h>

#define DEBUG

#ifdef DEBUG
#define DPRINTK(ARGS...)  printk("<%s>: ",__FUNCTION__);printk(ARGS)
#else
#define DPRINTK( x... )
#endif



// standard i2c insmod options
static unsigned short normal_fm_i2c[]		= { (TEA576X_FM_ADDR >> 1), I2C_CLIENT_END };
static unsigned short normal_rds_i2c[]		= { (TEA5766_RDS_ADDR >> 1), I2C_CLIENT_END };
 
static unsigned short probe[2]			= { I2C_CLIENT_END, I2C_CLIENT_END };
 
static unsigned short ignore[2]			= { I2C_CLIENT_END, I2C_CLIENT_END };
 
static unsigned short force[2]			= { I2C_CLIENT_END, I2C_CLIENT_END };

static unsigned short *forces[] = { force, NULL	};	

static struct i2c_client_address_data addr_data_fm = {
	.normal_i2c		= normal_fm_i2c,
	.probe			= probe,	
	.ignore			= ignore,
	.forces			= forces,
};

static struct i2c_client_address_data addr_data_rds = {
	.normal_i2c		= normal_rds_i2c,
	.probe			= probe,	
	.ignore			= ignore,
	.forces			= forces,
};


  
/****************************************************************************
			I2C Command
 ****************************************************************************/

static int i2c_tea576x_read_fm_regs( struct i2c_client *c, radio_tea5766_fm_regs_t *regs )
{
	int err;
	u_char buffer[NB_BYTES_FM_READ_BY_ACTION];
 

	if ( !c || !c->adapter )
		return -ENODEV;

	memset(buffer, 0, sizeof(buffer));

	err = i2c_master_recv( c, buffer, NB_BYTES_FM_READ_BY_ACTION );
	if ( err < 0 ) {
		DPRINTK( "recv error = %d\n", err );
		return 1;
	}

	regs->intreg  = ((buffer[0] << 8) & 0xff00) | (buffer[1] & 0x00ff);
	regs->frqset  = ((buffer[2] << 8) & 0xff00) | (buffer[3] & 0x00ff);
	regs->tnctrl  = ((buffer[4] << 8) & 0xff00) | (buffer[5] & 0x00ff);
	regs->frqchk  = ((buffer[6] << 8) & 0xff00) | (buffer[7] & 0x00ff);
	regs->tunchk  = ((buffer[8] << 8) & 0xff00) | (buffer[9] & 0x00ff);
	regs->testreg = ((buffer[10] << 8) & 0xff00) | (buffer[11] & 0x00ff);


	// DEBUG
// 	DPRINTK( "%d bytes read\n", err );
// 	for ( i=0 ; i<NB_BYTES_FM_READ_BY_ACTION ; i++ ) {
// 		DPRINTK( "%02x\n", buffer[i] );
// 	}

	return 0;
}

static int i2c_tea576x_write_fm_regs( struct i2c_client *c, radio_tea5766_fm_regs_t *regs )
{
	int err;
	u_char buffer[NB_BYTES_FM_WRITE_BY_ACTION];
 

	if ( !c || !c->adapter )
		return -ENODEV;

	// Serialize the structure
	buffer[0] = (u_char)(regs->intreg & 0xff);
	buffer[1] = (u_char)(regs->frqset >> 8);
	buffer[2] = (u_char)(regs->frqset & 0xff);
	buffer[3] = (u_char)(regs->tnctrl >> 8);
	buffer[4] = (u_char)(regs->tnctrl & 0xff);
	buffer[5] = (u_char)(regs->testreg >> 8);
	buffer[6] = (u_char)(regs->testreg & 0xff);

	err = i2c_master_send( c, buffer, NB_BYTES_FM_WRITE_BY_ACTION );
	if ( err < 0 ) {
		DPRINTK( "error = %d\n", err );
		return 1;
	}

	// DEBUG
// 	DPRINTK( "%d bytes written\n", err );
// 	for ( i=0 ; i<NB_BYTES_FM_WRITE_BY_ACTION ; i++ ) {
// 		DPRINTK( "%02x\n", buffer[i] );
// 	}

	return 0;
}

static int i2c_tea576x_fm_command( struct i2c_client *client,
			   unsigned int cmd, void *arg )
{
	switch ( cmd ) {

	case TEA576X_READ:
// 		DPRINTK( "TEA576X_READ\n" );
		return i2c_tea576x_read_fm_regs( client, (radio_tea5766_fm_regs_t *)arg );

	case TEA576X_WRITE:
// 		DPRINTK( "TEA576X_WRITE\n" );
		return i2c_tea576x_write_fm_regs( client, (radio_tea5766_fm_regs_t *)arg );

	default:
		return -EINVAL;
	}
}


static int i2c_tea576x_read_rds_regs( struct i2c_client *c, radio_tea5766_rds_regs_t *regs )
{
	int err;
	u_char buffer[NB_BYTES_RDS_READ_BY_ACTION];
 

	if ( !c || !c->adapter )
		return -ENODEV;

	memset(buffer, 0, sizeof(buffer));

	err = i2c_master_recv( c, buffer, NB_BYTES_RDS_READ_BY_ACTION );
	if ( err < 0 ) {
		DPRINTK( "recv error = %d\n", err );
		return 1;
	}

	regs->rdsr1	= ((buffer[0] << 8) & 0xff00) | (buffer[1] & 0x00ff);
	regs->rdsr2	= ((buffer[2] << 8) & 0xff00) | (buffer[3] & 0x00ff);
	regs->rdsr3	= ((buffer[4] << 8) & 0xff00) | (buffer[5] & 0x00ff);
	regs->rdsr4	= ((buffer[6] << 8) & 0xff00) | (buffer[7] & 0x00ff);
	regs->rdsw1	= ((buffer[8] << 8) & 0xff00) | (buffer[9] & 0x00ff);
	regs->rdsw2	= ((buffer[10] << 8) & 0xff00) | (buffer[11] & 0x00ff);
	regs->manid	= ((buffer[12] << 8) & 0xff00) | (buffer[13] & 0x00ff);
	regs->chipid	= ((buffer[14] << 8) & 0xff00) | (buffer[15] & 0x00ff);

// 	// DEBUG
// 	DPRINTK( "%d bytes read\n", err );
// 	for ( i=0 ; i<NB_BYTES_RDS_READ_BY_ACTION ; i++ ) {
// 		DPRINTK( "%02x\n", buffer[i] );
// 	}

	return 0;
}

static int i2c_tea576x_write_rds_regs( struct i2c_client *c, radio_tea5766_rds_regs_t *regs )
{
	int err;
	u_char buffer[NB_BYTES_RDS_WRITE_BY_ACTION];
 

	if ( !c || !c->adapter )
		return -ENODEV;

	// Serialize the structure
	buffer[0] = (u_char)(regs->rdsw1 >> 8);
	buffer[1] = (u_char)(regs->rdsw1 & 0xff);
	buffer[2] = (u_char)(regs->rdsw2 >> 8);
	buffer[3] = (u_char)(regs->rdsw2 & 0xff);

	err = i2c_master_send( c, buffer, NB_BYTES_RDS_WRITE_BY_ACTION );
	if ( err < 0 ) {
		DPRINTK( "error = %d\n", err );
		return 1;
	}

// 	// DEBUG
// 	DPRINTK( "%d bytes written\n", err );
// 	for ( i=0 ; i<NB_BYTES_RDS_WRITE_BY_ACTION ; i++ ) {
// 		DPRINTK( "%02x\n", buffer[i] );
// 	}

	return 0;
}

static int i2c_tea576x_rds_command( struct i2c_client *client,
			   unsigned int cmd, void *arg )
{
	switch ( cmd ) {

	case TEA576X_READ:
// 		DPRINTK( "TEA576X_READ\n" );
		return i2c_tea576x_read_rds_regs( client, (radio_tea5766_rds_regs_t *)arg );

	case TEA576X_WRITE:
// 		DPRINTK( "TEA576X_WRITE\n" );
		return i2c_tea576x_write_rds_regs( client, (radio_tea5766_rds_regs_t *)arg );

	default:
		return -EINVAL;
	}
}


/****************************************************************************
			I2C Client & Driver
 ****************************************************************************/

static struct i2c_driver i2c_tea576x_fm_driver;
 
static struct i2c_driver i2c_tea576x_rds_driver;
 


static int i2c_tea576x_fm_detect_client( struct i2c_adapter *adapter, int address, 
					int kind )
{
	int err;
	struct i2c_client *i2c_tea576x_fm_client;
	const char *client_name = "tea576x_fm";

	DPRINTK( "i2c_tea576x_fm_detect_client\r\n" );

	if ( !i2c_check_functionality( adapter, I2C_FUNC_SMBUS_WORD_DATA | 
				     I2C_FUNC_SMBUS_WRITE_BYTE ) ) {
		DPRINTK( "%s functionality check at 0x%x failed\r\n", client_name, address );
		err = -ENODEV;
		return err;
	}

	if ( !( i2c_tea576x_fm_client = kmalloc( sizeof(struct i2c_client), GFP_KERNEL ) ) ) {
		err = -ENOMEM;
		DPRINTK( "Couldn't allocate memory for %s\r\n", client_name );
		return err;
	}

	memset( i2c_tea576x_fm_client, 0x00, sizeof(struct i2c_client) );
	i2c_tea576x_fm_client->addr = address;
	i2c_tea576x_fm_client->adapter = adapter;
	i2c_tea576x_fm_client->driver = &i2c_tea576x_fm_driver;
	i2c_tea576x_fm_client->flags = 0;
	strlcpy( i2c_tea576x_fm_client->name, client_name, I2C_NAME_SIZE );
	
	if ( ( err = i2c_attach_client( i2c_tea576x_fm_client ) ) ) {
		DPRINTK( "Couldn't attach %s\r\n", client_name );
		kfree( i2c_tea576x_fm_client );
		return err;
	}

	DPRINTK( "client %s attached to adapter %s\r\n", i2c_tea576x_fm_client->name, adapter->name );

	return 0;
}

static int i2c_tea576x_fm_attach_adapter( struct i2c_adapter *adapter )
{
	int err;

 	if ( adapter->nr == 3 ) {
		if ( ( err = i2c_probe( adapter, &addr_data_fm, i2c_tea576x_fm_detect_client ) ) ) {
			DPRINTK("Adapter attach failed.\r\n");
		}
		return err;

	} else
		return 1;
}

static int i2c_tea576x_fm_detach_client( struct i2c_client *client )
{
	int err;

	DPRINTK("\r\n");
	if ( ( err = i2c_detach_client( client ) ) ) {
		DPRINTK("Client deregistration failed, client not detached.\r\n");
		return err;
	}
	kfree(client);
	return 0;
}


static int i2c_tea576x_rds_detect_client( struct i2c_adapter *adapter, int address, 
					int kind )
{
	int err;
	struct i2c_client *i2c_tea576x_rds_client;
	const char *client_name = "tea576x_rds";

	if ( !i2c_check_functionality( adapter, I2C_FUNC_SMBUS_WORD_DATA | 
				     I2C_FUNC_SMBUS_WRITE_BYTE ) ) {
		DPRINTK( "%s functionality check at 0x%x failed\r\n", client_name, address );
		err = -ENODEV;
		return err;
	}

	if ( !( i2c_tea576x_rds_client = kmalloc( sizeof(struct i2c_client), GFP_KERNEL ) ) ) {
		err = -ENOMEM;
		DPRINTK( "Couldn't allocate memory for %s\r\n", client_name );
		return err;
	}

	memset( i2c_tea576x_rds_client, 0x00, sizeof(struct i2c_client) );
	i2c_tea576x_rds_client->addr = address;
	i2c_tea576x_rds_client->adapter = adapter;
	i2c_tea576x_rds_client->driver = &i2c_tea576x_rds_driver;
	i2c_tea576x_rds_client->flags = 0;
	strlcpy( i2c_tea576x_rds_client->name, client_name, I2C_NAME_SIZE );

	if ( ( err = i2c_attach_client( i2c_tea576x_rds_client ) ) ) {
		DPRINTK( "Couldn't attach %s\r\n", client_name );
		kfree( i2c_tea576x_rds_client );
		return err;
	}

	DPRINTK( "client %s attached to adapter %s\r\n", i2c_tea576x_rds_client->name, adapter->name );

	return 0;
}

static int i2c_tea576x_rds_attach_adapter( struct i2c_adapter *adapter )
{
	int err;

	DPRINTK("\r\n");
 	if ( adapter->nr == 3 ) {
		if ( ( err = i2c_probe( adapter, &addr_data_rds, i2c_tea576x_rds_detect_client ) ) ) {
			DPRINTK( "Adapter attach failed.\r\n" );
		}
		return err;
	}
	else
		return 1;
}

static int i2c_tea576x_rds_detach_client( struct i2c_client *client )
{
	int err;

	DPRINTK("\r\n");
	if ( ( err = i2c_detach_client( client ) ) ) {
		DPRINTK( "Client deregistration failed, client not detached.\r\n" );
		return err;
	}
	kfree(client);
	return 0;
}


/* ----------------------------------------------------------------------- */

static struct i2c_driver i2c_tea576x_fm_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "tea576xfm_i2cdriver",
	},
	.id = I2C_DRIVERID_TEA576X_FM,
	.attach_adapter = i2c_tea576x_fm_attach_adapter,
	.detach_client = i2c_tea576x_fm_detach_client,
	.command = i2c_tea576x_fm_command,
};

static struct i2c_driver i2c_tea576x_rds_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "tea576xrds_i2cdriver",
	},
	.id = I2C_DRIVERID_TEA576X_RDS,
	.attach_adapter = i2c_tea576x_rds_attach_adapter,
	.detach_client = i2c_tea576x_rds_detach_client,
	.command = i2c_tea576x_rds_command,
};

static int __init i2c_tea576x_init( void )
{
	int res;
	
	DPRINTK( "NXP TEA576X init\r\n");

	// FM
	if ( ( res = i2c_add_driver( &i2c_tea576x_fm_driver ) ) ) {
		printk("i2c_tea576x_init : FM Driver registration failed, module not inserted.\r\n");
		return res;
	}
	DPRINTK( "i2c_tea576x_init : FM driver registered\r\n");

	// RDS
	if ( ( res = i2c_add_driver( &i2c_tea576x_rds_driver ) ) ) {
		printk("i2c_tea576x_init : RDS Driver registration failed, module not inserted.\r\n");
		return res;
	}
	DPRINTK("i2c_tea576x_init : RDS driver registered\r\n");
	DPRINTK( "i2c_tea576x_init : NXP TEA576X version %s (%s)\r\n", TEA576X_VERSION, TEA576X_DATE );

	return 0;
}

static void __exit i2c_tea576x_exit( void)
{
 

	DPRINTK( "NXP TEA576X exit\r\n");

	// FM
	i2c_del_driver( &i2c_tea576x_fm_driver );
	i2c_del_driver( &i2c_tea576x_rds_driver );
}

module_init( i2c_tea576x_init );
module_exit( i2c_tea576x_exit );

/* Module information */
MODULE_AUTHOR("Archos S.A.");
MODULE_DESCRIPTION("I2C interface for TEA5766 on AX05 FM remote");
MODULE_LICENSE("GPL");
