/*
 * tps65023 - driver for tps65023 power management chip
 *
 * Copyright (C) 2007  <archos.com> 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/suspend.h>

#include <asm/arch/tps65023.h>
#include <linux/miscdevice.h>

/*-------------------------------------------------------------------------*/

#define	DRIVER_VERSION	"2 Dec 2007"
#define	DRIVER_NAME	(tps65023_driver.driver.name)

MODULE_DESCRIPTION("TPS6502x Power Management Driver");
MODULE_LICENSE("GPL");

static unsigned short normal_i2c[] = { TPS65023_ADD, I2C_CLIENT_END };

I2C_CLIENT_INSMOD;


#undef TPS_DEBUG
#define DBG if(0)

/*-------------------------------------------------------------------------*/
struct tps65023 {
	struct i2c_client	client;
	struct mutex		lock;
	int			irq;
	struct delayed_work	work;
	struct dentry		*file;
	unsigned		charging:1;
	unsigned		por:1;
	unsigned		model:8;
	u16			vbus;
	unsigned long		flags;
	/* copies of last register state */
	u8			chgstatus, regctrl, convctrl, defcore, ldoctrl;
	u8			nmask1, nmask2;
};

static struct tps65023 *the_tps;
static struct i2c_driver tps65023_driver;

/*-------------------------------------------------------------------------*/
#ifdef	TPS_DEBUG

void dump_reg( void) {

	unsigned char val;
	int i;
printk( "dump registers:\n");

	for ( i= 0 ; i < TPS_LDO_CTRL; i++ ) {

		val = i2c_smbus_read_byte_data(&the_tps->client, i);
		printk( "reg: %d  val: %x\n", i, val);
	}
}

static int tps65023_release( struct inode *inode, struct file *file )
{
	printk(" tps65023_release \n");
	return 0;
}

static int tps65023_open(struct inode *inode, struct file *file)
{
	printk(" tps65023_open \n");
	return 0;
}

static int tps65023_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	tps_param_t *param = (tps_param_t *) arg;

	printk(" tps65023_ioctl %x %x\n", (unsigned char)param->reg, (unsigned char)param->val);

	switch (cmd) {

	case TPS_SET_REG:
		i2c_smbus_write_byte_data( &the_tps->client, (unsigned char) param->reg, (unsigned char) param->val);

	case TPS_GET_REG:
		the_tps->nmask1 = i2c_smbus_read_byte_data( &the_tps->client,(unsigned char) param->reg);
		printk(" reg: %x =  %x \n",param->reg, the_tps->nmask1);
		break;		

	case TPS_ENA_DCDC:
		tps65023_enable_DCDC( param->par1, param->par2 );
		break;
	case TPS_SET_DCVAL:
		tps65023_set_DCDC1( param->par1);
		break;
	case TPS_ENA_DCVAL:
		tps65023_set_slew( param->par1);
		break;
	case TPS_ENA_PM:
		tps65023_core_adjust( param->par1);
		break;
	case TPS_DUMP:
		dump_reg();
		break;
	default:
		ret = -ENOIOCTLCMD;
	}
	return ret;
}

static struct file_operations tps65023_fops = {
	.owner		= THIS_MODULE,
	.ioctl          = tps65023_ioctl,
	.open           = tps65023_open,
	.release        = tps65023_release,
};

static struct miscdevice tps65023_miscdev = {
        .minor  = TPS_PM_MINOR,
        .name   = "tps65023",
        .fops	= &tps65023_fops,
};

#endif

//-------------------------------------------------------------------------
//	tps65023_enable_DCDC:
//	mode: ON or OFF
//	dcdc_num: 1 to 3 
//-------------------------------------------------------------------------
int tps65023_enable_DCDC( int dcdc_num, int mode )
{
	the_tps->regctrl = i2c_smbus_read_byte_data( &the_tps->client, TPS_REG_CTRL) & TPS_REG_CTRL_MASK;
	if ( mode ) {
		the_tps->regctrl |= ( 1 << ( 6 - dcdc_num));
	} else {
		the_tps->regctrl &= ~( 1 << ( 6 - dcdc_num));
	}
	
	i2c_smbus_write_byte_data(&the_tps->client, TPS_REG_CTRL, the_tps->regctrl);

	return 0;
}

//-------------------------------------------------------------------------
//	tps65023_core_adjust:
//	mode: ON or OFF
//-------------------------------------------------------------------------
int tps65023_core_adjust( int enable )
{
	// is Vchange allowed?
	the_tps->convctrl = i2c_smbus_read_byte_data( &the_tps->client, TPS_CONV_CTRL2) & TPS_CONV_CTRL2_MASK;

DBG	printk("tps65023_core_adjust: %d\n", enable);
	
	if ( enable ) {
		// enable switch 
		the_tps->convctrl &= ~TPS_CORE_ADJ_NOT_ALLOWED;
	} else {
		the_tps->convctrl |= TPS_CORE_ADJ_NOT_ALLOWED;
	}

	return ( i2c_smbus_write_byte_data(&the_tps->client, TPS_CONV_CTRL2, the_tps->convctrl));
}

//-------------------------------------------------------------------------
//	tps65023_set_DCDC1:
//	value:	value to set in mV 
//-------------------------------------------------------------------------
int tps65023_set_DCDC1( int value )
{
	int status = 0;	
	
	// is Vchange allowed?
	if ( the_tps->convctrl & TPS_CORE_ADJ_NOT_ALLOWED )
		return -1;		// can't change vdcdc1

DBG	printk("tps65023_set_DCDC1: %d\n",value);

	the_tps->defcore = TPS_DCDC1_CONVERT( value );
	
	// write new value for vcddc1
	status = i2c_smbus_write_byte_data(&the_tps->client, TPS_DEFCORE, the_tps->defcore);

	// enable switch to this value
	the_tps->convctrl |= TPS_GO;
	status = i2c_smbus_write_byte_data(&the_tps->client, TPS_CONV_CTRL2, the_tps->convctrl);

	return status;
}

//-------------------------------------------------------------------------
//	tps65023_set_slew:
//	value:	slew rate for dcdc1 
//-------------------------------------------------------------------------
int tps65023_set_slew( int value )
{
	int status = 0;	
	
	// is Vchange allowed?
	if ( the_tps->convctrl & TPS_CORE_ADJ_NOT_ALLOWED )
		return -1;		// can't change vdcdc1

DBG	printk("tps65023_set_slew: %d\n",value);

	// write new value for vcddc1
	status = i2c_smbus_write_byte_data( &the_tps->client, TPS_DEFSLEW, value & TPS_DEFSLEW_MASK);

	return status;
}

//-------------------------------------------------------------------------
//	tps65023_enable_LDO:
//	mode: ON or OFF
//	ldo_num: 1 or 2
//-------------------------------------------------------------------------
int tps65023_enable_LDO( int ldo_num, int mode )
{
	the_tps->regctrl = i2c_smbus_read_byte_data( &the_tps->client, TPS_REG_CTRL) & TPS_REG_CTRL_MASK;
	if ( mode ) {
		the_tps->regctrl |= ( 1 << ldo_num );
	} else {
		the_tps->regctrl &= ~( 1 << ldo_num );
	}
	
	i2c_smbus_write_byte_data( &the_tps->client, TPS_REG_CTRL, the_tps->regctrl );
	
	return 0;
}

//-------------------------------------------------------------------------
//	tps65023_set_LDO:
//	value:  output level step 0 to 7
//	ldo_num: 1 or 2
//-------------------------------------------------------------------------
int tps65023_set_LDO( int ldo_num, int value )
{
	the_tps->ldoctrl = i2c_smbus_read_byte_data( &the_tps->client, TPS_LDO_CTRL) & TPS_LDO_CTRL_MASK;
	if ( ldo_num == LD01 ) {
		the_tps->regctrl &= TPS_LDO2_CTRL_MASK;
		the_tps->regctrl |= (value & TPS_LDO1_CTRL_MASK);
	} 
	if ( ldo_num == LD02 ) {
		the_tps->regctrl &= TPS_LDO1_CTRL_MASK;
		the_tps->regctrl |= (value & TPS_LDO2_CTRL_MASK);
	}
	
	i2c_smbus_write_byte_data( &the_tps->client, TPS_LDO_CTRL, the_tps->ldoctrl );
	return 0;
}


/*-------------------------------------------------------------------------*/
static int tps65023_probe( struct i2c_adapter *bus, int address, int kind )
{
	struct tps65023		*tps;
	int			status;

	int err = 0;
DBG	printk (" tps65023_probe %s id %d\n", bus->name, bus->id);

	if (the_tps) {
		dev_dbg(&bus->dev, "only one %s for now\n", DRIVER_NAME);
		return 0;
	}

	/* Check basic functionality */
	if (!(err = i2c_check_functionality( bus, I2C_FUNC_SMBUS_WORD_DATA |
						I2C_FUNC_SMBUS_WRITE_BYTE))) {
		printk("tps65023 functionality check failed\n");
		return err;
	}

	tps = kzalloc(sizeof *tps, GFP_KERNEL);
	if (!tps)
		return 0;

	memset(&(tps->client), 0, sizeof(struct i2c_client));

	tps->irq = -1;
	tps->client.addr = address;
	tps->client.adapter = bus;
	tps->client.driver = &tps65023_driver;

	strlcpy( tps->client.name, DRIVER_NAME, I2C_NAME_SIZE );

	status = i2c_attach_client( &tps->client );

	if (status < 0) {
		dev_dbg(&bus->dev, "can't attach %s to device %d, err %d\n",
				DRIVER_NAME, tps->client.addr, status);
		kfree(tps);
		return 0;
	}
DBG	printk(" client %s attached to device %d\n", DRIVER_NAME, tps->client.addr);

#ifdef TPS_DEBUG
	tps->nmask1 = i2c_smbus_read_byte_data( &tps->client, TPS_VERSION);

	printk(" version: %x \n", tps->nmask1);

	printk("%s: ctrl 0x%02x, defcore %02x, status %02x\n", DRIVER_NAME,
		i2c_smbus_read_byte_data( &tps->client, TPS_REG_CTRL),
		i2c_smbus_read_byte_data( &tps->client, TPS_DEFCORE),
		i2c_smbus_read_byte_data( &tps->client, TPS_PGOODSTATUS));


	if ( misc_register( &tps65023_miscdev ))
		printk("cannot register miscdev %d \n", TPS_PM_MINOR);
#endif
	the_tps = tps;

	return 0;
}

static int tps65023_attach_adapter(struct i2c_adapter *adapter )
{
DBG	printk("tps65023_attach_adapter \n" );

	return i2c_probe( adapter, &addr_data, tps65023_probe );
}

static int tps65023_detach_client(struct i2c_client *client)
{
	struct tps65023		*tps;
DBG	printk(" tps65023_detach_client\n");

	tps = container_of(client, struct tps65023, client);

	if (i2c_detach_client(client) == 0)
		kfree(tps);
	the_tps = NULL;
	return 0;
}

static struct i2c_driver tps65023_driver = {
	.driver = {
		.name	= "tps65023",
	},
	.attach_adapter	= tps65023_attach_adapter,
	.detach_client	= tps65023_detach_client,
};

/*-------------------------------------------------------------------------*/

static int __init tps65023_init(void)
{
	int	status = -ENODEV;

	printk(KERN_INFO "%s: version %s\n", DRIVER_NAME, DRIVER_VERSION);

	status = i2c_add_driver( &tps65023_driver );
	if ( status ) {
		printk(KERN_ERR "%s: no chip?\n", DRIVER_NAME);

	}
	return status;
}

static void __exit tps65023_exit(void)
{
printk(" tps65023_exit\n");
	i2c_del_driver( &tps65023_driver );
}


//subsys_initcall( tps65023_init );
module_init(tps65023_init);
module_exit( tps65023_exit );


