/*
 * tps65023.c : TPS65023 Power management IC driver for OMAP3 
 *
 * by Pratheesh Gangadhar <pratheesh@ti.com>
 *
 * Copyright (C) 2008-2009 Texas Instruments, Inc.
 *
 * Based on twl4030_core.c - driver for TWL4030
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * Modifications to defer interrupt handling to a kernel thread:
 * Copyright (C) 2006 MontaVista Software, Inc.
 *
 * Based on tlv320aic23.c:
 * Copyright (c) by Kai Svahn <kai.svahn@nokia.com>
 *
 * Code cleanup and modifications to IRQ handler.
 * by syed khasim <x0khasim@ti.com>
 *
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
#include <linux/module.h>
#include <linux/kernel_stat.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/random.h>
#include <linux/syscalls.h>
#include <linux/kthread.h>
#include <linux/slab_def.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <asm/irq.h>
#include <asm/mach/irq.h>

#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#include <asm/arch/power_companion.h>
#include <asm/arch/prcm.h>
#include <asm/arch/resource.h>


#define DRIVER_NAME			"tps65023"

#define pr_err(fmt, arg...)	printk(KERN_ERR DRIVER_NAME ": " fmt, ##arg);


#define CONFIG_I2C_TPS65023_ID   1   /* on I2C1 for OMAP3EVM */

#define TPS65023_SLAVEID_ID0		0x48

/**** Macro Definitions */
#define TPS65023_CLIENT_STRING		"TPS65023-ID"
#define TPS65023_CLIENT_USED			1
#define TPS65023_CLIENT_FREE			0

/* Register definitions */
#define TPS65023_VERSION		0x00
#define TPS65023_PGOODZ			0x01
#define TPS65023_MASK			0x02
#define TPS65023_REG_CTRL		0x03
#define TPS65023_CON_CTRL		0x04
#define TPS65023_CON_CTRL2		0x05
#define TPS65023_DEFCORE		0x06
#define TPS65023_DEFSLEW		0x07
#define TPS65023_LDOCTRL		0x08


struct tps65023_client  {
	struct i2c_client	client;
	const char client_name[sizeof(TPS65023_CLIENT_STRING) + 1];
	const unsigned char address;
	const char adapter_index;
	unsigned char inuse;
};

static struct tps65023_client tps65023_module = {
	.address	= TPS65023_SLAVEID_ID0,
	.client_name	= TPS65023_CLIENT_STRING "0",
	.adapter_index	=  CONFIG_I2C_TPS65023_ID,
};

struct constraint_handle *co_opp_tps65023_vdd2;
static struct constraint_id cnstr_id_vdd2 = {
	.type = RES_OPP_CO,
	.data = (void *)"vdd2_opp",
};

/**** Helper functions */
static int tps65023_detect_client(struct i2c_adapter *adapter);
static int tps65023_attach_adapter(struct i2c_adapter *adapter);
static int tps65023_detach_client(struct i2c_client *client);

static struct i2c_driver tps65023_driver = {
	.driver.name	= "TPS65023 I2C",
	.attach_adapter	= tps65023_attach_adapter,
	.detach_client	= tps65023_detach_client,
};


/**
 * @brief tps65023_i2c_write8 - Writes a 8 bit register in TPS65023
 *
 * @param value - the value to be written 8 bit
 * @param reg - register address (just offset will do)
 *
 * @return result of operation - 0 is success
 */
static int tps65023_i2c_write8( int reg, u8 val)
{
	struct tps65023_client *client = &tps65023_module;
	struct i2c_msg xfer_msg[1];
	u8 buf[2] = { reg, val };
	
	if (unlikely(client->inuse != TPS65023_CLIENT_USED)) {
		pr_err("tps65023_i2c_write: I2C Client not initialized\n");
		return -ENODEV;
	}

	xfer_msg[0] = (struct i2c_msg) { .addr = client->address, .flags = 0, .buf = buf, .len = 2 };
	
	if (i2c_transfer(client->client.adapter, xfer_msg, 1) == 1)
		return 0;
	
	return -EIO;
}

/**
 * @brief tps65023_i2c_read8 - Reads a 8 bit register from TPS65023
 *
 * @param *val - the value read 8 bit
 * @param reg - register address (just offset will do)
 *
 * @return result of operation - 0 is success
 */
static int tps65023_i2c_read8( int reg, u8 *val)
{
	struct tps65023_client *client = &tps65023_module;
	struct i2c_msg xfer_msg[2];

	u8 buf[1] = { reg };

	if (unlikely(client->inuse != TPS65023_CLIENT_USED)) {
		pr_err("tps65023_i2c_read: I2C Client not initialized\n");
		return -ENODEV;
	}

	xfer_msg[0] = (struct i2c_msg) { .addr = client->address, 
						.flags = 0,        .buf = buf, .len = 1 };
	xfer_msg[1] = (struct i2c_msg) { .addr = client->address, 
						.flags = I2C_M_RD, .buf = val, .len = 1 };
	
	if (i2c_transfer(client->client.adapter, xfer_msg, 2) == 2)
		return 0;
	
	return -EIO;
}

/* attach a client to the adapter */
static int tps65023_detect_client(struct i2c_adapter *adapter)
{
	int err = 0;
	struct tps65023_client *client = &tps65023_module;

 	/* Check basic functionality */
	if (!(err = i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA |
						I2C_FUNC_SMBUS_WRITE_BYTE))) {
		pr_err("TPS65023 client functionality check failed\n");
		return err;
	}
	
	if (unlikely(client->inuse)) {
		pr_err("Client %s is already in use\n", client->client_name);
		return -EPERM;
	}
	
	memset(&(client->client), 0, sizeof(struct i2c_client));
	client->client.addr	= client->address;
	client->client.adapter	= adapter;
	client->client.driver	= &tps65023_driver;
	
	memcpy(&(client->client.name), client->client_name,
			sizeof(TPS65023_CLIENT_STRING) + 1);

	pr_debug("TRY attach Slave %s on Adapter %s [%x]\n",
				client->client_name, adapter->name, err);

	if ((err = i2c_attach_client(&(client->client)))) {
		pr_err("Couldn't attach Slave %s on Adapter"
		       "%s [%x]\n", client->client_name, adapter->name, err);
		return err;
	}

	client->inuse = TPS65023_CLIENT_USED;
	return err;
}

/* adapter callback */
static int tps65023_attach_adapter(struct i2c_adapter *adapter)
{
	int ret = 0;
	
	/* hack to attach on I2C1 only */
	if (adapter->nr != CONFIG_I2C_TPS65023_ID)
	   return -1;	

	if ((ret = tps65023_detect_client(adapter)))
		goto free_client;

	return 0;

free_client:
	pr_err("client registration failed[0x%x]\n",ret);
	return ret;
}

/* adapter's callback */
static int tps65023_detach_client(struct i2c_client *iclient)
{
	int err;
	if ((err = i2c_detach_client(iclient))) {
		pr_err("Client detach failed\n");
		return err;
	}
	return 0;
}

/*
 *
 */
static int core_adjust_enable(int enable)
{
	int  ret;
	u8 val;

	struct tps65023_client *client = &tps65023_module;

	if (unlikely(client->inuse != TPS65023_CLIENT_USED)) {
		pr_err("core_adjust_enable: I2C Client is not initialized\n");
		return -ENODEV;
	}

	ret = tps65023_i2c_read8(TPS65023_CON_CTRL2, &val);  
	if (ret < 0) {
		pr_debug("failed: get CON_CTRL2\n");
		goto out;
	}
		
	if (enable)
	        val &= ~0x40;
	else
		val |= 0x40;

        ret = tps65023_i2c_write8(TPS65023_CON_CTRL2, val);  
	if (ret < 0) {
		pr_debug("failed: set CON_CTRL2 %d\n", val);
		goto out;
	}
	
out:
	return ret;
}

/*
 * This function sets the VSEL values in the Power IC. This is done in
 * the software configurable mode.
 */
int set_voltage_level (u8 vdd, u8 vsel)
{
	int  ret;
	u8 val;

	struct tps65023_client *client = &tps65023_module;

	if (unlikely(client->inuse != TPS65023_CLIENT_USED)) {
		pr_err("set_voltage_level: I2C Client is not initialized\n");
		return -ENODEV;
	}

        if (vdd != PRCM_VDD1) {
		pr_err("DVFS on VDD2: Not supported");
                ret = -EINVAL;
		goto out;		
        } 
	
	pr_debug("Setting voltage on VDD%d to %d \n",  vdd, vsel);

	ret =  tps65023_i2c_write8(TPS65023_DEFCORE, vsel);
	if (ret < 0) {
		pr_debug("failed: set DEFCORE vsel %d\n", vsel);
		goto out;
	}

#ifdef DEBUG
	ret = tps65023_i2c_read8(TPS65023_DEFCORE, &vsel);  
	if (ret < 0) {
		pr_debug("failed: get DEFCORE\n");
		goto out;
	}
	pr_debug("set_voltage_level: DEFCORE value %d\n", vsel);
#endif

	ret = tps65023_i2c_read8(TPS65023_CON_CTRL2, &val);  
	if (ret < 0) {
		pr_debug("failed: get CON_CTRL2\n");
		goto out;
	}
	
	/* set the GO bit to establish the new voltage */
	val |= 0x80;
	
        ret = tps65023_i2c_write8(TPS65023_CON_CTRL2, val);  
	if (ret < 0) {
		pr_debug("failed: set CON_CTRL2 %d\n", val);
		goto out;
	}
	
	/* Wait for voltage to stabilize */
	msleep(1);

out:
	return ret;
}
EXPORT_SYMBOL(set_voltage_level);

static int __init tps65023_init(void)
{
	int res;
	u8 val;
        tps65023_module.inuse = TPS65023_CLIENT_FREE;

	/* set a constraint on VDD2, since we can't scale VDD2 */
	co_opp_tps65023_vdd2 = constraint_get("tps65023", &cnstr_id_vdd2);
	constraint_set(co_opp_tps65023_vdd2, CO_VDD2_OPP3);
	
	if ((res = i2c_add_driver(&tps65023_driver)) < 0) {
		pr_err("TPS65023 PMIC driver registration failed\n");
		return res;
	}

	/* enable core adjustment via i2c */
	core_adjust_enable(1);
	
#ifdef DEBUG
	tps65023_i2c_read8(TPS65023_VERSION, &val);
        pr_debug("TPS650XX Version: %X \n", val);

	/* read out PGOODZ */
	tps65023_i2c_read8(TPS65023_PGOODZ, &val);
	pr_debug("PGOODZ: %d\n", val);
#endif

	return 0;
}

static void __exit tps65023_exit(void)
{
	i2c_del_driver(&tps65023_driver);
	constraint_remove(co_opp_tps65023_vdd2);
	constraint_put(co_opp_tps65023_vdd2);
}

device_initcall(tps65023_init);
