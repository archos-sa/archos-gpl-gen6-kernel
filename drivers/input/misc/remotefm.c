/*
 *   PCF8575 on AX05 Remote FM driver.
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
#include <linux/init.h>		/* Initdata			*/
#include <linux/delay.h>	/* udelay			*/
#include <linux/fs.h>
#include <asm/uaccess.h>	/* Copy to/from user		*/
#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/kthread.h>
#include <asm/semaphore.h>
#include <linux/suspend.h>
#include <linux/input.h>
#include <linux/freezer.h>

#include <linux/pcf8575.h>
#include "remotefm.h"

#define REMOTEFM_PCF8575_NAME		"remotefm"
//TODO: value
#define REMOTEFM_INPUT_POLL_DELAY	(HZ*80/1000)	// Poll remotefm input every 80msec */

#define MASK_READ		0x0FFF		// Read pins 0,1,2,3,4,5,6,7,10,11,12,13
#define MASK_WRITE		0xF000		// Write pins 14,15,16,17
#define	PC8575_REGS_INIT	0xFFFF		// All regs to on

#define LED_MASK		MASK_WRITE
#define LED_ON			1
#define LED_OFF			0


#define VOLUME_NOT_LOCKED

#define ACCESSORY_I2C_NB	3

static struct input_dev *remotefm_input_dev=NULL;
static struct task_struct *remotefm_input_task = NULL;
static int remotefm_pcf8575_use_count = 0;

// I2C
static struct i2c_client *remotefm_i2c_client_handle = NULL;
static struct semaphore remotefm_i2c_lock;

static uint16_t _pcf8575_regs_last;
static int remote_locked = 0;


/* key values are not significant in the driver side, we should only choose value <127 in input.h to get it work in avos */
#define REMOTEFM_NBKEYS	9
static const unsigned short remoteFM_keycodes[REMOTEFM_NBKEYS] = {
	KEY_KPEQUAL,//mp3
	KEY_KPPLUSMINUS,//fm
	KEY_PAUSE,//hold
	KEY_KPCOMMA,//next
	KEY_HANJA,//play pause
	KEY_RIGHTMETA,//vol up
	KEY_HANGUEL,//previous
	KEY_LEFTMETA,//vol down
	KEY_YEN,//rec
};

#define REMOTE_REPORT
#if defined REMOTE_REPORT
static char *remoteFM_keycodes_name[REMOTEFM_NBKEYS] = {
	[ 0 ] = "mp3",
	[ 1 ] = "fm",
	[ 2 ] = "hold",
	[ 3 ] = "next",
	[ 4 ] = "play, pause",
	[ 5 ] = "vol up",
	[ 6 ] = "previous",
	[ 7 ] = "vol down",
	[ 8 ] = "rec",
};

	static int report =0;
	module_param(report, int, S_IRUGO|S_IWUSR);
	#define print_report(x) {\
	if (report)\
		printk("remotefm key: %s\n",remoteFM_keycodes_name[x]);\
	}
#else
	#define print_report(x) while(0);
#endif



//////////////////////////////////////////////////////////////////////////////
// I2C
//////////////////////////////////////////////////////////////////////////////

static int remotefm_input_initialize_i2c( void );
static void remotefm_input_exit_i2c( void );
static int remotefm_input_write( uint16_t *regs );
static int remotefm_input_read( uint16_t *regs );

static int remotefm_input_initialize_i2c( void )
{
	if ( remotefm_i2c_client_handle == NULL) {

		remotefm_i2c_client_handle = i2c_get_client( I2C_DRIVERID_PCF8575, 3, remotefm_i2c_client_handle );
		if ( remotefm_i2c_client_handle == NULL ) {
			printk( KERN_ERR "remotefm_input_init: no PCF8575 found!\n" );
			return -ENODEV;
		}
	}

	init_MUTEX( &remotefm_i2c_lock );

	// Init regs
	_pcf8575_regs_last = PC8575_REGS_INIT;
	return remotefm_input_write( &_pcf8575_regs_last );
}

static void remotefm_input_exit_i2c( void )
{
	remotefm_i2c_client_handle = NULL;
}


static int remotefm_input_write( uint16_t *regs )
{
	int res;

	if ( remotefm_i2c_client_handle == NULL )
		return -ENODEV;

	// DEBUG
	//printk( KERN_INFO "remotefm_input_write: _last=%04x regs=%04x\r\n", _pcf8575_regs_last, *regs );

	down( &remotefm_i2c_lock );
	*regs &= MASK_WRITE;
	_pcf8575_regs_last = *regs;
	res = remotefm_i2c_client_handle->driver->command(remotefm_i2c_client_handle, PCF8575_WRITE, regs);
	up( &remotefm_i2c_lock );

	// DEBUG
	//printk( KERN_INFO "remotefm_input_write: _last=%04x regs=%04x\r\n", _pcf8575_regs_last, *regs );

	return res;
}

//
// remotefm_input_read()
//
// Reads the pins of the PCF8575
//	INPUT:	regs => pins 15-0
//		Set the corresponding bit to 1 if a pin is to be read
//	OUTPUT:	regs => pins 15-0
//
static int remotefm_input_read( uint16_t *regs )
{
	int res;

	if ( remotefm_i2c_client_handle == NULL )
		return -ENODEV;

	// Read current values
	down( &remotefm_i2c_lock );
	*regs = (*regs & MASK_READ) | (_pcf8575_regs_last & MASK_WRITE);
	res = remotefm_i2c_client_handle->driver->command( remotefm_i2c_client_handle, PCF8575_READ, regs );
	up( &remotefm_i2c_lock );

	return res;
}

static int remotefm_input_set_led(int leds, int state)
{
	uint16_t reg = 0;
	uint16_t ledreg = 0;
	
	remotefm_input_read(&reg);
	ledreg = reg & LED_MASK;
	printk( KERN_INFO "remotefm_input_set_led : read %#x, set leds %#x val = %d\n",ledreg,leds,state);
	if(state == LED_ON){
		ledreg &= ~leds;
	} else {
		ledreg |= leds;
	}
	printk( KERN_INFO "remotefm_input_set_led : write %#x\n",ledreg);
	return(remotefm_input_write(&ledreg));
}
//////////////////////////////////////////////////////////////////////////////
// Polling thread and key handling
//////////////////////////////////////////////////////////////////////////////

static void scan_keys( uint16_t regs )
{
	static uint16_t regs_old = ~0x0000;	// All keys released by default


	int i;
	uint16_t regs_masked;
	
	regs = ~regs;	// Switch pressed => bit to 0

	// The switches states can be read from the bit 0 of regs to the bit 8	
	for ( i = 0 ; i < REMOTEFM_NBKEYS ; i++ ) {
		regs_masked = regs & (1UL << i);
		if ( regs_masked ^ (regs_old & (1UL << i)) ) {
			if((remote_locked == 0) | (remoteFM_keycodes[i] == KEY_PAUSE)
#ifdef VOLUME_NOT_LOCKED
		| (remoteFM_keycodes[i] == KEY_LEFTMETA) 
		| (remoteFM_keycodes[i] == KEY_RIGHTMETA)
#endif
			  ){
				input_report_key( remotefm_input_dev, remoteFM_keycodes[i], regs_masked );
				print_report(i);

			}
			// DEBUG
			//printk( KERN_INFO "scan_keys: key=%i, code=%i\r\n", i, remoteFM_keycodes[i] );
		}
	}
	regs_old = regs;
}


static int remotefm_input_thread( void *null )
{	
	uint16_t regs;

	remote_locked = 0;
	remotefm_input_set_led(LED_HOLD|LED_MP3|LED_FM|LED_RECORD,LED_OFF);
	while ( !kthread_should_stop() ) {

		set_current_state( TASK_INTERRUPTIBLE );
		schedule_timeout( REMOTEFM_INPUT_POLL_DELAY );

		// swsusp cooperativity
		try_to_freeze();
// 		if ( current->flags & PF_FREEZE )
// 			refrigerator( PF_FREEZE );

		// Read and report keys
		regs = 0xffff;	// Read all readable pins

		//HACK: when device is unplugged, input_read return a regs=0x0000
		//therefore, scan_key generate 9 key events
		//so we test if regs != 0x0000
		if ( remotefm_input_read( &regs ) == 0 && regs != 0x0000 ) {
			scan_keys( regs );
		}
	}
	return 0;
}

static void remotefm_stop_thread( void )
{	
	if ( remotefm_input_task ) {
		kthread_stop( remotefm_input_task );
		remotefm_input_task = NULL;
	}
}


//////////////////////////////////////////////////////////////////////////
// Fops
//////////////////////////////////////////////////////////////////////////

static int remotefm_pcf8575_ioctl( struct inode *inode, struct file *file,
			  unsigned int cmd, unsigned long arg )
{
	int ret = 0;

	uint16_t pcf8575_regs;

	switch ( cmd )
	{
		case STOP_READ_THREAD:
			printk( KERN_INFO "remotefm_pcf8575_ioctl : STOP_READ_THREAD\n" );
			remotefm_stop_thread();
		break;
		case START_READ_THREAD:
			printk( KERN_INFO "remotefm_pcf8575_ioctl : START_READ_THREAD\n" );
			if ( remotefm_input_task == NULL ) {
				remotefm_input_task = kthread_run( remotefm_input_thread, NULL, "remotefm_input_thread" );
			}
		break;
		
		case READ_REGS:
			printk( KERN_INFO "remotefm_pcf8575_ioctl : READ_REGS\n" );
			pcf8575_regs = 0xffff;	// Read all possible pins
			remotefm_input_read( &pcf8575_regs );
			//TODO
		break;
		
		case WRITE_REGS:
			printk( KERN_INFO "remotefm_pcf8575_ioctl : WRITE_REGS\n" );
			pcf8575_regs = (uint16_t)arg;
			remotefm_input_write( &pcf8575_regs );
			//TODO: Return code
		break;
		
		case TOGGLE_REGS:
			//TODO?
		break;

		case SET_LED_ON:
			remotefm_input_set_led((uint16_t)arg,LED_ON);
			break;
		case SET_LED_OFF:
			remotefm_input_set_led((uint16_t)arg,LED_OFF);
			break;
		case REMOTE_LOCK:
			remote_locked = (int)arg;
			if(remote_locked){
				remotefm_input_set_led(LED_HOLD,LED_ON);
				printk( KERN_INFO "input  : KEY LOCK : lock\n");
			} else {
				remotefm_input_set_led(LED_HOLD,LED_OFF);
				printk( KERN_INFO "input  : KEY LOCK : unlock\n");
			}
			break;
		default:
		break;
	}

	return ret;
}

static int remotefm_pcf8575_open(struct inode * inode, struct file * file)
{
	if ( remotefm_pcf8575_use_count > 0 )
		return -EBUSY;
	
	printk( KERN_INFO "remotefm_pcf8575_open\r\n" );

	// The remotefm_input_initialize_i2c() should have been called at this point
	//remotefm_input_initialize_i2c();

	remotefm_pcf8575_use_count++;	
        return 0;
}

static int remotefm_pcf8575_release(struct inode * inode, struct file * file)
{
	printk(KERN_INFO "remotefm_pcf8575_release\r\n");
	remotefm_pcf8575_use_count--;
	return 0;
}


static struct file_operations remotefm_pcf8575_fops = {
	.owner		= THIS_MODULE,
	.llseek         = no_llseek,
	.ioctl		= remotefm_pcf8575_ioctl,
	.open           = remotefm_pcf8575_open,
	.release        = remotefm_pcf8575_release,
};

static struct miscdevice remotefm_pcf8575_miscdev = {
        .minor	= REMOTEFM_PCF8575_MINOR,
	.name	= REMOTEFM_PCF8575_NAME,
	.fops	= &remotefm_pcf8575_fops,
};



static int __init remotefm_input_init(void)
{
	int ret = 0;
	int i;

	_pcf8575_regs_last = PC8575_REGS_INIT;
	remote_locked = 0;

	// 0- allocate input device
	remotefm_input_dev=input_allocate_device();
	if (remotefm_input_dev == NULL) {
		printk(KERN_ERR "Cannot allocate fmremote input device\n");
		return -1;
	}

	// 1- Initialize I2C
	ret = remotefm_input_initialize_i2c();


	if ( ret ) {
		printk( KERN_ERR "remotefm_input_init: unable to initialize I2C\r\n" );
		//TODO: proper return code
		ret = 1;
		goto remotefm_input_init_err1;
	}

	// 2- Initialize the input device driver
	memset( remotefm_input_dev, 0, sizeof(remotefm_input_dev) );	
	remotefm_input_dev->name = "remotefm_input";
	remotefm_input_dev->evbit[LONG(EV_KEY)] |= BIT(EV_KEY) | BIT(EV_REP);

	for (i = 0 ; i < REMOTEFM_NBKEYS ; i++) {
		set_bit( remoteFM_keycodes[i], remotefm_input_dev->keybit);
	}
	
	input_register_device( remotefm_input_dev );

	//TODO: change?
	remotefm_input_dev->rep[REP_DELAY]  = 100;
	remotefm_input_dev->rep[REP_PERIOD] = 100;

	// Kernel thread init and start
	//TODO: thread params? (see /home/lorriaux/svn/605f/avx/linux_davinci/drivers/input/keyboard/ax05)
	remotefm_input_task = kthread_run( remotefm_input_thread, NULL, "remotefm_input_thread" );
	if ( remotefm_input_task <= 0 ) {
		printk( KERN_ERR "remotefm_input_init: unable to start kernel thread\r\n" );
		//TODO: proper return code
		ret = 1;
		goto remotefm_input_init_err2;
	}

	// 3- Initialize the "write" driver
	ret = misc_register( &remotefm_pcf8575_miscdev );
	if( ret < 0 ) {
		printk(KERN_INFO "remotefm_input_init : cannot register remotefm_pcf8575_miscdev\r\n");
		goto remotefm_input_init_err3;
	}

	printk( KERN_INFO "Archos Remote FM: keyboard input device installed\n");
	return ret;

	// Error cases
remotefm_input_init_err3:
	kthread_stop( remotefm_input_task );
	remotefm_input_task = NULL;
remotefm_input_init_err2:
	input_unregister_device( remotefm_input_dev );
	remotefm_input_exit_i2c();
remotefm_input_init_err1:
	return ret;
}

static void __exit remotefm_input_exit( void )
{
	// 3-
	misc_deregister( &remotefm_pcf8575_miscdev );
	// 2-
	remotefm_stop_thread();
	input_unregister_device( remotefm_input_dev );
	// 1-
	remotefm_input_exit_i2c();
	
	printk( "%s: done\r\n", __FUNCTION__ );
}

module_init(remotefm_input_init);
module_exit(remotefm_input_exit);

//  ----------------------------------------------------------------------------
//			Module Info
//  ----------------------------------------------------------------------------

MODULE_AUTHOR("ARCHOS");
MODULE_DESCRIPTION("Driver for the remote keyboard on the ARCHOS FM Receiver.");
MODULE_LICENSE("GPL");

