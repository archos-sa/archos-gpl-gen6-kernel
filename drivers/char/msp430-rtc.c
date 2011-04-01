/*
 * msp430-rtc.c
 *
 * Copyright (c) 2006 Archos
 * Author: Xavier Leclercq
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/proc_fs.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mutex.h>

#include <linux/msp430.h>

#include <asm/uaccess.h>
#include <asm/rtc.h>


#define ATMEGA_RTC_VERSION	"0.01"

static struct mutex rtc_mutex;

static int atmega_rtc_gettime( struct rtc_time *rtc_tm )
{
	unsigned long nowtime;

	if (atmega_read_value( ATMEGA_I2C_TIME_REG, &nowtime, ATMEGA_I2C_TIME_SIZE ) < 0)
		return -EIO;

	rtc_time_to_tm( nowtime, rtc_tm );

	return 0;
}

static int atmega_rtc_settime( struct rtc_time *rtc_tm )
{
	unsigned long nowtime;
	struct atmega_exchange_table table = { 0,0,0,0,0 };

	rtc_tm_to_time( rtc_tm, &nowtime);
	table.value = nowtime;
	table.control_byte = ATMEGA_I2C_CTRL_CMD_SET_RTC;
	return atmega_write_table( &table );

}

static int atmega_rtc_getalarm( struct rtc_wkalrm *rtc_alr )
{
	unsigned long nowtime;

	if ( atmega_read_value( ATMEGA_I2C_ALARM_REG, &nowtime, ATMEGA_I2C_ALARM_SIZE ) < 0)
		return -EIO;

	if ( nowtime ) {
		struct rtc_time *alr_tm = &rtc_alr->time;

		rtc_alr->enabled = 1;
		rtc_time_to_tm( nowtime, alr_tm );
	} else {
		rtc_alr->enabled = 0;
	}
	return 0;
}

static int atmega_rtc_setalarm( struct rtc_wkalrm *rtc_alr )
{
	struct rtc_time *alr_tm = &rtc_alr->time;
	struct atmega_exchange_table table = { 0,0,0,0,0 };

	if ( rtc_alr->enabled ) {
		unsigned long nowtime;

		rtc_tm_to_time( alr_tm, &nowtime);
		table.value = nowtime;
		table.control_byte = ATMEGA_I2C_CTRL_CMD_SET_ALARM;
	} else {
		table.control_byte = ATMEGA_I2C_CTRL_CMD_RESET_ALARM;
	}
	return atmega_write_table( &table );

}

static int atmega_rtc_open( void )
{
	if (mutex_lock_interruptible(&rtc_mutex))
		return -EINTR;
	
	return 0;
}

static void atmega_rtc_release( void )
{
	mutex_unlock(&rtc_mutex);
}

static struct rtc_ops atmega_rtcops = {
	.owner		= THIS_MODULE,
	.open		= atmega_rtc_open,
	.release	= atmega_rtc_release,
	.ioctl		= NULL,
	.read_time	= atmega_rtc_gettime,
	.set_time	= atmega_rtc_settime,
	.read_alarm	= atmega_rtc_getalarm,
	.set_alarm	= atmega_rtc_setalarm,
	.proc	        = NULL,
};


static int __devexit atmega_rtc_remove( struct platform_device *pdev )
{
	unregister_rtc( &atmega_rtcops);
	mutex_destroy(&rtc_mutex);
	
	return 0;
}

static int __devinit atmega_rtc_probe( struct platform_device  *pdev )
{
	printk( "%s: device=%p\n", __FUNCTION__, pdev );

	mutex_init(&rtc_mutex);
	register_rtc( &atmega_rtcops );

	return 0;
}

static struct platform_driver atmega_rtcdrv = {
	.probe		= atmega_rtc_probe,
	.remove		= __devexit_p(atmega_rtc_remove),
	.driver		= {
		.name	= "atmega-rtc",
		.owner	= THIS_MODULE,
	},
};

static struct platform_device *atmega_rtcdevice;

/* ++++++++++++++++++ Module Init +++++++++++++++++++++++*/

static int __init atmega_rtc_init( void )
{
	int ret;

	printk( "ATMEGA RTC Driver v%s for GEN6\n", ATMEGA_RTC_VERSION );

	atmega_rtcdevice = platform_device_alloc( "atmega-rtc", -1 );
	if ( !atmega_rtcdevice )
		return -ENOMEM;

	ret = platform_device_register( atmega_rtcdevice );
	if ( ret < 0 ) {
		platform_device_put( atmega_rtcdevice );
		return ret;
	}

	ret = platform_driver_register( &atmega_rtcdrv );
	if ( ret < 0 )
		platform_device_unregister( atmega_rtcdevice );

	printk( "ATMEGA RTC Driver register success\n" );

	return ret;
}

static void __exit atmega_rtc_exit( void )
{
	platform_driver_unregister( &atmega_rtcdrv );
	platform_device_unregister( atmega_rtcdevice );
}

module_init( atmega_rtc_init );
module_exit( atmega_rtc_exit );

/* Module information */
MODULE_AUTHOR("Archos S.A.");
MODULE_DESCRIPTION("ATMEGA RTC Driver for GEN6");
MODULE_LICENSE("GPL");

