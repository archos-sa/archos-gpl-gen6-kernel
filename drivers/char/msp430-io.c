/*
 * atmega-io.c
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
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/delay.h>

#include <linux/msp430-io.h>
#include <linux/msp430.h>

#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#include <asm/mach/irq.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/arch/battery.h>
#include <linux/poll.h>
#include <asm/arch/board.h>
#include <asm/arch/archosg6-gpio.h>

#include <asm/mach-types.h>

#define ATMEGA_IO_VERSION	"0.04"

#define ATMEGA_IO_IS_OPEN	0x01	/* means /dev/atmg is in use */

#define	ATMG_GPIO_NUM		170
#define ATMEGA_KB_DEV_NAME "atmega-key"

extern void platform_enable_irr_io(int en);
extern int platform_get_dock_dc(struct device* dev);


static unsigned char atmega_io_open_status;
static int atmega_io_irqno = NO_IRQ;
static unsigned long atmega_io_status;
static unsigned long atmega_version;
static unsigned long atmega_vbatt_back;
static struct workqueue_struct *atmega_io_workqueue;
static wait_queue_head_t atmega_io_wait_queue;
static int atmega_io_status_available;

static struct input_dev* atmega_input_dev;

static const struct str_module_resistors {
	int module_id;
	int min_resistor;
	int max_resistor;
}  module_resistors[] = 
{
	{ MODULE_ID_TV_CRADLE,		43,	51 },
	{ MODULE_ID_REMOTE_FM,		51,	62 },
	{ MODULE_ID_VRA_GEN6,		62,	75 },
	{ MODULE_ID_GPS_WITHOUT_TMC,	75,	91 },
	{ MODULE_ID_USB_HOST_CABLE,	91,	110},
	{ MODULE_ID_BATTERY_DOCK_GEN6,	110,	135},
	{ MODULE_ID_POWER_CABLE,	135,	165},
	{ MODULE_ID_SERIAL_ADAPTER,	165,	200},
	{ MODULE_ID_PC_CABLE,		200,	245},
	{ MODULE_ID_UNUSED10,		245,	300},
	{ MODULE_ID_GPS_WITH_TMC,	300,	360},
	{ MODULE_ID_UNUSED12,		360,	430},
	{ MODULE_ID_MINI_DOCK,		430,	515},
	{ MODULE_ID_DVBT_SNAP_ON,	516,	620},
	{ MODULE_ID_MUSIC_DOCK,		620,	750},
};

#undef DEBUG
#ifdef DEBUG
#define DEBUGMSG(ARGS...)  printk("<%s>: ",__FUNCTION__);printk(ARGS)
#else
#define DEBUGMSG( x... )
#endif

/* IRQ Handler */
 
static void atmega_io_getstatus_on_irq( void *data )
{
	unsigned long atmega_io_status_bak = atmega_io_status;
	int aux_change = 0;

	atmega_read_value( ATMEGA_I2C_STATUS_REG, &atmega_io_status, ATMEGA_I2C_STATUS_SIZE );
	DEBUGMSG( "Status %lx\n", atmega_io_status );

	if ( atmega_version == 0 ) {
		atmega_read_value( ATMEGA_I2C_VERSION_REG, &atmega_version, ATMEGA_I2C_VERSION_SIZE );
		DEBUGMSG("ATMEGA No.%lX\n",atmega_version );
	}

	if ( !machine_is_archos_g6tv() ) {

		if ( atmega_version >=0x060a ) {
			if ( !(atmega_io_status_bak & ATMEGA_IO_STATUS_KBON_PRESSED_BIT) ) {
				if (atmega_io_status & ATMEGA_IO_STATUS_KBON_PRESSED_BIT) {
					printk( "ATMEGA KBON pressed\n");
					input_event(atmega_input_dev, EV_KEY, KEY_POWER, 1);
					input_sync(atmega_input_dev);
				}
			} else {		
				if (!(atmega_io_status & ATMEGA_IO_STATUS_KBON_PRESSED_BIT)) {
					printk( "ATMEGA KBON released\n");
					input_event(atmega_input_dev, EV_KEY, KEY_POWER, 0);
					input_sync(atmega_input_dev);
				}
			}
		}

		if ( (atmega_io_status_bak & ATMEGA_IO_STATUS_AUX_DETECTED_BIT) != (atmega_io_status & ATMEGA_IO_STATUS_AUX_DETECTED_BIT))
			aux_change = 1;

		if ( ( atmega_version <= 0x060a ) || ( !machine_is_archos_g6l() && atmega_version >= 0x060b ) ) {
			if ( ( aux_change == 1 ) 
				|| ( ( atmega_io_status_bak & ATMEGA_IO_STATUS_USB_DETECTED_BIT) != (atmega_io_status & ATMEGA_IO_STATUS_USB_DETECTED_BIT)) 
				|| ( (atmega_io_status_bak & ATMEGA_IO_STATUS_DC_DETECTED_BIT) != (atmega_io_status & ATMEGA_IO_STATUS_DC_DETECTED_BIT) ) )	{
				archos_needs_battery( !!(atmega_io_status & ATMEGA_IO_STATUS_USB_DETECTED_BIT), atmega_io_status & ATMEGA_IO_STATUS_DC_DETECTED_BIT );
			}
		} else if ( ( aux_change == 1 ) || ( atmega_io_status_bak & ( ATMEGA_IO_STATUS_DCJACK_DETECT_BIT | ATMEGA_IO_STATUS_DC_DETECTED_BIT) ) 
				!= ( atmega_io_status & ( ATMEGA_IO_STATUS_DCJACK_DETECT_BIT | ATMEGA_IO_STATUS_DC_DETECTED_BIT)) ) { 
			
			if ( (atmega_io_status & ATMEGA_IO_STATUS_DCJACK_DETECT_BIT) ) {
				printk( "ATMEGA DCJACK detected\n");
				archos_needs_battery( 0, 1 );
			} else if ( (atmega_io_status & ATMEGA_IO_STATUS_DC_DETECTED_BIT) && ( atmega_io_getUsbType() == MODULE_ID_TV_CRADLE ) ){
				printk( "ATMEGA G6L TV_cradle detected\n");
				archos_needs_battery( 0, 1 );
			} else {
 				archos_needs_battery( 0, 0 );
			}
		}
	}

	if ( atmega_io_status & ATMEGA_IO_STATUS_ALARM_REACHED_BIT ){
		printk( "ATMEGA Alarm reached\n");
	} 

	atmega_io_status_available = 1;
	wake_up_interruptible(&atmega_io_wait_queue);
}

static DECLARE_WORK( atmega_io_worker,(work_func_t) atmega_io_getstatus_on_irq);

/* ----- Utility functions --------------------------------------------	*/

static irqreturn_t atmega_io_statusirq( int irq, void *id )
{
	DEBUGMSG("atmega_io_statusirq\n");
	queue_work( atmega_io_workqueue, &atmega_io_worker );

	return IRQ_HANDLED;
}

/* Register read/write */
static int atmega_io_gettime( struct rtc_time * arg )
{
	struct rtc_time *rtc_tm = arg;
	unsigned long nowtime;

	int res = atmega_read_value( ATMEGA_I2C_TIME_REG, &nowtime, ATMEGA_I2C_TIME_SIZE );
	DEBUGMSG("rtc_time_tp_tm,%lu\n",nowtime);
	rtc_time_to_tm( nowtime, rtc_tm );
	DEBUGMSG("isdst: %d day: %d wday: %d year: %d mon: %d mday; %d hour: %d min: %d sec: %d\t\n",
		rtc_tm->tm_isdst,rtc_tm->tm_yday,rtc_tm->tm_wday,rtc_tm->tm_year,rtc_tm->tm_mon,rtc_tm->tm_mday,rtc_tm->tm_hour,rtc_tm->tm_min,rtc_tm->tm_sec);
	return res;
}

static int atmega_io_settime( struct rtc_time * arg )
{
	struct rtc_time *rtc_tm = arg;
	unsigned long nowtime;
	struct atmega_exchange_table table = { 0, ATMEGA_I2C_CTRL_CMD_SET_RTC, 0,0,0 };
	rtc_tm_to_time( rtc_tm, &nowtime);
	table.value = nowtime;
	return ( atmega_write_table( &table ));
}

static int atmega_io_dump( unsigned long arg )
{
	unsigned int i;
	unsigned long data;

	for (i=ATMEGA_I2C_READ_REG_BASE; i < ATMEGA_I2C_READ_REG_BASE+26; i++) {
		data = atmega_read_value(i, &data,1);
		printk(KERN_INFO "atmg reg[%02X]: %lX\n", i, data & 0xFF);
	}
	for (i=ATMEGA_I2C_REREAD_REG_BASE; i < ATMEGA_I2C_REREAD_REG_BASE+10; i++) {
		data = atmega_read_value(i, &data,1);
		printk(KERN_INFO "atmg exchangtable[%02X]: %lX\n", i, data & 0xFF);
	}
	return 0;
}

static int atmega_io_getalarm( unsigned long arg )
{
	struct rtc_wkalrm *rtc_alr = (struct rtc_wkalrm *)arg;
	unsigned long nowtime;

	int res = atmega_read_value( ATMEGA_I2C_ALARM_REG, &nowtime, ATMEGA_I2C_ALARM_SIZE );

	if ( nowtime ) {
		struct rtc_time *alr_tm = &rtc_alr->time;

		rtc_alr->enabled = 1;
		rtc_time_to_tm( nowtime, alr_tm );
	} else {
		rtc_alr->enabled = 0;
	}
	
	return res;
}

static int atmega_io_setalarm( unsigned long arg )
{
	struct rtc_wkalrm *rtc_alr = (struct rtc_wkalrm *)arg;
	struct atmega_exchange_table table = { 0,0,0,0,0 };

	if ( rtc_alr->enabled ) {
		unsigned long nowtime;
		struct rtc_time *alr_tm = &rtc_alr->time;

		rtc_tm_to_time( alr_tm, &nowtime);
		table.value = nowtime;
		table.control_byte = ATMEGA_I2C_CTRL_CMD_SET_ALARM;
	} else {
		table.control_byte = ATMEGA_I2C_CTRL_CMD_RESET_ALARM;
	}
	return ( atmega_write_table( &table ));

}

static int atmega_io_reset_omap( void )
{
	struct atmega_exchange_table table = { 0, ATMEGA_I2C_CTRL_CMD_RESET_DAVINCI, 0,0,0 };
	return ( atmega_write_table( &table ) );
}

static int atmega_io_set_shutdown( void )
{
	struct atmega_exchange_table table = { 0, ATMEGA_I2C_CTRL_CMD_SHUTDOWN, 0,0,0 };
	return ( atmega_write_table( &table ) );
}

static int atmega_io_set_plug_power( unsigned long arg )
{
	return 0;
}

struct str_led_cmd {
	unsigned char period;
	unsigned char blink;
};

static int atmega_io_set_led_charge( unsigned long arg )
{
	struct str_led_cmd *led_cmd = (struct str_led_cmd *)arg;

	struct atmega_exchange_table table = { 1, ATMEGA_I2C_CTRL_CMD_CHG_LED_TOGGLE, 0,0,0 };

	DEBUGMSG(KERN_DEBUG "Set charge led: arg %lx, p1:%d(%04x), p2:%d(%04x)\n",arg,led_cmd->period,led_cmd->period,led_cmd->blink,led_cmd->blink);
	table.padding1 = led_cmd->period;
	table.padding2 = led_cmd->blink;

	return ( atmega_write_table( &table ) );
}

static int atmega_io_set_led_charge_off( unsigned long arg )
{
	struct atmega_exchange_table table = { 1, ATMEGA_I2C_CTRL_CMD_CHG_LED_TOGGLE, 10,0,0 };
	return ( atmega_write_table( &table ) );
}

int atmega_io_set_charge_mode( unsigned long arg )
{
	struct atmega_exchange_table table = { 0, ATMEGA_I2C_CTRL_CMD_CHARGEMODE, 0,0,0 };
	table.value = arg;

	return ( atmega_write_table( &table ) );
}

static int atmega_io_getversion( unsigned long arg )
{
	void __user *version = (void __user *)arg;
	unsigned long value;

	atmega_read_value( ATMEGA_I2C_VERSION_REG, &value, ATMEGA_I2C_VERSION_SIZE );
	return copy_to_user( version, &value, sizeof(value) ) ? -EFAULT : 0;
}

static int atmega_io_getboardID( unsigned long arg )
{
	void __user *boardID = (void __user *)arg;
	unsigned long value;

	atmega_read_value( ATMEGA_I2C_BOARDID_REG, &value, ATMEGA_I2C_BOARDID_SIZE );
	return copy_to_user( boardID, &value, sizeof(value) ) ? -EFAULT : 0;
}

static int atmega_io_getchecksum( unsigned long arg )
{
	void __user *version = (void __user *)arg;
	unsigned long value;

	atmega_read_value( ATMEGA_I2C_CS_REG, &value, ATMEGA_I2C_CS_SIZE );
	return copy_to_user( version, &value, sizeof(value) ) ? -EFAULT : 0;
}

/* flush the queue and get the last status value */
static int atmega_io_getstatus( unsigned long arg )
{
	void __user *status = (void __user *)arg;
	unsigned long value;

// 	atmega_read_value( ATMEGA_I2C_STATUS_REG, &atmega_io_status, ATMEGA_I2C_STATUS_SIZE );
//DEBUGMSG("atmega_io_getstatus\n");
	value = atmega_io_status;
	return copy_to_user( status, &value, sizeof(value) ) ? -EFAULT : 0;
}

static int atmega_io_getvbatt( unsigned long arg )
{
	void __user *vbatt = (void __user *)arg;
	unsigned long adc0,adc3,value/*,value_ref*/;

	atmega_read_value( ATMEGA_I2C_ADC0_REG, &adc0, ATMEGA_I2C_ADC0_SIZE );
	atmega_read_value( ATMEGA_I2C_ADC3_REG, &adc3, ATMEGA_I2C_ADC3_SIZE );
// 	DEBUGMSG("atmega_io : adc0:%lu adc3:%lu\n",adc0,adc3);
// 	value =  adc0*11*43047/1024/100;
// 	value_ref = adc3*11*430/1024;
	if ( adc3 == 0 ) {
		value = atmega_vbatt_back;
		printk(KERN_DEBUG "atmega_io : getVbatt ADC3 is 0 \n");
	} else {
		value = adc0 * 2500 / adc3;
		atmega_vbatt_back = value;
	}
// 	DEBUGMSG("atmega_io : atmega_io_getvbatt: %lu vbatt reference: %lu\n",value, value_ref);
	return copy_to_user( vbatt, &value, sizeof(value) ) ? -EFAULT : 0;
}

static int atmega_io_getmodule( unsigned long arg )
{
	void __user *module = (void __user *)arg;
	unsigned long value;
	unsigned long adc1,adc2;

	atmega_read_value( ATMEGA_I2C_ADC1_REG, &adc1, ATMEGA_I2C_ADC1_SIZE );
	atmega_read_value( ATMEGA_I2C_ADC2_REG, &adc2, ATMEGA_I2C_ADC2_SIZE );

	DEBUGMSG("atmega_io : GET MODULE TYPE adc1:%lu adc2:%lun\n",adc1,adc2);

	value = adc1*510*1000/(2*adc2-adc1)/1000;

	DEBUGMSG("atmega_io : GET MODULE TYPE resistor value %lu\n",value);	
	return copy_to_user( module, &value, sizeof(value) ) ? -EFAULT : 0;
}

int atmega_io_getUsbType( void )
{
	unsigned long value;
	unsigned long adc1,adc2;
	int i;

	atmega_read_value( ATMEGA_I2C_ADC1_REG, &adc1, ATMEGA_I2C_ADC1_SIZE );
	atmega_read_value( ATMEGA_I2C_ADC2_REG, &adc2, ATMEGA_I2C_ADC2_SIZE );

// 	DEBUGMSG("atmega_io : GET MODULE TYPE adc1:%lu adc2:%lun\n",adc1,adc2);

	value = adc1*510*1000/(2*adc2-adc1)/1000;

	for ( i = 0; i < sizeof( module_resistors ) / sizeof( struct str_module_resistors ); i++ ) {
		if (( value < module_resistors[i].max_resistor ) && value > module_resistors[i].min_resistor) {
			return module_resistors[i].module_id;
			DEBUGMSG("atmega_io : GET MODULE TYPE resistor value %lu, modules number %d\n",value,i);	
			break;
		}
	}
	return 0;
}

int atmega_io_battery_dock_check_dcin (void)
{
	int dc_in = 0;
	omap_cfg_reg(AF14_3430_GPIO184);
	mdelay(1);
	if ( omap_get_gpio_datain(184) ) {
		DEBUGMSG("not really got DC-in from battery dock.\n");
	} else {
		DEBUGMSG("really got DC-in from battery dock.\n");
		dc_in = 1;
	} 

	mdelay(1);
	omap_cfg_reg(AF14_3430_I2C3_SCL);
	return dc_in;
}

static void atmega_io_battery_dock_reget_battery (void)
{
	if ( atmega_io_battery_dock_check_dcin() ) {
		archos_needs_battery( 1,1 );
	}
	else {
		archos_needs_battery( 1,0 );
	}
}

static int atmega_io_battery_dock_read_DC_connect( unsigned long arg )
{
	void __user *dcin = (void __user *)arg;
	unsigned long value;

//DEBUGMSG("atmega_io_read_DC_connect\n");
	value = atmega_io_battery_dock_check_dcin();
	return copy_to_user( dcin, &value, sizeof(value) ) ? -EFAULT : 0;
}

static int atmega_io_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
// 	DEBUGMSG( "atmega_io : atmega_io_ioctl cmd->%d\n", cmd );


	switch( cmd ) {
	case ATMEGA_IO_READ_TIME:
		DEBUGMSG( "atmega_io : ATMEGA_IO_READ_TIME\n" );
		return atmega_io_gettime( (struct rtc_time *)arg );

	case ATMEGA_IO_READ_ALARM:
		DEBUGMSG( "atmega_io : ATMEGA_IO_READ_ALARM\n" );
		return atmega_io_getalarm( arg );

	case ATMEGA_IO_READ_VERSION:
		DEBUGMSG(KERN_DEBUG "atmega_io : ATMEGA_IO_READ_VERSION\n" );
		return atmega_io_getversion( arg );

	case ATMEGA_IO_READ_STATUS:
// 		DEBUGMSG( "atmega_io : ATMEGA_IO_READ_STATUS\n" );
		return atmega_io_getstatus( arg );

	case ATMEGA_IO_READ_VBATT:
// 		DEBUGMSG( "atmega_io : ATMEGA_IO_READ_VBATT\n" );
		return atmega_io_getvbatt( arg );

	case ATMEGA_IO_READ_MODULE_TYPE:
		DEBUGMSG( "atmega_io : ATMEGA_IO_READ_MODULE_TYPE\n" );
		return atmega_io_getmodule( arg );

	case ATMEGA_IO_SET_TIME:
		DEBUGMSG( "atmega_io : ATMEGA_IO_SET_TIME\n" );
		return atmega_io_settime( (struct rtc_time *)arg );

	case ATMEGA_IO_SET_ALARM:
		DEBUGMSG( "atmega_io : ATMEGA_IO_SET_ALARM\n" );
		return atmega_io_setalarm( arg );

	case ATMEGA_IO_RESET_OMAP:
		DEBUGMSG( "atmega_io : ATMEGA_IO_RESET_OMAP\n" );
		return atmega_io_reset_omap();

	case ATMEGA_IO_SET_SHUTDOWN:
		DEBUGMSG( "atmega_io : ATMEGA_IO_SET_SHUTDOWN\n" );
		return atmega_io_set_shutdown();

	case ATMEGA_IO_SET_PLUG_POWER:
		DEBUGMSG( "atmega_io : ATMEGA_IO_SET_PLUG_POWER\n" );
		return atmega_io_set_plug_power( arg );

	case ATMEGA_IO_SET_LED_CHARGE:
		DEBUGMSG( "atmega_io : ATMEGA_IO_SET_LED_CHARGE\n" );
		return atmega_io_set_led_charge( arg );

	case ATMEGA_IO_SET_LED_CHARGE_OFF:
		DEBUGMSG( "atmega_io : ATMEGA_IO_SET_LED_CHARGE\n" );
		return atmega_io_set_led_charge_off( arg );

	case ATMEGA_IO_SET_CHARGE_MODE:
		DEBUGMSG( "atmega_io : ATMEGA_IO_SET_CHARGE_MODE\n" );
		return atmega_io_set_charge_mode( arg );

	case ATMEGA_IO_TOGGLE_IRR_IO:
		DEBUGMSG( "atmega_io : ATMEGA_IO_TOGGLE_IRR_IO\n" );
		DEBUGMSG("ATMEGA_IO_TOGGLE_IRR_IO: %ld\n", arg);
#if defined(CONFIG_INPUT_AV600_IRR) || defined(CONFIG_DAVINCI_IRBLASTER)
		platform_enable_irr_io( arg );
#endif
		return 0;

	case ATMEGA_DUMP:
		atmega_io_dump(arg);
		return 0;

	case ATMEGA_IO_READ_BOARD_ID:
		DEBUGMSG( "atmega_io : ATMEGA_IO_READ_BOARD_ID\n" );
		atmega_io_getboardID(arg);
		return 0;

	case ATMEGA_IO_READ_CHECK_SUM:
		DEBUGMSG( "atmega_io : ATMEGA_IO_READ_CHECK_SUM\n" );
		atmega_io_getchecksum(arg);
		return 0;

	case ATMEGA_IO_BATTERYDOCK_REGET_BATTERY:
		DEBUGMSG( "atmega_io : ATMEGA_IO_BATTERYDOCK_REGET_BATTERY\n" );
		atmega_io_battery_dock_reget_battery();
		return 0;
	case ATMEGA_IO_BATTERYDOCK_READ_DC_CONNECT:
		DEBUGMSG( "atmega_io : ATMEGA_IO_BATTERYDOCK_READ_DC_CONNECT\n" );
		atmega_io_battery_dock_read_DC_connect(arg);
		return 0;
	default:
		return -EINVAL;
	}
}

static int atmega_io_open( struct inode *inode, struct file *file )
{
	if ( atmega_io_open_status & ATMEGA_IO_IS_OPEN )
		return -EBUSY;

	// Initialize IO status
	atmega_read_value( ATMEGA_I2C_STATUS_REG, &atmega_io_status, ATMEGA_I2C_STATUS_SIZE );

	if (!machine_is_archos_g6tv()) {

		if (machine_is_archos_g6l()) {
			if ( (atmega_io_status & ATMEGA_IO_STATUS_DCJACK_DETECT_BIT) ) {
//				printk( "ATMEGA DCJACK detected\n");
				archos_needs_battery( 0, 1 );
			} else if ( (atmega_io_status & ATMEGA_IO_STATUS_DC_DETECTED_BIT) && ( atmega_io_getUsbType() == MODULE_ID_TV_CRADLE ) ){
//				printk( "ATMEGA G6L TV_cradle detected\n");
				archos_needs_battery( 0, 1 );
			} else {
 				archos_needs_battery( 0, 0 );
			}
		} else {
			archos_needs_battery(!!(atmega_io_status & ATMEGA_IO_STATUS_USB_DETECTED_BIT),
						atmega_io_status & ATMEGA_IO_STATUS_DC_DETECTED_BIT);
		}
	}
	atmega_io_open_status |= ATMEGA_IO_IS_OPEN;

	return 0;
}

static int atmega_io_release( struct inode *inode, struct file *file )
{
	atmega_io_open_status &= ~ATMEGA_IO_IS_OPEN;
	return 0;
}

/* only for blocking read from the status queue. The get status ioctl flushs the queue */
static ssize_t atmega_io_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
	int len = 0;
		
	if (count > 0) {
		*buffer = atmega_io_status;
		len = 1;
	}
	
	atmega_io_status_available = 0;

	return len;
}

static unsigned int atmega_io_poll(struct file * file, poll_table * wait)
{
	unsigned int mask = 0;
	
	if (atmega_io_status_available)
		mask |= POLLIN | POLLRDNORM;
	
	poll_wait(file, &atmega_io_wait_queue, wait);

	return mask;
}

static ssize_t store_charge_rate(struct device *dev, struct device_attribute *attr, const char* buf, size_t len)
{
	int on_off = simple_strtol(buf, NULL, 10);	
	if( on_off > 1 || on_off < -1 )
		return -EINVAL;

	atmega_io_set_charge_mode( on_off );
	return len;
}

static DEVICE_ATTR(charge_rate, S_IWUSR, NULL, store_charge_rate);

/*
 *	The various file operations we support.
 */

static struct file_operations atmega_io_fops = {
	.owner		= THIS_MODULE,
	.ioctl		= atmega_io_ioctl,
	.open		= atmega_io_open,
	.release	= atmega_io_release,
	.read		= atmega_io_read,
	.poll		= atmega_io_poll,
};
static struct miscdevice atmega_io_dev =
{
	.minor		= ATMEGA_IO_MINOR,
	.name		= "atmg",
	.fops		= &atmega_io_fops,
};


static int atmega_key_probe(void)
{
	int error;

	DEBUGMSG("atmega_key_probe \n" );

	atmega_input_dev = input_allocate_device();
	if (!atmega_input_dev)
		return -ENOMEM;

	atmega_input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_REP);

	atmega_input_dev->name = ATMEGA_KB_DEV_NAME;
	atmega_input_dev->phys = "atmega-key/input0";

	input_set_capability(atmega_input_dev, EV_KEY, KEY_POWER);

	error = input_register_device(atmega_input_dev);
	if (error) {
		printk(KERN_ERR "Unable to register atmega-key input device\n");
		input_free_device(atmega_input_dev);
		return error;
	}

	return 0;
}

static int atmega_io_probe( struct platform_device *pdev )
{
	int ret;

	/* set J25 as a GPIO pin */
	omap_cfg_reg(J25_3430_GPIO170);

	/* find the IRQ */
	atmega_io_irqno = platform_get_irq( pdev, 0 );
	if ( atmega_io_irqno <= 0 ) {
		dev_err( &pdev->dev, "no irq for atmega status\n" );
		return -ENOENT;
	}

	DEBUGMSG("atmega_io: status irq %d on gpio %d\n", atmega_io_irqno, irq_to_gpio(atmega_io_irqno) );

	if (omap_request_gpio( irq_to_gpio(atmega_io_irqno) ) != 0) {
		printk(KERN_ERR "atmega: Could not reserve GPIO!\n");
		return -EINVAL;
	};

	omap_set_gpio_direction( irq_to_gpio(atmega_io_irqno), GPIO_DIR_INPUT);
	omap_set_gpio_dataout( irq_to_gpio(atmega_io_irqno), 0);

	if ( request_irq( atmega_io_irqno, atmega_io_statusirq, IRQF_SHARED|IRQF_TRIGGER_RISING, "atmega-io status", &pdev->dev.driver ) ) {
		DEBUGMSG( "IRQ%d already in use\n", gpio_to_irq(atmega_io_irqno) );
		return -EBUSY;
	}

	/* register our misc device */
	if ((ret = misc_register(&atmega_io_dev)) != 0) {
		printk(KERN_ERR "wdt: cannot register miscdev on minor=%d (err=%d)\n",
			MISC_DYNAMIC_MINOR, ret);
		return ret;
	}

// 	if ( atmega_version >=0x060a ) {
		// or (hardware_rev = 3 and msp versoin >=9)
		if (!atmega_key_probe())
			printk(KERN_ERR "atmega input device probe error!\n");
// 	}
		
	/* register ATMEGA-IO and exit */
	return ret;
}

static int atmega_io_remove( struct platform_device *pdev )
{
	free_irq( atmega_io_irqno, NULL );
	omap_free_gpio( irq_to_gpio(atmega_io_irqno) );
	misc_deregister( &atmega_io_dev );
	input_unregister_device(atmega_input_dev);
	return 0;
}

#ifdef CONFIG_PM
static int atmega_io_suspend( struct platform_device *pdev, pm_message_t msg)
{
	disable_irq( atmega_io_irqno );
	return 0;
}

static int atmega_io_resume( struct platform_device *pdev )
{
	enable_irq( atmega_io_irqno );
	// refresh status wake up event
	atmega_io_getstatus_on_irq(NULL);
	return 0;
}
#endif

static struct platform_driver atmega_io_drv = {
	.probe		= atmega_io_probe,
	.remove		= atmega_io_remove,
#ifdef CONFIG_PM
	.suspend	= atmega_io_suspend,
	.resume		= atmega_io_resume,
#endif
	.driver	= {
		.name = "atmega-io"
	},
};

static struct resource atmega_io_resource[] = {
	[0] = {
		.name	= "status",
		.start	= OMAP_GPIO_IRQ(ATMG_GPIO_NUM),
		.end	= OMAP_GPIO_IRQ(ATMG_GPIO_NUM),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device atmega_io_device = {
	.name = "atmega-io",
	.id = 0,
	.num_resources	= ARRAY_SIZE(atmega_io_resource),
	.resource	= atmega_io_resource,
};

/* ++++++++++++++++++ Module Init +++++++++++++++++++++++*/

static int __init atmega_io_init( void )
{
	int ret;

	printk( "ATMEGA IO Driver v%s for GEN6\n", ATMEGA_IO_VERSION );

	ret = platform_device_register( &atmega_io_device );
	if ( ret ) {
		goto register_out;
	}

	ret = platform_driver_register( &atmega_io_drv );
	if ( ret ) {
		platform_device_unregister( &atmega_io_device );
		goto register_out;
	}

	atmega_io_workqueue = create_workqueue("atmg");
	init_waitqueue_head(&atmega_io_wait_queue);
	
	if (device_create_file( &atmega_io_device.dev, &dev_attr_charge_rate) < 0)
		printk(KERN_DEBUG "unable to create charge_rate attribute\n");

	DEBUGMSG( "ATMEGA IO Driver register success\n" );

register_out:
	return ret;
}

static void __exit atmega_io_exit( void )
{
	device_remove_file( &atmega_io_device.dev, &dev_attr_charge_rate );
	destroy_workqueue( atmega_io_workqueue );
	platform_driver_unregister( &atmega_io_drv );
	platform_device_unregister( &atmega_io_device );
} 

module_init( atmega_io_init );
module_exit( atmega_io_exit );

EXPORT_SYMBOL(atmega_io_set_charge_mode);
EXPORT_SYMBOL(atmega_io_getUsbType);
EXPORT_SYMBOL(atmega_io_battery_dock_check_dcin);

/* Module information */
MODULE_DESCRIPTION("ATMEGA IO Driver for GEN6");
MODULE_LICENSE("GPL");
