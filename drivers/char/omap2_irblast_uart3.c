/*
 *  omap2_irblast.c 
 *
 * Copyright 2007 Archos
 * Author: Paul EBEYAN
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
#include <linux/version.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <asm/uaccess.h>        /* get_user,copy_to_user */
#include <linux/miscdevice.h>
#include <asm/setup.h>
#include <asm/arch/clock.h>
#include <asm/arch/omap24xx-uart.h>
#include <linux/string.h>
#include <asm/arch/mux.h>
#include <asm/arch/gpio.h>
#include <asm/arch/io.h>

#include "irblast.h"

#define IRBLAST_NAME	"irblast"

#define CONSTANT_CARRIER 0x100	/* from vg.h in the irblaster library */
#define NUM_OF_REPEATS	3

#define IRBLAST_TIMEOUT (2000) // 2 seconds timeout
#define IRBLAST_CHECK_TIME (4) // check every 4 milliseconds

#define UART_NO UART3
#define CONFIG_NORMAL_REG	LCR_MODE1
#define CONFIG_MODEA_REG	LCR_MODE2
#define CONFIG_MODEB_REG	LCR_MODE3

//#define CONFIG_TEST_UART

#if defined CONFIG_TEST_UART
	#define UART_CONFIG		(UART_16X_MODE)
	#define UART_TX_FULL_MASK	(0x20)
#else
	#define UART_CONFIG		(UART_CIR_MODE)
	#define UART_TX_FULL_MASK 	(0x80)
#endif

#define CIR_TX	166	/* gpio 166 */
#define CIR_TX_PIN	V21_3430_GPIO166

typedef struct irblast_t {
	int numRepeats;
} irblast_t; 

unsigned long OnceCtr;
unsigned long RepeatCtr;
unsigned long NumRepeats;
unsigned long ConstantCarrier;
unsigned short *IrblastDataRepeat;
unsigned short IrblastData[256];
//extern irblast_data_t irblast_data __iramdata;

static irblast_t irblast;

static 	u32 uart_base = 0;

/** 
 * brief get_uart_clocks
 * used to enable/disable UART clocks.
 * param *uarti 
 * param *uartf 
 * param uart_no 
 * 
 * return 
 */
static int get_uart_clocks(struct clk **uarti, struct clk **uartf, u8 uart_no)
{
	char *iname = NULL, *fname = NULL;

	switch (uart_no) {
	case UART1:
		iname = "uart1_ick";
		fname = "uart1_fck";
		break;
	case UART2:
		iname = "uart2_ick";
		fname = "uart2_fck";
		break;

	case UART3:
		iname = "uart3_ick";
		fname = "uart3_fck";
		break;
	default:
		return -EPERM;
	}
	*uarti = clk_get(NULL, iname);
	if (*uarti < 0) {
		printk(KERN_ERR "Error : %s : No entry in the clock table \n",
		       iname);
		return -ENOENT;
	}
	*uartf = clk_get(NULL, fname);
	if (*uartf < 0) {
		printk(KERN_ERR "Error : %s : No entry in the clock table \n",
		       fname);
		return -ENOENT;
	}
	return 0;
}

static void inline uart_reg_out(int reg, u8 val)
{
	outb(val, uart_base + reg);
}

static u8 inline uart_reg_in(int reg)
{
	u8 b = readb(uart_base + reg);
	return b;
}

static void _set_config_normal(void) {

	u8 dat = uart_reg_in(REG_LCR);
	dat &= (~BIT_LCR_DIV_EN_M);
	uart_reg_out(REG_LCR,dat);

	return;
}

static void inline _set_config_a(void) {

	uart_reg_out(REG_LCR,CONFIG_MODEA_REG);

	return;
}

static void inline _set_config_b(void) {

	uart_reg_out(REG_LCR,CONFIG_MODEB_REG);

	return;


}

static void _set_frame_length(int len){
#if !defined CONFIG_TEST_UART
	/* should be for irda only !!! */

	uart_reg_out(REG_TXFLL,(len & BIT_TXFLL_TXFLL_M));
	uart_reg_out(REG_TXFLH,((len>>8) & BIT_TXFLH_TXFLH_M));

#endif
}

static int _cir_write(unsigned char *data, int length) {

	u8 lsr_data;
	int size=length;
	int count=0;
	unsigned long timeout=0;
	unsigned char *txaddr = data;

	_set_frame_length(length);

	_set_config_normal();

	lsr_data = uart_reg_in(REG_LSR);
	printk("config: %d\n",uart_reg_in(REG_LCR));

	if (!(lsr_data & UART_TX_FULL_MASK)) {
		printk(KERN_ERR "Transmit FIFO Full\n");
		return -EIO;
	}

	while (size > 0 ) {
		lsr_data = uart_reg_in(REG_LSR);;
		if (lsr_data & UART_TX_FULL_MASK) {
			timeout = 0;
			printk(".");
			uart_reg_out(REG_THR,txaddr[count]);
			count++;
			size--;
		} else
			timeout++;

		if (timeout > 10000) {
			printk("Error : Tx timeout\n");
			return -EIO;
		}
	}

	printk("\nTransmission Done: %d\n",count);

	return count;

}


static void dump_reg_in_mode(u8 mode) {
	
	u8 current_mode = uart_reg_in(REG_LCR);
	int i=0;

	uart_reg_out(REG_LCR,mode);
	for (i=0;i<0x64;i+=4) {
		printk("%02x: 0x%02x\n",i,uart_reg_in(i));
	}
	
	uart_reg_out(REG_LCR,current_mode);
	return;
}

static void dump_reg(void) {

	printk("Mode: Normal\n");
	dump_reg_in_mode(CONFIG_NORMAL_REG);
	printk("Mode: A\n");
	dump_reg_in_mode(CONFIG_MODEA_REG);
	printk("Mode: B\n");
	dump_reg_in_mode(CONFIG_MODEB_REG);
}

static int irblast_do(irblast_t *irblast, irblast_data_t *data)
{
	int i;	

	if (data->details & CONSTANT_CARRIER)
		ConstantCarrier = 1;
	else
		ConstantCarrier = 0;
		
	NumRepeats = irblast->numRepeats;
	
	OnceCtr   = data->OnceCtr * 2;
	RepeatCtr = data->RepeatCtr * 2;

	for( i = 0; i < 256; i++ ) {
		IrblastData[i] = data->Data[i]*2;
	}
	IrblastDataRepeat = IrblastData + OnceCtr;


	return 0;
}

static int irblast_ioctl(struct inode * inode, struct file *filp,
	     unsigned int cmd, unsigned long arg)
{
	irblast_t *irblast = (irblast_t*)filp->private_data;

	int ret = -EINVAL;
	unsigned int tmp;
		
	switch (cmd) {
		case IRBLAST_SET_REPEATS:
			if( get_user(tmp, (int *) arg) )
			{
				ret = -EFAULT;
				break;
			}			
			irblast->numRepeats = tmp;
			ret = 0;
			break;
		case IRBLAST_DUMP_REGS:
			dump_reg();
		break;
		case IRBLAST_WRITE_SINGLE:
			/* Set MUX to UART3 */
			omap_cfg_reg(V21_3430_UART3_TX_IRTX);
			_set_config_normal();
  			uart_reg_out(REG_THR,0x55);
 			printk("single uart write done\n");
		break;
		case IRBLAST_SET_GIO:
  			omap_cfg_reg(CIR_TX_PIN);
  			omap_set_gpio_direction(CIR_TX, GPIO_DIR_OUTPUT);
  			omap_set_gpio_dataout(CIR_TX, 1);
		break;
		case IRBLAST_CLEAR_GIO:
  			omap_cfg_reg(CIR_TX_PIN);
  			omap_set_gpio_direction(CIR_TX, GPIO_DIR_OUTPUT);
  			omap_set_gpio_dataout(CIR_TX, 0);
		break;

		default:
			break;
	}
	return ret;
}

static ssize_t irblast_write(struct file * filp, const char * buffer, size_t count, loff_t * l)
{
// 	irblast_t* irblast = (irblast_t*)filp->private_data;
	u8 data[256];
	int ret = 0;

// 	if( count != sizeof(irblast_data_t) ) {
// 		return -EFAULT;
// 	}

// 	if( NULL == (data = kmalloc( sizeof( irblast_data_t ), GFP_KERNEL )) ) {
// 		return -EFAULT;
// 	}

	if( 0 != copy_from_user( data, buffer, count ) ) {
		kfree( data );
		return -EFAULT;
	}

	printk("writing %d data to irb ...\n",count);
//  	if( data->OnceCtr == 0 && data->RepeatCtr == 0 ) {
// 		kfree( data );
// 		return -EFAULT;
// 	}

	/* set up speed */

// 	if( irblast_do(irblast, data) )	{
// 		ret = -EFAULT;
// printk( KERN_ERR "irblast_do timed out\n" );
// 	} else {
// 		ret = sizeof(irblast_data_t);
// 	}

	/* Set MUX to UART3 */
	omap_cfg_reg(V21_3430_UART3_TX_IRTX);
	_cir_write(data,count);


//	kfree( data );

	return ret;
}

static int irblast_open(struct inode * inode, struct file * filp)
{
	filp->private_data = &irblast;
	irblast.numRepeats = NUM_OF_REPEATS; /* default */

	return 0;
}

static int irblast_release(struct inode * inode, struct file * filp)
{
	return 0;
}


static struct file_operations irblast_fops = {
	write:          irblast_write,
	ioctl:		irblast_ioctl,
	open:		irblast_open,
	release:	irblast_release,
};

static struct miscdevice irblast_miscdev = {
	.minor	= IRBLAST_MINOR,
	.name	= IRBLAST_NAME,
	.fops	= &irblast_fops,
};

/* +++++++++++++ End File operations ++++++++++++++*/

static void _set_speed(int speed) {

	int divisor;

	_set_config_a();

	/* Disable UART before changing the clock speed - TRM - 18-52 */
	uart_reg_out(REG_MDR1,UART_DISABLE);
	
	/* set Carrier Frequency */
	divisor = BASE_CLK / (12 * speed);
	printk(KERN_INFO " UART3 CIR Mode: %d CPFS\n", divisor );
	uart_reg_out(REG_CFPS,(divisor & 0xFF));

	/* we want the shift reg speed to be the same as CF */
	divisor = BASE_CLK /( 16 * speed);
	printk(KERN_INFO " UART3 CIR Mode: %d DL\n", divisor );
	uart_reg_out(REG_DLL,(divisor & 0xFF));
	uart_reg_out(REG_DLH,(divisor >> 8));

	/* set CIR MODE and disable sleep */
	uart_reg_out(REG_MDR1, UART_CONFIG);

}

static int _init_cir_mode(void) {

	unsigned long timeout;
	struct clk *uarti = NULL, *uartf = NULL;

	/* Set MUX to UART3 */
	omap_cfg_reg(V21_3430_UART3_TX_IRTX);

	/* Start clock for requested UART */
	if (get_uart_clocks(&uarti, &uartf, UART_NO)) {
		printk(KERN_ERR "UART clock configuration error\n");
		return -ENOENT;
	}
	clk_enable(uarti);
	clk_enable(uartf);


	/* diasble and sleep */
	uart_reg_out(REG_MDR1, UART_DISABLE);	/* Reset mode */

	/* Clear DLH and DLL */
	/* Config Mode A */
	_set_config_a();

	uart_reg_out(REG_DLL, 0);
	uart_reg_out(REG_DLH, 0);

	/* do a soft reset */
	uart_reg_out(REG_SYSC,BIT_SYSC_SOFTRESET_M);
	timeout = jiffies + msecs_to_jiffies(10);
	while (!(uart_reg_in(REG_SYSS) & BIT_SYSS_RESETDONE_M)
		&& time_before(jiffies, timeout)) {
	udelay(10);
	}
	
	printk("uart reset done\n");

	/* set default speed */
	_set_speed(115200);

	return 0;
}

static int __init irblast_init_module(void)
{
	irblast_t* pt_irblast = &irblast;
	int ret;
// 	char str[7];

	/* check that uart3 is not used by console */
//         if ( !console_detect(str)){
// 		if (!strcmp(str, "ttyS2")) {
// 			printk("uart3 already in use !!!\n");
// 		}
// 	}	

	uart_base = UART_MODULE_BASE(UART_NO);

	if ( (ret = _init_cir_mode()) < 0 ) {
		printk(KERN_ERR "cir init error: %d\n", ret);
		return ret;
	}

	/* register our misc device */
	if ((ret = misc_register(&irblast_miscdev)) != 0) {
		printk(KERN_ERR "wdt: cannot register miscdev on minor=%d (err=%d)\n",
			MISC_DYNAMIC_MINOR, ret);
		return ret;
	}
	
	printk("archos irblaster driver registered. minor: %d\n", irblast_miscdev.minor);

	memset(pt_irblast, 0, sizeof(irblast_t));

	
	OnceCtr = 0;
	RepeatCtr = 0;
	NumRepeats = 1;
	ConstantCarrier = 0;
	
	return 0;
}

static void __exit irblast_cleanup_module(void)
{
	misc_deregister(&irblast_miscdev);
}

module_init( irblast_init_module );
module_exit( irblast_cleanup_module );

/* Module information */
MODULE_AUTHOR("Paul EBEYAN, Archos S.A.");
MODULE_DESCRIPTION("IR-Blaster Driver");


