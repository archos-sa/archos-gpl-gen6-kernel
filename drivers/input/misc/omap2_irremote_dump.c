/*
 *  omap2_irremote_uart2.c 
 *
 * Copyright 2007 Archos
 * Author: Jinqin DONG
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>

#include <asm/arch/mux.h>

#include <asm/system.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/leds.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <asm/arch/dmtimer.h>


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/stddef.h>
#include <linux/timer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/time.h>
#include <linux/kthread.h>

#define GPTIMER9_BASE					0x49040000

// #define OMAP_TIMER_TCLR_REG_GPO_CFG			(1<<14)
// #define OMAP_TIMER_TCLR_REG_CAPT_MODE			(1<<13)
// #define OMAP_TIMER_TCLR_REG_PULSE_TOOGLE		(1<<12)
// #define OMAP_TIMER_TCLR_REG_TRIGGER_OUTPUT_MODE 	(1<<10)
// #define OMAP_TIMER_TCLR_REG_TRANSITION_CAPT_MODE	(1<<8)
// #define OMAP_TIMER_TCLR_REG_COMPARE_ENABLE		(1<<6)
// #define OMAP_TIMER_TCLR_REG_START_STOP_TIMER		(1<<0)


//Timer registers address offset
#define OMAP_TIMER_TIDR_REG 				0x00
#define OMAP_TIMER_TIOCP_CFG_REG			0x10
#define OMAP_TIMER_TISTAT_REG				0x14
#define OMAP_TIMER_INT_STAT_REG				0x18
#define OMAP_TIMER_TIER_REG 				0x1c
#define OMAP_TIMER_TWER_REG 				0x20
#define OMAP_TIMER_TCLR_REG				0x24
#define OMAP_TIMER_TCRR_REG				0x28
#define OMAP_TIMER_TLDR_REG 				0x2c
#define OMAP_TIMER_TTGR_REG				0x30
#define OMAP_TIMER_WRITE_PEND_REG			0x34
#define OMAP_TIMER_TMAR_REG				0x38

#define OMAP_TIMER_TCAR1_REG 				0x3c
#define OMAP_TIMER_TSICR_REG 				0x40
#define OMAP_TIMER_TCAR2_REG 				0x44
/*
#define OMAP_TIMER_TPIR_REG 				0x48
#define OMAP_TIMER_TNIR_REG				0x4c
#define OMAP_TIMER_TCVR_REG				0x50
#define OMAP_TIMER_TOCR_REG 				0x54
#define OMAP_TIMER_TOWR_REG				0x58*/

static char register_name [15][22] =
{
	"TIMER_TIDR_REG",
	"TIMER_TIOCP_CFG_REG",
	"TIMER_TISTAT_REG",
	"TIMER_INT_STAT_REG",
	"TIMER_TIER_REG",
	"TIMER_TWER_REG",
	"TIMER_TCLR_REG",
	"TIMER_TCRR_REG",
	"TIMER_TLDR_REG",
	"TIMER_TTGR_REG",
	"TIMER_WRITE_PEND_REG",
	"TIMER_TMAR_REG",
	"TIMER_TCAR1_REG",
	"TIMER_TSICR_REG",
	"TIMER_TCAR2_REG",
};

int mode = 0;
int n = 0;
unsigned int wvalue =0;

static unsigned int _read(unsigned int reg_offset)
{
	return readl( io_p2v(GPTIMER9_BASE) + reg_offset);

}

//Write Timer registers
//In 16-bit access mode, the 16 LSBs must be written before writing to the 16 MSBs.

static void _write(unsigned int reg_offset,unsigned int value)
{
	writel(value, (io_p2v(GPTIMER9_BASE) + reg_offset) );
	while (readl( io_p2v(GPTIMER9_BASE) + OMAP_TIMER_WRITE_PEND_REG) )
		;
}

static void _dump(void)
{
	int i;
	for (i = 0; i < 15; i++) {
		printk("%s : %x\n", register_name[i], _read( (i? (0x10+4*(i-1)):  0x00 ) ));
	}
}

static int __init gptimer9_init_module(void)
{
	if ( mode == 0 && n == 0) {
		_dump();
	} else if (mode == 1) {
		printk("%s : %x\n", register_name[n-1], _read( (n? (0x10+4*(n-2)):  0x00 ) ));
	} else if (mode == 2) {
		_write((n? (0x10+4*(n-2)):  0x00 ), wvalue);
		printk("write %s : %x\n", register_name[n-1],wvalue);
	}
	return 0;
}

static void __exit gptimer9_cleanup_module(void)
{
	printk("quit gptimer dump moudle.\n");
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("jinqin dong");

module_init( gptimer9_init_module );
module_exit( gptimer9_cleanup_module );


module_param(mode, int, 0);
MODULE_PARM_DESC(mode, "mode=0 dump; 1 read; 2 write");

module_param(n, int, 0);
MODULE_PARM_DESC(mode, "n=0 dump all; 1-15 single register dump");

module_param(wvalue, int, 0);
MODULE_PARM_DESC(wvalue, "write register value");

/* Module information */
MODULE_AUTHOR("jinqin DONG, Archos S.A.");
MODULE_DESCRIPTION("OMAP3430 GPTimer9 DUMP");

