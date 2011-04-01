/*
 * linux/drivers/i2c/busses/i2c-omap_hs.c
 *
 * Unified algorithm/adapter I2C driver for OMAP243x I2C controller.
 *
 * Author: Andy Lowe (source@mvista.com)
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 * Copyright (C) 2005-2006 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * History:
 * -------
 * Aug 2005 - Copied from drivers/i2c/i2c-omap24xx.c and ported to
 *            243X and added support for HSI2C - Texas Instruments
 * Nov 2006 - Added OMAP 3430 support, some cleanups, and renamed
 *            to i2c-omap_hs.c - Texas Instruments
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#ifdef CONFIG_DPM
#include <asm/arch/dpm.h>	/* to specify constraints */
#endif
#include <asm/io.h>
#include <asm/arch/clock.h>

/* ----- configuration macros ----------------------------------------- */
#define OMAP_HACK

/* The reset timeout is the time allowed for a reset of the I2C controller
 * to complete.
 */
#define I2C_RESET_TIMEOUT_MS 100

/* The bus busy timeout is the maximum time we will wait for the bus to
 * become idle before initiating a transfer.
 */
#define I2C_BUS_BUSY_TIMEOUT_MS 200

/* This driver doesn't support I2C slave mode, so we set our own address
 * to zero to make sure it won't conflict with any slave devices on the
 * bus.
 */
#define I2C_OWN_ADDRESS 0

/* MCODE For HS transfers */
#define I2C_MCODE  0

/* define registers */
#define OMAP_I2C_REV_REG		0x00
#define OMAP_I2C_IE_REG			0x04
#define OMAP_I2C_STAT_REG		0x08
#define OMAP_I2C_IV_REG			0x0c
#define OMAP_I2C_SYSS_REG		0x10
#define OMAP_I2C_BUF_REG		0x14
#define OMAP_I2C_CNT_REG		0x18
#define OMAP_I2C_DATA_REG		0x1c
#define OMAP_I2C_SYSC_REG		0x20
#define OMAP_I2C_CON_REG		0x24
#define OMAP_I2C_OA_REG			0x28
#define OMAP_I2C_SA_REG			0x2c
#define OMAP_I2C_PSC_REG		0x30
#define OMAP_I2C_SCLL_REG		0x34
#define OMAP_I2C_SCLH_REG		0x38
#define OMAP_I2C_SYSTEST_REG		0x3c
#define OMAP_I2C_BUFSTAT_REG		0x40

/* I2C Interrupt Enable Register (OMAP_I2C_IE): */
#define OMAP_I2C_IE_XDR		(1 << 14)	/* TX Buffer draining enable */
#define OMAP_I2C_IE_RDR		(1 << 13)	/* RX Buffer draining enable */
#define OMAP_I2C_IE_XRDY	(1 << 4)	/* TX data ready int enable */
#define OMAP_I2C_IE_RRDY	(1 << 3)	/* RX data ready int enable */
#define OMAP_I2C_IE_ARDY	(1 << 2)	/* Access ready int enable */
#define OMAP_I2C_IE_NACK	(1 << 1)	/* No ack interrupt enable */
#define OMAP_I2C_IE_AL		(1 << 0)	/* Arbitration lost int ena */

/* I2C Status Register (OMAP_I2C_STAT): */
#define OMAP_I2C_STAT_XDR	(1 << 14)	/* TX Buffer draining */
#define OMAP_I2C_STAT_RDR	(1 << 13)	/* RX Buffer draining */
#define OMAP_I2C_STAT_BB	(1 << 12)	/* Bus busy */
#define OMAP_I2C_STAT_ROVR	(1 << 11)	/* Receive overrun */
#define OMAP_I2C_STAT_XUDF	(1 << 10)	/* Transmit underflow */
#define OMAP_I2C_STAT_AAS	(1 << 9)	/* Address as slave */
#define OMAP_I2C_STAT_AD0	(1 << 8)	/* Address zero */
#define OMAP_I2C_STAT_XRDY	(1 << 4)	/* Transmit data ready */
#define OMAP_I2C_STAT_RRDY	(1 << 3)	/* Receive data ready */
#define OMAP_I2C_STAT_ARDY	(1 << 2)	/* Register access ready */
#define OMAP_I2C_STAT_NACK	(1 << 1)	/* No ack interrupt enable */
#define OMAP_I2C_STAT_AL	(1 << 0)	/* Arbitration lost int ena */

/* I2C Buffer Configuration Register (OMAP_I2C_BUF): */
#define OMAP_I2C_BUF_RDMA_EN	(1 << 15)	/* RX DMA channel enable */
#define OMAP_I2C_BUF_RXFIF_CLR	(1 << 14)	/* RX FIFO Clear */
#define OMAP_I2C_BUF_XDMA_EN	(1 << 7)	/* TX DMA channel enable */
#define OMAP_I2C_BUF_TXFIF_CLR	(1 << 6)	/* TX FIFO Clear */

/* I2C Configuration Register (OMAP_I2C_CON): */
#define OMAP_I2C_CON_EN		(1 << 15)	/* I2C module enable */
#define OMAP_I2C_CON_BE		(1 << 14)	/* Big endian mode */
#define OMAP_I2C_CON_OPMODE_HS	(1 << 12)	/* High Speed support */
#define OMAP_I2C_CON_STB	(1 << 11)	/* Start byte mode (master) */
#define OMAP_I2C_CON_MST	(1 << 10)	/* Master/slave mode */
#define OMAP_I2C_CON_TRX	(1 << 9)	/* TX/RX mode (master only) */
#define OMAP_I2C_CON_XA		(1 << 8)	/* Expand address */
#define OMAP_I2C_CON_RM		(1 << 2)	/* Repeat mode (master only) */
#define OMAP_I2C_CON_STP	(1 << 1)	/* Stop cond (master only) */
#define OMAP_I2C_CON_STT	(1 << 0)	/* Start condition (master) */

/* I2C System Status register (OMAP_I2C_SYSS): */
#define OMAP_I2C_SYSS_RDONE	(1 << 0)	/* Reset Done */

/* I2C System Configuration Register (OMAP_I2C_SYSC): */
#define OMAP_I2C_SYSC_SRST	(1 << 1)	/* Soft Reset */
#define OMAP_I2C_SYSC_SMART_IDLE (2 << 3)
#define OMAP_I2C_SYSC_CLK_ACT  	(2 << 8)
#define OMAP_I2C_SYSC_AUTO_IDLE (1 <<  0)

#define OMAP_I2C_WAKE_UP_ENABLE	(1 << 2)
#define OMAP_I2C_WAKUP_EVENT	0x00006C1F

/* I2C SCL time value when Master */
#define OMAP_I2C_SCLL_HSSCLL	8
#define OMAP_I2C_SCLH_HSSCLH	8

/* I2C System Test Register (OMAP_I2C_SYSTEST): */
#ifdef DEBUG
#define OMAP_I2C_SYSTEST_ST_EN		(1 << 15)	/* System test enable */
#define OMAP_I2C_SYSTEST_FREE		(1 << 14)	/* Free running mode */
#define OMAP_I2C_SYSTEST_TMODE_MASK	(3 << 12)	/* Test mode select */
#define OMAP_I2C_SYSTEST_TMODE_SHIFT	(12)		/* Test mode select */
#define OMAP_I2C_SYSTEST_SCL_I		(1 << 3)	/* SCL line sense in */
#define OMAP_I2C_SYSTEST_SCL_O		(1 << 2)	/* SCL line drive out */
#define OMAP_I2C_SYSTEST_SDA_I		(1 << 1)	/* SDA line sense in */
#define OMAP_I2C_SYSTEST_SDA_O		(1 << 0)	/* SDA line drive out */
#endif

#define I2C_BUFSTAT_FIFO_DEPTH         (14)
#define I2C_BUFSTAT_FIFO_DEPTH_M       (0x03)
#define I2C_BUFSTAT_RXLEVEL            (8)
#define I2C_BUFSTAT_RXLEVEL_M          (0x3F)
#define I2C_BUFSTAT_TXLEVEL            (0)
#define I2C_BUFSTAT_TXLEVEL_M          (0x3F)

#define I2C_OA_MCODE         (13)
#define I2C_OA_MCODE_M       (0x07)
#define I2C_OA_OA0           (0)
#define I2C_OA_OA0_M         (0x3FF)

#define SYSTEM_CLOCK_12       12000
#define SYSTEM_CLOCK_96       96000

#define OMAP_I2C_FAST_MODE         400

/* timeout waiting for the controller to respond */
#define OMAP_I2C_TIMEOUT (msecs_to_jiffies(1000))
#define I2C_POWER

#ifdef I2C_POWER
#define OMAP_I2C_SYS_CONFIG_LVL1 1
#define OMAP_I2C_SYS_CONFIG_LVL2 2
#endif

/* TODO: to see if 3430 has/needs it */
#define CONTROL_PBIAS_LITE_1  IO_ADDRESS(OMAP24XX_CTRL_BASE+0x4A0)
#define PULL_HS_BUS0          (0x1<<3)
#define PULL_HS_BUS1          (0x1<<4)
#ifdef CONFIG_I2C_OMAP243X_SECONDARY_PULL0
#    ifdef CONFIG_I2C_OMAP243X_SECONDARY_PULL1
#        define I2C_SECONDARY_PULL    (PULL_HS_BUS0|PULL_HS_BUS1)
#    else
#        define I2C_SECONDARY_PULL    (PULL_HS_BUS0)
#    endif
#elif defined (CONFIG_I2C_OMAP243X_SECONDARY_PULL1)
#    define I2C_SECONDARY_PULL    (PULL_HS_BUS1)
#endif

/* ----- debug defines ----------------------------------------------- */
/* Debug - four macros:
 * FN_IN, FN_OUT(value),D1,D2,D3 enabled based on log level
 */

/* Log level standard used here:
 * Log level 3 all messages
 * Log level 2 all entry-exit points
 * Log level 1 major messages
 * Log level 0 no messages
 */
#define I2C_LOG_LEVEL 0

/* detail - 0 - no detail
 *          1 - function name
 *          2 - function name, line number
 * prefix is added to every log message
 */
#define I2C_DETAIL    0

/* kernel log level*/
/* #define I2C_K_LOG_LEVEL KERN_DEBUG */
#define I2C_K_LOG_LEVEL

#if ( I2C_DETAIL > 0 )
#define DL1 "%s "
#define DR1 ,__FUNCTION__
#else
#define DL1
#define DR1
#endif
#if ( I2C_DETAIL > 1 )
#define DL2 "[%d] "
#define DR2 ,__LINE__
#else
#define DL2
#define DR2
#endif

/* Wanted to reduce printks... at the same time ease development too..
 * cant do with the format,args version.. does not work with null args :(
 */
#define D(format,...)\
	printk(DL1 DL2 format "\n" DR1 DR2, ## __VA_ARGS__)

#if (I2C_LOG_LEVEL >= 1)
#define D1(ARGS...) D(ARGS)
#else
#define D1(ARGS...)
#endif
#if (I2C_LOG_LEVEL >= 2)
#define D2(ARGS...) D(ARGS)
#else
#define D2(ARGS...)
#endif
#if (I2C_LOG_LEVEL >= 3)
#define D3(ARGS...) D(ARGS)
#else
#define D3(ARGS...)
#endif

#if (I2C_LOG_LEVEL >= 2)
#define FN_IN D("%s Entry",__FUNCTION__);
#define FN_OUT(ARG) D("%s[%d]:Exit(%d)",__FUNCTION__,__LINE__,ARG);
#else
#define FN_IN
#define FN_OUT(ARG)
#endif

/* ----- module data structures ---------------------------------------	*/
struct omap_i2c_dev {

	struct device *dev;
	unsigned long base;
	int irq;
	struct clk *iclk;	/* Interface clock */
	struct clk *fclk;
	struct completion	cmd_complete;
	u32			speed;		/* Speed of bus in Khz */
	u16			cmd_err;
	u8			*buf;
	size_t			buf_len;
	struct i2c_adapter adapter;
	u8 fifo_size;	/* The fifo size of the tx device */
	unsigned short oa;	/* own address */
	unsigned char mcode;	/* master code */
	int a_num;
	unsigned		rev1:1;
	unsigned		b_hw:1;
};

#ifdef CONFIG_PM
/* if suspended, simply return an error */
#define omap_i2c_suspend_lockout(s) \
	if ((s)->suspended) {\
		return -EBUSY;\
	}
#ifdef CONFIG_DPM
/* No scaling when we dont do DMA yet */
#undef I2C_DPM_SCALE
#ifdef I2C_DPM_SCALE
static int omap_i2c_scale(struct notifier_block *op, unsigned long level,
			      void *newop);
static struct notifier_block omapi2c_pre_scale = {
	.notifier_call = omap_i2c_scale,
};
static struct notifier_block omapi2c_post_scale = {
	.notifier_call = omap_i2c_scale,
};
#endif
#endif				/* CONFIG_DPM */

#else
#define omap_i2c_suspend_lockout(s) do {} while(0)
#define omap_i2c_suspend_lockout_queue(s) do {} while(0)
#endif				/* CONFIG_PM */

/* ----- Utility functions --------------------------------------------	*/

static inline void omap_i2c_write_reg(struct omap_i2c_dev *i2c_dev,
				      int reg, u16 val)
{
	__raw_writew(val, i2c_dev->base + reg);
}

static inline u16 omap_i2c_read_reg(struct omap_i2c_dev *i2c_dev, int reg)
{
	return __raw_readw(i2c_dev->base + reg);
}

#ifdef I2C_POWER
/* Sysconfig settings, Enable smart idle and auto idle */
static void i2c_power_settings(struct omap_i2c_dev *dev, int level)
{
	/* set I2C smart idle and auto idle */
	if(level == OMAP_I2C_SYS_CONFIG_LVL1)
		omap_i2c_write_reg(dev, OMAP_I2C_SYSC_REG, 
			OMAP_I2C_SYSC_AUTO_IDLE | OMAP_I2C_SYSC_SMART_IDLE |
			OMAP_I2C_SYSC_CLK_ACT |	OMAP_I2C_WAKE_UP_ENABLE);
	/* clk activity both iclk and fclk off*/
	if(level == OMAP_I2C_SYS_CONFIG_LVL2)
		omap_i2c_write_reg(dev, OMAP_I2C_SYSC_REG,
			OMAP_I2C_SYSC_AUTO_IDLE | OMAP_I2C_SYSC_SMART_IDLE |
						OMAP_I2C_WAKE_UP_ENABLE);
}
#endif

static int omap_i2c_get_clocks(struct omap_i2c_dev *dev)
{

	dev->iclk = clk_get(dev->dev, "i2c_ick");
	if (IS_ERR(dev->iclk)) {
		dev->iclk = NULL;
		return -ENODEV;
	}

	dev->fclk = clk_get(dev->dev, "i2c_fck");
	if (IS_ERR(dev->fclk)) {
		if (dev->iclk != NULL) {
			clk_put(dev->iclk);
			dev->iclk = NULL;
		}
		dev->fclk = NULL;
		return -ENODEV;
	}

	return 0;
}

static void omap_i2c_put_clocks(struct omap_i2c_dev *dev)
{
	clk_put(dev->fclk);
	dev->fclk = NULL;
	clk_put(dev->iclk);
	dev->iclk = NULL;
}

static int omap_i2c_enable_clocks(struct omap_i2c_dev *dev)
{
	if (clk_enable(dev->iclk)) {
		return -ENODEV;
	}
	if (clk_enable(dev->fclk)) {
		clk_disable(dev->iclk);
		return -ENODEV;
	}
#ifdef I2C_POWER
	i2c_power_settings(dev,OMAP_I2C_SYS_CONFIG_LVL1);
#endif
	return 0;
}

static void omap_i2c_disable_clocks(struct omap_i2c_dev *dev)
{
#ifdef I2C_POWER
	i2c_power_settings(dev,OMAP_I2C_SYS_CONFIG_LVL2);
#endif
	if (dev->iclk != NULL)
	clk_disable(dev->iclk);
	clk_disable(dev->fclk);
}

/**
 * @brief omap_i2c_init
 * Initialize the I2C controller.
 *
 * @param dev
 *
 * @return intialization status  Returns zero if successful,
 * non-zero otherwise.
 */
static int omap_i2c_init(struct omap_i2c_dev *dev)
{

	u16 psc = 0, scll = 0, sclh = 0;
	u16 fsscll = 0, fssclh = 0, hsscll = 0, hssclh = 0;
	unsigned long fclk_rate = 12000000;
	unsigned long timeout;
	unsigned long internal_clk = 0;

	FN_IN;

	timeout = jiffies + ((I2C_RESET_TIMEOUT_MS) * HZ) / 1000;
	/* reset the I2C controller */
	omap_i2c_write_reg(dev, OMAP_I2C_SYSC_REG, OMAP_I2C_SYSC_SRST);
	/* enable the block to allow reset complete */
	omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, OMAP_I2C_CON_EN);

	/*
	 * Wait for reset to happen -bit will get set by the h/w once
	 * reset is completed
	 */
	while (0 == ((omap_i2c_read_reg(dev, OMAP_I2C_SYSS_REG)
		 & OMAP_I2C_SYSS_RDONE)) && time_before(jiffies, timeout)) {
		if (!in_interrupt()) {
			if (!signal_pending(current))
				set_current_state(TASK_INTERRUPTIBLE);
			else
				set_current_state(TASK_UNINTERRUPTIBLE);
			schedule_timeout(1);
		} else
			udelay(100);
	}
	/* if the i2c controller does not report reset completed... */
	if (0 == ((omap_i2c_read_reg(dev, OMAP_I2C_SYSS_REG)
					& OMAP_I2C_SYSS_RDONE))) {
		printk(KERN_ERR
		   "I2C[%d]:Timeout waiting for I2C controller to reset%d %d\n",
		     dev->a_num, (u32) I2C_RESET_TIMEOUT_MS, (u32) timeout);
		FN_OUT(ETIMEDOUT);
		return -ETIMEDOUT;
	}
	/* initialize pre-computed prescalar values */
#ifdef I2C_POWER
	i2c_power_settings(dev,OMAP_I2C_SYS_CONFIG_LVL1);
#endif
	/* Enabling wakeup events */
	omap_i2c_write_reg(dev, OMAP_I2C_IV_REG, OMAP_I2C_WAKUP_EVENT);
	/* disable the I2C controller */
	omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, 0);

	if (cpu_class_is_omap1()) {
		struct clk *armxor_ck;

		armxor_ck = clk_get(NULL, "armxor_ck");
		if (IS_ERR(armxor_ck))
			dev_warn(dev->dev, "Could not get armxor_ck\n");
		else {
			fclk_rate = clk_get_rate(armxor_ck);
			clk_put(armxor_ck);
		}
		/* TRM for 5912 says the I2C clock must be prescaled to be
		 * between 7 - 12 MHz. The XOR input clock is typically
		 * 12, 13 or 19.2 MHz. So we should have code that produces:
		 *
		 * XOR MHz	Divider		Prescaler
		 * 12		1		0
		 * 13		2		1
		 * 19.2		2		1
		 */
		if (fclk_rate > 12000000)
			psc = fclk_rate / 12000000;
	}

	if (cpu_is_omap3430() || cpu_is_omap2430()) {
		internal_clk = 19200;
		fclk_rate = clk_get_rate(dev->fclk);
		fclk_rate = fclk_rate / 1000;
		if(cpu_is_omap2430())
		fclk_rate = SYSTEM_CLOCK_96;

		/* Compute prescaler divisor */
		psc = fclk_rate / internal_clk;
		psc = psc - 1;

		/* If configured for High Speed */
		if (dev->speed > 400) {
			/* For first phase of HS mode */
			fsscll = internal_clk / (400 * 2) - 6;
			fssclh = internal_clk / (400 * 2) - 6;

			/* For second phase of HS mode */
			hsscll = fclk_rate / (dev->speed * 2) - 6;
			hssclh = fclk_rate / (dev->speed * 2) - 6;
		} else {
			/* To handle F/S modes */
			fsscll = internal_clk / (dev->speed * 2) - 6;
			fssclh = internal_clk / (dev->speed * 2) - 6;
		}
		scll = (hsscll << OMAP_I2C_SCLL_HSSCLL) | fsscll;
		sclh = (hssclh << OMAP_I2C_SCLH_HSSCLH) | fssclh;
	} else {
		/* Program desired operating rate */
		fclk_rate /= (psc + 1) * 1000;
		if (psc > 2)
			psc = 2;
		scll = fclk_rate / (dev->speed * 2) - 7 + psc;
		sclh = fclk_rate / (dev->speed * 2) - 7 + psc;
	}


	omap_i2c_write_reg(dev, OMAP_I2C_PSC_REG, psc);
	/* program tlow and thigh with equal levels */
	omap_i2c_write_reg(dev, OMAP_I2C_SCLL_REG, scll);	/* tlow */
	omap_i2c_write_reg(dev, OMAP_I2C_SCLH_REG, sclh);	/* thigh */
	/* set our own slave address */
	omap_i2c_write_reg(dev, OMAP_I2C_OA_REG,
			    (((dev->mcode & I2C_OA_MCODE_M) << I2C_OA_MCODE) |
			     ((dev->oa & I2C_OA_OA0_M) << I2C_OA_OA0)));

	if (dev->fifo_size)
		/* Note: setup required fifo size - 1 */
		omap_i2c_write_reg(dev, OMAP_I2C_BUF_REG,
					(dev->fifo_size - 1) << 8 | /* RTRSH */
					OMAP_I2C_BUF_RXFIF_CLR |
					(dev->fifo_size - 1) | /* XTRSH */
					OMAP_I2C_BUF_TXFIF_CLR);

	omap_i2c_write_reg(dev, OMAP_I2C_IE_REG,
			(OMAP_I2C_IE_XRDY | OMAP_I2C_IE_RRDY |
			OMAP_I2C_IE_ARDY | OMAP_I2C_IE_NACK |
			OMAP_I2C_IE_AL)  | ((dev->fifo_size) ?
				(OMAP_I2C_IE_RDR | OMAP_I2C_IE_XDR) : 0));

	/* enable the I2C controller */
	omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, OMAP_I2C_CON_EN);

	return 0;
}

/**
 * @brief i2c_wait_while_bb
 * The I2C bus protocol supports multiple masters.  This driver only supports
 * using the OMAP I2C bus controller as a master, but we want to try to
 * accommodate situations where there are other I2C masters sharing the bus.
 * In order to allow for other bus masters, we have to check if the bus is
 * busy before initiating a transfer.
 *
 * @param dev
 *
 * @return This functions returns 0 if the bus is idle, or
 * -ETIMEDOUT if a timeout occurs before the bus becomes idle.
 */
static int i2c_wait_while_bb( struct omap_i2c_dev *dev)
{
	unsigned long timeout = jiffies + (I2C_BUS_BUSY_TIMEOUT_MS * HZ) / 1000;

	FN_IN;

	while ((omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG) & OMAP_I2C_STAT_BB)
	       && time_before(jiffies, timeout)) {
		if (!in_interrupt()) {
			if (!signal_pending(current))
				set_current_state(TASK_INTERRUPTIBLE);
			else
				set_current_state(TASK_UNINTERRUPTIBLE);
			schedule_timeout(1);
		} else
			udelay(100);
	}
	if (omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG) & OMAP_I2C_STAT_BB) {
		D1("timeout waiting for bus to be idle [%d]", (int)timeout);
		FN_OUT(ETIMEDOUT);
		return -ETIMEDOUT;
	}

	FN_OUT(0);
	return 0;
}


/**
 * @brief omap_i2c_xfer_msg
 * Read or write a single message.  Returns the number of bytes
 * successfully read or written.  A start (or restart) condition is always
 * generated first, followed by msg->len data bytes.  A stop condition is
 * generated if the stop parameter is non-zero.  Otherwise, the bus is left
 * busy (clock line pulled low) so that the next transfer can begin with a
 * restart.
 *
 * @param dev
 * @param msg
 * @param stop
 *
 * @return 0 if successful, else the error condition
 */
static int
omap_i2c_xfer_msg(struct omap_i2c_dev *dev,
		      struct i2c_msg *msg, int stop)
{
	//struct omap_i2c_dev *dev = i2c_get_adapdata(adap);
#ifdef OMAP_HACK
	u8 zero_byte = 0;
#endif
	int r;
	u16 w;

	dev_dbg(dev->dev, "addr: 0x%04x, len: %d, flags: 0x%x, stop: %d\n",
		msg->addr, msg->len, msg->flags, stop);

#ifndef OMAP_HACK
	if (msg->len == 0)
		return -EINVAL;

	omap_i2c_write_reg(dev, OMAP_I2C_SA_REG, msg->addr);

	/* REVISIT: Could the STB bit of
			OMAP_I2C_CON_REG be used with probing? */
	dev->buf = msg->buf;
	dev->buf_len = msg->len;

#else

	omap_i2c_write_reg(dev, OMAP_I2C_SA_REG, msg->addr);
	/* REVISIT: Remove this hack when we can get I2C chips from board-*.c
	 *	    files
	 * Sigh, seems we can't do zero length transactions. Thus, we
	 * can't probe for devices w/o actually sending/receiving at least
	 * a single byte. So we'll set count to 1 for the zero length
	 * transaction case and hope we don't cause grief for some
	 * arbitrary device due to random byte write/read during
	 * probes.
	 */
	if (msg->len == 0) {
		dev->buf = &zero_byte;
		dev->buf_len = 1;
	} else {
		dev->buf = msg->buf;
		dev->buf_len = msg->len;
	}
#endif

	omap_i2c_write_reg(dev, OMAP_I2C_CNT_REG, dev->buf_len);

	/* Clear the FIFO Buffers */
	w = omap_i2c_read_reg(dev, OMAP_I2C_BUF_REG);
	w |= OMAP_I2C_BUF_RXFIF_CLR | OMAP_I2C_BUF_TXFIF_CLR;
	omap_i2c_write_reg(dev, OMAP_I2C_BUF_REG, w);

	init_completion(&dev->cmd_complete);
	dev->cmd_err = 0;

	w = OMAP_I2C_CON_EN | OMAP_I2C_CON_MST | OMAP_I2C_CON_STT;

	/* High speed configuration */
	if (dev->speed > 400)
		w |= OMAP_I2C_CON_OPMODE_HS;

	if (msg->flags & I2C_M_TEN)
		w |= OMAP_I2C_CON_XA;
	if (!(msg->flags & I2C_M_RD))
		w |= OMAP_I2C_CON_TRX;

	if (!dev->b_hw && stop)
		w |= OMAP_I2C_CON_STP;

	omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, w);

	if (dev->b_hw && stop) {
		/* H/w behavior: dont write stt and stp together.. */
		while (omap_i2c_read_reg(dev, OMAP_I2C_CON_REG) & OMAP_I2C_CON_STT) {
			/* Dont do anything - this will come in a couple of loops at max*/
		}
		w |= OMAP_I2C_CON_STP;
		w &= ~OMAP_I2C_CON_STT;
		omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, w);
	}
	r = wait_for_completion_timeout(&dev->cmd_complete,
					OMAP_I2C_TIMEOUT);
	dev->buf_len = 0;
	if (r < 0)
		return r;
	if (r == 0) {
		dev_err(dev->dev, "controller timed out\n");
		omap_i2c_init(dev);
		return -ETIMEDOUT;
	}

	if (likely(!dev->cmd_err))
	{
		return 0;
	}

	/* We have an error */
	if (dev->cmd_err & (OMAP_I2C_STAT_AL | OMAP_I2C_STAT_ROVR |
			    OMAP_I2C_STAT_XUDF)) {
		omap_i2c_init(dev);
		return -EIO;
	}

	if (dev->cmd_err & OMAP_I2C_STAT_NACK) {
		if (msg->flags & I2C_M_IGNORE_NAK)
			return 0;
		if (stop) {
			w = omap_i2c_read_reg(dev, OMAP_I2C_CON_REG);
			w |= OMAP_I2C_CON_STP;
			omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, w);
		}
		return -EREMOTEIO;
	}
	return -EIO;
}

/**
 * @brief omap_i2c_xfer
 * Read or write to an I2C device.
 *
 * @param adapter
 * @param msgs
 * @param num
 *
 * @return Returns num if all messages are transferred successfully, or a
 * negative error code otherwise.
 */
static int
omap_i2c_xfer(struct i2c_adapter *adapter, struct i2c_msg msgs[], int num)
{
	struct omap_i2c_dev
	*dev = (struct omap_i2c_dev *)
	    adapter->algo_data;
	int i;
	int r;

	FN_IN;
	if ((r = omap_i2c_enable_clocks(dev)) != 0) {
		printk(KERN_ERR " unable to enable clocks .. err_code %d\n",r);
		return r;
	}

#ifdef CONFIG_OMAP34XX_OFFMODE
	omap_i2c_init(dev);     /*Initializes the I2C device for each transfer*/
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */

	enable_irq(dev->irq);
	if (i2c_wait_while_bb(dev)) {
		/* The bus is still busy.  We're going to reset our I2C bus
		 * controller and then go ahead with our transfer regardless.
		 */
		D3("Resetting I2C controller.");
		omap_i2c_init(dev);
	}
	omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, 0);

	for (i = 0; i < num; i++) {
		r = omap_i2c_xfer_msg(dev, &msgs[i], (i == (num - 1)));
		if (r != 0) {
			D1("I2C transfer failed."
				"Resetting I2C controller.-[0x%x]\n", -r);
			disable_irq(dev->irq);
			omap_i2c_init(dev);
			omap_i2c_disable_clocks(dev);
			FN_OUT((r < 0) ? r : -EREMOTEIO);
			return (r < 0) ? r : -EREMOTEIO;
		}
	}
	disable_irq(dev->irq);
	omap_i2c_disable_clocks(dev);
	FN_OUT(num);
	return num;
}

/**
 * @brief omap_i2c_func
 *  return the functional capability of the adaptor
 * @param adapter
 *
 * @return Functional capability in the form of the adapter flags
 */
static u32 omap_i2c_func(struct i2c_adapter *adapter)
{
#ifdef CONFIG_MACH_ARCHOS_G6
return (((I2C_FUNC_SMBUS_EMUL) | I2C_FUNC_SMBUS_QUICK) //& ~(I2C_FUNC_SMBUS_QUICK))
                        | I2C_FUNC_10BIT_ADDR | I2C_FUNC_I2C);
#else
return (((I2C_FUNC_SMBUS_EMUL) & ~(I2C_FUNC_SMBUS_QUICK))
                        | I2C_FUNC_10BIT_ADDR | I2C_FUNC_I2C);
#endif
	FN_IN;
}

#ifdef CONFIG_PM
/**
 * @brief omap_i2c_suspend
 * Disable the i2c controller and shut down the clocks
 * Currently using the inbuilt LDM. Need to move to CONFIG_DPM style
 *
 * @param odev
 * @param state
 * @param level
 *
 * @return 0
 */
static int
omap_i2c_suspend(struct platform_device *pdev, pm_message_t state)
{
	FN_IN;
	FN_OUT(0);
	return 0;
}

/**
 * @brief omap_i2c_resume
 *
 * @param odev
 * @param level
 *
 * @return 0
 */
static int omap_i2c_resume(struct platform_device *pdev)
{
	FN_IN;
	FN_OUT(0);
	return 0;
}

#ifdef I2C_DPM_SCALE
/**
 * @brief omap_i2c_scale
 * nothing to do in scale() now--placeholder only -
 * need to look once DMA enabled
 *
 * @param op
 * @param level
 *
 * @return  0
 */
static int omap_i2c_scale(struct notifier_block *op, unsigned long level,
			      void *newop)
{
	FN_IN;
	switch (level) {
	case SCALE_PRECHANGE:
		break;
	case SCALE_POSTCHANGE:
		break;
	}
	FN_OUT(0);
	return 0;
}
static struct constraints i2c_omap_constraints = {
	.count = 2,
	.param = {
		  {DPM_MD_V, 1050, 1300},
		  {DPM_MD_PWRST_CORE, 1, 1},	/* 1 = PRCM_ON */
		  },
};
#endif

#endif				/* CONFIG_PM */


/* complete command */
static inline void
omap_i2c_complete_cmd(struct omap_i2c_dev *dev, u16 err)
{
	dev->cmd_err |= err;
	complete(&dev->cmd_complete);
}

static inline void
omap_i2c_ack_stat(struct omap_i2c_dev *dev, u16 stat)
{
	omap_i2c_write_reg(dev, OMAP_I2C_STAT_REG, stat);
}

static irqreturn_t
omap_i2c_rev1_isr(int this_irq, void *dev_id)
{
	struct omap_i2c_dev *dev = dev_id;
	u16 iv, w;

	iv = omap_i2c_read_reg(dev, OMAP_I2C_IV_REG);
	switch (iv) {
	case 0x00:	/* None */
		break;
	case 0x01:	/* Arbitration lost */
		dev_err(dev->dev, "Arbitration lost\n");
		omap_i2c_complete_cmd(dev, OMAP_I2C_STAT_AL);
		break;
	case 0x02:	/* No acknowledgement */
		omap_i2c_complete_cmd(dev, OMAP_I2C_STAT_NACK);
		omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, OMAP_I2C_CON_STP);
		break;
	case 0x03:	/* Register access ready */
		omap_i2c_complete_cmd(dev, 0);
		break;
	case 0x04:	/* Receive data ready */
		if (dev->buf_len) {
			w = omap_i2c_read_reg(dev, OMAP_I2C_DATA_REG);
			*dev->buf++ = w;
			dev->buf_len--;
			if (dev->buf_len) {
				*dev->buf++ = w >> 8;
				dev->buf_len--;
			}
		} else
			dev_err(dev->dev, "RRDY IRQ while no data requested\n");
		break;
	case 0x05:	/* Transmit data ready */
		if (dev->buf_len) {
			w = *dev->buf++;
			dev->buf_len--;
			if (dev->buf_len) {
				w |= *dev->buf++ << 8;
				dev->buf_len--;
			}
			omap_i2c_write_reg(dev, OMAP_I2C_DATA_REG, w);
		} else
			dev_err(dev->dev, "XRDY IRQ while no data to send\n");
		break;
	default:
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}



/**
 * @brief omap_i2c_handler
 *
 * @param this_irq
 * @param dev_id
 * @param regs
 */
static irqreturn_t omap_i2c_handler(int this_irq, void *dev_id)
{
		struct omap_i2c_dev *dev = dev_id;
	u16 bits;
	u16 stat, w;
	int count = 0;

	bits = omap_i2c_read_reg(dev, OMAP_I2C_IE_REG);
	while ((stat = (omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG))) & bits) {
		dev_dbg(dev->dev, "IRQ (ISR = 0x%04x)\n", stat);
		if (count++ == 100) {
			dev_warn(dev->dev, "Too much work in one IRQ\n");
			break;
		}

		omap_i2c_write_reg(dev, OMAP_I2C_STAT_REG, stat);

		if (stat & OMAP_I2C_STAT_ARDY) {
			omap_i2c_complete_cmd(dev, 0);
			continue;
		}
		if (stat & (OMAP_I2C_STAT_RRDY | OMAP_I2C_STAT_RDR)) {
			u8 num_bytes = 1;
			if (dev->fifo_size) {
				num_bytes = (stat & OMAP_I2C_STAT_RRDY) ? dev->fifo_size :
						omap_i2c_read_reg(dev, OMAP_I2C_BUFSTAT_REG);
			}
			while (num_bytes) {
				num_bytes--;
				w = omap_i2c_read_reg(dev, OMAP_I2C_DATA_REG);
				if (dev->buf_len) {
					*dev->buf++ = w;
					dev->buf_len--;
					/*
					 * Data reg in 2430 is 8 bit wide,
					 */
					if (!cpu_is_omap3430() && !cpu_is_omap2430() ) {
						if (dev->buf_len) {
							*dev->buf++ = w >> 8;
							dev->buf_len--;
						}
					}
				} else {
					if (stat & OMAP_I2C_STAT_RRDY)
						dev_err(dev->dev, "RRDY IRQ while no data"
								"requested\n");
					if (stat & OMAP_I2C_STAT_RDR)
						dev_err(dev->dev, "RDR IRQ while no data"
								"requested\n");
					break;
				}
			}
			omap_i2c_ack_stat(dev, stat & (OMAP_I2C_STAT_RRDY | OMAP_I2C_STAT_RDR));
			continue;
		}
		if (stat & (OMAP_I2C_STAT_XRDY | OMAP_I2C_STAT_XDR)) {

			u8 num_bytes = 1;
			if (dev->fifo_size) {
				num_bytes = (stat & OMAP_I2C_STAT_XRDY) ? dev->fifo_size :
						omap_i2c_read_reg(dev, OMAP_I2C_BUFSTAT_REG);
			}
			while (num_bytes) {
				num_bytes--;
				w = 0;
				if (dev->buf_len) {
					w = *dev->buf++;
					dev->buf_len--;
					/*
					 * Data reg in 2430 is 8 bit wide,
					 */
					if (!cpu_is_omap3430() && !cpu_is_omap2430()) {
						if (dev->buf_len) {
							w |= *dev->buf++ << 8;
							dev->buf_len--;
						}
					}
				} else {
					if (stat & OMAP_I2C_STAT_XRDY)
						dev_err(dev->dev, "XRDY IRQ while no"
								"data to send\n");
					if (stat & OMAP_I2C_STAT_XDR)
						dev_err(dev->dev, "XDR IRQ while no"
								"data to send\n");
					break;
				}
				omap_i2c_write_reg(dev, OMAP_I2C_DATA_REG, w);
			}
			omap_i2c_ack_stat(dev, stat & (OMAP_I2C_STAT_XRDY | OMAP_I2C_STAT_XDR));
			continue;
		}
		if (stat & OMAP_I2C_STAT_ROVR) {
			dev_err(dev->dev, "Receive overrun\n");
			dev->cmd_err |= OMAP_I2C_STAT_ROVR;
		}
		if (stat & OMAP_I2C_STAT_XUDF) {
			dev_err(dev->dev, "Transmit overflow\n");
			dev->cmd_err |= OMAP_I2C_STAT_XUDF;
		}
		if (stat & OMAP_I2C_STAT_NACK) {
			omap_i2c_complete_cmd(dev, OMAP_I2C_STAT_NACK);
			omap_i2c_write_reg(dev, OMAP_I2C_CON_REG,
					   OMAP_I2C_CON_STP);
		}
		if (stat & OMAP_I2C_STAT_AL) {
			dev_err(dev->dev, "Arbitration lost\n");
			omap_i2c_complete_cmd(dev, OMAP_I2C_STAT_AL);
		}
	}

	return count ? IRQ_HANDLED : IRQ_NONE;
}


/* -----exported algorithm data: -------------------------------------	*/

static struct i2c_algorithm omap_i2c_algo = {
	.master_xfer = omap_i2c_xfer,
	.functionality = omap_i2c_func,
};




/*
 * @brief omap_i2c_probe
 * I2C Module Startup
 *
 * @return 0 for success, else return error condition
 */
static int omap_i2c_probe(struct platform_device *pdev)
{
	struct omap_i2c_dev *dev;
	struct i2c_adapter *adap = NULL;
	struct resource *mem, *irq, *ioarea;
	int r=0;
	u32 *speed = NULL;


	FN_IN;

	/* NOTE: driver uses the static register mapping */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -ENODEV;
	}
	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return -ENODEV;
	}

	ioarea = request_mem_region(mem->start, (mem->end - mem->start) + 1,
				    pdev->name);
	if (!ioarea) {
		dev_err(&pdev->dev, "I2C region already claimed\n");
		return -EBUSY;
	}



	dev = kzalloc(sizeof(struct omap_i2c_dev), GFP_KERNEL);
	if (!dev) {
		r = -ENOMEM;
		goto err_release_region;
	}
	if (pdev->dev.platform_data != NULL)
		speed = (u32 *) pdev->dev.platform_data;
	else
		*speed = 400;	/* Default speed */




	dev->speed = *speed;
	dev->dev = &pdev->dev;
	dev->irq = irq->start;

	/* Dont need to re-map an already remapped address
	 * L4Core and wakeup are default mapped
	 */
	dev->base = (unsigned long)IO_ADDRESS(mem->start);
	platform_set_drvdata(pdev, dev);

	if ((r = omap_i2c_get_clocks(dev)) != 0)
		goto err_free_mem;

	if ((r = omap_i2c_enable_clocks(dev)) != 0)
		goto err_free_mem;
	if (cpu_is_omap15xx())

		dev->rev1 = omap_i2c_read_reg(dev, OMAP_I2C_REV_REG) < 0x20;

	if (cpu_is_omap3430() || cpu_is_omap2430()) {
	/* Grab the adapter fifo size */
		dev->fifo_size = 0x8 <<
		    (omap_i2c_read_reg(dev, OMAP_I2C_BUFSTAT_REG) >>
		     I2C_BUFSTAT_FIFO_DEPTH) & I2C_BUFSTAT_FIFO_DEPTH_M;

		/* Set up notification threshold as half the total available size
		 * This is to ensure that we can handle the status on int call back
		 * latencies
		 */
		dev->fifo_size= dev->fifo_size/2;
		dev->b_hw = 1; /* Enable hardware fixes */
	}
#ifdef I2C_DPM_SCALE
	/* To be enabled on implementing DMA */
	dpm_register_scale(&omapi2c_pre_scale, SCALE_PRECHANGE);
	dpm_register_scale(&omapi2c_post_scale, SCALE_POSTCHANGE);
#endif

#ifdef I2C_SECONDARY_PULL
	/* Enables the internal secondary pull-up inside the HS
	 * I2C pad when the I2C is in High-Speed mode and the
	 * bus line capacitance is exceeding 45 pF.
	 * Valid only for ES2.0 and above
	 */
	if (get_cpu_rev() >= 2) {
		u32 reg = readl(CONTROL_PBIAS_LITE_1);
		reg &= ~(PULL_HS_BUS0 | PULL_HS_BUS1);
		reg |= I2C_SECONDARY_PULL;
		writel(reg, CONTROL_PBIAS_LITE_1);
	}
#endif

			/* Grab the adapter fifo size */

	/* initialize the i2c_adapter struct */
	adap = &dev->adapter;
	adap->algo_data = dev;
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_HWMON;
	strncpy(adap->name, "OMAP I2C adapter", sizeof(adap->name));
	adap->algo = &omap_i2c_algo;
	adap->dev.parent = &pdev->dev;
	dev->dev->driver_data = dev;

	dev->oa = I2C_OWN_ADDRESS;
	dev->mcode = I2C_MCODE;	/* needed for HS transfer */


	/* We delete I2C_FUNC_SMBUS_QUICK from our list of capabilities
	 * because that requires the ability to generate messages with
	 * zero bytes of data.  The OMAP I2C controller is not able to do
	 * this.  The lack of this feature prevents probing for I2C devices.
	 */

	if (omap_i2c_init(dev) < 0) {
		printk(KERN_ERR
		       "%s: cannot reset I2C controller\n", adap->name);
		goto err_unuse_clocks;
	}

	if (request_irq
	    (dev->irq, dev->rev1 ? omap_i2c_rev1_isr :omap_i2c_handler, 0, adap->name, dev) < 0) {
		printk(KERN_ERR
		       "%s: cannot intall handler for irq"
		       " %d\n", adap->name, dev->irq);
		goto err_unuse_clocks;
	}
	if (i2c_add_adapter(adap) < 0) {
		printk(KERN_ERR
		       "%s: cannot register I2C adapter\n", adap->name);
		goto err_free_irq;
	}
	/* request_irq enables the interrupt , disable it
		we will enable it as per request basis */
	disable_irq(dev->irq);

	omap_i2c_disable_clocks(dev);
	FN_OUT(0);
	return 0;
      err_free_irq:
	free_irq(dev->irq, dev);
      err_unuse_clocks:
	omap_i2c_disable_clocks(dev);
	omap_i2c_put_clocks(dev);
      err_free_mem:
	platform_set_drvdata(pdev, NULL);
	kfree(dev);
      err_release_region:
	omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, 0);
	release_mem_region(mem->start, (mem->end - mem->start) + 1);
	return -ENODEV;
}

static int omap_i2c_remove(struct platform_device *pdev)
{
	struct omap_i2c_dev *dev = platform_get_drvdata(pdev);
	struct resource *mem;

	platform_set_drvdata(pdev, NULL);

	free_irq(dev->irq, dev);
	i2c_del_adapter(&dev->adapter);
	if(!omap_i2c_enable_clocks(dev)) {
	omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, 0);
	omap_i2c_disable_clocks(dev);
	omap_i2c_put_clocks(dev);
	}
	kfree(dev);
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, (mem->end - mem->start) + 1);
	return 0;
}

static struct platform_driver omap_i2c_driver = {
	.probe = omap_i2c_probe,
	.remove = omap_i2c_remove,
	.driver = {
		   .name = "i2c_omap",
		   .owner = THIS_MODULE,
		   },
#ifdef CONFIG_PM
	.suspend = omap_i2c_suspend,
	.resume = omap_i2c_resume,
#endif

};

/* I2C may be needed to bring up other drivers */
static int __init omap_i2c_init_driver(void)
{
	return platform_driver_register(&omap_i2c_driver);
}

subsys_initcall(omap_i2c_init_driver);

static void __exit omap_i2c_exit_driver(void)
{
	platform_driver_unregister(&omap_i2c_driver);
}

module_exit(omap_i2c_exit_driver);

MODULE_AUTHOR("MontaVista Software, Inc. (and others)");
MODULE_DESCRIPTION("TI OMAP I2C bus adapter");
MODULE_LICENSE("GPL");
