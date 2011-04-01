/*
 * tvp5146 - Texas Instruments TVP5146 video decoder driver
 *
 */

#include <linux/i2c.h>
#include <linux/videodev.h>
#include <linux/delay.h>
#include <linux/video_decoder.h>
#include <media/v4l2-common.h>
#include <asm/io.h>
#include "tvp5146_reg.h"

#if defined (CONFIG_MACH_ARCHOS_G6)
#include <asm/arch/archosg6-videoin.h>
#endif

#ifdef CONFIG_ARCH_OMAP34XX
#include "omap/video_decoder_if.h"
#endif

#undef MODE_FAST_SWITCH

static void tvp5146_reset_ctrl_component(struct i2c_client *c);
static void tvp5146_reset_ctrl_rgb(struct i2c_client *c);

MODULE_DESCRIPTION("Texas Instruments TVP5146 video decoder driver");
MODULE_AUTHOR("ARCHOS SA");
MODULE_LICENSE("GPL");

#define TVP5150_GPCL_OUTPUT_MASK	0x40

/* FIXME: these need to be propagated to the application,
 * make a nice header file for them */
#define V4L2_CID_RED_GAIN		(V4L2_CID_PRIVATE_BASE+0)
#define V4L2_CID_BLUE_GAIN		(V4L2_CID_PRIVATE_BASE+1)
#define V4L2_CID_GREEN_GAIN		(V4L2_CID_PRIVATE_BASE+2)
#define V4L2_CID_PB_SATURATION		(V4L2_CID_PRIVATE_BASE+3)
#define V4L2_CID_PR_SATURATION		(V4L2_CID_PRIVATE_BASE+4)

/* standard i2c insmod options */
static unsigned short normal_i2c[] = { 0xb8 >> 1, I2C_CLIENT_END };

I2C_CLIENT_INSMOD;

static int debug = 0;
module_param(debug, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(debug, "Debug level (0-3)");

#define tvp5146_err(fmt, arg...) do { \
	printk(KERN_ERR "%s %d-%04x: " fmt, c->driver->driver.name, \
	       i2c_adapter_id(c->adapter), c->addr , ## arg); } while (0)
#define tvp5146_info(fmt, arg...) do { \
	printk(KERN_INFO "%s %d-%04x: " fmt, c->driver->driver.name, \
	       i2c_adapter_id(c->adapter), c->addr , ## arg); } while (0)
#define tvp5146_dbg(num, fmt, arg...) \
	do { \
		if (debug >= num) \
			printk(KERN_DEBUG "%s debug %d-%04x: " fmt,\
				c->driver->driver.name, \
				i2c_adapter_id(c->adapter), \
				c->addr , ## arg); } while (0)

#if defined (CONFIG_MACH_ARCHOS_G6)
static void _select_tvp(void) {
	struct video_in_io_fops *pt_video_in_get_io;

	
	pt_video_in_get_io = archosg6_video_in_get_io();

	pt_video_in_get_io->select_chip(SELECT_TVP);
	pt_video_in_get_io->tvp_bus_width_select(TVP_8_BITS);
}

static int _get_i2c_nb(void) {
	struct video_in_io_fops *pt_video_in_get_io;

	pt_video_in_get_io = archosg6_video_in_get_io();

	return pt_video_in_get_io->get_tvp_i2c_nb();
	
}

#endif

/* supported controls */
static struct v4l2_queryctrl tvp5146_qctrl[] = {
	{
		 .id = V4L2_CID_BRIGHTNESS,
		 .type = V4L2_CTRL_TYPE_INTEGER,
		 .name = "Brightness",
		 .minimum = 0,
		 .maximum = 255,
		 .step = 1,
		 .default_value = 0,
		 .flags = 0,
	 }, {
		.id = V4L2_CID_CONTRAST,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Contrast",
		.minimum = 0,
		.maximum = 255,
		.step = 0x1,
		.default_value = 0x10,
		.flags = 0,
	}, {
		 .id = V4L2_CID_SATURATION,
		 .type = V4L2_CTRL_TYPE_INTEGER,
		 .name = "Saturation",
		 .minimum = 0,
		 .maximum = 255,
		 .step = 0x1,
		 .default_value = 0x10,
		 .flags = 0,
	}, {
		.id = V4L2_CID_HUE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Hue",
		.minimum = -127,
		.maximum = 127,
		.step = 0x1,
		.default_value = 0x10,
		.flags = 0,
	}, {
		 .id = V4L2_CID_AUTOGAIN,
		 .type = V4L2_CTRL_TYPE_BOOLEAN,
		 .name = "Automatic gain",
		 .minimum = 0,
		 .maximum = 1,
		 .step = 1,
		 .default_value = 1,
		 .flags = 0,
	 }, {
		.id = V4L2_CID_GREEN_GAIN,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "ggain",
		.minimum = 0,
		.maximum = 99,
		.step = 0x1,
		.default_value = 0x31,
		.flags = 0,
	}, {
		.id = V4L2_CID_BLUE_GAIN,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "bgain",
		.minimum = 0,
		.maximum = 99,
		.step = 0x1,
		.default_value = 0x31,
		.flags = 0,
	}, {
		.id = V4L2_CID_RED_GAIN,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "rgain",
		.minimum = 0,
		.maximum = 99,
		.step = 0x1,
		.default_value = 0x31,
		.flags = 0,
	}, {
		.id = V4L2_CID_PB_SATURATION,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "PbSat",
		.minimum = 0,
		.maximum = 255,
		.step = 0x1,
		.default_value = 0x31,
		.flags = 0,
	}, {
		.id = V4L2_CID_PR_SATURATION,
		.type =
		V4L2_CTRL_TYPE_INTEGER,
		.name = "PrSat",
		.minimum = 0,
		.maximum = 255,
		.step = 0x1,
		.default_value = 0x31,
		.flags = 0,
	}
};

struct tvp5146 {
	struct i2c_client *client;

	v4l2_std_id norm;	/* Current set standard */
	int input;
	int enable;
	int bright;
	int contrast;
	int hue;
	int sat;
	int sat_pr;
	int sat_pb;
	int rgain;
	int bgain;
	int ggain;
};

static inline int tvp5146_read(struct i2c_client *c, unsigned char addr)
{
	unsigned char buffer[1];
	int rc;

	buffer[0] = addr;
	if (1 != (rc = i2c_master_send(c, buffer, 1)))
		tvp5146_dbg(0, "i2c i/o error: rc == %d (should be 1)\n",
			    rc);

	msleep(10);

	if (1 != (rc = i2c_master_recv(c, buffer, 1)))
		tvp5146_dbg(0, "i2c i/o error: rc == %d (should be 1)\n",
			    rc);

	tvp5146_dbg(3, "tvp5146: read 0x%02x = 0x%02x\n", addr, buffer[0]);

	return (buffer[0]);
}

static inline void tvp5146_write(struct i2c_client *c, unsigned char addr,
				 unsigned char value)
{
	unsigned char buffer[2];
	int rc;

	buffer[0] = addr;
	buffer[1] = value;
	tvp5146_dbg(3, "tvp5146: writing 0x%02x 0x%02x\n", buffer[0],
		    buffer[1]);
	if (2 != (rc = i2c_master_send(c, buffer, 2)))
		tvp5146_dbg(0, "i2c i/o error: rc == %d (should be 2)\n",
			    rc);
}

static inline unsigned int tvp5146_vbusread(struct i2c_client *c, unsigned int addr)
{
	unsigned int value;
	tvp5146_write(c, TVP5146_VBUS_ADRESS1, addr & 0xff);
	tvp5146_write(c, TVP5146_VBUS_ADRESS2, addr >>8 & 0xff );
	tvp5146_write(c, TVP5146_VBUS_ADRESS3, addr >>16 & 0xff);
	value = tvp5146_read(c, TVP5146_VBUS_DATA_ACCESS1);
	tvp5146_dbg(3,"tvp5146: read vbus reg 0x%x (%x:%x:%x)= %x\n", addr,addr & 0xff, addr >>8 & 0xff, addr >>16 & 0xff, value);
	return value;
}

static inline void tvp5146_vbuswrite(struct i2c_client *c, unsigned int addr, unsigned int value)
{
	tvp5146_write(c, TVP5146_VBUS_ADRESS1, addr & 0xff);
	tvp5146_write(c, TVP5146_VBUS_ADRESS2, addr >>8 & 0xff );
	tvp5146_write(c, TVP5146_VBUS_ADRESS3, addr >>16 & 0xff);
	tvp5146_write(c, TVP5146_VBUS_DATA_ACCESS1, value);
	tvp5146_dbg(3,"tvp5146: write vbus reg 0x%x (%x:%x:%x)= %x\n", addr,addr & 0xff, addr >>8 & 0xff, addr >>16 & 0xff, value);
}

static void dump_reg_range(struct i2c_client *c, char *s, u8 init,
			   const u8 end, int max_line)
{
	int i = 0;

	while (init != (u8) (end + 1)) {
		if ((i % max_line) == 0) {
			if (i > 0)
				printk("\n");
			printk("tvp5146: %s reg 0x%02x = ", s, init);
		}
		printk("%02x ", tvp5146_read(c, init));

		init++;
		i++;
	}
	printk("\n");
}

static void dump_reg(struct i2c_client *c)
{
	printk("tvp5146: Video input source selection #1 = 0x%02x\n",
	       tvp5146_read(c, TVP5146_VD_IN_SRC_SEL));
	printk("tvp5146: AFE gain controls = 0x%02x\n",
	       tvp5146_read(c, TVP5146_AFE_GAIN_CTL));
	printk("tvp5146: Video standard = 0x%02x\n",
	       tvp5146_read(c, TVP5146_VIDEO_STD));
	printk("tvp5146: Operation mode controls = 0x%02x\n",
	       tvp5146_read(c, TVP5146_OP_MODE_CTL));
	printk("tvp5146: Autoswitch mask= 0x%02x\n",
	       tvp5146_read(c, TVP5146_AUTOSW_MSK));
	printk("tvp5146: Color killer threshold control = 0x%02x\n",
	       tvp5146_read(c, TVP5146_COLOR_KIL_THSH_CTL));
	printk("tvp5146: Status regs #1 & #2 = %02x %02x \n",
	       tvp5146_read(c, TVP5146_STATUS_REG_1),
	       tvp5146_read(c, TVP5146_STATUS_REG_2));

	printk("tvp5146: Luminance processing controls #1 #2 and #3 = %02x %02x %02x\n",
	     tvp5146_read(c, TVP5146_LUMA_PROC_CTL_1), 
	     tvp5146_read(c, TVP5146_LUMA_PROC_CTL_2),
	     tvp5146_read(c, TVP5146_LUMA_PROC_CTL_3));
	printk("tvp5146: Brightness control = 0x%02x\n",
	       tvp5146_read(c, TVP5146_BRIGHT_CTL));
	printk("tvp5146: Color saturation control = 0x%02x\n",
	       tvp5146_read(c, TVP5146_SATURATION_CTL));
	printk("tvp5146: Hue control = 0x%02x\n",
	       tvp5146_read(c, TVP5146_HUE_CTL));
	printk("tvp5146: Contrast control = 0x%02x\n",
	       tvp5146_read(c, TVP5146_CONTRAST_CTL));
	printk("tvp5146: Chrominance processing control #1 and #2 = %02x %02x\n",
	     tvp5146_read(c, TVP5146_CHROMA_PROC_CTL_1), 
	     tvp5146_read(c, TVP5146_CHROMA_PROC_CTL_2));
	printk("tvp5146: AGC Gain Status = 0x%02x%02x\n",
	       tvp5146_read(c, TVP5146_ACG_GAIN2), 
	       tvp5146_read(c, TVP5146_ACG_GAIN1));
	printk("tvp5146: Video standard status = 0x%02x\n",
	       tvp5146_read(c, TVP5146_STD_STATUS));

	printk("tvp5146: Component Pr saturation = 0x%02x\n",
	       tvp5146_read(c, TVP5146_COMP_PR_SAT));
	printk("tvp5146: Component Y contrast = 0x%02x\n",
	       tvp5146_read(c, TVP5146_COMP_Y_CONT));
	printk("tvp5146: Component Pb saturation = 0x%02x\n",
	       tvp5146_read(c, TVP5146_COMP_PB_SAT));
	printk("tvp5146: Component Y brigthness = 0x%02x\n",
	       tvp5146_read(c, TVP5146_COMP_Y_BRIGHT));

	printk("tvp5146: Horizontal sync start = 0x%02x%02x\n",
	       tvp5146_read(c, TVP5146_HSYNC_START1),
	       tvp5146_read(c, TVP5146_HSYNC_START2));
	printk("tvp5146: Horizontal sync stop = 0x%02x%02x\n",
	       tvp5146_read(c, TVP5146_HSYNC_STOP1),
	       tvp5146_read(c, TVP5146_HSYNC_STOP2));
	printk("tvp5146: Vertical sync start = 0x%02x%02x\n",
	       tvp5146_read(c, TVP5146_VSYNC_START1),
	       tvp5146_read(c, TVP5146_VSYNC_START2));
	printk("tvp5146: Vertical sync stop = 0x%02x%02x\n",
	       tvp5146_read(c, TVP5146_VSYNC_STOP1),
	       tvp5146_read(c, TVP5146_VSYNC_STOP2));
	printk("tvp5146: Vertical blanking start = 0x%02x%02x\n",
	       tvp5146_read(c, TVP5146_VBLANKING_START1),
	       tvp5146_read(c, TVP5146_VBLANKING_START2));
	printk("tvp5146: Vertical blanking stop = 0x%02x%02x\n",
	       tvp5146_read(c, TVP5146_VBLANKING_STOP1),
	       tvp5146_read(c, TVP5146_VBLANKING_STOP2));
	printk("tvp5146: FSS ctrl = 0x%02x\n",
	       tvp5146_read(c, TVP5146_FAST_SWITCH_CTL));
	printk("tvp5146: AFE B gain = 0x%02x%02x\n",
	       tvp5146_read(c, TVP5146_AFE_GAIN_PB2),
	       tvp5146_read(c, TVP5146_AFE_GAIN_PB1));
	printk("tvp5146: AFE G gain = 0x%02x%02x\n",
	       tvp5146_read(c, TVP5146_AFE_GAIN_YG2),
	       tvp5146_read(c, TVP5146_AFE_GAIN_YG1));
	printk("tvp5146: AFE R gain = 0x%02x%02x\n",
	       tvp5146_read(c, TVP5146_AFE_GAIN_PR2),
	       tvp5146_read(c, TVP5146_AFE_GAIN_PR1));
	printk("tvp5146: AFE CVBS gain = 0x%02x%02x\n",
	       tvp5146_read(c, TVP5146_AFE_GAIN_CVBS2),
	       tvp5146_read(c, TVP5146_AFE_GAIN_CVBS1));
	printk("tvp5146: ROM version = 0x%02x\n",
	       tvp5146_read(c, TVP5146_ROM_VERSION));
	printk("tvp5146: AGC white peak = 0x%02x\n",
	       tvp5146_read(c, TVP5146_ACG_PEAK_PROC));
	printk("tvp5146: AGC speed = 0x%02x\n",
	       tvp5146_read(c, TVP5146_ACG_INC_SPEED));
	printk("tvp5146: AGC delay = 0x%02x\n",
	       tvp5146_read(c, TVP5146_ACG_INC_DELAY));

#if 0
// to fix
	printk("tvp5146: Outputs and data rates select = 0x%02x\n",
	       tvp5146_read(c, TVP5146_DATA_RATE_SEL));
	printk("tvp5146: Configuration shared pins = 0x%02x\n",
	       tvp5146_read(c, TVP5146_CONF_SHARED_PIN));
	printk("tvp5146: RTC = 0x%02x\n", tvp5146_read(c, TVP5146_RTC));
	printk("tvp5146: Interrupt reset register B = 0x%02x\n",
	       tvp5146_read(c, TVP5146_INT_RESET_REG_B));
	printk("tvp5146: Interrupt enable register B = 0x%02x\n",
	       tvp5146_read(c, TVP5146_INT_ENABLE_REG_B));
	printk("tvp5146: Interrupt configuration register B = 0x%02x\n",
	       tvp5146_read(c, TVP5146_INTT_CONFIG_REG_B));
	printk("tvp5146: Vertical line count = 0x%02x%02x\n",
	       tvp5146_read(c, TVP5146_VERT_LN_COUNT_MSB),
	       tvp5146_read(c, TVP5146_VERT_LN_COUNT_LSB));
	printk("tvp5146: Interrupt status register B = 0x%02x\n",
	       tvp5146_read(c, TVP5146_INT_STATUS_REG_B));
	printk("tvp5146: Interrupt active register B = 0x%02x\n",
	       tvp5146_read(c, TVP5146_INT_ACTIVE_REG_B));
#if 0				/* This will pop a value from vbi reg */
	printk("tvp5146: VBI FIFO read data = 0x%02x\n",
	       tvp5146_read(c, TVP5146_VBI_FIFO_READ_DATA));
#endif

	dump_reg_range(c, "Teletext filter 1", TVP5146_TELETEXT_FIL1_INI,
		       TVP5146_TELETEXT_FIL1_END, 8);
	dump_reg_range(c, "Teletext filter 2", TVP5146_TELETEXT_FIL2_INI,
		       TVP5146_TELETEXT_FIL2_END, 8);

	printk("tvp5146: Teletext filter enable = 0x%02x\n",
	       tvp5146_read(c, TVP5146_TELETEXT_FIL_ENA));
	printk("tvp5146: Interrupt status register A = 0x%02x\n",
	       tvp5146_read(c, TVP5146_INT_STATUS_REG_A));
	printk("tvp5146: Interrupt enable register A = 0x%02x\n",
	       tvp5146_read(c, TVP5146_INT_ENABLE_REG_A));
	printk("tvp5146: Interrupt configuration = 0x%02x\n",
	       tvp5146_read(c, TVP5146_INT_CONF));
	printk("tvp5146: VDP status register = 0x%02x\n",
	       tvp5146_read(c, TVP5146_VDP_STATUS_REG));
	printk("tvp5146: FIFO word count = 0x%02x\n",
	       tvp5146_read(c, TVP5146_FIFO_WORD_COUNT));
	printk("tvp5146: FIFO interrupt threshold = 0x%02x\n",
	       tvp5146_read(c, TVP5146_FIFO_INT_THRESHOLD));
	printk("tvp5146: FIFO reset = 0x%02x\n",
	       tvp5146_read(c, TVP5146_FIFO_RESET));
	printk("tvp5146: Line number interrupt = 0x%02x\n",
	       tvp5146_read(c, TVP5146_LINE_NUMBER_INT));
	printk("tvp5146: Pixel alignment register = 0x%02x%02x\n",
	       tvp5146_read(c, TVP5146_PIX_ALIGN_REG_HIGH),
	       tvp5146_read(c, TVP5146_PIX_ALIGN_REG_LOW));
	printk("tvp5146: FIFO output control = 0x%02x\n",
	       tvp5146_read(c, TVP5146_FIFO_OUT_CTRL));
	printk("tvp5146: Full field enable = 0x%02x\n",
	       tvp5146_read(c, TVP5146_FULL_FIELD_ENA));
	printk("tvp5146: Full field mode register = 0x%02x\n",
	       tvp5146_read(c, TVP5146_FULL_FIELD_MODE_REG));

	dump_reg_range(c, "CC   data", TVP5146_CC_DATA_INI,
		       TVP5146_CC_DATA_END, 8);

	dump_reg_range(c, "WSS  data", TVP5146_WSS_DATA_INI,
		       TVP5146_WSS_DATA_END, 8);

	dump_reg_range(c, "VPS  data", TVP5146_VPS_DATA_INI,
		       TVP5146_VPS_DATA_END, 8);

	dump_reg_range(c, "VITC data", TVP5146_VITC_DATA_INI,
		       TVP5146_VITC_DATA_END, 10);

	dump_reg_range(c, "Line mode", TVP5146_LINE_MODE_INI,
		       TVP5146_LINE_MODE_END, 8);
#endif
}



static v4l2_std_id get_current_standard(struct i2c_client *c)
{
	v4l2_std_id std;
	u8 reg5;

	std = 0;
	reg5 = tvp5146_read(c, TVP5146_STD_STATUS);

	switch (reg5 & TVP5146_STATUS_STANDARD_MASK) {
	case TVP5146_STANDARD_NTSC_M:
		std |= V4L2_STD_NTSC_M;
		break;
	case TVP5146_STANDARD_PAL_BGHIN:
		std |=
		    V4L2_STD_PAL_B | V4L2_STD_PAL_G | V4L2_STD_PAL_H |
		    V4L2_STD_PAL_I | V4L2_STD_PAL_N;
		break;
	case TVP5146_STANDARD_PAL_M:
		std |= V4L2_STD_PAL_M;
		break;
	case TVP5146_STANDARD_PAL_N:
		std |= V4L2_STD_PAL_N;
		break;
	case TVP5146_STANDARD_NTSC443:
		std |= V4L2_STD_NTSC_443;
		break;
	case TVP5146_STANDARD_SECAM:
		std |= V4L2_STD_SECAM;
		break;
	default:
		std = 0;
		break;
	}

	return std;
}

/****************************************************************************
			Basic functions
 ****************************************************************************/
enum tvp5146_input {
	TVP5146_ANALOG_CH0 = 0,
	TVP5146_SVIDEO = 1,
	TVP5146_SCART = 2,
	TVP5146_YPbPr = 3,
	TVP5146_RGB = 4,
	TVP5146_ANALOG_CH1 = 5,
	TVP5146_BLACK_SCREEN = 8
};

static inline void tvp5146_selmux(struct i2c_client *c,
				  enum tvp5146_input input)
{

	tvp5146_reset_ctrl_rgb(c);
	tvp5146_reset_ctrl_component(c);

	switch (input) {
	case TVP5146_ANALOG_CH0:
#ifdef MODE_FAST_SWITCH
		input = TVP5146_INPUT_SCART;
#else
		input = TVP5146_INPUT_CVBS4;
		tvp5146_write(c, TVP5146_LUMA_PROC_CTL_2, 0x04);
#endif
		printk("input = TVP5146_INPUT_CVBS 4\n");
		break;
	case TVP5146_ANALOG_CH1:
		input = TVP5146_INPUT_CVBS1;
		tvp5146_write(c, TVP5146_LUMA_PROC_CTL_2, 0x04);
		printk("input = TVP5146_INPUT_CVBS 1\n");
		break;
	case TVP5146_SVIDEO:
		input = TVP5146_INPUT_SVIDEO;
		tvp5146_write(c, TVP5146_LUMA_PROC_CTL_2, 0x4c);
		printk("input = TVP5146_INPUT_SVIDEO\n");
		break;
	case TVP5146_SCART:
		input = TVP5146_INPUT_SCART;
		printk("input = TVP5146_INPUT_SCART\n");
		break;
	case TVP5146_YPbPr:
		input = TVP5146_INPUT_YPbPr;
		printk("input = TVP5146_INPUT_YPbPr\n");
		break;
	case TVP5146_RGB:
		input = TVP5146_INPUT_RGB;
		printk("input = TVP5146_INPUT_RGB\n");
		break;
	default:
		input = TVP5146_INPUT_CVBS4;
		printk("input = default\n");
		break;
	}

	tvp5146_write(c, TVP5146_VD_IN_SRC_SEL, input);
	tvp5146_write(c, TVP5146_CLR_LOCK_DETECT, 1);	// clear lost lock detect status bit
};


/*
 * ======== setup656sync ========
 */
static int setup656sync(struct i2c_client *c, int enable)
{
	unsigned char output1, output2, output4;
	unsigned char output6;

	if (enable) {
		output1 = 0x40;
		output4 = 0xFF;
		output6 = 0;
	} else {
		output1 = 0x43;
		output4 = 0xAF;
		output6 = 0x1E;
	}

	output2 = 0x11;		/* enable clock, enable Y[9:0] */

	tvp5146_write(c, TVP5146_OUTPUT_FORMATER1, output1);
	tvp5146_write(c, TVP5146_OUTPUT_FORMATER2, output2);
	tvp5146_write(c, TVP5146_OUTPUT_FORMATER4, output4);
	tvp5146_write(c, TVP5146_SYNC_CTL, output6);
	return 0;
}

	// init tab to define
/* Default values as suggested at TVP5146 datasheet */
static const struct i2c_reg_value tvp5146_init_default[] = {
	{			/* 0xe8 */
	 TVP5146_VBUS_ADRESS1, 0x02},
	{			/* 0xe9 */
	 TVP5146_VBUS_ADRESS2, 0x00},
	{			/* 0xea */
	 TVP5146_VBUS_ADRESS3, 0x80},
	{			/* 0xe0 */
	 TVP5146_VBUS_DATA_ACCESS1, 0x01},
	{			/* 0xe8 */
	 TVP5146_VBUS_ADRESS1, 0x60},
	{			/* 0xe9 */
	 TVP5146_VBUS_ADRESS2, 0x00},
	{			/* 0xea */
	 TVP5146_VBUS_ADRESS3, 0xb0},
	{			/* 0xe0 */
	 TVP5146_VBUS_DATA_ACCESS1, 0x01},
	{			/* 0xe0 */
	 TVP5146_VBUS_DATA_ACCESS1, 0x00},
	{			/* 0x03 */
	 TVP5146_OP_MODE_CTL, 0x01},
 	{			/* 0x03 */
	 TVP5146_OP_MODE_CTL, 0x00},
	{			/* end of data */
	 0xff, 0xff}
};

/* Default values */
static const struct i2c_reg_value tvp5146_init_enable[] = {

	{			/* disable ACG lum */
	 TVP5146_AFE_GAIN_CTL, 0x0f	//0x0e       
	 },
	{			/* MB: Increase sharpness, successfully tested on AVxx series */
	 TVP5146_LUMA_PROC_CTL_2, 0x0c	//
	 },
	{			/* MB: Increase sharpness, successfully tested on AVxx series */
	 TVP5146_LUMA_PROC_CTL_3, 0x03	//
	 },
	{			/* MB: Disable wideband chroma filter, use notch 2, successfully tested on AVxx */
	 TVP5146_CHROMA_PROC_CTL_2, 0x02	//
	 },
#ifdef MODE_FAST_SWITCH
	{			/* enable FSS pin action for cvbs/rgb */
	 TVP5146_FAST_SWITCH_CTL, 0x0c	//
	 },
#endif
	{			/* MB: enable auto color killer, level min -> kill a max of color */
	 TVP5146_COLOR_KIL_THSH_CTL, 0x00	//
	 },
	{			// ??? unmask all video format in auto mode prevent detection of composite as svideo!!
	 TVP5146_AUTOSW_MSK, TVP5146_AUTOSWITCH_MASK},
	{
	 0xff, 0xff}
};


struct tvp5146_vbi_type {
	unsigned int vbi_type;
	unsigned int ini_line;
	unsigned int end_line;
	unsigned int by_field:1;
};

static struct tvp5146_vbi_type vbi_supported_services[] = {
	/* Teletext, SECAM, WST System A */
	{V4L2_SLICED_TELETEXT_SECAM, 6, 23, 1},
	/* Teletext, PAL, WST System B */
	{V4L2_SLICED_TELETEXT_PAL_B, 6, 22, 1},
	/* Teletext, PAL, WST System C */
	{V4L2_SLICED_TELETEXT_PAL_C, 6, 22, 1},
	/* Teletext, NTSC, WST System B */
	{V4L2_SLICED_TELETEXT_NTSC_B, 10, 21, 1},
	/* Tetetext, NTSC NABTS System C */
	{V4L2_SLICED_TELETEXT_NTSC_C, 10, 21, 1},
	/* Teletext, NTSC-J, NABTS System D */
	{V4L2_SLICED_TELETEXT_NTSC_D, 10, 21, 1},
	/* Closed Caption, PAL/SECAM */
	{V4L2_SLICED_CAPTION_625, 22, 22, 1},
	/* Closed Caption, NTSC */
	{V4L2_SLICED_CAPTION_525, 21, 21, 1},
	/* Wide Screen Signal, PAL/SECAM */
	{V4L2_SLICED_WSS_625, 23, 23, 1},
	/* Wide Screen Signal, NTSC C */
	{V4L2_SLICED_WSS_525, 20, 20, 1},
	/* Vertical Interval Timecode (VITC), PAL/SECAM */
	{V4L2_SLICED_VITC_625, 6, 22, 0},
	/* Vertical Interval Timecode (VITC), NTSC */
	{V4L2_SLICED_VITC_525, 10, 20, 0},
	/* Video Program System (VPS), PAL */
	{V4L2_SLICED_VPS, 16, 16, 0},
	/* End of struct */
	{(u16) - 1, 0, 0}
};

static void tvp5146_selgpcl(struct i2c_client *c, int on)
{
#if 0
	unsigned char value =
	    (unsigned char) tvp5146_read(c, TVP5150_MISC_CTL);

	if (on) {
		value |= TVP5150_GPCL_OUTPUT_MASK;
	} else {
		value &= (unsigned char) ~TVP5150_GPCL_OUTPUT_MASK;
	}

	tvp5146_dbg(1, "tvp5146_selgpcl on ? %d value %02x\r\n", on,
		    value);
	tvp5146_write(c, TVP5150_MISC_CTL, value);
#endif
};


static int tvp5146_write_inittab(struct i2c_client *c,
				 const struct i2c_reg_value *regs)
{

	while (regs->reg != 0xff) {
#if 0
		if (regs->reg == TVP5150_MISC_CTL) {

			unsigned char value =
			    (unsigned char) tvp5146_read(c, regs->reg)
			    & TVP5150_GPCL_OUTPUT_MASK;

			tvp5146_dbg(1,
				    "tvp5146_write_inittab TVP5150_MISC_CTL old value %02x \r\n",
				    value);
			value |= (regs->value & (unsigned char)
				  ~TVP5150_GPCL_OUTPUT_MASK);

			tvp5146_dbg(1,
				    "tvp5146_write_inittab TVP5150_MISC_CTL new value %02x wanted value %02x\r\n",
				    value, regs->value);
			tvp5146_write(c, regs->reg, value);
		} else
#endif
		{
			tvp5146_write(c, regs->reg, regs->value);
		}

		regs++;
	}

	return 0;
}

const unsigned char vbi_vdp_ram_default[512] = {
	0xAA, 0xAA, 0xFF, 0xFF, 0xE7, 0x2E, 0x20, 0xA6, 0xE4, 0xB4, 0x0E, 0x00, 0x07, 0x00, 0x10, 0x00,
	0xAA, 0xAA, 0xFF, 0xFF, 0xE7, 0x2E, 0x20, 0xA6, 0xE4, 0xB4, 0x0E, 0x00, 0x07, 0x00, 0x10, 0x00,
	0xAA, 0xAA, 0xFF, 0xFF, 0x27, 0x2E, 0x20, 0xAB, 0xA4, 0x72, 0x10, 0x00, 0x07, 0x00, 0x10, 0x00,
	0xAA, 0xAA, 0xFF, 0xFF, 0x27, 0x2E, 0x20, 0xAB, 0xA4, 0x72, 0x10, 0x00, 0x07, 0x00, 0x10, 0x00,
	0xAA, 0xAA, 0xFF, 0xFF, 0xE7, 0x2E, 0x20, 0x22, 0xA4, 0x98, 0x0D, 0x00, 0x00, 0x00, 0x10, 0x00,
	0xAA, 0xAA, 0xFF, 0xFF, 0xE7, 0x2E, 0x20, 0x22, 0xA4, 0x98, 0x0D, 0x00, 0x00, 0x00, 0x10, 0x00,
	0xAA, 0xAA, 0xFF, 0xFF, 0x27, 0x2E, 0x20, 0x23, 0x63, 0x93, 0x0D, 0x00, 0x00, 0x00, 0x10, 0x00,
	0xAA, 0xAA, 0xFF, 0xFF, 0x27, 0x2E, 0x20, 0x23, 0x63, 0x93, 0x0D, 0x00, 0x00, 0x00, 0x10, 0x00,
	0xAA, 0xAA, 0xFF, 0xFF, 0xE7, 0x2E, 0x20, 0xA2, 0x63, 0x93, 0x0D, 0x00, 0x07, 0x00, 0x15, 0x00,
	0xAA, 0xAA, 0xFF, 0xFF, 0xE7, 0x2E, 0x20, 0xA2, 0x63, 0x93, 0x0D, 0x00, 0x07, 0x00, 0x15, 0x00,
	0xAA, 0xAA, 0xFF, 0xFF, 0xA7, 0x2E, 0x20, 0xA3, 0x63, 0x93, 0x0D, 0x00, 0x07, 0x00, 0x10, 0x00,
	0xAA, 0xAA, 0xFF, 0xFF, 0xA7, 0x2E, 0x20, 0xA3, 0x63, 0x93, 0x0D, 0x00, 0x07, 0x00, 0x10, 0x00,
	0xAA, 0x2A, 0xFF, 0x3F, 0x04, 0x51, 0x6E, 0x02, 0xA4, 0x7B, 0x09, 0x00, 0x00, 0x00, 0x27, 0x00,
	0xAA, 0x2A, 0xFF, 0x3F, 0x04, 0x51, 0x6E, 0x02, 0xA4, 0x7B, 0x09, 0x00, 0x00, 0x00, 0x27, 0x00,
	0xAA, 0x2A, 0xFF, 0x3F, 0x04, 0x51, 0x6E, 0x02, 0x69, 0x8C, 0x09, 0x00, 0x00, 0x00, 0x27, 0x00,
	0xAA, 0x2A, 0xFF, 0x3F, 0x04, 0x51, 0x6E, 0x02, 0x69, 0x8C, 0x09, 0x00, 0x00, 0x00, 0x27, 0x00,	//$0F0 CC,NTSC
	0x5B, 0x55, 0xC5, 0xFF, 0x00, 0x71, 0x6E, 0x42, 0xA4, 0xCD, 0x0F, 0x00, 0x00, 0x00, 0x3A, 0x00,
	0x5B, 0x55, 0xC5, 0xFF, 0x00, 0x71, 0x6E, 0x42, 0xA4, 0xCD, 0x0F, 0x00, 0x00, 0x00, 0x3A, 0x00,
	0x38, 0x00, 0x3F, 0x00, 0x00, 0x71, 0x6E, 0x43, 0x63, 0x7C, 0x08, 0x00, 0x00, 0x00, 0x39, 0x00,
	0x38, 0x00, 0x3F, 0x00, 0x00, 0x71, 0x6E, 0x43, 0x63, 0x7C, 0x08, 0x00, 0x00, 0x00, 0x39, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x8F, 0x6D, 0x49, 0xA4, 0x85, 0x08, 0x00, 0x00, 0x00, 0x4C, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x8F, 0x6D, 0x49, 0xA4, 0x85, 0x08, 0x00, 0x00, 0x00, 0x4C, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x8F, 0x6D, 0x49, 0x63, 0x94, 0x08, 0x00, 0x00, 0x00, 0x4C, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x8F, 0x6D, 0x49, 0x63, 0x94, 0x08, 0x00, 0x00, 0x00, 0x4C, 0x00,
	0xAA, 0xAA, 0xFF, 0xFF, 0xBA, 0xCE, 0x2B, 0x8D, 0xA4, 0xDA, 0x0B, 0x00, 0x07, 0x00, 0x60, 0x00,
	0xAA, 0xAA, 0xFF, 0xFF, 0xBA, 0xCE, 0x2B, 0x8D, 0xA4, 0xDA, 0x0B, 0x00, 0x07, 0x00, 0x60, 0x00,
	0x99, 0x99, 0xFF, 0xFF, 0x05, 0x51, 0x6E, 0x05, 0x63, 0x18, 0x13, 0x80, 0x00, 0x00, 0x60, 0x00,
	0x99, 0x99, 0xFF, 0xFF, 0x05, 0x51, 0x6E, 0x05, 0x63, 0x18, 0x13, 0x80, 0x00, 0x00, 0x60, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static int tvp5146_vdp_init(struct i2c_client *c,
			    const unsigned char *ram_values)
{
#if 0
	unsigned int i;
	u8 val;

	/* Disable Full Field */
	tvp5146_write(c, TVP5150_FULL_FIELD_ENA, 0);

	/* Before programming, Line mode should be at 0xff */
	for (i = TVP5150_LINE_MODE_INI; i <= TVP5150_LINE_MODE_END; i++)
		tvp5146_write(c, i, 0xff);

	tvp5146_write(c, TVP5150_FULL_FIELD_ENA, 0);

	tvp5146_write(c, TVP5150_CONF_RAM_ADDR_LOW, 0);
	val = tvp5146_read(c, TVP5150_CONF_RAM_ADDR_HIGH);
	val &= 0xc5;
	tvp5146_write(c, TVP5150_CONF_RAM_ADDR_HIGH, val);

	for (i = 0; i < 512; i++) {
		tvp5146_write(c, TVP5150_VDP_CONF_RAM_DATA, ram_values[i]);
	}
#endif
	return 0;
}

/* Fills VBI capabilities based on i2c_vbi_ram_value struct */
static void tvp5146_vbi_get_cap(const struct tvp5146_vbi_type
				*supported_services,
				struct v4l2_sliced_vbi_cap *cap)
{
	int line;
	const struct tvp5146_vbi_type *service = supported_services;

	memset(cap, 0, sizeof *cap);

	while (service->vbi_type != (u16) - 1) {
		for (line = service->ini_line;
		     line <= service->end_line; line++) {
			cap->service_lines[0][line] |= service->vbi_type;
			cap->service_lines[1][line] |= service->vbi_type;
		}
		cap->service_set |= service->vbi_type;

		service++;
	}
}

static int tvp5146_get_vbi(struct i2c_client *c, const struct tvp5146_vbi_type
			   *supported_services, int line, const int field)
{
	struct tvp5146 *decoder = i2c_get_clientdata(c);
	v4l2_std_id std = decoder->norm;
	u8 reg,line_mode;
	int pos = 0, type = 0, i = 0;

	if (std == V4L2_STD_ALL) {
		std = get_current_standard(c);
	}

	if (std == V4L2_STD_UNKNOWN) {
		tvp5146_err
		    ("VBI can't be configured without knowing number of lines\n");
		return 0;
	}

	if (std && V4L2_STD_625_50) {
		/* Don't follow NTSC Line number convension */
		line += 3;
	}

	if (line < 6 || line > 27)
		return 0;

	for ( i==0; i<GENERAL_LINEREGS_NUM; i++ ) {
		if ( tvp5146_vbusread(c, 0x800600+i*2) == line ) {
			line_mode = tvp5146_vbusread(c, 0x800601+i*2);
			switch (line_mode & 0x07) {
			case 0x0:
				type = V4L2_SLICED_TELETEXT;
				break;
			case 0x01:
				type = V4L2_SLICED_CAPTION;
				break;
			case 0x02:
				type = V4L2_SLICED_WSS;
				break;
			case 0x03:
				type = V4L2_SLICED_VITC;
				break;
			case 0x04:
				type = V4L2_SLICED_VPS;
				break;
			default:
				break;
			}
		}
	}

	return type;
}

/* Set vbi processing
 * type - one of tvp5146_vbi_types
 * line - line to gather data
 * fields: bit 0 field1, bit 1, field2
 * flags (default=0xf0) is a bitmask, were set means:
 *	bit 7: enable filtering null bytes on CC
 *	bit 6: send data also to FIFO
 *	bit 5: don't allow data with errors on FIFO
 *	bit 4: enable ECC when possible
 * pix_align = pix alignment:
 *	LSB = field1
 *	MSB = field2
 * VDP line used vertical date processor VDP
 * Only on Teletextemode and VITC service, the VDP has to process multiple lines, so we use general line mode for the other services
 */

static int tvp5146_set_vbi(struct i2c_client *c, const struct tvp5146_vbi_type
			   *supported_services, unsigned int type,
			   u8 flags, int line, const int fields)
{
	struct tvp5146 *decoder = i2c_get_clientdata(c);
	v4l2_std_id std = decoder->norm;
	u8 reg,line_mode=0;
	int pos = 0, i = 0;
	const struct tvp5146_vbi_type *service = supported_services;
	static register_occupied = 0;  // count the number of the occupied general line VDP registers, max:9

	if (std == V4L2_STD_ALL) {
		std = get_current_standard(c);
	}

	tvp5146_dbg(1, "set_vbi type=%x, line=%d\r\n", type, line);

	if (std == V4L2_STD_UNKNOWN) {
		tvp5146_err
		    ("VBI can't be set without knowing number of lines\n");
		return 0;
	}

	while (service->vbi_type != (u16) - 1) {
		if ((type & service->vbi_type) &&
		    (line >= service->ini_line) &&
		    (line <= service->end_line)) {
			/*found */
			type = service->vbi_type;
			break;
		}

		service++;
		pos++;
	}
	if (service->vbi_type == (u16) - 1) {
		tvp5146_err("Unknown vbi\n");
		return 0;
	}

// 	if (std & V4L2_STD_625_50) {
// 		/* Don't follow NTSC Line number convention */
// 		/* TVP5150 seems to have different line counting */
// 		line += 3;
// 	}

	if (line < 6 || line > 27)
		return 0;

	switch ( type ) {
	case V4L2_SLICED_WSS_625:
	case V4L2_SLICED_WSS_525:
		line_mode = 0x02;
		break;
	case V4L2_SLICED_TELETEXT_SECAM:
	case V4L2_SLICED_TELETEXT_PAL_B:
	case V4L2_SLICED_TELETEXT_PAL_C:
	case V4L2_SLICED_TELETEXT_NTSC_B:
	case V4L2_SLICED_TELETEXT_NTSC_C:
	case V4L2_SLICED_TELETEXT_NTSC_D:
		line_mode = 0x0;
		break;
	case V4L2_SLICED_VITC_625:
	case V4L2_SLICED_VITC_525:
		line_mode = 0x03;
		break;
	case V4L2_SLICED_CAPTION_625:
	case V4L2_SLICED_CAPTION_525:
		line_mode = 0x01;
		break;
	case V4L2_SLICED_VPS:
		line_mode = 0x04;
		break;
	default:
		break;
	}

	if ( fields )
		line_mode |= 0x08;
	else
		line_mode &= ~0x08;

	printk("line mode type:%d.\n",line_mode);

	line_mode |= flags & 0xf0;

	printk("line mode type:%d.\n",line_mode);



	if ( service->ini_line == service->end_line ) {
		/* use general line mode */
		if ( register_occupied == 9 ) {
			tvp5146_err("General line mode VDP registers overload\n");
			return 0;
		}
		
		for ( i==0; i<register_occupied; i++ ) {
			if ( tvp5146_vbusread(c, 0x800600+i*2) == line ) {
				printk("The service of line %d is already registered\n",line);
				if ( tvp5146_vbusread(c, 0x800601+i*2) | 0xf == line_mode | 0xf ) {
					printk("The same service of line %d is already registered\n",line);
					tvp5146_vbuswrite(c, 0x800601+i*2, line_mode);
					return type;
				}
				printk("Go to regitser another service for line %d\n",line);	
			}
		}

		tvp5146_vbuswrite(c, 0x800600+register_occupied*2, line);
		tvp5146_vbuswrite(c, 0x800601+register_occupied*2, line_mode);
		register_occupied ++;
	} else {
		/* use global line mode */
		tvp5146_write(c, TVP5146_VDP_LINE_START, service->ini_line);
		tvp5146_write(c, TVP5146_VDP_LINE_STOP, service->end_line);
		tvp5146_write(c, TVP5146_VDP_LINE_MODE, line_mode);
	}
	return type;
}


#if 0
static int decode_vbi_data(struct i2c_client *c, vbi)
{
	count = tvp5146_read(c, TVP5150_FIFO_WORD_COUNT);

	for (i = 0; i < count; i++) {
		*p = tvp5146_read(c, TVP5150_VBI_FIFO_READ_DATA);
		p++;
	}
}
#endif
static int tvp5146_set_std(struct i2c_client *c, v4l2_std_id std)
{
	struct tvp5146 *decoder = i2c_get_clientdata(c);
	int fmt = 0;

	decoder->norm = std;

	/* First tests should be against specific std */

	if (std == V4L2_STD_ALL) {
		fmt = 0;	/* Autodetect mode */
	} else if (std & V4L2_STD_NTSC_443) {
		fmt = TVP5146_STANDARD_NTSC443;
	} else if (std & V4L2_STD_PAL_M) {
		fmt = TVP5146_STANDARD_PAL_M;
	} else if (std & (V4L2_STD_PAL_N | V4L2_STD_PAL_Nc)) {
		fmt = TVP5146_STANDARD_PAL_N;
	} else {
		/* Then, test against generic ones */
		if (std & V4L2_STD_NTSC) {
			fmt = TVP5146_STANDARD_NTSC_M;
		} else if (std & V4L2_STD_PAL) {
			fmt = TVP5146_STANDARD_PAL_BGHIN;
		} else if (std & V4L2_STD_SECAM) {
			fmt = TVP5146_STANDARD_SECAM;
		}
	}

	tvp5146_dbg(1,"Set video std register to %d.\n",fmt);
	tvp5146_write(c, TVP5146_VIDEO_STD, fmt);

	return 0;
}

static int tvp5146_detect_chip(struct i2c_client *c)
{
	int msb_id, lsb_id, lsb_rom;

	msb_id  = tvp5146_read(c, TVP5146_CHIP_ID_MSB);
	lsb_id  = tvp5146_read(c, TVP5146_CHIP_ID_LSB);
	lsb_rom = tvp5146_read(c, TVP5146_ROM_VERSION);

	tvp5146_dbg(0, "tvp%02x%02x chip detected\n", msb_id, lsb_id);
	tvp5146_dbg(0, "Rom version %d\n", lsb_rom);
	
	if (msb_id == 0x51 && lsb_id == 0x46)
		return 0;
printk(KERN_ERR "tvp5146: wrong chip ID: %02x%02x version %d\r\n", msb_id, lsb_id, lsb_rom );
	return -ENODEV;
}

static void tvp5146_reset(struct i2c_client *c)
{
	struct tvp5146 *decoder = i2c_get_clientdata(c);

	/* Initializes TVP5146 to its default values */
	tvp5146_write_inittab(c, tvp5146_init_default);

	/* Initializes VDP registers */
//      tvp5146_vdp_init(c, vbi_vdp_ram_default);

	/* Selects decoder input */
	tvp5146_selmux(c, decoder->input);

	// setup output
	setup656sync(c, 1);

	/* Initializes TVP5146 to stream enabled values */
	tvp5146_write_inittab(c, tvp5146_init_enable);

	// TI doc: set a standart after unmasking autoswitch register
	// and setting autodetect  
	tvp5146_set_std(c, V4L2_STD_SECAM);

	/* Initialize image preferences */
	tvp5146_write(c, TVP5146_BRIGHT_CTL, decoder->bright >> 8);
	tvp5146_write(c, TVP5146_CONTRAST_CTL, decoder->contrast >> 8);
	tvp5146_write(c, TVP5146_SATURATION_CTL, decoder->sat >> 8);
	tvp5146_write(c, TVP5146_HUE_CTL, (decoder->hue - 32768) >> 8);

	tvp5146_set_std(c, V4L2_STD_ALL);

	/* Set TVP5146_ANALOG to power-save mode after init */
	tvp5146_write(c, TVP5146_OP_MODE_CTL, 0x01);
};

static int tvp5146_get_ctrl(struct i2c_client *c, struct v4l2_control *ctrl)
{

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		ctrl->value = tvp5146_read(c, TVP5146_BRIGHT_CTL);
		return 0;
	case V4L2_CID_CONTRAST:
		ctrl->value = tvp5146_read(c, TVP5146_CONTRAST_CTL);
		return 0;
	case V4L2_CID_SATURATION:
		ctrl->value = tvp5146_read(c, TVP5146_SATURATION_CTL);
		return 0;
	case V4L2_CID_HUE:
		ctrl->value = tvp5146_read(c, TVP5146_HUE_CTL);
		return 0;
	}
	return -EINVAL;
}

static int tvp5146_set_ctrl(struct i2c_client *c,
			    struct v4l2_control *ctrl)
{
	struct tvp5146 *decoder = i2c_get_clientdata(c);

	printk("set ctrl, %d %x \n", ctrl->id, ctrl->value);
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		decoder->bright = ctrl->value;
		tvp5146_write(c, TVP5146_BRIGHT_CTL, ctrl->value);
		return 0;
	case V4L2_CID_CONTRAST:
		decoder->contrast = ctrl->value;
		tvp5146_write(c, TVP5146_CONTRAST_CTL, ctrl->value);
		return 0;
	case V4L2_CID_SATURATION:
		decoder->sat = ctrl->value;
		tvp5146_write(c, TVP5146_SATURATION_CTL, ctrl->value);
		return 0;
	case V4L2_CID_HUE:
		decoder->hue = ctrl->value;
		tvp5146_write(c, TVP5146_HUE_CTL, ctrl->value);
		return 0;
	}
	return -EINVAL;
}

static void tvp5146_reset_ctrl_rgb(struct i2c_client *c)
{

	tvp5146_write(c, TVP5146_AFE_GAIN_CTL, 0x0f);	// restore autogain

	tvp5146_write(c, TVP5146_AFE_GAIN_YG1, 0);
	tvp5146_write(c, TVP5146_AFE_GAIN_YG2, 0x04);
	tvp5146_write(c, TVP5146_AFE_GAIN_PR1, 0);
	tvp5146_write(c, TVP5146_AFE_GAIN_PR2, 0x04);
	tvp5146_write(c, TVP5146_AFE_GAIN_PB1, 0);
	tvp5146_write(c, TVP5146_AFE_GAIN_PB2, 0x04);

}

static void tvp5146_reset_ctrl_component(struct i2c_client *c)
{

	tvp5146_write(c, TVP5146_COMP_Y_BRIGHT, 0x80);
	tvp5146_write(c, TVP5146_COMP_Y_CONT, 0x80);
	tvp5146_write(c, TVP5146_COMP_PB_SAT, 0x80);
	tvp5146_write(c, TVP5146_COMP_PR_SAT, 0x80);

}

static int tvp5146_set_ctrl_rgb(struct i2c_client *c,
				struct v4l2_control *ctrl)
{
	struct tvp5146 *decoder = i2c_get_clientdata(c);
	printk("set ctrl rgb, %d %x \n", ctrl->id, ctrl->value);

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		// enable auto gain
		if (ctrl->value)
			tvp5146_write(c, TVP5146_AFE_GAIN_CTL, 0x0f);
		else
			tvp5146_write(c, TVP5146_AFE_GAIN_CTL, 0x0c);
		return 0;

	case V4L2_CID_GREEN_GAIN:
		decoder->ggain = 0x400 + (30 * ctrl->value);
		tvp5146_write(c, TVP5146_AFE_GAIN_YG1,
			      decoder->ggain & 0xff);
		tvp5146_write(c, TVP5146_AFE_GAIN_YG2,
			      (decoder->ggain >> 8) & 0x0f);
		return 0;

	case V4L2_CID_RED_GAIN:
		decoder->rgain = 0x400 + (30 * ctrl->value);
		tvp5146_write(c, TVP5146_AFE_GAIN_PR1,
			      decoder->rgain & 0xff);
		tvp5146_write(c, TVP5146_AFE_GAIN_PR2,
			      (decoder->rgain >> 8) & 0x0f);
		return 0;

	case V4L2_CID_BLUE_GAIN:
		decoder->bgain = 0x400 + (30 * ctrl->value);
		tvp5146_write(c, TVP5146_AFE_GAIN_PB1,
			      decoder->bgain & 0xff);
		tvp5146_write(c, TVP5146_AFE_GAIN_PB2,
			      (decoder->bgain >> 8) & 0x0f);
		return 0;
	}
	return -EINVAL;
}

static int tvp5146_set_ctrl_component(struct i2c_client *c,
				      struct v4l2_control *ctrl)
{
	struct tvp5146 *decoder = i2c_get_clientdata(c);

	printk("set ctrl comp, %d %x \n", ctrl->id, ctrl->value);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		decoder->bright = ctrl->value;
		tvp5146_write(c, TVP5146_COMP_Y_BRIGHT, ctrl->value);
		return 0;
	case V4L2_CID_CONTRAST:
		decoder->contrast = ctrl->value;
		tvp5146_write(c, TVP5146_COMP_Y_CONT, ctrl->value);
		return 0;
	case V4L2_CID_PB_SATURATION:
		decoder->bright = ctrl->value;
		tvp5146_write(c, TVP5146_COMP_PB_SAT, ctrl->value);
		return 0;
	case V4L2_CID_PR_SATURATION:
		decoder->contrast = ctrl->value;
		tvp5146_write(c, TVP5146_COMP_PR_SAT, ctrl->value);
		return 0;

	}
	return -EINVAL;
}


static int tvp5146_get_status( struct i2c_client *c, __u32 *status )
{
//		__u32 *status = arg;

		u8 reg1, reg2;
		static int stat1, stat2;
		reg1 = tvp5146_read(c, TVP5146_STATUS_REG_1);
		reg2 = tvp5146_read(c, TVP5146_STATUS_REG_2);
		tvp5146_dbg(1, "tvp status register 1 & 2: 0x%02x 0x%02x\n",
			    reg1, reg2);

		if ((stat1 != reg1) || ((stat2 & 0x0f) != (reg2 & 0x0f))) {
			printk("tvp status register 1 & 2: 0x%02x 0x%02x\n",
			     reg1, reg2);
			stat1 = reg1;
			stat2 = reg2;
		}
		*status = 0;
		if (!(reg1 & (1 << 1))) {
			// Horizontal Sync is lost
			*status |= V4L2_IN_ST_NO_H_LOCK;
		}
		if (!(reg1 & (1 << 2))) {
			// Vertical Sync is lost
			*status |= V4L2_IN_ST_NO_SIGNAL;
		}
		if (!(reg1 & (1 << 3))) {
			// Color Sync is lost
			*status |= V4L2_IN_ST_NO_COLOR;
		}

		if (reg2 & TVP5146_MACROVISION_DETECT_TYPE1) {
			// AGC (Macrovision Type 1) detection
			*status |= V4L2_IN_ST_MACROVISION;
			*status |= V4L2_IN_ST_MACROVISION_TYPE1;
		}
		if (reg2 & TVP5146_MACROVISION_DETECT_TYPE2) {
			// Colorstripe process Type 2 present (ignored)
			*status |= V4L2_IN_ST_MACROVISION_TYPE2;
		}
		if (reg2 & TVP5146_MACROVISION_DETECT_TYPE3) {
			// Colorstripe process Type 3 present (ignored)
			*status |= V4L2_IN_ST_MACROVISION_TYPE3;
		}

	return 0;
}
/****************************************************************************
			I2C Command
 ****************************************************************************/
static int tvp5146_command(struct i2c_client *c, unsigned int cmd, void *arg)
{
	struct tvp5146 *decoder = i2c_get_clientdata(c);

	switch (cmd) {

	case 0:
	case VIDIOC_INT_RESET:
	case DECODER_INIT:
		tvp5146_reset(c);
		break;

	case VIDIOC_G_SLICED_VBI_CAP: {
		struct v4l2_sliced_vbi_cap *cap = arg;

		tvp5146_vbi_get_cap(vbi_supported_services, cap);
		break;
	}

	case VIDIOC_S_FMT: {
		struct v4l2_format *fmt;
		struct v4l2_sliced_vbi_format *svbi;

		fmt = arg;

		if (fmt->type != V4L2_BUF_TYPE_SLICED_VBI_CAPTURE)
			return -EINVAL;

		svbi = &fmt->fmt.sliced;

		/* Fix me: user could set service_set = 0 and fill service_lines instead */
		if (svbi->service_set & V4L2_SLICED_WSS_625) {
			/* why if we set the field to 1, the wss info is always zero??*/
			if (!tvp5146_set_vbi(c, vbi_supported_services,
			     V4L2_SLICED_WSS_625, 0, 23, 0))
				return -EINVAL;

		} else if (svbi->service_set & V4L2_SLICED_WSS_525) {
			if (!tvp5146_set_vbi(c, vbi_supported_services,
			     V4L2_SLICED_WSS_525, 0, 20, 0))
				return -EINVAL;
		}
		break;
	}

	case VIDIOC_G_FMT: {
		struct v4l2_format *fmt;
		struct v4l2_sliced_vbi_format *svbi;

		int i, mask = 0;

		fmt = arg;
		if (fmt->type != V4L2_BUF_TYPE_SLICED_VBI_CAPTURE)
			return -EINVAL;

		svbi = &fmt->fmt.sliced;
		memset(svbi, 0, sizeof(*svbi));

		for (i = 0; i <= 23; i++) {
			svbi->service_lines[0][i] =
			    tvp5146_get_vbi(c, vbi_supported_services, i, 1);

			svbi->service_lines[1][i] =
			    tvp5146_get_vbi(c, vbi_supported_services, i, 2);

			mask |= svbi->service_lines[0][i];
			mask |= svbi->service_lines[1][i];
		}

		svbi->service_set = mask;

		break;
	}

	/* This will not work for USB devices */
	case VIDIOC_INT_G_VBI_DATA: {
			// fixme
		struct v4l2_sliced_vbi_data *data =
		    (struct v4l2_sliced_vbi_data *) arg;

		if (data->id & V4L2_SLICED_CAPTION) {
			/*if (!field && (vdp_status&0x10)) {
			   data->data[0]=tvp5146_read(c, TVP5150_CC_DATA_INI);
			   data->data[1]=tvp5146_read(c, TVP5150_CC_DATA_INI+1);
			   } if (field && (vdp_status&0x8)) {
			   data->data[0]=tvp5146_read(c, TVP5150_CC_DATA_INI+2);
			   data->data[1]=tvp5146_read(c, TVP5150_CC_DATA_INI+3);
			   } else data->id=0;
			   return 0; */
		} else if (data->id & V4L2_SLICED_WSS) {
			if ( tvp5146_read(c, TVP5146_INT_RAW_STATUS0) & 0x20 ) {
				data->field = 0;
				data->data[0] = tvp5146_vbusread(c, 0x800520);
				data->data[1] = tvp5146_vbusread(c, 0x800521);
				data->data[2] = tvp5146_vbusread(c, 0x800522);
				tvp5146_write(c, TVP5146_INT_CLEAR0,  0x20);
			} else {
				data->id = 0;
//				printk(KERN_ERR "wss is not available\n");
			}
		} else if (data->id & V4L2_SLICED_VPS) {
			printk(KERN_ERR "bad vbi1\n");
			data->id = 0;
		} else {
			printk(KERN_ERR "bad vbi2\n");
			data->id = 0;
		}
		break;
	}

#if 0
	case VIDIOC_INT_DECODE_VBI_LINE: {
		struct v4l2_decode_vbi_line *vbi = arg;
		u8 status;

		status = tvp5146_read(c, TVP5150_VDP_STATUS_REG);

		if (status & 0x80) {
			tvp5146_err("Full field error");
			status &= 0x7f;
			tvp5146_write(c,
				      TVP5150_VDP_STATUS_REG,
				      status);
		}

		/* FIFO */
		/* Current V4L2 API allows sliced VBI only with fifo mode,
		   since line and types are not provided on other means
		   on tvp5146.
		 */
		if (!(status & 0x40))	/* Has FIFO data */
			decode_vbi_data(c, vbi);

		break;
	}
#endif

#ifdef CONFIG_VIDEO_ADV_DEBUG
	case VIDIOC_INT_G_REGISTER: {
		struct v4l2_register *reg = arg;

		if (reg->i2c_id != I2C_DRIVERID_TVP5146)
			return -EINVAL;
		reg->val = tvp5146_read(c, reg->reg & 0xff);
		break;
	}

	case VIDIOC_INT_S_REGISTER: {
		struct v4l2_register *reg = arg;

		if (reg->i2c_id != I2C_DRIVERID_TVP5146)
			return -EINVAL;
		if (!capable(CAP_SYS_ADMIN))
			return -EPERM;
		tvp5146_write(c, reg->reg & 0xff, reg->val & 0xff);
		break;
	}
#endif

	case VIDIOC_LOG_STATUS:
	case DECODER_DUMP:
		dump_reg(c);
		break;

	case VIDIOC_QUERYSTD:
	case DECODER_GET_STANDARD: {
		v4l2_std_id *std = (v4l2_std_id *) arg;
		*std = get_current_standard(c);
		return 0;
	}

	case DECODER_GET_STATUS: {
		__u32 *status = arg;
		tvp5146_get_status( c, status );
		return 0;
	}

	case DECODER_SET_GPIO: {
		int *iarg = arg;
		tvp5146_selgpcl(c, *iarg);
		break;
	}

	case DECODER_SET_VBI_BYPASS:
		break;
	
	case DECODER_SET_INPUT: {
		int *iarg = arg;
		if (*iarg < 0 || *iarg > 5) {
			return -EINVAL;
		}
		printk("DECODER_SET_INPUT\n");
		decoder->input = *iarg;
		tvp5146_selmux(c, decoder->input);

		break;
	}
	
	case DECODER_SET_OUTPUT: {
		int *iarg = arg;

		/* not much choice of outputs */
		if (*iarg != 0) {
			return -EINVAL;
		}
		break;
	}

	default:
		return -EINVAL;
	}

	return 0;
}

/****************************************************************************
			I2C Client & Driver
 ****************************************************************************/
static struct i2c_driver driver;
static struct i2c_client* the_client;


#ifdef CONFIG_MACH_ARCHOS_G6
void tvp_gio_complente (struct i2c_client *c, int output)
{
	unsigned char value;
	if ( c == 0 )
		c = the_client;

	value = (unsigned char) tvp5146_read(c, TVP5146_OUTPUT_FORMATER6);

printk(KERN_INFO "[[TVP]] gpio read status: %08x\n",value);
	
	value &= ~TVP5146_GPIO_C6_OUTPUT;

	if (output) {
		value |= TVP5146_GPIO_C6_MASK;
	} else {
		value &= (unsigned char) ~TVP5146_GPIO_C6_MASK;
	}

printk(KERN_INFO "[[TVP]] gpio write status: %08x\n",value);

	tvp5146_write(c, TVP5146_OUTPUT_FORMATER6, value);
}

void tvp_gio_rgb (struct i2c_client *c, int output)
{
	unsigned char value;
	if ( c == 0 )
		c = the_client;

	value = (unsigned char) tvp5146_read(c, TVP5146_OUTPUT_FORMATER6);

printk(KERN_INFO "[[TVP]] gpio read status: %08x\n",value);
	
	value &= ~TVP5146_GPIO_C7_OUTPUT;

	if (output) {
		value |= TVP5146_GPIO_C7_MASK;
	} else {
		value &= (unsigned char) ~TVP5146_GPIO_C7_MASK;
	}

printk(KERN_INFO "[[TVP]] gpio write status: %08x\n",value);

	tvp5146_write(c, TVP5146_OUTPUT_FORMATER6, value);
}
#endif

static int tvp5146_detect_client(struct i2c_adapter *adapter,
				 int address, int kind)
{
	struct i2c_client *c;
	struct tvp5146 *core;
	int rv;

	// we can handle just one client
	if (the_client != NULL)
		return -EBUSY;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(adapter,
	     I2C_FUNC_SMBUS_READ_BYTE | I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
		return 0;

	c = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (c == NULL)
		return -ENOMEM;

	c->adapter = adapter;
	c->addr = address;
	c->driver = &driver;
	snprintf(c->name, sizeof(c->name) - 1, "tvp5146");

	core = kzalloc(sizeof(struct tvp5146), GFP_KERNEL);
	if (core == 0) {
		kfree(c);
		return -ENOMEM;
	}
	i2c_set_clientdata(c, core);

	rv = i2c_attach_client(c);
	if (rv) {
		kfree(c);
		kfree(core);
		return rv;
	}
	
	if (tvp5146_detect_chip(c) < 0)
		return 0;

	the_client = c;

	core->norm = V4L2_STD_ALL;	/* Default is autodetect */
	core->input = TVP5146_ANALOG_CH0;
	core->enable = 1;
	core->bright = 32768;
	core->contrast = 32768;
	core->hue = 32768;
	core->sat = 32768;
	core->sat_pr = 0x80;
	core->sat_pb = 0x80;
	core->bgain = 0x900;
	core->rgain = 0x900;
	core->ggain = 0x900;

	/* reset the chip */
	tvp5146_reset(c);
	if (debug > 1)
		dump_reg(c);

	return 0;
}

static int tvp5146_attach_adapter(struct i2c_adapter *adapter)
{
	int err;
	
#ifdef CONFIG_MACH_ARCHOS_G6
	/* only attach on adapter nr 3 (cradle bus) */
	if ( adapter->nr != _get_i2c_nb() )
		return -ENODEV;
#endif

	if ((err = i2c_probe(adapter, &addr_data, tvp5146_detect_client))) {
		printk(KERN_INFO "tvp i2c : Adapter attach failed.\n");
		return err;
	}
	return 0;

}

static int tvp5146_detach_client(struct i2c_client *c)
{
	int err;
	struct tvp5146 *decoder = i2c_get_clientdata(c);

	tvp5146_dbg(1, "tvp5146.c: removing adapter at 0x%x\n", c->addr << 1);

	err = i2c_detach_client(c);
	if (err)
		return err;

	kfree(decoder);
	kfree(c);

	return 0;
}

static struct i2c_driver driver = {
	.driver = {
		   .name = "tvp5146",
	},
	.id = I2C_DRIVERID_TVP5146,
	.attach_adapter = tvp5146_attach_adapter,
	.detach_client = tvp5146_detach_client,
};


#ifdef CONFIG_ARCH_OMAP34XX
/****************************************************************************
			OMAP3 video decoder interface
 ****************************************************************************/
static void* decoder_init(struct v4l2_pix_format *pfm)
{
	*pfm = (struct v4l2_pix_format){
		.width = 640,
		.height = 480,
		.pixelformat = V4L2_PIX_FMT_YUYV,
		.field = V4L2_FIELD_INTERLACED,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.bytesperline = 640*2,
		.sizeimage = 640*480*2,
	};

	return the_client;
}

static int decoder_cleanup(void* self)
{
	return 0;
}

static int decoder_try_format(struct v4l2_pix_format *pfm, void *self)
{
	if (pfm->width > 720)
		pfm->width = 720;
	if (pfm->height > 576)
		pfm->height = 576;
		
	pfm->pixelformat = V4L2_PIX_FMT_YUYV;
	pfm->field = V4L2_FIELD_INTERLACED;
	pfm->bytesperline = pfm->width * 2;
	pfm->sizeimage = pfm->height * pfm->bytesperline;
printk("decoder_try_format: pfm->height %d  pfm->bytesperline %d\r\n", pfm->height, pfm->bytesperline ); 	
	return 0;
}

static int decoder_configure(struct v4l2_pix_format *pfm, 
		struct v4l2_fract *tpf, void *self)
{
	struct i2c_client* c = self;
	tvp5146_dbg(1, "decoder_configure called\n");

	return 0;
}

static int decoder_stream_start(void *self)
{
	struct i2c_client* c = self;
	struct tvp5146* decoder = i2c_get_clientdata(c);
		
	decoder->enable = 1;
	tvp5146_write(c, TVP5146_OP_MODE_CTL, 0x00);
	tvp5146_selmux(c, decoder->input);
	return 0;
}

static int decoder_stream_stop(void *self)
{
	struct i2c_client* c = self;
	struct tvp5146* decoder = i2c_get_clientdata(c);
	
	decoder->enable = 0;
	tvp5146_write(c, TVP5146_OP_MODE_CTL, 0x01);
	return 0;
}

static int decoder_query_control(struct v4l2_queryctrl * qc, void *self)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tvp5146_qctrl); i++) {
		if (qc->id && qc->id == tvp5146_qctrl[i].id) {
			memcpy(qc, &(tvp5146_qctrl[i]), sizeof(*qc));
			return 0;
		}
	}

	return -EINVAL;
}

static int decoder_set_control(struct v4l2_control *ctrl, void *self)
{
	struct i2c_client *c = self;
	struct tvp5146* decoder = i2c_get_clientdata(c);
	
	int i, n;
		
	/* n: number of controls we support */
	n = sizeof(tvp5146_qctrl) / sizeof(tvp5146_qctrl[0]);

	for (i = 0; i < n; i++) {
		if (ctrl->id == tvp5146_qctrl[i].id) {
			if (   ctrl->value < tvp5146_qctrl[i].minimum
			    	|| ctrl->value >tvp5146_qctrl[i].maximum) 
			{
				tvp5146_dbg(1, "VIDIOC_S_CTRL out of range: id=%d, value=%d\n",
				    ctrl->id, ctrl->value);
				return -ERANGE;
			}
		}
	}
	switch (decoder->input) {
		case TVP5146_ANALOG_CH1:
		case TVP5146_ANALOG_CH0:
		case TVP5146_SVIDEO:
			tvp5146_dbg(1, "VIDIOC_S_CTRL: id=%d, value=%d\n",
				    ctrl->id, ctrl->value);
			return tvp5146_set_ctrl(c, ctrl);

		case TVP5146_YPbPr:
			tvp5146_dbg(1,  "decoder_set_control component: id=%d, value=%d\n",
				    ctrl->id, ctrl->value);
			return tvp5146_set_ctrl_component(c, ctrl);

		case TVP5146_RGB:
		case TVP5146_SCART:
			tvp5146_dbg(1, "decoder_set_control rgb: id=%d, value=%d\n",
				    ctrl->id, ctrl->value);
			return tvp5146_set_ctrl_rgb(c, ctrl);
	}

	return -EINVAL;
}

static int decoder_get_control(struct v4l2_control *ctl, void *self)
{
	struct i2c_client *c = self;
	return tvp5146_get_ctrl(c, ctl);
}

static int decoder_set_standard(v4l2_std_id *std, void *self) 
{
	struct i2c_client* c = self;
	struct tvp5146* decoder = i2c_get_clientdata(c);

	if (decoder->norm == *std)
		return 0;
	return tvp5146_set_std(c, *std);

}

static int decoder_get_standard(v4l2_std_id *std, void *self)
{
	struct i2c_client* c = self;
	struct tvp5146* decoder = i2c_get_clientdata(c);

	*std = decoder->norm;
	return 0;
}

static int decoder_enum_input(struct v4l2_input *input, void *self)
{
	struct i2c_client* c = self;

	switch (input->index) {
	default:
		return -EINVAL;
	
	case TVP5146_ANALOG_CH0:
		strlcpy(input->name, "CVBS0", sizeof(input->name));
		break;

	case TVP5146_ANALOG_CH1:
		strlcpy(input->name, "CVBS1", sizeof(input->name));
		break;
		
	case TVP5146_SVIDEO:
		strlcpy(input->name, "SVIDEO", sizeof(input->name));
		break;
		
	case TVP5146_SCART:
		strlcpy(input->name, "SCART", sizeof(input->name));
		break;
		
	case TVP5146_YPbPr:
		strlcpy(input->name, "YPbPr", sizeof(input->name));
		break;
		
	case TVP5146_RGB:
		strlcpy(input->name, "RGB", sizeof(input->name));
		break;
		
	}
	
	input->type = V4L2_INPUT_TYPE_TUNER;

	tvp5146_get_status( c, &input->status );

	return 0;
}

static int decoder_get_input(unsigned int *index, void *self)
{
	return -EINVAL;
}

static int decoder_set_input(unsigned int *index, void *self)
{
	struct i2c_client* c = self;
	struct tvp5146* decoder = i2c_get_clientdata(c);

	decoder->input = *index;
	tvp5146_selmux(c, decoder->input);
	return 0;
}

static int decoder_ioctl(unsigned int cmd, void* arg, void* self)
{
	struct i2c_client* c = self;
	return tvp5146_command(c, cmd, arg);
}

static struct video_decoder vdec = {
	.version=	1,
	.name=		"tvp5146",
	.decoder_interface =	SENSOR_PARALLEL,
	
	.init=		decoder_init,
	.cleanup= 	decoder_cleanup,
	.configure=	decoder_configure,
	.try_format=	decoder_try_format,

	.stream_start=	decoder_stream_start,
	.stream_stop=	decoder_stream_stop,
	
	.query_control=	decoder_query_control,
	.set_control=	decoder_set_control,
	.get_control=	decoder_get_control,
	
	.set_standard=	decoder_set_standard,
	.get_standard=	decoder_get_standard,
	
	.enum_input=	decoder_enum_input,
	.get_input=	decoder_get_input,
	.set_input=	decoder_set_input,
	
	.ioctl= 	decoder_ioctl,
};

#endif

/* ----------------------------------------------------------------------- */

static int __init tvp5146_init(void)
{
	int ret;
#if defined (CONFIG_MACH_ARCHOS_G6)
	_select_tvp();
#endif

	ret = i2c_add_driver(&driver);
#ifdef CONFIG_ARCH_OMAP34XX
	if ( !ret) {
		if ( (ret = omap_dvr_register_decoder(&vdec)) ) {
			i2c_del_driver(&driver);
		}
	}
#endif
	return ret;
}

static void __exit tvp5146_exit(void)
{
#ifdef CONFIG_ARCH_OMAP34XX
	omap_dvr_unregister_decoder(&vdec);
#endif
	i2c_del_driver(&driver);
}

module_init(tvp5146_init);
module_exit(tvp5146_exit);

EXPORT_SYMBOL(tvp_gio_complente);
EXPORT_SYMBOL(tvp_gio_rgb);
