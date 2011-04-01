/*
 * arch/arm/mach-omap2/display.c
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * Leveraged code from the OMAP24xx camera driver
 * Video-for-Linux (Version 2) camera capture driver for
 * the OMAP24xx camera controller.
 *
 * Author: Andy Lowe (source@mvista.com)
 * Copyright (C) 2004 MontaVista Software, Inc.
 *
 * History:
 * 20-APR-2006  Khasim		Modified VRFB based Rotation equations,
 *				The image data is always read from 0 degree 
 *				view and written to the virtual space of desired 
 *				rotation angle
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/arch/hardware.h>
#include <asm/arch/display.h>
#include <asm/arch/venc.h>
#include <asm/arch/clock.h>
#ifdef CONFIG_TRACK_RESOURCES
#include <linux/device.h>
#endif

#ifdef CONFIG_ARCH_OMAP34XX
extern int lpr_enabled;
#endif

#ifdef CONFIG_MACH_ARCHOS_G6
extern int get_panel_is_interlaced(void);
#endif

#define USE_FIVE_TAP_VERTICAL_UPSCALE
#define DISPC_VID_ATTRIBUTES_VIDLINEBUFFERSPLIT	(1 << 22)

/* usage count for DSS power management */
static int disp_usage;
#ifndef CONFIG_ARCH_OMAP3410
static int omap2_current_tvstandard = NTSC_M;
#endif
static spinlock_t dss_lock;
short int current_colorconv_values[2][3][3];
static struct omap_dss_regs dss_ctx;
#ifndef CONFIG_OMAP_USE_DSI_PLL
static struct clk *dss1f_scale;
#endif
static struct tvlcd_status_t tvlcd_status;

#ifndef CONFIG_OMAP_USE_DSI_PLL
static u32 dss_clk_rate = 26000000 * 4 /* default init value for lcd */;
#endif
static u32 dsi_clk_rate = 27000000 /* default init value for lcd */;
static struct clk *dss1f, *dss1i;
#if defined (CONFIG_OMAP_USE_DSI_PLL)
static struct clk *dss2f;
#endif

struct omap2_disp_dma_params {
	u32 ba0;
	u32 ba1;
	int row_inc;
	int pix_inc;
	int accu0;
	int accu1;
};

static struct layer_t {
	int output_dev;
	int in_use;
	int ctx_valid;

	/* one set of dma parameters each for LCD and TV */
	struct omap2_disp_dma_params dma[2];

	int size_x;
	int size_y;
	int field_offset;
	
} layer[DSS_CTX_NUMBER] = {
	{.ctx_valid = 0,},
	{.ctx_valid = 0,},
	{.ctx_valid = 0,},
	{.ctx_valid = 0,},
	{.ctx_valid = 0,},
};

#define MAX_ISR_NR   8
static int omap2_disp_irq;
static struct {
	omap2_disp_isr_t isr;
	void *arg;
	unsigned int mask;
} registered_isr[MAX_ISR_NR];

/* VRFB offset computation parameters */
#define SIDE_H		1 
#define SIDE_W		0 


/* GFX FIFO thresholds */
#define RMODE_GFX_FIFO_HIGH_THRES	0x3FC
#define RMODE_GFX_FIFO_LOW_THRES	0x3BC

#define DBG if(0)

#ifdef CONFIG_TRACK_RESOURCES
/* device name needed for resource tracking layer */
struct device_driver display_drv = {
	.name =  "display",
};

struct device display_dev = {
	.driver = &display_drv,
};
#endif

/* ----------------------------------------------------- */

#define DEFAULT_TIMEOUT	(HZ/10)	/* 100ms */
static void wait_for_reg_sync(int output_dev, unsigned long timeout);
static int omap2_disp_check_enabled(int output_dev);


/*
 * DSS register I/O routines
 */
static __inline__ u32
dss_reg_in(u32 offset)
{
	return  omap_readl(DSS_REG_BASE + DSS_REG_OFFSET + offset);
}

static __inline__ u32
dss_reg_out(u32 offset, u32 val)
{
	omap_writel(val,DSS_REG_BASE + DSS_REG_OFFSET + offset);
	return val;
}

static __inline__ u32
dss_reg_merge(u32 offset, u32 val, u32 mask)
{
	u32 addr = DSS_REG_BASE + DSS_REG_OFFSET + offset;
	u32 new_val = (omap_readl(addr) & ~mask) | (val & mask);

	omap_writel(new_val, addr);
	return new_val;
}

/*
 * Display controller register I/O routines
 */
static __inline__ u32
dispc_reg_in(u32 offset)
{
	return omap_readl(DSS_REG_BASE + DISPC_REG_OFFSET + offset);
}

static __inline__ u32
dispc_reg_out(u32 offset, u32 val)
{
	omap_writel(val, DSS_REG_BASE + DISPC_REG_OFFSET + offset);
	return val;
}

static __inline__ u32
dispc_reg_merge(u32 offset, u32 val, u32 mask)
{
	u32 addr = DSS_REG_BASE + DISPC_REG_OFFSET + offset;
	u32 new_val = (omap_readl(addr) & ~mask) | (val & mask);

	omap_writel(new_val, addr);
	return new_val;
}

/*
 * RFBI controller register I/O routines
 */
static __inline__ u32
rfbi_reg_in(u32 offset)
{
	return omap_readl(DSS_REG_BASE + RFBI_REG_OFFSET + offset);
}

static __inline__ u32
rfbi_reg_out(u32 offset, u32 val)
{
	omap_writel(val, DSS_REG_BASE + RFBI_REG_OFFSET + offset);
	return val;
}

/*
 * VENC register I/O Routines
 */
static __inline__ u32
venc_reg_in(u32 offset)
{
	return omap_readl(DSS_REG_BASE + VENC_REG_OFFSET + offset);
}

static __inline__ u32
venc_reg_out(u32 offset, u32 val)
{
	omap_writel(val, DSS_REG_BASE + VENC_REG_OFFSET + offset);
	return val;
}

static __inline__ u32
venc_reg_merge(u32 offset, u32 val, u32 mask)
{
	u32 addr = DSS_REG_BASE + VENC_REG_OFFSET + offset;
	u32 new_val = (omap_readl(addr) & ~mask) | (val & mask);

	omap_writel(new_val, addr);
	return new_val;
}

/* Write the color space conversion coefficients to the display controller
 * registers.  Each coefficient is a signed 11-bit integer in the range
 * [-1024, 1023].  The matrix coefficients are:
 *	[ RY  RCr  RCb ]
 *	[ GY  GCr  GCb ]
 *	[ BY  BCr  BCb ]
 */

static void
set_colorconv(int v,int colorspace)
{
	unsigned long ccreg;
	short int mtx[3][3];
	int i,j;
	for(i=0;i<3;i++)
		for(j=0;j<3;j++){
			mtx[i][j] = current_colorconv_values[v][i][j];
		}

	ccreg = (mtx[0][0] & 0x7ff) | ((mtx[0][1] & 0x7ff) << 16);
	dispc_reg_out(DISPC_VID_CONV_COEF0(v), ccreg);
	ccreg = (mtx[0][2] & 0x7ff) | ((mtx[1][0] & 0x7ff) << 16);
	dispc_reg_out(DISPC_VID_CONV_COEF1(v), ccreg);
	ccreg = (mtx[1][1] & 0x7ff) | ((mtx[1][2] & 0x7ff) << 16);
	dispc_reg_out(DISPC_VID_CONV_COEF2(v), ccreg);
	ccreg = (mtx[2][0] & 0x7ff) | ((mtx[2][1] & 0x7ff) << 16);
	dispc_reg_out(DISPC_VID_CONV_COEF3(v), ccreg);
	ccreg = mtx[2][2] & 0x7ff;
	dispc_reg_out(DISPC_VID_CONV_COEF4(v), ccreg);

	if(colorspace == V4L2_COLORSPACE_JPEG || 
			colorspace == V4L2_COLORSPACE_SRGB){
		dispc_reg_merge(DISPC_VID_ATTRIBUTES(v), 
			DISPC_VID_ATTRIBUTES_VIDFULLRANGE, 
			DISPC_VID_ATTRIBUTES_VIDFULLRANGE);
	}
}

static void
update_colorconv_mtx(int v,const short int mtx[3][3])
{
	int i,j;
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			current_colorconv_values[v][i][j] = mtx[i][j];
}

void
omap2_disp_set_default_colorconv(int ltype, struct v4l2_pix_format *pix)
{
	int v;

	if (ltype == OMAP2_VIDEO1) v = 0;
	else if (ltype == OMAP2_VIDEO2) v = 1;
	else return;

	switch (pix->colorspace) {
	case V4L2_COLORSPACE_SMPTE170M:
	case V4L2_COLORSPACE_SMPTE240M:
	case V4L2_COLORSPACE_BT878:
	case V4L2_COLORSPACE_470_SYSTEM_M:
	case V4L2_COLORSPACE_470_SYSTEM_BG:
		/* luma (Y) range lower limit is 16, BT.601 standard */
		update_colorconv_mtx(v,cc_bt601);
		set_colorconv(v,pix->colorspace);
		break;
	case V4L2_COLORSPACE_REC709:
		/* luma (Y) range lower limit is 16, BT.709 standard */
		update_colorconv_mtx(v,cc_bt709);
		set_colorconv(v,pix->colorspace);
		break;
	case V4L2_COLORSPACE_JPEG:
	case V4L2_COLORSPACE_SRGB:
		/* full luma (Y) range, assume BT.601 standard */
		update_colorconv_mtx(v,cc_bt601_full);
		set_colorconv(v,pix->colorspace);
		break;
	}
}

void
omap2_disp_set_colorconv(int ltype, struct v4l2_pix_format *pix)
{
	int v;

	if (ltype == OMAP2_VIDEO1) v = 0;
	else if (ltype == OMAP2_VIDEO2) v = 1;
	else return;

	set_colorconv(v,pix->colorspace);
}

/* Write the horizontal and vertical resizing coefficients to the display
 * controller registers.  Each coefficient is a signed 8-bit integer in the
 * range [-128, 127] except for the middle coefficient (vc[1][i] and hc[3][i])
 * which is an unsigned 8-bit integer in the range [0, 255].  The first index of
 * the matrix is the coefficient number (0 to 2 vertical or 0 to 4 horizontal)
 * and the second index is the phase (0 to 7).
 */
static void
omap2_disp_set_resize(int v, short int *vc, short int *hc, int five_tap)
{
	int i;
	unsigned long reg;

	for (i = 0; i < 8; i++) {
		/* HC0 | (HC1 << 8) | (HC2 << 16) | (HC3 << 24) */
		reg =      (*(hc+(8*0)+i) & 0xff)        | ((*(hc+(8*1)+i) & 0xff) <<  8)
			| ((*(hc+(8*2)+i) & 0xff) << 16) | ((*(hc+(8*3)+i) & 0xff) << 24);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v, i), reg);
		
		if( five_tap ) {
			/* HC4 | (VC0 << 8) | (VC1 << 16) | (VC2 << 24) */
			reg =      (*(hc+(8*4)+i) & 0xff)        | ((*(vc+(8*0)+i) & 0xff) <<  8)
				| ((*(vc+(8*1)+i) & 0xff) << 16) | ((*(vc+(8*2)+i) & 0xff) << 24);			
			dispc_reg_out(DISPC_VID_FIR_COEF_HV(v, i), reg);

			/* VC00 | (VC22 << 8) */
			reg =      (*(vc+(8*3)+i) & 0xff)        | ((*(vc+(8*4)+i) & 0xff) <<  8);
			dispc_reg_out(DISPC_VID_FIR_COEF_V(v,i),reg);
		} else {
			/* HC4 | (VC0 << 8) | (VC1 << 16) | (VC2 << 24) */
			reg =      (*(hc+(8*4)+i) & 0xff)        | ((*(vc+(8*0)+i) & 0xff) <<  8)
				| ((*(vc+(8*1)+i) & 0xff) << 16) | ((*(vc+(8*2)+i) & 0xff) << 24);
			dispc_reg_out(DISPC_VID_FIR_COEF_HV(v, i), reg);
		}
	}
}

static int _lcd_interlaced( void ) 
{
	return get_panel_is_interlaced();
}

/*---------------------------------------------------------------------------*/
void omap2_disp_get_panel_size(int output_dev, int *width, int *height)
{
	unsigned long size;

	if (output_dev == OMAP2_OUTPUT_TV) {
		size = dispc_reg_in(DISPC_SIZE_DIG);
		*width  = 1 + ((size & DISPC_SIZE_DIG_PPL) >> DISPC_SIZE_DIG_PPL_SHIFT);
		*height = 1 + ((size & DISPC_SIZE_DIG_LPP) >> DISPC_SIZE_DIG_LPP_SHIFT);
		*height = *height << 1;
	} else if (output_dev == OMAP2_OUTPUT_LCD) {
		size = dispc_reg_in(DISPC_SIZE_LCD);
		*width  = 1 + ((size & DISPC_SIZE_LCD_PPL) >> DISPC_SIZE_LCD_PPL_SHIFT);
		*height = 1 + ((size & DISPC_SIZE_LCD_LPP) >> DISPC_SIZE_LCD_LPP_SHIFT);
		if( _lcd_interlaced() ) {
			*height = *height << 1;
		}
	} 
}

void omap2_disp_set_panel_size(int output_dev, int width, int height)
{
	unsigned long size;
DBG printk("omap2_disp_set_panel_size: dev %d  WxH %d x %d\r\n", output_dev, width, height );   
	if (output_dev == OMAP2_OUTPUT_TV) {
		height = height >> 1;
DBG printk("omap2_disp_set_panel_size: TV %d x %d\r\n", width, height);   
		size  = ((width  - 1) << DISPC_SIZE_DIG_PPL_SHIFT) &  DISPC_SIZE_DIG_PPL;
		size |= ((height - 1) << DISPC_SIZE_DIG_LPP_SHIFT) & DISPC_SIZE_DIG_LPP;
		dispc_reg_out(DISPC_SIZE_DIG, size);
	} else if (output_dev == OMAP2_OUTPUT_LCD) {
		if( _lcd_interlaced() ) {
			height = height >> 1;
		}
DBG printk("omap2_disp_set_panel_size: LCD %d %d\r\n", width, height);   
		size  = ((width  - 1) << DISPC_SIZE_LCD_PPL_SHIFT) & DISPC_SIZE_LCD_PPL;
		size |= ((height - 1) << DISPC_SIZE_LCD_LPP_SHIFT) & DISPC_SIZE_LCD_LPP;
#ifndef CONFIG_OMAP_DVI_SUPPORT
		dispc_reg_out(DISPC_SIZE_LCD, size);
#else
		dispc_reg_out(DISPC_SIZE_LCD, 0x02CF04FF);
#endif
	}
}

static int graphics_in_use;

/* Turn off the GFX, or video1, or video2 layer. */
void
omap2_disp_disable_layer(int ltype)
{
	unsigned long attributes;
	int digital, v;

	if (ltype == OMAP2_GRAPHICS) {
		attributes = dispc_reg_merge(DISPC_GFX_ATTRIBUTES, 0, DISPC_GFX_ATTRIBUTES_ENABLE);
		digital = attributes & DISPC_GFX_ATTRIBUTES_GFXCHANNELOUT;
		graphics_in_use = 0;
	} else {
		if (ltype == OMAP2_VIDEO1) 
			v = 0;
		else if (ltype == OMAP2_VIDEO2) 
			v = 1;
		else 
			return;

		attributes = dispc_reg_merge(DISPC_VID_ATTRIBUTES(v), 0, DISPC_VID_ATTRIBUTES_ENABLE);
		digital = attributes & DISPC_VID_ATTRIBUTES_VIDCHANNELOUT;
	}
	if (digital) {
		/* digital output */
		omap2_disp_reg_sync( OMAP2_OUTPUT_TV );
		if ( omap2_disp_check_enabled( OMAP2_OUTPUT_TV ))
			wait_for_reg_sync( OMAP2_OUTPUT_TV, DEFAULT_TIMEOUT );
	} else {
		/* LCD */
		omap2_disp_reg_sync( OMAP2_OUTPUT_LCD );
		if ( omap2_disp_check_enabled( OMAP2_OUTPUT_LCD ))
			wait_for_reg_sync( OMAP2_OUTPUT_LCD, DEFAULT_TIMEOUT );
	}
	
	dispc_reg_merge(DISPC_CONTROL, 0,
			 DISPC_CONTROL_OVERLAYOPTIMIZATION);
}

/* Turn on the GFX, or video1, or video2 layer. */
void
omap2_disp_enable_layer(int ltype)
{
	unsigned long attributes;
	int digital, v;

	if (ltype == OMAP2_GRAPHICS) {
		attributes = dispc_reg_merge(DISPC_GFX_ATTRIBUTES, DISPC_GFX_ATTRIBUTES_ENABLE, DISPC_GFX_ATTRIBUTES_ENABLE);
		digital = attributes & DISPC_GFX_ATTRIBUTES_GFXCHANNELOUT;
		graphics_in_use = 1;
	} else {
		if (ltype == OMAP2_VIDEO1) 
			v = 0;
		else if (ltype == OMAP2_VIDEO2) 
			v = 1;
		else 
			return;

		attributes = dispc_reg_merge(DISPC_VID_ATTRIBUTES(v), DISPC_VID_ATTRIBUTES_ENABLE, DISPC_VID_ATTRIBUTES_ENABLE);
		digital = attributes & DISPC_VID_ATTRIBUTES_VIDCHANNELOUT;
	}
	if (digital) {
		/* digital output */
		omap2_disp_reg_sync( OMAP2_OUTPUT_TV );
		if ( omap2_disp_check_enabled( OMAP2_OUTPUT_TV ))
			wait_for_reg_sync( OMAP2_OUTPUT_TV, DEFAULT_TIMEOUT );
	} else {
		/* LCD */
		omap2_disp_reg_sync( OMAP2_OUTPUT_LCD );
		if ( omap2_disp_check_enabled( OMAP2_OUTPUT_LCD ))
			wait_for_reg_sync( OMAP2_OUTPUT_LCD, DEFAULT_TIMEOUT );
	}
}

/*
 * Save the DSS state before doing a GO LCD/DIGITAL
 */

void
omap2_disp_save_ctx(int ltype)
{
	int v1=0, v2=1;
	struct omap24xx_dispc_regs *dispc = &dss_ctx.dispc;

	switch(ltype){
	case OMAP_DSS_GENERIC:
		dss_ctx.sysconfig = dss_reg_in(DSS_SYSCONFIG);
		dss_ctx.control   = dss_reg_in(DSS_CONTROL) & ~(DISPC_CONTROL_GODIGITAL | DISPC_CONTROL_GOLCD);
#ifdef CONFIG_ARCH_OMAP3430
		dss_ctx.sdi_control = dss_reg_in(DSS_SDI_CONTROL);
		dss_ctx.pll_control = dss_reg_in(DSS_PLL_CONTROL);
#endif
		break;

	case OMAP_DSS_DISPC_GENERIC:
		dispc->revision  = dispc_reg_in(DISPC_REVISION);
		dispc->sysconfig = dispc_reg_in(DISPC_SYSCONFIG);
		dispc->sysstatus = dispc_reg_in(DISPC_SYSSTATUS);
		dispc->irqstatus = dispc_reg_in(DISPC_IRQSTATUS);
		dispc->irqenable = dispc_reg_in(DISPC_IRQENABLE);
		dispc->control   = dispc_reg_in(DISPC_CONTROL);
		dispc->config    = dispc_reg_in(DISPC_CONFIG);
		dispc->capable   = dispc_reg_in(DISPC_CAPABLE);
		dispc->default_color0 = dispc_reg_in(DISPC_DEFAULT_COLOR0);
		dispc->default_color1 = dispc_reg_in(DISPC_DEFAULT_COLOR1);
		dispc->trans_color0   = dispc_reg_in(DISPC_TRANS_COLOR0);
		dispc->trans_color1   = dispc_reg_in(DISPC_TRANS_COLOR1);
		dispc->line_status    = dispc_reg_in(DISPC_LINE_STATUS);
		dispc->line_number    = dispc_reg_in(DISPC_LINE_NUMBER);
		dispc->data_cycle1    = dispc_reg_in(DISPC_DATA_CYCLE1);
		dispc->data_cycle2    = dispc_reg_in(DISPC_DATA_CYCLE2);
		dispc->data_cycle3    = dispc_reg_in(DISPC_DATA_CYCLE3);
		dispc->timing_h = dispc_reg_in(DISPC_TIMING_H);
		dispc->timing_v = dispc_reg_in(DISPC_TIMING_V);
		dispc->pol_freq = dispc_reg_in(DISPC_POL_FREQ);
		dispc->divisor  = dispc_reg_in(DISPC_DIVISOR);
		dispc->global_alpha = dispc_reg_in(DISPC_GLOBAL_ALPHA);
		dispc->size_lcd = dispc_reg_in(DISPC_SIZE_LCD);
		dispc->size_dig = dispc_reg_in(DISPC_SIZE_DIG);
		break;

	case OMAP2_GRAPHICS:
		dispc->gfx_ba0  = dispc_reg_in(DISPC_GFX_BA0);
		dispc->gfx_ba1  = dispc_reg_in(DISPC_GFX_BA1);
		dispc->gfx_position   = dispc_reg_in(DISPC_GFX_POSITION);
		dispc->gfx_size       = dispc_reg_in(DISPC_GFX_SIZE);
		dispc->gfx_attributes = dispc_reg_in(DISPC_GFX_ATTRIBUTES);
		dispc->gfx_fifo_size  = dispc_reg_in(DISPC_GFX_FIFO_SIZE);
		dispc->gfx_fifo_threshold = dispc_reg_in(DISPC_GFX_FIFO_THRESHOLD);
		dispc->gfx_row_inc        = dispc_reg_in(DISPC_GFX_ROW_INC);
		dispc->gfx_pixel_inc      = dispc_reg_in(DISPC_GFX_PIXEL_INC);
		dispc->gfx_window_skip    = dispc_reg_in(DISPC_GFX_WINDOW_SKIP);
		dispc->gfx_table_ba       = dispc_reg_in(DISPC_GFX_TABLE_BA);
		break;

	case OMAP2_VIDEO1:
		dispc->vid1_ba0 = dispc_reg_in(DISPC_VID_BA0(v1));
		dispc->vid1_ba1 = dispc_reg_in(DISPC_VID_BA0(v1));
		dispc->vid1_position = dispc_reg_in(DISPC_VID_POSITION(v1));
		dispc->vid1_size = dispc_reg_in(DISPC_VID_SIZE(v1));
		dispc->vid1_attributes = dispc_reg_in(DISPC_VID_ATTRIBUTES(v1));
		dispc->vid1_fifo_size  = dispc_reg_in(DISPC_VID_FIFO_SIZE(v1));
		dispc->vid1_fifo_threshold = dispc_reg_in(DISPC_VID_FIFO_THRESHOLD(v1));
		dispc->vid1_row_inc   = dispc_reg_in(DISPC_VID_ROW_INC(v1));
		dispc->vid1_pixel_inc = dispc_reg_in(DISPC_VID_PIXEL_INC(v1));
		dispc->vid1_fir 	     = dispc_reg_in(DISPC_VID_FIR(v1));
		dispc->vid1_accu0 = dispc_reg_in(DISPC_VID_ACCU0(v1));
		dispc->vid1_accu1 = dispc_reg_in(DISPC_VID_ACCU1(v1));
		dispc->vid1_picture_size = dispc_reg_in(DISPC_VID_PICTURE_SIZE(v1));
		dispc->vid1_fir_coef_h0  = dispc_reg_in(DISPC_VID_FIR_COEF_H(v1,0));
		dispc->vid1_fir_coef_h1  = dispc_reg_in(DISPC_VID_FIR_COEF_H(v1,1));
		dispc->vid1_fir_coef_h2  = dispc_reg_in(DISPC_VID_FIR_COEF_H(v1,2));
		dispc->vid1_fir_coef_h3  = dispc_reg_in(DISPC_VID_FIR_COEF_H(v1,3));
		dispc->vid1_fir_coef_h4  = dispc_reg_in(DISPC_VID_FIR_COEF_H(v1,4));
		dispc->vid1_fir_coef_h5  = dispc_reg_in(DISPC_VID_FIR_COEF_H(v1,5));
		dispc->vid1_fir_coef_h6  = dispc_reg_in(DISPC_VID_FIR_COEF_H(v1,6));
		dispc->vid1_fir_coef_h7  = dispc_reg_in(DISPC_VID_FIR_COEF_H(v1,7));
		dispc->vid1_fir_coef_hv0 = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1,0));
		dispc->vid1_fir_coef_hv1 = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1,1));
		dispc->vid1_fir_coef_hv2 = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1,2));
		dispc->vid1_fir_coef_hv3 = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1,3));
		dispc->vid1_fir_coef_hv4 = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1,4));
		dispc->vid1_fir_coef_hv5 = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1,5));
		dispc->vid1_fir_coef_hv6 = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1,6));
		dispc->vid1_fir_coef_hv7 = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1,7));
		dispc->vid1_conv_coef0   = dispc_reg_in(DISPC_VID_CONV_COEF0(v1));
		dispc->vid1_conv_coef1   = dispc_reg_in(DISPC_VID_CONV_COEF1(v1));
		dispc->vid1_conv_coef2   = dispc_reg_in(DISPC_VID_CONV_COEF2(v1));
		dispc->vid1_conv_coef3   = dispc_reg_in(DISPC_VID_CONV_COEF3(v1));
		dispc->vid1_conv_coef4   = dispc_reg_in(DISPC_VID_CONV_COEF4(v1));
#ifdef CONFIG_ARCH_OMAP3430
		dispc->vid1_fir_coef_v0   = dispc_reg_in(DISPC_VID_FIR_COEF_V(v1,0));
		dispc->vid1_fir_coef_v1   = dispc_reg_in(DISPC_VID_FIR_COEF_V(v1,1));
		dispc->vid1_fir_coef_v2   = dispc_reg_in(DISPC_VID_FIR_COEF_V(v1,2));
		dispc->vid1_fir_coef_v3   = dispc_reg_in(DISPC_VID_FIR_COEF_V(v1,3));
		dispc->vid1_fir_coef_v4   = dispc_reg_in(DISPC_VID_FIR_COEF_V(v1,4));
		dispc->vid1_fir_coef_v5   = dispc_reg_in(DISPC_VID_FIR_COEF_V(v1,5));
		dispc->vid1_fir_coef_v6   = dispc_reg_in(DISPC_VID_FIR_COEF_V(v1,6));
		dispc->vid1_fir_coef_v7   = dispc_reg_in(DISPC_VID_FIR_COEF_V(v1,7));
#endif
		break;

	case OMAP2_VIDEO2:
		dispc->vid2_ba0 = dispc_reg_in(DISPC_VID_BA0(v2));
		dispc->vid2_ba1 = dispc_reg_in(DISPC_VID_BA1(v2));
		dispc->vid2_position = dispc_reg_in(DISPC_VID_POSITION(v2));
		dispc->vid2_size = dispc_reg_in(DISPC_VID_SIZE(v2));
		dispc->vid2_attributes = dispc_reg_in(DISPC_VID_ATTRIBUTES(v2));
		dispc->vid2_fifo_size  = dispc_reg_in(DISPC_VID_FIFO_SIZE(v2));
		dispc->vid2_fifo_threshold = dispc_reg_in(DISPC_VID_FIFO_THRESHOLD(v2));
		dispc->vid2_row_inc   = dispc_reg_in(DISPC_VID_ROW_INC(v2));
		dispc->vid2_pixel_inc = dispc_reg_in(DISPC_VID_PIXEL_INC(v2));
		dispc->vid2_fir = dispc_reg_in(DISPC_VID_FIR(v2));
		dispc->vid2_accu0 = dispc_reg_in(DISPC_VID_ACCU0(v2));
		dispc->vid2_accu1 = dispc_reg_in(DISPC_VID_ACCU1(v2));
		dispc->vid2_picture_size = dispc_reg_in(DISPC_VID_PICTURE_SIZE(v2));
		dispc->vid2_fir_coef_h0  = dispc_reg_in(DISPC_VID_FIR_COEF_H(v2,0));
		dispc->vid2_fir_coef_h1  = dispc_reg_in(DISPC_VID_FIR_COEF_H(v2,1));
		dispc->vid2_fir_coef_h2  = dispc_reg_in(DISPC_VID_FIR_COEF_H(v2,2));
		dispc->vid2_fir_coef_h3  = dispc_reg_in(DISPC_VID_FIR_COEF_H(v2,3));
		dispc->vid2_fir_coef_h4  = dispc_reg_in(DISPC_VID_FIR_COEF_H(v2,4));
		dispc->vid2_fir_coef_h5  = dispc_reg_in(DISPC_VID_FIR_COEF_H(v2,5));
		dispc->vid2_fir_coef_h6  = dispc_reg_in(DISPC_VID_FIR_COEF_H(v2,6));
		dispc->vid2_fir_coef_h7  = dispc_reg_in(DISPC_VID_FIR_COEF_H(v2,7));
		dispc->vid2_fir_coef_hv0 = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2,0));
		dispc->vid2_fir_coef_hv1 = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2,1));
		dispc->vid2_fir_coef_hv2 = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2,2));
		dispc->vid2_fir_coef_hv3 = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2,3));
		dispc->vid2_fir_coef_hv4 = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2,4));
		dispc->vid2_fir_coef_hv5 = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2,5));
		dispc->vid2_fir_coef_hv6 = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2,6));
		dispc->vid2_fir_coef_hv7 = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2,7));
		dispc->vid2_conv_coef0   = dispc_reg_in(DISPC_VID_CONV_COEF0(v2));
		dispc->vid2_conv_coef1   = dispc_reg_in(DISPC_VID_CONV_COEF1(v2));
		dispc->vid2_conv_coef2   = dispc_reg_in(DISPC_VID_CONV_COEF2(v2));
		dispc->vid2_conv_coef3   = dispc_reg_in(DISPC_VID_CONV_COEF3(v2));
		dispc->vid2_conv_coef4   = dispc_reg_in(DISPC_VID_CONV_COEF4(v2));
#ifdef CONFIG_ARCH_OMAP3430
		dispc->vid2_fir_coef_v0   = dispc_reg_in(DISPC_VID_FIR_COEF_V(v2,0));
		dispc->vid2_fir_coef_v1   = dispc_reg_in(DISPC_VID_FIR_COEF_V(v2,1));
		dispc->vid2_fir_coef_v2   = dispc_reg_in(DISPC_VID_FIR_COEF_V(v2,2));
		dispc->vid2_fir_coef_v3   = dispc_reg_in(DISPC_VID_FIR_COEF_V(v2,3));
		dispc->vid2_fir_coef_v4   = dispc_reg_in(DISPC_VID_FIR_COEF_V(v2,4));
		dispc->vid2_fir_coef_v5   = dispc_reg_in(DISPC_VID_FIR_COEF_V(v2,5));
		dispc->vid2_fir_coef_v6   = dispc_reg_in(DISPC_VID_FIR_COEF_V(v2,6));
		dispc->vid2_fir_coef_v7   = dispc_reg_in(DISPC_VID_FIR_COEF_V(v2,7));
#endif
		break;
	}
	layer[ltype].ctx_valid = 1;
}

void omap2_disp_save_initstate(int ltype)
{
	unsigned long flags;

	spin_lock_irqsave(&dss_lock, flags);
	omap2_disp_save_ctx(ltype);
	spin_unlock_irqrestore(&dss_lock, flags);
}

/* 
 *  NOte, that VENC registers are not restored here 
 *  Note, that DISPC_CONTROL register is not restored here
 */
void
omap2_disp_restore_ctx(int ltype)
{
	int v1=0, v2=1;
	struct omap24xx_dispc_regs *dispc = &dss_ctx.dispc;

	if (layer[ltype].ctx_valid == 0)
		return;

	switch(ltype){
	case OMAP_DSS_GENERIC:
		dss_reg_out(DSS_SYSCONFIG, dss_ctx.sysconfig);
		dss_reg_out(DSS_CONTROL, dss_ctx.control);
#ifdef CONFIG_ARCH_OMAP3430
		dss_reg_out(DSS_SDI_CONTROL, dss_ctx.sdi_control);
		dss_reg_out(DSS_PLL_CONTROL, dss_ctx.pll_control);
#endif
		break;

	case OMAP_DSS_DISPC_GENERIC:
		dispc_reg_out(DISPC_SYSCONFIG, dispc->sysconfig);
		dispc_reg_out(DISPC_IRQENABLE, dispc->irqenable);
		dispc_reg_out(DISPC_CONFIG, dispc->config);
		dispc_reg_out(DISPC_DEFAULT_COLOR0, dispc->default_color0);
		dispc_reg_out(DISPC_DEFAULT_COLOR1, dispc->default_color1);
		dispc_reg_out(DISPC_TRANS_COLOR0, dispc->trans_color0);
		dispc_reg_out(DISPC_TRANS_COLOR1, dispc->trans_color1);
		dispc_reg_out(DISPC_LINE_NUMBER, dispc->line_number);
		dispc_reg_out(DISPC_DATA_CYCLE1, dispc->data_cycle1);
		dispc_reg_out(DISPC_DATA_CYCLE2, dispc->data_cycle2);
		dispc_reg_out(DISPC_DATA_CYCLE3, dispc->data_cycle3);
		dispc_reg_out(DISPC_TIMING_H, dispc->timing_h);
		dispc_reg_out(DISPC_TIMING_V, dispc->timing_v);
		dispc_reg_out(DISPC_POL_FREQ, dispc->pol_freq);
		dispc_reg_out(DISPC_DIVISOR, dispc->divisor);
		dispc_reg_out(DISPC_GLOBAL_ALPHA, dispc->global_alpha);
		dispc_reg_out(DISPC_SIZE_LCD, dispc->size_lcd);
		dispc_reg_out(DISPC_SIZE_DIG, dispc->size_dig);
		break;

	case OMAP2_GRAPHICS:
		dispc_reg_out(DISPC_GFX_BA0, dispc->gfx_ba0);
		dispc_reg_out(DISPC_GFX_BA1, dispc->gfx_ba1);
		dispc_reg_out(DISPC_GFX_POSITION, dispc->gfx_position);
		dispc_reg_out(DISPC_GFX_SIZE, dispc->gfx_size);
		dispc_reg_out(DISPC_GFX_ATTRIBUTES, dispc->gfx_attributes);
		dispc_reg_out(DISPC_GFX_FIFO_SIZE,dispc->gfx_fifo_size);
		dispc_reg_out(DISPC_GFX_FIFO_THRESHOLD, dispc->gfx_fifo_threshold);
		dispc_reg_out(DISPC_GFX_ROW_INC, dispc->gfx_row_inc);
		dispc_reg_out(DISPC_GFX_PIXEL_INC, dispc->gfx_pixel_inc);
		dispc_reg_out(DISPC_GFX_WINDOW_SKIP, dispc->gfx_window_skip);
		dispc_reg_out(DISPC_GFX_TABLE_BA, dispc->gfx_table_ba);

		break;

	case OMAP2_VIDEO1:
		dispc_reg_out(DISPC_VID_BA0(v1), dispc->vid1_ba0);
		dispc_reg_out(DISPC_VID_BA1(v1), dispc->vid1_ba1);
		dispc_reg_out(DISPC_VID_POSITION(v1), dispc->vid1_position);
		dispc_reg_out(DISPC_VID_SIZE(v1), dispc->vid1_size);
		dispc_reg_out(DISPC_VID_ATTRIBUTES(v1), dispc->vid1_attributes);
		dispc_reg_out(DISPC_VID_FIFO_THRESHOLD(v1),dispc->vid1_fifo_threshold);
		dispc_reg_out(DISPC_VID_ROW_INC(v1), dispc->vid1_row_inc);
		dispc_reg_out(DISPC_VID_PIXEL_INC(v1), dispc->vid1_pixel_inc);
		dispc_reg_out(DISPC_VID_FIR(v1), dispc->vid1_fir);
		dispc_reg_out(DISPC_VID_ACCU0(v1), dispc->vid1_accu0);
		dispc_reg_out(DISPC_VID_ACCU1(v1), dispc->vid1_accu1);
		dispc_reg_out(DISPC_VID_PICTURE_SIZE(v1), dispc->vid1_picture_size);

		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1,0), dispc->vid1_fir_coef_h0);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1,1), dispc->vid1_fir_coef_h1);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1,2), dispc->vid1_fir_coef_h2);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1,3), dispc->vid1_fir_coef_h3);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1,4), dispc->vid1_fir_coef_h4);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1,5), dispc->vid1_fir_coef_h5);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1,6), dispc->vid1_fir_coef_h6);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1,7), dispc->vid1_fir_coef_h7);

		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1,0), dispc->vid1_fir_coef_hv0);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1,1), dispc->vid1_fir_coef_hv1);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1,2), dispc->vid1_fir_coef_hv2);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1,3), dispc->vid1_fir_coef_hv3);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1,4), dispc->vid1_fir_coef_hv4);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1,5), dispc->vid1_fir_coef_hv5);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1,6), dispc->vid1_fir_coef_hv6);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1,7), dispc->vid1_fir_coef_hv7);

		dispc_reg_out(DISPC_VID_CONV_COEF0(v1), dispc->vid1_conv_coef0);
		dispc_reg_out(DISPC_VID_CONV_COEF1(v1), dispc->vid1_conv_coef1);
		dispc_reg_out(DISPC_VID_CONV_COEF2(v1), dispc->vid1_conv_coef2);
		dispc_reg_out(DISPC_VID_CONV_COEF3(v1), dispc->vid1_conv_coef3);
		dispc_reg_out(DISPC_VID_CONV_COEF4(v1), dispc->vid1_conv_coef4);
#ifdef CONFIG_ARCH_OMAP3430
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v1,0), dispc->vid1_fir_coef_v0);
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v1,1), dispc->vid1_fir_coef_v1);
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v1,2), dispc->vid1_fir_coef_v2);
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v1,3), dispc->vid1_fir_coef_v3);
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v1,4), dispc->vid1_fir_coef_v4);
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v1,5), dispc->vid1_fir_coef_v5);
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v1,6), dispc->vid1_fir_coef_v6);
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v1,7), dispc->vid1_fir_coef_v7);
#endif
		break;

	case OMAP2_VIDEO2:
		dispc_reg_out(DISPC_VID_BA0(v2), dispc->vid2_ba0);
		dispc_reg_out(DISPC_VID_BA1(v2), dispc->vid2_ba1);
		dispc_reg_out(DISPC_VID_POSITION(v2), dispc->vid2_position);
		dispc_reg_out(DISPC_VID_SIZE(v2), dispc->vid2_size);
		dispc_reg_out(DISPC_VID_ATTRIBUTES(v2), dispc->vid2_attributes);
		dispc_reg_out(DISPC_VID_FIFO_THRESHOLD(v2),dispc->vid2_fifo_threshold);
		dispc_reg_out(DISPC_VID_ROW_INC(v2), dispc->vid2_row_inc);
		dispc_reg_out(DISPC_VID_PIXEL_INC(v2), dispc->vid2_pixel_inc);
		dispc_reg_out(DISPC_VID_FIR(v2), dispc->vid2_fir);
		dispc_reg_out(DISPC_VID_ACCU0(v2), dispc->vid2_accu0);
		dispc_reg_out(DISPC_VID_ACCU1(v2), dispc->vid2_accu1);
		dispc_reg_out(DISPC_VID_PICTURE_SIZE(v2), dispc->vid2_picture_size);

		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2,0), dispc->vid2_fir_coef_h0);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2,1), dispc->vid2_fir_coef_h1);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2,2), dispc->vid2_fir_coef_h2);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2,3), dispc->vid2_fir_coef_h3);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2,4), dispc->vid2_fir_coef_h4);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2,5), dispc->vid2_fir_coef_h5);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2,6), dispc->vid2_fir_coef_h6);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2,7), dispc->vid2_fir_coef_h7);

		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2,0), dispc->vid2_fir_coef_hv0);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2,1), dispc->vid2_fir_coef_hv1);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2,2), dispc->vid2_fir_coef_hv2);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2,3), dispc->vid2_fir_coef_hv3);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2,4), dispc->vid2_fir_coef_hv4);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2,5), dispc->vid2_fir_coef_hv5);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2,6), dispc->vid2_fir_coef_hv6);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2,7), dispc->vid2_fir_coef_hv7);

		dispc_reg_out(DISPC_VID_CONV_COEF0(v2), dispc->vid2_conv_coef0);
		dispc_reg_out(DISPC_VID_CONV_COEF1(v2), dispc->vid2_conv_coef1);
		dispc_reg_out(DISPC_VID_CONV_COEF2(v2), dispc->vid2_conv_coef2);
		dispc_reg_out(DISPC_VID_CONV_COEF3(v2), dispc->vid2_conv_coef3);
		dispc_reg_out(DISPC_VID_CONV_COEF4(v2), dispc->vid2_conv_coef4);
#ifdef CONFIG_ARCH_OMAP3430
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v2,0), dispc->vid2_fir_coef_v0);
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v2,1), dispc->vid2_fir_coef_v1);
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v2,2), dispc->vid2_fir_coef_v2);
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v2,3), dispc->vid2_fir_coef_v3);
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v2,4), dispc->vid2_fir_coef_v4);
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v2,5), dispc->vid2_fir_coef_v5);
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v2,6), dispc->vid2_fir_coef_v6);
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v2,7), dispc->vid2_fir_coef_v7);
#endif
		break;
	}
}

/*
 * Sync Lost interrupt handler
 */
static void
omap2_synclost_isr(void *arg, struct pt_regs *regs, u32 irqstatus)
{
	int i=0;
	printk(KERN_WARNING "Sync Lost LCD %x\n",dispc_reg_in(DISPC_IRQSTATUS));
	arg = NULL; regs = NULL;

	/*
	 * Disable and Clear all the interrupts before we start
	 */
	dispc_reg_out(DISPC_IRQENABLE, 0x00000000);
	dispc_reg_out(DISPC_IRQSTATUS, 0x0000FFFF);

	/* disable the display controller */
	omap2_disp_disable(HZ/5);

	/*
	 * Update the state of the display controller.
	 */
	dss_ctx.dispc.sysconfig &= ~DISPC_SYSCONFIG_SOFTRESET;
	dss_ctx.dispc.control   &= ~(DISPC_CONTROL_GODIGITAL |
				     DISPC_CONTROL_GOLCD);

        i = 0;

        dispc_reg_out(DISPC_SYSCONFIG, DISPC_SYSCONFIG_SOFTRESET);
        while (!(dispc_reg_in(DISPC_SYSSTATUS) & DISPC_SYSSTATUS_RESETDONE)) {
                udelay (100);
                if (i++ > 5) {
                        printk(KERN_WARNING "Failed to soft reset the DSS !! \n");
                        break;
                }
        }

	/* Configure the VENC for the default standard */
	if ((omap2_disp_get_output_dev(OMAP2_GRAPHICS) == OMAP2_OUTPUT_TV) ||
            (omap2_disp_get_output_dev(OMAP2_VIDEO1) == OMAP2_OUTPUT_TV) ||
            (omap2_disp_get_output_dev(OMAP2_VIDEO2) == OMAP2_OUTPUT_TV)){
		omap2_disp_set_tvstandard(omap2_current_tvstandard);
	}

	/* Restore the registers */
	omap2_disp_restore_ctx(OMAP_DSS_DISPC_GENERIC);
	omap2_disp_restore_ctx(OMAP2_GRAPHICS);
	omap2_disp_restore_ctx(OMAP2_VIDEO1);
	omap2_disp_restore_ctx(OMAP2_VIDEO2);

	/* enable the display controller */
	if (layer[OMAP_DSS_DISPC_GENERIC].ctx_valid)
		dispc_reg_out(DISPC_CONTROL, dss_ctx.dispc.control);

	omap2_disp_reg_sync(OMAP2_OUTPUT_LCD);
	if ((omap2_disp_get_output_dev(OMAP2_GRAPHICS) == OMAP2_OUTPUT_TV) ||
            (omap2_disp_get_output_dev(OMAP2_VIDEO1) == OMAP2_OUTPUT_TV) ||
            (omap2_disp_get_output_dev(OMAP2_VIDEO2) == OMAP2_OUTPUT_TV)){
		omap2_disp_reg_sync(OMAP2_OUTPUT_TV);
	}
}

static inline u32 pages_per_side(u32 img_side, u32 page_exp)
{
	/*  page_side = 2 ^ page_exp
	 * (page_side - 1) is added for rounding up
	 */
	return (u32) (img_side + (1<<page_exp) - 1) >> page_exp;
}

int omap2_disp_get_vrfb_offset(u32 img_len, u32 bytes_per_pixel,int side)
{
	int page_width_exp, page_height_exp, pixel_size_exp,offset =0;
	
	/* Maximum supported is 4 bytes (RGB32) */
	if (bytes_per_pixel > 4)
		return -EINVAL;

	page_width_exp  = PAGE_WIDTH_EXP;
	page_height_exp = PAGE_HEIGHT_EXP;
	pixel_size_exp  = bytes_per_pixel >> 1;
	
	if(side == SIDE_W) {
		offset = ((1<<page_width_exp) * (pages_per_side(img_len * bytes_per_pixel, page_width_exp))) >> pixel_size_exp;	/* in pixels */
	} else {
		offset = (1<<page_height_exp) * (pages_per_side(img_len, page_height_exp));
	}	
	
	return (offset);
}

/* Flip the video overlay framebuffer.  The video overlay window may initially
 * be either enabled or disabled.  The overlay window will be enabled by this
 * routine.  fb_base_phys is the physical base address of the framebuffer for
 * the video overlay.  The address programmed into the base address register of
 * the video overlay window is calculated based on the cropped size and the full
 * size of the overlay framebuffer.
 */
void
omap2_disp_start_vlayer(int ltype, struct v4l2_pix_format *pix,
			struct v4l2_rect *crop, struct v4l2_window *win, 
			unsigned long fb_base_phys,
			int rotation_deg, int mirroring)
{
	unsigned long cropped_base_phys;
	int v, ps = 2;
	int bytesperpixel;
	int line_length;
	u32 gfx_format;

//printk("omap2_disp_start_vlayer: ltype %d\r\n", ltype );

	gfx_format = dispc_reg_in(DISPC_GFX_ATTRIBUTES);

	switch (gfx_format & DISPC_GFX_ATTRIBUTES_GFXFORMAT) {
		case DISPC_GFX_ATTRIBUTES_GFXFORMAT_BITMAP1:
		case DISPC_GFX_ATTRIBUTES_GFXFORMAT_BITMAP2:
		case DISPC_GFX_ATTRIBUTES_GFXFORMAT_BITMAP4:
		case DISPC_GFX_ATTRIBUTES_GFXFORMAT_BITMAP8:
			bytesperpixel = 1;
			break;
		case DISPC_GFX_ATTRIBUTES_GFXFORMAT_RGB12:
		case DISPC_GFX_ATTRIBUTES_GFXFORMAT_RGB16:
			bytesperpixel = 2;
			break;
		case DISPC_GFX_ATTRIBUTES_GFXFORMAT_RGB24:
		default:
			bytesperpixel = 4;
			break;
	}

	if (ltype == OMAP2_VIDEO1) 
		v = 0;
	else if (ltype == OMAP2_VIDEO2)
		v = 1;
	else 
		return;

	/*
	 * If pixel format is YUV then PS = 4, for RGB16 PS = 2 RGB24 Unpack PS =4
	 */

	if (V4L2_PIX_FMT_YUYV == pix->pixelformat || V4L2_PIX_FMT_UYVY == pix->pixelformat) {
		ps = 2; /* otherwise the pixel size is 2 byte */
	} else if(V4L2_PIX_FMT_RGB32 == pix->pixelformat) {
		ps = 4;
	} else if(V4L2_PIX_FMT_RGB24 == pix->pixelformat) {
		ps = 3;
	}

	line_length = pix->width;

	cropped_base_phys = fb_base_phys + line_length * ps * crop->top + crop->left * ps;

	/* 
	 * We store the information in the layer structure for : If user 
	 * dynamically switches the pipeline from LCD to TV or vice versa 
	 * we should have the necessary configurations for the output device 
	 * (LCD/TV) */

	if ( omap2_disp_get_output_dev(ltype) == OMAP2_OUTPUT_TV || _lcd_interlaced() ){
		/* If output path is set to TV */
		layer[ltype].dma[0].ba0 = cropped_base_phys;
		layer[ltype].dma[0].ba1 = cropped_base_phys + layer[ltype].field_offset; 
		
		dispc_reg_out(DISPC_VID_BA0(v), layer[ltype].dma[0].ba0);
		dispc_reg_out(DISPC_VID_BA1(v),	layer[ltype].dma[0].ba1);

		dispc_reg_merge(DISPC_VID_ATTRIBUTES(v), DISPC_VID_ATTRIBUTES_ENABLE, DISPC_VID_ATTRIBUTES_ENABLE);
	} else {
		/* If output path is set to LCD */
		layer[ltype].dma[0].ba0 = cropped_base_phys;
		layer[ltype].dma[0].ba1 = cropped_base_phys;
		
		dispc_reg_out(DISPC_VID_BA0(v), layer[ltype].dma[0].ba0);
		dispc_reg_out(DISPC_VID_BA1(v), layer[ltype].dma[0].ba1);
		dispc_reg_merge(DISPC_VID_ATTRIBUTES(v), DISPC_VID_ATTRIBUTES_ENABLE, DISPC_VID_ATTRIBUTES_ENABLE);
	}

	/*
	 * Store BA0 BA1 for TV, BA1 points to the alternate row
	 */
	layer[ltype].dma[1].ba0 = cropped_base_phys;
	layer[ltype].dma[1].ba1 = cropped_base_phys + line_length * ps;

//printk("layer[%d].dma[0].ba0: %08X\r\n",ltype, layer[ltype].dma[0].ba0 );
//printk("layer[%d].dma[0].ba1: %08X\r\n",ltype, layer[ltype].dma[0].ba1 );
//printk("layer[%d].dma[1].ba0: %08X\r\n",ltype, layer[ltype].dma[1].ba0 );
//printk("layer[%d].dma[1].ba1: %08X\r\n",ltype, layer[ltype].dma[1].ba1 );
}

/* Configure VIDEO1 or VIDEO2 layer parameters*/
void
omap2_disp_config_vlayer(int ltype, struct v4l2_pix_format *pix,
			 struct v4l2_rect *crop, struct v4l2_window *win,
			 int rotation_deg, int mirroring)
{
	int vid_position_x, vid_position_y, ps = 2,vr_ps = 1;
	unsigned long vid_position, vid_size, vid_picture_size;
	unsigned long vid_attributes;
	unsigned long firvinc, firhinc;
	int winheight, winwidth, cropheight, cropwidth, pixheight, pixwidth;
	int winleft, wintop, cleft, ctop;
	int panelwidth, panelheight, row_inc_value=0, pixel_inc_value=0;
	int vc_five_tap = 0;
	
	/* vertical upsampling 3 taps */
	const static short int v_up_3taps[3][8] =
	      { {   0,   2,   5,   7,  64,  32,  12,   3  },
		{ 128, 123, 111,  89,  64,  89, 111, 123  },
		{  0,    3,  12,  32,   0,   7,   5,   2  } };
		
	/* horizontal upsampling 5 taps */
	const static short int h_up_5taps[5][8] =
	      { {   0,   0,  -1,  -2,  -9,  -5,  -2,  -1 },
		{   0,  -8, -11, -11,  73,  51,  30,  13 },
		{ 128, 124, 112,  95,  73,  95, 112, 124 },
		{   0,  13,  30,  51,  -9, -11, -11,  -8 },
		{   0,  -1,  -2,  -5,   0,  -2,  -1,   0 } };
	
	/* vertical upsampling 5 taps */
	const static short int v_up_5taps[5][8] =
	      { {   0,  -8, -11, -11,  73,  51,  30,  13 },
		{ 128, 124, 112,  95,  73,  95, 112, 124 },
		{   0,  13,  30,  51,  -9, -11, -11,  -8 },
		{   0,   0,  -1,  -2,  -9,  -5,  -2,  -1 },
		{   0,  -1,  -2,  -5,   0,  -2,  -1,   0 } };
	
	/* vertical downsampling 3 taps */
	const static short int v_dn_3taps[3][8] =
	      { {   36,   31,   27,  23,  55,  50,  45,  40  },
		{   56,   57,   56,  55,  55,  55,  56,  57  },
		{   36,   40,   45,  50,  18,  23,  27,  31  } };

	/* horizontal downsampling 5 taps */
	const static short int h_dn_5taps[5][8] =
	      { {    0,  -2,  -5,  -7,  17,  12,  8,    4 },
		{   36,  31,  27,  22,  51,  48,  44,  40 },
		{   56,  55,  54,  53,  52,  53,  54,  55 },
		{   36,  40,  44,  48,  17,  22,  27,  31 },
		{    0,   4,   8, -12,  -9,  -7, -5,   -2 } };

	/* vertical downsampling 5 taps */
	const static short int v_dn_5taps[5][8] =

	      { {  36, 31, 27, 22,  51, 48, 44, 40 },
		{  56, 55, 54, 53,  52, 53, 54, 55 },
		{  36, 40, 44, 48,  17, 22, 27, 31 },
		{   0, -2, -5, -7,  17, 12,  8,  4 },
		{   0,  4,  8, 12,  -9, -7, -5, -2 } };
	
	short int* vc = (short int*)v_up_5taps;
	short int* hc = (short int*)h_up_5taps;

	int v;

DBG printk("omap2_disp_config_vlayer: ltype %d\r\n", ltype );

	if (ltype == OMAP2_VIDEO1)
		v = 0;
	else if (ltype == OMAP2_VIDEO2)
		v = 1;
	else
		return;

	/* make sure the video overlay is disabled before we reconfigure it */
	omap2_disp_disable_layer(ltype);

	/* configure the video attributes register */
	vid_attributes = 0;
	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
		if(pix->pixelformat == V4L2_PIX_FMT_YUYV){
			vid_attributes |= DISPC_VID_ATTRIBUTES_VIDFORMAT_YUV2;
			vid_attributes |= DISPC_VID_ATTRIBUTES_VIDCOLORCONVENABLE;
		} else{
			vid_attributes |= DISPC_VID_ATTRIBUTES_VIDFORMAT_UYVY;
			vid_attributes |= DISPC_VID_ATTRIBUTES_VIDCOLORCONVENABLE;
		}

		break;
	case V4L2_PIX_FMT_RGB24:
		ps = 3; /* pixel size is 3 bytes */
		vid_attributes |= DISPC_VID_ATTRIBUTES_VIDFORMAT_RGB24P;
		vid_attributes |= ((dispc_reg_in(DISPC_GFX_ATTRIBUTES) & DISPC_GFX_ATTRIBUTES_GFXREPLICATIONENABLE) << DISPC_GFX_ATTRIBUTES_GFXREPEN);
		break;

		/* The picture format is a bit confusing in V4L2.. as per the V4L2 spec
		 * RGB32 and BGR32 are always with alpha bits enabled.. (i.e always in
		 * packed mode) */
	case V4L2_PIX_FMT_RGB32:
		ps = 4; /* pixel size is 4 bytes */
		if ((is_sil_rev_less_than(OMAP3430_REV_ES2_0)) || (ltype == OMAP2_VIDEO1)) {
			vid_attributes |= DISPC_VID_ATTRIBUTES_VIDFORMAT_RGB24;
			vid_attributes |= ((dispc_reg_in(DISPC_GFX_ATTRIBUTES) & DISPC_GFX_ATTRIBUTES_GFXREPLICATIONENABLE) << DISPC_GFX_ATTRIBUTES_GFXREPEN);
		} else {
			vid_attributes |= DISPC_VID_ATTRIBUTES_VIDFORMAT_ARGB32;
			vid_attributes |= DISPC_VID_ATTRIBUTES_VIDENDIANNESS;
		}
		break;
	case V4L2_PIX_FMT_BGR32:
		ps = 4; /* pixel size is 4 bytes */
		if ((is_sil_rev_less_than(OMAP3430_REV_ES2_0)) || (ltype == OMAP2_VIDEO1)) {
			vid_attributes |= DISPC_VID_ATTRIBUTES_VIDFORMAT_RGB24;
			vid_attributes |= ((dispc_reg_in(DISPC_GFX_ATTRIBUTES) & DISPC_GFX_ATTRIBUTES_GFXREPLICATIONENABLE) << DISPC_GFX_ATTRIBUTES_GFXREPEN);
		} else {
			vid_attributes |= DISPC_VID_ATTRIBUTES_VIDFORMAT_ARGB32;
		}
		break;
	case V4L2_PIX_FMT_RGB565:
	default:
		ps = 2; /* pixel size is 2 bytes */
		vid_attributes |= DISPC_VID_ATTRIBUTES_VIDFORMAT_RGB16;
		vid_attributes |= ((dispc_reg_in(DISPC_GFX_ATTRIBUTES) & DISPC_GFX_ATTRIBUTES_GFXREPLICATIONENABLE) << DISPC_GFX_ATTRIBUTES_GFXREPEN);
		break;
	case V4L2_PIX_FMT_RGB565X:
		ps = 2; /* pixel size is 2 bytes */
		vid_attributes |= DISPC_VID_ATTRIBUTES_VIDFORMAT_RGB16;
		vid_attributes |= DISPC_VID_ATTRIBUTES_VIDENDIANNESS;
		vid_attributes |= ((dispc_reg_in(DISPC_GFX_ATTRIBUTES) & DISPC_GFX_ATTRIBUTES_GFXREPLICATIONENABLE) << DISPC_GFX_ATTRIBUTES_GFXREPEN);
		break;
	}

	if (omap2_disp_get_output_dev(ltype) == OMAP2_OUTPUT_TV)
		vid_attributes |= DISPC_VID_ATTRIBUTES_VIDCHANNELOUT;

	/* Enable 16 x 32 burst size */
	vid_attributes |= DISPC_VID_ATTRIBUTES_VIDBURSTSIZE_BURST16X32;

	/* Set FIFO threshold to 0xFF (high) and 0xFF - (16x4bytes) = 0xC0 (low)*/ 
	// dispc_reg_out(DISPC_VID_FIFO_THRESHOLD(v),0x00FF00C0);
	dispc_reg_out(DISPC_VID_FIFO_THRESHOLD(v),0x03FC03BC);

	/* Set the color converion parameters */
	set_colorconv(v,pix->colorspace);

	omap2_disp_get_panel_size(omap2_disp_get_output_dev(ltype), &panelwidth, &panelheight);
	DBG printk("panel WxH %4d x %4d\n", panelwidth, panelheight);

	winwidth   = win->w.width;
	winheight  = win->w.height;
	winleft    = win->w.left;
	wintop     = win->w.top;
	pixheight  = pix->height;
	pixwidth   = pix->width;
	cropwidth  = crop->width;
	cropheight = crop->height;
	ctop       = crop->top;
	cleft      = crop->left;

	DBG printk("win   WxH %4d x %4d  ofs %4d x %4d\n", winwidth, winheight, winleft, wintop);
	DBG printk("pix   WxH %4d x %4d\n",                pixwidth, pixheight);
	DBG printk("crop  WxH %4d x %4d  ofs %4d x %4d\n", cropwidth, cropheight, cleft, ctop );
	DBG printk("rotate %d  mirror %d\n", rotation_deg, mirroring);
	
	/* horizontal */
	if (winwidth != cropwidth) {
		vid_attributes |= DISPC_VID_ATTRIBUTES_VIDRESIZEENABLE_HRESIZE;
		if (winwidth < cropwidth){
			/* downscale */
			DBG printk("H 5-tap downscale\n");
			vid_attributes |= DISPC_VID_ATTRIBUTES_VIDHRESIZECONF;
			/* yes, we should use the dn coeffs, but with them we have vertical black stripes
			hc = (short int *) h_dn_5taps;
			*/
			hc = (short int *) h_up_5taps;
		} else {
			/* upscale */
			DBG printk("H 5-tap upscale\n");
			hc = (short int *) h_up_5taps;
		}
	}
	
	/* vertical */
	if (winheight != cropheight) {
		vid_attributes |= DISPC_VID_ATTRIBUTES_VIDRESIZEENABLE_VRESIZE;
		if (winheight < cropheight){
			/* downscale */
			vid_attributes |= DISPC_VID_ATTRIBUTES_VIDVRESIZECONF;
			if( winheight * 2 < cropheight ) {
				DBG printk("V 5-tap downscale\n");
				vid_attributes |= DISPC_VID_ATTRIBUTES_VIDVERTICALTAPS;
				vid_attributes |= DISPC_VID_ATTRIBUTES_VIDLINEBUFFERSPLIT;
				vc = (short int *) v_dn_5taps;
				vc_five_tap = 1;
			} else {
				DBG printk("V 3-tap downscale\n");
				vc = (short int *) v_dn_3taps;
			}
		} else {
			/* upscale */
#ifdef USE_FIVE_TAP_VERTICAL_UPSCALE
			DBG printk("V 5-tap upscale\n");
			vid_attributes |= DISPC_VID_ATTRIBUTES_VIDVERTICALTAPS;
			vid_attributes |= DISPC_VID_ATTRIBUTES_VIDLINEBUFFERSPLIT;
			vc = (short int *) v_up_5taps;
			vc_five_tap = 1;
#else
			DBG printk("V 3-tap upscale\n");
			vc = (short int *)  v_up_3taps;
#endif
		}
		
	}

	dispc_reg_out(DISPC_VID_ATTRIBUTES(v), vid_attributes);

	/* initialize the resizing filter */
	omap2_disp_set_resize(v, vc, hc, vc_five_tap);

	firhinc = (1024 * (cropwidth - 1)) / (winwidth - 1);
	if (firhinc < 1)
		firhinc = 1;
	else if (firhinc > 4095)
		firhinc = 4095;
	
	firvinc = (1024 * (cropheight - 1)) / (winheight - 1);
	if (firvinc < 1)
		firvinc = 1;
	else if (firvinc > 4095)
		firvinc = 4095;

	DBG printk("rsz           H %4d  V %4d\n", firhinc, firvinc);

	vid_position_x  = win->w.left;
	vid_position_y  = win->w.top;
	pixel_inc_value = 1;

	/* ACCU values */
	layer[ltype].dma[0].accu0 = 0;
	layer[ltype].dma[0].accu1 = 0;

	// 1) for TV the window height should be divided by two
	// 2) configure the source window in the framebuffer
	if ( omap2_disp_get_output_dev(ltype) == OMAP2_OUTPUT_TV || _lcd_interlaced() ) {
		//
		// TV/interlaced
		// 
		
		vid_position_y = vid_position_y / 2;
		// we output 2 fields of 1/2 the size
		
		row_inc_value  = 1 + (pixwidth - cropwidth) * ps;
		
		vid_size = (((winwidth      - 1) << DISPC_VID_SIZE_VIDSIZEX_SHIFT) & DISPC_VID_SIZE_VIDSIZEX)
			 | (((winheight / 2 - 1) << DISPC_VID_SIZE_VIDSIZEY_SHIFT) & DISPC_VID_SIZE_VIDSIZEY);
		
		if( cropheight < winheight ) {
			// if we are upscaling interlaced, we need to take care that we cannot just jump over one line,
			// instead we do this tricky stuff here:
DBG printk("upscale interlaced!\n" );
			// get a new resizer coeff
			firvinc = (1024 * (cropheight * 2 - 1)) / (winheight - 1);
			if (firvinc < 1)
				firvinc = 1;
			else if (firvinc > 4095)
				firvinc = 4095;
				
			DBG printk("rsz           H %4d  V %4d\n", firhinc, firvinc);
			
			// and use the full frame
			vid_picture_size =
				(((cropwidth  - 1) << DISPC_VID_PICTURE_SIZE_VIDORGSIZEX_SHIFT) & DISPC_VID_PICTURE_SIZE_VIDORGSIZEX) |
				(((cropheight - 1) << DISPC_VID_PICTURE_SIZE_VIDORGSIZEY_SHIFT) & DISPC_VID_PICTURE_SIZE_VIDORGSIZEY);
		
			// start from the same line for both fields
			layer[ltype].field_offset = 0;
			
			// get the correct phase for the 2nd field
			layer[ltype].dma[0].accu0 = 512; // what is the correct way to set the phase, e.g. (1024 - (1024 * cropheight / winheight) ) ???
		} else if ( cropheight > winheight ) {
			if( cropwidth <= 1024 ) {
				// if we are downscaling interlaced, we need to take care that we cannot just jump over one line,
				// instead we do this tricky stuff here:
DBG printk("downscale interlaced < 1024\n" );
				// get a new resizer coeff
				firvinc = (1024 * (cropheight * 2 - 1)) / (winheight - 1);
				if (firvinc < 1)
					firvinc = 1;
				else if (firvinc > 4095)
					firvinc = 4095;

				DBG printk("rsz           H %4d  V %4d\n", firhinc, firvinc);

				// and use the full frame
				vid_picture_size =
					(((cropwidth      - 1) << DISPC_VID_PICTURE_SIZE_VIDORGSIZEX_SHIFT) & DISPC_VID_PICTURE_SIZE_VIDORGSIZEX) |
					(((cropheight     - 1) << DISPC_VID_PICTURE_SIZE_VIDORGSIZEY_SHIFT) & DISPC_VID_PICTURE_SIZE_VIDORGSIZEY);

				if( cropheight > winheight ) {
DBG printk("downscale > 2!\n" );
					// start 2nd field from next line
					layer[ltype].field_offset = pixwidth * ps;
					// get the correct phase for the 2nd field
					layer[ltype].dma[0].accu0 = firvinc / 2 - 1024;
				} else {
DBG printk("downscale < 2!\n" );
					// start from the same line for both fields
					layer[ltype].field_offset = 0;
					// get the correct phase for the 2nd field
					layer[ltype].dma[0].accu0 = firvinc / 2; 
				}

				/* downscale with 5tap filter*/
				vid_attributes |= DISPC_VID_ATTRIBUTES_VIDVRESIZECONF;
				DBG printk("V 5-tap downscale\n");
				vid_attributes |= DISPC_VID_ATTRIBUTES_VIDVERTICALTAPS;
				vid_attributes |= DISPC_VID_ATTRIBUTES_VIDLINEBUFFERSPLIT;

				dispc_reg_out(DISPC_VID_ATTRIBUTES(v), vid_attributes);

				/* initialize the resizing filter */
				omap2_disp_set_resize(v, v_dn_5taps, hc, 1);
			} else {
DBG printk("downscale interlaced > 1024\n" );
				// seems like we cannot use the above "tricky" scheme for a crop width > 1024, in this case use the crude 
				// "skip over every 2nd line in the source" scheme:
				vid_picture_size =
				(((cropwidth      - 1) << DISPC_VID_PICTURE_SIZE_VIDORGSIZEX_SHIFT) & DISPC_VID_PICTURE_SIZE_VIDORGSIZEX) |
				(((cropheight / 2 - 1) << DISPC_VID_PICTURE_SIZE_VIDORGSIZEY_SHIFT) & DISPC_VID_PICTURE_SIZE_VIDORGSIZEY);
			
				layer[ltype].field_offset = pixwidth * ps;
				row_inc_value = 1 + (pixwidth - cropwidth) * ps + pix->width * ps;
			}
		} else {
DBG printk("1:1 interlaced!\n" );
			// actually resize 1/2 the crop 
			vid_picture_size =
				(((cropwidth      - 1) << DISPC_VID_PICTURE_SIZE_VIDORGSIZEX_SHIFT) & DISPC_VID_PICTURE_SIZE_VIDORGSIZEX) |
				(((cropheight / 2 - 1) << DISPC_VID_PICTURE_SIZE_VIDORGSIZEY_SHIFT) & DISPC_VID_PICTURE_SIZE_VIDORGSIZEY);

			// start 2nd field from next line
			layer[ltype].field_offset = pixwidth * ps;
			
			// and add one line to the row_inc
			row_inc_value            += pix->width * ps;
		}
	} else {
		//
		// LCD
		//
		vid_size = (((winwidth - 1)      << DISPC_VID_SIZE_VIDSIZEX_SHIFT) & DISPC_VID_SIZE_VIDSIZEX)
			| (((winheight - 1)      << DISPC_VID_SIZE_VIDSIZEY_SHIFT) & DISPC_VID_SIZE_VIDSIZEY);
		
		vid_picture_size =
			(((cropwidth  - 1) << DISPC_VID_PICTURE_SIZE_VIDORGSIZEX_SHIFT) & DISPC_VID_PICTURE_SIZE_VIDORGSIZEX) |
			(((cropheight - 1) << DISPC_VID_PICTURE_SIZE_VIDORGSIZEY_SHIFT) & DISPC_VID_PICTURE_SIZE_VIDORGSIZEY);
		
		layer[ltype].field_offset = 0;

		row_inc_value   = 1 + (pixwidth - cropwidth) * ps;
	}

	layer[ltype].dma[0].row_inc = row_inc_value;
	layer[ltype].dma[0].pix_inc = pixel_inc_value;
	layer[ltype].dma[1].pix_inc = pixel_inc_value;

	DBG printk("vid_size      %08X\n", vid_size);
	DBG printk("vid_pic_siz   %08X\n", vid_picture_size);
	DBG printk("field_offset  %d\n", layer[ltype].field_offset );
	DBG printk("row_inc_value %d\n", row_inc_value);
	DBG printk("pxl_inc_value %d\n", pixel_inc_value);
	DBG printk("accu0         %08X\n", layer[ltype].dma[0].accu0);
	DBG printk("accu1         %08X\n", layer[ltype].dma[0].accu1);

	dispc_reg_out(DISPC_VID_FIR(v),       firhinc | (firvinc << 16));
	dispc_reg_out(DISPC_VID_ACCU0(v),     layer[ltype].dma[0].accu0);
	dispc_reg_out(DISPC_VID_ACCU1(v),     layer[ltype].dma[0].accu1);
	dispc_reg_out(DISPC_VID_ROW_INC(v),   row_inc_value);
	dispc_reg_out(DISPC_VID_PIXEL_INC(v), pixel_inc_value);

	layer[ltype].size_x = cropwidth;
	layer[ltype].size_y = cropheight;

	vid_position =    ((vid_position_x << DISPC_VID_POSITION_VIDPOSX_SHIFT) & DISPC_VID_POSITION_VIDPOSX)
			| ((vid_position_y << DISPC_VID_POSITION_VIDPOSY_SHIFT) & DISPC_VID_POSITION_VIDPOSY);

	DBG printk("vid_position  %08X\n", vid_position);
	dispc_reg_out(DISPC_VID_POSITION(v), vid_position);
	dispc_reg_out(DISPC_VID_SIZE(v), vid_size);
	dispc_reg_out(DISPC_VID_PICTURE_SIZE(v), vid_picture_size);
	omap2_disp_save_initstate(ltype);
}

static int omap2_disp_check_enabled(int output_dev)
{
	u32 control = dispc_reg_in (DISPC_CONTROL);

	if(output_dev == OMAP2_OUTPUT_LCD){
		return !!(control & DISPC_CONTROL_LCDENABLE);
	} else {
		return !!(control & DISPC_CONTROL_DIGITALENABLE);
	} 
}

/* Many display controller registers are shadowed. Setting the GO bit causes
 * changes to these registers to take effect in hardware.
 */
void
omap2_disp_reg_sync(int output_dev)
{
	if (output_dev == OMAP2_OUTPUT_LCD)
		dispc_reg_merge(DISPC_CONTROL, DISPC_CONTROL_GOLCD,
				DISPC_CONTROL_GOLCD);
	else
		dispc_reg_merge(DISPC_CONTROL, DISPC_CONTROL_GODIGITAL,
				DISPC_CONTROL_GODIGITAL);
}

/* This function provides the status of the GO bit. After the GO bit is set
 * through software, register changes take affect at the next VFP (vertical
 * front porch) or EVSYNC. Per the specs, no further register changes
 * must be done until the GO bit is reset by hardware. This function allows
 * drivers to poll the status of the GO bit, and wait until it is reset if they
 * wish to.
 */
int
omap2_disp_reg_sync_done(int output_dev)
{
	u32 control = dispc_reg_in(DISPC_CONTROL);

	if (output_dev == OMAP2_OUTPUT_LCD)
		return !(control & DISPC_CONTROL_GOLCD);
	else
		return !(control & DISPC_CONTROL_GODIGITAL);
}

/* The GO bit needs to be set for the shadowed registers to take effect in
 * hardware. Once the hardware is ready, the GO bit will be reset. Per the
 * hardware specifications, we should not change any display controller
 * registers until the GO bit is reset.
 * This function polls the GO bit and waits until it is reset.
 *
 *
 *	copied from drivers/video/omap_omap_fb
 */
static void
wait_for_reg_sync(int output_dev, unsigned long timeout)
{
	int count = 100;
	if ( output_dev == OMAP2_OUTPUT_TV && ( dispc_reg_in(DISPC_CONTROL) & ( DISPC_CONTROL_GODIGITAL | DISPC_CONTROL_DIGITALENABLE)) == DISPC_CONTROL_GODIGITAL) {
		printk("can't clear GODIGITAL - DIGITAL not enabled yet\n");
		__backtrace();
		return;
	}
	if ( output_dev == OMAP2_OUTPUT_LCD && ( dispc_reg_in(DISPC_CONTROL) & ( DISPC_CONTROL_GOLCD | DISPC_CONTROL_LCDENABLE)) == DISPC_CONTROL_GOLCD) {
		printk("can't clear GOLCD - LCD not enabled yet\n");
		__backtrace();
		return;
	}
	timeout = jiffies + timeout;
	while (!omap2_disp_reg_sync_done(output_dev) &&
	       time_before(jiffies, timeout) && count) {

		if (in_interrupt()) {
			udelay(100);
			count--;
		}
		else {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(1);
		}
	}
        if (!omap2_disp_reg_sync_done(output_dev)) {
		printk(KERN_WARNING "timeout waiting for display controller "
		       "to save registers\n");
		__backtrace();
	}
}

/* This function turns on/off the clocks needed for TV-out.
 *  - 2430SDP: Controls the dss_54m_fck 
 *  - 3430SDP: Controls the dss_tv_fck 
 *  - 3430LAB: Controls both dss_tv_fck and dss_96m_fck.
 *             By default Labrador turns off the 96MHz DAC clock for
 *             power saving reasons.
 */
#ifndef CONFIG_ARCH_OMAP3410
static void
omap24xx_ll_config_tv_clocks(int sleep_state)
{
	static int start = 1;
	static struct clk *tv_clk;
#ifdef CONFIG_MACH_OMAP_3430LABRADOR
	static struct clk *dac_clk;
#endif
	static int disabled = 0;
	static int enabled = 0;

	if(start) {
#ifdef CONFIG_MACH_OMAP_2430SDP
		tv_clk = clk_get(NULL,"dss_54m_fck");
#endif
#if defined(CONFIG_MACH_OMAP_3430SDP) || defined(CONFIG_MACH_OMAP3_EVM) || defined(CONFIG_MACH_ARCHOS_G6)
		tv_clk = clk_get(NULL,"dss_tv_fck");
#endif
		if(IS_ERR(tv_clk)) {
			printk("\n UNABLE to get dss TV fclk \n");
			return;
		}
		start = 0;
	}

	if (sleep_state == 1) {
		if (disabled == 0) {
			clk_disable(tv_clk);
			disabled = 1;
		}
		enabled = 0;
	}
	else {
		if (enabled == 0) {
			if(clk_enable(tv_clk) != 0) {
				printk("\n UNABLE to enable dss TV fclk \n");
				return;
			}
			enabled = 1;
		}
		disabled = 0;
	}
}
#endif

/*
 * Disable the display controller. May be called in interrupt or process
 * context. However, this function should not be called with interrupts
 * disabled as jiffies will not increment.
 */
void
omap2_disp_disable(unsigned long timeout_ticks)
{
	unsigned long timeout;
	u32 dispc_control = dispc_reg_in(DISPC_CONTROL);
	
	if (dispc_control & (DISPC_CONTROL_DIGITALENABLE | DISPC_CONTROL_LCDENABLE))
	{
		/* disable the display controller */
		dispc_reg_merge(DISPC_CONTROL, 0,
				DISPC_CONTROL_DIGITALENABLE | DISPC_CONTROL_LCDENABLE);

		if ( dispc_control & DISPC_CONTROL_DIGITALENABLE ) {
			/* if we output to the video encoder
			   wait for any frame in progress to complete */
			dispc_reg_out(DISPC_IRQSTATUS,
					DISPC_IRQSTATUS_FRAMEDONE);
			timeout = jiffies + timeout_ticks;
			while(!(dispc_reg_in(DISPC_IRQSTATUS)
						& DISPC_IRQSTATUS_FRAMEDONE)
					&& time_before(jiffies, timeout))
			{
				int a_ctx = (in_atomic() || irqs_disabled() 
						|| in_interrupt());
				if (!a_ctx) {
					set_current_state(TASK_INTERRUPTIBLE);
					schedule_timeout(1);
				} else
					udelay(100);
			}
			if (!(dispc_reg_in(DISPC_IRQSTATUS)
						& DISPC_IRQSTATUS_FRAMEDONE)) {
				printk(KERN_WARNING "timeout waiting for "
						"frame-done interrupt\n");
			}
		}
#ifndef CONFIG_ARCH_OMAP3410
		// this is done already in omap2_disp_put_dss() and causes an error message from the CM...
//		omap24xx_ll_config_tv_clocks(1);
#endif
	}

	return;
}

void
omap2_disp_set_gfx_palette(u32 palette_ba)
{
	dispc_reg_out(DISPC_GFX_TABLE_BA, palette_ba);
	dispc_reg_merge(DISPC_CONFIG, DISPC_CONFIG_LOADMODE_PGTABUSETB,
			DISPC_CONFIG_LOADMODE_PGTABUSETB);
}

/* Configure Graphics layer parameters */
void
omap2_disp_config_gfxlayer(u32 size_x, u32 size_y, int color_depth)
{
	u32 config = 0;
	u32 gfx_attributes = 0, gfx_fifo_threshold = 0, gfx_format = 0;
	u32 gfx_position = 0, gfx_window_skip = 0;
DBG printk("omap2_disp_config_gfxlayer %d x %d @ %d\r\n", size_x, size_y, color_depth );  

	config = dispc_reg_in(DISPC_CONFIG);

	config |= (DISPC_CONFIG_LOADMODE_PGTABUSETB |
			DISPC_CONFIG_LOADMODE_FRDATLEFR);

	/* This driver doesn't currently support the video windows, so
	 * we force the palette/gamma table to be a palette table and
	 * force both video windows to be disabled.
	 */
	config &= ~DISPC_CONFIG_PALETTEGAMMATABLE;

	gfx_attributes = DISPC_GFX_ATTRIBUTES_GFXBURSTSIZE_BURST16X32;

#if 0
	/* enable the graphics window only if its size is not zero */
	if (size_x > 0 && size_y > 0)
		gfx_attributes |= DISPC_GFX_ATTRIBUTES_ENABLE;
#endif 

	gfx_fifo_threshold =
		(RMODE_GFX_FIFO_HIGH_THRES << DISPC_GFX_FIFO_THRESHOLD_HIGH_SHIFT) |
		(RMODE_GFX_FIFO_LOW_THRES << DISPC_GFX_FIFO_THRESHOLD_LOW_SHIFT);

	gfx_position = 0;
	gfx_window_skip = 0;

	switch(color_depth) {
	case 1:
		gfx_format = DISPC_GFX_ATTRIBUTES_GFXFORMAT_BITMAP1;
		break;
	case 2:
		gfx_format = DISPC_GFX_ATTRIBUTES_GFXFORMAT_BITMAP2;
		break;
	case 4:
		gfx_format = DISPC_GFX_ATTRIBUTES_GFXFORMAT_BITMAP4;
		break;
	case 8:
		gfx_format = DISPC_GFX_ATTRIBUTES_GFXFORMAT_BITMAP8;
		break;
	case 12:
		gfx_format = DISPC_GFX_ATTRIBUTES_GFXFORMAT_RGB12;
		break;
	case 16:
		gfx_format = DISPC_GFX_ATTRIBUTES_GFXFORMAT_RGB16;
		break;
	case 24:
		gfx_format = DISPC_GFX_ATTRIBUTES_GFXFORMAT_RGB24;
		break;
	case 32:
		gfx_format = DISPC_GFX_ATTRIBUTES_GFXFORMAT_ARGB32;
		break;
	default:
		gfx_format = dispc_reg_in(DISPC_GFX_ATTRIBUTES)
			& DISPC_GFX_ATTRIBUTES_GFXFORMAT;
		break;
	}

	gfx_attributes |= gfx_format;

	dispc_reg_out(DISPC_GFX_FIFO_THRESHOLD, gfx_fifo_threshold);
	dispc_reg_out(DISPC_GFX_POSITION, gfx_position);
	dispc_reg_out(DISPC_GFX_WINDOW_SKIP, gfx_window_skip);

	dispc_reg_out(DISPC_CONFIG, config);
	dispc_reg_out(DISPC_GFX_ATTRIBUTES, gfx_attributes);

	layer[OMAP2_GRAPHICS].size_x = size_x;
	layer[OMAP2_GRAPHICS].size_y = size_y;
}

/* Calculate the number of pixels sent to the display per pixel clock as
 * (nom/den) pixels per clock.
 */
void
omap2_disp_pixels_per_clock(unsigned int *nom, unsigned int *den)
{
	u32 dispc_control;

	dispc_control = dispc_reg_in(DISPC_CONTROL);

	if (dispc_control & DISPC_CONTROL_STNTFT) {
		/* active display (TFT) */
		if (dispc_control & DISPC_CONTROL_TDMENABLE) {
			/* TFT with TDM */
			switch (dispc_control & DISPC_CONTROL_TDMCYCLEFORMAT) {
			case DISPC_CONTROL_TDMCYCLEFORMAT_1CYCPERPIX:
				*nom = 1;
				*den = 1;
				break;
			case DISPC_CONTROL_TDMCYCLEFORMAT_2CYCPERPIX:
				*nom = 1;
				*den = 2;
				break;
			case DISPC_CONTROL_TDMCYCLEFORMAT_3CYCPERPIX:
				*nom = 1;
				*den = 3;
				break;
			case DISPC_CONTROL_TDMCYCLEFORMAT_3CYCPER2PIX:
				*nom = 2;
				*den = 3;
				break;
			}
		}
		else {
			/* TFT without TDM */
			*nom = 1;
			*den = 1;
		}
	}
	else {
		/* passive display (STN) */
		if (dispc_control & DISPC_CONTROL_MONOCOLOR) {
			/* STN mono */
			if (dispc_control & DISPC_CONTROL_M8B) {
				/* 8 pixels per pixclock */
				*nom = 8;
				*den = 1;
			}
			else {
				/* 4 pixels per pixclock */
				*nom = 4;
				*den = 1;
			}
		}
		else {
			/* STN color--8 pixels per 3 pixclocks */
			*nom = 8;
			*den = 3;
		}
	}
}

/* Configure the signal configuration for LCD panel */
void
omap2_disp_lcdcfg_polfreq(u32 hsync_high, u32 vsync_high,
			  u32 acb,u32 ipc, u32 onoff,u32 rf)
{
	u32 pol_freq = 0;
	/* set the sync polarity */
	if (!hsync_high)
		pol_freq &=  ~DISPC_POL_FREQ_IHS;
	else
		pol_freq |=  DISPC_POL_FREQ_IHS;
	if (!vsync_high)
		pol_freq &= ~DISPC_POL_FREQ_IVS;
	else
		pol_freq |=  DISPC_POL_FREQ_IVS;

	if(!rf)
		pol_freq &= ~DISPC_POL_FREQ_RF;
	else
		pol_freq |=  DISPC_POL_FREQ_RF;
	
	pol_freq |= acb | (ipc << DISPC_POL_FREQ_IPC_SHIFT) |
		(onoff << DISPC_POL_FREQ_ONOFF_SHIFT) ;
	dispc_reg_out(DISPC_POL_FREQ, pol_freq);
}

/* Configure LCD parameters */
void omap2_disp_config_lcd(u32 clkdiv, u32 hbp, u32 hfp, u32 hsw, u32 vbp, u32 vfp, u32 vsw)
{
	u32 control, divisor, timing_h, timing_v;
#ifndef CONFIG_OMAP_DVI_SUPPORT
	divisor = (1 << DISPC_DIVISOR_LCD_SHIFT) 
		| (clkdiv << DISPC_DIVISOR_PCD_SHIFT);
#else
	divisor = (1 << DISPC_DIVISOR_LCD_SHIFT) 
		| (2 << DISPC_DIVISOR_PCD_SHIFT); 
#endif

	if (hbp > 255) hbp = 255;
	if (hfp > 255) hfp = 255; 
	if (hsw > 63)  hsw = 63;
	if (vbp > 255) vbp = 255;
	if (vfp > 255) vfp = 255;
	if (vsw > 63)  vsw = 63;

	timing_h = (hbp << DISPC_TIMING_H_HBP_SHIFT) | (hfp << DISPC_TIMING_H_HFP_SHIFT) 
		| (hsw << DISPC_TIMING_H_HSW_SHIFT);
	timing_v = (vbp << DISPC_TIMING_V_VBP_SHIFT) | (vfp << DISPC_TIMING_V_VFP_SHIFT)
		| (vsw << DISPC_TIMING_V_VSW_SHIFT);

#ifndef CONFIG_OMAP_DVI_SUPPORT
	dispc_reg_out(DISPC_TIMING_H, timing_h);
	dispc_reg_out(DISPC_TIMING_V, timing_v);
#else
	/* 720p@60Hz timings */
	dispc_reg_out(DISPC_TIMING_H, 0x06D0DB27/*0x0FF03F31*/);/*HBP=109+1, HFP=219+1,HSW=39+1*/
	dispc_reg_out(DISPC_TIMING_V, 0x01400504);/*VBP=20,VHP=5,VSW=4+1*/
#endif
	dispc_reg_out(DISPC_DIVISOR, divisor);
	control = dispc_reg_in(DISPC_CONTROL);
#ifdef CONFIG_MACH_OMAP3_EVM
	control |= DISPC_CONTROL_GPOUT1	| DISPC_CONTROL_GPOUT0 
	| DISPC_CONTROL_TFTDATALINES_OALSB18B | DISPC_CONTROL_STNTFT;
#else
	control |= DISPC_CONTROL_GPOUT1	| DISPC_CONTROL_GPOUT0 
	| DISPC_CONTROL_TFTDATALINES_OALSB16B | DISPC_CONTROL_STNTFT;
#endif	
	dispc_reg_out(DISPC_CONTROL, control);

#if 0
	printk("omap2_disp_config_lcd, clkdiv=%d, hbp=%d, hfp%d, hsw%d, vbp%d, vfp%d, vsw%d \n",
	        clkdiv, hbp, hfp, hsw, vbp, vfp, vsw);
	printk("contr %8x\n",dispc_reg_in(DISPC_CONTROL));
	printk("TIMINH_H 0x%x \n",dispc_reg_in(DISPC_TIMING_H));
	printk("TIMINH_V 0x%x \n",dispc_reg_in(DISPC_TIMING_V));
	printk("DIVISOR 0x%x \n",dispc_reg_in(DISPC_DIVISOR));
#endif
}

/* Set the pixel clock divisor for the LCD */
void
omap2_disp_set_pcd(u32 pcd)
{
	dispc_reg_out(DISPC_DIVISOR, (1 << DISPC_DIVISOR_LCD_SHIFT) |
			(pcd << DISPC_DIVISOR_PCD_SHIFT));
}

#ifndef CONFIG_OMAP_USE_DSI_PLL

/*
 * Set the DSS Functional clock
 * The DSS clock should be 4 times the Panel's Pixel clock
 * For TV the Pixel clock required is 13.5Mhz
 * For LCD the Pixel clock is 6Mhz
 */
void
omap2_disp_set_dssfclk(void)
{
	/* TODO set the LCD pixel clock rate based on the LCD configuration */
#if defined(CONFIG_MACH_OMAP_3430SDP) || defined(CONFIG_MACH_OMAP3_EVM)
	static int LCD_pixel_clk = 26000000; /* to get more bandwidth*/
#else	
	static int LCD_pixel_clk = 10000000; /* to get more bandwidth*/
#endif
#ifdef CONFIG_VIDEO_OMAP24XX_TVOUT
	static int TV_pixel_clk  = 14000000; /* rounded 13.5 to 14*/
#endif
	u32 ask_clkrate=0,sup_clkrate=0,tgt_clkrate=0/*,i*/;

#ifndef CONFIG_OMAP_DVI_SUPPORT
	ask_clkrate = LCD_pixel_clk * 4;
#else
	ask_clkrate = 148500000;
#endif

#ifdef CONFIG_VIDEO_OMAP24XX_TVOUT
	if(ask_clkrate < (TV_pixel_clk * 4))
		ask_clkrate = TV_pixel_clk * 4;
#endif

	//printk("omap2_disp_set_dssfclk = %ld\n",(long int)ask_clkrate);
	tgt_clkrate = ask_clkrate;

	sup_clkrate = clk_round_rate(dss1f_scale, ask_clkrate);
	if(is_sil_rev_less_than(OMAP3430_REV_ES2_0)){
		if(clk_get_rate(dss1f_scale) == 96000000){
			/*96M already, dont do anything for ES 1.0*/
			return;
		}
	}else 
	{
		for(i=1;i<=20;i++){
			sup_clkrate = clk_round_rate(dss1f_scale, ask_clkrate);
			if(sup_clkrate >= tgt_clkrate) break;
			ask_clkrate = ask_clkrate + 1000000;
		}
		if(clk_set_rate(dss1f_scale,sup_clkrate)==-EINVAL)
			printk(KERN_ERR "Unable to set the DSS"
					"functional clock to %d\n",sup_clkrate);
	}
}

#else /* CONFIG_OMAP_USE_DSI_PLL */

/*
 * DSI Proto Engine register I/O routines
 */
static __inline__ u32
dsiproto_reg_in(u32 offset)
{
	u32 val;
	val = omap_readl(DSI_PROTO_ENG_REG_BASE + offset);
	return  val;
}

static __inline__ u32
dsiproto_reg_out(u32 offset, u32 val)
{
	omap_writel(val, DSI_PROTO_ENG_REG_BASE + offset);
	return val;
}

/*
 * DSI PLL register I/O routines
 */
static __inline__ u32
dsipll_reg_in(u32 offset)
{
	u32 val;
	val = omap_readl(DSI_PLL_CONTROLLER_REG_BASE + offset);
	return  val;
}

static __inline__ u32
dsipll_reg_out(u32 offset, u32 val)
{
	omap_writel(val, DSI_PLL_CONTROLLER_REG_BASE + offset);
	return val;
}

static int omap34xx_lock_dsi_pll( u32 M, u32 N, u32 M3, u32 M4, u32 freqsel)
{
	u32 count = 1000, val1, val2;
	
	val1 =  ((M4<<23)|(M3<<19)|(M<<8)|(N<<1)|(1));
	
	val2 =  ((0<<20)|(0<<19)|(1<<18)|(0<<17)|(1<<16)|(0<<14)|
                        (1<<13)|(0<<12)|(0<<11)|(0<< 8)|(freqsel<<1));

	if (dsipll_reg_in( DSI_PLL_CONFIGURATION1 ) == val1 &&
	    dsipll_reg_in( DSI_PLL_CONFIGURATION2 ) == val2 &&
	    dsipll_reg_in(DSI_PLL_GO) == 0 &&
	    (dsipll_reg_in(DSI_PLL_STATUS) & 0x2) 
	) {
		printk("DSI PLL already programmed\n");
		return 1;
	}
 	
	dsipll_reg_out(DSI_PLL_CONFIGURATION1, val1);
	dsipll_reg_out(DSI_PLL_CONFIGURATION2, val2);
	
	dsipll_reg_out(DSI_PLL_GO, 1);

	while ((dsipll_reg_in(DSI_PLL_GO) != 0) && (--count)) 
		udelay(100);

	if (count == 0) { 
		printk ("GO bit not cleared\n");
		return 0;
	}	

	count = 1000;
	while (((dsipll_reg_in(DSI_PLL_STATUS) & 0x2 ) != 0x2) && (--count) )
		udelay(100);

	if (count == 0)  {
		printk ("DSI PLL lock request failed = %X\n", dsipll_reg_in(DSI_PLL_STATUS) );
		return 0;
	}

	return 1;	
}

static void omap34xx_switch_to_dsipll_clk_source (void) 
{
	u32 val;
	//Switch DISPC FCLK to DSI PLL HS divider
	val = dss_reg_in(DSS_CONTROL);
	val = val | (1<<1) | (1<<0);
	dss_reg_out(DSS_CONTROL, val);
}

static int omap34xx_power_dsi_pll (u32 cmd) 
{
	u32 val, count = 10000;
//printk("omap34xx_power_dsi_pll %x \n", cmd);
	/* send the power command */
 	val = dsiproto_reg_in(DSI_CLK_CTRL);
	val = ((val & ~(3 << 30)) | (cmd << 30));	
        dsiproto_reg_out(DSI_CLK_CTRL,val);		

	/* Check whether the power status is changed */ 
	do {
		val = dsiproto_reg_in(DSI_CLK_CTRL);
		val = ((val & 0x30000000) >> 28);
	    	udelay(100); 			
	} while( (val != cmd) && (--count));

	return count;
} 

static int omap3_disp_set_dsi_freq(u32 freq)
{
	int ret;
DBG printk("omap3_disp_set_dsi_freq %d \n", freq);	
	/* PLL settings based on 12MHz SYS_CLK:
	 * Fint = 2MHz -> N+1 = 6, freqsel = 7
	 */
	const u32 N = 5;
	const u32 freqsel = 7;
	const u32 sys_clk = 12000; /* kHz */
	const u32 regm = 7;
	
	u32 M = ( (regm+1) * freq * (N+1) ) / (sys_clk * 2);

	ret = omap34xx_lock_dsi_pll(M, N, regm, regm, freqsel);

	if (0) {
	  /* for debugging, compute the output frequency */
	  u32 dsi_clk = (2 * M * sys_clk) / ((N+1) * (regm+1));
	  printk(KERN_INFO "set_dsi_clkrate desired freq %lukHz, M=%lu N+1=%lu -> dsi_clk=%lukHz, ret=%i\n",
			(unsigned long)freq, (unsigned long)M, (unsigned long)N+1, (unsigned long)dsi_clk, ret);
	}
	
	return ret;
}

static int dsi_pll_pwr;

static void omap3_disp_start_dsi_pll(void)
{
	//printk(KERN_INFO "omap3_disp_start_dsi_pll ,dsi_clk_rate= %d\n",dsi_clk_rate);
	if (!dsi_pll_pwr) {
		//Command to change to ON state for both PLL and HSDIVISER
		//(no clock output to the DSI complex I/O)
		if (!omap34xx_power_dsi_pll(2)) {
			printk("Unable to power DSI PLL\n");
			return;
		}
		dsi_pll_pwr = 1;
	}

	if (!omap3_disp_set_dsi_freq(dsi_clk_rate / 1000)) {
		printk("FATAL ERROR: DSI PLL lock failed = %X\n",  dsipll_reg_in(DSI_PLL_STATUS));
		return;
	}
		
	omap34xx_switch_to_dsipll_clk_source();
}

static void omap3_disp_stop_dsi_pll(void)
{
	if (dsi_pll_pwr) {
		if (!omap34xx_power_dsi_pll(0)) {
			printk("Unable to shut down DSI PLL\n");
			return;
		}
		dsi_pll_pwr = 0;
	}
}

#endif /* CONFIG_OMAP_USE_DSI_PLL */


/* NOTE used to find the appropriate clkrate for a video out display (pixclock is in kHz) */
u32 omap2_disp_get_clkrate_g6_from_pixclock(u32 pixclock)
{
	/* 
		the dss1_alwon_clk we will use is DPll4/divisor
		the divisor value can be 1 to 16.
	        This clock value can be also divided during omap2_disp_config_lcd with the clkdiv value .
		clkdiv_value , set in the DISPC_DIVISOR can be 1->255
		example:
	PIXELCLOCK = 74,25Mhz > dss1_clkrate should be 216000000 to be divided by 3 -> 72Mhz, PCD = 1 ..
	We can tolerate +-5% on the pixelclock value, so 72MHz is acceptable.
	*/
	
	u32 clkrate = 27000000;
	
	switch(pixclock){
	case 25200:
		/*vga @60hz pixel clock*/
		clkrate = 5 * 25200000;
		break;
	case 27000:
		/*576p @50hz pixel clock*/
	case 27027:
		/*480p @60hz & analog tv pixel clock*/
		clkrate = 4 * 27000000;
		break;
	case 74250:
		/*720p pixel clock*/
		clkrate = 74250000;
		break;
	case 13500:
		/* for PAL display*/
		clkrate = 12 * 13500000;
		break;
	case 13513:
		/* for NTSC display*/
		clkrate = 12 * 13513000;
		break;
	default:
		//clkrate = pixclock * 2 * 1000/*kHz->Hz*/;
		break;
	}

DBG printk( "omap2_disp_get_clkrate_g6_from_pixclock = %ld -> %ld\n",pixclock, (long int)clkrate);
	return clkrate;
}

static void
omap24xx_ll_config_disp_clocks(int sleep_state)
{
#ifdef CONFIG_TRACK_RESOURCES
	struct device *dev = &display_dev;
#else
	struct device *dev = NULL;
#endif
	static int start = 1;
	
	//printk(KERN_DEBUG "omap24xx_ll_config_disp_clocks sleep_state=%d\n",sleep_state);

	if (start) {
#ifndef CONFIG_OMAP_USE_DSI_PLL
		/* when using the DSI pll, dss1_fck doesn't need to be adjusted */
		omap2_disp_set_dssfclk();
#endif
		dss1i = clk_get(dev, "dss_ick");
		dss1f = clk_get(dev, "dss1_fck");
		if(IS_ERR(dss1i) || IS_ERR(dss1f)) {
			printk("Could not get DSS clocks\n");
			return;
		}
#ifdef CONFIG_OMAP_USE_DSI_PLL
		dss2f = clk_get(dev, "dss2_fck");
		if(IS_ERR(dss2f)) {
			printk("Could not get DSS2 FCLK\n");
			return;
		}
#endif		
		start = 0;
	}
	if(sleep_state == 1){
#ifdef CONFIG_OMAP_USE_DSI_PLL
		omap3_disp_stop_dsi_pll();
		clk_disable(dss2f);
#endif
		clk_disable(dss1i);
		clk_disable(dss1f);
	}
	else {
		if(clk_enable(dss1i) != 0) {
			printk("Unable to enable DSS ICLK\n");
			return;
		}
		if(clk_enable(dss1f) != 0) {
			printk("Unable to enable DSS FCLK\n");
			return;
		}
#ifdef CONFIG_OMAP_USE_DSI_PLL
		if(clk_enable(dss2f) != 0) {
			printk("Unable to enable DSS FCLK\n");
			return;
		}
		omap3_disp_start_dsi_pll();
#endif
	}
}

void omap2_disp_set_clkrate_g6(u32 clkrate)
{
#ifdef CONFIG_MACH_ARCHOS_G6
#ifdef CONFIG_OMAP_USE_DSI_PLL
	/* if the pll is already on, program a new frequency,
	 * if it's off, just remember the clock rate, it will
	 * be used when the display restarts next time */
	dsi_clk_rate = clkrate;
	if (dsi_pll_pwr) {
		if (!omap3_disp_set_dsi_freq(dsi_clk_rate / 1000)) {
			printk("FATAL ERROR: DSI PLL lock failed = %X\n",
				dsipll_reg_in(DSI_PLL_STATUS));
		}
	} else 
		printk("dsi_pll_pwr is off, remember clkrate %d\n", clkrate);
#else
	/* store this static value */
	dss_clk_rate = clkrate;
	omap2_disp_set_dssfclk(); /* This will not work due to a couple of constrains on when to call this */
#endif /*CONFIG_OMAP_USE_DSI_PLL*/
#endif
}

u32 omap2_disp_get_dssfclk(void)
{
#ifndef CONFIG_OMAP_USE_DSI_PLL
	return clk_get_rate(dss1f);
#else
	return dsi_clk_rate;
#endif
}


void omap2_disp_check_dispusage(void)
{
	if ( disp_usage != 1 )
		printk("\nALERT: disp_usage is %d and should be 1!!!!\n", disp_usage);
}

/* This function must be called by any driver that needs to use the display
 * controller before calling any routine that accesses the display controller
 * registers. It increments the count of the number of users of the display
 * controller, and turns the clocks ON only when required.
 */
void
omap2_disp_get_all_clks(void)
{
	u32 idle_dispc;
#ifdef CONFIG_HW_SUP_TRANS
	u32 idle_dss;
#endif /* #ifdef CONFIG_HW_SUP_TRANS */

	spin_lock(&dss_lock);
	if (disp_usage == 0) {
		/* turn on DSS clock */
		omap24xx_ll_config_disp_clocks(0);
#ifndef CONFIG_ARCH_OMAP3410
		omap2_disp_set_tvref(TVREF_ON);
		omap24xx_ll_config_tv_clocks(0);
#endif
#ifdef CONFIG_OMAP34XX_OFFMODE
		/* Set the TV standard first */
#ifndef CONFIG_ARCH_OMAP3410
		omap2_disp_set_tvstandard(omap2_current_tvstandard);
#endif
		/* restore dss context */
		omap2_disp_restore_ctx(OMAP_DSS_GENERIC);
		omap2_disp_restore_ctx(OMAP_DSS_DISPC_GENERIC);
		omap2_disp_restore_ctx(OMAP2_GRAPHICS);
		omap2_disp_restore_ctx(OMAP2_VIDEO1);
		omap2_disp_restore_ctx(OMAP2_VIDEO2);

#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */			
#ifdef CONFIG_HW_SUP_TRANS
		/* Set smart idle for Display subsystem */
		idle_dss = dss_reg_in(DSS_SYSCONFIG);
		idle_dss |= DSS_SYSCONFIG_AUTOIDLE;
		dss_reg_out(DSS_SYSCONFIG, idle_dss);
#endif /* #ifdef CONFIG_HW_SUP_TRANS */

		/* Set smart idle, autoidle for Display controller */
		idle_dispc = dispc_reg_in(DISPC_SYSCONFIG);
		idle_dispc &= ~(DISPC_SYSCONFIG_MIDLEMODE | 
				DISPC_SYSCONFIG_SIDLEMODE);

#ifdef CONFIG_HW_SUP_TRANS
		idle_dispc |= (DISPC_SYSCONFIG_MIDLEMODE_SSTANDBY |
				DISPC_SYSCONFIG_SIDLEMODE_SIDLE |
				DISPC_SYSCONFIG_ENABLE_WKUP);
		idle_dispc |= DISPC_SYSCONFIG_AUTOIDLE;
#else
		idle_dispc |= DISPC_SYSCONFIG_MIDLEMODE_NSTANDBY |
			DISPC_SYSCONFIG_SIDLEMODE_NIDLE;
#endif /* #ifdef CONFIG_HW_SUP_TRANS */

		dispc_reg_out(DISPC_SYSCONFIG, idle_dispc);
#ifdef CONFIG_OMAP34XX_OFFMODE
		dispc_reg_out(DISPC_CONTROL, dss_ctx.dispc.control);
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */
	}
	else {
		/* enable the TV clocks, since we are not if they are */
#ifndef CONFIG_ARCH_OMAP3410
		omap2_disp_set_tvref(TVREF_ON);
		omap24xx_ll_config_tv_clocks(0);
		{
			omap2_disp_set_tvstandard(omap2_current_tvstandard);
		}
#endif
	}
	disp_usage++;
	spin_unlock(&dss_lock);
}

/* This function must be called by a driver when it not going to use the
 * display controller anymore. E.g., when a driver suspends, it must call
 * omap2_disp_put_dss. When it wakes up, it must call omap2_disp_get_dss again.
 * It decrements the count of the number of users of the display
 * controller, and turns the clocks OFF when not required.
 */
void
omap2_disp_put_all_clks(void)
{
#ifndef CONFIG_HW_SUP_TRANS
	u32 idle_dss;	
#endif /* #ifndef CONFIG_HW_SUP_TRANS */

	spin_lock(&dss_lock);
	if (disp_usage == 0) {
		printk(KERN_ERR "trying to put DSS when usage count is zero\n");
		spin_unlock(&dss_lock);
		return;
	}

	disp_usage--;

	if (disp_usage == 0) {
#ifdef CONFIG_OMAP34XX_OFFMODE
		/* save dss context */
		omap2_disp_save_ctx(OMAP_DSS_GENERIC);
		omap2_disp_save_ctx(OMAP_DSS_DISPC_GENERIC);
		omap2_disp_save_ctx(OMAP2_GRAPHICS);
		omap2_disp_save_ctx(OMAP2_VIDEO1);
		omap2_disp_save_ctx(OMAP2_VIDEO2);
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */
#ifndef CONFIG_HW_SUP_TRANS
		idle_dss = dispc_reg_in(DISPC_SYSCONFIG);
		idle_dss &= ~(DISPC_SYSCONFIG_MIDLEMODE | DISPC_SYSCONFIG_SIDLEMODE);
		idle_dss |= DISPC_SYSCONFIG_MIDLEMODE_SSTANDBY |
			DISPC_SYSCONFIG_SIDLEMODE_SIDLE;
		dispc_reg_out(DISPC_SYSCONFIG, idle_dss);
#endif /* #ifdef CONFIG_HW_SUP_TRANS */

		omap2_disp_disable(HZ/5);
		/* turn off TV clocks */
#ifndef CONFIG_ARCH_OMAP3410
		omap24xx_ll_config_tv_clocks(1);
		omap2_disp_set_tvref(TVREF_OFF);
#endif
		mdelay(4);

		omap24xx_ll_config_disp_clocks(1);
	}
	spin_unlock(&dss_lock);
}

/* This function must be called by any driver that needs to use the display
 * controller before calling any routine that accesses the display controller
 * registers. It increments the count of the number of users of the display
 * controller, and turns the clocks ON only when required.
 */
void
omap2_disp_get_dss(void)
{
	u32 idle_dispc;
#ifdef CONFIG_HW_SUP_TRANS
	u32 idle_dss;
#endif /* #ifdef CONFIG_HW_SUP_TRANS */

	spin_lock(&dss_lock);
	
	//printk(KERN_DEBUG "omap2_disp_get_dss disp_usage %i\n", disp_usage);

	if (disp_usage == 0) {
		/* turn on DSS clock */
		omap24xx_ll_config_disp_clocks(0);
#ifndef CONFIG_ARCH_OMAP3410

		if ((omap2_disp_get_output_dev(OMAP2_GRAPHICS) == OMAP2_OUTPUT_TV) ||
				(omap2_disp_get_output_dev(OMAP2_VIDEO1) == OMAP2_OUTPUT_TV) ||
				(omap2_disp_get_output_dev(OMAP2_VIDEO2) == OMAP2_OUTPUT_TV))
		{	
			omap2_disp_set_tvref(TVREF_ON);
			omap24xx_ll_config_tv_clocks(0);
#ifdef CONFIG_OMAP34XX_OFFMODE
			omap2_disp_set_tvstandard(omap2_current_tvstandard);
#endif	   
		}
#endif
#ifdef CONFIG_OMAP34XX_OFFMODE
		/* restore dss context */
		omap2_disp_restore_ctx(OMAP_DSS_GENERIC);
		omap2_disp_restore_ctx(OMAP_DSS_DISPC_GENERIC);
		omap2_disp_restore_ctx(OMAP2_GRAPHICS);
		omap2_disp_restore_ctx(OMAP2_VIDEO1);
		omap2_disp_restore_ctx(OMAP2_VIDEO2);

#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */			
#ifdef CONFIG_HW_SUP_TRANS
		/* Set smart idle for Display subsystem */
		idle_dss = dss_reg_in(DSS_SYSCONFIG);
		idle_dss |= DSS_SYSCONFIG_AUTOIDLE;
		dss_reg_out(DSS_SYSCONFIG, idle_dss);
#endif /* #ifdef CONFIG_HW_SUP_TRANS */

		/* Set smart idle, autoidle for Display controller */
		idle_dispc = dispc_reg_in(DISPC_SYSCONFIG);
		idle_dispc &= ~(DISPC_SYSCONFIG_MIDLEMODE | 
				DISPC_SYSCONFIG_SIDLEMODE);

#ifdef CONFIG_HW_SUP_TRANS
		idle_dispc |= (DISPC_SYSCONFIG_MIDLEMODE_SSTANDBY |
				DISPC_SYSCONFIG_SIDLEMODE_SIDLE |
				DISPC_SYSCONFIG_ENABLE_WKUP);
		idle_dispc |= DISPC_SYSCONFIG_AUTOIDLE;
#else
		idle_dispc |= DISPC_SYSCONFIG_MIDLEMODE_NSTANDBY |
			DISPC_SYSCONFIG_SIDLEMODE_NIDLE;
#endif /* #ifdef CONFIG_HW_SUP_TRANS */

		dispc_reg_out(DISPC_SYSCONFIG, idle_dispc);
#ifdef CONFIG_OMAP34XX_OFFMODE
		dispc_reg_out(DISPC_CONTROL, dss_ctx.dispc.control);
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */
	}
	disp_usage++;
	spin_unlock(&dss_lock);
}

/* This function must be called by a driver when it not going to use the
 * display controller anymore. E.g., when a driver suspends, it must call
 * omap2_disp_put_dss. When it wakes up, it must call omap2_disp_get_dss again.
 * It decrements the count of the number of users of the display
 * controller, and turns the clocks OFF when not required.
 */
void
omap2_disp_put_dss(void)
{
#ifndef CONFIG_HW_SUP_TRANS
	u32 idle_dss;
#endif /* #ifndef CONFIG_HW_SUP_TRANS */

	spin_lock(&dss_lock);


	if (disp_usage == 0) {
		printk(KERN_ERR "trying to put DSS when usage count is zero\n");
		spin_unlock(&dss_lock);
		return;
	}

	disp_usage--;

	if (disp_usage == 0) {
#ifdef CONFIG_OMAP34XX_OFFMODE
		/* save dss context */
		omap2_disp_save_ctx(OMAP_DSS_GENERIC);
		omap2_disp_save_ctx(OMAP_DSS_DISPC_GENERIC);
		omap2_disp_save_ctx(OMAP2_GRAPHICS);
		omap2_disp_save_ctx(OMAP2_VIDEO1);
		omap2_disp_save_ctx(OMAP2_VIDEO2);
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */
#ifndef CONFIG_HW_SUP_TRANS
		idle_dss = dispc_reg_in(DISPC_SYSCONFIG);
		idle_dss &= ~(DISPC_SYSCONFIG_MIDLEMODE | DISPC_SYSCONFIG_SIDLEMODE);
		idle_dss |= DISPC_SYSCONFIG_MIDLEMODE_SSTANDBY |
				DISPC_SYSCONFIG_SIDLEMODE_SIDLE;
		dispc_reg_out(DISPC_SYSCONFIG, idle_dss);
#endif /* #ifdef CONFIG_HW_SUP_TRANS */

		omap2_disp_disable(HZ/5);
#ifndef CONFIG_ARCH_OMAP3410
		// printk("put_dss: Disable TV clocks \n");
		omap24xx_ll_config_tv_clocks(1);
		omap2_disp_set_tvref(TVREF_OFF);
#endif
		mdelay(4);
		omap24xx_ll_config_disp_clocks(1);
	}
	spin_unlock(&dss_lock);
}

/* This function must be called by any driver that wishes to use a particular
 * display pipeline (layer).
 */
int
omap2_disp_request_layer(int ltype)
{
	int ret = 0;

	spin_lock(&dss_lock);
	if (!layer[ltype].in_use) {
		layer[ltype].in_use = 1;
		ret = 1;
	}
	spin_unlock(&dss_lock);

	return ret;
}

/* This function must be called by a driver when it is done using a particular
 * display pipeline (layer).
 */
void
omap2_disp_release_layer(int ltype)
{
	spin_lock(&dss_lock);
	layer[ltype].in_use = 0;
	layer[ltype].ctx_valid = 0;
	spin_unlock(&dss_lock);
}

/* Used to enable LCDENABLE or DIGITALENABLE of the display controller.
 */
void
omap2_disp_enable_output_dev(int output_dev)
{
	if (output_dev == OMAP2_OUTPUT_LCD) {
		dispc_reg_merge(DISPC_CONTROL, DISPC_CONTROL_LCDENABLE,
				DISPC_CONTROL_LCDENABLE);
	}
#ifndef CONFIG_ARCH_OMAP3410
	else if (output_dev == OMAP2_OUTPUT_TV) {
		dispc_reg_merge(DISPC_CONTROL, DISPC_CONTROL_DIGITALENABLE,
				DISPC_CONTROL_DIGITALENABLE);
	}
#endif
}

/* Used to disable LCDENABLE or DIGITALENABLE of the display controller.
 */
void
omap2_disp_disable_output_dev(int output_dev)
{
	if (output_dev == OMAP2_OUTPUT_LCD) {
		dispc_reg_merge(DISPC_CONTROL, ~DISPC_CONTROL_LCDENABLE,
				DISPC_CONTROL_LCDENABLE);
	}
#ifndef CONFIG_ARCH_OMAP3410
	else if (output_dev == OMAP2_OUTPUT_TV) {
		dispc_reg_merge(DISPC_CONTROL, ~DISPC_CONTROL_DIGITALENABLE,
				DISPC_CONTROL_DIGITALENABLE);
	}
#endif
}

int
omap2_disp_get_output_dev(int ltype)
{
	return layer[ltype].output_dev;
}

int omap2_disp_get_gfx_fifo_low_threshold(void)
{
	return ((dispc_reg_in(DISPC_GFX_FIFO_THRESHOLD) &
				DISPC_GFX_FIFO_THRESHOLD_LOW) >>
			DISPC_GFX_FIFO_THRESHOLD_LOW_SHIFT);
}

void omap2_disp_set_gfx_fifo_low_threshold(int thrs)
{
	dispc_reg_merge(DISPC_GFX_FIFO_THRESHOLD,
			thrs << DISPC_GFX_FIFO_THRESHOLD_LOW_SHIFT,
			DISPC_GFX_FIFO_THRESHOLD_LOW);

	omap2_disp_reg_sync( OMAP2_OUTPUT_LCD );
	wait_for_reg_sync(OMAP2_OUTPUT_LCD, DEFAULT_TIMEOUT);
}

int omap2_disp_get_gfx_fifo_high_threshold(void)
{
	return ((dispc_reg_in(DISPC_GFX_FIFO_THRESHOLD) &
				DISPC_GFX_FIFO_THRESHOLD_HIGH) >>
			DISPC_GFX_FIFO_THRESHOLD_HIGH_SHIFT);
}

void omap2_disp_set_gfx_fifo_high_threshold(int thrs)
{
	dispc_reg_merge(DISPC_GFX_FIFO_THRESHOLD,
			thrs << DISPC_GFX_FIFO_THRESHOLD_HIGH_SHIFT,
			DISPC_GFX_FIFO_THRESHOLD_HIGH);

	omap2_disp_reg_sync( OMAP2_OUTPUT_LCD );
	wait_for_reg_sync(OMAP2_OUTPUT_LCD, DEFAULT_TIMEOUT);
}

/* This is used to dynamically switch the output of a particular layer to
 * either the LCD or TV.
 */
void
omap2_disp_set_output_dev(int ltype, int output_dev)
{
	struct omap2_disp_dma_params *dma_param = 0;
	int vid_pic_size = 0;

	layer[ltype].output_dev = output_dev;
DBG printk("omap2_disp_set_output_dev: ltype %d  output_dev %d\r\n", ltype, output_dev );  
	switch(ltype) {
	case OMAP2_GRAPHICS:
		if (layer[ltype].in_use) {
			if (output_dev == OMAP2_OUTPUT_LCD ) {
				int gfx_size = 0;
				dma_param = &layer[OMAP2_GRAPHICS].dma[0];

				if( _lcd_interlaced() ) {
					gfx_size =   (((layer[OMAP2_GRAPHICS].size_x     - 1) << DISPC_GFX_SIZE_GFXSIZEX_SHIFT) & DISPC_GFX_SIZE_GFXSIZEX)
						   | (((layer[OMAP2_GRAPHICS].size_y / 2 - 1) << DISPC_GFX_SIZE_GFXSIZEY_SHIFT) & DISPC_GFX_SIZE_GFXSIZEY);
				} else {
					gfx_size =   (((layer[OMAP2_GRAPHICS].size_x - 1) << DISPC_GFX_SIZE_GFXSIZEX_SHIFT) & DISPC_GFX_SIZE_GFXSIZEX)
						   | (((layer[OMAP2_GRAPHICS].size_y - 1) << DISPC_GFX_SIZE_GFXSIZEY_SHIFT) & DISPC_GFX_SIZE_GFXSIZEY);
				}
				dispc_reg_out(DISPC_GFX_SIZE, gfx_size);
			
			}
#ifndef CONFIG_ARCH_OMAP3410
			else if (output_dev == OMAP2_OUTPUT_TV) {
				int gfx_size = 0;
				dma_param = &layer[OMAP2_GRAPHICS].dma[1];
				/* dividing the size_y by two,
				 * because TV operates in interleaved mode
				 */
				gfx_size = (  ((layer[OMAP2_GRAPHICS].size_x     - 1) << DISPC_GFX_SIZE_GFXSIZEX_SHIFT) & DISPC_GFX_SIZE_GFXSIZEX)
					   | (((layer[OMAP2_GRAPHICS].size_y / 2 - 1) << DISPC_GFX_SIZE_GFXSIZEY_SHIFT) & DISPC_GFX_SIZE_GFXSIZEY);
					      
			/* move graphics display position to cover
	 		 * TV overscan
	 		 */
				dispc_reg_out(DISPC_GFX_SIZE, gfx_size);
			}
#endif


			dispc_reg_out(DISPC_GFX_BA0, dma_param->ba0);
			dispc_reg_out(DISPC_GFX_BA1, dma_param->ba1);
			dispc_reg_out(DISPC_GFX_ROW_INC, dma_param->row_inc);
			dispc_reg_out(DISPC_GFX_PIXEL_INC, dma_param->pix_inc);
		}
		
		/* AK: this needs to be done even if the layer is not in use...
		   omap2_disp_enable_layer/omap2_disp_disable_layer relies on it to be set correctly */
		dispc_reg_merge(DISPC_GFX_ATTRIBUTES,
				(output_dev == OMAP2_OUTPUT_LCD) ? 0 : DISPC_GFX_ATTRIBUTES_GFXCHANNELOUT,
				DISPC_GFX_ATTRIBUTES_GFXCHANNELOUT);

		/* AK: why only for graphics and not for video, too??!? 
		   Does this mean that the access to gfx should come *after* vid1/2? */
		if (layer[ltype].in_use) {
			omap2_disp_reg_sync(output_dev);
			if ( omap2_disp_check_enabled( output_dev ))
				wait_for_reg_sync(output_dev, DEFAULT_TIMEOUT);
		}
		break;
	
	case OMAP2_VIDEO1:
		if (layer[ltype].in_use){
			tvlcd_status.output_dev = output_dev;
			tvlcd_status.ltype = ltype;
			tvlcd_status.status = TVLCD_STOP;
			if (output_dev == OMAP2_OUTPUT_LCD) {
				dma_param = &layer[OMAP2_VIDEO1].dma[0];
				vid_pic_size =
					(((layer[OMAP2_VIDEO1].size_x - 1) << DISPC_VID_PICTURE_SIZE_VIDORGSIZEX_SHIFT) & DISPC_VID_PICTURE_SIZE_VIDORGSIZEX) |
					(((layer[OMAP2_VIDEO1].size_y - 1) << DISPC_VID_PICTURE_SIZE_VIDORGSIZEY_SHIFT) & DISPC_VID_PICTURE_SIZE_VIDORGSIZEY);
				dispc_reg_out(DISPC_VID_PICTURE_SIZE(0), vid_pic_size);
				}
#ifndef CONFIG_ARCH_OMAP3410
			 else if (output_dev == OMAP2_OUTPUT_TV) {
				dma_param = &layer[OMAP2_VIDEO1].dma[1];
				vid_pic_size =
					(((layer[OMAP2_VIDEO1].size_x     - 1) << DISPC_VID_PICTURE_SIZE_VIDORGSIZEX_SHIFT) & DISPC_VID_PICTURE_SIZE_VIDORGSIZEX) |
					(((layer[OMAP2_VIDEO1].size_y / 2 - 1) << DISPC_VID_PICTURE_SIZE_VIDORGSIZEY_SHIFT) & DISPC_VID_PICTURE_SIZE_VIDORGSIZEY);
				dispc_reg_out(DISPC_VID_PICTURE_SIZE(0), vid_pic_size);
			}
#endif

			dispc_reg_out(DISPC_VID_BA0(0), dma_param->ba0);
			dispc_reg_out(DISPC_VID_BA1(0), dma_param->ba1);
			dispc_reg_out(DISPC_VID_ROW_INC(0), dma_param->row_inc);
			dispc_reg_out(DISPC_VID_PIXEL_INC(0), dma_param->pix_inc);
			
			dispc_reg_merge(DISPC_VID_ATTRIBUTES(0),
				(output_dev == OMAP2_OUTPUT_LCD) ? 0 : DISPC_VID_ATTRIBUTES_VIDCHANNELOUT,
				DISPC_VID_ATTRIBUTES_VIDCHANNELOUT);
			break;
		}

	case OMAP2_VIDEO2:
		if (layer[ltype].in_use){
	
			tvlcd_status.output_dev = output_dev;
			tvlcd_status.ltype = ltype;
			tvlcd_status.status = TVLCD_STOP;
			if (output_dev == OMAP2_OUTPUT_LCD) {
				dma_param = &layer[OMAP2_VIDEO2].dma[0];
				vid_pic_size =
					(((layer[OMAP2_VIDEO2].size_x - 1) << DISPC_VID_PICTURE_SIZE_VIDORGSIZEX_SHIFT) & DISPC_VID_PICTURE_SIZE_VIDORGSIZEX) |
					(((layer[OMAP2_VIDEO2].size_y - 1) << DISPC_VID_PICTURE_SIZE_VIDORGSIZEY_SHIFT)	& DISPC_VID_PICTURE_SIZE_VIDORGSIZEY);
				dispc_reg_out(DISPC_VID_PICTURE_SIZE(1), vid_pic_size);
			}
#ifndef CONFIG_ARCH_OMAP3410
			 else if (output_dev == OMAP2_OUTPUT_TV) {
				dma_param = &layer[OMAP2_VIDEO2].dma[1];
				vid_pic_size =
					(((layer[OMAP2_VIDEO2].size_x     - 1) << DISPC_VID_PICTURE_SIZE_VIDORGSIZEX_SHIFT) & DISPC_VID_PICTURE_SIZE_VIDORGSIZEX) |
					(((layer[OMAP2_VIDEO2].size_y / 2 - 1) << DISPC_VID_PICTURE_SIZE_VIDORGSIZEY_SHIFT) & DISPC_VID_PICTURE_SIZE_VIDORGSIZEY);
				dispc_reg_out(DISPC_VID_PICTURE_SIZE(1), vid_pic_size);
			}
#endif		
			dispc_reg_out(DISPC_VID_BA0(1), dma_param->ba0);
			dispc_reg_out(DISPC_VID_BA1(1), dma_param->ba1);
			dispc_reg_out(DISPC_VID_ROW_INC(1), dma_param->row_inc);
			dispc_reg_out(DISPC_VID_PIXEL_INC(1), dma_param->pix_inc);
			
			dispc_reg_merge(DISPC_VID_ATTRIBUTES(1),
				(output_dev == OMAP2_OUTPUT_LCD) ? 0 : DISPC_VID_ATTRIBUTES_VIDCHANNELOUT,
				DISPC_VID_ATTRIBUTES_VIDCHANNELOUT);
			break;
		}
	}
}

/* Used to save the DMA parameter settings for a particular layer to be
 * displayed on a particular output device. These values help the
 * omap2_disp_set_output_dev() function to dynamically switch the output of a
 * layer to any output device.
 */
void
omap2_disp_set_dma_params(int ltype, int output_dev,
			  u32 ba0, u32 ba1, u32 row_inc, u32 pix_inc)
{
	struct omap2_disp_dma_params *dma;

DBG 	printk("omap2_disp_set_dma_params: ltype %d  output_dev %d  ba0 %#x  ba1 %#x  row_inc %d  pix_inc %d\n", ltype, output_dev, ba0, ba1, row_inc, pix_inc);
	
	if (output_dev == OMAP2_OUTPUT_LCD)
		dma = &layer[ltype].dma[0];
	else
		dma = &layer[ltype].dma[1];

	dma->ba0 = ba0;
	dma->ba1 = ba1;
	dma->row_inc = row_inc;
	dma->pix_inc = pix_inc;
}

void
omap2_disp_start_gfxlayer(void)
{
	omap2_disp_set_output_dev(OMAP2_GRAPHICS, 
			layer[OMAP2_GRAPHICS].output_dev);
	omap2_disp_enable_layer(OMAP2_GRAPHICS);
}

/* Sets the background color */
void
omap2_disp_set_bg_color(int output_dev, int color)
{
	if (output_dev == OMAP2_OUTPUT_LCD)
		dispc_reg_out(DISPC_DEFAULT_COLOR0, color);
#ifndef CONFIG_ARCH_OMAP3410
	else if (output_dev == OMAP2_OUTPUT_TV)
		dispc_reg_out(DISPC_DEFAULT_COLOR1, color);
#endif

/* AK: we don't do this here - this is called in early in init and turns on GODIGITAL for good...
	omap2_disp_reg_sync(output_dev);
	wait_for_reg_sync(output_dev, DEFAULT_TIMEOUT);
*/
}

/* Returns the current background color */
void
omap2_disp_get_bg_color(int output_dev, int *color)
{
	if (output_dev == OMAP2_OUTPUT_LCD)
		*color = dispc_reg_in(DISPC_DEFAULT_COLOR0);
#ifndef CONFIG_ARCH_OMAP3410
	else if (output_dev == OMAP2_OUTPUT_TV)
		*color = dispc_reg_in(DISPC_DEFAULT_COLOR1);
#endif
}

/* Enable/Disable the Dithering block */
void
omap2_disp_set_dithering(int dither_state)
{
	omap2_disp_get_dss();	
	switch(dither_state){
	case DITHERING_ON:
		dispc_reg_out(DISPC_CONTROL,
			(dispc_reg_in(DISPC_CONTROL) | DISPC_CONTROL_TFTDITHERENABLE));
	break;
	case DITHERING_OFF:
		dispc_reg_out(DISPC_CONTROL,
			(dispc_reg_in(DISPC_CONTROL) & ~DISPC_CONTROL_TFTDITHERENABLE));
	break;
	}
	omap2_disp_reg_sync(OMAP2_OUTPUT_LCD);
	wait_for_reg_sync(OMAP2_OUTPUT_LCD, DEFAULT_TIMEOUT);
	// omap2_disp_reg_sync(OMAP2_OUTPUT_TV);
	omap2_disp_put_dss();	
}

/* Get the Dithering state */
int
omap2_disp_get_dithering(void)
{
	omap2_disp_get_dss();	
	if(dispc_reg_in(DISPC_CONTROL) & DISPC_CONTROL_TFTDITHERENABLE){
		return(DITHERING_ON);
	}
	else{
		return(DITHERING_OFF);
	}
	omap2_disp_put_dss();	
}

/* Get the number of data lines connected to LCD panel*/
int
omap2_disp_get_lcddatalines(void)
{
	u32 tft_data_lines=0;

	omap2_disp_get_dss();	
	tft_data_lines = dispc_reg_in(DISPC_CONTROL) 
		& (DISPC_CONTROL_TFTDATALINES);

	switch(tft_data_lines){
	case DISPC_CONTROL_TFTDATALINES_OALSB12B:
		return(LCD_DATA_LINE_12BIT);
	case DISPC_CONTROL_TFTDATALINES_OALSB16B:
		return(LCD_DATA_LINE_16BIT);
	case DISPC_CONTROL_TFTDATALINES_OALSB18B:
		return(LCD_DATA_LINE_18BIT);
	case DISPC_CONTROL_TFTDATALINES_OALSB24B:
		return(LCD_DATA_LINE_24BIT);
	}
	omap2_disp_put_dss();	
	return(LCD_DATA_LINE_16BIT);
}

/* Set number of data lines to be connected to LCD panel*/
void
omap2_disp_set_lcddatalines(int no_of_lines)
{
	omap2_disp_get_dss();	
	dispc_reg_out(DISPC_CONTROL,
		(dispc_reg_in(DISPC_CONTROL) & ~DISPC_CONTROL_TFTDATALINES));
		
	switch(no_of_lines){
	case LCD_DATA_LINE_12BIT:
		dispc_reg_out(DISPC_CONTROL,
			(dispc_reg_in(DISPC_CONTROL) 
				| DISPC_CONTROL_TFTDATALINES_OALSB12B));
	break;
	case LCD_DATA_LINE_16BIT:
		dispc_reg_out(DISPC_CONTROL,
			(dispc_reg_in(DISPC_CONTROL) 
				| DISPC_CONTROL_TFTDATALINES_OALSB16B));
	break;
	case LCD_DATA_LINE_18BIT:
		dispc_reg_out(DISPC_CONTROL,
			(dispc_reg_in(DISPC_CONTROL) 
					| DISPC_CONTROL_TFTDATALINES_OALSB18B));
	break;
	case LCD_DATA_LINE_24BIT:
		dispc_reg_out(DISPC_CONTROL,
			(dispc_reg_in(DISPC_CONTROL) 
					| DISPC_CONTROL_TFTDATALINES_OALSB24B));
	break;
	}
	omap2_disp_put_dss();	
}

#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3430) && !defined(CONFIG_ARCH_OMAP3410)
/* Turn on/off the TV reference voltage from OMAP */
void
omap2_disp_set_tvref(int tvref_state)
{
	switch(tvref_state) {
	case TVREF_ON:
		dss_reg_out(DSS_CONTROL,(dss_reg_in(DSS_CONTROL)
					| DSS_CONTROL_TV_REF));
		break;
	case TVREF_OFF:
		dss_reg_out(DSS_CONTROL, (dss_reg_in(DSS_CONTROL) &
					~(DSS_CONTROL_TV_REF)));
		break;
	}
}
#endif

/* Sets the SMS settings for rotation using the VRFB.
 */
int
omap2_disp_set_vrfb(int context, u32 phy_addr,
			u32 width, u32 height, u32 bytes_per_pixel)
{
	int page_width_exp, page_height_exp, pixel_size_exp;

	if (bytes_per_pixel > 4)
		return -EINVAL;

	page_width_exp = PAGE_WIDTH_EXP;
	page_height_exp = PAGE_HEIGHT_EXP;
	pixel_size_exp = bytes_per_pixel >> 1;

	width = ((1<<page_width_exp) *
		 (pages_per_side(width * bytes_per_pixel, page_width_exp))
		) >> pixel_size_exp;	// in pixels

	height = (1<<page_height_exp) *
		 (pages_per_side(height, page_height_exp));
	
	SMS_ROT0_PHYSICAL_BA(context) = phy_addr;
	SMS_ROT0_SIZE(context) = 0;
	SMS_ROT0_SIZE(context)	|= (width << SMS_IMAGEWIDTH_OFFSET)
				| (height << SMS_IMAGEHEIGHT_OFFSET);
	SMS_ROT_CONTROL(context) = 0;
	
	SMS_ROT_CONTROL(context) |= pixel_size_exp << SMS_PS_OFFSET
			| (page_width_exp - pixel_size_exp) << SMS_PW_OFFSET
			| page_height_exp << SMS_PH_OFFSET;
	
	return 0;
}

#ifndef CONFIG_ARCH_OMAP3410
/* Sets VENC registers for TV operation.
*/
static void
config_venc(struct tv_standard_config *tvstd)
{
	venc_reg_out(VENC_F_CONTROL, tvstd->venc_f_ctrl);
	venc_reg_out(VENC_SYNC_CONTROL,tvstd->venc_sync_ctrl);
	venc_reg_out(VENC_LLEN, tvstd->venc_llen);
	venc_reg_out(VENC_FLENS, tvstd->venc_flens);
	venc_reg_out(VENC_HFLTR_CTRL, tvstd->venc_hfltr_ctrl);
	venc_reg_out(VENC_CC_CARR_WSS_CARR, tvstd->venc_cc_carr_wss_carr);
	venc_reg_out(VENC_C_PHASE, tvstd->venc_c_phase);
	venc_reg_out(VENC_GAIN_U, tvstd->venc_gain_u);
	venc_reg_out(VENC_GAIN_V, tvstd->venc_gain_v);
	venc_reg_out(VENC_GAIN_Y, tvstd->venc_gain_y);
	venc_reg_out(VENC_BLACK_LEVEL, tvstd->venc_black_level);
	venc_reg_out(VENC_BLANK_LEVEL, tvstd->venc_blank_level);
	venc_reg_out(VENC_X_COLOR, tvstd->venc_x_color);
	venc_reg_out(VENC_M_CONTROL, tvstd->venc_m_control);
	venc_reg_out(VENC_BSTAMP_WSS_DATA, tvstd->venc_bstamp_wss_data);
	venc_reg_out(VENC_S_CARR, tvstd->venc_s_carr);
	venc_reg_out(VENC_LINE21, tvstd->venc_line21);
	venc_reg_out(VENC_LN_SEL, tvstd->venc_ln_sel);
	venc_reg_out(VENC_L21_WC_CTL, tvstd->venc_l21_wc_ctl);
	venc_reg_out(VENC_HTRIGGER_VTRIGGER, tvstd->venc_htrigger_vtrigger);
	venc_reg_out(VENC_SAVID_EAVID, tvstd->venc_savid_eavid);
	venc_reg_out(VENC_FLEN_FAL, tvstd->venc_flen_fal);
	venc_reg_out(VENC_LAL_PHASE_RESET, tvstd->venc_lal_phase_reset);
	venc_reg_out(VENC_HS_INT_START_STOP_X,
			tvstd->venc_hs_int_start_stop_x);
	venc_reg_out(VENC_HS_EXT_START_STOP_X,
			tvstd->venc_hs_ext_start_stop_x);
	venc_reg_out(VENC_VS_INT_START_X, tvstd->venc_vs_int_start_x);
	venc_reg_out(VENC_VS_INT_STOP_X_VS_INT_START_Y,
			tvstd-> venc_vs_int_stop_x_vs_int_start_y);
	venc_reg_out(VENC_VS_INT_STOP_Y_VS_EXT_START_X,
			tvstd->venc_vs_int_stop_y_vs_ext_start_x);
	venc_reg_out(VENC_VS_EXT_STOP_X_VS_EXT_START_Y,
			tvstd->venc_vs_ext_stop_x_vs_ext_start_y);
	venc_reg_out(VENC_VS_EXT_STOP_Y, tvstd->venc_vs_ext_stop_y);
	venc_reg_out(VENC_AVID_START_STOP_X, tvstd->venc_avid_start_stop_x);
	venc_reg_out(VENC_AVID_START_STOP_Y, tvstd->venc_avid_start_stop_y);
	venc_reg_out(VENC_FID_INT_START_X_FID_INT_START_Y,
			tvstd-> venc_fid_int_start_x_fid_int_start_y);
	venc_reg_out(VENC_FID_INT_OFFSET_Y_FID_EXT_START_X,
			tvstd->venc_fid_int_offset_y_fid_ext_start_x);
	venc_reg_out(VENC_FID_EXT_START_Y_FID_EXT_OFFSET_Y,
			tvstd->venc_fid_ext_start_y_fid_ext_offset_y);
	venc_reg_out(VENC_TVDETGP_INT_START_STOP_X,
			tvstd->venc_tvdetgp_int_start_stop_x);
	venc_reg_out(VENC_TVDETGP_INT_START_STOP_Y,
			tvstd->venc_tvdetgp_int_start_stop_y);
	venc_reg_out(VENC_GEN_CTRL, tvstd->venc_gen_ctrl);
//	NOTE : SL comment this in order to dynamically configure the analog tv out type 
// 	venc_reg_out(VENC_DAC_OUTPUT, tvstd->venc_dac_tst);
// 	venc_reg_out(VENC_DAC_TST, venc_reg_in(VENC_DAC));
}

int
omap2_disp_get_tvstandard(void)
{
	return(omap2_current_tvstandard);
}

#else
int
omap2_disp_get_tvstandard(void){ return 0; }
#endif

void
omap2_disp_get_tvlcd(struct tvlcd_status_t *status)
{
	status->status = tvlcd_status.status;
	status->output_dev = tvlcd_status.output_dev;
	status->ltype = tvlcd_status.ltype;
}

void
omap2_disp_set_tvlcd(int status)
{
	tvlcd_status.status = status;
}

#ifndef CONFIG_ARCH_OMAP3410
void
omap2_disp_set_tvstandard(int tvstandard)
{
	int a_ctx;
	struct omap24xx_dispc_regs *dispc = &dss_ctx.dispc;
	omap2_current_tvstandard = tvstandard;
	switch (tvstandard) {
	case PAL_BDGHI:
		config_venc(&pal_bdghi_cfg);
		break;
	case PAL_NC:
		config_venc(&pal_nc_cfg);
		break;
	case PAL_N:
		config_venc(&pal_n_cfg);
		break;
	case PAL_M:
		config_venc(&pal_m_cfg);
		break;
	case PAL_60:
		config_venc(&pal_60_cfg);
		omap2_disp_set_panel_size(OMAP2_OUTPUT_TV, 720, 574);
		break;
	case NTSC_M:
		config_venc(&ntsc_m_cfg);
		break;
	case NTSC_J:
		config_venc(&ntsc_j_cfg);
		break;
	case NTSC_443:
		config_venc(&ntsc_443_cfg);
		omap2_disp_set_panel_size(OMAP2_OUTPUT_TV, 720, 480);
		break;
	}
	dispc->size_dig = dispc_reg_in(DISPC_SIZE_DIG);
	{	
		a_ctx = (in_atomic() || irqs_disabled() 
				|| in_interrupt());
		if (!a_ctx) {
			msleep(500);
		} else {
			udelay(100);
		}
	}
	// omap2_disp_enable_output_dev(OMAP2_OUTPUT_TV);	//why enable the output dev here?? called from omap2_disp_get_all_clks()!!
}
#else
void
omap2_disp_set_tvstandard(int tvstandard) {
	return ;
}
#endif

/* Sets the transparency color key type and value.
*/
void
omap2_disp_set_colorkey(int output_dev, int key_type, int key_val)
{
	if (output_dev == OMAP2_OUTPUT_LCD) {
		if (key_type == OMAP2_VIDEO_SOURCE)
			dispc_reg_merge(DISPC_CONFIG, DISPC_CONFIG_TCKLCDSELECTION,
					DISPC_CONFIG_TCKLCDSELECTION);
		else
			dispc_reg_merge(DISPC_CONFIG, 0,
					DISPC_CONFIG_TCKLCDSELECTION);
		dispc_reg_out(DISPC_TRANS_COLOR0, key_val);
	}
#ifndef CONFIG_ARCH_OMAP3410
	else if (output_dev == OMAP2_OUTPUT_TV) {
		if (key_type == OMAP2_VIDEO_SOURCE)
			dispc_reg_merge(DISPC_CONFIG, DISPC_CONFIG_TCKDIGSELECTION,
					DISPC_CONFIG_TCKDIGSELECTION);
		else
			dispc_reg_merge(DISPC_CONFIG, 0,
					DISPC_CONFIG_TCKDIGSELECTION);
		dispc_reg_out(DISPC_TRANS_COLOR1, key_val);
	}
#endif

}

/* Returns the current transparency color key type and value.
*/
void
omap2_disp_get_colorkey(int output_dev, int *key_type, int *key_val)
{
	if (output_dev == OMAP2_OUTPUT_LCD) {
		if (dispc_reg_in(DISPC_CONFIG) & DISPC_CONFIG_TCKLCDSELECTION)
			*key_type = OMAP2_VIDEO_SOURCE;
		else
			*key_type = OMAP2_GFX_DESTINATION;
		*key_val = dispc_reg_in(DISPC_TRANS_COLOR0);
	}
#ifndef CONFIG_ARCH_OMAP3410
	else if (output_dev == OMAP2_OUTPUT_TV) {
		if (dispc_reg_in(DISPC_CONFIG) & DISPC_CONFIG_TCKDIGSELECTION)
			*key_type = OMAP2_VIDEO_SOURCE;
		else
			*key_type = OMAP2_GFX_DESTINATION;
		*key_val = dispc_reg_in(DISPC_TRANS_COLOR1);
	}
#endif
}

void
omap2_disp_enable_colorkey(int output_dev)
{
	if (output_dev == OMAP2_OUTPUT_LCD)
		dispc_reg_merge(DISPC_CONFIG, DISPC_CONFIG_TCKLCDENABLE,
				DISPC_CONFIG_TCKLCDENABLE);
#ifndef CONFIG_ARCH_OMAP3410
	else if (output_dev == OMAP2_OUTPUT_TV)
		dispc_reg_merge(DISPC_CONFIG, DISPC_CONFIG_TCKDIGENABLE,
				DISPC_CONFIG_TCKDIGENABLE);
#endif
	/* AK: no reg_sync here, needs to be done somewhere else */
}

void
omap2_disp_disable_colorkey(int output_dev)
{
	if (output_dev == OMAP2_OUTPUT_LCD)
		dispc_reg_merge(DISPC_CONFIG, ~DISPC_CONFIG_TCKLCDENABLE,
				DISPC_CONFIG_TCKLCDENABLE);
#ifndef CONFIG_ARCH_OMAP3410
	else if (output_dev == OMAP2_OUTPUT_TV)
		dispc_reg_merge(DISPC_CONFIG, ~DISPC_CONFIG_TCKDIGENABLE,
				DISPC_CONFIG_TCKDIGENABLE);
#endif
}
#ifdef CONFIG_ARCH_OMAP34XX
void
omap2_disp_set_alphablend(int output_dev,int value)
{
	if (output_dev == OMAP2_OUTPUT_LCD) {
		if (value)	
			dispc_reg_merge(DISPC_CONFIG, DISPC_CONFIG_LCDALPHAENABLE,
					DISPC_CONFIG_LCDALPHAENABLE);
		else	
			dispc_reg_merge(DISPC_CONFIG, ~DISPC_CONFIG_LCDALPHAENABLE,
					DISPC_CONFIG_LCDALPHAENABLE);
	}	
#ifndef CONFIG_ARCH_OMAP3410
	else if (output_dev == OMAP2_OUTPUT_TV) {
		if (value)	
			dispc_reg_merge(DISPC_CONFIG, DISPC_CONFIG_TVALPHAENABLE,
					DISPC_CONFIG_TVALPHAENABLE);
		else
			dispc_reg_merge(DISPC_CONFIG, ~DISPC_CONFIG_TVALPHAENABLE,
					DISPC_CONFIG_TVALPHAENABLE);
	}
#endif
}

void
omap2_disp_set_global_alphablend_value(int ltype,int value)
{
	unsigned int alpha_value;

	if (ltype == OMAP2_GRAPHICS) {
		alpha_value = dispc_reg_in(DISPC_GLOBAL_ALPHA);
		alpha_value &= (~DISPC_GLOBAL_ALPHA_GFX_GALPHA);
		alpha_value |= (value << DISPC_GLOBAL_ALPHA_GFX_GALPHA_SHIFT);
		dispc_reg_out(DISPC_GLOBAL_ALPHA,alpha_value);

	}	
	else if (ltype == OMAP2_VIDEO2) {
		alpha_value = dispc_reg_in(DISPC_GLOBAL_ALPHA);
		alpha_value &= (~DISPC_GLOBAL_ALPHA_VID2_GALPHA);
		alpha_value |= (value << DISPC_GLOBAL_ALPHA_VID2_GALPHA_SHIFT);
		dispc_reg_out(DISPC_GLOBAL_ALPHA,alpha_value);

	}
}

unsigned char
omap2_disp_get_global_alphablend_value(int ltype)
{
	unsigned int alpha_value = 0;

	if (ltype == OMAP2_GRAPHICS) {
		alpha_value = dispc_reg_in(DISPC_GLOBAL_ALPHA);
		alpha_value &= (DISPC_GLOBAL_ALPHA_GFX_GALPHA);
		alpha_value = alpha_value >> 
			DISPC_GLOBAL_ALPHA_GFX_GALPHA_SHIFT;
	} else if (ltype == OMAP2_VIDEO2) {
		alpha_value = dispc_reg_in(DISPC_GLOBAL_ALPHA);
		alpha_value &= (DISPC_GLOBAL_ALPHA_VID2_GALPHA);
		alpha_value = alpha_value >> 
			DISPC_GLOBAL_ALPHA_VID2_GALPHA_SHIFT;
	}
	return (unsigned char) alpha_value;
}

int
omap2_disp_get_alphablend(int output_dev)
{

	if (output_dev == OMAP2_OUTPUT_LCD) {
		if(dispc_reg_in(DISPC_CONFIG) & 0x00040000)
			return 1;
		else
			return 0;
	}
#ifndef CONFIG_ARCH_OMAP3410
	else if (output_dev == OMAP2_OUTPUT_TV) {
		if(dispc_reg_in(DISPC_CONFIG) & 0x00080000)
			return 1;
		else
			return 0;
	}
#endif
	return 0;
}
#endif	

int 
omap2_disp_reg_sync_bit(int output_dev)
{
	u32 control = dispc_reg_in (DISPC_CONTROL);

	if(output_dev == OMAP2_OUTPUT_LCD){
		return (control & DISPC_CONTROL_GOLCD) >> 5;
	} else {
		return (control & DISPC_CONTROL_GODIGITAL) >> 6;
	} 
}

/*
 * This function is required for 2420 errata 1.97
 */

#ifndef CONFIG_ARCH_OMAP3410
static void
omap2_reset_venc(void)
{
	u32 i=0;
	struct clk *dss_tv_fck;
#ifdef CONFIG_MACH_OMAP_2430SDP
	dss_tv_fck = clk_get(NULL,"dss_54m_fck");
#endif
#if defined(CONFIG_MACH_OMAP_3430SDP) || defined(CONFIG_MACH_OMAP3_EVM) || defined(CONFIG_MACH_ARCHOS_G6)
#ifdef CONFIG_TRACK_RESOURCES
	dss_tv_fck = clk_get(&display_dev,"dss_tv_fck");
#else
	dss_tv_fck = clk_get(NULL,"dss_tv_fck");
#endif
#endif 

	if(IS_ERR(dss_tv_fck)){
		printk("\n UNABLE to get dss TV fclk \n");
		return;
	}

	/* Enable VENC clock  */
	if(clk_enable(dss_tv_fck) != 0) {
		printk("\n UNABLE to enable dss TV fclk \n");
		return;
	}

	/*
	 * Write 1 to the 8th bit of the F_Control register to reset the VENC
	 */
	venc_reg_merge(VENC_F_CONTROL, VENC_FCONTROL_RESET, VENC_FCONTROL_RESET);

	/* wait for reset to complete */
	while ((venc_reg_in(VENC_F_CONTROL)  & VENC_FCONTROL_RESET) == 0x00000100) {
		udelay(10);
		if(i++ >10) break;
	}

	if (venc_reg_in(VENC_F_CONTROL) & VENC_FCONTROL_RESET) {
		printk(KERN_WARNING
				"omap2_disp: timeout waiting for venc reset\n");

		/* remove the reset */
		venc_reg_merge(VENC_F_CONTROL,(0<<8),VENC_FCONTROL_RESET);
	}

	/* disable the VENC clock */
	clk_disable(dss_tv_fck);
}
#endif

/*
 * Enables an IRQ in DSPC_IRQENABLE. 
 */
int
omap2_disp_irqenable(omap2_disp_isr_t isr,unsigned int mask)
{
	int i;
	unsigned long flags;

	if (omap2_disp_irq == 0 || mask == 0)
		return -EINVAL;

	omap2_disp_get_dss();
	spin_lock_irqsave(&dss_lock, flags);
	for (i = 0; i < MAX_ISR_NR; i++) {
		if (registered_isr[i].isr == isr) {
			registered_isr[i].mask |= mask;
			dispc_reg_out(DISPC_IRQENABLE,dispc_reg_in(DISPC_IRQENABLE) | mask);
			spin_unlock_irqrestore(&dss_lock, flags);
			omap2_disp_put_dss();
			return 0;
		}
	}
	spin_unlock_irqrestore(&dss_lock, flags);
	omap2_disp_put_dss();
	return -EBUSY;
}

/*
 * Disables an IRQ in DISPC_IRQENABLE, 
 * The IRQ will be active if any other ISR is still using the same.
 * mask : should contain '0' for irq to be disable and rest should be '1'.
 */
int
omap2_disp_irqdisable(omap2_disp_isr_t isr,unsigned int mask)
{
	int i;
	unsigned long flags;
	unsigned int new_mask = 0;

	if (omap2_disp_irq == 0)
		return -EINVAL;

	omap2_disp_get_dss();
	spin_lock_irqsave(&dss_lock, flags);
	for (i = 0; i < MAX_ISR_NR; i++) 
		if (registered_isr[i].isr == isr) break;
	
	if(i == MAX_ISR_NR){
		spin_unlock_irqrestore(&dss_lock, flags);
		omap2_disp_put_dss();
		return -EINVAL;
	}
	
	registered_isr[i].mask &= mask;
	
	/* disable an IRQ if every one wishes to do so */
	for (i = 0; i < MAX_ISR_NR; i++){
		new_mask |= registered_isr[i].mask;
	}
	
	dispc_reg_out(DISPC_IRQENABLE, new_mask);
	spin_unlock_irqrestore(&dss_lock, flags);
	omap2_disp_put_dss();

	return -EBUSY;
}

/* Display controller interrupts are handled first by this display library.
 * Drivers that need to use certain interrupts should register their ISRs and
 * interrupt enable mask with the display library.
 */
int
omap2_disp_register_isr(omap2_disp_isr_t isr, void *arg, unsigned int mask)
{
	int i;
	unsigned long flags;

	if (omap2_disp_irq == 0 || isr == 0 || arg == 0)
		return -EINVAL;

	omap2_disp_get_dss();
	/* Clear all the interrupt, so that you dont get an immediate interrupt*/
	dispc_reg_out(DISPC_IRQSTATUS, 0xFFFFFFFF);
	spin_lock_irqsave(&dss_lock, flags);
	for (i = 0; i < MAX_ISR_NR; i++) {
		if (registered_isr[i].isr == NULL) {
			registered_isr[i].isr = isr;
			registered_isr[i].arg = arg;
			registered_isr[i].mask = mask;

			/* Clear previous interrupts if any */
			dispc_reg_out(DISPC_IRQSTATUS, mask);
			dispc_reg_out(DISPC_IRQENABLE,dispc_reg_in(DISPC_IRQENABLE) | mask);
			spin_unlock_irqrestore(&dss_lock, flags);
			omap2_disp_put_dss();
			return 0;
		}
	}
	spin_unlock_irqrestore(&dss_lock, flags);
	omap2_disp_put_dss();
	return -EBUSY;
}

int
omap2_disp_unregister_isr(omap2_disp_isr_t isr)
{
	int i,j;
	unsigned long flags;
	unsigned int new_mask =0;

	if (omap2_disp_irq == 0)
		return -EINVAL;

	omap2_disp_get_dss();
	spin_lock_irqsave(&dss_lock, flags);
	for (i = 0; i < MAX_ISR_NR; i++) {
		if (registered_isr[i].isr == isr) {
			registered_isr[i].isr = NULL;
			registered_isr[i].arg = NULL;
			registered_isr[i].mask = 0;

			/* The interrupt may no longer be valid, re-set the IRQENABLE */
			for (j = 0; j < MAX_ISR_NR; j++){
				new_mask |= registered_isr[j].mask;
			}
			dispc_reg_out(DISPC_IRQENABLE, new_mask);
			spin_unlock_irqrestore(&dss_lock, flags);
			omap2_disp_put_dss();
			return 0;
		}
	}
	spin_unlock_irqrestore(&dss_lock, flags);
	omap2_disp_put_dss();
	return -EINVAL;
}

/* DSS Interrupt master service routine. */
static irqreturn_t
omap2_disp_master_isr(int irq, void *arg, struct pt_regs *regs)
{
	unsigned long dispc_irqstatus = dispc_reg_in(DISPC_IRQSTATUS);
	int i;

	for (i = 0; i < MAX_ISR_NR; i++) {
		if (registered_isr[i].isr == NULL)
			continue;
		if (registered_isr[i].mask & dispc_irqstatus)
			registered_isr[i].isr(registered_isr[i].arg, regs, dispc_irqstatus);
	}

	/* ack the interrupt */
	dispc_reg_out(DISPC_IRQSTATUS, dispc_irqstatus);

	return IRQ_HANDLED;
}

#if defined(CONFIG_FB_OMAP_BOOTLOADER_INIT)
static void __init omap2_disp_check(void)
{
	u32 idle_dispc;
#ifdef CONFIG_HW_SUP_TRANS
	u32 idle_dss;
#endif /* #ifdef CONFIG_HW_SUP_TRANS */

	/* turn on DSS clock */
	omap24xx_ll_config_disp_clocks(0);
	omap2_disp_set_tvref(TVREF_ON);
	omap24xx_ll_config_tv_clocks(0);

#ifdef CONFIG_HW_SUP_TRANS
	/* Set smart idle for Display subsystem */
	idle_dss = dss_reg_in(DSS_SYSCONFIG);
	idle_dss |= DSS_SYSCONFIG_AUTOIDLE;
	dss_reg_out(DSS_SYSCONFIG, idle_dss);
#endif /* #ifdef CONFIG_HW_SUP_TRANS */

	/* Set smart idle, autoidle for Display controller */
	idle_dispc = dispc_reg_in(DISPC_SYSCONFIG);
	idle_dispc &= ~(DISPC_SYSCONFIG_MIDLEMODE |
			DISPC_SYSCONFIG_SIDLEMODE);

#ifdef CONFIG_HW_SUP_TRANS
	idle_dispc |= (DISPC_SYSCONFIG_MIDLEMODE_SSTANDBY |
			DISPC_SYSCONFIG_SIDLEMODE_SIDLE |
			DISPC_SYSCONFIG_ENABLE_WKUP);
	idle_dispc |= DISPC_SYSCONFIG_AUTOIDLE;
#else
	idle_dispc |= DISPC_SYSCONFIG_MIDLEMODE_NSTANDBY |
		DISPC_SYSCONFIG_SIDLEMODE_NIDLE;
#endif /* #ifdef CONFIG_HW_SUP_TRANS */

	dispc_reg_out(DISPC_SYSCONFIG, idle_dispc);

	disp_usage++;
}
#endif

int __init omap2_disp_init(void)
{
	int rev,i;
	u32 dss_control;

	spin_lock_init(&dss_lock);

#ifndef CONFIG_OMAP_USE_DSI_PLL
	/* Required for scale call */
#ifdef CONFIG_TRACK_RESOURCES
	dss1f_scale = clk_get(&display_dev,"dss1_fck");
#else
	dss1f_scale = clk_get(NULL,"dss1_fck");
#endif
	if(IS_ERR(dss1f_scale)) {
		printk("Could not get DSS1 FCLK\n");
		return PTR_ERR(dss1f_scale);
	}
#endif

#if defined(CONFIG_FB_OMAP_BOOTLOADER_INIT)
	omap2_disp_check();
#else
	/* enable the clocks */
	omap2_disp_get_dss();

	/* disable the display controller */
	omap2_disp_disable(HZ/5);

	omap24xx_ll_config_disp_clocks(0);
	disp_usage++;
#endif
	rev = dss_reg_in(DSS_REVISION);
	printk(KERN_INFO "OMAP Display hardware version %d.%d\n",
		(rev & DISPC_REVISION_MAJOR) >> DISPC_REVISION_MAJOR_SHIFT,
		(rev & DISPC_REVISION_MINOR) >> DISPC_REVISION_MINOR_SHIFT);

	/* Disable RFBI mode, which is not currently supported. */
	dispc_reg_merge(DISPC_CONTROL, 0, DISPC_CONTROL_RFBIMODE);

	/* For 2420 VENC errata 1.97 */
	/* For 2430 VENC errata 1.20 */
#ifndef CONFIG_ARCH_OMAP3410
	omap2_reset_venc();
#endif
	/* enable DAC_DEMEN and VENC_4X_CLOCK in DSS for TV operation */
	dss_control = dss_reg_in(DSS_CONTROL);

	/* Should be replaced by FPGA register read  ADD A 2420 ifdef here*/

#ifdef CONFIG_ARCH_OMAP2420
	dss_control |= (DSS_CONTROL_DAC_DEMEN |
			DSS_CONTROL_VENC_CLOCK_4X_ENABLE);
#endif

#ifdef CONFIG_MACH_OMAP_2430SDP
#ifdef CONFIG_TWL4030_CORE_T2 
	dss_control |= (DSS_CONTROL_TV_REF | DSS_CONTROL_DAC_DEMEN |
			DSS_CONTROL_VENC_CLOCK_4X_ENABLE);
#endif

#ifdef CONFIG_TWL4030_CORE_M1
	dss_control |= (DSS_CONTROL_DAC_DEMEN |
			DSS_CONTROL_VENC_CLOCK_4X_ENABLE);
#endif
#endif

#if defined(CONFIG_MACH_OMAP_3430SDP) || defined(CONFIG_MACH_OMAP3_EVM)|| defined(CONFIG_MACH_ARCHOS_G6)
	/* enabling S-video connector for 3430 SDP */
#ifndef CONFIG_ARCH_OMAP3410
	dss_control |= (DSS_CONTROL_DAC_DEMEN | DSS_CONTROL_TV_REF |
		DSS_CONTROL_VENC_CLOCK_4X_ENABLE | DSS_CONTROL_VENC_OUT);
#else
	dss_control |= (DSS_CONTROL_DAC_DEMEN |
		DSS_CONTROL_VENC_CLOCK_4X_ENABLE | DSS_CONTROL_VENC_OUT);
#endif
#endif

	dss_control &= ~DSS_CONTROL_VENC_CLOCK_MODE;
	dss_reg_out(DSS_CONTROL, dss_control);

	/* By default, all layers go to LCD */
	layer[OMAP2_GRAPHICS].output_dev = OMAP2_OUTPUT_LCD;
	layer[OMAP2_VIDEO1].output_dev = OMAP2_OUTPUT_LCD;
	layer[OMAP2_VIDEO2].output_dev = OMAP2_OUTPUT_LCD;

	/* 
	 * Set the default color conversion parameters for Video pipelines 
	 * by default the color space is set to JPEG
	 */

	update_colorconv_mtx(0,cc_bt601_full);
	set_colorconv(0,V4L2_COLORSPACE_JPEG);

	update_colorconv_mtx(1,cc_bt601_full);
	set_colorconv(1,V4L2_COLORSPACE_JPEG);


	/* Set the default background color to be black*/
	omap2_disp_set_bg_color(OMAP2_OUTPUT_LCD, 0x000000);
#ifndef CONFIG_ARCH_OMAP3410
	omap2_disp_set_bg_color(OMAP2_OUTPUT_TV, 0x000000);
#endif

	if (request_irq(INT_24XX_DSS_IRQ, (void *)omap2_disp_master_isr, 
				IRQF_SHARED, "OMAP2 Display", 
				registered_isr)) {
		printk(KERN_WARNING "omap2_disp: request_irq failed\n");
		omap2_disp_irq = 0;
	} else {
		omap2_disp_irq = 1;
		for(i=0;i<MAX_ISR_NR;i++){
			registered_isr[i].isr  = NULL;
			registered_isr[i].mask = 0;
		}
		/* Clear all the pending interrupts, if any */ 
		dispc_reg_out(DISPC_IRQSTATUS, 0xFFFFFFFF);
		omap2_disp_register_isr(omap2_synclost_isr, layer,
				DISPC_IRQSTATUS_SYNCLOST);
	}

	omap2_disp_register_isr(omap2_synclost_isr, layer,
			DISPC_IRQSTATUS_SYNCLOST);

#if !defined(CONFIG_FB_OMAP_BOOTLOADER_INIT)
	omap2_disp_put_dss();
#else
	/* save dss context after init */
	omap2_disp_save_ctx(OMAP_DSS_GENERIC);
	omap2_disp_save_ctx(OMAP_DSS_DISPC_GENERIC);
	omap2_disp_save_ctx(OMAP2_GRAPHICS);
	omap2_disp_save_ctx(OMAP2_VIDEO1);
	omap2_disp_save_ctx(OMAP2_VIDEO2);
#endif
	return 0;

}

#ifdef CONFIG_ARCH_OMAP34XX

/*
 * optimal precalculated frequency for DSS1_ALWON_CLK. The freq is optimal
 * in the sense that there is a divider that gives pixel clock rate closest to
 * the lowest possible pixel clock rate for a given panel
 */ 
#ifdef CONFIG_FB_OMAP_LCD_VGA
#define LPR_DSS1_ALWON_FCLK_OPT_FREQ	96000000
#define LPR_DSS_LOGIC_DIV		2
#define LPR_HBP				79
#define LPR_HFP				59
#define LPR_HSW				2
#define LPR_VBP				0
#define LPR_VFP				0
#define LPR_VSW				1
#define LPR_GFX_FIFO_LOW_THRES		0x99C
#define LPR_GFX_FIFO_HIGH_THRES		0xB9C
#else
/* These values are validated only on 3430 ES2 */
/* Might not work on 3430 ES1 */
#  define LPR_DSS1_ALWON_FCLK_OPT_FREQ	108000000
#  define LPR_DSS_LOGIC_DIV		9
#  define LPR_HBP			31
#  define LPR_HFP			37
#  define LPR_HSW			3
#  define LPR_VBP			8
#  define LPR_VFP			2
#  define LPR_VSW			1
#define LPR_GFX_FIFO_LOW_THRES		0x7F8
#define LPR_GFX_FIFO_HIGH_THRES		0xB9C
#endif /* CONFIG_FB_OMAP_LCD_VGA */

#define LPR_DEFAULT_FPS	60

#ifdef CONFIG_MACH_OMAP_3430SDP

#endif /* CONFIG_MACH_OMAP_3430SDP */

struct dss_run_mode {
	int rate;
	int divisors;
	int timing_h;
	int timing_v;
};

static struct dss_run_mode rmode;
int lpr_enabled;

/*
 * if set, the lowest dss logic frequency to achieve given(lowest) pixel clock 
 * frequency is set. Otherwise 'lpr_lowest_dss_logic_freq' value is used.
 * The flag is set by default and can be reset via sysfs interface
 */
int lpr_lowest_dss_logic_freq_enabled = 1;

/*
 * dss logic freq. initially initialized to be equal to DSS1_ALWON_FCLK.
 * the value may be adjusted via sysfs interface if needed
 */ 
int lpr_dss_logic_freq = LPR_DSS1_ALWON_FCLK_OPT_FREQ;

/* FPS value in LPR. may be adjusted via sysfs interface */
int lpr_fps = LPR_DEFAULT_FPS;

static DECLARE_MUTEX(lpr_mutex);

/*
 * adjust porches and pulses to get desired FPS after pixel clock is adjusted
 * to LPR value
 */
static void lpr_set_fps(int fps)
{
	int timing_h, timing_v;

	if (lpr_lowest_dss_logic_freq_enabled) {

		/* adjust porches and pulses to get 60 FPS */
		timing_h = ((LPR_HBP - 1) << DISPC_TIMING_H_HBP_SHIFT) |
			((LPR_HFP - 1) << DISPC_TIMING_H_HFP_SHIFT) | 
			((LPR_HSW - 1) << DISPC_TIMING_H_HSW_SHIFT);
		timing_v = (LPR_VBP << DISPC_TIMING_V_VBP_SHIFT) |
			(LPR_VFP << DISPC_TIMING_V_VFP_SHIFT) | 
			(LPR_VSW << DISPC_TIMING_V_VSW_SHIFT);

		dispc_reg_merge(DISPC_TIMING_H, timing_h,
				(DISPC_TIMING_H_HBP |
				 DISPC_TIMING_H_HFP |
				 DISPC_TIMING_H_HSW));

		dispc_reg_merge(DISPC_TIMING_V, timing_v,
				(DISPC_TIMING_V_VBP |
				 DISPC_TIMING_V_VFP |
				 DISPC_TIMING_V_VSW));

	}
	return;
}

/**
 * lpr_set_dss_logic_freq - chose lcd div and pcd based on user input
 *
 * If lpr_lowest_dss_logic_freq_enabled flag is set (default) the routine sets
 * dss logic frequency equal to twice pixel clock frequency. If the default
 * setup does not work for an application (dss logic frequency equal to twice
 * pixel clock frequency is not enough) user can affect the settings using
 * sysfs interface:
 * if lpr_lowest_dss_logic_freq_enabled flag is reset the logic frequency is 
 * set to user specified value (may be adjusted using sysfs interface).
 * Default value in this case is DSS1_ALWON_FCLK frequency.
 */ 
static int lpr_set_dss_logic_freq(void)
{
	int lcd_div, pcd;

	if (lpr_lowest_dss_logic_freq_enabled) {
		/*
		 * set pcd and lcd div to get lowest _both_ pixel clock freq
		 * _and_ logic frequency
		 */
		pcd = 2;
		lcd_div = LPR_DSS_LOGIC_DIV;
	} else {
		/* set requested DSS logic freq */
#if 0
		TODO, not tested!

			lcd_div = LPR_DSS1_ALWON_FCLK_OPT_FREQ / lpr_dss_logic_freq;
		if (lcd_div < 1)
			return -EINVAL;

		logic_freq = LPR_DSS1_ALWON_FCLK_OPT_FREQ / lcd_div;

		/*
		 * (LPR_DSS1_ALWON_FCLK_OPT_FREQ / LPR_DSS_LOGIC_DIV) gives best
		 * minimal pixel clock freq for a panel
		 */
		pcd = logic_freq / (LPR_DSS1_ALWON_FCLK_OPT_FREQ / LPR_DSS_LOGIC_DIV);
		if (pcd < 1)
			return -EINVAL;
#else
		return -ENOTSUPP;
#endif /* 0 */
	}

	dispc_reg_merge(DISPC_DIVISOR,
			(lcd_div << DISPC_DIVISOR_LCD_SHIFT) |
			(pcd << DISPC_DIVISOR_PCD_SHIFT),
			DISPC_DIVISOR_LCD | DISPC_DIVISOR_PCD);

	return 0;
}


/**
 * omap2_disp_lpr_enable - trigger Low Power Refresh mode
 *
 * TODO: desc of LPR
 */
int omap2_disp_lpr_enable(void)
{
	int dss1_rate;
	int gfx_fifo_thresholds;
	int rc = 0;
	int v_attr;
	int digitalen;
	unsigned long flags;

	/* Cannot enable lpr if DSS is inactive */
	if (!graphics_in_use)
		return -1;

	down(&lpr_mutex);

	if (lpr_enabled)
		goto lpr_out;
	/* 
	 * Check whether LPR can be triggered
	 *   - gfx pipeline is routed to LCD
	 *   - both video pipelines are disabled (this allows FIFOs merge)
	 */

	/* may be more meaningful error code is required */
	if (omap2_disp_get_output_dev(OMAP2_GRAPHICS) != OMAP2_OUTPUT_LCD) {
		rc = -1;
		goto lpr_out;
	}

	omap2_disp_get_dss();

	v_attr = dispc_reg_in(DISPC_VID_ATTRIBUTES(0)) |
		 dispc_reg_in(DISPC_VID_ATTRIBUTES(1));

	if (v_attr & DISPC_VID_ATTRIBUTES_ENABLE) {
		rc = -1;
		goto lpr_out_clk;
	} 

	/* 
	 * currently DSS is running on DSS1 by default. just warn if it has
	 * changed in the future 
	 */
	if (dss_reg_in(DSS_CONTROL) & DSS_CONTROL_APLL_CLK)
		BUG();

	/* save run mode rate */
	rmode.rate = dss1_rate = clk_get_rate(dss1f);

	digitalen = dispc_reg_in(DISPC_CONTROL) & DISPC_CONTROL_DIGITALENABLE;

	/* disable DSS before adjusting DSS clock */
	omap2_disp_disable(HZ/5);

	/* set DSS1_ALWON_FCLK freq */
	rc = clk_set_rate(dss1f, LPR_DSS1_ALWON_FCLK_OPT_FREQ);
	if (rc != 0)
		goto lpr_out_clk_en;

	rmode.divisors = (dispc_reg_in(DISPC_DIVISOR) & (DISPC_DIVISOR_LCD |
							 DISPC_DIVISOR_PCD));

	rmode.timing_h = (dispc_reg_in(DISPC_TIMING_H) & (DISPC_TIMING_H_HBP |
							DISPC_TIMING_H_HFP |
							DISPC_TIMING_H_HSW));

	rmode.timing_v = (dispc_reg_in(DISPC_TIMING_V) & (DISPC_TIMING_V_VBP |
							DISPC_TIMING_V_VFP |
							DISPC_TIMING_V_VSW));

	/* chose lcd div and pcd based on user input */
	rc = lpr_set_dss_logic_freq();
	if (rc != 0)
		goto lpr_out_clk_en_rate;

	lpr_set_fps(lpr_fps);
 
	/* set up LPR default  FIFO thresholds */
	gfx_fifo_thresholds =
	     (LPR_GFX_FIFO_HIGH_THRES << DISPC_GFX_FIFO_THRESHOLD_HIGH_SHIFT) |
	     (LPR_GFX_FIFO_LOW_THRES << DISPC_GFX_FIFO_THRESHOLD_LOW_SHIFT);

	dispc_reg_merge(DISPC_GFX_FIFO_THRESHOLD, gfx_fifo_thresholds, 
			(DISPC_GFX_FIFO_THRESHOLD_HIGH |
			 DISPC_GFX_FIFO_THRESHOLD_LOW));

	dispc_reg_merge(DISPC_CONFIG, DISPC_CONFIG_FIFOMERGE,
			DISPC_CONFIG_FIFOMERGE);	

	/*
	 * save LPR configuration of DISPC and GFX register in case synclost
	 * happens during LPR. If synclost happens LPR parameters get reset
	 * to last-known-good LPR parameters
	 *
	 * may be useful to restore known-good parameters if FIFO underrun
	 * occurs as well
	 */
	omap2_disp_save_ctx(OMAP2_GRAPHICS);	

	spin_lock_irqsave(&dss_lock, flags);
	omap2_disp_enable_output_dev(OMAP2_OUTPUT_LCD);
	if (digitalen)
		omap2_disp_enable_output_dev(OMAP2_OUTPUT_TV);

	omap2_disp_save_ctx(OMAP_DSS_DISPC_GENERIC);
	spin_unlock_irqrestore(&dss_lock, flags);

	/* let LPR settings to take effect */
	omap2_disp_reg_sync(OMAP2_OUTPUT_LCD);
	wait_for_reg_sync(OMAP2_OUTPUT_LCD, DEFAULT_TIMEOUT);
	
	lpr_enabled = 1;

	omap2_disp_put_dss();
	up(&lpr_mutex);

	return 0;

lpr_out_clk_en_rate:
	clk_set_rate(dss1f, rmode.rate);

lpr_out_clk_en:
	omap2_disp_enable_output_dev(OMAP2_OUTPUT_LCD);
	if (digitalen)
		omap2_disp_enable_output_dev(OMAP2_OUTPUT_TV);
lpr_out_clk:
	omap2_disp_put_dss();	
lpr_out:
	up(&lpr_mutex);
	return rc;
}

int omap2_disp_lpr_disable(void)
{
	int rc = 0;
	int gfx_fifo_thresholds;
	int digitalen;
	unsigned long flags;

	if (!graphics_in_use)
		return -1;

	down(&lpr_mutex);

	if (!lpr_enabled) {
		up(&lpr_mutex);
		return rc;
	}


	omap2_disp_get_dss();

	/* restore DSS  divisors */
	dispc_reg_merge(DISPC_DIVISOR, rmode.divisors,
			DISPC_DIVISOR_LCD | DISPC_DIVISOR_PCD);

	/* split FIFOs and restore FIFO thresholds */
	dispc_reg_merge(DISPC_CONFIG, 0, DISPC_CONFIG_FIFOMERGE);

	gfx_fifo_thresholds =
	(RMODE_GFX_FIFO_HIGH_THRES << DISPC_GFX_FIFO_THRESHOLD_HIGH_SHIFT) |
	(RMODE_GFX_FIFO_LOW_THRES << DISPC_GFX_FIFO_THRESHOLD_LOW_SHIFT);

	dispc_reg_merge(DISPC_GFX_FIFO_THRESHOLD, gfx_fifo_thresholds, 
			(DISPC_GFX_FIFO_THRESHOLD_HIGH |
			 DISPC_GFX_FIFO_THRESHOLD_LOW));

	dispc_reg_merge(DISPC_TIMING_H, rmode.timing_h, (DISPC_TIMING_H_HBP |
				DISPC_TIMING_H_HFP |
				DISPC_TIMING_H_HSW));

	dispc_reg_merge(DISPC_TIMING_V, rmode.timing_v, (DISPC_TIMING_V_VBP |
				DISPC_TIMING_V_VFP |
				DISPC_TIMING_V_VSW));
	/* TODO: adjust porches and pulses if bigger fps is not acceptable */

	digitalen = dispc_reg_in(DISPC_CONTROL) & DISPC_CONTROL_DIGITALENABLE;

	/* disable DSS before adjusting DSS clock */
	omap2_disp_disable(HZ/5);

	/* restore DSS run mode rate */ 
	rc = clk_set_rate(dss1f, rmode.rate);

	omap2_disp_save_ctx(OMAP2_GRAPHICS);	

	spin_lock_irqsave(&dss_lock, flags);
	omap2_disp_enable_output_dev(OMAP2_OUTPUT_LCD);
	if (digitalen)
		omap2_disp_enable_output_dev(OMAP2_OUTPUT_TV);

	omap2_disp_save_ctx(OMAP_DSS_DISPC_GENERIC);
	spin_unlock_irqrestore(&dss_lock, flags);

	omap2_disp_reg_sync(OMAP2_OUTPUT_LCD);
	wait_for_reg_sync(OMAP2_OUTPUT_LCD, DEFAULT_TIMEOUT);

	omap2_disp_put_dss();

	lpr_enabled = 0;

	up(&lpr_mutex);
	return rc;
}
#endif /* CONFIG_ARCH_OMAP34XX */

#ifdef CONFIG_MACH_OMAP3EVM
void omap2_disp_replication_enable ( void )
{
    u32 attributes;
    attributes = dispc_reg_in (DISPC_GFX_ATTRIBUTES);

    /* Enable GFX repilcation to make 16-bit colour as 18-bit colour. */
    attributes |= DISPC_GFX_ATTRIBUTES_GFXREPLICATIONENABLE;
    dispc_reg_out (DISPC_GFX_ATTRIBUTES, attributes);
}
EXPORT_SYMBOL(omap2_disp_replication_enable);
#endif

void omap2_disp_gfxformat ( int *format , int read_access)
{
	u32 attributes;
	attributes = dispc_reg_in(DISPC_GFX_ATTRIBUTES);

	if ( read_access ) {
		(*format)= ((attributes & DISPC_GFX_ATTRIBUTES_GFXFORMAT)>>1);
	} else {
		attributes &= ~DISPC_GFX_ATTRIBUTES_GFXFORMAT;
		attributes |= ((*format)<<1);
		dispc_reg_out (DISPC_GFX_ATTRIBUTES, attributes);

		if (omap2_disp_get_output_dev(OMAP2_GRAPHICS) == OMAP2_OUTPUT_TV ) {
			omap2_disp_reg_sync( OMAP2_OUTPUT_TV );
			wait_for_reg_sync(OMAP2_OUTPUT_TV, DEFAULT_TIMEOUT);
		} else {
			omap2_disp_reg_sync( OMAP2_OUTPUT_LCD );
			wait_for_reg_sync(OMAP2_OUTPUT_LCD, DEFAULT_TIMEOUT);
		}
	}
	return;
}
EXPORT_SYMBOL(omap2_disp_gfxformat);


void omap2_disp_set_irq_on_linenumber(int val) {

	dispc_reg_out(DISPC_LINE_NUMBER,val);
}

void omap2_disp_swap_dma_ba(int i, int time_base) {
	struct omap2_disp_dma_params *dma_param_graphics = &layer[OMAP2_GRAPHICS].dma[0];
	struct omap2_disp_dma_params *dma_param_video1   = &layer[OMAP2_VIDEO1].dma[0];	
	struct omap2_disp_dma_params *dma_param_video2   = &layer[OMAP2_VIDEO2].dma[0];	
	int timing_v = dispc_reg_in(DISPC_TIMING_V) & ~(DISPC_TIMING_V_VBP);
	int vfp;

	if (i == EVEN_FIELD) {
	/* EVEN FIELD */
		dispc_reg_out(DISPC_GFX_BA0,    dma_param_graphics->ba1);
		dispc_reg_out(DISPC_GFX_BA1,    dma_param_graphics->ba1);
		dispc_reg_out(DISPC_VID_BA0(0), dma_param_video1->ba1);
		dispc_reg_out(DISPC_VID_BA1(0), dma_param_video1->ba1);
		dispc_reg_out(DISPC_VID_ACCU0(0), dma_param_video1->accu1 << 16 );
		dispc_reg_out(DISPC_VID_ACCU1(0), dma_param_video1->accu1 << 16 );
		
		dispc_reg_out(DISPC_VID_BA0(1), dma_param_video2->ba1);
		dispc_reg_out(DISPC_VID_BA1(1), dma_param_video2->ba1);
		vfp=time_base;
	} else  {
	/* ODD FIELD */
		dispc_reg_out(DISPC_GFX_BA0,    dma_param_graphics->ba0);
		dispc_reg_out(DISPC_GFX_BA1,    dma_param_graphics->ba0);
		dispc_reg_out(DISPC_VID_BA0(0), dma_param_video1->ba0);
		dispc_reg_out(DISPC_VID_BA1(0), dma_param_video1->ba0);
		dispc_reg_out(DISPC_VID_ACCU0(0), dma_param_video1->accu0 << 16 );
		dispc_reg_out(DISPC_VID_ACCU1(0), dma_param_video1->accu0 << 16 );
		
		dispc_reg_out(DISPC_VID_BA0(1), dma_param_video2->ba0);
		dispc_reg_out(DISPC_VID_BA1(1), dma_param_video2->ba0);
		vfp=(time_base-1);
	}
/* cycle the accu0 value for testing
	dma_param_video1->accu0 ++;
	if( dma_param_video1->accu0 > 1023 ) {
		dma_param_video1->accu0 = 0;
	}
printk("accu0 %4d  accu1 %4d\r\n", dma_param_video1->accu0, dma_param_video1->accu1 );
*/	
	
	timing_v |= (vfp << DISPC_TIMING_V_VBP_SHIFT);

	dispc_reg_out(DISPC_TIMING_V, timing_v);

	omap2_disp_reg_sync( OMAP2_OUTPUT_LCD );
	/* AK: we don't wait here - this is only called from videoenc_isr_handler()  */
}

/* Start before devices */
subsys_initcall(omap2_disp_init);

EXPORT_SYMBOL(omap2_disp_request_layer);
EXPORT_SYMBOL(omap2_disp_release_layer);
EXPORT_SYMBOL(omap2_disp_disable_layer);
EXPORT_SYMBOL(omap2_disp_enable_layer);
EXPORT_SYMBOL(omap2_disp_config_vlayer);
EXPORT_SYMBOL(omap2_disp_config_gfxlayer);
EXPORT_SYMBOL(omap2_disp_start_vlayer);
EXPORT_SYMBOL(omap2_disp_start_gfxlayer);

EXPORT_SYMBOL(omap2_disp_set_bg_color);
EXPORT_SYMBOL(omap2_disp_get_bg_color);
EXPORT_SYMBOL(omap2_disp_set_dithering);
EXPORT_SYMBOL(omap2_disp_get_dithering);
EXPORT_SYMBOL(omap2_disp_get_panel_size);
EXPORT_SYMBOL(omap2_disp_set_panel_size);
EXPORT_SYMBOL(omap2_disp_disable_output_dev);
EXPORT_SYMBOL(omap2_disp_enable_output_dev);
EXPORT_SYMBOL(omap2_disp_set_tvstandard);
EXPORT_SYMBOL(omap2_disp_get_tvstandard);
EXPORT_SYMBOL(omap2_disp_set_tvlcd);
EXPORT_SYMBOL(omap2_disp_get_tvlcd);
EXPORT_SYMBOL(omap2_disp_config_lcd);
EXPORT_SYMBOL(omap2_disp_lcdcfg_polfreq);
EXPORT_SYMBOL(omap2_disp_set_pcd);

EXPORT_SYMBOL(omap2_disp_set_dma_params);
EXPORT_SYMBOL(omap2_disp_get_output_dev);
EXPORT_SYMBOL(omap2_disp_set_output_dev);

#ifdef CONFIG_ARCH_OMAP34XX
EXPORT_SYMBOL(omap2_disp_set_alphablend);
EXPORT_SYMBOL(omap2_disp_get_alphablend);
EXPORT_SYMBOL(omap2_disp_set_global_alphablend_value);
EXPORT_SYMBOL(omap2_disp_get_global_alphablend_value);
#endif

#ifndef CONFIG_OMAP_USE_DSI_PLL	
EXPORT_SYMBOL(omap2_disp_set_dssfclk);
#endif
EXPORT_SYMBOL(omap2_disp_get_dss);
EXPORT_SYMBOL(omap2_disp_put_dss);
EXPORT_SYMBOL(omap2_disp_get_all_clks);
EXPORT_SYMBOL(omap2_disp_put_all_clks);
EXPORT_SYMBOL(omap2_disp_set_default_colorconv);
EXPORT_SYMBOL(omap2_disp_set_colorconv);
EXPORT_SYMBOL(omap2_disp_set_colorkey);
EXPORT_SYMBOL(omap2_disp_get_colorkey);
EXPORT_SYMBOL(omap2_disp_enable_colorkey);
EXPORT_SYMBOL(omap2_disp_disable_colorkey);
EXPORT_SYMBOL(omap2_disp_reg_sync_bit);

EXPORT_SYMBOL(omap2_disp_set_gfx_palette);
EXPORT_SYMBOL(omap2_disp_pixels_per_clock);

EXPORT_SYMBOL(omap2_disp_set_vrfb);
EXPORT_SYMBOL(omap2_disp_save_initstate);
EXPORT_SYMBOL(omap2_disp_reg_sync);
EXPORT_SYMBOL(omap2_disp_reg_sync_done);
EXPORT_SYMBOL(omap2_disp_disable);

EXPORT_SYMBOL(omap2_disp_register_isr);
EXPORT_SYMBOL(omap2_disp_unregister_isr);
EXPORT_SYMBOL(omap2_disp_irqenable);
EXPORT_SYMBOL(omap2_disp_irqdisable);

EXPORT_SYMBOL(current_colorconv_values);

EXPORT_SYMBOL(omap2_disp_set_irq_on_linenumber);
EXPORT_SYMBOL(omap2_disp_swap_dma_ba);
EXPORT_SYMBOL(omap2_disp_get_vrfb_offset);

//EXPORT_SYMBOL(omap2_disp_start_video_layer);
//EXPORT_SYMBOL(omap2_disp_set_addr);
