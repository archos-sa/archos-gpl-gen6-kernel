/*
 * TVP5146 - Texas Instruments TVP5146 video decoder registers
 *
 * This code is placed under the terms of the GNU General Public License v2
 */

#define TVP5146_VD_IN_SRC_SEL      0x00 /* Video input source selection #1 */
#define TVP5146_AFE_GAIN_CTL         0x01 /* AFE gain  controls */
#define TVP5146_VIDEO_STD           0x02 /* Video standard */
#define TVP5146_OP_MODE_CTL          0x03 /* Operation mode controls */
#define TVP5146_AUTOSW_MSK           0x04 /* Autoswitch mask: TVP5146A / TVP5146AM */

#define TVP5146_COLOR_KIL_THSH_CTL   0x05 /* Color killer threshold control */
#define TVP5146_LUMA_PROC_CTL_1      0x06 /* Luminance processing control #1 */
#define TVP5146_LUMA_PROC_CTL_2      0x07 /* Luminance processing control #2 */
#define TVP5146_LUMA_PROC_CTL_3      0x08 /* Luminance processing control #3 */
#define TVP5146_BRIGHT_CTL           0x09 /* Brightness control for cvbs and svideo */
#define TVP5146_SATURATION_CTL       0x0b /* Color saturation control for cvbs and svideo */
#define TVP5146_HUE_CTL              0x0c /* Hue control for cvbs and svideo */
#define TVP5146_CONTRAST_CTL         0x0a /* Contrast control for cvbs and svideo*/
#define TVP5146_CHROMA_PROC_CTL_1    0x0d /* Chrominance processing control #1 */
#define TVP5146_CHROMA_PROC_CTL_2    0x0e /* Chrominance processing control #2 */
/* Reserved 0fh */

#define TVP5146_COMP_PR_SAT   0x10 /* Component Pr saturation */
#define TVP5146_COMP_Y_CONT   0x11 /* Component Y contrast */
#define TVP5146_COMP_PB_SAT   0x12 /* Component Pb saturation */
/* Reserved 13h */
#define TVP5146_COMP_Y_BRIGHT   0x14 /* Component Y brightness */
/* Reserved 15h */

#define TVP5146_AVID_START_PIX1     0x16 /* Horizontal start pixel MSB */
#define TVP5146_AVID_START_PIX2     0x17 /* Horizontal start pixel LSB */
#define TVP5146_AVID_STOP_PIX1     0x18 /* Horizontal stop pixel MSB */
#define TVP5146_AVID_STOP_PIX2     0x19 /* Horizontal stop pixel LSB */

#define TVP5146_HSYNC_START1     0x1a /* Horizontal sync start MSB */
#define TVP5146_HSYNC_START2     0x1b /* Horizontal sync start LSB */
#define TVP5146_HSYNC_STOP1     0x1c /* Horizontal sync stop MSB */
#define TVP5146_HSYNC_STOP2     0x1d /* Horizontal sync stop LSB */

#define TVP5146_VSYNC_START1     0x1e /* Horizontal sync start MSB */
#define TVP5146_VSYNC_START2     0x1f /* Horizontal sync start LSB */
#define TVP5146_VSYNC_STOP1     0x20 /* Horizontal sync stop MSB */
#define TVP5146_VSYNC_STOP2     0x21 /* Horizontal sync stop LSB */

#define TVP5146_VBLANKING_START1 0x22 /* Vertical blanking start */
#define TVP5146_VBLANKING_START2 0x23 /* Vertical blanking start */
#define TVP5146_VBLANKING_STOP1  0x24 /* Vertical blanking stop */
#define TVP5146_VBLANKING_STOP2  0x25 /* Vertical blanking stop */

/* Reserved 26h */
/* Reserved 27h */

#define TVP5146_FAST_SWITCH_CTL      0x28 /* Miscellaneous controls */
/* Reserved 29h */

#define TVP5146_FAST_SWITCH_SCART    0x2a /* Miscellaneous controls */
/* Reserved 2bh */
#define TVP5146_SCART_DELAY          0x2c /* Miscellaneous controls */

#define TVP5146_CTI_DELAY 	     0x2d /* Miscellaneous controls */
#define TVP5146_CTI_CTL 	     0x2e /* Miscellaneous controls */
/* Reserved 2fh */
/* Reserved 30h */

#define TVP5146_RTC	 	     0x31 /* Miscellaneous controls */
#define TVP5146_SYNC_CTL 	     0x32 /* Miscellaneous controls */
#define TVP5146_OUTPUT_FORMATER1     0x33 /* Miscellaneous controls */
#define TVP5146_OUTPUT_FORMATER2     0x34 /* Miscellaneous controls */
#define TVP5146_OUTPUT_FORMATER3     0x35 /* Miscellaneous controls */
#define TVP5146_OUTPUT_FORMATER4     0x36 /* Miscellaneous controls */
#define TVP5146_OUTPUT_FORMATER5     0x37 /* Miscellaneous controls */
#define TVP5146_OUTPUT_FORMATER6     0x38 /* Miscellaneous controls */

#define TVP5146_CLR_LOCK_DETECT      0x39 /* Miscellaneous controls */

#define TVP5146_STATUS_REG_1         0x3a /* Status register #1 */
#define TVP5146_STATUS_REG_2         0x3b /* Status register #2 */

#define TVP5146_ACG_GAIN1	     0x3c /* Miscellaneous controls */
#define TVP5146_ACG_GAIN2	     0x3d /* Miscellaneous controls */
/* Reserved 3eh */
#define TVP5146_STD_STATUS           0x3f /* Standart Status register */

#define TVP5146_GPIO_IN1             0x40 /* gpio input #1 */
#define TVP5146_GPIO_IN2             0x41 /* gpio input #2 */

#define TVP5146_VLINE_COUNT1         0x42 /* vertical line count #1 */
#define TVP5146_VLINE_COUNT2         0x43 /* vertical line count #2 */
/* Reserved 44h */
/* Reserved 45h */

#define TVP5146_AFE_GAIN_CH1         0x46 /* Status register #2 */
#define TVP5146_AFE_GAIN_CH2         0x47 /* Status register #2 */
#define TVP5146_AFE_GAIN_CH3         0x48 /* Status register #2 */
#define TVP5146_AFE_GAIN_CH4         0x49 /* Status register #2 */

#define TVP5146_AFE_GAIN_PB1         0x4a /* Status register #2 */
#define TVP5146_AFE_GAIN_PB2         0x4b /* Status register #2 */
#define TVP5146_AFE_GAIN_YG1         0x4c /* Status register #2 */
#define TVP5146_AFE_GAIN_YG2         0x4d /* Status register #2 */
#define TVP5146_AFE_GAIN_PR1         0x4e /* Status register #2 */
#define TVP5146_AFE_GAIN_PR2         0x4f /* Status register #2 */
#define TVP5146_AFE_GAIN_CVBS1       0x50 /* Status register #2 */
#define TVP5146_AFE_GAIN_CVBS2       0x51 /* Status register #2 */

/* Reserved 52h to 6fh*/

#define TVP5146_ROM_VERSION          0x70 /* Miscellaneous controls */
/* Reserved 71h to 73h*/
#define TVP5146_ACG_PEAK_PROC	     0x74 /* Miscellaneous controls */

/* Reserved 75h to 77h*/
#define TVP5146_ACG_INC_SPEED	     0x78 /* Miscellaneous controls */
#define TVP5146_ACG_INC_DELAY	     0x79 /* Miscellaneous controls */
/* Reserved 7ah to 7fh*/

#define TVP5146_CHIP_ID_MSB          0x80 /*  MSB */
#define TVP5146_CHIP_ID_LSB          0x81 /*  LSB */

/* Reserved 82h to b0h*/

#define TVP5146_TTX_F1_MASK1         0xb1 /* MSB of device ID */
#define TVP5146_TTX_F1_MASK2         0xb2 /* MSB of device ID */
#define TVP5146_TTX_F1_MASK3         0xb3 /* MSB of device ID */
#define TVP5146_TTX_F1_MASK4         0xb4 /* MSB of device ID */
#define TVP5146_TTX_F1_MASK5         0xb5 /* MSB of device ID */
#define TVP5146_TTX_F2_MASK1         0xb6 /* MSB of device ID */
#define TVP5146_TTX_F2_MASK2         0xb7 /* MSB of device ID */
#define TVP5146_TTX_F2_MASK3         0xb8 /* MSB of device ID */
#define TVP5146_TTX_F2_MASK4         0xb9 /* MSB of device ID */
#define TVP5146_TTX_F2_MASK5         0xba /* MSB of device ID */
#define TVP5146_TTX_CONTROL          0xbb /* MSB of device ID */
#define TVP5146_VDP_FIFO_COUNT       0xbc /* MSB of device ID */
#define TVP5146_VDP_FIFO_INT_THRESHOLD       0xbd /* MSB of device ID */

#define TVP5146_VDP_FIFO_RESET       0xbf /* MSB of device ID */
#define TVP5146_VDP_FIFO_OUTCTL      0xc0 /* MSB of device ID */
#define TVP5146_VDP_LINE_INT         0xc1 /* MSB of device ID */
#define TVP5146_VDP_PIX_ALIGN1       0xc2 /* MSB of device ID */
#define TVP5146_VDP_PIX_ALIGN2       0xc3 /* MSB of device ID */
/* Reserved c4h  to d5h*/
#define TVP5146_VDP_LINE_START       0xd6 /* MSB of device ID */
#define TVP5146_VDP_LINE_STOP        0xd7 /* MSB of device ID */
#define TVP5146_VDP_LINE_MODE        0xd8 /* MSB of device ID */
#define TVP5146_VDP_FULL_ENABLE      0xd9 /* MSB of device ID */
#define TVP5146_VDP_FULL_MODE        0xda /* MSB of device ID */
/* Reserved dbh  to dfh*/

/* VBUS access with no address increment */
#define TVP5146_VBUS_DATA_ACCESS1    0xe0 /* MSB of device ID */
/* VBUS access with address increment */
#define TVP5146_VBUS_DATA_ACCESS2    0xe1 /* MSB of device ID */
#define TVP5146_FIFO_READ	     0xe2 /* MSB of device ID */

/* Reserved	e3h-e7h */
#define TVP5146_VBUS_ADRESS1	     0xe8 /* MSB of device ID */
#define TVP5146_VBUS_ADRESS2	     0xe9 /* MSB of device ID */
#define TVP5146_VBUS_ADRESS3	     0xea /* MSB of device ID */

/* Reserved	ebh-eFh */

#define TVP5146_INT_RAW_STATUS0      0xf0 /* MSB of device ID */
#define TVP5146_INT_RAW_STATUS1      0xf1 /* MSB of device ID */

#define TVP5146_INT_STATUS0          0xf2 /* MSB of device ID */
#define TVP5146_INT_STATUS1          0xf3 /* MSB of device ID */
#define TVP5146_INT_MASK0            0xf4 /* MSB of device ID */
#define TVP5146_INT_MASK1            0xf5 /* MSB of device ID */
#define TVP5146_INT_CLEAR0           0xf6 /* MSB of device ID */
#define TVP5146_INT_CLEAR1           0xf7 /* MSB of device ID */
/* Reserved	F8h-FFh */

/*****************************************************/

#define TVP5146_STATUS_STANDARD_MASK	0x07
#define TVP5146_POWER_SAVE 0x01

// gen6 GPIO control for RGB analog video-out
#define TVP5146_GPIO_C6_MASK		0x01
#define TVP5146_GPIO_C7_MASK		0x04

#define TVP5146_GPIO_C6_OUTPUT		0x02
#define TVP5146_GPIO_C7_OUTPUT		0x08

// input configuration for dvr205
#define TVP5146_INPUT_CVBS1	0x0
#define TVP5146_INPUT_CVBS4	0x0c
#define TVP5146_INPUT_SVIDEO	0x5c
#define TVP5146_INPUT_RGB	0x84
#define TVP5146_INPUT_YPbPr	0x94
#define TVP5146_INPUT_SCART	0xCC


#define TVP5146_STANDARD_UNKNOWN	0
#define TVP5146_STANDARD_NTSC_M		1
#define TVP5146_STANDARD_PAL_BGHIN	2
#define TVP5146_STANDARD_PAL_M		3
#define TVP5146_STANDARD_PAL_N		4
#define TVP5146_STANDARD_NTSC443	5
#define TVP5146_STANDARD_SECAM		6
#define TVP5146_STANDARD_PAL60		7

#define TVP5146_AUTOSWITCH_MASK		0x3f	// All video formats are avalaible

/* VDP status register helpers */
#define TVP5146_VDP_VITC_AVAILABLE	0x01
#define TVP5146_VDP_VPS_AVAILABLE	0x02
#define TVP5146_VDP_WSS_AVAILABLE	0x04
#define TVP5146_VDP_CC1_AVAILABLE	0x08
#define TVP5146_VDP_CC2_AVAILABLE	0x10

/* status registers helpers */
#define TVP5146_FIELD_1			1<<4
#define TVP5146_FIELD_RATE_50		1<<5

/* Status Register #2 helpers */
#define TVP5146_MACROVISION_DETECT_TYPE1	0x01
#define TVP5146_MACROVISION_DETECT_TYPE2	0x03
#define TVP5146_MACROVISION_DETECT_TYPE3	0x07

/* Maximum number of the General Line Registers */
#define GENERAL_LINEREGS_NUM		9

struct i2c_reg_value {
	unsigned char reg;
	unsigned char value;
};

#define	TVP51XX_SET_REG   _IOW('d', 191, struct i2c_reg_value)

