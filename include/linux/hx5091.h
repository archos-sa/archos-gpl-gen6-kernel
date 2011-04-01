#ifndef _LINUX_HX5091_H
#define _LINUX_HX5091_H

// HX5078-A registers for test on evm with 4"3 lcd

#define  HX5078_TEST			0x00
#define  HX5078_ID			0x01
#define  HX5078_RESOLUTION		0x02
#define  HX5078_OUTPUT			0x03
#define  HX5078_MODE			0x04
#define  HX5078_HPOS			0x05
#define  HX5078_VPOS			0x06
#define  HX5078_CKHWIDTH		0x07
#define  HX5078_CKH			0x08
#define  HX5078_ENBCKH			0x09
#define  HX5078_ENB			0x0a
#define  HX5078_RGAIN			0x0b
#define  HX5078_GGAIN			0x0c
#define  HX5078_BGAIN			0x0d
#define  HX5078_ROFFSET			0x0e
#define  HX5078_GOFFSET			0x0f
#define  HX5078_BOFFSET			0x10
#define  HX5078_HGAMMA0			0x11
#define  HX5078_HGAMMA64		0x12
#define  HX5078_HGAMMA224		0x13
#define  HX5078_LGAMMA0			0x14
#define  HX5078_LGAMMA8			0x15
#define  HX5078_LGAMMA16		0x16
#define  HX5078_LGAMMA32		0x17
#define  HX5078_LGAMMA64		0x18
#define  HX5078_LGAMMA96		0x19
#define  HX5078_LGAMMA128		0x1a
#define  HX5078_LGAMMA192		0x1b
#define  HX5078_LGAMMA224		0x1c
#define  HX5078_LGAMMA240		0x1d
#define  HX5078_LGAMMA248		0x1e
#define  HX5078_LGAMMA256		0x1f
#define  HX5078_POSITIVEGAMMA		0x20
#define  HX5078_NEGATIVEGAMMA		0x21
#define  HX5078_DCCOM			0x22

// HX5091-A registers for 4"8 lcd

#define  HX5091_HW			0x00
#define  HX5091_INTERLACED		0x01
#define  HX5091_VDISP			0x02
#define  HX5091_HDISP			0x03
#define  HX5091_NDI			0x04
#define  HX5091_INVERT			0x05
#define  HX5091_POL			0x06
#define  HX5091_MODE			0x07
#define  HX5091_STEPUP			0x08
#define  HX5091_BSEQ			0x09
#define  HX5091_BNO			0x0a
#define  HX5091_BON			0x0b
#define  HX5091_GNO			0x0c
#define  HX5091_GSD			0x0d
#define  HX5091_PON			0x0e
#define  HX5091_PCS			0x0f
#define  HX5091_POSITIVEGAMMA		0x10
#define  HX5091_NEGATIVEGAMMA		0x11
#define  HX5091_VCOM			0x12
#define  HX5091_RGAIN			0x13
#define  HX5091_GGAIN			0x14
#define  HX5091_BGAIN			0x15
#define  HX5091_ROFFSET			0x16
#define  HX5091_GOFFSET			0x17
#define  HX5091_BOFFSET			0x18

#define  HX5091_RGAMMA0			0x20
#define  HX5091_RGAMMA1			0x21
#define  HX5091_RGAMMA16		0x22
#define  HX5091_RGAMMA32		0x23
#define  HX5091_RGAMMA64		0x24
#define  HX5091_RGAMMA96		0x25
#define  HX5091_RGAMMA128		0x26
#define  HX5091_RGAMMA192		0x27
#define  HX5091_RGAMMA224		0x28
#define  HX5091_RGAMMA240		0x29
#define  HX5091_RGAMMA248		0x2a
#define  HX5091_RGAMMA256		0x2b
#define  HX5091_HRGAMMA1		0x2c
#define  HX5091_HRGAMMA2		0x2d
#define  HX5091_HRGAMMA3		0x2e
#define  HX5091_SGAMMA			0x2f

#define  HX5091_PGGAMMA			0x30
#define  HX5091_PBGAMMA			0x40
#define  HX5091_NRGAMMA			0x50
#define  HX5091_NGGAMMA			0x60
#define  HX5091_NBGAMMA			0x70

typedef struct {
	int reg;
	int val;
	} hx5091_t;

#endif /* _LINUX_HX5091_H */
