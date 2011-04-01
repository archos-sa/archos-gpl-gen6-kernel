/* linux/include/asm-arm/arch-omap/tps65023.h
 *
 * Functions to access TPS65023 power management device.
 *
 * Copyright (C) 2007  <archos.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __ASM_ARCH_TPS65023_H
#define __ASM_ARCH_TPS65023_H

#define TPS65023_ADD 0x48

/*
 * ----------------------------------------------------------------------------
 * Registers, all 8 bits
 * ----------------------------------------------------------------------------
 */
#define	TPS_VERSION		0x00
#	define TPS_65023		0x23

#define	TPS_PGOODSTATUS		0x01
#	define	TPS_PGOODSTATUS_MASK	0xFE

#	define	TPS_PGOOD_PWRFAIL	(1 << 7)
#	define	TPS_PGOOD_LOWBATT	(1 << 6)
#	define	TPS_PGOOD_VDCDC1	(1 << 5)
#	define	TPS_PGOOD_VDCDC2	(1 << 4)
#	define	TPS_PGOOD_VDCDC3	(1 << 3)
#	define	TPS_PGOOD_LDO2		(1 << 2)
#	define	TPS_PGOOD_LDO1		(1 << 1)
#	define	TPS_PGOOD_NDEF		(1 << 0)


#define	TPS_INT_MASK		0x02
#	define	TPS_INT_MASK_MASK	0xFE

#	define	TPS_INT_PWRFAIL		(1 << 7)
#	define	TPS_INT_LOWBATT		(1 << 6)
#	define	TPS_INT_VDCDC1		(1 << 5)
#	define	TPS_INT_VDCDC2		(1 << 4)
#	define	TPS_INT_VDCDC3		(1 << 3)
#	define	TPS_INT_LDO2		(1 << 2)
#	define	TPS_INT_LDO1		(1 << 1)
#	define	TPS_INT_NDEF		(1 << 0)


#define	TPS_REG_CTRL		0x03
#	define	TPS_REG_CTRL_MASK	0x3E

#	define	TPS_REG_EN_VDCDC1	(1 << 5)
#	define	TPS_REG_EN_VDCDC2	(1 << 4)
#	define	TPS_REG_EN_VDCDC3	(1 << 3)
#	define	TPS_REG_EN_LDO2		(1 << 2)
#	define	TPS_REG_EN_LDO1		(1 << 1)
#	define	TPS_REG_NDEF		(1 << 0)

#define	TPS_CONV_CTRL1		0x04

#	define	TPS_PHASE1_DCDC2	(1 << 7)
#	define	TPS_PHASE0_DCDC2	(1 << 6)
#	define	TPS_PHASE1_DCDC3	(1 << 5)
#	define	TPS_PHASE0_DCDC3	(1 << 4)
#	define	TPS_LOW_RIPPLE		(1 << 3)
#	define	TPS_FPWM_DCDC2		(1 << 2)
#	define	TPS_FPWM_DCDC1		(1 << 1)
#	define	TPS_FPWM_DCDC3		(1 << 0)


#define	TPS_CONV_CTRL2		0x05
#	define	TPS_CONV_CTRL2_MASK	0xC7

#	define	TPS_GO			(1 << 7)
#	define	TPS_CORE_ADJ_NOT_ALLOWED	(1 << 6)
#	define	TPS_DISCH_DCDC2		(1 << 2)
#	define	TPS_DISCH_DCDC1		(1 << 1)
#	define	TPS_DISCH_DCDC3		(1 << 0)

// DCDC1 VOLTAGE VALUE
#define	TPS_DEFCORE		0x06
#	define	TPS_DEFCORE_MASK	0x1F

#	define	TPS_DCDC1_CONVERT(x)	( (x >= 800) ?  ((( x - 800 ) / 25) & TPS_DEFCORE_MASK ) : 0 )	

// DCDC1 SLEW RATE
#define	TPS_DEFSLEW		0x07
#	define	TPS_DEFSLEW_MASK	0x07

#	define	SLEW_RATE_IMMEDIATE	7
#	define	SLEW_RATE_14_4		6
#	define	SLEW_RATE_7_2		5
#	define	SLEW_RATE_3_6		4
#	define	SLEW_RATE_1_8		3
#	define	SLEW_RATE_0_9		2
#	define	SLEW_RATE_0_45		1
#	define	SLEW_RATE_0_225		0

// LDO Voltage Value
#define	TPS_LDO_CTRL		0x08
#	define	TPS_LDO_CTRL_MASK	0x77
#	define	TPS_LDO2_CTRL_MASK	0x70
#	define	TPS_LDO1_CTRL_MASK	0x07

#	define	TPS_VLDO2_3_3V	0x70
#	define	TPS_VLDO2_3_0V	0x60
#	define	TPS_VLDO2_2_8V	0x50
#	define	TPS_VLDO2_2_5V	0x40
#	define	TPS_VLDO2_1_8V	0x30
#	define	TPS_VLDO2_1_3V	0x20
#	define	TPS_VLDO2_1_2V	0x10
#	define	TPS_VLDO2_1_05V	0x00
#	define	TPS_VLDO1_3_15V	0x07
#	define	TPS_VLDO1_2_8V	0x06
#	define	TPS_VLDO1_2_6V	0x05
#	define	TPS_VLDO1_2_2V	0x04
#	define	TPS_VLDO1_1_8V	0x03
#	define	TPS_VLDO1_1_3V	0x02
#	define	TPS_VLDO1_1_1V	0x01
#	define	TPS_VLDO1_1V	0x00

#define LD01  1
#define LD02  2
#define OFF   0
#define ON    1

#define DCDC1 1
#define DCDC2 2
#define DCDC3 3

#define LOW   0
#define HIGH  1

/*
 * ----------------------------------------------------------------------------
 * Exported functions
 * ----------------------------------------------------------------------------
 */
// exemple of typical info for prm settings via i2c4
struct prm_interface {
	u8 address;
	u8 nb_dcdc;
	u8 voltage1_reg;
	u8 config1_reg;
	u8 voltage2_reg;
	u8 config2_reg;
};

static struct prm_interface tps65023_interface = {
	.address =  TPS65023_ADD,
	.nb_dcdc = 1,
	.voltage1_reg = TPS_DEFCORE,
	.config1_reg = TPS_CONV_CTRL2,
	.voltage2_reg = 0,
	.config2_reg = 0,
};

typedef struct {
	int reg;
	int val;
	int par1;
	int par2;
	} tps_param_t;

#define  TPS_GET_REG			_IOWR ( 'T', 0, tps_param_t*)
#define  TPS_SET_REG			_IOWR ( 'T', 1, tps_param_t*)
#define  TPS_ENA_DCDC			_IOWR ( 'T', 2, tps_param_t*)
#define  TPS_SET_DCVAL			_IOWR ( 'T', 3, tps_param_t*)
#define  TPS_ENA_DCVAL			_IOWR ( 'T', 4, tps_param_t*)
#define  TPS_DUMP			_IOWR ( 'T', 5, tps_param_t*)
#define  TPS_ENA_PM			_IOWR ( 'T', 6, tps_param_t*)


//-------------------------------------------------------------------------
//	tps65023_enable_DCDC:
//	mode: ON or OFF
//	dcdc_num: 1 to 3 
//-------------------------------------------------------------------------
extern int tps65023_enable_DCDC( int dcdc_num, int mode );

//-------------------------------------------------------------------------
//	tps65023_core_adjust:
//	mode: ON or OFF
//-------------------------------------------------------------------------
extern int tps65023_core_adjust( int enable );

//-------------------------------------------------------------------------
//	tps65023_set_DCDC1:
//	value:	value to set in mV 
//-------------------------------------------------------------------------
extern int tps65023_set_DCDC1( int value );

//-------------------------------------------------------------------------
//	tps65023_set_slew:
//	value:	slew rate for dcdc1 
//-------------------------------------------------------------------------
extern int tps65023_set_slew( int value );

//-------------------------------------------------------------------------
//	tps65023_enable_LDO:
//	mode: ON or OFF
//	ldo_num 
//-------------------------------------------------------------------------
extern int tps65023_enable_LDO( int ldo_num, int mode );

//-------------------------------------------------------------------------
//	tps65023_set_LDO:
//	value:  output level step 
//	ldo_num:
//-------------------------------------------------------------------------
extern int tps65023_set_LDO( int ldo_num, int value );

/* tps65023_set_low_pwr parameter:
 * mode: ON or OFF
 */
extern int tps65023_set_low_pwr(unsigned mode);

/* tps65023_config_vregs1 parameter:
 * value to be written to VREGS1 register
 * Note: The complete register is written, set all bits you need
 */
extern int tps65023_config_vregs1(unsigned value);

/* tps65013_set_low_pwr parameter:
 * mode: ON or OFF
 */
extern int tps65013_set_low_pwr(unsigned mode);
extern int tps65023_i2c_read( u8 value, u8 reg, u8 num_bytes);
extern int tps65023_i2c_write( u8 value, u8 reg, u8 num_bytes);

#endif /*  __ASM_ARCH_TPS65023_H */

