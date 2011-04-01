
#ifndef _LINUX_TEA576X_H
#define _LINUX_TEA576X_H

#include <linux/types.h>


#define TEA576X_READ	0
#define TEA576X_WRITE	1

#define TEA576X_VERSION		"1.0.0"
#define TEA576X_DATE		"27-December-2006"

#define TEA576X_FM_ADDR		0x20
#define TEA5766_RDS_ADDR	0x22

#define NB_BYTES_FM_READ_BY_ACTION	12
#define NB_BYTES_RDS_READ_BY_ACTION	16

#define NB_BYTES_FM_WRITE_BY_ACTION	7
#define NB_BYTES_RDS_WRITE_BY_ACTION	4


// 16 bit register definition of NXP FM receiver TEA5766
typedef struct {
	u_int16_t intreg;
	u_int16_t frqset;
	u_int16_t tnctrl;
	u_int16_t frqchk;
	u_int16_t tunchk;
	u_int16_t testreg;
} radio_tea5766_fm_regs_t;

// 16 bit register definition of NXP FM receiver TEA5766 - RDS
typedef struct {
	u_int16_t rdsr1;
	u_int16_t rdsr2;
	u_int16_t rdsr3;
	u_int16_t rdsr4;
	u_int16_t rdsw1;
	u_int16_t rdsw2;
	u_int16_t manid;
	u_int16_t chipid;
} radio_tea5766_rds_regs_t;


#endif // _LINUX_TEA576X_H
