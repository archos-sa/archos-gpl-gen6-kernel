#ifndef _RADIO_TEA5766_H_
#define _RADIO_TEA5766_H_



enum {	
	SET_SEARCH_MODE = 0,
	SET_FREQUENCY,
	SEARCH_UP,
	SEARCH_DOWN,
	STOP_SEARCH,
	GET_SEARCH_STATUS,
	GET_CURRENT_FREQUENCY,
	GET_LEVEL,
	SET_SEARCH_DIR,

	STEP_UP,
	STEP_DOWN,

	SET_AHLSI,
	SET_SWPORT,
	SET_MONO_STEREO,
	SET_HLSI,
	TOGGLE_HLSI,
	SET_SSL,
	SET_HWMUTE,
	SET_SWMUTE,
	SET_AFMUTE,
	SET_IFCOUNT,
	SET_REGION,
	SET_POWERUP,
	SET_POWERDOWN,
	SET_STEREO_SENSITIVITY,
	SET_SNC,

	GET_FM_STATUS,
	GET_RDS_STATUS,

	SET_INIT,
	
	PRINT_REGS_NOW,
	ENABLE_DEBUG_PRINT,

	DUMMY_CMD,
	RDS_STOP,
	RDS_START,
};

//INTREG
#define TEA_5766_DAVFLG		0x8000
#define TEA_5766_LSYNCFL	0x2000
#define TEA_5766_IFFLAG		0x1000
#define TEA_5766_LEVFLAG	0x800
#define TEA_5766_FRRFLAG	0x200
#define TEA_5766_BLFLAG		0x100



//FRQSET
#define	TEA_5766_SUD		0x8000
#define TEA_5766_SUD_SEARCH_UP	1
#define TEA_5766_SUD_SEARCH_DOWN	0
#define	TEA_5766_SM		0x4000
#define TEA_5766_SM_SEARCH_MODE	1
#define TEA_5766_SM_PRESET_MODE	0
#define TEA_5766_FR_MASK	0x3FFF

//TNCTRL
#define TEA_5766_PUPD_MASK		0xc000
#define TEA_5766_PUPD_OFFSET		14
#define TEA_5766_PUPD_FM_OFF_RDS_OFF	0
#define TEA_5766_PUPD_FM_ON_RDS_OFF	1
#define TEA_5766_PUPD_FM_ON_RDS_ON	3
#define TEA_5766_BLIM		0x2000
#define TEA_5766_BLIM_JAPAN	1
#define TEA_5766_BLIM_US_EU	0
#define TEA_5766_SWPM		0x1000
#define TEA_5766_SWPM_FRRFLAG	1
#define TEA_5766_SWPM_SWP	0
#define TEA_5766_IFCT		0x0800
#define TEA_5766_IFCT_15_625	1
#define TEA_5766_IFCT_2_02	0
#define TEA_5766_AFM		0x0400
#define TEA_5766_AFM_MUTED	1
#define TEA_5766_AFM_NOT_MUTED	0
#define TEA_5766_SMUTE		0x0200
#define TEA_5766_SMUTE_ON	1
#define TEA_5766_SMUTE_OFF	0
#define TEA_5766_SNC		0x0100
#define TEA_5766_SNC_ON		1
#define TEA_5766_SNC_OFF		0
#define TEA_5766_MU		0x0080
#define TEA_5766_MU_ON		1
#define TEA_5766_MU_OFF		0
#define TEA_5766_SSL_MASK	0x0060
#define TEA_5766_SSL_OFFSET	5
#define TEA_5766_SSL_3		0
#define TEA_5766_SSL_5		1
#define TEA_5766_SSL_7		2
#define TEA_5766_SSL_10		3
#define TEA_5766_HLSI		0x0010
#define TEA_5766_HLSI_HIGH_SIDE	1
#define TEA_5766_HLSI_LOW_SIDE	0
#define TEA_5766_MST		0x0008
#define TEA_5766_MST_MONO	1
#define TEA_5766_MST_STEREO	0
#define TEA_5766_SWP		0x0004
#define TEA_5766_SWP_HIGH	1
#define TEA_5766_SWP_LOW		0
#define TEA_5766_DTC		0x0002
#define TEA_5766_DTC_50us	1
#define TEA_5766_DTC_75us	0
#define TEA_5766_AHLSI		0x0001
#define TEA_5766_AHSLI_STOP	1
#define TEA_5766_AHSLI_CONTINU	0

//FRQCHK
#define TEA_5766_PLL		0x3FFF

//TUNCHK
#define TEA_5766_IF_COUNT_MASK	0xFE00
#define TEA_5766_IF_COUNT_OFFSET 9
#define	TEA_5766_TUNTO		0x0100
#define TEA_5766_TUNTO_TIMEOUT	1
#define TEA_5766_TUNTO_SETTLED	0
#define TEA_5766_LEV_MASK	0x00F0
#define TEA_5766_LEV_OFFSET	4
#define TEA_5766_LD		0x0008
#define TEA_5766_LD_LOCKED	1
#define TEA_5766_LD_NOT_LOCKED	0
#define TEA_5766_MONO_STEREO	0x0004
#define TEA_5766_STEREO		1
#define TEA_5766_MONO		0

// INTREG register
// TODO: bits
// INIT value
#define INTREG_INIT		0x0000


// TESTREG register
#define TEA_5766_SNCLEV		0x0020
#define TEA_5766_SNCLEV_LOW	1
#define TEA_5766_SNCLEV_HIGH	0

// bit 13 (AFM)
#define TESTREG_AFMDIS_B	13
#define TESTREG_AFMDIS_MASK	(0x01<<(TESTREG_AFMDIS_B))
#define TESTREG_AFMDIS_ON	1
#define TESTREG_AFMDIS_OFF	0
// bit 4 (Software test mode)
#define TESTREG_TM_B		4
#define TESTREG_TM_MASK		(0x01<<(TESTREG_TM_B))
#define TESTREG_TM_TEST		1
#define TESTREG_TM_NORMAL	0
// INIT
#define TESTREG_INIT		0

// MANID register
#define TEA576X_MAN_ID			0x2b // NXP
#define TEA576X_MAN_ID_MASK		0xfff

// CHIPID register
#define TEA5766_CHIP_ID			0x5766

#define PSN_NAME_LEN		9
typedef struct {
	unsigned int freq_kHz;	/* current frequency */
	unsigned int stereo;	/* stereo detected */
	unsigned int rf_level;	/* rf power */
	unsigned int if_count;	/* quality info */
	unsigned int tune_timeout;
} radio_fm_status_t;

typedef struct {
	unsigned int	rds_sync;
	unsigned int 	pi;	/* prgme id */
	unsigned int 	freq_kHz;	/* current frequency */
	unsigned char 	prgm_service_name[PSN_NAME_LEN];/* 8 char + \0 */
} radio_rds_status_t;
#endif // _RADIO_TEA5766_H_
