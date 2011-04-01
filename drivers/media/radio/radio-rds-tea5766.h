#ifndef _RADIO_RDS_TEA5766_H_
#define _RADIO_RDS_TEA5766_H_

#include <linux/tea576x.h>

/* special defines for rds data */
#define DAVC 2
#define LIST_SIZE	49

enum {
	eAType = 0,
	eBType,
	eCType,
	eDType,
	eC_Type,
	eEType,
	eE_Type,
	eNone
};

enum {
	FALSE = 0,
	TRUE = 1
};

/* special struct for rds processing */
typedef struct {
//bit 0 to bit 15
	__u16 dovf	:1;
	__u16 rstd	:1;
	__u16 sync	:1;
	__u16 epb	:2;
	__u16 bpid	:3;
	__u16 elb	:2;
	__u16 dummy1	:2;
	__u16 blid	:3;
	__u16 dummy0	:1;
} _rdsr1_field_st;

typedef struct {
//bit 0 to bit 15
	__u16 bbg	:5;
	__u16 dummy1	:5;
	__u16 dac	:2;
	__u16 rbds	:1;
	__u16 sym	:2;
	__u16 nwsy	:1;
} _rdsw1_field_st;

typedef union  {
	_rdsr1_field_st field;
	__u16		reg;
} _rdsr1_field;


typedef union  {
	_rdsw1_field_st field;
	__u16		reg;
} _rdsw1_field;

typedef struct {
//bit 0 to bit 32
	__u8 ebn	:2; // Error
	__u8 bidn	:3; // Block id
	__u8 rstd  	:1; // Reset
	__u8 dovf	:1; // Overflow
	__u8 sync	:1; // In Sync
} _status_field_st;

typedef union  {
	_status_field_st field;
	__u8		reg;
} _status_field;

typedef char string8[10]; /* 8 char + end of string */ 
//typedef char string64[66]; /* 64 char + end of string */ 


typedef struct {
//	string64 rt;		// radio text + \n
	int psn_ok;		// full psn received
	string8 psn;		// program service name
//	string8 rdsdate;	// rds date
//	string8 rdstime;	// rds time
	int pi;			// pi code
//	int pty;		// program type code
//	string8 ptyn;		// program type name + \n
//	int di;			// decoder information
//	int ms;			// music/speech flag
//	int tp;			// traffic program
//	int ta;			// trafic announcement
//	int eon;		// enhanced other networks
//	int ownlink;		// link info own transmitter
} _trdsbasic_t;

#if 0
typedef struct {
// AF definitions
	unsigned char AF1;	// AF frequency 1
	unsigned char AF2;	// AF frequency 2
	int nAF;		// nr of times received
} TAFdata_t;

typedef struct {
// Header definition
	unsigned char ParentF;	// Parent frequency
	unsigned char ListLen;	// Nr of AFs in list
	int nHead;		// nr of times received
} TAFheader_t;

typedef struct {
// AF list structure
	TAFheader_t Header;	// header definition
	TAFdata_t AFs[48];	// AF data
	unsigned char NrOfAFinList; // Nr of AFs in list
} TAFlist_t;

typedef TAFlist_t TAFlists_t[LIST_SIZE];

         // mapped frequency definitions
typedef struct {
	unsigned char AF; // mapped frequency
	int nAF;	  // nr of times received
} TMAFdata_t;

typedef TMAFdata_t TMAFlist_t[6];   // single mapped frequency list
typedef TMAFlist_t TMAFlists_t[LIST_SIZE];

typedef struct {
// EON definitions
// other network information
	int PI;			// pi code
	string8 PSN;		// program service name
	TAFlist_t AFL;		// af list
	TMAFlists_t MAF;	// mapped frequencies
	int NrOfMappedAFs;     // nr of mapped freqs
	int TP;			// traffic program
	int TA;			// traffic announcement
	int PTY;		// pty code
	string8 PINcode;	// prog item number code
	int UA_A;		// tbd
	int UA_B;		// tbd
	int Linking;		// linkage information
	int UA_F;		// tbd
} TEON_t;

typedef TEON_t TEONlists[LIST_SIZE];
#endif

void RDS_init(void);
void RDS_exit(void);
void RDS_ClearData(void) ;
void Process_RDSdata(radio_tea5766_rds_regs_t *pt_rds_data) ;
int RDS_get_basic_data(_trdsbasic_t * pdata);

#endif
