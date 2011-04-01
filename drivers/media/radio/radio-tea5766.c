/*
 *   NXP Semiconductor TEA5766 on AX05 Remote FM driver.
 *
 * Copyright (c) 2006 Archos
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>		/* Initdata			*/
#include <linux/delay.h>	/* udelay			*/
#include <linux/fs.h>
#include <asm/uaccess.h>	/* Copy to/from user		*/
#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/kthread.h>
#include <linux/suspend.h>
#include <asm/semaphore.h>
#include <linux/tea576x.h>
#include "radio-tea5766.h"
#include "radio-rds-tea5766.h"
#include <linux/freezer.h>


// Local Defines
#define HIGH_SENSITIVITY_LEV	5
#define HIGH_SENSITIVITY_IF_COUNT	3
#define LOW_SENSITIVITY_LEV	7
#define LOW_SENSITIVITY_IF_COUNT	2
#define GOOD_IF_COUNT	56
#define GOOD_IF_COUNT_HIGH_LIMIT	58
#define GOOD_IF_COUNT_LOW_LIMIT		53
#define RADIO_NAME	"radio-nxp"
#define DEFAULT_FREQ	87500		// default frequency (kHz)
#define FIF		225000		// intermediate frequency in Hz
#define XTAL_RSHIFT	15		// the reference frequency 32768 = 1<<15



enum {
	FREQ_STEP_DOWN = 0,
	FREQ_STEP_UP	
};


#define RDS_POLL_DELAY	(2*HZ/1000)	// poll RDS every 2 msec
#define SCAN_POLL_DELAY	(20*HZ/1000)	// poll every 20 msec
#define IDLE_POLL_DELAY	(HZ/5)		// poll every 200 msec

#define TUNE_FILTER_SIZE	10

#define FM_BAND_MIN_JAPAN	76000
#define FM_BAND_MAX_JAPAN	90000
#define FM_BAND_MIN_WESTERN	87500
#define FM_BAND_MAX_WESTERN	108000

static int radio_tea5766_fm_set_frequency(radio_tea5766_fm_regs_t *regs, int freq);

struct band_st {
	unsigned int low;
	unsigned int high;
	unsigned int grid;	
};


struct band_st band_limit[] = {
	[TEA_5766_BLIM_US_EU] = {
		.low	= FM_BAND_MIN_WESTERN,	// Min freq (kHz)
		.high	= FM_BAND_MAX_WESTERN,	// Max
		.grid	= 100,		// Frequency steps (kHz)
	},
	[TEA_5766_BLIM_JAPAN] = {
		.low	= FM_BAND_MIN_JAPAN,
		.high	= FM_BAND_MAX_JAPAN,
		.grid	= 100,
	}
};

typedef struct{
	unsigned long startfreq;
	int dir;
	int station_found;
}autosearch_t;

static struct i2c_client *fm_i2c_client_handle = NULL;
static struct i2c_client *rds_i2c_client_handle = NULL;
static struct semaphore radio_tea5766_fm_i2c_lock;
static int radio_tea5766_fm_use_count = 0;
static struct semaphore radio_tea5766_ioctl_lock;

static struct task_struct *autoscan_task_handle = NULL;
static struct task_struct *rds_task_handle = NULL;
static autosearch_t autosearch_info;
static int autosearch_sensitivity_level = HIGH_SENSITIVITY_LEV;
static int autosearch_sensitivity_ifcount = HIGH_SENSITIVITY_IF_COUNT;

static int fm_region = TEA_5766_BLIM_US_EU;
static int tea5766_dbg_print = 0;
static int rds_sync = 0;

// FM register functions
static void set_search_dir( radio_tea5766_fm_regs_t *regs, int dir );
static void set_search_mode( radio_tea5766_fm_regs_t *regs, int mode );
static void put_frequencyOnGrid( unsigned long *kHz, int region );
static void set_frequency( radio_tea5766_fm_regs_t *regs, unsigned long kHz);
static unsigned long get_frequency_found( radio_tea5766_fm_regs_t *regs );
static int step_freq( radio_tea5766_fm_regs_t *regs, int dir );

static void set_ahlsi( radio_tea5766_fm_regs_t *regs, int arg );
static void set_hlsi( radio_tea5766_fm_regs_t *regs, int arg );
static void set_mono( radio_tea5766_fm_regs_t *regs, int arg );
static void set_ssl( radio_tea5766_fm_regs_t *regs, int mode );
static void set_hwmute( radio_tea5766_fm_regs_t *regs, int arg );
static void set_swmute( radio_tea5766_fm_regs_t *regs, int arg );
static void set_IFcount( radio_tea5766_fm_regs_t *regs, int arg );
static void set_swport( radio_tea5766_fm_regs_t *regs, int arg );
static void set_region( radio_tea5766_fm_regs_t *regs, int region );
static int get_region( radio_tea5766_fm_regs_t *regs );
static void set_power( radio_tea5766_fm_regs_t *regs, int action );
static void fm_tuner_init( radio_tea5766_fm_regs_t *regs );



//////////////////////////////////////////////////////////////////////////////
// Register read & write
//////////////////////////////////////////////////////////////////////////////

static void print_fm_regs_intreg( unsigned short intreg )
{
	printk( KERN_INFO "INTREG : ");
	if(intreg & TEA_5766_DAVFLG){
		printk( KERN_INFO "| DAVFLG |");
	}
	if(intreg & TEA_5766_LSYNCFL){
		printk( KERN_INFO "| LSYNCFL |");
	}
	if(intreg & TEA_5766_IFFLAG){
		printk( KERN_INFO "| IFFLAG |");
	}
	if(intreg & TEA_5766_LEVFLAG){
		printk( KERN_INFO "| LEVFLAG |");
	}
	if(intreg & TEA_5766_FRRFLAG){
		printk( KERN_INFO "| FRRFLAG |");
	}
	if(intreg & TEA_5766_BLFLAG){
		printk( KERN_INFO "| BLFLAG |");
	}
	printk( KERN_INFO "\r\n");
}

static void print_fm_regs_frqset( unsigned short frqset )
{
	printk( KERN_INFO "FRQSET : ");
	if(frqset&TEA_5766_SUD){
		printk( KERN_INFO "| SUD_SEARCH_UP |");
	} else {
		printk( KERN_INFO "| SUD_SEARCH_DOWN |");
	}
	if(frqset&TEA_5766_SM){
		printk( KERN_INFO "| SEARCH_MODE |");
	}else{
		printk( KERN_INFO "| PRESET_MODE |");
	}
	printk( KERN_INFO "| FR = %d ",frqset&TEA_5766_FR_MASK);
	
	printk( KERN_INFO "\r\n");
}

static void print_fm_regs_tnctrl( unsigned short tnctrl )
{
	printk( KERN_INFO "TNCTRL : ");
	if((tnctrl & TEA_5766_PUPD_MASK)>>TEA_5766_PUPD_OFFSET == TEA_5766_PUPD_FM_ON_RDS_ON){
		printk( KERN_INFO "| FM_ON_RDS_ON |");
	} else if((tnctrl & TEA_5766_PUPD_MASK)>>TEA_5766_PUPD_OFFSET == TEA_5766_PUPD_FM_ON_RDS_OFF){
		printk( KERN_INFO "| FM_ON_RDS_OFF |");
	} else {
		printk( KERN_INFO "| FM_OFF_RDS_OFF |");
	}
	if(tnctrl & TEA_5766_BLIM){
		printk( KERN_INFO "| JAPAN |");
	}else{
		printk( KERN_INFO "| US_EU |");
	}
	if(tnctrl & TEA_5766_SWPM){
		printk( KERN_INFO "| SWPM_FRRFLAG |");
	}else{
		printk( KERN_INFO "| SWPM_SWP |");
	}
	if(tnctrl & TEA_5766_IFCT){
		printk( KERN_INFO "| IFCT_15ms |");
	}else{
		printk( KERN_INFO "| IFCT_2ms |");
	}
	if(tnctrl & TEA_5766_AFM){
		printk( KERN_INFO "| AFM_MUTED |");
	}
	if(tnctrl & TEA_5766_SMUTE){
		printk( KERN_INFO "| SMUTE |");
	}
	if(tnctrl & TEA_5766_SNC){
		printk( KERN_INFO "| SNC |");
	}
	if(tnctrl & TEA_5766_MU){
		printk( KERN_INFO "| HMUTE |");
	}
	if(((tnctrl & TEA_5766_SSL_MASK)>>TEA_5766_SSL_OFFSET) == TEA_5766_SSL_10){
		printk( KERN_INFO "| SSL10 |");
	} else if(((tnctrl & TEA_5766_SSL_MASK)>>TEA_5766_SSL_OFFSET) == TEA_5766_SSL_7){
		printk( KERN_INFO "| SSL7 |");
	} else if(((tnctrl & TEA_5766_SSL_MASK)>>TEA_5766_SSL_OFFSET) == TEA_5766_SSL_5){
		printk( KERN_INFO "| SSL5 |");
	} else {
		printk( KERN_INFO "| SSL3 |");
	}
	if(tnctrl & TEA_5766_HLSI){
		printk( KERN_INFO "| HLSI_HIGH_SIDE |");
	}else{
		printk( KERN_INFO "| HLSI_LOW_SIDE |");
	}
	if(tnctrl & TEA_5766_MST){
		printk( KERN_INFO "| MONO |");
	}else{
		printk( KERN_INFO "| STEREO |");
	}
	if(tnctrl & TEA_5766_SWP){
		printk( KERN_INFO "| SWP HIGH |");
	}else{
		printk( KERN_INFO "| SWP LOW |");
	}
	if(tnctrl & TEA_5766_DTC){
		printk( KERN_INFO "| DTC 50us |");
	}else{
		printk( KERN_INFO "| DTC 75us |");
	}
	if(tnctrl & TEA_5766_AHLSI){
		printk( KERN_INFO "| AHLSI_STOP |");
	}else{
		printk( KERN_INFO "| AHLSI_CONT |");
	}
	printk( KERN_INFO "\r\n");
}

static void print_fm_regs_tunchk( unsigned short tunchk )
{
	printk( KERN_INFO "TUNCHK : ");
	printk( KERN_INFO "| IF = %d ",((tunchk&TEA_5766_IF_COUNT_MASK)>>TEA_5766_IF_COUNT_OFFSET));
	if(tunchk & TEA_5766_TUNTO){
		printk( KERN_INFO "| TUNE TO |");
	} else{
		printk( KERN_INFO "| TUNE SETTLED |");
	} 
	printk( KERN_INFO "| LEV = %d ",((tunchk&TEA_5766_LEV_MASK)>>TEA_5766_LEV_OFFSET));
	if(tunchk & TEA_5766_LD){
		printk( KERN_INFO "| PLL LOCKED |");
	}else{
		printk( KERN_INFO "| PLL NOT LOCKED |");
	}
	if(tunchk & TEA_5766_MONO_STEREO){
		printk( KERN_INFO "| STEREO |");
	}else{
		printk( KERN_INFO "| MONO |");
	}
	printk( KERN_INFO "\r\n");
}

static void print_fm_regs( radio_tea5766_fm_regs_t *regs )
{
// 	printk( KERN_INFO "regs->intreg  = %04x\r\n", regs->intreg );
	print_fm_regs_intreg(regs->intreg);
// 	printk( KERN_INFO "regs->frqset  = %04x\r\n", regs->frqset );
	print_fm_regs_frqset(regs->frqset);
// 	printk( KERN_INFO "regs->tnctrl  = %04x\r\n", regs->tnctrl );
	print_fm_regs_tnctrl(regs->tnctrl);
	printk( KERN_INFO "regs->frqchk  = %04x (%d)\r\n", regs->frqchk ,(regs->frqchk&TEA_5766_PLL));
// 	printk( KERN_INFO "regs->tunchk  = %04x\r\n", regs->tunchk );
	print_fm_regs_tunchk(regs->tunchk);
	printk( KERN_INFO "regs->testreg = %04x\r\n", regs->testreg );
}

static int radio_tea5766_write( radio_tea5766_fm_regs_t *regs )
{
	int res;

	if ( fm_i2c_client_handle == NULL )
		return -ENODEV;

	//printk( KERN_INFO "radio_tea5766_write\r\n" );

	down( &radio_tea5766_fm_i2c_lock );
	res = fm_i2c_client_handle->driver->command(fm_i2c_client_handle, TEA576X_WRITE, regs);
	up( &radio_tea5766_fm_i2c_lock );
	return res;
}

static int radio_tea5766_read( radio_tea5766_fm_regs_t *regs )
{
	int res;

	if ( fm_i2c_client_handle == NULL )
		return -ENODEV;

	//printk( KERN_INFO "radio_tea5766_read\r\n" );

	down( &radio_tea5766_fm_i2c_lock );
	res = fm_i2c_client_handle->driver->command( fm_i2c_client_handle, TEA576X_READ, regs );
	up( &radio_tea5766_fm_i2c_lock );
	return res;
}


static void print_rds_regs( radio_tea5766_rds_regs_t *regs )
{
	printk( KERN_INFO "regs->rdsr1  = %04x\r\n", regs->rdsr1 );
	printk( KERN_INFO "regs->rdsr2  = %04x\r\n", regs->rdsr2 );
	printk( KERN_INFO "regs->rdsr3  = %04x\r\n", regs->rdsr3 );
	printk( KERN_INFO "regs->rdsr4  = %04x\r\n", regs->rdsr4 );
	printk( KERN_INFO "regs->rdsw1  = %04x\r\n", regs->rdsw1 );
	printk( KERN_INFO "regs->rdsw2  = %04x\r\n", regs->rdsw2 );
// 	printk( KERN_INFO "regs->manid  = %04x\r\n", regs->manid );
// 	printk( KERN_INFO "regs->chipid  = %04x\r\n", regs->chipid );
}

static int radio_tea5766_rds_write( radio_tea5766_rds_regs_t *regs )
{
	int res;

	if ( rds_i2c_client_handle == NULL )
		return -ENODEV;

// 	printk( KERN_INFO "radio_tea5766_rds_write\r\n" );

	down( &radio_tea5766_fm_i2c_lock );	// Use the same lock as the FM part of the chip
	res = rds_i2c_client_handle->driver->command( rds_i2c_client_handle, TEA576X_WRITE, regs );
	up( &radio_tea5766_fm_i2c_lock );
	return res;
}

static int radio_tea5766_rds_read( radio_tea5766_rds_regs_t *regs )
{
	int res;

	if ( rds_i2c_client_handle == NULL )
		return -ENODEV;

// 	printk( KERN_INFO "radio_tea5766_rds_read\r\n" );

	down( &radio_tea5766_fm_i2c_lock );	// Use the same lock as the FM part of the chip
	res = rds_i2c_client_handle->driver->command( rds_i2c_client_handle, TEA576X_READ, regs );
	up( &radio_tea5766_fm_i2c_lock );
	return res;
}


//////////////////////////////////////////////////////////////////////////////
// FM: FRQSET settings
//////////////////////////////////////////////////////////////////////////////
static void set_search_dir( radio_tea5766_fm_regs_t *regs, int dir )
{
	if ( dir == TEA_5766_SUD_SEARCH_UP ) {
		regs->frqset |= TEA_5766_SUD;	// Up
	}
	else {
		regs->frqset &= ~TEA_5766_SUD;	// Down
	}
}

static void set_search_mode( radio_tea5766_fm_regs_t *regs, int mode )
{

	if ( mode == TEA_5766_SM_SEARCH_MODE ) {
		regs->frqset |= TEA_5766_FR_MASK;		// Search mode
	}
	else {
		regs->frqset &= ~TEA_5766_FR_MASK;	// Preset mode
	}
}

static void put_frequencyOnGrid( unsigned long *kHz, int region )
{
printk( KERN_INFO "%s : put %lu on grid: ", __FUNCTION__, *kHz );
	unsigned long freq;

	freq = (unsigned long)(*kHz + band_limit[region].grid/2 )/ band_limit[region].grid;
	*kHz = freq * band_limit[region].grid;

printk( KERN_INFO "%lu\r\n", *kHz );
}

static void set_frequency( radio_tea5766_fm_regs_t *regs, unsigned long kHz)
{
	unsigned long frqset_new = 0;
	int region;

	region = get_region( regs );

	// Check limits & grid
	if ( kHz > band_limit[region].high ) {
		printk( KERN_INFO "%s : Frequency too high (%lu > %d)\r\n", __FUNCTION__, kHz, band_limit[region].high );
		kHz = band_limit[region].high;	// Clip to the max frequency
	}
	else if ( kHz < band_limit[region].low ) {
		printk( KERN_INFO "%s : Frequency too low (%lu < %d)\r\n", __FUNCTION__, kHz, band_limit[region].low );
		kHz = band_limit[region].low;	// Clip to the min frequency
	}
	else {
		put_frequencyOnGrid( &kHz, region );
	}

	// Calculate PLL register
	if (regs->tnctrl & TEA_5766_HLSI) {
		// HLSI: high
		// frqset_new = ( (kHz * 1000 + FIF) << 2 ) >> XTAL_RSHIFT;
		frqset_new = (kHz * 1000 + FIF) >> (XTAL_RSHIFT - 2);
	}
	else {
		// HLSI: low
		frqset_new = (kHz * 1000 - FIF) >> (XTAL_RSHIFT - 2);
	}
	regs->frqset &= ~TEA_5766_FR_MASK;
	regs->frqset |= (u_int16_t)frqset_new & TEA_5766_FR_MASK;

	printk( KERN_INFO "%s : %lu kHz -> FRQSET=%x (frqset_new=%lu)\r\n", __FUNCTION__, kHz, regs->frqset, frqset_new );
}


/*

static int get_tune_status(void) {
	unsigned int result;
	unsigned int ifflag,frrflag,blflag;
	// let 450 ms max to search => mainly for auto scan mode with no RF power
	unsigned int counter = 45; 

	frrflag = 0;
	while ( !frrflag && counter) {
		// read result
		radio_tea5766_read();
	
		// intreg is a read and clear reg: read once only
		result = regs.fm.intreg;
	
		ifflag = (result&IFFLAG_MASK)>>IFFLAG_BIT;
		frrflag = (result&FRRFLAG_MASK)>>FRRFLAG_BIT;
		blflag = (result&BLFLAG_MASK)>>BLFLAG_BIT;
		
		// let frrflag few chances to be set
		if ( !frrflag ) {
			msleep(10);
			counter --;
		}

	}

	printk(KERN_INFO "intreg after %d x 10ms: 0x%x\n",(45-counter),result);


	return (frrflag + (blflag<<1) + (ifflag<<2));
}

static void set_pll(unsigned int kHz, int hlsi) {
	unsigned int pll;

	pll = CalculatePLL(kHz,hlsi);
	regs.fm.frqset &= ~(FRQ_SET_MASK);
	regs.fm.frqset |= pll;

	if (hlsi)
		regs.fm.tnctrl |= HLSI_HIGH;
	else
		regs.fm.tnctrl &= (~HLSI_HIGH);


	radio_tea5766_write();
	// wait for tune
	msleep(17);

}

*/


static unsigned long get_frequency_found( radio_tea5766_fm_regs_t *regs )
{
	unsigned long kHz;

	kHz = regs->frqchk & TEA_5766_PLL;
	kHz <<= (XTAL_RSHIFT - 2);

	if(regs->tnctrl & TEA_5766_HLSI)  {//TEA_5766_HLSI_HIGH_SIDE
		// HLSI: high
		kHz -= FIF;
	}
	else {
		// HLSI: low
		kHz += FIF;
	}
	
	kHz /= 1000;	// Hz -> kHz

	return kHz;
}

static unsigned int get_level( radio_tea5766_fm_regs_t *regs )
{
	return ((regs->tunchk & TEA_5766_LEV_MASK) >> TEA_5766_LEV_OFFSET);
}

static unsigned int get_ifcount( radio_tea5766_fm_regs_t *regs )
{
	return ((regs->tunchk & TEA_5766_IF_COUNT_MASK) >> TEA_5766_IF_COUNT_OFFSET);
}


static int get_next_freq( radio_tea5766_fm_regs_t *regs, int freq, int dir ,int step)
{
	unsigned long kHz;
	unsigned long min,max;
	
	if(fm_region == TEA_5766_BLIM_US_EU){
		min = FM_BAND_MIN_WESTERN;
		max = FM_BAND_MAX_WESTERN;
	} else {
		min = FM_BAND_MIN_JAPAN;
		max = FM_BAND_MAX_JAPAN;
	}
	if(freq < min)
		freq = min;
	if(freq > max)
		freq = max;
	if(dir == SEARCH_UP){//up
		if((freq + step) > max){
			kHz = min;
		} else {
			kHz = (freq + step);
		}
	} else { //down
		if((freq - step) < min){
			kHz = max;
		} else {
			kHz = (freq - step);
		}
	}
	return kHz;
}

static int step_freq( radio_tea5766_fm_regs_t *regs, int dir )
{
	unsigned long kHz;
	int region;
	int freq_step;

	kHz = get_frequency_found( regs );
	region = get_region( regs );
	freq_step = band_limit[region].grid;
	
	if( dir == FREQ_STEP_UP ) {
		kHz += freq_step;
	}
	else {
		kHz -= freq_step;
	}

	if ( kHz > band_limit[region].high ) {
		printk( KERN_INFO "%s : Frequency too high (%lu > %d)\r\n", __FUNCTION__, kHz, band_limit[region].high );
		kHz = band_limit[region].low;	// Jump back to the lowest frequency
	}
	else if ( kHz < band_limit[region].low ) {
		printk( KERN_INFO "%s : Frequency too low (%lu < %d)\r\n", __FUNCTION__, kHz, band_limit[region].low );
		kHz = band_limit[region].high;	// Jump back to the highest frequency
	}
	else {
		put_frequencyOnGrid( &kHz, region );
	}
	if( dir == FREQ_STEP_UP ) {
		set_hlsi( regs, TEA_5766_HLSI_LOW_SIDE );
	}
	else {
		set_hlsi( regs, TEA_5766_HLSI_HIGH_SIDE );
	}
	printk(KERN_INFO "step_up : set_frequency = %lu\n", kHz);
	set_frequency( regs, kHz );
	return kHz;
}


static void get_fm_radio_status( radio_tea5766_fm_regs_t *regs, radio_fm_status_t *pt_status )
{

	printk( KERN_INFO "%s\n", __FUNCTION__ );
	
	pt_status->freq_kHz	= get_frequency_found(regs);
	pt_status->stereo	= (regs->tunchk & TEA_5766_MONO_STEREO) >> 2;
	pt_status->rf_level	= get_level(regs);
	pt_status->if_count	= get_ifcount(regs);
	pt_status->tune_timeout	= (regs->tunchk & TEA_5766_TUNTO)>>8;

	return;
}

static void get_rds_radio_status( radio_tea5766_fm_regs_t *regs, _trdsbasic_t *rds_basic, radio_rds_status_t *pt_status )
{	
	pt_status->freq_kHz	= get_frequency_found( regs );
	pt_status->pi		= rds_basic->pi;
	memcpy(pt_status->prgm_service_name, rds_basic->psn,9);
	//use full psn name as a flag instead of real sync signal
	pt_status->rds_sync = rds_basic->psn_ok;

	return;
}

static int rds_task( void *data )
{
	unsigned int result = 0;
	unsigned int rds_data_flag = 0;
	unsigned int blckid = 0;
	radio_tea5766_rds_regs_t rds_regs_mon;
	radio_tea5766_fm_regs_t fm_regs_mon;
	int cpt_sync_raz = 0;
	int timeout_value = IDLE_POLL_DELAY;
	_trdsbasic_t rds_basic;
	int first_rds_sync = 0;
	int ret = 0;

	current->flags &= ~PF_NOFREEZE;

// 	//force DAVC mode !!
	if ( (ret = radio_tea5766_rds_read(&rds_regs_mon)) != 0 )
		return ret;
	rds_regs_mon.rdsw1 |= 0x2<<10;//DAC = DAVC
	if ( (ret = radio_tea5766_rds_write(&rds_regs_mon)) != 0 )
		return ret;
	
	printk(KERN_INFO "rds_task\n");
	while ( !kthread_should_stop() ) {
 
		
		rds_data_flag = 0;
		set_current_state( TASK_INTERRUPTIBLE );
		schedule_timeout( timeout_value );

// 		swsusp cooperativity
		try_to_freeze();
		
		if(down_trylock(&radio_tea5766_ioctl_lock) == 0){
			if(!first_rds_sync){
				//wait for first rds sync
				timeout_value = IDLE_POLL_DELAY;
				if (radio_tea5766_rds_read(&rds_regs_mon) !=0 )
					goto up_lock;
				if(rds_regs_mon.rdsr1 & 0x4){
					first_rds_sync = 1;
				}
			} else {
				if (radio_tea5766_read(&fm_regs_mon) != 0)
					goto up_lock;
				result = fm_regs_mon.intreg;
				rds_data_flag = (result & TEA_5766_DAVFLG);
				if (rds_data_flag == TEA_5766_DAVFLG){
					// rds monitor
					RDS_get_basic_data(&rds_basic);
					if(rds_basic.psn_ok){
						printk( KERN_INFO "radio_task:rds psn ok\n" );
							// if rds info for pgarm service name is already ok, just sleep longer
						timeout_value = IDLE_POLL_DELAY;
							// check if rds sync signal info is always there
						if (radio_tea5766_rds_read(&rds_regs_mon) != 0)
							goto up_lock;
						if(rds_regs_mon.rdsr1 & 0x4){
							rds_sync |= 1;
						}
						cpt_sync_raz++;
						if(cpt_sync_raz == 10){
							if(rds_sync == 0){
									//rds lost
								printk( KERN_INFO "radio_task: rds lost -> clear data\n" );
								RDS_ClearData();
							}
							cpt_sync_raz = 0;
							rds_sync = 0;
						}
					}else{
							//need to find program service name
						timeout_value = RDS_POLL_DELAY;
						result = fm_regs_mon.intreg;
						rds_data_flag = (result & TEA_5766_DAVFLG);
						if (rds_data_flag == TEA_5766_DAVFLG){
							if (radio_tea5766_rds_read(&rds_regs_mon) != 0)
								goto up_lock;
							if(tea5766_dbg_print){
								print_fm_regs(&fm_regs_mon);
								print_rds_regs(&rds_regs_mon);	
								printk(KERN_INFO "rds: id block %d\n",blckid);
							}
							blckid = (rds_regs_mon.rdsr1 & (7<<12) )>>12;
							Process_RDSdata(&rds_regs_mon);
							rds_data_flag = 0;
						}
					}
				}
			}
up_lock:
			up(&radio_tea5766_ioctl_lock);
		}
	}
	return 0;
}
static void rds_start(void)
{
//#warning "quick fix to prevent unit freeze while hardware test : don't start RDS"
//return;

	if ( rds_task_handle == NULL )
		rds_task_handle = kthread_run( rds_task, NULL, "radio_rdstask" );

}

static void rds_stop(void)
{
//#warning "quick fix to prevent unit freeze while hardware test : don't start RDS"
//return;
	if ( rds_task_handle != NULL ) {
		kthread_stop( rds_task_handle );
		rds_task_handle = NULL;
	}
	RDS_ClearData();
}

static int fine_tune_check(unsigned long i_freq, int i_lev)
{
	int ret = 0;
	radio_tea5766_fm_regs_t fm_regs;
	unsigned long min,max;
	const unsigned long fine_tune_value = 32;
	int level = 0;
	int ifcount = 0;
	unsigned long frqset_new = i_freq;
	int filter_cpt = 0;
	
	if(fm_region == TEA_5766_BLIM_US_EU){
		min = FM_BAND_MIN_WESTERN;
		max = FM_BAND_MAX_WESTERN;
	} else {
		min = FM_BAND_MIN_JAPAN;
		max = FM_BAND_MAX_JAPAN;
	}
		
	if((i_freq + fine_tune_value) < max){
		if ( ( ret = radio_tea5766_read(&fm_regs) ) != 0 )
			return ret;
		set_ahlsi( &fm_regs, TEA_5766_AHSLI_CONTINU );
		set_search_mode( &fm_regs, TEA_5766_SM_PRESET_MODE );
		set_hlsi( &fm_regs, TEA_5766_HLSI_LOW_SIDE );
		set_IFcount( &fm_regs, TEA_5766_IFCT_15_625 );
		// Calculate PLL register
		if (fm_regs.tnctrl & TEA_5766_HLSI) {
		// HLSI: high
			frqset_new = ((i_freq + fine_tune_value) * 1000 + FIF) >> (XTAL_RSHIFT - 2);
		}
		else {
		// HLSI: low
			frqset_new = ((i_freq + fine_tune_value) * 1000 - FIF) >> (XTAL_RSHIFT - 2);
		}
		fm_regs.frqset &= ~TEA_5766_FR_MASK;
		fm_regs.frqset |= (u_int16_t)frqset_new & TEA_5766_FR_MASK;
		if ( ( ret = radio_tea5766_write( &fm_regs ) ) != 0 )
			return ret;
		for(filter_cpt = 0; filter_cpt < TUNE_FILTER_SIZE ;filter_cpt ++ ){
			msleep(16);

			if ( ( ret = radio_tea5766_read(&fm_regs) ) != 0 )
				return ret;
			level += get_level(&fm_regs);
			ifcount += get_ifcount(&fm_regs);
		}
		level = level / TUNE_FILTER_SIZE;
		ifcount = ifcount / TUNE_FILTER_SIZE;
		printk(KERN_INFO "fine_tune_check %lu Hz : lev = %d, ifcnt=%d\n", (i_freq + fine_tune_value),level,ifcount);

		if((level > i_lev) || (ifcount > GOOD_IF_COUNT_LOW_LIMIT) ){
			ret = -1;
		} else{
			if((i_freq - fine_tune_value) > min){
				ifcount = 0;
				level = 0;
				if ( ( ret = radio_tea5766_read(&fm_regs) )!= 0 )
					return ret;
				set_ahlsi( &fm_regs, TEA_5766_AHSLI_CONTINU );
				set_search_mode( &fm_regs, TEA_5766_SM_PRESET_MODE );
				set_hlsi( &fm_regs, TEA_5766_HLSI_LOW_SIDE );
				set_IFcount( &fm_regs, TEA_5766_IFCT_15_625 );
						// Calculate PLL register
				if (fm_regs.tnctrl & TEA_5766_HLSI) {
		// HLSI: high
					frqset_new = ((i_freq - fine_tune_value) * 1000 + FIF) >> (XTAL_RSHIFT - 2);
				}
				else {
		// HLSI: low
					frqset_new = ((i_freq - fine_tune_value) * 1000 - FIF) >> (XTAL_RSHIFT - 2);
				}
				fm_regs.frqset &= ~TEA_5766_FR_MASK;
				fm_regs.frqset |= (u_int16_t)frqset_new & TEA_5766_FR_MASK;
				if ( ( ret = radio_tea5766_write( &fm_regs ) ) != 0 )
					return ret;
				for(filter_cpt = 0; filter_cpt < TUNE_FILTER_SIZE ;filter_cpt ++ ){
					msleep(16);
					if ( ( ret = radio_tea5766_read(&fm_regs) ) != 0 )
						return ret;
					level += get_level(&fm_regs);
					ifcount += get_ifcount(&fm_regs);
				}
				level = level / TUNE_FILTER_SIZE;
				ifcount = ifcount / TUNE_FILTER_SIZE;
				printk(KERN_INFO "fine_tune_check %lu Hz : lev = %d, ifcnt=%d\n", (i_freq - fine_tune_value),level,ifcount);
				if((level > i_lev) || (ifcount < GOOD_IF_COUNT_HIGH_LIMIT) ){
					ret = -1;
				}
			}
		}
	}
	return (ret);	
}

#define AUTOSEARCH_HWMUTE
static int autoscan_task( void *data )
{
	radio_tea5766_fm_regs_t fm_regs;
 
	int timeout_value = SCAN_POLL_DELAY;
	int need_tune = 1;
	autosearch_t * p_search_info = (autosearch_t*)data;
	unsigned long autosearch_freq = p_search_info->startfreq;

	unsigned long level = 0;
	unsigned long if_count = 0;
	int filter_cpt = 0;

	current->flags &= ~PF_NOFREEZE;
	
	rds_stop();
	p_search_info->station_found = 0;
	printk(KERN_INFO "autoscan_task\n");
#ifdef AUTOSEARCH_HWMUTE
	radio_tea5766_read(&fm_regs);
	set_hwmute( &fm_regs, TEA_5766_MU_ON );
	radio_tea5766_write( &fm_regs );
#endif
	while (!kthread_should_stop()) {
 
		
		if(down_trylock(&radio_tea5766_ioctl_lock) == 0){
			if(need_tune){
				radio_tea5766_read(&fm_regs);
				//set new freq in preset mode
				radio_tea5766_fm_set_frequency(&fm_regs, autosearch_freq);
#ifdef TRUE_AUTOSEARCH
				// launch search mode
				if(p_search_info->dir == SEARCH_UP){
					set_search_dir( &fm_regs, TEA_5766_SUD_SEARCH_UP );
					printk( KERN_INFO "autoscan_task UP\n");
					set_hlsi( &fm_regs, TEA_5766_HLSI_LOW_SIDE );
				} else {
					set_search_dir( &fm_regs, TEA_5766_SUD_SEARCH_DOWN );
					printk( KERN_INFO "autoscan_task DOWN\n");
					set_hlsi( &fm_regs, TEA_5766_HLSI_HIGH_SIDE );
				}
				// Direction is set, now set generic parameters
				set_ahlsi( &fm_regs, TEA_5766_AHSLI_CONTINU );
				set_search_mode( &fm_regs, TEA_5766_SM_SEARCH_MODE);
				set_IFcount( &fm_regs, TEA_5766_IFCT_15_625 );
				radio_tea5766_write( &fm_regs );
#endif
				need_tune = 0;
			}
			up(&radio_tea5766_ioctl_lock);
		}
		
		set_current_state( TASK_INTERRUPTIBLE );
		schedule_timeout( timeout_value );

// 		swsusp cooperativity
		try_to_freeze();
		
		if(down_trylock(&radio_tea5766_ioctl_lock) == 0){
			if(p_search_info->station_found == 0){
				radio_tea5766_read(&fm_regs);
				timeout_value = SCAN_POLL_DELAY;
				if(tea5766_dbg_print)
					print_fm_regs(&fm_regs);
					//FRRFLAG only  ?
#ifdef TRUE_AUTOSEARCH
				if((fm_regs.intreg & (TEA_5766_FRRFLAG|TEA_5766_BLFLAG|TEA_5766_IFFLAG) ) == TEA_5766_FRRFLAG){
				} else {
					if((fm_regs.intreg & TEA_5766_BLFLAG) == TEA_5766_BLFLAG) {//BLFLAG ?
						p_search_info->station_found = 0;

					} else if((fm_regs.intreg & TEA_5766_IFFLAG) == TEA_5766_IFFLAG) {//IFFLAG?
						p_search_info->station_found = 0;
						printk( KERN_INFO "autoscan_task : count is not correct,freq=%lu\n", autosearch_freq);

					} else {
						printk( KERN_INFO "autoscan_task : running,freq=%lu\n", autosearch_freq);
					}
					autosearch_freq = get_next_freq(&fm_regs,autosearch_freq,p_search_info->dir,100);
					need_tune = 1;
				}
#else
				if(fm_regs.tunchk & TEA_5766_LD){//PLL locked
// 					printk( KERN_INFO "autoscan_task : running,freq=%lu\n", autosearch_freq);
					level += get_level(&fm_regs);
					if_count += get_ifcount(&fm_regs);
					filter_cpt++;
					if(filter_cpt == TUNE_FILTER_SIZE){
						level = level / TUNE_FILTER_SIZE;
						if_count = if_count / TUNE_FILTER_SIZE;
						if((level >= autosearch_sensitivity_level) && 
						(if_count >= (GOOD_IF_COUNT - autosearch_sensitivity_ifcount)) &&
						(if_count <= (GOOD_IF_COUNT + autosearch_sensitivity_ifcount)) ){
							printk( KERN_INFO "autoscan_task : tuner state machine ready (lev=%lu ifcnt=%lu)\n",
									level,if_count);
							if(fine_tune_check(autosearch_freq,level) == 0){
								p_search_info->station_found = 1;
								timeout_value = IDLE_POLL_DELAY;
#ifdef AUTOSEARCH_HWMUTE
								set_hwmute( &fm_regs, TEA_5766_MU_OFF );
#endif
								radio_tea5766_fm_set_frequency(&fm_regs, autosearch_freq);
							} else {
								p_search_info->station_found = 0;
								need_tune = 1;
							}
						} else {
							printk( KERN_INFO "autoscan_task : no station lev=%lu ifcnt=%lu\n",
									level,if_count);
							p_search_info->station_found = 0;
							need_tune = 1;
						}
						filter_cpt = 0;
						level = 0;
						if_count = 0;
					}
					if(need_tune){
						autosearch_freq = get_next_freq(&fm_regs,autosearch_freq,p_search_info->dir,100);
					}
				}
	// 				// we made a complete loop -> exit
	// 				if(autosearch_info.startfreq == autosearch_freq){
	// 				
	// 				}
				}
#endif
			}
		up(&radio_tea5766_ioctl_lock);
	}
	printk( KERN_INFO "autoscan_task : quit\n");
	return 0;
}

static void autoscan_start(autosearch_t * p_autosearch)
{
	rds_stop();
	if ( autoscan_task_handle == NULL )
		autoscan_task_handle = kthread_run( autoscan_task, p_autosearch, "radio_autoscantask" );

}

static void autoscan_stop(autosearch_t * p_autosearch)
{
#ifdef AUTOSEARCH_HWMUTE
	radio_tea5766_fm_regs_t fm_regs;
#endif
	if ( autoscan_task_handle != NULL ) {
		kthread_stop( autoscan_task_handle );
		autoscan_task_handle = NULL;
#ifdef AUTOSEARCH_HWMUTE
		radio_tea5766_read(&fm_regs);
		set_hwmute( &fm_regs, TEA_5766_MU_OFF );
		radio_tea5766_write( &fm_regs );
#endif
	}
	if(p_autosearch->station_found){
		rds_start();
	}
	p_autosearch->station_found = 0;
}

//////////////////////////////////////////////////////////////////////////////
// FM: TNCTRL settings
//////////////////////////////////////////////////////////////////////////////

static void set_ahlsi( radio_tea5766_fm_regs_t *regs, int arg )
{
	if ( arg == TEA_5766_AHSLI_STOP ) {
		regs->tnctrl |= TEA_5766_AHLSI;	// Stop on correct IF
	}
	else {
		regs->tnctrl &= ~TEA_5766_AHLSI;	// Search continuously
	}
}

static void set_hlsi( radio_tea5766_fm_regs_t *regs, int arg )
{
	if ( arg == TEA_5766_HLSI_HIGH_SIDE ) {
		regs->tnctrl |= TEA_5766_HLSI;	// High-side injection
	}
	else {
		regs->tnctrl &= ~TEA_5766_HLSI;	// Low-side injection
	}
}

static void toggle_hlsi( radio_tea5766_fm_regs_t *regs )
{
	regs->tnctrl ^= TEA_5766_HLSI;
}

static void set_mono( radio_tea5766_fm_regs_t *regs, int arg )
{
	if ( arg == TEA_5766_MST_MONO ) {
		regs->tnctrl |= TEA_5766_MST;	// Mono
	}
	else {
		regs->tnctrl &= ~TEA_5766_MST;	// Stereo
	}
}

static void set_ssl( radio_tea5766_fm_regs_t *regs, int mode )
{
	int mask;
	switch( mode )
	{
		case TEA_5766_SSL_3:
		case TEA_5766_SSL_5:
		case TEA_5766_SSL_7:
		case TEA_5766_SSL_10:
			mask = mode << TEA_5766_SSL_OFFSET;
		break;	
		default:
			return;
		break;
	}
	regs->tnctrl &= ~TEA_5766_SSL_MASK;	// Zero out the bits
	regs->tnctrl |= mask;
}

static void set_hwmute( radio_tea5766_fm_regs_t *regs, int arg )
{
	if ( arg == TEA_5766_MU_ON) {
		regs->tnctrl |= TEA_5766_MU;		// Mute on
	}
	else {
		regs->tnctrl &= ~TEA_5766_MU;	// Mute off
	}
}

static void set_swmute( radio_tea5766_fm_regs_t *regs, int arg )
{
	if ( arg == TEA_5766_SMUTE_ON) {
		regs->tnctrl |= TEA_5766_SMUTE;	// Mute on
	}
	else {
		regs->tnctrl &= ~TEA_5766_SMUTE;	// Mute off
	}
}

static void set_afmute( radio_tea5766_fm_regs_t *regs, int arg )
{
	if ( arg == TEA_5766_AFM_MUTED) {
		regs->tnctrl |= TEA_5766_AFM;	// Mute on
	}
	else {
		regs->tnctrl &= ~TEA_5766_AFM;	// Mute off
	}
}

static void set_IFcount( radio_tea5766_fm_regs_t *regs, int arg )
{
	if ( arg == TEA_5766_IFCT_15_625 ) {
		regs->tnctrl |= TEA_5766_IFCT;
	}
	else {
		regs->tnctrl &= ~TEA_5766_IFCT;
	}
}

static void set_swport( radio_tea5766_fm_regs_t *regs, int arg )
{
	if ( arg == TEA_5766_SWP_HIGH ) {
		regs->tnctrl |= TEA_5766_SWP;	// SWPORT = High
	}
	else {
		regs->tnctrl &= ~TEA_5766_SWP;	// SWPORT = Low
	}
}

static void set_region( radio_tea5766_fm_regs_t *regs, int region )
{
	if ( region == TEA_5766_BLIM_JAPAN ) {
		regs->tnctrl |= TEA_5766_BLIM;	// Japan band
		fm_region = TEA_5766_BLIM_JAPAN;
	}
	else {
		regs->tnctrl &= ~TEA_5766_BLIM;	// Western band
		fm_region = TEA_5766_BLIM_US_EU;
	}
}

static int get_region( radio_tea5766_fm_regs_t *regs )
{
	if  ((regs->tnctrl & TEA_5766_BLIM) == TEA_5766_BLIM) {
		return TEA_5766_BLIM_JAPAN;
	}
	else {
		return TEA_5766_BLIM_US_EU;
	}
}

static void set_power( radio_tea5766_fm_regs_t *regs, int action )
{
	int mask;
	switch( action )
	{
		case TEA_5766_PUPD_FM_OFF_RDS_OFF:
			mask = TEA_5766_PUPD_FM_OFF_RDS_OFF << TEA_5766_PUPD_OFFSET;
		break;
		case TEA_5766_PUPD_FM_ON_RDS_OFF:
			mask = TEA_5766_PUPD_FM_ON_RDS_OFF << TEA_5766_PUPD_OFFSET;
		break;
		case TEA_5766_PUPD_FM_ON_RDS_ON:
			mask = TEA_5766_PUPD_FM_ON_RDS_ON << TEA_5766_PUPD_OFFSET;
		break;		
		default:
			return;
		break;
	}
	regs->tnctrl &= ~TEA_5766_PUPD_MASK;	// Zero out the bits
	regs->tnctrl |= mask;
}

static void set_stereo_noise_cancellation( radio_tea5766_fm_regs_t *regs, int level )
{
	if ( level == TEA_5766_SNC_ON ) {
		regs->tnctrl |= TEA_5766_SNC;	
	}
	else {
		regs->tnctrl &= ~TEA_5766_SNC;	
	}
}
static void set_stereo_sensitivity( radio_tea5766_fm_regs_t *regs, int level )
{
	if ( level == TEA_5766_SNCLEV_LOW ) {
		regs->testreg |= TEA_5766_SNCLEV;
	}
	else {
		regs->testreg &= ~TEA_5766_SNCLEV;
	}
}

//////////////////////////////////////////////////////////////////////////////
// FM: misc
//////////////////////////////////////////////////////////////////////////////

static void fm_tuner_init( radio_tea5766_fm_regs_t *regs )
{
	// Default values for these registers
	regs->intreg = INTREG_INIT;
// 	regs->frqset = FRQSET_INIT;
// 	regs->testreg = TESTREG_INIT;

	// Power up, HW mute on
	set_power( regs, TEA_5766_PUPD_FM_ON_RDS_ON );	// Power on
	set_hwmute( regs, TEA_5766_MU_ON );		// HW mute on
	set_swmute( regs, TEA_5766_SMUTE_ON );

	// Manual mode, set default frequency
	autosearch_sensitivity_level = LOW_SENSITIVITY_LEV;
	autosearch_sensitivity_ifcount = LOW_SENSITIVITY_IF_COUNT;
	set_ssl( regs, TEA_5766_SSL_7 );		// ssl low sensitivity
	set_region( regs, TEA_5766_BLIM_US_EU );	// Western region
	set_ahlsi( regs, TEA_5766_AHSLI_CONTINU );
	set_search_mode( regs, TEA_5766_SM_PRESET_MODE );
	set_hlsi( regs, TEA_5766_HLSI_LOW_SIDE );
	set_IFcount( regs, TEA_5766_IFCT_15_625 );
	set_frequency( regs, DEFAULT_FREQ );
	set_stereo_noise_cancellation(regs,TEA_5766_SNC_ON);
	set_stereo_sensitivity(regs,TEA_5766_SNCLEV_LOW);
}

static int radio_tea5766_fm_set_frequency(radio_tea5766_fm_regs_t *regs, int freq)
{
	int ret = 0;
	
	set_ahlsi( regs, TEA_5766_AHSLI_CONTINU );
	set_search_mode( regs, TEA_5766_SM_PRESET_MODE );
	set_hlsi( regs, TEA_5766_HLSI_LOW_SIDE );
	set_IFcount( regs, TEA_5766_IFCT_15_625 );
	if(freq >= FM_BAND_MIN_JAPAN && freq <= FM_BAND_MAX_WESTERN){
		set_frequency( regs, freq );	// set frequency in kHz
	} else {
		ret =   -1;
	}
	// Write registers
	radio_tea5766_write( regs );
	msleep(20);

	return (ret);
}


// static int radio_tea5766_fm_autosearch(radio_tea5766_fm_regs_t *regs,const int dir,const int startfreq)
// {
// 	int ret = 0;
// 	int quit = 0;
// 	int timeout = 0;
// // 	int need_tune = 1;
// 	int ifflag_cpt = 0;
// 	int freq = startfreq;
// 	int lev = 0;
// 	
// 	do{
// 		if(need_tune){
// 			if(dir == SEARCH_UP){
// 				set_search_dir( regs, TEA_5766_SUD_SEARCH_UP );
// 				printk( KERN_INFO "radio_tea5766_fm_autosearch UP\n");
// 				set_hlsi( regs, TEA_5766_HLSI_LOW_SIDE );
// 			} else if (dir == SEARCH_DOWN ){
// 				set_search_dir( regs, TEA_5766_SUD_SEARCH_DOWN );
// 				printk( KERN_INFO "radio_tea5766_fm_autosearch DOWN\n");
// 				set_hlsi( regs, TEA_5766_HLSI_HIGH_SIDE );
// 			} else{
// 				ret = -1;
// 			}
			// Direction is set, now set generic parameters
// 			set_ahlsi( regs, TEA_5766_AHSLI_CONTINU );
// 			set_search_mode( regs, TEA_5766_SM_SEARCH_MODE );
// 			set_IFcount( regs, TEA_5766_IFCT_15_625 );
// 			if(freq >= FM_BAND_MIN_JAPAN && freq <= FM_BAND_MAX_WESTERN){
// 				set_frequency( regs, freq );	// set frequency to start auto search
// 			}
// 			// Write registers
// 			radio_tea5766_write( regs );
// 			need_tune = 0;
// 		}
// 		// wait for band limit reached, signal ok, read error or timeout
// 		if ( radio_tea5766_read( regs ) ) {
// 			printk( KERN_INFO "radio_tea5766_fm_autosearch: Failed to read registers\n" );
// 			quit = 1;
// 		} else {
// 			if(tea5766_dbg_print)
// 				print_fm_regs(regs);
// 			//FRRFLAG only  ?
// 			if((regs->intreg & (TEA_5766_FRRFLAG|TEA_5766_BLFLAG|TEA_5766_IFFLAG) ) == TEA_5766_FRRFLAG){
// 				lev = get_level(regs);
// 				printk( KERN_INFO "radio_tea5766_fm_autosearch: tuner state machine ready, lev=%d\n",lev );
// 				quit = 1;
// 				ret = 0;
// 			}
// 			//BLFLAG ?
// 			if((regs->intreg & TEA_5766_BLFLAG) == TEA_5766_BLFLAG) {
// 				printk( KERN_INFO "radio_tea5766_fm_autosearch: range limit reached\n" );
// 				quit = 1;
// 				ret = -1;
// 			}
// 			//IFFLAG?
// 			if((regs->intreg & TEA_5766_IFFLAG) == TEA_5766_IFFLAG) {
// 				printk( KERN_INFO "radio_tea5766_fm_autosearch: count is not correct\n" );
// 				need_tune = 1;
// 				timeout = 0;
// 				ifflag_cpt++;
// 				if(ifflag_cpt == 10){					
// 					printk( KERN_INFO "radio_tea5766_fm_set_frequency: too many consecutive ifflag\n" );
// 					quit = 1;
// 					ret = -1;
// 					ifflag_cpt = 0;
// 				}
// 			}
// 			if(!quit) {
// 				msleep(1000);
// 				timeout++;
// 				if(timeout == 3){
// 					quit = 1;
// 					ret = -1;
// 					printk( KERN_INFO "radio_tea5766_fm_set_frequency: timeout 3s\n" );
// 				}
// 			}
// 		}
// 	} while(!quit);
// 	return (ret);
// }



//////////////////////////////////////////////////////////////////////////
// Fops
//////////////////////////////////////////////////////////////////////////

static int radio_tea5766_fm_open(struct inode * inode, struct file * file)
{
	if (radio_tea5766_fm_use_count > 0)
		return -EBUSY;
	
	printk(KERN_INFO "radio_tea5766_fm_open\r\n");
		
	if ( fm_i2c_client_handle == NULL) {
		fm_i2c_client_handle = i2c_get_client( I2C_DRIVERID_TEA576X_FM, 3, fm_i2c_client_handle );
		if ( fm_i2c_client_handle == NULL ) {
			printk( KERN_ERR "radio_tea5766_fm_open: no tea5766 FM found!\r\n" );
			return -ENODEV;
		}
	}

	//TODO: check presence

	if ( rds_i2c_client_handle == NULL) {
		rds_i2c_client_handle = i2c_get_client( I2C_DRIVERID_TEA576X_RDS, 3, rds_i2c_client_handle );
		if ( rds_i2c_client_handle == NULL ) {
			printk( KERN_ERR "radio_tea5766_fm_open: no tea5766 RDS found!\r\n" );
			return -ENODEV;
		}
	}

	//TODO: check presence

	radio_tea5766_fm_use_count++;	
        return 0;
}

static int radio_tea5766_fm_release(struct inode * inode, struct file * file)
{
	printk(KERN_INFO "radio_tea5766_fm_release\r\n");

	fm_i2c_client_handle = NULL;
	rds_i2c_client_handle = NULL;

	rds_stop();
	autoscan_stop(&autosearch_info);
	rds_stop();//if "rds_start" launched in "autoscan_stop" because fm station has been found ...
	
        radio_tea5766_fm_use_count--;
	return 0;
}

static int radio_tea5766_fm_ioctl(struct inode *inode, struct file *file,
			  unsigned int cmd, unsigned long arg )
{
	int ret = 0;
	unsigned long val;
	unsigned long freq;
	radio_tea5766_fm_regs_t fm_regs;
	radio_tea5766_rds_regs_t rds_regs;
	radio_fm_status_t fm_radio_status;
	radio_rds_status_t rds_radio_status;
	_trdsbasic_t rds_basic;

	

	down(&radio_tea5766_ioctl_lock);
	if ( radio_tea5766_rds_read( &rds_regs ) ) {
		printk( KERN_ERR "radio_tea5766_fm_ioctl : Failed to read RDS registers\n" );
	}

	
	// Read registers	
	if ( radio_tea5766_read( &fm_regs ) ) {
		printk( KERN_ERR "radio_tea5766_fm_ioctl : Failed to read registers\n" );
	}
	else {
	
		switch ( cmd )
		{
	
			case SET_FREQUENCY:
				if( copy_from_user( (void *)&freq, (void *)arg, sizeof(freq) ) )
					return -EFAULT;

				rds_stop();
				autoscan_stop(&autosearch_info);
				printk(KERN_INFO "radio_tea5766_fm_ioctl : SET_FREQUENCY\n");
				radio_tea5766_fm_set_frequency(&fm_regs,freq);
				rds_start();
				break;
			case SEARCH_UP:
				rds_stop();
				autoscan_stop(&autosearch_info);
				msleep(10);
				autosearch_info.startfreq = get_next_freq( &fm_regs, get_frequency_found( &fm_regs ), SEARCH_UP, 100 );
				autosearch_info.dir = SEARCH_UP;
				autosearch_info.station_found = 0;
				printk(KERN_INFO "radio_tea5766_fm_ioctl : SEARCH_UP from freq = %lu\n",autosearch_info.startfreq);
				autoscan_start(&autosearch_info);
				break;
			case SEARCH_DOWN:
				rds_stop();
				autoscan_stop(&autosearch_info);
				msleep(10);
				autosearch_info.startfreq = get_next_freq( &fm_regs, get_frequency_found( &fm_regs ), SEARCH_DOWN, 100 );
				autosearch_info.dir = SEARCH_DOWN;
				autosearch_info.station_found = 0;
				printk(KERN_INFO "radio_tea5766_fm_ioctl : SEARCH_DOWN from freq = %lu\n",autosearch_info.startfreq);
				autoscan_start(&autosearch_info);
				break;
			case GET_CURRENT_FREQUENCY:
				freq = get_frequency_found( &fm_regs );
				if ( copy_to_user((void *)arg, &freq, sizeof(val)) )
				{
					ret = -EFAULT;
				}
				printk(KERN_INFO "radio_tea5766_fm_ioctl : GET_CURRENT_FREQUENCY = %lu\n",freq);
				break;
			case GET_LEVEL:
				printk(KERN_INFO "radio_tea5766_fm_ioctl : GET_LEVEL\n");
				val = get_level( &fm_regs );
				if ( copy_to_user((void *)arg, &val, sizeof(val)) )
				{
					ret = -EFAULT;
				}	
				break;
			case SET_SWPORT:
				printk(KERN_INFO "radio_tea5766_fm_ioctl : SET_SWPORT\n");
				set_swport( &fm_regs, arg);
				radio_tea5766_write( &fm_regs );
				break;
				
			case SET_MONO_STEREO:
				autoscan_stop(&autosearch_info);
				printk(KERN_INFO "radio_tea5766_fm_ioctl : SET_MONO_STEREO\n");
				set_mono( &fm_regs, arg );
				radio_tea5766_write( &fm_regs );
				break;
			case SET_HWMUTE:
				printk( KERN_INFO "radio_tea5766_fm_ioctl : SET_HWMUTE\n" );
				set_hwmute( &fm_regs, arg );
				radio_tea5766_write( &fm_regs );
				break;
			case SET_SWMUTE:
				printk( KERN_INFO "radio_tea5766_fm_ioctl : SET_SWMUTE\n" );
				set_swmute( &fm_regs, arg );
				radio_tea5766_write( &fm_regs );
				break;
			case SET_REGION:
				rds_stop();
				autoscan_stop(&autosearch_info);
				printk(KERN_INFO "radio_tea5766_fm_ioctl : SET_REGION\n");
				set_region( &fm_regs, arg );
				radio_tea5766_write( &fm_regs );
				break;
	
			case SET_POWERUP:
				rds_stop();
				autoscan_stop(&autosearch_info);
				printk( KERN_INFO "radio_tea5766_fm_ioctl : SET_POWERUP\n" );
				set_power( &fm_regs, TEA_5766_PUPD_FM_ON_RDS_ON );
				//TODO: RDS ON/OFF
				radio_tea5766_write( &fm_regs );
				break;
			case SET_POWERDOWN:
				rds_stop();
				autoscan_stop(&autosearch_info);
				printk( KERN_INFO "radio_tea5766_fm_ioctl : SET_POWERDOWN\n" );
				set_power( &fm_regs, TEA_5766_PUPD_FM_OFF_RDS_OFF );
				radio_tea5766_write( &fm_regs );
				break;
			case SET_INIT:
				rds_stop();
				autoscan_stop(&autosearch_info);
				printk( KERN_INFO "radio_tea5766_fm_ioctl : SET_INIT\n" );
				fm_tuner_init( &fm_regs );
				radio_tea5766_write( &fm_regs );
				break;
			case STEP_UP:
				rds_stop();
				autoscan_stop(&autosearch_info);
				printk(KERN_INFO "radio_tea5766_fm_ioctl : STEP_UP\n");
				step_freq( &fm_regs, FREQ_STEP_UP );
				radio_tea5766_write( &fm_regs );
				rds_start();
				break;
			case STEP_DOWN:
				rds_stop();
				autoscan_stop(&autosearch_info);
				printk(KERN_INFO "radio_tea5766_fm_ioctl : STEP_DOWN\n");
				step_freq( &fm_regs, FREQ_STEP_DOWN );
				radio_tea5766_write( &fm_regs );
				rds_start();
				break;
			case RDS_STOP:
				rds_stop();
				break;
			case RDS_START:
				rds_stop();
				rds_start();
				break;
	////////////////////////
	//// LOW LEVEL
	////////////////////////
			case SET_SEARCH_DIR:
				printk(KERN_INFO "radio_tea5766_fm_ioctl : SET_SEARCH_DIR\n");
				set_search_dir( &fm_regs, arg);
				radio_tea5766_write( &fm_regs );
				break;
			case SET_SEARCH_MODE:
				printk(KERN_INFO "radio_tea5766_fm_ioctl : SET_SEARCH_MODE\n");
				set_search_mode( &fm_regs, arg );
				radio_tea5766_write( &fm_regs );
				break;			
				
			case SET_AHLSI:
				printk(KERN_INFO "radio_tea5766_fm_ioctl : SET_AHLSI\n");
				set_ahlsi( &fm_regs, arg);
				radio_tea5766_write( &fm_regs );
				break;
			case SET_HLSI:
				printk(KERN_INFO "radio_tea5766_fm_ioctl : SET_HLSI\n");
				set_hlsi( &fm_regs, arg);
				radio_tea5766_write( &fm_regs );
				break;
			case TOGGLE_HLSI:
				printk(KERN_INFO "radio_tea5766_fm_ioctl : TOGGLE_HLSI\n");
				toggle_hlsi( &fm_regs );
				radio_tea5766_write( &fm_regs );
				break;
			case SET_SSL:
				printk( KERN_INFO "radio_tea5766_fm_ioctl : SET_SSL\n" );
				set_ssl( &fm_regs, (int)arg );
				radio_tea5766_write( &fm_regs );
				if((int)arg >= TEA_5766_SSL_7){
					autosearch_sensitivity_level = LOW_SENSITIVITY_LEV ;
					autosearch_sensitivity_ifcount = LOW_SENSITIVITY_IF_COUNT;
				} else {
					autosearch_sensitivity_level = HIGH_SENSITIVITY_LEV;
					autosearch_sensitivity_ifcount = HIGH_SENSITIVITY_IF_COUNT;
				}
				break;
	
			case SET_IFCOUNT:
				printk( KERN_INFO "radio_tea5766_fm_ioctl : SET_IFCOUNT\n" );
				set_IFcount( &fm_regs, arg);
				radio_tea5766_write( &fm_regs );
				break;
			case SET_AFMUTE:
				printk( KERN_INFO "radio_tea5766_fm_ioctl : SET_AFMUTE\n" );
				set_afmute( &fm_regs, arg);
				radio_tea5766_write( &fm_regs );
				break;
			case PRINT_REGS_NOW:
				print_fm_regs(&fm_regs );
				print_rds_regs(&rds_regs);
				break;
			case ENABLE_DEBUG_PRINT:
				tea5766_dbg_print = 1;
				break;
			case SET_STEREO_SENSITIVITY:
				autoscan_stop(&autosearch_info);
				set_stereo_sensitivity(&fm_regs ,arg);
				radio_tea5766_write( &fm_regs );
				break;
			case SET_SNC:
				set_stereo_noise_cancellation(&fm_regs ,arg);
				radio_tea5766_write( &fm_regs );
				break;
			case GET_FM_STATUS:	
				get_fm_radio_status(&fm_regs,&fm_radio_status);
				if ( copy_to_user((void *)arg, &fm_radio_status, sizeof(radio_fm_status_t)) ) {
					ret = -EFAULT;
				}	
				break;
			case GET_RDS_STATUS:
				memset(&rds_radio_status,0,sizeof(radio_rds_status_t));
				RDS_get_basic_data(&rds_basic);
				get_rds_radio_status(&fm_regs,&rds_basic,&rds_radio_status);
				if ( copy_to_user((void *)arg, &rds_radio_status, sizeof(radio_rds_status_t)) ) {
					ret = -EFAULT;
				}
				break;
			case STOP_SEARCH:
				autoscan_stop(&autosearch_info);
				break;
			case GET_SEARCH_STATUS:
				if ( copy_to_user((void *)arg, &(autosearch_info.station_found), sizeof(autosearch_info.station_found)) ) {
					ret = -EFAULT;
				}	
				break;
			default:
				break;
		}
	}

	up(&radio_tea5766_ioctl_lock);
	return ret;
}

static struct file_operations radio_tea5766_fm_fops = {
	.owner		= THIS_MODULE,
	.llseek         = no_llseek,
	.ioctl		= radio_tea5766_fm_ioctl,
	.open           = radio_tea5766_fm_open,
	.release        = radio_tea5766_fm_release,
};

static struct miscdevice radio_tea5766_fm_miscdev = {
        .minor	= NXP_RADIO_MINOR,
	.name	= RADIO_NAME,
	.fops	= &radio_tea5766_fm_fops,
};


/*
static int radio_tea5766_fm_chip_detect( void )
{
	//let do the detection of radio_tea5766 here
	// Check Man ID
	printk(KERN_INFO "radio_tea5766_fm_chip_detect : Check Man ID ...");
	memset( &regs, 0x00, sizeof(radio_tea5766_regs) );
	radio_tea5766_read();

	// Check Manufacturer id
	if ( (regs.id.manid & TEA576X_MAN_ID_MASK) != TEA576X_MAN_ID ) {
		printk(KERN_INFO "Bad man id found is 0x%x\n",regs.id.manid);
		return -ENODEV;
	}

	printk(KERN_INFO " found\n");

	printk(KERN_INFO "radio_tea5766_fm_chip_detect : Check chip ID ...");
	// Check chip id
	if ( regs.id.chipid != TEA5766_CHIP_ID ) {
		printk(KERN_INFO "Bad chip id found is 0x%x\n",regs.id.chipid);
		return -ENODEV;
	}
	printk(KERN_INFO " found\n");

	return 0;
}
*/

//TODO
/*
static int radio_tea5766_rds_chip_detect() {

	// let do the detection of radio_tea5766 here
	// Check Man ID
	printk(KERN_INFO "radio_tea5766_rds_chip_detect : Check rds man ID ...");
	memset( &rds_regs, 0x00, sizeof(radio_tea5766_rds_regs) );
	radio_tea5766_rds_read();

	// Check Manufacturer id
	if ( (rds_regs.id.manid & TEA576X_MAN_ID_MASK) != TEA576X_MAN_ID ) {
		printk(KERN_INFO "Bad man id found is 0x%x\n",regs.id.manid);
		return -ENODEV;
	}

	printk(KERN_INFO "found\n");

	printk(KERN_INFO "radio_tea5766_rds_chip_detect : Check rds chip ID ...");
	// Check chip id
	if ( rds_regs.id.chipid != TEA5766_CHIP_ID ) {
		printk(KERN_INFO "Bad chip id found is 0x%x\n",regs.id.chipid);
		return -ENODEV;
	}
	printk(KERN_INFO " found\n");

	return 0;

}
*/

static int __init radio_tea5766_fm_init(void)
{
	int ret;
	
	init_MUTEX( &radio_tea5766_fm_i2c_lock );
	init_MUTEX( &radio_tea5766_ioctl_lock );
	printk( "radio_tea5766_probe : misc_register radio_tea5766_fm_miscdev\r\n" );
	ret = misc_register(&radio_tea5766_fm_miscdev);
	if(ret < 0)
	{
		printk(KERN_INFO "cannot register radio_tea5766 fm\r\n");
		return -EINVAL;
	}
	RDS_init();
	printk( "ARCHOS Radio FM Driver register success\r\n" );
	return 0;
}


static void __exit radio_tea5766_fm_cleanup_module(void)
{
	RDS_exit();
	misc_deregister( &radio_tea5766_fm_miscdev );	
	printk( "%s: done\r\n", __FUNCTION__ );
}

module_init(radio_tea5766_fm_init);
module_exit(radio_tea5766_fm_cleanup_module);

/****************************************************************************
			Module Info
*****************************************************************************/

MODULE_AUTHOR("ARCHOS");
MODULE_DESCRIPTION("A driver for the ARCHOS FM Receiver.");
MODULE_LICENSE("GPL");

