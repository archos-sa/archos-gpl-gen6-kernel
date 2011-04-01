/** @file wlan_wext.h
 * @brief This file contains definition for IOCTL call.
 *  
 * (c) Copyright � 2003-2007, Marvell International Ltd. 
 *
 * This software file (the "File") is distributed by Marvell International 
 * Ltd. under the terms of the GNU General Public License Version 2, June 1991 
 * (the "License").  You may use, redistribute and/or modify this File in 
 * accordance with the terms and conditions of the License, a copy of which 
 * is available along with the File in the gpl.txt file or by writing to 
 * the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 
 * 02111-1307 or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE 
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about 
 * this warranty disclaimer.
 *
 */
/********************************************************
Change log:
	10/11/05: Add Doxygen format comments
	12/19/05: Correct a typo in structure _wlan_ioctl_wmm_tspec
	01/11/06: Conditionalize new scan/join ioctls
	04/10/06: Add hostcmd generic API
	04/18/06: Remove old Subscrive Event and add new Subscribe Event
	          implementation through generic hostcmd API
	06/08/06: Add definitions of custom events
********************************************************/

#ifndef	_WLAN_WEXT_H_
#define	_WLAN_WEXT_H_

#define SUBCMD_OFFSET			4
/** PRIVATE CMD ID */
#define	WLANIOCTL			0x8BE0

#define WLANSETWPAIE			(WLANIOCTL + 0)
#define WLANCISDUMP 			(WLANIOCTL + 1)
#ifdef MFG_CMD_SUPPORT
#define	WLANMANFCMD			(WLANIOCTL + 2)
#endif
#define	WLANREGRDWR			(WLANIOCTL + 3)
#define MAX_EEPROM_DATA     			256
#define	WLANHOSTCMD			(WLANIOCTL + 4)

#define WLANHOSTSLEEPCFG		(WLANIOCTL + 5)
#define WLANARPFILTER			(WLANIOCTL + 6)

#define WLAN_SETINT_GETINT		(WLANIOCTL + 7)
#define WLANNF					1
#define WLANRSSI				2
#define WLANBGSCAN				4
#define WLANENABLE11D				5
#define WLANADHOCGRATE				6
#define WLANSDIOCLOCK				7
#define WLANWMM_ENABLE				8
#define WLANNULLGEN				10
#define WLAN_SUBCMD_SET_PRESCAN			11
#define WLANADHOCCSET				12

#define WLAN_SETNONE_GETNONE	        (WLANIOCTL + 8)
#define WLANDEAUTH                  		1
#define WLANRADIOON                 		2
#define WLANRADIOOFF                		3
#define WLANREMOVEADHOCAES          		4
#define WLANADHOCSTOP               		5
#ifdef REASSOCIATION
#define WLANREASSOCIATIONAUTO			8
#define WLANREASSOCIATIONUSER			9
#endif /* REASSOCIATION */
#define WLANWLANIDLEON				10
#define WLANWLANIDLEOFF				11

#define WLANGETLOG                  	(WLANIOCTL + 9)
#define WLAN_SETCONF_GETCONF		(WLANIOCTL + 10)

#define BG_SCAN_CONFIG				1
#define WMM_ACK_POLICY              2
#define WMM_PARA_IE                 3
#define WMM_ACK_POLICY_PRIO         4
#define CAL_DATA_EXT_CONFIG         5

#define WLANSCAN_TYPE			(WLANIOCTL + 11)

#define WLAN_SET_GET_2K         (WLANIOCTL + 13)
#define WLAN_SET_USER_SCAN              1
#define WLAN_GET_SCAN_TABLE             2
#define WLAN_SET_MRVL_TLV               3
#define WLAN_GET_ASSOC_RSP              4
#define WLAN_ADDTS_REQ                  5
#define WLAN_DELTS_REQ                  6
#define WLAN_QUEUE_CONFIG               7
#define WLAN_QUEUE_STATS                8

#define WLAN_SETNONE_GETONEINT		(WLANIOCTL + 15)
#define WLANGETREGION				1
#define WLAN_GET_LISTEN_INTERVAL		2
#define WLAN_GET_MULTIPLE_DTIM			3
#define WLAN_GET_TX_RATE			4
#define	WLANGETBCNAVG				5
#define WLANGETDATAAVG				6

#define WLAN_SETNONE_GETTWELVE_CHAR (WLANIOCTL + 19)
#define WLAN_SUBCMD_GETRXANTENNA    1
#define WLAN_SUBCMD_GETTXANTENNA    2
#define WLAN_GET_TSF                3

#define WLAN_SETWORDCHAR_GETNONE	(WLANIOCTL + 20)
#define WLANSETADHOCAES				1

#define WLAN_SETNONE_GETWORDCHAR	(WLANIOCTL + 21)
#define WLANGETADHOCAES				1
#define WLANVERSION				2

#define WLAN_SETONEINT_GETONEINT	(WLANIOCTL + 23)
#define WLAN_WMM_QOSINFO			2
#define	WLAN_LISTENINTRVL			3
#define WLAN_FW_WAKEUP_METHOD			4
#define WAKEUP_FW_UNCHANGED			0
#define WAKEUP_FW_THRU_INTERFACE		1
#define WAKEUP_FW_THRU_GPIO			2

#define WLAN_TXCONTROL				5
#define WLAN_NULLPKTINTERVAL			6
#define WLAN_ADHOC_AWAKE_PERIOD			8
#define WLAN_LDO				9

#define WLAN_SETONEINT_GETNONE		(WLANIOCTL + 24)
#define WLAN_SUBCMD_SETRXANTENNA		1
#define WLAN_SUBCMD_SETTXANTENNA		2
#define WLANSETAUTHALG				5
#define WLANSETENCRYPTIONMODE			6
#define WLANSETREGION				7
#define WLAN_SET_LISTEN_INTERVAL		8

#define WLAN_SET_MULTIPLE_DTIM			9

#define WLANSETBCNAVG				10
#define WLANSETDATAAVG				11

#define WLAN_SET64CHAR_GET64CHAR	(WLANIOCTL + 25)
#define WLANSLEEPPARAMS 			2
#define	WLAN_BCA_TIMESHARE			3
#define WLANSCAN_MODE				6

#define WLAN_GET_ADHOC_STATUS			9

#define WLAN_SET_GEN_IE                 	10
#define WLAN_GET_GEN_IE                 	11
#define WLAN_REASSOCIATE                	12
#define WLAN_WMM_QUEUE_STATUS               	14

#define WLANEXTSCAN			(WLANIOCTL + 26)
#define WLANDEEPSLEEP			(WLANIOCTL + 27)
#define DEEP_SLEEP_ENABLE			1
#define DEEP_SLEEP_DISABLE  			0

#define WLAN_SET_GET_SIXTEEN_INT       (WLANIOCTL + 29)
#define WLAN_TPCCFG                             1
#define WLAN_AUTO_FREQ_SET			3
#define WLAN_AUTO_FREQ_GET			4
#define WLAN_LED_GPIO_CTRL			5
#define WLAN_SCANPROBES 			6
#define WLAN_SLEEP_PERIOD			7
#define	WLAN_ADAPT_RATESET			8
#define	WLAN_INACTIVITY_TIMEOUT			9
#define WLANSNR					10
#define WLAN_GET_RATE				11
#define	WLAN_GET_RXINFO				12
#define	WLAN_SET_ATIM_WINDOW			13
#define WLAN_BEACON_INTERVAL			14
#define WLAN_SCAN_TIME				16
#define WLANHSCFG				20
#ifdef DEBUG_LEVEL1
#define WLAN_DRV_DBG				23
#endif

#define WLANCMD52RDWR			(WLANIOCTL + 30)
#define WLANCMD53RDWR			(WLANIOCTL + 31)
#define CMD53BUFLEN				32

#define	REG_MAC					0x19
#define	REG_BBP					0x1a
#define	REG_RF					0x1b
#define	REG_EEPROM				0x59

#define	CMD_DISABLED				0
#define	CMD_ENABLED				1
#define	CMD_GET					2
#define SKIP_CMDNUM				4
#define SKIP_TYPE				1
#define SKIP_SIZE				2
#define SKIP_ACTION				2
#define SKIP_TYPE_SIZE			(SKIP_TYPE + SKIP_SIZE)
#define SKIP_TYPE_ACTION		(SKIP_TYPE + SKIP_ACTION)

/* define custom events */
#define CUS_EVT_HWM_CFG_DONE		"HWM_CFG_DONE.indication "
#define CUS_EVT_BEACON_RSSI_LOW		"EVENT=BEACON_RSSI_LOW"
#define CUS_EVT_BEACON_SNR_LOW		"EVENT=BEACON_SNR_LOW"
#define CUS_EVT_BEACON_RSSI_HIGH	"EVENT=BEACON_RSSI_HIGH"
#define CUS_EVT_BEACON_SNR_HIGH		"EVENT=BEACON_SNR_HIGH"
#define CUS_EVT_MAX_FAIL		"EVENT=MAX_FAIL"
#define CUS_EVT_MLME_MIC_ERR_UNI	"MLME-MICHAELMICFAILURE.indication unicast "
#define CUS_EVT_MLME_MIC_ERR_MUL	"MLME-MICHAELMICFAILURE.indication multicast "

#define CUS_EVT_DEEP_SLEEP_AWAKE	"EVENT=DS_AWAKE"

#define CUS_EVT_ADHOC_LINK_SENSED	"EVENT=ADHOC_LINK_SENSED"
#define CUS_EVT_ADHOC_BCN_LOST		"EVENT=ADHOC_BCN_LOST"

/** wlan_ioctl */
typedef struct _wlan_ioctl
{
        /** Command ID */
    u16 command;
        /** data length */
    u16 len;
        /** data pointer */
    u8 *data;
} wlan_ioctl;

/** wlan_ioctl_rfantenna */
typedef struct _wlan_ioctl_rfantenna
{
    u16 Action;
    u16 AntennaMode;
} wlan_ioctl_rfantenna;

/** wlan_ioctl_regrdwr */
typedef struct _wlan_ioctl_regrdwr
{
        /** Which register to access */
    u16 WhichReg;
        /** Read or Write */
    u16 Action;
    u32 Offset;
    u16 NOB;
    u32 Value;
} wlan_ioctl_regrdwr;

/** wlan_ioctl_cfregrdwr */
typedef struct _wlan_ioctl_cfregrdwr
{
        /** Read or Write */
    u8 Action;
        /** register address */
    u16 Offset;
        /** register value */
    u16 Value;
} wlan_ioctl_cfregrdwr;

/** wlan_ioctl_rdeeprom */
typedef struct _wlan_ioctl_rdeeprom
{
    u16 WhichReg;
    u16 Action;
    u16 Offset;
    u16 NOB;
    u8 Value;
} wlan_ioctl_rdeeprom;

/** wlan_ioctl_adhoc_key_info */
typedef struct _wlan_ioctl_adhoc_key_info
{
    u16 action;
    u8 key[16];
    u8 tkiptxmickey[16];
    u8 tkiprxmickey[16];
} wlan_ioctl_adhoc_key_info;

/** sleep_params */
typedef struct _wlan_ioctl_sleep_params_config
{
    u16 Action;
    u16 Error;
    u16 Offset;
    u16 StableTime;
    u8 CalControl;
    u8 ExtSleepClk;
    u16 Reserved;
} __ATTRIB_PACK__ wlan_ioctl_sleep_params_config,
    *pwlan_ioctl_sleep_params_config;

/** BCA TIME SHARE */
typedef struct _wlan_ioctl_bca_timeshare_config
{
        /** ACT_GET/ACT_SET */
    u16 Action;
        /** Type: WLAN, BT */
    u16 TrafficType;
        /** Interval: 20msec - 60000msec */
    u32 TimeShareInterval;
        /** PTA arbiter time in msec */
    u32 BTTime;
} __ATTRIB_PACK__ wlan_ioctl_bca_timeshare_config,
    *pwlan_ioctl_bca_timeshare_config;

typedef struct _wlan_ioctl_reassociation_info
{
    u8 CurrentBSSID[6];
    u8 DesiredBSSID[6];
    char DesiredSSID[IW_ESSID_MAX_SIZE + 1];
} __ATTRIB_PACK__ wlan_ioctl_reassociation_info;

#endif /* _WLAN_WEXT_H_ */
