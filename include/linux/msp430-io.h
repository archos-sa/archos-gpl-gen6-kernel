
#ifndef _LINUX_ATMEGA_IO_H
#define _LINUX_ATMEGA_IO_H

#define ATMEGA_IO_READ_TIME			0
#define ATMEGA_IO_READ_VERSION			1
#define ATMEGA_IO_READ_STATUS			2
#define ATMEGA_IO_READ_VBATT			3
#define ATMEGA_IO_READ_ALARM			4
#define ATMEGA_IO_READ_MODULE_TYPE		5
#define ATMEGA_IO_READ_DC_CONNECT		6		// new gen5
#define ATMEGA_DUMP				7		// ATMEGA Dump all registers
#define ATMEGA_IO_GET_CURRENT_SAT		8		
#define ATMEGA_IO_GET_NEXT_SAT			9		

#define ATMEGA_IO_SET_TIME			10
#define ATMEGA_IO_SET_ALARM			11
#define ATMEGA_IO_SET_POWER_OFF			12
#define ATMEGA_IO_RESET_ALARM			13
#define ATMEGA_IO_RESET_OMAP			14
#define ATMEGA_IO_SET_SHUTDOWN			15
#define ATMEGA_IO_SET_PLUG_POWER		16
#define ATMEGA_IO_SET_LED_CHARGE		17
#define ATMEGA_IO_SET_CHARGE_MODE		18		// new gen5
#define ATMEGA_IO_CLR_SD_FLAG			19		// new gen5

#define ATMEGA_IO_TOGGLE_IRR_IO			20
#define ATMEGA_IO_SET_LED_CHARGE_OFF		21
// #define ATMEGA_IO_SET_GREEN_BLINK		22
#define ATMEGA_IO_SET_PTH			23		// new gen5
#define ATMEGA_IO_SET_LED_CHG			24		// for AMP
#define ATMEGA_IO_SET_LED_PWR			25		// for AMP
#define ATMEGA_IO_READ_LED_CHG_STATUS		26		// for AMP
#define ATMEGA_IO_READ_LED_PWR_STATUS		27		// for AMP
#define ATMEGA_IO_SET_REBOOT			28		// dvr205
#define ATMEGA_IO_CLR_COLDSTART			29		// dvr205
#define ATMEGA_IO_READ_BOARD_ID			30		// new gen6  G06S:60vv G06H: 61vv
#define ATMEGA_IO_READ_CHECK_SUM		31	

#define ATMEGA_IO_BATTERYDOCK_REGET_BATTERY	33		// new gen6  check the real DC-in status for battery dock and set the charge level
#define ATMEGA_IO_BATTERYDOCK_READ_DC_CONNECT	34		// new gen6  check the real DC-in status for battery dock

// ATMEGA STATUS register bit mask
#define ATMEGA_IO_STATUS_DC_DETECTED_BIT	0x0001
#define ATMEGA_IO_STATUS_USB_DETECTED_BIT	0x0002
#define ATMEGA_IO_STATUS_BATTERY_OK_BIT		0x0004
#define ATMEGA_IO_STATUS_DDRAM_COLD_STARTED_BIT	0x0008
#define ATMEGA_IO_STATUS_MSP_COLD_STARTED_BIT	0x0010
#define ATMEGA_IO_STATUS_AUX_DETECTED_BIT	0x0020
#define ATMEGA_IO_STATUS_ALARM_REACHED_BIT	0x0040
#define ATMEGA_IO_STATUS_REQ_POWER_OFF_BIT	0x0080

#define ATMEGA_IO_STATUS_CHARGE_HIGH_BIT	0x0100		// new gen5
#define ATMEGA_IO_STATUS_CHARGE_ON_BIT		0x0200		// new gen5
#define ATMEGA_IO_STATUS_SD_CHANGED_BIT		0x0400		// new gen5
#define ATMEGA_IO_STATUS_KBON_PRESSED_BIT	0x0800		// new gen6
#define ATMEGA_IO_STATUS_DCJACK_DETECT_BIT	0x1000		// new gen6  for G06L, valid from ATMEGA version 0x0b

#define ATMEGA_IO_STATUS_RED_BLINK_BIT		0x0100		// dvr205
#define ATMEGA_IO_STATUS_GREEN_BLINK_BIT	0x0200		// dvr205

// must be in the same order of module_resistors!!
// see common/Include/sys_atmega.h
#define MODULE_ID_TV_CRADLE			1
#define	MODULE_ID_REMOTE_FM			2
#define MODULE_ID_VRA_GEN6			3
#define	MODULE_ID_GPS_WITHOUT_TMC		4
#define MODULE_ID_USB_HOST_CABLE		5
#define MODULE_ID_BATTERY_DOCK_GEN6		6
#define MODULE_ID_POWER_CABLE			7
#define MODULE_ID_SERIAL_ADAPTER		8
#define MODULE_ID_PC_CABLE			9
#define MODULE_ID_UNUSED10			10
#define MODULE_ID_GPS_WITH_TMC			11
#define MODULE_ID_UNUSED12			12
#define MODULE_ID_MINI_DOCK			13
#define MODULE_ID_DVBT_SNAP_ON			14
#define MODULE_ID_MUSIC_DOCK			15


int atmega_io_set_charge_mode( unsigned long arg );
int atmega_io_getUsbType( void );
int atmega_io_battery_dock_check_dcin (void);

#endif /* _LINUX_ATMEGA_IO_H */
