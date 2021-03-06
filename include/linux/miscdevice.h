#ifndef _LINUX_MISCDEVICE_H
#define _LINUX_MISCDEVICE_H
#include <linux/module.h>
#include <linux/major.h>

#define PSMOUSE_MINOR  1
#define MS_BUSMOUSE_MINOR 2
#define ATIXL_BUSMOUSE_MINOR 3
/*#define AMIGAMOUSE_MINOR 4	FIXME OBSOLETE */
#define ATARIMOUSE_MINOR 5
#define SUN_MOUSE_MINOR 6
#define APOLLO_MOUSE_MINOR 7
#define PC110PAD_MINOR 9
/*#define ADB_MOUSE_MINOR 10	FIXME OBSOLETE */
#define WATCHDOG_MINOR		130	/* Watchdog timer     */
#define TEMP_MINOR		131	/* Temperature Sensor */
#define RTC_MINOR 135
#define EFI_RTC_MINOR		136	/* EFI Time services */
#define SUN_OPENPROM_MINOR 139
#define DMAPI_MINOR		140	/* DMAPI */
#define NVRAM_MINOR 144
#define SGI_MMTIMER        153
#define STORE_QUEUE_MINOR	155
#define I2O_MINOR 166
#define MICROCODE_MINOR		184
#define MWAVE_MINOR	219		/* ACP/Mwave Modem */
#define MPT_MINOR	220
#define MISC_DYNAMIC_MINOR 255


#define TUN_MINOR	     200
#define	HPET_MINOR	     228
#define KVM_MINOR            232
#define HDMI_MINOR		239
#define ATMEGA_IO_MINOR		240
#define KEYSTORE_MINOR		242
#define FLASHRW_MINOR		243
#define IRBLAST_MINOR		244
#define HELMET_MINOR		245
#define DISP_OUT_MINOR		246
#define NXP_RADIO_MINOR		247
#define REMOTEFM_PCF8575_MINOR	248
#define TPS_PM_MINOR		249

#define HSDPA_MINOR		250

#define PALKERN_MINOR		134
struct device;

struct miscdevice  {
	int minor;
	const char *name;
	const struct file_operations *fops;
	struct list_head list;
	struct device *parent;
	struct device *this_device;
};

extern int misc_register(struct miscdevice * misc);
extern int misc_deregister(struct miscdevice * misc);

#define MODULE_ALIAS_MISCDEV(minor)				\
	MODULE_ALIAS("char-major-" __stringify(MISC_MAJOR)	\
	"-" __stringify(minor))
#endif
