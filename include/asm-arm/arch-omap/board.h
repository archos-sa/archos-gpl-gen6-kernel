/*
 *  linux/include/asm-arm/arch-omap/board.h
 *
 *  Information structures for board-specific data
 *
 *  Copyright (C) 2004	Nokia Corporation
 *  Written by Juha Yrjölä <juha.yrjola@nokia.com>
 */

#ifndef _OMAP_BOARD_H
#define _OMAP_BOARD_H

#include <linux/types.h>

#include <asm/arch/gpio-switch.h>

/* Different peripheral ids */
#define OMAP_TAG_CLOCK		0x4f01
#define OMAP_TAG_MMC		0x4f02
#define OMAP_TAG_SERIAL_CONSOLE 0x4f03
#define OMAP_TAG_USB		0x4f04
#define OMAP_TAG_LCD		0x4f05
#define OMAP_TAG_GPIO_SWITCH	0x4f06
#define OMAP_TAG_UART		0x4f07
#define OMAP_TAG_FBMEM		0x4f08
#define OMAP_TAG_STI_CONSOLE	0x4f09
#define OMAP_TAG_CAMERA_SENSOR	0x4f0a
#define OMAP_TAG_PARTITION      0x4f0b
#define OMAP_TAG_TEA5761	0x4f10
#define OMAP_TAG_TMP105		0x4f11

#define OMAP_TAG_BOOT_REASON    0x4f80
#define OMAP_TAG_FLASH_PART_STR	0x4f81
#define OMAP_TAG_VERSION_STR	0x4f82

struct omap_clock_config {
	/* 0 for 12 MHz, 1 for 13 MHz and 2 for 19.2 MHz */
	u8 system_clock_type;
};

struct omap_mmc_conf {
	unsigned enabled:1;
	/* nomux means "standard" muxing is wrong on this board, and that
	 * board-specific code handled it before common init logic.
	 */
	unsigned nomux:1;
	/* switch pin can be for card detect (default) or card cover */
	unsigned cover:1;
	/* 4 wire signaling is optional, and is only used for SD/SDIO */
	unsigned wire4:1;
	/* 8 wire signaling is optional */
	unsigned wire8:1;
	/* use the internal clock */
	unsigned internal_clock:1;
	s16 power_pin;
	s16 switch_pin;
	s16 wp_pin;
	/* maximum clock for this host */
	u32 f_max;
};

struct omap_mmc_config {
	struct omap_mmc_conf mmc[3];
};

struct omap_serial_console_config {
	u8 console_uart;
	u32 console_speed;
};

struct omap_sti_console_config {
	unsigned enable:1;
	u8 channel;
};

struct omap_camera_sensor_config {
	u16 reset_gpio;
	int (*power_on)(void * data);
	int (*power_off)(void * data);
};

struct omap_usb_config {
	/* Configure drivers according to the connectors on your board:
	 *  - "A" connector (rectagular)
	 *	... for host/OHCI use, set "register_host".
	 *  - "B" connector (squarish) or "Mini-B"
	 *	... for device/gadget use, set "register_dev".
	 *  - "Mini-AB" connector (very similar to Mini-B)
	 *	... for OTG use as device OR host, initialize "otg"
	 */
	unsigned	register_host:1;
	unsigned	register_dev:1;
	u8		otg;	/* port number, 1-based:  usb1 == 2 */

	u8		hmc_mode;

	/* implicitly true if otg:  host supports remote wakeup? */
	u8		rwc;

	/* signaling pins used to talk to transceiver on usbN:
	 *  0 == usbN unused
	 *  2 == usb0-only, using internal transceiver
	 *  3 == 3 wire bidirectional
	 *  4 == 4 wire bidirectional
	 *  6 == 6 wire unidirectional (or TLL)
	 */
	u8		pins[3];
};

struct omap_lcd_config {
	char panel_name[16];
	char ctrl_name[16];
	s16  nreset_gpio;
	u8   data_lines;
};

struct device;
struct fb_info;
struct omap_backlight_config {
	int default_intensity;
	int (*set_power)(struct device *dev, int state);
	int (*check_fb)(struct fb_info *fb);
};

struct omap_fbmem_config {
	u32 start;
	u32 size;
};

struct omap_pwm_led_platform_data {
	const char *name;
	int intensity_timer;
	int blink_timer;
	void (*set_power)(struct omap_pwm_led_platform_data *self, int on_off);
};

/* See include/asm-arm/arch-omap/gpio-switch.h for definitions */
struct omap_gpio_switch_config {
	char name[12];
	u16 gpio;
	int flags:4;
	int type:4;
	int key_code:24; /* Linux key code */
};

struct omap_uart_config {
	/* Bit field of UARTs present; bit 0 --> UART1 */
	unsigned int enabled_uarts;
};

struct omap_tea5761_config {
	u16 enable_gpio;
};

struct omap_board_wlan_config {
	int (*set_config)(void);
	void (*set_power)(int enable);
	void (*reset)(void);
	void *priv_data;	
};

/* This cannot be passed from the bootloader */
struct omap_tmp105_config {
	u16 tmp105_irq_pin;
	int (* set_power)(int enable);
};

struct omap_partition_config {
	char name[16];
	unsigned int size;
	unsigned int offset;
	/* same as in include/linux/mtd/partitions.h */
	unsigned int mask_flags;
};

struct omap_flash_part_str_config {
	char part_table[0];
};

struct omap_boot_reason_config {
	char reason_str[12];
};

struct omap_version_config {
	char component[12];
	char version[12];
};


#include <asm-arm/arch-omap/board-nokia.h>
#include <asm-arm/arch-omap/board-archosg6.h>

struct omap_board_config_entry {
	u16 tag;
	u16 len;
	u8  data[0];
};

struct omap_board_config_kernel {
	u16 tag;
	const void *data;
};

extern const void *__omap_get_config(u16 tag, size_t len, int nr);

#define omap_get_config(tag, type) \
	((const type *) __omap_get_config((tag), sizeof(type), 0))
#define omap_get_nr_config(tag, type, nr) \
	((const type *) __omap_get_config((tag), sizeof(type), (nr)))

extern const void *omap_get_var_config(u16 tag, size_t *len);

extern struct omap_board_config_kernel *omap_board_config;
extern int omap_board_config_size;


/* for TI reference platforms sharing the same debug card */
extern int debug_card_init(u32 addr, unsigned gpio);

#endif
