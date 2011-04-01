/*
 * sdrc.c
 *
 *  Created on: Apr 28, 2009
 *      Author: Matthias Welwarsky ARCHOS S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * 
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/io.h>

#define OMAP343X_CTRL_BASE	0x48002000
#define OMAP343X_CONTROL_PROG_IO0	(OMAP343X_CTRL_BASE + 0x270 + 0x1d4)

static struct sysdev_class sdrc_sysclass = {
	set_kset_name("sdrc"),
};

static struct sys_device sdrc_sysdev = {
	.id = -1,
	.cls = &sdrc_sysclass,
};

/* low data bits drive strength */
static ssize_t show_lowdata_drivestrength(struct sys_device *dev, char *buf)
{
	return sprintf(buf, "%d\n", !!(omap_readl(OMAP343X_CONTROL_PROG_IO0) & (1UL<<31)));
}
static ssize_t store_lowdata_drivestrength(struct sys_device *dev, 
		const char *buf, size_t count)
{
	int low_high = !!simple_strtol(buf, NULL, 10);

	omap_writel( (omap_readl(OMAP343X_CONTROL_PROG_IO0) & ~(1UL<<31)) | (low_high << 31),
			OMAP343X_CONTROL_PROG_IO0);
	
	return count;
}
static struct sysdev_attribute attr_lowdata_drivestrength = {
	.attr = { .name = "lowdata_drivestrength", .mode = S_IRUGO|S_IWUSR },
	.show = show_lowdata_drivestrength,
	.store = store_lowdata_drivestrength,
};

/* high data bits drive strength */
static ssize_t show_highdata_drivestrength(struct sys_device *dev, char *buf)
{
	return sprintf(buf, "%d\n", !!(omap_readl(OMAP343X_CONTROL_PROG_IO0) & (1UL<<30)));
}
static ssize_t store_highdata_drivestrength(struct sys_device *dev, 
		const char *buf, size_t count)
{
	int low_high = !!simple_strtol(buf, NULL, 10);

	omap_writel( (omap_readl(OMAP343X_CONTROL_PROG_IO0) & ~(1UL<<30)) | (low_high << 30),
			OMAP343X_CONTROL_PROG_IO0);
	
	return count;
}
static struct sysdev_attribute attr_highdata_drivestrength = {
	.attr = { .name = "highdata_drivestrength", .mode = S_IRUGO|S_IWUSR },
	.show = show_highdata_drivestrength,
	.store = store_highdata_drivestrength,
};

static ssize_t show_addrctrl_drivestrength(struct sys_device *dev, char *buf)
{
	return sprintf(buf, "%d\n", !!(omap_readl(OMAP343X_CONTROL_PROG_IO0) & (1UL<<29)));
}
static ssize_t store_addrctrl_drivestrength(struct sys_device *dev, 
		const char *buf, size_t count)
{
	int low_high = !!simple_strtol(buf, NULL, 10);

	omap_writel( (omap_readl(OMAP343X_CONTROL_PROG_IO0) & ~(1UL<<29)) | (low_high << 29),
			OMAP343X_CONTROL_PROG_IO0);
	
	return count;
}
static struct sysdev_attribute attr_addrctrl_drivestrength = {
	.attr = { .name = "addrctrl_drivestrength", .mode = S_IRUGO|S_IWUSR },
	.show = show_addrctrl_drivestrength,
	.store = store_addrctrl_drivestrength,
};

static ssize_t show_ncs0_drivestrength(struct sys_device *dev, char *buf)
{
	return sprintf(buf, "%d\n", !!(omap_readl(OMAP343X_CONTROL_PROG_IO0) & (1UL<<28)));
}
static ssize_t store_ncs0_drivestrength(struct sys_device *dev, 
		const char *buf, size_t count)
{
	int low_high = !!simple_strtol(buf, NULL, 10);

	omap_writel( (omap_readl(OMAP343X_CONTROL_PROG_IO0) & ~(1UL<<28)) | (low_high << 28),
			OMAP343X_CONTROL_PROG_IO0);
	
	return count;
}
static struct sysdev_attribute attr_ncs0_drivestrength = {
	.attr = { .name = "ncs0_drivestrength", .mode = S_IRUGO|S_IWUSR },
	.show = show_ncs0_drivestrength,
	.store = store_ncs0_drivestrength,
};

static ssize_t show_ncs1_drivestrength(struct sys_device *dev, char *buf)
{
	return sprintf(buf, "%d\n", !!(omap_readl(OMAP343X_CONTROL_PROG_IO0) & (1UL<<27)));
}
static ssize_t store_ncs1_drivestrength(struct sys_device *dev, 
		const char *buf, size_t count)
{
	int low_high = !!simple_strtol(buf, NULL, 10);

	omap_writel( (omap_readl(OMAP343X_CONTROL_PROG_IO0) & ~(1UL<<27)) | (low_high << 27),
			OMAP343X_CONTROL_PROG_IO0);
	
	return count;
}
static struct sysdev_attribute attr_ncs1_drivestrength = {
	.attr = { .name = "ncs1_drivestrength", .mode = S_IRUGO|S_IWUSR },
	.show = show_ncs1_drivestrength,
	.store = store_ncs1_drivestrength,
};

static int __init omap2_sdrc_sysdev_init(void)
{
	int low_high = 1;

	sysdev_class_register(&sdrc_sysclass);
	
	sysdev_register(&sdrc_sysdev);
	sysdev_create_file(&sdrc_sysdev, &attr_lowdata_drivestrength);
	sysdev_create_file(&sdrc_sysdev, &attr_highdata_drivestrength);
	sysdev_create_file(&sdrc_sysdev, &attr_addrctrl_drivestrength);
	sysdev_create_file(&sdrc_sysdev, &attr_ncs0_drivestrength);
	sysdev_create_file(&sdrc_sysdev, &attr_ncs1_drivestrength);

	/* set addrctrl drive strength to high by default */
	omap_writel( (omap_readl(OMAP343X_CONTROL_PROG_IO0) & ~(1UL<<29)) | (low_high << 29),
			OMAP343X_CONTROL_PROG_IO0);

	return 0;
}

arch_initcall(omap2_sdrc_sysdev_init);
