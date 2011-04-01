CONFIG_DEBUG=n

# Proc in /proc/net/wlan
CONFIG_PROC_DEBUG=n

# Linux Power Management
CONFIG_ENABLE_PM=n

# Re-association in driver
CONFIG_REASSOCIATION=y

# Manufacturing firmware support
CONFIG_MFG_CMD_SUPPORT=y

ifeq ($(CONFIG_DEBUG),1)
	CFLAGS += -DDEBUG_LEVEL1
endif

ifeq ($(CONFIG_DEBUG),2)
	CFLAGS += -DDEBUG_LEVEL1
	CFLAGS += -DDEBUG_LEVEL2
	DBG=	-dbg
endif

ifeq ($(CONFIG_PROC_DEBUG),y)
	CFLAGS += -DPROC_DEBUG
	export CONFIG_PROC_DEBUG
endif

ifeq ($(CONFIG_ENABLE_PM),y)
	CFLAGS += -DENABLE_PM
endif

ifeq ($(CONFIG_REASSOCIATION),y)
	CFLAGS += -DREASSOCIATION
endif

ifeq ($(CONFIG_MFG_CMD_SUPPORT),y)
	CFLAGS += -DMFG_CMD_SUPPORT
endif

#CFLAGS += -DUSE_FIRMWARE_KERNEL_MODULE

sd8xxx-objs =	wlan/wlan_main.o wlan/wlan_fw.o wlan/wlan_wext.o \
		wlan/wlan_rx.o wlan/wlan_tx.o \
		wlan/wlan_cmd.o wlan/wlan_cmdresp.o \
		wlan/wlan_proc.o wlan/wlan_scan.o wlan/wlan_join.o \
		wlan/wlan_wmm.o wlan/wlan_11d.o wlan/wlan_fops.o \
		if/if_sdio_fb/if_sdio.o

ifeq ($(CONFIG_PROC_DEBUG), y)
sd8xxx-objs += wlan/wlan_debug.o
endif

obj-$(CONFIG_MMC_SD8686) += sd8xxx.o

	CFLAGS += -DFPNUM='"13"'
	CFLAGS += -DFB

ifdef CONFIG_MMC_SD8686
	EXTRA_CFLAGS += -Idrivers/net/wireless/8686_wlan/os/linux -Idrivers/net/wireless/8686_wlan/wlan


ifdef CONFIG_MMC_DEBUG
#	CFLAGS += -DDEBUG_LEVEL1
#	CFLAGS += -DDEBUG_LEVEL2
#	CFLAGS += -DDEBUG_LEVEL3
#	CFLAGS += -DDEBUG_LEVEL4
#	DBG=    -dbg
endif

endif 
