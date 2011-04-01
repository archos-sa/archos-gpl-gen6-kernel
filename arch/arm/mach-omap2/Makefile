#
# Makefile for the linux kernel.
#

# Common support
obj-y := irq.o id.o io.o memory.o mux.o devices.o \
	 serial.o gpmc.o timer-gp.o

#obj-$(CONFIG_OMAP_MPU_TIMER)		+= timer-gp.o

# Power Management
obj-$(CONFIG_MACH_OMAP_2430SDP) += pm.o sleep.o

# DSP
obj-$(CONFIG_OMAP_MMU_FWK)	+= mmu_mach.o
obj-$(CONFIG_OMAP_MBOX_FWK)	+= mailbox_mach.o
mailbox_mach-objs		:= mailbox.o
mmu_mach-objs			:= mmu.o

# Specific board support
obj-$(CONFIG_MACH_OMAP_2430SDP)		+= clock.o
obj-$(CONFIG_MACH_OMAP_2430SDP)		+= sram-fn.o
obj-$(CONFIG_MACH_OMAP_2430SDP)		+= prcm.o
obj-$(CONFIG_ARCH_OMAP34XX) 		+= pm_34xx.o \
					   sleep_34xx.o \
					   prcm-debug.o \
					   sram-fn_34xx.o \
					   clock_34xx.o \
					   resource_34xx.o \
					   prcm_34xx.o \
					   sr_core.o \
					   sdrc.o
obj-$(CONFIG_MACH_OMAP_GENERIC)		+= board-generic.o
obj-$(CONFIG_MACH_OMAP_H4)		+= board-h4.o
obj-$(CONFIG_MACH_OMAP_2430SDP)		+= board-2430sdp.o \
					   board-2430sdp-flash.o \
					   board-2430sdp-usb.o \
					   board-2430sdp-hsmmc.o
obj-$(CONFIG_MACH_OMAP_3430SDP)		+= board-3430sdp.o \
					   board-3430sdp-flash.o \
					   board-3430sdp-usb.o \
					   board-3430sdp-hsmmc.o \
					   battery.o
obj-$(CONFIG_MACH_OMAP3_EVM) 		+= board-omap3evm.o \
					   board-omap3evm-flash.o \
					   board-omap3evm-usb.o \
					   board-omap3evm-hsmmc.o
obj-$(CONFIG_MACH_OMAP_APOLLON)		+= board-apollon.o \
					   board-apollon-keys.o
obj-$(CONFIG_MACH_NOKIA_N800)		+= board-n800.o board-n800-flash.o \
					   board-n800-mmc.o board-n800-bt.o \
					   board-n800-audio.o board-n800-usb.o \
					   board-n800-dsp.o

# Archos Gen6 board specific
obj-$(CONFIG_MACH_ARCHOS_G6)		+= board-archosg6-hsmmc.o \
				           board-archosg6-usb.o \
				           board-archosg6-memory.o \
				           battery.o \
				           pwm-gp.o \
					   archosg6-audio.o \
				           archosg6-display.o \
				           archosg6-cradle.o \
				           archosg6-wakeup.o \
				           archosg6-videoin.o \
				           archosg6-keys.o \
					   archosg6-wakeup.o \
					   archosg6-dieID.o \
					   archosg6-touchscreen.o \
					   archosg6-irr.o \
					   archosg6-wifi.o

obj-$(CONFIG_MACH_ARCHOS_G6S)		+= board-archosg6s.o
obj-$(CONFIG_MACH_ARCHOS_G6H)		+= board-archosg6h.o \
					   archosg6-usb2sata.o
obj-$(CONFIG_MACH_ARCHOS_G6PLUS)	+= board-archosg6plus.o \
					   archosg6-hsdpa.o \
					   archosg6-usbhdd.o
obj-$(CONFIG_MACH_ARCHOS_G6L)		+= board-archosg6l.o
obj-$(CONFIG_MACH_ARCHOS_G6TV)		+= board-archosg6tv.o

# TUSB 6010 chips
obj-$(CONFIG_MACH_OMAP2_TUSB6010)	+= usb-tusb6010.o

# TPS65023 power companion
obj-$(CONFIG_TPS65023)			+= tps65023.o

# FIQ Support
obj-$(CONFIG_FIQ) += fiq.o

