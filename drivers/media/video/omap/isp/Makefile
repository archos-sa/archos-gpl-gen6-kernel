# Makefile for OMAP3 ISP driver

obj-$(CONFIG_VIDEO_OMAP34XX_ISP) += isp_driver.o
ifdef CONFIG_ARCH_OMAP3410
isp_driver-objs = 	isp.o  ispccdc.o ispmmu.o
else
isp_driver-objs =	isp.o isph3a.o isphist.o isppreview.o ispresizer.o ispccdc.o ispmmu.o
endif

