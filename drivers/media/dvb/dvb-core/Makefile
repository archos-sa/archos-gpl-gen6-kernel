#
# Makefile for the kernel DVB device drivers.
#

dvb-core-objs := dvbdev.o dmxdev.o dvb_demux.o dvb_filter.o 	\
		 dvb_ca_en50221.o dvb_frontend.o 		\
		 dvb_net.o dvb_ringbuffer.o dvb_math.o

obj-$(CONFIG_DVB_CORE) += dvb-core.o

extra-cflags-$(CONFIG_VPFE_DVB) += -DDAVINCI_VPFE_DVB=1

EXTRA_CFLAGS += $(extra-cflags-y) $(extra-cflags-m)
