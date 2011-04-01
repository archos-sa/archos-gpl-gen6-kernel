#
# Makefile for the gci-pal-mapper and testcases.
#

ifndef CROSS_COMPILE
CROSS_COMPILE=arm-linux-
endif
#CROSS=arm_v5t_le-
#CROSS=arm_v6_vfp_le-

ifndef SENSOR
SENSOR=5MP_RAW
endif

ifeq ($(SENSOR),5MP_RAW)
SENSOR_FILE = mt9p012
SENSOR_EN := -DRAW_5MP_PARLL_GR2
endif

ifeq ($(SENSOR),3MP_RAW)
SENSOR_FILE := mt9t012
endif

ifeq ($(SENSOR),3MP_SMART)
SENSOR_FILE = ov3640
SENSOR_EN := -DSMART_3MP_PARLL_OV
endif

CC := arm-none-linux-gnueabi-gcc #$(CROSS_COMPILE)gcc

KDIR        := /vobs/wtbu/OMAPSW_L/linux/kernel_org/2.6_kernel
CFLAGS := -I$(KDIR)/include

all: gci-pal-mapper.o streaming imagecapture read read_save


gci-pal-mapper.o: $(SENSOR_FILE)_config.o gci-pal-mapper.c ../paluser.o 
	$(CC) -c -I$(KDIR)/include ./$(SENSOR_FILE)_config.o gci-pal-mapper.c  ../paluser.o $(SENSOR_EN)

$(SENSOR_FILE)_config.o: $(SENSOR_FILE)_config.c
	$(CC) -c -I$(KDIR)/include $(SENSOR_FILE)_config.c $(SENSOR_EN)

streaming: streaming.c gci-pal-mapper.o ./$(SENSOR_FILE)_config.o ../paluser.o
	$(CC) -g -I$(KDIR)/include streaming.c  gci-pal-mapper.o ./$(SENSOR_FILE)_config.o ../paluser.o  -o streaming $(SENSOR_EN)

imagecapture: imagecapture.c gci-pal-mapper.o ../paluser.o
	$(CC) -g -I$(KDIR)/include imagecapture.c  gci-pal-mapper.o ./$(SENSOR_FILE)_config.o ../paluser.o -o imagecapture $(SENSOR_EN)

read: read.c gci-pal-mapper.o ../paluser.o
	$(CC) -g -I$(KDIR)/include read.c  gci-pal-mapper.o ./$(SENSOR_FILE)_config.o ../paluser.o -o read $(SENSOR_EN)

read_save: read_save.c gci-pal-mapper.o ../paluser.o
	$(CC) -g -I$(KDIR)/include read_save.c  gci-pal-mapper.o ./$(SENSOR_FILE)_config.o ../paluser.o -o read_save $(SENSOR_EN)
	
clean:
	rm -f  gci-pal-mapper *~ *.o *.bak streaming *~ *.o *.bak imagecapture *~ *.o *.bak read *~ *.o *.bak read_save *~ *.o *.bak
