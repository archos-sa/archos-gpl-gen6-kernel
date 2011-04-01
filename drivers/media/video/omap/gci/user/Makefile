#
# Makefile for the paluser.
#

ifndef CROSS_COMPILE
CROSS_COMPILE=arm-linux-
endif

CC := arm-none-linux-gnueabi-gcc

KDIR        := /vobs/wtbu/OMAPSW_L/linux/kernel_org/2.6_kernel
CFLAGS := -I$(KDIR)/include

all: paluser.o

paluser.o: paluser.c
	$(CC) -c -I$(KDIR)/include paluser.c


clean:
	rm -f paluser *~ *.o *.bak


