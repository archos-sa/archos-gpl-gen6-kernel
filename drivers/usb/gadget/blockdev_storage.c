/*
 * file_storage.c -- File-backed USB Storage Gadget, for USB development
 *
 * Copyright (C) 2003, 2004 Alan Stern
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/*
 * The File-backed Storage Gadget acts as a USB Mass Storage device,
 * appearing to the host as a disk drive.  In addition to providing an
 * example of a genuinely useful gadget driver for a USB device, it also
 * illustrates a technique of double-buffering for increased throughput.
 * Last but not least, it gives an easy way to probe the behavior of the
 * Mass Storage drivers in a USB host.
 *
 * Backing storage is provided by a regular file or a block device, specified
 * by the "file" module parameter.  Access can be limited to read-only by
 * setting the optional "ro" module parameter.  The gadget will indicate that
 * it has removable media if the optional "removable" module parameter is set.
 *
 * The gadget supports the Control-Bulk (CB), Control-Bulk-Interrupt (CBI),
 * and Bulk-Only (also known as Bulk-Bulk-Bulk or BBB) transports, selected
 * by the optional "transport" module parameter.  It also supports the
 * following protocols: RBC (0x01), ATAPI or SFF-8020i (0x02), QIC-157 (0c03),
 * UFI (0x04), SFF-8070i (0x05), and transparent SCSI (0x06), selected by
 * the optional "protocol" module parameter.  In addition, the default
 * Vendor ID, Product ID, and release number can be overridden.
 *
 * There is support for multiple logical units (LUNs), each of which has
 * its own backing file.  The number of LUNs can be set using the optional
 * "luns" module parameter (anywhere from 1 to 8), and the corresponding
 * files are specified using comma-separated lists for "file" and "ro".
 * The default number of LUNs is taken from the number of "file" elements;
 * it is 1 if "file" is not given.  If "removable" is not set then a backing
 * file must be specified for each LUN.  If it is set, then an unspecified
 * or empty backing filename means the LUN's medium is not loaded.
 *
 * Requirements are modest; only a bulk-in and a bulk-out endpoint are
 * needed (an interrupt-out endpoint is also needed for CBI).  The memory
 * requirement amounts to two 16K buffers, size configurable by a parameter.
 * Support is included for both full-speed and high-speed operation.
 *
 * Module options:
 *
 *	file=filename[,filename...]
 *				Required if "removable" is not set, names of
 *					the files or block devices used for
 *					backing storage
 *	ro=b[,b...]		Default false, booleans for read-only access
 *	removable		Default false, boolean for removable media
 *	luns=N			Default N = number of filenames, number of
 *					LUNs to support
 *	transport=XXX		Default BBB, transport name (CB, CBI, or BBB)
 *	protocol=YYY		Default SCSI, protocol name (RBC, 8020 or
 *					ATAPI, QIC, UFI, 8070, or SCSI;
 *					also 1 - 6)
 *	vendor=0xVVVV		Default 0x0525 (NetChip), USB Vendor ID
 *	product=0xPPPP		Default 0xa4a5 (FSG), USB Product ID
 *	release=0xRRRR		Override the USB release number (bcdDevice)
 *	manufacturer_name=mn	Override the USB manufacturer name string (descriptor)
 *	product_name=pd		Override the USB product name string (descriptor)
 *	serial_string=sn	Override the USB serial number string (descriptor)
 *	buflen=N		Default N=16384, buffer size used (will be
 *					rounded down to a multiple of
 *					PAGE_CACHE_SIZE)
 *	stall			Default determined according to the type of
 *					USB device controller (usually true),
 *					boolean to permit the driver to halt
 *					bulk endpoints
 *
 * If CONFIG_USB_FILE_STORAGE_TEST is not set, only the "file", "ro",
 * "removable", and "luns" options are available; default values are used
 * for everything else.
 *
 * The pathnames of the backing files and the ro settings are available in
 * the attribute files "file" and "ro" in the lun<n> subdirectory of the
 * gadget's sysfs directory.  If the "removable" option is set, writing to
 * these files will simulate ejecting/loading the medium (writing an empty
 * line means eject) and adjusting a write-enable tab.  Changes to the ro
 * setting are not allowed when the medium is loaded.
 *
 * This gadget driver is heavily based on "Gadget Zero" by David Brownell.
 */


/*
 *				Driver Design
 *
 * The FSG driver is fairly straightforward.  There is a main kernel
 * thread that handles most of the work.  Interrupt routines field
 * callbacks from the controller driver: bulk- and interrupt-request
 * completion notifications, endpoint-0 events, and disconnect events.
 * Completion events are passed to the main thread by wakeup calls.  Many
 * ep0 requests are handled at interrupt time, but SetInterface,
 * SetConfiguration, and device reset requests are forwarded to the
 * thread in the form of "exceptions" using SIGUSR1 signals (since they
 * should interrupt any ongoing file I/O operations).
 *
 * The thread's main routine implements the standard command/data/status
 * parts of a SCSI interaction.  It and its subroutines are full of tests
 * for pending signals/exceptions -- all this polling is necessary since
 * the kernel has no setjmp/longjmp equivalents.  (Maybe this is an
 * indication that the driver really wants to be running in userspace.)
 * An important point is that so long as the thread is alive it keeps an
 * open reference to the backing file.  This will prevent unmounting
 * the backing file's underlying filesystem and could cause problems
 * during system shutdown, for example.  To prevent such problems, the
 * thread catches INT, TERM, and KILL signals and converts them into
 * an EXIT exception.
 *
 * In normal operation the main thread is started during the gadget's
 * fsg_bind() callback and stopped during fsg_unbind().  But it can also
 * exit when it receives a signal, and there's no point leaving the
 * gadget running when the thread is dead.  So just before the thread
 * exits, it deregisters the gadget driver.  This makes things a little
 * tricky: The driver is deregistered at two places, and the exiting
 * thread can indirectly call fsg_unbind() which in turn can tell the
 * thread to exit.  The first problem is resolved through the use of the
 * REGISTERED atomic bitflag; the driver will only be deregistered once.
 * The second problem is resolved by having fsg_unbind() check
 * fsg->state; it won't try to stop the thread if the state is already
 * FSG_STATE_TERMINATED.
 *
 * To provide maximum throughput, the driver uses a circular pipeline of
 * buffer heads (struct fsg_buffhd).  In principle the pipeline can be
 * arbitrarily long; in practice the benefits don't justify having more
 * than 2 stages (i.e., double buffering).  But it helps to think of the
 * pipeline as being a long one.  Each buffer head contains a bulk-in and
 * a bulk-out request pointer (since the buffer can be used for both
 * output and input -- directions always are given from the host's
 * point of view) as well as a pointer to the buffer and various state
 * variables.
 *
 * Use of the pipeline follows a simple protocol.  There is a variable
 * (fsg->next_buffhd_to_fill) that points to the next buffer head to use.
 * At any time that buffer head may still be in use from an earlier
 * request, so each buffer head has a state variable indicating whether
 * it is EMPTY, FULL, or BUSY.  Typical use involves waiting for the
 * buffer head to be EMPTY, filling the buffer either by file I/O or by
 * USB I/O (during which the buffer head is BUSY), and marking the buffer
 * head FULL when the I/O is complete.  Then the buffer will be emptied
 * (again possibly by USB I/O, during which it is marked BUSY) and
 * finally marked EMPTY again (possibly by a completion routine).
 *
 * A module parameter tells the driver to avoid stalling the bulk
 * endpoints wherever the transport specification allows.  This is
 * necessary for some UDCs like the SuperH, which cannot reliably clear a
 * halt on a bulk endpoint.  However, under certain circumstances the
 * Bulk-only specification requires a stall.  In such cases the driver
 * will halt the endpoint and set a flag indicating that it should clear
 * the halt in software during the next device reset.  Hopefully this
 * will permit everything to work correctly.  Furthermore, although the
 * specification allows the bulk-out endpoint to halt when the host sends
 * too much data, implementing this would cause an unavoidable race.
 * The driver will always use the "no-stall" approach for OUT transfers.
 *
 * One subtle point concerns sending status-stage responses for ep0
 * requests.  Some of these requests, such as device reset, can involve
 * interrupting an ongoing file I/O operation, which might take an
 * arbitrarily long time.  During that delay the host might give up on
 * the original ep0 request and issue a new one.  When that happens the
 * driver should not notify the host about completion of the original
 * request, as the host will no longer be waiting for it.  So the driver
 * assigns to each ep0 request a unique tag, and it keeps track of the
 * tag value of the request associated with a long-running exception
 * (device-reset, interface-change, or configuration-change).  When the
 * exception handler is finished, the status-stage response is submitted
 * only if the current ep0 request tag is equal to the exception request
 * tag.  Thus only the most recently received ep0 request will get a
 * status-stage response.
 *
 * Warning: This driver source file is too long.  It ought to be split up
 * into a header file plus about 3 separate .c files, to handle the details
 * of the Gadget, USB Mass Storage, and SCSI protocols.
 */


#undef DEBUG
#undef VERBOSE
#undef DUMP_MSGS

#include <asm/system.h>
#include <asm/uaccess.h>

#include <linux/bitops.h>
#include <linux/blkdev.h>
#include <linux/compiler.h>
#include <linux/completion.h>
#include <linux/dcache.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fcntl.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kref.h>
#include <linux/kthread.h>
#include <linux/limits.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pagemap.h>
#include <linux/rwsem.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/freezer.h>
#include <linux/utsname.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/syscalls.h>
#include <asm/atomic.h>

#include <linux/usb/ch9.h>
#include <linux/usb_gadget.h>

#include <net/sock.h>
#include <linux/net.h>
#include <linux/netlink.h> /* for sending sector/count commands */

#include "gadget_chips.h"


/*-------------------------------------------------------------------------*/

#define DRIVER_DESC		"Block-device-backed Storage Gadget"
#define DRIVER_NAME		"g_blockdev_storage"
#define DRIVER_VERSION		"20 October 2004"

static const char longname[] = DRIVER_DESC;
static const char shortname[] = DRIVER_NAME;

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Alan Stern");
MODULE_LICENSE("Dual BSD/GPL");

/* Thanks to NetChip Technologies for donating this product ID.
 *
 * DO NOT REUSE THESE IDs with any other driver!!  Ever!!
 * Instead:  allocate your own, using normal USB-IF procedures. */
#define DRIVER_VENDOR_ID	0x0525	// NetChip
#define DRIVER_PRODUCT_ID	0xa4a5	// Linux-USB File-backed Storage Gadget

/*
 * This driver assumes self-powered hardware and has no way for users to
 * trigger remote wakeup.  It uses autoconfiguration to select endpoints
 * and endpoint addresses.
 */


/*-------------------------------------------------------------------------*/

#define xprintk(f,level,fmt,args...) \
	dev_printk(level , &(f)->gadget->dev , fmt , ## args)
#define yprintk(l,level,fmt,args...) \
	dev_printk(level , &(l)->dev , fmt , ## args)

#ifdef DEBUG
#define DBG(fsg,fmt,args...) \
	xprintk(fsg , KERN_DEBUG , fmt , ## args)
#define LDBG(lun,fmt,args...) \
	yprintk(lun , KERN_DEBUG , fmt , ## args)
#define MDBG(fmt,args...) \
	printk(KERN_DEBUG DRIVER_NAME ": " fmt , ## args)
#else
#define DBG(fsg,fmt,args...) \
	do { } while (0)
#define LDBG(lun,fmt,args...) \
	do { } while (0)
#define MDBG(fmt,args...) \
	do { } while (0)
#undef VERBOSE
#undef DUMP_MSGS
#endif /* DEBUG */

#ifdef VERBOSE
#define VDBG	DBG
#define VLDBG	LDBG
#else
#define VDBG(fsg,fmt,args...) \
	do { } while (0)
#define VLDBG(lun,fmt,args...) \
	do { } while (0)
#endif /* VERBOSE */

#define ERROR(fsg,fmt,args...) \
	xprintk(fsg , KERN_ERR , fmt , ## args)
#define LERROR(lun,fmt,args...) \
	yprintk(lun , KERN_ERR , fmt , ## args)

#define WARN(fsg,fmt,args...) \
	xprintk(fsg , KERN_WARNING , fmt , ## args)
#define LWARN(lun,fmt,args...) \
	yprintk(lun , KERN_WARNING , fmt , ## args)

#define INFO(fsg,fmt,args...) \
	xprintk(fsg , KERN_INFO , fmt , ## args)
#define LINFO(lun,fmt,args...) \
	yprintk(lun , KERN_INFO , fmt , ## args)

#define MINFO(fmt,args...) \
	printk(KERN_INFO DRIVER_NAME ": " fmt , ## args)


/*-------------------------------------------------------------------------*/

/* Encapsulate the module parameter settings */

#define MAX_LUNS	8

	/* Arggh!  There should be a module_param_array_named macro! */
static char		*file[MAX_LUNS] = {NULL, };
static int		ro[MAX_LUNS] = {0, };
static int		size[MAX_LUNS] = {0, };

static struct sock	* gadgetfs_sock;
static struct usb_gadgetfs_info_t gadgetfs_info;

static struct {
	int		num_filenames;
	int		num_ros;
	int		num_sizes;
	unsigned int	nluns;
	unsigned int	netlink;

	char		*transport_parm;
	char		*protocol_parm;
	int		removable;
	unsigned short	vendor;
	unsigned short	product;
	unsigned short	release;
	char		*manufacturer_name;
	char		*product_name;
	char		*serial_string;
	unsigned int	buflen;
	int		can_stall;

	int		transport_type;
	char		*transport_name;
	int		protocol_type;
	char		*protocol_name;

} mod_data = {					// Default values
	.transport_parm		= "BBB",
	.protocol_parm		= "SCSI",
	.removable		= 0,
	.vendor			= DRIVER_VENDOR_ID,
	.product		= DRIVER_PRODUCT_ID,
	.release		= 0xffff,	// Use controller chip type
	.manufacturer_name	= "Manufacturer        ",
	.product_name		= "Product             ",
	.serial_string		= "1234567890123",
	.buflen			= 65536,
	.can_stall		= 0,
	};


module_param_array(file, charp, &mod_data.num_filenames, S_IRUGO);
MODULE_PARM_DESC(file, "names of backing files or devices");

module_param_array(ro, bool, &mod_data.num_ros, S_IRUGO);
MODULE_PARM_DESC(ro, "true to force read-only");

module_param_array(size, int, &mod_data.num_sizes, S_IRUGO);
MODULE_PARM_DESC(size, "sizes of backing files or devices");

module_param_named(netlink, mod_data.netlink, uint, S_IRUGO);
MODULE_PARM_DESC(netlink, "PID of user process for netlink socket");

module_param_named(luns, mod_data.nluns, uint, S_IRUGO);
MODULE_PARM_DESC(luns, "number of LUNs");

module_param_named(removable, mod_data.removable, bool, S_IRUGO);
MODULE_PARM_DESC(removable, "true to simulate removable media");

module_param_named(vendor, mod_data.vendor, ushort, S_IRUGO);
MODULE_PARM_DESC(vendor, "USB Vendor ID");

module_param_named(product, mod_data.product, ushort, S_IRUGO);
MODULE_PARM_DESC(product, "USB Product ID");

module_param_named(release, mod_data.release, ushort, S_IRUGO);
MODULE_PARM_DESC(release, "USB release number");

module_param_named(manufacturer_name, mod_data.manufacturer_name, charp, S_IRUGO);
MODULE_PARM_DESC(manufacturer_name, "USB descriptor manufacturer name (string 20 char max)");

module_param_named(product_name, mod_data.product_name, charp, S_IRUGO);
MODULE_PARM_DESC(product_name, "USB descriptor product name (string 20 char max)");

module_param_named(serial_string, mod_data.serial_string, charp, S_IRUGO);
MODULE_PARM_DESC(serial_string, "USB descriptor serial number (string 13 char max)");

/* In the non-TEST version, only the module parameters listed above
 * are available. */
#ifdef CONFIG_USB_FILE_STORAGE_TEST

module_param_named(transport, mod_data.transport_parm, charp, S_IRUGO);
MODULE_PARM_DESC(transport, "type of transport (BBB, CBI, or CB)");

module_param_named(protocol, mod_data.protocol_parm, charp, S_IRUGO);
MODULE_PARM_DESC(protocol, "type of protocol (RBC, 8020, QIC, UFI, "
		"8070, or SCSI)");

module_param_named(buflen, mod_data.buflen, uint, S_IRUGO);
MODULE_PARM_DESC(buflen, "I/O buffer size");

module_param_named(stall, mod_data.can_stall, bool, S_IRUGO);
MODULE_PARM_DESC(stall, "false to prevent bulk stalls");

#endif /* CONFIG_USB_FILE_STORAGE_TEST */


/*-------------------------------------------------------------------------*/

/* USB protocol value = the transport method */
#define USB_PR_CBI	0x00		// Control/Bulk/Interrupt
#define USB_PR_CB	0x01		// Control/Bulk w/o interrupt
#define USB_PR_BULK	0x50		// Bulk-only

/* USB subclass value = the protocol encapsulation */
#define USB_SC_RBC	0x01		// Reduced Block Commands (flash)
#define USB_SC_8020	0x02		// SFF-8020i, MMC-2, ATAPI (CD-ROM)
#define USB_SC_QIC	0x03		// QIC-157 (tape)
#define USB_SC_UFI	0x04		// UFI (floppy)
#define USB_SC_8070	0x05		// SFF-8070i (removable)
#define USB_SC_SCSI	0x06		// Transparent SCSI

/* Bulk-only data structures */

/* Command Block Wrapper */
struct bulk_cb_wrap {
	__le32	Signature;		// Contains 'USBC'
	u32	Tag;			// Unique per command id
	__le32	DataTransferLength;	// Size of the data
	u8	Flags;			// Direction in bit 7
	u8	Lun;			// LUN (normally 0)
	u8	Length;			// Of the CDB, <= MAX_COMMAND_SIZE
	u8	CDB[16];		// Command Data Block
};

#define USB_BULK_CB_WRAP_LEN	31
#define USB_BULK_CB_SIG		0x43425355	// Spells out USBC
#define USB_BULK_IN_FLAG	0x80

/* Command Status Wrapper */
struct bulk_cs_wrap {
	__le32	Signature;		// Should = 'USBS'
	u32	Tag;			// Same as original command
	__le32	Residue;		// Amount not transferred
	u8	Status;			// See below
};

#define USB_BULK_CS_WRAP_LEN	13
#define USB_BULK_CS_SIG		0x53425355	// Spells out 'USBS'
#define USB_STATUS_PASS		0
#define USB_STATUS_FAIL		1
#define USB_STATUS_PHASE_ERROR	2

/* Bulk-only class specific requests */
#define USB_BULK_RESET_REQUEST		0xff
#define USB_BULK_GET_MAX_LUN_REQUEST	0xfe


/* CBI Interrupt data structure */
struct interrupt_data {
	u8	bType;
	u8	bValue;
};

#define CBI_INTERRUPT_DATA_LEN		2

/* CBI Accept Device-Specific Command request */
#define USB_CBI_ADSC_REQUEST		0x00


#define MAX_COMMAND_SIZE	16	// Length of a SCSI Command Data Block

/* SCSI commands that we recognize */
#define SC_FORMAT_UNIT			0x04
#define SC_INQUIRY			0x12
#define SC_MODE_SELECT_6		0x15
#define SC_MODE_SELECT_10		0x55
#define SC_MODE_SENSE_6			0x1a
#define SC_MODE_SENSE_10		0x5a
#define SC_PREVENT_ALLOW_MEDIUM_REMOVAL	0x1e
#define SC_READ_6			0x08
#define SC_READ_10			0x28
#define SC_READ_12			0xa8
#define SC_READ_CAPACITY		0x25
#define SC_READ_FORMAT_CAPACITIES	0x23
#define SC_RELEASE			0x17
#define SC_REQUEST_SENSE		0x03
#define SC_RESERVE			0x16
#define SC_SEND_DIAGNOSTIC		0x1d
#define SC_START_STOP_UNIT		0x1b
#define SC_SYNCHRONIZE_CACHE		0x35
#define SC_TEST_UNIT_READY		0x00
#define SC_VERIFY			0x2f
#define SC_WRITE_6			0x0a
#define SC_WRITE_10			0x2a
#define SC_WRITE_12			0xaa

/* SCSI Sense Key/Additional Sense Code/ASC Qualifier values */
#define SS_NO_SENSE				0
#define SS_COMMUNICATION_FAILURE		0x040800
#define SS_INVALID_COMMAND			0x052000
#define SS_INVALID_FIELD_IN_CDB			0x052400
#define SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE	0x052100
#define SS_LOGICAL_UNIT_NOT_SUPPORTED		0x052500
#define SS_MEDIUM_NOT_PRESENT			0x023a00
#define SS_MEDIUM_REMOVAL_PREVENTED		0x055302
#define SS_NOT_READY_TO_READY_TRANSITION	0x062800
#define SS_RESET_OCCURRED			0x062900
#define SS_SAVING_PARAMETERS_NOT_SUPPORTED	0x053900
#define SS_UNRECOVERED_READ_ERROR		0x031100
#define SS_WRITE_ERROR				0x030c02
#define SS_WRITE_PROTECTED			0x072700

#define SK(x)		((u8) ((x) >> 16))	// Sense Key byte, etc.
#define ASC(x)		((u8) ((x) >> 8))
#define ASCQ(x)		((u8) (x))


/*-------------------------------------------------------------------------*/

/*
 * These definitions will permit the compiler to avoid generating code for
 * parts of the driver that aren't used in the non-TEST version.  Even gcc
 * can recognize when a test of a constant expression yields a dead code
 * path.
 */

#ifdef CONFIG_USB_FILE_STORAGE_TEST

#define transport_is_bbb()	(mod_data.transport_type == USB_PR_BULK)
#define transport_is_cbi()	(mod_data.transport_type == USB_PR_CBI)
#define protocol_is_scsi()	(mod_data.protocol_type == USB_SC_SCSI)

#else

#define transport_is_bbb()	1
#define transport_is_cbi()	0
#define protocol_is_scsi()	1

#endif /* CONFIG_USB_FILE_STORAGE_TEST */


struct lun {
	struct file	*filp;
	loff_t		file_length;
	loff_t		num_sectors;

	unsigned int	ro : 1;
	unsigned int	prevent_medium_removal : 1;
	unsigned int	registered : 1;

	u32		sense_data;
	u32		sense_data_info;
	u32		unit_attention_data;
	
	atomic_t pending_bio_count; 
        atomic_t pending_write_count;

	struct device	dev;
};

#define backing_file_is_open(curlun)	((curlun)->filp != NULL)

static inline struct lun *dev_to_lun(struct device *dev)
{
	return container_of(dev, struct lun, dev);
}


/* Big enough to hold our biggest descriptor */
#define EP0_BUFSIZE	256
#define DELAYED_STATUS	(EP0_BUFSIZE + 999)	// An impossibly large value

/* Number of buffers we will use.  2 is enough for double-buffering */
#define NUM_BUFFERS     	4
#define MSC_MAX_NR_PAGES        16
#define DMA_ADDR_INVALID        (~(dma_addr_t)0)
#define MSC_READ                1
#define MSC_WRITE               2

enum fsg_buffer_state {
	BUF_STATE_EMPTY = 0,
	BUF_STATE_FULL,
	BUF_STATE_BUSY,
	BUF_STATE_DIRTY
};

struct fsg_buffhd {
	void				*buf;
	dma_addr_t			dma;
	volatile enum fsg_buffer_state	state;
	struct fsg_buffhd		*next;

	/* The NetChip 2280 is faster, and handles some protocol faults
	 * better, if we don't submit any short bulk-out read requests.
	 * So we will record the intended request length here. */
	unsigned int			bulk_out_intended_length;

	struct usb_request		*inreq;
	volatile int			inreq_busy;
	struct usb_request		*outreq;
	volatile int			outreq_busy;
	
	struct page			*pDataPage[MSC_MAX_NR_PAGES];
	unsigned int			amount;
	loff_t				*ppos;
	int				slot_num;
	struct lun                      *currentlun;
	struct fsg_dev			*fsg;	
	struct completion 		complete;
};

enum fsg_state {
	FSG_STATE_COMMAND_PHASE = -10,		// This one isn't used anywhere
	FSG_STATE_DATA_PHASE,
	FSG_STATE_STATUS_PHASE,

	FSG_STATE_IDLE = 0,
	FSG_STATE_ABORT_BULK_OUT,
	FSG_STATE_RESET,
	FSG_STATE_INTERFACE_CHANGE,
	FSG_STATE_CONFIG_CHANGE,
	FSG_STATE_DISCONNECT,
	FSG_STATE_EXIT,
	FSG_STATE_TERMINATED
};

enum data_direction {
	DATA_DIR_UNKNOWN = 0,
	DATA_DIR_FROM_HOST,
	DATA_DIR_TO_HOST,
	DATA_DIR_NONE
};

struct fsg_dev {
	/* lock protects: state, all the req_busy's, and cbbuf_cmnd */
	spinlock_t		lock;
	struct usb_gadget	*gadget;

	/* filesem protects: backing files in use */
	struct rw_semaphore	filesem;

	struct usb_ep		*ep0;		// Handy copy of gadget->ep0
	struct usb_request	*ep0req;	// For control responses
	volatile unsigned int	ep0_req_tag;
	const char		*ep0req_name;

	struct usb_request	*intreq;	// For interrupt responses
	volatile int		intreq_busy;
	struct fsg_buffhd	*intr_buffhd;

 	unsigned int		bulk_out_maxpacket;
	enum fsg_state		state;		// For exception handling
	unsigned int		exception_req_tag;

	u8			config, new_config;

	unsigned int		running : 1;
	unsigned int		bulk_in_enabled : 1;
	unsigned int		bulk_out_enabled : 1;
	unsigned int		intr_in_enabled : 1;
	unsigned int		phase_error : 1;
	unsigned int		short_packet_received : 1;
	unsigned int		bad_lun_okay : 1;

	unsigned long		atomic_bitflags;
#define REGISTERED		0
#define CLEAR_BULK_HALTS	1
#define SUSPENDED		2

	struct usb_ep		*bulk_in;
	struct usb_ep		*bulk_out;
	struct usb_ep		*intr_in;

	struct fsg_buffhd	*next_buffhd_to_fill;
	struct fsg_buffhd	*next_buffhd_to_drain;
	struct fsg_buffhd	buffhds[NUM_BUFFERS];	
	
	wait_queue_head_t	thread_wqh;
	int			thread_wakeup_needed;
	struct completion	thread_notifier;
	int			thread_pid;
	struct task_struct	*thread_task;
	sigset_t		thread_signal_mask;

	int			cmnd_size;
	u8			cmnd[MAX_COMMAND_SIZE];
	enum data_direction	data_dir;
	u32			data_size;
	u32			data_size_from_cmnd;
	u32			tag;
	unsigned int		lun;
	u32			residue;
	u32			usb_amount_left;

	/* The CB protocol offers no way for a host to know when a command
	 * has completed.  As a result the next command may arrive early,
	 * and we will still have to handle it.  For that reason we need
	 * a buffer to store new commands when using CB (or CBI, which
	 * does not oblige a host to wait for command completion either). */
	int			cbbuf_cmnd_size;
	u8			cbbuf_cmnd[MAX_COMMAND_SIZE];

	unsigned int		nluns;
	struct lun		*luns;
	struct lun		*curlun;
	struct completion	lun_released;

	/* read ahead */
	struct fsg_buffhd	ahead_bh;
	int			ahead_start;
	int			ahead_len;
	int			ahead_valid;
};

static const int ahead_size = 64 * 1024;

typedef void (*fsg_routine_t)(struct fsg_dev *);

static int inline exception_in_progress(struct fsg_dev *fsg)
{
	return (fsg->state > FSG_STATE_IDLE);
}

/* Make bulk-out requests be divisible by the maxpacket size */
static void inline set_bulk_out_req_length(struct fsg_dev *fsg,
		struct fsg_buffhd *bh, unsigned int length)
{
	unsigned int	rem;

	bh->bulk_out_intended_length = length;
	rem = length % fsg->bulk_out_maxpacket;
	if (rem > 0)
		length += fsg->bulk_out_maxpacket - rem;
	bh->outreq->length = length;
}

static struct fsg_dev			*the_fsg;
static struct usb_gadget_driver		fsg_driver;

static void	close_backing_file(struct lun *curlun);
static void	close_all_backing_files(struct fsg_dev *fsg);


/*-------------------------------------------------------------------------*/

#ifdef DUMP_MSGS

static void dump_msg(struct fsg_dev *fsg, const char *label,
		const u8 *buf, unsigned int length)
{
	unsigned int	start, num, i;
	char		line[52], *p;

	if (length >= 512)
		return;
	DBG(fsg, "%s, length %u:\n", label, length);

	start = 0;
	while (length > 0) {
		num = min(length, 16u);
		p = line;
		for (i = 0; i < num; ++i) {
			if (i == 8)
				*p++ = ' ';
			sprintf(p, " %02x", buf[i]);
			p += 3;
		}
		*p = 0;
		printk(KERN_DEBUG "%6x: %s\n", start, line);
		buf += num;
		start += num;
		length -= num;
	}
}

static void inline dump_cdb(struct fsg_dev *fsg)
{}

#else

static void inline dump_msg(struct fsg_dev *fsg, const char *label,
		const u8 *buf, unsigned int length)
{}

static void inline dump_cdb(struct fsg_dev *fsg)
{
	int	i;
	char	cmdbuf[3*MAX_COMMAND_SIZE + 1];

	for (i = 0; i < fsg->cmnd_size; ++i)
		sprintf(cmdbuf + i*3, " %02x", fsg->cmnd[i]);
	VDBG(fsg, "SCSI CDB: %s\n", cmdbuf);
}

#endif /* DUMP_MSGS */


static int fsg_set_halt(struct fsg_dev *fsg, struct usb_ep *ep)
{
	const char	*name;

	if (ep == fsg->bulk_in)
		name = "bulk-in";
	else if (ep == fsg->bulk_out)
		name = "bulk-out";
	else
		name = ep->name;
	DBG(fsg, "%s set halt\n", name);
	return usb_ep_set_halt(ep);
}


/*-------------------------------------------------------------------------*/

/* Routines for unaligned data access */

static u16 inline get_be16(u8 *buf)
{
	return ((u16) buf[0] << 8) | ((u16) buf[1]);
}

static u32 inline get_be32(u8 *buf)
{
	return ((u32) buf[0] << 24) | ((u32) buf[1] << 16) |
			((u32) buf[2] << 8) | ((u32) buf[3]);
}

static void inline put_be16(u8 *buf, u16 val)
{
	buf[0] = val >> 8;
	buf[1] = val;
}

static void inline put_be32(u8 *buf, u32 val)
{
	buf[0] = val >> 24;
	buf[1] = val >> 16;
	buf[2] = val >> 8;
	buf[3] = val & 0xff;
}


/*-------------------------------------------------------------------------*/

/*
 * DESCRIPTORS ... most are static, but strings and (full) configuration
 * descriptors are built on demand.  Also the (static) config and interface
 * descriptors are adjusted during fsg_bind().
 */
#define STRING_MANUFACTURER	1
#define STRING_PRODUCT		2
#define STRING_SERIAL		3

/* There is only one configuration. */
#define	CONFIG_VALUE		1

static struct usb_device_descriptor
device_desc = {
	.bLength =		sizeof device_desc,
	.bDescriptorType =	USB_DT_DEVICE,

	.bcdUSB =		__constant_cpu_to_le16(0x0200),
	.bDeviceClass =		USB_CLASS_PER_INTERFACE,

	/* The next three values can be overridden by module parameters */
	.idVendor =		__constant_cpu_to_le16(DRIVER_VENDOR_ID),
	.idProduct =		__constant_cpu_to_le16(DRIVER_PRODUCT_ID),
	.bcdDevice =		__constant_cpu_to_le16(0xffff),

	.iManufacturer =	STRING_MANUFACTURER,
	.iProduct =		STRING_PRODUCT,
	.iSerialNumber =	STRING_SERIAL,
	.bNumConfigurations =	1,
};

static struct usb_config_descriptor
config_desc = {
	.bLength =		sizeof config_desc,
	.bDescriptorType =	USB_DT_CONFIG,

	/* wTotalLength computed by usb_gadget_config_buf() */
	.bNumInterfaces =	1,
	.bConfigurationValue =	CONFIG_VALUE,
	.bmAttributes =		USB_CONFIG_ATT_ONE,
	.bMaxPower =		250,	//draw up to 500 mA
};

static struct usb_otg_descriptor
otg_desc = {
	.bLength =		sizeof(otg_desc),
	.bDescriptorType =	USB_DT_OTG,

	.bmAttributes =		USB_OTG_SRP,
};

/* There is only one interface. */

static struct usb_interface_descriptor
intf_desc = {
	.bLength =		sizeof intf_desc,
	.bDescriptorType =	USB_DT_INTERFACE,

	.bNumEndpoints =	2,		// Adjusted during fsg_bind()
	.bInterfaceClass =	USB_CLASS_MASS_STORAGE,
	.bInterfaceSubClass =	USB_SC_SCSI,	// Adjusted during fsg_bind()
	.bInterfaceProtocol =	USB_PR_BULK,	// Adjusted during fsg_bind()
};

/* Three full-speed endpoint descriptors: bulk-in, bulk-out,
 * and interrupt-in. */

static struct usb_endpoint_descriptor
fs_bulk_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	/* wMaxPacketSize set by autoconfiguration */
};

static struct usb_endpoint_descriptor
fs_bulk_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	/* wMaxPacketSize set by autoconfiguration */
};

static struct usb_endpoint_descriptor
fs_intr_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	__constant_cpu_to_le16(2),
	.bInterval =		32,	// frames -> 32 ms
};

static const struct usb_descriptor_header *fs_function[] = {
	(struct usb_descriptor_header *) &otg_desc,
	(struct usb_descriptor_header *) &intf_desc,
	(struct usb_descriptor_header *) &fs_bulk_in_desc,
	(struct usb_descriptor_header *) &fs_bulk_out_desc,
	(struct usb_descriptor_header *) &fs_intr_in_desc,
	NULL,
};
#define FS_FUNCTION_PRE_EP_ENTRIES	2


#ifdef	CONFIG_USB_GADGET_DUALSPEED

/*
 * USB 2.0 devices need to expose both high speed and full speed
 * descriptors, unless they only run at full speed.
 *
 * That means alternate endpoint descriptors (bigger packets)
 * and a "device qualifier" ... plus more construction options
 * for the config descriptor.
 */
static struct usb_qualifier_descriptor
dev_qualifier = {
	.bLength =		sizeof dev_qualifier,
	.bDescriptorType =	USB_DT_DEVICE_QUALIFIER,

	.bcdUSB =		__constant_cpu_to_le16(0x0200),
	.bDeviceClass =		USB_CLASS_PER_INTERFACE,

	.bNumConfigurations =	1,
};

static struct usb_endpoint_descriptor
hs_bulk_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	/* bEndpointAddress copied from fs_bulk_in_desc during fsg_bind() */
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor
hs_bulk_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	/* bEndpointAddress copied from fs_bulk_out_desc during fsg_bind() */
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
	.bInterval =		1,	// NAK every 1 uframe
};

static struct usb_endpoint_descriptor
hs_intr_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	/* bEndpointAddress copied from fs_intr_in_desc during fsg_bind() */
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	__constant_cpu_to_le16(2),
	.bInterval =		9,	// 2**(9-1) = 256 uframes -> 32 ms
};

static const struct usb_descriptor_header *hs_function[] = {
	(struct usb_descriptor_header *) &otg_desc,
	(struct usb_descriptor_header *) &intf_desc,
	(struct usb_descriptor_header *) &hs_bulk_in_desc,
	(struct usb_descriptor_header *) &hs_bulk_out_desc,
	(struct usb_descriptor_header *) &hs_intr_in_desc,
	NULL,
};
#define HS_FUNCTION_PRE_EP_ENTRIES	2

/* Maxpacket and other transfer characteristics vary by speed. */
#define ep_desc(g,fs,hs)	(((g)->speed==USB_SPEED_HIGH) ? (hs) : (fs))

#else

/* If there's no high speed support, always use the full-speed descriptor. */
#define ep_desc(g,fs,hs)	fs

#endif	/* !CONFIG_USB_GADGET_DUALSPEED */


/* The CBI specification limits the serial string to 12 uppercase hexadecimal
 * characters. */
static char	manufacturer_name[20+1] = "ARCHOS";
static char	product_name[20+1];
static char	serial_string[13+1];

/* Static strings, in UTF-8 (for simplicity we use only ASCII characters) */
static struct usb_string		strings[] = {
	{STRING_MANUFACTURER,	manufacturer_name},
	{STRING_PRODUCT,	product_name},
	{STRING_SERIAL,		serial_string},
	{}
};

static struct usb_gadget_strings	stringtab = {
	.language	= 0x0409,		// en-us
	.strings	= strings,
};


/*
 * Config descriptors must agree with the code that sets configurations
 * and with code managing interfaces and their altsettings.  They must
 * also handle different speeds and other-speed requests.
 */
static int populate_config_buf(struct usb_gadget *gadget,
		u8 *buf, u8 type, unsigned index)
{
#ifdef CONFIG_USB_GADGET_DUALSPEED
	enum usb_device_speed			speed = gadget->speed;
#endif
	int					len;
	const struct usb_descriptor_header	**function;

	if (index > 0)
		return -EINVAL;

#ifdef CONFIG_USB_GADGET_DUALSPEED
	if (type == USB_DT_OTHER_SPEED_CONFIG)
		speed = (USB_SPEED_FULL + USB_SPEED_HIGH) - speed;
	if (speed == USB_SPEED_HIGH)
		function = hs_function;
	else
#endif
		function = fs_function;

	/* for now, don't advertise srp-only devices */
	if (!gadget->is_otg)
		function++;

	len = usb_gadget_config_buf(&config_desc, buf, EP0_BUFSIZE, function);
	((struct usb_config_descriptor *) buf)->bDescriptorType = type;
	return len;
}


/*-------------------------------------------------------------------------*/

/* These routines may be called in process context or in_irq */

static void wakeup_thread(struct fsg_dev *fsg)
{
	/* Tell the main thread that something has happened */
	fsg->thread_wakeup_needed = 1;
	if (fsg->thread_task)
		wake_up_process(fsg->thread_task);
}

static void raise_exception(struct fsg_dev *fsg, enum fsg_state new_state)
{
	unsigned long		flags;
	struct task_struct	*thread_task;

	/* Do nothing if a higher-priority exception is already in progress.
	 * If a lower-or-equal priority exception is in progress, preempt it
	 * and notify the main thread by sending it a signal. */
	spin_lock_irqsave(&fsg->lock, flags);
	if (fsg->state <= new_state) {
		fsg->exception_req_tag = fsg->ep0_req_tag;
		fsg->state = new_state;
		thread_task = fsg->thread_task;
		if (thread_task)
			send_sig_info(SIGUSR1, SEND_SIG_FORCED, thread_task);
	}
	spin_unlock_irqrestore(&fsg->lock, flags);
}


/*-------------------------------------------------------------------------*/

/* The disconnect callback and ep0 routines.  These always run in_irq,
 * except that ep0_queue() is called in the main thread to acknowledge
 * completion of various requests: set config, set interface, and
 * Bulk-only device reset. */

static void fsg_disconnect(struct usb_gadget *gadget)
{
	struct fsg_dev		*fsg = get_gadget_data(gadget);

	DBG(fsg, "disconnect or port reset\n");
	raise_exception(fsg, FSG_STATE_DISCONNECT);
}


static int ep0_queue(struct fsg_dev *fsg)
{
	int	rc;

	rc = usb_ep_queue(fsg->ep0, fsg->ep0req, GFP_ATOMIC);
	if (rc != 0 && rc != -ESHUTDOWN) {

		/* We can't do much more than wait for a reset */
		WARN(fsg, "error in submission: %s --> %d\n",
				fsg->ep0->name, rc);
	}
	return rc;
}

static void ep0_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct fsg_dev		*fsg = (struct fsg_dev *) ep->driver_data;

	if (req->actual > 0)
		dump_msg(fsg, fsg->ep0req_name, req->buf, req->actual);
	if (req->status || req->actual != req->length)
		DBG(fsg, "%s --> %d, %u/%u\n", __FUNCTION__,
				req->status, req->actual, req->length);
	if (req->status == -ECONNRESET)		// Request was cancelled
		usb_ep_fifo_flush(ep);

	if (req->status == 0 && req->context)
		((fsg_routine_t) (req->context))(fsg);
}


/*-------------------------------------------------------------------------*/

/* Bulk and interrupt endpoint completion handlers.
 * These always run in_irq. */

static void bulk_in_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct fsg_dev		*fsg = (struct fsg_dev *) ep->driver_data;
	struct fsg_buffhd	*bh = (struct fsg_buffhd *) req->context;

	if (req->status || req->actual != req->length)
		DBG(fsg, "%s --> %d, %u/%u\n", __FUNCTION__,
				req->status, req->actual, req->length);
	if (req->status == -ECONNRESET)		// Request was cancelled
		usb_ep_fifo_flush(ep);

	/* Hold the lock while we update the request and buffer states */
	spin_lock(&fsg->lock);
	bh->inreq_busy = 0;
	bh->state = BUF_STATE_EMPTY;
	wakeup_thread(fsg);
	spin_unlock(&fsg->lock);
}

static void bulk_out_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct fsg_dev		*fsg = (struct fsg_dev *) ep->driver_data;
	struct fsg_buffhd	*bh = (struct fsg_buffhd *) req->context;

	dump_msg(fsg, "bulk-out", req->buf, req->actual);
	if (req->status || req->actual != bh->bulk_out_intended_length)
		DBG(fsg, "%s --> %d, %u/%u\n", __FUNCTION__,
				req->status, req->actual,
				bh->bulk_out_intended_length);
	if (req->status == -ECONNRESET)		// Request was cancelled
		usb_ep_fifo_flush(ep);

	/* Hold the lock while we update the request and buffer states */
	spin_lock(&fsg->lock);
	bh->outreq_busy = 0;
	bh->state = BUF_STATE_FULL;
	wakeup_thread(fsg);
	spin_unlock(&fsg->lock);
}


#ifdef CONFIG_USB_FILE_STORAGE_TEST
static void intr_in_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct fsg_dev		*fsg = (struct fsg_dev *) ep->driver_data;
	struct fsg_buffhd	*bh = (struct fsg_buffhd *) req->context;

	if (req->status || req->actual != req->length)
		DBG(fsg, "%s --> %d, %u/%u\n", __FUNCTION__,
				req->status, req->actual, req->length);
	if (req->status == -ECONNRESET)		// Request was cancelled
		usb_ep_fifo_flush(ep);

	/* Hold the lock while we update the request and buffer states */
	spin_lock(&fsg->lock);
	fsg->intreq_busy = 0;
	bh->state = BUF_STATE_EMPTY;
	wakeup_thread(fsg);
	spin_unlock(&fsg->lock);
}

#else
static void intr_in_complete(struct usb_ep *ep, struct usb_request *req)
{}
#endif /* CONFIG_USB_FILE_STORAGE_TEST */


/*-------------------------------------------------------------------------*/

/* Ep0 class-specific handlers.  These always run in_irq. */

#ifdef CONFIG_USB_FILE_STORAGE_TEST
static void received_cbi_adsc(struct fsg_dev *fsg, struct fsg_buffhd *bh)
{
	struct usb_request	*req = fsg->ep0req;
	static u8		cbi_reset_cmnd[6] = {
			SC_SEND_DIAGNOSTIC, 4, 0xff, 0xff, 0xff, 0xff};

	/* Error in command transfer? */
	if (req->status || req->length != req->actual ||
			req->actual < 6 || req->actual > MAX_COMMAND_SIZE) {

		/* Not all controllers allow a protocol stall after
		 * receiving control-out data, but we'll try anyway. */
		fsg_set_halt(fsg, fsg->ep0);
		return;			// Wait for reset
	}

	/* Is it the special reset command? */
	if (req->actual >= sizeof cbi_reset_cmnd &&
			memcmp(req->buf, cbi_reset_cmnd,
				sizeof cbi_reset_cmnd) == 0) {

		/* Raise an exception to stop the current operation
		 * and reinitialize our state. */
		DBG(fsg, "cbi reset request\n");
		raise_exception(fsg, FSG_STATE_RESET);
		return;
	}

	VDBG(fsg, "CB[I] accept device-specific command\n");
	spin_lock(&fsg->lock);

	/* Save the command for later */
	if (fsg->cbbuf_cmnd_size)
		WARN(fsg, "CB[I] overwriting previous command\n");
	fsg->cbbuf_cmnd_size = req->actual;
	memcpy(fsg->cbbuf_cmnd, req->buf, fsg->cbbuf_cmnd_size);

	wakeup_thread(fsg);
	spin_unlock(&fsg->lock);
}

#else
static void received_cbi_adsc(struct fsg_dev *fsg, struct fsg_buffhd *bh)
{}
#endif /* CONFIG_USB_FILE_STORAGE_TEST */


static int class_setup_req(struct fsg_dev *fsg,
		const struct usb_ctrlrequest *ctrl)
{
	struct usb_request	*req = fsg->ep0req;
	int			value = -EOPNOTSUPP;
	u16			wIndex = le16_to_cpu(ctrl->wIndex);
	u16			wLength = le16_to_cpu(ctrl->wLength);

	if (!fsg->config)
		return value;

	/* Handle Bulk-only class-specific requests */
	if (transport_is_bbb()) {
		switch (ctrl->bRequest) {

		case USB_BULK_RESET_REQUEST:
			if (ctrl->bRequestType != (USB_DIR_OUT |
					USB_TYPE_CLASS | USB_RECIP_INTERFACE))
				break;
			if (wIndex != 0) {
				value = -EDOM;
				break;
			}

			/* Raise an exception to stop the current operation
			 * and reinitialize our state. */
			DBG(fsg, "bulk reset request\n");
			raise_exception(fsg, FSG_STATE_RESET);
			value = DELAYED_STATUS;
			break;

		case USB_BULK_GET_MAX_LUN_REQUEST:
			if (ctrl->bRequestType != (USB_DIR_IN |
					USB_TYPE_CLASS | USB_RECIP_INTERFACE))
				break;
			if (wIndex != 0) {
				value = -EDOM;
				break;
			}
			VDBG(fsg, "get max LUN\n");
			*(u8 *) req->buf = fsg->nluns - 1;
			value = min(wLength, (u16) 1);
			break;
		}
	}

	/* Handle CBI class-specific requests */
	else {
		switch (ctrl->bRequest) {

		case USB_CBI_ADSC_REQUEST:
			if (ctrl->bRequestType != (USB_DIR_OUT |
					USB_TYPE_CLASS | USB_RECIP_INTERFACE))
				break;
			if (wIndex != 0) {
				value = -EDOM;
				break;
			}
			if (wLength > MAX_COMMAND_SIZE) {
				value = -EOVERFLOW;
				break;
			}
			value = wLength;
			fsg->ep0req->context = received_cbi_adsc;
			break;
		}
	}

	if (value == -EOPNOTSUPP)
		VDBG(fsg,
			"unknown class-specific control req "
			"%02x.%02x i%04x l%u\n",
			ctrl->bRequestType, ctrl->bRequest,
			wIndex, wLength);
	return value;
}


/*-------------------------------------------------------------------------*/

/* Ep0 standard request handlers.  These always run in_irq. */

static int standard_setup_req(struct fsg_dev *fsg,
		const struct usb_ctrlrequest *ctrl)
{
	struct usb_request	*req = fsg->ep0req;
	int			value = -EOPNOTSUPP;
	u16			wIndex = le16_to_cpu(ctrl->wIndex);
	u16			wValue = le16_to_cpu(ctrl->wValue);
	u16			wLength = le16_to_cpu(ctrl->wLength);

	/* Usually this just stores reply data in the pre-allocated ep0 buffer,
	 * but config change events will also reconfigure hardware. */
	switch (ctrl->bRequest) {

	case USB_REQ_GET_DESCRIPTOR:
		if (ctrl->bRequestType != (USB_DIR_IN | USB_TYPE_STANDARD |
				USB_RECIP_DEVICE))
			break;
		switch (wValue >> 8) {

		case USB_DT_DEVICE:
			VDBG(fsg, "get device descriptor\n");
			value = min(wLength, (u16) sizeof device_desc);
			memcpy(req->buf, &device_desc, value);
			break;
#ifdef CONFIG_USB_GADGET_DUALSPEED
		case USB_DT_DEVICE_QUALIFIER:
			VDBG(fsg, "get device qualifier\n");
			if (!fsg->gadget->is_dualspeed)
				break;
			value = min(wLength, (u16) sizeof dev_qualifier);
			memcpy(req->buf, &dev_qualifier, value);
			break;

		case USB_DT_OTHER_SPEED_CONFIG:
			VDBG(fsg, "get other-speed config descriptor\n");
			if (!fsg->gadget->is_dualspeed)
				break;
			goto get_config;
#endif
		case USB_DT_CONFIG:
			VDBG(fsg, "get configuration descriptor\n");
#ifdef CONFIG_USB_GADGET_DUALSPEED
		get_config:
#endif
			value = populate_config_buf(fsg->gadget,
					req->buf,
					wValue >> 8,
					wValue & 0xff);
			if (value >= 0)
				value = min(wLength, (u16) value);
			break;

		case USB_DT_STRING:
			VDBG(fsg, "get string descriptor\n");

			/* wIndex == language code */
			value = usb_gadget_get_string(&stringtab,
					wValue & 0xff, req->buf);
			if (value >= 0)
				value = min(wLength, (u16) value);
			break;
		}
		break;

	/* One config, two speeds */
	case USB_REQ_SET_CONFIGURATION:
		if (ctrl->bRequestType != (USB_DIR_OUT | USB_TYPE_STANDARD |
				USB_RECIP_DEVICE))
			break;
		VDBG(fsg, "set configuration\n");
		if (wValue == CONFIG_VALUE || wValue == 0) {
			fsg->new_config = wValue;

			/* Raise an exception to wipe out previous transaction
			 * state (queued bufs, etc) and set the new config. */
			raise_exception(fsg, FSG_STATE_CONFIG_CHANGE);
			value = DELAYED_STATUS;
		}
		break;
	case USB_REQ_GET_CONFIGURATION:
		if (ctrl->bRequestType != (USB_DIR_IN | USB_TYPE_STANDARD |
				USB_RECIP_DEVICE))
			break;
		VDBG(fsg, "get configuration\n");
		*(u8 *) req->buf = fsg->config;
		value = min(wLength, (u16) 1);
		break;

	case USB_REQ_SET_INTERFACE:
		if (ctrl->bRequestType != (USB_DIR_OUT| USB_TYPE_STANDARD |
				USB_RECIP_INTERFACE))
			break;
		if (fsg->config && wIndex == 0) {

			/* Raise an exception to wipe out previous transaction
			 * state (queued bufs, etc) and install the new
			 * interface altsetting. */
			raise_exception(fsg, FSG_STATE_INTERFACE_CHANGE);
			value = DELAYED_STATUS;
		}
		break;
	case USB_REQ_GET_INTERFACE:
		if (ctrl->bRequestType != (USB_DIR_IN | USB_TYPE_STANDARD |
				USB_RECIP_INTERFACE))
			break;
		if (!fsg->config)
			break;
		if (wIndex != 0) {
			value = -EDOM;
			break;
		}
		VDBG(fsg, "get interface\n");
		*(u8 *) req->buf = 0;
		value = min(wLength, (u16) 1);
		break;

	default:
		VDBG(fsg,
			"unknown control req %02x.%02x v%04x i%04x l%u\n",
			ctrl->bRequestType, ctrl->bRequest,
			wValue, wIndex, wLength);
	}

	return value;
}


static int fsg_setup(struct usb_gadget *gadget,
		const struct usb_ctrlrequest *ctrl)
{
	struct fsg_dev		*fsg = get_gadget_data(gadget);
	int			rc;
	u16			wLength = le16_to_cpu(ctrl->wLength);

	++fsg->ep0_req_tag;		// Record arrival of a new request
	fsg->ep0req->context = NULL;
	fsg->ep0req->length = 0;
	dump_msg(fsg, "ep0-setup", (u8 *) ctrl, sizeof(*ctrl));

	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_CLASS)
		rc = class_setup_req(fsg, ctrl);
	else
		rc = standard_setup_req(fsg, ctrl);

	/* Respond with data/status or defer until later? */
	if (rc >= 0 && rc != DELAYED_STATUS) {
		fsg->ep0req->length = rc;
		fsg->ep0req->zero = (rc < wLength &&
				(rc % gadget->ep0->maxpacket) == 0);
		fsg->ep0req_name = (ctrl->bRequestType & USB_DIR_IN ?
				"ep0-in" : "ep0-out");
		rc = ep0_queue(fsg);
	}

	/* Device either stalls (rc < 0) or reports success */
	return rc;
}


/*-------------------------------------------------------------------------*/

/* All the following routines run in process context */


/* Use this for bulk or interrupt transfers, not ep0 */
static void start_transfer(struct fsg_dev *fsg, struct usb_ep *ep,
		struct usb_request *req, volatile int *pbusy,
		volatile enum fsg_buffer_state *state)
{
	int	rc;

	if (ep == fsg->bulk_in)
		dump_msg(fsg, "bulk-in", req->buf, req->length);
	else if (ep == fsg->intr_in)
		dump_msg(fsg, "intr-in", req->buf, req->length);

	spin_lock_irq(&fsg->lock);
	*pbusy = 1;
	*state = BUF_STATE_BUSY;
	spin_unlock_irq(&fsg->lock);
	rc = usb_ep_queue(ep, req, GFP_KERNEL);
	if (rc != 0) {
		*pbusy = 0;
		*state = BUF_STATE_EMPTY;

		/* We can't do much more than wait for a reset */

		/* Note: currently the net2280 driver fails zero-length
		 * submissions if DMA is enabled. */
		if (rc != -ESHUTDOWN && !(rc == -EOPNOTSUPP &&
						req->length == 0))
			WARN(fsg, "error in submission: %s --> %d\n",
					ep->name, rc);
	}
}


static int sleep_thread(struct fsg_dev *fsg)
{
	int	rc = 0;

	/* Wait until a signal arrives or we are woken up */
	for (;;) {
		try_to_freeze();
		set_current_state(TASK_INTERRUPTIBLE);
		if (signal_pending(current)) {
			rc = -EINTR;
			break;
		}
		if (fsg->thread_wakeup_needed)
			break;
		schedule();
	}
	__set_current_state(TASK_RUNNING);
	fsg->thread_wakeup_needed = 0;
	return rc;
}


/*-------------------------------------------------------------------------*/
static int msc_bio_read_req_complete(struct bio *bio, unsigned int bytes_done, int error)
{
	struct fsg_buffhd *bh = (struct fsg_buffhd *)bio->bi_private;

	atomic_dec(&bh->currentlun->pending_bio_count); 
   
	bh->amount = bytes_done;
	if(bh->ppos)
	{
		*(bh->ppos) += bytes_done;
	}
	complete(&bh->complete);    
   
	bio_put(bio);
	wakeup_thread(bh->fsg);
     
	MDBG("BIO READ REQ COMPLETE: Bytes Transferred = %u, error = %d, pending_bio_count is %d\n", 
		bytes_done, error, atomic_read(&bh->currentlun->pending_bio_count));
        
	return 1;     
}

static int msc_bio_write_req_complete(struct bio *bio, unsigned int bytes_done, int error)
{
	struct fsg_buffhd *bh = (struct fsg_buffhd *)bio->bi_private;

	atomic_dec(&bh->currentlun->pending_bio_count); 
	atomic_dec(&bh->currentlun->pending_write_count);
   
	bh->amount = bytes_done;
	if(bh->ppos)
	{
		*(bh->ppos) += bytes_done;
	}
    
	bh->state = BUF_STATE_EMPTY;
   
	bio_put(bio);
	wakeup_thread(bh->fsg);
     
	MDBG("BIO WRITE REQ COMPLETE: Bytes Transferred = %u, error = %d, pending_bio_count is %d\n", 
		bytes_done, error, atomic_read(&bh->currentlun->pending_bio_count));
        
	return 1;     
}

#if 0
static int timeval_diff(struct timeval *start, struct timeval *end)
{
	return (end->tv_sec - start->tv_sec) * 1000000 +
			(end->tv_usec - start->tv_usec);
}
#endif

static ssize_t wait_for_bio_completion(struct fsg_buffhd *bh)
{ 
	wait_for_completion(&bh->complete);

	if(atomic_read(&bh->currentlun->pending_bio_count) > 0) 
		printk("Possible error. Completion signalled immediately \n");
     
	return (ssize_t)(bh->amount);        
}        

static ssize_t msc_device_req(unsigned int rw,                                
                              struct fsg_buffhd *bh,
                              int wait)
{
	/* This function shall construct the bio structure for the pages in the buffer
	** and submit it to the block layer for IO request generation */
	struct file *filp = bh->fsg->curlun->filp;
	struct page *pPage = bh->pDataPage[0];
	size_t len = bh->amount;
	loff_t *ppos = bh->ppos;
	struct bio *bio;
	struct block_device *bdev = filp->f_mapping->host->i_bdev;
	unsigned int nPages = 0;
	unsigned int remaining = 0;
	int i;
	ssize_t nrBytes = 0;   

	/* prepare a structure for requests that want to wait for completion */
	if(wait)
	{
		init_completion(&bh->complete);
	}

 	/* calculate the number of bio submissions */
 	nPages = len/PAGE_SIZE;
 	remaining = len%PAGE_SIZE;
 	if(remaining)
     		nPages++;    
         
 	/* allocate memory for bio */
 	bio = bio_alloc(GFP_NOIO, nPages);
 	if(bio == NULL)
		return nrBytes;       
 
	/* fill in the physical and number of hardware segments */       
	bio->bi_sector = (sector_t)((*ppos) >> 9);
	bio->bi_bdev = bdev;

	/* free the reference to the position pointer if we aren't going to wait */
	if(!wait)
	{
		bh->ppos = NULL;
	}

	/* fill in the bio vectors */
	for(i = 0; i < nPages; i++)
	{
		/* we need to run through the list of biovecs here */
		bio->bi_io_vec[i].bv_page = (pPage + i);
		if((i==nPages) && (remaining))
			bio->bi_io_vec[i].bv_len = remaining;
		else
			bio->bi_io_vec[i].bv_len = PAGE_SIZE;        
		bio->bi_io_vec[i].bv_offset = 0;  		
	}
               
	bio->bi_vcnt = nPages;
	bio->bi_idx = 0;
	bio->bi_hw_segments = 1;
	bio->bi_phys_segments = 1;     
	bio->bi_size = (unsigned int)len;      
	bio->bi_end_io = (rw == MSC_WRITE) ? msc_bio_write_req_complete : msc_bio_read_req_complete;
	bio->bi_private = (void *)bh;	
        
	bio_get(bio);
	atomic_inc(&bh->currentlun->pending_bio_count);
     
	if(rw == MSC_READ)
	{
		submit_bio(READ,bio);		
	}  
	else if(rw == MSC_WRITE) 
	{
		atomic_inc(&bh->currentlun->pending_write_count);
		submit_bio(WRITE,bio);		
	}
             
	if (bio_flagged(bio, BIO_EOPNOTSUPP))
	{	        
		return nrBytes;
	}     
	bio_put(bio);

	if(wait)
	{    	
		nrBytes = wait_for_bio_completion(bh);    
	}
	else
	{
		nrBytes = len;    
	}
     
	return nrBytes;
} 

static int do_read(struct fsg_dev *fsg)
{
	struct lun		*curlun = fsg->curlun;
	u32			lba;
	struct fsg_buffhd	*bh;
	int			rc;
	u32			amount_left;
	loff_t			file_offset, file_offset_tmp;
	unsigned int		amount;
#if 0
	unsigned int		partial_page;
#endif
	ssize_t			nread;
	int			diff;

	/* Get the starting Logical Block Address and check that it's
	 * not too big */
	if (fsg->cmnd[0] == SC_READ_6)
		lba = (fsg->cmnd[1] << 16) | get_be16(&fsg->cmnd[2]);
	else {
		lba = get_be32(&fsg->cmnd[2]);

		/* We allow DPO (Disable Page Out = don't save data in the
		 * cache) and FUA (Force Unit Access = don't read from the
		 * cache), but we don't implement them. */
		if ((fsg->cmnd[1] & ~0x18) != 0) {
			curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
			return -EINVAL;
		}
	}
	if (lba >= curlun->num_sectors) {
		curlun->sense_data = SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
		return -EINVAL;
	}
	file_offset = ((loff_t) lba) << 9;

	/* Carry out the file reads */
	amount_left = fsg->data_size_from_cmnd;

//	ERROR(fsg, "do_read read %10u @ %10llu\n", amount_left,(unsigned long long) file_offset);

	if (unlikely(amount_left == 0))
		return -EIO;		// No default reply

	for (;;) {

		/* Figure out how much we need to read:
		 * Try to read the remaining amount.
		 * But don't read more than the buffer size.
		 * And don't try to read past the end of the file.
		 * Finally, if we're not at a page boundary, don't read past
		 *	the next page.
		 * If this means reading 0 then we were asked to read past
		 *	the end of file. */
		amount = min((unsigned int) amount_left, mod_data.buflen);
		amount = min((loff_t) amount,
				curlun->file_length - file_offset);
#if 0				
		partial_page = file_offset & (PAGE_CACHE_SIZE - 1);
		if (partial_page > 0)
			amount = min(amount, (unsigned int) PAGE_CACHE_SIZE -
					partial_page);
#endif					

		/* Wait for the next buffer to become available */
		bh = fsg->next_buffhd_to_fill;
		while (bh->state != BUF_STATE_EMPTY) {
			if ((rc = sleep_thread(fsg)) != 0)
				return rc;
		}

		/* set the lun associated with this buffer */
		bh->currentlun = curlun;
		fsg->ahead_bh.currentlun = curlun;

		/* If we were asked to read past the end of file,
		 * end with an empty buffer. */		 
		if (amount == 0) {
			curlun->sense_data =
					SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
			curlun->sense_data_info = file_offset >> 9;
			bh->inreq->length = 0;
			bh->state = BUF_STATE_FULL;
			break;
		}		

		/* Perform the read only after committing all the writes*/
		while(atomic_read(&bh->currentlun->pending_write_count) > 0)
		{
			VLDBG(curlun, "Waiting on pending writes %d", 
				atomic_read(&bh->currentlun->pending_write_count));
			schedule();
		}
		
		/* perform the read from the ahead buf - synchronous call*/
		file_offset_tmp = file_offset;

		if( !fsg->ahead_valid
		|| file_offset < fsg->ahead_start
		|| file_offset >= fsg->ahead_start + fsg->ahead_len ){
			int to_read = min((loff_t)mod_data.buflen, curlun->file_length - file_offset);
			int bytes;

// 			ERROR(fsg, "hdd     read %10u @ %10llu\n", to_read,(unsigned long long) file_offset);

			fsg->ahead_bh.ppos 	= &file_offset_tmp;
			fsg->ahead_bh.amount 	= to_read;

			bytes = msc_device_req(MSC_READ,&fsg->ahead_bh,1);

			if (bytes < 0) {
				ERROR(fsg, "error in file read: %d\n", (int)bytes);
				bytes = 0;
			}

			/* If an error occurred, report it and its position */
			if (bytes < to_read) {
				curlun->sense_data = SS_UNRECOVERED_READ_ERROR;
				curlun->sense_data_info = file_offset >> 9;
				break;
			}

			fsg->ahead_len   = bytes;
			fsg->ahead_start = file_offset;
			fsg->ahead_valid = 1;
		}

		diff = file_offset - fsg->ahead_start;
		nread = min((int)amount, fsg->ahead_len - diff);

// 		ERROR(fsg, "file    read %10u @ %10llu -> %10d (avail %6d diff %6d)\n", amount,
// 				(unsigned long long) file_offset, nread, fsg->ahead_len, diff);

		if (signal_pending(current))
			return -EINTR;

		file_offset  += nread;
		amount_left  -= nread;
		fsg->residue -= nread;

		/* simulate filling the real bh */
		bh->ppos = &file_offset_tmp;
		bh->amount = amount;

		memcpy(bh->buf, ((char*)fsg->ahead_bh.buf) + diff, nread);

		*bh->ppos  = file_offset_tmp;
		bh->amount = nread;

		bh->inreq->length = nread;
		bh->state = BUF_STATE_FULL;

		if (amount_left == 0)
			break;		// No more left to read

		/* Send this buffer and go read some more */
		bh->inreq->zero = 0;
		start_transfer(fsg, fsg->bulk_in, bh->inreq,
				&bh->inreq_busy, &bh->state);
		fsg->next_buffhd_to_fill = bh->next;
	}

	return -EIO;		// No default reply
}

/*-------------------------------------------------------------------------*/

/**
 * clear gadgetfs_info buffer
 *
 */
static void clear_gadgetfs_info(void)
{
  // INFO(the_fsg, "clearing buffer.\n");
  gadgetfs_info.count=0;
  memzero(gadgetfs_info.data, NB_USB_GADGETFS_INFO*sizeof(struct usb_gadgetfs_info_write_t));
}

/**
 * add an usb_gadgetfs_info_write_t to gadgetfs_info buffer
 *
 * @lun: LUN number
 * @lba: disk sector number
 * @count: number of sectors to write
 *
 * returns 1 if add was performed, 0 if buffer full
 */
static int add_gadgetfs_info(unsigned int lun, unsigned int lba, unsigned int amount)
{
  unsigned int size = gadgetfs_info.count;
  if( size < NB_USB_GADGETFS_INFO ){
// INFO(the_fsg, "adding <%u@%.8x#%u> in buffer\n", amount, lba, lun );
    gadgetfs_info.data[size].lun = lun;
    gadgetfs_info.data[size].lba = lba;
    gadgetfs_info.data[size].amount = amount;
    ++gadgetfs_info.count;
    return 1;
  }
  return 0;
}

/**
 * manage incoming user connexions
 */
static void netlink_input(struct sock *sk, int len)
{
  wake_up_interruptible(sk->sk_sleep);
}

/**
 * send netlink info from buffer
 */
static int send_netlink(int nonblocking)
{
  struct sk_buff * skb=alloc_skb(NLMSG_SPACE(sizeof(struct usb_gadgetfs_info_t)),GFP_KERNEL);
  struct nlmsghdr * nlh;
  struct usb_gadgetfs_info_t * data;

  if( skb==NULL ){
    ERROR(the_fsg, "unable to allocate netlink socket buffer!\n");
    return -ENOBUFS;
  }

  // fill in socket buffer
  nlh = (struct nlmsghdr *)skb_put(skb, NLMSG_SPACE(sizeof(struct usb_gadgetfs_info_t)));
  if( !nlh ){
    ERROR(the_fsg, "%s: NULL data socket buffer.\n", __FUNCTION__);
    return -ENOBUFS;
  }

  nlh->nlmsg_len = NLMSG_SPACE(sizeof(struct usb_gadgetfs_info_t));
  nlh->nlmsg_pid = 0;
  nlh->nlmsg_flags = 0;

  data = (struct usb_gadgetfs_info_t *)NLMSG_DATA(nlh);
  if( !data ){
    ERROR(the_fsg, "%s: no data in socket buffer.\n", __FUNCTION__);
    return -ENOBUFS;
  }

  memcpy(data, &gadgetfs_info, sizeof(struct usb_gadgetfs_info_t) );

  NETLINK_CB(skb).dst_group = 0;
  NETLINK_CB(skb).pid = 0;

  return netlink_unicast(gadgetfs_sock, skb, mod_data.netlink, nonblocking);
}

/*-------------------------------------------------------------------------*/
static int do_write(struct fsg_dev *fsg)
{
	struct lun		*curlun = fsg->curlun;
	u32			lba;
	struct fsg_buffhd	*bh;
	int			get_some_more;
	u32			amount_left_to_req, amount_left_to_write;
	loff_t			usb_offset, file_offset, file_offset_tmp;
	unsigned int		amount;
#if 0
	unsigned int		partial_page;
#endif
	ssize_t			nwritten;
	int			rc;

	if (curlun->ro) {
		curlun->sense_data = SS_WRITE_PROTECTED;
		return -EINVAL;
	}
	curlun->filp->f_flags &= ~O_SYNC;	// Default is not to wait

	/* force rereading the drive again */
	fsg->ahead_valid = 0;

	/* Get the starting Logical Block Address and check that it's
	 * not too big */
	if (fsg->cmnd[0] == SC_WRITE_6)
		lba = (fsg->cmnd[1] << 16) | get_be16(&fsg->cmnd[2]);
	else {
		lba = get_be32(&fsg->cmnd[2]);

		/* We allow DPO (Disable Page Out = don't save data in the
		 * cache) and FUA (Force Unit Access = write directly to the
		 * medium).  We don't implement DPO; we implement FUA by
		 * performing synchronous output. */
		if ((fsg->cmnd[1] & ~0x18) != 0) {
			curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
			return -EINVAL;
		}
		if (fsg->cmnd[1] & 0x08)	// FUA
			curlun->filp->f_flags |= O_SYNC;
	}
	if (lba >= curlun->num_sectors) {
		curlun->sense_data = SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
		return -EINVAL;
	}

	/* Carry out the file writes */
	get_some_more = 1;
	file_offset = usb_offset = ((loff_t) lba) << 9;
	amount_left_to_req = amount_left_to_write = fsg->data_size_from_cmnd;


	while (amount_left_to_write > 0) {

		/* Queue a request for more data from the host */
		bh = fsg->next_buffhd_to_fill;
		if (bh->state == BUF_STATE_EMPTY && get_some_more) {

			/* set the lun associated with this buffer */
			bh->currentlun = curlun;
            
			/* Figure out how much we want to get:
			 * Try to get the remaining amount.
			 * But don't get more than the buffer size.
			 * And don't try to go past the end of the file.
			 * If we're not at a page boundary,
			 *	don't go past the next page.
			 * If this means getting 0, then we were asked
			 *	to write past the end of file.
			 * Finally, round down to a block boundary. */
			amount = min(amount_left_to_req, mod_data.buflen);
			amount = min((loff_t) amount, curlun->file_length -
					usb_offset);
#if 0					
			partial_page = usb_offset & (PAGE_CACHE_SIZE - 1);
			if (partial_page > 0)
				amount = min(amount,
	(unsigned int) PAGE_CACHE_SIZE - partial_page);
#endif	

			if (amount == 0) {
				get_some_more = 0;
				curlun->sense_data =
					SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
				curlun->sense_data_info = usb_offset >> 9;
				continue;
			}
			amount -= (amount & 511);
			if (amount == 0) {

				/* Why were we were asked to transfer a
				 * partial block? */
				get_some_more = 0;
				continue;
			}

			/* Get the next buffer */
			usb_offset += amount;
			fsg->usb_amount_left -= amount;
			amount_left_to_req -= amount;
			if (amount_left_to_req == 0)
				get_some_more = 0;

			/* amount is always divisible by 512, hence by
			 * the bulk-out maxpacket size */
			bh->outreq->length = bh->bulk_out_intended_length = amount;
			bh->outreq->short_not_ok = 1;
			start_transfer(fsg, fsg->bulk_out, bh->outreq,
					&bh->outreq_busy, &bh->state);
			fsg->next_buffhd_to_fill = bh->next;
			continue;
		}


		/* Write the received data to the backing file */
		bh = fsg->next_buffhd_to_drain;
		if (bh->state == BUF_STATE_EMPTY && !get_some_more)
			break;			// We stopped early
		if (bh->state == BUF_STATE_FULL) {
			fsg->next_buffhd_to_drain = bh->next;		

			bh->state = BUF_STATE_EMPTY;

			/* Did something go wrong with the transfer? */
			if (bh->outreq->status != 0) {
				curlun->sense_data = SS_COMMUNICATION_FAILURE;
				curlun->sense_data_info = file_offset >> 9;
				break;
			}

			amount = bh->outreq->actual;
			if (curlun->file_length - file_offset < amount) {
				LERROR(curlun,
					"write %u @ %llu beyond end %llu\n",
					amount, (unsigned long long) file_offset,
					(unsigned long long) curlun->file_length);
				amount = curlun->file_length - file_offset;
			}

			/* Perform the write */
			file_offset_tmp = file_offset;
			
			/* Perform the write */
			bh->ppos = &file_offset_tmp;
			bh->amount = amount;
			bh->state = BUF_STATE_DIRTY;
			
            		nwritten = msc_device_req(MSC_WRITE, bh, 0);
			VLDBG(curlun, "file write %u @ %llu -> %d\n", amount,
					(unsigned long long) file_offset,
					(int) nwritten);		
				
			if (signal_pending(current))
				return -EINTR;		// Interrupted!

			if (nwritten < 0) {
				LDBG(curlun, "error in file write: %d\n",
						(int) nwritten);
				nwritten = 0;
			} else if (nwritten < amount) {
				LDBG(curlun, "partial file write: %d/%u\n",
						(int) nwritten, amount);
				nwritten -= (nwritten & 511);
                                                // Round down to a block
                        }

                        if( mod_data.netlink ){
                                // add data to buffer
				if( add_gadgetfs_info(fsg->lun, (unsigned int)(file_offset>>9), (unsigned int)(nwritten>>9)) == 0 ){
					int status=0;
					// buffer is full, we try to send data blocking until it is read
					INFO(fsg, "buffer full, sending will be blocking.\n");
					// blocking
					do {
						// INFO(the_fsg, "try sending...\n");
					} while( (status=send_netlink(0)) == 0 );
					if( status < 0 ){
						ERROR( fsg, "could not send. Buffered data is thrown away.\n");
					}
					clear_gadgetfs_info();

					// now add the new data in fresh buffer
					add_gadgetfs_info(fsg->lun, (unsigned int)(file_offset>>9), (unsigned int)(nwritten>>9));
				}
			}

			file_offset += nwritten;
			amount_left_to_write -= nwritten;
			fsg->residue -= nwritten;

			/* If an error occurred, report it and its position */
			if (nwritten < amount) {
				curlun->sense_data = SS_WRITE_ERROR;
				curlun->sense_data_info = file_offset >> 9;
				break;
			}

			/* Did the host decide to stop early? */
			if (bh->outreq->actual != bh->outreq->length) {
				fsg->short_packet_received = 1;
				break;
			}
			continue;
		}

		/* Wait for something to happen */
		if ((rc = sleep_thread(fsg)) != 0)
			return rc;
	}

	return -EIO;		// No default reply
}


/*-------------------------------------------------------------------------*/

/* Sync the file data, don't bother with the metadata.
 * This code was copied from fs/buffer.c:sys_fdatasync(). */
static int fsync_sub(struct lun *curlun)
{
#if 0       
	struct file	*filp = curlun->filp;
	struct inode	*inode;
	int		rc, err;

	if (curlun->ro || !filp)
		return 0;
	if (!filp->f_op->fsync)
		return -EINVAL;

	inode = filp->f_dentry->d_inode;
	down(&inode->i_sem);
	current->flags |= PF_SYNCWRITE;
	rc = filemap_fdatawrite(inode->i_mapping);
	err = filp->f_op->fsync(filp, filp->f_dentry, 1);
	if (!rc)
		rc = err;
	err = filemap_fdatawait(inode->i_mapping);
	if (!rc)
		rc = err;
	current->flags &= ~PF_SYNCWRITE;
	up(&inode->i_sem);
	VLDBG(curlun, "fdatasync -> %d\n", rc);
	return rc;
#endif

	while(atomic_read(&curlun->pending_write_count) > 0)
	{
		VLDBG(curlun, "Waiting on pending writes %d", 
			atomic_read(&curlun->pending_write_count));
		schedule();
	}
	
	return 0;
}

static void fsync_all(struct fsg_dev *fsg)
{
	int	i;

	for (i = 0; i < fsg->nluns; ++i)
		fsync_sub(&fsg->luns[i]);
}

static int do_synchronize_cache(struct fsg_dev *fsg)
{
	struct lun	*curlun = fsg->curlun;
	int		rc;

	/* We ignore the requested LBA and write out all file's
	 * dirty data buffers. */
	rc = fsync_sub(curlun);
	if (rc)
		curlun->sense_data = SS_WRITE_ERROR;
	return 0;
}


/*-------------------------------------------------------------------------*/
#if 0
static void invalidate_sub(struct lun *curlun)
{
	struct file	*filp = curlun->filp;
	struct inode	*inode = filp->f_dentry->d_inode;
	unsigned long	rc;

	rc = invalidate_inode_pages(inode->i_mapping);
	VLDBG(curlun, "invalidate_inode_pages -> %ld\n", rc);
}
#endif

static int do_verify(struct fsg_dev *fsg)
{
	struct lun		*curlun = fsg->curlun;
	u32			lba;
	u32			verification_length;
	struct fsg_buffhd	*bh = fsg->next_buffhd_to_fill;
	loff_t			file_offset, file_offset_tmp;
	u32			amount_left;
	unsigned int		amount;
	ssize_t			nread;

	/* Get the starting Logical Block Address and check that it's
	 * not too big */
	lba = get_be32(&fsg->cmnd[2]);
	if (lba >= curlun->num_sectors) {
		curlun->sense_data = SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
		return -EINVAL;
	}

	/* We allow DPO (Disable Page Out = don't save data in the
	 * cache) but we don't implement it. */
	if ((fsg->cmnd[1] & ~0x10) != 0) {
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}

	verification_length = get_be16(&fsg->cmnd[7]);
	if (unlikely(verification_length == 0))
		return -EIO;		// No default reply

	/* Prepare to carry out the file verify */
	amount_left = verification_length << 9;
	file_offset = ((loff_t) lba) << 9;

	/* Write out all the dirty buffers before invalidating them */
	fsync_sub(curlun);
	if (signal_pending(current))
		return -EINTR;

#if 0
	invalidate_sub(curlun);
	if (signal_pending(current))
		return -EINTR;
#endif		

	/* Just try to read the requested blocks */
	while (amount_left > 0) {
		/* set the lun associated with this buffer */
		bh->currentlun = curlun;

		/* Figure out how much we need to read:
		 * Try to read the remaining amount, but not more than
		 * the buffer size.
		 * And don't try to read past the end of the file.
		 * If this means reading 0 then we were asked to read
		 * past the end of file. */
		amount = min((unsigned int) amount_left, mod_data.buflen);
		amount = min((loff_t) amount,
				curlun->file_length - file_offset);
		if (amount == 0) {
			curlun->sense_data =
					SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE;
			curlun->sense_data_info = file_offset >> 9;
			break;
		}

		/* Perform the read */
		file_offset_tmp = file_offset;
		bh->ppos = &file_offset_tmp;
		bh->amount = amount;
		
		nread = msc_device_req(MSC_READ,bh,1);		
		VLDBG(curlun, "file read %u @ %llu -> %d\n", amount,
				(unsigned long long) file_offset,
				(int) nread);
				
		if (signal_pending(current))
			return -EINTR;

		if (nread < 0) {
			LDBG(curlun, "error in file verify: %d\n",
					(int) nread);
			nread = 0;
		} else if (nread < amount) {
			LDBG(curlun, "partial file verify: %d/%u\n",
					(int) nread, amount);
			nread -= (nread & 511);	// Round down to a sector
		}
		if (nread == 0) {
			curlun->sense_data = SS_UNRECOVERED_READ_ERROR;
			curlun->sense_data_info = file_offset >> 9;
			break;
		}
		file_offset += nread;
		amount_left -= nread;
	}
	return 0;
}


/*-------------------------------------------------------------------------*/

static int do_inquiry(struct fsg_dev *fsg, struct fsg_buffhd *bh)
{
	u8	*buf = (u8 *) bh->buf;

	static char vendor_id[] = "Archos  ";
	static char product_id[] = "PC Hard Drive   ";

	if (!fsg->curlun) {		// Unsupported LUNs are okay
		fsg->bad_lun_okay = 1;
		memset(buf, 0, 36);
		buf[0] = 0x7f;		// Unsupported, no device-type
		return 36;
	}

	memset(buf, 0, 8);	// Non-removable, direct-access device
	if (mod_data.removable)
		buf[1] = 0x80;
	buf[2] = 2;		// ANSI SCSI level 2
	buf[3] = 2;		// SCSI-2 INQUIRY data format
	buf[4] = 31;		// Additional length
				// No special options
	sprintf(buf + 8, "%-8s%-16s%04x", vendor_id, product_id,
			mod_data.release);
	return 36;
}


static int do_request_sense(struct fsg_dev *fsg, struct fsg_buffhd *bh)
{
	struct lun	*curlun = fsg->curlun;
	u8		*buf = (u8 *) bh->buf;
	u32		sd, sdinfo;

	/*
	 * From the SCSI-2 spec., section 7.9 (Unit attention condition):
	 *
	 * If a REQUEST SENSE command is received from an initiator
	 * with a pending unit attention condition (before the target
	 * generates the contingent allegiance condition), then the
	 * target shall either:
	 *   a) report any pending sense data and preserve the unit
	 *	attention condition on the logical unit, or,
	 *   b) report the unit attention condition, may discard any
	 *	pending sense data, and clear the unit attention
	 *	condition on the logical unit for that initiator.
	 *
	 * FSG normally uses option a); enable this code to use option b).
	 */
#if 0
	if (curlun && curlun->unit_attention_data != SS_NO_SENSE) {
		curlun->sense_data = curlun->unit_attention_data;
		curlun->unit_attention_data = SS_NO_SENSE;
	}
#endif

	if (!curlun) {		// Unsupported LUNs are okay
		fsg->bad_lun_okay = 1;
		sd = SS_LOGICAL_UNIT_NOT_SUPPORTED;
		sdinfo = 0;
	} else {
		sd = curlun->sense_data;
		sdinfo = curlun->sense_data_info;
		curlun->sense_data = SS_NO_SENSE;
		curlun->sense_data_info = 0;
	}

	memset(buf, 0, 18);
	buf[0] = 0x80 | 0x70;			// Valid, current error
	buf[2] = SK(sd);
	put_be32(&buf[3], sdinfo);		// Sense information
	buf[7] = 18 - 8;			// Additional sense length
	buf[12] = ASC(sd);
	buf[13] = ASCQ(sd);
	return 18;
}


static int do_read_capacity(struct fsg_dev *fsg, struct fsg_buffhd *bh)
{
	struct lun	*curlun = fsg->curlun;
	u32		lba = get_be32(&fsg->cmnd[2]);
	int		pmi = fsg->cmnd[8];
	u8		*buf = (u8 *) bh->buf;

	/* Check the PMI and LBA fields */
	if (pmi > 1 || (pmi == 0 && lba != 0)) {
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}

	put_be32(&buf[0], curlun->num_sectors - 1);	// Max logical block
	put_be32(&buf[4], 512);				// Block length
	return 8;
}


static int do_mode_sense(struct fsg_dev *fsg, struct fsg_buffhd *bh)
{
	struct lun	*curlun = fsg->curlun;
	int		mscmnd = fsg->cmnd[0];
	u8		*buf = (u8 *) bh->buf;
	u8		*buf0 = buf;
	int		pc, page_code;
	int		changeable_values, all_pages;
	int		valid_page = 0;
	int		len, limit;

	if ((fsg->cmnd[1] & ~0x08) != 0) {		// Mask away DBD
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}
	pc = fsg->cmnd[2] >> 6;
	page_code = fsg->cmnd[2] & 0x3f;
	if (pc == 3) {
		curlun->sense_data = SS_SAVING_PARAMETERS_NOT_SUPPORTED;
		return -EINVAL;
	}
	changeable_values = (pc == 1);
	all_pages = (page_code == 0x3f);

	/* Write the mode parameter header.  Fixed values are: default
	 * medium type, no cache control (DPOFUA), and no block descriptors.
	 * The only variable value is the WriteProtect bit.  We will fill in
	 * the mode data length later. */
	memset(buf, 0, 8);
	if (mscmnd == SC_MODE_SENSE_6) {
		buf[2] = (curlun->ro ? 0x80 : 0x00);		// WP, DPOFUA
		buf += 4;
		limit = 255;
	} else {			// SC_MODE_SENSE_10
		buf[3] = (curlun->ro ? 0x80 : 0x00);		// WP, DPOFUA
		buf += 8;
		limit = 65535;		// Should really be mod_data.buflen
	}

	/* No block descriptors */

	/* The mode pages, in numerical order.  The only page we support
	 * is the Caching page. */
	if (page_code == 0x08 || all_pages) {
		valid_page = 1;
		buf[0] = 0x08;		// Page code
		buf[1] = 10;		// Page length
		memset(buf+2, 0, 10);	// None of the fields are changeable

		if (!changeable_values) {
			buf[2] = 0x04;	// Write cache enable,
					// Read cache not disabled
					// No cache retention priorities
			put_be16(&buf[4], 0xffff);  // Don't disable prefetch
					// Minimum prefetch = 0
			put_be16(&buf[8], 0xffff);  // Maximum prefetch
			put_be16(&buf[10], 0xffff); // Maximum prefetch ceiling
		}
		buf += 12;
	}

	/* Check that a valid page was requested and the mode data length
	 * isn't too long. */
	len = buf - buf0;
	if (!valid_page || len > limit) {
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}

	/*  Store the mode data length */
	if (mscmnd == SC_MODE_SENSE_6)
		buf0[0] = len - 1;
	else
		put_be16(buf0, len - 2);
	return len;
}


static int do_start_stop(struct fsg_dev *fsg)
{
	struct lun	*curlun = fsg->curlun;
	int		loej, start;

	if (!mod_data.removable) {
		curlun->sense_data = SS_INVALID_COMMAND;
		return -EINVAL;
	}

	// int immed = fsg->cmnd[1] & 0x01;
	loej = fsg->cmnd[4] & 0x02;
	start = fsg->cmnd[4] & 0x01;

	/* MB (15/12/2006): the code below ensures that all further TEST_UNIT_READY
	   will fail. This makes MacOSX happy. Without this, you get an error message
	   when trying to remove your unit safely */
	if ((fsg->cmnd[1] & ~0x01) != 0 ||		// Mask away Immed
			(fsg->cmnd[4] & ~0x03) != 0) {	// Mask LoEj, Start
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}

	if (!start) {

		/* Are we allowed to unload the media? */
		if (curlun->prevent_medium_removal) {
			LDBG(curlun, "unload attempt prevented\n");
			curlun->sense_data = SS_MEDIUM_REMOVAL_PREVENTED;
			return -EINVAL;
		}
		if (loej) {		// Simulate an unload/eject
			up_read(&fsg->filesem);
			down_write(&fsg->filesem);
			close_backing_file(curlun);
			up_write(&fsg->filesem);
			down_read(&fsg->filesem);
		}
	} else {

		/* Our emulation doesn't support mounting; the medium is
		 * available for use as soon as it is loaded. */
		if (!backing_file_is_open(curlun)) {
			curlun->sense_data = SS_MEDIUM_NOT_PRESENT;
			return -EINVAL;
		}
	}
	return 0;
}


static int do_prevent_allow(struct fsg_dev *fsg)
{
	struct lun	*curlun = fsg->curlun;
	int		prevent;

	if (!mod_data.removable) {
		curlun->sense_data = SS_INVALID_COMMAND;
		return -EINVAL;
	}

	prevent = fsg->cmnd[4] & 0x01;
	if ((fsg->cmnd[4] & ~0x01) != 0) {		// Mask away Prevent
		curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
		return -EINVAL;
	}

	if (curlun->prevent_medium_removal && !prevent)
		fsync_sub(curlun);
	curlun->prevent_medium_removal = prevent;
	return 0;
}


static int do_read_format_capacities(struct fsg_dev *fsg,
			struct fsg_buffhd *bh)
{
	struct lun	*curlun = fsg->curlun;
	u8		*buf = (u8 *) bh->buf;

	buf[0] = buf[1] = buf[2] = 0;
	buf[3] = 8;		// Only the Current/Maximum Capacity Descriptor
	buf += 4;

	put_be32(&buf[0], curlun->num_sectors);		// Number of blocks
	put_be32(&buf[4], 512);				// Block length
	buf[4] = 0x02;					// Current capacity
	return 12;
}


static int do_mode_select(struct fsg_dev *fsg, struct fsg_buffhd *bh)
{
	struct lun	*curlun = fsg->curlun;

	/* We don't support MODE SELECT */
	curlun->sense_data = SS_INVALID_COMMAND;
	return -EINVAL;
}


/*-------------------------------------------------------------------------*/

static int halt_bulk_in_endpoint(struct fsg_dev *fsg)
{
	int	rc;

	rc = fsg_set_halt(fsg, fsg->bulk_in);
	if (rc == -EAGAIN)
		VDBG(fsg, "delayed bulk-in endpoint halt\n");
	while (rc != 0) {
		if (rc != -EAGAIN) {
			WARN(fsg, "usb_ep_set_halt -> %d\n", rc);
			rc = 0;
			break;
		}

		/* Wait for a short time and then try again */
		if (msleep_interruptible(100) != 0)
			return -EINTR;
		rc = usb_ep_set_halt(fsg->bulk_in);
	}
	return rc;
}

static int pad_with_zeros(struct fsg_dev *fsg)
{
	struct fsg_buffhd	*bh = fsg->next_buffhd_to_fill;
	u32			nkeep = bh->inreq->length;
	u32			nsend;
	int			rc;

	bh->state = BUF_STATE_EMPTY;		// For the first iteration
	fsg->usb_amount_left = nkeep + fsg->residue;
	while (fsg->usb_amount_left > 0) {

		/* Wait for the next buffer to be free */
		while (bh->state != BUF_STATE_EMPTY) {
			if ((rc = sleep_thread(fsg)) != 0)
				return rc;
		}

		nsend = min(fsg->usb_amount_left, (u32) mod_data.buflen);
		memset(bh->buf + nkeep, 0, nsend - nkeep);
		bh->inreq->length = nsend;
		bh->inreq->zero = 0;
		start_transfer(fsg, fsg->bulk_in, bh->inreq,
				&bh->inreq_busy, &bh->state);
		bh = fsg->next_buffhd_to_fill = bh->next;
		fsg->usb_amount_left -= nsend;
		nkeep = 0;
	}
	return 0;
}

static int throw_away_data(struct fsg_dev *fsg)
{
	struct fsg_buffhd	*bh;
	u32			amount;
	int			rc;

	while ((bh = fsg->next_buffhd_to_drain)->state != BUF_STATE_EMPTY ||
			fsg->usb_amount_left > 0) {

		/* Throw away the data in a filled buffer */
		if (bh->state == BUF_STATE_FULL) {
			bh->state = BUF_STATE_EMPTY;
			fsg->next_buffhd_to_drain = bh->next;

			/* A short packet or an error ends everything */
			if (bh->outreq->actual != bh->outreq->length ||
					bh->outreq->status != 0) {
				raise_exception(fsg, FSG_STATE_ABORT_BULK_OUT);
				return -EINTR;
			}
			continue;
		}

		/* Try to submit another request if we need one */
		bh = fsg->next_buffhd_to_fill;
		if (bh->state == BUF_STATE_EMPTY && fsg->usb_amount_left > 0) {
			amount = min(fsg->usb_amount_left,
					(u32) mod_data.buflen);

			/* amount is always divisible by 512, hence by
			 * the bulk-out maxpacket size */
			bh->outreq->length = bh->bulk_out_intended_length = amount;
			bh->outreq->short_not_ok = 1;
			start_transfer(fsg, fsg->bulk_out, bh->outreq,
					&bh->outreq_busy, &bh->state);
			fsg->next_buffhd_to_fill = bh->next;
			fsg->usb_amount_left -= amount;
			continue;
		}

		/* Otherwise wait for something to happen */
		if ((rc = sleep_thread(fsg)) != 0)
			return rc;
	}
	return 0;
}


static int finish_reply(struct fsg_dev *fsg)
{
	struct fsg_buffhd	*bh = fsg->next_buffhd_to_fill;
	int			rc = 0;

	switch (fsg->data_dir) {
	case DATA_DIR_NONE:
		break;			// Nothing to send

	/* If we don't know whether the host wants to read or write,
	 * this must be CB or CBI with an unknown command.  We mustn't
	 * try to send or receive any data.  So stall both bulk pipes
	 * if we can and wait for a reset. */
	case DATA_DIR_UNKNOWN:
		if (mod_data.can_stall) {
			fsg_set_halt(fsg, fsg->bulk_out);
			rc = halt_bulk_in_endpoint(fsg);
		}
		break;

	/* All but the last buffer of data must have already been sent */
	case DATA_DIR_TO_HOST:
		if (fsg->data_size == 0)
			;		// Nothing to send

		/* If there's no residue, simply send the last buffer */
		else if (fsg->residue == 0) {
			bh->inreq->zero = 0;
			start_transfer(fsg, fsg->bulk_in, bh->inreq,
					&bh->inreq_busy, &bh->state);
			fsg->next_buffhd_to_fill = bh->next;
		}

		/* There is a residue.  For CB and CBI, simply mark the end
		 * of the data with a short packet.  However, if we are
		 * allowed to stall, there was no data at all (residue ==
		 * data_size), and the command failed (invalid LUN or
		 * sense data is set), then halt the bulk-in endpoint
		 * instead. */
		else if (!transport_is_bbb()) {
			if (mod_data.can_stall &&
					fsg->residue == fsg->data_size &&
	(!fsg->curlun || fsg->curlun->sense_data != SS_NO_SENSE)) {
				bh->state = BUF_STATE_EMPTY;
				rc = halt_bulk_in_endpoint(fsg);
			} else {
				bh->inreq->zero = 1;
				start_transfer(fsg, fsg->bulk_in, bh->inreq,
						&bh->inreq_busy, &bh->state);
				fsg->next_buffhd_to_fill = bh->next;
			}
		}

		/* For Bulk-only, if we're allowed to stall then send the
		 * short packet and halt the bulk-in endpoint.  If we can't
		 * stall, pad out the remaining data with 0's. */
		else {
			if (mod_data.can_stall) {
				bh->inreq->zero = 1;
				start_transfer(fsg, fsg->bulk_in, bh->inreq,
						&bh->inreq_busy, &bh->state);
				fsg->next_buffhd_to_fill = bh->next;
				rc = halt_bulk_in_endpoint(fsg);
			} else
				rc = pad_with_zeros(fsg);
		}
		break;

	/* We have processed all we want from the data the host has sent.
	 * There may still be outstanding bulk-out requests. */
	case DATA_DIR_FROM_HOST:
		if (fsg->residue == 0)
			;		// Nothing to receive

		/* Did the host stop sending unexpectedly early? */
		else if (fsg->short_packet_received) {
			raise_exception(fsg, FSG_STATE_ABORT_BULK_OUT);
			rc = -EINTR;
		}

		/* We haven't processed all the incoming data.  Even though
		 * we may be allowed to stall, doing so would cause a race.
		 * The controller may already have ACK'ed all the remaining
		 * bulk-out packets, in which case the host wouldn't see a
		 * STALL.  Not realizing the endpoint was halted, it wouldn't
		 * clear the halt -- leading to problems later on. */
#if 0
		else if (mod_data.can_stall) {
			fsg_set_halt(fsg, fsg->bulk_out);
			raise_exception(fsg, FSG_STATE_ABORT_BULK_OUT);
			rc = -EINTR;
		}
#endif

		/* We can't stall.  Read in the excess data and throw it
		 * all away. */
		else
			rc = throw_away_data(fsg);
		break;
	}
	return rc;
}


static int send_status(struct fsg_dev *fsg)
{
	struct lun		*curlun = fsg->curlun;
	struct fsg_buffhd	*bh;
	int			rc;
	u8			status = USB_STATUS_PASS;
	u32			sd, sdinfo = 0;

	/* Wait for the next buffer to become available */
	bh = fsg->next_buffhd_to_fill;
	while (bh->state != BUF_STATE_EMPTY) {
		if ((rc = sleep_thread(fsg)) != 0)
			return rc;
	}

	if (curlun) {
		sd = curlun->sense_data;
		sdinfo = curlun->sense_data_info;
	} else if (fsg->bad_lun_okay)
		sd = SS_NO_SENSE;
	else
		sd = SS_LOGICAL_UNIT_NOT_SUPPORTED;

	if (fsg->phase_error) {
		DBG(fsg, "sending phase-error status\n");
		status = USB_STATUS_PHASE_ERROR;
		sd = SS_INVALID_COMMAND;
	} else if (sd != SS_NO_SENSE) {
		DBG(fsg, "sending command-failure status\n");
		status = USB_STATUS_FAIL;
		VDBG(fsg, "  sense data: SK x%02x, ASC x%02x, ASCQ x%02x;"
				"  info x%x\n",
				SK(sd), ASC(sd), ASCQ(sd), sdinfo);
	}

	if (transport_is_bbb()) {
		struct bulk_cs_wrap	*csw = (struct bulk_cs_wrap *) bh->buf;

		/* Store and send the Bulk-only CSW */
		csw->Signature = __constant_cpu_to_le32(USB_BULK_CS_SIG);
		csw->Tag = fsg->tag;
		csw->Residue = cpu_to_le32(fsg->residue);
		csw->Status = status;

		bh->inreq->length = USB_BULK_CS_WRAP_LEN;
		bh->inreq->zero = 0;
		start_transfer(fsg, fsg->bulk_in, bh->inreq,
				&bh->inreq_busy, &bh->state);

	} else if (mod_data.transport_type == USB_PR_CB) {

		/* Control-Bulk transport has no status phase! */
		return 0;

	} else {			// USB_PR_CBI
		struct interrupt_data	*buf = (struct interrupt_data *)
						bh->buf;

		/* Store and send the Interrupt data.  UFI sends the ASC
		 * and ASCQ bytes.  Everything else sends a Type (which
		 * is always 0) and the status Value. */
		if (mod_data.protocol_type == USB_SC_UFI) {
			buf->bType = ASC(sd);
			buf->bValue = ASCQ(sd);
		} else {
			buf->bType = 0;
			buf->bValue = status;
		}
		fsg->intreq->length = CBI_INTERRUPT_DATA_LEN;

		fsg->intr_buffhd = bh;		// Point to the right buffhd
		fsg->intreq->buf = bh->inreq->buf;
		fsg->intreq->dma = bh->inreq->dma;
		fsg->intreq->context = bh;
		start_transfer(fsg, fsg->intr_in, fsg->intreq,
				&fsg->intreq_busy, &bh->state);
	}

	fsg->next_buffhd_to_fill = bh->next;
	return 0;
}


/*-------------------------------------------------------------------------*/

/* Check whether the command is properly formed and whether its data size
 * and direction agree with the values we already have. */
static int check_command(struct fsg_dev *fsg, int cmnd_size,
		enum data_direction data_dir, unsigned int mask,
		int needs_medium, const char *name)
{
	int			i;
	int			lun = fsg->cmnd[1] >> 5;
	static const char	dirletter[4] = {'u', 'o', 'i', 'n'};
	char			hdlen[20];
	struct lun		*curlun;

	/* Adjust the expected cmnd_size for protocol encapsulation padding.
	 * Transparent SCSI doesn't pad. */
	if (protocol_is_scsi())
		;

	/* There's some disagreement as to whether RBC pads commands or not.
	 * We'll play it safe and accept either form. */
	else if (mod_data.protocol_type == USB_SC_RBC) {
		if (fsg->cmnd_size == 12)
			cmnd_size = 12;

	/* All the other protocols pad to 12 bytes */
	} else
		cmnd_size = 12;

	hdlen[0] = 0;
	if (fsg->data_dir != DATA_DIR_UNKNOWN)
		sprintf(hdlen, ", H%c=%u", dirletter[(int) fsg->data_dir],
				fsg->data_size);
	VDBG(fsg, "SCSI command: %s;  Dc=%d, D%c=%u;  Hc=%d%s\n",
			name, cmnd_size, dirletter[(int) data_dir],
			fsg->data_size_from_cmnd, fsg->cmnd_size, hdlen);

	/* We can't reply at all until we know the correct data direction
	 * and size. */
	if (fsg->data_size_from_cmnd == 0)
		data_dir = DATA_DIR_NONE;
	if (fsg->data_dir == DATA_DIR_UNKNOWN) {	// CB or CBI
		fsg->data_dir = data_dir;
		fsg->data_size = fsg->data_size_from_cmnd;

	} else {					// Bulk-only
		if (fsg->data_size < fsg->data_size_from_cmnd) {

			/* Host data size < Device data size is a phase error.
			 * Carry out the command, but only transfer as much
			 * as we are allowed. */
			fsg->data_size_from_cmnd = fsg->data_size;
			fsg->phase_error = 1;
		}
	}
	fsg->residue = fsg->usb_amount_left = fsg->data_size;

	/* Conflicting data directions is a phase error */
	if (fsg->data_dir != data_dir && fsg->data_size_from_cmnd > 0) {
		fsg->phase_error = 1;
		return -EINVAL;
	}

	/* Verify the length of the command itself */
	if (cmnd_size != fsg->cmnd_size) {

		/* Special case workaround*/
		/* MS-Windows, some Bios and the Xbox issue commands with 
		cbw->Length differing from what transparent scsi should be. */
		WARN(fsg, "command size is %d instead of %d\n", fsg->cmnd_size, cmnd_size);
		cmnd_size = fsg->cmnd_size;
	}

	/* Check that the LUN values are oonsistent */
	if (transport_is_bbb()) {
		if (fsg->lun != lun)
			DBG(fsg, "using LUN %d from CBW, "
					"not LUN %d from CDB\n",
					fsg->lun, lun);
	} else
		fsg->lun = lun;		// Use LUN from the command

	/* Check the LUN */
	if (fsg->lun >= 0 && fsg->lun < fsg->nluns) {
		fsg->curlun = curlun = &fsg->luns[fsg->lun];
		if (fsg->cmnd[0] != SC_REQUEST_SENSE) {
			curlun->sense_data = SS_NO_SENSE;
			curlun->sense_data_info = 0;
		}
	} else {
		fsg->curlun = curlun = NULL;
		fsg->bad_lun_okay = 0;

		/* INQUIRY and REQUEST SENSE commands are explicitly allowed
		 * to use unsupported LUNs; all others may not. */
		if (fsg->cmnd[0] != SC_INQUIRY &&
				fsg->cmnd[0] != SC_REQUEST_SENSE) {
			DBG(fsg, "unsupported LUN %d\n", fsg->lun);
			return -EINVAL;
		}
	}

	/* If a unit attention condition exists, only INQUIRY and
	 * REQUEST SENSE commands are allowed; anything else must fail. */
	if (curlun && curlun->unit_attention_data != SS_NO_SENSE &&
			fsg->cmnd[0] != SC_INQUIRY &&
			fsg->cmnd[0] != SC_REQUEST_SENSE) {
		curlun->sense_data = curlun->unit_attention_data;
		curlun->unit_attention_data = SS_NO_SENSE;
		return -EINVAL;
	}

	/* Check that only command bytes listed in the mask are non-zero */
	fsg->cmnd[1] &= 0x1f;			// Mask away the LUN
	for (i = 1; i < cmnd_size; ++i) {
		if (fsg->cmnd[i] && !(mask & (1 << i))) {
			if (curlun)
				curlun->sense_data = SS_INVALID_FIELD_IN_CDB;
			return -EINVAL;
		}
	}

	/* If the medium isn't mounted and the command needs to access
	 * it, return an error. */
	if (curlun && !backing_file_is_open(curlun) && needs_medium) {
		curlun->sense_data = SS_MEDIUM_NOT_PRESENT;
		return -EINVAL;
	}

	return 0;
}


static int do_scsi_command(struct fsg_dev *fsg)
{
	struct fsg_buffhd	*bh;
	int			rc;
	int			reply = -EINVAL;
	int			i;
	static char		unknown[16];

	dump_cdb(fsg);

	/* Wait for the next buffer to become available for data or status */
	bh = fsg->next_buffhd_to_drain = fsg->next_buffhd_to_fill;
	while (bh->state != BUF_STATE_EMPTY) {
		if ((rc = sleep_thread(fsg)) != 0)
			return rc;
		}
	fsg->phase_error = 0;
	fsg->short_packet_received = 0;

	down_read(&fsg->filesem);	// We're using the backing file
	switch (fsg->cmnd[0]) {

	case SC_INQUIRY:
		fsg->data_size_from_cmnd = fsg->cmnd[4];
		if ((reply = check_command(fsg, 6, DATA_DIR_TO_HOST,
				(1<<4), 0,
				"INQUIRY")) == 0)
			reply = do_inquiry(fsg, bh);
		break;

	case SC_MODE_SELECT_6:
		fsg->data_size_from_cmnd = fsg->cmnd[4];
		if ((reply = check_command(fsg, 6, DATA_DIR_FROM_HOST,
				(1<<1) | (1<<4), 0,
				"MODE SELECT(6)")) == 0)
			reply = do_mode_select(fsg, bh);
		break;

	case SC_MODE_SELECT_10:
		fsg->data_size_from_cmnd = get_be16(&fsg->cmnd[7]);
		if ((reply = check_command(fsg, 10, DATA_DIR_FROM_HOST,
				(1<<1) | (3<<7), 0,
				"MODE SELECT(10)")) == 0)
			reply = do_mode_select(fsg, bh);
		break;

	case SC_MODE_SENSE_6:
		fsg->data_size_from_cmnd = fsg->cmnd[4];
		if ((reply = check_command(fsg, 6, DATA_DIR_TO_HOST,
				(1<<1) | (1<<2) | (1<<4), 0,
				"MODE SENSE(6)")) == 0)
			reply = do_mode_sense(fsg, bh);
		break;

	case SC_MODE_SENSE_10:
		fsg->data_size_from_cmnd = get_be16(&fsg->cmnd[7]);
		if ((reply = check_command(fsg, 10, DATA_DIR_TO_HOST,
				(1<<1) | (1<<2) | (3<<7), 0,
				"MODE SENSE(10)")) == 0)
			reply = do_mode_sense(fsg, bh);
		break;

	case SC_PREVENT_ALLOW_MEDIUM_REMOVAL:
		fsg->data_size_from_cmnd = 0;
		if ((reply = check_command(fsg, 6, DATA_DIR_NONE,
				(1<<4), 0,
				"PREVENT-ALLOW MEDIUM REMOVAL")) == 0)
			reply = do_prevent_allow(fsg);
		break;

	case SC_READ_6:
		i = fsg->cmnd[4];
		fsg->data_size_from_cmnd = (i == 0 ? 256 : i) << 9;
		if ((reply = check_command(fsg, 6, DATA_DIR_TO_HOST,
				(7<<1) | (1<<4), 1,
				"READ(6)")) == 0)
			reply = do_read(fsg);
		break;

	case SC_READ_10:
		fsg->data_size_from_cmnd = get_be16(&fsg->cmnd[7]) << 9;
		if ((reply = check_command(fsg, 10, DATA_DIR_TO_HOST,
				(1<<1) | (0xf<<2) | (3<<7), 1,
				"READ(10)")) == 0)
			reply = do_read(fsg);
		break;

	case SC_READ_12:
		fsg->data_size_from_cmnd = get_be32(&fsg->cmnd[6]) << 9;
		if ((reply = check_command(fsg, 12, DATA_DIR_TO_HOST,
				(1<<1) | (0xf<<2) | (0xf<<6), 1,
				"READ(12)")) == 0)
			reply = do_read(fsg);
		break;

	case SC_READ_CAPACITY:
		fsg->data_size_from_cmnd = 8;
		if ((reply = check_command(fsg, 10, DATA_DIR_TO_HOST,
				(0xf<<2) | (1<<8), 1,
				"READ CAPACITY")) == 0)
			reply = do_read_capacity(fsg, bh);
		break;

	case SC_READ_FORMAT_CAPACITIES:
		fsg->data_size_from_cmnd = get_be16(&fsg->cmnd[7]);
		if ((reply = check_command(fsg, 10, DATA_DIR_TO_HOST,
				(3<<7), 1,
				"READ FORMAT CAPACITIES")) == 0)
			reply = do_read_format_capacities(fsg, bh);
		break;

	case SC_REQUEST_SENSE:
		fsg->data_size_from_cmnd = fsg->cmnd[4];
		if ((reply = check_command(fsg, 6, DATA_DIR_TO_HOST,
				(1<<4), 0,
				"REQUEST SENSE")) == 0)
			reply = do_request_sense(fsg, bh);
		break;

	case SC_START_STOP_UNIT:
		fsg->data_size_from_cmnd = 0;
		if ((reply = check_command(fsg, 6, DATA_DIR_NONE,
				(1<<1) | (1<<4), 0,
				"START-STOP UNIT")) == 0)
			reply = do_start_stop(fsg);
		break;

	case SC_SYNCHRONIZE_CACHE:
		fsg->data_size_from_cmnd = 0;
		if ((reply = check_command(fsg, 10, DATA_DIR_NONE,
				(0xf<<2) | (3<<7), 1,
				"SYNCHRONIZE CACHE")) == 0)
			reply = do_synchronize_cache(fsg);
		break;

	case SC_TEST_UNIT_READY:
		fsg->data_size_from_cmnd = 0;
		reply = check_command(fsg, 6, DATA_DIR_NONE,
				0, 1,
				"TEST UNIT READY");
		break;

	/* Although optional, this command is used by MS-Windows.  We
	 * support a minimal version: BytChk must be 0. */
	case SC_VERIFY:
		fsg->data_size_from_cmnd = 0;
		if ((reply = check_command(fsg, 10, DATA_DIR_NONE,
				(1<<1) | (0xf<<2) | (3<<7), 1,
				"VERIFY")) == 0)
			reply = do_verify(fsg);
		break;

	case SC_WRITE_6:
		i = fsg->cmnd[4];
		fsg->data_size_from_cmnd = (i == 0 ? 256 : i) << 9;
		if ((reply = check_command(fsg, 6, DATA_DIR_FROM_HOST,
				(7<<1) | (1<<4), 1,
				"WRITE(6)")) == 0)
			reply = do_write(fsg);
		break;

	case SC_WRITE_10:
		fsg->data_size_from_cmnd = get_be16(&fsg->cmnd[7]) << 9;
		if ((reply = check_command(fsg, 10, DATA_DIR_FROM_HOST,
				(1<<1) | (0xf<<2) | (3<<7), 1,
				"WRITE(10)")) == 0)
			reply = do_write(fsg);
		break;

	case SC_WRITE_12:
		fsg->data_size_from_cmnd = get_be32(&fsg->cmnd[6]) << 9;
		if ((reply = check_command(fsg, 12, DATA_DIR_FROM_HOST,
				(1<<1) | (0xf<<2) | (0xf<<6), 1,
				"WRITE(12)")) == 0)
			reply = do_write(fsg);
		break;

	/* Some mandatory commands that we recognize but don't implement.
	 * They don't mean much in this setting.  It's left as an exercise
	 * for anyone interested to implement RESERVE and RELEASE in terms
	 * of Posix locks. */
	case SC_FORMAT_UNIT:
	case SC_RELEASE:
	case SC_RESERVE:
	case SC_SEND_DIAGNOSTIC:
		// Fall through

	default:
		fsg->data_size_from_cmnd = 0;
		sprintf(unknown, "Unknown x%02x", fsg->cmnd[0]);
		if ((reply = check_command(fsg, fsg->cmnd_size,
				DATA_DIR_UNKNOWN, 0xff, 0, unknown)) == 0) {
			fsg->curlun->sense_data = SS_INVALID_COMMAND;
			reply = -EINVAL;
		}
		break;
	}
	up_read(&fsg->filesem);

	if (reply == -EINTR || signal_pending(current))
		return -EINTR;

	/* Set up the single reply buffer for finish_reply() */
	if (reply == -EINVAL)
		reply = 0;		// Error reply length
	if (reply >= 0 && fsg->data_dir == DATA_DIR_TO_HOST) {
		reply = min((u32) reply, fsg->data_size_from_cmnd);
		bh->inreq->length = reply;
		bh->state = BUF_STATE_FULL;
		fsg->residue -= reply;
	}				// Otherwise it's already set

	return 0;
}


/*-------------------------------------------------------------------------*/

static int received_cbw(struct fsg_dev *fsg, struct fsg_buffhd *bh)
{
	struct usb_request	*req = bh->outreq;
	struct bulk_cb_wrap	*cbw = (struct bulk_cb_wrap *) req->buf;

	/* Was this a real packet? */
	if (req->status)
		return -EINVAL;

	/* Is the CBW valid? */
	if (req->actual != USB_BULK_CB_WRAP_LEN ||
			cbw->Signature != __constant_cpu_to_le32(
				USB_BULK_CB_SIG)) {
		DBG(fsg, "invalid CBW: len %u sig 0x%x\n",
				req->actual,
				le32_to_cpu(cbw->Signature));

		/* The Bulk-only spec says we MUST stall the bulk pipes!
		 * If we want to avoid stalls, set a flag so that we will
		 * clear the endpoint halts at the next reset. */
		if (!mod_data.can_stall)
			set_bit(CLEAR_BULK_HALTS, &fsg->atomic_bitflags);
		fsg_set_halt(fsg, fsg->bulk_out);
		halt_bulk_in_endpoint(fsg);
		return -EINVAL;
	}

	/* Is the CBW meaningful? */
	if (cbw->Lun >= MAX_LUNS || cbw->Flags & ~USB_BULK_IN_FLAG ||
			cbw->Length < 6 || cbw->Length > MAX_COMMAND_SIZE) {
		DBG(fsg, "non-meaningful CBW: lun = %u, flags = 0x%x, "
				"cmdlen %u\n",
				cbw->Lun, cbw->Flags, cbw->Length);

		/* We can do anything we want here, so let's stall the
		 * bulk pipes if we are allowed to. */
		if (mod_data.can_stall) {
			fsg_set_halt(fsg, fsg->bulk_out);
			halt_bulk_in_endpoint(fsg);
		}
		return -EINVAL;
	}

	/* Save the command for later */
	fsg->cmnd_size = cbw->Length;
	memcpy(fsg->cmnd, cbw->CDB, fsg->cmnd_size);
	if (cbw->Flags & USB_BULK_IN_FLAG)
		fsg->data_dir = DATA_DIR_TO_HOST;
	else
		fsg->data_dir = DATA_DIR_FROM_HOST;
	fsg->data_size = le32_to_cpu(cbw->DataTransferLength);
	if (fsg->data_size == 0)
		fsg->data_dir = DATA_DIR_NONE;
	fsg->lun = cbw->Lun;
	fsg->tag = cbw->Tag;
	return 0;
}


static int get_next_command(struct fsg_dev *fsg)
{
	struct fsg_buffhd	*bh;
	int			rc = 0;

	if (transport_is_bbb()) {

		/* Wait for the next buffer to become available */
		bh = fsg->next_buffhd_to_fill;
		while (bh->state != BUF_STATE_EMPTY) {
			if ((rc = sleep_thread(fsg)) != 0)
				return rc;
			}

		/* Queue a request to read a Bulk-only CBW */
		set_bulk_out_req_length(fsg, bh, USB_BULK_CB_WRAP_LEN);
		bh->outreq->short_not_ok = 1;
		start_transfer(fsg, fsg->bulk_out, bh->outreq,
				&bh->outreq_busy, &bh->state);

		/* We will drain the buffer in software, which means we
		 * can reuse it for the next filling.  No need to advance
		 * next_buffhd_to_fill. */

		/* Wait for the CBW to arrive */
		while (bh->state != BUF_STATE_FULL) {
			if ((rc = sleep_thread(fsg)) != 0)
				return rc;
			}
		rc = received_cbw(fsg, bh);
		bh->state = BUF_STATE_EMPTY;

	} else {		// USB_PR_CB or USB_PR_CBI

		/* Wait for the next command to arrive */
		while (fsg->cbbuf_cmnd_size == 0) {
			if ((rc = sleep_thread(fsg)) != 0)
				return rc;
			}

		/* Is the previous status interrupt request still busy?
		 * The host is allowed to skip reading the status,
		 * so we must cancel it. */
		if (fsg->intreq_busy)
			usb_ep_dequeue(fsg->intr_in, fsg->intreq);

		/* Copy the command and mark the buffer empty */
		fsg->data_dir = DATA_DIR_UNKNOWN;
		spin_lock_irq(&fsg->lock);
		fsg->cmnd_size = fsg->cbbuf_cmnd_size;
		memcpy(fsg->cmnd, fsg->cbbuf_cmnd, fsg->cmnd_size);
		fsg->cbbuf_cmnd_size = 0;
		spin_unlock_irq(&fsg->lock);
	}
	return rc;
}


/*-------------------------------------------------------------------------*/

static int enable_endpoint(struct fsg_dev *fsg, struct usb_ep *ep,
		const struct usb_endpoint_descriptor *d)
{
	int	rc;

	ep->driver_data = fsg;
	rc = usb_ep_enable(ep, d);
	if (rc)
		ERROR(fsg, "can't enable %s, result %d\n", ep->name, rc);
	return rc;
}

static int alloc_request(struct fsg_dev *fsg, struct usb_ep *ep,
		struct usb_request **preq)
{
	*preq = usb_ep_alloc_request(ep, GFP_ATOMIC);
	if (*preq)
		return 0;
	ERROR(fsg, "can't allocate request for %s\n", ep->name);
	return -ENOMEM;
}

/*
 * Reset interface setting and re-init endpoint state (toggle etc).
 * Call with altsetting < 0 to disable the interface.  The only other
 * available altsetting is 0, which enables the interface.
 */
static int do_set_interface(struct fsg_dev *fsg, int altsetting)
{
	int	rc = 0;
	int	i;
	const struct usb_endpoint_descriptor	*d;

	if (fsg->running)
		DBG(fsg, "reset interface\n");

reset:
	/* Deallocate the requests */
	for (i = 0; i < NUM_BUFFERS; ++i) {
		struct fsg_buffhd *bh = &fsg->buffhds[i];

		if (bh->inreq) {
			usb_ep_free_request(fsg->bulk_in, bh->inreq);
			bh->inreq = NULL;
		}
		if (bh->outreq) {
			usb_ep_free_request(fsg->bulk_out, bh->outreq);
			bh->outreq = NULL;
		}
	}
	if (fsg->intreq) {
		usb_ep_free_request(fsg->intr_in, fsg->intreq);
		fsg->intreq = NULL;
	}

	/* Disable the endpoints */
	if (fsg->bulk_in_enabled) {
		usb_ep_disable(fsg->bulk_in);
		fsg->bulk_in_enabled = 0;
	}
	if (fsg->bulk_out_enabled) {
		usb_ep_disable(fsg->bulk_out);
		fsg->bulk_out_enabled = 0;
	}
	if (fsg->intr_in_enabled) {
		usb_ep_disable(fsg->intr_in);
		fsg->intr_in_enabled = 0;
	}

	fsg->running = 0;
	if (altsetting < 0 || rc != 0)
		return rc;

	DBG(fsg, "set interface %d\n", altsetting);

	/* Enable the endpoints */
	d = ep_desc(fsg->gadget, &fs_bulk_in_desc, &hs_bulk_in_desc);
	if ((rc = enable_endpoint(fsg, fsg->bulk_in, d)) != 0)
		goto reset;
	fsg->bulk_in_enabled = 1;

	d = ep_desc(fsg->gadget, &fs_bulk_out_desc, &hs_bulk_out_desc);
	if ((rc = enable_endpoint(fsg, fsg->bulk_out, d)) != 0)
		goto reset;
	fsg->bulk_out_enabled = 1;
	fsg->bulk_out_maxpacket = d->wMaxPacketSize;

	if (transport_is_cbi()) {
		d = ep_desc(fsg->gadget, &fs_intr_in_desc, &hs_intr_in_desc);
		if ((rc = enable_endpoint(fsg, fsg->intr_in, d)) != 0)
			goto reset;
		fsg->intr_in_enabled = 1;
	}

	/* Allocate the requests */
	for (i = 0; i < NUM_BUFFERS; ++i) {
		struct fsg_buffhd	*bh = &fsg->buffhds[i];

		if ((rc = alloc_request(fsg, fsg->bulk_in, &bh->inreq)) != 0)
			goto reset;
		if ((rc = alloc_request(fsg, fsg->bulk_out, &bh->outreq)) != 0)
			goto reset;
		bh->inreq->buf = bh->outreq->buf = bh->buf;
		bh->inreq->dma = bh->outreq->dma = bh->dma;
		bh->inreq->context = bh->outreq->context = bh;
		bh->inreq->complete = bulk_in_complete;
		bh->outreq->complete = bulk_out_complete;
	}
	if (transport_is_cbi()) {
		if ((rc = alloc_request(fsg, fsg->intr_in, &fsg->intreq)) != 0)
			goto reset;
		fsg->intreq->complete = intr_in_complete;
	}

	fsg->running = 1;
	for (i = 0; i < fsg->nluns; ++i)
		fsg->luns[i].unit_attention_data = SS_RESET_OCCURRED;
	return rc;
}


/*
 * Change our operational configuration.  This code must agree with the code
 * that returns config descriptors, and with interface altsetting code.
 *
 * It's also responsible for power management interactions.  Some
 * configurations might not work with our current power sources.
 * For now we just assume the gadget is always self-powered.
 */
static int do_set_config(struct fsg_dev *fsg, u8 new_config)
{
	int	rc = 0;

	/* Disable the single interface */
	if (fsg->config != 0) {
		DBG(fsg, "reset config\n");
		fsg->config = 0;
		rc = do_set_interface(fsg, -1);
	}

	/* Enable the interface */
	if (new_config != 0) {
		fsg->config = new_config;
		if ((rc = do_set_interface(fsg, 0)) != 0)
			fsg->config = 0;	// Reset on errors
		else {
			char *speed;

			switch (fsg->gadget->speed) {
			case USB_SPEED_LOW:	speed = "low";	break;
			case USB_SPEED_FULL:	speed = "full";	break;
			case USB_SPEED_HIGH:	speed = "high";	break;
			default: 		speed = "?";	break;
			}
			INFO(fsg, "%s speed config #%d\n", speed, fsg->config);
		}
	}
	return rc;
}


/*-------------------------------------------------------------------------*/

static void handle_exception(struct fsg_dev *fsg)
{
	siginfo_t		info;
	int			sig;
	int			i;
	int			num_active;
	struct fsg_buffhd	*bh;
	enum fsg_state		old_state;
	u8			new_config;
	struct lun		*curlun;
	unsigned int		exception_req_tag;
	int			rc;

	/* Clear the existing signals.  Anything but SIGUSR1 is converted
	 * into a high-priority EXIT exception. */
	for (;;) {
		sig = dequeue_signal_lock(current, &fsg->thread_signal_mask,
				&info);
		if (!sig)
			break;
		if (sig != SIGUSR1) {
			if (fsg->state < FSG_STATE_EXIT)
				DBG(fsg, "Main thread exiting on signal\n");
			raise_exception(fsg, FSG_STATE_EXIT);
		}
	}

	/* Cancel all the pending transfers */
	if (fsg->intreq_busy)
		usb_ep_dequeue(fsg->intr_in, fsg->intreq);
	for (i = 0; i < NUM_BUFFERS; ++i) {
		bh = &fsg->buffhds[i];
		if (bh->inreq_busy)
			usb_ep_dequeue(fsg->bulk_in, bh->inreq);
		if (bh->outreq_busy)
			usb_ep_dequeue(fsg->bulk_out, bh->outreq);
	}

	/* Wait until everything is idle */
	for (;;) {
		num_active = fsg->intreq_busy;
		for (i = 0; i < NUM_BUFFERS; ++i) {
			bh = &fsg->buffhds[i];
			num_active += bh->inreq_busy + bh->outreq_busy;
		}
		if (num_active == 0)
			break;
		if (sleep_thread(fsg))
			return;
	}

	/* Clear out the controller's fifos */
	if (fsg->bulk_in_enabled)
		usb_ep_fifo_flush(fsg->bulk_in);
	if (fsg->bulk_out_enabled)
		usb_ep_fifo_flush(fsg->bulk_out);
	if (fsg->intr_in_enabled)
		usb_ep_fifo_flush(fsg->intr_in);

	/* Reset the I/O buffer states and pointers, the SCSI
	 * state, and the exception.  Then invoke the handler. */
	spin_lock_irq(&fsg->lock);

	for (i = 0; i < NUM_BUFFERS; ++i) {
		bh = &fsg->buffhds[i];
		bh->state = BUF_STATE_EMPTY;
	}
	fsg->next_buffhd_to_fill = fsg->next_buffhd_to_drain =
			&fsg->buffhds[0];

	exception_req_tag = fsg->exception_req_tag;
	new_config = fsg->new_config;
	old_state = fsg->state;

	if (old_state == FSG_STATE_ABORT_BULK_OUT)
		fsg->state = FSG_STATE_STATUS_PHASE;
	else {
		for (i = 0; i < fsg->nluns; ++i) {
			curlun = &fsg->luns[i];
			curlun->prevent_medium_removal = 0;
			curlun->sense_data = curlun->unit_attention_data =
					SS_NO_SENSE;
			curlun->sense_data_info = 0;
		}
		fsg->state = FSG_STATE_IDLE;
	}
	spin_unlock_irq(&fsg->lock);

	/* Carry out any extra actions required for the exception */
	switch (old_state) {
	default:
		break;

	case FSG_STATE_ABORT_BULK_OUT:
		send_status(fsg);
		spin_lock_irq(&fsg->lock);
		if (fsg->state == FSG_STATE_STATUS_PHASE)
			fsg->state = FSG_STATE_IDLE;
		spin_unlock_irq(&fsg->lock);
		break;

	case FSG_STATE_RESET:
		/* In case we were forced against our will to halt a
		 * bulk endpoint, clear the halt now.  (The SuperH UDC
		 * requires this.) */
		if (test_and_clear_bit(CLEAR_BULK_HALTS,
				&fsg->atomic_bitflags)) {
			usb_ep_clear_halt(fsg->bulk_in);
			usb_ep_clear_halt(fsg->bulk_out);
		}

		if (transport_is_bbb()) {
			if (fsg->ep0_req_tag == exception_req_tag)
				ep0_queue(fsg);	// Complete the status stage

		} else if (transport_is_cbi())
			send_status(fsg);	// Status by interrupt pipe

		/* Technically this should go here, but it would only be
		 * a waste of time.  Ditto for the INTERFACE_CHANGE and
		 * CONFIG_CHANGE cases. */
		// for (i = 0; i < fsg->nluns; ++i)
		//	fsg->luns[i].unit_attention_data = SS_RESET_OCCURRED;
		break;

	case FSG_STATE_INTERFACE_CHANGE:
		rc = do_set_interface(fsg, 0);
		if (fsg->ep0_req_tag != exception_req_tag)
			break;
		if (rc != 0)			// STALL on errors
			fsg_set_halt(fsg, fsg->ep0);
		else				// Complete the status stage
			ep0_queue(fsg);
		break;

	case FSG_STATE_CONFIG_CHANGE:
		rc = do_set_config(fsg, new_config);
		if (fsg->ep0_req_tag != exception_req_tag)
			break;
		if (rc != 0)			// STALL on errors
			fsg_set_halt(fsg, fsg->ep0);
		else				// Complete the status stage
			ep0_queue(fsg);
		break;

	case FSG_STATE_DISCONNECT:
		fsync_all(fsg);
		do_set_config(fsg, 0);		// Unconfigured state
		break;

	case FSG_STATE_EXIT:
	case FSG_STATE_TERMINATED:
		do_set_config(fsg, 0);			// Free resources
		spin_lock_irq(&fsg->lock);
		fsg->state = FSG_STATE_TERMINATED;	// Stop the thread
		spin_unlock_irq(&fsg->lock);
		break;
	}
}


/*-------------------------------------------------------------------------*/

static int fsg_main_thread(void *fsg_)
{
	struct fsg_dev		*fsg = (struct fsg_dev *) fsg_;

	fsg->thread_task = current;

	/* Release all our userspace resources */
	daemonize("file-storage-gadget");

	/* Allow the thread to be killed by a signal, but set the signal mask
	 * to block everything but INT, TERM, KILL, and USR1. */
	siginitsetinv(&fsg->thread_signal_mask, sigmask(SIGINT) |
			sigmask(SIGTERM) | sigmask(SIGKILL) |
			sigmask(SIGUSR1));
	sigprocmask(SIG_SETMASK, &fsg->thread_signal_mask, NULL);

	/* Arrange for userspace references to be interpreted as kernel
	 * pointers.  That way we can pass a kernel pointer to a routine
	 * that expects a __user pointer and it will work okay. */
	set_fs(get_ds());

	/* Wait for the gadget registration to finish up */
	wait_for_completion(&fsg->thread_notifier);

	/* The main loop */
	while (fsg->state != FSG_STATE_TERMINATED) {
		if (exception_in_progress(fsg) || signal_pending(current)) {
			handle_exception(fsg);
			continue;
		}

		// try sending data to user (non-blocking)
		if( mod_data.netlink && gadgetfs_info.count ){
		  // INFO(fsg, "%s: sending some data non-blocking (%u).\n", __FUNCTION__, gadgetfs_info.count);
		  // non-blocking
		  if( send_netlink(1) > 0 ){
		    clear_gadgetfs_info();
		  }
		}

		if (!fsg->running) {
			sleep_thread(fsg);
			continue;
		}

		if (get_next_command(fsg))
			continue;

		spin_lock_irq(&fsg->lock);
		if (!exception_in_progress(fsg))
			fsg->state = FSG_STATE_DATA_PHASE;
		spin_unlock_irq(&fsg->lock);

		if (do_scsi_command(fsg) || finish_reply(fsg))
			continue;

		spin_lock_irq(&fsg->lock);
		if (!exception_in_progress(fsg))
			fsg->state = FSG_STATE_STATUS_PHASE;
		spin_unlock_irq(&fsg->lock);

		if (send_status(fsg))
			continue;

		spin_lock_irq(&fsg->lock);
		if (!exception_in_progress(fsg))
			fsg->state = FSG_STATE_IDLE;
		spin_unlock_irq(&fsg->lock);
		}

	spin_lock_irq(&fsg->lock);
	fsg->thread_task = NULL;
	spin_unlock_irq(&fsg->lock);
	flush_signals(current);

	/* In case we are exiting because of a signal, unregister the
	 * gadget driver and close the backing file. */
	if (test_and_clear_bit(REGISTERED, &fsg->atomic_bitflags)) {
		usb_gadget_unregister_driver(&fsg_driver);
		close_all_backing_files(fsg);
	}

	/* Let the unbind and cleanup routines know the thread has exited */
	complete_and_exit(&fsg->thread_notifier, 0);
}


/*-------------------------------------------------------------------------*/

/* If the next two routines are called while the gadget is registered,
 * the caller must own fsg->filesem for writing. */

static int open_backing_file(struct lun *curlun, const char *filename, int filesize)
{
	int				ro;
	struct file			*filp = NULL;
	int				rc = -EINVAL;
	struct inode			*inode = NULL;
	loff_t				size;
	loff_t				num_sectors;

	/* R/W if we can, R/O if we must */
	ro = curlun->ro;
	if (!ro) {
		filp = filp_open(filename, O_RDWR | O_LARGEFILE, 0);
		if (-EROFS == PTR_ERR(filp))
			ro = 1;
	}
	if (ro)
		filp = filp_open(filename, O_RDONLY | O_LARGEFILE, 0);
	if (IS_ERR(filp)) {
		LINFO(curlun, "unable to open backing file: %s\n", filename);
		return PTR_ERR(filp);
	}

	if (!(filp->f_mode & FMODE_WRITE))
		ro = 1;

	if (filp->f_dentry)
		inode = filp->f_dentry->d_inode;
	if (inode && S_ISBLK(inode->i_mode)) {
		if (bdev_read_only(inode->i_bdev))
			ro = 1;
	} else if (!inode || !S_ISREG(inode->i_mode)) {
		LINFO(curlun, "invalid file type: %s\n", filename);
		goto out;
	}

	/* If we can't read the file, it's no good.
	 * If we can't write the file, use it read-only. */
	if (!filp->f_op || !(filp->f_op->read || filp->f_op->aio_read)) {
		LINFO(curlun, "file not readable: %s\n", filename);
		goto out;
	}
	if (!(filp->f_op->write || filp->f_op->aio_write))
		ro = 1;

	size = i_size_read(inode->i_mapping->host);
	if (size < 0) {
		LINFO(curlun, "unable to find file size: %s\n", filename);
		rc = (int) size;
		goto out;
	}
	num_sectors = size >> 9;	// File size in 512-byte sectors
	if (num_sectors == 0) {
		LINFO(curlun, "file too small: %s\n", filename);
		rc = -ETOOSMALL;
		goto out;
	}

	/* set file size if necessary */
	if( filesize!=0 && num_sectors > filesize ){
	  num_sectors=filesize;
	  size=num_sectors << 9;
	}

	get_file(filp);
	curlun->ro = ro;
	curlun->filp = filp;
	curlun->file_length = size;
	curlun->num_sectors = num_sectors;
	LDBG(curlun, "open backing file: %s\n", filename);
	rc = 0;

out:
	filp_close(filp, current->files);
	return rc;
}


static void close_backing_file(struct lun *curlun)
{
	if (curlun->filp) {
		LDBG(curlun, "close backing file\n");
		fsync_sub(curlun);
		fput(curlun->filp);
		curlun->filp = NULL;
	}
}

static void close_all_backing_files(struct fsg_dev *fsg)
{
	int	i;

	for (i = 0; i < fsg->nluns; ++i)
		close_backing_file(&fsg->luns[i]);
}


static ssize_t show_ro(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct lun	*curlun = dev_to_lun(dev);

	return sprintf(buf, "%d\n", curlun->ro);
}

static ssize_t show_file(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct lun	*curlun = dev_to_lun(dev);
	struct fsg_dev	*fsg = (struct fsg_dev *) dev_get_drvdata(dev);
	char		*p;
	ssize_t		rc;

	down_read(&fsg->filesem);
	if (backing_file_is_open(curlun)) {	// Get the complete pathname
		p = d_path(curlun->filp->f_dentry, curlun->filp->f_vfsmnt,
				buf, PAGE_SIZE - 1);
		if (IS_ERR(p))
			rc = PTR_ERR(p);
		else {
			rc = strlen(p);
			memmove(buf, p, rc);
			buf[rc] = '\n';		// Add a newline
			buf[++rc] = 0;
		}
	} else {				// No file, return 0 bytes
		*buf = 0;
		rc = 0;
	}
	up_read(&fsg->filesem);
	return rc;
}


static ssize_t store_ro(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t		rc = count;
	struct lun	*curlun = dev_to_lun(dev);
	struct fsg_dev	*fsg = (struct fsg_dev *) dev_get_drvdata(dev);
	int		i;

	if (sscanf(buf, "%d", &i) != 1)
		return -EINVAL;

	/* Allow the write-enable status to change only while the backing file
	 * is closed. */
	down_read(&fsg->filesem);
	if (backing_file_is_open(curlun)) {
		LDBG(curlun, "read-only status change prevented\n");
		rc = -EBUSY;
	} else {
		curlun->ro = !!i;
		LDBG(curlun, "read-only status set to %d\n", curlun->ro);
	}
	up_read(&fsg->filesem);
	return rc;
}

static ssize_t store_file(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct lun	*curlun = dev_to_lun(dev);
	struct fsg_dev	*fsg = (struct fsg_dev *) dev_get_drvdata(dev);
	int		rc = 0;

	if (curlun->prevent_medium_removal && backing_file_is_open(curlun)) {
		LDBG(curlun, "eject attempt prevented\n");
		return -EBUSY;				// "Door is locked"
	}

	/* Remove a trailing newline */
	if (count > 0 && buf[count-1] == '\n')
		((char *) buf)[count-1] = 0;		// Ugh!

	/* Eject current medium */
	down_write(&fsg->filesem);
	if (backing_file_is_open(curlun)) {
		close_backing_file(curlun);
		curlun->unit_attention_data = SS_MEDIUM_NOT_PRESENT;
	}

	/* Load new medium */
	if (count > 0 && buf[0]) {
	  rc = open_backing_file(curlun, buf, 0);
		if (rc == 0)
			curlun->unit_attention_data =
					SS_NOT_READY_TO_READY_TRANSITION;
	}
	up_write(&fsg->filesem);
	return (rc < 0 ? rc : count);
}


/* The write permissions and store_xxx pointers are set in fsg_bind() */
static DEVICE_ATTR(ro, 0444, show_ro, NULL);
static DEVICE_ATTR(file, 0444, show_file, NULL);


/*-------------------------------------------------------------------------*/

static void lun_release(struct device *dev)
{
	struct fsg_dev	*fsg = (struct fsg_dev *) dev_get_drvdata(dev);

	complete(&fsg->lun_released);
}

static void fsg_unbind(struct usb_gadget *gadget)
{
	struct fsg_dev		*fsg = get_gadget_data(gadget);
	int			i;
	struct lun		*curlun;
	struct usb_request	*req = fsg->ep0req;

	DBG(fsg, "unbind\n");
	clear_bit(REGISTERED, &fsg->atomic_bitflags);

	/* Unregister the sysfs attribute files and the LUNs */
	init_completion(&fsg->lun_released);
	for (i = 0; i < fsg->nluns; ++i) {
		curlun = &fsg->luns[i];
		if (curlun->registered) {
			device_remove_file(&curlun->dev, &dev_attr_ro);
			device_remove_file(&curlun->dev, &dev_attr_file);
			device_unregister(&curlun->dev);
			wait_for_completion(&fsg->lun_released);
			curlun->registered = 0;
		}
	}

	/* If the thread isn't already dead, tell it to exit now */
	if (fsg->state != FSG_STATE_TERMINATED) {
		raise_exception(fsg, FSG_STATE_EXIT);
		wait_for_completion(&fsg->thread_notifier);

		/* The cleanup routine waits for this completion also */
		complete(&fsg->thread_notifier);
	}

	if( fsg->ahead_bh.buf ){
		int order = get_order(mod_data.buflen);

		free_pages((unsigned long)fsg->ahead_bh.buf,order);

		fsg->ahead_bh.buf = NULL;
	}

	/* Free the data buffers */
	for (i = 0; i < NUM_BUFFERS; ++i) {
		struct fsg_buffhd	*bh = &fsg->buffhds[i];
		int order;

		order = get_order(mod_data.buflen);

		if (bh->buf)
			free_pages((unsigned long)bh->buf,order);			
	}

	/* Free the request and buffer for endpoint 0 */
	if (req) {
		if (req->buf)
			usb_ep_free_buffer(fsg->ep0, req->buf,
					req->dma, EP0_BUFSIZE);
		usb_ep_free_request(fsg->ep0, req);
	}

	set_gadget_data(gadget, NULL);
}


static int __init check_parameters(struct fsg_dev *fsg)
{
	int	prot;
	int	gcnum;

	/* Store the default values */
	mod_data.transport_type = USB_PR_BULK;
	mod_data.transport_name = "Bulk-only";
	mod_data.protocol_type = USB_SC_SCSI;
	mod_data.protocol_name = "Transparent SCSI";

	if (gadget_is_sh(fsg->gadget))
		mod_data.can_stall = 0;

	if (mod_data.release == 0xffff) {	// Parameter wasn't set
		/* The sa1100 controller is not supported */
		if (gadget_is_sa1100(fsg->gadget))
			gcnum = -1;
		else
			gcnum = usb_gadget_controller_number(fsg->gadget);
		if (gcnum >= 0)
			mod_data.release = 0x0300 + gcnum;
		else {
			WARN(fsg, "controller '%s' not recognized\n",
				fsg->gadget->name);
			mod_data.release = 0x0399;
		}
	}

	prot = simple_strtol(mod_data.protocol_parm, NULL, 0);

#ifdef CONFIG_USB_FILE_STORAGE_TEST
	if (strnicmp(mod_data.transport_parm, "BBB", 10) == 0) {
		;		// Use default setting
	} else if (strnicmp(mod_data.transport_parm, "CB", 10) == 0) {
		mod_data.transport_type = USB_PR_CB;
		mod_data.transport_name = "Control-Bulk";
	} else if (strnicmp(mod_data.transport_parm, "CBI", 10) == 0) {
		mod_data.transport_type = USB_PR_CBI;
		mod_data.transport_name = "Control-Bulk-Interrupt";
	} else {
		ERROR(fsg, "invalid transport: %s\n", mod_data.transport_parm);
		return -EINVAL;
	}

	if (strnicmp(mod_data.protocol_parm, "SCSI", 10) == 0 ||
			prot == USB_SC_SCSI) {
		;		// Use default setting
	} else if (strnicmp(mod_data.protocol_parm, "RBC", 10) == 0 ||
			prot == USB_SC_RBC) {
		mod_data.protocol_type = USB_SC_RBC;
		mod_data.protocol_name = "RBC";
	} else if (strnicmp(mod_data.protocol_parm, "8020", 4) == 0 ||
			strnicmp(mod_data.protocol_parm, "ATAPI", 10) == 0 ||
			prot == USB_SC_8020) {
		mod_data.protocol_type = USB_SC_8020;
		mod_data.protocol_name = "8020i (ATAPI)";
	} else if (strnicmp(mod_data.protocol_parm, "QIC", 3) == 0 ||
			prot == USB_SC_QIC) {
		mod_data.protocol_type = USB_SC_QIC;
		mod_data.protocol_name = "QIC-157";
	} else if (strnicmp(mod_data.protocol_parm, "UFI", 10) == 0 ||
			prot == USB_SC_UFI) {
		mod_data.protocol_type = USB_SC_UFI;
		mod_data.protocol_name = "UFI";
	} else if (strnicmp(mod_data.protocol_parm, "8070", 4) == 0 ||
			prot == USB_SC_8070) {
		mod_data.protocol_type = USB_SC_8070;
		mod_data.protocol_name = "8070i";
	} else {
		ERROR(fsg, "invalid protocol: %s\n", mod_data.protocol_parm);
		return -EINVAL;
	}

	mod_data.buflen &= PAGE_CACHE_MASK;
	if (mod_data.buflen <= 0) {
		ERROR(fsg, "invalid buflen\n");
		return -ETOOSMALL;
	}
#endif /* CONFIG_USB_FILE_STORAGE_TEST */

	return 0;
}


static int __init fsg_bind(struct usb_gadget *gadget)
{
	struct fsg_dev		*fsg = the_fsg;
	int			rc;
	int			i;
	struct lun		*curlun;
	struct usb_ep		*ep;
	struct usb_request	*req;
	char			*pathbuf, *p;
	int 			tempCount, nMscPagesInSingleBuffer;
	unsigned int order;    
	struct page *dataPage;

	fsg->gadget = gadget;
	set_gadget_data(gadget, fsg);
	fsg->ep0 = gadget->ep0;
	fsg->ep0->driver_data = fsg;	
	
	if ((rc = check_parameters(fsg)) != 0)
		goto out;

	if (mod_data.removable) {	// Enable the store_xxx attributes
		dev_attr_ro.attr.mode = dev_attr_file.attr.mode = 0644;
		dev_attr_ro.store   = store_ro;
		dev_attr_file.store = store_file;
	}

	/* Find out how many LUNs there should be */
	i = mod_data.nluns;
	if (i == 0)
		i = max(mod_data.num_filenames, 1);
	if (i > MAX_LUNS) {
		ERROR(fsg, "invalid number of LUNs: %d\n", i);
		rc = -EINVAL;
		goto out;
	}

	/* Create the LUNs, open their backing files, and register the
	 * LUN devices in sysfs. */
	fsg->luns = kzalloc(i * sizeof(struct lun), GFP_KERNEL);
	if (!fsg->luns) {
		rc = -ENOMEM;
		goto out;
	}
	fsg->nluns = i;

	for (i = 0; i < fsg->nluns; ++i) {
		curlun = &fsg->luns[i];
		curlun->ro = ro[i];
		curlun->dev.parent = &gadget->dev;
		curlun->dev.driver = &fsg_driver.driver;
		dev_set_drvdata(&curlun->dev, fsg);
		snprintf(curlun->dev.bus_id, BUS_ID_SIZE,
				"%s-lun%d", gadget->dev.bus_id, i);

		if ((rc = device_register(&curlun->dev)) != 0)
			INFO(fsg, "failed to register LUN%d: %d\n", i, rc);
		else {
			curlun->registered = 1;
			curlun->dev.release = lun_release;
			if( device_create_file(&curlun->dev, &dev_attr_ro) || device_create_file(&curlun->dev, &dev_attr_file) ){
				device_unregister(&curlun->dev);
				goto out;
			}
		}

		if (file[i] && *file[i]) {
		  if ((rc = open_backing_file(curlun, file[i], size[i])) != 0)
				goto out;
		} else if (!mod_data.removable) {
			ERROR(fsg, "no file given for LUN%d\n", i);
			rc = -EINVAL;
			goto out;
		}
		
		atomic_set(&curlun->pending_bio_count, 0);
		atomic_set(&curlun->pending_write_count, 0);
	}

	/* Find all the endpoints we will use */
	usb_ep_autoconfig_reset(gadget);
	ep = usb_ep_autoconfig(gadget, &fs_bulk_in_desc);
	if (!ep)
		goto autoconf_fail;
	ep->driver_data = fsg;		// claim the endpoint
	fsg->bulk_in = ep;

	ep = usb_ep_autoconfig(gadget, &fs_bulk_out_desc);
	if (!ep)
		goto autoconf_fail;
	ep->driver_data = fsg;		// claim the endpoint
	fsg->bulk_out = ep;

	if (transport_is_cbi()) {
		ep = usb_ep_autoconfig(gadget, &fs_intr_in_desc);
		if (!ep)
			goto autoconf_fail;
		ep->driver_data = fsg;		// claim the endpoint
		fsg->intr_in = ep;
	}

	/* Fix up the descriptors */
	device_desc.bMaxPacketSize0 = fsg->ep0->maxpacket;
	device_desc.idVendor = cpu_to_le16(mod_data.vendor);
	device_desc.idProduct = cpu_to_le16(mod_data.product);
	device_desc.bcdDevice = cpu_to_le16(mod_data.release);

	strncpy(manufacturer_name, mod_data.manufacturer_name, sizeof(manufacturer_name)-1);
	manufacturer_name[sizeof(manufacturer_name)-1] = 0;

	strncpy(product_name, mod_data.product_name, sizeof(product_name)-1);
	product_name[sizeof(product_name)-1] = 0;

	strncpy(serial_string, mod_data.serial_string, sizeof(serial_string)-1);
	serial_string[sizeof(serial_string)-1] = 0;

	i = (transport_is_cbi() ? 3 : 2);	// Number of endpoints
	intf_desc.bNumEndpoints = i;
	intf_desc.bInterfaceSubClass = mod_data.protocol_type;
	intf_desc.bInterfaceProtocol = mod_data.transport_type;
	fs_function[i + FS_FUNCTION_PRE_EP_ENTRIES] = NULL;

#ifdef CONFIG_USB_GADGET_DUALSPEED
	hs_function[i + HS_FUNCTION_PRE_EP_ENTRIES] = NULL;

	/* Assume ep0 uses the same maxpacket value for both speeds */
	dev_qualifier.bMaxPacketSize0 = fsg->ep0->maxpacket;

	/* Assume that all endpoint addresses are the same for both speeds */
	hs_bulk_in_desc.bEndpointAddress = fs_bulk_in_desc.bEndpointAddress;
	hs_bulk_out_desc.bEndpointAddress = fs_bulk_out_desc.bEndpointAddress;
	hs_intr_in_desc.bEndpointAddress = fs_intr_in_desc.bEndpointAddress;
#endif

	if (gadget->is_otg) {
		otg_desc.bmAttributes |= USB_OTG_HNP,
		config_desc.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	}

	rc = -ENOMEM;

	/* Allocate the request and buffer for endpoint 0 */
	fsg->ep0req = req = usb_ep_alloc_request(fsg->ep0, GFP_KERNEL);
	if (!req)
		goto out;
	req->buf = usb_ep_alloc_buffer(fsg->ep0, EP0_BUFSIZE,
			&req->dma, GFP_KERNEL);
	if (!req->buf)
		goto out;
	req->complete = ep0_complete;

	nMscPagesInSingleBuffer = (mod_data.buflen/PAGE_SIZE);
	if(nMscPagesInSingleBuffer > MSC_MAX_NR_PAGES)
		goto out;

	/* Allocate the data buffers */	
	for (i = 0; i < NUM_BUFFERS; ++i) {
		struct fsg_buffhd	*bh = &fsg->buffhds[i];
		
		/* allocate buffers using alloc pages */
		order = get_order(mod_data.buflen);
		dataPage = alloc_pages(GFP_KERNEL,order);
		bh->buf = page_address(dataPage);
		bh->dma = DMA_ADDR_INVALID;
		bh->fsg = fsg;
		for(tempCount = 0; tempCount < nMscPagesInSingleBuffer; 
			tempCount++) {
			bh->pDataPage[tempCount] = dataPage + tempCount;
		}

		if (!bh->buf)
			goto out;

		bh->next = bh + 1;
	} /* end of for loop */

	fsg->buffhds[NUM_BUFFERS - 1].next = &fsg->buffhds[0];

	/* read ahead */
	ERROR(fsg, "allocating read_ahead...\n");
	order = get_order(mod_data.buflen);
	dataPage = alloc_pages(GFP_KERNEL,order);

	fsg->ahead_bh.buf = page_address(dataPage);
	fsg->ahead_bh.dma = DMA_ADDR_INVALID;
	fsg->ahead_bh.fsg = fsg;
	for(tempCount = 0; tempCount < nMscPagesInSingleBuffer; tempCount++) {
		fsg->ahead_bh.pDataPage[tempCount] = dataPage + tempCount;
	}
	if (!fsg->ahead_bh.buf){
		ERROR(fsg, "cannot allocate ahead pages...\n");
		goto out;
	}
	fsg->ahead_bh.next = NULL;


	/* This should reflect the actual gadget power source */
	// MB: we're not self powered
	// usb_gadget_set_selfpowered(gadget);

	if ((rc = kernel_thread(fsg_main_thread, fsg, (CLONE_VM | CLONE_FS |
			CLONE_FILES))) < 0)
		goto out;
	fsg->thread_pid = rc;

	INFO(fsg, DRIVER_DESC ", version: " DRIVER_VERSION "\n");
	INFO(fsg, "Number of LUNs=%d\n", fsg->nluns);

	pathbuf = kmalloc(PATH_MAX, GFP_KERNEL);
	for (i = 0; i < fsg->nluns; ++i) {
		curlun = &fsg->luns[i];
		if (backing_file_is_open(curlun)) {
			p = NULL;
			if (pathbuf) {
				p = d_path(curlun->filp->f_dentry,
					curlun->filp->f_vfsmnt,
					pathbuf, PATH_MAX);
				if (IS_ERR(p))
					p = NULL;
			}
			LINFO(curlun, "ro=%d, file: %s\n",
					curlun->ro, (p ? p : "(error)"));
		}
	}
	kfree(pathbuf);

	if( mod_data.netlink ){
	  INFO(fsg, "Netlink user PID=%d\n", mod_data.netlink);
	} else {
	  INFO(fsg, "no Netlink\n");
	}

	DBG(fsg, "transport=%s (x%02x)\n",
			mod_data.transport_name, mod_data.transport_type);
	DBG(fsg, "protocol=%s (x%02x)\n",
			mod_data.protocol_name, mod_data.protocol_type);
	DBG(fsg, "VendorID=x%04x, ProductID=x%04x, Release=x%04x\n",
			mod_data.vendor, mod_data.product, mod_data.release);
	DBG(fsg, "removable=%d, stall=%d, buflen=%u\n",
			mod_data.removable, mod_data.can_stall,
			mod_data.buflen);
	DBG(fsg, "I/O thread pid: %d\n", fsg->thread_pid);
	return 0;

autoconf_fail:
	ERROR(fsg, "unable to autoconfigure all endpoints\n");
	rc = -ENOTSUPP;

out:
	fsg->state = FSG_STATE_TERMINATED;	// The thread is dead
	fsg_unbind(gadget);
	close_all_backing_files(fsg);
	return rc;
}


/*-------------------------------------------------------------------------*/

static void fsg_suspend(struct usb_gadget *gadget)
{
	struct fsg_dev		*fsg = get_gadget_data(gadget);

	DBG(fsg, "suspend\n");
	set_bit(SUSPENDED, &fsg->atomic_bitflags);
}

static void fsg_resume(struct usb_gadget *gadget)
{
	struct fsg_dev		*fsg = get_gadget_data(gadget);

	DBG(fsg, "resume\n");
	clear_bit(SUSPENDED, &fsg->atomic_bitflags);
}


/*-------------------------------------------------------------------------*/

static struct usb_gadget_driver		fsg_driver = {
#ifdef CONFIG_USB_GADGET_DUALSPEED
	.speed		= USB_SPEED_HIGH,
#else
	.speed		= USB_SPEED_FULL,
#endif
	.function	= (char *) longname,
	.bind		= fsg_bind,
	.unbind		= fsg_unbind,
	.disconnect	= fsg_disconnect,
	.setup		= fsg_setup,
	.suspend	= fsg_suspend,
	.resume		= fsg_resume,

	.driver		= {
		.name		= (char *) shortname,
		.owner		= THIS_MODULE,
		// .release = ...
		// .suspend = ...
		// .resume = ...
	},
};


static int __init fsg_alloc(void)
{
	struct fsg_dev		*fsg;

	fsg = kzalloc(sizeof *fsg, GFP_KERNEL);
	if (!fsg)
		return -ENOMEM;
	spin_lock_init(&fsg->lock);
	init_rwsem(&fsg->filesem);
	init_waitqueue_head(&fsg->thread_wqh);
	init_completion(&fsg->thread_notifier);

	the_fsg = fsg;
	return 0;
}


static void fsg_free(struct fsg_dev *fsg)
{
	kfree(fsg->luns);
	kfree(fsg);
}


static int __init fsg_init(void)
{
	int		rc;
	struct fsg_dev	*fsg;

	if ((rc = fsg_alloc()) != 0)
		return rc;
	fsg = the_fsg;
	if ((rc = usb_gadget_register_driver(&fsg_driver)) != 0) {
		fsg_free(fsg);
		return rc;
	}
	set_bit(REGISTERED, &fsg->atomic_bitflags);

	if( mod_data.netlink ){
	  gadgetfs_sock = netlink_kernel_create(NETLINK_USB_GADGETFS, 0, netlink_input, NULL, THIS_MODULE);

	  if ( gadgetfs_sock==NULL ) {
	    ERROR(fsg, "unable to create netlink socket!\n");
	    return -ENOBUFS;
	  }

	  clear_gadgetfs_info();
	}

	/* Tell the thread to start working */
	complete(&fsg->thread_notifier);
	return 0;
}
module_init(fsg_init);


static void __exit fsg_cleanup(void)
{
	struct fsg_dev	*fsg = the_fsg;

	/* Unregister the driver iff the thread hasn't already done so */
	if (test_and_clear_bit(REGISTERED, &fsg->atomic_bitflags))
		usb_gadget_unregister_driver(&fsg_driver);

	/* Wait for the thread to finish up */
	wait_for_completion(&fsg->thread_notifier);

	if( mod_data.netlink ){
	  if( gadgetfs_info.count ){
	    ERROR(fsg, "buffer not empty when exiting.\n");
	  }
	  sock_release(gadgetfs_sock->sk_socket);
	}

	close_all_backing_files(fsg);
	fsg_free(fsg);
}
module_exit(fsg_cleanup);
