/*
 * mtpfs.c
 *
 * Copyright (c) 2008 Archos
 * Author: Martin Bonnin
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/pagemap.h>

#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/poll.h>

#include <linux/usb/ch9.h>
#include <linux/usb_gadget.h>

#define STATUS_SUPPORT
#ifdef STATUS_SUPPORT
#define ENDPOINT_COUNT	3
#else
#define ENDPOINT_COUNT	2
#endif

#define BUFFER_COUNT	3
#define BUFFER_SIZE	(64 * 1024)

#define MAX_EP0_TRANSFER	512
#define DMA_ADDR_INVALID	(~(dma_addr_t)0)

// MTP enumeration
#define MTP_GET_MS_DESCRIPTOR			0xFE

// those two macro are taken from drivers/usb/gadget/file_storage.c
// USB protocol value = the transport method
#define USB_PR_BULK     0x50            // Bulk-only
// USB subclass value = the protocol encapsulation
#define USB_SC_SCSI     0x06            // Transparent

// PTP Class specific requests (cf USB Still Image Capture Device Definition)
#define PTP_REQUEST_CANCEL			0x64
#define PTP_REQUEST_GET_EXTENDED_EVENT_DATA	0x65
#define PTP_REQUEST_DEVICE_RESET		0x66
#define PTP_REQUEST_GET_DEVICE_STATUS		0x67

#define PTP_RC_OK				0x2001
#define PTP_RC_BUSY				0x2019
#define	SHORTNAME	"mtpfs"

#define DBGP	if(0)
#define DBGQ	if(1)
enum {
	STATE_FREE = 0,
	STATE_READY,
	STATE_BUSY,
	STATE_ERROR,
};

static const char shortname [] = SHORTNAME;

MODULE_DESCRIPTION (SHORTNAME);
MODULE_AUTHOR ("Martin Bonnin");
MODULE_LICENSE ("GPL");

/*----------------------------------------------------------------------*/
static struct {
	unsigned short	vendor;
	unsigned short	product;
	char		*manufacturer_string;
	char		*product_string;
	char		*serial_string;
} mod_data = {	
	.vendor			= 0x0E79,
	.product		= 0x1333,
	.manufacturer_string	= "Manufacturer        ",
	.product_string		= "Product             ",
	.serial_string		= "1234567890123",
};

module_param_named(vendor, mod_data.vendor, ushort, S_IRUGO);
MODULE_PARM_DESC(vendor, "USB Vendor ID");

module_param_named(product, mod_data.product, ushort, S_IRUGO);
MODULE_PARM_DESC(product, "USB Product ID");

module_param_named(manufacturer_string, mod_data.manufacturer_string, charp, S_IRUGO);
MODULE_PARM_DESC(manufacturer_string, "USB descriptor manufacturer string (string 20 char max)");

module_param_named(product_string, mod_data.product_string, charp, S_IRUGO);
MODULE_PARM_DESC(product_string, "USB descriptor product string (string 20 char max)");

module_param_named(serial_string, mod_data.serial_string, charp, S_IRUGO);
MODULE_PARM_DESC(serial_string, "USB descriptor serial string (string 13 char max)");


/*----------------------------------------------------------------------*/

struct dev_data;


struct status_data {
	__le16  wLength;
	__le16	code;	
}__attribute__ ((packed));

struct mtpfs_buf {
	void				*buf;
	dma_addr_t			dma;

	struct usb_request		*req;

	int				state;

	struct completion		done;

	int				available;

	struct dev_data			*dev;
};

int _init_mtpfs_buf(struct mtpfs_buf *mbuf, struct usb_ep *ep)
{
	unsigned int order 	= get_order(BUFFER_SIZE);
	struct page *pa 	= alloc_pages(GFP_KERNEL, order);

	mbuf->buf = page_address(pa);
	mbuf->dma = DMA_ADDR_INVALID;

	mbuf->req = usb_ep_alloc_request(ep, GFP_ATOMIC);
	if( !mbuf->req )
		return 1;

	mbuf->req->buf 		= mbuf->buf;
	mbuf->req->dma 		= mbuf->dma;
	mbuf->req->context	= mbuf;

	return 0;
}

int _deinit_mtpfs_buf(struct mtpfs_buf *mbuf, struct usb_ep *ep)
{
	unsigned int order 	= get_order(BUFFER_SIZE);
	
	if( mbuf->buf )
		free_pages((unsigned long)mbuf->buf, order);

	if( mbuf->req )
		usb_ep_free_request(ep, mbuf->req);

	return 0;
}
/*----------------------------------------------------------------------*/

#define MTPFS_MAGIC		0xbbbbbbbb


struct dev_data {
	spinlock_t			lock;
	atomic_t			count;
	int				configured;
	int				canceling;

	struct usb_request		*ep0_req;
	u8				ep0_buf[512];
	
	struct mtpfs_buf		in_buf[ BUFFER_COUNT];
	struct mtpfs_buf		out_buf[BUFFER_COUNT];

	int				in_off;
	int				in_index;
	struct				usb_ep *in_ep;

	int				out_off;
	int				out_index;
	int				out_submit_index;
	struct				usb_ep *out_ep;
	wait_queue_head_t		out_wait;
	u32				out_transfer_length;
	u32				out_queued;
	u32				out_transfered;

	struct super_block		*sb;
	struct dentry			*dentry;
};

static struct dev_data		*the_device;

static inline void get_dev (struct dev_data *data)
{
	atomic_inc (&data->count);
}

static void put_dev (struct dev_data *data)
{
	if (likely (!atomic_dec_and_test (&data->count)))
		return;
	/* needs no more cleanup */
	BUG_ON (waitqueue_active (&data->out_wait));
	kfree (data);
}

static struct dev_data *dev_new (void)
{
	struct dev_data		*dev;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return NULL;

	memset(dev, 0, sizeof(*dev));

	atomic_set (&dev->count, 1);
	spin_lock_init (&dev->lock);
	init_waitqueue_head (&dev->out_wait);
	return dev;
}

/*----------------------------------------------------------------------*/

static void _queue_first_out_request(struct dev_data *dev)
{
	struct mtpfs_buf *mbuf = &dev->out_buf[dev->out_submit_index];

	int i;
	for( i = 0; i < BUFFER_COUNT; i++ ){
		if( dev->out_buf[dev->out_submit_index].state != STATE_FREE ){
			printk(KERN_ERR "get_first_out_request: queue still active !!!\n");
		}
	}

	if( mbuf->state == STATE_FREE ){
		mbuf->state	  	= STATE_BUSY;
		mbuf->req->length 	= BUFFER_SIZE;
		mbuf->req->short_not_ok = 0;

		init_completion(&mbuf->done);

DBGQ		printk( KERN_DEBUG "\t%p queue first\n", mbuf );

		if( usb_ep_queue(dev->out_ep, mbuf->req, GFP_ATOMIC) ){
			printk(KERN_DEBUG "out_queue error\n");
			mbuf->state = STATE_ERROR;
		}
		dev->out_submit_index++;
		if( dev->out_submit_index >= BUFFER_COUNT )
			dev->out_submit_index = 0;
	}
}


static void _fill_out_queue(struct dev_data *dev)
{
	int i;
	for( i = 0; i < BUFFER_COUNT; i++ ){
		struct mtpfs_buf *mbuf = &dev->out_buf[dev->out_submit_index];
		if( mbuf->state == STATE_FREE && dev->out_queued < dev->out_transfer_length ){
			mbuf->state	  	= STATE_BUSY;
			mbuf->req->length 	= BUFFER_SIZE;

			dev->out_queued		+= BUFFER_SIZE;

			if( /*1 ||*/ dev->out_queued >= dev->out_transfer_length ){
DBGQ				printk( KERN_DEBUG "\t%p queue mode 0 %u/%u\n", mbuf, dev->out_queued, dev->out_transfer_length);
				mbuf->req->short_not_ok = 0;
			} else {
DBGQ				printk( KERN_DEBUG "\t%p queue mode 1 %u/%u\n", mbuf, dev->out_queued, dev->out_transfer_length);
				mbuf->req->short_not_ok = 1;
			}

			init_completion(&mbuf->done);

			if( usb_ep_queue(dev->out_ep, mbuf->req, GFP_ATOMIC) ){
				printk(KERN_DEBUG "out_queue error\n");
				mbuf->state = STATE_ERROR;
			}
			dev->out_submit_index++;
			if( dev->out_submit_index >= BUFFER_COUNT )
				dev->out_submit_index = 0;
		}
	}
}

static void _flush_all_out_requests(struct dev_data *dev)
{
	int i;
	int err;

	// wait for all request and free them
	for( i = 0; i < BUFFER_COUNT; i++ ){
		int index = dev->out_index - 1 - i;
		struct mtpfs_buf *mbuf;
		if( index < 0 )
			index += BUFFER_COUNT;

		mbuf = &dev->out_buf[index];

		if( mbuf->state == STATE_BUSY ){
DBGQ			printk( KERN_DEBUG "\tin %p dequeue\n", mbuf );

			usb_ep_dequeue( dev->out_ep, mbuf->req );
		}
	}

	for( i = 0; i < BUFFER_COUNT; i++ ){
		struct mtpfs_buf *mbuf = &dev->out_buf[dev->out_index];

		printk( KERN_DEBUG "in flush %p state %d\n",  mbuf, mbuf->state);

		if( mbuf->state == STATE_FREE )
			continue;

		if( mbuf->state == STATE_BUSY ){

			err = wait_event_interruptible (mbuf->done.wait, mbuf->done.done);
			if( err ){
				printk("interrupted out 3 %d\n", i);
			}
		}

		if( mbuf->state != STATE_ERROR && mbuf->state != STATE_READY ){
			printk("in didn't wait long enough %d ?\n", i);
		}

		mbuf->state = STATE_FREE;

		dev->out_index++;
		if( dev->out_index >= BUFFER_COUNT )
			dev->out_index = 0;
	}
}

static void _flush_all_in_requests(struct dev_data *dev)
{
	int i;
	int err;

	// wait for all request and free them
	for( i = 0; i < BUFFER_COUNT; i++ ){
		int index = dev->in_index - 1 - i;
		struct mtpfs_buf *mbuf;
		if( index < 0 )
			index += BUFFER_COUNT;

		mbuf = &dev->in_buf[index];

		if( mbuf->state == STATE_BUSY ){
DBGQ			printk( KERN_DEBUG "\t%p dequeue\n", mbuf );

			usb_ep_dequeue( dev->in_ep, mbuf->req );
		}
	}

	// wait for all request and free them
	for( i = 0; i < BUFFER_COUNT; i++ ){
		struct mtpfs_buf *mbuf = &dev->in_buf[i];

		if( mbuf->state == STATE_FREE )
			continue;

		if( mbuf->state == STATE_BUSY ){
			usb_ep_dequeue( dev->in_ep, dev->in_buf[i].req );

			err = wait_event_interruptible (mbuf->done.wait, mbuf->done.done);
			if( err ){
				printk("interrupted in 3 %d\n", i);
			}
		}

		if( mbuf->state != STATE_ERROR && mbuf->state != STATE_READY ){
			printk("didn't wait long enough %d ?\n", i);
		}

		mbuf->state = STATE_FREE;

		dev->in_index++;
		if( dev->in_index >= BUFFER_COUNT )
			dev->in_index = 0;
	}
}

static ssize_t ep_out_read (struct file *fd, char __user *buf, size_t len, loff_t *ptr)
{
	int 	read = 0;
	struct 	dev_data *dev = fd->private_data;
	char	*p = buf;
	struct mtpfs_buf *mbuf;
	int err;

	if( !dev->configured )
		return -EAGAIN;

	printk(KERN_DEBUG "ep_out_read %d\n", len);

	if( !dev->out_transfer_length ){
		mbuf = &dev->out_buf[dev->out_index];

		err = wait_event_interruptible (mbuf->done.wait, mbuf->done.done);
		if( err ){
			printk("interrupted out 1\n");
			read = -ECANCELED;
			goto transfer_end;
		}
		if( mbuf->state != STATE_READY ){
			printk("cannot get transfer length\n");
			read = -ECANCELED;

			_flush_all_out_requests(dev);

			goto transfer_end;
		}
		memcpy(&dev->out_transfer_length, mbuf->buf, sizeof(dev->out_transfer_length));
		dev->out_queued = mbuf->available;

		printk(KERN_DEBUG "New OUT transfer %u bytes\n", dev->out_transfer_length);
	}

	while( read < len ){
		if( dev->canceling ){
			printk(KERN_DEBUG "ep_out_read canceling\n");

			_flush_all_out_requests(dev);

			dev->canceling = 0;

			read = -ECANCELED;
			goto transfer_end;
		} else {
			/*
			* Make sure the queue never empties
			*/
			_fill_out_queue(dev);
		} 

		/*
		* Read data we have available
		*/
		mbuf = &dev->out_buf[dev->out_index];

		if( mbuf->state == STATE_READY ){
			int to_read = min(len - read, (size_t)(mbuf->available - dev->out_off));
			memcpy(p, mbuf->buf + dev->out_off, to_read);
			p 	    += to_read;
			dev->out_off+= to_read;
			read        += to_read;

			dev->out_transfered += to_read;

		} else if( mbuf->state == STATE_BUSY ){
			struct timespec t;
			t.tv_sec  = 0;
			t.tv_nsec = 200 * 1000 * 1000;
			err = wait_event_interruptible_timeout(mbuf->done.wait, mbuf->done.done, timespec_to_jiffies(&t) + 1);

			continue;
		} else if( mbuf->state == STATE_ERROR ){
			printk(KERN_DEBUG "ep_out_read error %d\n", len);

			_flush_all_out_requests(dev);				

			read = -ECANCELED;
			goto transfer_end;
		} else {
			printk("Queue empty!\n");
		}

		if( dev->out_off == mbuf->available ){
			printk(KERN_DEBUG "ep_out_read %p used\n", mbuf);

			// we read the buffer completely, free it
			mbuf->state 	= STATE_FREE;
			mbuf->available = 0;

			dev->out_off = 0;
			dev->out_index ++;
			if( dev->out_index == BUFFER_COUNT )
				dev->out_index = 0;

			if( dev->out_transfered >= dev->out_transfer_length ){
				goto transfer_end;
			}
		}
	}

	printk(KERN_DEBUG "ep_out_read ok %d\n", read);

	return read;

transfer_end:

	printk(KERN_DEBUG "ep_out_read transfer done %d\n", read);

	dev->out_transfer_length 	= 0;
	dev->out_transfered 		= 0;

	_queue_first_out_request(dev);

	return read;
}

static ssize_t ep_in_write (struct file *fd, const char __user *buf, size_t len, loff_t *ptr)
{
	int 	written = 0;
	struct 	dev_data *dev = fd->private_data;
	const char	*p = buf;
	struct mtpfs_buf *mbuf;

	if( !dev->configured )
		return -EAGAIN;

	printk(KERN_DEBUG "ep_in_write %d\n", len);

	while( written < len ){

		/*
		* If we have a buffer free, fill it
		*/
		mbuf = &dev->in_buf[dev->in_index];

		if( dev->canceling ){
			_flush_all_in_requests(dev);

			dev->canceling = 0;

			written = -ECANCELED;
			goto transfer_end;
		}

		if( mbuf->state == STATE_FREE ){
			int to_write = min(len, (size_t)(BUFFER_SIZE - dev->in_off));
			memcpy(mbuf->buf + dev->in_off, p, to_write);
			p 	     += to_write;
			dev->in_off  += to_write;
			written      += to_write;

			if( dev->in_off == BUFFER_SIZE || written == len ){
				// we wrote the buffer completely, submit it
				mbuf->state 	= STATE_BUSY;

				mbuf->req->length = to_write;
				mbuf->req->short_not_ok = 1;

				init_completion(&mbuf->done);

DBGQ				printk( KERN_DEBUG "\tin %p queue\n", mbuf );

				if( usb_ep_queue(dev->in_ep, mbuf->req, GFP_ATOMIC) ){
					printk(KERN_DEBUG "in_queue error\n");
					mbuf->state = STATE_ERROR;
				}

				dev->in_off = 0;
				dev->in_index ++;
				if( dev->in_index == BUFFER_COUNT )
					dev->in_index = 0;
			}
		} else if( mbuf->state == STATE_BUSY ){
			struct timespec t;
			int err;
			t.tv_sec  = 0;
			t.tv_nsec = 200 * 1000 * 1000;

			err = wait_event_interruptible_timeout (mbuf->done.wait, mbuf->done.done, timespec_to_jiffies(&t) + 1);
		} else if( mbuf->state == STATE_ERROR ){
			// error, ackowledge it and continue
			printk("MTPFS: In error\n");

			_flush_all_in_requests(dev);

			written = -ECANCELED;
			goto transfer_end;
		} else if( mbuf->state == STATE_READY ){
			mbuf->state = STATE_FREE;
		}
	}

	printk(KERN_DEBUG "ep_in_write ok %d\n", written);

	return written;

transfer_end:
	printk(KERN_DEBUG "ep_in_write transfer ok %d\n", written);

	return written;
}

static int ep_release (struct inode *inode, struct file *fd)
{
	printk("ep_release\n");

	return 0;
}

static int ep_ioctl (struct inode *inode, struct file *fd,
		unsigned code, unsigned long value)
{
	printk("ep_ioctl\n");

	return 0;
}

static int ep_out_open (struct inode *inode, struct file *fd)
{
	printk("ep_out_open\n");

	fd->private_data = inode->i_private;

	return 0;
}

static int ep_in_open (struct inode *inode, struct file *fd)
{
	printk("ep_in_open\n");

	fd->private_data = inode->i_private;

	return 0;
}

static unsigned int ep_out_poll (struct file *fd, poll_table *wait)
{
	struct dev_data         *dev = fd->private_data;
	int                     mask = 0;

DBGP	printk(KERN_DEBUG "ep_out_poll %lu\n", jiffies);

	poll_wait(fd, &dev->out_wait, wait);

	if( !dev->configured ){
DBGP		printk(KERN_DEBUG "ep_out_poll unconfigured\n");
	} else if( dev->out_buf[dev->out_index].state == STATE_READY ){
DBGP		printk(KERN_DEBUG "ep_out_poll ready \n");
		mask = POLLIN;
	} else if( dev->out_buf[dev->out_index].state == STATE_ERROR ){
DBGP		printk(KERN_DEBUG "ep_out_poll error\n");
	} else if( dev->out_buf[dev->out_index].state == STATE_BUSY ){
DBGP		printk(KERN_DEBUG "ep_out_poll busy\n");
	} else {
DBGP		printk(KERN_DEBUG "ep_out_poll ???\n");
	}

DBGP	printk(KERN_DEBUG "ep_out_poll -> %d\n", mask);
	return mask;
}

static const struct file_operations ep_in_operations = {
	.owner =	THIS_MODULE,
	.llseek =	no_llseek,

	.open =		ep_in_open,
	.write =	ep_in_write,
	.ioctl =	ep_ioctl,
	.release =	ep_release,
};

static const struct file_operations ep_out_operations = {
	.owner =	THIS_MODULE,
	.llseek =	no_llseek,

	.open =		ep_out_open,
	.read =		ep_out_read,
	.poll =		ep_out_poll,
	.ioctl =	ep_ioctl,
	.release =	ep_release,
};

/*-------------------------------------------------------------------------*/


#define	STRINGID_MFGR		1
#define	STRINGID_PRODUCT	2
#define	STRINGID_SERIAL		3

static struct usb_string stringtab [4];

static void _fixup_stringtab(void)
{
	stringtab[0].id = STRINGID_MFGR;
	stringtab[0].s  = mod_data.manufacturer_string;
	stringtab[1].id = STRINGID_PRODUCT;
	stringtab[1].s  = mod_data.product_string;
	stringtab[2].id = STRINGID_SERIAL;
	stringtab[2].s  = mod_data.serial_string;
}

static struct usb_gadget_strings strings = {
	.language =	0x0409,		/* "en-us" */
	.strings =	stringtab,
};

/*-------------------------------------------------------------------------*/

#define bMS_Length 0x12
// MTP enumeration
static const u8 ms_os_string_desc[bMS_Length] = {
	bMS_Length,
	USB_DT_STRING,
	'M',0,'S',0,'F',0,'T',0,'1',0,'0',0,'0',0,
	MTP_GET_MS_DESCRIPTOR,0
};
#undef bMS_Length

static const u8 ms_os_ext_conf_desc[] = {
				// Header
	40,0,0,0,		// DWORD length
	0x00, 0x01,		// WORD Version in BCD little endian (i.e. 1.00)
	0x04, 0x00,		// WORD MSOS_FEATURE_INDEX_EXTENDED_CONFIG_DESC    in little endian
	1,			// BYTE total number of Function Sections that follow the header section
	0,0,0,0,0,0,0,		// BYTE[7] reserved

				// Function
	0,			// BYTE starting at interface
	1,			// BYTE Number of interfaces
	'M','T','P', 0,0,0,0,0,	// BYTE[8] compatible ID
	0,0,0,0,0,0,0,0,	// BYTE[8] sub compatible ID
	0,0,0,0,0,0, 		// BYTE[6] reserved
};

/*-------------------------------------------------------------------------*/

/* Descriptors */

static struct usb_device_descriptor device_desc = {
	.bLength =		sizeof(device_desc),
	.bDescriptorType =	USB_DT_DEVICE,

	.bcdUSB =		cpu_to_le16 (0x0200),
	.bDeviceClass =		USB_CLASS_PER_INTERFACE,
	.bDeviceSubClass =	0,
	.bDeviceProtocol =	0,
	.bcdDevice =		cpu_to_le16 (0x0100),
	.iManufacturer =	STRINGID_MFGR,
	.iProduct =		STRINGID_PRODUCT,
	.iSerialNumber =	STRINGID_SERIAL,
	.bNumConfigurations =	1,
};

static struct usb_qualifier_descriptor	qualifier_desc = {
	.bLength =		sizeof(qualifier_desc),
	.bDescriptorType =	USB_DT_DEVICE_QUALIFIER,
	.bcdUSB = 		cpu_to_le16 (0x0200),
	.bDeviceClass =		USB_CLASS_PER_INTERFACE,
	.bDeviceSubClass =	0,
	.bDeviceProtocol =	0,
	.bMaxPacketSize0 = 	64,
	.bNumConfigurations =	1,
	.bRESERVED = 		0,
};

#define	MAX_USB_POWER		500

#define	CONFIG_VALUE		4

static struct usb_config_descriptor config_desc = {
	.bLength =		sizeof config_desc,
	.bDescriptorType =	USB_DT_CONFIG,
	.bNumInterfaces =	1,
	.bConfigurationValue =	CONFIG_VALUE,
	.iConfiguration =	0,
	.bmAttributes =		USB_CONFIG_ATT_ONE,
	.bMaxPower =		(MAX_USB_POWER + 1) / 2,
};

static struct usb_interface_descriptor interface_desc = {
	.bLength =		sizeof(interface_desc),
	.bDescriptorType =	USB_DT_INTERFACE,
	.bInterfaceNumber =	0,
	.bAlternateSetting = 	0,
	.bNumEndpoints =	ENDPOINT_COUNT,
	.bInterfaceClass =	USB_CLASS_MASS_STORAGE, 
	.bInterfaceSubClass =   USB_SC_SCSI,
	.bInterfaceProtocol =   USB_PR_BULK,
	.iInterface =		0,
};

static struct usb_endpoint_descriptor bulkin_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN | 1,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor bulkout_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT | 2,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

#ifdef STATUS_SUPPORT
static struct usb_endpoint_descriptor status_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN | 3,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	cpu_to_le16 (64),
	.bInterval =		4,
};
#endif

static void _fixup_device_desc(void)
{
	device_desc.bMaxPacketSize0 	= 64;
	device_desc.idVendor		= cpu_to_le16(mod_data.vendor);
	device_desc.idProduct		= cpu_to_le16(mod_data.product);
}

static void _fixup_qualifier_desc(void)
{
	//nothing to do here
}

static void _fixup_config_desc(int hs)
{
	int total = sizeof(config_desc) + sizeof(interface_desc) + ENDPOINT_COUNT * USB_DT_ENDPOINT_SIZE;

	config_desc.wTotalLength = cpu_to_le16(total);

	bulkin_desc.wMaxPacketSize  = hs ? cpu_to_le16(512) : cpu_to_le16(64);
	bulkout_desc.wMaxPacketSize = hs ? cpu_to_le16(512) : cpu_to_le16(64);
}

/*----------------------------------------------------------------------*/

/* Gadget Driver
 *
 */
static void _ep0_complete (struct usb_ep *ep, struct usb_request *req)
{
}

static void _in_complete (struct usb_ep *ep, struct usb_request *req)
{
	struct mtpfs_buf *mbuf = req->context;

DBGQ	printk(KERN_DEBUG "\tin %p complete %8d (%d)\n", mbuf, req->actual, req->status);

	if( req->status ){
		mbuf->state = STATE_ERROR;
	} else {
		mbuf->state = STATE_READY;
	}

	mbuf->available = req->actual;

	complete (&mbuf->done);
}

static void _out_complete (struct usb_ep *ep, struct usb_request *req)
{
	struct mtpfs_buf *mbuf = req->context;

DBGQ	printk(KERN_DEBUG "\t%p complete %8d (%d)\n", mbuf, req->actual, req->status);

	if( req->status ){
		mbuf->state = STATE_ERROR;
	} else {
		mbuf->state = STATE_READY;
	}

	mbuf->available = req->actual;

	wake_up(&mbuf->dev->out_wait);

	complete (&mbuf->done);
}

static int _init_bulkin(struct dev_data *dev, struct usb_ep *ep)
{
	int i;

	for( i = 0; i < BUFFER_COUNT ; i++ ){
		if( _init_mtpfs_buf(&dev->in_buf[i], ep) ){
			return 1;
		}

		dev->in_buf[i].req->complete = _in_complete;
		dev->in_buf[i].state = STATE_FREE;

		dev->in_buf[i].dev = dev;
	}

	dev->in_ep = ep;

	return 0;
}

static int _init_bulkout(struct dev_data *dev, struct usb_ep *ep)
{
	int i;

	for( i = 0; i < BUFFER_COUNT ; i++ ){
		if( _init_mtpfs_buf(&dev->out_buf[i], ep) ){
			return 1;
		}

		dev->out_buf[i].req->complete = _out_complete;
		dev->out_buf[i].state = STATE_FREE;

		dev->out_buf[i].dev = dev;
	}

	dev->out_ep = ep;

	return 0;
}

static int _deinit_bulkin(struct dev_data *dev, struct usb_ep *ep)
{
	int i;

	for( i = 0; i < BUFFER_COUNT ; i++ ){
		if( _deinit_mtpfs_buf(&dev->in_buf[i], ep) ){
			return 1;
		}
	}

	return 0;
}

static int _deinit_bulkout(struct dev_data *dev, struct usb_ep *ep)
{
	int i;

	for( i = 0; i < BUFFER_COUNT ; i++ ){
		if( _deinit_mtpfs_buf(&dev->out_buf[i], ep) ){
			return 1;
		}
	}

	return 0;
}

static int _init_ep0(struct dev_data *dev, struct usb_ep *ep)
{
	dev->ep0_req = usb_ep_alloc_request(ep, GFP_KERNEL);

	dev->ep0_req->buf 		= dev->ep0_buf;
	dev->ep0_req->dma 		= DMA_ADDR_INVALID;

	dev->ep0_req->no_interrupt	= 0;
	dev->ep0_req->short_not_ok	= 0;
	dev->ep0_req->zero		= 0;

	dev->ep0_req->complete		= _ep0_complete;
	dev->ep0_req->context		= NULL;

	return 0;
}

static int _deinit_ep0(struct dev_data *dev, struct usb_ep *ep)
{
	usb_ep_free_request (ep, dev->ep0_req);

	return 0;
}

static int mtpfs_bind (struct usb_gadget *gadget)
{
	struct dev_data	 *dev = the_device;
	struct usb_ep	 *ep;

	printk("mtpfs_bind\n");

	if (!dev)
		return -ESRCH;

	set_gadget_data (gadget, dev);

	_init_ep0(dev, gadget->ep0);

	gadget_for_each_ep (ep, gadget) {

		if( !strcmp(ep->name, "ep1in") ){
			printk("+bulkin\n");
			if( _init_bulkin(dev, ep) ){
				printk("cannot init bulkin\n");
				return -ENOMEM;
			}
		} else if( !strcmp(ep->name, "ep2out") ){
			printk("+bulkout\n");
			if( _init_bulkout(dev, ep) ){
				printk("cannot init bulkout\n");
				return -ENOMEM;
			}
		}
	}

	return 0;
}

static void mtpfs_unbind (struct usb_gadget *gadget)
{
	struct dev_data	*dev = get_gadget_data (gadget);
	struct usb_ep	 *ep;

	gadget_for_each_ep (ep, gadget) {

		if( !strcmp(ep->name, "ep1in") ){
			printk("-bulkin\n");
			if( _deinit_bulkin(dev, ep) ){
				printk("cannot deinit bulkin\n");
			}
		} else if( !strcmp(ep->name, "ep2out") ){
			printk("-bulkout\n");
			if( _deinit_bulkout(dev, ep) ){
				printk("cannot deinit bulkout\n");
			}
		}
	}

	_deinit_ep0(dev, gadget->ep0);

	set_gadget_data (gadget, NULL);

	printk("mtpfs_unbind\n");
}

static int mtpfs_setup (struct usb_gadget *gadget, const struct usb_ctrlrequest *ctrl)
{
	u16	w_value 	= le16_to_cpu(ctrl->wValue);
	u16	w_length 	= le16_to_cpu(ctrl->wLength);

	int	desc_type	= w_value >> 8;
	int	desc_index	= w_value & 0xff;

	int	transfer_size	= -1;
	void	*buffer 	= NULL;

	u8	scratch_buffer[512];
	u8	*p;

	int	config_size;
	int	hs = 1;
	int	status;

	struct dev_data	*dev = get_gadget_data (gadget);

	switch (ctrl->bRequest) {

	case USB_REQ_GET_DESCRIPTOR:
		if (ctrl->bRequestType != USB_DIR_IN)
			return -EOPNOTSUPP;

		switch (desc_type) {

		case USB_DT_DEVICE:
			_fixup_device_desc();
			transfer_size 	= min (w_length, (u16) sizeof(device_desc));
			buffer 		= &device_desc;
			break;
#ifdef	CONFIG_USB_GADGET_DUALSPEED
		case USB_DT_DEVICE_QUALIFIER:
			_fixup_qualifier_desc();
			transfer_size	= min (w_length, (u16) sizeof(qualifier_desc));
			buffer		= &qualifier_desc;
			break;
		case USB_DT_OTHER_SPEED_CONFIG:
			// FALLTHROUGH
#endif
		case USB_DT_CONFIG:
			config_size = sizeof(config_desc) + sizeof(interface_desc) + ENDPOINT_COUNT * USB_DT_ENDPOINT_SIZE;

			if( config_size > sizeof(scratch_buffer) ){
				printk("scratch bug too small (%d > %d)\n", config_size, sizeof(scratch_buffer));
				return -EINVAL;
			}

			/* only one configuration */
			if (desc_index > 0){
				printk("bad configuration value %d\n", desc_index);
				return -EINVAL;
			}

			hs = (gadget->speed == USB_SPEED_HIGH);

			if( desc_type == USB_DT_OTHER_SPEED_CONFIG )
				hs = !hs;

			_fixup_config_desc(hs);
			
			p = scratch_buffer;
			memcpy(p, &config_desc, sizeof(config_desc));
			p += sizeof(config_desc);
			memcpy(p, &interface_desc, sizeof(interface_desc));
			p += sizeof(interface_desc);
			memcpy(p, &bulkin_desc, USB_DT_ENDPOINT_SIZE);
			p += USB_DT_ENDPOINT_SIZE;
			memcpy(p, &bulkout_desc, USB_DT_ENDPOINT_SIZE);
			p += USB_DT_ENDPOINT_SIZE;
#ifdef STATUS_SUPPORT
			memcpy(p, &status_desc, USB_DT_ENDPOINT_SIZE);
			p += USB_DT_ENDPOINT_SIZE;
#endif
			transfer_size = min (w_length, (u16) config_size);
			buffer = scratch_buffer;

			break;
		case USB_DT_STRING:
			if ( desc_index == 0xEE ) {
				printk("MTPFS: os string desciptor\n");
				transfer_size = min (w_length, (u16) sizeof(ms_os_string_desc));
				buffer = &ms_os_string_desc;
			} else {
				if (desc_index != 0 && ctrl->wIndex != strings.language)
					return -EINVAL;

				_fixup_stringtab();

				transfer_size = usb_gadget_get_string (&strings, desc_index, scratch_buffer);

				if( transfer_size < 0 )
					return -EINVAL;

				transfer_size = min((u16)transfer_size, w_length);
				buffer = scratch_buffer;
			}
			break;

		default:		// all others are errors
			printk("MTPFS: unknown descriptor %d\n", desc_type);
			break;
		}
		break;

	/* currently one config, two speeds */
	case USB_REQ_SET_CONFIGURATION:
		if (ctrl->bRequestType != 0)
			break;

		if( usb_ep_enable(dev->in_ep, &bulkin_desc) ){
			printk("cannot enable bulkin\n");
		}
		if( usb_ep_enable(dev->out_ep, &bulkout_desc) ){
			printk("cannot enable bulkout\n");
		}

		if (0 == (u8) w_value) {
			printk("MTPFS: configuration #0\n");
			dev->configured = 0;
		} else {
			printk("MTPFS: configuration #%d\n", w_value);
			dev->configured = 1;
			_queue_first_out_request(dev);
		}
		break;

	case USB_REQ_GET_CONFIGURATION:
		if (ctrl->bRequestType != USB_DIR_IN)
			break;
		scratch_buffer[0] = config_desc.bConfigurationValue;

		transfer_size = min (w_length, (u16) 1);
		buffer = scratch_buffer;

		break;

	case USB_REQ_GET_INTERFACE:
		if (!(ctrl->bRequestType & USB_DIR_IN))
			break;
		scratch_buffer[0] = interface_desc.bInterfaceNumber;

		transfer_size = min (w_length, (u16) 1);
		buffer = scratch_buffer;

		break;

	case USB_REQ_SET_INTERFACE:
		printk("MTPFS: set interface\n");
		break;


	case PTP_REQUEST_CANCEL:
		printk("MTPFS: cancel %d bytes\n", w_length);
		dev->canceling = 1;

		transfer_size = w_length;
		buffer = scratch_buffer;

		break;
	case PTP_REQUEST_GET_DEVICE_STATUS: {
		static int status_count = 0;
		struct status_data ptp_status;
		printk("MTPFS: device status\n");
		ptp_status.wLength = cpu_to_le16(4);

		if( dev->canceling ){
			status_count = 5;
		}

		if( status_count > 0 )
			status_count--;

		ptp_status.code = status_count ? cpu_to_le16(PTP_RC_BUSY) : cpu_to_le16(PTP_RC_OK);

		memcpy(scratch_buffer, &ptp_status, 4);

		transfer_size = min (w_length, (u16)4);
		buffer = scratch_buffer;
		break;
		}
	case PTP_REQUEST_GET_EXTENDED_EVENT_DATA:
		printk("MTPFS: extended event data\n");
		break;
	case PTP_REQUEST_DEVICE_RESET: {
		struct status_data ptp_status;
		printk("MTPFS: device reset\n");
		ptp_status.wLength = cpu_to_le16(4);
		ptp_status.code    = cpu_to_le16(PTP_RC_OK);
		memcpy(scratch_buffer, &ptp_status, 4);

		transfer_size = min (w_length, (u16)4);
		buffer = scratch_buffer;
		break;
		}
		break;


	case MTP_GET_MS_DESCRIPTOR:
		printk("MTPFS: os extended config\n");
		transfer_size = min (w_length, (u16) sizeof(ms_os_ext_conf_desc));
		buffer = &ms_os_ext_conf_desc;
		break;
	default:
		printk("MTPFS: unknown request %d\n", ctrl->bRequest);

	}


	if( w_length == 0 ){
		transfer_size = 0;
	}

	if( transfer_size > MAX_EP0_TRANSFER ){
		return -EINVAL;
	}

	if( transfer_size < 0 ){
		return -EINVAL;
	}

	if( transfer_size > 0 && !buffer ){
		printk("you did not setup the buffer\n");
		return -EINVAL;
	}

	memcpy(dev->ep0_buf, buffer, transfer_size);
	dev->ep0_req->length = transfer_size;

	status = usb_ep_queue(gadget->ep0, dev->ep0_req, GFP_ATOMIC);

	return status;
}

static void mtpfs_disconnect (struct usb_gadget *gadget)
{

	printk("mtpfs_disconnect\n");
}

static void mtpfs_suspend (struct usb_gadget *gadget)
{
	struct dev_data	*dev = get_gadget_data (gadget);

	printk("mtpfs_suspend\n");

	dev->canceling = 1;
}

static void mtpfs_resume (struct usb_gadget *gadget)
{
	printk("mtpfs_resume\n");
}

static struct usb_gadget_driver mtpfs_driver = {
	.speed		= USB_SPEED_HIGH,
	.function	= (char *) shortname,
	.bind		= mtpfs_bind,
	.unbind		= mtpfs_unbind,
	.setup		= mtpfs_setup,
	.disconnect	= mtpfs_disconnect,
	.suspend	= mtpfs_suspend,
	.resume		= mtpfs_resume,

	.driver	= {
		.name		= (char *) shortname,
	},
};

/*----------------------------------------------------------------------*/

/* FILESYSTEM AND SUPERBLOCK OPERATIONS
 *
 */
static unsigned default_uid;
static unsigned default_gid;
static unsigned default_perm = S_IRUSR | S_IWUSR;

module_param (default_uid, uint, 0644);
module_param (default_gid, uint, 0644);
module_param (default_perm, uint, 0644);

static struct inode *_create_file (struct super_block *sb, char const *name,
		const struct file_operations *ops,
		void *data, struct dentry **dentry_p)
{
	struct dentry	*dentry;
	struct inode	*inode;

	dentry = d_alloc_name(sb->s_root, name);
	if (!dentry)
		return NULL;

	inode = new_inode (sb);

	if (!inode) {
		dput(dentry);
		return NULL;
	}

	inode->i_mode 		= S_IFREG | (default_perm & S_IRWXUGO);
	inode->i_uid 		= default_uid;
	inode->i_gid 		= default_gid;
	inode->i_blocks 	= 0;
	inode->i_atime 		= inode->i_mtime = inode->i_ctime
				= CURRENT_TIME;
	inode->i_private 	= data;
	inode->i_fop 		= ops;

	d_add (dentry, inode);
	*dentry_p = dentry;
	return inode;
}

static struct super_operations gadget_fs_operations = {
	.statfs 	= simple_statfs,
	.drop_inode 	= generic_delete_inode,
};

static int
mtpfs_fill_super (struct super_block *sb, void *opts, int silent)
{
	struct inode	*inode;
	struct dentry	*d;
	struct dev_data	*dev;

	int status;

	if (the_device)
		return -ESRCH;

	dev = dev_new ();
	if (!dev)
		goto enomem0;

	the_device = dev;

	status = usb_gadget_register_driver (&mtpfs_driver);
	if (status != 0){
		printk("cannot register gadget driver\n");
		goto enomem1;
	}	

	/* superblock */
	sb->s_blocksize 	= PAGE_CACHE_SIZE;
	sb->s_blocksize_bits 	= PAGE_CACHE_SHIFT;
	sb->s_magic 		= MTPFS_MAGIC;
	sb->s_op 		= &gadget_fs_operations;
	sb->s_time_gran 	= 1;

	/* root inode */
	inode = new_inode (sb);

	if (!inode)
		goto enomem2;

	inode->i_mode 	= S_IFDIR | S_IRUGO | S_IXUGO;
	inode->i_uid 	= default_uid;
	inode->i_gid 	= default_gid;
	inode->i_blocks = 0;
	inode->i_atime 	= inode->i_mtime = inode->i_ctime
			= CURRENT_TIME;
	inode->i_private= NULL;
	inode->i_fop 	= &simple_dir_operations;
	inode->i_op 	= &simple_dir_inode_operations;

	if (!(d = d_alloc_root (inode)))
		goto enomem3;

	sb->s_root = d;

	dev->sb = sb;

	if (!_create_file (sb, "bulkin", &ep_in_operations, dev, &dev->dentry))
		goto enomem4;

	if (!_create_file (sb, "bulkout", &ep_out_operations, dev, &dev->dentry))
		goto enomem4;

	return 0;

enomem4:
	dput (d);
enomem3:
	iput (inode);
enomem2:
	usb_gadget_unregister_driver(&mtpfs_driver);
enomem1:
	put_dev (dev);
enomem0:
	return -ENOMEM;
}

static int
mtpfs_get_sb (struct file_system_type *t, int flags,
		const char *path, void *opts, struct vfsmount *mnt)
{
	return get_sb_single (t, flags, opts, mtpfs_fill_super, mnt);
}

static void
mtpfs_kill_sb (struct super_block *sb)
{
	kill_litter_super (sb);

	if (the_device) {
		usb_gadget_unregister_driver(&mtpfs_driver);

		put_dev (the_device);
		the_device = NULL;
	}
}

static struct file_system_type mtpfs_type = {
	.owner		= THIS_MODULE,
	.name		= shortname,
	.get_sb		= mtpfs_get_sb,
	.kill_sb	= mtpfs_kill_sb,
};

/*----------------------------------------------------------------------*/

/* module */

static int __init init (void)
{
	int status;

	status = register_filesystem (&mtpfs_type);
	if (status != 0){
		return status;
	}

	pr_info("mtpfs registered\n");

	return status;
}

module_init (init);

static void __exit cleanup (void)
{
	usb_gadget_unregister_driver (&mtpfs_driver);

	unregister_filesystem (&mtpfs_type);
	pr_info("mtpfs unregistered\n");
}

module_exit (cleanup);

