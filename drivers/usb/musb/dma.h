/******************************************************************
 * Copyright 2005 Mentor Graphics Corporation
 * Copyright (C) 2005-2006 by Texas Instruments
 *
 * This file is part of the Inventra Controller Driver for Linux.
 *
 * The Inventra Controller Driver for Linux is free software; you
 * can redistribute it and/or modify it under the terms of the GNU
 * General Public License version 2 as published by the Free Software
 * Foundation.
 *
 * The Inventra Controller Driver for Linux is distributed in
 * the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with The Inventra Controller Driver for Linux ; if not,
 * write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307  USA
 *
 * ANY DOWNLOAD, USE, REPRODUCTION, MODIFICATION OR DISTRIBUTION
 * OF THIS DRIVER INDICATES YOUR COMPLETE AND UNCONDITIONAL ACCEPTANCE
 * OF THOSE TERMS.THIS DRIVER IS PROVIDED "AS IS" AND MENTOR GRAPHICS
 * MAKES NO WARRANTIES, EXPRESS OR IMPLIED, RELATED TO THIS DRIVER.
 * MENTOR GRAPHICS SPECIFICALLY DISCLAIMS ALL IMPLIED WARRANTIES
 * OF MERCHANTABILITY; FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT.  MENTOR GRAPHICS DOES NOT PROVIDE SUPPORT
 * SERVICES OR UPDATES FOR THIS DRIVER, EVEN IF YOU ARE A MENTOR
 * GRAPHICS SUPPORT CUSTOMER.
 ******************************************************************/

#ifndef __MUSB_DMA_H__
#define __MUSB_DMA_H__

struct musb_hw_ep;

/*
 * DMA Controller Abstraction
 *
 * DMA Controllers are abstracted to allow use of a variety of different
 * implementations of DMA, as allowed by the Inventra USB cores.  On the
 * host side, usbcore sets up the DMA mappings and flushes caches; on the
 * peripheral side, the gadget controller driver does.  Responsibilities
 * of a DMA controller driver include:
 *
 *  - Handling the details of moving multiple USB packets
 *    in cooperation with the Inventra USB core, including especially
 *    the correct RX side treatment of short packets and buffer-full
 *    states (both of which terminate transfers).
 *
 *  - Knowing the correlation between dma channels and the
 *    Inventra core's local endpoint resources and data direction.
 *
 *  - Maintaining a list of allocated/available channels.
 *
 *  - Updating channel status on interrupts,
 *    whether shared with the Inventra core or separate.
 */

#define	DMA_ADDR_INVALID	(~(dma_addr_t)0)

#ifndef CONFIG_USB_INVENTRA_FIFO
#define	is_dma_capable()	(1)
#else
#define	is_dma_capable()	(0)
#endif

#ifdef CONFIG_USB_TI_CPPI_DMA
#define	is_cppi_enabled()	1
#else
#define	is_cppi_enabled()	0
#endif

#ifdef CONFIG_USB_TUSB_OMAP_DMA
#define tusb_dma_omap()			1
#else
#define tusb_dma_omap()			0
#endif

/*
 * DMA channel status ... updated by the dma controller driver whenever that
 * status changes, and protected by the overall controller spinlock.
 */
enum dma_channel_status {
	/* unallocated */
	MGC_DMA_STATUS_UNKNOWN,
	/* allocated ... but not busy, no errors */
	MGC_DMA_STATUS_FREE,
	/* busy ... transactions are active */
	MGC_DMA_STATUS_BUSY,
	/* transaction(s) aborted due to ... dma or memory bus error */
	MGC_DMA_STATUS_BUS_ABORT,
	/* transaction(s) aborted due to ... core error or USB fault */
	MGC_DMA_STATUS_CORE_ABORT
};

struct dma_controller;

/**
 * struct dma_channel - A DMA channel.
 * @pPrivateData: channel-private data
 * @wMaxLength: the maximum number of bytes the channel can move in one
 *	transaction (typically representing many USB maximum-sized packets)
 * @dwActualLength: how many bytes have been transferred
 * @bStatus: current channel status (updated e.g. on interrupt)
 * @bDesiredMode: TRUE if mode 1 is desired; FALSE if mode 0 is desired
 *
 * channels are associated with an endpoint for the duration of at least
 * one usb transfer.
 */
struct dma_channel {
	void			*pPrivateData;
	// FIXME not void* private_data, but a dma_controller *
	size_t			dwMaxLength;
	size_t			dwActualLength;
	enum dma_channel_status	bStatus;
	u8			bDesiredMode;
};

/*
 * Program a DMA channel to move data at the core's request.
 * The local core endpoint and direction should already be known,
 * since they are specified in the channel_alloc call.
 *
 * @channel: pointer to a channel obtained by channel_alloc
 * @maxpacket: the maximum packet size
 * @bMode: TRUE if mode 1; FALSE if mode 0
 * @dma_addr: base address of data (in DMA space)
 * @length: the number of bytes to transfer; no larger than the channel's
 *	reported dwMaxLength
 *
 * Returns TRUE on success, else FALSE
 */
typedef int (*MGC_pfDmaProgramChannel) (
		struct dma_channel	*channel,
		u16			maxpacket,
		u8			bMode,
		dma_addr_t		dma_addr,
		u32			length);

/*
 * dma_channel_status - return status of dma channel
 * @c: the channel
 *
 * Returns the software's view of the channel status.  If that status is BUSY
 * then it's possible that the hardware has completed (or aborted) a transfer,
 * so the driver needs to update that status.
 */
static inline enum dma_channel_status
dma_channel_status(struct dma_channel *c)
{
	return (is_dma_capable() && c) ? c->bStatus : MGC_DMA_STATUS_UNKNOWN;
}

/**
 * struct dma_controller - A DMA Controller.
 * @pPrivateData: controller-private data;
 * @start: call this to start a DMA controller;
 *	return 0 on success, else negative errno
 * @stop: call this to stop a DMA controller
 *	return 0 on success, else negative errno
 * @channel_alloc: call this to allocate a DMA channel
 * @channel_release: call this to release a DMA channel
 * @channel_abort: call this to abort a pending DMA transaction,
 *	returning it to FREE (but allocated) state
 *
 * Controllers manage dma channels.
 */
struct dma_controller {
	void			*pPrivateData;
	int			(*start)(struct dma_controller *);
	int			(*stop)(struct dma_controller *);
	struct dma_channel	*(*channel_alloc)(struct dma_controller *,
					struct musb_hw_ep *, u8 is_tx);
	void			(*channel_release)(struct dma_channel *);
	MGC_pfDmaProgramChannel	channel_program;
	int			(*channel_abort)(struct dma_channel *);
};

/* called after channel_program(), may indicate a fault */
extern void musb_dma_completion(struct musb *musb, u8 bLocalEnd, u8 bTransmit);


extern struct dma_controller *__init
dma_controller_create(struct musb *, void __iomem *);

extern void dma_controller_destroy(struct dma_controller *);

#endif	/* __MUSB_DMA_H__ */
