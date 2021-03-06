/*
 *  Copyright (C) 2007 
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or (at
 *  your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#ifndef _LINUX_HSDPA_H
#define _LINUX_HSDPA_H

struct hsdpa_io_fops {
	void (* hsdpa_usb_enable)( int high_level );
	void (* hsdpa_reset)( int high_level );
	void (* hsdpa_power)( int high_level );
};


struct hsdpa_io_fops *archosg6_hsdpa_get_io(void);

/* IOCTL commands. */
#define HSDPA_CMD_SET_POWER		1
#define HSDPA_CMD_SET_USB_ENABLE	2
#define HSDPA_CMD_SET_RESET		3

#endif
