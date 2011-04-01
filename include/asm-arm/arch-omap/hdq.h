/*
 * include/asm-arm/arch-omap2/hdq.h
 *
 * Copyright (C) 2006 Texas Instruments, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License 
 * version 2. This program is licensed "as is" without any warranty of any 
 * kind, whether express or implied.
 *
 */

#ifndef __ASM_OMAP2_HDQ_H__
#define __ASM_OMAP2_HDQ_H__

/* Note the following API has to be called in a process context */

int omap_hdq_get(void); /* requset the HDQ block */
int omap_hdq_put(void); /* release the HDQ block */

int omap_hdq_reset(void); /* reset the HDQ block */
int omap_hdq_break(void);  /* reset the salve by sending it a break pulse */

int omap_hdq_write(u8 reg, u8 val); /* write a byte to a slave register */
int omap_hdq_read(u8 reg, u8 *val); /* read a byte from a slave register */

void omap_hdq_reg_dump(void); /* dump the HDQ registers */

#endif
