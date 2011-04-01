/*
 *  linux/arch/arm/mach-davinci/archosg6-irr.c
 *
 * DESCRIPTION
 *   Initialization GP Timer9 by dmtimer for IR remote driver
 *
 * Copyright (C) 2007 Archos SA.
 * Author:
 *         Jinqin DONG 
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
 */
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <asm/hardware.h>
#include <asm/arch/dmtimer.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/dmtimer.h>
#include <asm/arch/mux.h>
#include <asm/arch/clock.h>

static struct resource irremote_resources[] = {
	[0] = {
		.start	= INT_24XX_GPTIMER9,
		.end	= INT_24XX_GPTIMER9,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device irremote_device = {
	.name		= "archos-ir",
	.id		= -1,
	.dev		= {
		.platform_data	= NULL,
	},
	.num_resources	= 1,
	.resource	= irremote_resources,
};

static int __init archosg6_irr_init(void)
{
	return platform_device_register(&irremote_device);
}

arch_initcall(archosg6_irr_init);
