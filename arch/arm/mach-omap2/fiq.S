/*
 *  linux/arch/arm/mach-omap2/gp-irblaster_fiq.S
 *
 * BRIEF MODULE DESCRIPTION
 *   FIQ handler function
 *
 * Copyright (C) 2004 Archos SA.
 * Author:
 *
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

#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/hardware.h>
#include <asm/arch/irqs.h>

#define GPT02_STAT_V	IO_ADDRESS(0x49032018)
#define GPT02_CTRL_V	IO_ADDRESS(0x49032024)
#define GPT02_PEND_V	IO_ADDRESS(0x49032034)
#define GPT_STOP_MASK	0xFFFFFFFE
#define IRQ_INTC_V	IO_ADDRESS(0x48200048)
#define INTCPS_SIR_FIQ  0x48200044
#define INTCPS_SIR_FIQ_V  IO_ADDRESS(0x48200044)

#define INTC_PENDING_FIQ_0_V INTC_PENDING_FIQ_0
#define INTC_PENDING_FIQ_1_V INTC_PENDING_FIQ_1
#define INTC_PENDING_FIQ_2_V INTC_PENDING_FIQ_2

		.text

ENTRY(set_fiq_stack)
		mrs	r1, cpsr
		bic	r2, r1, #2
		msr	cpsr_c, r2	@ fiq mode
		mov	r0, r0		@ NOP after changing to FIQ mode??
		mov	sp, r0		@ set FIQ mode stack pointer
		msr	cpsr_c, r1	@ return to SVC mode
		mov	r0, r0		@ NOP after return from FIQ mode??
		mov 	pc, lr


ENTRY(fiq_handler_start)
		stmfd	sp!, {r0-r12, lr}		@ (save cpu registers)		
		mrs 	r11, spsr

		ldr 	r9, =omap_sram_fiq_handler
		mov	lr, pc	
		ldr	pc, [r9]


@ Ack the Int Controller
		ldr	r8, =IRQ_INTC_V
		mov	r9, #2
		str	r9, [r8]

@ Data Synchronization Barrier
		mov r0, #0
		mcr p15, #0, r0, c7, c10, #4

		msr 	spsr, r11 @ Restore the SPSR from R11
		ldmfd sp!, {r0-r12, lr} @ Restore working registers and Link register
		@ Return after handling the interrupt
		subs pc, lr, #4

		.ltorg
ENTRY(fiq_handler_sz)
	.word	. - fiq_handler_start

		.data
ENTRY(omap_sram_fiq_handler)
	.word	0
