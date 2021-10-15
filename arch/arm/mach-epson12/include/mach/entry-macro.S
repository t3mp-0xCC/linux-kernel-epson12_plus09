/*
 *  Copyright (C) 2009 Samsung Electronics TLD.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 * Base on linux/include/asm-arm/arch-realview/entry-macro.S
 *
 * History 
 *		01/08/10	jiseong oh 	create source
 *		fixed gic bug- don't read cpu interrupt  before read ack flag
        modified jiseong.oh
 */
#include <mach/hardware.h>
#include <asm/hardware/gic.h>

		.macro	disable_fiq
		.endm

		.macro  get_irqnr_preamble, base, tmp
		mov	\tmp, #0
		ldr	\base, =IO_ADDRESS(CORE_PM_BASE)
		add \base, \base, #0x100
		.endm

		.macro  arch_ret_to_user, tmp1, tmp2
		.endm

		.macro irq_prio_table
		.endm
		
		.macro  get_irqnr_and_base, irqnr, irqstat, base, tmp
		ldr     \irqstat, [\base, #GIC_CPU_INTACK] 
		ldr	\tmp, =1021
		bic     \irqnr, \irqstat, #0x1c00
		cmp     \irqnr, #29
		cmpcc	\irqnr, \irqnr
		cmpne	\irqnr, \tmp
		cmpcs	\irqnr, \irqnr
		.endm

		/* We assume that irqstat (the raw value of the IRQ acknowledge
		 * register) is preserved from the macro above.
		 * If there is an IPI, we immediately signal end of interrupt on the
		 * controller, since this requires the original irqstat value which
		 * we won't easily be able to recreate later.
		 */

		.macro test_for_ipi, irqnr, irqstat, base, tmp
		bic	\irqnr, \irqstat, #0x1c00
		cmp	\irqnr, #16
		it	cc
		strcc	\irqstat, [\base, #GIC_CPU_EOI]
		it	cs
		cmpcs	\irqnr, \irqnr
		.endm

		/* As above, this assumes that irqstat and base are preserved.. */

		.macro test_for_ltirq, irqnr, irqstat, base, tmp
		bic	\irqnr, \irqstat, #0x1c00
		mov 	\tmp, #0
		cmp	\irqnr, #29
		itt	eq
		moveq	\tmp, #1
		streq	\irqstat, [\base, #GIC_CPU_EOI]
		cmp	\tmp, #0
		.endm
