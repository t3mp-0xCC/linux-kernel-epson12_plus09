/*
 * include/asm-arm/arch-epson09/
 *
 *  COPYRIGHT (C) 2008 SEIKO EPSON, CORPORATION, All rights reserved.
 *
 * Low-level IRQ helper macros for EPSON09 platforms
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <mach/hardware.h>

	.macro	disable_fiq
	.endm

	.macro  get_irqnr_preamble, base, tmp
		mov	\tmp, #0
		ldr	\base, =IO_ADDRESS(EPSON09_GICU_9_BASE)
		add \base, \base, #0x100
	.endm

	.macro  arch_ret_to_user, tmp1, tmp2
	.endm

	.macro  get_irqnr_and_base, irqnr, irqstat, base, tmp

	ldr     \base, =IO_ADDRESS(EPSON09_GICU_9_BASE)
	ldr     \irqstat, [\base, #0] @ ICSTS register

	mov	\tmp, #0x00000000		@ for GICU_9
	mov	\base, #0xD0000000		@ for GICU_9
	add \base, #0x1000			@ for GICU_9
	str	\tmp, [\base, #0x08]	@ for GICU_9

	mov	\tmp, \irqstat, lsl #8
	mov	\irqnr, \tmp, lsr #24

	.endm
