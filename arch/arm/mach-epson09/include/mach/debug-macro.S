/* linux/include/asm-arm/arch-epson09/debug-macro.S
 *
 * Debugging macro include header
 *
 *  COPYRIGHT (C) 2008 SEIKO EPSON, CORPORATION.
 *  Copyright (C) 1994-1999 Russell King
 *  Moved from linux/arch/arm/kernel/debug.S by Ben Dooks
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

	.macro	addruart,rx,tmp
	mrc		p15, 0, \rx, c1, c0
	tst		\rx, 		#1 @ MMU enabled?
	moveq	\rx,		#0xe0000000
	movne	\rx,		#0xe0000000	@ virtual base
	orr		\rx, \rx,	#0x00800000
	orr		\rx, \rx,	#0x00016000
	.endm

	.macro	senduart,rd,rx
	strb \rd, [\rx, #0x0c]
	.endm

	.macro	waituart,rd,rx
	.endm

	.macro	busyuart,rd,rx
1002:
	ldrh	\rd, [\rx, #0x10]
	tst		\rd, 		#(0x40 + 0x20)
	beq		1002b
	bic		\rd, \rd, 	#(0x40 + 0x20)
	strh	\rd, [\rx, #0x10]
	.endm
