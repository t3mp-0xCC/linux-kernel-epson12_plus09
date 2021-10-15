/* 
 *  Copyright (C) 2007 Samsung Electronics TLD.
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
 * Base on linux/include/asm-arm/arch-realview/debug-macro.S
 *
 * History 
 *  11/08/07    Donghoon Yu     source code create
 *
 */

/* Copyright (c)SEIKO EPSON CORPORATION 2012. All rights reserved. */

#include <mach/serial.h>
#include <mach/hardware.h>

		.macro	addruart,rx,tmp /* add arg from 2.6.35 */
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1		@ MMU enabled?
		ldreq	\rx,  =0x30280000
		ldrne	\rx,  =IO_ADDRESS(EPSON12_UART0_BASE)
		.endm

		.macro	senduart,rd,rx
		strb	\rd, [\rx, #UART_DR]
		.endm

		.macro	waituart,rd,rx		
1001:		ldr	\rd, [\rx, #UART_FR]
		tst	\rd, #UART_FR_TXFF
		bne	1001b
		.endm

		.macro	busyuart,rd,rx
1001:		ldr	\rd, [\rx, #UART_FR]
		tst	\rd, #UART_FR_BUSY
		bne	1001b
		.endm
