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
 * Base on linux/include/asm-arm/arch-realview/uncompress.h
 *
 * History                                                                    
 *  11/08/07    Donghoon Yu     include header create
 *
 */
#include <mach/hardware.h>

#define EPSON12_UART_DR		(*(volatile unsigned char *) (EPSON12_UART0_BASE + 0x00))
#define EPSON12_UART_LCRH	(*(volatile unsigned char *) (EPSON12_UART0_BASE + 0x2c))
#define EPSON12_UART_CR		(*(volatile unsigned char *) (EPSON12_UART0_BASE + 0x30))
#define EPSON12_UART_FR		(*(volatile unsigned char *) (EPSON12_UART0_BASE + 0x18))

/*
 * This does not append a newline
 */
static inline void putc(int c)
{
	while (EPSON12_UART_FR & (1 << 5))
		barrier();

	EPSON12_UART_DR = c;
}

static inline void flush(void)
{
	while (EPSON12_UART_FR & (1 << 3))
		barrier();
}

/*
 * nothing to do
 */
#define arch_decomp_setup()
#define arch_decomp_wdog()
