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
 * Base on /include/asm-arm/arch-realview/hardware.h
 *
 * History                                            
 *  11/08/07    Donghoon Yu     include header create
 *
 */
#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <asm/sizes.h>
#include <mach/epson12.h>

#define pcibios_assign_all_busses()	1
#define PCIBIOS_MIN_IO			0x10000
#define PCIBIOS_MIN_MEM			0x10000

/* macro to get at IO space when running virtually */
#ifdef CONFIG_MMU
#define IO_ADDRESS(x)		((((x) >> 4) & 0x0fffffff) + 0xf0000000)
#else
#define IO_ADDRESS(x)		(x)
#endif
#define __io_address(n)		((void __iomem *)IO_ADDRESS(n))

#endif
