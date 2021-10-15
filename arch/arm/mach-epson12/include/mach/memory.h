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
 * Base on linux/include/asm-arm/arch-realview/memory.h
 *
 * History 
 *  11/08/07    Donghoon Yu     include header create
 *
 */
 
/* Copyright (c)SEIKO EPSON CORPORATION 2012. All rights reserved. */
 
#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/*
 * Physical DRAM offset.
 */
#define PHYS_OFFSET		UL(0x40000000) // EPSON12
#define CONSISTENT_DMA_SIZE 	SZ_4M
/*
 * Virtual view <-> DMA view memory address translations
 * virt_to_bus: Used to translate the virtual address to an
 *              address suitable to be passed to set_dma_addr
 * bus_to_virt: Used to convert an address for DMA operations
 *              to an address that the kernel can use.
 */
/*  Defined elsewhere 2.6.33
    #define __virt_to_bus(x)	((x) - PAGE_OFFSET)
    #define __virt_to_bus(x)	__virt_to_phys(x)
    #define __bus_to_virt(x)	((x) + PAGE_OFFSET)
 */

#endif
