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
 * Base on linux/arch/arm/mach-realview/core.h
 *
 * History
 *  12/19/09	Jiseong oh		include header create
 *
 */

/* Copyright (c)SEIKO EPSON CORPORATION 2012. All rights reserved. */

#ifndef __ASM_ARCH_EPSON09_PLAT_H
#define __ASM_ARCH_EPSON09_PLAT_H

#include <asm/leds.h>
#include <asm/io.h>

extern struct flash_platform_data epson09_flash_data;

#define PLATFORM_DEVICE(dev_name,id_num,base,size,irq_num,irq_cnt,plat,bus_type)	\
static struct resource epson09_##base##_resource[] = {			\
	[0] = {								\
		.start	= EPSON09_##base##_BASE,			\
		.end	= (EPSON09_##base##_BASE) + size - 1,		\
		.flags	= IORESOURCE_MEM,				\
	},								\
	[1] = {								\
		.start	= irq_num,					\
		.end	= irq_num	 + irq_cnt-1,			\
		.flags	= IORESOURCE_IRQ,				\
	},								\
};									\
struct platform_device base##_device = {				\
	.name	= dev_name,						\
	.id		= id_num,					\
	.num_resources	= ARRAY_SIZE(epson09_##base##_resource),	\
	.resource	= epson09_##base##_resource,			\
	.dev 		= {						\
	.coherent_dma_mask = ~0,				\
	.platform_data = plat,					\
	.bus = bus_type                 \
	},								\
}

#endif
