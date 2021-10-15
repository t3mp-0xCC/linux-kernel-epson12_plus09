/*
 *  linux/arch/arm/mach-epson09/epson09.c
 *
 *  COPYRIGHT (C) 2008 SEIKO EPSON, CORPORATION.
 *  Copyright (C) 2004 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
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
 */

#include <linux/init.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>

#include <mach/hardware.h>

#include "core.h"
#include "itim2.h"

extern struct sys_timer epson09_itim2_timer;

static int __init epson09_base_init(void)
{
	machine_is_epson09();

	return 0;
}

arch_initcall(epson09_base_init);

MACHINE_START(EPSON09, "ARM-EPSON09")
	/* Maintainer: ARM Ltd/Deep Blue Solutions Ltd */
	.phys_io	= EPSON09_UART0_BASE,
	.io_pg_offst	= (IO_ADDRESS(EPSON09_UART0_BASE) >> 18) & 0xfffc,
	.boot_params	= 0x40000100,
	.map_io		= epson09_map_io,
	.init_irq	= epson09_init_irq,
	.timer		= &epson09_itim2_timer,
	.init_machine	= epson09_init,
MACHINE_END
