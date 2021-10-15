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
 * Base on linux/arch/arm/mach-realview/clock.h
 *
 * History                                            
 *  11/08/07    Donghoon Yu     include header create
 *
 */
#ifndef __ASM_ARM_ARCH_CLOCK_H
#define __ASM_ARM_ARCH_CLOCK_H

struct module;

struct clk {
	struct list_head	node;

	unsigned int		enable_reg;
	unsigned int		enable_mask;

	unsigned int		div_reg;
	unsigned int		div_shift;
	unsigned long		*div_clk;

	unsigned long		*bus_clock;

	struct module		*owner;
	const char		*name;
	void			*data;
};
int clk_init(void);
int clk_register(struct clk *clk);
void clk_unregister(struct clk *clk);

#endif
