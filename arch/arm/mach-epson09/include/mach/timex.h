/*
 *  linux/include/asm-arm/arch-epson09/timex.h
 *
 *  Versatile architecture timex specifications
 *
 *  COPYRIGHT (C) 2008 SEIKO EPSON, CORPORATION.
 *  Copyright (C) 2003 ARM Limited
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
/* SSCG rate 1.665%  N -> (N-(N*0.01665))   7500000->7375125 */
/* #define CLOCK_TICK_RATE		((120000000 / 16) - ((120000000 / 16) * 0.01665)) */
#define CLOCK_TICK_RATE		(7375125)

extern u64 epson09_get_cycles(void);
#define mach_read_cycles() epson09_get_cycles()
