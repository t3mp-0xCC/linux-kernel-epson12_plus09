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
 * Base on /include/asm/hardware/gic.h
 *
 * History                                            
 *  11/08/07    Donghoon Yu     include header create
 *
 */

/* Copyright (c)SEIKO EPSON CORPORATION 2012. All rights reserved. */

#ifndef __EPSON12_GIC_H
#define __EPSON12_GIC_H

#include <linux/compiler.h>

/************************************************* 
 * 	Interrupt Distributor Register
 * 	*/
#define INTD_CONTROL				0x000
#define INTD_TYPE				0x004
#define INTD_SELECT_SECURITY(n)			(0x080 + n*4)
#define INTD_ENABLE_SET(n)			(0x100 + n*4)
#define INTD_ENABLE_CLEAR(n)			(0x180 + n*4)
#define INTD_PEND_SET(n)			(0x200 + n*4)
#define INTD_PEND_CLEAR(n)			(0x280 + n*4)
#define INTD_ACTIVE(n)				(0x300 + n*4)
#define INTD_PRIORITY(n)			(0x400 + n*4)
#define INTD_CPU_TARGET(n)			(0x800 + n*4)
#define INTD_INT_CONFING(n)			(0xC00 + n*4)
#define INTD_PPI_STATUS				0xD00
#define INTD_SPI_STATUS(n)			(0xD04 + n*4)
#define INTD_SOFT_INTERRUPT			0xF00

/************************************************** 
 * 	Interrupt Interface Register
 * 	*/
#define INTI_CONTROL				0x00
#define INTI_PRIORITY_MASK			0x04
#define INTI_BINARY_POINT			0x08
#define INTI_ACK				0x0C
#define INTI_EOI				0x10
#define INTI_RUNNING_PRIORITY			0x14
#define INTI_HIGHEST_PEND			0x18

/**************************************************
 * 	Interrupt Target CPU Definition
 * 	*/
#define INT_CPU_ALL				(0x3)
#define INT_CPU0				(1 << 0)
#define INT_CPU1				(1 << 1)

#define INT_CONFIG_LEVEL			0
#define INT_CONFIG_EDGE				1

#define INT_ALL					0xFFFF

#endif
