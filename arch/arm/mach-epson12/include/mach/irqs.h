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
 * History
 *
 */
 
/* Copyright (c)SEIKO EPSON CORPORATION 2012-2013. All rights reserved. */

#ifndef __EPSON12_IRQS_H
#define __EPSON12_IRQS_H

#include <mach/epson12.h>

#ifdef CONFIG_MACH_EPSON12_M
#include "irqs_m.h" /* EPSON12M platform */
#elif defined CONFIG_MACH_EPSON12_H
#include "irqs_h.h" /* EPSON12H platform */
#endif

#define IRQ_MPC_PPI		0
#define IRQ_SGI00		(IRQ_MPC_PPI+0)
#define IRQ_SGI01		(IRQ_MPC_PPI+1)
#define IRQ_SGI02		(IRQ_MPC_PPI+2)
#define IRQ_SGI03		(IRQ_MPC_PPI+3)
#define IRQ_SGI04		(IRQ_MPC_PPI+4)
#define IRQ_SGI05		(IRQ_MPC_PPI+5)
#define IRQ_SGI06		(IRQ_MPC_PPI+6)
#define IRQ_SGI07		(IRQ_MPC_PPI+7)
#define IRQ_SGI08		(IRQ_MPC_PPI+8)
#define IRQ_SGI09		(IRQ_MPC_PPI+9)
#define IRQ_SGI10		(IRQ_MPC_PPI+10)
#define IRQ_SGI11		(IRQ_MPC_PPI+11)
#define IRQ_SGI12		(IRQ_MPC_PPI+12)
#define IRQ_SGI13		(IRQ_MPC_PPI+13)
#define IRQ_SGI14		(IRQ_MPC_PPI+14)
#define IRQ_SGI15		(IRQ_MPC_PPI+15)
/* #define IRQ_RESERVED		(INT_MPC_PPI+16~31) */

#ifdef CONFIG_MACH_EPSON12_EMG_SYSTEM_STOP
#define IRQ_EMG_STOP_FROM_CORE0	(CONFIG_EMG_IRQ_NO) // EMG INTERRUPT FROM CORE0
#endif

#endif
