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

#ifndef __EPSON12_IRQSH_H
#define __EPSON12_IRQSH_H

#define IRQ_MPC_SPI		32
/* #define IRQ_BK		(IRQ_MPC_SPI+0~1) */
#define IRQ_SPIU_FAX		(IRQ_MPC_SPI+2)
/* #define IRQ_BK		(IRQ_MPC_SPI+3~35) */
#define IRQ_MISC_BK36		(IRQ_MPC_SPI+36) // CA9Core0 to CA9Core1
#define IRQ_MISC_BK37		(IRQ_MPC_SPI+37) // CA9Core0 to ARM946
#define IRQ_MISC_BK38		(IRQ_MPC_SPI+38) // CA9Core1 to CA9Core0
#define IRQ_MISC_BK39		(IRQ_MPC_SPI+39) // CA9Core1 to ARM946
/* #define IRQ_LCD_TOP		(IRQ_MPC_SPI+40) */
#define IRQ_SPIU_EX		(IRQ_MPC_SPI+41)
/* #define IRQ_SCN_TOP		(IRQ_MPC_SPI+42~44) */
/* #define IRQ_TIMERU0~3	(IRQ_MPC_SPI+45~48)(for RTOS) */
#define IRQ_TIMERU4		(IRQ_MPC_SPI+49) // TIMER G1 CH4
#define IRQ_TIMERU5		(IRQ_MPC_SPI+50) // TIMER G1 CH5
#define IRQ_TIMERU6		(IRQ_MPC_SPI+51) // TIMER G1 CH6
#define IRQ_TIMERU7		(IRQ_MPC_SPI+52) // TIMER G1 CH7
/* #define IRQ_TIMERU8~13	(IRQ_MPC_SPI+53~58)(for RTOS) */
/* #define IRQ_ADCU		(IRQ_MPC_SPI+59) */
#define IRQ_SFLU3   		(IRQ_MPC_SPI+60)
#define IRQ_SQR   		(IRQ_MPC_SPI+61)
/* #define IRQ_RESERVED		(IRQ_MPC_SPI+62~91) */
#define IRQ_MISC_WH92		(IRQ_MPC_SPI+92) // ARM946 to CA9Core0
#define IRQ_MISC_WH93		(IRQ_MPC_SPI+93) // ARM946 to CA9Core1
#define IRQ_EIP_94		(IRQ_MPC_SPI+94)
#define IRQ_EIP_150_TRNG	(IRQ_MPC_SPI+95)
#define IRQ_EIP_AIC_OUT		(IRQ_MPC_SPI+96)
#define IRQ_SFAX		(IRQ_MPC_SPI+97)
/* #define IRQ_RESERVED		(IRQ_MPC_SPI+98~99) */
#define IRQ_nCTIIRQ0		(IRQ_MPC_SPI+100)
#define IRQ_nCTIIRQ1		(IRQ_MPC_SPI+101)
#define IRQ_DMU			(IRQ_MPC_SPI+102)
#define IRQ_GETH_LPI		(IRQ_MPC_SPI+103)
#define IRQ_GETH_PMT		(IRQ_MPC_SPI+104)
#define IRQ_GETH_SBD		(IRQ_MPC_SPI+105)
#define IRQ_USBHOST0		(IRQ_MPC_SPI+106)
#define IRQ_USBHOST1		(IRQ_MPC_SPI+107)
#define IRQ_USBHOST2		(IRQ_MPC_SPI+108)
#define IRQ_USBDEV0		(IRQ_MPC_SPI+109)
#define IRQ_USBDEV1		(IRQ_MPC_SPI+109)
#define IRQ_UART0		(IRQ_MPC_SPI+111)
#define IRQ_UART1		(IRQ_MPC_SPI+112)
#define IRQ_UART2		(IRQ_MPC_SPI+113)
#define IRQ_TIMER0		(IRQ_MPC_SPI+114)
#define IRQ_TIMER1		(IRQ_MPC_SPI+115)
#define IRQ_TIMER2		(IRQ_MPC_SPI+116)
#define IRQ_TIMER3		(IRQ_MPC_SPI+117)
#define IRQ_WDT			(IRQ_MPC_SPI+118)
#define IRQ_DMAC		(IRQ_MPC_SPI+119)
#define IRQ_SDMMC		(IRQ_MPC_SPI+120)
/* #define IRQ_RESERVED		(IRQ_MPC_SPI+121~127) */

#define IRQ_MAX			(IRQ_MPC_SPI+128)

#endif
