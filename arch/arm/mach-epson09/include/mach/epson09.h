/*
 * linux/include/asm-arm/arch-epson09/platform.h
 *
 * Copyright (c) ARM Limited 2003.  All rights reserved.
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

#ifndef __address_h
#define __address_h                     1

/*
 * Memory definitions
 */
#define EPSON09_SSRAM_BASE		/* EPSON09_SSMC_BASE ? */
#define EPSON09_SSRAM_SIZE		SZ_2M

/* #define EPSON09_FLASH_BASE		0x04000000 */
#define EPSON09_FLASH_BASE		0xF4000000
#define EPSON09_FLASH_SIZE		CONFIG_EPSON09_FLASH_ROM_SIZE

#define EPSON_FLASHROM0_BASE	0x00000000
#define EPSON_FLASHROM0_SIZE	0x01000000
#define EPSON_FLASHROM1_BASE	0x04000000
#define EPSON_FLASHROM1_SIZE	0x01000000

/* 
 *  SDRAM
 */
#define EPSON09_SDRAM_BASE		0x40000000

/* EtherMAC descriptors 
 *
 */
 #if defined(CONFIG_MFP09_DESCRIPTOR_ADDR)
#define EPSON09_ETH_TXDESC	(CONFIG_MFP09_DESCRIPTOR_ADDR)
#define EPSON09_ETH_RXDESC	(EPSON09_ETH_TXDESC + SZ_8K)
#endif

/* 
 * Values for EPSON09_SYS_RESET_CTRL
 */
#define EPSON09_SYS_CTRL_RESET_CONFIGCLR	0x01
#define EPSON09_SYS_CTRL_RESET_CONFIGINIT	0x02
#define EPSON09_SYS_CTRL_RESET_DLLRESET		0x03
#define EPSON09_SYS_CTRL_RESET_PLLRESET		0x04
#define EPSON09_SYS_CTRL_RESET_POR			0x05
#define EPSON09_SYS_CTRL_RESET_DoC			0x06

#define EPSON09_SYS_CTRL_LED	(1 << 0)


/* ------------------------------------------------------------------------
 *  Versatile control registers
 * ------------------------------------------------------------------------
 */

/*
 * EPSON09_SYS_LOCK
 *     control access to SYS_OSCx, SYS_CFGDATAx, SYS_RESETCTL, 
 *     SYS_CLD, SYS_BOOTCS
 */
#define EPSON09_SYS_LOCK_LOCKED		(1 << 16)
#define EPSON09_SYS_LOCKVAL_MASK	0xFFFF		/* write 0xA05F to enable write access */

/*
 * EPSON09_SYS_FLASH
 */
#define EPSON09_FLASHPROG_FLVPPEN	(1 << 0)	/* Enable writing to flash */

/*
 * EPSON09_INTREG
 *     - used to acknowledge and control MMCI and UART interrupts 
 */
#define EPSON09_INTREG_WPROT		0x00	/* MMC protection status (no interrupt generated)	*/
#define EPSON09_INTREG_RI0			0x01	/* Ring indicator UART0 is asserted,				*/
#define EPSON09_INTREG_CARDIN		0x08	/* MMCI card in detect								*/
											/* write 1 to acknowledge and clear					*/
#define EPSON09_INTREG_RI1			0x02	/* Ring indicator UART1 is asserted,				*/
#define EPSON09_INTREG_CARDINSERT	0x03	/* Signal insertion of MMC card						*/

/*
 * EPSON09 peripheral addresses
 */
#define EPSON09_GOBI_9_BASE			0xD0000000

/* 0x1000C000 - 0x1000CFFF = reserved */
#define EPSON09_ETH_BASE			0xD0400000	/* Ethernet */
#define EPSON09_CPUIF_BASE			0xD0800000	/* USB */
#define EPSON09_MYUSB_BASE			0xE0800000	/* USB */
/* 0x10030000 - 0x100FFFFF = reserved */

#define EPSON09_GICU_9_BASE			0xD0001000	/* GICU_9 interrupt controller */

#define EPSON09_WATCHDOG_BASE		0xD000B000	/* Watchdog */
#define EPSON09_TIMER0_1_BASE		0xD0003000	/* Timer 0 and 1 */
#define EPSON09_TIMER2_3_BASE		0xD0003100	/* Timer 2 and 3 */

#define EPSON09_ASIC_SYSCU_BASE		(0xE0812000)
#define EPSON09_UART0_BASE			0xE0816000	/* Uart 0 */
#define EPSON09_UART1_BASE			0xE0817000	/* Uart 1 */
#define EPSON09_UART2_BASE			0xE0818000	/* Uart 2 */

#define EPSON09_IDC_BASE			0xF0080000	/* IDC  */
#define EPSON09_ASIC_TSYSU_BASE		(0xf0080000)
#define	EPSON09_ASIC_SFLU_BASE		(0xF0088000)/* serial parallel I/F */
#define EPSON09_ASIC_SFLU_SIZE		SZ_8K		/* in fact SZ_4K is enough, but SZ_4K is not defined */
#define EPSON09_SPIU_BASE			0xF0089000	/* SPIU(FAX)  */
#define EPSON09_SRAM_BASE			0xE0C00000	/* SRAM */
#define EPSON09_ASICIOU_BASE		0xF0000000	/* ASICIOU */

/* 
 * Control registers
 */
#define EPSON09_IDFIELD_OFFSET		0x0	/* Versatile build information */
#define EPSON09_FLASHPROG_OFFSET	0x4	/* Flash devices */
#define EPSON09_INTREG_OFFSET		0x8	/* Interrupt control */
#define EPSON09_DECODE_OFFSET		0xC	/* Fitted logic modules */


/* ------------------------------------------------------------------------
 *  Interrupts - bit assignment (primary)
 * ------------------------------------------------------------------------
 */

#define INT_WDOGINT					0	/* Watchdog timer */
#define INT_SOFTINT					1	/* Software interrupt */
#define INT_COMMRx					2	/* Debug Comm Rx interrupt */
#define INT_COMMTx					3	/* Debug Comm Tx interrupt */
#define INT_TIMERINT0_1				4	/* Timer 0 and 1 */
#define INT_TIMERINT2_3				5	/* Timer 2 and 3 */
#define INT_GPIOINT0				6	/* GPIO 0 */
#define INT_GPIOINT1				7	/* GPIO 1 */
#define INT_GPIOINT2				8	/* GPIO 2 */
#define INT_GPIOINT3				9	/* GPIO 3 */
#define INT_RTCINT					10	/* Real Time Clock */
#define INT_SSPINT					11	/* Synchronous Serial Port */
#define INT_UARTINT0				12	/* UART 0 on development chip */
#define INT_UARTINT1				13	/* UART 1 on development chip */
#define INT_UARTINT2				14	/* UART 2 on development chip */
#define INT_SCIINT					15	/* Smart Card Interface */
#define INT_CLCDINT					16	/* CLCD controller */
#define INT_DMAINT					17	/* DMA controller */
#define INT_PWRFAILINT				18	/* Power failure */
#define INT_MBXINT					19	/* Graphics processor */
#define INT_GNDINT					20	/* Reserved */
/* External interrupt signals from logic tiles or secondary controller */
#define INT_VICSOURCE21				21	/* Disk on Chip */
#define INT_VICSOURCE22				22	/* MCI0A */
#define INT_VICSOURCE23				23	/* MCI1A */
#define INT_VICSOURCE24				24	/* AACI */
#define INT_VICSOURCE25				25	/* Ethernet */
#define INT_VICSOURCE26				26	/* USB */
#define INT_VICSOURCE27				27	/* PCI 0 */
#define INT_VICSOURCE28				28	/* PCI 1 */
#define INT_VICSOURCE29				29	/* PCI 2 */
#define INT_VICSOURCE30				30	/* PCI 3 */
#define INT_VICSOURCE31				31	/* SIC source */

/* 
 *  Interrupt bit positions
 * 
 */
#define INTMASK_WDOGINT				(1 << INT_WDOGINT)
#define INTMASK_SOFTINT				(1 << INT_SOFTINT)
#define INTMASK_COMMRx				(1 << INT_COMMRx)
#define INTMASK_COMMTx				(1 << INT_COMMTx)
#define INTMASK_TIMERINT0_1			(1 << INT_TIMERINT0_1)
#define INTMASK_TIMERINT2_3			(1 << INT_TIMERINT2_3)
#define INTMASK_GPIOINT0			(1 << INT_GPIOINT0)
#define INTMASK_GPIOINT1			(1 << INT_GPIOINT1)
#define INTMASK_GPIOINT2			(1 << INT_GPIOINT2)
#define INTMASK_GPIOINT3			(1 << INT_GPIOINT3)
#define INTMASK_RTCINT				(1 << INT_RTCINT)
#define INTMASK_SSPINT				(1 << INT_SSPINT)
#define INTMASK_UARTINT0			(1 << INT_UARTINT0)
#define INTMASK_UARTINT1			(1 << INT_UARTINT1)
#define INTMASK_UARTINT2			(1 << INT_UARTINT2)
#define INTMASK_SCIINT				(1 << INT_SCIINT)
#define INTMASK_CLCDINT				(1 << INT_CLCDINT)
#define INTMASK_DMAINT				(1 << INT_DMAINT)
#define INTMASK_PWRFAILINT			(1 << INT_PWRFAILINT)
#define INTMASK_MBXINT				(1 << INT_MBXINT)
#define INTMASK_GNDINT				(1 << INT_GNDINT)
#define INTMASK_VICSOURCE21			(1 << INT_VICSOURCE21)
#define INTMASK_VICSOURCE22			(1 << INT_VICSOURCE22)
#define INTMASK_VICSOURCE23			(1 << INT_VICSOURCE23)
#define INTMASK_VICSOURCE24			(1 << INT_VICSOURCE24)
#define INTMASK_VICSOURCE25			(1 << INT_VICSOURCE25)
#define INTMASK_VICSOURCE26			(1 << INT_VICSOURCE26)
#define INTMASK_VICSOURCE27             (1 << INT_VICSOURCE27)
#define INTMASK_VICSOURCE28             (1 << INT_VICSOURCE28)
#define INTMASK_VICSOURCE29             (1 << INT_VICSOURCE29)
#define INTMASK_VICSOURCE30             (1 << INT_VICSOURCE30)
#define INTMASK_VICSOURCE31             (1 << INT_VICSOURCE31)


#define EPSON09_SC_VALID_INT               0x003FFFFF

#define MAXIRQNUM                       31
#define MAXFIQNUM                       31
#define MAXSWINUM                       31

/* 
 *  Application Flash
 * 
 */
#define FLASH_BASE                      EPSON09_FLASH_BASE
#define FLASH_SIZE                      EPSON09_FLASH_SIZE
#define FLASH_END                       (FLASH_BASE + FLASH_SIZE - 1)
#define FLASH_BLOCK_SIZE                SZ_128K


/* 
 *  Clean base - dummy
 * 
 */
#define CLEAN_BASE                      EPROM_BASE

/*
 * System controller bit assignment
 */
#define EPSON09_REFCLK	0
#define EPSON09_TIMCLK	1

#define EPSON09_TIMER1_EnSel	15
#define EPSON09_TIMER2_EnSel	17
#define EPSON09_TIMER3_EnSel	19
#define EPSON09_TIMER4_EnSel	21


#define MAX_TIMER                       2
#define MAX_PERIOD                      699050
#define TICKS_PER_uSEC                  1

/* 
 *  These are useconds NOT ticks.  
 * 
 */
#define mSEC_1                          1000
#define mSEC_5                          (mSEC_1 * 5)
#define mSEC_10                         (mSEC_1 * 10)
#define mSEC_25                         (mSEC_1 * 25)
#define SEC_1                           (mSEC_1 * 1000)

#define EPSON09_CSR_BASE             0x10000000
#define EPSON09_CSR_SIZE             0x10000000

#include "epson09_usb_ehci.h"

#endif /* __address_h */

/* 	END */
