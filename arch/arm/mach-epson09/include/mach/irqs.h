/*
 *  linux/include/asm-arm/arch-epson09/irqs.h
 *
 *  COPYRIGHT (C) 2008 SEIKO EPSON, CORPORATION.
 *  Copyright (C) 2003 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd.
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

#include <mach/epson09.h>

#define do_level_IRQ	handle_level_irq

/*
 *  IRQ interrupts definitions are the same the INT definitions
 *  held within platform.h
 */
#define IRQ_VIC_START		0

#define IRQ_ARM9_ACQCOMP	0	/* ARM9_ACQCOMP 0 ARM11/9 core debug (ETM trace data trigger) */
#define IRQ_ARM9_FULL		1	/* ARM9_FULL 1 ARM11/9 core debug (ETM FIFO Full) */
#define IRQ_ARM9_COMMRX		2	/* ARM9_COMMRX 2 ARM11/9 core debug (channel recive buffer empty) */
#define IRQ_ARM9_COMMTX		3	/* ARM9_COMMTX 3 ARM11/9 core debug (channel send buffer empty) */
#define IRQ_XINT_PC0		19	/* XINT_PC0 19 External LSI interrupt */
#define IRQ_XINT_USW1		32	/* XINT_USW1 32 core communication interrupt1 */
#define IRQ_XINT_USBH1		51	/* XINT_USBH1 51 USB_Host_9 interrupt */
#ifdef CONFIG_MACH_EPSON09_EMG_SYSTEM_STOP_FOR09SU
#define IRQ_EMG_STOP_FROM_CORE0	100	/* EMG_STOP request from core 0, as same as FIQ */
#else
#define IRQ_EMG_STOP_FROM_CORE0	59	/* EMG_STOP request from core 0, as same as FIQ */
#endif
#define IRQ_INT_ETHER		65	/* INT_ETHER 65 Ether DMAC interrupt */
#define IRQ_INT_IPSEC		66	/* INT_IPSEC 66 IPsec interrupt */
#define IRQ_INT_WDT_9		70	/* INT_WDT_9 70 Gobi_9 WDT2 interrupt */
#define IRQ_INT_ITIM_9_0	73	/* INT_ITIM_9_0 73 Gobi_9 ITIM2(ch0) interrupt */
#define IRQ_INT_ITIM_9_1	74	/* INT_ITIM_9_1 74 Gobi_9 ITIM2(ch1) interrupt */
#define IRQ_SCIF0_RERR		113	/* SCIF0_RERR 113 Receive Error Interrupt0 (SCIF ch0) */
#define IRQ_SCIF0_BRK		114	/* SCIF0_BRK 114 Break Interrupt0 (SCIF ch0) */
#define IRQ_SCIF0_RXI		115	/* SCIF0_RXI 115 Receive-FIFO Data-full Interrupt0 (SCIF ch0) */
#define IRQ_SCIF0_TXI		116	/* SCIF0_TXI 116 Transmitt-FIFO Data-empty Interrupt0 (SCIF ch0) */
#define IRQ_SCIF1_RERR		117	/* SCIF1_RERR 117 Receive Error Interrupt1 (scif1) */
#define IRQ_SCIF1_BRK		118	/* SCIF1_BRK 118 Break Interrupt1 (scif1) */
#define IRQ_SCIF1_RXI		119	/* SCIF1_RXI 119 Receive-FIFO Data-full Interrupt1 (scif1) */
#define IRQ_SCIF1_TXI		120	/* SCIF1_TXI 120 Transmitt-FIFO Data-empty Interrupt1 (scif1) */
#define IRQ_SCIF2_RERR		121	/* SCIF2_RERR 121 Receive Error Interrupt2 (scif2) */
#define IRQ_SCIF2_BRK		122	/* SCIF2_BRK 122 Break Interrupt2 (scif2) */
#define IRQ_SCIF2_RXI		123	/* SCIF2_RXI 123 Receive-FIFO Data-full Interrupt2 (scif2)Transmitt-FIFO Data-empty Interrupt2 (scif2) */
#define IRQ_SCIF2_TXI		124	/* SCIF2_TXI 124 Transmitt-FIFO Data-empty Interrupt2 (scif2) */
#define IRQ_GOBI_9_VOIDINT		126	/* GOBI_9_VOIDINT 126 Gobi_9 bus error interrupt */

#define NR_IRQS			128
