/*

Copyright (c)SEIKO EPSON CORPORATION 2012. All rights reserved.

Module Name:
	Inter Domain Communication Driver.

Abstract:

Author:
	IJP.LP Division (ijplpd) 08/09/2011

Revision History:

Date		Who		What
--------	-------	------------------------------------------------
08/09/2011	ijplpd	birth

*/

#ifndef __EPSON12_PS_H__
#define __EPSON12_PS_H__

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/pm.h>

#include <linux/etherdevice.h>
#include <linux/platform_device.h>

#include <asm/cacheflush.h>
#include <asm/io.h>
#include <asm/leds.h>
#include <asm/processor.h>
#include "mach/epson12.h"

#undef POWERSAVE_DEBUG
/*#define POWERSAVE_DEBUG*/
#ifdef POWERSAVE_DEBUG
#define PSDBG(args...)	printk(args)
#else
#define PSDBG(args...)	do {} while(0)
#endif


/*
 * EPSON12 Power Save Level 1 scratchpad address
 */
#define EPSON12_PSMODE1_SCRATCHPAD		0xE9003FF4

#define OS1_PRIVATE_SAVEAREA_SIZE		128
#define OS1_PRIVATE_SAVEAREA_OFFSET		(0x120 / sizeof(unsigned long))

/*
 * EPSON12 Power Save Level 2 status info pointer address
 */
#define EPSON12_PSMODE2_STATUS_INFO		0x30220000

#define EPSON12_PSMODE2_SI_OFFSET		0x0C
#define EPSON12_PSMODE2_SI_COMMON_OFFSET	0x0A8
#define EPSON12_PSMODE2_SI_LINUX_OFFSET		0x014
#define EPSON12_PSMODE2_SI_LINUX_SIZE		0x400

#define EPSON12_PSMODE2_CH2_CNT_ADDR		0x22000094
#define EPSON12_PSMODE2_CH2_CNT_SIZE		0x4


/***********************************************************
	usb (for wifi)
************************************************************/
struct usb_hcd;
extern int qtd_ctrl(struct usb_hcd *hcd, int is_on);


/***********************************************************
	Target device for cpu stop
************************************************************/
#define EPSTG_WIFI		(0x1 << 0)
#define EPSTG_ETHER		(0x1 << 1)
//#define EPSTG_ETHER_DESC	(0x1 << 2)
#define EPSTG_TIMER		(0x1 << 3)
#define EPSTG_UART		(0x1 << 4)
#define EPSTG_GIC		(0x1 << 5)
#define EPSTG_GIC_ROOT		(0x1 << 6)
//#define EPSTG_DELAY		(0x1 << 7)
#define EPSTG_SFAX		(0x1 << 8)
#define EPSTG_IDC		(0x1 << 9)
#define EPSTG_IPSEC		(0x1 << 10)
#define EPSTG_ETHER_L2		(0x1 << 11)
#define EPSTG_PCIE		(0x1 << 12)

#define EPSTG_ETHER_SET		EPSTG_ETHER/* | EPSTG_DELAY | EPSTG_ETHER_DESC*/
#define EPSTG_PSMODE1		EPSTG_WIFI | EPSTG_TIMER | EPSTG_UART | EPSTG_GIC | EPSTG_SFAX | EPSTG_ETHER
#define EPSTG_PSMODE1_NOETH0	EPSTG_WIFI | EPSTG_TIMER | EPSTG_UART | EPSTG_GIC | EPSTG_SFAX
#define EPSTG_PSMODE2		EPSTG_WIFI | EPSTG_TIMER | EPSTG_UART | EPSTG_GIC | EPSTG_SFAX | EPSTG_IDC | EPSTG_GIC_ROOT | EPSTG_IPSEC | EPSTG_ETHER_L2 | EPSTG_PCIE
#define EPSTG_GOHPL		EPSTG_PSMODE2

#if defined(CONFIG_POWER_SAVING_SWITCH_L1)
	#if defined(CONFIG_MACH_EPSON12_DIVI)
		#define EPSTG_TARGET_PS_MODE	EPSTG_PSMODE1
	#elif defined(CONFIG_MACH_EPSON12_DIVL)
		#define EPSTG_TARGET_PS_MODE	EPSTG_PSMODE1
	#elif defined(CONFIG_MACH_EPSON12_DIVF)
		#define EPSTG_TARGET_PS_MODE	EPSTG_PSMODE1 | EPSTG_PCIE
	#endif
#elif defined(CONFIG_POWER_SAVING_SWITCH_L2)
	#if defined(CONFIG_MACH_EPSON12_DIVI)
		#define EPSTG_TARGET_PS_MODE	EPSTG_PSMODE2
	#elif defined(CONFIG_MACH_EPSON12_DIVL)
		#define EPSTG_TARGET_PS_MODE	EPSTG_PSMODE2
	#elif defined(CONFIG_MACH_EPSON12_DIVF)
		#define EPSTG_TARGET_PS_MODE	EPSTG_PSMODE2 | EPSTG_PCIE
	#endif
#endif


/***********************************************************
	ethernet
************************************************************/
#define GMAC_ST		(0x1 << 13)					/* Start/Stop Transmission */
#define GMAC_SR		(0x1 <<  1)					/* Start/Stop Receive */

#define GMAC_TS_MASK	((0x1 << 22) | (0x1 << 21) | (0x1 << 20))	/* Transmit Process State */
#define GMAC_RS_MASK	((0x1 << 19) | (0x1 << 18) | (0x1 << 17))	/* Receive Process State */


/* DMA CRS Control and Status Register Mapping */
#define DMA_BUS_MODE		0x00001000	/* Bus Mode */
#define DMA_XMT_POLL_DEMAND	0x00001004	/* Transmit Poll Demand */
#define DMA_RCV_POLL_DEMAND	0x00001008	/* Received Poll Demand */
#define DMA_RCV_BASE_ADDR	0x0000100c	/* Receive List Base */
#define DMA_TX_BASE_ADDR	0x00001010	/* Transmit List Base */
#define DMA_STATUS		0x00001014	/* Status Register */
#define DMA_CONTROL		0x00001018	/* Ctrl (Operational Mode) */
#define DMA_INTR_ENA		0x0000101c	/* Interrupt Enable */
#define DMA_MISSED_FRAME_CTR	0x00001020	/* Missed Frame Counter */
#define DMA_CUR_TX_BUF_ADDR	0x00001050	/* Current Host Tx Buffer */
#define DMA_CUR_RX_BUF_ADDR	0x00001054	/* Current Host Rx Buffer */


/****************************************************************
	UART
****************************************************************/
#define UART_IMSC	0x38		/* Interrupt mask. */


/****************************************************************
	timer
****************************************************************/
#define CONTROL_RUN		0x00000001
#define TIMER_CTL_CH4		0x10		/* Timer Control Register */
#define EPSON12_TIMER_BASE	0x22000000	/* TIMERU */
#define EPSON12_TIMERU1		0x0100		/* TIMERU1 */
#define TIMER_REGISTER_ADDRESS(x) (IO_ADDRESS(EPSON12_TIMER_BASE) + EPSON12_TIMERU1 + x)


/****************************************************************
	interrupt
****************************************************************/
#define JPROBE_MAGIC_ADDR	0xffffffff
#define MISC_WH_BASE		0x24030000

#define GIC_DIST_CTRL		0x000
#define GIC_DIST_TARGET		0x800

#define GIC_CPU_CTRL		0x00
#define GIC_CPU_PRIMASK		0x04


/****************************************************************
	powersave mode flag status (for L1)
****************************************************************/
#define PSS_CPU_NORMAL_MODE	10
#define PSS_CPU_STANDBY_MODE	1



#define CP15_SCTLR_M	(1 << 0)
#define CP15_SCTLR_C	(1 << 2)
#define CP15_SCTLR_I	(1 << 12)
//#define SCTLR_INIT	(CP15_SCTLR_M + CP15_SCTLR_C + CP15_SCTLR_I)
#define SCTLR_INIT	0x0000100D


/****************************************************************
	ethernet stop mode (for L2)
****************************************************************/
#define PSS_ETHSTA_IFDOWN	-1
#define PSS_ETHSTA_IFUP		0
#define PSS_ETHSTA_STOP_API	1
#define PSS_ETHSTA_STOP_KERN	2


/***********************************************************
	powersave status
************************************************************/
#define EPPSM_ERROR	-1
#define EPPSM_NORMAL	0
#define EPPSM_L1	1
#define EPPSM_L2	2


/****************************************************************
	functin for powersave mode
****************************************************************/
extern int eps_device_stop(unsigned long target);
extern int eps_device_recovery(unsigned long target);

/****************************************************************
	for stmmac ethernet
****************************************************************/
#define SIOC_EP_MAC_TX_DESCRIPTER_EMPTY (SIOCDEVPRIVATE + 9)
#define SIOC_EP_MAC_DMA_TRANSFER	(SIOCDEVPRIVATE + 10)
#define SIOC_EP_MAC_DESCRIPTER_MAX	(SIOCDEVPRIVATE + 11)
#define SIOC_EP_MAC_DESCRIPTER_EMPTY	(SIOCDEVPRIVATE + 12)
#define SIOC_EP_SKB_EMPTY		(SIOCDEVPRIVATE + 13)
#define SIOC_EP_SET_DESCRIPTER_DATA	(SIOCDEVPRIVATE + 14)

#define GMAC_DMA_OFFSET		0x1000
#define GMAC_DMA_STATUS		(GMAC_DMA_OFFSET + 0x0014)	/* Dma status Register */
#define GMAC_DMA_OPMODE		(GMAC_DMA_OFFSET + 0x0018)	/* Dma Operation Mode Register */
#define GMAC_DMA_INT_ENABLE	(GMAC_DMA_OFFSET + 0x001C)	/* Interrupt enable */

// Bit description	|[Bits]							| R/W| Reset value
//RESERVED		// Reserved						|[31:17]| R_W| 0
#define	GMAC_NIE	(0x1 << 16) // Normal Interrupt Summary Enable|[16]	| R_W| 0
#define	GMAC_AIE	(0x1 << 15) // Abnormal Interrupt Summary Enable|[15]	| R_W| 0
#define	GMAC_EIE	(0x1 << 14) // Early Receive Interrupt Enable|[14]	| R_W| 0
#define	GMAC_FBE	(0x1 << 13) // Fatal Bus Error Enable			|[13]	| R_W| 0
//RESERVED		// Reserved						|[31:17]| R_W| 0
#define	GMAC_ETE	(0x1 << 10) // Early Transmit Interrupt Enable		|[10]	| R_W| 0
#define	GMAC_RWE	(0x1 <<  9) // Receive Watchdog Timeout Enable		|[ 9]   | R_W| 0
#define	GMAC_RSE	(0x1 <<  8) // Receive Stopped Enable			|[ 8]   | R_W| 0
#define GMAC_RUE	(0x1 <<  7) // Receive Buffer Unavailable Enable	|[ 7]   | R_W| 0
#define GMAC_RIE	(0x1 <<  6) // Receive Interrupt Enable			|[ 6]   | R_W| 0
#define GMAC_UNE	(0x1 <<  5) // Transmit Underflow Interrupt Enable	|[ 5]   | R_W| 0
#define GMAC_OVE	(0x1 <<  4) // Receive Overflow Interrupt Enable	|[ 4]   | R_W| 0
#define GMAC_TJE	(0x1 <<  3) // Transmit Jabber Timeout Enable		|[ 3]   | R_W| 0
#define GMAC_TUE	(0x1 <<  2) // Transmit Buffer Unavailable Enable	|[ 2]   | R_W| 0
#define GMAC_TSE	(0x1 <<  1) // Transmit Stopped Enable			|[ 1]   | R_W| 0
#define GMAC_TIE	(0x1 <<  0) // Transmit Interrupt Enable		|[ 0]   | R_W| 0

#define GMAC_ST		(0x1 << 13) // Start/Stop Transmission				|[13]   | R_W| 0
#define GMAC_SR		(0x1 <<  1) // Start/Stop Receive				|[ 1]   | R_W| 0

#define GMAC_TS_MASK	((0x1 << 22) | (0x1 << 21) | (0x1 << 20))	/* Transmit Process State	|[22:20]| RO | 0 */
#define GMAC_RS_MASK	((0x1 << 19) | (0x1 << 18) | (0x1 << 17))	/* Receive Process State	|[19:17]| RO | 0 */

typedef struct _ep_interface_skb_set {
	__u32				len;
	__u32				offset;
	__u8				descriptor[1024];
	struct _ep_interface_skb_set	*next;
} EP_INTERFACE_SKB_SET;


int eps_stmmac_dma_stop(struct net_device *ndev, int force);
int eps_stmmac_dma_restart(struct net_device *ndev, int force);

extern unsigned int g_alloc_skb;
void discripter_send(struct net_device *dev);
extern __u32 g_Receive_List_Base;
extern __u32 g_Transmit_List_Base;
extern __u32 g_reg_ether_op;
extern __u32 g_reg_ether_int;
extern __u32 g_ps_ether_status;

#define GMAC_ST		(0x1 << 13)					/* Start/Stop Transmission */
#define GMAC_SR		(0x1 <<  1)					/* Start/Stop Receive */

#define GMAC_TS_MASK	((0x1 << 22) | (0x1 << 21) | (0x1 << 20))	/* Transmit Process State */
#define GMAC_RS_MASK	((0x1 << 19) | (0x1 << 18) | (0x1 << 17))	/* Receive Process State */

/* DMA CRS Control and Status Register Mapping */
#define DMA_RCV_BASE_ADDR	0x0000100c	/* Receive List Base */
#define DMA_TX_BASE_ADDR	0x00001010	/* Transmit List Base */
#define DMA_STATUS		0x00001014	/* Status Register */
#define DMA_CONTROL		0x00001018	/* Ctrl (Operational Mode) */
#define DMA_INTR_ENA		0x0000101c	/* Interrupt Enable */
#define DMA_MISSED_FRAME_CTR	0x00001020	/* Missed Frame Counter */

extern struct net_device *g_stmmac_dev;


#endif /* __EPSON12_PS_H__ */
