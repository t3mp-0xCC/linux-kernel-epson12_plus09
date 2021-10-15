/*
 * Copyright (c)SEIKO EPSON CORPORATION 2012. All rights reserved.
 *
 * License : GPL2.0
 */

#ifndef _EPSON12_PCIE_REGS_H_
#define _EPSON12_PCIE_REGS_H_

//#define EPSON12_PCIE_CFG_DEBUG

#if 0	// EPSON
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//#define EPSON12_PCIE0_BASE		0xC0000000 /* PCI-EX Link0 */
//#define EPSON12_PCIE1_BASE		0xD8000000 /* PCI-EX Link1 */
//#define EPSON12_PCIE_DBI0_BASE	0xE8000000 /* PCIE_DBI0 */
//#define EPSON12_PCIE_DBI1_BASE	0xE8100000 /* PCIE_DBI1 */
//#define EPSON12_PCIE_PHY_BASE		0x33000000 /* PCIE_PHY */
//#define EPSON12_PCIE_DMA_BASE		0xEA400000 /* PCIE_DMAC_SFR */

#define EPSON12_PCIE_LNK_BASE	(EPSON12_PCIE0_BASE)
#define EPSON12_PCIE_DBI_BASE	(EPSON12_PCIE_DBI0_BASE)
#define EPSON12_PCIE_SFR_BASE	(EPSON12_PCIE_PHY_BASE)
#define EPSON12_MISC_OFF	(0x30200098) // MISC_PCIEPHY_CONT@MISC_SFR
#define EPSON12_MISC_LEN	(8)
#define MISC_PCIEPHY_CONT	(misc_va)
#define MISC_PCIEPHY_STATUS	(misc_va+0x4)
#define IRQ_PCIE		(48)
////////////////////////////////////////////////////////////////////////////////////////////////////////////
#else
#undef EPSON12_PCIE_LNK_BASE
#undef EPSON12_PCIE_DBI_BASE
#undef EPSON12_PCIE_SFR_BASE
#undef EPSON12_MISC_OFF
#undef EPSON12_MISC_LEN
#undef MISC_PCIEPHY_CONT
#undef MISC_PCIEPHY_STATUS
#undef IRQ_PCIE
#endif	// EPSON

#define DEBUGMSG(cond,printf_exp)	((void)((cond)?(printk printf_exp),1:0))
#define DBG_CONF	1
#define DBG_LINK	1
#define DBG_STEP	1
#define DBG_REG		1
#define DBG_ISR		1

#define PCIE_SB		1		// 1:pcie normal speed, 0:sata speed

//--pcie_mode
#define	device_type_min				0		// device_type = pcie_mode[3:0]
#define	device_type_max				3
#define	PCIe_EP					0x00000000	// device_type = pcie_mode[3:0]
#define	PCI_Legacy				0x00000001	// device_type = pcie_mode[3:0]
#define	PCIe_RC					0x00000004	// device_type = pcie_mode[3:0]
#define	app_ltssm_enable			0x00000100	// application signal to enable the LTSSM, if deasserted, it indicates that application is not ready
#define	rx_lane_flip_en				0x00000200	// Enable the physical RX lane flip
#define	tx_lane_flip_en				0x00000400	// Enable the physical TX lane flip
#define	app_req_retry_en			0x00010000	// Allow application to enable LBC to return request retry status
#define	app_req_entr_l1				0x00020000	// Provide a capability for applications to request PM state to enter L1.
#define	app_ready_entr_l23			0x00040000	// Provide a capability for applications to notify PM module that it is ready
#define	app_req_exit_l1				0x00080000	// Provide a capability for applications to request PM

//--pcie_int0_signal
#define	pcie_int1_status_a			0x80000000	//[31]
#define	pcie_doorbell_31			(0x1<<30)	//0x40000000
#define	pcie_msibell0_31			(0x1<<29)	//0x20000000
#define	pcie_msibell1_31			(0x1<<28)	//0x10000000
#define	cfg_aer_rc_err_int			(0x1<<23)	//0x00080000 ? <-0x00800000
#define	cfg_pme_int				(0x1<<22)	//0x00020000 ? <-0x00400000
#define	hp_pme					(0x1<<15)	//x00008000
#define	hp_int					(0x1<<14)	//0x00004000
#define	pm_pme_en				(0x1<<13)	//0x00002000
#define	pm_status				(0x1<<12)	//0x00001000
#define	aux_pm_en				(0x1<<11)	//0x00000800
#define	pcie_wake				(0x1<<10)	//0x00000400
#define	pcie_clk_req_n				(0x1<<9)	//0x00000200
#define	inv_training_rst_n			(0x1<<8)	//0x00000100
#define	inv_link_req_rst_not			(0x1<<7)	//0x00000080
#define	cfg_link_auto_bw_int			(0x1<<6)	//0x00000040
#define	cfg_bw_mgt_int				(0x1<<5)	//0x00000020
#define	trgt_cpl_timeout			(0x1<<4)	//0x00000010
#define	pm_xtlh_block_tlp			(0x1<<3)	//0x00000008
#define	radm_cpl_timeout			(0x1<<2)	//0x00000004
#define	rdlh_link_up				(0x1<<1)	//0x00000002
#define	xmlh_link_up				(0x1<<0)	//0x00000001

#define both_link_up		(rdlh_link_up | xmlh_link_up)

//--pcie_int1_signal
#define	cfg_aer_rc_err_msi			(0x1<<23)	//0x00800000
#define	cfg_pme_msi				(0x1<<22)	//0x00400000
#define	gm_cmposer_lookup_err			(0x1<<21)	//0x00200000
#define	radmx_cmposer_lookup_err		(0x1<<20)	//0x00100000
#define	cfg_sys_err_rc				(0x1<<19)	//0x00080000
#define	hp_msi					(0x1<<18)	//0x00040000
#define	pm_pme_en_chg				(0x1<<17)	//0x00020000
#define	pm_dstate_chg				(0x1<<16)	//0x00010000
#define	radm_pm_pme				(0x1<<15)	//0x00008000
#define	radm_pm_to_ack				(0x1<<14)	//0x00004000
#define	radm_msg_unlock				(0x1<<13)	//0x00002000
#define	radm_vendor_msg				(0x1<<12)	//0x00001000
#define	radm_pm_turnoff				(0x1<<11)	//0x00000800
#define	radm_correctable_err			(0x1<<10)	//0x00000400
#define	radm_nonfatal_err			(0x1<<9)	//0x00000200
#define	radm_fatal_err				(0x1<<8)	//0x00000100
#define	radm_intd_deasserted			(0x1<<7)	//0x00000080
#define	radm_intc_deasserted			(0x1<<6)	//0x00000040
#define	radm_intb_deasserted			(0x1<<5)	//0x00000020
#define	radm_inta_deasserted			(0x1<<4)	//0x00000010
#define	radm_intd_asserted			(0x1<<3)	//0x00000008
#define	radm_intc_asserted			(0x1<<2)	//0x00000004
#define	radm_intb_asserted			(0x1<<1)	//0x00000002
#define	radm_inta_asserted			(0x1<<0)	//0x00000001


///////////////////////////////////////////////
//// LTSSM States (xmlh_ltssm_state or lts_state)
////	from /link/include/cxpl_defs.vh
/////////////////////////////////////////////////
#define	S_DETECT_QUIET			0x00
#define	S_DETECT_ACT			0x01
#define	S_POLL_ACTIVE			0x02
#define	S_POLL_COMPLIANCE		0x03
#define	S_POLL_CONFIG			0x04
#define	S_PRE_DETECT_QUIET		0x05
#define	S_DETECT_WAIT			0x06
#define	S_CFG_LINKWD_START		0x07
#define	S_CFG_LINKWD_ACEPT		0x08
#define	S_CFG_LANENUM_WAIT		0x09
#define	S_CFG_LANENUM_ACEPT		0x0A
#define	S_CFG_COMPLETE			0x0B
#define	S_CFG_IDLE			0x0C
#define	S_RCVRY_LOCK			0x0D
#define	S_RCVRY_SPEED			0x0E
#define	S_RCVRY_RCVRCFG			0x0F
#define	S_RCVRY_IDLE			0x10
#define	S_L0				0x11
#define	S_L0S				0x12
#define	S_L123_SEND_EIDLE		0x13
#define	S_L1_IDLE			0x14
#define	S_L2_IDLE			0x15
#define	S_L2_WAKE			0x16
#define	S_DISABLED_ENTRY		0x17
#define	S_DISABLED_IDLE			0x18
#define	S_DISABLED			0x19
#define	S_LPBK_ENTRY			0x1A
#define	S_LPBK_ACTIVE			0x1B
#define	S_LPBK_EXIT			0x1C
#define	S_LPBK_EXIT_TIMEOUT		0x1D
#define	S_HOT_RESET_ENTRY		0x1E
#define	S_HOT_RESET			0x1F

#define LINK_STATE_DISP(state)  \
	(state == S_DETECT_QUIET)  ? "S_DETECT_QUIET ":  \
	(state == S_DETECT_ACT) ? "S_DETECT_ACT":  \
	(state == S_POLL_ACTIVE) ? "S_POLL_ACTIVE":  \
	(state == S_POLL_COMPLIANCE     ) ? "S_POLL_COMPLIANCE" :  \
	(state == S_POLL_CONFIG) ? "S_POLL_CONFIG" :  \
	(state == S_PRE_DETECT_QUIET) ? "S_PRE_DETECT_QUIET":  \
	(state == S_DETECT_WAIT) ? "S_DETECT_WAIT":  \
	(state == S_CFG_LINKWD_START) ? "S_DETECT_WAIT" : \
	(state == S_CFG_LINKWD_ACEPT) ? "S_CFG_LINKWD_ACEPT": \
	(state == S_CFG_LANENUM_WAIT) ? "S_CFG_LANENUM_WAIT": \
	(state == S_CFG_LANENUM_ACEPT) ? "S_CFG_LANENUM_WAIT" : \
	(state == S_CFG_COMPLETE) ? "S_CFG_LANENUM_WAIT" : \
	(state == S_CFG_IDLE    ) ?"S_CFG_IDLE" : \
	(state == S_RCVRY_LOCK) ? "S_RCVRY_LOCK" : \
	(state == S_RCVRY_SPEED) ? "S_RCVRY_SPEED" : \
	(state == S_RCVRY_RCVRCFG) ? "S_RCVRY_RCVRCFG":  \
	(state == S_RCVRY_IDLE) ? "S_RCVRY_IDLE" : \
	(state == S_L0 ) ? "S_L0" : \
	(state == S_L0S) ? "S_L0S" : \
	(state == S_L123_SEND_EIDLE) ? "S_L123_SEND_EIDLE" : \
	(state == S_L1_IDLE     ) ? "S_L1_IDLE " : \
	(state == S_L2_IDLE     ) ? "S_L2_IDLE" : \
	(state == S_L2_WAKE     ) ? "S_L2_WAKE" : \
	(state == S_DISABLED_ENTRY) ? "S_DISABLED_ENTRY"        : \
	(state == S_DISABLED_IDLE) ? "S_DISABLED_IDLE" : \
	(state == S_DISABLED    ) ? "S_DISABLED " : \
	(state == S_LPBK_ENTRY) ? "S_LPBK_ENTRY " : \
	(state == S_LPBK_ACTIVE) ? "S_LPBK_ACTIVE" : \
	(state == S_LPBK_EXIT   ) ? "S_LPBK_EXIT" : \
	(state == S_LPBK_EXIT_TIMEOUT) ? "S_LPBK_EXIT_TIMEOUT" : \
	(state == S_HOT_RESET_ENTRY) ? "S_HOT_RESET_ENTRY" : \
	(state == S_HOT_RESET) ? "S_HOT_RESET" : \
	" Unknown state..!! "

#define LINK_DISABLE	0
#define LINK_ENABLE	1
#define LOOPBACK	2

/*
 ** PCIe Area Information
 **
 ** 0x08000000	 ~ 0x0BFFFFFF		PCIe Memory BASE boundary (64MB)
 ** 0x08000000	 ~ 0x0807FFFF		PCIe CFG0 Area	(512KB)
 ** 0x08080000	 ~ 0x080FFFFF		PCIe CFG1 Area	(512KB)
 ** 0x08100000	 ~ 0x081FFFFF		PCIe IO Area	(1MB)
 ** 0x08200000	 ~ 0x082FFFFF		PCIe ATU Area	(1MB)
 ** 0x08300000	 ~ 0x083FFFFF		PCIe MSI Area	(1MB)
 ** 0x08400000	 ~ 0x0BFFFFFF		PCIe MEM Area	(60MB)
 **
 **/

#if 0	// EPSON
///#define PCIE_AREA_BASE  0x80000000			// PCIe Space (0x80000000 ~ 0x83FFFFFF :64MB)
#define PCIE_AREA_BASE	EPSON12_PCIE_LNK_BASE		// PCIe Space (0xC0000000 ~ 0xD7FFFFFF :64MB)
#define PCIE_AREA_SIZE	384*1024*1024			///0x18000000
#define PCIE_CFG_BASE	PCIE_AREA_BASE			// Config Space (0x08000000 ~ 0x080FFFFF :1MB)
#define PCIE_CFG_SIZE	1*1024*1024
#define PCIE_CFG0_BASE	PCIE_CFG_BASE			// Cnfg0 Space: 0x08000000 ~ 0x0807FFFF
#define PCIE_CFG0_SIZE	512*1024
#define PCIE_CFG1_BASE	PCIE_CFG0_BASE+PCIE_CFG0_SIZE	// Cnfg1 Space: 0x08080000 ~ 0x080FFFFF
#define PCIE_CFG1_SIZE	512*1024
#define PCIE_IO_BASE	PCIE_CFG_BASE+PCIE_CFG_SIZE	// IO Space (0x80100000 ~ 0x801FFFFF :1MB)
#define PCIE_IO_SIZE	1*1024*1024
#define PCIE_MSI_BASE	PCIE_IO_BASE+PCIE_IO_SIZE	// MSI Test Space (0x80200000 ~ 0x802FFFFF :1MB)
#define PCIE_MSI_SIZE	1*1024*1024
#define PCIE_ATU_BASE	PCIE_MSI_BASE+PCIE_MSI_SIZE	// iATU Test Space (0x80300000 ~ 0x803FFFFF :1MB)
#define PCIE_ATU_SIZE	1*1024*1024
#define PCIE_MEM_BASE	PCIE_ATU_BASE+PCIE_ATU_SIZE	// Memory Space (0x80400000 ~ 0x83FFFFFF :60MB)
#define PCIE_MEM_SIZE	(PCIE_AREA_SIZE-PCIE_CFG_SIZE-PCIE_IO_SIZE-PCIE_MSI_SIZE-PCIE_ATU_SIZE)
#define PCIE_CFG0_LIMIT	PCIE_CFG0_BASE+PCIE_CFG0_SIZE-1
#define PCIE_CFG1_LIMIT	PCIE_CFG1_BASE+PCIE_CFG1_SIZE-1
#define PCIE_IO_LIMIT	PCIE_IO_BASE+PCIE_IO_SIZE-1
#define PCIE_MSI_LIMIT	PCIE_MSI_BASE+PCIE_MSI_SIZE-1
#define PCIE_ATU_LIMIT	PCIE_ATU_BASE+PCIE_ATU_SIZE-1
#define PCIE_MEM_LIMIT	PCIE_MEM_BASE+PCIE_MEM_SIZE-1
#define MSICTR_ADDR	PCIE_AREA_BASE
#else
#undef PCIE_AREA_BASE
#undef PCIE_AREA_SIZE
#undef PCIE_CFG_BASE
#undef PCIE_CFG_SIZE
#undef PCIE_CFG0_BASE
#undef PCIE_CFG0_SIZE
#undef PCIE_CFG1_BASE
#undef PCIE_CFG1_SIZE
#undef PCIE_IO_BASE
#undef PCIE_IO_SIZE
#undef PCIE_MSI_BASE
#undef PCIE_MSI_SIZE
#undef PCIE_ATU_BASE
#undef PCIE_ATU_SIZE
#undef PCIE_MEM_BASE
#undef PCIE_MEM_SIZE
#undef PCIE_CFG0_LIMIT
#undef PCIE_CFG1_LIMIT
#undef PCIE_IO_LIMIT
#undef PCIE_MSI_LIMIT
#undef PCIE_ATU_LIMIT
#undef PCIE_MEM_LIMIT
#undef MSICTR_ADDR
#endif	// EPSON


///////////////////////////////////////////
//  PCIe Port Logic Registers (0x0700)
//////////////////////////////////////////////
#define PL_LAT_REL_TIM		(PORTLOG_OFFSET+0x0)   //0	32	Ack Latency and Replay Timer Register
#define PL_OT_MSG_R		(PORTLOG_OFFSET+0x4)   //0	32	Other Message Register
#define PL_PT_LNK_R		(PORTLOG_OFFSET+0x8)   //0	32	Port Force Link Register
#define PL_ACk_FREQ_R		(PORTLOG_OFFSET+0xC)   //0	32	Ack Frequency Register
#define PL_PT_LNK_CTRL_R	(PORTLOG_OFFSET+0x10)  //0	32	Port Link Control Register
#define PL_LN_SKW_R		(PORTLOG_OFFSET+0x14)  //0	32	Lane Skew Register
#define PL_SYMB_N_R		(PORTLOG_OFFSET+0x18)  //0	32	Symbol Number Register
#define PL_SYMB_T_R		(PORTLOG_OFFSET+0x1C)  //0	32	Symbol Timer and Filter Mask Register 1
#define PL_FL_MSK_R2		(PORTLOG_OFFSET+0x20)  //0	32	Filter Mask Register 2
#define PL_DB_R0		(PORTLOG_OFFSET+0x28)  //0	32	Debug Register 0
#define PL_DB_R1		(PORTLOG_OFFSET+0x2C)  //0	32	Debug Register 1
#define PL_TR_P_STS_R		(PORTLOG_OFFSET+0x30)  //0	32	Transmit Posted FC Credit Status Register
#define PL_TR_NP_STS_R		(PORTLOG_OFFSET+0x34)  //0	32	Transmit Non-Posted FC Credit Status Register
#define PL_TR_C_STS_R		(PORTLOG_OFFSET+0x38)  //0	32	Transmit Completion FC Credit Status Register
#define PL_Q_STS_R		(PORTLOG_OFFSET+0x3C)  //0	32	Queue Status Register
#define PL_VC_TR_A_R1		(PORTLOG_OFFSET+0x40)  //0	32	VC Transmit Arbitration Register 1
#define PL_VC_TR_A_R2		(PORTLOG_OFFSET+0x44)  //0	32	VC Transmit Arbitration Register 2
#define PL_VC0_PR_Q_C		(PORTLOG_OFFSET+0x48)  //0	32	VC0 Posted Receive Queue Control
#define PL_VC0_NPR_Q_C		(PORTLOG_OFFSET+0x4C)  //0	32	VC0 Non-Posted Receive Queue Control
#define PL_VC0_CR_Q_C		(PORTLOG_OFFSET+0x50)  //0	32	VC0 Completion Receive Queue Control
#define PL_VC1_PR_Q_C		(PORTLOG_OFFSET+0x54)  //0	32	VC1 Posted Receive Queue Control
#define PL_VC1_NPR_Q_C		(PORTLOG_OFFSET+0x58)  //0	32	VC1 Non-Posted Receive Queue Control
#define PL_VC1_CR_Q_C		(PORTLOG_OFFSET+0x5C)  //0	32	VC1 Completion Receive Queue Control
#define PL_VC2_PR_Q_C		(PORTLOG_OFFSET+0x60)  //0	32	VC2 Posted Receive Queue Control
#define PL_VC2_NPR_Q_C		(PORTLOG_OFFSET+0x64)  //0	32	VC2 Non-Posted Receive Queue Control
#define PL_VC2_CR_Q_C		(PORTLOG_OFFSET+0x68)  //0	32	VC2 Completion Receive Queue Control
#define PL_VC3_PR_Q_C		(PORTLOG_OFFSET+0x6C)  //0	32	VC3 Posted Receive Queue Control
#define PL_VC3_NPR_Q_C		(PORTLOG_OFFSET+0x70)  //0	32	VC3 Non-Posted Receive Queue Control
#define PL_VC3_CR_Q_C		(PORTLOG_OFFSET+0x74)  //0	32	VC3 Completion Receive Queue Control
#define PL_VC4_PR_Q_C		(PORTLOG_OFFSET+0x78)  //0	32	VC4 Posted Receive Queue Control
#define PL_VC4_NPR_Q_C		(PORTLOG_OFFSET+0x7C)  //0	32	VC4 Non-Posted Receive Queue Control
#define PL_VC4_CR_Q_C		(PORTLOG_OFFSET+0x80)  //0	32	VC4 Completion Receive Queue Control
#define PL_VC5_PR_Q_C		(PORTLOG_OFFSET+0x84)  //0	32	VC5 Posted Receive Queue Control
#define PL_VC5_NPR_Q_C		(PORTLOG_OFFSET+0x88)  //0	32	VC5 Non-Posted Receive Queue Control
#define PL_VC5_CR_Q_C		(PORTLOG_OFFSET+0x8C)  //0	32	VC5 Completion Receive Queue Control
#define PL_VC5_NPR_Q_C		(PORTLOG_OFFSET+0x88)  //0	32	VC5 Non-Posted Receive Queue Control
#define PL_VC5_CR_Q_C		(PORTLOG_OFFSET+0x8C)  //0	32	VC5 Completion Receive Queue Control
#define PL_VC6_PR_Q_C		(PORTLOG_OFFSET+0x90)  //0	32	VC6 Posted Receive Queue Control
#define PL_VC6_NPR_Q_C		(PORTLOG_OFFSET+0x94)  //0	32	VC6 Non-Posted Receive Queue Control
#define PL_VC6_CR_Q_C		(PORTLOG_OFFSET+0x98)  //0	32	VC6 Completion Receive Queue Control
#define PL_VC7_PR_Q_C		(PORTLOG_OFFSET+0x9C)  //0	32	VC7 Posted Receive Queue Control
#define PL_VC7_NPR_Q_C		(PORTLOG_OFFSET+0xA0)  //0	32	VC7 Non-Posted Receive Queue Control
#define PL_VC7_CR_Q_C		(PORTLOG_OFFSET+0xA4)  //0	32	VC7 Completion Receive Queue Control
#define PL_VC0_PB_D		(PORTLOG_OFFSET+0xA8)  //0	32	VC0 Posted Buffer Depth
#define PL_VC0_NPB_D		(PORTLOG_OFFSET+0xAC)  //0	32	VC0 Non-Posted Buffer Depth
#define PL_VC0_CB_D		(PORTLOG_OFFSET+0xB0)  //0	32	VC0 Completion Buffer Depth
#define PL_VC1_PB_D		(PORTLOG_OFFSET+0xB4)  //0	32	VC1 Posted Buffer Depth
#define PL_VC1_NPB_D		(PORTLOG_OFFSET+0xB8)  //0	32	VC1 Non-Posted Buffer Depth
#define PL_VC1_CB_D		(PORTLOG_OFFSET+0xBC)  //0	32	VC1 Completion Buffer Depth
#define PL_VC2_PB_D		(PORTLOG_OFFSET+0xC0)  //0	32	VC2 Posted Buffer Depth
#define PL_VC2_NPB_D		(PORTLOG_OFFSET+0xC4)  //0	32	VC2 Non-Posted Buffer Depth
#define PL_VC2_CB_D		(PORTLOG_OFFSET+0xC8)  //0	32	VC2 Completion Buffer Depth
#define PL_VC3_PB_D		(PORTLOG_OFFSET+0xCC)  //0	32	VC3 Posted Buffer Depth
#define PL_VC3_NPB_D		(PORTLOG_OFFSET+0xD0)  //0	32	VC3 Non-Posted Buffer Depth
#define PL_VC3_CB_D		(PORTLOG_OFFSET+0xD4)  //0	32	VC3 Completion Buffer Depth
#define PL_VC4_PB_D		(PORTLOG_OFFSET+0xD8)  //0	32	VC4 Posted Buffer Depth
#define PL_VC4_NPB_D		(PORTLOG_OFFSET+0xDC)  //0	32	VC4 Non-Posted Buffer Depth
#define PL_VC4_CB_D		(PORTLOG_OFFSET+0xE0)  //0	32	VC4 Completion Buffer Depth
#define PL_VC5_PB_D		(PORTLOG_OFFSET+0xE4)  //0	32	VC5 Posted Buffer Depth
#define PL_VC5_NPB_D		(PORTLOG_OFFSET+0xE8)  //0	32	VC5 Non-Posted Buffer Depth
#define PL_VC5_CB_D		(PORTLOG_OFFSET+0xEC)  //0	32	VC5 Completion Buffer Depth
#define PL_VC6_PB_D		(PORTLOG_OFFSET+0xF0)  //0	32	VC6 Posted Buffer Depth
#define PL_VC6_NPB_D		(PORTLOG_OFFSET+0xF4)  //0	32	VC6 Non-Posted Buffer Depth
#define PL_VC6_CB_D		(PORTLOG_OFFSET+0xF8)  //0	32	VC6 Completion Buffer Depth
#define PL_VC7_PB_D		(PORTLOG_OFFSET+0xFC)  //0	32	VC7 Posted Buffer Depth
#define PL_VC7_NPB_D		(PORTLOG_OFFSET+0x100) //0	32	VC7 Non-Posted Buffer Depth
#define PL_VC7_CB_D		(PORTLOG_OFFSET+0x104) //0	32	VC7 Completion Buffer Depth
#define PL_GEN2			(PORTLOG_OFFSET+0x10C) //0	32	Gen2
#define PL_PHY_STS_R		(PORTLOG_OFFSET+0x110) //0	32	PHY Status Register
#define PL_PHY_CTRL_R		(PORTLOG_OFFSET+0x114) //0	32	PHY Control Register


/////////////////////////////////////////////////////////
//  new feature (from 3.60a)
//  iATU (located in port logic resister, +0x20)
/////////////////////////////////////////////////////////////
#define PL_iATU_VIEWPORT	(0x200)	//iATU Viewport Register
#define PL_iATU_REGION_CNT1	(0x204)	//iATU Region Control 1 Register
#define PL_iATU_REGION_CNT2	(0x208)	//iATU Region Control 2 Register
#define PL_iATU_LOWERBASE_ADDR	(0x20C)	//iATU Region Lower Base Address Register
#define PL_iATU_UPPERBASE_ADDR	(0x210)	//iATU Region Upper Base Address Register
#define PL_iATU_LIMIT_ADDR	(0x214)	//iATU Region Limit Address Register
#define PL_iATU_LOWERTGT_ADDR	(0x218)	//iATU Region Lower Target Address Register
#define PL_iATU_UPPERTGT_ADDR	(0x21C)	//iATU Region Upper Target Address Register

// bit definition for iATU Viewport Register(0x200)
#define iATU_REGION_INDEX	(0xf)		// [3:0] Region Index
#define iATU_REGION_DIRECTION	(0x1<<31)	// [31]  Region Direction. 0:Outbound, 1:Inbound
#define REGION_INBOUND		1
#define REGION_OUTBOUND		0

// bit definition for iATU Region Control 2 Register
#define iATU_REGION_ENABLE	(0x1<<31)	// [31]  Region Enable


/////////////////////////////////////////////////////////////
////  new feature (from 3.60a)
////  MSI Controller (located in port logic resister, +0x120)
///////////////////////////////////////////////////////////////
#define	PL_MSI_CNT_ADDR		(0x120)	//MSI controller Adress
#define	PL_MSI_CNT_ADDR_UPPER	(0x124)	//MSI controller Upper Adress
#define	PL_MSI_CNT_INT_EN(num)	(0x128+(num*0xc)) //MSI controller Interrupt Enable
#define	PL_MSI_CNT_INT_MSK(num)	(0x12C+(num*0xc)) //MSI controller Interrupt Mask
#define	PL_MSI_CNT_INT_STS(num)	(0x130+(num*0xc)) //MSI controller Interrupt Status
#define	PL_MSI_CNT_GPIO		(0x188)		  //MSI controller GPIO Register

///////////////////////////////////////////////
//// MSI Capability Register
/////////////////////////////////////////////////
#define	MSG_CTR			MSI_CTR
#define	MSI_CTR			(CFG_MSI_CAP+0x0)	//0	32	MSI cap structure
#define	MSI_L32			(CFG_MSI_CAP+0x4)	//0	32	MSI Lower 32-bit address register
#define	MSI_U32			(CFG_MSI_CAP+0x8)	//0	32	MSI Upper 32-bit address register
#define	MSI_DATA		(CFG_MSI_CAP+0xC)	//0	32	MSI Data
#define MSI_MASK		(CFG_MSI_CAP+0x10)	//0	32	MSI Mask Bits
#define MSI_PEND		(CFG_MSI_CAP+0x14)	//0	32	MSI Peinding Bits

#define	MSI_CTR_Enbale_MSI		0x00010000	//[16]=1
#define	MSI_CTR_Enbale_MultiMSI		0x005A0000	//[22:20]=101b,[19:17]=101b
#define	MSI_CTR_Enbale_64bit_Addr	0x00800000



///////////////////////////////////////////////
////// PCI Express Capability Register
///////////////////////////////////////////////////
#define PCIE_CAP		(CFG_PCIE_CAP+0x0       )	//0	32	PCIE cap structure
#define CAP_ID			(CFG_PCIE_CAP+0x0       )	//0	8	Capability ID
#define PCIE_NX_PTR		(CFG_PCIE_CAP+0x1       )	//8	8	Next Capability Pointer
#define PCIE_CAP_R		(CFG_PCIE_CAP+0x2       )	//16	16	PCIE Capability Register

#define DEV_CAP			(CFG_PCIE_CAP+0x4       )	//0	32	PCIE Device cap
#define DEV_STS_CTRL		(CFG_PCIE_CAP+0x8       )	//0	32	PCIE cap Device status and control

#define DEV_STS			(CFG_PCIE_CAP+0xA       )	//16	16	PCIE Capability Device Status
#define LNK_CAP			(CFG_PCIE_CAP+0xC       )	//0	32	PCIE Link cap
#define LNK_STS_CTRL		(CFG_PCIE_CAP+0x10      )	//0	32	PCIE cap Link status and control

#define LNK_STS			(CFG_PCIE_CAP+0x12      )	//16	16	PCIE Capability Link Status
#define SLT_CAP			(CFG_PCIE_CAP+0x14      )	//0	32	PCIE Slot cap
#define SLT_STS_CTRL		(CFG_PCIE_CAP+0x18      )	//0	32	PCIE cap Slot status and control

#define SLT_STS			(CFG_PCIE_CAP+0x1A      )	//16	16	PCIE Capability Slot Status
#define RC_CAP_CTRL		(CFG_PCIE_CAP+0x1C      )	//0	32	Root Capability and Control

#define RC_CAP			(CFG_PCIE_CAP+0x1E      )	//16	16	Root Capability
#define RC_STS			(CFG_PCIE_CAP+0x20      )	//0	32	Root Status

///////////////////////////////////////////////
////// PCI Express Extended Capabilities Register (0x100 + ~)
//////  added by eslim (2010/04/05)
///////////////////////////////////////////////////
#define PCIE_XCAP		(ADVERR_OFFSET+0x0)	// PCIE extended cap structure
#define PCIE_XCAP_UCERR_STS	(ADVERR_OFFSET+0x4)	// UnCorrectabl Error Status
#define PCIE_XCAP_UCERR_MSK	(ADVERR_OFFSET+0x8)	// UnCorrectabl Error Mask
#define PCIE_XCAP_UCERR_SEV	(ADVERR_OFFSET+0xC)	// UnCorrectabl Error Severity
#define PCIE_XCAP_CERR_STS	(ADVERR_OFFSET+0x10)	// Correctabl Error Status
#define PCIE_XCAP_CERR_MSK	(ADVERR_OFFSET+0x14)	// Correctabl Error Mask
#define PCIE_XCAP_ADV_CAPNCTRL	(ADVERR_OFFSET+0x18)	// Advanced Capabilities & Control
#define PCIE_XCAP_HEADER_LOG1	(ADVERR_OFFSET+0x1C)	// Header Log (1st DWORD)
#define PCIE_XCAP_HEADER_LOG2	(ADVERR_OFFSET+0x20)	// Header Log (2nd DWORD)
#define PCIE_XCAP_HEADER_LOG3	(ADVERR_OFFSET+0x24)	// Header Log (3rd DWORD)
#define PCIE_XCAP_HEADER_LOG4	(ADVERR_OFFSET+0x28)	// Header Log (4th DWORD)
#define PCIE_XCAP_ROOTERR_CMD	(ADVERR_OFFSET+0x2C)	// Root Error Command
#define PCIE_XCAP_ROOTERR_STS	(ADVERR_OFFSET+0x30)	// Root Error Status
#define PCIE_XCAP_ERRSRC_IDENT	(ADVERR_OFFSET+0x34)	// Error Source Identification

#define PCI_DEVICE_ID_EPSON12	0x0130
#define PCI_VENDOR_ID_EPSON12	0x14eb

#if 0	// EPSON
#define dbi_readb(off)		__raw_readb(dbi + (off))
#define dbi_readw(off)		__raw_readw(dbi + (off))
#define dbi_readl(off)		__raw_readl(dbi + (off))
#define dbi_writeb(val, off)	__raw_writeb(val, dbi + (off))
#define dbi_writew(val, off)	__raw_writew(val, dbi + (off))
#define dbi_writel(val, off)	__raw_writel(val, dbi + (off))
#else
#undef dbi_readb
#undef dbi_readw
#undef dbi_readl
#undef dbi_writeb
#undef dbi_writew
#undef dbi_writel
#endif	// EPSON

#define CFG_PM_CAP	0x40
#define CFG_MSI_CAP	0x50
#define CFG_MSIX_CAP	0xB0
#define CFG_PCIE_CAP	0x70
#define CFG_SLOT_CAP	0xC0
#define CFG_VPD_CAP	0xD0

#define	ADVERR_OFFSET	0x100
#define	VC_CAP_OFFSET	0x140
#define	DEVSERN_OFFSET	0x300
#define	PORTLOG_OFFSET	0x700
#define	BARMASK_OFFSET	0x1000

/* The CFG space of RC starts with DBI */
#define PCIE_RCCFG_OFF		0x0
#if 0	// EPSON
#define rccfg_readb(off)	dbi_readb(PCIE_RCCFG_OFF + (off))
#define rccfg_readw(off)	dbi_readw(PCIE_RCCFG_OFF + (off))
#define rccfg_readl(off)	dbi_readl(PCIE_RCCFG_OFF + (off))
#define rccfg_writeb(val, off)	dbi_writeb(val, PCIE_RCCFG_OFF + (off))
#define rccfg_writew(val, off)	dbi_writew(val, PCIE_RCCFG_OFF + (off))
#define rccfg_writel(val, off)	dbi_writel(val, PCIE_RCCFG_OFF + (off))
#else
#undef rccfg_readb
#undef rccfg_readw
#undef rccfg_readl
#undef rccfg_writeb
#undef rccfg_writew
#undef rccfg_writel
#endif	// EPSON

/* The ATU is located at offset 0x4000 above DBI */
#define PCIE_ATU_OFF	0x4000
#if 0	// EPSON
#define atu_readb(off)		dbi_readb(PCIE_ATU_OFF + (off))
#define atu_readw(off)		dbi_readw(PCIE_ATU_OFF + (off))
#define atu_readl(off)		dbi_readl(PCIE_ATU_OFF + (off))
#define atu_writeb(val, off)	dbi_writeb(val, PCIE_ATU_OFF + (off))
#define atu_writew(val, off)	dbi_writew(val, PCIE_ATU_OFF + (off))
#define atu_writel(val, off)	dbi_writel(val, PCIE_ATU_OFF + (off))
#else
#undef atu_readb
#undef atu_readw
#undef atu_readl
#undef atu_writeb
#undef atu_writew
#undef atu_writel
#endif	// EPSON

/* E-ATU registers */
#define	PCIE_ATU_ID	0x000
#define	PCIE_MODE	0x008
#define	MSTR_CTRL	0x010
#define	MSTR_AWCTRL	0x014
#define	MSTR_ARCTRL	0x018
#define	MSTR_BCTRL	0x01C
#define	MSTR_RCTRL	0x020
#define	SLV_CTRL	0x030
#define	SLV_AWCTRL	0x034
#define	SLV_ARCTRL	0x038
#define	SLV_BCTRL	0x03C
#define	SLV_RCTRL	0x040

#define	APP_CTRL		0x050
#define	APP_PULSE_CTRL		0x054
#define	APP_HDR_LOG0		0x060
#define	APP_HDR_LOG1		0x064
#define	APP_HDR_LOG2		0x068
#define	APP_HDR_LOG3		0x06C
#define	PCIE_STATE_INFO		0x070
#define	CXPL_DEBUG_INFO0	0x074
#define	CXPL_DEBUG_INFO1	0x078
#define	RADM_MSG_PAYLOAD	0x07C
#define	RTLH_RFC_DATA		0x080
#define	TIMEOUT_INFO0		0x084
#define	TIMEOUT_INFO1		0x088
#define	TIMEOUT_INFO2		0x08C
#define	PCIE_ETC_INFO		0x090

#define	PCIE_INT0_ENABLE	0x0A0
#define	PCIE_INT0_SIGNAL	0x0A4
#define	PCIE_INT0_STATUS	0x0A8
#define	PCIE_INT1_ENABLE	0x0AC
#define	PCIE_INT1_SIGNAL	0x0B0
#define	PCIE_INT1_STATUS	0x0B4
#define	PCIE_DOORBELL		0x0C0
#define	PCIE_MSIBELL0		0x0C4
#define	PCIE_MSIBELL1		0x0C8
#define	PCIE_INTMSI		0x0CC
#define	PCIE_MAILBOX0		0x0D0
#define	PCIE_MAILBOX1		0x0D4
#define	PCIE_MAILBOX2		0x0D8
#define	PCIE_MAILBOX3		0x0DC

#define	IN0_MEM_START_LO	0x100
#define	IN0_MEM_LIMIT_LO	0x108
#define	IN1_MEM_START_LO	0x110
#define	IN1_MEM_LIMIT_LO	0x118
#define	IN_IO_START_LO		0x130
#define	IN_IO_LIMIT_LO		0x138
#define	IN_CFG0_START_LO	0x140
#define	IN_CFG0_LIMIT_LO	0x148
#define	IN_CFG1_START_LO	0x150
#define	IN_CFG1_LIMIT_LO	0x158
#define	IN_MSG_START_LO		0x160
#define	IN_MSG_LIMIT_LO		0x168
#define	POM0_MEM_START_LO	0x180
#define	POM0_MEM_START_HI	0x184
#define	POM1_MEM_START_LO	0x188
#define	POM1_MEM_START_HI	0x18C
#define	POM_IO_START_LO		0x1A0
#define	POM_CFG0_START_LO	0x1A8
#define	POM_CFG1_START_LO	0x1B0
#define	POM_MSG_START_LO	0x1B8
#define	POM_MSG_START_HI	0x1BC

#define	PEX0_MEM_START_LO	0x200
#define	PEX0_MEM_START_HI	0x204
#define	PEX0_MEM_LIMIT_LO	0x208
#define	PEX0_MEM_LIMIT_HI	0x20C
#define	PEX1_MEM_START_LO	0x210
#define	PEX1_MEM_START_HI	0x214
#define	PEX1_MEM_LIMIT_LO	0x218
#define	PEX1_MEM_LIMIT_HI	0x21C
#define	PEX2_MEM_START_LO	0x220
#define	PEX2_MEM_START_HI	0x224
#define	PEX2_MEM_LIMIT_LO	0x228
#define	PEX2_MEM_LIMIT_HI	0x22C
#define	PEX_IO_START_LO		0x230
#define	PEX_IO_LIMIT_LO		0x238
#define	IEX0_MEM_START_LO	0x280
#define	IEX1_MEM_START_LO	0x288
#define	IEX2_MEM_START_LO	0x290
#define	IEX_IO_START_LO		0x298
#define	IB_BAR0_START_LO	0x2B0
#define	IB_BAR1_START_LO	0x2B8
#define	IB_BAR2_START_LO	0x2C0
#define	IB_BAR3_START_LO	0x2C8
#define	IB_ROMBAR_START_LO	0x2E0

#define	PHY_MODE		0x300
#define	PHY_CTRL		0x304
#define	PHY_STATUS		0x308
#define	PHY_TX_CTRL0		0x310
#define	PHY_TX_CTRL1		0x314
#define PHY_RX_CTRL		0x318

#define RC_MODE			0x4
#define PCIE_MODE_MASK		0xf
#define APP_LTSSM_ENABLE	0x100

#define RDLH_LINK_UP	(1 << 1)
#define XMLH_LINK_UP	(1 << 0)
#define BOTH_LINK_UP	(RDLH_LINK_UP | XMLH_LINK_UP)
#define MASK_LINK_UP	0x3

#if 0	// EPSON
#define APP_LTSSM_EN()		atu_writel(atu_readl(PCIE_MODE) | APP_LTSSM_ENABLE, PCIE_MODE)
#define APP_LTSSM_DIS()		atu_writel(atu_readl(PCIE_MODE) & ~APP_LTSSM_ENABLE, PCIE_MODE)
#else
#undef APP_LTSSM_EN
#undef APP_LTSSM_DIS
#endif	// EPSON

#define COM_LANE_BASE	0xa00

#define EPSON12_PCIE_DBI_LEN	0x8000
#define EPSON12_PCIE_SFR_LEN	0x8000

#endif	// _EPSON12_PCIE_REGS_H_
