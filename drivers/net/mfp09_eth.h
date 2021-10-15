/*
 *  MFP09 Ethernet device driver
 *
 *  Copyright (C) 2008 xxxxx
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms and conditions of the GNU General Public License,
 *  version 2, as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 *  more details.
 *  You should have received a copy of the GNU General Public License along 
with
 *  this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 *  The full GNU General Public License is included in this distribution in
 *  the file called "COPYING".
 */

/*
 *  MFP09 Ethernet device driver  Ver1.00 08.07.28
 *
 */

#ifndef __MFP09_ETH_H__
#define __MFP09_ETH_H__

#define __LITTLE_ENDIAN__
#define CONFIG_CPU_LITTLE_ENDIAN

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/netdevice.h>

#define CARDNAME       "mfp09"
#define TX_TIMEOUT     (5*HZ)

#define TX_RING_SIZE   128     /* Tx ring size */
#define RX_RING_SIZE   128     /* Rx ring size */

#define TX_BUF_SIZE   0x1000   /* Tx buffer size */
#define RX_BUF_SIZE   0x1000   /* Rx buffer size */
#define RX_OFFSET              2       /* skb offset */
#define ETHERSMALL             60
#define PKT_BUF_SZ             1538

/* Chip Registers */
/* E-DMAC */
#define CXR0   0x0000
#define CXR1  0x0008
#define CXR2  0x0010
#define CXR3  0x0018
#define CXR4  0x0020
#define CXR5   0x0028
#define CXR6 0x0030
#define CXR8  0x0040
#define CXR9   0x0048
#define CXR10    0x0050
#define CXR11   0x0058
#define CXR16  0x0070
#define CXRS2  0x0078
#define CXR19  0x0080
#define CXR18  0x007C
#define CXRD0  0x00C8
#define CXRD1  0x00CC
#define CXRD3  0x00D4
#define CXRD4  0x00D8
/* Ether Register */
#define CXR20   0x0100
#define CXR21   0x0110
#define CXR22 0x0118
#define CXR23    0x0120
#define CXR24   0x01C0
#define CXR25   0x01C8
#define CXR2A   0x0108
#define CXR2B    0x0128
#define CXR40  0x01D0
#define CXR41   0x01D4
#define CXR42   0x01D8
#define CXR43  0x01DC
#define CXR50  0x01E4
#define CXR51  0x01E8
#define CXR52 0x01EC
#define CXR53 0x01F0
#define CXR54   0x01F4
#define CXR55  0x01F8
#define CXR70   0x0150
#define CXR71   0x0154
#define CXR72   0x0158


/* PHY */
#define PHYCR  (0x0000)


/* Register's bits */

/* CXR0 */
enum DMAC_M_BIT {
       CXR0_DE = 0x40, CXR0_DL1 = 0x20, CXR0_DL0 = 0x10, CXR0_SRST = 0x01,
};

/* CXR1 */
enum DMAC_T_BIT {
       CXR1_TRNS = 0x01,
};

/* CXR2*/
enum CXR2_R_BIT {
       CXR2_R = 0x01,
};

/* CXR23 */
enum CXR23_BIT {
       CXR23_MDI = 0x08, CXR23_MDO = 0x04, CXR23_MMD = 0x02, CXR23_MDC = 0x01,
};

/* CXR2B */
enum PHY_STATUS_BIT { PHY_ST_LINK = 0x01, };

/* CXR5 */
enum CXR5_BIT {
       CXR5_TWB = 0x40000000, CXR5_TABT = 0x04000000,
       CXR5_RABT = 0x02000000, CXR5_RFRMER = 0x01000000,
       CXR5_ADE = 0x00800000, CXR5_ECI = 0x00400000,
       CXR5_FTC = 0x00200000, CXR5_TDE = 0x00100000,
       CXR5_TFE = 0x00080000, CXR5_FRC = 0x00040000,
       CXR5_RDE = 0x00020000, CXR5_RFE = 0x00010000,
       CXR5_TINT4 = 0x00000800, CXR5_TINT3 = 0x00000400,
       CXR5_TINT2 = 0x00000200, CXR5_TINT1 = 0x00000100,
       CXR5_RINT8 = 0x00000080, CXR5_RINT5 = 0x00000010,
       CXR5_RINT4 = 0x00000008, CXR5_RINT3 = 0x00000004,
       CXR5_RINT2 = 0x00000002, CXR5_RINT1 = 0x00000001,
};

#define CXR5_ERR_CHECK (CXR5_TWB | CXR5_TABT | CXR5_RABT | CXR5_RDE \
               | CXR5_RFRMER | CXR5_ADE | CXR5_TFE | CXR5_TDE | CXR5_ECI)

/* CXR6 */
enum DMAC_IM_BIT {
       DMAC_M_TWB = 0x40000000, DMAC_M_TABT = 0x04000000,
       DMAC_M_RABT = 0x02000000,
       DMAC_M_RFRMER = 0x01000000, DMAC_M_ADF = 0x00800000,
       DMAC_M_ECI = 0x00400000, DMAC_M_FTC = 0x00200000,
       DMAC_M_TDE = 0x00100000, DMAC_M_TFE = 0x00080000,
       DMAC_M_FRC = 0x00040000, DMAC_M_RDE = 0x00020000,
       DMAC_M_RFE = 0x00010000, DMAC_M_TINT4 = 0x00000800,
       DMAC_M_TINT3 = 0x00000400, DMAC_M_TINT2 = 0x00000200,
       DMAC_M_TINT1 = 0x00000100, DMAC_M_RINT8 = 0x00000080,
       DMAC_M_RINT5 = 0x00000010, DMAC_M_RINT4 = 0x00000008,
       DMAC_M_RINT3 = 0x00000004, DMAC_M_RINT2 = 0x00000002,
       DMAC_M_RINT1 = 0x00000001,
};

/* Receive descriptor bit */
enum RD_STS_BIT {
       RD_RACT = 0x80000000, RC_RDEL = 0x40000000,
       RC_RFP1 = 0x20000000, RC_RFP0 = 0x10000000,
       RD_RFE = 0x08000000, RD_RFS10 = 0x00000200,
       RD_RFS9 = 0x00000100, RD_RFS8 = 0x00000080,
       RD_RFS7 = 0x00000040, RD_RFS6 = 0x00000020,
       RD_RFS5 = 0x00000010, RD_RFS4 = 0x00000008,
       RD_RFS3 = 0x00000004, RD_RFS2 = 0x00000002,
       RD_RFS1 = 0x00000001,
};
#define RDF1ST RC_RFP1
#define RDFEND RC_RFP0
#define RD_RFP (RC_RFP1|RC_RFP0)

/* CXR19 */
enum CXR19_BIT {
       CXR19_RFF2 = 0x00040000, CXR19_RFF1 = 0x00020000,
       CXR19_RFF0 = 0x00010000, CXR19_RFD2 = 0x00000004,
       CXR19_RFD1 = 0x00000002, CXR19_RFD0 = 0x00000001,
};
#define FIFO_F_D_RFF   (CXR19_RFF2|CXR19_RFF1|CXR19_RFF0)
#define FIFO_F_D_RFD   (CXR19_RFD2|CXR19_RFD1|CXR19_RFD0)

/* Transfer descriptor bit */
enum TD_STS_BIT {
       TD_TACT = 0x80000000, TD_TDLE = 0x40000000, TD_TFP1 = 0x20000000,
       TD_TFP0 = 0x10000000,
};
#define TDF1ST TD_TFP1
#define TDFEND TD_TFP0
#define TD_TFP (TD_TFP1|TD_TFP0)

/* CXR11 */
enum RECV_RST_BIT { CXR11_RST = 0x01, };
/* CXR20 */
enum FELIC_MODE_BIT {
       CXR20_ZPF = 0x00080000, CXR20_PFR = 0x00040000, CXR20_RXF = 0x00020000,
       CXR20_TXF = 0x00010000, CXR20_MCT = 0x00002000, CXR20_PRCEF = 0x00001000,
       CXR20_PMDE = 0x00000200, CXR20_RE = 0x00000040, CXR20_TE = 0x00000020,
       CXR20_ILB = 0x00000008, CXR20_ELB = 0x00000004, CXR20_DM = 0x00000002,
       CXR20_PRM = 0x00000001,
};

/* CXR21 */
enum CXR21_STATUS_BIT {
       CXR21_BRCRX = 0x20, CXR21_PSRTO = 0x10, CXR21_LCHNG = 0x04,
       CXR21_MPD = 0x02, CXR21_ICD = 0x01,
};

/* CXR22 */
enum CXR22_STATUS_MASK_BIT {
       CXR22_BRCRXIP = 0x20, CXR22_PSRTOIP = 0x10, CXR22_LCHNGIP = 0x04,
       CXR22_MPDIP = 0x02, CXR22_ICDIP = 0x01,
};

/* CXR10 */
enum FIFO_SIZE_BIT {
       FIFO_SIZE_T = 0x00000700, FIFO_SIZE_R = 0x00000007,
};
enum phy_offsets {
       PHY_CTRL = 0, PHY_STAT = 1, PHY_IDT1 = 2, PHY_IDT2 = 3,
       PHY_ANA = 4, PHY_ANL = 5, PHY_ANE = 6,
       PHY_16 = 16,
};

/* PHY_CTRL */
enum PHY_CTRL_BIT {
       PHY_C_RESET = 0x8000, PHY_C_LOOPBK = 0x4000, PHY_C_SPEEDSL = 0x2000,
       PHY_C_ANEGEN = 0x1000, PHY_C_PWRDN = 0x0800, PHY_C_ISO = 0x0400,
       PHY_C_RANEG = 0x0200, PHY_C_DUPLEX = 0x0100, PHY_C_COLT = 0x0080,
};
#define DM9161_PHY_C_ANEGEN 0  /* auto nego special */

/* PHY_STAT */
enum PHY_STAT_BIT {
       PHY_S_100T4 = 0x8000, PHY_S_100X_F = 0x4000, PHY_S_100X_H = 0x2000,
       PHY_S_10T_F = 0x1000, PHY_S_10T_H = 0x0800, PHY_S_ANEGC = 0x0020,
       PHY_S_RFAULT = 0x0010, PHY_S_ANEGA = 0x0008, PHY_S_LINK = 0x0004,
       PHY_S_JAB = 0x0002, PHY_S_EXTD = 0x0001,
};

/* PHY_ANA */
enum PHY_ANA_BIT {
       PHY_A_NP = 0x8000, PHY_A_ACK = 0x4000, PHY_A_RF = 0x2000,
       PHY_A_FCS = 0x0400, PHY_A_T4 = 0x0200, PHY_A_FDX = 0x0100,
       PHY_A_HDX = 0x0080, PHY_A_10FDX = 0x0040, PHY_A_10HDX = 0x0020,
       PHY_A_SEL = 0x001f,
};
/* PHY_ANL */
enum PHY_ANL_BIT {
       PHY_L_NP = 0x8000, PHY_L_ACK = 0x4000, PHY_L_RF = 0x2000,
       PHY_L_FCS = 0x0400, PHY_L_T4 = 0x0200, PHY_L_FDX = 0x0100,
       PHY_L_HDX = 0x0080, PHY_L_10FDX = 0x0040, PHY_L_10HDX = 0x0020,
       PHY_L_SEL = 0x001f,
};

/* PHY_ANE */
enum PHY_ANE_BIT {
       PHY_E_PDF = 0x0010, PHY_E_LPNPA = 0x0008, PHY_E_NPA = 0x0004,
       PHY_E_PRX = 0x0002, PHY_E_LPANEGA = 0x0001,
};

#define POST_RX                0x08
#define POST_FW                0x04
#define POST0_RX       (POST_RX)
#define POST0_FW       (POST_FW)
#define POST1_RX       (POST_RX >> 2)
#define POST1_FW       (POST_FW >> 2)
#define POST_ALL       (POST0_RX | POST0_FW | POST1_RX | POST1_FW)


/*
 * The sh ether Tx buffer descriptors.
 * This structure should be 20 bytes.
 */
struct mfp09_eth_txdesc {
       u32 status;             /* TD0 */
#if defined(CONFIG_CPU_LITTLE_ENDIAN)
       u16 pad0;               /* TD1 */
       u16 buffer_length;      /* TD1 */
#else
       u16 buffer_length;      /* TD1 */
       u16 pad0;               /* TD1 */
#endif
       u32 addr;               /* TD2 */
       u32 pad1;               /* padding data */
};

/*
 * The sh ether Rx buffer descriptors.
 * This structure should be 20 bytes.
 */
struct mfp09_eth_rxdesc {
       u32 status;             /* RD0 */
#if defined(CONFIG_CPU_LITTLE_ENDIAN)
       u16 frame_length;       /* RD1 */
       u16 buffer_length;      /* RD1 */
#else
       u16 buffer_length;      /* RD1 */
       u16 frame_length;       /* RD1 */
#endif
       u32 addr;               /* RD2 */
       u32 pad0;               /* padding data */
};

struct mfp09_eth_private {
       dma_addr_t rx_desc_dma;
       dma_addr_t tx_desc_dma;
       struct mfp09_eth_rxdesc *rx_ring;
       struct mfp09_eth_txdesc *tx_ring;
       struct sk_buff **rx_skbuff;
       struct sk_buff **tx_skbuff;
       struct net_device_stats stats;
       struct timer_list timer;
       spinlock_t lock;
       u32 cur_rx, dirty_rx;   /* Producer/consumer ring indices */
       u32 cur_tx, dirty_tx;
       u32 rx_buf_sz;          /* Based on MTU+slack. */
       /* MII transceiver section. */
	    spinlock_t	mdio_lock;				/* Serialise access to mdio hardware */
		struct mii_if_info mii;
		struct timer_list 	mii_timer;
       int link;
       int msg_enable;
       int speed;
       int duplex;
       u32 rx_int_var, tx_int_var;     /* interrupt control variables */
       char post_rx;           /* POST receive */
       char post_fw;           /* POST forward */
       struct net_device_stats tsu_stats;      /* TSU forward status */

		/* ethtool */
		unsigned long link_speed;
		unsigned long link_duplex;
		unsigned int supported_features;
		unsigned int advertised_features;
	
		/* OS defined structs */
		struct net_device *dev;

		/* hw value */
		unsigned int autoneg;
	    	unsigned int autoneg_advertised;
		unsigned int forced_speed_duplex;
		unsigned int max_frame_size;    /* Maximum frame size supported     */
		unsigned int mc_filter_type;    /* Multicast filter hash type       */
		unsigned int num_mc_addrs;      /* Number of current Multicast addrs */
      
};

/* ethtool */
#define SPEED_10    10
#define SPEED_100   100
#define SPEED_1000  1000
typedef enum
{
    mfp09_10_half = 0,
    mfp09_10_full = 1,
    mfp09_100_half = 2,
    mfp09_100_full = 3
} mfp09_speed_duplex_type; 

#ifdef __LITTLE_ENDIAN__
static inline void swaps(char *src, int len)
{
       u32 *p = (u32 *)src;
       u32 *maxp;
       maxp = p + ((len + sizeof(u32) - 1) / sizeof(u32));

       for (; p < maxp; p++)
               *p = swab32(*p);
}
#else
#define swaps(x, y)
#endif

#endif /* __MFP09_ETH_H__ */

