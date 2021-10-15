/*
 * linux/arch/arm/mach/gicu.h
 *
 *   COPYRIGHT (C) 2008 SEIKO EPSON, CORPORATION, All rights reserved.
 *
 *  License : GPL2.0
 */
#include <mach/epson09.h>
#include <mach/irqs.h>

void epson09_gicu_dist_init(void __iomem *base);

/* value for interrupt control register factor */
#define ICCNT_INT_STOP	0x000F0000
#define ICCNT_INT_CLEAR	0x10000000
#define ICCNT_INIT_EMG_STOP_FROM_CORE0	0x00210000	/* IRQ_EMG_STOP_FROM_CORE0  */
#define ICCNT_INIT_ITIM_9_0	0x00010000	/* IRQ_INT_ITIM_9_0 */
#define ICCNT_INIT_SCIF0_RERR	0x002E0000	/* IRQ_SCIF0_RERR */
#define ICCNT_INIT_SCIF0_BRK	0x002D0000	/* IRQ_SCIF0_BRK */
#define ICCNT_INIT_SCIF0_RXI	0x002C0000	/* IRQ_SCIF0_RXI */
#define ICCNT_INIT_SCIF0_TXI	0x002B0000	/* IRQ_SCIF0_TXI */
#define ICCNT_INIT_XINT_USW1	0x00230000	/* IRQ_XINT_USW1 (IDC) */

#define ICCNT_INIT_XINT_PC0	0x00240000	/* IRQ_XINT_PC0 */
#define ICCNT_INIT_SCIF1_RERR	0x002E0000	/* IRQ_SCIF1_RERR */
#define ICCNT_INIT_SCIF1_BRK	0x002D0000	/* IRQ_SCIF1_BRK */
#define ICCNT_INIT_SCIF1_RXI	0x002C0000	/* IRQ_SCIF1_RXI */
#define ICCNT_INIT_SCIF1_TXI	0x002B0000	/* IRQ_SCIF1_TXI */
#define ICCNT_INIT_SCIF2_RERR	0x002E0000	/* IRQ_SCIF2_RERR */
#define ICCNT_INIT_SCIF2_BRK	0x002D0000	/* IRQ_SCIF2_BRK */
#define ICCNT_INIT_SCIF2_RXI	0x002C0000	/* IRQ_SCIF2_RXI */
#define ICCNT_INIT_SCIF2_TXI	0x002B0000	/* IRQ_SCIF2_TXI */


#define ICCNT_INIT_XINT_USBH1	0x00250000	/* IRQ_XINT_USBH1 */
#define ICCNT_INIT_INT_ETHER	0x00260000	/* IRQ_INT_ETHER */
#define ICCNT_INIT_INT_IPSEC	0x00270000	/* IRQ_INT_IPSEC */
#define ICCNT_INIT_INT_WDT_9	0x00280000	/* IRQ_INT_WDT_9 */
#define ICCNT_INIT_INT_ITIM_9_1	0x00290000	/* IRQ_INT_ITIM_9_1 */
#define ICCNT_INIT_GOBI_9_VOIDINT	0x002A0000	/* IRQ_GOBI_9_VOIDINT */

#define ICCNT_INIT_DEFAULT	0x002F0000	/* other */

/* register address offset */
#define EPSON09_GICU_ICSTS	0x000	/* interrupt status register */
#define EPSON09_GICU_ICIMSK	0x008	/* interrupt mask level control register */

#define EPSON09_GICU_ICREQ0	0x010	/* interrupt request register 0 */
#define EPSON09_GICU_ICREQ1	0x014	/* interrupt request register 1 */
#define EPSON09_GICU_ICREQ2	0x018	/* interrupt request register 2 */
#define EPSON09_GICU_ICREQ3	0x01C	/* interrupt request register 3 */

#define EPSON09_GICU_ICCNT0	0x040	/* interrupt control register factor 0 */
#define EPSON09_GICU_ICCNT1	0x044	/* interrupt control register factor 1 */
#define EPSON09_GICU_ICCNT2	0x048	/* interrupt control register factor 2 */
#define EPSON09_GICU_ICCNT3	0x04C	/* interrupt control register factor 3 */
#define EPSON09_GICU_ICCNT4	0x050	/* interrupt control register factor 4 */
#define EPSON09_GICU_ICCNT5	0x054	/* interrupt control register factor 5 */
#define EPSON09_GICU_ICCNT6	0x058	/* interrupt control register factor 6 */
#define EPSON09_GICU_ICCNT7	0x05C	/* interrupt control register factor 7 */
#define EPSON09_GICU_ICCNT8	0x060	/* interrupt control register factor 8 */
#define EPSON09_GICU_ICCNT9	0x064	/* interrupt control register factor 9 */
#define EPSON09_GICU_ICCNT10	0x068	/* interrupt control register factor 10 */
#define EPSON09_GICU_ICCNT11	0x06C	/* interrupt control register factor 11 */
#define EPSON09_GICU_ICCNT12	0x070	/* interrupt control register factor 12 */
#define EPSON09_GICU_ICCNT13	0x074	/* interrupt control register factor 13 */
#define EPSON09_GICU_ICCNT14	0x078	/* interrupt control register factor 14 */
#define EPSON09_GICU_ICCNT15	0x07C	/* interrupt control register factor 15 */
#define EPSON09_GICU_ICCNT16	0x080	/* interrupt control register factor 16 */
#define EPSON09_GICU_ICCNT17	0x084	/* interrupt control register factor 17 */
#define EPSON09_GICU_ICCNT18	0x088	/* interrupt control register factor 18 */
#define EPSON09_GICU_ICCNT19	0x08C	/* interrupt control register factor 19 */
#define EPSON09_GICU_ICCNT20	0x090	/* interrupt control register factor 20 */
#define EPSON09_GICU_ICCNT21	0x094	/* interrupt control register factor 21 */
#define EPSON09_GICU_ICCNT22	0x098	/* interrupt control register factor 22 */
#define EPSON09_GICU_ICCNT23	0x09C	/* interrupt control register factor 23 */
#define EPSON09_GICU_ICCNT24	0x0A0	/* interrupt control register factor 24 */
#define EPSON09_GICU_ICCNT25	0x0A4	/* interrupt control register factor 25 */
#define EPSON09_GICU_ICCNT26	0x0A8	/* interrupt control register factor 26 */
#define EPSON09_GICU_ICCNT27	0x0AC	/* interrupt control register factor 27 */
#define EPSON09_GICU_ICCNT28	0x0B0	/* interrupt control register factor 28 */
#define EPSON09_GICU_ICCNT29	0x0B4	/* interrupt control register factor 29 */
#define EPSON09_GICU_ICCNT30	0x0B8	/* interrupt control register factor 30 */
#define EPSON09_GICU_ICCNT31	0x0BC	/* interrupt control register factor 31 */
#define EPSON09_GICU_ICCNT32	0x0C0	/* interrupt control register factor 32 */
#define EPSON09_GICU_ICCNT33	0x0C4	/* interrupt control register factor 33 */
#define EPSON09_GICU_ICCNT34	0x0C8	/* interrupt control register factor 34 */
#define EPSON09_GICU_ICCNT35	0x0CC	/* interrupt control register factor 35 */
#define EPSON09_GICU_ICCNT36	0x0D0	/* interrupt control register factor 36 */
#define EPSON09_GICU_ICCNT37	0x0D4	/* interrupt control register factor 37 */
#define EPSON09_GICU_ICCNT38	0x0D8	/* interrupt control register factor 38 */
#define EPSON09_GICU_ICCNT39	0x0DC	/* interrupt control register factor 39 */
#define EPSON09_GICU_ICCNT40	0x0E0	/* interrupt control register factor 40 */
#define EPSON09_GICU_ICCNT41	0x0E4	/* interrupt control register factor 41 */
#define EPSON09_GICU_ICCNT42	0x0E8	/* interrupt control register factor 42 */
#define EPSON09_GICU_ICCNT43	0x0EC	/* interrupt control register factor 43 */
#define EPSON09_GICU_ICCNT44	0x0F0	/* interrupt control register factor 44 */
#define EPSON09_GICU_ICCNT45	0x0F4	/* interrupt control register factor 45 */
#define EPSON09_GICU_ICCNT46	0x0F8	/* interrupt control register factor 46 */
#define EPSON09_GICU_ICCNT47	0x0FC	/* interrupt control register factor 47 */
#define EPSON09_GICU_ICCNT48	0x100	/* interrupt control register factor 48 */
#define EPSON09_GICU_ICCNT49	0x104	/* interrupt control register factor 49 */
#define EPSON09_GICU_ICCNT50	0x108	/* interrupt control register factor 50 */
#define EPSON09_GICU_ICCNT51	0x10C	/* interrupt control register factor 51 */
#define EPSON09_GICU_ICCNT52	0x110	/* interrupt control register factor 52 */
#define EPSON09_GICU_ICCNT53	0x114	/* interrupt control register factor 53 */
#define EPSON09_GICU_ICCNT54	0x118	/* interrupt control register factor 54 */
#define EPSON09_GICU_ICCNT55	0x11C	/* interrupt control register factor 55 */
#define EPSON09_GICU_ICCNT56	0x120	/* interrupt control register factor 56 */
#define EPSON09_GICU_ICCNT57	0x124	/* interrupt control register factor 57 */
#define EPSON09_GICU_ICCNT58	0x128	/* interrupt control register factor 58 */
#define EPSON09_GICU_ICCNT59	0x12C	/* interrupt control register factor 59 */
#define EPSON09_GICU_ICCNT60	0x130	/* interrupt control register factor 60 */
#define EPSON09_GICU_ICCNT61	0x134	/* interrupt control register factor 61 */
#define EPSON09_GICU_ICCNT62	0x138	/* interrupt control register factor 62 */
#define EPSON09_GICU_ICCNT63	0x13C	/* interrupt control register factor 63 */
#define EPSON09_GICU_ICCNT64	0x140	/* interrupt control register factor 64 */
#define EPSON09_GICU_ICCNT65	0x144	/* interrupt control register factor 65 */
#define EPSON09_GICU_ICCNT66	0x148	/* interrupt control register factor 66 */
#define EPSON09_GICU_ICCNT67	0x14C	/* interrupt control register factor 67 */
#define EPSON09_GICU_ICCNT68	0x150	/* interrupt control register factor 68 */
#define EPSON09_GICU_ICCNT69	0x154	/* interrupt control register factor 69 */
#define EPSON09_GICU_ICCNT70	0x158	/* interrupt control register factor 70 */
#define EPSON09_GICU_ICCNT71	0x15C	/* interrupt control register factor 71 */
#define EPSON09_GICU_ICCNT72	0x160	/* interrupt control register factor 72 */
#define EPSON09_GICU_ICCNT73	0x164	/* interrupt control register factor 73 */
#define EPSON09_GICU_ICCNT74	0x168	/* interrupt control register factor 74 */
#define EPSON09_GICU_ICCNT75	0x16C	/* interrupt control register factor 75 */
#define EPSON09_GICU_ICCNT76	0x170	/* interrupt control register factor 76 */
#define EPSON09_GICU_ICCNT77	0x174	/* interrupt control register factor 77 */
#define EPSON09_GICU_ICCNT78	0x178	/* interrupt control register factor 78 */
#define EPSON09_GICU_ICCNT79	0x17C	/* interrupt control register factor 79 */
#define EPSON09_GICU_ICCNT80	0x180	/* interrupt control register factor 80 */
#define EPSON09_GICU_ICCNT81	0x184	/* interrupt control register factor 81 */
#define EPSON09_GICU_ICCNT82	0x188	/* interrupt control register factor 82 */
#define EPSON09_GICU_ICCNT83	0x18C	/* interrupt control register factor 83 */
#define EPSON09_GICU_ICCNT84	0x190	/* interrupt control register factor 84 */
#define EPSON09_GICU_ICCNT85	0x194	/* interrupt control register factor 85 */
#define EPSON09_GICU_ICCNT86	0x198	/* interrupt control register factor 86 */
#define EPSON09_GICU_ICCNT87	0x19C	/* interrupt control register factor 87 */
#define EPSON09_GICU_ICCNT88	0x1A0	/* interrupt control register factor 88 */
#define EPSON09_GICU_ICCNT89	0x1A4	/* interrupt control register factor 89 */
#define EPSON09_GICU_ICCNT90	0x1A8	/* interrupt control register factor 90 */
#define EPSON09_GICU_ICCNT91	0x1AC	/* interrupt control register factor 91 */
#define EPSON09_GICU_ICCNT92	0x1B0	/* interrupt control register factor 92 */
#define EPSON09_GICU_ICCNT93	0x1B4	/* interrupt control register factor 93 */
#define EPSON09_GICU_ICCNT94	0x1B8	/* interrupt control register factor 94 */
#define EPSON09_GICU_ICCNT95	0x1BC	/* interrupt control register factor 95 */
#define EPSON09_GICU_ICCNT96	0x1C0	/* interrupt control register factor 96 */
#define EPSON09_GICU_ICCNT97	0x1C4	/* interrupt control register factor 97 */
#define EPSON09_GICU_ICCNT98	0x1C8	/* interrupt control register factor 98 */
#define EPSON09_GICU_ICCNT99	0x1CC	/* interrupt control register factor 99 */
#define EPSON09_GICU_ICCNT100	0x1D0	/* interrupt control register factor 100 */
#define EPSON09_GICU_ICCNT101	0x1D4	/* interrupt control register factor 101 */
#define EPSON09_GICU_ICCNT102	0x1D8	/* interrupt control register factor 102 */
#define EPSON09_GICU_ICCNT103	0x1DC	/* interrupt control register factor 103 */
#define EPSON09_GICU_ICCNT104	0x1E0	/* interrupt control register factor 104 */
#define EPSON09_GICU_ICCNT105	0x1E4	/* interrupt control register factor 105 */
#define EPSON09_GICU_ICCNT106	0x1E8	/* interrupt control register factor 106 */
#define EPSON09_GICU_ICCNT107	0x1EC	/* interrupt control register factor 107 */
#define EPSON09_GICU_ICCNT108	0x1F0	/* interrupt control register factor 108 */
#define EPSON09_GICU_ICCNT109	0x1F4	/* interrupt control register factor 109 */
#define EPSON09_GICU_ICCNT110	0x1F8	/* interrupt control register factor 110 */
#define EPSON09_GICU_ICCNT111	0x1FC	/* interrupt control register factor 111 */
#define EPSON09_GICU_ICCNT112	0x200	/* interrupt control register factor 112 */
#define EPSON09_GICU_ICCNT113	0x204	/* interrupt control register factor 113 */
#define EPSON09_GICU_ICCNT114	0x208	/* interrupt control register factor 114 */
#define EPSON09_GICU_ICCNT115	0x20C	/* interrupt control register factor 115 */
#define EPSON09_GICU_ICCNT116	0x210	/* interrupt control register factor 116 */
#define EPSON09_GICU_ICCNT117	0x214	/* interrupt control register factor 117 */
#define EPSON09_GICU_ICCNT118	0x218	/* interrupt control register factor 118 */
#define EPSON09_GICU_ICCNT119	0x21C	/* interrupt control register factor 119 */
#define EPSON09_GICU_ICCNT120	0x220	/* interrupt control register factor 120 */
#define EPSON09_GICU_ICCNT121	0x224	/* interrupt control register factor 121 */
#define EPSON09_GICU_ICCNT122	0x228	/* interrupt control register factor 122 */
#define EPSON09_GICU_ICCNT123	0x22C	/* interrupt control register factor 123 */
#define EPSON09_GICU_ICCNT124	0x230	/* interrupt control register factor 124 */
#define EPSON09_GICU_ICCNT125	0x234	/* interrupt control register factor 125 */
#define EPSON09_GICU_ICCNT126	0x238	/* interrupt control register factor 126 */
#define EPSON09_GICU_ICCNT127	0x23C	/* interrupt control register factor 127 */

