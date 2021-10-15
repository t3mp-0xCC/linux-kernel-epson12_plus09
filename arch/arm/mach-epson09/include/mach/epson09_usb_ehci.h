/*
 *  EPSON09 EHCI Host Controller driver
 *
 *  Copyright (C) 2008-2013 SEIKO EPSON Corp.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef __EPSON09_USB_EHCI_H
#define __EPSON09_USB_EHCI_H

/* common definitions */
#define EPSON09_EHCI_CONTROLLER_NAME	"EPSON09 EHCI Host Controller"
#define EPSON09_EHCI_DEVICE_NAME		"epson09-ehci"

#define EHCI_IRQ						(IRQ_XINT_USBH1)

/* GLUE11/9 base address */
#define GLUE11_REG_ADRS					(0xE0812000)
#define GLUE9_REG_ADRS					(0xD0809000)

/* GLUE registers related definitions */
#define GLUE_REG_ADRS					(GLUE9_REG_ADRS + 0x040)
#define GLUE_REG_ADRS_LEN				(4)

#define GLUE_OFFSET_UHIMR9				(0)

/* SYSCU registers related definitions */
#define SYSCU_REG_ADRS					(GLUE11_REG_ADRS + 0x178)
#define SYSCU_REG_ADRS_LEN				(8)

#define SYSCU_OFFSET_SYCSRST5			(0)
#define SYSCU_OFFSET_SYCSRSTS5			(4)

/* USB Master Bridge registers related definitions */
#define USB_MST_BRG_REG_ADRS			(GLUE9_REG_ADRS + 0x200)
#define USB_MST_BRG_REG_ADRS_LEN		(12)

#define USB_MST_BRG_OFFSET_AMDR9		(0)
#define USB_MST_BRG_OFFSET_SRMCR9		(4)
#define USB_MST_BRG_OFFSET_CDAR9		(8)

/* EHCI registers related definitions */
#define EHCI_REG_ADRS					(0xD0802000)
#define EHCI_REG_ADRS_LEN				(0x1000)

#define EHCI_OFFSET_HCCAPBASE			(0x0000)
#define EHCI_OFFSET_HCSPARAMS			(0x0004)
#define EHCI_OFFSET_HCCPARAMS			(0x0008)
#define EHCI_OFFSET_USBCMD				(0x0010)
#define EHCI_OFFSET_USBSTS				(0x0014)
#define EHCI_OFFSET_USBINTR				(0x0018)
#define EHCI_OFFSET_FRINDEX				(0x001C)
#define EHCI_OFFSET_CTRLDSSEGMENT		(0x0020)
#define EHCI_OFFSET_PERIODICLISTBASE	(0x0024)
#define EHCI_OFFSET_ASYNCLISTADDR		(0x0028)
#define EHCI_OFFSET_CONFIGFLAG			(0x0050)
#define EHCI_OFFSET_PORTSC1				(0x0054)
#define EHCI_OFFSET_EIIBC1				(0x0094)
#define EHCI_OFFSET_EIIBC2				(0x009C)

/* USB TEST registers related definitions */
#if !defined(CONFIG_USB_EPSON09SS_REGISTERSET)
#define USB_TEST_REG_ADRS				(0xD0800000)
#else
#define USB_TEST_REG_ADRS				(0xE0800000)
#endif	/* !defined(CONFIG_ USB_EPSON09SS_REGISTERSET) */
#define USB_TEST_REG_ADRS_LEN			(0x1000)

#define USB_TEST_OFFSET_RESET			(0x0000)
#define USB_TEST_OFFSET_PWEN			(0x0002)
#define USB_TEST_OFFSET_UHTR			(0x0006)
#define USB_TEST_OFFSET_OC_MSK_TIM		(0x0008)
#define USB_TEST_OFFSET_STBY_INT		(0x000C)
#define USB_TEST_OFFSET_INT_CLS			(0x000E)

/* SRAM related definitions */
#define SRAM_REG_ADRS					(0xD0804000)
#define SRAM_REG_ADRS_LEN				(0x2000)

#define SRAM_OFFSET_QTD					(QTD_REG_ADRS-SRAM_REG_ADRS)
#define SRAM_OFFSET_QH					(QH_REG_ADRS-SRAM_REG_ADRS)
#define SRAM_OFFSET_PERIODIC			(PERIODIC_REG_ADRS-SRAM_REG_ADRS)

#define QTD_REG_ADRS					(SRAM_REG_ADRS)
#define QTD_REG_ADRS_LEN				(sizeof(struct ehci_qtd)*QTD_MAXN)
/* BulkIn=11, BulkOut=10, Control=3, Interrupt=1, qh=5; total 30 */
#define QTD_MAXN						(32) /* it depends on DHD */

#define QH_REG_ADRS						(QTD_REG_ADRS+QTD_REG_ADRS_LEN)
#define QH_REG_ADRS_LEN					(sizeof(struct ehci_qh)*QH_MAXN)
/* BulkIn, BulkOut, Control, Interrupt, Dummy; total 5 */
#define QH_MAXN							(8)

#define PERIODIC_REG_ADRS				(QH_REG_ADRS+QH_REG_ADRS_LEN)
#define PERIODIC_REG_ADRS_LEN			(0x1000)

/* Register operation macros */
#define REG_START(m)					(m ## _REG_ADRS)
#define REG_LENGTH(m)					(m ## _REG_ADRS_LEN)
#define REG_END(m)						(REG_START(m) + REG_LENGTH(m) - 1)
#define REG_ADDRESS(m,r)				(REG_START(m) + REG_OFFSET(m,r))
#define REG_OFFSET(m,r)					(m ## _OFFSET_ ## r)

#endif	/* __EPSON09_USB_EHCI_H */
