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
#ifndef __EHCI_EPSON09_H
#define __EHCI_EPSON09_H

/* Kconfig compilation options */
/* #define CONFIG_EPSON09_EHCI_DEBUG */
/* #define CONFIG_EPSON09_EHCI_ENABLE_SRAM */

/* DEBUG related definitions */
#if defined(CONFIG_EPSON09_EHCI_DEBUG)
#define DEBUG_PRINT			printk
#else
#define DEBUG_PRINT(...)
#endif

#ifndef CONFIG_EPSON09_BOOT_NO_PRINTK
#define READW(m,r) readw(regs+REG_OFFSET(m,r))
#define WRITEW(v,m,r) writew((v),regs+REG_OFFSET(m,r));	\
	DEBUG_PRINT("<EP> W: %08x = %04x (%s)\n",			\
				REG_ADDRESS(m,r),						\
				readw(regs+REG_OFFSET(m,r)),			\
				((readw(regs+REG_OFFSET(m,r)) == (v)) ? "OK" : "NG"))

#define READL(m,r) readl(regs+REG_OFFSET(m,r))
#define WRITEL(v,m,r) writel((v),regs+REG_OFFSET(m,r));	\
	DEBUG_PRINT("<EP> W: %08x = %08x (%s)\n",			\
				REG_ADDRESS(m,r),						\
				readl(regs+REG_OFFSET(m,r)),			\
				((readl(regs+REG_OFFSET(m,r)) == (v)) ? "OK" : "NG"))
#else
#define READW(m,r) readw(regs+REG_OFFSET(m,r))
#define WRITEW(v,m,r) writew((v),regs+REG_OFFSET(m,r))
#define READL(m,r) readl(regs+REG_OFFSET(m,r))
#define WRITEL(v,m,r) writel((v),regs+REG_OFFSET(m,r))
#endif

#endif	/* __EHCI_EPSON09_H */
