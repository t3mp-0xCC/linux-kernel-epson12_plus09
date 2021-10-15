/*
 * Copyright (c)SEIKO EPSON CORPORATION 2012. All rights reserved.
 *
 * License : GPL2.0
 */

#ifndef _EPSON12_PCIE_H_
#define _EPSON12_PCIE_H_

#define IRQ_PCIE0	(54)
#define IRQ_PCIE1	(55)

extern int epson12_pcie0_recovery(void);
extern int epson12_pcie0_stop(void);
extern int epson12_pcie1_recovery(void);
extern int epson12_pcie1_stop(void);

#endif	// _EPSON12_PCIE_H_
