/* Copyright (c)SEIKO EPSON CORPORATION 2013. All rights reserved. */

#ifndef __EHCI_SEC09_H
#define __EHCI_SEC09_H

#include <linux/ioctl.h>

#define SEC_EHCI_MAJOR_NUM	248
#define SEC_EHCI_CHRDEV_NAME	"ehci-vbuspower"

#define SEC_EHCI_MAGIC 'E'

#define SEC_EHCI_PORT_OFF	_IO(SEC_EHCI_MAGIC, 0)
#define SEC_EHCI_PORT_ON	_IO(SEC_EHCI_MAGIC, 1)

#endif	/* __EHCI_SEC09_H */
