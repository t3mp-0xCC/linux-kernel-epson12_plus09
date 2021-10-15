#
# Makefile for the linux kernel.
# Copyright (c)SEIKO EPSON CORPORATION 2012. All rights reserved.
#

obj-y				:= epson12.o epson12_clock.o epson12_gic.o epson12_ps.o
obj-$(CONFIG_MACH_EPSON12_M)	+= epson12_m.o
obj-$(CONFIG_MACH_EPSON12_H)	+= epson12_h.o
obj-$(CONFIG_EPSON12_TIMER)	+= epson12_timer.o

obj-$(CONFIG_EPSON12_PCIE_PORT1)	+= epson12_pcie1.o
obj-$(CONFIG_EPSON12_PCIE_PORT0)	+= epson12_pcie0.o

ifdef CONFIG_STMMAC_ETH
obj-y				+= epson12_eth.o
else
ifdef CONFIG_STMMAC_ETH_MODULE
obj-y				+= epson12_eth.o
endif
endif
