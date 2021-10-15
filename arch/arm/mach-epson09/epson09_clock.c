/*
 *  Copyright (C) 2009 Samsung Electronics TLD.
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
 *
 * Base on linux/arch/arm/mach-realview/clock.c
 *
 * History
 *  12/29/09	jiseong oh		source code create
 *
 */

/* Copyright (c)SEIKO EPSON CORPORATION 2012. All rights reserved. */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#include <asm/io.h>
#include <asm/hardware/icst.h> // name change from 2.6.35

#include <mach/clock.h>
#include <mach/irqs.h>

#if defined(CONFIG_MACH_EPSON09_DBG)
#define DBG(args...)	printk(args)
#define IFDBG(condition, args...)	if(condition)	printk(args)
#else
#define DBG(args...)	do {} while(0)
#define IFDBG(condition, args...)	do {} while(0)
#endif

static LIST_HEAD(clocks);
static DEFINE_MUTEX(clocks_mutex);

unsigned long UART_EXTCLK;

struct clk *clk_get(struct device *dev, const char *id)
{
	struct clk *p, *clk = ERR_PTR(-ENOENT);
	IFDBG(!dev,"%s *dev is NULL\n",__FUNCTION__);

	mutex_lock(&clocks_mutex);
	list_for_each_entry(p, &clocks, node) {
		if (strcmp(id, p->name) == 0 && try_module_get(p->owner)) {
			clk = p;
			break;
		}
	}
	mutex_unlock(&clocks_mutex);

	return clk;
}
EXPORT_SYMBOL(clk_get);

void clk_put(struct clk *clk)
{
	IFDBG(!clk,"%s *clk is NULL\n",__FUNCTION__);
	module_put(clk->owner);
}
EXPORT_SYMBOL(clk_put);

int clk_enable(struct clk *clk)
{
	IFDBG(!clk,"%s *clk is NULL\n",__FUNCTION__);

	if(clk->bus_clock)
		return 0;

	return 0;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
	IFDBG(!clk,"%s *clk is NULL\n",__FUNCTION__);

	if(clk->bus_clock)
		return;
}
EXPORT_SYMBOL(clk_disable);

unsigned long clk_get_rate(struct clk *clk)
{
	IFDBG(!clk,"%s *clk is NULL\n",__FUNCTION__);

	if(clk->bus_clock)
		return *clk->bus_clock;
	return *clk->bus_clock;
}
EXPORT_SYMBOL(clk_get_rate);

long clk_round_rate(struct clk *clk, unsigned long rate)
{
	IFDBG(!clk,"%s *clk is NULL\n",__FUNCTION__);
	return rate;
}
EXPORT_SYMBOL(clk_round_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	IFDBG(!clk,"%s *clk is NULL\n",__FUNCTION__);

	if(clk->bus_clock)
		return *clk->bus_clock;
	return *clk->bus_clock;
}
EXPORT_SYMBOL(clk_set_rate);

/*
 * These are fixed clocks.
 */

static struct clk uart_clk = {
	.name		= "sci_ick",
	.bus_clock	= &UART_EXTCLK,
	.enable_reg 	= 0,
	.enable_mask	= 0,
	.div_reg 	= 0,
	.div_shift 	= 0,
	.div_clk	= 0,
};

int clk_register(struct clk *clk)
{
	IFDBG(!clk,"%s *clk is NULL\n",__FUNCTION__);
	mutex_lock(&clocks_mutex);
	list_add(&clk->node, &clocks);
	mutex_unlock(&clocks_mutex);
	return 0;
}
EXPORT_SYMBOL(clk_register);

void clk_unregister(struct clk *clk)
{
	IFDBG(!clk,"%s *clk is NULL\n",__FUNCTION__);
	mutex_lock(&clocks_mutex);
	list_del(&clk->node);
	mutex_unlock(&clocks_mutex);
}
EXPORT_SYMBOL(clk_unregister);

int clk_init(void)
{
	UART_EXTCLK = (48L) * 1000 * 1000; /* 48MHz */

	clk_register(&uart_clk);
	DBG("clk_init\n");

	return 0;
}
arch_initcall(clk_init);
