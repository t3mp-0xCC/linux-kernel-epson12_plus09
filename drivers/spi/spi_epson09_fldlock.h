/*
 * Copyright (C) 2010-2011 SEIKO EPSON CORPORATION, All rights reserved .
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

 /* 
  * FlashROM排他のための関数を定義
  * 切り替え用カーネルコンフィグレーションのdefineと共に
  * SQROMIF/SFLU3のドライバにインクルードおよび呼び出しを実装する
  * 
  */


#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>

/**** CoreState ****/
#define FLD_IDLE		(0xABCD)

/**** OFFSET 	 ****/
/* Phisidal SRAM BASE 0xE0c00000 */
/* Phisical Address start 0xE0c00408 */
#define FLD_OFFSET_CORE_STATE 0x0408

/* MAX LOOP TIME = SPI_WAIT_LOOP_MAX * SPI_WAIT_DELAY_MS */
#define SPI_WAIT_LOOP_MAX	5000
#define SPI_WAIT_DELAY_MSEC 1

/* #define FLD_LOCK_TEST_DEBUG  */

int flashrom_right_ctrl_while_lock = 0;
EXPORT_SYMBOL(flashrom_right_ctrl_while_lock);

static const char const proc_entry_name[] = "epson_flashrom_right_ctrl_while_lock";

volatile void __iomem *  fld_access_status_addr = (volatile void __iomem *)(IO_ADDRESS(EPSON09_SRAM_BASE) + FLD_OFFSET_CORE_STATE);

static DEFINE_MUTEX(epson_fld_lock);

static int epson09_flashrom_right_release();
static int epson09_flashrom_right_request();

static int epson09_flashrom_right_request()
{
#if defined(FLD_LOCK_TEST_DEBUG)
	int state = 0;
	int wait_loop = 0;
#endif
	if(flashrom_right_ctrl_while_lock == 2){
		return -EBUSY;
	}

	/* change self status */
	flashrom_right_ctrl_while_lock = 1;

	/* check and wait rerease  other core */
#if defined(FLD_LOCK_TEST_DEBUG)
	state = readl(fld_access_status_addr);
	printk(KERN_INFO "* (%d)[%x]",flashrom_right_ctrl_while_lock,state );
#endif
	do{
		if( readl(fld_access_status_addr) == FLD_IDLE){
			/* FLD_IDLE or FLD_RESERV */
			break;
		}
		//cond_resched();
		msleep(10);
		//mdelay(SPI_WAIT_DELAY_MSEC);
#if defined(FLD_LOCK_TEST_DEBUG)
		if(wait_loop == 0)
			printk(KERN_INFO "* (%d)[%x]",flashrom_right_ctrl_while_lock,state );
		if(wait_loop == 100)
			printk(KERN_INFO "*100 (%d)[%x]",flashrom_right_ctrl_while_lock,state );
		if(wait_loop == 5000)
			printk(KERN_INFO "wait FLD_OFFSET_CORE1STATE wait 5s\n");

		wait_loop++;
#endif
	}while(1);

#if defined(FLD_LOCK_TEST_DEBUG)
	state = readl(fld_access_status_addr);
	printk(KERN_INFO "+ (%d)[%x]",flashrom_right_ctrl_while_lock,state );
	wait_loop = 0;
#endif

	return 0;
}

static int epson09_flashrom_right_release()
{
	/* change self status */
	flashrom_right_ctrl_while_lock = 0;
#if defined(FLD_LOCK_TEST_DEBUG)
	int state = 0;
	state = readl(fld_access_status_addr);
	printk(KERN_INFO "- (%d)[%x]",flashrom_right_ctrl_while_lock,state );
#endif
	return 0;
}
	
static int epson09_flashrom_right_state_init()
{
	writel(FLD_IDLE, fld_access_status_addr );
}

static int epson09_flashrom_right_ctrl_read(
        char *page, char **start, off_t off, int count, int *eof, void *data)
{
       
        return sprintf(page, "%d", flashrom_right_ctrl_while_lock);
}

/* only  write at onece ,  boot seaquence (lock(driver probe function) -> release(bootup cript) */
static int epson09_flashrom_right_ctrl_write(
        struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	int input_val = -1;

        input_val = *buffer - 0x30;

	
	mutex_lock(&epson_fld_lock);
#if defined(FLD_LOCK_TEST_DEBUG)
		printk(KERN_INFO "%d", input_val);
#endif
        	flashrom_right_ctrl_while_lock = input_val;
	mutex_unlock(&epson_fld_lock);

        return count;
}

