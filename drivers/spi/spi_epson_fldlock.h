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

/**** CoreState ****/
#define FLD_RESERV		(0xAA)
#define FLD_WAITRET		(0xBB)
#define FLD_IDLE		(0xFF)

/**** ExeRight  ****/
#define FLD_EXEC_CORE0	(0x00)		/*A9RTOS*/
#define FLD_EXEC_CORE1	(0x01)		/*A9Linux*/
#define FLD_EXEC_CORE2	(0x02)		/*946*/
#define FLD_EXEC_NONE	(0xFF)

/**** ExeMode 	 ****/
#define FLD_EXEMODE_ON		(0xD0)
#define FLD_EXEMODE_OFF		(0x00)

/**** OFFSET 	 ****/
/* Phisidal SRAM BASE 0xE9000000 */
/* Phisical Address start 0xE9003FE8 */
#define FLD_OFFSET_CORE0STATE 0x3FE8
#define FLD_OFFSET_CORE1STATE 0x3FE9
#define FLD_OFFSET_CORE2STATE 0x3FEA
#define FLD_OFFSET_EXE_RIGHT 0x3FEB
#define FLD_OFFSET_EXE_MODE 0x3FEC

/* MAX LOOP TIME = SPI_WAIT_LOOP_MAX * SPI_WAIT_DELAY_MS */
#define SPI_WAIT_LOOP_MAX	5000
#define SPI_WAIT_DELAY_MSEC 1

/* #define FLD_LOCK_TEST_DEBUG */

volatile static int flashrom_right_ctrl_enable_lock = 0;
static const char const proc_entry_name[] = "epson_flashrom_right_ctrl_enable_lock";

volatile void __iomem *  fld_access_status_addr = (volatile void __iomem *)(IO_ADDRESS(EPSON12_SRAM_BASE) + FLD_OFFSET_CORE1STATE);
volatile void __iomem *  fld_access_right_addr = (volatile void __iomem *)(IO_ADDRESS(EPSON12_SRAM_BASE) + FLD_OFFSET_EXE_RIGHT);

static DEFINE_MUTEX(epson_fld_lock);

static int epson12_flashrom_right_release();
static int epson12_flashrom_right_request();

static int epson12_flashrom_right_request()
{
#if defined(FLD_LOCK_TEST_DEBUG)
	int wait_loop = 0;
#endif
	if(flashrom_right_ctrl_enable_lock == 2){
		return -EBUSY;
	}
	else if(flashrom_right_ctrl_enable_lock == 0){
		/* check wait rerease */
		do{
			if( readb(fld_access_status_addr) != FLD_WAITRET){
				/* FLD_IDLE or FLD_RESERV */
				break;
			}
			mdelay(SPI_WAIT_DELAY_MSEC);
#if defined(FLD_LOCK_TEST_DEBUG)
			if(wait_loop == 5000)
				printk(KERN_INFO "wait FLD_OFFSET_CORE1STATE wait 5s\n");

			wait_loop++;
#endif
		}while(1);

#if defined(FLD_LOCK_TEST_DEBUG)
		printk(KERN_INFO "+");
		wait_loop = 0;
#endif

		/* request reserv */
		writeb(FLD_RESERV, fld_access_status_addr);
		/* check ExecRight */
		do{
			if( readb(fld_access_right_addr) == FLD_EXEC_CORE1){
				break;
			}
			mdelay(SPI_WAIT_DELAY_MSEC);
#if defined(FLD_LOCK_TEST_DEBUG)
			if(wait_loop == 5000)
				printk(KERN_INFO "wait FLD_OFFSET_EXE_RIGHT wait 5s\n");

			wait_loop++;
#endif
		}while(1);
	}
	return 0;
}

static int epson12_flashrom_right_release()
{
	if(flashrom_right_ctrl_enable_lock == 0){
/* request retern */
		writeb(FLD_WAITRET, fld_access_status_addr);
#if defined(FLD_LOCK_TEST_DEBUG)
		printk(KERN_INFO "-");
#endif
	}
	return 0;
}
	

static int epson12_flashrom_right_ctrl_read(
        char *page, char **start, off_t off, int count, int *eof, void *data)
{
       
        return sprintf(page, "%d", flashrom_right_ctrl_enable_lock);
}

/* only  write at onece ,  boot seaquence (lock(driver probe function) -> release(bootup cript) */
static int epson12_flashrom_right_ctrl_write(
        struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	int input_val = -1;

        input_val = *buffer - 0x30;

#if 0
	if(input_val != 0 || (flashrom_right_ctrl_enable_lock == 0)){
		printk(KERN_NOTICE "only write at onece value=0,boot seaquence(lock(driver probe function) -> release(bootup cript)");
		return count;
	}
	
#endif
#if defined(FLD_LOCK_TEST_DEBUG)
		printk(KERN_INFO "%d", input_val);
#endif
	mutex_lock(&epson_fld_lock);
	if((input_val == 0) &&(flashrom_right_ctrl_enable_lock == 1)){
	//	epson12_flashrom_right_release();
	// proc filesystem is not syncronize
		printk("Must filesystem read for AccessRight release complete(call spi_transfer -> request release )\n");
        	flashrom_right_ctrl_enable_lock = 0;
	}
	else{
        	flashrom_right_ctrl_enable_lock = input_val;
	}
	mutex_unlock(&epson_fld_lock);

        return count;
}

