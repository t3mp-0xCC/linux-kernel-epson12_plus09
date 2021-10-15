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
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <linux/ctype.h>

#ifdef CONFIG_MACH_EPSON12_FLD_ACCESS_CTRL
#include "spi_epson_fldlock.h"
#endif
#ifdef CONFIG_MACH_EPSON09_FLD_ACCESS_CTRL
#include "spi_epson09_fldlock.h"
#endif

#define SF_NotMount	0x0000
#define SF_S25FL001D	0x0101
#define SF_AT25F512A	0x0201
#define SF_SST25VF512	0x0301

#define	ASIC_SFLU_BASE		(0x26000000)

#define G_SFLU_STATE_REGS       (0x000)
#define G_SFLU_CTRL_REGS1       (0x002)
#define G_SFLU_CTRL_REGS2       (0x004)
#define G_SFLU_CTRL_REGS3       (0x006)
#define G_SFLU_TRIGER_REGS1     (0x008)
#define G_SFLU_TRIGER_REGS2     (0x00A)
#define G_SFLU_PORT_CTRL        (0x010)

#define	G_SFLU_BUSY_EXT		(0x020)

#define G_SFLU_SDR_WSIZE1       (0x030)
#define G_SFLU_SDR_WSIZE2       (0x032)
#define G_SFLU_SDR_ADDR1        (0x034)
#define G_SFLU_SDR_ADDR2        (0x036)

#define G_SFLU_SI_BUFF          (0x050)
#define G_SFLU_SI_BUFF_L        (0x050)
#define G_SFLU_SO_BUFF1         (0x060)
#define G_SFLU_SO_BUFF2         (0x062)
#define G_SFLU_SO_BUFF3         (0x064)
#define G_SFLU_SO_BUFF4         (0x066)

#define G_SFLU3_CTRL_REGS       (0x102)

u16 gusSF_DevID = SF_NotMount ;   /* Device-ID */

//#define SFLU3_DEBUG	1

typedef u8	UINT8;
typedef u16	UINT16;
typedef u32	UINT32;
typedef int 	BOOL;

#define TRUE	(1)
#define FALSE	(0)

struct epson_sflu3_data {
	unsigned long syscu_base;
	unsigned long tsysu_base;
	unsigned long sflu_base;
	int			  alloc_success;	//all i/o area remap succeed?
	struct	spi_master * master;
	spinlock_t	lock;
	u16	ctrl_reg;	//ctrl reg value to recover when cleanup sflu3
};

int sflu3_write(struct epson_sflu3_data * sflu3,
	const void * write_buf, 
	unsigned int len,
	dma_addr_t flash_buf
);

/* SFLU system functions */
void   SFLU_Busy_Wait(const struct epson_sflu3_data * sflu3);
int sflu3_write(struct epson_sflu3_data * sflu3,
	const void * write_buf, 
	unsigned int len,
	dma_addr_t flash_buf
);
int sflu3_read(struct epson_sflu3_data * sflu3,
	void * read_buf,
	unsigned int len,
	dma_addr_t flash_buf
);

u16 readw_sflu3(const struct epson_sflu3_data * sflu3, int reg)
{
	volatile u16 * addr = (volatile u16 *)(sflu3->sflu_base + reg);
	u16 value;
	value = *addr;

	return value;
}

u16 writew_sflu3(const struct epson_sflu3_data * sflu3, int reg, u16 value)
{
	volatile u16 * addr = (volatile u16 *)(sflu3->sflu_base + reg);
	*addr = value;

	return value;
}

u16 orw_sflu3(const struct epson_sflu3_data * sflu3, int reg, u16 value)
{
	volatile u16 * addr = (volatile u16 *)(sflu3->sflu_base + reg);
	*addr |= value;

	return value;
}


u8 readb_sflu3(const struct epson_sflu3_data * sflu3, int reg)
{
	volatile u8 * addr = (volatile u8 *)(sflu3->sflu_base + reg);
	u16 value;
	value = *addr;

	return value;
}

u8 writeb_sflu3(const struct epson_sflu3_data * sflu3, int reg, u8 value)
{
	volatile u8 * addr = (volatile u8 *)(sflu3->sflu_base + reg);
	*addr = value;

	return value;
}

void   SFLU_Busy_Wait(const struct epson_sflu3_data * sflu3)
{
	u16 status;
	/* wait for busy flag == 0 */
	do {
		status = readw_sflu3(sflu3, G_SFLU_STATE_REGS);
	} while ((status & 0x0001) != 0);
}

void HAL_SF_Init(struct epson_sflu3_data * sflu3,  u16 usDevID )
{
#if !defined(CONFIG_EPSON09_WITH_RTOS) && !defined(CONFIG_EPSON12_WITH_RTOS)
	printk(KERN_ERR "called %s\n", __FUNCTION__);
	sflu3->ctrl_reg = readw_sflu3(sflu3, G_SFLU_PORT_CTRL);	/* get ctrl_reg value to recover */

	gusSF_DevID = usDevID;         /* Set Device-ID */

	/* move these codes to main\system_io\sf_hal\sfhal.o */
	/* writew_sflu3(sflu3, G_SFLU3_CTRL_REGS, 0x0001);	Normal(SFLU1) Access Mode */

	SFLU_Busy_Wait(sflu3);

	writew_sflu3(sflu3, G_SFLU_PORT_CTRL, 0x000F);	/* Non Port Mode */
	writew_sflu3(sflu3, G_SFLU_CTRL_REGS1, 0x0001);	/* SCK = 24MHz,  XINT = Disable. */
	writew_sflu3(sflu3, G_SFLU_CTRL_REGS2, 0x0101);	/* XCE = H */
	writew_sflu3(sflu3, G_SFLU_CTRL_REGS3, 0x0101);	/* XCE = H */
	writew_sflu3(sflu3, G_SFLU_BUSY_EXT, 0x0000);	/* BUSY_EXT = 0 cycle */
#endif
}

void SFLU_SPC_low(struct spi_device * spi)
{
	struct spi_master * master = spi->master;
	struct epson_sflu3_data * sflu3 = spi_master_get_devdata(master);

	u8 select = spi->chip_select;
	int reg = G_SFLU_CTRL_REGS2;
	u16 dat = 0x0100;
	switch (select) {
	case 0:
		/* use default */
		break;
	case 1:
		reg = G_SFLU_CTRL_REGS2; dat = 0x0001;
		break;
	case 2:
		reg = G_SFLU_CTRL_REGS3; dat = 0x0100;
		break;
	case 3:
		reg = G_SFLU_CTRL_REGS3; dat = 0x0001;
		break;
	default:
		return;
	}
	writew_sflu3(sflu3, reg, dat);
}

void SFLU_SPC_hi(struct spi_device * spi)
{
	struct spi_master * master = spi->master;
	struct epson_sflu3_data * sflu3 = spi_master_get_devdata(master);

	u8 select = spi->chip_select;
	switch (select) {
	case 0:
	case 1:
		writew_sflu3(sflu3, G_SFLU_CTRL_REGS2, 0x0101);
		writew_sflu3(sflu3, G_SFLU_CTRL_REGS2, 0x0101);
		break;
	case 2:
	case 3:
		writew_sflu3(sflu3, G_SFLU_CTRL_REGS3, 0x0101);
		writew_sflu3(sflu3, G_SFLU_CTRL_REGS3, 0x0101);
		break;
	default:
		break;
	}
}

void HAL_SF_End(const struct epson_sflu3_data * sflu3)
{
#if 0
	SFLU_Busy_Wait(sflu3);
	writew_sflu3(sflu3, G_SFLU_PORT_CTRL, sflu3->ctrl_reg);	/* Non Port Mode */
#endif
}

u8 epson_sflu3_read_status_register(struct spi_device * spi)
{
	struct spi_master * master = spi->master;
	struct epson_sflu3_data * sflu3 = spi_master_get_devdata(master);
	char read_status[] = { 0x05,  };
	char result = 0;
	SFLU_SPC_low(spi);    /* XCE = L, msg we want send command to SF */
	sflu3_write(sflu3, read_status, sizeof(read_status), 0);
	SFLU_Busy_Wait(sflu3);
	
	sflu3_read(sflu3, &result, 1, 0);
	SFLU_Busy_Wait(sflu3);
	
	SFLU_SPC_hi(spi);
	
	return result;
}

void epson_sf_busywait(struct spi_device * spi)
{
	u8 status = 0;
	
	do {
		status = epson_sflu3_read_status_register(spi);
	} while ( status & 1 );
}

void epson_sflu3_writeenable(struct spi_device * spi)
{
	struct spi_master * master = spi->master;
	struct epson_sflu3_data * sflu3 = spi_master_get_devdata(master);
	char write_enable[] = { 0x06 };

	SFLU_SPC_low(spi);    /* XCE = L, msg we want send command to SF */
	sflu3_write(sflu3, write_enable, sizeof(write_enable), 0);
	SFLU_Busy_Wait(sflu3);
	SFLU_SPC_hi(spi);
}

void epson_sflu3_protectoff(struct spi_device * spi)
{
/* Serial Flash ROM mode always protect off */
#if 0
	char write_buf[] = { 0x01, 0x00 };  /* Numonix */
	struct spi_master * master = spi->master;
	struct epson_sflu3_data * sflu3 = spi_master_get_devdata(master);

	epson_sflu3_writeenable(spi);
	
	SFLU_SPC_low(spi);    /* XCE = L, msg we want send command to SF */
	sflu3_write(sflu3, write_buf, sizeof(write_buf), 0);
	SFLU_Busy_Wait(sflu3);
	SFLU_SPC_hi(spi);
	
	epson_sf_busywait(spi);
#endif
}

void epson_sflu3_protecton(struct spi_device * spi)
{
/* Serial Flash ROM mode always protect off */
#if 0
	char write_buf[] = { 0x01, 0x1C, 0x00 }; /* numonix */
	struct spi_master * master = spi->master;
	struct epson_sflu3_data * sflu3 = spi_master_get_devdata(master);

	epson_sflu3_writeenable(spi);
	
	SFLU_SPC_low(spi);
	sflu3_write(sflu3, write_buf, sizeof(write_buf), 0);
	SFLU_Busy_Wait(sflu3);
	SFLU_SPC_hi(spi);
	
	epson_sf_busywait(spi);
#endif
}

int HAL_SF_NormalMode(struct spi_device * spi, int protect_off)
{
	int ret = 0;
	struct spi_master * master = spi->master;
	struct epson_sflu3_data * sflu3 = spi_master_get_devdata(master);

#ifdef CONFIG_MACH_EPSON12_FLD_ACCESS_CTRL
	mutex_lock(&epson_fld_lock);
	epson12_flashrom_right_request();
#endif
#ifdef CONFIG_MACH_EPSON09_FLD_ACCESS_CTRL
	mutex_lock(&epson_fld_lock);
	ret = epson09_flashrom_right_request();
	if (ret != 0)
	{
		return ret;
	}
#endif

	writew_sflu3(sflu3, G_SFLU3_CTRL_REGS, 0x0001);	/* Normal Access Mode */
	SFLU_Busy_Wait(sflu3);
	if (protect_off) {
		epson_sflu3_protectoff(spi);
	}
	return ret;
}

int HAL_SF_DirectMode(struct spi_device * spi, int protect_on)
{
	int ret = 0;
	struct spi_master * master = spi->master;
	struct epson_sflu3_data * sflu3 = spi_master_get_devdata(master);

	if (protect_on) {
		epson_sflu3_protecton(spi);
	}
	writew_sflu3(sflu3, G_SFLU3_CTRL_REGS, 0x0000);	/* Direct Mode */
	SFLU_Busy_Wait(sflu3);
#ifdef CONFIG_MACH_EPSON12_FLD_ACCESS_CTRL
	epson12_flashrom_right_release();
	mutex_unlock(&epson_fld_lock);
#endif
#ifdef CONFIG_MACH_EPSON09_FLD_ACCESS_CTRL
	ret = epson09_flashrom_right_release();
	mutex_unlock(&epson_fld_lock);
	if (ret != 0)
	{
		return ret;
	}
#endif
	return 0;
}


#if defined(CONFIG_SPI_EPSON_SFLU3_DIRECTMODE)

int HAL_SF_Lock_forDirectMode(struct spi_device * spi, int protect_on){
	int ret = 0;
#ifdef CONFIG_MACH_EPSON12_FLD_ACCESS_CTRL
	mutex_lock(&epson_fld_lock);
	epson12_flashrom_right_request();
#endif
#ifdef CONFIG_MACH_EPSON09_FLD_ACCESS_CTRL
	mutex_lock(&epson_fld_lock);
	ret = epson09_flashrom_right_request();
	if (ret != 0)
	{
		return ret;
	}
#endif
	return 0;
}

int HAL_SF_Unlock_forDirectMode(struct spi_device * spi, int protect_on){
#ifdef CONFIG_MACH_EPSON12_FLD_ACCESS_CTRL
	epson12_flashrom_right_release();
	mutex_unlock(&epson_fld_lock);
#endif
#ifdef CONFIG_MACH_EPSON09_FLD_ACCESS_CTRL
	int ret = 0;
	ret = epson09_flashrom_right_release();
	mutex_unlock(&epson_fld_lock);
	if (ret != 0)
	{
		return ret;
	}
#endif
	return 0;
}

#endif

int epson_sflu3_setup(struct spi_device * spi)
{
	struct spi_master * master = spi->master;
	struct epson_sflu3_data * sflu3 = spi_master_get_devdata(master);

	/* must uncomment this when IPL set correct h/w depend value */
	HAL_SF_Init(sflu3, SF_S25FL001D);
	/* nothing to do in this function 'cause we do sflu3 initialize at m25p80 */
#if defined(SFLU3_DEBUG)
	 printk(KERN_ERR "%s called \n", __FUNCTION__);
#endif

	return 0;
}

#if defined(SFLU3_DEBUG)
char myline[256];
char mychars[256];
char myform[256];

void sflu3_dump(const void * buf, unsigned int len)
{
	const char * p = buf;
	const char * end = p + len;
	int j;
	int pos;
	char a_char;
	do {
		pos = 0;
		for (j = 0; j < 16; ++j) {
			if (p + j >= end) {
				pos += sprintf(myline + pos, "-- ");
				sprintf(mychars + j, " ");
			} else {
				pos += sprintf(myline + pos, "%02x ", p[j]);
				a_char = (p[j] > 0x20 && p[j] < 0x7c) ? p[j] : '.';
				sprintf(mychars + j, "%c", a_char);
			}	
		}
		sprintf(myform, "%p : %s : %s\n\n", p, myline, mychars);
		printk(KERN_ERR "%s", myform);
		if (p + j >= end) {
			break;
		}
		p += 16;
	} while (1);
	return;
}
#else
inline void sflu3_dump(const void * buf, unsigned int len) {}
#endif

unsigned short mvalue;

int sflu3_write(struct epson_sflu3_data * sflu3,
	const void * write_buf, 
	unsigned int len,
	dma_addr_t flash_buf
)
{
	int i;
	const unsigned char * p = write_buf;
//printk(KERN_ERR "in %s write_buf = %p, len = %d, flash = %08x\n", __FUNCTION__, write_buf, len, flash_buf);	
if (len < 100) {
	sflu3_dump(p, len);
}
	for (i = 0; i < len; ) {
		if (i + 1 < len) {
			mvalue = ( p[i] & 0xff ) | (( p[i + 1] & 0xff ) << 8);
			i += 2;
			writew_sflu3(sflu3, G_SFLU_SI_BUFF, mvalue);
		} else {
			writeb_sflu3(sflu3, G_SFLU_SI_BUFF_L, p[i]);
			++i;
		}
		SFLU_Busy_Wait(sflu3);
	}

	return len;
}

int sflu3_read(struct epson_sflu3_data * sflu3,
	void * read_buf,
	unsigned int len,
	dma_addr_t flash_buf
)
{
	int i;
	char * p = read_buf;
//	printk(KERN_ERR "in %s read_buf = %p, len = %d, flash = %08x\n", __FUNCTION__, read_buf, len, flash_buf);	

	for (i = 0; i < len; ++i) {
		writew_sflu3(sflu3, G_SFLU_TRIGER_REGS1, 0x0001);     /* Read Triger ( 1Byte ) */
		SFLU_Busy_Wait(sflu3);
		p[i] = readw_sflu3(sflu3, G_SFLU_SO_BUFF1);
	}
if (len < 100) {
	sflu3_dump(read_buf, len);
}
	return len;
}

unsigned int sflu3_handle_1transfer(
	struct spi_device * spi, 
	struct spi_transfer * transfer
)
{
	struct spi_master * master = spi->master;
	struct epson_sflu3_data * sflu3 = spi_master_get_devdata(master);

	struct spi_transfer * last_transfer = transfer;
	unsigned int len = 0;
	if (last_transfer->tx_buf != NULL) {
		len = sflu3_write(sflu3, last_transfer->tx_buf, last_transfer->len, last_transfer->tx_dma);
	}
	if (last_transfer->rx_buf != NULL) {
		len = sflu3_read(sflu3, last_transfer->rx_buf, last_transfer->len, last_transfer->rx_dma);
	}
	return len;
}

int epson_sflu3_transfer(struct spi_device * spi, struct spi_message * mesg)
{
	struct spi_transfer * last_transfer; 
	unsigned int len = 0;

	SFLU_SPC_low(spi);

	list_for_each_entry(last_transfer, &(mesg->transfers), transfer_list) {
		len += sflu3_handle_1transfer(spi, last_transfer);
	}

	SFLU_SPC_hi(spi);
	mesg->complete(mesg->context);
	mesg->status = 0;
	mesg->actual_length = len;
#if defined(SFLU3_DEBUG)
	printk(KERN_ERR "message complate\n");
#endif
	return 0;
}

#if defined(CONFIG_MACH_EPSON12_FLD_ACCESS_CTRL) || defined(CONFIG_MACH_EPSON09_FLD_ACCESS_CTRL)
int epson_sflu3_transfer_with_access_control(struct spi_device * spi, struct spi_message * mesg)
{
	struct spi_transfer * last_transfer; 
	unsigned int len = 0;


	SFLU_SPC_low(spi);

	list_for_each_entry(last_transfer, &(mesg->transfers), transfer_list) {
		len += sflu3_handle_1transfer(spi, last_transfer);
	}

	SFLU_SPC_hi(spi);


	mesg->complete(mesg->context);
	mesg->status = 0;
	mesg->actual_length = len;
#if defined(SFLU3_DEBUG)
	printk(KERN_ERR "message complate\n");
#endif
	return 0;
}
#endif

static void epson_sflu3_cleanup(const struct spi_device * spi)
{
	struct spi_master * master = spi->master;
	struct epson_sflu3_data * sflu3 = spi_master_get_devdata(master);
#if defined(SFLU3_DEBUG)
	 printk(KERN_ERR "%s called \n", __FUNCTION__);
#endif
	HAL_SF_End(sflu3);

}

/* guarantee that all i/o area allocated or no i/o area allocated */
int sflu3_alloc_memory(struct spi_master * master, resource_size_t start)
{
	struct epson_sflu3_data * sflu3_data = spi_master_get_devdata(master);

	sflu3_data->sflu_base = (unsigned long)ioremap(start, SZ_8K);
	if (sflu3_data->sflu_base == 0) {
		goto err;
	}
	sflu3_data->alloc_success = 1;
	return 0;

err:
	sflu3_data->alloc_success = 0;

	return ENOMEM;
}

int sflu3_release_memory(struct spi_master * master)
{
	struct epson_sflu3_data * sflu3_data = spi_master_get_devdata(master);
	if (!sflu3_data->alloc_success) {
		return 0;	// nothing to do in this case
	}
	iounmap((void *)sflu3_data->sflu_base);

	return 0;
}

int __init epson_sflu3_probe(struct platform_device * dev)
{
	struct spi_master *master;
	struct epson_sflu3_data * sflu3_data;
	int ret = 0;
	struct proc_dir_entry *entry;

	/* Get resources(memory, IRQ) associated with the device */
	master = spi_alloc_master(&dev->dev, sizeof(struct epson_sflu3_data));
	if (master == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	platform_set_drvdata(dev, master);

	ret = sflu3_alloc_memory(master, dev->resource->start);
	if (ret != 0) {
		goto free_master;
	}
	
	/* SPI controller can be understood this mode */
        master->mode_bits = SPI_MODE_3;

	/* SPI controller initializations */
	sflu3_data = spi_master_get_devdata(master);
	sflu3_data->master = master;

printk(KERN_ERR "called %s, master = %p, sflu3_data = %p\n", __FUNCTION__, master, sflu3_data);
	
	spin_lock_init(&sflu3_data->lock);

	master->bus_num = 0;
	master->num_chipselect = 3;
	master->cleanup = epson_sflu3_cleanup;
	master->setup   = epson_sflu3_setup;

#ifdef CONFIG_MACH_EPSON12_FLD_ACCESS_CTRL
	if( readb((char *)(IO_ADDRESS(EPSON12_SRAM_BASE) + FLD_OFFSET_EXE_MODE)) == FLD_EXEMODE_ON){
		master->transfer = epson_sflu3_transfer_with_access_control;
		printk(KERN_NOTICE "sflu3 : master->transfer is transfer with accesscontrol \n");
		printk(KERN_NOTICE "sflu3 : MUST WRITE dicimal 0 to %s, when system boot finish",proc_entry_name);
		entry = create_proc_entry(proc_entry_name, S_IFREG | S_IRUGO | S_IWUGO, NULL);
		if (entry != NULL) {
			entry->read_proc = epson12_flashrom_right_ctrl_read;
			entry->write_proc = epson12_flashrom_right_ctrl_write;
			/* while systembootup, Linux core handle flashrom access right */
			epson12_flashrom_right_request();
			flashrom_right_ctrl_enable_lock = 1;
		}

	}else{
		master->transfer = epson_sflu3_transfer;
		printk(KERN_NOTICE "sflu3 : master->transfer is normal transfer \n");
	}
#else 
#ifdef CONFIG_MACH_EPSON09_FLD_ACCESS_CTRL
		master->transfer = epson_sflu3_transfer_with_access_control;
		printk(KERN_NOTICE "sflu3 : master->transfer is transfer with accesscontrol \n");
		printk(KERN_NOTICE "sflu3 : MUST WRITE dicimal 1 to %s, when system terminate",proc_entry_name);

		entry = create_proc_entry(proc_entry_name, S_IFREG | S_IRUGO | S_IWUGO, NULL);
		if (entry != NULL) {
			entry->read_proc = epson09_flashrom_right_ctrl_read;
			entry->write_proc = epson09_flashrom_right_ctrl_write;
		}
		flashrom_right_ctrl_while_lock = 0;

#else
	master->transfer = epson_sflu3_transfer;
#endif
#endif

	
	ret = spi_register_master(master);
	if (ret != 0) {
		printk(KERN_INFO "sflu3 : can not register as spi device\n");
		goto free_master;
	}
	
	return ret;
free_master:
	kfree(master);
err:
	return ret;
} 

int __devexit epson_sflu3_remove(struct platform_device *dev)
{
	struct spi_master * master = platform_get_drvdata(dev);

	sflu3_release_memory(master);
	kfree(master);
	
	return 0;
}

struct platform_driver epson_sflu3_driver = {
	.probe = epson_sflu3_probe,
	.remove = epson_sflu3_remove,
	.driver = {
		.name = "sflu3",
		.owner = THIS_MODULE,
	},
};

int /*__init*/ epson_sflu3_init(void)
{
printk(KERN_ERR "called %s\n", __FUNCTION__);
	epson09_flashrom_right_state_init();
	return platform_driver_register(&epson_sflu3_driver);
}

void __exit epson_sflu3_exit(void)
{
	platform_driver_unregister(&epson_sflu3_driver);
}

module_init(epson_sflu3_init);
module_exit(epson_sflu3_exit);

MODULE_AUTHOR("SEIKO EPSON CORPORATION");
MODULE_DESCRIPTION("EPSON SPI Driver");
MODULE_LICENSE("GPL");
