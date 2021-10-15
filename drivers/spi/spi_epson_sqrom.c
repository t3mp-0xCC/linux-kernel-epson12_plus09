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

 /* ���»���
  * 256Mbit�ʾ��ROM�ؤ�32bit���ɥ쥹���������ˤ�̤�б���128Mbit�ʤޤǡ�
  * �����쥯�ȥ⡼�ɤǤ��ɤ߹��ߤ�̤�б���SPI�ե졼������ؤ��б���ɬ��)
  * ASIC�ν������RTOS¦�ǹԤ�����Ȥ������������̤����
  * Write�ˤĤ��ơ�ü���Х��Ȥν����ˤ��¥����˾����԰��פ�������Τ�
  * �饤�Ȼ��˥Хåե��ؤ�4�Х���ñ�̤ǤĤ�ơ�����ȥ��顼�ˤ�4�Х���ñ�̤����ꤹ��
  * JFFS2��4�Х��ȶ����Υ��ɥ쥹�ǥ����������Ƥ���Τ�����ʤ�
  * ü���Х��Ȥ�0xFF�����Ƥ�������
  * 
  * 
  */
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>

#ifdef CONFIG_MACH_EPSON12_FLD_ACCESS_CTRL
#include "spi_epson_fldlock.h"
#endif

#define SF_NotMount	0x0000
#define SF_S25FL001D	0x0101
#define SF_AT25F512A	0x0201
#define SF_SST25VF512	0x0301

#define SF_W25Q128BV	0xef4018


u16 gusSF_DevID = SF_NotMount ;   /* Device-ID */


#define	ASIC_SQROMIF_BASE		(EPSON12_SQROM_BASE)

/*#define CS0_MODE_SET	(0x004)*/
#define CS0_CMD_SET		(0x00C)
#define CS0_CLK_SET		(0x014)
#define CS0_SSET		(0x01C)
#define CS1_MODE_SET	(0x024)
#define CS1_CMD_SET		(0x02C)
#define CS1_CLK_SET		(0x034)
/*#define CS2_MODE_SET	(0x03C)*/
#define CS2_CMD_SET		(0x044)
#define CS2_CLK_SET		(0x04C)
/*#define CS3_MODE_SET	(0x054)*/
#define CS3_CMD_SET		(0x05C)
#define CS3_CLK_SET		(0x064)

#define CSX_AHB_BST		(0x06C)

#define SFSW_CMD_SET	(0x074)
#define SFSW_MODE_SET	(0x07C)
#define SFSW_ADR		(0x084)
#define SFSW_DW_SET		(0x08C)
#define SFSW_WDATA		(0x094)
#define SFSW_WDATA_PTR	(0x09C)
#define SFSW_RDATA0		(0x0A4)
#define SFSW_RDATA1		(0x0AC)
#define SFSW_RDATA2		(0x0B4)
#define SFSW_RDATA3		(0x0BC)
#define SFSW_CLK_SET	(0x0C4)
#define SFSW_CS_SET		(0x0CC)
#define SFSW_START		(0x0D4)
#define SFSW_CYC_STS	(0x0DC)
#define SFSW_INT		(0x0E4)
#define SFSW_INT_MASK	(0x0EC)
#define SFSW_INT_CLR	(0x0F4)
/*#define SFSW_CSD		(0x0FC)*/

#define CS_DS_TIME		(0x104)
#define DDP_ROM			(0x10C)
#define SPDSCL			(0x204)
#define SPDSDA_IN		(0x20C)
#define SPDSDA_OUT		(0x214)
#define ROM_SETPIN		(0x21C)

/* chip mode */
#define SYSTEM_CHIP_MODE        (0x2)

/* 2chip mode ��ư��Ƥ���Τǡ�Linux��CS0�ˤ���褦�˸����� */
#define LINUX_CS_CS0 

#ifdef LINUX_CS_CS0
	/* CS_SSET ��������� */
#define CS_LINUX_PLACE  (0x00000000)
#else
	/* CS_SSET ��������� */
#define CS_LINUX_PLACE  (0x00000001)
#endif

#define SFSW_CLK_64MHZ	(0x00010302)
#define SFSW_CLK_32MHZ	(0x00001001)
#define SFSW_CLK_16MHZ	(0x00000000)
#define SFSW_CLK_SPEED	SFSW_CLK_64MHZ
#define CSX_CLK_SPEED	SFSW_CLK_64MHZ

/* Flash opcodes. */
#define OPCODE_WRDIS             0x04    /* Write disable */
#define OPCODE_WREN             0x06    /* Write enable */
#define OPCODE_RDSR             0x05    /* Read status register */
#define OPCODE_WRSR             0x01    /* Write status register 1 byte */
#define OPCODE_NORM_READ        0x03    /* Read data bytes (low frequency) */
#define OPCODE_FAST_READ        0x0b    /* Read data bytes (high frequency) */
#define OPCODE_QUAD_READ        0xeb    /* Read data bytes (Quad i/o read) */
#define OPCODE_PP               0x02    /* Page program (up to 256 bytes) */
#define OPCODE_BE_4K            0x20    /* Erase 4KiB block */
#define OPCODE_BE_32K           0x52    /* Erase 32KiB block */
#define OPCODE_CHIP_ERASE       0xc7    /* Erase whole flash chip */
#define OPCODE_SE               0xd8    /* Sector erase (usually 64KiB) */
#define OPCODE_RDID             0x9f    /* Read JEDEC ID */
/* Status Register bits. */
#define SR_WIP                  1       /* Write in progress */
#define SR_WEL                  2       /* Write enable latch */
/* meaning of other SR_* bits may differ between vendors */
#define SR_BP0                  4       /* Block protect 0 */
#define SR_BP1                  8       /* Block protect 1 */
#define SR_BP2                  0x10    /* Block protect 2 */
#define SR_SRWD                 0x80    /* SR write protect */


#define  MODE_NOT_SET	0 
#define  NORMAL_MODE	1 
#define  DIRECT_MODE	2

#define  READ_SINGLE	1
#define  READ_DOUBLE	2

#define  READ_DIRECT	1
#define  READ_NORMAL	2

#define  WRITE_DATA	1
#define  WRITE_COMMAND	2

typedef u32	UINT32;
typedef int 	BOOL;

#define TRUE	(1)
#define FALSE	(0)

/* use kernel boot param cm(chip mode 1 or 2), cs(chip select 0 or 1) */
#define CMDPARAM_NON_INITIAL_VALUE (3)
static u32 sqrom_chip_select = CMDPARAM_NON_INITIAL_VALUE;
u32 sqrom_chip_mode = CMDPARAM_NON_INITIAL_VALUE;

/*
#define SQROM_DEBUG	1
*/


struct epson_sqrom_data {
	unsigned long sqrom_base;
	int			  alloc_success;	//all i/o area remap succeed?
	int			  chip_mode;	/* 1chpe = 1, 2chip = 2, not set =0  */
	int			  sqrom_mode;	/* normal or direct  */
	unsigned long		chip_base_addr;	/* ioremap chip base addr  */
	int			  chip_read;	/* single or double  */
	int			  read_len;	/* read command read_len  */
	unsigned long		  read_addr;	/* read command addr  */
	int 			  read_mode;	/* read command direct flag  */
	unsigned long		  write_addr;	/* read command addr  */
	int			  write_mode;	/* Write access command or data */
	struct	spi_master * master;
	spinlock_t	lock;
	u16	ctrl_reg;	//ctrl reg value to recover when cleanup sqrom
};


/* SQROM system functions */
/* function proto wait IP busy */
void   SQROM_Busy_Wait(const struct epson_sqrom_data * sqrom);

/* function proto read write data */
int sqrom_write(struct epson_sqrom_data * sqrom,
	const void * write_buf, 
	unsigned int len,
	dma_addr_t flash_buf
);
int sqrom_read(struct epson_sqrom_data * sqrom,
	void * read_buf,
	unsigned int len,
	dma_addr_t flash_buf
);

void sqrom_dump(const void * buf, unsigned int len);

/* read write sqrom IP register */
u32 readl_sqrom(const struct epson_sqrom_data * sqrom, int reg)
{
	volatile u32 * addr = (volatile u32 *)(sqrom->sqrom_base + reg);
	u32 value;
	value = *addr;

	return value;
}

u32 writel_sqrom(const struct epson_sqrom_data * sqrom, int reg, u32 value)
{
	volatile u32 * addr = (volatile u32 *)(sqrom->sqrom_base + reg);
	*addr = value;

	return value;
}

u32 orl_sqrom(const struct epson_sqrom_data * sqrom, int reg, u32 value)
{
	volatile u32 * addr = (volatile u32 *)(sqrom->sqrom_base + reg);
	*addr |= value;

	return value;
}



/* �������뽪λ�Ԥ� */
void   SQROM_Busy_Wait(const struct epson_sqrom_data * sqrom)
{
	u32 status;
#if defined(SQROM_DEBUG)
/*
	 printk(KERN_INFO "%s called \n", __FUNCTION__);
*/
#endif
	/* wait for busy flag == 0 */
	do {
		status = readl_sqrom(sqrom, SFSW_CYC_STS);
	} while ((status & 0x00000001) != 0);
	udelay(100);
}


/* ���٥��������� */
void HAL_SF_Init(struct epson_sqrom_data * sqrom,  u16 usDevID )
{
#if 0
/* �������RTOS¦�Υ����Ǥ�� */
#endif
}

/* ���٥뽪λ���� */
void HAL_SF_End(const struct epson_sqrom_data * sqrom)
{
#if 0
/* �����֥�⥸�塼��ˤ��ʤ��Τ�����̤���� */
#endif
}

/* Flash chip �Υ��ơ������쥸�������ɤ� */
u32 epson_sqrom_read_status_register(struct epson_sqrom_data * sqrom)
{
	unsigned long mvalue;
	u32 result =0;
#if defined(SQROM_DEBUG)
	 printk(KERN_INFO "%s called \n", __FUNCTION__);
#endif
	//�꡼�ɥ��ơ������쥸�������ޥ��(05)ȯ��
	writel_sqrom(sqrom, SFSW_CMD_SET, 0x00000005) ;
	if (sqrom->chip_mode == 2) {
		writel_sqrom(sqrom, SFSW_MODE_SET, 0x00010000);
		writel_sqrom(sqrom, SFSW_DW_SET, 0x11001001);
	}
	else if (sqrom->chip_mode == 1) {
		writel_sqrom(sqrom, SFSW_MODE_SET, 0x00000000);
		writel_sqrom(sqrom, SFSW_DW_SET, 0x11001000);
	}
	writel_sqrom(sqrom, SFSW_START, 0x00000001);
	
	/* �������뽪λ���Ԥ� */
	SQROM_Busy_Wait(sqrom);
	
	/* ���ơ������쥸�������ͤ���� */
	if (sqrom->chip_mode == 2) {
		mvalue = 0;
		mvalue = readl_sqrom(sqrom, SFSW_RDATA0) ;
		result = ( mvalue & 0x000000ff ) | ((mvalue & 0x00ff0000 ) >> 8) | ((mvalue & 0x0000ff00 ) << 8) |( mvalue & 0xff000000  ) ;
	}
	else if (sqrom->chip_mode == 1) {
		result = readl_sqrom(sqrom, SFSW_RDATA0);
	}
	/* �������뽪λ���Ԥ� */
	SQROM_Busy_Wait(sqrom);
	
	return result;
}

/* BUSY=0��ǧ */
void Sqrom_BUSY0_Wait(const struct epson_sqrom_data * sqrom) {

#if defined(SQROM_DEBUG)
	 printk(KERN_INFO "%s called \n", __FUNCTION__);
#endif
	//�꡼�ɥ��ơ������쥸�������ޥ��(05)ȯ��
	writel_sqrom(sqrom, SFSW_CMD_SET, 0x00000005) ;
	if (sqrom->chip_mode == 2) {
		writel_sqrom(sqrom, SFSW_MODE_SET, 0x00010000);
		writel_sqrom(sqrom, SFSW_DW_SET, 0x11001001);
	}
	else if (sqrom->chip_mode == 1) {
		writel_sqrom(sqrom, SFSW_MODE_SET, 0x00000000);
		writel_sqrom(sqrom, SFSW_DW_SET, 0x11001000);
	}
	writel_sqrom(sqrom, SFSW_START, 0x00000001);
	
	/* �������뽪λ���Ԥ� */
	SQROM_Busy_Wait(sqrom);

	//���ơ������쥸����BUSY=0��ǧ
	if (sqrom->chip_mode == 2) {
		while ((readl_sqrom(sqrom, SFSW_RDATA0) & 0x00010001) != 0x00000000) {
			writel_sqrom(sqrom, SFSW_START, 0x00000001);
			/* �������뽪λ���Ԥ� */
			SQROM_Busy_Wait(sqrom);
		}
	}
	else if (sqrom->chip_mode == 1) {
		while ((readl_sqrom(sqrom, SFSW_RDATA0) & 0x00000001) != 0x00000000) {
			writel_sqrom(sqrom, SFSW_START, 0x00000001);
			/* �������뽪λ���Ԥ� */
			SQROM_Busy_Wait(sqrom);
		}
	}
}


void Sqrom_Write_Enable(const struct epson_sqrom_data * sqrom) {
#if defined(SQROM_DEBUG2)
	 printk(KERN_INFO "%s called \n", __FUNCTION__);
#endif
	/* write Enable(06)ȯ�� */
	writel_sqrom(sqrom, SFSW_CMD_SET, 0x00000006) ;
	if (sqrom->chip_mode == 2) {
		writel_sqrom(sqrom, SFSW_MODE_SET, 0x00010000);
	}
	else if (sqrom->chip_mode == 1) {
		writel_sqrom(sqrom, SFSW_MODE_SET, 0x00000000);
	}
	writel_sqrom(sqrom, SFSW_DW_SET, 0x01000000);
	writel_sqrom(sqrom, SFSW_START, 0x00000001);
	
	/* �������뽪λ���Ԥ� */
	SQROM_Busy_Wait(sqrom);

	/* ��ǧ���ɤ߽Ф� */
	/* ���ޥ��(05)ȯ�� (statsu register read) */
	writel_sqrom(sqrom, SFSW_CMD_SET, 0x00000005) ;
	if (sqrom->chip_mode == 2) {
		writel_sqrom(sqrom, SFSW_MODE_SET, 0x00010000);
		writel_sqrom(sqrom, SFSW_DW_SET, 0x11001001);
	}
	else if (sqrom->chip_mode == 1) {
		writel_sqrom(sqrom, SFSW_MODE_SET, 0x00000000);
		writel_sqrom(sqrom, SFSW_DW_SET, 0x11001000);
	}
	writel_sqrom(sqrom, SFSW_START, 0x00000001);

	/* WE �ˤʤ�ޤǷ����֤�ȯ�Ԥ��Ƴ�ǧ���� */
	while((readl_sqrom(sqrom, SFSW_RDATA0) & 0x00020002) != 0x00020002) {
	/* �������뽪λ���Ԥ� */
		writel_sqrom(sqrom, SFSW_START, 0x00000001);
		SQROM_Busy_Wait(sqrom);
	}


#if defined(SQROM_DEBUG)
	printk(KERN_INFO "WriteEnable call StatusReg=%x\n", readl_sqrom(sqrom, SFSW_RDATA0));
#endif
}


/* ���եȥ⡼�ɤǻ��ꥢ�ɥ쥹����16�Х����ɤ߽Ф��Υ��ޥ��ȯ�Ԥ�Ԥ� */
void Sqrom_soft_read_set_16byte(const struct epson_sqrom_data * sqrom) {
#if defined(SQROM_DEBUG3)
	 printk(KERN_INFO "%s called(addr=%08x) \n", __FUNCTION__, sqrom->read_addr);
#endif


		/* �оݤΥ��ɥ쥹�򥻥å� */
		writel_sqrom(sqrom, SFSW_ADR, sqrom->read_addr);

		/* ���åץ��쥯�� */
		writel_sqrom(sqrom, SFSW_CS_SET, sqrom_chip_select );

		//�꡼�ɥ��ޥ�ɥ��ޥ�ɥ��å�
		/* SFSW_DW_SET SoftModeSet2(Data/Cmd/Addr/Modebit��DataTyep��DataLen)  */
		/* ����16�Х����ɤ߽Ф��Ƥ���(0xF)*/
		writel_sqrom(sqrom, SFSW_CMD_SET, (0x010000EB) ) ;
		if (sqrom->chip_mode == 2){
			writel_sqrom(sqrom, SFSW_MODE_SET, 0x02010053) ;
			writel_sqrom(sqrom, SFSW_DW_SET,   (0x1111100F) );
		}
		else if (sqrom->chip_mode == 1){
			writel_sqrom(sqrom, SFSW_MODE_SET,  0x02000053) ;
			writel_sqrom(sqrom, SFSW_DW_SET,  (0x1111100F));
		}


		/* �񤭹��߼¹� */
		writel_sqrom(sqrom, SFSW_START,   0x00000001 );
		/* ���ޥ�ɼ¹Դ�λ�Ԥ� */
		SQROM_Busy_Wait(sqrom);
		
}

		
/* ���եȥ꡼�ɤ�ȯ�Ԥ��줿�ǡ������ɤ߽Ф� */		
void Sqrom_soft_read_data_16byte(const struct epson_sqrom_data * sqrom, char *data) {
		
	unsigned long data_buf = 0;
	unsigned long * p = (unsigned long)data;
	int i = 0;
	
		data_buf= readl_sqrom(sqrom, SFSW_RDATA0);
#if defined(SQROM_DEBUG2)
		printk(KERN_INFO "RDATA0 = %08X, ", data_buf);	
#endif
		*p = data_buf;


		data_buf= readl_sqrom(sqrom, SFSW_RDATA1);
#if defined(SQROM_DEBUG2)
		printk(KERN_INFO "RDATA1 = %08X, ", data_buf);	
#endif	
		p++;
		*p = data_buf;

		data_buf= readl_sqrom(sqrom, SFSW_RDATA2);
#if defined(SQROM_DEBUG2)
		printk(KERN_INFO "RDATA2 = %08X, ", data_buf);	
#endif
		p++;
		*p = data_buf;

		data_buf= readl_sqrom(sqrom, SFSW_RDATA3);
#if defined(SQROM_DEBUG2)
			printk(KERN_INFO "RDATA3 = %08X\n", data_buf);	
#endif
		p++;
		*p = data_buf;

}

/* Linux�ɥ饤����Ϥ��٤ƥ��եȥ⡼��(�Ρ��ޥ�⡼��)���ɤ߽� */
void SF_NormalMode(struct epson_sqrom_data * sqrom){
#if defined(SQROM_DEBUG)
	 printk(KERN_INFO "%s called \n", __FUNCTION__);
#endif

	sqrom->sqrom_mode = NORMAL_MODE; 
	/* CS 0 */

	/* ���åץ��쥯�� */
	writel_sqrom(sqrom, SFSW_CS_SET, sqrom_chip_select );

}
void HAL_SF_NormalMode(struct spi_device * spi )
{
	struct spi_master * master = spi->master;
	struct epson_sqrom_data * sqrom = spi_master_get_devdata(master);
#ifdef CONFIG_MACH_EPSON12_FLD_ACCESS_CTRL
	mutex_lock(&epson_fld_lock);
	epson12_flashrom_right_request();
#endif
	SF_NormalMode(sqrom);
}

/* Linux¦�ν����򽪤��ݤ˥����쥯�ȥ⡼�ɤˤ��Ƥ���ȴ���뤬��SQROM�Ǥ��ä˽�����Ԥ�ʤ� */
void HAL_SF_DirectMode(struct spi_device * spi)
{
	struct spi_master * master = spi->master;
	struct epson_sqrom_data * sqrom = spi_master_get_devdata(master);
#if defined(SQROM_DEBUG)
	 printk(KERN_INFO "%s called \n", __FUNCTION__);
#endif
	
	SQROM_Busy_Wait(sqrom);
	sqrom->sqrom_mode = DIRECT_MODE; 
#ifdef CONFIG_MACH_EPSON12_FLD_ACCESS_CTRL
	epson12_flashrom_right_release();
	mutex_unlock(&epson_fld_lock);
#endif

}

/* ASIC�Υ��åȥ��å� */
int epson_sqrom_setup(struct spi_device * spi)
{
	struct spi_master * master = spi->master;
	struct epson_sqrom_data * sqrom = spi_master_get_devdata(master);

	/* must uncomment this when IPL set correct h/w depend value */
	HAL_SF_Init(sqrom, SF_W25Q128BV);
	/* nothing to do in this function 'cause we do sqrom initialize at m25p80 */
#if defined(SQROM_DEBUG)
	 printk(KERN_INFO "%s called \n", __FUNCTION__);
#endif

	return 0;
}

#if defined(SQROM_DEBUG)
char myline[256];
char mychars[256];
char myform[256];

void sqrom_dump(const void * buf, unsigned int len)
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
		printk(KERN_INFO "%s", myform);
		if (p + j >= end) {
			break;
		}
		p += 16;
	} while (1);
	return;
}
#else
inline void sqrom_dump(const void * buf, unsigned int len) {}
#endif

/* �饤�ȥ��ޥ�ɤθ�����Ǥ���饤�ȤΥǡ������Τ�񤭹���ؿ� */
/* ���åפ��Ф���饤�ȥ��͡��֥�䥢�ɥ쥹����ʤɤϥ���ȥ��餬��ư�ǹԤ� */
void sqrom_write_fifo(struct epson_sqrom_data * sqrom,
	const void * write_buf, 
	unsigned int len)
{
	int i = 0;
	unsigned int reg_len = 4*((unsigned int)((len+3)/4) );
	unsigned long  * write_p = write_buf;
	unsigned long mvalue;

#if defined(SQROM_DEBUG)
	printk(KERN_INFO "write_len= %d,reg_len=%d write_arddr=%x\n", len, reg_len, sqrom->write_addr);
#endif
		/* �饤�ȥ��͡��֥�ȯ�� */
		Sqrom_Write_Enable(sqrom) ;

		writel_sqrom(sqrom, SFSW_CS_SET, sqrom_chip_select );

		/* FIFO���ꥢ */
		writel_sqrom(sqrom, SFSW_WDATA_PTR, 0x00000000);

		/* �Хåե������ɤ߽Ф��ơ�FIFO������� */		
		/* �ǥХ����ȥե����륷���ƥ���Թ��4�Х��ȶ����ˤʤ�褦�˥��ߡ��Х��Ȥ�Ĥ�� */
		/* �ե����륷���ƥ�ϵ��ˤ��ʤ��ǽ񤭹��ߤ��ɤ߽Ф���Ԥ� */

                /* 4�Х���̤���ξ�� ü��ʬ��FF�Ȥ��ƥӥåȤ���Ȥ��ʤ��褦�ˤ�������� */
		if(len < 4){
			if(len == 1){
				mvalue = *write_p;
				mvalue = (mvalue | 0xFFFFFF00);
			}
			else if(len == 2 ){
				mvalue = *write_p;
				mvalue = (mvalue | 0xFFFF0000);
			}
			else if(len == 3 ){
				mvalue = *write_p;
				mvalue = (mvalue | 0xFF000000);
			}
#if defined(SQROM_DEBUG) 
			printk(KERN_INFO "SFSW_WDATA set %08x len=%d\n", mvalue, len);
#endif
			writel_sqrom(sqrom, SFSW_WDATA, mvalue);
		}
		else{


				/* 4�Х��Ȱʾ�ξ���ü���ʳ� */
				for (i = 0; i < (reg_len/4)  ; i++ ) {
						mvalue = *write_p;
#if defined(SQROM_DEBUG)
						printk(KERN_INFO "SFSW_WDATA set %x\n", mvalue);
#endif
						writel_sqrom(sqrom, SFSW_WDATA, mvalue);
						write_p++;
				}
				/* ü���Х��Ȥ�ޤ�ǽ�4�Х���ʬ������� */
			if((reg_len - len) == 3){
				mvalue = *write_p;
				mvalue = (mvalue | 0xFFFFFF00);
			}
			else if((reg_len - len) == 2 ){
				mvalue = *write_p;
				mvalue = (mvalue | 0xFFFF0000);
			}
			else if((reg_len - len) == 1 ){
				mvalue = *write_p;
				mvalue = (mvalue | 0xFF000000);
			}
			writel_sqrom(sqrom, SFSW_WDATA, mvalue);


		}

		/* �оݤΥ��ɥ쥹�򥻥å� */
		writel_sqrom(sqrom, SFSW_ADR, sqrom->write_addr);

		/* �ڡ����ץ���ॳ�ޥ�ɥ��å�Quad mode�ǽ񤯤Τ�(32) */
		/* Quadmode�ǤϤʤ����ϡ�MODE_SET�������ͤ��ۤʤ� */
		writel_sqrom(sqrom, SFSW_CMD_SET, 0x00000032) ;
		if (sqrom->chip_mode == 2){
			writel_sqrom(sqrom, SFSW_MODE_SET, 0x00010040) ;
		}
		else if (sqrom->chip_mode == 1){
			writel_sqrom(sqrom, SFSW_MODE_SET,  0x00000040) ;
		}
		
	
		/* SFSW_DW_SET SoftModeSet2(Data/Cmd/Addr/Modebit��DataTyep��DataLen)  */
		/* ���ޥ�ɤΤۤ��˥ǡ��������뤫�ɤ��������ꤹ���ͤ��Ѥ�� ����256Byte�Ȥ��� */
		/* 1chip�⡼�ɤ�2chip�⡼�ɤǤΥ饤�Ⱥ��祵�������ۤʤ�ΤǾ������ۤ��ˤ��碌�Ƥ��� */
		/* �쥸�������ͤ�(�񤭹���Ĺ��-1)�����ꤹ����� */
		if(sqrom->chip_mode == 2){
			writel_sqrom(sqrom, SFSW_DW_SET,   (0x11100000)+(reg_len-1) );
#if defined(SQROM_DEBUG)
			printk(KERN_INFO "SFSW_DW_SET= %x\n", readl_sqrom(sqrom, SFSW_DW_SET ));
			printk(KERN_INFO "SFSW_ADR= %x\n", readl_sqrom(sqrom, SFSW_ADR ));
#endif
		}
		else if(sqrom->chip_mode == 1){
			writel_sqrom(sqrom, SFSW_DW_SET,  (0x11100000)+(reg_len-1));
#if defined(SQROM_DEBUG)
			printk(KERN_INFO "SFSW_DW_SET= %x\n", readl_sqrom(sqrom, SFSW_DW_SET ));
			printk(KERN_INFO "SFSW_ADR= %x\n", readl_sqrom(sqrom, SFSW_ADR ));
#endif
		}
		else{
			 printk(KERN_WARNING "chip_mode %d is not suport \n", sqrom->chip_mode);
		}

		/* �񤭹��߼¹� */
		writel_sqrom(sqrom, SFSW_START,   0x00000001 );
		/* ���ޥ�ɼ¹Դ�λ�Ԥ� */
		SQROM_Busy_Wait(sqrom);
		// BUSY=0���ǧ
		Sqrom_BUSY0_Wait(sqrom) ;
}

/******************************************************************/

/* �񤭹��ߴؿ� */
/*
* OPCODE_WREN             0x06     Write enable 
* OPCODE_WRSR             0x01     Write status register 1 byte 
* OPCODE_PP               0x02     Page program (up to 256 bytes) 
* OPCODE_BE_4K            0x20     Erase 4KiB block 
* OPCODE_BE_32K           0x52     Erase 32KiB block 
* OPCODE_CHIP_ERASE       0xc7     Erase whole flash chip 
* OPCODE_SE               0xd8     Sector erase (usually 64KiB) 
*
* OPCODE_NORM_READ        0x03     Read data bytes (low frequency) 
* OPCODE_FAST_READ        0x0b     Read data bytes (high frequency)
* OPCODE_QUAD_READ        0x0b     Read data bytes (Quad i/o read)
* OPCODE_RDSR             0x05     Read status register 
* OPCODE_RDID             0x9f     Read JEDEC ID 
*/
int sqrom_write(struct epson_sqrom_data * sqrom,
	const void * write_buf, 
	unsigned int len,
	dma_addr_t flash_buf
)
{
	int i;
	const unsigned char * p = write_buf;
	char opcode = p[0];
	unsigned long mvalue;

#if defined(SQROM_DEBUG)
	printk(KERN_INFO "in %s,  write_buf = %p, len = %d, flash = %08x chip_mode=%d \n",
					 __FUNCTION__, write_buf, len, flash_buf, sqrom->chip_mode);	
	if (len < 100) {
		sqrom_dump(p, len);
	}
#endif

	if(sqrom->sqrom_mode != NORMAL_MODE){
		SF_NormalMode(sqrom);
	} 
	/* CS  */
	writel_sqrom(sqrom, SFSW_CS_SET, sqrom_chip_select );

	/* OPCODE��Write�Τ��ȤǸƤФ��ǡ������Τξ�� */
	if(sqrom->write_mode == WRITE_DATA){
		/* ����ȥ����FIFO��ͳ�ǽ񤭹���ؿ��ǽ��� */
		sqrom_write_fifo(sqrom, write_buf, len);
		/* ���ޥ�ɼ��ե⡼�ɤ��᤹ */
		sqrom->write_mode = WRITE_COMMAND;
		return len;
	}

	/* ���ޥ�ɤξ�� */
	/* ���졼���ξ�� */
	if( (opcode==OPCODE_BE_4K) ||(opcode == OPCODE_BE_32K) ||(opcode == OPCODE_CHIP_ERASE) || (opcode == OPCODE_SE) ){
		/* �оݥ��ɥ쥹����� */
		mvalue = 0;
		mvalue = ( p[4] & 0x000000ff ) | (( p[3] & 0x000000ff ) << 8) | (( p[2] & 0x0000ff ) << 16) |(( p[1] & 0x0000ff ) << 24) ;

#if defined(SQROM_DEBUG)
	printk(KERN_INFO "Erase command opcode= %x,  addr=%08x  \n", opcode, mvalue);
#endif
		/* �оݤΥ��ɥ쥹�򥻥å� */
		writel_sqrom(sqrom, SFSW_ADR, mvalue);
		/* �饤�ȥ��͡��֥�ȯ�� */
		Sqrom_Write_Enable(sqrom) ;
		/* eraseȯ�� */
		writel_sqrom(sqrom, SFSW_CMD_SET, (u32)opcode) ;
		if (sqrom->chip_mode == 2){
			writel_sqrom(sqrom, SFSW_MODE_SET, 0x00010000) ;
		}
		else if (sqrom->chip_mode == 1){
			writel_sqrom(sqrom, SFSW_MODE_SET, 0x00000000) ;
		}
			
		/* erase�ϥǡ���Ĺ���ʤ��Τǡ�Ĺ���ϴط��ʤ� */
                if(opcode == OPCODE_CHIP_ERASE){
                        writel_sqrom(sqrom, SFSW_DW_SET, 0x01000000) ;
                }else{
                        writel_sqrom(sqrom, SFSW_DW_SET, 0x01100000) ;
                }
		/* erase���� */
		writel_sqrom(sqrom, SFSW_START, 0x00000001) ;
		/* ���ޥ�ɻ��¹Դ�λ�Ԥ� */
		SQROM_Busy_Wait(sqrom);

		/* BUSY=0��ǧ ���å׼��Τν����Ԥ� */
		Sqrom_BUSY0_Wait(sqrom) ;
		return len;
	
	}
	else if( opcode==OPCODE_WRSR){
		/* ���ơ������쥸�����饤�� */
		printk(KERN_ERR "NOT YET INPRIMENT Write Status register\n");	
	}
	else if( opcode==OPCODE_WREN){
		Sqrom_Write_Enable(sqrom) ;
	}
	else if(opcode==OPCODE_PP){
		/* �оݥ��ɥ쥹����� */
		mvalue = 0;
		mvalue = ( p[4] & 0x000000ff ) | (( p[3] & 0x000000ff ) << 8) | (( p[2] & 0x0000ff ) << 16) |(( p[1] & 0x0000ff ) << 24) ;
		/* �оݤΥ��ɥ쥹��Ф��� */
		sqrom->write_addr = mvalue;

		/* �ǡ����饤�ȤΥ⡼�ɤ򥻥åȤ��ƴؿ���ȴ���� */
		sqrom->write_mode = WRITE_DATA;
		return len;
	}
	else if(opcode == OPCODE_RDSR ){
	 /* ���ơ������쥸�����Υ꡼�� */

		//�꡼�ɥ��ơ������쥸�������ޥ��(05)ȯ��
		writel_sqrom(sqrom, SFSW_CMD_SET, 0x00000005) ;
		if (sqrom->chip_mode == 2) {
			sqrom->chip_read = READ_SINGLE;
			writel_sqrom(sqrom, SFSW_MODE_SET, 0x00010000);
			writel_sqrom(sqrom, SFSW_DW_SET, 0x11001001);
		}
		else if (sqrom->chip_mode == 1) {
			writel_sqrom(sqrom, SFSW_MODE_SET, 0x00000000);
			writel_sqrom(sqrom, SFSW_DW_SET, 0x11001000);
		}
		writel_sqrom(sqrom, SFSW_START, 0x00000001);
	
		/* �������뽪λ���Ԥ� */
		SQROM_Busy_Wait(sqrom);
		return len;
	}
	else if(opcode == OPCODE_RDID ){
		/* JDEC CODE �Υ꡼�� */
		//(9F)ȯ��
		writel_sqrom(sqrom, SFSW_CMD_SET, 0x0000009F) ;
		if (sqrom->chip_mode == 2) {
			/* read chip single */
			sqrom->chip_read = READ_SINGLE;
			writel_sqrom(sqrom, SFSW_MODE_SET, 0x00010000);
			/* micronix:11001003, winbond:11001005, spansion:0x11001009 */
			writel_sqrom(sqrom, SFSW_DW_SET, 0x11001005);
		}
		else if (sqrom->chip_mode == 1) {
			writel_sqrom(sqrom, SFSW_MODE_SET, 0x00000000);
			/* micronix:11001001, winbond:11001002, spansion:0x11001004 */
			writel_sqrom(sqrom, SFSW_DW_SET, 0x11001002);
		}
		writel_sqrom(sqrom, SFSW_START, 0x00000001);
	
		/* �������뽪λ���Ԥ� */
		SQROM_Busy_Wait(sqrom);
		return len;
	}
	else if((opcode == OPCODE_NORM_READ) || (opcode == OPCODE_FAST_READ) || (opcode == OPCODE_QUAD_READ) ){
		/* read ���� */
		/* �оݥ��ɥ쥹����� */
		mvalue = 0;
		mvalue = ( p[4] & 0x000000ff ) | (( p[3] & 0x000000ff ) << 8) | (( p[2] & 0x0000ff ) << 16) |(( p[1] & 0x0000ff ) << 24) ;
		/* �꡼���оݥ��ɥ쥹��Ф��Ƥ���  */
		sqrom->read_addr = mvalue;

#if 1
		/* ���եȥ⡼�ɤǤϤʤ����ɤ߽Ф��Τߥ����쥯�ȥ⡼�ɤ�ư���褦�ˤ���ե饰 */
		sqrom->read_mode = READ_DIRECT;
#endif

#if defined(SQROM_DEBUG)
		printk(KERN_INFO "read address %08x  \n", mvalue);
#endif
		Sqrom_soft_read_set_16byte(sqrom);


	}
	else{
		/* ̤�����Υ��ޥ�� */
		printk(KERN_ERR "NOT YET INPRIMENT OPCODE=%d\n", opcode);	
	}
		return len;
}

/******************************************************************/
/* �ɤ߽Ф��ؿ� */
/*
	�ǡ����Υ쥸������꡼�ɤ���
	�꡼�ɥ��ޥ�ɤ�ȯ�Ԥȥ��ơ������Υӥ����ƻ��Write�����ǽ���äƤ���
*/
int sqrom_read(struct epson_sqrom_data * sqrom,
	void * read_buf,
	unsigned int len,
	dma_addr_t flash_buf
)
{
	char * p = read_buf;
	unsigned long * src_ptr ;
	unsigned long data_buf;
	unsigned long * read_buf_tmp = read_buf;
	unsigned int read_len = 0;
	int i = 0;
#if defined(SQROM_DEBUG)
	printk(KERN_INFO "in %s read_buf = %p, len = %d, flash = %08x\n", __FUNCTION__, read_buf, len, flash_buf);	
#endif
	/* opcode��read�ξ������쥯�ȥ꡼�ɤ��֤� */
	if( sqrom->read_mode == READ_DIRECT){
#if defined(SQROM_DEBUG)
		printk(KERN_INFO "sqrom->read_mode=%d\n", sqrom->read_mode );	
		printk(KERN_INFO "sqrom->chip_base_addr=%08x\n", sqrom->chip_base_addr );	
		printk(KERN_INFO "sqrom->read_addr=%08x\n", sqrom->read_addr );	
		printk(KERN_INFO "in %s read_addr = %x, len = %d,\n", __FUNCTION__, (sqrom->read_addr + sqrom->chip_base_addr), len  );	
#endif
		/* �����쥯�ȥ⡼�ɤ����ꤹ�����̤ʽ����Ϥʤ� */
		SQROM_Busy_Wait(sqrom);

		/* �ɤ߽Ф����� */
		src_ptr = (unsigned long *)(sqrom->chip_base_addr + sqrom->read_addr);
		memcpy(read_buf, src_ptr, len);
		read_len = len;
#if 0
		for(i=0; i<(len/4); i++){	
			*read_buf_tmp= *src_ptr;
			src_ptr++;
			read_buf_tmp++;
			read_len+=4;
		}
#endif

		sqrom->read_mode = READ_NORMAL;
#if defined(SQROM_DEBUG)
		if (read_len < 100) {
			sqrom_dump(read_buf, read_len);
		}
		printk(KERN_INFO "return read len = %d\n", read_len);
	
#endif
		return read_len;
	}

	/* data read from 1chip on 2chip mode */
	if(sqrom->chip_read == READ_SINGLE){
		/* JDEC CODE �ʤ�2chip�⡼�ɤǤ���¦�Υ��åפΤߤ����ɤ߹��ߤ������ */

		data_buf= readl_sqrom(sqrom, SFSW_RDATA0);
#if defined(SQROM_DEBUG)
		printk(KERN_INFO "RDATA0 = %08x, ", (void *)data_buf);	
#endif
		*p = (char)(data_buf & 0x000000ff);
		read_len++;
		if(len > 1){
			p++;
			*p = (char)((data_buf & 0x0000ff00) >> 8);
			read_len++;
		}

		if(len > 2){
			data_buf= readl_sqrom(sqrom, SFSW_RDATA1);
#if defined(SQROM_DEBUG)
			printk(KERN_INFO "RDATA1 = %08x, ",(void *)data_buf);	
#endif
			p++;
			*p = (char)(data_buf & 0x000000ff);
			read_len++;
		}
		if(len > 3){
			p++;
			*p = (char)((data_buf & 0x0000ff00) >> 8);
			read_len++;
		}

		if(len > 4){
			data_buf= readl_sqrom(sqrom, SFSW_RDATA2);
#if defined(SQROM_DEBUG)
			printk(KERN_INFO "RDATA2 = %08x, ", (void *)data_buf);	
#endif
			p++;
			*p = (char)(data_buf & 0x000000ff);
			read_len++;
		}
		if(len > 5){
			p++;
			*p = (char)((data_buf & 0x0000ff00) >> 8);
			read_len++;
		}

		if(len > 6){
			data_buf= readl_sqrom(sqrom, SFSW_RDATA3);
#if defined(SQROM_DEBUG)
		printk(KERN_INFO "RDATA3 = %08x\n", data_buf);	
#endif
			p++;
			*p = (char)(data_buf & 0x000000ff);
			read_len++;
		}
		if(len > 7){
			p++;
			*p = (char)((data_buf & 0x0000ff00) >> 8);
			read_len++;
		}

		/* 2chip�����ɤ�����ˤ�ɤ�(�Хåե��򤽤Τޤ��ɤ�⡼��)  */
		sqrom->chip_read = READ_DOUBLE;
	}
	else{
	/* data read from 2chip on 2chip mode */
#if defined(SQROM_DEBUG3)
		printk(KERN_INFO "data read from 2chip on 2chip mode(len=%d)\n", len);
#endif
		char data_tmp[16];
		Sqrom_soft_read_data_16byte(sqrom, data_tmp);
		for(i=0; i<16; i++){
			*p = data_tmp[i];
			read_len++;
			p++;

			if(read_len >= len){
			/* �ɤ߽Ф���Ĺ�������ꥵ�����ޤǤ����齪λ */
				return read_len;
			}
		}
		if(len > 16){
			/*��16�Х��Ȱʾ�ξ�硢���ɥ쥹��ʤ�ʤ���Ĺ��ʬ�ޤ��ɤ� */
			while(read_len < len){
				sqrom->read_addr +=16; 
				Sqrom_soft_read_set_16byte(sqrom);
				Sqrom_soft_read_data_16byte(sqrom, data_tmp);
				for(i=0; i<16; i++){
					*p = data_tmp[i];
					read_len++;
					p++;
					if(read_len >= len){
					/* �ɤ߽Ф���Ĺ�������ꥵ�����ޤǤ����齪λ */
						return read_len;
					}
				}
			}
		}


	}


#if defined(SQROM_DEBUG)
	if (read_len < 512) {
		sqrom_dump(read_buf, read_len);
	}
	printk("return read len = %d\n", read_len);
	
#endif
	return read_len;
}

/******************************************************************/
/* ž���ؿ����� �ɤ߹��ߤȽ񤭹��ߤ�Ԥ� */
unsigned int sqrom_handle_1transfer(
	struct spi_device * spi, 
	struct spi_transfer * transfer
)
{
	struct spi_master * master = spi->master;
	struct epson_sqrom_data * sqrom = spi_master_get_devdata(master);

	struct spi_transfer * last_transfer = transfer;
	unsigned int len = 0;
	if (last_transfer->tx_buf != NULL) {
		len = sqrom_write(sqrom, last_transfer->tx_buf, last_transfer->len, last_transfer->tx_dma);
	}
	if (last_transfer->rx_buf != NULL) {
		len = sqrom_read(sqrom, last_transfer->rx_buf, last_transfer->len, last_transfer->rx_dma);
	}
	return len;
}

/* ž���ؿ��θƤӽФ����󥿡��ե����� ��̤�SPI�ե졼��������ƤӽФ���� */
int epson_sqrom_transfer(struct spi_device * spi, struct spi_message * mesg)
{
	struct spi_transfer * last_transfer; 
	unsigned int len = 0;


	list_for_each_entry(last_transfer, &(mesg->transfers), transfer_list) {
		len += sqrom_handle_1transfer(spi, last_transfer);
	}

	mesg->complete(mesg->context);
	mesg->status = 0;
	mesg->actual_length = len;
#if defined(SQROM_DEBUG2)
	printk(KERN_INFO "transfer message complete(len=%d)\n", len);
#endif
	return 0;
}


#ifdef CONFIG_MACH_EPSON12_FLD_ACCESS_CTRL
int epson_sqrom_transfer_with_access_control(struct spi_device * spi, struct spi_message * mesg)
{
	struct spi_transfer * last_transfer; 
	unsigned int len = 0;



	list_for_each_entry(last_transfer, &(mesg->transfers), transfer_list) {
		len += sqrom_handle_1transfer(spi, last_transfer);
	}




	mesg->complete(mesg->context);
	mesg->status = 0;
	mesg->actual_length = len;
#if defined(SQROM_DEBUG2)
	printk(KERN_INFO "transfer message complete(len=%d)\n", len);
#endif
	return 0;
}
#endif

/* ������겼�ϥ����ͥ륤�󥿡��ե������ˤ��������Ͽ�����ʬ */
void epson_sqrom_cleanup(const struct spi_device * spi)
{
	struct spi_master * master = spi->master;
	struct epson_sqrom_data * sqrom = spi_master_get_devdata(master);
#if defined(SQROM_DEBUG)
	 printk(KERN_INFO "%s called \n", __FUNCTION__);
#endif
	HAL_SF_End(sqrom);

	return;
}

/* guarantee that all i/o area allocated or no i/o area allocated */
int sqrom_alloc_memory(struct spi_master * master)
{
	struct epson_sqrom_data * sqrom_data = spi_master_get_devdata(master);
#if defined(SQROM_DEBUG)
	printk(KERN_INFO  "called %s\n", __FUNCTION__);
#endif

	sqrom_data->sqrom_base = (unsigned long)ioremap(ASIC_SQROMIF_BASE, SZ_8K);
	if (sqrom_data->sqrom_base == 0) {
		goto err;
	}

	sqrom_data->chip_base_addr = IO_ADDRESS(EPSON12_SFLASH_BASE);
	sqrom_data->alloc_success = 1;
#if defined(SQROM_DEBUG)
	 printk(KERN_INFO "ioremap sqrom_base=%08x  \n", sqrom_data->sqrom_base);
	 printk(KERN_INFO "ioremap chip_base_addr=%08x  \n", sqrom_data->chip_base_addr);
#endif
	return 0;

err:
	sqrom_data->alloc_success = 0;

	return ENOMEM;
}

int sqrom_release_memory(struct spi_master * master)
{
	struct epson_sqrom_data * sqrom_data = spi_master_get_devdata(master);
	if (!sqrom_data->alloc_success) {
		return 0;	// nothing to do in this case
	}
	iounmap((void *)sqrom_data->sqrom_base);

	return 0;
}

int __init epson_sqrom_probe(struct platform_device * dev)
{
	struct spi_master *master;
	struct epson_sqrom_data * sqrom_data;
	int ret = 0;
        struct proc_dir_entry *entry;
	int i;

#if defined(SQROM_DEBUG)
	printk(KERN_INFO  "called %s\n", __FUNCTION__);
#endif
#ifdef CONFIG_EPSON12_WAIT_RTOS_WAKEUP
	/* Wait RTOS magic number. */
	local_irq_disable();
	for (i = 0; 0xFACEFACE != readl(__io_address(EPSON12_SRAM_BASE) + SCRATCH_PAD_OFFSET); i++ ) {
		mdelay(10);
		if (i % 100 == 0) {
			printk(KERN_INFO  "wait RTOS magic number\n");
		}
	}
	local_irq_enable();
#endif
	/* Get resources(memory, IRQ) associated with the device */
	master = spi_alloc_master(&dev->dev, sizeof(struct epson_sqrom_data));
	if (master == NULL) {
		ret = -ENOMEM;
#if defined(SQROM_DEBUG)
		printk(KERN_INFO  "spi_alloc_master err\n");
#endif
		goto err;
	}

#if defined(SQROM_DEBUG)
	printk(KERN_INFO  "platform_set_drvdata\n");
#endif
	platform_set_drvdata(dev, master);

	ret = sqrom_alloc_memory(master);
	if (ret != 0) {
#if defined(SQROM_DEBUG)
	printk(KERN_INFO  "sqrom_alloc_memory fail\n");
#endif
		goto free_master;
	}
	
	/* SPI controller can be understood this mode */
        master->mode_bits = SPI_MODE_3;

	/* SPI controller initializations */
	sqrom_data = spi_master_get_devdata(master);
	sqrom_data->master = master;
	sqrom_data->chip_mode = sqrom_chip_mode;
	sqrom_data->sqrom_mode = MODE_NOT_SET;
	printk(KERN_NOTICE "SQROMIF ChipMode=%d, ChipSelect=%d\n", sqrom_chip_mode, sqrom_chip_select);

#if defined(SQROM_DEBUG)
	printk(KERN_INFO "called %s, master = %p, sqrom_data = %p\n", __FUNCTION__, master, sqrom_data);
#endif
	
	spin_lock_init(&sqrom_data->lock);

	master->bus_num = 0;
	master->num_chipselect = 3;
	master->cleanup = epson_sqrom_cleanup;
	master->setup   = epson_sqrom_setup;

#ifndef CONFIG_MACH_EPSON12_FLD_ACCESS_CTRL
	master->transfer = epson_sqrom_transfer;
#else

	if( readb((char *)(IO_ADDRESS(EPSON12_SRAM_BASE) + FLD_OFFSET_EXE_MODE)) == FLD_EXEMODE_ON){
		master->transfer = epson_sqrom_transfer_with_access_control;
		printk(KERN_NOTICE "sqrom : master->transfer is transfer with accesscontrol \n");
		printk(KERN_NOTICE "sqrom : MUST WRITE dicimal 0 to %s, when system boot finish",proc_entry_name);
        	entry = create_proc_entry(proc_entry_name, S_IFREG | S_IRUGO | S_IWUGO, NULL);
		if (entry != NULL) {
			entry->read_proc = epson12_flashrom_right_ctrl_read;
			entry->write_proc = epson12_flashrom_right_ctrl_write;
			/* while systembootup, Linux core handle flashrom access right */
			epson12_flashrom_right_request();
			flashrom_right_ctrl_enable_lock = 1;
		}


	}else{
		master->transfer = epson_sqrom_transfer;
		printk(KERN_NOTICE "sqrom : master->transfer is normal transfer \n");
	}
#endif
	
	ret = spi_register_master(master);
	if (ret != 0) {
		printk(KERN_ERR "sqrom : can not register as spi device\n");
		goto free_master;
	}
	
	return ret;
free_master:
	kfree(master);
err:
	return ret;
} 


static __init int sqrom_chip_select_setup(char *s)
{
        sqrom_chip_select = (u32)simple_strtoul(s, NULL, 10);
        return 1;
}
static __init int sqrom_chip_mode_setup(char *s)
{
        sqrom_chip_mode = (u32)simple_strtoul(s, NULL, 10);
	if(sqrom_chip_mode == 0){
		sqrom_chip_mode = 2;
	}
        return 1;
}

__setup("cs=", sqrom_chip_select_setup);
__setup("cm=", sqrom_chip_mode_setup);

int __devexit epson_sqrom_remove(struct platform_device *dev)
{
	struct spi_master * master = platform_get_drvdata(dev);

	sqrom_release_memory(master);
	kfree(master);
	
	return 0;
}

struct platform_driver epson_sqrom_driver = {
	.probe = epson_sqrom_probe,
	.remove = epson_sqrom_remove,
	.driver = {
		.name = "sqrom",
		.owner = THIS_MODULE,
	},
};

int /*__init*/ epson_sqrom_init(void)
{
	printk(KERN_INFO "called %s\n", __FUNCTION__);
	return platform_driver_register(&epson_sqrom_driver);
}

void __exit epson_sqrom_exit(void)
{
	platform_driver_unregister(&epson_sqrom_driver);
}

module_init(epson_sqrom_init);
module_exit(epson_sqrom_exit);

MODULE_AUTHOR("SEIKO EPSON CORPORATION");
MODULE_DESCRIPTION("EPSON SPI(SQROMIF) Driver");
MODULE_LICENSE("GPL");
