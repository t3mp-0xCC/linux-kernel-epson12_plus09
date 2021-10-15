/*
 *  linux/include/asm-arm/arch-epson09/uncompress.h
 *
 *  Copyright (C) 2003 ARM Limited
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
 */
#include <mach/hardware.h>

#define SCIF_REG	0xe0816000

enum {
	SCIF_SCSMR2	= SCIF_REG + 0x00,
	SCIF_SCBRR2	= SCIF_REG + 0x04,
	SCIF_SCSCR2	= SCIF_REG + 0x08,
	SCIF_SCFTDR2	= SCIF_REG + 0x0c,
	SCIF_SCFSR2	= SCIF_REG + 0x10,
	SCIF_SCFRDR2	= SCIF_REG + 0x14,
	SCIF_SCFCR2	= SCIF_REG + 0x18,
	SCIF_SCFDR2	= SCIF_REG + 0x1c,
	SCIF_SCSPTR2	= SCIF_REG + 0x20,
	SCIF_SCLSR2	= SCIF_REG + 0x24,
};

static inline void ctrl_outb(unsigned char b, unsigned long addr)
{
	*(volatile unsigned char*)addr = b;
}

static inline void ctrl_outw(unsigned short b, unsigned long addr)
{
	*(volatile unsigned short*)addr = b;
}

static inline unsigned short ctrl_inw(unsigned long addr)
{
	return *(volatile unsigned short*)addr;
}

/*
 * This does not append a newline
 */
static inline void putc(int c)
{
	while (!(ctrl_inw(SCIF_SCFSR2) & 0x20))
		barrier();

	ctrl_outb(c, SCIF_SCFTDR2);
	ctrl_outw((ctrl_inw(SCIF_SCFSR2) & 0x9f), SCIF_SCFSR2);

	if (c == '\n') {
		while (!(ctrl_inw(SCIF_SCFSR2) & 0x20))
			barrier();

		ctrl_outb('\r', SCIF_SCFTDR2);
		ctrl_outw((ctrl_inw(SCIF_SCFSR2) & 0x9f), SCIF_SCFSR2);
	}

#if 0	/* test ozawa */
#define EPSON08_TEST_ADDR           0x101F8000  /* UART */
#define EPSON08_TEST_ADDR1 (*(volatile unsigned char *) (EPSON08_TEST_ADDR + 0x00))

	EPSON08_TEST_ADDR1 = 0xFF;
#endif
}

static inline void flush(void)
{
/*
	ctrl_outw((ctrl_inw(SCIF_SCFSR2) & 0xbf), SCIF_SCFSR2);

	while (!(ctrl_inw(SCIF_SCFSR2) & 0x40))
		barrier();

	ctrl_outw((ctrl_inw(SCIF_SCFSR2) & 0xbf), SCIF_SCFSR2);
*/
}

/*
 * nothing to do
 */
#define arch_decomp_setup()
#define arch_decomp_wdog()

