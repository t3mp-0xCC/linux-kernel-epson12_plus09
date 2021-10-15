/*
 * linux/arch/arm/mach/itim2.h
 *
 *  COPYRIGHT (C) 2008 SEIKO EPSON, CORPORATION, All rights reserved.
 *
 *  License : GPL2.0
 */

#define ITIM2_BASE_ADDR	IO_ADDRESS(EPSON09_TIMER0_1_BASE)	/* 0xD0003000 */

/* Register */
#define ITIM2_ITCNT 0x00 /* start/stop control */
#define ITIM2_ITSRC 0x04 /* clk src */
#define ITIM2_ITRLD 0x08 /* reload */
#define ITIM2_ITCT 0x0C /* counter */

/*
 * How long is the timer interval?
 */
#define TIMER_INTERVAL	(TICKS_PER_uSEC * mSEC_10)
#define TIMER_RELOAD	(TIMER_INTERVAL >> 4)		/* Divide by 16 */
#define TIMER_DIVISOR	(TIMER_CTRL_DIV16)
#define TICKS2USECS(x)	(16 * (x) / TICKS_PER_uSEC)
