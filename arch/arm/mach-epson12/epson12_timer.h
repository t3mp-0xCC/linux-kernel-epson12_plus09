/*
 * Copyright (c)SEIKO EPSON CORPORATION 2012. All rights reserved.
 *
 * License : GPL2.0
 */

#ifndef __EPSON12_TIMER_H
#define __EPSON12_TIMER_H

/*
 * Register offset R/W
 */
#define TIMER_CLKCTL	IO_ADDRESS(EPSON12_TIMER_BASE) /* Clock Control Register */

#define TIMER_REGISTER_ADDRESS(x) (IO_ADDRESS(EPSON12_TIMER_BASE) + EPSON12_TIMERU1 + x)
#define TIMER_INTSTS	0x04 /* Timer Interrupt Status Register */
#define TIMER_INTCLR	0x08 /* Timer Interrupt Clear Register */
#define TIMER_INTMSK	0x0c /* Timer Interrupt Mask Register */
#define TIMER_CTL_CH4	0x10 /* Timer Control Register */
#define TIMER_CNT_CH4	0x14 /* Timer Counter Register */
#define TIMER_CMPA_CH4	0x18 /* Timer Compare-A Register */
#define TIMER_CMPB_CH4	0x1c /* Timer Compare-B Register */
#define TIMER_FINT_CH4	0x20

/*
 * Control register bitfields
 */
/* 11-8	: clock select, No division */
#define CONTROL_PSCSEL		0x00000000
/* 6	: target b, Allow count clear when comparing matched */
#define CONTROL_CMPB_CLR	0x00000040
/* 5	: target a, Allow count clear when comparing matched */
#define CONTROL_CMPA_CLR	0x00000020
/* 4	: Allow overflow interrupt */
#define CONTROL_OVF_INTEN	0x00000010
/* 3	: target b, Allow compare match interrupt */
#define CONTROL_CMPB_INTEN	0x00000008
/* 2	: target a, Allow compare match interrupt */
#define CONTROL_CMPA_INTEN	0x00000004
/* 1	: Overflow stop setting */
#define CONTROL_OVFSTP		0x00000002
/* 0	: Allow timer run bit */
#define CONTROL_RUN		0x00000001

/*
 * Interrupt Status bitfields
 */
/* 0	: Event Flag */
#define INTERRUPT_STATUS_INT_CLR 0x00000007

#endif
