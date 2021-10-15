/*
 *  Copyright (C) 2007 Samsung Electronics TLD.
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
 * Base on linux/drivers/serial/amba-pl011.c
 *
 * History                                                                    
 *  11/08/07    Donghoon Yu     include header create 
 */
 
/* Copyright (c)SEIKO EPSON CORPORATION 2012. All rights reserved. */

#if defined(CONFIG_SERIAL_EPSON12_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/platform_device.h>
#include <mach/serial.h>

#include <linux/clk.h>
#include <linux/slab.h> /* add 2.6.35 */
#include <linux/proc_fs.h>

#include <asm/io.h>
#include <asm/sizes.h>

#define UART_NR			14

#define SERIAL_EPSON12_MAJOR	204
#define SERIAL_EPSON12_MINOR	64
#define SERIAL_EPSON12_NR	UART_NR

#define EPSON12_ISR_PASS_LIMIT	256

#define UART_DR_ERROR		(UART_DR_OE|UART_DR_BE|UART_DR_PE|UART_DR_FE)
#define UART_DUMMY_DR_RX	(1 << 16)

#define PORT_EPSON12 		75


#if defined(CONFIG_SERIAL_EPSON12_CONSOLE_DBG) || defined(CONFIG_SERIAL_EPSON12_DBG)
#define DBG(args...)	printk(args)
#define IFDBG(condition, args...)	if(condition)	printk(args)
#else
#define DBG(args...)	do {} while(0)
#define IFDBG(condition, args...)	do {} while(0)
#endif

static int epson12_console_ctrl_read(
	char *page, char **start, off_t off, int count, int *eof, void *data);
static int epson12_console_ctrl_write(
	struct file *file, const char __user *buffer, unsigned long count, void *data);

static const char const entry_name[] = "epson_console_enable";
#if defined(CONFIG_MACH_EPSON12_DBG)
static int console_enable = 1;
#else
static int console_enable = 0;
#endif

/*
 * We wrap our port structure around the generic uart_port.
 */
struct uart_epson12_port {
	struct uart_port	port;
	struct clk		*clk;
	unsigned int		im;	/* interrupt mask */
	unsigned int		old_status;
};

static void epson12_serial_stop_tx(struct uart_port *port)
{
	struct uart_epson12_port *uap = (struct uart_epson12_port *)port;

	/* 
	 * clear transmit interrupt mask
	 */
	uap->im &= ~UART_TXIM;	
	writew(uap->im, uap->port.membase + UART_IMSC);
}

static void epson12_serial_start_tx(struct uart_port *port)
{
	struct uart_epson12_port *uap = (struct uart_epson12_port *)port;

	IFDBG(!port,"%s: *port is NULL\n",__FUNCTION__);
	/* 
	 * set transmit interrupt mask
	 */
	uap->im |= UART_TXIM;
	writew(uap->im, uap->port.membase + UART_IMSC);
}

static void epson12_serial_stop_rx(struct uart_port *port)
{
	struct uart_epson12_port *uap = (struct uart_epson12_port *)port;

	IFDBG(!port,"%s: *port is NULL\n",__FUNCTION__);
	/* 
	 * clear Receive/Receive timeout/Framing error/Parity error/
	 * Break error/Overrun error interrupt mask	
	 */
	uap->im &= ~(UART_RXIM|UART_RTIM|UART_FEIM|
		     UART_PEIM|UART_BEIM|UART_OEIM);
	writew(uap->im, uap->port.membase + UART_IMSC);
}

static void epson12_serial_enable_ms(struct uart_port *port)
{
	struct uart_epson12_port *uap = (struct uart_epson12_port *)port;
	IFDBG(!port,"%s: *port is NULL\n",__FUNCTION__);
	/*
	 * set nUARTRI/nUARTCTS/nUARTDCD/nUARTDSR modem interrupt mask
	 */
	uap->im |= UART_RIMIM|UART_CTSMIM|UART_DCDMIM|UART_DSRMIM;
	writew(uap->im, uap->port.membase + UART_IMSC);
}

static void epson12_serial_rx_chars(struct uart_epson12_port *uap)
{
	struct tty_struct *tty = uap->port.state->port.tty; // name changed 2.6.33
	unsigned int status, ch, flag, max_count = 256;

	status = readw(uap->port.membase + UART_FR);
	/*
	 * Read until Receive FIFO empty
	 */
	while ((status & UART_FR_RXFE) == 0 && max_count--) {
		ch = readw(uap->port.membase + UART_DR) | UART_DUMMY_DR_RX;
		flag = TTY_NORMAL;
		uap->port.icount.rx++;

		/*
		 * Note that the error handling code is
		 * out of the main execution path
		 */
		if (unlikely(ch & UART_DR_ERROR)) {			// OE | BE | PE | FE
			if (ch & UART_DR_BE) {				// Break error
				ch &= ~(UART_DR_FE | UART_DR_PE);	// Parity | Framing error
				uap->port.icount.brk++;
				if (uart_handle_break(&uap->port))
					goto ignore_char;
			} else if (ch & UART_DR_PE)			// Parity error
				uap->port.icount.parity++;
			else if (ch & UART_DR_FE)			// Framing error
				uap->port.icount.frame++;
			if (ch & UART_DR_OE)				// Overrun error
				uap->port.icount.overrun++;

			ch &= uap->port.read_status_mask;

			if (ch & UART_DR_BE)
				flag = TTY_BREAK;
			else if (ch & UART_DR_PE)
				flag = TTY_PARITY;
			else if (ch & UART_DR_FE)
				flag = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(&uap->port, ch & 255))
			goto ignore_char;

		uart_insert_char(&uap->port, ch, UART_DR_OE, ch, flag);

	ignore_char:
		status = readw(uap->port.membase + UART_FR);
	}
	tty_flip_buffer_push(tty);
	return;
}

static void epson12_serial_tx_chars(struct uart_epson12_port *uap)
{
	struct circ_buf *xmit = &uap->port.state->xmit; // name changed 2.6.33
	int count;

	if (uap->port.x_char) {	/* xon / xoff char */
		writew(uap->port.x_char, uap->port.membase + UART_DR);
		uap->port.icount.tx++;
		uap->port.x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(&uap->port)) {
		epson12_serial_stop_tx(&uap->port);
		return;
	}

	count = uap->port.fifosize >> 1;
	do {
		writew(xmit->buf[xmit->tail], uap->port.membase + UART_DR);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		uap->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&uap->port);

	if (uart_circ_empty(xmit))
		epson12_serial_stop_tx(&uap->port);
}

static void epson12_serial_modem_status(struct uart_epson12_port *uap)
{
	unsigned int status, delta;

	status = readw(uap->port.membase + UART_FR) & UART_FR_MODEM_ANY;

	delta = status ^ uap->old_status;
	uap->old_status = status;

	if (!delta)
		return;

	if (delta & UART_FR_DCD)
		uart_handle_dcd_change(&uap->port, status & UART_FR_DCD);

	if (delta & UART_FR_DSR)
		uap->port.icount.dsr++;

	if (delta & UART_FR_CTS)
		uart_handle_cts_change(&uap->port, status & UART_FR_CTS);

	wake_up_interruptible(&uap->port.state->port.delta_msr_wait); // name changed 2.6.33
}

static irqreturn_t epson12_serial_int(int irq, void *dev_id)
{
	struct uart_epson12_port *uap = dev_id;
	unsigned int status, pass_counter = EPSON12_ISR_PASS_LIMIT;
	int handled = 0;

	spin_lock(&uap->port.lock);

	status = readw(uap->port.membase + UART_MIS);
	if (status) {
		do {
			writew(status & ~(UART_TXIS|UART_RTIS|
					  UART_RXIS),
			       uap->port.membase + UART_ICR);

			if (status & (UART_RTIS|UART_RXIS))
				epson12_serial_rx_chars(uap);
			if (status & (UART_DSRMIS|UART_DCDMIS|
				      UART_CTSMIS|UART_RIMIS))
				epson12_serial_modem_status(uap);
			if (status & UART_TXIS)
				epson12_serial_tx_chars(uap);

			if (pass_counter-- == 0)
				break;

			status = readw(uap->port.membase + UART_MIS);
		} while (status != 0);
		handled = 1;
	}

	spin_unlock(&uap->port.lock);

	return IRQ_RETVAL(handled);
}

static unsigned int epson12_serial_tx_empty(struct uart_port *port)
{
	struct uart_epson12_port *uap = (struct uart_epson12_port *)port;
	unsigned int status = readw(uap->port.membase + UART_FR);
	IFDBG(!port,"%s: *port is NULL\n",__FUNCTION__);
	return status & (UART_FR_BUSY|UART_FR_TXFF) ? 0 : TIOCSER_TEMT;
}

static unsigned int epson12_serial_get_mctrl(struct uart_port *port)
{
	struct uart_epson12_port *uap = (struct uart_epson12_port *)port;
	unsigned int result = 0;
	unsigned int status = readw(uap->port.membase + UART_FR);

	IFDBG(!port,"%s: *port is NULL\n",__FUNCTION__);
#define EPSON12_BIT(uartbit, tiocmbit)	\
	if (status & uartbit)		\
		result |= tiocmbit

	EPSON12_BIT(UART_FR_DCD, TIOCM_CAR);
	EPSON12_BIT(UART_FR_DSR, TIOCM_DSR);
	EPSON12_BIT(UART_FR_CTS, TIOCM_CTS);
	EPSON12_BIT(UART_FR_RI, TIOCM_RNG);
#undef EPSON12_BIT
	return result;
}

static void epson12_serial_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_epson12_port *uap = (struct uart_epson12_port *)port;
	unsigned int cr;

	IFDBG(!port,"%s: *port is NULL\n",__FUNCTION__);
	cr = readw(uap->port.membase + UART_CR);

#define	EPSON12_BIT(tiocmbit, uartbit)	\
	if (mctrl & tiocmbit)		\
		cr |= uartbit;		\
	else				\
		cr &= ~uartbit

	EPSON12_BIT(TIOCM_RTS, UART_CR_RTS);
	EPSON12_BIT(TIOCM_DTR, UART_CR_DTR);
	EPSON12_BIT(TIOCM_OUT1, UART_CR_OUT1);
	EPSON12_BIT(TIOCM_OUT2, UART_CR_OUT2);
	EPSON12_BIT(TIOCM_LOOP, UART_CR_LBE);
#undef BIT

	writew(cr, uap->port.membase + UART_CR);
}

static void epson12_serial_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_epson12_port *uap = (struct uart_epson12_port *)port;
	unsigned long flags;
	unsigned int lcr_h;

	IFDBG(!port,"%s: *port is NULL\n",__FUNCTION__);
	spin_lock_irqsave(&uap->port.lock, flags);
	lcr_h = readw(uap->port.membase + UART_LCRH);
	if (break_state == -1)
		lcr_h |= UART_LCRH_BRK;
	else
		lcr_h &= ~UART_LCRH_BRK;
	writew(lcr_h, uap->port.membase + UART_LCRH);
	spin_unlock_irqrestore(&uap->port.lock, flags);
}

static int epson12_serial_startup(struct uart_port *port)
{
	struct uart_epson12_port *uap = (struct uart_epson12_port *)port;
	unsigned int cr;
	int retval;

	/*
	 * Try to enable the clock producer.
	 */
	retval = clk_enable(uap->clk);
	if (retval)
		goto out;

	uap->port.uartclk = clk_get_rate(uap->clk);

	/*
	 * Allocate the IRQ
	 */
	retval = request_irq(uap->port.irq, epson12_serial_int, 0, "uart-epson12", uap);
	if (retval){
		DBG("%s: request_irq error ret=%x\n",__FUNCTION__,retval);
		goto clk_dis;
	}

	writew(UART_IFLS_RX4_8|UART_IFLS_TX4_8,			// Interrupt fifo level select
	       uap->port.membase + UART_IFLS);			// RX/TX FIFO becomes >= 1/2 full
	/*
	 * Provoke TX FIFO interrupt into asserting.
	 */
	cr = UART_CR_UARTEN | UART_CR_TXE | UART_CR_LBE;	// UART / Tx / Loop back enable
	writew(cr, uap->port.membase + UART_CR);
	writew(0, uap->port.membase + UART_FBRD);	
	writew(1, uap->port.membase + UART_IBRD);
	writew(0, uap->port.membase + UART_LCRH);
	writew(0, uap->port.membase + UART_DR);
	while (readw(uap->port.membase + UART_FR) & UART_FR_BUSY)
		barrier();

	cr = UART_CR_UARTEN | UART_CR_RXE | UART_CR_TXE;	// UART / Tx / Rx enable
	writew(cr, uap->port.membase + UART_CR);

	/*
	 * initialise the old status of the modem signals
	 */
	uap->old_status = readw(uap->port.membase + UART_FR) & UART_FR_MODEM_ANY;

	/*
	 * Finally, enable interrupts
	 */
	spin_lock_irq(&uap->port.lock);
	uap->im = UART_RXIM | UART_RTIM;
	writew(uap->im, uap->port.membase + UART_IMSC);
	spin_unlock_irq(&uap->port.lock);

	return 0;

 clk_dis:
	clk_disable(uap->clk);
 out:
	return retval;
}

static void epson12_serial_shutdown(struct uart_port *port)
{
	struct uart_epson12_port *uap = (struct uart_epson12_port *)port;
	unsigned long val;

	/*
	 * disable all interrupts
	 */
	spin_lock_irq(&uap->port.lock);
	uap->im = 0;
	writew(uap->im, uap->port.membase + UART_IMSC);
	writew(0xffff, uap->port.membase + UART_ICR);
	spin_unlock_irq(&uap->port.lock);

	/*
	 * Free the interrupt
	 */
	free_irq(uap->port.irq, uap);

	/*
	 * disable the port
	 */
	writew(UART_CR_UARTEN | UART_CR_TXE, uap->port.membase + UART_CR);

	/*
	 * disable break condition and fifos
	 */
	val = readw(uap->port.membase + UART_LCRH);
	val &= ~(UART_LCRH_BRK | UART_LCRH_FEN);
	writew(val, uap->port.membase + UART_LCRH);

	/*
	 * Shut down the clock producer
	 */
	clk_disable(uap->clk);
	DBG("%s: serial shutdown\n",__FUNCTION__);
}

static void
epson12_serial_set_termios(struct uart_port *port, struct ktermios *termios,
		     struct ktermios *old)
{
	unsigned int lcr_h, old_cr;
	unsigned long flags;
	unsigned int baud, quot;

	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/16);
	quot = port->uartclk * 4 / baud;

	switch (termios->c_cflag & CSIZE) {		// word length
	case CS5:
		lcr_h = UART_LCRH_WLEN_5;		// 5bits
		break;
	case CS6:
		lcr_h = UART_LCRH_WLEN_6;		// 6bits
		break;
	case CS7:
		lcr_h = UART_LCRH_WLEN_7;		// 7bits
		break;
	default: // CS8
		lcr_h = UART_LCRH_WLEN_8;		// 8bits
		break;
	}
	if (termios->c_cflag & CSTOPB)
		lcr_h |= UART_LCRH_STP2;		// Two stop bits select
	if (termios->c_cflag & PARENB) {
		lcr_h |= UART_LCRH_PEN;			// Parity enable
		if (!(termios->c_cflag & PARODD))
			lcr_h |= UART_LCRH_EPS;		// Even parity select
	}
	if (port->fifosize > 1)
		lcr_h |= UART_LCRH_FEN;			// enable FIFOs 

	spin_lock_irqsave(&port->lock, flags);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	port->read_status_mask = UART_DR_OE | 255;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= UART_DR_FE | UART_DR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= UART_DR_BE;

	/*
	 * Characters to ignore
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= UART_DR_FE | UART_DR_PE;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= UART_DR_BE;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= UART_DR_OE;
	}

	/*
	 * Ignore all characters if CREAD is not set.
	 */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= UART_DUMMY_DR_RX;

	if (UART_ENABLE_MS(port, termios->c_cflag))
		epson12_serial_enable_ms(port);

	/* first, disable everything */
	old_cr = readw(port->membase + UART_CR);
	writew(0, port->membase + UART_CR);

	/* Set baud rate */
	writew(quot & 0x3f, port->membase + UART_FBRD);
	writew(quot >> 6, port->membase + UART_IBRD);

	/*
	 * ----------v----------v----------v----------v-----
	 * NOTE: MUST BE WRITTEN AFTER UARTLCR_M & UARTLCR_L
	 * ----------^----------^----------^----------^-----
	 */
	writew(lcr_h, port->membase + UART_LCRH);
	writew(old_cr, port->membase + UART_CR);

	spin_unlock_irqrestore(&port->lock, flags);
	DBG("%s: serial_set_termis\n", __FUNCTION__);
}

static const char *epson12_serial_type(struct uart_port *port)
{
	IFDBG(!port,"%s: *port is NULL\n",__FUNCTION__);
	return port->type == PORT_EPSON12 ? "EPSON12/PL011" : NULL;
}

/*
 * Release the memory region(s) being used by 'port'
 */
static void epson12_serial_release_port(struct uart_port *port)
{
	IFDBG(!port,"%s: *port is NULL\n",__FUNCTION__);
	release_mem_region(port->mapbase, SZ_4K);
}

/*
 * Request the memory region(s) being used by 'port'
 */
static int epson12_serial_request_port(struct uart_port *port)
{
	int temp;
	temp = (int)request_mem_region(port->mapbase, SZ_4K, "uart-pl011");
	IFDBG(!temp,"%s: request_mem_region return error\n",__FUNCTION__);
	return temp	!= 0  ? 0 : -EBUSY;
}

/*
 * Configure/autoconfigure the port.
 */
static void epson12_serial_config_port(struct uart_port *port, int flags)
{
	IFDBG(!port,"%s: *port is NULL\n",__FUNCTION__);
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_EPSON12;
		epson12_serial_request_port(port);
	}
}

/*
 * verify the new serial_struct (for TIOCSSERIAL).
 */
static int epson12_serial_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int ret = 0;
	IFDBG(!port,"%s: *port is NULL\n",__FUNCTION__);
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_EPSON12)
		ret = -EINVAL;
	if (ser->irq < 0 || ser->irq >= NR_IRQS)
		ret = -EINVAL;
	if (ser->baud_base < 9600)
		ret = -EINVAL;
	return ret;
}

static struct uart_ops epson12_serial_pops = {
	.tx_empty	= epson12_serial_tx_empty,
	.set_mctrl	= epson12_serial_set_mctrl,
	.get_mctrl	= epson12_serial_get_mctrl,
	.stop_tx	= epson12_serial_stop_tx,
	.start_tx	= epson12_serial_start_tx,
	.stop_rx	= epson12_serial_stop_rx,
	.enable_ms	= epson12_serial_enable_ms,
	.break_ctl	= epson12_serial_break_ctl,
	.startup	= epson12_serial_startup,
	.shutdown	= epson12_serial_shutdown,
	.set_termios	= epson12_serial_set_termios,
	.type		= epson12_serial_type,
	.release_port	= epson12_serial_release_port,
	.request_port	= epson12_serial_request_port,
	.config_port	= epson12_serial_config_port,
	.verify_port	= epson12_serial_verify_port,
};

static struct uart_epson12_port *serial_ports[UART_NR];

#ifdef CONFIG_SERIAL_EPSON12_CONSOLE

static void epson12_serial_console_putchar(struct uart_port *port, int ch)
{
	struct uart_epson12_port *uap = (struct uart_epson12_port *)port;

	while (readw(uap->port.membase + UART_FR) & UART_FR_TXFF)
		barrier();
	writew(ch, uap->port.membase + UART_DR);
}

static void
epson12_serial_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_epson12_port *uap = serial_ports[co->index];
	unsigned int status, old_cr, new_cr;

	if (console_enable) {
		IFDBG(!uap->clk,"%s: *clk is NULL\n",__FUNCTION__);
		clk_enable(uap->clk);

		/*
		 *	First save the CR then disable the interrupts
		 */
		old_cr = readw(uap->port.membase + UART_CR);
		new_cr = old_cr & ~UART_CR_CTSEN;
		new_cr |= UART_CR_UARTEN | UART_CR_TXE;
		writew(new_cr, uap->port.membase + UART_CR);

		uart_console_write(&uap->port, s, count, epson12_serial_console_putchar);

		/*
		 *	Finally, wait for transmitter to become empty
		 *	and restore the TCR
		 */
		do {
			status = readw(uap->port.membase + UART_FR);
		} while (status & UART_FR_BUSY);
		writew(old_cr, uap->port.membase + UART_CR);

		clk_disable(uap->clk);
	}
}

static void __init
epson12_serial_console_get_options(struct uart_epson12_port *uap, int *baud,
			     int *parity, int *bits)
{
	if (readw(uap->port.membase + UART_CR) & UART_CR_UARTEN) {
		unsigned int lcr_h, ibrd, fbrd;

		lcr_h = readw(uap->port.membase + UART_LCRH);

		*parity = 'n';
		if (lcr_h & UART_LCRH_PEN) {
			if (lcr_h & UART_LCRH_EPS)
				*parity = 'e';
			else
				*parity = 'o';
		}

		if ((lcr_h & 0x60) == UART_LCRH_WLEN_7)
			*bits = 7;
		else
			*bits = 8;

		ibrd = readw(uap->port.membase + UART_IBRD);
		fbrd = readw(uap->port.membase + UART_FBRD);

		*baud = uap->port.uartclk * 4 / (64 * ibrd + fbrd);
	}
}

static int __init epson12_serial_console_setup(struct console *co, char *options)
{
	struct uart_epson12_port *uap;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index >= UART_NR)
		co->index = 0;
	uap = serial_ports[co->index];
	if (!uap)
		return -ENODEV;

	IFDBG(!uap->clk,"%s: *clk is NULL\n",__FUNCTION__);

	uap->port.uartclk = clk_get_rate(uap->clk);

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		epson12_serial_console_get_options(uap, &baud, &parity, &bits);

	return uart_set_options(&uap->port, co, baud, parity, bits, flow);
}

static struct uart_driver serial_reg;
static struct console epson12_console = {
	.name		= "ttyAMA",
	.write		= epson12_serial_console_write,
	.device		= uart_console_device,
	.setup		= epson12_serial_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &serial_reg,
};

#define EPSON12_CONSOLE	(&epson12_console)
#else
#define EPSON12_CONSOLE	NULL
#endif

static struct uart_driver serial_reg = {
	.owner			= THIS_MODULE,
	.driver_name		= "ttyAMA",
	.dev_name		= "ttyAMA",
	.major			= SERIAL_EPSON12_MAJOR,
	.minor			= SERIAL_EPSON12_MINOR,
	.nr			= UART_NR,
	.cons			= EPSON12_CONSOLE,
};

static int epson12_serial_probe(struct platform_device *pdev)
{
	struct uart_epson12_port *uap;
	void __iomem *base;
	int i, ret=0;

	for (i = 0; i < ARRAY_SIZE(serial_ports); i++)
		if (serial_ports[i] == NULL)
			break;

	if (i == ARRAY_SIZE(serial_ports)) {
		ret = -EBUSY;
		goto out;
	}

	uap = kzalloc(sizeof(struct uart_epson12_port), GFP_KERNEL);
	if (uap == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	base = ioremap(pdev->resource->start, PAGE_SIZE);
	if (!base) {
		ret = -ENOMEM;
		goto free;
	}

	memset(uap, 0, sizeof(struct uart_epson12_port));
	uap->clk = clk_get(&pdev->dev, "UARTCLK");
	if (IS_ERR(uap->clk)) {
		ret = PTR_ERR(uap->clk);
		goto unmap;
	}

	uap->port.dev = &pdev->dev;
	uap->port.mapbase = pdev->resource->start;
	uap->port.membase = base;
	uap->port.iotype = UPIO_MEM;
	uap->port.irq = platform_get_irq(pdev, 0);
	if(uap->port.irq == 0){
		printk("%s: Couldn't get device irq\n", "EPSON12_serial");
		goto unmap;
	}
	uap->port.fifosize = 1;
	
	uap->port.ops = &epson12_serial_pops;
	uap->port.flags = UPF_BOOT_AUTOCONF;
	uap->port.line = i;

	serial_ports[i] = uap;

	dev_set_drvdata(&pdev->dev, uap);
	ret = uart_add_one_port(&serial_reg, &uap->port);
	if (ret) {
		dev_set_drvdata(&pdev->dev, NULL);
		serial_ports[i] = NULL;
		clk_put(uap->clk);
 unmap:
		iounmap(base);
 free:
		kfree(uap);
	}


 out:
	return ret;
}

static int epson12_serial_remove(struct platform_device *dev)
{
	struct uart_epson12_port *uap = platform_get_drvdata(dev);
	int i;

	dev_set_drvdata(&dev->dev, NULL);
	uart_remove_one_port(&serial_reg, &uap->port);

	for (i = 0; i < ARRAY_SIZE(serial_ports); i++)
		if (serial_ports[i] == uap)
			serial_ports[i] = NULL;

	iounmap(uap->port.membase);
	clk_put(uap->clk);
	kfree(uap);
	return 0;
}

static struct platform_driver epson12_serial_driver = {
	.probe		= epson12_serial_probe,
	.remove		= epson12_serial_remove,
	.driver		= {
		.name	= "uart"
	},
};

static int __init epson12_serial_init(void)
{
	int ret;
	struct proc_dir_entry *entry;
	
	printk(KERN_INFO "Serial: EPSON12 UART driver\n");

        entry = create_proc_entry(entry_name, S_IFREG | S_IRUGO | S_IWUGO, NULL);
	if (entry != NULL) {
		entry->read_proc = epson12_console_ctrl_read;
		entry->write_proc = epson12_console_ctrl_write;
		
		ret = uart_register_driver(&serial_reg);
		if (ret == 0) {
			ret = platform_driver_register(&epson12_serial_driver);
			if (ret)
				uart_unregister_driver(&serial_reg);
		}
	} else {
		ret = -ENOMEM;
	}

	return ret;
}

static void __exit epson12_serial_exit(void)
{
	platform_driver_unregister(&epson12_serial_driver);
	uart_unregister_driver(&serial_reg);
	remove_proc_entry(entry_name, NULL);
}

static int epson12_console_ctrl_read(
	char *page, char **start, off_t off, int count, int *eof, void *data) {
	
        return sprintf(page, "%d", console_enable);
}

static int epson12_console_ctrl_write(
	struct file *file, const char __user *buffer, unsigned long count, void *data) {

        console_enable = *buffer - 0x30;
        return count;
}

module_init(epson12_serial_init);
module_exit(epson12_serial_exit);

MODULE_AUTHOR("ARM Ltd/Deep Blue Solutions Ltd");
MODULE_DESCRIPTION("EPSON12 serial port driver");
MODULE_LICENSE("GPL");
