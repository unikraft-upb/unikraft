/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Author(s): Marc Rittinghaus <marc.rittinghaus@kit.edu>
 *
 * Copyright (c) 2021, Karlsruhe Institute of Technology. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <uk/plat/dbg.h>
#include <uk/config.h>
#include <x86/cpu.h>

/* TODO: This is a copy from serial_console configured for COM2 and a
 * temporary solution until we have configurable serial devices. We initialize
 * the interface inline so we do not have to integrate this temporary
 * implementation into the platform any further.
 */

#define COM2 0x2f8

#define COM2_DATA (COM2 + 0)
#define COM2_INTR (COM2 + 1)
#define COM2_CTRL (COM2 + 3)
#define COM2_STATUS (COM2 + 5)

/* only when DLAB is set */
#define COM2_DIV_LO (COM2 + 0)
#define COM2_DIV_HI (COM2 + 1)

/* baudrate divisor */
#define COM2_BAUDDIV_HI 0x00

#if CONFIG_KVM_SERIAL_BAUD_19200
#define COM2_BAUDDIV_LO 0x04
#elif CONFIG_KVM_SERIAL_BAUD_38400
#define COM2_BAUDDIV_LO 0x03
#elif CONFIG_KVM_SERIAL_BAUD_57600
#define COM2_BAUDDIV_LO 0x02
#else /* default, CONFIG_KVM_SERIAL_BAUD_115200 */
#define COM2_BAUDDIV_LO 0x01
#endif

#define DLAB 0x80
#define PROT 0x03 /* 8N1 (8 bits, no parity, one stop bit) */

static void ensure_init_debug_console(void)
{
	static int debug_console_initialized = 0;
	if (debug_console_initialized) {
		return;
	}

	outb(COM2_INTR, 0x00);  /* Disable all interrupts */
	outb(COM2_CTRL, DLAB);  /* Enable DLAB (set baudrate divisor) */
	outb(COM2_DIV_LO, COM2_BAUDDIV_LO);/* Div (lo byte) */
	outb(COM2_DIV_HI, COM2_BAUDDIV_HI);/*     (hi byte) */
	outb(COM2_CTRL, PROT);  /* Set 8N1, clear DLAB */

	debug_console_initialized = 1;
}

static int debug_tx_empty(void)
{
	return inb(COM2_STATUS) & 0x20;
}

static void debug_putc(char a)
{
	while (!debug_tx_empty())
		;

	outb(COM2_DATA, a);
}

static int debug_rx_ready(void)
{
	return inb(COM2_STATUS) & 0x01;
}

static int debug_getc(void)
{
	if (!debug_rx_ready())
		return -1;

	return (int) inb(COM2_DATA);
}

int ukplat_dbg_getc(void)
{
	int r;

	ensure_init_debug_console();

	while ((r = debug_getc()) < 0)
		;

	return r;
}

int ukplat_dbg_putc(char c)
{
	ensure_init_debug_console();

	debug_putc(c);
	return 0;
}
