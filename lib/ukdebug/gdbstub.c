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

#include "gdbstub.h"
#include <gdbsup.h>
#include <uk/plat/dbg.h>
#include <uk/plat/lcpu.h>

#include <uk/assert.h>
#include <uk/essentials.h>
#include <uk/arch/types.h>
#include <uk/arch/limits.h>
#include <uk/isr/string.h>

#include <stddef.h>
#include <errno.h>

struct gdb_excpt_ctx {
	struct __regs *regs;
	int errnr;
};

#define GDB_BUF_SIZE 2048
#define GDB_BUF_SIZE_HEX_STR "800"
static char gdb_recv_buffer[GDB_BUF_SIZE];
static char gdb_send_buffer[GDB_BUF_SIZE];

#define GDB_PACKET_RETRIES 5

#define GDB_STR_A_LEN(str) str, sizeof(str) - 1
#define GDB_CHECK(expr) \
	do { \
		__ssz __r = (expr); \
		if (__r < 0) { return (int)__r; } \
	} while (0)

typedef int (*gdb_cmd_handler_func)(char *buf, __sz buf_len,
	struct gdb_excpt_ctx *g);

struct gdb_cmd_table_entry {
	gdb_cmd_handler_func f;
	const char *cmd;
	__sz cmd_len;
};

/* Linked to architecture-specfic target.xml description */
extern const char __gdb_target_xml_start[];
extern const char __gdb_target_xml_end[];

static int gdb_starts_with(const char *buf, __sz buf_len,
		const char *prefix, __sz prefix_len)
{
	if (buf_len < prefix_len) {
		return 0;
	}

	return (memcmp_isr(buf, prefix, prefix_len) == 0);
}

static int gdb_consume_str(char **buf, __sz *buf_len,
		const char *prefix, __sz prefix_len)
{
	if (!gdb_starts_with(*buf, *buf_len, prefix, prefix_len)) {
		return 0;
	}

	*buf += prefix_len;
	*buf_len -= prefix_len;

	return 1;
}

static int gdb_is_equal(const char *buf1, __sz buf1_len,
		const char *buf2, __sz buf2_len)
{
	if (buf1_len != buf2_len) {
		return 0;
	}

	return (memcmp_isr(buf1, buf2, buf1_len) == 0);
}

static __sz gdb_find_or_max(const char *buf, __sz buf_len, char c)
{
	__sz l;

	for (l = 0; l < buf_len; l++) {
		if (buf[l] == c) {
			return l;
		}
	}

	return buf_len;
}

static char gdb_checksum(const char *buf, __sz len)
{
	char c = 0;

	while (len--) {
		c += *buf++;
	}

	return c;
}

static char* gdb_byte2hex(char *hex, __sz hex_len, char b)
{
	static const char map_byte2hex[] = "0123456789abcdef";

	if (hex_len < 2) {
		return NULL;
	}

	*hex++ = map_byte2hex[(b & 0xf0) >> 4]; /* Encode high nibble */
	*hex++ = map_byte2hex[(b & 0x0f) >> 0]; /* Encode low nibble */

	return hex;
}

static __ssz gdb_mem2hex(char *hex, __sz hex_len, const char *mem, __sz mem_len)
{
	__sz l = mem_len;

	if (hex_len < mem_len * 2) {
		return -ENOMEM;
	}

	while (l--) {
		hex = gdb_byte2hex(hex, 2, *mem++);
	}

	return mem_len * 2;
}

static int gdb_hex2int(char hex)
{
	if ((hex >= '0') && (hex <= '9')) {
		return hex - '0';
	} else if ((hex >= 'a') && (hex <= 'f')) {
		return hex - 'a' + 0xa;
	} else if ((hex >= 'A') && (hex <= 'F')) {
		return hex - 'A' + 0xa;
	}

	return -EINVAL;
}

static unsigned long gdb_hex2ulong(const char *buf, __sz buf_len, char **endptr)
{
	unsigned long val = 0;
	int i;

	/* Skip any whitespace */
	while ((buf_len > 0) && (*buf == ' ')) {
		buf++;
		buf_len--;
	}

	/* Skip hex prefix if present */
	if ((buf_len >= 2) && (*buf == '0') &&
	    ((*(buf + 1) == 'x') || (*(buf + 1) == 'X'))) {
		buf += 2;
		buf_len -= 2;
	}

	/* Parse hexadecimal integer */
	while ((buf_len > 0) && (val < __UL_MAX)) {
		i = gdb_hex2int(*buf);
		if (i < 0) {
			break;
		}

		val <<= 4;
		val |= i;

		buf++;
		buf_len--;
	}

	if (endptr != NULL) {
		*endptr = DECONST(char *, buf);
	}

	return val;
}

static __ssz gdb_hex2mem(char *mem, __sz mem_len, const char *hex, __sz hex_len)
{
	__sz l = hex_len;
	int i;

	if (hex_len % 2 != 0) {
		return -EINVAL;
	}

	if (mem_len < hex_len / 2) {
		return -ENOMEM;
	}

	for (; l > 0; l -= 2, mem++) {
		/* Decode high nibble */
		i = gdb_hex2int(*hex++);
		if (i < 0) {
			return i;
		}

		*mem = i << 4;

		/* Decode low nibble */
		i = gdb_hex2int(*hex++);
		if (i < 0) {
			return i;
		}

		*mem |= i;
	}

	return hex_len / 2;
}

static __ssz gdb_mem2bin(char *bin, __sz bin_len, const char *mem, __sz mem_len)
{
	__sz rem_len = bin_len;

	while (mem_len--) {
		switch (*mem) {
		/* The following characters must be escaped */
		case '}':
		case '#':
		case '$':
		case '*':
			if (rem_len < 2) {
				return bin_len - rem_len;
			}
			*bin++ = '}';
			*bin++ = *mem++ ^ 0x20;

			rem_len -= 2;
			break;
		default:
			if (rem_len < 1) {
				return bin_len - rem_len;
			}
			*bin++ = *mem++;

			rem_len--;
			break;
		}
	}

	return bin_len - rem_len;
}

static __ssz gdb_send(const char *buf, __sz len)
{
	__sz l = len;

	while (l--) {
		GDB_CHECK(ukplat_dbg_putc(*buf++));
	}

	return len;
}

static __ssz gdb_recv(char *buf, __sz len)
{
	__sz l = len;
	int r;

	while (l--) {
		r = ukplat_dbg_getc();
		if (r < 0) {
			return r;
		}

		*buf++ = (char)r;
	}

	return len;
}

static int gdb_send_ack(void)
{
	return ukplat_dbg_putc('+');
}

static int gdb_send_nack(void)
{
	return ukplat_dbg_putc('-');
}

static int gdb_recv_ack(void)
{
	int r = ukplat_dbg_getc();

	return (r < 0) ? r : ((char)r == '+');
}

static __ssz gdb_send_packet(const char *buf, __sz len)
{
	char hex[2];
	char chksum = gdb_checksum(buf, len);
	int r, retries = 0;

	gdb_mem2hex(hex, sizeof(hex), &chksum, 1);

	/* GDB packet format: $<DATA>#<CC>
	 * where CC is the GDB packet checksum
	 */
	do {
		if (retries++ > GDB_PACKET_RETRIES) {
			return -1;
		}

		GDB_CHECK(ukplat_dbg_putc('$'));
		GDB_CHECK(gdb_send(buf, len));
		GDB_CHECK(ukplat_dbg_putc('#'));
		GDB_CHECK(gdb_send(hex, sizeof(hex)));
	} while ((r = gdb_recv_ack()) == 0);

	return (r == 1) ? (__ssz)len : r;
}

/* '' */
static __ssz gdb_send_empty_packet(void)
{
	return gdb_send_packet(NULL, 0);
}

/* S nn */
static __ssz gdb_send_signal_packet(int errnr)
{
	char buf[3];

	UK_ASSERT((errnr & 0xff) == errnr);

	buf[0] = 'S';
	gdb_byte2hex(buf + 1, sizeof(buf) - 1, errnr);

	return gdb_send_packet(buf, sizeof(buf));
}

/* O XX... */
static __ssz gdb_send_message_packet(const char *str, __sz len)
{
	UK_ASSERT(str != NULL);

	gdb_send_buffer[0] = 'O';
	len = gdb_mem2hex(gdb_send_buffer + 1, sizeof(gdb_send_buffer) - 1,
			  str, len);

	return gdb_send_packet(gdb_send_buffer, len + 1);
}

/* E nn */
static __ssz gdb_send_error_packet(int err)
{
	char buf[3];

	UK_ASSERT((err & 0xff) == err);

	buf[0] = 'E';
	gdb_byte2hex(buf + 1, sizeof(buf) - 1, err);

	return gdb_send_packet(buf, sizeof(buf));
}

static __ssz gdb_recv_packet(char *buf, __sz len)
{
	int c, retries = 0;
	char *p = buf;
	__sz n = 0;

	/* Wait for packet start character */
	while ((c = ukplat_dbg_getc()) != '$') {
		if (c < 0) {
			return c;
		}
	}

	while (n < len) {
		c = ukplat_dbg_getc();
		if (c < 0) {
			return c;
		} else if (c == '#') {
			char hex[2];
			char chksum = 0;

			GDB_CHECK(gdb_recv(hex, sizeof(hex)));

			if (gdb_hex2mem(&chksum, 1, hex, sizeof(hex)) < 0) {
				GDB_CHECK(gdb_send_nack());
				continue;
			}

			if (chksum != gdb_checksum(buf, n)) {
				GDB_CHECK(gdb_send_nack());
				continue;
			}

			GDB_CHECK(gdb_send_ack());

			/* Null-terminate the data */
			*p = 0;
			return n;
		} else if (c == '$') {
			if (retries++ > GDB_PACKET_RETRIES) {
				break;
			}

			/* We received a packet start character and maybe
			 * missed some characters on the way.
			 * Start all over again.
			 */
			p = buf;
			n = 0;
		} else {
			*p++ = c;
			n++;
		}
	}

	/* We ran out of space */
	return -ENOMEM;
}

/* ? */
static int gdb_handle_stop_reason(char *buf __unused,
		__sz buf_len __unused, struct gdb_excpt_ctx *g)
{
	GDB_CHECK(gdb_send_signal_packet(g->errnr));

	return 0;
}

/* g */
static int gdb_handle_read_registers(char *buf __unused,
		__sz buf_len __unused, struct gdb_excpt_ctx *g)
{
	char *tmp = gdb_recv_buffer; /* Use receive buffer as temporary space */
	__ssz r;
	int i;

	for (i = 0; i < GDB_REGS_NUM; i++) {
		r = gdb_arch_read_register(i, g->regs, tmp,
			sizeof(gdb_recv_buffer) - (tmp - gdb_recv_buffer));
		if (r < 0) {
			/* Send Enn. */
			GDB_CHECK(gdb_send_error_packet(-r));
			return 0;
		}

		tmp += r;
	}

	r = gdb_mem2hex(gdb_send_buffer, sizeof(gdb_send_buffer),
		gdb_recv_buffer, tmp - gdb_recv_buffer);

	UK_ASSERT(r >= 0);

	GDB_CHECK(gdb_send_packet(gdb_send_buffer, r));

	return 0;
}

/* G */
static int gdb_handle_write_registers(char *buf, __sz buf_len,
		struct gdb_excpt_ctx *g)
{
	char *tmp = gdb_send_buffer; /* Use send buffer as temporary space */
	__ssz r, l;
	int i;

	l = gdb_hex2mem(gdb_send_buffer, sizeof(gdb_send_buffer),
		buf, buf_len);
	if (l < 0) {
		/* Send Enn. */
		GDB_CHECK(gdb_send_error_packet(-l));
		return 0;
	}

	for (i = 0; i < GDB_REGS_NUM; i++) {
		r = gdb_arch_write_register(i, g->regs, tmp, l);
		if (r < 0) {
			/* Send Enn. */
			GDB_CHECK(gdb_send_error_packet(-r));
			return 0;
		}

		tmp += r;
		l -= r;
	}

	GDB_CHECK(gdb_send_packet(GDB_STR_A_LEN("OK")));

	return 0;
}

/* m addr,length*/
static int gdb_handle_read_memory(char *buf, __sz buf_len,
		struct gdb_excpt_ctx *g __unused)
{
	unsigned long addr, length;
	char *buf_end = buf + buf_len;
	__ssz r;

	addr = gdb_hex2ulong(buf, buf_end - buf, &buf);
	if ((buf == buf_end) || (*buf++ != ',')) {
		/* Send E22. EINVAL */
		GDB_CHECK(gdb_send_error_packet(EINVAL));
		return 0;
	}

	length = gdb_hex2ulong(buf, buf_end - buf, &buf);
	if (buf != buf_end) {
		/* Send E22. EINVAL */
		GDB_CHECK(gdb_send_error_packet(EINVAL));
		return 0;
	}

	r = gdb_arch_read_memory(addr, length, gdb_recv_buffer,
		sizeof(gdb_recv_buffer) / 2);
	if (r < 0) {
		/* Send Enn. */
		GDB_CHECK(gdb_send_error_packet(-r));
		return 0;
	}

	r = gdb_mem2hex(gdb_send_buffer, sizeof(gdb_send_buffer),
		gdb_recv_buffer, r);

	GDB_CHECK(gdb_send_packet(gdb_send_buffer, r));

	return 0;
}

/* M addr,length*/
static int gdb_handle_write_memory(char *buf, __sz buf_len,
		struct gdb_excpt_ctx *g __unused)
{
	unsigned long addr, length;
	char *buf_end = buf + buf_len;
	__ssz r;

	addr = gdb_hex2ulong(buf, buf_end - buf, &buf);
	if ((buf == buf_end) || (*buf++ != ',')) {
		/* Send E22. EINVAL */
		GDB_CHECK(gdb_send_error_packet(EINVAL));
		return 0;
	}

	length = gdb_hex2ulong(buf, buf_end - buf, &buf);
	if ((buf == buf_end) || (*buf++ != ':')) {
		/* Send E22. EINVAL */
		GDB_CHECK(gdb_send_error_packet(EINVAL));
		return 0;
	}

	r = gdb_hex2mem(gdb_send_buffer, sizeof(gdb_send_buffer),
		buf, buf_end - buf);
	if (r < 0) {
		/* Send Enn. */
		GDB_CHECK(gdb_send_error_packet(-r));
		return 0;
	}

	r = gdb_arch_write_memory(addr, length, gdb_send_buffer, r);
	if (r < 0) {
		/* Send Enn. */
		GDB_CHECK(gdb_send_error_packet(-r));
		return 0;
	}

	GDB_CHECK(gdb_send_packet(GDB_STR_A_LEN("OK")));

	return 0;
}

/* c/s [addr] */
static int gdb_handle_step_cont(char *buf, __sz buf_len,
		struct gdb_excpt_ctx *g)
{
	unsigned long addr;
	char *buf_end = buf + buf_len;

	if (buf != buf_end) {
		addr = gdb_hex2ulong(buf, buf_len, &buf);
		if (buf != buf_end) {
			/* Send E22. EINVAL */
			GDB_CHECK(gdb_send_error_packet(EINVAL));
			return 0;
		}

		gdb_arch_set_ip(addr, g->regs);
	}

	return 1;
}

/* C/S sig[;addr] */
static int gdb_handle_step_cont_sig(char *buf, __sz buf_len,
		struct gdb_excpt_ctx *g)
{
	unsigned long addr;
	char *buf_end = buf + buf_len;

	/* Just ignore the signal */
	gdb_hex2ulong(buf, buf_end - buf, &buf);
	if (buf != buf_end) {
		if (*buf++ != ';') {
			/* Send E22. EINVAL */
			GDB_CHECK(gdb_send_error_packet(EINVAL));
			return 0;
		}

		addr = gdb_hex2ulong(buf, buf_len, &buf);
		if (buf != buf_end) {
			/* Send E22. EINVAL */
			GDB_CHECK(gdb_send_error_packet(EINVAL));
			return 0;
		}

		gdb_arch_set_ip(addr, g->regs);
	}

	return 1;
}

/* c[addr] */
static int gdb_handle_continue(char *buf, __sz buf_len,
		struct gdb_excpt_ctx *g)
{
	int r;

	return (((r = gdb_handle_step_cont(buf, buf_len, g)) <= 0) ?
			r : GDB_DBG_CONT);
}

/* C sig[;addr] */
static int gdb_handle_continue_sig(char *buf, __sz buf_len,
		struct gdb_excpt_ctx *g)
{
	int r;

	return (((r = gdb_handle_step_cont_sig(buf, buf_len, g)) <= 0) ?
			r : GDB_DBG_CONT);
}

/* s[addr] */
static int gdb_handle_step(char *buf, __sz buf_len,
		struct gdb_excpt_ctx *g)
{
	int r;

	return (((r = gdb_handle_step_cont(buf, buf_len, g)) <= 0) ?
			r : GDB_DBG_STEP);
}

/* S sig[;addr] */
static int gdb_handle_step_sig(char *buf, __sz buf_len,
		struct gdb_excpt_ctx *g)
{
	int r;

	return (((r = gdb_handle_step_cont_sig(buf, buf_len, g)) <= 0) ?
			r : GDB_DBG_STEP);
}

static int gdb_handle_multiletter_cmd(struct gdb_cmd_table_entry *table,
		__sz table_entries, char *buf, __sz buf_len,
		struct gdb_excpt_ctx *g)
{
	__sz i, l, cmd_len = gdb_find_or_max(buf, buf_len, ':');

	for (i = 0; i < table_entries; i++) {
		l = table[i].cmd_len;

		if (!gdb_is_equal(buf, cmd_len, table[i].cmd, l)) {
			continue;
		}

		return table[i].f(buf + l + 1,
			buf_len - l - 1, g);
	}

	/* Send empty packet to signal GDB that
	 * we do not support the command
	 */
	GDB_CHECK(gdb_send_empty_packet());

	return 0;
}

/* qSupported[:gdbfeature [;gdbfeature]... ] */
static int gdb_handle_qsupported(char *buf __unused, __sz buf_len __unused,
		struct gdb_excpt_ctx *g __unused)
{
#define GDB_SUPPORT_STR \
	"PacketSize="GDB_BUF_SIZE_HEX_STR \
	";qXfer:features:read+"

	GDB_CHECK(gdb_send_packet(GDB_STR_A_LEN(GDB_SUPPORT_STR)));

	return 0;
}

static int gdb_qXfer_mem(const char *mem, __sz mem_len, __sz offset,
		__sz length)
{
	__ssz r;

	if (offset >= mem_len) {
		/* Send l. No more data to be read */
		GDB_CHECK(gdb_send_packet(GDB_STR_A_LEN("l")));
	} else {
		length = MIN(length, mem_len - offset);
		length = MIN(length, sizeof(gdb_send_buffer) - 1);

		gdb_send_buffer[0] = 'm'; /* There is more data */

		r = gdb_mem2bin(gdb_send_buffer + 1,
			sizeof(gdb_send_buffer) - 1,
			mem + offset, length);

		UK_ASSERT(r > 0);

		GDB_CHECK(gdb_send_packet(gdb_send_buffer, r + 1));
	}

	return 0;
}

/* qXfer:object:read:annex:offset,length */
static int gdb_handle_qXfer(char *buf, __sz buf_len,
		struct gdb_excpt_ctx *g __unused)
{
	unsigned long offset, length;
	char *buf_end = buf + buf_len;

	if (!gdb_consume_str(&buf, &buf_len, GDB_STR_A_LEN("features:read:"))) {
		/* Send empty packet. Unsupported object requested */
		GDB_CHECK(gdb_send_empty_packet());
		return 0;
	}

	if (!gdb_consume_str(&buf, &buf_len, GDB_STR_A_LEN("target.xml:"))) {
		/* Send E00. Annex invalid */
		GDB_CHECK(gdb_send_error_packet(0x00));
		return 0;
	}

	offset = gdb_hex2ulong(buf, buf_end - buf, &buf);
	if ((buf >= buf_end) || (*buf++ != ',')) {
		/* Send E00. Request malformed */
		GDB_CHECK(gdb_send_error_packet(0x00));
		return 0;
	}

	length = gdb_hex2ulong(buf, buf_end - buf, &buf);
	if (buf != buf_end) {
		/* Send E00. Request malformed */
		GDB_CHECK(gdb_send_error_packet(0x00));
		return 0;
	}

	return gdb_qXfer_mem(__gdb_target_xml_start,
		__gdb_target_xml_end - __gdb_target_xml_start,
		offset, length);
}

static struct gdb_cmd_table_entry gdb_q_cmd_table[] = {
	{ gdb_handle_qsupported, GDB_STR_A_LEN("Supported") },
	{ gdb_handle_qXfer, GDB_STR_A_LEN("Xfer") }
};

#define NUM_GDB_Q_CMDS (sizeof(gdb_q_cmd_table) / \
		sizeof(struct gdb_cmd_table_entry))

static int gdb_handle_q_cmd(char *buf, __sz buf_len,
		struct gdb_excpt_ctx *g)
{
	return gdb_handle_multiletter_cmd(gdb_q_cmd_table, NUM_GDB_Q_CMDS,
		buf, buf_len, g);
}

/* vCont? */
static int gdb_handle_vCont_ask(char *buf __unused, __sz buf_len __unused,
		struct gdb_excpt_ctx *g __unused)
{
	GDB_CHECK(gdb_send_empty_packet()); /* not supported yet */

	return 0;
}

static struct gdb_cmd_table_entry gdb_v_cmd_table[] = {
	{ gdb_handle_vCont_ask, GDB_STR_A_LEN("Cont?") }
};

#define NUM_GDB_V_CMDS (sizeof(gdb_v_cmd_table) / \
		sizeof(struct gdb_cmd_table_entry))

static int gdb_handle_v_cmd(char *buf, __sz buf_len,
		struct gdb_excpt_ctx *g)
{
	return gdb_handle_multiletter_cmd(gdb_v_cmd_table, NUM_GDB_V_CMDS,
		buf, buf_len, g);
}

static struct gdb_cmd_table_entry gdb_cmd_table[] = {
	{ gdb_handle_stop_reason, GDB_STR_A_LEN("?") },
	{ gdb_handle_read_registers, GDB_STR_A_LEN("g") },
	{ gdb_handle_write_registers, GDB_STR_A_LEN("G") },
	{ gdb_handle_read_memory, GDB_STR_A_LEN("m") },
	{ gdb_handle_write_memory, GDB_STR_A_LEN("M") },
	{ gdb_handle_continue, GDB_STR_A_LEN("c") },
	{ gdb_handle_continue_sig, GDB_STR_A_LEN("C") },
	{ gdb_handle_step, GDB_STR_A_LEN("s") },
	{ gdb_handle_step_sig, GDB_STR_A_LEN("S") },
	{ gdb_handle_q_cmd, GDB_STR_A_LEN("q") },
	{ gdb_handle_v_cmd, GDB_STR_A_LEN("v") }
};

#define NUM_GDB_CMDS (sizeof(gdb_cmd_table) / \
		sizeof(struct gdb_cmd_table_entry))

static int gdb_main_loop(struct gdb_excpt_ctx *g)
{
	__ssz r;
	__sz i, l;

	GDB_CHECK(gdb_send_signal_packet(g->errnr));

	do {
		r = gdb_recv_packet(gdb_recv_buffer, sizeof(gdb_recv_buffer));
		if (r < 0) {
			break;
		} else if (r == 0) {
			/* We received an empty packet */
			continue;
		}

		for (i = 0; i < NUM_GDB_CMDS; i++) {
			l = gdb_cmd_table[i].cmd_len;

			if (!gdb_starts_with(gdb_recv_buffer, r,
				gdb_cmd_table[i].cmd, l)) {
				continue;
			}

			r = gdb_cmd_table[i].f(gdb_recv_buffer + l, r - l,
				g);

			break;
		}

		if (i == NUM_GDB_CMDS) {
			/* Send empty packet to signal GDB that
			 * we do not support the command
			 */
			GDB_CHECK(gdb_send_empty_packet());

			r = 0;	/* Ignore unsupported commands */
		}
	} while (r == 0);

	UK_ASSERT(r < 0 || r == GDB_DBG_CONT || r == GDB_DBG_STEP);

	return (int)r;
}

int gdb_dbg_trap(int errnr, struct __regs *regs)
{
	struct gdb_excpt_ctx g = {regs, errnr};
	static int nest_cnt = 0;
	unsigned long irqs;
	int r;

	/* TODO: SMP support
	 * If we have SMP support, we must freeze all other CPUs here and
	 * resume them before returning. However, it might be that another
	 * CPU tries to enter the debugger at the same time. We therefore
	 * need to protect the debugger with a spinlock and try to lock it.
	 * If the current CPU does not get the lock, a different CPU is
	 * already in the debugger. The unsuccessful CPU has to wait for the
	 * lock to become available, but also remain responsive to the freeze
	 * that the first CPU initiates.
	 *
	 * Until we have SMP support, it is safe to just disable interrupts.
	 * We might re-enter nevertheless, for example, due to an UK_ASSERT
	 * in the debugger code itself or if we set a breakpoint in code
	 * called by the debugger.
	 */
#ifdef UKPLAT_LCPU_MULTICORE
#warning The GDB debugger stub does not support multicore systems
#endif

	irqs = ukplat_lcpu_save_irqf();
	if (nest_cnt > 0) {
		ukplat_lcpu_restore_irqf(irqs);
		return GDB_DBG_CONT;
	}

	nest_cnt++;
	r = gdb_main_loop(&g);
	nest_cnt--;

	ukplat_lcpu_restore_irqf(irqs);

	return r;
}

void gdb_dbg_print(const char *str, __sz len)
{
	unsigned long irqs;

	if (str) {
		/* TODO: SMP Support */
		irqs = ukplat_lcpu_save_irqf();
		gdb_send_message_packet(str, len);
		ukplat_lcpu_restore_irqf(irqs);
	}
}
