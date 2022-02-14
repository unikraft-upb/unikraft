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

#include "gdbsup.h"
#include "../../../gdbstub.h"

#include <uk/arch/traps.h>
#include <uk/essentials.h>
#include <uk/assert.h>
#include <uk/nofault.h>
#include <uk/isr/string.h>

#include <errno.h>

#ifndef X86_EFLAGS_TF
#define X86_EFLAGS_TF (1 << 8)
#endif

/* We get here either via traps raised by the platform or via the direct call
 * in _uk_crash().
 */
static int gdb_arch_dbg_trap(int errnr, struct __regs *regs)
{
	int r;

	/* Unset trap flag, i.e., continue */
	regs->eflags &= ~X86_EFLAGS_TF;

	r = gdb_dbg_trap(errnr, regs);
	if (r < 0) {
		return r;
	} else if (r == GDB_DBG_STEP) { /* Single step */
		regs->eflags |= X86_EFLAGS_TF;
	}

	return UK_EVENT_HANDLED;
}

static int gdb_arch_debug_handler(void *data)
{
	struct ukarch_trap_ctx *ctx = (struct ukarch_trap_ctx*)data;
	return gdb_arch_dbg_trap(5 /* SIGTRAP */, ctx->regs);
}
UK_EVENT_HANDLER(UKARCH_TRAP_DEBUG, gdb_arch_debug_handler);

/* This table maps struct __regs to the gdb register file */
static struct {
	unsigned int offset;
	unsigned int length;
} gdb_register_map[] = {
	{__REGS_OFFSETOF_RAX, 8},
	{__REGS_OFFSETOF_RBX, 8},
	{__REGS_OFFSETOF_RCX, 8},
	{__REGS_OFFSETOF_RDX, 8},
	{__REGS_OFFSETOF_RSI, 8},
	{__REGS_OFFSETOF_RDI, 8},
	{__REGS_OFFSETOF_RBP, 8},
	{__REGS_OFFSETOF_RSP, 8},
	{__REGS_OFFSETOF_R8, 8},
	{__REGS_OFFSETOF_R9, 8},
	{__REGS_OFFSETOF_R10, 8},
	{__REGS_OFFSETOF_R11, 8},
	{__REGS_OFFSETOF_R12, 8},
	{__REGS_OFFSETOF_R13, 8},
	{__REGS_OFFSETOF_R14, 8},
	{__REGS_OFFSETOF_R15, 8},

	{__REGS_OFFSETOF_RIP, 8},
	{__REGS_OFFSETOF_EFLAGS, 4},

	{__REGS_OFFSETOF_CS, 4},
	{__REGS_OFFSETOF_SS, 4}
};

#define GDB_REGISTER_MAP_NUM (int)(sizeof(gdb_register_map) / \
		sizeof(gdb_register_map[0]))

__ssz gdb_arch_read_register(int regnr, struct __regs *regs,
		void *buf, __sz buf_len __maybe_unused)
{
	if (regnr < GDB_REGISTER_MAP_NUM) {
		UK_ASSERT(buf_len >= gdb_register_map[regnr].length);

		memcpy_isr(buf,
			   (char*)regs + gdb_register_map[regnr].offset,
			   gdb_register_map[regnr].length);

		return gdb_register_map[regnr].length;
	}

	/* TODO: Implement getting the following registers */

	switch (regnr) {
	case GDB_REGS_DS:
	case GDB_REGS_ES:
	case GDB_REGS_FS:
	case GDB_REGS_GS:
		UK_ASSERT(buf_len >= 4);
		memset_isr(buf, 0, 4);
		return 4;

	case GDB_REGS_FS_BASE:
	case GDB_REGS_GS_BASE:
	case GDB_REGS_KGS_BASE:
	case GDB_REGS_CR0:
	case GDB_REGS_CR2:
	case GDB_REGS_CR3:
	case GDB_REGS_CR4:
	case GDB_REGS_CR8:
	case GDB_REGS_EFER:
	case GDB_REGS_STAR:
	case GDB_REGS_LSTAR:
	case GDB_REGS_FMASK:
		UK_ASSERT(buf_len >= 8);
		memset_isr(buf, 0, 8);
		return 8;

	case GDB_REGS_ST0:
	case GDB_REGS_ST1:
	case GDB_REGS_ST2:
	case GDB_REGS_ST3:
	case GDB_REGS_ST4:
	case GDB_REGS_ST5:
	case GDB_REGS_ST6:
	case GDB_REGS_ST7:
		UK_ASSERT(buf_len >= 10);
		memset_isr(buf, 0, 10);
		return 10;

	case GDB_REGS_FCTRL:
	case GDB_REGS_FSTAT:
	case GDB_REGS_FTAG:
	case GDB_REGS_FI_SEG:
	case GDB_REGS_FI_OFF:
	case GDB_REGS_FO_SEG:
	case GDB_REGS_FO_OFF:
	case GDB_REGS_FOP:
		UK_ASSERT(buf_len >= 4);
		memset_isr(buf, 0, 4);
		return 4;
	}

	return -EINVAL;
}

__ssz gdb_arch_write_register(int regnr, struct __regs *regs,
		void *buf, __sz buf_len)
{
	if (regnr < GDB_REGISTER_MAP_NUM) {
		if (buf_len < gdb_register_map[regnr].length) {
			return -EINVAL;
		}

		memcpy_isr((char*)regs + gdb_register_map[regnr].offset,
			   buf, gdb_register_map[regnr].length);

		return gdb_register_map[regnr].length;
	}

	/* TODO: Implement setting the following registers */

	switch (regnr) {
	case GDB_REGS_DS:
	case GDB_REGS_ES:
	case GDB_REGS_FS:
	case GDB_REGS_GS:
		return 0;

	case GDB_REGS_FS_BASE:
	case GDB_REGS_GS_BASE:
	case GDB_REGS_KGS_BASE:
	case GDB_REGS_CR0:
	case GDB_REGS_CR2:
	case GDB_REGS_CR3:
	case GDB_REGS_CR4:
	case GDB_REGS_CR8:
	case GDB_REGS_EFER:
	case GDB_REGS_STAR:
	case GDB_REGS_LSTAR:
	case GDB_REGS_FMASK:
		return 0;

	case GDB_REGS_ST0:
	case GDB_REGS_ST1:
	case GDB_REGS_ST2:
	case GDB_REGS_ST3:
	case GDB_REGS_ST4:
	case GDB_REGS_ST5:
	case GDB_REGS_ST6:
	case GDB_REGS_ST7:
		return 0;

	case GDB_REGS_FCTRL:
	case GDB_REGS_FSTAT:
	case GDB_REGS_FTAG:
	case GDB_REGS_FI_SEG:
	case GDB_REGS_FI_OFF:
	case GDB_REGS_FO_SEG:
	case GDB_REGS_FO_OFF:
	case GDB_REGS_FOP:
		return 0;
	}

	return -EINVAL;
}

__ssz gdb_arch_read_memory(unsigned long addr, __sz len,
		void *buf, __sz buf_len)
{
	return uk_memcpy_nofault_isr(buf, (void*)addr, MIN(len, buf_len));
}

__ssz gdb_arch_write_memory(unsigned long addr, __sz len,
		void *buf, __sz buf_len)
{
	return uk_memcpy_nofault_isr((void*)addr, buf, MIN(len, buf_len));
}
