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

/* Copied from plat/common/include/arm/arm64/cpu.h
 * TODO: Should we extend the public CPU header to include all architecture
 * definitions that are currently only defined in the platform?
 */
#define SYSREG_READ(reg) \
({	__sz val; \
	__asm__ __volatile__("mrs %0, " __STRINGIFY(reg) \
			: "=r" (val)); \
	val; \
})

#define SYSREG_WRITE(reg, val) \
	__asm__ __volatile__("msr " __STRINGIFY(reg) ", %0" \
			: : "r" ((__sz)(val)))

#define MDSCR_EL1_SS		(1 << 0)
#define MDSCR_EL1_KDE		(1 << 13)

#define SPSR_EL1_SS		(1 << 21)
#define SPSR_EL1_D		(1 << 9)

#define ESR_EC_SHIFT		26
#define ESR_EC(x)		((x) << ESR_EC_SHIFT)
#define ESR_EC_MASK		0x00000000fc000000UL
#define ESR_EC_FROM(x)		(((x) & ESR_EC_MASK) >> ESR_EC_SHIFT)

#define ESR_EL1_EC_BRK64	0x3c
#define BRK_OPCODE		0xd4200000UL

static void gdb_arch_enable_single_step(struct __regs *regs)
{
	__sz mdscr = SYSREG_READ(mdscr_el1);

	mdscr |= MDSCR_EL1_SS;
	mdscr |= MDSCR_EL1_KDE;

	SYSREG_WRITE(mdscr_el1, mdscr);

	regs->pstate |= SPSR_EL1_SS;
}

static void gdb_arch_disable_single_step(struct __regs *regs)
{
	__sz mdscr = SYSREG_READ(mdscr_el1);

	mdscr &= ~MDSCR_EL1_SS;
	mdscr &= ~MDSCR_EL1_KDE;

	SYSREG_WRITE(mdscr_el1, mdscr);

	regs->pstate &= ~SPSR_EL1_SS;
}

/* We get here either via traps raised by the platform or via the direct call
 * in _uk_crash().
 */
static int gdb_arch_dbg_trap(int errnr, struct __regs *regs)
{
	int r;

	gdb_arch_disable_single_step(regs);

	r = gdb_dbg_trap(errnr, regs);
	if (r < 0) {
		return r;
	} else if (r == GDB_DBG_STEP) { /* Single step */
		gdb_arch_enable_single_step(regs);

		regs->pstate &= ~SPSR_EL1_D;
	}

	return UK_EVENT_HANDLED;
}

static int gdb_arch_is_brk(unsigned long pc)
{
	__u32 opcode;

	if (!uk_memcpy_nofault_isr(&opcode, (void*)pc, sizeof(opcode))) {
		return 0;
	}

	return ((opcode & BRK_OPCODE) == BRK_OPCODE);
}

static int gdb_arch_debug_handler(void *data)
{
	struct ukarch_trap_ctx *ctx = (struct ukarch_trap_ctx*)data;
	int r = gdb_arch_dbg_trap(5 /* SIGTRAP */, ctx->regs);

	/* If we return from an brk trap, we have to explicitely skip the
	 * corresponding brk instruction. Otherwise, we will not make
	 * any progress and break again. However, software breakpoints
	 * temporarily overwrite instructions with brk so that we are also
	 * returning from an brk trap in this case but must not skip any
	 * instructions as GDB will have restored the original instruction by
	 * now. If the current PC is still brk, then it is compiled in and
	 * must be skipped. Otherwise, just continue at the current PC.
	 */
	if ((ESR_EC_FROM(ctx->esr) == ESR_EL1_EC_BRK64) &&
	    (gdb_arch_is_brk(ctx->regs->pc))) {
		ctx->regs->pc += 4; /* instructions are all 4 bytes wide */
		ctx->regs->pstate &= ~SPSR_EL1_SS;
	}

	return r;
}
UK_EVENT_HANDLER(UKARCH_TRAP_DEBUG, gdb_arch_debug_handler);

/* This table maps struct __regs to the gdb register file */
static struct {
	unsigned int offset;
	unsigned int length;
} gdb_register_map[] = {
	{__REGS_OFFSETOF_LR, 8},
	{__REGS_OFFSETOF_SP, 8},
	{__REGS_OFFSETOF_PC, 8},
	{__REGS_OFFSETOF_PSTATE, 8}
};

#define GDB_REGISTER_MAP_NUM (int)(sizeof(gdb_register_map) / \
		sizeof(gdb_register_map[0]))

__ssz gdb_arch_read_register(int regnr, struct __regs *regs,
		void *buf, __sz buf_len __maybe_unused)
{
	if (regnr < 30) {
		UK_ASSERT(buf_len >= sizeof(unsigned long));
		*((unsigned long*)buf) = regs->x[regnr];

		return sizeof(unsigned long);
	}

	regnr -= 30;

	if (regnr < GDB_REGISTER_MAP_NUM) {
		UK_ASSERT(buf_len >= gdb_register_map[regnr].length);

		memcpy_isr(buf,
			   (char*)regs + gdb_register_map[regnr].offset,
			   gdb_register_map[regnr].length);

		return gdb_register_map[regnr].length;
	}

	return -EINVAL;
}

__ssz gdb_arch_write_register(int regnr, struct __regs *regs,
		void *buf, __sz buf_len)
{
	if (regnr < 30) {
		if (buf_len < sizeof(unsigned long)) {
			return -EINVAL;
		}

		regs->x[regnr] = *((unsigned long*)buf);

		return sizeof(unsigned long);
	}

	regnr -= 30;

	if (regnr < GDB_REGISTER_MAP_NUM) {
		if (buf_len < gdb_register_map[regnr].length) {
			return -EINVAL;
		}

		memcpy_isr((char*)regs + gdb_register_map[regnr].offset,
			   buf, gdb_register_map[regnr].length);

		return gdb_register_map[regnr].length;
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
