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

#ifndef __UKDEBUG_INTERNAL_ARCH_GDBSUP_H__
#define __UKDEBUG_INTERNAL_ARCH_GDBSUP_H__

#include <uk/arch/types.h>
#include <uk/arch/lcpu.h>

/* The following list must match the description in target.xml */
enum gdb_arch_register_index {

	/* General registers */
	GDB_REGS_RAX,
	GDB_REGS_RBX,
	GDB_REGS_RCX,
	GDB_REGS_RDX,
	GDB_REGS_RSI,
	GDB_REGS_RDI,
	GDB_REGS_RBP,
	GDB_REGS_RSP,
	GDB_REGS_R8,
	GDB_REGS_R9,
	GDB_REGS_R10,
	GDB_REGS_R11,
	GDB_REGS_R12,
	GDB_REGS_R13,
	GDB_REGS_R14,
	GDB_REGS_R15,

	GDB_REGS_RIP,
	GDB_REGS_EFLAGS,

	/* Segment registers */
	GDB_REGS_CS,
	GDB_REGS_SS,
	GDB_REGS_DS,
	GDB_REGS_ES,
	GDB_REGS_FS,
	GDB_REGS_GS,

	GDB_REGS_FS_BASE,
	GDB_REGS_GS_BASE,
	GDB_REGS_KGS_BASE,

	/* Control registers */
	GDB_REGS_CR0,
	GDB_REGS_CR2,
	GDB_REGS_CR3,
	GDB_REGS_CR4,
	GDB_REGS_CR8,

	GDB_REGS_EFER,
	GDB_REGS_STAR,
	GDB_REGS_LSTAR,
	GDB_REGS_FMASK,

	/* x87 FPU */
	GDB_REGS_ST0,
	GDB_REGS_ST1,
	GDB_REGS_ST2,
	GDB_REGS_ST3,
	GDB_REGS_ST4,
	GDB_REGS_ST5,
	GDB_REGS_ST6,
	GDB_REGS_ST7,

	GDB_REGS_FCTRL,
	GDB_REGS_FSTAT,
	GDB_REGS_FTAG,
	GDB_REGS_FI_SEG,
	GDB_REGS_FI_OFF,
	GDB_REGS_FO_SEG,
	GDB_REGS_FO_OFF,
	GDB_REGS_FOP,

	GDB_REGS_NUM
};

__ssz gdb_arch_read_register(int regnr, struct __regs *regs,
		void *buf, __sz buf_len);
__ssz gdb_arch_write_register(int regnr, struct __regs *regs,
		void *buf, __sz buf_len);
__ssz gdb_arch_read_memory(unsigned long addr, __sz len,
		void *buf, __sz buf_len);
__ssz gdb_arch_write_memory(unsigned long addr, __sz len,
		void *buf, __sz buf_len);

#define gdb_arch_set_ip(ip, regs) do { regs->rip = (ip); } while (0)

#endif /* __UKDEBUG_INTERNAL_ARCH_GDBSUP_H__ */
