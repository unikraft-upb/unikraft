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
	GDB_REGS_X0,
	GDB_REGS_X1,
	GDB_REGS_X2,
	GDB_REGS_X3,
	GDB_REGS_X4,
	GDB_REGS_X5,
	GDB_REGS_X6,
	GDB_REGS_X7,
	GDB_REGS_X8,
	GDB_REGS_X9,
	GDB_REGS_X10,
	GDB_REGS_X11,
	GDB_REGS_X12,
	GDB_REGS_X13,
	GDB_REGS_X14,
	GDB_REGS_X15,
	GDB_REGS_X16,
	GDB_REGS_X17,
	GDB_REGS_X18,
	GDB_REGS_X19,
	GDB_REGS_X20,
	GDB_REGS_X21,
	GDB_REGS_X22,
	GDB_REGS_X23,
	GDB_REGS_X24,
	GDB_REGS_X25,
	GDB_REGS_X26,
	GDB_REGS_X27,
	GDB_REGS_X28,
	GDB_REGS_X29,
	GDB_REGS_FP = GDB_REGS_X29, /* Frame pointer */
	GDB_REGS_X30,
	GDB_REGS_LR = GDB_REGS_X30, /* Link register */

	GDB_REGS_PC,
	GDB_REGS_CPSR,

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

#define gdb_arch_set_ip(ip, regs) do { regs->pc = (ip); } while (0)

#endif /* __UKDEBUG_INTERNAL_ARCH_GDBSUP_H__ */
