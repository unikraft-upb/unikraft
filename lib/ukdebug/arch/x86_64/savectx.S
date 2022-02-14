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

#include <uk/arch/lcpu.h>
#include <uk/asm.h>

/**
 * The function saves a register snapshot in the supplied struct __regs*.
 * It expects rdi to be already saved in regs->rdi.
 *
 * @param regs<rdi>
 *   Pointer to struct _regs.
 */
.section .text
ENTRY(_uk_crash_do_save_ctx)
	/* Save general-purpose registers */
	mov %rsi, __REGS_OFFSETOF_RSI(%rdi)
	mov %rdx, __REGS_OFFSETOF_RDX(%rdi)
	mov %rcx, __REGS_OFFSETOF_RCX(%rdi)
	mov %rax, __REGS_OFFSETOF_RAX(%rdi)
	mov %r8, __REGS_OFFSETOF_R8(%rdi)
	mov %r9, __REGS_OFFSETOF_R9(%rdi)
	mov %r10, __REGS_OFFSETOF_R10(%rdi)
	mov %r11, __REGS_OFFSETOF_R11(%rdi)
	mov %rbx, __REGS_OFFSETOF_RBX(%rdi)
	mov %rbp, __REGS_OFFSETOF_RBP(%rdi)
	mov %r12, __REGS_OFFSETOF_R12(%rdi)
	mov %r13, __REGS_OFFSETOF_R13(%rdi)
	mov %r14, __REGS_OFFSETOF_R14(%rdi)
	mov %r15, __REGS_OFFSETOF_R15(%rdi)

	/* Save EFLAGS */
	pushf
	pop %rax
	mov %rax, __REGS_OFFSETOF_EFLAGS(%rdi)

	/* Clear extra fields and use the cleared RAX to zero-extend and save
	 * the segment registers
	 */
	xor %rax, %rax
	mov %rax, __REGS_OFFSETOF_ORIG_RAX(%rdi)
	mov %rax, __REGS_OFFSETOF_PAD(%rdi)

	movw %cs, %ax
	mov %rax, __REGS_OFFSETOF_CS(%rdi)

	movw %ss, %ax
	mov %rax, __REGS_OFFSETOF_SS(%rdi)

	/* We have the return address at the current RSP. We save it as IP */
	mov 0(%rsp), %rax
	mov %rax, __REGS_OFFSETOF_RIP(%rdi)

	/* Take the current stack pointer but without the return address */
	mov %rsp, %rax
	add $8, %rax
	mov %rax, __REGS_OFFSETOF_RSP(%rdi)

	ret
ENDPROC(_uk_crash_do_save_ctx)
