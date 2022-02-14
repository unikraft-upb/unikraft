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

#include "../../crashdump.h"
#include "../../outf.h"

#include <uk/nofault.h>

void cdmp_arch_print_registers(struct out_dev *o, struct __regs *regs)
{
	outf(o, "RIP: %04lx:%016lx\n",
		regs->cs & 0xffff, regs->rip);
	outf(o, "RSP: %04lx:%016lx EFLAGS: %08lx ORIG_RAX: %016lx\n",
		regs->ss & 0xffff, regs->rsp, regs->eflags & 0xffffffff,
		regs->orig_rax);
	outf(o, "RAX: %016lx RBX: %016lx RCX:%016lx\n",
		regs->rax, regs->rbx, regs->rcx);
	outf(o, "RDX: %016lx RSI: %016lx RDI:%016lx\n",
		regs->rdx, regs->rsi, regs->rdi);
	outf(o, "RBP: %016lx R08: %016lx R09:%016lx\n",
		regs->rbp, regs->r8, regs->r9);
	outf(o, "R10: %016lx R11: %016lx R12:%016lx\n",
		regs->r10, regs->r11, regs->r12);
	outf(o, "R13: %016lx R14: %016lx R15:%016lx\n",
		regs->r13, regs->r14, regs->r15);
}

void cdmp_arch_print_stack(struct out_dev *o, struct __regs *regs)
{
	/* Nothing special to be done. Just call the generic version */
	cdmp_gen_print_stack(o, regs->rsp);
}

#if !__OMIT_FRAMEPOINTER__
void cdmp_arch_print_call_trace(struct out_dev *o, struct __regs *regs)
{
	unsigned long fp = regs->rbp;
	unsigned long *frame;
	int depth_left = 32;

	outf(o, "Call Trace:\n");

	cdmp_gen_print_call_trace_entry(o, regs->rip);

	while (((frame = (void*)fp)) && (depth_left-- > 0)) {
		if (!uk_memprobe_r_isr(fp, sizeof(unsigned long) * 2)) {
			outf(o, " Bad frame pointer\n");
			break;
		}

		cdmp_gen_print_call_trace_entry(o, frame[1]);

		/* Goto next frame */
		fp = frame[0];
	}
}
#endif /* !__OMIT_FRAMEPOINTER__ */
