/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Costin Lupu <costin.lupu@cs.pub.ro>
 *
 * Copyright (c) 2018, NEC Europe Ltd., NEC Corporation. All rights reserved.
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
/* Ported from Mini-OS */

#include <uk/arch/lcpu.h>
#include <x86/cpu.h>
#include <x86/traps.h>
#include <uk/assert.h>

static int trap_to_sig(int trapnr)
{
	/* Map trap number to signal number. We use the same mapping like
	 * kgdb in the Linux kernel.
	 */
	switch (trapnr) {
	case TRAP_divide_error:
	case TRAP_coproc_error:
	case TRAP_simd_error:
		return 8;  /* SIGFPE  - Floating-point exception */
	case TRAP_debug:
	case TRAP_int3:
		return 5;  /* SIGTRAP - Trace/breakpoint trap */
	case TRAP_invalid_op:
		return 4;  /* SIGILL  - Illegal Instruction */
	case TRAP_no_segment:
	case TRAP_stack_error:
	case TRAP_alignment_check:
		return 7;  /* SIGBUS  - Bus error (bad memory access) */
	case TRAP_page_fault:
		return 11; /* SIGSEGV - Invalid memory reference */
	}

	/* There is no suitable signal. So just return SIGSEGV */
	return 11;
}

/* A general word of caution when writing trap handlers. The platform trap
 * entry code is set up to properly save general-purpose registers (e.g., rsi,
 * rdi, rax, r8, ...), but it does NOT save any floating-point or SSE/AVX
 * registers. (This would require figuring out in the trap handler code whether
 * these are available to not risk a #UD trap inside the trap handler itself.)
 * Hence, you need to be extra careful not to do anything that clobbers these
 * registers if you intend to return from the handler. This includes calling
 * other functions, which may clobber those registers.
 * Of course, if you end your trap handler with a UK_CRASH, knock yourself out,
 * it's not like the function you came from will ever have the chance to notice.
 */

/* Traps handled on both Xen and KVM */

DECLARE_TRAP_EVENT(UKARCH_TRAP_INVALID_OP);
DECLARE_TRAP_EVENT(UKARCH_TRAP_DEBUG);
DECLARE_TRAP_EVENT(UKARCH_TRAP_PAGE_FAULT);
DECLARE_TRAP_EVENT(UKARCH_TRAP_BUS_ERROR);
DECLARE_TRAP_EVENT(UKARCH_TRAP_MATH);
DECLARE_TRAP_EVENT(UKARCH_TRAP_SECURITY);
DECLARE_TRAP_EVENT(UKARCH_TRAP_X86_GP);

DECLARE_TRAP_EC(divide_error,    "divide error",         UKARCH_TRAP_MATH)
DECLARE_TRAP   (debug,           "debug",                UKARCH_TRAP_DEBUG)
DECLARE_TRAP_EC(int3,            "int3",                 UKARCH_TRAP_DEBUG)
DECLARE_TRAP_EC(overflow,        "overflow",             NULL)
DECLARE_TRAP_EC(bounds,          "bounds",               NULL)
DECLARE_TRAP_EC(invalid_op,      "invalid opcode",       UKARCH_TRAP_INVALID_OP)
DECLARE_TRAP_EC(no_device,       "device not available", UKARCH_TRAP_MATH)
DECLARE_TRAP_EC(invalid_tss,     "invalid TSS",          NULL)
DECLARE_TRAP_EC(no_segment,      "segment not present",  UKARCH_TRAP_BUS_ERROR)
DECLARE_TRAP_EC(stack_error,     "stack segment",        UKARCH_TRAP_BUS_ERROR)
DECLARE_TRAP_EC(gp_fault,        "general protection",   UKARCH_TRAP_X86_GP)
DECLARE_TRAP   (coproc_error,    "coprocessor",          UKARCH_TRAP_MATH)
DECLARE_TRAP_EC(alignment_check, "alignment check",      UKARCH_TRAP_BUS_ERROR)
DECLARE_TRAP_EC(machine_check,   "machine check",        NULL)
DECLARE_TRAP   (simd_error,      "SIMD coprocessor",     UKARCH_TRAP_MATH)
DECLARE_TRAP_EC(security_error,  "control protection",   UKARCH_TRAP_SECURITY)

void do_unhandled_trap(int trapnr, char *str, struct __regs *regs,
		unsigned long error_code)
{
	UK_CRASH_EX(trap_to_sig(trapnr), regs,
		    "Unhandled trap %d (%s), error=0x%lx\n",
		    trapnr, str, error_code);
}

#if 0
int getcpu(unsigned *cpu, unsigned *node, unsigned *tcache)
{
	return 0;
}
#endif

void do_page_fault(struct __regs *regs, unsigned long error_code)
{
	unsigned long vaddr = read_cr2();
	struct ukarch_trap_ctx ctx = {regs, TRAP_page_fault, error_code, vaddr};

#if 0
	if (regs->rip == 0xffffffffff600000) {
		uk_pr_debug_once("Address of gettimeofday: %p\n", gettimeofday);
		regs->rip =  (uint64_t)gettimeofday;
		return;
	}

	if (regs->rip == 0xffffffffff600800) {
		uk_pr_debug_once("Address of getcpu: %p\n", getcpu);
		regs->rip =  (uint64_t)getcpu;
		return;
	}
#endif

	if (uk_raise_event(UKARCH_TRAP_PAGE_FAULT, &ctx))
		return;

	UK_CRASH_EX(trap_to_sig(TRAP_page_fault), regs,
		    "Unhandled trap 13 (page fault), vaddr=0x%lx\n",
		    vaddr);
}
