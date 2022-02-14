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
 * It expects x0, x1, and lr to be already saved in regs->x[x0 ~ x1, lr].
 *
 * @param regs<x0>
 *   Pointer to struct _regs.
 */
.section .text
ENTRY(_uk_crash_do_save_ctx)
	/* Save general-purpose registers */
	stp x2, x3, [x0, #16 * 1]
	stp x4, x5, [x0, #16 * 2]
	stp x6, x7, [x0, #16 * 3]
	stp x8, x9, [x0, #16 * 4]
	stp x10, x11, [x0, #16 * 5]
	stp x12, x13, [x0, #16 * 6]
	stp x14, x15, [x0, #16 * 7]
	stp x16, x17, [x0, #16 * 8]
	stp x18, x19, [x0, #16 * 9]
	stp x20, x21, [x0, #16 * 10]
	stp x22, x23, [x0, #16 * 11]
	stp x24, x25, [x0, #16 * 12]
	stp x26, x27, [x0, #16 * 13]
	stp x28, x29, [x0, #16 * 14]

	/* Save the current link register as regs->pc */
	str lr, [x0, #__REGS_OFFSETOF_PC]

	/* Build a meaningful PSTATE and save the stack pointer */
	mrs x1, nzcv
	mrs x2, daif
	orr x1, x1, x2
	mrs x2, currentel
	orr x1, x1, x2
	mrs x2, spsel
	orr x1, x1, x2

	mov x2, sp
	stp x1, x2, [x0, #__REGS_OFFSETOF_PSTATE]

	ret
ENDPROC(_uk_crash_do_save_ctx)
