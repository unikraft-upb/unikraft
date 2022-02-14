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

#include <uk/nofault.h>
#include <uk/arch/types.h>

#include <arch/maccess.h>

/* Must be a library global symbol so that the linker can resolve it */
int _nf_mf_handler(const struct nf_excpttab_entry *e,
		   struct ukarch_trap_ctx *ctx)
{
	/* Just continue execution at the fault label */
	nf_regs_ip(ctx->regs) = nf_excpttab_get_cont_ip(e);
	return UK_EVENT_HANDLED;
}

static int _nf_mem_fault_handler(void *data)
{
	struct ukarch_trap_ctx *ctx = (struct ukarch_trap_ctx *)data;

	return nf_handle_trap(nf_regs_ip(ctx->regs), ctx);
}
UK_EVENT_HANDLER(UKARCH_TRAP_PAGE_FAULT, _nf_mem_fault_handler);
UK_EVENT_HANDLER(UKARCH_TRAP_BUS_ERROR,  _nf_mem_fault_handler);
#ifdef CONFIG_ARCH_X86_64
UK_EVENT_HANDLER(UKARCH_TRAP_X86_GP,     _nf_mem_fault_handler);
#endif /* CONFIG_ARCH_X86_64 */

#define _nf_memcpy_loop(dst, src, len, type, cont_label)	\
	while (len >= sizeof(type)) {				\
		nf_memcpy((type*)(dst), (type*)(src),		\
				type, cont_label);		\
		dst += sizeof(type);				\
		src += sizeof(type);				\
		len -= sizeof(type);				\
	}

#define _nf_probe_r_loop(addr, len, type, cont_label)		\
	while (len >= sizeof(type)) {				\
		nf_probe_r((type*)(addr), type, cont_label);	\
		addr += sizeof(type);				\
		len  -= sizeof(type);				\
	}

int uk_memprobe_r(unsigned long addr, __sz len)
{
	_nf_probe_r_loop(addr, len, __u64, fault);
	_nf_probe_r_loop(addr, len, __u32, fault);
	_nf_probe_r_loop(addr, len, __u16, fault);
	_nf_probe_r_loop(addr, len, __u8, fault);
	return 1;
fault:
	return 0;
}

int uk_memprobe_r_isr(unsigned long addr, __sz len)
{
	int r;

	/* TODO: Disable paging */
	r = uk_memprobe_r(addr, len);
	/* TODO: Enable paging */

	return r;
}

int uk_memprobe_rw(unsigned long addr, __sz len)
{
	return (uk_memcpy_nofault((void*)addr, (void*)addr, len) > 0);
}

int uk_memprobe_rw_isr(unsigned long addr, __sz len)
{
	int r;

	/* TODO: Disable paging */
	r = uk_memprobe_rw(addr, len);
	/* TODO: Enable paging */

	return r;
}

__ssz uk_memcpy_nofault(char *dst, const char *src, __sz len)
{
	__sz l = len;
	_nf_memcpy_loop(dst, src, l, __u64, fault);
	_nf_memcpy_loop(dst, src, l, __u32, fault);
	_nf_memcpy_loop(dst, src, l, __u16, fault);
	_nf_memcpy_loop(dst, src, l, __u8, fault);
	return len;
fault:
	return -1;
}

__ssz uk_memcpy_nofault_isr(char *dst, const char *src, __sz len)
{
	__ssz r;

	/* TODO: Disable paging */
	r = uk_memcpy_nofault(dst, src, len);
	/* TODO: Enable paging */

	return r;
}
