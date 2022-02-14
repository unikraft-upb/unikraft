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

#ifndef __UKNOFAULT_ARCH_MACCESS_H__
#define __UKNOFAULT_ARCH_MACCESS_H__

#include "../../../../excpttab.h"

#define _nf_do_memcpy(dst, src, reg, cont_label)			\
	__asm__ goto(							\
		"1: mov (%1), %%"reg"\n"				\
		"2: mov %%"reg", (%0)\n"				\
		NF_EXCPTTAB_ENTRY_L(1b, cont_label, _nf_mf_handler)	\
		NF_EXCPTTAB_ENTRY_L(2b, cont_label, _nf_mf_handler)	\
		: /* no outputs in goto asm allowed */			\
		: "r"(dst), "r"(src)					\
		: reg : cont_label)

#define nf_memcpy(dst, src, type, cont_label)				\
	do {								\
		switch (sizeof(type)) {					\
		case 1:							\
			_nf_do_memcpy(dst, src, "al", cont_label);	\
			break;						\
		case 2:							\
			_nf_do_memcpy(dst, src, "ax", cont_label);	\
			break;						\
		case 4:							\
			_nf_do_memcpy(dst, src, "eax",cont_label);	\
			break;						\
		case 8:							\
			_nf_do_memcpy(dst, src, "rax",cont_label);	\
			break;						\
		}							\
	} while (0)

#define _nf_do_probe_r(addr, reg, cont_label)				\
	__asm__ goto(							\
		"1: mov (%0), %%"reg"\n"				\
		NF_EXCPTTAB_ENTRY_L(1b, cont_label, _nf_mf_handler)	\
		: /* no outputs in goto asm allowed */			\
		: "r"(addr)						\
		: reg : cont_label)

#define nf_probe_r(addr, type, cont_label)				\
	do {								\
		switch (sizeof(type)) {					\
		case 1:							\
			_nf_do_probe_r(addr, "al", cont_label);		\
			break;						\
		case 2:							\
			_nf_do_probe_r(addr, "ax", cont_label);		\
			break;						\
		case 4:							\
			_nf_do_probe_r(addr, "eax",cont_label);		\
			break;						\
		case 8:							\
			_nf_do_probe_r(addr, "rax",cont_label);		\
			break;						\
		}							\
	} while (0)

#define nf_regs_ip(regs) ((regs)->rip)

#endif /* __UKNOFAULT_ARCH_MACCESS_H__ */
