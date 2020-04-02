/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Stefan Teodorescu <stefanl.teodorescu@gmail.com>
 *
 * Copyright (c) 2020, University Politehnica of Bucharest. All rights reserved.
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
 *
 * THIS HEADER MAY NOT BE EXTRACTED OR MODIFIED IN ANY WAY.
 *
 * Some of these macros here were inspired from Xen code.
 * For example, from "xen/include/asm-x86/x86_64/page.h" file.
 */

#ifndef __UKPLAT_X86_64_MM__
#define __UKPLAT_X86_64_MM__

#include "mm_common.h"
#include <uk/bitmap.h>
#include <uk/assert.h>
#include <uk/print.h>

/*
 * TODO: provide real implementations of pte_to_virt and virt_to_mfn
 * These one works only for 1:1 mapping (pte contains physical address, not
 * virtual) We might maintain a cvasi-linear mapping, just by offseting virtual
 * addresses of PTs from their physical addreses (vaddr = paddr + CONSTANT)
 */
#define pte_to_virt(vaddr) PTE_REMOVE_FLAGS(vaddr)
#define virt_to_mfn(vaddr) ((vaddr) >> PAGE_SHIFT)

#define pte_to_pfn(pte) (PTE_REMOVE_FLAGS(pte) >> PAGE_SHIFT)

static inline unsigned long ukarch_read_pt_base(void)
{
	unsigned long cr3;

	__asm__ __volatile__("movq %%cr3, %0" : "=r"(cr3)::);

	/*
	 * For consistency with Xen implementation, which returns a virtual
	 * address, this should return the same.
	 */
	return pte_to_virt(cr3);
}

static inline int ukarch_flush_tlb_entry(unsigned long vaddr)
{
	__asm__ __volatile__("invlpg (%0)" ::"r" (vaddr) : "memory");

	return 0;
}

static inline int ukarch_pte_write(unsigned long pt, size_t offset,
		unsigned long val, size_t level)
{
	UK_ASSERT(level >= 1 && level <= PAGETABLE_LEVELS);
	UK_ASSERT(PAGE_ALIGNED(pt));
	UK_ASSERT(offset < pagetable_entries[level - 1]);

	*((unsigned long *) pt + offset) = val;

	return 0;
}

#endif	/* __UKPLAT_X86_64_MM__ */
