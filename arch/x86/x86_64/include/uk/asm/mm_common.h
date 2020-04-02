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

#ifndef __UKPLAT_X86_64_MM_COMMON__
#define __UKPLAT_X86_64_MM_COMMON__

#include <uk/bitmap.h>

#define PAGE_SIZE		0x1000UL
#define PAGE_SHIFT		12
#define PAGE_MASK		(~(PAGE_SIZE - 1))
#define PAGETABLE_LEVELS	4

#define PADDR_BITS		44
#define PADDR_MASK		((1UL << PADDR_BITS) - 1)

#define L1_PAGETABLE_SHIFT	12
#define L2_PAGETABLE_SHIFT	21
#define L3_PAGETABLE_SHIFT	30
#define L4_PAGETABLE_SHIFT	39

#define L1_PAGETABLE_ENTRIES	512
#define L2_PAGETABLE_ENTRIES	512
#define L3_PAGETABLE_ENTRIES	512
#define L4_PAGETABLE_ENTRIES	512

static unsigned long pagetable_entries[PAGETABLE_LEVELS] = {
	L1_PAGETABLE_ENTRIES,
	L2_PAGETABLE_ENTRIES,
	L3_PAGETABLE_ENTRIES,
	L4_PAGETABLE_ENTRIES,
};

static unsigned long pagetable_shifts[PAGETABLE_LEVELS] = {
	L1_PAGETABLE_SHIFT,
	L2_PAGETABLE_SHIFT,
	L3_PAGETABLE_SHIFT,
	L4_PAGETABLE_SHIFT,
};

#define L1_OFFSET(vaddr) \
	(((vaddr) >> L1_PAGETABLE_SHIFT) & (L1_PAGETABLE_ENTRIES - 1))
#define L2_OFFSET(vaddr) \
	(((vaddr) >> L2_PAGETABLE_SHIFT) & (L2_PAGETABLE_ENTRIES - 1))
#define L3_OFFSET(vaddr) \
	(((vaddr) >> L3_PAGETABLE_SHIFT) & (L3_PAGETABLE_ENTRIES - 1))
#define L4_OFFSET(vaddr) \
	(((vaddr) >> L4_PAGETABLE_SHIFT) & (L4_PAGETABLE_ENTRIES - 1))

#define Lx_OFFSET(vaddr, lvl) \
	(((vaddr) >> pagetable_shifts[lvl - 1]) \
		 & (pagetable_entries[lvl - 1] - 1))

#define _PAGE_PRESENT	0x001UL
#define _PAGE_RW	0x002UL
#define _PAGE_USER	0x004UL
#define _PAGE_PWT	0x008UL
#define _PAGE_PCD	0x010UL
#define _PAGE_ACCESSED	0x020UL
#define _PAGE_DIRTY	0x040UL
#define _PAGE_PAT	0x080UL
#define _PAGE_PSE	0x080UL
#define _PAGE_GLOBAL	0x100UL
#define _PAGE_NX	(1UL << 63)
#define _PAGE_PROTNONE	(1UL << 62) /* one of the user available bits */

/*
 * If the user maps the page with PROT_NONE, the _PAGE_PRESENT bit is not set,
 * but PAGE_PRESENT must return true, so no other page is mapped on top.
 */
#define PAGE_PRESENT(vaddr)	((vaddr) & (_PAGE_PRESENT | _PAGE_PROTNONE))
#define PAGE_LARGE(vaddr)	((vaddr) & _PAGE_PSE)
#define PAGE_HUGE(vaddr)	((vaddr) & _PAGE_PSE)

#define L1_PROT    (_PAGE_PRESENT | _PAGE_RW | _PAGE_ACCESSED)
#define L1_PROT_RO (_PAGE_PRESENT | _PAGE_ACCESSED)
#define L2_PROT    (_PAGE_PRESENT | _PAGE_RW | _PAGE_ACCESSED | _PAGE_DIRTY)
#define L3_PROT    (_PAGE_PRESENT | _PAGE_RW | _PAGE_ACCESSED | _PAGE_DIRTY)
#define L4_PROT    (_PAGE_PRESENT | _PAGE_RW | _PAGE_ACCESSED | _PAGE_DIRTY)

static unsigned long pagetable_protections[PAGETABLE_LEVELS] = {
	L1_PROT,
	L2_PROT,
	L3_PROT,
	L4_PROT,
};

/* round down to nearest page address */
#define PAGE_ALIGN_DOWN(vaddr) ((vaddr) & PAGE_MASK)
#define PTE_REMOVE_FLAGS(vaddr) (((vaddr) & PADDR_MASK) & PAGE_MASK)

/* round up to nearest page address */
#define PAGE_ALIGN(vaddr) (((vaddr) + (PAGE_SIZE - 1)) & PAGE_MASK)

/* returns 1 if |vaddr| is page aligned */
#define PAGE_ALIGNED(vaddr) (!((vaddr) & (PAGE_SIZE - 1)))

static inline unsigned long ukarch_pte_l1e(unsigned long paddr,
		unsigned long prot)
{
	unsigned long flags;

	if (prot == PAGE_PROT_NONE)
		flags = _PAGE_ACCESSED | _PAGE_PROTNONE;

	if (prot & PAGE_PROT_WRITE)
		flags = L1_PROT;
	else
		flags = L1_PROT_RO;

	if (!(prot & PAGE_PROT_EXEC))
		flags |= _PAGE_NX;

	return paddr | flags;
}

static inline unsigned long ukarch_phys_frame_get(void)
{
	unsigned long offset;
	unsigned long pfn;

	offset = uk_bitmap_find_next_zero_area(
			(unsigned long *) phys_bitmap_start_addr,
			phys_bitmap_length,
			0 /* start */, 1 /* nr */, 0 /* align_mask */);

	uk_bitmap_set((unsigned long *) phys_bitmap_start_addr, offset, 1);

	pfn = (allocatable_memory_start_addr >> PAGE_SHIFT) + offset;

	if (offset * PAGE_SIZE > allocatable_memory_length) {
		uk_pr_err("Out of memory\n");
		return PAGE_INVALID;
	}

#ifdef CONFIG_PARAVIRT
	return pfn_to_mfn(pfn) << PAGE_SHIFT;
#else
	return pfn << PAGE_SHIFT;
#endif /* CONFIG_PARAVIRT */
}

#endif	/* __UKPLAT_X86_64_MM_COMMON__ */
