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

#define PAGE_LARGE_SIZE		0x200000UL
#define PAGE_LARGE_SHIFT	20
#define PAGE_LARGE_MASK		 (~(PAGE_LARGE_SIZE - 1))

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

static unsigned long pagetable_shifts[PAGETABLE_LEVELS] __used = {
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
#define PAGE_ALIGN_DOWN(vaddr)		((vaddr) & PAGE_MASK)
#define PAGE_LARGE_ALIGN_DOWN(vaddr)	((vaddr) & PAGE_LARGE_MASK)

#define PTE_REMOVE_FLAGS(pte)		(((pte) & PADDR_MASK) & PAGE_MASK)

/* round up to nearest page address */
#define PAGE_ALIGN(vaddr)	(((vaddr) + (PAGE_SIZE - 1)) & PAGE_MASK)
#define PAGE_LARGE_ALIGN(vaddr) \
	(((vaddr) + (PAGE_LARGE_SIZE - 1)) & PAGE_LARGE_MASK)

#define PAGE_ALIGNED(vaddr)		(!((vaddr) & (PAGE_SIZE - 1)))
#define PAGE_LARGE_ALIGNED(vaddr)	(!((vaddr) & (PAGE_LARGE_SIZE - 1)))

/* TODO: find if there is a better way to change offset */
/* I use the same functions before and after switching the PT */
static unsigned long _virt_offset;

static inline unsigned long ukarch_pte_create(unsigned long paddr,
					      unsigned long prot, size_t level)
{
	unsigned long flags = 0;

	/* For level == 2 it is a large page and level == 3 huge page */
	if (level >= 2)
		flags |= _PAGE_PSE;

	if (prot == PAGE_PROT_NONE)
		flags |= _PAGE_ACCESSED | _PAGE_PROTNONE;
	else
		flags |= pagetable_protections[level - 1];

	if (!(prot & PAGE_PROT_WRITE))
		flags &= ~_PAGE_RW;

	if (!(prot & PAGE_PROT_EXEC))
		flags |= _PAGE_NX;

	return paddr | flags;
}

static inline unsigned long ukarch_phys_frame_get(unsigned long flags)
{
	unsigned long offset;
	unsigned long pfn;
	unsigned long frame_size;

#ifdef CONFIG_PARAVIRT
	if (flags & PAGE_FLAG_LARGE) {
		uk_pr_err("Large pages are not supported on PV guest\n");
		return PAGE_INVALID;
	}
#endif /* CONFIG_PARAVIRT */

	if (flags & PAGE_FLAG_LARGE)
		frame_size = PAGE_LARGE_SIZE / PAGE_SIZE;
	else
		frame_size = 1;

	offset = uk_bitmap_find_next_zero_area(
			(unsigned long *) phys_bitmap_start_addr,
			phys_bitmap_length,
			0 /* start */,
			frame_size /* nr */,
			frame_size - 1 /* align_mask */);

	uk_bitmap_set((unsigned long *) phys_bitmap_start_addr, offset,
		      frame_size);

	pfn = (phys_mem_start_addr >> PAGE_SHIFT) + offset;

	if (offset * PAGE_SIZE > phys_mem_length) {
		uk_pr_err("Out of physical memory\n");
		return PAGE_INVALID;
	}

#ifdef CONFIG_PARAVIRT
	return pfn_to_mfn(pfn) << PAGE_SHIFT;
#else
	return pfn << PAGE_SHIFT;
#endif /* CONFIG_PARAVIRT */
}

#endif	/* __UKPLAT_X86_64_MM_COMMON__ */
