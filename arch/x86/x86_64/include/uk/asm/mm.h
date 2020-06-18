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

#ifndef __UKARCH_X86_64_MM__
#define __UKARCH_X86_64_MM__

#include <uk/assert.h>
#include <uk/bitmap.h>
#include <uk/essentials.h>

/* TODO: fix duplicate definitions of these macros */
#define PAGE_SIZE		0x1000UL
#define PAGE_SHIFT		12
#define PAGE_MASK		(~(PAGE_SIZE - 1))

#define PAGE_LARGE_SIZE		0x200000UL
#define PAGE_LARGE_SHIFT	21
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
#define PAGE_ALIGN_DOWN(vaddr)		ALIGN_DOWN(vaddr, PAGE_SIZE)
#define PAGE_LARGE_ALIGN_DOWN(vaddr)	ALIGN_DOWN(vaddr, PAGE_LARGE_SIZE)

#define PTE_REMOVE_FLAGS(pte)		(((pte) & PADDR_MASK) & PAGE_MASK)

/* round up to nearest page address */
#define PAGE_ALIGN_UP(vaddr)	ALIGN_UP(vaddr, PAGE_SIZE)
#define PAGE_LARGE_ALIGN_UP(vaddr) ALIGN_UP(vaddr, PAGE_LARGE_SIZE)

#define PAGE_ALIGNED(vaddr)		(!((vaddr) & (PAGE_SIZE - 1)))
#define PAGE_LARGE_ALIGNED(vaddr)	(!((vaddr) & (PAGE_LARGE_SIZE - 1)))

/* This variable represents the offset between the virtual address of the page
 * table memory area and the physical address of it. This offset changes at
 * runtime between the booting phase and the running phase after that.
 *
 * While booting, the physical addresses and the virtual addresses are equal
 * (either running with paging disabled or with a linear mapping), which means
 * this variable has the value 0.
 *
 * After initializing the new set of page tables, these can be placed at any
 * virtual address. The offset in this case is PAGETABLES_VIRT_OFFSET, defined
 * in include/uk/mem_layout.h
 *
 * Functions of the page table API use this variable to be agnostic of whether
 * they are used in the booting phase or afterwards.
 *
 * TODO: find if there is a better way to achieve this behavior
 */
static unsigned long _virt_offset;

/**
 * Create a PTE (page table entry) that maps to a given physical address with
 * given protections and for the given page table level.
 *
 * @param paddr: physical address where the PTE points to
 * @param prot: protection flags (values defined in include/uk/plat/mm.h)
 * (e.g. page readable, writeable, executable)
 * @param level: the level of the page table where the PTE will be written to
 * (if we create a PTE inside a level 2 page table, for example, it means
 * that it points to a large page, and the large page flag is set
 * accordingly)
 *
 * @return: PTE with flags set accordingly
 */
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

static inline int _ukarch_pte_write_raw(unsigned long pt, size_t offset,
		unsigned long val, size_t level)
{
	UK_ASSERT(level >= 1 && level <= PAGETABLE_LEVELS);
	UK_ASSERT(PAGE_ALIGNED(pt));
	UK_ASSERT(offset < pagetable_entries[level - 1]);

	*((unsigned long *) pt + offset) = val;

	return 0;
}

static inline unsigned long ukarch_pte_read(unsigned long pt, size_t offset,
		size_t level)
{
	UK_ASSERT(level >= 1 && level <= PAGETABLE_LEVELS);
	UK_ASSERT(PAGE_ALIGNED(pt));
	UK_ASSERT(offset < pagetable_entries[level - 1]);

	return *((unsigned long *) pt + offset);
}

#endif	/* __UKARCH_X86_64_MM__ */
