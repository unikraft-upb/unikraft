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
 */

#include <errno.h>
#include <string.h>
#include <unistd.h>

#include <uk/assert.h>
#include <uk/print.h>
#include <uk/plat/mm.h>
#include <uk/arch/mem_layout.h>

unsigned long phys_bitmap_start_addr;
unsigned long phys_bitmap_length;

unsigned long phys_mem_start_addr;
unsigned long phys_mem_length;

static unsigned long pt_bitmap_start_addr;
static unsigned long pt_bitmap_length;

static unsigned long pt_mem_start_addr;
static unsigned long pt_mem_length;

static size_t _used_pts;

static unsigned long _pt_get(void)
{
	unsigned long offset;

	offset = uk_bitmap_find_next_zero_area(
			(unsigned long *) pt_bitmap_start_addr, pt_bitmap_length,
			0 /* start */, 1 /* nr */, 0 /* align_mask */);

	uk_bitmap_set((unsigned long *) pt_bitmap_start_addr, offset, 1);

	if (offset * PAGE_SIZE > pt_mem_length) {
		uk_pr_err("Filled up all available space for page tables\n");
		return PAGE_INVALID;
	}

	return pt_mem_start_addr + (offset << PAGE_SHIFT);
}

static unsigned long uk_pt_alloc_table(size_t level)
{
	unsigned long pt_vaddr = _pt_get() + _virt_offset;

	if (pt_vaddr == PAGE_INVALID)
		return PAGE_INVALID;

#ifdef CONFIG_PARAVIRT
	uk_page_set_prot(pt_vaddr, PAGE_PROT_READ | PAGE_PROT_WRITE);
#endif	/* CONFIG_PARAVIRT */

	memset((void *) pt_vaddr, 0,
	       sizeof(unsigned long) * pagetable_entries[level - 1]);

	/* Xen requires that PTs are mapped read-only */
#ifdef CONFIG_PARAVIRT
	/*
	 * TODO: when using this function on Xen for the initmem part, the page
	 * must not be set to read-only, as we are currently writing
	 * directly into it. All page tables will be set later to read-only
	 * before setting the new pt_base.
	 */
	uk_page_set_prot(pt_vaddr, PAGE_PROT_READ);
#endif	/* CONFIG_PARAVIRT */

	/*
	 * This is an L(n + 1) entry, so we set L(n + 1) flags
	 * (Index in pagetable_protections is level of PT - 1)
	 */
	return (virt_to_mfn(pt_vaddr) << PAGE_SHIFT)
		| pagetable_protections[level];
}

static void uk_pt_release_if_unused(unsigned long vaddr, unsigned long pt,
		unsigned long parent_pt, size_t level)
{
	unsigned long offset;
	size_t i;

	if (!PAGE_ALIGNED(pt) || !PAGE_ALIGNED(parent_pt)) {
		uk_pr_err("Table's address must be aligned to page size\n");
		return;
	}

	for (i = 0; i < pagetable_entries[level - 1]; i++)
		if (PAGE_PRESENT(uk_pte_read(pt, i, level)))
			return;

	ukarch_pte_write(parent_pt, Lx_OFFSET(vaddr, level + 1), 0, level + 1);
	ukarch_flush_tlb_entry(parent_pt);

	offset = (pt - pt_mem_start_addr - _virt_offset) >> PAGE_SHIFT;
	uk_bitmap_clear((unsigned long *) pt_bitmap_start_addr, offset, 1);
}

int _page_map(unsigned long pt, unsigned long vaddr, unsigned long paddr,
	      unsigned long prot, unsigned long flags,
	      int (*pte_write)(unsigned long, size_t, unsigned long, size_t))
{
	unsigned long pte;

	if (!PAGE_ALIGNED(vaddr)) {
		uk_pr_err("Virt address must be aligned to page size\n");
		return -1;
	}
	if (flags & PAGE_FLAG_LARGE && !PAGE_LARGE_ALIGNED(vaddr)) {
		uk_pr_err("Virt ddress must be aligned to large page size\n");
		return -1;
	}

#ifdef CONFIG_PARAVIRT
	if (flags & PAGE_FLAG_LARGE) {
		uk_pr_err("Large pages are not supported on PV guest\n");
		return -1;
	}
#endif /* CONFIG_PARAVIRT */

	if (paddr == (unsigned long) -1) {
		paddr = ukarch_phys_frame_get(flags);

		if (paddr == PAGE_INVALID)
			return -1;
	} else if (!PAGE_ALIGNED(paddr)) {
		uk_pr_err("Phys address must be aligned to page size\n");
		return -1;
	} else if ((flags & PAGE_FLAG_LARGE) && !PAGE_LARGE_ALIGNED(paddr)) {
		uk_pr_err("Phys address must be aligned to large page size\n");
		return -1;
	}

	/*
	 * XXX: On 64-bits architectures (x86_64 and arm64) the hierarchical
	 * page tables have a 4 level layout. This implementation will need a
	 * revision when introducing support for 32-bits architectures, since
	 * there are only 3 levels of page tables.
	 */
	pte = uk_pte_read(pt, L4_OFFSET(vaddr), 4);
	if (!PAGE_PRESENT(pte)) {
		pte = uk_pt_alloc_table(3);

		if (pte == PAGE_INVALID)
			return -1;

		pte_write(pt, L4_OFFSET(vaddr), pte, 4);
	}

	pt = (unsigned long) pte_to_virt(pte);
	pte = uk_pte_read(pt, L3_OFFSET(vaddr), 3);
	if (!PAGE_PRESENT(pte)) {
		pte = uk_pt_alloc_table(2);

		if (pte == PAGE_INVALID)
			return -1;

		pte_write(pt, L3_OFFSET(vaddr), pte, 3);
	}

	pt = (unsigned long) pte_to_virt(pte);
	pte = uk_pte_read(pt, L2_OFFSET(vaddr), 2);
	if (flags & PAGE_FLAG_LARGE) {
		if (PAGE_PRESENT(pte))
			return -1;

		pte = ukarch_pte_create(PTE_REMOVE_FLAGS(paddr), prot, 2);
		pte_write(pt, L2_OFFSET(vaddr), pte, 2);

		return 0;
	}
	if (!PAGE_PRESENT(pte)) {
		pte = uk_pt_alloc_table(1);

		if (pte == PAGE_INVALID)
			return -1;

		pte_write(pt, L2_OFFSET(vaddr), pte, 2);
	}

	pt = (unsigned long) pte_to_virt(pte);
	pte = uk_pte_read(pt, L1_OFFSET(vaddr), 1);
	if (!PAGE_PRESENT(pte)) {
		pte = ukarch_pte_create(PTE_REMOVE_FLAGS(paddr), prot, 1);
		pte_write(pt, L1_OFFSET(vaddr), pte, 1);
	} else {
		uk_pr_info("Virtual address 0x%08lx is already mapped\n", vaddr);
		return -1;
	}

	return 0;
}

int _initmem_page_map(unsigned long pt, unsigned long vaddr,
		      unsigned long paddr, unsigned long prot,
		      unsigned long flags)
{
	return _page_map(pt, vaddr, paddr, prot, flags, _initmem_pte_write);
}

int uk_page_map(unsigned long vaddr, unsigned long paddr, unsigned long prot,
		unsigned long flags)
{
	return _page_map(ukarch_read_pt_base(), vaddr, paddr, prot, flags,
			 ukarch_pte_write);
}

int uk_page_unmap(unsigned long vaddr)
{
	unsigned long l1_table, l2_table, l3_table, l4_table, pte;
	unsigned long offset, pfn;
	unsigned long frame_size = 1;

	if (!PAGE_ALIGNED(vaddr)) {
		uk_pr_err("Address must be aligned to page size\n");
		return -1;
	}

	l4_table = ukarch_read_pt_base();
	pte = uk_pte_read(l4_table, L4_OFFSET(vaddr), 4);
	if (!PAGE_PRESENT(pte))
		return -1;

	l3_table = (unsigned long) pte_to_virt(pte);
	pte = uk_pte_read(l3_table, L3_OFFSET(vaddr), 3);
	if (!PAGE_PRESENT(pte))
		return -1;

	l2_table = (unsigned long) pte_to_virt(pte);
	pte = uk_pte_read(l2_table, L2_OFFSET(vaddr), 2);
	if (!PAGE_PRESENT(pte))
		return -1;
	if (PAGE_LARGE(pte)) {
		if (!PAGE_LARGE_ALIGNED(vaddr))
			return -1;

		pfn = pte_to_pfn(pte);
		ukarch_pte_write(l2_table, L2_OFFSET(vaddr), 0, 2);
		frame_size = PAGE_LARGE_SIZE / PAGE_SIZE;
	} else {
		l1_table = (unsigned long) pte_to_virt(pte);
		pte = uk_pte_read(l1_table, L1_OFFSET(vaddr), 1);
		if (!PAGE_PRESENT(pte))
			return -1;

		pfn = pte_to_pfn(pte);
		ukarch_pte_write(l1_table, L1_OFFSET(vaddr), 0, 1);
		uk_pt_release_if_unused(vaddr, l1_table, l2_table, 1);
	}

	ukarch_flush_tlb_entry(vaddr);

	offset = pfn - (phys_mem_start_addr >> PAGE_SHIFT);
	uk_bitmap_clear((unsigned long *) phys_bitmap_start_addr,
			offset, frame_size);

	uk_pt_release_if_unused(vaddr, l2_table, l3_table, 2);
	uk_pt_release_if_unused(vaddr, l3_table, l4_table, 3);

	return 0;
}

int uk_page_set_prot(unsigned long vaddr, unsigned long new_prot)
{
	unsigned long pt, pte;
	unsigned long new_pte;

	if (!PAGE_ALIGNED(vaddr)) {
		uk_pr_info("Address must be aligned to page size\n");
		return -1;
	}

	pt = ukarch_read_pt_base();
	pte = uk_pte_read(pt, L4_OFFSET(vaddr), 4);
	if (!PAGE_PRESENT(pte))
		return -1;

	pt = (unsigned long) pte_to_virt(pte);
	pte = uk_pte_read(pt, L3_OFFSET(vaddr), 3);
	if (!PAGE_PRESENT(pte))
		return -1;

	pt = (unsigned long) pte_to_virt(pte);
	pte = uk_pte_read(pt, L2_OFFSET(vaddr), 2);
	if (!PAGE_PRESENT(pte))
		return -1;
	if (PAGE_LARGE(pte)) {
		new_pte = ukarch_pte_create(PTE_REMOVE_FLAGS(pte), new_prot, 2);
		ukarch_pte_write(pt, L2_OFFSET(vaddr), new_pte, 2);
		ukarch_flush_tlb_entry(vaddr);

		return 0;
	}

	pt = (unsigned long) pte_to_virt(pte);
	pte = uk_pte_read(pt, L1_OFFSET(vaddr), 1);
	if (!PAGE_PRESENT(pte))
		return -1;

	new_pte = ukarch_pte_create(PTE_REMOVE_FLAGS(pte), new_prot, 1);
	ukarch_pte_write(pt, L1_OFFSET(vaddr), new_pte, 1);
	ukarch_flush_tlb_entry(vaddr);

	return 0;
}

unsigned long uk_virt_to_pte(unsigned long vaddr)
{
	unsigned long pt, pt_entry;

	if (!PAGE_ALIGNED(vaddr)) {
		uk_pr_err("Address must be aligned to page size\n");
		return PAGE_NOT_MAPPED;
	}

	pt = ukarch_read_pt_base();
	pt_entry = uk_pte_read(pt, L4_OFFSET(vaddr), 4);
	if (!PAGE_PRESENT(pt_entry))
		return PAGE_NOT_MAPPED;

	pt = (unsigned long) pte_to_virt(pt_entry);
	pt_entry = uk_pte_read(pt, L3_OFFSET(vaddr), 3);
	if (!PAGE_PRESENT(pt_entry))
		return PAGE_NOT_MAPPED;
	if (PAGE_HUGE(pt_entry))
		return pt_entry;

	pt = (unsigned long) pte_to_virt(pt_entry);
	pt_entry = uk_pte_read(pt, L2_OFFSET(vaddr), 2);
	if (!PAGE_PRESENT(pt_entry))
		return PAGE_NOT_MAPPED;
	if (PAGE_LARGE(pt_entry))
		return pt_entry;

	pt = (unsigned long) pte_to_virt(pt_entry);
	pt_entry = uk_pte_read(pt, L1_OFFSET(vaddr), 1);

	return pt_entry;
}

static unsigned long _initmem_pt_get(void)
{
	return PAGETABLES_AREA_START
	       - PAGETABLES_VIRT_OFFSET
	       + (_used_pts++) * PAGE_SIZE;
}

static unsigned long _pt_create(unsigned long vaddr_start, unsigned long len,
				unsigned long virt_offset)
{
	unsigned long pt_l4, pt_l3, pt_l2, pt_l1;
	unsigned long prev_l4_offset, prev_l3_offset, prev_l2_offset;
	unsigned long page, frame;

	UK_ASSERT(PAGE_ALIGNED(vaddr_start));
	UK_ASSERT(PAGE_ALIGNED(len));
	UK_ASSERT(PAGE_ALIGNED(virt_offset));

	pt_l4 = _initmem_pt_get();
	pt_l3 = _initmem_pt_get();
	pt_l2 = _initmem_pt_get();
	pt_l1 = _initmem_pt_get();

	prev_l4_offset = L4_OFFSET(vaddr_start);
	prev_l3_offset = L3_OFFSET(vaddr_start);
	prev_l2_offset = L2_OFFSET(vaddr_start);

	_initmem_pte_write(pt_l4, prev_l4_offset, pt_l3 | L4_PROT, 4);
	_initmem_pte_write(pt_l3, prev_l3_offset, pt_l2 | L3_PROT, 3);
	_initmem_pte_write(pt_l2, prev_l2_offset, pt_l1 | L2_PROT, 2);

	for (page = vaddr_start; page < vaddr_start + len; page += PAGE_SIZE) {
		frame = page - virt_offset;

		if (L2_OFFSET(page) != prev_l2_offset) {
			pt_l1 = _initmem_pt_get();
			_initmem_pte_write(pt_l2, L2_OFFSET(page),
					   pt_l1 | L2_PROT, 2);
			prev_l2_offset = L2_OFFSET(page);
		}

		if (L3_OFFSET(page) != prev_l3_offset) {
			pt_l2 = _initmem_pt_get();
			_initmem_pte_write(pt_l3, L3_OFFSET(page),
					   pt_l2 | L3_PROT, 3);
			prev_l3_offset = L3_OFFSET(page);

			pt_l1 = _initmem_pt_get();
			_initmem_pte_write(pt_l2, L2_OFFSET(page),
					   pt_l1 | L2_PROT, 2);
			prev_l2_offset = L2_OFFSET(page);
		}

		if (L4_OFFSET(page) != prev_l4_offset) {
			pt_l3 = _initmem_pt_get();
			_initmem_pte_write(pt_l4, L4_OFFSET(page),
					   pt_l3 | L4_PROT, 4);
			prev_l4_offset = L4_OFFSET(page);

			pt_l2 = _initmem_pt_get();
			_initmem_pte_write(pt_l3, L3_OFFSET(page),
					   pt_l2 | L3_PROT, 3);
			prev_l3_offset = L3_OFFSET(page);

			pt_l1 = _initmem_pt_get();
			_initmem_pte_write(pt_l2, L2_OFFSET(page),
					   pt_l1 | L2_PROT, 2);
			prev_l2_offset = L2_OFFSET(page);
		}

		*((unsigned long *) pt_l1 + L1_OFFSET(page)) = frame | L1_PROT;
	}

	return pt_l4;
}

void uk_pt_init(unsigned long pt_area_start, unsigned long paddr_start,
		unsigned long len)
{
	/*
	 * The needed bookkeeping internal structures are:
	 * - a physical address bitmap, to keep track of all available physical
	 *   addresses (which will have a bit for every frame, so the size
	 *   phys_mem_length / PAGE_SIZE)
	 * - a memory area where page tables are stored
	 * - a bitmap for pages used as page tables
	 */
	phys_mem_start_addr = paddr_start;
	phys_mem_length = len;

	if (paddr_start == (unsigned long) -1) {
		phys_mem_length -= PAGETABLES_AREA_SIZE;
		phys_mem_length -= KERNEL_AREA_SIZE;
	}

	/*
	 * Need to bookkeep |phys_mem_length| bytes of physical
	 * memory, starting from |phys_mem_start_addr|. This is the
	 * physical memory given by the hypervisor.
	 *
	 * In Xen's case, the bitmap keeps the pseudo-physical addresses, the
	 * translation to machine frames being done later.
	 */
	phys_bitmap_start_addr = pt_area_start
				 + _used_pts * PAGE_SIZE
				 - PAGETABLES_VIRT_OFFSET;
	phys_bitmap_length = phys_mem_length >> PAGE_SHIFT;
	uk_bitmap_zero((unsigned long *) phys_bitmap_start_addr,
			phys_bitmap_length);

	pt_mem_start_addr =
		PAGE_ALIGN(phys_bitmap_start_addr + phys_bitmap_length);
	pt_mem_length = ((PAGETABLES_AREA_SIZE
			  - phys_bitmap_length
			  - _used_pts * PAGE_SIZE)
			 * PAGE_SIZE / (PAGE_SIZE + 1));
	pt_mem_length = PAGE_ALIGN_DOWN(pt_mem_length);

	/* Bookkeeping free pages used for PT allocations */
	pt_bitmap_start_addr =
		PAGE_ALIGN(pt_mem_start_addr + pt_mem_length);
	pt_bitmap_length = pt_mem_length >> PAGE_SHIFT;
	uk_bitmap_zero((unsigned long *) pt_bitmap_start_addr, pt_bitmap_length);

	/*
	 * If no specific area is given to be managed, the remaining memory is
	 * considered the actual usable memory.
	 */
	if (paddr_start == (unsigned long) -1) {
		phys_mem_start_addr =
		    PAGE_ALIGN(pt_bitmap_start_addr + pt_bitmap_length);
	}
}

static int _mmap_kernel(unsigned long pt_base,
			unsigned long kernel_start_vaddr,
			unsigned long kernel_start_paddr,
			unsigned long kernel_area_size)
{
	unsigned long kernel_pages;
	unsigned long page, frame;
	size_t i;

	UK_ASSERT(PAGE_ALIGNED(kernel_start_vaddr));
	UK_ASSERT(PAGE_ALIGNED(kernel_start_paddr));

#ifdef CONFIG_PLAT_KVM
	unsigned long mbinfo_pages, vgabuffer_pages;

	mbinfo_pages = DIV_ROUND_UP(MBINFO_AREA_SIZE, PAGE_SIZE);
	vgabuffer_pages = DIV_ROUND_UP(VGABUFFER_AREA_SIZE, PAGE_SIZE);

	for (i = 0; i < mbinfo_pages; i++) {
		page = MBINFO_AREA_START + i * PAGE_SIZE;

		if (_initmem_page_map(pt_base, page, page, PAGE_PROT_READ, 0))
			return -1;
	}

	for (i = 0; i < vgabuffer_pages; i++) {
		page = VGABUFFER_AREA_START + i * PAGE_SIZE;

		if (_initmem_page_map(pt_base, page, page,
				      PAGE_PROT_READ | PAGE_PROT_WRITE, 0))
			return -1;
	}
#endif /* CONFIG_PLAT_KVM */

	kernel_pages = DIV_ROUND_UP(kernel_area_size, PAGE_SIZE);

	for (i = 0; i < kernel_pages; i++) {
		page = kernel_start_vaddr + i * PAGE_SIZE;
		frame = kernel_start_paddr + i * PAGE_SIZE;

		/* TODO: break down into RW regions and RX regions */
		if (_initmem_page_map(pt_base, page, frame,
				      PAGE_PROT_READ
				      | PAGE_PROT_WRITE
				      | PAGE_PROT_EXEC,
				      0))
			return -1;
	}

	/*
	 * It is safe to return from this function, since we are still on the
	 * bootstrap stack, which is in the bss section, in the binary.
	 * The switch to another stack is done later.
	 */
	return 0;
}

void uk_pt_build(unsigned long heap_len, unsigned long paddr_start,
		 unsigned long len)
{
	size_t i;
	unsigned long stack_pages, heap_pages, heap_large_pages;
	unsigned long pt_base;
	unsigned long page;

	UK_ASSERT(PAGE_ALIGNED(paddr_start));
	UK_ASSERT(PAGE_ALIGNED(len));

	pt_base = _pt_create(PAGETABLES_AREA_START, PAGETABLES_AREA_SIZE,
			     PAGETABLES_VIRT_OFFSET);
	uk_pt_init(PAGETABLES_AREA_START, (unsigned long) -1, len);
	if (_mmap_kernel(pt_base, KERNEL_AREA_START, paddr_start,
			 KERNEL_AREA_SIZE))
		UK_CRASH("Could not map kernel\n");

#ifdef CONFIG_PARAVIRT
	/*
	 * TODO: Change protections of page tables to read-only before setting
	 * the new pt base.
	 */
#endif
	ukarch_write_pt_base(pt_base);
	_virt_offset = PAGETABLES_VIRT_OFFSET;
	phys_bitmap_start_addr += _virt_offset;
	pt_bitmap_start_addr += _virt_offset;

#ifdef CONFIG_PARAVIRT
	heap_pages = heap_len >> PAGE_SHIFT;
	for (i = 0; i < heap_pages; i++) {
		page = HEAP_AREA_START + i * PAGE_SIZE;

		if (uk_page_map(page, -1, PAGE_PROT_READ | PAGE_PROT_WRITE, 0))
			uk_pr_err("Error mapping heap page 0x%08lx\n", page);
	}
#else
	/* Map heap in large and regular pages */
	heap_large_pages = heap_len / PAGE_LARGE_SIZE;
	for (i = 0; i < heap_large_pages; i++)
		if (uk_page_map(HEAP_AREA_START + i * PAGE_LARGE_SIZE, -1,
				PAGE_PROT_READ | PAGE_PROT_WRITE,
				PAGE_FLAG_LARGE))
			uk_pr_err("Error mapping heap large page 0x%08lx\n",
					HEAP_AREA_START + i * PAGE_LARGE_SIZE);

	/*
	 * If the heap is not properly aligned to PAGE_LARGE_SIZE,
	 * map the rest in regular pages
	 */
	if (heap_large_pages * PAGE_LARGE_SIZE < heap_len) {
		heap_pages = (heap_len - heap_large_pages * PAGE_LARGE_SIZE)
				>> PAGE_SHIFT;
	} else {
		heap_pages = 0;
	}
	for (i = 0; i < heap_pages; i++) {
		page = HEAP_AREA_START + heap_large_pages * PAGE_LARGE_SIZE
			+ i * PAGE_SIZE;

		if (uk_page_map(page, -1, PAGE_PROT_READ | PAGE_PROT_WRITE, 0))
			uk_pr_err("Error mapping heap page 0x%08lx\n", page);
	}
#endif /* CONFIG_PARAVIRT */

	/* Map stack in regular pages */
	stack_pages = STACK_AREA_SIZE >> PAGE_SHIFT;
	for (i = 0; i < stack_pages; i++) {
		if (uk_page_map(STACK_AREA_START + i * PAGE_SIZE, -1,
				PAGE_PROT_READ | PAGE_PROT_WRITE, 0))
			uk_pr_err("Error mapping stack page 0x%08lx\n",
				  (unsigned long) STACK_AREA_START
				  + i * PAGE_SIZE);
	}
}
