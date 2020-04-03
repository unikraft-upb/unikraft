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

unsigned long phys_bitmap_start_addr;
unsigned long phys_bitmap_length;

unsigned long bitmap_start_addr;
unsigned long bitmap_length;

unsigned long internal_pt_start_addr;
unsigned long internal_pt_length;

unsigned long allocatable_memory_start_addr;
unsigned long allocatable_memory_length;

unsigned long uk_virt_to_l1_pte(unsigned long vaddr)
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

int uk_pt_init(unsigned long paddr_start, size_t len)
{
	/*
	 * The needed bookkeeping internal structures are:
	 * - a physical address bitmap, to keep track of all available physical
	 *   addresses (which will have a bit for every frame, so the size
	 *   allocatable_memory_length / PAGE_SIZE)
	 * - a memory area where page tables are stored
	 * - a bitmap for pages used as page tables
	 *
	 * In order to obtain the size of each structure, the following equation
	 * must be solved:
	 * size(phys_bitmap) + size(pt_bitmap) + size(pt_area)
	 * + size(allocatable_memory) == len
	 */
	double internal_pt_const = 1.0 / L1_PAGETABLE_ENTRIES
		+ 1.0 / (L1_PAGETABLE_ENTRIES * L2_PAGETABLE_ENTRIES)
		+ 1.0 / (L1_PAGETABLE_ENTRIES * L2_PAGETABLE_ENTRIES * L3_PAGETABLE_ENTRIES);

	double allocatable_memory_constant = 1.0 /
		(1 + internal_pt_const + 1.0 / PAGE_SIZE + internal_pt_const / PAGE_SIZE);

	UK_ASSERT(PAGE_ALIGNED(paddr_start));
	UK_ASSERT(PAGE_ALIGNED(len));

	/*
	 * |allocatable_memory_length| is the length of the remaining memory,
	 * after allocating the necessary bitmaps.
	 */
	allocatable_memory_length = len * allocatable_memory_constant;
	allocatable_memory_length = PAGE_ALIGN_DOWN(allocatable_memory_length);

	/*
	 * Need to bookkeep |allocatable_memory_length| bytes of physical
	 * memory, starting from |allocatable_memory_start_addr|. This is the
	 * physical memory given by the hypervisor.
	 *
	 * In Xen's case, the bitmap keeps the pseudo-physical addresses, the
	 * translation to machine frames being done later.
	 */
	phys_bitmap_start_addr = paddr_start;
	phys_bitmap_length = allocatable_memory_length >> PAGE_SHIFT;
	uk_bitmap_zero((unsigned long *) phys_bitmap_start_addr,
			phys_bitmap_length);

	internal_pt_start_addr =
		PAGE_ALIGN(phys_bitmap_start_addr + phys_bitmap_length);
	internal_pt_length = PAGE_ALIGN((unsigned long)
			(allocatable_memory_length * internal_pt_const));
	memset((void *) internal_pt_start_addr, 0, internal_pt_length);

	/* Bookkeeping free pages used for PT allocations */
	bitmap_start_addr =
		PAGE_ALIGN(internal_pt_start_addr + internal_pt_length);
	bitmap_length = internal_pt_length >> PAGE_SHIFT;
	uk_bitmap_zero((unsigned long *) bitmap_start_addr, bitmap_length);

	/* The remaining memory is the actual usable memory */
	allocatable_memory_start_addr =
	    PAGE_ALIGN(bitmap_start_addr + bitmap_length);

	if (allocatable_memory_start_addr + allocatable_memory_length > paddr_start + len)
		allocatable_memory_length -=
			allocatable_memory_start_addr
			+ allocatable_memory_length
			- paddr_start - len;

	return 0;
}
