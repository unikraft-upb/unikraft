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

