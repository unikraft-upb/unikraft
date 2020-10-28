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

#ifndef __UKPLAT_MM__
#define __UKPLAT_MM__

#include <uk/config.h>

#ifndef CONFIG_PT_API
#error Using this header requires enabling the virtual memory management API
#endif /* CONFIG_PT_API */

#define PAGE_PROT_NONE	0x0
#define PAGE_PROT_READ	0x1
#define PAGE_PROT_WRITE 0x2
#define PAGE_PROT_EXEC	0x4

#define PAGE_FLAG_LARGE 0x1

#define PAGE_PADDR_ANY	((unsigned long) -1)

#define PAGE_INVALID	((unsigned long) -1)
#define PAGE_NOT_MAPPED 0

extern unsigned long phys_bitmap_start_addr;
extern unsigned long phys_bitmap_length;

extern unsigned long phys_mem_start_addr;
extern unsigned long phys_mem_length;

#ifdef CONFIG_PARAVIRT
#include <uk/asm/mm_pv.h>
#else
#include <uk/asm/mm_native.h>
#endif	/* CONFIG_PARAVIRT */

/**
 * Get a free frame in the physical memory where a new mapping can be created.
 *
 * @param flags: specify any criteria that the frame has to meet (e.g. a 2MB
 * frame for a large page). These are constructed by or'ing PAGE_FLAG_* flags.
 *
 * @return: physical address of an unused frame or PAGE_INVALID on failure.
 */
static inline unsigned long uk_get_next_free_frame(unsigned long flags)
{
	unsigned long offset;
	unsigned long pfn;
	unsigned long frame_size;

#ifdef CONFIG_PARAVIRT
	/*
	 * Large/Huge pages are not supported in PV guests on Xen.
	 * https://wiki.xenproject.org/wiki/Huge_Page_Support
	 */
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

	if (offset * PAGE_SIZE > phys_mem_length) {
		uk_pr_err("Out of physical memory\n");
		return PAGE_INVALID;
	}

	uk_bitmap_set((unsigned long *) phys_bitmap_start_addr, offset,
		      frame_size);

	pfn = (phys_mem_start_addr >> PAGE_SHIFT) + offset;

	return pfn_to_frame(pfn);
}

/**
 * Create a mapping from a virtual address to a physical address, with given
 * protections and flags.
 *
 * @param vaddr: the virtual address of the page that is to be mapped.
 * @param paddr: the physical address of the frame to which the virtual page
 * is mapped to. This parameter can be equal to PAGE_PADDR_ANY when the caller
 * is not interested in the physical address where the mapping is created.
 * @param prot: protection permissions of the page (obtained by or'ing
 * PAGE_PROT_* flags).
 * @param flags: flags of the page (obtained by or'ing PAGE_FLAG_* flags).
 *
 * @return: 0 on success and -1 on failure. The uk_page_map call can fail if:
 * - the given physical or virtual addresses are not aligned to page size;
 * - any page in the region is already mapped to another frame;
 * - if PAGE_PADDR_ANY flag is selected and there are no more available
 *   free frames in the physical memory;
 * - (on Xen PV) if flags contains PAGE_FLAG_LARGE - large pages are not
 *   supported on PV guests;
 * - (on Xen PV) the hypervisor rejected the mapping.
 *
 * In case of failure, the mapping is not created.
 */
int uk_page_map(unsigned long vaddr, unsigned long paddr, unsigned long prot,
		unsigned long flags);

/**
 * Create a mapping from a region starting at a virtual address to a physical
 * address, with given protections and flags.
 *
 * @param vaddr: the virtual address of the page where the region that is to be
 * mapped starts.
 * @param paddr: the physical address of the starting frame of the region to
 * which the virtual region is mapped to. This parameter can be equal to
 * PAGE_PADDR_ANY when the caller is not interested in the physical address
 * where the mappings are created.
 * @param prot: protection permissions of the pages (obtained by or'ing
 * PAGE_PROT_* flags).
 * @param flags: flags of the page (obtained by or'ing PAGE_FLAG_* flags).
 *
 * @return: 0 on success and -1 on failure. The uk_page_map call can fail if:
 * - the given physical or virtual addresses are not aligned to page size;
 * - any page in the region is already mapped to another frame;
 * - if PAGE_PADDR_ANY flag is selected and there are no more available
 *   free frames in the physical memory;
 * - (on Xen PV) if flags contains PAGE_FLAG_LARGE - large pages are not
 *   supported on PV guests;
 * - (on Xen PV) the hypervisor rejected any of the mappings.
 *
 * In case of failure, no new mapping is created.
 */
int uk_map_region(unsigned long vaddr, unsigned long paddr,
		unsigned long pages, unsigned long prot, unsigned long flags);

/**
 * Frees a mapping for a page.
 *
 * @param vaddr: the virtual address of the page that is to be unmapped.
 *
 * @return: 0 in case of success and -1 on failure. The call fails if:
 * - the given page is not mapped to any frame;
 * - the virtual address given is not aligned to page (simple/large/huge) size.
 * - (on Xen PV) the hypervisor rejected the unmapping.
 */
int uk_page_unmap(unsigned long vaddr);

/**
 * Sets new protections for a given page.
 *
 * @param vaddr: the virtual address of the page whose protections are updated.
 * @param new_prot: new protections that will be set to the page (obtained by
 * or'ing PAGE_PROT_* flags).
 *
 * @return: 0 in case of success and -1 on failure. The call fails if:
 * - the given page is not mapped to any frame;
 * - the virtual address given is not aligned to page (simple/large/huge) size.
 * - (on Xen PV) the hypervisor rejected the unmapping.
 */
int uk_page_set_prot(unsigned long vaddr, unsigned long new_prot);

/**
 * Return page table entry corresponding to given virtual address.
 * @param vaddr: the virtual address, aligned to the corresponding page
 * dimesion (simple, large or huge) size.
 * @return: page table entry (PTE) obtained by doing a page table walk.
 */
unsigned long uk_virt_to_pte(unsigned long vaddr);

/**
 * Initialize internal page table bookkeeping for using the PT API when
 * attaching to an existing page table.
 * @param pt_area_start: the virtual address of the area for page tables and
 * internal bookkeeping.
 * @param paddr_start: the physical address of the beginning of the area that
 * should be managed by the API.
 * @param len: the length of the (physical) memory area that should be managed.
 */
void uk_pt_init(unsigned long pt_area_start, unsigned long paddr_start,
		unsigned long len);

/**
 * Build page table structure from scratch
 * @param paddr_start: the first address in the usable physical memory.
 * @param len: the length (in bytes) of the physical memory that will be
 * managed by the API.
 *
 * This function builds a structure of page tables (by calling _pt_create),
 * initializes the page table API (by calling uk_pt_init), maps the kernel in
 * the virtual address space (with _mmap_kernel), switches to the new address
 * space and sets the _virt_offset variable.
 */
void uk_pt_build(unsigned long paddr_start, unsigned long len);

/**
 * Allocate a new stack and return address to its lower address.
 *
 * @return: the lower address of the stack. If the returned address is `addr`,
 * then the allocated stack region is [`addr`, `addr + __STACK_SIZE`]. The
 * maximum number of stacks that can be allocated denotes the maximum number
 * of threads that can co-exist. More details about the number of stacks in
 * include/uk/mem_layout.h. Returns NULL in case of failure.
 */
void *uk_stack_alloc();

/**
 * Frees a stack previously allocated with uk_stack_alloc().
 *
 * @param vaddr: the virtual address of the beginning of the stack (i.e. the
 * address returned by uk_stack_alloc()).
 *
 * @return: 0 in case of success and -1 on failure. The call can fail if:
 * - the given address is not a stack address previously returned by
 *   uk_stack_alloc (which is between STACK_AREA_BEGIN and STACK_AREA_END);
 * - the given address is not page aligned;
 * - (on Xen) the hypervisor rejected the unmapping.
 */
int uk_stack_free(void *vaddr);

#endif /* __UKPLAT_MM__ */
