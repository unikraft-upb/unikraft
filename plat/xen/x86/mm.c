/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Marc Rittinghaus <marc.rittinghaus@kit.edu>
 *
 * Copyright (c) 2022, Karlsruhe Institute of Technology (KIT).
 *                     All rights reserved.
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

#include <uk/essentials.h>
#include <uk/config.h>
#include <uk/arch/types.h>
#include <uk/arch/paging.h>
#include <uk/assert.h>
#include <uk/print.h>
#include <uk/event.h>

#include <uk/plat/common/sections.h>

#include <string.h>
#include <common/hypervisor.h>
#include <xen-x86/mm.h>
#include <xen-x86/mm_pv.h>
#include <xen/xen.h>
#ifdef CONFIG_XEN_GNTTAB
#include <xen/grant_table.h>
#endif

#ifndef CONFIG_PARAVIRT
#error Static page table setup code only supports paravirt mode
#endif

/* The static default page table should map the first range of RAM according
 * to the following definitions. We reserve space in the .bss section for the
 * page tables so that the memory is accessible when entering boot from Xen
 */
#define MiB (1024 * 1024)
#define XPG_MAX_STATIC_MEMORY		(CONFIG_XEN_MAX_STATIC_MEMORY * MiB)
#define XPG_MAX_PAGES			(XPG_MAX_STATIC_MEMORY >> PAGE_SHIFT)
#define XPG_PREMAP_MEMORY		(CONFIG_XEN_PREMAP_MEMORY * MiB)

#if XPG_MAX_STATIC_MEMORY < XPG_PREMAP_MEMORY
#error Premap memory larger than total memory
#endif

#ifdef CONFIG_XEN_GNTTAB
#define XPG_TOTAL_PT_PAGES					\
	(PT_PAGES(XPG_MAX_PAGES) + PT_LEVELS - 1)

#define XPG_GRANT_BASE			0xffff900000000000

#if CONFIG_XEN_GNTTAB_FRAMES > PT_Lx_PTES(PAGE_LEVEL)
#error Too many grant table frames
#endif

grant_entry_v1_t *gnttab_arch_init(int grant_frames_num __maybe_unused)
{
	UK_ASSERT(grant_frames_num == CONFIG_XEN_GNTTAB_FRAMES);

	return (grant_entry_v1_t*)(XPG_GRANT_BASE);
}

#else /* CONFIG_XEN_GNTTAB */
#define XPG_TOTAL_PT_PAGES		PT_PAGES(XPG_MAX_PAGES)
#endif /* !CONFIG_XEN_GNTTAB */

static char __align(PAGE_SIZE) xpg_boot_pt[XPG_TOTAL_PT_PAGES][PAGE_SIZE];

/* We need space for batching the MMU updates when setting xpg_boot_pt to
 * readonly. Since hypercalls can be extremely expensive, we rather waste this
 * memory than performing multiple calls
 */
static mmu_update_t mmu_updates[XPG_TOTAL_PT_PAGES];

/* TODO: Comment */
unsigned long *phys_to_machine_mapping;
static unsigned long xpg_max_pfn;

#if 0
void xpg_dump_pt(__vaddr_t pt_vaddr, __vaddr_t vaddr, unsigned int lvl,
		 int start, unsigned int flags)
{
	__pte_t *pte = (__pte_t*)pt_vaddr;
	unsigned long mfn;

	uk_pr_err("%lx\n", pt_vaddr);
	for (int i = start; i < 512; i++) {
		__vaddr_t a = vaddr + i * PAGE_Lx_SIZE(lvl);
		a = X86_VADDR_CANONICALIZE(a);
		if (pte[i] != 0 && ((pte[i] & flags) == 0)) {
			mfn = PT_Lx_PTE_PADDR(pte[i], lvl) >> PAGE_SHIFT;

			uk_pr_err("%lx %d:%lx -> mfn:%lx pfn:%lx "
				  "[%c%c%c%c%c%c%c]\n",
				  a, i, pte[i], mfn, mfn_to_pfn(mfn),
				  PT_Lx_PTE_PRESENT(pte[i], lvl) ? 'P' : '-',
				  pte[i] & X86_PTE_RW ? 'W' : 'R',
				  pte[i] & X86_PTE_US ? 'U' : 'K',
				  pte[i] & X86_PTE_NX ? '-' : 'X',
				  pte[i] & X86_PTE_PSE ? 'H' : '-',
				  pte[i] & X86_PTE_ACCESSED ? 'A' : '-',
				  pte[i] & X86_PTE_DIRTY ? 'D' : '-');
		}
	}
}
#endif

static inline __vaddr_t xpg_pt_get(unsigned int pt_idx)
{
	UK_ASSERT(pt_idx < XPG_TOTAL_PT_PAGES);

	return (__vaddr_t)(&xpg_boot_pt[pt_idx][0]);
}

static inline __pte_t xpg_pte_create(unsigned long mfn)
{
	__pte_t pte;

	pte = (mfn << PAGE_SHIFT) & X86_PTE_PADDR_MASK;

	/* Since we effectively run in user mode, all pages must be marked as
	 * user pages. As we do not evaluate access and dirty bits, we set them
	 * so the CPU does have to do it later on.
	 */
	pte |= (X86_PTE_PRESENT | X86_PTE_ACCESSED | X86_PTE_DIRTY);
	pte |= (X86_PTE_RW | X86_PTE_US);

	return pte;
}

static inline __vaddr_t xpg_pt_pte_to_vaddr(__pte_t pte,
					    unsigned int lvl __maybe_unused)
{
	return (__vaddr_t)mfn_to_virt(PT_Lx_PTE_PADDR(pte, lvl) >> PAGE_SHIFT);
}

static inline __pte_t xpg_pte_read(__vaddr_t pt_vaddr,
				   unsigned int lvl __maybe_unused,
				   unsigned int idx)
{
	UK_ASSERT(idx < PT_Lx_PTES(lvl));

	return *((__pte_t *)pt_vaddr + idx);
}

static inline void xpg_pte_write(__vaddr_t pt_vaddr,
				 unsigned int lvl __maybe_unused,
				 unsigned int idx, __pte_t pte)
{
	UK_ASSERT(idx < PT_Lx_PTES(lvl));

	*((__pte_t *)pt_vaddr + idx) = pte;
}

static inline int xpg_must_be_ro(__vaddr_t vaddr __unused)
{
/*	if (vaddr >= __TEXT && vaddr < __ETEXT)
		return 1;
*/
/*	if (vaddr >= __RODATA && vaddr < __ERODATA)
		return 1;
*/
	return 0;
}

void xpg_setup(unsigned long * restrict start_pfn,
	       unsigned long * restrict max_pfn)
{
	unsigned int lvl = PT_LEVELS - 1;
	__vaddr_t vaddr, vaddr_end, pt_svaddr, pt_dvaddr;
	__vaddr_t vaddr_mend;
	__vaddr_t pt_svaddr_cache[PT_LEVELS];
	__vaddr_t pt_dvaddr_cache[PT_LEVELS];
	__pte_t pte;
	unsigned int pte_idx;
	unsigned int pt_idx = 0;
	int rc, count = 0;
	union {
		mmuext_op_t mmu;
#ifdef CONFIG_XEN_GNTTAB
		gnttab_setup_table_t gnt;
#endif /* CONFIG_XEN_GNTTAB */
	} xen_op;
#ifdef CONFIG_XEN_GNTTAB
	unsigned long gnt_frames[CONFIG_XEN_GNTTAB_FRAMES];
#endif /* CONFIG_XEN_GNTTAB */

	phys_to_machine_mapping =
		(unsigned long *)HYPERVISOR_start_info->mfn_list;

	/* If available memory exceeds the configured maximum trim it down */
	*max_pfn = MIN(*max_pfn, XPG_MAX_PAGES);
	xpg_max_pfn = *max_pfn;

	vaddr = (__vaddr_t)pfn_to_virt(0);
	vaddr_end = (__vaddr_t)pfn_to_virt(*max_pfn);

	/* Since switching to the new page table is very expensive in
	 * paravirtual mode as Xen has to verify all PTEs, we actually only
	 * map pages until vaddr_mend and do the rest on demand. However, we
	 * already create the full page table hierarchy because otherwise we
	 * potentially need multiple hypercalls later to map a single page.
	 */
	vaddr_mend = (*start_pfn << PAGE_SHIFT) + XPG_PREMAP_MEMORY;

	/* We worked out the virtual memory range to map */
	uk_pr_info("Mapping memory range 0x%lx - 0x%lx\n", vaddr, vaddr_end);

	/* Grab the current top-level directory */
	pt_svaddr = HYPERVISOR_start_info->pt_base;
	pt_svaddr_cache[lvl] = pt_svaddr;

	/* Allocate a top-level page directory */
	pt_dvaddr = xpg_pt_get(pt_idx++);
	pt_dvaddr_cache[lvl] = pt_dvaddr;

	pte_idx = PT_Lx_IDX(vaddr, lvl);
	do {
		UK_ASSERT(vaddr_end >= vaddr);
		UK_ASSERT(vaddr_end - vaddr >= PAGE_SIZE);
		UK_ASSERT(PAGE_ALIGNED(vaddr_end - vaddr));

		/* If we are not at the page level, walk down the active page
		 * table and also the new one. Allocate and link page tables
		 * in the new one on the way.
		 */
		while (lvl > PAGE_LEVEL) {
			/* If the source page table does not map this region
			 * do not try to walk it further. We interpret a page
			 * table address of 0 to be invalid.
			 */
			if (pt_svaddr) {
				pte = xpg_pte_read(pt_svaddr, lvl, pte_idx);

				pt_svaddr = PT_Lx_PTE_PRESENT(pte, lvl) ?
					xpg_pt_pte_to_vaddr(pte, lvl) : 0;
			}

			pt_dvaddr = xpg_pt_get(pt_idx++);

			pte = xpg_pte_create(virt_to_mfn(pt_dvaddr));
			xpg_pte_write(pt_dvaddr_cache[lvl], lvl, pte_idx, pte);

			lvl--;
			pt_dvaddr_cache[lvl] = pt_dvaddr;
			pt_svaddr_cache[lvl] = pt_svaddr;

			pte_idx = PT_Lx_IDX(vaddr, lvl);
		}

		UK_ASSERT(lvl == PAGE_LEVEL);
		UK_ASSERT(PAGE_ALIGNED(vaddr));

		/* If we have a valid source page table, copy the PTE over to
		 * the new page table. Otherwise, create a new one.
		 */
		if (pt_svaddr) {
			pte = xpg_pte_read(pt_svaddr, lvl, pte_idx);

			if (vaddr >= xpg_pt_get(0) &&
			    vaddr <= xpg_pt_get(XPG_TOTAL_PT_PAGES - 1)) {
				/* We have to mark the pages read-only in both
				 * page tables that cover the new page tables
				 */
				pte &= ~X86_PTE_RW;

				mmu_updates[count].ptr = virt_to_mach(
					pt_svaddr + pte_idx * sizeof(__pte_t));
				mmu_updates[count].val = pte;
				count++;
			} else if (xpg_must_be_ro(vaddr)) {
				/* The page should be read-only in the new page
				 * even if it is not marked read-only in the
				 * active page table. However, the active one
				 * does not need to be changed
				 */
				pte &= ~X86_PTE_RW;
			}

			UK_ASSERT(vaddr < vaddr_mend);
		} else {
			pte = xpg_pte_create(virt_to_mfn(vaddr));

			if (vaddr >= vaddr_mend)
				pte &= ~X86_PTE_PRESENT;
		}

		xpg_pte_write(pt_dvaddr, PAGE_LEVEL, pte_idx, pte);

		if (vaddr == vaddr_end - PAGE_SIZE)
			break;

		/* We need to map more pages. If we have reached the last PTE
		 * in this page table, we have to walk up again until we reach
		 * a page table where this is not the last PTE. We then walk
		 * down to the page level again.
		 */
		if (pte_idx == PT_Lx_PTES(lvl) - 1) {
			do {
				UK_ASSERT(lvl < PT_LEVELS);

				/* Go up one level */
				pte_idx = PT_Lx_IDX(vaddr, ++lvl);
				UK_ASSERT(pte_idx < PT_Lx_PTES(lvl));
			} while (pte_idx == PT_Lx_PTES(lvl) - 1);

			pt_dvaddr = pt_dvaddr_cache[lvl];
			pt_svaddr = pt_svaddr_cache[lvl];
		}

		UK_ASSERT(vaddr <= __VADDR_MAX - PAGE_SIZE);
		vaddr += PAGE_SIZE;

		UK_ASSERT(pte_idx < PT_Lx_PTES(lvl) - 1);
		pte_idx++;
	} while (1);

	UK_ASSERT(count == XPG_TOTAL_PT_PAGES);

#ifdef CONFIG_XEN_GNTTAB
	UK_ASSERT(pt_idx == PT_PAGES(XPG_MAX_PAGES));

	/* We also need to setup and map the grant tables at XPG_GRANT_BASE */
	xen_op.gnt.dom = DOMID_SELF;
	xen_op.gnt.nr_frames = CONFIG_XEN_GNTTAB_FRAMES;
	set_xen_guest_handle(xen_op.gnt.frame_list, gnt_frames);

	rc = HYPERVISOR_grant_table_op(GNTTABOP_setup_table, &xen_op.gnt, 1);
	if (unlikely(rc || xen_op.gnt.status != GNTST_okay))
		UK_CRASH("Failed to allocate grant tables.\n");

	vaddr = XPG_GRANT_BASE;
	pt_dvaddr = xpg_pt_get(0);

	lvl = PT_LEVELS - 1;
	pte_idx = PT_Lx_IDX(vaddr, lvl);
	while (lvl > PAGE_LEVEL) {
		pt_svaddr = xpg_pt_get(pt_idx++);

		pte = xpg_pte_create(virt_to_mfn(pt_svaddr));
		xpg_pte_write(pt_dvaddr, lvl, pte_idx, pte);

		lvl--;
		pt_dvaddr = pt_svaddr;

		pte_idx = PT_Lx_IDX(vaddr, lvl);
	}

	UK_ASSERT(pte_idx == 0);
	UK_ASSERT(lvl == PAGE_LEVEL);

	for (; pte_idx < CONFIG_XEN_GNTTAB_FRAMES; pte_idx++) {
		pte = xpg_pte_create(gnt_frames[pte_idx]);
		xpg_pte_write(pt_dvaddr, PAGE_LEVEL, pte_idx, pte);
	}
#endif /* CONFIG_XEN_GNTTAB */

	/* Perform hypercall to mark the new page tables as read-only in the
	 * active page table
	 */
	UK_ASSERT(pt_idx == XPG_TOTAL_PT_PAGES);

	rc = HYPERVISOR_mmu_update(mmu_updates, count, NULL, DOMID_SELF);
	if (unlikely(rc))
		UK_CRASH("Failed to mark page table pages as read-only.\n");

	/* Set the new base pointer. Note that we do not pin the page table */
	xen_op.mmu.cmd = MMUEXT_NEW_BASEPTR;
	xen_op.mmu.arg1.mfn = virt_to_mfn(xpg_pt_get(0));

	rc = HYPERVISOR_mmuext_op(&xen_op.mmu, 1, NULL, DOMID_SELF);
	if (unlikely(rc))
		UK_CRASH("Failed to set static page table.\n");
}

inline __pte_t xpg_walk_pt(__vaddr_t vaddr, __vaddr_t * restrict pt_vaddr)
{
	unsigned int pte_idx, lvl = PT_LEVELS - 1;
	__pte_t pte;

	*pt_vaddr = xpg_pt_get(0);

	pte_idx = PT_Lx_IDX(vaddr, lvl);
	while (lvl > PAGE_LEVEL) {
		pte = xpg_pte_read(*pt_vaddr, lvl, pte_idx);

		UK_ASSERT(PT_Lx_PTE_PRESENT(pte, lvl));

		*pt_vaddr = xpg_pt_pte_to_vaddr(pte, lvl);

		lvl--;
		pte_idx = PT_Lx_IDX(vaddr, lvl);
	}

	return xpg_pte_read(*pt_vaddr, PAGE_LEVEL, pte_idx);
}

static int xpg_make_present(__vaddr_t vaddr)
{
	unsigned int pte_idx;
	__vaddr_t pt_vaddr;
	__pte_t pte;
	mmu_update_t mmu_update;
	int rc;

	if (unlikely(vaddr >= XPG_MAX_STATIC_MEMORY))
		return -1;

	/* The page tables are already set up. Just walk down the hierarchy
	 * and enable the present bit for the respective page
	 */
	pte = xpg_walk_pt(vaddr, &pt_vaddr);

	UK_ASSERT(pte != 0);
	UK_ASSERT(!PT_Lx_PTE_PRESENT(pte, lvl));

	pte |= X86_PTE_PRESENT;

	pte_idx = PT_Lx_IDX(vaddr, PAGE_LEVEL);
	mmu_update.ptr = virt_to_mach(pt_vaddr + pte_idx * sizeof(__pte_t));
	mmu_update.val = pte;

	rc = HYPERVISOR_mmu_update(&mmu_update, 1, NULL, DOMID_SELF);
	if (unlikely(rc))
		return -1;

	return 0;
}

static int xpg_mem_fault_handler(void *data)
{
	struct ukarch_trap_ctx *ctx = (struct ukarch_trap_ctx *)data;

	if (xpg_make_present(ctx->fault_address))
		return UK_EVENT_NOT_HANDLED;

	return UK_EVENT_HANDLED;
}

UK_EVENT_HANDLER(UKARCH_TRAP_PAGE_FAULT, xpg_mem_fault_handler);

#ifdef CONFIG_XEN_PV_BUILD_P2M
#include <uk/alloc.h>

static unsigned long *l3_list;
static unsigned long *l2_list_pages[P2M_ENTRIES];

void _arch_init_p2m(struct uk_alloc *a)
{
	unsigned long pfn;
	unsigned long *l2_list = NULL;

	if (((xpg_max_pfn - 1) >> L3_P2M_SHIFT) > 0)
		UK_CRASH("Too many pfns.\n");

	l3_list = uk_palloc(a, 1);
	for (pfn = 0; pfn < xpg_max_pfn; pfn += P2M_ENTRIES) {
		if (!(pfn % (P2M_ENTRIES * P2M_ENTRIES))) {
			l2_list = uk_palloc(a, 1);
			l3_list[L3_P2M_IDX(pfn)] = virt_to_mfn(l2_list);
			l2_list_pages[L3_P2M_IDX(pfn)] = l2_list;
		}

		l2_list[L2_P2M_IDX(pfn)] =
			virt_to_mfn(phys_to_machine_mapping + pfn);
	}
	HYPERVISOR_shared_info->arch.pfn_to_mfn_frame_list_list =
		virt_to_mfn(l3_list);
	HYPERVISOR_shared_info->arch.max_pfn = xpg_max_pfn;
}
#endif /* CONFIG_XEN_PV_BUILD_P2M */

void arch_mm_init(struct uk_alloc *a)
{
#ifdef CONFIG_XEN_PV_BUILD_P2M
	_arch_init_p2m(a);
#endif /* CONFIG_XEN_PV_BUILD_P2M */
}
