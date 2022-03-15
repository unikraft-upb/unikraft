/* SPDX-License-Identifier: MIT */
/*
 * (C) 2003 - Rolf Neugebauer - Intel Research Cambridge
 * Copyright (c) 2005, Keir A Fraser
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef _ARCH_MM_H_
#define _ARCH_MM_H_

#include <uk/arch/paging.h>
#include <uk/plat/common/sections.h>

#ifndef __ASSEMBLY__
#include <xen/xen.h>
#if defined(__x86_64__)
#include <xen/arch-x86_64.h>
#ifdef CONFIG_PARAVIRT
#include <xen-x86/mm_pv.h>
#endif
#else
#error "Unsupported architecture"
#endif
#else
#define PTE(val) .quad val
#endif

#define PFN_UP(x)		(((x) + PAGE_SIZE-1) >> PAGE_SHIFT)

#ifndef __ASSEMBLY__
typedef __paddr_t __maddr_t;

extern unsigned long *phys_to_machine_mapping;

static inline __maddr_t _paddr_to_maddr(__paddr_t paddr)
{
	__maddr_t maddr = pfn_to_mfn(paddr >> PAGE_SHIFT);
	maddr = (maddr << PAGE_SHIFT) | (paddr & ~PAGE_MASK);
	return maddr;
}

static inline __paddr_t _maddr_to_paddr(__maddr_t maddr)
{
	__paddr_t phys = mfn_to_pfn(maddr >> PAGE_SHIFT);
	phys = (phys << PAGE_SHIFT) | (maddr & ~PAGE_MASK);
	return phys;
}

#define VIRT_START		((unsigned long)(__TEXT))

#define to_phys(x)		((unsigned long)(x) - VIRT_START)
#define to_virt(x)		((void *)((unsigned long)(x) + VIRT_START))

#define virt_to_pfn(_virt)	(to_phys(_virt) >> PAGE_SHIFT)
#define virt_to_mfn(_virt)	(pfn_to_mfn(virt_to_pfn(_virt)))
#define mach_to_virt(_mach)	(to_virt(_maddr_to_paddr(_mach)))
#define virt_to_mach(_virt)	(_paddr_to_maddr(to_phys(_virt)))
#define mfn_to_virt(_mfn)	(to_virt(mfn_to_pfn(_mfn) << PAGE_SHIFT))
#define pfn_to_virt(_pfn)	(to_virt((_pfn) << PAGE_SHIFT))

#define pte_to_mfn(_pte)	(((_pte) & X86_PTE_PADDR_MASK) >> PAGE_SHIFT)

/**
 * TODO: comment
 */
void xpg_setup(unsigned long * restrict start_pfn,
	       unsigned long * restrict max_pfn);

__pte_t xpg_walk_pt(__vaddr_t vaddr, __vaddr_t * restrict pt_vaddr);

/**
 * TODO: comment
 */
static inline unsigned long xpg_virt_to_mfn(__vaddr_t vaddr)
{
	__vaddr_t pt_vaddr;

	return pte_to_mfn(xpg_walk_pt(vaddr, &pt_vaddr));
}
#endif /* !__ASSEMBLY */
#endif /* _ARCH_MM_H_ */
