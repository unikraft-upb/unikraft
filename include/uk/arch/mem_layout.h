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

#ifndef __UKARCH_MEM_LAYOUT__
#define __UKARCH_MEM_LAYOUT__

#include <uk/sections.h>

/* These regions exist only for KVM and are mapped 1:1 */
#ifdef CONFIG_PLAT_KVM
#define VGABUFFER_AREA_START	0xb8000
#define VGABUFFER_AREA_END	0xc0000
#define VGABUFFER_AREA_SIZE	(VGABUFFER_AREA_END - VGABUFFER_AREA_START)

#define MBINFO_AREA_START	0x9000
#define MBINFO_AREA_END		0xa000
#define MBINFO_AREA_SIZE	(MBINFO_AREA_END - MBINFO_AREA_START)
#endif /* CONFIG_PLAT_KVM */

/* This has to be broken down further */
/* Here are the regions: Code + Data + BSS + Rodata etc. */
#define KERNEL_AREA_START	(1UL << 20) /* 1MB */
#define KERNEL_AREA_END		PAGE_LARGE_ALIGN(__END)
#define KERNEL_AREA_SIZE	(KERNEL_AREA_END - KERNEL_AREA_START)

#ifdef CONFIG_DYNAMIC_PT
#define PAGETABLES_VIRT_OFFSET	0x400000000
#else
#define PAGETABLES_VIRT_OFFSET	0x0
#endif /* CONFIG_DYNAMIC_PT */
#define PAGETABLES_AREA_START	(KERNEL_AREA_END + PAGETABLES_VIRT_OFFSET)
#define PAGETABLES_AREA_END	(PAGETABLES_AREA_START + 0x1000000) /* 16MB */
#define PAGETABLES_AREA_SIZE	(PAGETABLES_AREA_END - PAGETABLES_AREA_START)

#define STACK_AREA_END		(1UL << 47) /* 128TB */
#define STACK_AREA_START	(STACK_AREA_END - __STACK_SIZE)
#define STACK_AREA_SIZE		__STACK_SIZE

#define HEAP_AREA_START		(1UL << 32) /* 4GB */
#ifdef CONFIG_LIBPOSIX_MMAP
#define HEAP_AREA_END		(1UL << 45) /* 32TB */
#define HEAP_AREA_SIZE		(HEAP_AREA_END - HEAP_AREA_START)

#define MMAP_AREA_START		HEAP_AREA_END
#define MMAP_AREA_END		STACK_AREA_START
#define MMAP_AREA_SIZE		(MMAP_AREA_END - MMAP_AREA_START)
#else /* When we don't use mmap, heap is the rest of the memory */
#define HEAP_AREA_END		STACK_AREA_START
#define HEAP_AREA_SIZE		(HEAP_AREA_END - HEAP_AREA_START)

#define MMAP_AREA_START		0x0
#define MMAP_AREA_END		0x0
#define MMAP_AREA_SIZE		0x0
#endif /* CONFIG_LIBPOSIX_MMAP */

#endif /* __UKARCH_MEM_LAYOUT__ */

