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

#define PAGE_PROT_NONE	0x0
#define PAGE_PROT_READ	0x1
#define PAGE_PROT_WRITE 0x2
#define PAGE_PROT_EXEC	0x4

#define PAGE_INVALID	((unsigned long) -1)
#define PAGE_NOT_MAPPED 0

extern unsigned long phys_bitmap_start_addr;
extern unsigned long phys_bitmap_length;

extern unsigned long bitmap_start_addr;
extern unsigned long bitmap_length;

extern unsigned long internal_pt_start_addr;
extern unsigned long internal_pt_length;

extern unsigned long allocatable_memory_start_addr;
extern unsigned long allocatable_memory_length;

#ifdef CONFIG_PARAVIRT
#include <uk/asm/mm_pv.h>
#else
#include <uk/asm/mm.h>
#endif	/* CONFIG_PARAVIRT */

static inline unsigned long uk_allocatable_memory_start_addr(void)
{
	return allocatable_memory_start_addr;
}

unsigned long uk_virt_to_l1_pte(unsigned long vaddr);

int uk_pt_init(unsigned long paddr_start, size_t len);

#endif /* __UKPLAT_MM__ */

