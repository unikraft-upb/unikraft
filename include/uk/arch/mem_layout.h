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

#include <uk/plat/common/sections.h>

#define PAGE_SIZE_2MB 0x200000UL

/* This has to be broken down further and maybe placed somewhere else in memory */
/* Here are the regions: Code + Data + BSS + Rodata etc. */
#define KERNEL_AREA_START 	0x0UL
#define KERNEL_AREA_END		((__END + PAGE_SIZE_2MB) & (~(PAGE_SIZE_2MB - 1)))
#define KERNEL_AREA_SIZE 	(KERNEL_AREA_END - KERNEL_AREA_START)

#define PAGETABLES_AREA_START	KERNEL_AREA_END
#define PAGETABLES_AREA_END	(KERNEL_AREA_END + 0x4000000) /* 64MB size */
#define PAGETABLES_AREA_SIZE	(PAGETABLES_AREA_END - PAGETABLES_AREA_START)

/* Stack is present at the end of the 128TB area */
#define STACK_AREA_END 		0x800000000000
#define STACK_AREA_START 	(STACK_AREA_END - __STACK_SIZE)
#define STACK_AREA_SIZE 	__STACK_SIZE

/* Heap is the rest of the memory */
/* Maybe this should be broken down as well between heap and mmap area */
#define HEAP_AREA_START		PAGETABLES_AREA_END
#define HEAP_AREA_END		STACK_AREA_START
#define HEAP_AREA_SIZE		(HEAP_AREA_END - HEAP_AREA_START)

#endif /* __UKARCH_MEM_LAYOUT__ */

