/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Assertions
 *
 * Authors: Simon Kuenzer <simon.kuenzer@neclab.eu>
 *
 *
 * Copyright (c) 2017, NEC Europe Ltd., NEC Corporation. All rights reserved.
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

#ifndef __UKDEBUG_SYMBOL_H__
#define __UKDEBUG_SYMBOL_H__

#include <uk/arch/types.h>
#include <uk/essentials.h>

#ifdef __cplusplus
extern "C" {
#endif

struct uk_symbol {
	const char *name; /* Null-terminated */
	__sz name_len;
	unsigned long addr;
};

#ifdef CONFIG_DEBUG_EMBED_SYMBOLS
/**
 * Returns the nearest symbol below the supplied address.
 *
 * @param addr
 *   Address to perform a symbol lookup for
 * @param sym
 *   Receives the information on the found symbol.
 * @return
 *   1 if a symbol could be found, 0 otherwise.
 */
int uk_resolve_symbol(unsigned long addr, struct uk_symbol *sym);
#else
static int uk_resolve_symbol(unsigned long addr __unused,
		struct uk_symbol *sym __unused) {
	return 0;
}
#endif /* CONFIG_DEBUG_EMBED_SYMBOLS */

#ifdef __cplusplus
}
#endif

#endif /* __UKDEBUG_SYMBOL_H__ */
