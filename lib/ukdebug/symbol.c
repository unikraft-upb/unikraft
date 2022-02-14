/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Author(s): Marc Rittinghaus <marc.rittinghaus@kit.edu>
 *
 * Copyright (c) 2021, Karlsruhe Institute of Technology. All rights reserved.
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

#include <uk/symbol.h>
#include <uk/essentials.h>
#include <uk/assert.h>

#include <stddef.h>

/* Linked to respective tables in the .uk_symtab section */
extern const int _uk_symtab_addrs_start[] __weak;
extern const int _uk_symtab_addrs_end[] __weak;

extern const char _uk_symtab_names_start[] __weak;
extern const char _uk_symtab_names_end[] __weak;

#define NUM_SYMBOLS (_uk_symtab_addrs_end - _uk_symtab_addrs_start)
#define SYMBOL_ADDR(idx)					\
	(unsigned long)((__uptr)&_uk_symtab_addrs_start[idx] -	\
		_uk_symtab_addrs_start[idx])

static int sym_find_symbol(unsigned long addr)
{
	unsigned long sym_addr;
	int l, r, m;
	int best_match = -1;

	/* Perform binary search on the symbol table */
	l = 0;
	r = NUM_SYMBOLS - 1; /* If symbol table is empty, r will be -1 */
	while (l <= r) {
		m = (r - l) / 2 + l;
		sym_addr = SYMBOL_ADDR(m);

		if (addr == sym_addr) {
			return m;
		} else if (addr > sym_addr) {
			/* If this is an address in the symbol body
			 * this might already be the requested symbol.
			 */
			best_match = m;
			l = m + 1;
		} else {
			r = m - 1;
		}
	}

	return best_match;
}

static const char* sym_find_name(int symnr)
{
	const char *p = _uk_symtab_names_start;

	UK_ASSERT(symnr >= 0);

	/* Each string begins with a byte that indicates the string's length
	 * without null-terminator. We can thus simply read the length byte and
	 * skip over strings until we reached the supplied index.
	 */
	while (p < _uk_symtab_names_end) {
		if (symnr == 0) {
			return p;
		}

		p += *p + 2; /* + 1 length byte + 1 null-terminator */
		symnr--;
	}

	return NULL;
}

int uk_resolve_symbol(unsigned long addr, struct uk_symbol *sym)
{
	int s;

	UK_ASSERT(sym != NULL);

	s = sym_find_symbol(addr);
	if (s < 0) {
		return 0;
	}

	sym->addr = SYMBOL_ADDR(s);
	sym->name = sym_find_name(s);
	if (sym->name == NULL) {
		sym->name_len = 0;
	} else {
		sym->name_len = *sym->name++;
	}

	return 1;
}
