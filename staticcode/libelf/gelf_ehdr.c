/*-
 * Copyright (c) 2006,2008 Joseph Koshy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <assert.h>
#include <gelf.h>
#include <libelf.h>
#include <limits.h>
#include <stdint.h>
#include <string.h>

#include "_libelf.h"

ELFTC_VCSID("$Id: gelf_ehdr.c 3177 2015-03-30 18:19:41Z emaste $");

GElf_Ehdr *
gelf_getehdr(Elf *e, GElf_Ehdr *d)
{
	int ec;
	Elf32_Ehdr *eh32;
	Elf64_Ehdr *eh64;

	if (d == NULL || e == NULL ||
	    ((ec = e->e_class) != ELFCLASS32 && ec != ELFCLASS64)) {
		LIBELF_SET_ERROR(ARGUMENT, 0);
		return (NULL);
	}

	if (ec == ELFCLASS32) {
		if ((eh32 = _libelf_ehdr(e, ELFCLASS32, 0)) == NULL)
			return (NULL);

		(void) memcpy(d->e_ident, eh32->e_ident,
		    sizeof(eh32->e_ident));
		d->e_type		= eh32->e_type;
		d->e_machine		= eh32->e_machine;
		d->e_version		= eh32->e_version;
		d->e_entry		= eh32->e_entry;
		d->e_phoff		= eh32->e_phoff;
		d->e_shoff		= eh32->e_shoff;
		d->e_flags		= eh32->e_flags;
		d->e_ehsize		= eh32->e_ehsize;
		d->e_phentsize		= eh32->e_phentsize;
		d->e_phnum		= eh32->e_phnum;
		d->e_shentsize		= eh32->e_shentsize;
		d->e_shnum		= eh32->e_shnum;
		d->e_shstrndx		= eh32->e_shstrndx;

		return (d);
	}

	assert(ec == ELFCLASS64);

	if ((eh64 = _libelf_ehdr(e, ELFCLASS64, 0)) == NULL)
		return (NULL);
	*d = *eh64;

	return (d);
}
