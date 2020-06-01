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

#include <gelf.h>
#include <libelf.h>
#include <limits.h>
#include <stdint.h>

#include "_libelf.h"

ELFTC_VCSID("$Id: gelf_phdr.c 3177 2015-03-30 18:19:41Z emaste $");

GElf_Phdr *
gelf_getphdr(Elf *e, int index, GElf_Phdr *d)
{
	int ec;
	Elf32_Ehdr *eh32;
	Elf64_Ehdr *eh64;
	Elf32_Phdr *ep32;
	Elf64_Phdr *ep64;

	if (d == NULL || e == NULL ||
	    ((ec = e->e_class) != ELFCLASS32 && ec != ELFCLASS64) ||
	    (e->e_kind != ELF_K_ELF) || index < 0) {
		LIBELF_SET_ERROR(ARGUMENT, 0);
		return (NULL);
	}

	if (ec == ELFCLASS32) {
		if ((eh32 = _libelf_ehdr(e, ELFCLASS32, 0)) == NULL ||
		    ((ep32 = _libelf_getphdr(e, ELFCLASS32)) == NULL))
			return (NULL);

		if (index >= eh32->e_phnum) {
			LIBELF_SET_ERROR(ARGUMENT, 0);
			return (NULL);
		}

		ep32 += index;

		d->p_type   = ep32->p_type;
		d->p_offset = ep32->p_offset;
		d->p_vaddr  = (Elf64_Addr) ep32->p_vaddr;
		d->p_paddr  = (Elf64_Addr) ep32->p_paddr;
		d->p_filesz = (Elf64_Xword) ep32->p_filesz;
		d->p_memsz  = (Elf64_Xword) ep32->p_memsz;
		d->p_flags  = ep32->p_flags;
		d->p_align  = (Elf64_Xword) ep32->p_align;

	} else {
		if ((eh64 = _libelf_ehdr(e, ELFCLASS64, 0)) == NULL ||
		    (ep64 = _libelf_getphdr(e, ELFCLASS64)) == NULL)
			return (NULL);

		if (index >= eh64->e_phnum) {
			LIBELF_SET_ERROR(ARGUMENT, 0);
			return (NULL);
		}

		ep64 += index;

		*d = *ep64;
	}

	return (d);
}
