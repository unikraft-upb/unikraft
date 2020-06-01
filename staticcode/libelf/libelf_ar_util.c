/*-
 * Copyright (c) 2006,2009,2010 Joseph Koshy
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS `AS IS' AND
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
#include <libelf.h>
#include <stdlib.h>
#include <string.h>

#include "_libelf.h"
#include "_libelf_ar.h"

ELFTC_VCSID("$Id: libelf_ar_util.c 3174 2015-03-27 17:13:41Z emaste $");

/*
 * Convert a string bounded by `start' and `start+sz' (exclusive) to a
 * number in the specified base.
 */
int
_libelf_ar_get_number(const char *src, size_t sz, unsigned int base,
    size_t *ret)
{
	size_t r;
	unsigned int c, v;
	const unsigned char *e, *s;

	assert(base <= 10);

	s = (const unsigned char *) src;
	e = s + sz;

	/* skip leading blanks */
	for (;s < e && (c = *s) == ' '; s++)
		;

	r = 0L;
	for (;s < e; s++) {
		if ((c = *s) == ' ')
			break;
		if (c < '0' || c > '9')
			return (0);
		v = c - '0';
		if (v >= base)		/* Illegal digit. */
			break;
		r *= base;
		r += v;
	}

	*ret = r;

	return (1);
}

/*
 * Open an 'ar' archive.
 */
Elf *
_libelf_ar_open(Elf *e, int reporterror)
{
	size_t sz;
	int scanahead;
	struct ar_hdr arh;
	unsigned char *s, *end;

	_libelf_init_elf(e, ELF_K_AR);

	e->e_u.e_ar.e_nchildren = 0;
	e->e_u.e_ar.e_next = (off_t) -1;

	/*
	 * Look for special members.
	 */

	s = e->e_rawfile + SARMAG;
	end = e->e_rawfile + e->e_rawsize;

	assert(e->e_rawsize > 0);

	/*
	 * We use heuristics to determine the flavor of the archive we
	 * are examining.
	 *
	 * SVR4 flavor archives use the name "/ " and "// " for
	 * special members.
	 *
	 * In BSD flavor archives the symbol table, if present, is the
	 * first archive with name "__.SYMDEF".
	 */

#define	READ_AR_HEADER(S, ARH, SZ, END)					\
	do {								\
		if ((S) + sizeof((ARH)) > (END))			\
		        goto error;					\
		(void) memcpy(&(ARH), (S), sizeof((ARH)));		\
		if ((ARH).ar_fmag[0] != '`' || (ARH).ar_fmag[1] != '\n') \
			goto error;					\
		if (_libelf_ar_get_number((char *) (ARH).ar_size,	\
		    sizeof((ARH).ar_size), 10, &(SZ)) == 0)		\
			goto error;					\
	} while (0)

	READ_AR_HEADER(s, arh, sz, end);

	/*
	 * Handle special archive members for the SVR4 format.
	 */
	if (arh.ar_name[0] == '/') {
		if (sz == 0)
			goto error;

		e->e_flags |= LIBELF_F_AR_VARIANT_SVR4;

		scanahead = 0;

		/*
		 * The symbol table (file name "/ ") always comes before the
		 * string table (file name "// ").
		 */
		if (arh.ar_name[1] == ' ') {
			/* "/ " => symbol table. */
			scanahead = 1;	/* The string table to follow. */

			s += sizeof(arh);
			e->e_u.e_ar.e_rawsymtab = s;
			e->e_u.e_ar.e_rawsymtabsz = sz;

			sz = LIBELF_ADJUST_AR_SIZE(sz);
			s += sz;

		} else if (arh.ar_name[1] == '/' && arh.ar_name[2] == ' ') {
			/* "// " => string table for long file names. */
			s += sizeof(arh);
			e->e_u.e_ar.e_rawstrtab = s;
			e->e_u.e_ar.e_rawstrtabsz = sz;

			sz = LIBELF_ADJUST_AR_SIZE(sz);
			s += sz;
		}

		/*
		 * If the string table hasn't been seen yet, look for
		 * it in the next member.
		 */
		if (scanahead) {
			READ_AR_HEADER(s, arh, sz, end);

			/* "// " => string table for long file names. */
			if (arh.ar_name[0] == '/' && arh.ar_name[1] == '/' &&
			    arh.ar_name[2] == ' ') {

				s += sizeof(arh);

				e->e_u.e_ar.e_rawstrtab = s;
				e->e_u.e_ar.e_rawstrtabsz = sz;

				sz = LIBELF_ADJUST_AR_SIZE(sz);
				s += sz;
			}
		}
	} else if (strncmp(arh.ar_name, LIBELF_AR_BSD_SYMTAB_NAME,
		sizeof(LIBELF_AR_BSD_SYMTAB_NAME) - 1) == 0) {
		/*
		 * BSD style archive symbol table.
		 */
		s += sizeof(arh);
		e->e_u.e_ar.e_rawsymtab = s;
		e->e_u.e_ar.e_rawsymtabsz = sz;

		sz = LIBELF_ADJUST_AR_SIZE(sz);
		s += sz;
	}

	/*
	 * Update the 'next' offset, so that a subsequent elf_begin()
	 * works as expected.
	 */
	e->e_u.e_ar.e_next = (off_t) (s - e->e_rawfile);

	return (e);

error:
	if (!reporterror) {
		e->e_kind = ELF_K_NONE;
		return (e);
	}

	LIBELF_SET_ERROR(ARCHIVE, 0);
	return (NULL);
}
