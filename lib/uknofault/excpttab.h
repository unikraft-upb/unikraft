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

#ifndef __UKNOFAULT_INTERNAL_EXCPTTAB_H__
#define __UKNOFAULT_INTERNAL_EXCPTTAB_H__

#include <uk/arch/traps.h>
#include <uk/arch/types.h>
#include <uk/essentials.h>

/**
 * The exception entries are stored in the .uk_excpttab section and map
 * potentially faulting addresses to corresponding exception handlers. The
 * field values are stored relative to the respective field's address in the
 * exception table to make them position-independent.
 *
 * N.B. This breaks if the table is more than 2 GiB above or below the code.
 */
struct nf_excpttab_entry {
	/**< Faulting instruction pointer */
	__s32 ip;
	/**< Instruction pointer at which to continue execution */
	__s32 cont_ip;
	/**< Exception handler function (nf_excpt_handler) */
	__s32 handler;
} __packed;

typedef int (*nf_excpt_handler)(const struct nf_excpttab_entry *e,
		struct ukarch_trap_ctx *ctx);

#define _NF_DECLARE_EXCPTTAB_ENTRY_GETTER(field, type)		\
static inline type nf_excpttab_get_##field(			\
		const struct nf_excpttab_entry *e)		\
{								\
	return (type)((__uptr)&e->field + e->field);		\
}

_NF_DECLARE_EXCPTTAB_ENTRY_GETTER(ip, unsigned long);
_NF_DECLARE_EXCPTTAB_ENTRY_GETTER(handler, nf_excpt_handler);
_NF_DECLARE_EXCPTTAB_ENTRY_GETTER(cont_ip, unsigned long);
#undef _NF_DECLARE_EXCPTTAB_ENTRY_GETTER

/* Exception table; provided by the platform's linker script */
extern const struct nf_excpttab_entry uk_excpttab_start[];
extern const struct nf_excpttab_entry uk_excpttab_end;

/**
 * Helper macro to add an entry to the exception handler table in an inline
 * asm block.
 *
 * @param label A local asm label for which to register the exception handler
 * @param cont_label A local C label where the handler should resume execution
 * @param handler The exception handler to call
 */
#define NF_EXCPTTAB_ENTRY_L(label, cont_label, handler)		\
	".pushsection .uk_excpttab, \"a\"\n"			\
	".long " #label " - .\n"				\
	".long %l[" #cont_label "] - .\n"			\
	".long " STRINGIFY(handler) " - .\n"			\
	".popsection\n"

/**
 * Helper macro for iterating over exception handler pointer tables
 *
 * @param itr
 *   Iterator variable (nf_excpt_handler *) which points to the individual
 *   table entries during iteration
 * @param ctortab_start
 *   Start address of table (type: const nf_excpttab_entry[])
 * @param ctortab_end
 *   End address of table (type: const nf_excpttab_entry)
 */
#define nf_excpttab_foreach(itr, excpttab_start, excpttab_end)		\
	for ((itr) = DECONST(struct nf_excpttab_entry*, excpttab_start);\
	     (itr) < &(excpttab_end);					\
	     (itr)++)

static inline int nf_handle_trap(unsigned long ip, struct ukarch_trap_ctx *ctx)
{
	const struct nf_excpttab_entry *itr;
	nf_excpt_handler handler;

	/* Scan the exception table to find the faulting address */
	nf_excpttab_foreach(itr, uk_excpttab_start, uk_excpttab_end) {
		UK_ASSERT(itr);
		if (nf_excpttab_get_ip(itr) == ip) {
			handler = nf_excpttab_get_handler(itr);
			UK_ASSERT(handler);

			return handler(itr, ctx);
		}
	}

	return UK_EVENT_NOT_HANDLED;
}

#endif /* __UKNOFAULT_INTERNAL_EXCPTTAB_H__ */
