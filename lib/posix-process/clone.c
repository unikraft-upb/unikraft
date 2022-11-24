/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Simon Kuenzer <simon.kuenzer@neclab.eu>
 *
 * Copyright (c) 2022, NEC Laboratories Europe GmbH, NEC Corporation.
 *                     All rights reserved.
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

#include <errno.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <uk/process.h>
#include <uk/print.h>
#include <uk/syscall.h>
#include <uk/arch/limits.h>
#include <uk/sched.h>

#include "process.h"

/* Up to cl_args->tls, the fields of clone_args are required arguments */
#define CL_ARGS_REQUIRED_LEN					\
	(__offsetof(struct clone_args, tls)			\
	 + sizeof(((struct clone_args *)0)->tls))

#ifdef CONFIG_LIBUKDEBUG_ENABLE_ASSERT
#define CL_UKTLS_SANITY_MAGIC 0xb0b0f00d /* Bobo food */
static __thread uint32_t cl_uktls_magic = CL_UKTLS_SANITY_MAGIC;
#endif /* CONFIG_LIBUKDEBUG_ENABLE_ASSERT */

/*
 * NOTE: From man pages about clone(2)
 *       (https://man7.org/linux/man-pages/man2/clone.2.html):
 *       "The raw clone() system call corresponds more closely to fork(2)
 *        in that execution in the child continues from the point of the
 *        call.  As such, the fn and arg arguments of the clone() wrapper
 *        function are omitted.
 *
 *        In contrast to the glibc wrapper, the raw clone() system call
 *        accepts NULL as a stack argument (and clone3() likewise allows
 *        cl_args.stack to be NULL).  In this case, the child uses a
 *        duplicate of the parent's stack.  (Copy-on-write semantics ensure
 *        that the child gets separate copies of stack pages when either
 *        process modifies the stack.)  In this case, for correct
 *        operation, the CLONE_VM option should not be specified.  (If the
 *        child shares the parent's memory because of the use of the
 *        CLONE_VM flag, then no copy-on-write duplication occurs and chaos
 *        is likely to result.)
 *
 *        The order of the arguments also differs in the raw system call,
 *        and there are variations in the arguments across architectures,
 *        as detailed in the following paragraphs.
 *
 *        The raw system call interface on x86-64 and some other
 *        architectures (including sh, tile, and alpha) is:
 *
 *            long clone(unsigned long flags, void *stack,
 *                       int *parent_tid, int *child_tid,
 *                       unsigned long tls);
 *
 *        On x86-32, and several other common architectures (including
 *        score, ARM, ARM 64, PA-RISC, arc, Power PC, xtensa, and MIPS),
 *        the order of the last two arguments is reversed:
 *
 *            long clone(unsigned long flags, void *stack,
 *                       int *parent_tid, unsigned long tls,
 *                       int *child_tid);
 *       "
 */
static void _clone_child_gc(struct uk_thread *t)
{
	if (t->name) {
		free(DECONST(char *, t->name));
		t->name = NULL;
	}
}

/*
 * NOTE: The clone system call and the handling of the TLS
 *
 *       `_clone()` assumes that a passed TLS pointer is an Unikraft TLS.
 *       The only exception exists if `_clone()` is called from a context
 *       where a custom TLS is already active (depends on
 *       `CONFIG_LIBSYSCALL_SHIM_HANDLER_ULTLS`). In such a case, an
 *       Unikraft TLS is allocated but the passed TLS pointer is activated.
 *       The reason is that Unikraft libraries place TLS variables and use
 *       the TLS effectively as TCB.
 *       In case no TLS is handed over (CLONE_SETTLS is not set), _clone will
 *       still allocate an Unikraft TLS but sets the TLS architecture pointer
 *       to zero.
 */
static int _clone(struct clone_args *cl_args, size_t cl_args_len,
		  __uptr return_addr)
{
	struct uk_thread *t;
	struct uk_sched *s;
	struct uk_thread *child = NULL;
	__u64 flags;
	int ret;

	t = uk_thread_current();
	s = uk_sched_current();

	UK_ASSERT(s);
	UK_ASSERT(t);
	/* Parent must have ECTX and a Unikraft TLS */
	UK_ASSERT((t->flags & UK_THREADF_ECTX)
		  && (t->flags & UK_THREADF_UKTLS));
	UK_ASSERT(return_addr);

	if (!cl_args || cl_args_len < CL_ARGS_REQUIRED_LEN) {
		uk_pr_debug("No or invalid clone arguments given\n");
		ret = -EINVAL;
		goto err_out;
	}

	flags = cl_args->flags;

	if ((flags & CLONE_SETTLS)
#if CONFIG_LIBSYSCALL_SHIM_HANDLER_ULTLS
	    && (uk_syscall_ultlsp() == 0x0)
#endif /* CONFIG_LIBSYSCALL_SHIM_HANDLER_ULTLS */
	) {
		/* The caller already created a TLS for the child (for instance
		 * by a pthread API wrapper). We expect that this TLS is a
		 * Unikraft TLS.
		 */
		uk_pr_debug("Using passed TLS pointer %p as an Unikraft TLS\n",
			    (void *) cl_args->tls);
		child = uk_thread_create_container2(s->a,
						    (__uptr) cl_args->stack,
						    (__uptr) cl_args->tls,
						    true, /* TLS is an UKTLS */
						    false, /* We want ECTX */
						    (t->name) ? strdup(t->name)
							      : NULL,
						    NULL,
						    _clone_child_gc);
	} else {
		/* If no TLS was given or the parent calls us already from
		 * a context with an userland TLS activated (kernel land vs.
		 * user land), we allocate an Unikraft TLS because Unikraft
		 * places TLS variables and uses them effectively as TCB.
		 */
#if CONFIG_LIBSYSCALL_SHIM_HANDLER_ULTLS
		if (uk_syscall_ultlsp() != 0x0) {
			uk_pr_debug("Allocating an Unikraft TLS for the new child, parent called from context with custom TLS\n");
		} else
#endif /* CONFIG_LIBSYSCALL_SHIM_HANDLER_ULTLS */
		{
			uk_pr_debug("Allocating an Unikraft TLS for the new child, no TLS given by parent\n");
		}
		child = uk_thread_create_container(s->a,
						   NULL, 0, /* Stack is given */
						   s->a_uktls,
						   false, /* We want ECTX */
						   (t->name) ? strdup(t->name)
							     : NULL,
						   NULL,
						   _clone_child_gc);
	}
	if (PTRISERR(child)) {
		ret = (PTR2ERR(child) != 0) ? PTR2ERR(child) : -ENOMEM;
		goto err_out;
	}
#ifdef CONFIG_LIBUKDEBUG_ENABLE_ASSERT
	/* Sanity check that the UKTLS of the child is really a Unikraft TLS:
	 * Do we find our magic on the TLS, is Bobo's banana there?
	 */
	UK_ASSERT(uk_thread_uktls_var(child, cl_uktls_magic)
		  == CL_UKTLS_SANITY_MAGIC);
#endif /* CONFIG_LIBUKDEBUG_ENABLE_ASSERT */

	/* CLONE_SETTLS: Instead of just activating the Unikraft TLS, we
	 * activate the passed TLS pointer as soon as the child wakes up.
	 * NOTE: If SETTLS is not set, we do not activate any TLS although
	 *       an Unikraft TLS was allocated.
	 */
	child->tlsp = (flags & CLONE_SETTLS) ? cl_args->tls : 0x0;
	uk_pr_debug("Child is going to wake up with TLS pointer set to: %p (%s TLS)\n",
		    (void *) child->tlsp,
		    (child->tlsp != child->uktlsp) ? "custom" : "Unikraft");

	uk_pr_debug("Thread cloned %p (%s) -> %p (%s): %d\n",
		    t, t->name ? child->name : "<unnamed>",
		    child, child->name ? child->name : "<unnamed>", ret);

	/*
	 * Child starts at return address, sets given stack and given TLS.
	 * Register clearing has the effect that it looks like `clone`
	 * returns `0` in the child.
	 */
	ukarch_ctx_init(&child->ctx,
			(__uptr) cl_args->stack,
			false,
			return_addr);
	uk_thread_set_runnable(child);

	/* We will return the child's thread ID in the parent */
	ret = ukthread2tid(child);

	/* Assign the child to the scheduler */
	uk_sched_thread_add(s, child);

	return ret;

err_free_child:
	uk_thread_release(child);
err_out:
	return ret;
}

#if CONFIG_ARCH_X86_64
UK_LLSYSCALL_R_DEFINE(int, clone,
		      unsigned long, flags,
		      void *, sp,
		      int *, parent_tid,
		      int *, child_tid,
		      unsigned long, tlsp)
#else /* !CONFIG_ARCH_X86_64 */
UK_LLSYSCALL_R_DEFINE(int, clone,
		      unsigned long, flags,
		      void *, sp,
		      int *, parent_tid,
		      unsigned long, tlsp,
		      int *, child_tid)
#endif /* !CONFIG_ARCH_X86_64 */
{
	/* Translate */
	struct clone_args cl_args = {
		.flags       = (__u64) (flags & ~0xff),
		.pidfd       = (__u64) ((flags & CLONE_PIDFD) ? parent_tid : 0),
		.child_tid   = (__u64) child_tid,
		.parent_tid  = (__u64) ((flags & CLONE_PIDFD) ? 0 : parent_tid),
		.exit_signal = (__u64) (flags & 0xff),
		.stack       = (__u64) sp,
		.tls         = (__u64) tlsp
	};

	return _clone(&cl_args, sizeof(cl_args), uk_syscall_return_addr());
}

#if UK_LIBC_SYSCALLS
int clone(int (*fn)(void *) __unused, void *sp __unused,
	  int flags __unused, void *arg __unused,
	  ... /* pid_t *parent_tid, void *tls, pid_t *child_tid */)
{
	/* TODO */
	errno = EINVAL;
	return -1;
}
#endif /* UK_LIBC_SYSCALLS */

/* NOTE: There are currently no libc wrapper for clone3 */
UK_LLSYSCALL_R_DEFINE(long, clone3,
		      struct clone_args *, cl_args,
		      size_t, cl_args_len)
{
	return _clone(cl_args, cl_args_len, uk_syscall_return_addr());
}
