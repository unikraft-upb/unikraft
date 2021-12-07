/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Simon Kuenzer <simon.kuenzer@neclab.eu>
 *
 *
 * Copyright (c) 2017, NEC Europe Ltd., NEC Corporation. All rights reserved.
 * Copyright (c) 2021, NEC Laboratories Europe GmbH, NEC Corporation.
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

#include <inttypes.h>
#include <string.h>
#include <uk/arch/lcpu.h>
#include <uk/plat/bootstrap.h>
#include <uk/errptr.h>
#include <errno.h>
#if CONFIG_XEN_SHUTDOWN
#include <uk/init.h>
#include <uk/sched.h>
#include <xenbus/xs.h>
#include <xenbus/client.h>
#include <stdlib.h>
#endif /* CONFIG_XEN_SHUTDOWN */

#include <xen/xen.h>
#include <common/console.h>

#if defined __X86_32__
#include <xen-x86/hypercall32.h>
#elif defined __X86_64__
#include <xen-x86/hypercall64.h>
#elif (defined __ARM_32__) || (defined __ARM_64__)
#include <xen-arm/hypercall.h>
#endif

void ukplat_terminate(enum ukplat_gstate request)
{
	int reason;

	switch (request) {
	case UKPLAT_HALT:
		uk_pr_info("Powering off...\n");
		reason = SHUTDOWN_poweroff;
		break;
	case UKPLAT_RESTART:
		uk_pr_info("Rebooting...\n");
		reason = SHUTDOWN_reboot;
		break;
	default: /* UKPLAT_CRASH */
		uk_pr_info("Crashing...\n");
		reason = SHUTDOWN_crash;
		break;
	}

	flush_console();

	for (;;) {
		struct sched_shutdown sched_shutdown = { .reason = reason };

		HYPERVISOR_sched_op(SCHEDOP_shutdown, &sched_shutdown);
	}
}

int ukplat_suspend(void)
{
	int ret;

	ret = EBUSY;
	/* ret = HYPERVISOR_suspend(virt_to_mfn(start_info_ptr)); */
	return -ret;
}

#if CONFIG_XEN_SHUTDOWN
static void shutdown_handler(void *argp)
{
	struct xenbus_watch *watch = (struct xenbus_watch *) argp;
	char *req;
	int err;

	UK_ASSERT(watch);

	for (;;) {
		xenbus_watch_wait_event(watch);
		req = xs_read(XBT_NIL, "control", "shutdown");
		if (PTRISERR(req)) {
			uk_pr_debug("Ignore read error of incoming request: %d\n",
				    PTR2ERR(req));
			continue;
		}
		if (!strcmp(req, "")) {
			/* FIXME: Investigate reason of spurious events */
			uk_pr_debug("Ignore spurious event\n");
			free(req);
			continue;
		}

		/* Acknowledge request by clearing the node */
		/* TODO: Is this really needed? */
		err = xs_write(XBT_NIL, "control", "shutdown", "");
		if (err) {
			/* Ignore failure but throw a message */
			uk_pr_err("Failed to acknowledge '%s' request: %d\n",
				  req, err);
		}

		/*
		 * Take action
		 * TODO: We call here directly `ukplat_terminate()`. However,
		 *       as soon as we are ready, we should forward a signal
		 *       to our application/general libraries so that it can
		 *       take all needed actions for a clean shutdown (e.g.,
		 *       clean filesystem unmount). We should only receive
		 *       each request here and forward it.
		 */
		if (!strcmp(req, "poweroff")) {
			uk_pr_info("Shutdown requested\n");
			ukplat_halt();
		} else if (!strcmp(req, "reboot")) {
			uk_pr_info("Restart requested\n");
			ukplat_restart();
		} else {
			uk_pr_err("Ignore unsupported request: %s\n",
				  req);
		}
		free(req);
	}
}

static int shutdown_handler_init(void)
{
	struct xenbus_watch *watch;
	struct uk_thread *t;
	int ret = 0;

	watch = xs_watch_path(XBT_NIL, "control/shutdown");
	if (PTRISERR(watch)) {
		uk_pr_crit("Failed to create watch for 'control/shutdown': %d\n",
			   PTR2ERR(watch));
		ret = -PTR2ERR(watch);
		goto err_out;
	}

	t = uk_thread_create("xen-shutdown",
			     shutdown_handler, watch);
	if (!t) {
		uk_pr_crit("Failed to create shutdown handler thread\n");
		ret = -ENOMEM;
		goto err_unwatch;
	}
	return 0;

err_unwatch:
	xs_unwatch(XBT_NIL, watch);
err_out:
	return ret;
}

uk_plat_initcall(shutdown_handler_init);
#endif /* CONFIG_XEN_SHUTDOWN */
