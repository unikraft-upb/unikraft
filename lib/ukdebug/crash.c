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

#include <uk/crash.h>
#include <uk/config.h>
#include <uk/crashdump.h>
#include <uk/plat/lcpu.h>
#include <uk/plat/bootstrap.h>
#include <uk/plat/time.h>
#include <uk/preempt.h>
#include <uk/essentials.h>
#include <uk/arch/atomic.h>
#if defined(CONFIG_LIBUKSCHED) && defined(CONFIG_LIBUKNOFAULT)
#include <uk/nofault.h>
#include <uk/thread.h>
#endif /* CONFIG_LIBUKSCHED && CONFIG_LIBUKNOFAULT */
#include "snprintf.h"
#include "gdbstub.h"

#include <stdarg.h>
#include <stddef.h>

#ifdef CONFIG_LIBUKDEBUG_CRASH_SCREEN
#define crsh_printk(fmt, ...)						\
	_uk_printk(KLVL_CRIT, NULL, NULL, 0, fmt, ##__VA_ARGS__)
#define crsh_vprintk(fmt, ap)						\
	_uk_vprintk(KLVL_CRIT, NULL, NULL, 0, fmt, ap)
#define crsh_crashdumpk(regs)						\
	_uk_crashdumpk(KLVL_CRIT, NULL, NULL, 0, regs)

static void crsh_print_thread_info()
{
#if defined(CONFIG_LIBUKSCHED) && defined(CONFIG_LIBUKNOFAULT)
	struct uk_thread *current = uk_thread_current();

	if (!current)
		return;

	if (!uk_memprobe_r((unsigned long)current, sizeof(struct uk_thread))) {
		crsh_printk("Current thread information corrupted\n");
		return;
	}

	crsh_printk("Thread \"%s\"@%p (Stack: %p, TLS: %p)\n",
		    uk_memprobe_r((unsigned long)current->name, 1) ?
			current->name : "(corrupted)",
		    current, current->stack,
		    current->tls);

#endif /* CONFIG_LIBUKSCHED && CONFIG_LIBUKNOFAULT */
}
#else /* CONFIG_LIBUKDEBUG_CRASH_SCREEN */
#define crsh_printk(fmt, ...)
#define crsh_vprintk(fmt, ap)
#define crsh_crashdumpk(regs)
#endif /* !CONFIG_LIBUKDEBUG_CRASH_SCREEN */

#define __CRSH_REBOOT_HINT(delay)					\
	"System rebooting in " #delay " seconds...\n"
#define _CRSH_REBOOT_HINT(delay)					\
	__CRSH_REBOOT_HINT(delay)
#define CRSH_REBOOT_HINT						\
	_CRSH_REBOOT_HINT(CONFIG_LIBUKDEBUG_CRASH_REBOOT_DELAY)

#ifdef CONFIG_LIBUKDEBUG_CRASH_ACTION_REBOOT
#define CRSH_SHUTDONW_ACTION "reboot"
#else /* CONFIG_LIBUKDEBUG_CRASH_ACTION_REBOOT */
#define CRSH_SHUTDONW_ACTION "halt"
#endif /* !CONFIG_LIBUKDEBUG_CRASH_ACTION_REBOOT */

#define CRSH_SHUTDOWN_WARNING						\
	"==========================================\n"			\
	"The system crashed. Continuing or stepping\n"			\
	"will " CRSH_SHUTDONW_ACTION " the system!\n"			\
	"==========================================\n"

#ifdef CONFIG_LIBUKDEBUG_GDBSTUB
static void crsh_dbg_trap(int errnr, struct __regs *regs,
			  const char *fmt, va_list ap)
{
	char buf[256];

	crsh_printk("Waiting for debugger...\n");

	/* N.B. Extended architecture registers (e.g., XMM on intel) might be
	 * invalidate by now (varargs!). So although the debugger displays the
	 * system state as captured by __regs as it was when the UK_CRASH
	 * macro was invoked, the extended state may not be correct.
	 *
	 * TODO: Maybe compile the whole debug library with |isr?! Do we have
	 * any external dependencies (libc?) that might corrupt the state?
	 */
	__uk_vsnprintf(buf, sizeof(buf), fmt, ap);

	//gdb_dbg_print(CRSH_SHUTDOWN_WARNING);
	//gdb_dbg_print(buf);
	gdb_dbg_trap(errnr, regs);
}
#endif /* CONFIG_LIBUKDEBUG_GDBSTUB */

__noreturn static void crsh_shutdown(void)
{
#ifdef CONFIG_LIBUKDEBUG_CRASH_ACTION_REBOOT
	__nsec until = CONFIG_LIBUKDEBUG_CRASH_REBOOT_DELAY;

	if (until > 0) {
		until += ukarch_time_sec_to_nsec(secs);

		crsh_printk(CRSH_REBOOT_HINT);

		/* Interrupts are disabled. Just busy spin... */
		while (until > ukplat_monotonic_clock()) {}
	}

	ukplat_restart();
#else /* CONFIG_LIBUKDEBUG_CRASH_ACTION_REBOOT */
	ukplat_crash();
#endif /* CONFIG_LIBUKDEBUG_CRASH_ACTION_REBOOT */
}

void _uk_crash(int errnr __maybe_unused, const char *libname __unused,
	       const char *srcname __unused, unsigned int srcline __unused,
	       struct __regs *regs, const char *fmt, ...)
{
#ifdef UKPLAT_LCPU_MULTICORE
	static __s32 crash_cpu = -1;
	int cpu_id = ukplat_lcpu_id();
#endif /* UKPLAT_LCPU_MULTICORE */
	va_list ap __maybe_unused;

	ukplat_lcpu_disable_irq();
	uk_preempt_disable();

#ifdef UKPLAT_LCPU_MULTICORE
#warning The crash code does not support multicore systems

	/* Only let one CPU perform the crash */
	if (ukarch_compare_exchange_sync(&crash_cpu, -1, cpu_id) != cpu_id) {
		/* TODO: Finish SMP Support
		 * Freeze CPU or wait until the crash_cpu initiates a freeze
		 * (e.g., through IPI). For now, just busy wait.
		 */
		while (1) {}
	}
#endif /* UKPLAT_LCPU_MULTICORE */

#ifdef CONFIG_LIBUKDEBUG_CRASH_SCREEN
	crsh_printk("Unikraft crash - " STRINGIFY(UK_CODENAME)
		    " (" STRINGIFY(UK_FULLVERSION) ")\n");

	crsh_print_thread_info();
	crsh_crashdumpk(regs);

	va_start(ap, fmt);
	crsh_vprintk(fmt, ap);
	va_end(ap);
#endif /* CONFIG_LIBUKDEBUG_CRASH_SCREEN */

#ifdef CONFIG_LIBUKDEBUG_GDBSTUB
	/* Enter the debugger stub */
	va_start(ap, fmt);
	crsh_dbg_trap(errnr, regs, fmt, ap);
	va_end(ap);
#endif /* CONFIG_LIBUKDEBUG_GDBSTUB */

	/* Halt or reboot the system */
	crsh_shutdown();
}
