/* SPDX-License-Identifier: ISC */
/*
 * Authors: Martin Lucina
 *          Felipe Huici <felipe.huici@neclab.eu>
 *
 * Copyright (c) 2016-2017 Docker, Inc.
 * Copyright (c) 2017 NEC Europe Ltd., NEC Corporation
 *
 * Permission to use, copy, modify, and/or distribute this software
 * for any purpose with or without fee is hereby granted, provided
 * that the above copyright notice and this permission notice appear
 * in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS
 * OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <errno.h>
#include <uk/plat/common/cpu.h>
#include <uk/plat/common/irq.h>
#include <uk/print.h>
#include <uk/plat/bootstrap.h>

#if CONFIG_OPTIMIZE_PGO_GENERATE
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>

#define GCOV_GENERATED_DIR	"/gcov_profiling"
#define GCOV_PREFIX_STRIP_LENGTH	"100"

void set_path_gcov_files()
{
	const char *gcov_prefix;
	const char *gcov_strip;

	gcov_prefix = getenv("GCOV_PREFIX");
	if (!gcov_prefix)
		setenv("GCOV_PREFIX", GCOV_GENERATED_DIR, 1);

	gcov_strip = getenv("GCOV_PREFIX_STRIP");
	if (!gcov_strip)
		setenv("GCOV_PREFIX_STRIP", GCOV_PREFIX_STRIP_LENGTH, 1);

	DIR* dir = opendir(GCOV_GENERATED_DIR);
	if (dir)
		closedir(dir);
	else if (ENOENT == errno)
		mkdir(GCOV_GENERATED_DIR, 0777);
}
#endif

static void cpu_halt(void) __noreturn;

/* TODO: implement CPU reset */
void ukplat_terminate(enum ukplat_gstate request __unused)
{

#if CONFIG_OPTIMIZE_PGO_GENERATE
	set_path_gcov_files();
	__gcov_exit();
#endif

	uk_pr_info("Unikraft halted\n");

	/* Try to make system off */
	system_off();

	/*
	 * If we got here, there is no way to initiate "shutdown" on virtio
	 * without ACPI, so just halt.
	 */
	cpu_halt();
}

static void cpu_halt(void)
{
	__CPU_HALT();
}

int ukplat_suspend(void)
{
	return -EBUSY;
}
