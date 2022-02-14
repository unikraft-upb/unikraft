/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2005-2007, Kohsuke Ohtani
 * Copyright (C) 2014 Cloudius Systems, Ltd.
 * Copyright (c) 2019, NEC Europe Ltd., NEC Corporation.
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
 * 3. Neither the name of the author nor the names of any co-contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#define _BSD_SOURCE

#include <string.h>
#include <stdlib.h>

#include <uk/list.h>
#include <uk/mutex.h>
#include <vfscore/dentry.h>
#include <vfscore/vnode.h>
#include "vfs.h"

/** Number of hash buckets in dentry cache */
#define DENTRY_CACHE_LINES		1024
/** Maximum number of unreferenced dentries until eviction starts */
#define DENTRY_CACHE_MAX_STANDY		256
/** Number of unreferenced dentries to evict per batch. */
#define DENTRY_CACHE_EVICT_BATCH	8

#if DENTRY_CACHE_EVICT_BATCH > DENTRY_CACHE_LINES
#error "DENTRY_CACHE_EVICT_BATCH must be smaller than DENTRY_CACHE_LINES"
#endif

/** List of hash buckets to lookup allocated dentries */
static struct uk_list_head dentry_cache[DENTRY_CACHE_LINES];
/** Number of unreferenced dentries in cache */
static unsigned int dentry_cache_standy;
/** Lock to protect the cache
 *  NOTE: Must also be acquired when changing dentry fields that
 *        determine the position in cache (e.g., d_path, d_refcnt)
 */
static struct uk_mutex dentry_cache_lock =
	UK_MUTEX_INITIALIZER(dentry_cache_lock);

/**
 * Get the hash value from the mount point and path name.
 * TODO: replace with a better hash for 64-bit pointers.
 */
static inline unsigned int
dentry_hash(struct mount *mp, const char *path)
{
	unsigned int val = 0;

	UK_ASSERT(path);

	while (*path)
		val = ((val << 5) + val) + *path++;

	return (val ^ (unsigned long)mp) & (DENTRY_CACHE_LINES - 1);
}

struct dentry *
dentry_alloc(struct dentry *parent_dp, struct vnode *vp, const char *path)
{
	struct dentry *dp;

	UK_ASSERT(vp);
	UK_ASSERT(!path || !dentry_lookup(vp->v_mount, path));

	dp = (struct dentry*)calloc(sizeof(*dp), 1);
	if (!dp)
		return NULL;

	vref(vp);

	dp->d_refcnt = 1;
	dp->d_vnode = vp;
	dp->d_mount = vp->v_mount;
	UK_INIT_LIST_HEAD(&dp->d_child_list);
	uk_mutex_init(&dp->d_lock);

	if (path) {
		dp->d_path = strdup(path);
		if (!dp->d_path) {
			vrele(vp);
			free(dp);

			return NULL;
		}

		dp->d_hash = dentry_hash(dp->d_mount, path);

		if (parent_dp) {
			dref(parent_dp);

			dp->d_parent = parent_dp;

			/* Insert dentry into the parent's child list */
			uk_mutex_lock(&parent_dp->d_lock);
			uk_list_add(&dp->d_child_link,
				    &parent_dp->d_child_list);
			uk_mutex_unlock(&parent_dp->d_lock);
		}

		/* Insert dentry into names list of vnode */
		vn_add_name(vp, dp);

		uk_mutex_lock(&dentry_cache_lock);
		uk_list_add(&dp->d_link, &dentry_cache[dp->d_hash]);
		uk_mutex_unlock(&dentry_cache_lock);
	} else {
		UK_INIT_LIST_HEAD(&dp->d_link);

		/* We do not cache unnamed dentries. */
		dp->d_donotcache = 1;
	}

	return dp;
}

static void
dentry_free(struct dentry *dp)
{
	UK_ASSERT(dp);
	UK_ASSERT(dp->d_refcnt == 0);

	if (dp->d_parent) {
		/* Remove dentry from its parent's child list */
		uk_mutex_lock(&dp->d_parent->d_lock);
		uk_list_del(&dp->d_child_link);
		uk_mutex_unlock(&dp->d_parent->d_lock);

		drele(dp->d_parent);
	}

	if (dp->d_path) {
		vn_del_name(dp->d_vnode, dp);
		free(dp->d_path);
	}

	vrele(dp->d_vnode);

	free(dp);
}

struct dentry *
dentry_lookup(struct mount *mp, const char *path)
{
	struct dentry *dp;
	unsigned int hash;

	UK_ASSERT(mp);
	UK_ASSERT(path);

	hash = dentry_hash(mp, path);

	uk_mutex_lock(&dentry_cache_lock);
	uk_list_for_each_entry(dp, &dentry_cache[hash], d_link) {
		UK_ASSERT(dp->d_path);
		UK_ASSERT(dp->d_hash == hash);

		if (dp->d_mount == mp && !strncmp(dp->d_path, path, PATH_MAX)) {
			if (dp->d_refcnt == 0) {
				UK_ASSERT(!dp->d_donotcache);

				/* We keep referenced entries at the front of
				 * the cache line so we find them faster and
				 * can quickly evict standy entries by looking
				 * at the tail of the line
				 */
				uk_list_move(&dp->d_link, &dentry_cache[hash]);

				UK_ASSERT(dentry_cache_standy > 0);
				dentry_cache_standy--;
			}

			dp->d_refcnt++;

			uk_mutex_unlock(&dentry_cache_lock);
			return dp;
		}
	}
	uk_mutex_unlock(&dentry_cache_lock);

	return NULL; /* not found */
}

static void
dentry_remove_children(struct dentry *dp)
{
	struct dentry *entry;

	UK_ASSERT(dp);

	uk_mutex_lock(&dp->d_lock);
	uk_list_for_each_entry(entry, &dp->d_child_list, d_child_link) {
		UK_ASSERT(!uk_list_empty(&entry->d_link));
		UK_ASSERT(entry->d_refcnt > 0);

		uk_list_del_init(&entry->d_link);
	}
	uk_mutex_unlock(&dp->d_lock);
}

int
dentry_move(struct dentry *dp, struct dentry *parent_dp, const char *path)
{
	struct dentry *old_pdp = dp->d_parent;
	char *new_path, *old_path = dp->d_path;
	unsigned int hash;

	/* Only valid for named dentries */
	UK_ASSERT(path);
	UK_ASSERT(old_path);

	new_path = strdup(path);
	if (!new_path)
		return ENOMEM;

	if (old_pdp) {
		/* Remove dp from its old parent's child list */
		uk_mutex_lock(&old_pdp->d_lock);
		uk_list_del(&dp->d_child_link);
		uk_mutex_unlock(&old_pdp->d_lock);
	}

	if (parent_dp) {
		dref(parent_dp);

		/* Insert dp into its new parent's child list */
		uk_mutex_lock(&parent_dp->d_lock);
		uk_list_add(&dp->d_child_link,
				&parent_dp->d_child_list);
		uk_mutex_unlock(&parent_dp->d_lock);
	}

	hash = dentry_hash(dp->d_mount, new_path);

	uk_mutex_lock(&dentry_cache_lock);

	/* Remove all child dentries from the cache. They remain open
	 * until their last reference is released. However, they cannot
	 * be looked up anymore with the old path (i.e., a new lookup
	 * will create a new dentry). NOTE: They remain linked via this
	 * dentry's child list
	 */
	dentry_remove_children(dp);
	UK_ASSERT(uk_list_empty(&dp->d_child_list));

	/* Update dentry and move to correct cache line */
	dp->d_hash = hash;
	dp->d_path = new_path;
	dp->d_parent = parent_dp;

	/* Move dentry to new cache position */
	uk_list_add(&dp->d_link, &dentry_cache[hash]);

	uk_mutex_unlock(&dentry_cache_lock);

	if (old_pdp)
		drele(old_pdp);

	free(old_path);
	return 0;
}

void
dentry_remove(struct dentry *dp)
{
	UK_ASSERT(dp);
	UK_ASSERT(dp->d_path);	/* Only valid for named dentries */

	uk_mutex_lock(&dentry_cache_lock);
	UK_ASSERT(!uk_list_empty(&dp->d_link));
	UK_ASSERT(dp->d_refcnt > 0);

	uk_list_del_init(&dp->d_link);
	uk_mutex_unlock(&dentry_cache_lock);
}

void
dref(struct dentry *dp)
{
	UK_ASSERT(dp);

	uk_mutex_lock(&dentry_cache_lock);
	UK_ASSERT(dp->d_refcnt > 0);

	dp->d_refcnt++;
	uk_mutex_unlock(&dentry_cache_lock);
}

void
drele(struct dentry *dp)
{
	unsigned int ev_line, ev_num;
	struct dentry *dp2, *ev_dp = dp;

	UK_ASSERT(dp);

	uk_mutex_lock(&dentry_cache_lock);

	if (--dp->d_refcnt) {
		uk_mutex_unlock(&dentry_cache_lock);
		return;
	}

	uk_list_del(&dp->d_link);
	dp->d_ev_link = NULL;

	/* If the dentry should be cached, move it in the cache to the back */
	if (!dp->d_donotcache && !uk_list_empty(&dp->d_link)) {
		UK_ASSERT(dp->d_path);
		ev_dp = NULL;

		if (dentry_cache_standy >= DENTRY_CACHE_MAX_STANDY) {
			/* We start eviction in the line of this dentry to
			 * somewhat balance eviction over all lines
			 */
			ev_line = dp->d_hash;
			ev_num = 0;

			do {
				dp2 = uk_list_last_entry_or_null(
					&dentry_cache[ev_line], struct dentry,
					d_link);

				UK_ASSERT(dp2 != dp);
				if (dp2 && dp2->d_refcnt == 0) {
					uk_list_del(&dp2->d_link);
					dp2->d_ev_link = ev_dp;
					ev_dp = dp2;

					ev_num++;
					dentry_cache_standy--;
				}

				ev_line = (ev_line + 1) % DENTRY_CACHE_LINES;
			} while (ev_num < DENTRY_CACHE_EVICT_BATCH &&
				 ev_line != dp->d_hash);
		}

		dentry_cache_standy++;
		uk_list_add_tail(&dp->d_link, &dentry_cache[dp->d_hash]);
	}

	uk_mutex_unlock(&dentry_cache_lock);

	while (ev_dp) {
		dp = ev_dp->d_ev_link;

		dentry_free(ev_dp);
		ev_dp = dp;
	}
}

void
dentry_init(void)
{
	int i;

	for (i = 0; i < DENTRY_CACHE_LINES; i++)
		UK_INIT_LIST_HEAD(&dentry_cache[i]);
}
