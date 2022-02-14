/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (C) 2014 Cloudius Systems, Ltd.
 * Copyright (c) 2019, NEC Europe Ltd., NEC Corporation.
 *
 * All rights reserved.
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

#ifndef _VFSCORE_DENTRY_H_
#define _VFSCORE_DENTRY_H_

#include <uk/mutex.h>
#include <uk/list.h>

struct vnode;

struct dentry {
	union {
		/** Link into cache for fast lookup */
		struct uk_list_head d_link;
		/** Link into eviction list */
		struct dentry *d_ev_link;
	};
	/** Hash over mount point and path */
	unsigned int d_hash;
	/** Set if dentry should not be kept open after last reference */
	unsigned int d_donotcache : 1;

	/** Number of references */
	unsigned int d_refcnt;

	/** Path in file system */
	char *d_path;

	/** Pointer to vnode */
	struct vnode *d_vnode;
	/** Link into vnode's names list */
	struct uk_list_head d_names_link;

	/** Pointer to mount point */
	struct mount *d_mount;

	/** Pointer to parent dentry */
	struct dentry *d_parent;
	/** List of child dentries */
	struct uk_list_head d_child_list;
	/** Link into the parent's dentry child_list */
	struct uk_list_head d_child_link;

	/** Lock to synchronize access */
	struct uk_mutex d_lock;
};

struct dentry *dentry_alloc(struct dentry *parent_dp, struct vnode *vp,
			    const char *path);
struct dentry *dentry_lookup(struct mount *mp, const char *path);
int dentry_move(struct dentry *dp, struct dentry *parent_dp, const char *path);
void dentry_remove(struct dentry *dp);
void dref(struct dentry *dp);
void drele(struct dentry *dp);

#endif /* _VFSCORE_DENTRY_H_ */
