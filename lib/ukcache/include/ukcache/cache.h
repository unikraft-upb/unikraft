/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Marco Schlumpp <marco.schlumpp@gmail.com>
 *
 * Copyright (c) 2021 Karlsruhe Institute of Technology (KIT)
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

#ifndef __UKCACHE_CACHE_H__
#define __UKCACHE_CACHE_H__

#include <uk/config.h>
#include <uk/allocpool.h>
#include <uk/list.h>

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ukcache;

struct ukcache_key_space {
	struct ukcache *cache;
	struct uk_list_head address_space_head;
	struct ukcache_key_space_ops *ops;
	struct uk_list_head nodes_list;
	void *user;
};

struct ukcache_node {
	uint64_t id;
	struct ukcache_key_space *key_space;
	struct uk_list_head lru_head;
	struct uk_list_head nodes_head;
	uint32_t ref_strong;
	uint32_t ref_weak;

	union {
		uint64_t u64;
		void *ptr;
	} user;
	void *data;
};

#define UKCACHE_METADATA_ENTRY_SIZE sizeof(struct ukcache_node)

typedef int (*cache_evict_t)(struct ukcache_key_space *key_space,
			     struct ukcache_node *node);
typedef int (*cache_init_t)(struct ukcache_key_space *key_space,
			    struct ukcache_node *node);

struct ukcache_key_space_ops {
	/* Function to initialize a value in the cache (Required) */
	cache_init_t init;
	/* Function which is called when a value is evicted (Optional) */
	cache_evict_t evict;
};

struct ukcache *ukcache_new(struct uk_alloc *alloc,
			    struct uk_allocpool *data_alloc,
			    struct uk_allocpool *metadata_alloc);

void ukcache_free(struct ukcache *cache);

struct ukcache_key_space *
ukcache_key_space_new(struct ukcache *cache, struct ukcache_key_space_ops *ops,
		      void *user);

void ukcache_key_space_free(struct ukcache_key_space *key_space);

struct ukcache_node *ukcache_get(struct ukcache_key_space *key_space,
				 uint64_t id);

void ukcache_node_rel(struct ukcache_node *node);

__sz ukcache_reserved_memory(struct ukcache *cache);
__sz ukcache_used_memory(struct ukcache *cache);

#ifdef __cplusplus
}
#endif

#endif /* __UKCACHE_CACHE_H__ */
