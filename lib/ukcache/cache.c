#include "ukcache/cache.h"

#include "cache_priv.h"

#include <uk/print.h>

static void ukcache_node_free(struct ukcache_node *node);

struct ukcache *ukcache_new(struct uk_alloc *alloc,
			    struct uk_allocpool *data_alloc,
			    struct uk_allocpool *metadata_alloc)
{
	struct ukcache *cache;

	UK_ASSERT(alloc);
	UK_ASSERT(data_alloc);
	UK_ASSERT(metadata_alloc);

	if (uk_allocpool_objlen(metadata_alloc)
	    != UKCACHE_METADATA_ENTRY_SIZE) {
		uk_pr_err("metadatadata pool allocator must allocate metadata-sized entries\n");
		return NULL;
	}

	cache = uk_malloc(alloc, sizeof(struct ukcache));
	if (cache == NULL)
		return NULL;

	cache->alloc = alloc;
	cache->data_alloc = data_alloc;
	cache->metadata_alloc = metadata_alloc;

	UK_INIT_LIST_HEAD(&cache->address_space_list);
	UK_INIT_LIST_HEAD(&cache->lru_list);

	return cache;
}

void ukcache_free(struct ukcache *cache)
{
	UK_ASSERT(uk_list_empty(&cache->address_space_list));
	uk_free(cache->alloc, cache);
}

struct ukcache_key_space *
ukcache_key_space_new(struct ukcache *cache, struct ukcache_key_space_ops *ops,
		      void *user)
{
	struct ukcache_key_space *as;

	UK_ASSERT(cache);
	UK_ASSERT(ops);

	as = uk_malloc(cache->alloc, sizeof(struct ukcache_key_space));
	if (as == NULL)
		return NULL;

	as->ops = ops;
	as->cache = cache;
	as->user = user;
	UK_INIT_LIST_HEAD(&as->nodes_list);

	uk_list_add(&as->address_space_head, &cache->address_space_list);

	return as;
}

void ukcache_key_space_free(struct ukcache_key_space *key_space)
{

	struct ukcache *cache;
	struct ukcache_node *node, *temp_node;

	UK_ASSERT(key_space);

	uk_list_del_init(&key_space->address_space_head);

	uk_list_for_each_entry_safe(node, temp_node, &key_space->nodes_list,
				    nodes_head) {
		// There should be no external references when calling
		// key_space_free.
		UK_ASSERT(node->ref_strong == 1);
		if (key_space->ops->evict)
			key_space->ops->evict(key_space, node);
		UK_ASSERT(node->ref_strong == 1);

		ukcache_node_rel(node);
	}

	cache = key_space->cache;
	uk_free(cache->alloc, key_space);
}

static void ukcache_node_free(struct ukcache_node *node)
{
	struct ukcache *cache;

	UK_ASSERT(node);
	UK_ASSERT(node->ref_strong == 0);

	uk_list_del_init(&node->lru_head);
	uk_list_del_init(&node->nodes_head);

	cache = node->key_space->cache;

	uk_allocpool_return(cache->data_alloc, node->data);
	uk_allocpool_return(cache->metadata_alloc, node);
}

void ukcache_node_rel(struct ukcache_node *node)
{
	struct ukcache *cache;
	/* Ignore null pointers */
	if (node == NULL)
		return;

	cache = node->key_space->cache;

	UK_ASSERT(node->ref_strong > 0);
	node->ref_strong--;

	if (node->ref_strong == 1)
		/* Reference was the last outside user
		 * => Insert node into LRU
		 */
		uk_list_add_tail(&node->lru_head, &cache->lru_list);
	else if (node->ref_strong == 0)
		/* This was the last reference */
		ukcache_node_free(node);
}

static struct ukcache_node *ukcache_node_alloc(struct ukcache *cache)
{
	struct ukcache_node *node;

	UK_ASSERT(cache);

	node = uk_allocpool_take(cache->metadata_alloc);
	if (node == NULL)
		return NULL;

	node->data = uk_allocpool_take(cache->data_alloc);
	if (node->data == NULL) {
		uk_allocpool_return(cache->metadata_alloc, node);
		return NULL;
	}

	return node;
}

static struct ukcache_node *ukcache_find(struct ukcache_key_space *key_space,
					 uint64_t id)
{
	struct ukcache_node *node;

	UK_ASSERT(key_space);

	uk_list_for_each_entry(node, &key_space->nodes_list, nodes_head) {
		if (node->id == id)
			return node;
	}
	return NULL;
}

static struct ukcache_node *ukcache_evict(struct ukcache *cache)
{
	struct ukcache_node *node;
	struct ukcache_key_space *as;

	UK_ASSERT(cache);

	/* Choose a victim */
	node = uk_list_first_entry_or_null(&cache->lru_list,
					   struct ukcache_node, lru_head);
	if (node == NULL)
		return NULL;

	UK_ASSERT(node->ref_strong == 1);

	as = node->key_space;

	if (as->ops->evict && as->ops->evict(as, node))
		return NULL;
	uk_list_del(&node->nodes_head);
	uk_list_del_init(&node->lru_head);

	return node;
}

struct ukcache_node *ukcache_get(struct ukcache_key_space *key_space,
				 uint64_t id)
{
	struct ukcache *cache;
	struct ukcache_node *node;

	UK_ASSERT(key_space);

	cache = key_space->cache;

	/* Search for node in cache */
	/* TODO: use proper lookup structure */
	node = ukcache_find(key_space, id);
	if (node != NULL) {
		uk_list_del_init(&node->lru_head);
		node->ref_strong++;
		return node;
	}

	/* Retrieve metadata structure */
	if ((node = ukcache_node_alloc(cache)) == NULL) {
		node = ukcache_evict(cache);
		if (!node)
			return NULL;
	}

	/* Initialize structure */
	node->key_space = key_space;
	node->id = id;
	node->ref_strong = 1;
	node->user.u64 = 0;
	UK_INIT_LIST_HEAD(&node->lru_head);
	UK_INIT_LIST_HEAD(&node->nodes_head);

	UK_ASSERT(key_space->ops->init);
	if (key_space->ops->init(key_space, node)) {
		ukcache_node_rel(node);
		return NULL;
	}

	node->ref_strong++;

	/* Make it accessible in cache structure */
	uk_list_add(&node->nodes_head, &key_space->nodes_list);

	return node;
}
