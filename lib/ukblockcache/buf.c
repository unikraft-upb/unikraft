#include "sys/buf.h"
#include "uk/essentials.h"

#include <errno.h>
#include <stdbool.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include <uk/assert.h>
#include <uk/print.h>

#define NODE_PAGE_BLOCK_IDX(BO, BLKNO)                                         \
	((BLKNO) % (__PAGE_SIZE / (BO)->bo_bsize))
#define NODE_PAGE_BLOCK_OFFSET(BO, BLKNO)                                      \
	(((BLKNO) * (BO)->bo_bsize) & ~__PAGE_MASK)
#define NODE_BLKNO_TO_ID(BO, BLKNO) ((BLKNO) * ((BO)->bo_bsize) >> __PAGE_SHIFT)

#define NODE_FLAG_DIRTY 0x1

#define NODE_BITMAP_SHIFT 16
#define NODE_BITMAP_BITS 48

static int node_bitmap_test(struct bufobj *bo, struct ukcache_node *node,
			    daddr_t blkno, int size)
{
	uint64_t idx;
	int i;
	int requested = size / bo->bo_bsize;
	int present = 0;

	for (i = 0; i < requested; i++) {
		idx = NODE_PAGE_BLOCK_IDX(bo, blkno + i);
		if (node->user.u64 & (1 << (NODE_BITMAP_SHIFT + idx)))
			present += 1;
	}
	/* All accesses should happen in a consistent manner. That is the block
	 * sizes must match across the accesses to an address. This implies that
	 * the requested should be either all present or all missing.
	 */
	UK_ASSERT(present == 0 || present == requested);

	return present != 0;
}

static void node_bitmap_set(struct buf *bp)
{
	uint64_t idx;
	int i;
	int blocks = bp->b_bcount / bp->b_bufobj->bo_bsize;

	for (i = 0; i < blocks; i++) {
		idx = NODE_PAGE_BLOCK_IDX(bp->b_bufobj, bp->b_lblkno + i);
		bp->b_cache_node->user.u64 |= (1 << (NODE_BITMAP_SHIFT + idx));
	}
}

int vtruncbuf_bo(struct bufobj *bo __unused, off_t length __unused,
		 int blksize __unused)
{
	// FIXME: implement this
	return 0;
}

static struct buf *buf_alloc(void)
{
	struct buf *bp;

	bp = malloc(sizeof(struct buf));
	if (bp == NULL)
		return NULL;

	bp->b_bufobj = NULL;
	bp->b_cache_node = NULL;
	bp->b_error = 0;
	bp->b_flags = 0;

	bp->b_blkno = bp->b_lblkno = 0;

	bp->b_bcount = 0;
	bp->b_bufsize = 0;
	bp->b_data = NULL;

	bp->b_iooffset = 0;
	bp->b_offset = 0;
	bp->b_resid = 0;

	return bp;
}

void brelse(struct buf *bp)
{
	if (bp == NULL)
		return;

	UK_ASSERT(!!(bp->b_flags & B_VMIO) == !!bp->b_cache_node);

	if (bp->b_flags & B_VMIO)
		ukcache_node_rel(bp->b_cache_node);
	else
		free(bp->b_data);

	free(bp);
}

void bqrelse(struct buf *bp)
{
	brelse(bp);
}

void bufdone(struct buf *bp)
{
	if (bp->b_flags & B_ASYNC)
		brelse(bp);
	else
		bp->b_flags |= B_DONE;
}

int bufwait(struct buf *bp __unused)
{
	/* no-op because everything is sync */

	if (bp->b_ioflags & BIO_ERROR)
		return bp->b_error ? bp->b_error : EIO;

	return 0;
}

int bread_bo(struct bufobj *bo, daddr_t blkno, int size, struct buf **bpp)
{
	struct buf *bp;
	int error;

	uk_pr_debug("bread %p blkno=%" PRIu64 "\n", bo->bo_key_space,
		    (uint64_t)blkno);

	bp = getblk_bo(bo, blkno, size, 0, 0, 0);
	if (bp == NULL)
		return ENOMEM;

	if (bp->b_flags & B_CACHE)
		goto done;

	bp->b_iocmd = BIO_READ;
	bp->b_flags &= ~B_INVAL;
	bp->b_ioflags &= ~BIO_ERROR;
	bp->b_iooffset = bp->b_blkno * DEV_BSIZE;

	bstrategy(bp);
	error = bufwait(bp);
	if (error) {
		brelse(bp);
		*bpp = NULL;
		return error;
	}

	bp->b_iocmd = 0;

done:
	*bpp = bp;
	return 0;
}

static struct buf *buf_new_vmio(struct bufobj *bo, struct ukcache_node *node,
				daddr_t blkno, int size, int flags)
{
	struct buf *bp;

	UK_ASSERT(bo);
	UK_ASSERT(node);
	UK_ASSERT(__PAGE_SIZE % size == 0);
	UK_ASSERT((unsigned long long)size <= __PAGE_SIZE);

	bp = buf_alloc();
	if (bp == NULL)
		return NULL;

	bp->b_bufobj = bo;
	bp->b_cache_node = node;
	bp->b_error = 0;
	bp->b_flags = flags | B_VMIO;

	bp->b_blkno = blkno;
	bp->b_lblkno = blkno;

	bp->b_bcount = size;
	bp->b_bufsize = size;
	bp->b_data = node->data + NODE_PAGE_BLOCK_OFFSET(bo, blkno);
	/* The current buffer must always be contained completely contained in
	 * the cache page
	 */
	UK_ASSERT(bp->b_data + size <= (char *)node->data + __PAGE_SIZE);

	bp->b_iooffset = 0; // PORT: TODO: Remove this?
	bp->b_offset = blkno * bo->bo_bsize;
	bp->b_resid = 0;

	if (node_bitmap_test(bo, node, blkno, size))
		bp->b_flags |= B_CACHE;
	else
		bp->b_flags |= B_INVAL;

	return bp;
}

struct buf *getblk_bo(struct bufobj *bo, daddr_t blkno, int size,
		      int slpflag __unused, int slptimeo __unused, int flags)
{
	struct ukcache_node *node;
	struct ukcache_key_space *key_space = bo->bo_key_space;

	UK_ASSERT(__PAGE_SIZE % size == 0);

	node = ukcache_get(key_space, NODE_BLKNO_TO_ID(bo, blkno));
	if (node == NULL)
		return NULL;

	return buf_new_vmio(bo, node, blkno, size, flags);
}

struct buf *incore(struct bufobj *bo, daddr_t blkno)
{
	struct ukcache_node *node;
	struct ukcache_key_space *key_space = bo->bo_key_space;

	// TODO: introduce method to query cache without automatic fetch
	node = ukcache_get(key_space, NODE_BLKNO_TO_ID(bo, blkno));
	if (node && node_bitmap_test(bo, node, blkno, bo->bo_bsize))
		return buf_new_vmio(bo, node, blkno, bo->bo_bsize, 0);

	// Free node if one was returned
	if (node)
		ukcache_node_rel(node);

	return NULL;
}

static void bundirty(struct buf *bp __unused)
{
	// FIXME: per-block dirty bits
	// bp->b_cache_node->user.u64 &= ~NODE_FLAG_DIRTY;
}

static void bdirty(struct buf *bp)
{
	bp->b_flags &= ~B_INVAL;
	bp->b_cache_node->user.u64 |= NODE_FLAG_DIRTY;
}

int bwrite(struct buf *bp)
{
	int res;

	uk_pr_debug("%s: %p offset=%" PRIu64 "\n", __func__,
		    bp->b_bufobj->bo_key_space, (uint64_t)bp->b_offset);
	bundirty(bp);

	bp->b_flags &= ~B_DONE;
	bp->b_ioflags &= ~BIO_ERROR;
	bp->b_iocmd = BIO_WRITE;
	bp->b_iooffset = bp->b_blkno * DEV_BSIZE;

	bstrategy(bp);

	if ((bp->b_flags & B_ASYNC) == 0) {
		res = bufwait(bp);
		brelse(bp);
		return res;
	}

	return 0;
}

void bdwrite(struct buf *bp)
{
	uk_pr_debug("%s: %p offset=%" PRIu64 "\n", __func__,
		    bp->b_bufobj->bo_key_space, bp->b_offset);
	bdirty(bp);
	node_bitmap_set(bp);
	bqrelse(bp);
}

void bawrite(struct buf *bp)
{
	uk_pr_debug("%s: -- ", __func__);
	bp->b_flags |= B_ASYNC;
	bwrite(bp);
}

int allocbuf(struct buf *bp, int size)
{
	void *data;

	data = realloc(bp->b_data, size);
	if (data == NULL)
		return ENOMEM;

	bp->b_data = data;
	bp->b_bcount = size;
	bp->b_bufsize = size;

	return 0;
}

void vfs_busy_pages(struct buf *bp __unused, int clear_modify __unused) {}

void bstrategy(struct buf *bp)
{
	uk_pr_debug("%s:\tks=%p:\tid=%" PRIu64 "\n", __func__,
		    bp->b_bufobj->bo_key_space, bp->b_cache_node->id);

	UK_ASSERT(bp->b_bufobj != NULL && "no bufobj");
	UK_ASSERT(bp->b_bufobj->bo_ops != NULL && "no bo_ops");
	UK_ASSERT(bp->b_bufobj->bo_ops->bop_strategy != NULL
		  && "no bop_strategy");

	node_bitmap_set(bp);
	bp->b_flags |= B_CACHE;

	BO_STRATEGY(bp->b_bufobj, bp);
}

/*
 * Update buffer flags based on I/O request parameters, optionally releasing the
 * buffer.  If it's VMIO or direct I/O, the buffer pages are released to the VM,
 * where they may be placed on a page queue (VMIO) or freed immediately (direct
 * I/O).  Otherwise, the buffer is released to the cache.
 */
static void b_io_dismiss(struct buf *bp, int ioflag __unused, bool release)
{
	/* TODO: IO_* are vfscore definitions
	if ((ioflag & IO_DIRECT) != 0)
		bp->b_flags |= B_DIRECT;
	if ((ioflag & IO_DIRECT) != 0) {
		bp->b_flags |= B_RELBUF;
		if (release)
			brelse(bp);
	} else
	*/
	if (release)
		bqrelse(bp);
}

void vfs_bio_brelse(struct buf *bp, int ioflags)
{
	b_io_dismiss(bp, ioflags, true);
}

void vfs_bio_set_flags(struct buf *bp, int ioflags)
{
	b_io_dismiss(bp, ioflags, false);
}

void vfs_bio_clrbuf(struct buf *bp)
{
	memset(bp->b_data, 0, bp->b_bcount);
	bp->b_flags &= ~B_INVAL;
	bp->b_ioflags &= ~BIO_ERROR;
	bp->b_resid = 0;
}

void vfs_bio_bzero_buf(struct buf *bp, int base, int size)
{
	memset(bp->b_data + base, 0, size);
}

static int bufobj_cache_evict(struct ukcache_key_space *key_space,
			      struct ukcache_node *node)
{
	struct bufobj *bo = key_space->user;
	struct buf *bp;
	unsigned long i;
	daddr_t blkno;

	uk_pr_debug("evict\tks=%p:\tid=%" PRIu64 "\n", node->key_space,
		    node->id);

	if ((node->user.u64 & NODE_FLAG_DIRTY) == 0)
		return 0;

	for (i = 0; i < (__PAGE_SIZE / bo->bo_bsize); i++) {
		blkno = node->id * (__PAGE_SIZE / bo->bo_bsize) + i;
		if (!node_bitmap_test(bo, node, blkno, bo->bo_bsize))
			continue;

		node->ref_strong++;
		bp = buf_new_vmio(bo, node, blkno, bo->bo_bsize, 0);
		if (bp == NULL) {
			node->ref_strong--;
			return ENOMEM;
		}
		bwrite(bp);
	}

	return 0;
}

static int bufobj_cache_init(struct ukcache_key_space *key_space __maybe_unused,
			     struct ukcache_node *node)
{
	uk_pr_debug("init\tks=%p:\tid=%" PRIu64 "\n", node->key_space,
		    node->id);
	memset(node->data, 0, __PAGE_SIZE);
	node->user.u64 = 0;

	return 0;
}

static struct ukcache_key_space_ops bufobj_key_space_ops = {
	.evict = bufobj_cache_evict,
	.init = bufobj_cache_init,
};

struct ukcache_key_space *ukblockcache_create_key_space(struct ukcache *cache,
							struct bufobj *bo)
{
	return ukcache_key_space_new(cache, &bufobj_key_space_ops, bo);
}

struct ukcache *ukblockcache_get_default_cache(void)
{
	static struct ukcache *default_cache;

	struct uk_alloc *alloc;
	struct uk_allocpool *data_pool, *metadata_pool;

	if (default_cache == NULL) {
		alloc = uk_alloc_get_default();
		data_pool =
			uk_allocpool_alloc(alloc, CONFIG_LIBUKBLOCKCACHE_SIZE,
					__PAGE_SIZE, __PAGE_SIZE);
		metadata_pool =
			uk_allocpool_alloc(alloc, CONFIG_LIBUKBLOCKCACHE_SIZE,
					UKCACHE_METADATA_ENTRY_SIZE, 8);
		default_cache = ukcache_new(alloc, data_pool, metadata_pool);
	}

	return default_cache;
}
