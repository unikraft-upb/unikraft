#include "uk/buf_ukblk.h"

#include <string.h>

static void ukblk_bufobj_strategy(struct bufobj *bo, struct buf *buf)
{
	int op, err;
	struct uk_blkdev *blkdev;

	if (buf->b_iocmd == BIO_READ)
		op = UK_BLKREQ_READ;
	else if (buf->b_iocmd == BIO_WRITE)
		op = UK_BLKREQ_WRITE;
	else if (buf->b_iocmd == BIO_FLUSH)
		op = UK_BLKREQ_FFLUSH;
	else
		UK_CRASH("invalid iocmd");

	blkdev = (struct uk_blkdev *)bo->bo_private;

	UK_ASSERT((buf->b_bcount % uk_blkdev_ssize(blkdev)) == 0);
	/* Currently, this implementation assumes that iooffset = blkno *
	 * DEV_BSIZE
	 */
	UK_ASSERT(buf->b_iooffset == buf->b_blkno * 512);
	err = uk_blkdev_sync_io(blkdev, 0, op, buf->b_blkno,
				buf->b_bcount / uk_blkdev_ssize(blkdev),
				buf->b_data);
	if (err) {
		buf->b_ioflags |= BIO_ERROR;
		buf->b_error = err;
		uk_pr_err(
		    "failed to fulfill request iocmd=%d blkno=%lu bcount=%ld\n",
		    buf->b_iocmd, buf->b_blkno, buf->b_bcount);
	} else
		buf->b_ioflags |= BIO_DONE;

	/* In case of B_ASYNC, we are responsible for cleaning up the buffer.
	 * For this synchronous implementation we are already at this point.
	 */
	if (buf->b_flags & B_ASYNC)
		brelse(buf);
}

static void ukblk_bufobj_sync(struct bufobj *buf __unused, int waitfor __unused)
{
	// FIXME: implement this
}

static struct bufobj_ops ukblk_bufobj_ops = {
	.bop_sync = ukblk_bufobj_sync,
	.bop_strategy = ukblk_bufobj_strategy,
};

struct bufobj *ukblockcache_create_blkdev_bufobj(struct uk_blkdev *dev,
						 struct ukcache *cache)
{
	struct bufobj *bo;

	UK_ASSERT(dev);
	UK_ASSERT(cache);

	bo = uk_malloc(uk_alloc_get_default(), sizeof(struct bufobj));
	if (bo == NULL)
		return NULL;

	bo->bo_key_space = ukblockcache_create_key_space(cache, bo);
	if (bo->bo_key_space == NULL) {
		uk_free(uk_alloc_get_default(), bo);
		return NULL;
	}
	bo->bo_ops = &ukblk_bufobj_ops;
	bo->bo_private = dev;
	bo->bo_bsize = DEV_BSIZE;

	return bo;
}

void ukblockcache_free_blkdev_bufobj(struct bufobj *bo)
{
	ukcache_key_space_free(bo->bo_key_space);
	uk_free(uk_alloc_get_default(), bo);
}
