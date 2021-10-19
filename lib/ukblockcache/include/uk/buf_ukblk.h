#ifndef __UK_BLOCKCACHE_BUF_UKBLK__
#define __UK_BLOCKCACHE_BUF_UKBLK__

#include <uk/blkdev.h>
#include <sys/buf.h>

#ifdef __cplusplus
extern "C" {
#endif

struct bufobj *ukblockcache_create_blkdev_bufobj(struct uk_blkdev *dev,
						 struct ukcache *cache);

void ukblockcache_free_blkdev_bufobj(struct bufobj *bo);

#ifdef __cplusplus
}
#endif

#endif /* __UK_BLOCKCACHE_BUF_UKBLK__ */
