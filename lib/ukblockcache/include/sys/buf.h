/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 1982, 1986, 1989, 1993
 *      The Regents of the University of California.  All rights reserved.
 * (c) UNIX System Laboratories, Inc.
 * All or some portions of this file are derived from material licensed
 * to the University of California by American Telephone and Telegraph
 * Co. or Unix System Laboratories, Inc. and are reproduced herein with
 * the permission of UNIX System Laboratories, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *      @(#)buf.h       8.9 (Berkeley) 3/30/95
 * $FreeBSD$
 */

#ifndef __UK_BLOCKCACHE_BUF__
#define __UK_BLOCKCACHE_BUF__

#include <stdint.h>
#include <sys/stat.h>
#include <ukcache/cache.h>

#ifdef __cplusplus
extern "C" {
#endif

/* newlibc does not provide DEV_BSIZE */
#ifndef DEV_BSIZE
#define DEV_BSIZE 512
#endif

typedef char *caddr_t;
typedef __s64 daddr_t;

struct bufobj_ops;

struct bufobj {
	struct bufobj_ops *bo_ops;
	struct ukcache_key_space *bo_key_space;

	void *bo_private;
	int bo_bsize;
};

struct buf {
	struct bufobj *b_bufobj;
	struct ukcache_node *b_cache_node;
	long b_bcount;
	off_t b_offset;	  /* Offset into file. */
	daddr_t b_blkno;  /* Underlying physical block number. */
	daddr_t b_lblkno; /* Logical block number. */
	caddr_t b_data;
	__u32 b_flags;	/* B_* flags. */
	long b_bufsize; /* Allocated buffer size. */
	long b_resid;

	int b_error;
	__u16 b_iocmd;	 /* BIO_* bio_cmd from bio.h */
	__u16 b_ioflags; /* BIO_* bio_flags from bio.h */
	off_t b_iooffset;
};

struct bufobj_ops {
	void (*bop_strategy)(struct bufobj *bo, struct buf *buf);
	void (*bop_sync)(struct bufobj *buf, int waitfor);
};

int bread_bo(struct bufobj *bo, daddr_t blkno, int size, struct buf **bpp);
struct buf *getblk_bo(struct bufobj *bo, daddr_t blkno, int size, int slpflag,
		      int slptimeo, int flags);

void brelse(struct buf *bp);
void bqrelse(struct buf *bp);
void bufdone(struct buf *bp);

struct buf *incore(struct bufobj *bo, daddr_t blkno);
int bufwait(struct buf *bp);
int bwrite(struct buf *bp);
void bdwrite(struct buf *bp);
void bawrite(struct buf *bp);
int allocbuf(struct buf *bp, int size);
void vfs_busy_pages(struct buf *bp, int clear_modify);
void bstrategy(struct buf *bp);

void vfs_bio_clrbuf(struct buf *bp);
void vfs_bio_bzero_buf(struct buf *bp, int base, int size);
void vfs_bio_brelse(struct buf *bp, int ioflags);
void vfs_bio_set_flags(struct buf *bp, int ioflags);

int vtruncbuf_bo(struct bufobj *bo __unused, off_t length __unused,
		 int blksize __unused);

#define BO_STRATEGY(bo, bp) ((bo)->bo_ops->bop_strategy((bo), (bp)))

/*
 * These flags are kept in b_flags.
 *
 * Notes:
 *
 *	B_ASYNC		VOP calls on bp's are usually async whether or not
 *			B_ASYNC is set, but some subsystems, such as NFS, like
 *			to know what is best for the caller so they can
 *			optimize the I/O.
 *
 *	B_CACHE		This may only be set if the buffer is entirely valid.
 *			The situation where B_DELWRI is set and B_CACHE is
 *			clear MUST be committed to disk by getblk() so
 *			B_DELWRI can also be cleared.  See the comments for
 *			getblk() in kern/vfs_bio.c.  If B_CACHE is clear,
 *			the caller is expected to clear BIO_ERROR and B_INVAL,
 *			set BIO_READ, and initiate an I/O.
 *
 *			The 'entire buffer' is defined to be the range from
 *			0 through b_bcount.
 *
 *	B_CLUSTEROK	This flag is typically set for B_DELWRI buffers
 *			by filesystems that allow clustering when the buffer
 *			is fully dirty and indicates that it may be clustered
 *			with other adjacent dirty buffers.  Note the clustering
 *			may not be used with the stage 1 data write under NFS
 *			but may be used for the commit rpc portion.
 *
 *	B_VMIO		Indicates that the buffer is tied into an VM object.
 *			The buffer's data is always PAGE_SIZE aligned even
 *			if b_bufsize and b_bcount are not.  ( b_bufsize is
 *			always at least DEV_BSIZE aligned, though ).
 *
 *	B_DIRECT	Hint that we should attempt to completely free
 *			the pages underlying the buffer.  B_DIRECT is
 *			sticky until the buffer is released and typically
 *			only has an effect when B_RELBUF is also set.
 *
 */
/* clang-format off */
#define B_ASYNC     0x00000001 /* Start I/O, do not wait. */
#define B_DIRECT    0x00000002 /* direct I/O flag (pls free vmio) */
#define B_CACHE     0x00000004 /* Bread found us in the cache. */
#define B_DELWRI    0x00000008 /* Delay I/O until buffer reused. */
#define B_DONE      0x00000010 /* I/O completed. */
#define B_INVAL     0x00000020 /* Does not contain valid info. */
#define B_NOCACHE   0x00000040 /* Do not cache block after use. */
#define B_CLUSTEROK 0x00000080 /* Pagein op, so swap() can count it. */
#define B_RELBUF    0x00000100 /* Release VMIO buffer. */
#define B_VMIO      0x00000200 /* VMIO flag */
/* clang-format on */

/**
 * bio.h
 */

/* bio_cmd */
#define BIO_READ 0x01  /* Read I/O data */
#define BIO_WRITE 0x02 /* Write I/O data */
#define BIO_FLUSH 0x05 /* Commit outstanding I/O now */

/* bio_flags */
#define BIO_ERROR 0x01 /* An error occurred processing this bio. */
#define BIO_DONE 0x02  /* This bio is finished. */

struct ukcache_key_space *ukblockcache_create_key_space(struct ukcache *cache,
							struct bufobj *bo);

struct ukcache *ukblockcache_get_default_cache(void);

#ifdef __cplusplus
}
#endif

#endif /* __UK_BLOCKCACHE_BUF__ */
