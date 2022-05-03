/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Sharan Santhanam <sharan.santhanam@neclab.eu>
 *
 * Copyright (c) 2018, NEC Europe Ltd., NEC Corporation. All rights reserved.
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
/**
 * Inspired from the FreeBSD.
 * Commit-id: a89e7a10d501
 */
#include <uk/config.h>
#include <string.h>
#include <uk/print.h>
#include <uk/errptr.h>
#include <uk/plat/common/cpu.h>
#include <uk/plat/lcpu.h>
#include <uk/sglist.h>
#include <uk/arch/atomic.h>
#include <uk/plat/io.h>
#include <virtio/virtio_ring.h>
#include <virtio/virtqueue.h>

#define VIRTQUEUE_MAX_SIZE  32768
#define to_virtqueue_vring(vq)			\
	__containerof(vq, struct virtqueue_vring, vq)

struct virtqueue_desc_info {
	void *cookie;
};

union virtqueue_free_list {
	struct {
		/* Number of available descriptors in virtqueue */
		__u16 len;
		/* Index of first descriptor in list */
		__u16 head;
	};
	__u32 _atomic;
};

struct virtqueue_vring {
	struct virtqueue vq;
	/* Descriptor Ring */
	struct vring vring;
	/* Reference to the vring */
	void   *vring_mem;
	/* List of free descriptors */
	union virtqueue_free_list free_list;
	/* Index of the last used descriptor by the host */
	__u16 last_used_desc_idx;
	/* Cookie to identify driver buffer */
	struct virtqueue_desc_info vq_info[];
};

/**
 * Static function Declaration(s).
 */
static inline void virtqueue_ring_update_avail(struct virtqueue_vring *vrq,
					       __u16 idx);
static inline void virtqueue_buffer_enqueue_segments(
						    struct virtqueue_vring *vrq,
						    __u16 head,
						    struct uk_sglist *sg,
						    __u16 read_bufs,
						    __u16 write_bufs);
static void virtqueue_vring_init(struct virtqueue_vring *vrq, __u16 nr_desc,
				 __u16 align);

//#define UK_DEBUG_TRACE
#include <uk/trace.h>

UK_TRACEPOINT(trace_vq_intr, "%p %d", void *, int);
UK_TRACEPOINT(trace_vq_intr_ret, "%p %d", void *, int);
UK_TRACEPOINT(trace_vq_intr_en, "%p", void *);
UK_TRACEPOINT(trace_vq_intr_en_fin, "%p %d", void *, __u8);
UK_TRACEPOINT(trace_vq_intr_dis, "%p", void *);

#ifdef VIRTIO_RING_TRACE_QUEUE
UK_TRACEPOINT(trace_vr_enqueue, "%p %u->%u %u %p %u", void *, __u16, __u16,
	      unsigned, __paddr_t, unsigned);
UK_TRACEPOINT(trace_vr_dequeue, "%p %u->%u %p %u", void *, __u16, __u16,
		__paddr_t, unsigned);
UK_TRACEPOINT(trace_vr_set_head, "%p from=%u to=%u", void *, __u16, __u16);
#endif /* VIRTIO_RING_TRACE_QUEUE */

/**
 * Driver implementation
 */
void virtqueue_intr_disable(struct virtqueue *vq)
{
	struct virtqueue_vring *vrq;

	UK_ASSERT(vq);

	vrq = to_virtqueue_vring(vq);
	vrq->vring.avail->flags |= (VRING_AVAIL_F_NO_INTERRUPT);

	trace_vq_intr_dis(vq);
}

int virtqueue_intr_enable(struct virtqueue *vq)
{
	struct virtqueue_vring *vrq;
	int rc = 0;

	UK_ASSERT(vq);

	trace_vq_intr_en(vq);

	vrq = to_virtqueue_vring(vq);
	/* Check if there are no more packets enabled */
	if (!virtqueue_hasdata(vq)) {
		if (vrq->vring.avail->flags | VRING_AVAIL_F_NO_INTERRUPT) {
			vrq->vring.avail->flags &=
				(~VRING_AVAIL_F_NO_INTERRUPT);

			/**
			 * We enabled the interrupts. We ensure it using the
			 * memory barrier and check if there are any further
			 * data available in the queue. The check for data
			 * after enabling the interrupt is to make sure we do
			 * not miss any interrupt while transitioning to enable
			 * interrupt. This is inline with the requirement from
			 * virtio specification section 3.2.2
			 */
			mb();
			/* Check if there are further descriptors */
			if (virtqueue_hasdata(vq)) {
				virtqueue_intr_disable(vq);
				rc = 1;
			}
		}
	} else {
		/**
		 * There are more packet in the virtqueue to be processed while
		 * the interrupt was disabled.
		 */
		rc = 1;
	}

	trace_vq_intr_en_fin(vq, rc);

	return rc;
}

static inline void virtqueue_ring_update_avail(struct virtqueue_vring *vrq,
					__u16 idx)
{
	__u16 avail_idx;

	avail_idx = vrq->vring.avail->idx & (vrq->vring.num - 1);
	/* Adding the idx to available ring */
	vrq->vring.avail->ring[avail_idx] = idx;
	/**
	 * Write barrier to make sure we push the descriptor on the available
	 * descriptor and then increment available index.
	 */
	wmb();
	vrq->vring.avail->idx++;
}

static __u16 virtqueue_desc_alloc(struct virtqueue_vring *vqr, __u16 n)
{
	union virtqueue_free_list old, new;
	__u16 i, idx;

	do {
		old._atomic = UK_READ_ONCE(vqr->free_list._atomic);
		if (old.len < n)
			return VIRTQUEUE_MAX_SIZE;

		/* Walk the free list to find the new head */
		for (i = 0, idx = old.head; i < n; i++) {
			UK_ASSERT(idx < vqr->vring.num);
			idx = vqr->vring.desc[idx].next;
		}

		new.head = idx;
		new.len = old.len - n;

		UK_ASSERT(new.len > 0 || new.head == VIRTQUEUE_MAX_SIZE);
	} while (ukarch_compare_exchange_sync(&vqr->free_list._atomic,
			old._atomic, new._atomic) != new._atomic);

#ifdef VIRTIO_RING_TRACE_QUEUE
	trace_vr_set_head(vqr, old.head, new.head);
#endif /* VIRTIO_RING_TRACE_QUEUE */

	return old.head;
}

static __u16 virtqueue_desc_free(struct virtqueue_vring *vqr, __u16 idx)
{
	union virtqueue_free_list old, new;
	struct vring_desc *desc;
	__u16 len = 1;

	new.head = idx;

	UK_ASSERT(idx < vqr->vring.num);
	desc = &vqr->vring.desc[idx];

	/* Walk provided list to find its last descriptor */
	while (1) {
#ifdef VIRTIO_RING_TRACE_QUEUE
		trace_vr_dequeue(vqr, idx, desc->next, desc->addr, desc->len);
#endif /* VIRTIO_RING_TRACE_QUEUE */
		desc->addr = 0x0;
		desc->len = 0x0;
		idx = desc->next;

		if (!(desc->flags & VRING_DESC_F_NEXT))
			break;

		UK_ASSERT(desc->next < vqr->vring.num);
		desc = &vqr->vring.desc[desc->next];
		len++;
	}

	UK_ASSERT(len <= vqr->vring.num);

	/* Try to insert the given descriptor list to the free list */
	do {
		old._atomic = UK_READ_ONCE(vqr->free_list._atomic);

		UK_ASSERT(old.len > 0 || old.head == VIRTQUEUE_MAX_SIZE);
		UK_ASSERT(old.len <= vqr->vring.num - len);

		new.len = old.len + len;
		desc->next = old.head;
	} while (ukarch_compare_exchange_sync(&vqr->free_list._atomic,
			old._atomic, new._atomic) != new._atomic);

#ifdef VIRTIO_RING_TRACE_QUEUE
	trace_vr_set_head(vqr, old.head, new.head);
#endif /* VIRTIO_RING_TRACE_QUEUE */

	return new.len;
}

int virtqueue_notify_enabled(struct virtqueue *vq)
{
	struct virtqueue_vring *vrq;

	UK_ASSERT(vq);
	vrq = to_virtqueue_vring(vq);

	return ((vrq->vring.used->flags & VRING_USED_F_NO_NOTIFY) == 0);
}

static inline void virtqueue_buffer_enqueue_segments(
		struct virtqueue_vring *vrq,
		__u16 head, struct uk_sglist *sg, __u16 read_bufs,
		__u16 write_bufs)
{
	__u16 i, idx;
	__u32 total_desc;
	struct uk_sglist_seg *segs;

	total_desc = read_bufs + write_bufs;

	for (i = 0, idx = head; i < total_desc; i++) {
		segs = &sg->sg_segs[i];

		UK_ASSERT(idx < vrq->vring.num);
		vrq->vring.desc[idx].addr = segs->ss_paddr;
		vrq->vring.desc[idx].len = segs->ss_len;
		vrq->vring.desc[idx].flags = 0;
		if (i >= read_bufs)
			vrq->vring.desc[idx].flags |= VRING_DESC_F_WRITE;

		if (i < total_desc - 1)
			vrq->vring.desc[idx].flags |= VRING_DESC_F_NEXT;

#ifdef VIRTIO_RING_TRACE_QUEUE
		trace_vr_enqueue(&vrq->vq, idx, vrq->vring.desc[idx].next,
				 vrq->vring.desc[idx].flags,
				 segs->ss_paddr, segs->ss_len);
#endif /* VIRTIO_RING_TRACE_QUEUE */

		idx = vrq->vring.desc[idx].next;
	}
}

int virtqueue_hasdata(struct virtqueue *vq)
{
	struct virtqueue_vring *vring;

	UK_ASSERT(vq);

	vring = to_virtqueue_vring(vq);
	return (vring->last_used_desc_idx != vring->vring.used->idx);
}

__u64 virtqueue_feature_negotiate(__u64 feature_set)
{
	__u64 feature = (1ULL << VIRTIO_TRANSPORT_F_START) - 1;

	/**
	 * Currently out vring driver does not support any ring feature. We will
	 * add support to transport feature in the future.
	 */
	feature &= feature_set;
	return feature;
}

int virtqueue_ring_interrupt(void *obj)
{
	struct virtqueue *vq = (struct virtqueue *)obj;
	int rc;
	UK_ASSERT(vq);

	trace_vq_intr(vq, virtqueue_hasdata(vq));

	if (!virtqueue_hasdata(vq)) {
		trace_vq_intr_ret(vq, -1);
		return 0;
	}

	/*
	 * If the device provides a callback to handle ISRs, we let the
	 * driver decide if the IRQ has been handled or not. Otherwise, we
	 * expect IRQs to be handled automatically (i.e., the driver just
	 * ignores them at this point).
	 */
	rc = (likely(vq->vq_callback)) ? vq->vq_callback(vq, vq->priv) : 1;
	trace_vq_intr_ret(vq, rc);

	return rc;
}

__paddr_t virtqueue_physaddr(struct virtqueue *vq)
{
	struct virtqueue_vring *vrq = NULL;

	UK_ASSERT(vq);

	vrq = to_virtqueue_vring(vq);
	return ukplat_virt_to_phys(vrq->vring_mem);
}

__paddr_t virtqueue_get_avail_addr(struct virtqueue *vq)
{
	struct virtqueue_vring *vrq = NULL;

	UK_ASSERT(vq);

	vrq = to_virtqueue_vring(vq);
	return virtqueue_physaddr(vq) +
		((char *)vrq->vring.avail - (char *)vrq->vring.desc);
}

__paddr_t virtqueue_get_used_addr(struct virtqueue *vq)
{
	struct virtqueue_vring *vrq = NULL;

	UK_ASSERT(vq);

	vrq = to_virtqueue_vring(vq);
	return virtqueue_physaddr(vq) +
		((char *)vrq->vring.used - (char *)vrq->vring.desc);
}

unsigned int virtqueue_vring_get_num(struct virtqueue *vq)
{
	struct virtqueue_vring *vrq = NULL;

	UK_ASSERT(vq);

	vrq = to_virtqueue_vring(vq);
	return vrq->vring.num;
}

int virtqueue_buffer_dequeue(struct virtqueue *vq, void **cookie, __u32 *len)
{
	struct virtqueue_vring *vqr = NULL;
	struct vring_used_elem *elem;
	__u16 used_idx, head_idx;

	UK_ASSERT(vq);
	UK_ASSERT(cookie);
	vqr = to_virtqueue_vring(vq);

	/* No new descriptor since last dequeue operation */
	if (!virtqueue_hasdata(vq))
		return -ENOMSG;

	used_idx = vqr->last_used_desc_idx++ & (vqr->vring.num - 1);
	elem = &vqr->vring.used->ring[used_idx];

	/* We are reading from the used descriptor information updated by the
	 * host.
	 *
	 * TODO: Does this make sense?
	 */
	rmb();

	head_idx = elem->id;
	if (len)
		*len = elem->len;

	*cookie = vqr->vq_info[head_idx].cookie;

	return (vqr->vring.num - virtqueue_desc_free(vqr, head_idx));
}

int virtqueue_buffer_enqueue(struct virtqueue *vq, void *cookie,
			     struct uk_sglist *sg, __u16 read_bufs,
			     __u16 write_bufs)
{
	struct virtqueue_vring *vqr;
	__u16 head_idx;
	__u32 total_desc;

	UK_ASSERT(vq);
	vqr = to_virtqueue_vring(vq);

	total_desc = read_bufs + write_bufs;
	if (unlikely(total_desc < 1 || total_desc > vqr->vring.num)) {
		uk_pr_err("Invalid number of descriptors: %"__PRIu32"\n",
			  total_desc);
		return -EINVAL;
	}

	/* Allocate descriptors from the virtqueue */
	head_idx = virtqueue_desc_alloc(vqr, total_desc);
	if (unlikely(head_idx == VIRTQUEUE_MAX_SIZE)) {
		//uk_pr_err("Not enough descriptors available: %"__PRIu32"\n",
		//	  total_desc);
		return -ENOSPC;
	}

	UK_ASSERT(head_idx < vqr->vring.num);
	UK_ASSERT(cookie);

	/* Additional information to reconstruct the data buffer */
	vqr->vq_info[head_idx].cookie = cookie;

	virtqueue_buffer_enqueue_segments(vqr, head_idx, sg, read_bufs,
					  write_bufs);

	virtqueue_ring_update_avail(vqr, head_idx);

	return 0;
}

static void virtqueue_vring_init(struct virtqueue_vring *vrq, __u16 nr_desc,
				 __u16 align)
{
	int i = 0;

	vring_init(&vrq->vring, nr_desc, vrq->vring_mem, align);

	vrq->free_list.len = vrq->vring.num;
	vrq->free_list.head = 0;
	vrq->last_used_desc_idx = 0;
	for (i = 0; i < nr_desc - 1; i++) {
		vrq->vring.desc[i].addr = 0x0;
		vrq->vring.desc[i].len = 0x0;
		vrq->vring.desc[i].next = i + 1;
	}
	/**
	 * When we reach this descriptor we have completely used all the
	 * descriptor in the vring.
	 */
	vrq->vring.desc[nr_desc - 1].next = VIRTQUEUE_MAX_SIZE;
}

struct virtqueue *virtqueue_create(__u16 queue_id, __u16 nr_descs, __u16 align,
				   virtqueue_callback_t callback,
				   virtqueue_notify_host_t notify,
				   struct virtio_dev *vdev, struct uk_alloc *a)
{
	struct virtqueue_vring *vrq;
	struct virtqueue *vq;
	int rc;
	size_t ring_size = 0;

	UK_ASSERT(a);

	vrq = uk_malloc(a, sizeof(*vrq) +
			nr_descs * sizeof(struct virtqueue_desc_info));
	if (!vrq) {
		uk_pr_err("Allocation of virtqueue failed\n");
		rc = -ENOMEM;
		goto err_exit;
	}
	/**
	 * Initialize the value before referencing it in
	 * uk_posix_memalign as we don't set NULL on all failures in the
	 * allocation.
	 */
	vrq->vring_mem = NULL;

	ring_size = vring_size(nr_descs, align);
	if (uk_posix_memalign(a, &vrq->vring_mem,
			      __PAGE_SIZE, ring_size) != 0) {
		uk_pr_err("Allocation of vring failed\n");
		rc = -ENOMEM;
		goto err_freevq;
	}
	memset(vrq->vring_mem, 0, ring_size);
	virtqueue_vring_init(vrq, nr_descs, align);

	vq = &vrq->vq;
	vq->queue_id = queue_id;
	vq->vdev = vdev;
	vq->vq_callback = callback;
	vq->vq_notify_host = notify;
	return vq;

err_freevq:
	uk_free(a, vrq);
err_exit:
	return ERR2PTR(rc);
}

void virtqueue_destroy(struct virtqueue *vq, struct uk_alloc *a)
{
	struct virtqueue_vring *vrq;

	UK_ASSERT(vq);

	vrq = to_virtqueue_vring(vq);

	/* Free the ring */
	uk_free(a, vrq->vring_mem);

	/* Free the virtqueue metadata */
	uk_free(a, vrq);
}

int virtqueue_is_full(struct virtqueue *vq)
{
	struct virtqueue_vring *vrq;

	UK_ASSERT(vq);

	vrq = to_virtqueue_vring(vq);
	return (vrq->free_list.len == 0);
}
