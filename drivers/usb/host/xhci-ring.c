// SPDX-License-Identifier: GPL-2.0+
/*
 * USB HOST XHCI Controller stack
 *
 * Based on xHCI host controller driver in linux-kernel
 * by Sarah Sharp.
 *
 * Copyright (C) 2008 Intel Corp.
 * Author: Sarah Sharp
 *
 * Copyright (C) 2013 Samsung Electronics Co.Ltd
 * Authors: Vivek Gautam <gautam.vivek@samsung.com>
 *	    Vikas Sajjan <vikas.sajjan@samsung.com>
 */

#include <cpu_func.h>
#include <log.h>
#include <asm/byteorder.h>
#include <usb.h>
#include <watchdog.h>
#include <asm/unaligned.h>
#include <linux/bug.h>
#include <linux/errno.h>

#include <usb/xhci.h>

/*
 * Returns zero if the TRB isn't in this segment, otherwise it returns the DMA
 * address of the TRB.
 */
dma_addr_t xhci_trb_virt_to_dma(struct xhci_segment *seg,
				union xhci_trb *trb)
{
	unsigned long segment_offset;

	if (!seg || !trb || trb < seg->trbs)
		return 0;
	/* offset in TRBs */
	segment_offset = trb - seg->trbs;
	if (segment_offset >= TRBS_PER_SEGMENT)
		return 0;
	return seg->dma + (segment_offset * sizeof(*trb));
}

/**
 * Is this TRB a link TRB or was the last TRB the last TRB in this event ring
 * segment?  I.e. would the updated event TRB pointer step off the end of the
 * event seg ?
 *
 * @param ctrl	Host controller data structure
 * @param ring	pointer to the ring
 * @param seg	poniter to the segment to which TRB belongs
 * @param trb	poniter to the ring trb
 * Return: 1 if this TRB a link TRB else 0
 */
static int last_trb(struct xhci_ctrl *ctrl, struct xhci_ring *ring,
			struct xhci_segment *seg, union xhci_trb *trb)
{
	if (ring == ctrl->event_ring)
		return trb == &seg->trbs[TRBS_PER_SEGMENT];
	else
		return TRB_TYPE_LINK_LE32(trb->link.control);
}

/**
 * Does this link TRB point to the first segment in a ring,
 * or was the previous TRB the last TRB on the last segment in the ERST?
 *
 * @param ctrl	Host controller data structure
 * @param ring	pointer to the ring
 * @param seg	poniter to the segment to which TRB belongs
 * @param trb	poniter to the ring trb
 * Return: 1 if this TRB is the last TRB on the last segment else 0
 */
static bool last_trb_on_last_seg(struct xhci_ctrl *ctrl,
				 struct xhci_ring *ring,
				 struct xhci_segment *seg,
				 union xhci_trb *trb)
{
	if (ring == ctrl->event_ring)
		return ((trb == &seg->trbs[TRBS_PER_SEGMENT]) &&
			(seg->next == ring->first_seg));
	else
		return le32_to_cpu(trb->link.control) & LINK_TOGGLE;
}

/**
 * See Cycle bit rules. SW is the consumer for the event ring only.
 * Don't make a ring full of link TRBs.  That would be dumb and this would loop.
 *
 * If we've just enqueued a TRB that is in the middle of a TD (meaning the
 * chain bit is set), then set the chain bit in all the following link TRBs.
 * If we've enqueued the last TRB in a TD, make sure the following link TRBs
 * have their chain bit cleared (so that each Link TRB is a separate TD).
 *
 * Section 6.4.4.1 of the 0.95 spec says link TRBs cannot have the chain bit
 * set, but other sections talk about dealing with the chain bit set.  This was
 * fixed in the 0.96 specification errata, but we have to assume that all 0.95
 * xHCI hardware can't handle the chain bit being cleared on a link TRB.
 *
 * @param ctrl	Host controller data structure
 * @param ring	pointer to the ring
 * @param more_trbs_coming	flag to indicate whether more trbs
 *				are expected or NOT.
 *				Will you enqueue more TRBs before calling
 *				prepare_ring()?
 * Return: none
 */
static void inc_enq(struct xhci_ctrl *ctrl, struct xhci_ring *ring,
						bool more_trbs_coming)
{
	u32 chain;
	union xhci_trb *next;

	chain = le32_to_cpu(ring->enqueue->generic.field[3]) & TRB_CHAIN;
	next = ++(ring->enqueue);

	/*
	 * Update the dequeue pointer further if that was a link TRB or we're at
	 * the end of an event ring segment (which doesn't have link TRBS)
	 */
	while (last_trb(ctrl, ring, ring->enq_seg, next)) {
		if (ring != ctrl->event_ring) {
			/*
			 * If the caller doesn't plan on enqueueing more
			 * TDs before ringing the doorbell, then we
			 * don't want to give the link TRB to the
			 * hardware just yet.  We'll give the link TRB
			 * back in prepare_ring() just before we enqueue
			 * the TD at the top of the ring.
			 */
			if (!chain && !more_trbs_coming)
				break;

			/*
			 * If we're not dealing with 0.95 hardware or
			 * isoc rings on AMD 0.96 host,
			 * carry over the chain bit of the previous TRB
			 * (which may mean the chain bit is cleared).
			 */
			next->link.control &= cpu_to_le32(~TRB_CHAIN);
			next->link.control |= cpu_to_le32(chain);

			next->link.control ^= cpu_to_le32(TRB_CYCLE);
			xhci_flush_cache((uintptr_t)next,
					 sizeof(union xhci_trb));
		}
		/* Toggle the cycle bit after the last ring segment. */
		if (last_trb_on_last_seg(ctrl, ring,
					ring->enq_seg, next))
			ring->cycle_state = (ring->cycle_state ? 0 : 1);

		ring->enq_seg = ring->enq_seg->next;
		ring->enqueue = ring->enq_seg->trbs;
		next = ring->enqueue;
	}
}

/**
 * See Cycle bit rules. SW is the consumer for the event ring only.
 * Don't make a ring full of link TRBs.  That would be dumb and this would loop.
 *
 * @param ctrl	Host controller data structure
 * @param ring	Ring whose Dequeue TRB pointer needs to be incremented.
 * return none
 */
static void inc_deq(struct xhci_ctrl *ctrl, struct xhci_ring *ring)
{
	do {
		/*
		 * Update the dequeue pointer further if that was a link TRB or
		 * we're at the end of an event ring segment (which doesn't have
		 * link TRBS)
		 */
		if (last_trb(ctrl, ring, ring->deq_seg, ring->dequeue)) {
			if (ring == ctrl->event_ring &&
					last_trb_on_last_seg(ctrl, ring,
						ring->deq_seg, ring->dequeue)) {
				ring->cycle_state = (ring->cycle_state ? 0 : 1);
			}
			ring->deq_seg = ring->deq_seg->next;
			ring->dequeue = ring->deq_seg->trbs;
		} else {
			ring->dequeue++;
		}
	} while (last_trb(ctrl, ring, ring->deq_seg, ring->dequeue));
}

/**
 * Generic function for queueing a TRB on a ring.
 * The caller must have checked to make sure there's room on the ring.
 *
 * @param	more_trbs_coming:   Will you enqueue more TRBs before calling
 *				prepare_ring()?
 * @param ctrl	Host controller data structure
 * @param ring	pointer to the ring
 * @param more_trbs_coming	flag to indicate whether more trbs
 * @param trb_fields	pointer to trb field array containing TRB contents
 * Return: pointer to the enqueued trb
 */
static dma_addr_t queue_trb(struct xhci_ctrl *ctrl, struct xhci_ring *ring,
			    bool more_trbs_coming, unsigned int *trb_fields)
{
	struct xhci_generic_trb *trb;
	dma_addr_t addr;
	int i;

	trb = &ring->enqueue->generic;

	for (i = 0; i < 4; i++)
		trb->field[i] = cpu_to_le32(trb_fields[i]);

	xhci_flush_cache((uintptr_t)trb, sizeof(struct xhci_generic_trb));

	addr = xhci_trb_virt_to_dma(ring->enq_seg, (union xhci_trb *)trb);

	inc_enq(ctrl, ring, more_trbs_coming);

	return addr;
}

/**
 * Does various checks on the endpoint ring, and makes it ready
 * to queue num_trbs.
 *
 * @param ctrl		Host controller data structure
 * @param ep_ring	pointer to the EP Transfer Ring
 * @param ep_state	State of the End Point
 * Return: error code in case of invalid ep_state, 0 on success
 */
static int prepare_ring(struct xhci_ctrl *ctrl, struct xhci_ring *ep_ring,
							u32 ep_state)
{
	union xhci_trb *next = ep_ring->enqueue;

	/* Make sure the endpoint has been added to xHC schedule */
	switch (ep_state) {
	case EP_STATE_DISABLED:
		/*
		 * USB core changed config/interfaces without notifying us,
		 * or hardware is reporting the wrong state.
		 */
		puts("WARN urb submitted to disabled ep\n");
		return -ENOENT;
	case EP_STATE_ERROR:
		puts("WARN waiting for error on ep to be cleared\n");
		return -EINVAL;
	case EP_STATE_HALTED:
		puts("WARN endpoint is halted\n");
		return -EINVAL;
	case EP_STATE_STOPPED:
	case EP_STATE_RUNNING:
		debug("EP STATE RUNNING.\n");
		break;
	default:
		puts("ERROR unknown endpoint state for ep\n");
		return -EINVAL;
	}

	while (last_trb(ctrl, ep_ring, ep_ring->enq_seg, next)) {
		/*
		 * If we're not dealing with 0.95 hardware or isoc rings
		 * on AMD 0.96 host, clear the chain bit.
		 */
		next->link.control &= cpu_to_le32(~TRB_CHAIN);

		next->link.control ^= cpu_to_le32(TRB_CYCLE);

		xhci_flush_cache((uintptr_t)next, sizeof(union xhci_trb));

		/* Toggle the cycle bit after the last ring segment. */
		if (last_trb_on_last_seg(ctrl, ep_ring,
					ep_ring->enq_seg, next))
			ep_ring->cycle_state = (ep_ring->cycle_state ? 0 : 1);
		ep_ring->enq_seg = ep_ring->enq_seg->next;
		ep_ring->enqueue = ep_ring->enq_seg->trbs;
		next = ep_ring->enqueue;
	}

	return 0;
}

/**
 * Generic function for queueing a command TRB on the command ring.
 * Check to make sure there's room on the command ring for one command TRB.
 *
 * @param ctrl		Host controller data structure
 * @param ptr		Pointer address to write in the first two fields (opt.)
 * @param slot_id	Slot ID to encode in the flags field (opt.)
 * @param ep_index	Endpoint index to encode in the flags field (opt.)
 * @param cmd		Command type to enqueue
 * Return: none
 */
void xhci_queue_command(struct xhci_ctrl *ctrl, dma_addr_t addr, u32 slot_id,
			u32 ep_index, trb_type cmd)
{
	u32 fields[4];

	BUG_ON(prepare_ring(ctrl, ctrl->cmd_ring, EP_STATE_RUNNING));

	fields[0] = lower_32_bits(addr);
	fields[1] = upper_32_bits(addr);
	fields[2] = 0;
	fields[3] = TRB_TYPE(cmd) | SLOT_ID_FOR_TRB(slot_id) |
		    ctrl->cmd_ring->cycle_state;

	/*
	 * Only 'reset endpoint', 'stop endpoint' and 'set TR dequeue pointer'
	 * commands need endpoint id encoded.
	 */
	if (cmd >= TRB_RESET_EP && cmd <= TRB_SET_DEQ)
		fields[3] |= EP_ID_FOR_TRB(ep_index);

	queue_trb(ctrl, ctrl->cmd_ring, false, fields);

	/* Ring the command ring doorbell */
	xhci_writel(&ctrl->dba->doorbell[0], DB_VALUE_HOST);
}

/*
 * For xHCI 1.0 host controllers, TD size is the number of max packet sized
 * packets remaining in the TD (*not* including this TRB).
 *
 * Total TD packet count = total_packet_count =
 *     DIV_ROUND_UP(TD size in bytes / wMaxPacketSize)
 *
 * Packets transferred up to and including this TRB = packets_transferred =
 *     rounddown(total bytes transferred including this TRB / wMaxPacketSize)
 *
 * TD size = total_packet_count - packets_transferred
 *
 * For xHCI 0.96 and older, TD size field should be the remaining bytes
 * including this TRB, right shifted by 10
 *
 * For all hosts it must fit in bits 21:17, so it can't be bigger than 31.
 * This is taken care of in the TRB_TD_SIZE() macro
 *
 * The last TRB in a TD must have the TD size set to zero.
 *
 * @param ctrl	host controller data structure
 * @param transferred	total size sent so far
 * @param trb_buff_len	length of the TRB Buffer
 * @param td_total_len	total packet count
 * @param maxp	max packet size of current pipe
 * @param more_trbs_coming	indicate last trb in TD
 * Return: remainder
 */
static u32 xhci_td_remainder(struct xhci_ctrl *ctrl, int transferred,
			     int trb_buff_len, unsigned int td_total_len,
			     int maxp, bool more_trbs_coming)
{
	u32 total_packet_count;

	/* MTK xHCI 0.96 contains some features from 1.0 */
	if (ctrl->hci_version < 0x100 && !(ctrl->quirks & XHCI_MTK_HOST))
		return ((td_total_len - transferred) >> 10);

	/* One TRB with a zero-length data packet. */
	if (!more_trbs_coming || (transferred == 0 && trb_buff_len == 0) ||
	    trb_buff_len == td_total_len)
		return 0;

	/* for MTK xHCI 0.96, TD size include this TRB, but not in 1.x */
	if ((ctrl->quirks & XHCI_MTK_HOST) && (ctrl->hci_version < 0x100))
		trb_buff_len = 0;

	total_packet_count = DIV_ROUND_UP(td_total_len, maxp);

	/* Queueing functions don't count the current TRB into transferred */
	return (total_packet_count - ((transferred + trb_buff_len) / maxp));
}

/**
 * Ring the doorbell of the End Point
 *
 * @param udev		pointer to the USB device structure
 * @param ep_index	index of the endpoint
 * @param start_cycle	cycle flag of the first TRB
 * @param start_trb	pionter to the first TRB
 * Return: none
 */
static void giveback_first_trb(struct usb_device *udev, int ep_index,
				int start_cycle,
				struct xhci_generic_trb *start_trb)
{
	struct xhci_ctrl *ctrl = xhci_get_ctrl(udev);

	/*
	 * Pass all the TRBs to the hardware at once and make sure this write
	 * isn't reordered.
	 */
	if (start_cycle)
		start_trb->field[3] |= cpu_to_le32(start_cycle);
	else
		start_trb->field[3] &= cpu_to_le32(~TRB_CYCLE);

	xhci_flush_cache((uintptr_t)start_trb, sizeof(struct xhci_generic_trb));

	/* Ringing EP doorbell here */
	xhci_writel(&ctrl->dba->doorbell[udev->slot_id],
				DB_VALUE(ep_index, 0));

	return;
}

/**** POLLING mechanism for XHCI ****/

/**
 * Finalizes a handled event TRB by advancing our dequeue pointer and giving
 * the TRB back to the hardware for recycling. Must call this exactly once at
 * the end of each event handler, and not touch the TRB again afterwards.
 *
 * @param ctrl	Host controller data structure
 * Return: none
 */
void xhci_acknowledge_event(struct xhci_ctrl *ctrl)
{
	dma_addr_t deq;

	/* Advance our dequeue pointer to the next event */
	inc_deq(ctrl, ctrl->event_ring);

	/* Inform the hardware */
	deq = xhci_trb_virt_to_dma(ctrl->event_ring->deq_seg,
				   ctrl->event_ring->dequeue);
	xhci_writeq(&ctrl->ir_set->erst_dequeue, deq | ERST_EHB);
}

/**
 * Checks if there is a new event to handle on the event ring.
 *
 * @param ctrl	Host controller data structure
 * Return: 0 if failure else 1 on success
 */
static int event_ready(struct xhci_ctrl *ctrl)
{
	union xhci_trb *event;

	xhci_inval_cache((uintptr_t)ctrl->event_ring->dequeue,
			 sizeof(union xhci_trb));

	event = ctrl->event_ring->dequeue;

	/* Does the HC or OS own the TRB? */
	if ((le32_to_cpu(event->event_cmd.flags) & TRB_CYCLE) !=
		ctrl->event_ring->cycle_state)
		return 0;

	return 1;
}

/* 0 keeps XHCI_TIMEOUT. A non-zero event_wait_ms is one transfer's ceiling. */
static unsigned long xhci_event_limit(struct xhci_ctrl *ctrl)
{
	if (ctrl->event_wait_ms)
		return ctrl->event_wait_ms;
	return XHCI_TIMEOUT;
}

/**
 * Waits for a specific type of event and returns it. Discards unexpected
 * events. Caller *must* call xhci_acknowledge_event() after it is finished
 * processing the event, and must not access the returned pointer afterwards.
 *
 * @param ctrl		Host controller data structure
 * @param expected	TRB type expected from Event TRB
 * Return: pointer to event trb
 */
union xhci_trb *xhci_wait_for_event(struct xhci_ctrl *ctrl, trb_type expected)
{
	trb_type type;
	unsigned long ts = get_timer(0);

	do {
		union xhci_trb *event = ctrl->event_ring->dequeue;

		if (!event_ready(ctrl))
			continue;

		type = TRB_FIELD_TO_TYPE(le32_to_cpu(event->event_cmd.flags)); XHCI_ASYNC_DISPATCH(ctrl, event, type);
		/*
		 * A Stop Endpoint transfer event belongs to the TD that was
		 * cancelled, not to whatever transfer is waiting now. Leave
		 * it for that waiter and the next control transfer times out.
		 */
		if (type == TRB_TRANSFER) {
			u32 comp = GET_COMP_CODE(le32_to_cpu(
					event->trans_event.transfer_len));

			if (comp == COMP_STOP || comp == COMP_STOP_INVAL) {
				xhci_acknowledge_event(ctrl);
				continue;
			}
		}
		if (type == expected ||
		    (expected == TRB_NONE && type != TRB_PORT_STATUS))
			return event;

		if (type == TRB_PORT_STATUS)
		/* TODO: remove this once enumeration has been reworked */
			/*
			 * Port status change events always have a
			 * successful completion code
			 */
			BUG_ON(GET_COMP_CODE(
				le32_to_cpu(event->generic.field[2])) !=
								COMP_SUCCESS);
		else if (!ctrl->event_wait_ms)
			/*
			 * A 1 ms cyclic ceiling discards a burst of these.
			 * Printing each one is what held the scan tick.
			 */
			printf("Unexpected XHCI event TRB, skipping... "
				"(%08x %08x %08x %08x)\n",
				le32_to_cpu(event->generic.field[0]),
				le32_to_cpu(event->generic.field[1]),
				le32_to_cpu(event->generic.field[2]),
				le32_to_cpu(event->generic.field[3]));

		xhci_acknowledge_event(ctrl);
	} while (get_timer(ts) < xhci_event_limit(ctrl));

	if (expected == TRB_TRANSFER)
		return NULL;

	printf("XHCI timeout on event type %d...\n", expected);

	return NULL;
}

/*
 * Send reset endpoint command for given endpoint. This recovers from a
 * halted endpoint (e.g. due to a stall error).
 */
static void reset_ep(struct usb_device *udev, int ep_index)
{
	struct xhci_ctrl *ctrl = xhci_get_ctrl(udev);
	struct xhci_ring *ring =  ctrl->devs[udev->slot_id]->eps[ep_index].ring;
	union xhci_trb *event;
	u64 addr;
	u32 field;

	printf("Resetting EP %d...\n", ep_index);
	xhci_queue_command(ctrl, 0, udev->slot_id, ep_index, TRB_RESET_EP);
	event = xhci_wait_for_event(ctrl, TRB_COMPLETION);
	if (!event)
		return;

	field = le32_to_cpu(event->trans_event.flags);
	BUG_ON(TRB_TO_SLOT_ID(field) != udev->slot_id);
	xhci_acknowledge_event(ctrl);
	addr = xhci_trb_virt_to_dma(ring->enq_seg, ring->enqueue) |
		ring->cycle_state;
	xhci_queue_command(ctrl, addr, udev->slot_id, ep_index, TRB_SET_DEQ);
	event = xhci_wait_for_event(ctrl, TRB_COMPLETION);
	if (!event)
		return;

	BUG_ON(TRB_TO_SLOT_ID(le32_to_cpu(event->event_cmd.flags)) != udev->slot_id ||
	       GET_COMP_CODE(le32_to_cpu(event->event_cmd.status)) != COMP_SUCCESS);
	xhci_acknowledge_event(ctrl);
}

/*
 * Stops transfer processing for an endpoint and throws away all unprocessed
 * TRBs by setting the xHC's dequeue pointer to our enqueue pointer. The next
 * xhci_bulk_tx/xhci_ctrl_tx on this enpoint will add new transfers there and
 * ring the doorbell, causing this endpoint to start working again.
 * An endpoint that is already stopped completes with Context State Error
 * and is left alone.
 */
static void abort_td(struct usb_device *udev, int ep_index)
{
	struct xhci_ctrl *ctrl = xhci_get_ctrl(udev);
	struct xhci_ring *ring =  ctrl->devs[udev->slot_id]->eps[ep_index].ring;
	union xhci_trb *event;
	xhci_comp_code comp;
	u64 addr;

	xhci_queue_command(ctrl, 0, udev->slot_id, ep_index, TRB_STOP_RING);

	/*
	 * Command completion and the Stopped transfer event arrive in
	 * either order. Set TR Dequeue is issued only after the Stopped
	 * event has been acknowledged: doing it on the command completion
	 * alone leaves that event for the next control transfer, which
	 * then waits out the event ceiling. An endpoint that is already stopped
	 * completes with Context State Error and has no transfer event.
	 */
	{
		bool seen_cmd = false, seen_stop = false;
		unsigned long ts = get_timer(0);

		comp = COMP_SUCCESS;
		while (get_timer(ts) < xhci_event_limit(ctrl)) {
			trb_type type;
			u32 status, flags;

			if (!event_ready(ctrl))
				continue;
			event = ctrl->event_ring->dequeue;
			flags = le32_to_cpu(event->generic.field[3]);
			status = le32_to_cpu(event->generic.field[2]);
			type = TRB_FIELD_TO_TYPE(flags);
			if (type == TRB_TRANSFER &&
			    TRB_TO_SLOT_ID(flags) == udev->slot_id &&
			    TRB_TO_EP_INDEX(flags) == ep_index &&
			    (GET_COMP_CODE(status) == COMP_STOP ||
			     GET_COMP_CODE(status) == COMP_STOP_INVAL)) {
#if CONFIG_IS_ENABLED(USB_XHCI_ASYNC_BULK)
				if (!xhci_async_consume(ctrl, event))
					xhci_acknowledge_event(ctrl);
#else
				xhci_acknowledge_event(ctrl);
#endif
				seen_stop = true;
				if (seen_cmd)
					break;
				continue;
			}
#if CONFIG_IS_ENABLED(USB_XHCI_ASYNC_BULK)
			if (type == TRB_TRANSFER &&
			    xhci_async_consume(ctrl, event))
				continue;
#endif
			if (type == TRB_COMPLETION &&
			    TRB_TO_SLOT_ID(flags) == udev->slot_id) {
				comp = GET_COMP_CODE(status);
				xhci_acknowledge_event(ctrl);
				seen_cmd = true;
				if (comp == COMP_CTX_STATE || seen_stop)
					break;
				continue;
			}
			if (type != TRB_PORT_STATUS && !ctrl->event_wait_ms)
				/*
				 * A 1 ms cyclic ceiling discards a burst of these.
				 * Printing each one is what held the scan tick.
				 */
				printf("Unexpected XHCI event TRB, skipping... "
				       "(%08x %08x %08x %08x)\n",
				       le32_to_cpu(event->generic.field[0]),
				       le32_to_cpu(event->generic.field[1]),
				       le32_to_cpu(event->generic.field[2]),
				       le32_to_cpu(event->generic.field[3]));
			xhci_acknowledge_event(ctrl);
		}
		if (!seen_cmd)
			return;
		BUG_ON(comp != COMP_SUCCESS && comp != COMP_CTX_STATE);
		if (comp == COMP_CTX_STATE)
			return;
		/* No Stopped event yet: still move the dequeue pointer.
		 * A late event is discarded by xhci_wait_for_event(). */
	}
	addr = xhci_trb_virt_to_dma(ring->enq_seg, ring->enqueue) |
		ring->cycle_state;
	xhci_queue_command(ctrl, addr, udev->slot_id, ep_index, TRB_SET_DEQ);
	event = xhci_wait_for_event(ctrl, TRB_COMPLETION);
	if (!event)
		return;

	BUG_ON(TRB_TO_SLOT_ID(le32_to_cpu(event->event_cmd.flags)) != udev->slot_id ||
	       GET_COMP_CODE(le32_to_cpu(event->event_cmd.status)) != COMP_SUCCESS);
	xhci_acknowledge_event(ctrl);
}

static void record_transfer_result(struct usb_device *udev,
				   union xhci_trb *event, int length)
{
	udev->act_len = min(length, length -
		(int)EVENT_TRB_LEN(le32_to_cpu(event->trans_event.transfer_len)));

	switch (GET_COMP_CODE(le32_to_cpu(event->trans_event.transfer_len))) {
	case COMP_SUCCESS:
		BUG_ON(udev->act_len != length);
		/* fallthrough */
	case COMP_SHORT_TX:
		udev->status = 0;
		break;
	case COMP_STALL:
		udev->status = USB_ST_STALLED;
		break;
	case COMP_DB_ERR:
	case COMP_TRB_ERR:
		udev->status = USB_ST_BUF_ERR;
		break;
	case COMP_BABBLE:
		udev->status = USB_ST_BABBLE_DET;
		break;
	default:
		udev->status = 0x80;  /* USB_ST_TOO_LAZY_TO_MAKE_A_NEW_MACRO */
	}
}

/**** Bulk and Control transfer methods ****/
/**
 * Queues up the BULK Request
 *
 * @param udev		pointer to the USB device structure
 * @param pipe		contains the DIR_IN or OUT , devnum
 * @param length	length of the buffer
 * @param buffer	buffer to be read/written based on the request
 * Return: returns 0 if successful else -1 on failure
 */
int xhci_bulk_tx(struct usb_device *udev, unsigned long pipe,
			int length, void *buffer)
{
	int num_trbs = 0;
	struct xhci_generic_trb *start_trb;
	bool first_trb = false;
	int start_cycle;
	u32 field = 0;
	u32 length_field = 0;
	struct xhci_ctrl *ctrl = xhci_get_ctrl(udev);
	int slot_id = udev->slot_id;
	int ep_index;
	struct xhci_virt_device *virt_dev;
	struct xhci_ep_ctx *ep_ctx;
	struct xhci_ring *ring;		/* EP transfer ring */
	union xhci_trb *event;

	int running_total, trb_buff_len;
	bool more_trbs_coming = true;
	int maxpacketsize;
	u64 addr;
	int ret;
	u32 trb_fields[4];
	u64 buf_64 = xhci_dma_map(ctrl, buffer, length);
	dma_addr_t last_transfer_trb_addr;
	int available_length;

	debug("dev=%p, pipe=%lx, buffer=%p, length=%d\n",
		udev, pipe, buffer, length);

	available_length = length;
	ep_index = usb_pipe_ep_index(pipe);
	virt_dev = ctrl->devs[slot_id];

	xhci_inval_cache((uintptr_t)virt_dev->out_ctx->bytes,
			 virt_dev->out_ctx->size);

	ep_ctx = xhci_get_ep_ctx(ctrl, virt_dev->out_ctx, ep_index);

	/*
	 * If the endpoint was halted due to a prior error, resume it before
	 * the next transfer. It is the responsibility of the upper layer to
	 * have dealt with whatever caused the error.
	 */
	if ((le32_to_cpu(ep_ctx->ep_info) & EP_STATE_MASK) == EP_STATE_HALTED) {
		reset_ep(udev, ep_index);
		/*
		 * The context read above still says Halted; prepare_ring()
		 * below would refuse the transfer on it, and the next call
		 * would reset the endpoint again -- a keyboard polled from the
		 * console did exactly that on every poll. Read the state the
		 * reset left.
		 */
		xhci_inval_cache((uintptr_t)virt_dev->out_ctx->bytes,
				 virt_dev->out_ctx->size);
	}

	ring = virt_dev->eps[ep_index].ring;
	if (!ring)
		return -EINVAL;

	/*
	 * How much data is (potentially) left before the 64KB boundary?
	 * XHCI Spec puts restriction( TABLE 49 and 6.4.1 section of XHCI Spec)
	 * that the buffer should not span 64KB boundary. if so
	 * we send request in more than 1 TRB by chaining them.
	 */
	running_total = TRB_MAX_BUFF_SIZE -
			(lower_32_bits(buf_64) & (TRB_MAX_BUFF_SIZE - 1));
	trb_buff_len = running_total;
	running_total &= TRB_MAX_BUFF_SIZE - 1;

	/*
	 * If there's some data on this 64KB chunk, or we have to send a
	 * zero-length transfer, we need at least one TRB
	 */
	if (running_total != 0 || length == 0)
		num_trbs++;

	/* How many more 64KB chunks to transfer, how many more TRBs? */
	while (running_total < length) {
		num_trbs++;
		running_total += TRB_MAX_BUFF_SIZE;
	}

	/*
	 * XXX: Calling routine prepare_ring() called in place of
	 * prepare_trasfer() as there in 'Linux' since we are not
	 * maintaining multiple TDs/transfer at the same time.
	 */
	ret = prepare_ring(ctrl, ring,
			   le32_to_cpu(ep_ctx->ep_info) & EP_STATE_MASK);
	if (ret < 0)
		return ret;

	/*
	 * Don't give the first TRB to the hardware (by toggling the cycle bit)
	 * until we've finished creating all the other TRBs.  The ring's cycle
	 * state may change as we enqueue the other TRBs, so save it too.
	 */
	start_trb = &ring->enqueue->generic;
	start_cycle = ring->cycle_state;

	running_total = 0;
	maxpacketsize = usb_maxpacket(udev, pipe);

	/* How much data is in the first TRB? */
	/*
	 * How much data is (potentially) left before the 64KB boundary?
	 * XHCI Spec puts restriction( TABLE 49 and 6.4.1 section of XHCI Spec)
	 * that the buffer should not span 64KB boundary. if so
	 * we send request in more than 1 TRB by chaining them.
	 */
	addr = buf_64;

	if (trb_buff_len > length)
		trb_buff_len = length;

	first_trb = true;

	/* flush the buffer before use */
	xhci_flush_cache((uintptr_t)buffer, length);

	/* Queue the first TRB, even if it's zero-length */
	do {
		u32 remainder = 0;
		field = 0;
		/* Don't change the cycle bit of the first TRB until later */
		if (first_trb) {
			first_trb = false;
			if (start_cycle == 0)
				field |= TRB_CYCLE;
		} else {
			field |= ring->cycle_state;
		}

		/*
		 * Chain all the TRBs together; clear the chain bit in the last
		 * TRB to indicate it's the last TRB in the chain.
		 */
		if (num_trbs > 1) {
			field |= TRB_CHAIN;
		} else {
			field |= TRB_IOC;
			more_trbs_coming = false;
		}

		/* Only set interrupt on short packet for IN endpoints */
		if (usb_pipein(pipe))
			field |= TRB_ISP;

		/* Set the TRB length, TD size, and interrupter fields. */
		remainder = xhci_td_remainder(ctrl, running_total, trb_buff_len,
					      length, maxpacketsize,
					      more_trbs_coming);

		length_field = (TRB_LEN(trb_buff_len) |
				TRB_TD_SIZE(remainder) |
				TRB_INTR_TARGET(0));

		trb_fields[0] = lower_32_bits(addr);
		trb_fields[1] = upper_32_bits(addr);
		trb_fields[2] = length_field;
		trb_fields[3] = field | TRB_TYPE(TRB_NORMAL);

		last_transfer_trb_addr = queue_trb(ctrl, ring, (num_trbs > 1), trb_fields);

		--num_trbs;

		running_total += trb_buff_len;

		/* Calculate length for next transfer */
		addr += trb_buff_len;
		trb_buff_len = min((length - running_total), TRB_MAX_BUFF_SIZE);

		schedule();
	} while (running_total < length);

	giveback_first_trb(udev, ep_index, start_cycle, start_trb);

again:
	event = xhci_wait_for_event(ctrl, TRB_TRANSFER);
	if (!event) {
		debug("XHCI bulk transfer timed out, aborting...\n");
		abort_td(udev, ep_index);
		udev->status = USB_ST_NAK_REC;  /* closest thing to a timeout */
		udev->act_len = 0;
		return -ETIMEDOUT;
	}

	if ((uintptr_t)(le64_to_cpu(event->trans_event.buffer)) !=
	    (uintptr_t)last_transfer_trb_addr) {
		available_length -=
			(int)EVENT_TRB_LEN(le32_to_cpu(event->trans_event.transfer_len));
		xhci_acknowledge_event(ctrl);
		goto again;
	}

	field = le32_to_cpu(event->trans_event.flags);
	BUG_ON(TRB_TO_SLOT_ID(field) != slot_id);
	BUG_ON(TRB_TO_EP_INDEX(field) != ep_index);

	record_transfer_result(udev, event, available_length);
	xhci_acknowledge_event(ctrl);
	xhci_inval_cache((uintptr_t)buffer, length);
	xhci_dma_unmap(ctrl, buf_64, length);

	return (udev->status != USB_ST_NOT_PROC) ? 0 : -1;
}

/**
 * Queues up the Control Transfer Request
 *
 * @param udev	pointer to the USB device structure
 * @param pipe		contains the DIR_IN or OUT , devnum
 * @param req		request type
 * @param length	length of the buffer
 * @param buffer	buffer to be read/written based on the request
 * Return: returns 0 if successful else error code on failure
 */
int xhci_ctrl_tx(struct usb_device *udev, unsigned long pipe,
			struct devrequest *req,	int length,
			void *buffer)
{
	int ret;
	int start_cycle;
	int num_trbs;
	u32 field;
	u32 length_field;
	u64 buf_64 = 0;
	struct xhci_generic_trb *start_trb;
	struct xhci_ctrl *ctrl = xhci_get_ctrl(udev);
	int slot_id = udev->slot_id;
	int ep_index;
	u32 trb_fields[4];
	struct xhci_virt_device *virt_dev = ctrl->devs[slot_id];
	struct xhci_ring *ep_ring;
	union xhci_trb *event;
	u32 remainder;

	debug("req=%u (%#x), type=%u (%#x), value=%u (%#x), index=%u\n",
		req->request, req->request,
		req->requesttype, req->requesttype,
		le16_to_cpu(req->value), le16_to_cpu(req->value),
		le16_to_cpu(req->index));

	ep_index = usb_pipe_ep_index(pipe);

	ep_ring = virt_dev->eps[ep_index].ring;
	if (!ep_ring)
		return -EINVAL;

	/*
	 * Check to see if the max packet size for the default control
	 * endpoint changed during FS device enumeration
	 */
	if (udev->speed == USB_SPEED_FULL) {
		ret = xhci_check_maxpacket(udev);
		if (ret < 0)
			return ret;
	}

	xhci_inval_cache((uintptr_t)virt_dev->out_ctx->bytes,
			 virt_dev->out_ctx->size);

	struct xhci_ep_ctx *ep_ctx = NULL;
	ep_ctx = xhci_get_ep_ctx(ctrl, virt_dev->out_ctx, ep_index);

	/* 1 TRB for setup, 1 for status */
	num_trbs = 2;
	/*
	 * Don't need to check if we need additional event data and normal TRBs,
	 * since data in control transfers will never get bigger than 16MB
	 * XXX: can we get a buffer that crosses 64KB boundaries?
	 */

	if (length > 0)
		num_trbs++;
	/*
	 * XXX: Calling routine prepare_ring() called in place of
	 * prepare_trasfer() as there in 'Linux' since we are not
	 * maintaining multiple TDs/transfer at the same time.
	 */
	ret = prepare_ring(ctrl, ep_ring,
				le32_to_cpu(ep_ctx->ep_info) & EP_STATE_MASK);

	if (ret < 0)
		return ret;

	/*
	 * Don't give the first TRB to the hardware (by toggling the cycle bit)
	 * until we've finished creating all the other TRBs.  The ring's cycle
	 * state may change as we enqueue the other TRBs, so save it too.
	 */
	start_trb = &ep_ring->enqueue->generic;
	start_cycle = ep_ring->cycle_state;

	debug("start_trb %p, start_cycle %d\n", start_trb, start_cycle);

	/* Queue setup TRB - see section 6.4.1.2.1 */
	/* FIXME better way to translate setup_packet into two u32 fields? */
	field = 0;
	field |= TRB_IDT | TRB_TYPE(TRB_SETUP);
	if (start_cycle == 0)
		field |= 0x1;

	/* xHCI 1.0 6.4.1.2.1: Transfer Type field */
	if (ctrl->hci_version >= 0x100 || ctrl->quirks & XHCI_MTK_HOST) {
		if (length > 0) {
			if (req->requesttype & USB_DIR_IN)
				field |= TRB_TX_TYPE(TRB_DATA_IN);
			else
				field |= TRB_TX_TYPE(TRB_DATA_OUT);
		}
	}

	debug("req->requesttype = %d, req->request = %d, req->value = %d, req->index = %d, req->length = %d\n",
	      req->requesttype, req->request, le16_to_cpu(req->value),
	      le16_to_cpu(req->index), le16_to_cpu(req->length));

	trb_fields[0] = req->requesttype | req->request << 8 |
				le16_to_cpu(req->value) << 16;
	trb_fields[1] = le16_to_cpu(req->index) |
			le16_to_cpu(req->length) << 16;
	/* TRB_LEN | (TRB_INTR_TARGET) */
	trb_fields[2] = (TRB_LEN(8) | TRB_INTR_TARGET(0));
	/* Immediate data in pointer */
	trb_fields[3] = field;
	queue_trb(ctrl, ep_ring, true, trb_fields);

	/* Re-initializing field to zero */
	field = 0;
	/* If there's data, queue data TRBs */
	/* Only set interrupt on short packet for IN endpoints */
	if (usb_pipein(pipe))
		field = TRB_ISP | TRB_TYPE(TRB_DATA);
	else
		field = TRB_TYPE(TRB_DATA);

	remainder = xhci_td_remainder(ctrl, 0, length, length,
				      usb_maxpacket(udev, pipe), true);
	length_field = TRB_LEN(length) | TRB_TD_SIZE(remainder) |
		       TRB_INTR_TARGET(0);
	debug("length_field = %d, length = %d,"
		"xhci_td_remainder(length) = %d , TRB_INTR_TARGET(0) = %d\n",
		length_field, TRB_LEN(length),
		TRB_TD_SIZE(remainder), 0);

	if (length > 0) {
		if (req->requesttype & USB_DIR_IN)
			field |= TRB_DIR_IN;
		buf_64 = xhci_dma_map(ctrl, buffer, length);

		trb_fields[0] = lower_32_bits(buf_64);
		trb_fields[1] = upper_32_bits(buf_64);
		trb_fields[2] = length_field;
		trb_fields[3] = field | ep_ring->cycle_state;

		xhci_flush_cache((uintptr_t)buffer, length);
		queue_trb(ctrl, ep_ring, true, trb_fields);
	}

	/*
	 * Queue status TRB -
	 * see Table 7 and sections 4.11.2.2 and 6.4.1.2.3
	 */

	/* If the device sent data, the status stage is an OUT transfer */
	field = 0;
	if (length > 0 && req->requesttype & USB_DIR_IN)
		field = 0;
	else
		field = TRB_DIR_IN;

	trb_fields[0] = 0;
	trb_fields[1] = 0;
	trb_fields[2] = TRB_INTR_TARGET(0);
		/* Event on completion */
	trb_fields[3] = field | TRB_IOC |
			TRB_TYPE(TRB_STATUS) | ep_ring->cycle_state;

	queue_trb(ctrl, ep_ring, false, trb_fields);

	giveback_first_trb(udev, ep_index, start_cycle, start_trb);

	event = xhci_wait_for_event(ctrl, TRB_TRANSFER);
	if (!event)
		goto abort;
	field = le32_to_cpu(event->trans_event.flags);

	BUG_ON(TRB_TO_SLOT_ID(field) != slot_id);
	BUG_ON(TRB_TO_EP_INDEX(field) != ep_index);

	record_transfer_result(udev, event, length);
	xhci_acknowledge_event(ctrl);
	if (udev->status == USB_ST_STALLED) {
		reset_ep(udev, ep_index);
		return -EPIPE;
	}

	/* Invalidate buffer to make it available to usb-core */
	if (length > 0) {
		xhci_inval_cache((uintptr_t)buffer, length);
		xhci_dma_unmap(ctrl, buf_64, length);
	}

	if (GET_COMP_CODE(le32_to_cpu(event->trans_event.transfer_len))
			== COMP_SHORT_TX) {
		/* Short data stage, clear up additional status stage event */
		event = xhci_wait_for_event(ctrl, TRB_TRANSFER);
		if (!event)
			goto abort;
		BUG_ON(TRB_TO_SLOT_ID(field) != slot_id);
		BUG_ON(TRB_TO_EP_INDEX(field) != ep_index);
		xhci_acknowledge_event(ctrl);
	}

	return (udev->status != USB_ST_NOT_PROC) ? 0 : -1;

abort:
	debug("XHCI control transfer timed out, aborting...\n");
	abort_td(udev, ep_index);
	udev->status = USB_ST_NAK_REC;
	udev->act_len = 0;
	return -ETIMEDOUT;
}

#if CONFIG_IS_ENABLED(USB_XHCI_ASYNC_BULK)
/*
 * Non-blocking bulk-IN transfer descriptors.
 *
 * One TD per registered endpoint is queued and left pending. Its transfer
 * event is consumed by whoever reads the event ring first: a driver poll
 * (xhci_async_bulk_poll, which drains ready events without waiting) or any
 * synchronous control / bulk / command wait (xhci_wait_for_event via the
 * XHCI_ASYNC_DISPATCH hook -> xhci_async_consume). Either way the completion
 * is recorded in the slot and handed to the owner at its next poll. Nothing
 * here waits or stops an endpoint except close and a synchronous transfer on
 * the same endpoint (one Stop Endpoint + Set TR Dequeue), and controller
 * teardown (slots forgotten).
 *
 * Handles are slot_index | generation << 8, so a slot forgotten at
 * xhci_deregister() (usb reset / usb stop) is detected without touching the
 * stale usb_device. Buffers are single-TRB (the caller aligns them and keeps
 * them < 64 KiB), so there is no chained-TD inner-event handling.
 */
#define XHCI_ASYNC_MAX		6

struct xhci_async_td {
	struct xhci_ctrl *ctrl;		/* NULL = free */
	struct usb_device *udev;
	unsigned long pipe;
	int slot_id;
	int ep_index;
	u32 gen;
	bool active;			/* TD queued, event not yet seen */
	bool done;			/* event seen, not handed over */
	void *buffer;
	int length;
	u64 buf_64;
	dma_addr_t last_trb;
	int comp;
	int act_len;
	struct xhci_async_stats st;
};

static struct xhci_async_td xhci_async[XHCI_ASYNC_MAX];

static struct xhci_async_td *xhci_async_lookup(int h)
{
	int i = h & 0xff;

	if (h < 0 || i >= XHCI_ASYNC_MAX)
		return NULL;
	if (!xhci_async[i].ctrl || xhci_async[i].gen != ((u32)h >> 8))
		return NULL;
	return &xhci_async[i];
}

/* Queue one single-TRB bulk-IN TD and ring the doorbell; does not wait. */
static int xhci_async_queue(struct xhci_async_td *e)
{
	struct xhci_ctrl *ctrl = e->ctrl;
	struct usb_device *udev = e->udev;
	int ep_index = e->ep_index;
	struct xhci_virt_device *virt_dev = ctrl->devs[udev->slot_id];
	struct xhci_ep_ctx *ep_ctx;
	struct xhci_ring *ring;
	struct xhci_generic_trb *start_trb;
	int start_cycle, maxpacketsize, ret;
	u32 field, remainder, length_field;
	u32 trb_fields[4];
	u64 addr;

	e->buf_64 = xhci_dma_map(ctrl, e->buffer, e->length);

	xhci_inval_cache((uintptr_t)virt_dev->out_ctx->bytes,
			 virt_dev->out_ctx->size);
	ep_ctx = xhci_get_ep_ctx(ctrl, virt_dev->out_ctx, ep_index);
	if ((le32_to_cpu(ep_ctx->ep_info) & EP_STATE_MASK) == EP_STATE_HALTED) {
		reset_ep(udev, ep_index);
		xhci_inval_cache((uintptr_t)virt_dev->out_ctx->bytes,
				 virt_dev->out_ctx->size);
	}

	ring = virt_dev->eps[ep_index].ring;
	if (!ring)
		return -EINVAL;

	ret = prepare_ring(ctrl, ring,
			   le32_to_cpu(ep_ctx->ep_info) & EP_STATE_MASK);
	if (ret < 0)
		return ret;

	start_trb = &ring->enqueue->generic;
	start_cycle = ring->cycle_state;
	maxpacketsize = usb_maxpacket(udev, e->pipe);
	addr = e->buf_64;

	xhci_flush_cache((uintptr_t)e->buffer, e->length);

	field = (start_cycle == 0) ? TRB_CYCLE : 0;
	field |= TRB_IOC | TRB_ISP;
	remainder = xhci_td_remainder(ctrl, 0, e->length, e->length,
				      maxpacketsize, false);
	length_field = TRB_LEN(e->length) | TRB_TD_SIZE(remainder) |
		TRB_INTR_TARGET(0);
	trb_fields[0] = lower_32_bits(addr);
	trb_fields[1] = upper_32_bits(addr);
	trb_fields[2] = length_field;
	trb_fields[3] = field | TRB_TYPE(TRB_NORMAL);
	e->last_trb = queue_trb(ctrl, ring, false, trb_fields);

	giveback_first_trb(udev, ep_index, start_cycle, start_trb);
	return 0;
}

/*
 * An event at the ring dequeue: consume it if it belongs to a pending async
 * TD. Several TDs share one endpoint (the RX pool). A synchronous bulk OUT
 * waits on this same event ring, so attributing the completion to the first
 * active TD marks the wrong buffer done and leaves the TD that actually
 * finished posted forever. Match the event's TRB, the same way the poll
 * drain does. A single posted TD may take an event with no TRB pointer.
 */
bool xhci_async_consume(struct xhci_ctrl *ctrl, union xhci_trb *event)
{
	u32 flags = le32_to_cpu(event->trans_event.flags);
	u32 tl = le32_to_cpu(event->trans_event.transfer_len);
	u64 ev_trb = le64_to_cpu(event->trans_event.buffer);
	int slot = TRB_TO_SLOT_ID(flags);
	int ep = TRB_TO_EP_INDEX(flags);
	struct xhci_async_td *hit = NULL;
	int i, nactive = 0;

	for (i = 0; i < XHCI_ASYNC_MAX; i++) {
		struct xhci_async_td *e = &xhci_async[i];

		if (e->ctrl != ctrl || !e->active || e->slot_id != slot ||
		    e->ep_index != ep)
			continue;
		nactive++;
		if (ev_trb && e->last_trb == ev_trb)
			hit = e;
	}
	if (!hit && nactive == 1) {
		for (i = 0; i < XHCI_ASYNC_MAX; i++) {
			struct xhci_async_td *e = &xhci_async[i];

			if (e->ctrl != ctrl || !e->active ||
			    e->slot_id != slot || e->ep_index != ep)
				continue;
			hit = e;
			break;
		}
	}
	if (!hit)
		return false;

	hit->comp = GET_COMP_CODE(tl);
	hit->act_len = min(hit->length, hit->length - (int)EVENT_TRB_LEN(tl));
	hit->st.last_comp = hit->comp;
	hit->active = false;
	hit->done = true;
	hit->st.completed++;
	hit->st.completed_in_wait++;
	xhci_acknowledge_event(ctrl);
	return true;
}

/* Drain every ready event without waiting (driver poll context). */
static void xhci_async_drain(struct xhci_ctrl *ctrl, struct xhci_async_td *me)
{
	while (event_ready(ctrl)) {
		union xhci_trb *event = ctrl->event_ring->dequeue;
		trb_type type;

		type = TRB_FIELD_TO_TYPE(le32_to_cpu(event->event_cmd.flags));
		if (type == TRB_TRANSFER) {
			u32 flags = le32_to_cpu(event->trans_event.flags);
			u32 tl = le32_to_cpu(event->trans_event.transfer_len);
			u64 ev_trb = le64_to_cpu(event->trans_event.buffer);
			int slot = TRB_TO_SLOT_ID(flags);
			int ep = TRB_TO_EP_INDEX(flags);
			struct xhci_async_td *hit = NULL;
			int i;

			/*
			 * Pass 1: exact match on the completed TRB pointer, so
			 * several TDs posted on the SAME endpoint (the RX pool)
			 * are attributed to the right handle and buffer.
			 */
			for (i = 0; i < XHCI_ASYNC_MAX; i++) {
				struct xhci_async_td *e = &xhci_async[i];

				if (e->ctrl != ctrl || !e->active ||
				    e->slot_id != slot || e->ep_index != ep)
					continue;
				if (ev_trb && e->last_trb == ev_trb) {
					hit = e;
					break;
				}
			}
			/* Pass 2: fall back to the first active TD on this ep
			 * (single-handle users, or no TRB-pointer match). */
			if (!hit) {
				for (i = 0; i < XHCI_ASYNC_MAX; i++) {
					struct xhci_async_td *e = &xhci_async[i];

					if (e->ctrl != ctrl || !e->active ||
					    e->slot_id != slot ||
					    e->ep_index != ep)
						continue;
					hit = e;
					break;
				}
			}
			if (hit) {
				hit->comp = GET_COMP_CODE(tl);
				hit->act_len = min(hit->length,
						   hit->length -
						   (int)EVENT_TRB_LEN(tl));
				hit->st.last_comp = hit->comp;
				hit->active = false;
				hit->done = true;
				hit->st.completed++;
				xhci_acknowledge_event(ctrl);
				continue;
			}
		}

		/*
		 * Not a pending async TD: no synchronous waiter is running
		 * while the owner polls, so nobody else will read it.
		 * Acknowledge it so it does not block the ring; keep a copy.
		 */
		if (type != TRB_PORT_STATUS) {
			me->st.orphans++;
			me->st.last_orphan[0] = le32_to_cpu(event->generic.field[0]);
			me->st.last_orphan[1] = le32_to_cpu(event->generic.field[1]);
			me->st.last_orphan[2] = le32_to_cpu(event->generic.field[2]);
			me->st.last_orphan[3] = le32_to_cpu(event->generic.field[3]);
		}
		xhci_acknowledge_event(ctrl);
	}
}

int xhci_async_bulk_open(struct usb_device *udev, unsigned long pipe)
{
	struct xhci_ctrl *ctrl = xhci_get_ctrl(udev);
	int ep_index = usb_pipe_ep_index(pipe);
	int i, free_i = -1;

	if (!ctrl || !udev->slot_id || usb_pipetype(pipe) != PIPE_BULK)
		return -EINVAL;

	/*
	 * Multiple handles on the same endpoint are allowed: the RX data path
	 * keeps a pool of TDs posted so the endpoint is never left empty (an
	 * empty endpoint wedges the dongle's USB RX DMA). Completions are
	 * attributed by TRB pointer in xhci_async_drain().
	 */
	for (i = 0; i < XHCI_ASYNC_MAX; i++) {
		struct xhci_async_td *e = &xhci_async[i];

		if (!e->ctrl && free_i < 0)
			free_i = i;
	}
	if (free_i < 0)
		return -ENOSPC;

	i = free_i;
	memset(&xhci_async[i].st, 0, sizeof(xhci_async[i].st));
	xhci_async[i].ctrl = ctrl;
	xhci_async[i].udev = udev;
	xhci_async[i].pipe = pipe;
	xhci_async[i].slot_id = udev->slot_id;
	xhci_async[i].ep_index = ep_index;
	xhci_async[i].active = false;
	xhci_async[i].done = false;

	return i | (xhci_async[i].gen << 8);
}

int xhci_async_bulk_submit(int h, void *buffer, int length)
{
	struct xhci_async_td *e = xhci_async_lookup(h);
	int ret;

	if (!e)
		return -ENODEV;
	if (e->active || e->done)
		return -EBUSY;

	e->buffer = buffer;
	e->length = length;
	ret = xhci_async_queue(e);
	if (ret) {
		e->st.errors++;
		return ret;
	}

	e->active = true;
	e->st.submitted++;
	return 0;
}

int xhci_async_bulk_poll(int h, int *actual)
{
	struct xhci_async_td *e = xhci_async_lookup(h);

	if (!e)
		return -ENODEV;

	e->st.polls++;
	if (!e->done)
		xhci_async_drain(e->ctrl, e);

	if (e->done) {
		e->done = false;
		xhci_inval_cache((uintptr_t)e->buffer, e->length);
		xhci_dma_unmap(e->ctrl, e->buf_64, e->length);
		*actual = e->act_len;
		return (e->comp == COMP_SUCCESS || e->comp == COMP_SHORT_TX) ?
			0 : -EIO;
	}

	return e->active ? -EINPROGRESS : -ENOENT;
}

/*
 * Stop a pending TD. Stop Endpoint on an endpoint that is not running
 * returns a command completion instead of a transfer event. abort_td()
 * then consumes that completion as if it were the stop, and the next
 * control transfer times out. Two TDs on one endpoint hit this on the
 * second close: the first stop already left the endpoint stopped.
 */
static void xhci_async_stop(struct xhci_async_td *e)
{
	struct xhci_virt_device *virt_dev;
	struct xhci_ep_ctx *ep_ctx;
	u32 ep_state = EP_STATE_RUNNING;

	if (!e->active)
		return;

	/* A completion that landed before close is not a transfer to stop. */
	xhci_async_drain(e->ctrl, e);
	if (!e->active) {
		if (e->done) {
			e->done = false;
			xhci_inval_cache((uintptr_t)e->buffer, e->length);
			xhci_dma_unmap(e->ctrl, e->buf_64, e->length);
		}
		return;
	}

	/*
	 * Leave active set so abort_td() can match the Stopped transfer
	 * event to this TD. Clearing active first leaves that event on
	 * the ring for the next control transfer.
	 */
	virt_dev = e->ctrl->devs[e->slot_id];
	if (virt_dev && virt_dev->out_ctx) {
		xhci_inval_cache((uintptr_t)virt_dev->out_ctx->bytes,
				 virt_dev->out_ctx->size);
		ep_ctx = xhci_get_ep_ctx(e->ctrl, virt_dev->out_ctx,
					 e->ep_index);
		ep_state = le32_to_cpu(ep_ctx->ep_info) & EP_STATE_MASK;
	}
	if (ep_state == EP_STATE_RUNNING)
		abort_td(e->udev, e->ep_index);
	e->active = false;
	e->done = false;
	xhci_dma_unmap(e->ctrl, e->buf_64, e->length);
	e->st.cancels++;
}

void xhci_async_bulk_close(int h)
{
	struct xhci_async_td *e = xhci_async_lookup(h);

	if (!e)
		return;

	xhci_async_stop(e);
	e->done = false;
	e->ctrl = NULL;
	e->gen++;
}

int xhci_async_bulk_stats(int h, struct xhci_async_stats *st)
{
	struct xhci_async_td *e = xhci_async_lookup(h);

	if (!e)
		return -ENODEV;
	*st = e->st;
	st->active = e->active;
	return 0;
}

/* Controller going away (usb stop / usb reset): forget its async TDs. */
void xhci_async_forget_ctrl(struct xhci_ctrl *ctrl)
{
	int i;

	for (i = 0; i < XHCI_ASYNC_MAX; i++)
		if (xhci_async[i].ctrl == ctrl) {
			xhci_async[i].ctrl = NULL;
			xhci_async[i].active = false;
			xhci_async[i].done = false;
			xhci_async[i].gen++;
		}
}

/* Read an endpoint context state from the output context (wifi diag). */
int xhci_ep_debug_state(struct usb_device *udev, unsigned long pipe,
			u32 *ep_state, u64 *hw_deq, u64 *sw_enq)
{
	struct xhci_ctrl *ctrl = xhci_get_ctrl(udev);
	int ep_index = usb_pipe_ep_index(pipe);
	struct xhci_virt_device *virt_dev;
	struct xhci_ep_ctx *ep_ctx;
	struct xhci_ring *ring;

	if (!ctrl || !udev->slot_id)
		return -ENODEV;
	virt_dev = ctrl->devs[udev->slot_id];
	if (!virt_dev)
		return -ENODEV;

	xhci_inval_cache((uintptr_t)virt_dev->out_ctx->bytes,
			 virt_dev->out_ctx->size);
	ep_ctx = xhci_get_ep_ctx(ctrl, virt_dev->out_ctx, ep_index);
	*ep_state = le32_to_cpu(ep_ctx->ep_info) & EP_STATE_MASK;
	*hw_deq = le64_to_cpu(ep_ctx->deq);
	ring = virt_dev->eps[ep_index].ring;
	*sw_enq = ring ? (xhci_trb_virt_to_dma(ring->enq_seg, ring->enqueue) |
			  ring->cycle_state) : 0;
	return 0;
}
#endif /* USB_XHCI_ASYNC_BULK */
