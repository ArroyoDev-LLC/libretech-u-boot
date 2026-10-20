/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Early GXL USB identify (02 22). Host-proven via tools/usb-id-stub/id-try.S
 * (keep-handle 02220020 at +0.01s).
 *
 * Product (U1_ONLY==0): serve on-wire long enough for host keep-handle, then
 * restore caller SP/LR and return to board_init_f (power/DDR).
 * Isolation (U1_ONLY==3): caller hangs after return; same asm is fine.
 *
 * Green baseline (ee74670): SP@0xd900ff00, no RX drain, Linux EP0 TX order,
 * re-enter poll after first ID TX. Product adds: save/restore SP+x30 in SRAM
 * scratch so return does not clobber C stack; finite post-ID serve then ret.
 */
	.global meson_u1_asm_identify
	.section .text.meson_u1_asm, "ax"

	.equ GINTSTS,   0x014
	.equ GRXSTSP,   0x020
	.equ DIEPCTL0,  0x900
	.equ DIEPTSIZ0, 0x910
	.equ DOEPCTL0,  0xb00
	.equ DOEPINT0,  0xb08
	.equ DOEPTSIZ0, 0xb10
	.equ EP0_FIFO,  0x1000
	/* SRAM scratch: save caller SP / x30 (out of DWC2 path) */
	.equ SCRATCH,   0xd900c120

meson_u1_asm_identify:
	/* Save C context before SP switch (product path returns). */
	mov	x9, #0xc120
	movk	x9, #0xd900, lsl #16
	mov	x10, sp
	str	x10, [x9]
	str	x30, [x9, #8]
	str	x29, [x9, #16]
	str	x19, [x9, #24]
	str	x20, [x9, #32]
	str	x21, [x9, #40]
	str	x22, [x9, #48]

	/* Dedicated stack — host-proven isolation path. */
	mov	x0, #0xff00
	movk	x0, #0xd900, lsl #16
	mov	sp, x0
	msr	daifset, #0xf
	ic	iallu
	dsb	sy
	isb

	/* OMOR at 0xd900c100 */
	mov	x0, #0xc100
	movk	x0, #0xd900, lsl #16
	mov	w1, #0x4d4f
	movk	w1, #0x524f, lsl #16
	str	w1, [x0]

	/* prog START|WORLDCUP at 0xd900c104 */
	mov	x2, #0xc104
	movk	x2, #0xd900, lsl #16
	mov	w1, #0x11
	str	w1, [x2]

	/* x20 = DWC2 */
	mov	x20, #0x0000
	movk	x20, #0xc910, lsl #16

	/* Do NOT drain RXFIFO — that discards the host identify SETUP. */
	/* DOEPTSIZ = 0x60080018 */
	mov	w0, #0x0018
	movk	w0, #0x6008, lsl #16
	str	w0, [x20, #DOEPTSIZ0]
	/* DOEPCTL EPENA|CNAK */
	ldr	w0, [x20, #DOEPCTL0]
	orr	w0, w0, #0x04000000
	orr	w0, w0, #0x80000000
	str	w0, [x20, #DOEPCTL0]

	/*
	 * Outer loop ~2500 * (inner 10000) — green baseline.  After first ID
	 * TX we reload a much larger counter so host keep-handle can land.
	 */
	mov	w19, #2500
poll:
	mov	w18, #10000
delay:
	subs	w18, w18, #1
	b.ne	delay
	ldr	w0, [x20, #GINTSTS]
	tbz	w0, #4, check_doep
	ldr	w0, [x20, #GRXSTSP]
	ubfx	w1, w0, #17, #4
	cmp	w1, #6
	b.ne	check_doep
	ldr	w3, [x20, #EP0_FIFO]
	ldr	w4, [x20, #EP0_FIFO]
	ldr	w1, [x2]
	orr	w1, w1, #0x800
	str	w1, [x2]
	mov	x5, #0xc108
	movk	x5, #0xd900, lsl #16
	str	w3, [x5]
	ubfx	w1, w3, #8, #8
	cmp	w1, #0x20
	b.eq	do_id

check_doep:
	ldr	w0, [x20, #DOEPINT0]
	tbz	w0, #3, next
	str	w0, [x20, #DOEPINT0]
	ldr	w1, [x2]
	orr	w1, w1, #0x1000
	str	w1, [x2]
	mov	x5, #0xc108
	movk	x5, #0xd900, lsl #16
	ldr	w3, [x5]
	ubfx	w1, w3, #8, #8
	cmp	w1, #0x20
	b.ne	next

do_id:
	ldr	w1, [x2]
	orr	w1, w1, #0x2000
	str	w1, [x2]
	/* DIEPTSIZ 0x00080008 */
	mov	w0, #0x0008
	movk	w0, #0x0008, lsl #16
	str	w0, [x20, #DIEPTSIZ0]
	/* DIEPCTL EPENA|CNAK|USBACTEP, clear SNAK — then push FIFO (Linux order) */
	ldr	w0, [x20, #DIEPCTL0]
	orr	w0, w0, #0x80000000
	orr	w0, w0, #0x04000000
	orr	w0, w0, #0x00008000
	bic	w0, w0, #0x08000000
	str	w0, [x20, #DIEPCTL0]
	/* 02 22 00 20 little-endian word + zeros */
	mov	w0, #0x2202
	movk	w0, #0x2000, lsl #16
	str	w0, [x20, #EP0_FIFO]
	str	wzr, [x20, #EP0_FIFO]
	ldr	w1, [x2]
	orr	w1, w1, #0x4000
	str	w1, [x2]
	/*
	 * Re-enter poll so host retries get another TX (green).  Finite
	 * outer count so product can return after ~seconds of serve.
	 * ~0x3fffff * 10k iters is long; product uses 0x20000 (~enough for
	 * host keep-handle measured at +0.01..0.43s).
	 */
	mov	w19, #0x0000
	movk	w19, #0x2, lsl #16	/* 0x20000 outer after first TX */
	b	poll

next:
	subs	w19, w19, #1
	b.ne	poll

	/* Serve window done (or never saw SETUP) — restore C and return. */
out:
	mov	x9, #0xc120
	movk	x9, #0xd900, lsl #16
	ldr	x22, [x9, #48]
	ldr	x21, [x9, #40]
	ldr	x20, [x9, #32]
	ldr	x19, [x9, #24]
	ldr	x29, [x9, #16]
	ldr	x30, [x9, #8]
	ldr	x10, [x9]
	mov	sp, x10
	ret

/*
 * WR_LARGE bulk OUT PIO spin (3 pointers — LTO-safe args).
 * void meson_bulk_spin_pio(u32 *dst, u32 *total, u32 *recv);
 *
 * v2: 4-wide FIFO unroll, inline EP2 rearm (no C call per arm),
 *     tight re-poll after each packet.
 * Returns early, *recv < *total, when GINTSTS.USBRST is set (left set).
 */
	.global meson_bulk_spin_pio

	.equ GINTSTS_OFF,  0x014
	.equ GRXSTSP_OFF,  0x020
	.equ DAINTMSK_OFF, 0x81c
	.equ DOEPTSIZ2_OFF,0xb50
	.equ DOEPCTL2_OFF, 0xb40
	.equ EP2_FIFO_OFF, 0x3000
	.equ EP0_FIFO_OFF, 0x1000
	/* USBACTEP|EPTYPE_BULK|EPENA|CNAK|512 */
	.equ DOEPCTL2_ARM, 0x94008800
	/* 0x94008800 = EPENA|CNAK|USBACTEP|EPTYPE_BULK(2<<18)|MPS 512
	 * Built as: (1<<31)|(1<<26)|(1<<15)|(2<<18)|512 */

meson_bulk_spin_pio:
	stp	x29, x30, [sp, #-48]!
	mov	x29, sp
	stp	x19, x20, [sp, #16]
	stp	x21, x22, [sp, #32]
	/* no x23 on stack — keep DWC2 in x23 without save if callee-saved */
	/* x23 is callee-saved: save it */
	str	x23, [sp, #-16]!

	ldr	w19, [x0]		/* dst */
	ldr	w20, [x1]		/* total */
	mov	x21, x2			/* &recv */
	ldr	w22, [x21]		/* recv */
	mov	x23, #0x0000
	movk	x23, #0xc910, lsl #16

.Lloop:
	cmp	w22, w20
	b.hs	.Ldone

.Lpoll:
	ldr	w0, [x23, #GINTSTS_OFF]
	/* GINTSTS.USBRST: the host reset the bus; the C caller handles it */
	tbnz	w0, #12, .Ldone
	tbz	w0, #4, .Lrearm

	ldr	w0, [x23, #GRXSTSP_OFF]
	and	w1, w0, #0xf
	ubfx	w2, w0, #17, #4
	ubfx	w3, w0, #4, #11

	cmp	w1, #2
	b.ne	.Lother
	cmp	w2, #3			/* OUTDONE */
	b.eq	.Lpoll
	cmp	w2, #2			/* OUTRX */
	b.ne	.Lpoll
	cbz	w3, .Lpoll

	cmp	w3, #512
	b.ne	.Lshort
	add	x0, x19, x22, uxtw
	tbnz	x0, #0, .Lshort
	tbnz	x0, #1, .Lshort

	/* 128 words, 4 per iter = 32 loops */
	mov	w1, #32
.Lcopy:
	ldr	w2, [x23, #EP2_FIFO_OFF]
	ldr	w3, [x23, #EP2_FIFO_OFF]
	ldr	w4, [x23, #EP2_FIFO_OFF]
	ldr	w5, [x23, #EP2_FIFO_OFF]
	stp	w2, w3, [x0], #8
	stp	w4, w5, [x0], #8
	subs	w1, w1, #1
	b.ne	.Lcopy
	add	w22, w22, #512
	/* tight re-poll — most packets arrive back-to-back */
	b	.Lpoll

.Lshort:
	add	x0, x19, x22, uxtw
	mov	w4, w3
.Lsw:
	cmp	w4, #4
	b.lo	.Lsb
	ldr	w2, [x23, #EP2_FIFO_OFF]
	str	w2, [x0], #4
	sub	w4, w4, #4
	b	.Lsw
.Lsb:
	cbz	w4, .Lsdone
	ldr	w2, [x23, #EP2_FIFO_OFF]
	strb	w2, [x0], #1
	subs	w4, w4, #1
	b.eq	.Lsdone
	lsr	w2, w2, #8
	strb	w2, [x0], #1
	subs	w4, w4, #1
	b.eq	.Lsdone
	lsr	w2, w2, #8
	strb	w2, [x0], #1
.Lsdone:
	add	w22, w22, w3
	b	.Lpoll

.Lrearm:
	/* if EPENA still set, host just slow — spin */
	ldr	w0, [x23, #DOEPCTL2_OFF]
	tbnz	w0, #31, .Lloop
	/* remain = total - recv; if 0 done */
	subs	w0, w20, w22
	b.eq	.Ldone
	/* npkt = min(ceil(remain/512), EP2_RX_PKT_MAX) */
	add	w1, w0, #511
	lsr	w1, w1, #9		/* /512 */
	mov	w2, #512
	cmp	w1, w2
	csel	w1, w1, w2, ls		/* npkt = min */
	/* xfer = npkt * 512; if xfer > remain, xfer = remain */
	lsl	w2, w1, #9
	cmp	w2, w0
	csel	w2, w2, w0, ls
	/* DOEPTSIZ2 = (npkt << 19) | xfer */
	orr	w3, w2, w1, lsl #19
	str	w3, [x23, #DOEPTSIZ2_OFF]
	/* DAINTMSK |= OUT2 */
	ldr	w3, [x23, #DAINTMSK_OFF]
	orr	w3, w3, #(1 << 18)	/* bit 16+2 */
	str	w3, [x23, #DAINTMSK_OFF]
	/* DOEPCTL2 = EPENA|CNAK|USBACTEP|BULK|512 */
	mov	w3, #0x8200
	movk	w3, #0x8408, lsl #16	/* 0x94008800 */
	str	w3, [x23, #DOEPCTL2_OFF]
	/* keep bulk_recv_len in sync for any C observer */
	str	w22, [x21]
	b	.Lloop

.Lother:
	cmp	w2, #2
	b.eq	.Ldrain
	cmp	w2, #6
	b.ne	.Lpoll
.Ldrain:
	add	w3, w3, #3
	lsr	w3, w3, #2
	mov	w0, #EP0_FIFO_OFF
	add	w0, w0, w1, lsl #12
.Ldw:
	cbz	w3, .Lpoll
	ldr	w2, [x23, x0]
	sub	w3, w3, #1
	b	.Ldw

.Ldone:
	str	w22, [x21]
	ldr	x23, [sp], #16
	ldp	x21, x22, [sp, #32]
	ldp	x19, x20, [sp, #16]
	ldp	x29, x30, [sp], #48
	ret
