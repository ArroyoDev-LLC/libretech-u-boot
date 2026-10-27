#include <asm-offsets.h>
#include <config.h>
#include <linux/linkage.h>

/*
 * Early UART banner with *bounded* TX wait.
 * Preserve x30: U-Boot CRT0 calls this with bl; we must return via
 * save_boot_params_ret without clobbering the caller's LR.
 *
 * AO UART: GXL 0xc81004c0 / G12A+SM1 0xff803000 (BROM leaves it up).
 */
.globl save_boot_params
save_boot_params:
	mov	x9, x30		/* save caller LR */

	/* short delay to separate from BootROM output */
	mov	x3, #0x80000
1:	subs	x3, x3, #1
	b.ne	1b

#if defined(CONFIG_MESON_G12A)
	ldr	x0, =0xff803000	/* AO UART WFIFO */
	ldr	x2, =0xff80300c	/* AO UART STATUS */
#else
	ldr	x0, =0xc81004c0	/* AO UART WFIFO */
	ldr	x2, =0xc81004cc	/* AO UART STATUS */
#endif

	/*
	 * The first-instruction "TPL" / "SPL" mark: on GX a bring-up
	 * diagnostic (MESON_GX_XPL_VERBOSE), since the stage's banner follows
	 * in C; G12 keeps it.
	 */
#if defined(CONFIG_MESON_G12A) || defined(CONFIG_MESON_GX_XPL_VERBOSE)
	mov	w1, #0x0d
	bl	spl_uart_putc_asm
	mov	w1, #0x0a
	bl	spl_uart_putc_asm
#if defined(CONFIG_TPL_BUILD)
	mov	w1, #0x54	/* T */
	bl	spl_uart_putc_asm
	mov	w1, #0x50	/* P */
	bl	spl_uart_putc_asm
	mov	w1, #0x4c	/* L */
	bl	spl_uart_putc_asm
#else
	mov	w1, #0x53	/* S */
	bl	spl_uart_putc_asm
	mov	w1, #0x50	/* P */
	bl	spl_uart_putc_asm
	mov	w1, #0x4c	/* L */
	bl	spl_uart_putc_asm
#endif
	mov	w1, #0x0d
	bl	spl_uart_putc_asm
	mov	w1, #0x0a
	bl	spl_uart_putc_asm
#endif

#if defined(CONFIG_MESON_GX)
	/*
	 * Raise to EL3 via the BootROM "run image" SMC, if we are not there.
	 *
	 * The USB BootROM enters an image over the REQ_RUN_IN_ADDR path at
	 * EL1 (measured over the brick-safe USB-BROM loop on AML-S905X-CC-V2:
	 * a first-instruction CurrentEL print read "E1"), while BL31 -- and the
	 * spl_invoke_atf() handoff -- can only be entered from EL3.  The
	 * vendor product BL2 for this exact board raises itself the same
	 * way; read from out/aml-s905x-cc-v2.usb.bl2 at 0xd9001800:
	 *
	 *   mov x0, #0xc0000000        ; ROM "run image" function ID
	 *   ldr x1, =0xd9001880        ; -> entry_point_info
	 *   mov x2..x7, #0
	 *   smc #0
	 *   mrs x1, vbar_el3           ; <- resumes here, now at EL3
	 *
	 * and its entry_point_info at 0xd9001880 is
	 *   +0x00 = 0                  ; param_header h + attr
	 *   +0x08 = 0xd9001824         ; pc  = the instruction after the SMC
	 *   +0x10 = 0x3cd              ; spsr: M[3:0]=0b1101 (EL3h), DAIF masked
	 *
	 * i.e. the ROM's EL3 handler ERETs to {pc, spsr} of the caller's
	 * choosing.  We build the same structure in free scratch SRAM and
	 * point pc at the label below, so we come back at EL3h.
	 *
	 * Done here, at the first instruction, so that start.S's switch_el
	 * then takes its EL3 arm and CRT0 establishes SP_EL3 -- nothing
	 * between here and _main touches a stack.
	 */
#define GXL_ROM_SMC_RUN_IMAGE	0xc0000000
#define GXL_SPL_EP_INFO		0xd900c180	/* free SRAM scratch */
#define GXL_EP_SPSR_EL3H	0x3cd

	ldr	x7, =GXL_SPL_EP_INFO
	str	x9, [x7, #32]		/* stash caller LR across the ROM call */

	mrs	x6, CurrentEL
	cmp	x6, #(3 << 2)
	b.eq	2f

	str	xzr, [x7]		/* h + attr */
	adr	x8, 2f
	str	x8, [x7, #8]		/* pc */
	mov	x8, #GXL_EP_SPSR_EL3H
	str	x8, [x7, #16]		/* spsr */
	str	xzr, [x7, #24]		/* arg0 */
	dsb	sy
	isb

	mov	x0, #GXL_ROM_SMC_RUN_IMAGE
	mov	x1, x7
	mov	x2, xzr
	mov	x3, xzr
	mov	x4, xzr
	mov	x5, xzr
	mov	x6, xzr
	mov	x7, xzr
	smc	#0
	/* Not reached on success; if the ROM returns, fall through. */

	/*
	 * The ROM ERETs here at EL3.  Only x7 is known-good (we set it), and
	 * the caller LR must come back out of scratch SRAM because the ROM
	 * call does not preserve it.
	 */
2:	ldr	x7, =GXL_SPL_EP_INFO
	ldr	x9, [x7, #32]		/* caller LR back */

	/*
	 * Take an SError the previous stage left pending, before start.S
	 * unmasks it (msr DAIFClr, #4) with no vectors of ours installed.
	 *
	 * After a Linux warm restart the BootROM's USB session sometimes ends
	 * with an asynchronous external abort pending: the ROM runs with
	 * SError masked, so it carries into this image.  start.S then unmasks
	 * it, the abort is taken on the ROM's own EL3 vectors, the image never
	 * reaches _main, and the ROM's 5 s watchdog resets the chip.  Measured
	 * on AML-S905X-CC v1 (claude amlogic/software/boot/audit/
	 * 2026-10-17-gxl-optee-xtest-and-warm-reset.md): ISR_EL1.A set at this
	 * point, the abort taken at the unmask with ESR_EL3 0xbf000002, and
	 * the image boots once it is taken on vectors that just return.
	 *
	 * So: if one is pending, point VBAR_EL3 for a moment at a table whose
	 * every slot is a bare ERET (no register is touched), unmask so it is
	 * taken, mask again, put the previous VBAR_EL3 back, and log one line.
	 * Nothing changes when none is pending.
	 */
	mrs	x6, isr_el1
	tbz	x6, #8, 3f		/* ISR_EL1.A: no SError pending */
	/*
	 * An SError is taken at EL3 only with SCR_EL3.EA set; with it clear
	 * it stays pending here (measured: ESR_EL3 still read the ROM SMC's
	 * syndrome, 0x5e000000, and start.S then took the abort on the ROM's
	 * vectors).  start.S sets EA (orr #0xf) before its own unmask anyway.
	 */
	mrs	x6, scr_el3
	orr	x6, x6, #(1 << 3)	/* SCR_EL3.EA */
	msr	scr_el3, x6
	isb
	mrs	x8, vbar_el3		/* the ROM's (or previous stage's) */
	adr	x6, gx_serror_take_vectors
	msr	vbar_el3, x6
	isb
	msr	DAIFClr, #4		/* taken here, returns at once */
	isb
	msr	DAIFSet, #4
	msr	vbar_el3, x8
	isb

	ldr	x0, =0xc81004c0	/* AO UART WFIFO */
	ldr	x2, =0xc81004cc	/* AO UART STATUS */
	adr	x3, gx_serror_msg
4:	ldrb	w1, [x3], #1
	cbz	w1, 5f
	bl	spl_uart_putc_asm
	b	4b
5:	mrs	x3, esr_el3		/* still the abort's syndrome */
	mov	x6, #28
6:	lsr	x1, x3, x6
	and	w1, w1, #0xf
	add	w1, w1, #0x30		/* '0' */
	cmp	w1, #0x39
	b.le	7f
	add	w1, w1, #0x27		/* 'a' - '0' - 10 */
7:	bl	spl_uart_putc_asm
	subs	x6, x6, #4
	b.ge	6b
	mov	w1, #0x0d
	bl	spl_uart_putc_asm
	mov	w1, #0x0a
	bl	spl_uart_putc_asm
3:
#endif /* CONFIG_MESON_GX */

	mov	x30, x9		/* restore caller LR */
	b	save_boot_params_ret

#if defined(CONFIG_MESON_GX)
gx_serror_msg:
	.asciz	"SError pending at entry; ESR_EL3 after taking it="

/* Live only between the two VBAR_EL3 writes above: every slot returns. */
	.balign	0x800
gx_serror_take_vectors:
	.rept	16
	eret
	.balign	0x80
	.endr
#endif

/* w1=char, x0=wfifo, x2=status. Clobbers w4,x5. */
spl_uart_putc_asm:
	mov	x5, #0x10000
10:	ldr	w4, [x2]
	tbz	w4, #21, 11f
	subs	x5, x5, #1
	b.ne	10b
	ret
11:	str	w1, [x0]
	ret
