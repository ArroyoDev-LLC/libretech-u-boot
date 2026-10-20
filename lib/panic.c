/*
 *  linux/lib/vsprintf.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 */

/* vsprintf.c -- Lars Wirzenius & Linus Torvalds. */
/*
 * Wirzenius wrote this portably, Torvalds fucked it up :-)
 */

#include <console.h>
#include <hang.h>
#if !defined(CONFIG_PANIC_HANG)
#include <command.h>
#endif
#include <linux/delay.h>
#include <stdio.h>

static void panic_finish(void) __attribute__ ((noreturn));

/*
 * A panic is the definition of "this boot did not happen", so it is exactly
 * what console_silent_defeat() is for -- and it is the one failure the silent
 * console could hide completely.
 *
 * With CONFIG_SILENT_CONSOLE_UNTIL_ENV the console is silent from
 * console_init_f() until initr_env(), and everything printed in that window
 * goes to the pre-console buffer instead of the UART. console_init_r() would
 * replay it, but a panic before that point calls do_reset() and the board is
 * gone: the buffer, the panic message and the reason all disappear together
 * and the symptom is a board that reboots in silence. That window covers
 * print_cpuinfo, dram_init, initr_dm, board_init and initr_mmc -- the most
 * diagnostic failures on the board.
 *
 * Called BEFORE the message is formatted, so the message itself lands on the
 * live console rather than in the buffer. console_silent_defeat() flushes the
 * buffer too, so the context leading up to the panic comes out with it. It is
 * a no-op on a console that is not silent, and compiles away entirely without
 * CONFIG_SILENT_CONSOLE.
 *
 * The SERIAL guard mirrors lib/hang.c, which has always carried one, and the
 * asymmetry was the defect rather than any observed breakage. panic.o is obj-y
 * in every phase while console.o is obj-$(CONFIG_$(PHASE_)SERIAL), and
 * console_silent_defeat()'s own header guard is a plain #ifdef
 * CONFIG_SILENT_CONSOLE with no phase prefix -- so a phase with
 * SILENT_CONSOLE=y and SERIAL=n compiles a call to a function that was not
 * built. No in-tree defconfig is in that state today (all 98 with
 * SILENT_CONSOLE=y were checked), which is exactly why it wants fixing now:
 * the failure is a link error in somebody else's future defconfig, with
 * nothing in this file to explain it.
 */
static void panic_unsilence(void)
{
#if CONFIG_IS_ENABLED(SERIAL)
	console_silent_defeat("panic");
#endif
}

static void panic_finish(void)
{
	putc('\n');
#if defined(CONFIG_PANIC_HANG)
	hang();
#else
	flush();  /* flush the panic message before reset */

	do_reset(NULL, 0, 0, NULL);
#endif
	while (1)
		;
}

void panic_str(const char *str)
{
	panic_unsilence();
	puts(str);
	panic_finish();
}

void panic(const char *fmt, ...)
{
	panic_unsilence();
#if CONFIG_IS_ENABLED(PRINTF)
	va_list args;
	va_start(args, fmt);
	vprintf(fmt, args);
	va_end(args);
#endif
	panic_finish();
}

void __assert_fail(const char *assertion, const char *file, unsigned int line,
		   const char *function)
{
	/* This will not return */
	panic("%s:%u: %s: Assertion `%s' failed.", file, line, function,
	      assertion);
}
