/* SPDX-License-Identifier: GPL-2.0+ */
/*
 *  crt0 - C-runtime startup Code for ARM U-Boot
 *
 *  Copyright (c) 2012  Albert ARIBAUD <albert.u.boot@aribaud.net>
 */

#include <config.h>
#include <asm-offsets.h>
#include <linux/linkage.h>
#include <asm/assembler.h>

/*
 * This file handles the target-independent stages of the U-Boot
 * start-up where a C runtime environment is needed. Its entry point
 * is _main and is branched into from the target's start.S file.
 *
 * _main execution sequence is:
 *
 * 1. Set up initial environment for calling board_init_f().
 *    This environment only provides a stack and a place to store
 *    the GD ('global data') structure, both located in some readily
 *    available RAM (SRAM, locked cache...). In this context, VARIABLE
 *    global data, initialized or not (BSS), are UNAVAILABLE; only
 *    CONSTANT initialized data are available. GD should be zeroed
 *    before board_init_f() is called.
 *
 * 2. Call board_init_f(). This function prepares the hardware for
 *    execution from system RAM (DRAM, DDR...) As system RAM may not
 *    be available yet, , board_init_f() must use the current GD to
 *    store any data which must be passed on to later stages. These
 *    data include the relocation destination, the future stack, and
 *    the future GD location.
 *
 * 3. Set up intermediate environment where the stack and GD are the
 *    ones allocated by board_init_f() in system RAM, but BSS and
 *    initialized non-const data are still not available.
 *
 * 4a.For U-Boot proper (not SPL), call relocate_code(). This function
 *    relocates U-Boot from its current location into the relocation
 *    destination computed by board_init_f().
 *
 * 4b.For SPL, board_init_f() just returns (to crt0). There is no
 *    code relocation in SPL.
 *
 * 5. Set up final environment for calling board_init_r(). This
 *    environment has BSS (initialized to 0), initialized non-const
 *    data (initialized to their intended value), and stack in system
 *    RAM (for SPL moving the stack and GD into RAM is optional - see
 *    CONFIG_SPL_STACK_R). GD has retained values set by board_init_f().
 *
 * 6. For U-Boot proper (not SPL), some CPUs have some work left to do
 *    at this point regarding memory, so call c_runtime_cpu_setup.
 *
 * 7. Branch to board_init_r().
 *
 * For more information see 'Board Initialisation Flow in README.
 */

/*
 * Macro for clearing BSS during SPL execution. Usually called during the
 * relocation process for most boards before entering board_init_r(), but
 * can also be done early before entering board_init_f() on plaforms that
 * can afford it due to sufficient memory being available early.
 */

.macro CLEAR_BSS
	ldr	r0, =__bss_start	/* this is auto-relocated! */

#ifdef CONFIG_USE_ARCH_MEMSET
	ldr	r3, =__bss_end		/* this is auto-relocated! */
	mov	r1, #0x00000000		/* prepare zero to clear BSS */

	subs	r2, r3, r0		/* r2 = memset len */
	bl	memset
#else
	ldr	r1, =__bss_end		/* this is auto-relocated! */
	mov	r2, #0x00000000		/* prepare zero to clear BSS */

clbss_l:cmp	r0, r1			/* while not at end of BSS */
	strlo	r2, [r0]		/* clear 32-bit BSS word */
	addlo	r0, r0, #4		/* move to next */
	blo	clbss_l
#endif
.endm

#if defined(CONFIG_TARGET_ITOP4412)
/*
 * Relocate code
 */
relocate_code_xinnian:
	adr	r0, nscode_base			@ r0: source address (start)
	adr	r1, nscode_end			@ r1: source address (end)
	ldr	r2, =#0x0204f000		@ r2: target address

1:
	ldmia	r0!, {r3-r6}
	stmia	r2!, {r3-r6}
	cmp	r0, r1
	blt	1b

	dsb
	isb

	mov	pc, lr


	.align	4
nscode_base:
	adr	r0, _ns_reg5
	b	1f

	.word	0x0		@ REG0: RESUME_ADDR
	.word	0x0		@ REG1: RESUME_FLAG
	.word	0x0		@ REG2
	.word	0x0		@ REG3
	.word	0x0		@ REG4
_ns_reg5:
	.word	0x0		@ REG5: CPU1_BOOT_REG
	.word	0x0		@ REG6: REG_DIRECTGO_FLAG
	.word	0x0		@ REG7: REG_DIRECTGO_ADDR
	.word	0x0		@ REG8
	.word	0x0		@ REG9

	nop
	nop

1:
	mrc	p15, 0, r1, c0, c0, 5		@ MPIDR
	and	r1, r1, #0x3
	add	r0, r0, r1, lsl #0x2
cpu1_wait:
	.word	0xE320F002			@ wfe instruction
	ldr	r1, [r0]
	cmp	r1, #0x0
	bxne	r1
	b	cpu1_wait
	nop
nscode_end:
#endif
/*
 * entry point of crt0 sequence
 */

ENTRY(_main)

/*
 * Set up initial C runtime environment and call board_init_f(0).
 */

#if defined(CONFIG_TPL_BUILD) && defined(CONFIG_TPL_NEEDS_SEPARATE_STACK)
	ldr	r0, =(CONFIG_TPL_STACK)
#elif defined(CONFIG_SPL_BUILD) && defined(CONFIG_SPL_STACK)
	ldr	r0, =(CONFIG_SPL_STACK)
#else
	ldr	r0, =(CONFIG_SYS_INIT_SP_ADDR)
#endif
	bic	r0, r0, #7	/* 8-byte alignment for ABI compliance */
	mov	sp, r0
	bl	board_init_f_alloc_reserve
	mov	sp, r0
	/* set up gd here, outside any C code */
	mov	r9, r0
	bl	board_init_f_init_reserve

#if defined(CONFIG_SPL_BUILD) && defined(CONFIG_SPL_EARLY_BSS)
	CLEAR_BSS
#endif
	
#if defined(CONFIG_TARGET_ITOP4412)
	bl	relocate_code_xinnian
#endif	
	mov	r0, #0
	bl	board_init_f

#if ! defined(CONFIG_SPL_BUILD)

/*
 * Set up intermediate environment (new sp and gd) and call
 * relocate_code(addr_moni). Trick here is that we'll return
 * 'here' but relocated.
 */

	ldr	r0, [r9, #GD_START_ADDR_SP]	/* sp = gd->start_addr_sp */
	bic	r0, r0, #7	/* 8-byte alignment for ABI compliance */
	mov	sp, r0
	ldr	r9, [r9, #GD_NEW_GD]		/* r9 <- gd->new_gd */

	adr	lr, here
	ldr	r0, [r9, #GD_RELOC_OFF]		/* r0 = gd->reloc_off */
	add	lr, lr, r0
#if defined(CONFIG_CPU_V7M)
	orr	lr, #1				/* As required by Thumb-only */
#endif
	ldr	r0, [r9, #GD_RELOCADDR]		/* r0 = gd->relocaddr */
	b	relocate_code
here:
/*
 * now relocate vectors
 */

	bl	relocate_vectors

/* Set up final (full) environment */

	bl	c_runtime_cpu_setup	/* we still call old routine here */
#endif
#if !defined(CONFIG_SPL_BUILD) || CONFIG_IS_ENABLED(FRAMEWORK)

#if !defined(CONFIG_SPL_BUILD) || !defined(CONFIG_SPL_EARLY_BSS)
	CLEAR_BSS
#endif

# ifdef CONFIG_SPL_BUILD
	/* Use a DRAM stack for the rest of SPL, if requested */
	bl	spl_relocate_stack_gd
	cmp	r0, #0
	movne	sp, r0
	movne	r9, r0
# endif

#if ! defined(CONFIG_SPL_BUILD)
	bl coloured_LED_init
	bl red_led_on
#endif
	/* call board_init_r(gd_t *id, ulong dest_addr) */
	mov     r0, r9                  /* gd_t */
	ldr	r1, [r9, #GD_RELOCADDR]	/* dest_addr */
	/* call board_init_r */
#if CONFIG_IS_ENABLED(SYS_THUMB_BUILD)
	ldr	lr, =board_init_r	/* this is auto-relocated! */
	bx	lr
#else
	ldr	pc, =board_init_r	/* this is auto-relocated! */
#endif
	/* we should not return here. */
#endif

ENDPROC(_main)
