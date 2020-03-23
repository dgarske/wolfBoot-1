/* boot_aarch64.c
 *
 * Copyright (C) 2020 wolfSSL Inc.
 *
 * This file is part of wolfBoot.
 *
 * wolfBoot is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * wolfBoot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1335, USA
 */

#include <stdint.h>

#include "image.h"
#include "loader.h"
#include "wolfboot/wolfboot.h"

extern unsigned int __bss_start__;
extern unsigned int __bss_end__;
static volatile unsigned int cpu_id;
extern unsigned int *END_STACK;

extern void main(void);
extern void gicv2_init_secure(void);

void boot_entry_C(void) 
{
    register unsigned int *dst;
    /* Initialize the BSS section to 0 */
    dst = &__bss_start__;
    while (dst < (unsigned int *)&__bss_end__) {
        *dst = 0U;
        dst++;
    }

    /* Run wolfboot! */
    main();
}

void disable_dcache(void)
{
	register uint32_t CsidReg;
	register uint32_t C7Reg;
	register uint32_t LineSize;
	register uint32_t NumWays;
	register uint32_t Way;
	register uint32_t WayIndex;
	register uint32_t WayAdjust;
	register uint32_t Set;
	register uint32_t SetIndex;
	register uint32_t NumSet;
	register uint32_t CacheLevel;

	asm volatile("dsb sy");

	asm volatile("mov x0, #0");
	asm volatile("mrs x0, sctlr_el3");
	asm volatile("and w0, w0, #0xfffffffb");
	asm volatile("msr sctlr_el3, x0");
	asm volatile("dsb sy");

	/* Number of level of cache */
	CacheLevel = 0U;
	/* Select cache level 0 and D cache in CSSR */
    asm volatile("msr CSSELR_EL1, %0"  : : "r" (CacheLevel));
    asm volatile("isb sy");
    asm volatile("mrs %0, CCSIDR_EL1" : "=r" (CsidReg));

	/* Get the cacheline size, way size, index size from csidr */
	LineSize = (CsidReg & 0x00000007U) + 0x00000004U;

	/* Number of Ways */
	NumWays = (CsidReg & 0x00001FFFU) >> 3U;
	NumWays += 0x00000001U;

	/* Number of Set */
	NumSet = (CsidReg >> 13U) & 0x00007FFFU;
	NumSet += 0x00000001U;

    asm volatile("clz %0,%1" : "=r" (WayAdjust) : "r" (NumWays));
	WayAdjust -= (uint32_t)0x0000001FU;

	Way = 0U;
	Set = 0U;

	/* Flush all the cachelines */
	for (WayIndex = 0U; WayIndex < NumWays; WayIndex++) {
		for (SetIndex = 0U; SetIndex < NumSet; SetIndex++) {
			C7Reg = Way | Set | CacheLevel;
            asm volatile("dc CISW, %0"  : : "r" (C7Reg));
			Set += (0x00000001U << LineSize);
		}
		Set = 0U;
		Way += (0x00000001U << WayAdjust);
	}

	/* Wait for Flush to complete */
	asm volatile("dsb sy");

	/* Select cache level 1 and D cache in CSSR */
	CacheLevel += (0x00000001U << 1U);
    asm volatile("msr CSSELR_EL1, %0"  : : "r" (CacheLevel));
	asm volatile("isb sy");
    asm volatile("mrs %0, CCSIDR_EL1" : "=r" (CsidReg));

	/* Get the cacheline size, way size, index size from csidr */
	LineSize = (CsidReg & 0x00000007U) + 0x00000004U;

	/* Number of Ways */
	NumWays = (CsidReg & 0x00001FFFU) >> 3U;
	NumWays += 0x00000001U;

	/* Number of Sets */
	NumSet = (CsidReg >> 13U) & 0x00007FFFU;
	NumSet += 0x00000001U;

    asm volatile("clz %0,%1" : "=r" (WayAdjust) : "r" (NumWays));
	WayAdjust -= (uint32_t)0x0000001FU;

	Way = 0U;
	Set = 0U;

	/* Flush all the cachelines */
	for (WayIndex =0U; WayIndex < NumWays; WayIndex++) {
		for (SetIndex =0U; SetIndex < NumSet; SetIndex++) {
			C7Reg = Way | Set | CacheLevel;
			asm volatile("dc CISW,%0"  : : "r" (C7Reg));
			Set += (0x00000001U << LineSize);
		}
		Set=0U;
		Way += (0x00000001U<<WayAdjust);
	}
	/* Wait for Flush to complete */
	asm volatile("dsb sy");

	asm volatile("tlbi ALLE3");
    asm volatile("dsb sy");
    asm volatile("isb");
}

static void switch_to_el2(void)
{
    asm volatile("mov   x0, #0x5b1");           /* Non-secure EL0/EL1 | HVC | 64bit EL2 */
	asm volatile("msr   scr_el3, x0");
	asm volatile("msr   cptr_el3, xzr");        /* Disable coprocessor traps to EL3 */
	asm volatile("mov   x0, #0x33ff");
	asm volatile("msr   cptr_el2, x0");         /* Disable coprocessor traps to EL2 */

	/* Initialize SCTLR_EL2 */
	asm volatile("msr   sctlr_el2, xzr");

	/* Return to the EL2_SP2 mode from EL3 */
	asm volatile("mov   x0, sp");
	asm volatile("msr   sp_el2, x0");           /* Migrate SP */
	asm volatile("mrs   x0, vbar_el3");
	asm volatile("msr   vbar_el2, x0");	        /* Migrate VBAR */
	asm volatile("mov   x0, #0x3c9");
	asm volatile("msr   spsr_el3, x0");         /* EL2_SP2 | D | A | I | F */
	asm volatile("msr   elr_el3, x30");
	asm volatile("eret");
}

static void switch_to_el1(void)
{
    /* Initialize Generic Timers */
	asm volatile("mrs   x0, cnthctl_el2");
	asm volatile("orr   x0, x0, #0x3");         /* Enable EL1 access to timers */
	asm volatile("msr   cnthctl_el2, x0");
	asm volatile("msr   cntvoff_el2, xzr");
	asm volatile("mrs   x0, cntkctl_el1");
	asm volatile("orr   x0, x0, #0x3");         /* Enable EL0 access to timers */
	asm volatile("msr   cntkctl_el1, x0");

	/* Initialize MPID/MPIDR registers */
	asm volatile("mrs   x0, midr_el1");
	asm volatile("mrs   x1, mpidr_el1");
	asm volatile("msr   vpidr_el2, x0");
	asm volatile("msr   vmpidr_el2, x1");

	/* Disable coprocessor traps */
	asm volatile("mov   x0, #0x33ff");
	asm volatile("msr   cptr_el2, x0");         /* Disable coprocessor traps to EL2 */
	asm volatile("msr   hstr_el2, xzr");        /* Disable coprocessor traps to EL2 */
	asm volatile("mov   x0, #3 << 20");
	asm volatile("msr   cpacr_el1, x0");        /* Enable FP/SIMD at EL1 */

	/* Initialize HCR_EL2 */
	asm volatile("mov   x0, #(1 << 31)");       /* 64 bit EL1 */
	asm volatile("orr   x0, x0, #(1 << 29)");   /* Disable HVC */
	asm volatile("msr   hcr_el2, x0");

	/* SCTLR_EL1 initialization */
	asm volatile("mov   x0, #0x0800");
	asm volatile("movk  x0, #0x30d0, lsl #16");
	asm volatile("msr   sctlr_el1, x0");

	/* Return to the EL1_SP1 mode from EL2 */
	asm volatile("mov   x0, sp");
	asm volatile("msr   sp_el1, x0");           /* Migrate SP */
	asm volatile("mrs   x0, vbar_el2");
	asm volatile("msr   vbar_el1, x0");         /* Migrate VBAR */
	asm volatile("mov   x0, #0x3c5");
	asm volatile("msr   spsr_el2, x0");         /* EL1_SP1 | D | A | I | F */
	asm volatile("msr   elr_el2, x30");
	asm volatile("eret");
}


/* This is the main loop for the bootloader.
 *
 * It performs the following actions:
 *  - Call the application entry point
 *
 */

#ifdef MMU
void RAMFUNCTION do_boot(const uint32_t *app_offset, const uint32_t* dts_offset)
#else
void RAMFUNCTION do_boot(const uint32_t *app_offset)
#endif
{
    register uint32_t current_el;

    /* Set application address via x4 */
    asm volatile("mov x4, %0" : : "r"(app_offset));
#ifdef MMU
    /* move the device tree address into x5 register */
    asm volatile("mov x5, %0" : : "r"(dts_offset));
#endif

    /* Disable Cache */
    disable_dcache();

    /* Read current EL */
    asm volatile("mrs %0, CurrentEL" : "=r" (current_el) : : "cc");
    current_el >>= 2;

    /* Switch to Exception Level 2 */
    if (current_el == 3) {
        switch_to_el2();

        /* Read current EL */
        asm volatile("mrs %0, CurrentEL" : "=r" (current_el) : : "cc");
        current_el >>= 2;
    }

#ifdef CONFIG_ARMV8_SWITCH_TO_EL1
    /* Switch to Exception Level 1 (Kernel) */
    if (current_el == 2) {
        switch_to_el1();
    }
#endif

#ifdef MMU
    /* move the dts pointer to x0 */
    asm volatile("mov x0, x5");
#else
	asm volatile("mov x5, xzr");
#endif

    /* Initialize GICv2 for Kernel */
    gicv2_init_secure();

    /* Zero registers x1, x2, x3 */
    asm volatile("mov x3, xzr");
    asm volatile("mov x2, xzr");
    asm volatile("mov x1, xzr");

    /* Move the dts pointer to x0 (as first argument) */
    asm volatile("mov x0, x5");

    /* Unconditionally jump to app_entry at x4 */
    asm volatile("br x4");
}

#ifdef RAM_CODE

#define AIRCR *(volatile uint32_t *)(0xE000ED0C)
#define AIRCR_VKEY (0x05FA << 16)
#define AIRCR_SYSRESETREQ (1 << 2)

void RAMFUNCTION arch_reboot(void)
{
    AIRCR = AIRCR_SYSRESETREQ | AIRCR_VKEY;
    while(1)
        ;

}
#endif
