/* boot_aarch64.c
 *
 * Copyright (C) 2024 wolfSSL Inc.
 *
 * This file is part of wolfBoot.
 *
 * wolfBoot is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
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

/* Linker exported variables */
extern unsigned int __bss_start__;
extern unsigned int __bss_end__;
#ifndef NO_XIP
extern unsigned int _stored_data;
extern unsigned int _start_data;
extern unsigned int _end_data;
#endif

extern void main(void);
extern void gicv2_init_secure(void);

unsigned int current_el(void)
{
    unsigned long el;
    asm volatile("mrs %0, CurrentEL" : "=r" (el) : : "cc");
    return (unsigned int)((el >> 2) & 0x3U);
}

void boot_entry_C(void)
{
    register unsigned int *dst, *src;

    /* Initialize the BSS section to 0 */
    dst = &__bss_start__;
    while (dst < (unsigned int *)&__bss_end__) {
        *dst = 0U;
        dst++;
    }

#ifndef NO_XIP
    /* Copy data section from flash to RAM if necessary */
    src = (unsigned int*)&_stored_data;
    dst = (unsigned int*)&_start_data;
    if (src != dst) {
        while (dst < (unsigned int *)&_end_data) {
            *dst = *src;
            dst++;
            src++;
        }
    }
#else
    (void)src;
#endif

    /* Run wolfboot! */
    main();
}


#ifdef MMU
int WEAKFUNCTION hal_dts_fixup(void* dts_addr)
{
    (void)dts_addr;
    return 0;
}
#endif


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
#ifdef MMU
    hal_dts_fixup((uint32_t*)dts_offset);
#endif

    /* Set application address via x4 */
    asm volatile("mov x4, %0" : : "r"(app_offset));

#ifdef MMU
    /* Move the dts pointer to x5 (as first argument) */
    asm volatile("mov x5, %0" : : "r"(dts_offset));
#else
    asm volatile("mov x5, xzr");
#endif

#ifndef NO_QNX
    /* Initialize GICv2 for Kernel */
    gicv2_init_secure();
#endif

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
    wolfBoot_panic();

}
#endif

void SynchronousInterrupt(void)
{

}
void IRQInterrupt(void)
{

}
void FIQInterrupt(void)
{

}
void SErrorInterrupt(void)
{

}