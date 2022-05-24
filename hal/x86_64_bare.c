
/* x86_64_native.c
 *
 * Copyright (C) 2022 wolfSSL Inc.
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
#include <target.h>

#include "image.h"
#include "loader.h"
#include "printf.h"

#ifdef PLATFORM_X86_64_BARE

/* Intel microcode header */
typedef struct MICROCODE_IMAGE_HEADER {
    uint32_t HeaderVersion;
    uint32_t PatchId;
    uint32_t Date;
    uint32_t CpuId;
    uint32_t Checksum;
    uint32_t LoaderVersion;
    uint32_t PlatformId;
    uint32_t DataSize;
    uint32_t TotalSize;
    uint32_t Reserved[3];
} MICROCODE_IMAGE_HEADER;

#ifdef __WOLFBOOT
void hal_init(void)
{
}

void hal_prepare_boot(void)
{
}

#endif

int RAMFUNCTION hal_flash_write(uint32_t address, const uint8_t *data, int len)
{
    (void)address;
    (void)data;
    (void)len;
    return 0;
}

void RAMFUNCTION hal_flash_unlock(void)
{
}

void RAMFUNCTION hal_flash_lock(void)
{
}

int RAMFUNCTION hal_flash_erase(uint32_t address, int len)
{
    (void)address;
    (void)len;
    return 0;
}

void* hal_get_primary_address(void)
{
    /* TODO: Get kernel_addr */
    return (void*)0;
}
void* hal_get_update_address(void)
{
    /* TODO: Get update_addr */
    return (void*)0;
}

#endif /* PLATFORM_X86_64_BARE */
