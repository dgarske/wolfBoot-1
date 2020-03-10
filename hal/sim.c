/* sim.c
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
#include <string.h>
#include <stdio.h>

#include <target.h>
#include "image.h"

/* public HAL functions */
void hal_init(void)
{

}

void hal_prepare_boot(void)
{

}

/* Flash functions must be relocated to RAM for execution */
int RAMFUNCTION hal_flash_write(uint32_t address, const uint8_t *data, int len)
{
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
    return 0;
}

int RAMFUNCTION ext_flash_write(uintptr_t address, const uint8_t *data, int len)
{
    return 0;
}

int RAMFUNCTION ext_flash_read(uintptr_t address, uint8_t *data, int len)
{
    return 0;
}

int RAMFUNCTION ext_flash_erase(uintptr_t address, int len)
{
    return 0;
}

void RAMFUNCTION ext_flash_lock(void)
{

}

void RAMFUNCTION ext_flash_unlock(void)
{

}
