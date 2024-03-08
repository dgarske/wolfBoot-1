/* app_sim.c
 *
 * Test bare-metal boot-led-on application
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

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "wolfboot/wolfboot.h"

#ifdef PLATFORM_sim

/* Matches all keys:
 *    - chacha (32 + 12)
 *    - aes128 (16 + 16)
 *    - aes256 (32 + 16)
 */
/* Longest key possible: AES256 (32 key + 16 IV = 48) */
char enc_key[] = "0123456789abcdef0123456789abcdef"
		 "0123456789abcdef";

#ifdef TEST_DELTA_DATA
static volatile char __attribute__((used)) garbage[TEST_DELTA_DATA] = {0x01, 0x02, 0x03, 0x04 };

#endif

void hal_init(void);

int do_cmd(const char *cmd)
{
    if (strcmp(cmd, "powerfail") == 0) {
        return 1;
    }
    if (strcmp(cmd, "get_version") == 0) {
        printf("%d\n", wolfBoot_current_firmware_version());
        return 0;
    }
    if (strcmp(cmd, "success") == 0) {
        wolfBoot_success();
        return 0;
    }
    if (strcmp(cmd, "update_trigger") == 0) {
#if EXT_ENCRYPTED
        wolfBoot_set_encrypt_key((uint8_t *)enc_key,(uint8_t *)(enc_key +  32));
#endif
        wolfBoot_update_trigger();
        return 0;
    }
    if (strcmp(cmd, "reset") == 0) {
        exit(0);
    }
    if (strncmp(cmd, "get_tlv",7) == 0) {
        uint8_t* imageHdr = (uint8_t*)WOLFBOOT_PARTITION_BOOT_ADDRESS;
        uint8_t* ptr = NULL;
        uint16_t tlv = 0x34; /* default */
        int size;
        int i;

        const char* tlvStr = strstr(cmd, "get_tlv=");
        if (tlvStr) {
            tlvStr += strlen("get_tlv=");
            tlv = (uint16_t)atoi(tlvStr);
        }
        printf("Get TLV %04x\r\n", tlv);

        size = wolfBoot_find_header(imageHdr + IMAGE_HEADER_OFFSET, tlv, &ptr);
        if (size > 0 && ptr != NULL) {
            /* From here, the value 0xAABBCCDD is at ptr */
            printf("TLV 0x%x: found. Size: %d\n", tlv, size);
            for (i=0; i<size; i++) {
                printf("%02X", ptr[i]);
            }
            printf("\n");
            return 0;
        } else {
            printf("TLV: not found!\r\n");
            return -1;
        }
    }
    /* wrong command */
    return -1;
}

int main(int argc, char *argv[]) {

    int i;
    int ret;

    hal_init();

    for (i = 1; i < argc; ++i) {
        ret = do_cmd(argv[i]);
        if (ret < 0)
            return -1;
        i+= ret;
    }
    return 0;

}
#endif /** PLATFORM_sim **/
