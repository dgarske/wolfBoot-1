/** @file HL_sys_main.c
*   @brief Application main file
*   @date 11-Dec-2018
*   @version 04.07.01
*
*   This file contains an empty main function,
*   which can be used for the application.
*/

/*
* Copyright (C) 2009-2018 Texas Instruments Incorporated - www.ti.com
*
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


/* USER CODE BEGIN (0) */
/* USER CODE END */

/* Include Files */

#include "HL_sys_common.h"
#include "HL_system.h"
#include <wolfboot/wolfboot.h>

/* USER CODE BEGIN (1) */
#include "HL_gio.h"
#include "HL_sci.h"
#include <stdio.h>
#include <stdint.h>
#include "printf.h"
#include "hal.h"
/* USER CODE END */

/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */
int sci_printf(const char *_format, ...);

static const char* state2str(uint8_t s)
{
    switch(s) {
        case IMG_STATE_NEW: return "New";
        case IMG_STATE_UPDATING: return "Updating";
        case IMG_STATE_TESTING: return "Testing";
        case IMG_STATE_SUCCESS: return "Success";
        default: return "Unknown";
    }
}

static void printPart(uint8_t *part)
{
#ifdef WOLFBOOT_DEBUG_PARTION
    uint32_t *v;
    int i;
#endif
    uint8_t  *magic;
    uint8_t  state;
    uint32_t ver;

    magic = part;
    sci_printf("Magic:    %c%c%c%c\n", magic[0], magic[1], magic[2], magic[3]);
    ver = wolfBoot_get_blob_version(part);
    sci_printf("Version:  %02x\n", ver);
    state = *(part + WOLFBOOT_PARTITION_SIZE - sizeof(uint32_t) - 1);
    sci_printf("Status:   %02x (%s)\n", state,state2str(state));
    magic = part + WOLFBOOT_PARTITION_SIZE - sizeof(uint32_t);
    sci_printf("Tail Mgc: %c%c%c%c\n", magic[0], magic[1], magic[2], magic[3]);

#ifdef WOLFBOOT_DEBUG_PARTION
    v = (uint32_t *)part;
    for(i = 0; i < 0x100/4; i++) {
        if(i % 4 == 0)
            print("\n%08x: ", (uint32_t)v+i*4);
        print("%08x ", v[i]);
    }
#endif

}


static void printPartitions(void)
{
    sci_printf("\n=== Boot Partition[%08x] ===\n", WOLFBOOT_PARTITION_BOOT_ADDRESS);
    printPart((uint8_t*)WOLFBOOT_PARTITION_BOOT_ADDRESS);
    sci_printf("\n=== Update Partition[%08x] ===\n", WOLFBOOT_PARTITION_UPDATE_ADDRESS);
    printPart((uint8_t*)WOLFBOOT_PARTITION_UPDATE_ADDRESS);
}

/* USER CODE END */

int main(void)
{
/* USER CODE BEGIN (3) */
    gioInit();
    sciInit();
    hal_init();

    uint8_t firmware_version = 0;
    gioSetDirection(gioPORTA, 0x00000001);

    sci_printf("\n=== Sector Size[%08x] ===\n", WOLFBOOT_SECTOR_SIZE);
    printPartitions();

    /* The same as: wolfBoot_get_image_version(PART_BOOT); */
    firmware_version = wolfBoot_current_firmware_version();

    sci_printf("\nCurrent Firmware Version: %d\n", firmware_version);

    if (firmware_version >= 1) {
        if (firmware_version == 1) {
            sci_printf("Press GPIOA7 to call wolfBoot_success the firmware.\n");
            while (gioGetBit(gioPORTA, 7) != 0)
            {
                // Wait
            }

            wolfBoot_success();
            printPartitions();

            sci_printf("\nPress GPIOA7 to update the firmware.\n");
            while (gioGetBit(gioPORTA, 7) != 0)
            {
                // Wait
            }

            wolfBoot_update_trigger();
            sci_printf("Firmware Update is triggered\n");
            printPartitions();

        } else if (firmware_version == 2) {
            sci_printf("Press GPIOA7 to call wolfBoot_success the firmware.\n");
            while (gioGetBit(gioPORTA, 7) != 0)
            {
                // Wait
            }

            wolfBoot_success();
            printPartitions();
        }
    } else {
        sci_printf("Invalid Firmware Version\n");
        goto busy_idle;
    }

    /* busy wait */
busy_idle:
    while (1)
        ;
/* USER CODE END */
}


/* USER CODE BEGIN (4) */
void sciNotification(sciBASE_t *sci, unsigned flags)
{
   return;
}

int sci_printf(const char *_format, ...)
{
   char str[128];
   int length = -1;

   va_list argList;
   va_start( argList, _format );

   length = vsnprintf(str, sizeof(str), _format, argList);

   va_end( argList );

   if (length > 0)
   {
      sciSend(sciREG1, (unsigned)length, (unsigned char*)str);
   }

   return length;
}
/* USER CODE END */
