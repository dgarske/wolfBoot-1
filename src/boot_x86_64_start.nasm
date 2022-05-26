;  boot_x86_64_start.nasm
; 
;  Copyright (C) 2022 wolfSSL Inc.
; 
;  This file is part of wolfBoot.
; 
;  wolfBoot is free software; you can redistribute it and/or modify
;  it under the terms of the GNU General Public License as published by
;  the Free Software Foundation; either version 2 of the License, or
;  (at your option) any later version.
; 
;  wolfBoot is distributed in the hope that it will be useful,
;  but WITHOUT ANY WARRANTY; without even the implied warranty of
;  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;  GNU General Public License for more details.
; 
;  You should have received a copy of the GNU General Public License
;  along with this program; if not, write to the Free Software
;  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1335, USA
;

; NASM assembly format

extern main

; FSP API offset
%define FSPT_BASE                     0xFFFF0000
%define FSP_HEADER_TEMPRAMINIT_OFFSET 0x30

; Bootloader Stack
%define BOOTLOADER_STACK_SIZE         0x00002000
%define BOOTLOADER_STACK_BASE         0x0

SECTION .text
global FspTempRamInit
FspTempRamInit:
    ; Set FSP-T base in eax
    mov eax, dword FSPT_BASE

    ; Find the FSP info header
    add eax, dword [eax + 094h + FSP_HEADER_TEMPRAMINIT_OFFSET]

    ; Jump to TempRamInit API, FSP will return using esp
    jmp eax

_start:
    movd mm0, eax

    ; Read time stamp
    rdtsc
    mov esi, eax
    mov edi, edx

    ; Init Temp RAM
    mov esp, FspTempRamInitRet
    jmp FspTempRamInit

FspTempRamInitRet:

    ; Setup stack
    ;  ECX: Bootloader stack base
    ;  edx: Bootloader stack top
    mov esp, ecx
    add esp, dword [BOOTLOADER_STACK_BASE]
    add esp, dword [BOOTLOADER_STACK_SIZE]

    xor ebx, ebx ; Use EBX for Status

    ; CpuBist error check
    movd eax, mm0
    emms                ; Exit MMX Instruction
    cmp eax, 0
    jz CheckStatusDone

    ; Error in CpuBist
    bts ebx, 0          ; Set 0x00000001 in Status

CheckStatusDone:

    ; Setup HOB
    ;  This structure has to match the layout of STAGE1A_ASM_PARAM
    push $0 ; Status[63:32]
    push ebx ; Status[31:0]
    push edi ; TimeStamp[0] [63:32]
    push esi ; TimeStamp[0] [31:0]
    push edx ; CarTop
    push ecx ; CarBase
    push $0 ; Keep the stack 16-byte aligned

    lea ecx, [esp + 4]
    push ecx
    call main ; Jump to C code
    jmp $
