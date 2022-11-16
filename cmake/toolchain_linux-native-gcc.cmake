# toolchain_linux-native-gcc.cmake
#
# Copyright (C) 2022 wolfSSL Inc.
#
# This file is part of wolfBoot.
#
# wolfBoot is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# wolfBoot is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1335, USA

set(TARGET_ARCH "ARCH_NATIVE" CACHE INTERNAL "")
set(TARGET_PLATFORM "linux" CACHE INTERNAL "")
set(TARGET_PLATFORM_CONFIG "linux" CACHE INTERNAL "")

#-----------------------------------------------------------------------------
# Add flags here that should be applied to help enforce the Rogue SW coding standard
# NOTE: flags added here should also have their equivalent 'deactivator' (as applicable) in ROGUE_CODE_STD_FLAG_DISABLES
#-----------------------------------------------------------------------------
# -Werror=switch-enum   Trigger compiler error on enum switch statement that does not have a case for every enum value
# -Werror=return-type   Warn about any return statement with no return value in a function whose return type is not void
set(ROGUE_CODE_STD_FLAGS "-Werror=switch-enum -Werror=return-type")
set(ROGUE_CODE_STD_CXX_FLAGS "-Wsuggest-override")
# Use this variable in target_compile_options to disable rogue code-standard warnings/errors on external libraries.
# Additions to this list should be space delimited. ie: "addition1" "addition2"...
set(ROGUE_CODE_STD_FLAG_DISABLES "-Wno-switch-enum" "-Wno-return-type")
set(ROGUE_CODE_STD_CXX_FLAG_DISABLES "-Wno-suggest-override")
# TODO: These flags are not final and need to be smoothed out
#
#
set(OBJECT_GEN_FLAGS "-Wall -Wextra")

# NOTE: Use CMAKE_*_STANDARD instead of -std=
set(CMAKE_C_FLAGS   "${OBJECT_GEN_FLAGS}" CACHE INTERNAL "C Compiler options")
set(CMAKE_CXX_FLAGS "${OBJECT_GEN_FLAGS}" CACHE INTERNAL "C++ Compiler options")
set(CMAKE_ASM_FLAGS "${OBJECT_GEN_FLAGS} -x assembler-with-cpp " CACHE INTERNAL "ASM Compiler options")

# Set flags for the 4 standard build types
# Options for DEBUG build
# -O0   Reduce compilation time and make debugging produce the expected results. This is the default.
# -g    Produce debugging information in the operating system’s native format.
set(CMAKE_C_FLAGS_DEBUG       "${ROGUE_CODE_STD_FLAGS} -O0 -g"    CACHE INTERNAL "C Compiler options for debug build type")
set(CMAKE_CXX_FLAGS_DEBUG     "${ROGUE_CODE_STD_FLAGS} ${ROGUE_CODE_STD_CXX_FLAGS} -O0 -g"  CACHE INTERNAL "C++ Compiler options for debug build type")
set(CMAKE_ASM_FLAGS_DEBUG       "-g"        CACHE INTERNAL "ASM Compiler options for debug build type")
set(CMAKE_EXE_LINKER_FLAGS_DEBUG ""         CACHE INTERNAL "Linker options for debug build type")

set(CMAKE_C_FLAGS_RELEASE       "${ROGUE_CODE_STD_FLAGS} -O3 -g"    CACHE INTERNAL "C Compiler options for release build type")
set(CMAKE_CXX_FLAGS_RELEASE     "${ROGUE_CODE_STD_FLAGS} ${ROGUE_CODE_STD_CXX_FLAGS} -O3 -g"  CACHE INTERNAL "C++ Compiler options for release build type")
set(CMAKE_ASM_FLAGS_RELEASE ""                                  CACHE INTERNAL "ASM Compiler options for release build type")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE ""  CACHE INTERNAL "Linker options for release build type")

set(CMAKE_C_FLAGS_RELWITHDEBINFO       "${ROGUE_CODE_STD_FLAGS} -O3 -g"    CACHE INTERNAL "C Compiler options for release with debug symbols build type")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO     "${ROGUE_CODE_STD_FLAGS} ${ROGUE_CODE_STD_CXX_FLAGS} -O3 -g"  CACHE INTERNAL "C++ Compiler options for release with debug symbols build type")
set(CMAKE_ASM_FLAGS_RELWITHDEBINFO ""                                  CACHE INTERNAL "ASM Compiler options for RELWITHDEBINFO build type")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE ""  CACHE INTERNAL "Linker options for release build type")

set(CMAKE_C_FLAGS_MINSIZEREL       "${ROGUE_CODE_STD_FLAGS} -Os"    CACHE INTERNAL "C Compiler options for minimum size release build type")
set(CMAKE_CXX_FLAGS_MINSIZEREL     "${ROGUE_CODE_STD_FLAGS} ${ROGUE_CODE_STD_CXX_FLAGS} -Os"  CACHE INTERNAL "C++ Compiler options for minimum size release build type")
set(CMAKE_ASM_FLAGS_MINSIZEREL ""                                  CACHE INTERNAL "ASM Compiler options for RELWITHDEBINFO build type")
set(CMAKE_EXE_LINKER_FLAGS_MINSIZEREL ""  CACHE INTERNAL "Linker options for release build type")


# Detect if we are compiling locally on the xavier-nx (can't use CMAKE_HOST_SYSTEM_PROCESSOR
# because the toolchain isn't setup until the project command has been called)
# TODO: Can we use CMAKE_HOST_SYSTEM_PROCESSOR now that we're using the toolchain "correctly"?
execute_process(COMMAND uname -m
    OUTPUT_VARIABLE  HOST_UNAME_ARCH
    OUTPUT_STRIP_TRAILING_WHITESPACE)
if (HOST_UNAME_ARCH STREQUAL "aarch64")
    set(TARGET_HARDWARE "XAVIER_NX" CACHE INTERNAL "")
endif()
