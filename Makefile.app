# Example for build wolfBoot in standalone application side
APPNAME=wolfbootapp

# Put build options into "include/target.h"
-include .config
TARGET=sim
ARCH=sim

# The WOLFBOOT_APP indicates use of wolfBoot API's only, no verification or signing
CFLAGS+=-DWOLFBOOT_APP -Iinclude
CFLAGS+=-DARCH_$(ARCH)
CFLAGS+=-g

OBJS:= \
    ./hal/$(TARGET).o \
    ./src/libwolfboot.o

ifeq ($(HASH),SHA256)
  CFLAGS+=-DWOLFBOOT_HASH_SHA256
endif
ifeq ($(HASH),SHA3)
  CFLAGS+=-DWOLFBOOT_HASH_SHA3_384
endif
ifeq ($(EXT_FLASH),1)
  CFLAGS+= -DEXT_FLASH=1 -DPART_UPDATE_EXT=1 -DPART_SWAP_EXT=1
  ifeq ($(NO_XIP),1)
    CFLAGS+=-DPART_BOOT_EXT=1
  endif
endif


## Toolchain setup
CROSS_COMPILE=
CC=$(CROSS_COMPILE)gcc
LD=$(CROSS_COMPILE)gcc
AS=$(CROSS_COMPILE)gcc
OBJCOPY:=$(CROSS_COMPILE)objcopy
SIZE:=$(CROSS_COMPILE)size

all: $(APPNAME)

$(APPNAME):

$(APPNAME): include/target.h $(OBJS)
	@echo "\t[BIN] $@"
	$(Q)$(LD) $(LDFLAGS) $(OBJS) -o $@

%.o:%.c
	@echo "\t[CC-$(ARCH)] $@"
	$(Q)$(CC) $(CFLAGS) -c -o $@ $^

clean:
	@rm -rf $(APPNAME)
	@rm -f src/*.o hal/*.o
