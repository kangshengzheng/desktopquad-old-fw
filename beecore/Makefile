###############################################################################
# Generic Makefile Template for C/C++ for use with STM32 Microcontrollers
#
# Copyright (c) 2016 - James Jackson
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the <organization> nor the
#      names of its contributors may be used to endorse or promote products
#      derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

TARGET	?= rosflight

DEBUG ?= GDB

SERIAL_DEVICE ?= /dev/ttyACM0

# Flash size (KB).  Some low-end chips actually have more flash than advertised, use this to override.
FLASH_SIZE ?= 256

#################################
# GNU ARM Embedded Toolchain
#################################
CC=arm-none-eabi-gcc
CXX=arm-none-eabi-g++
LD=arm-none-eabi-ld
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump
NM=arm-none-eabi-nm
SIZE=arm-none-eabi-size
A2L=arm-none-eabi-addr2line

#################################
# Working directories
#################################
ROOT			= $(dir $(lastword $(MAKEFILE_LIST)))
BOARD_DIR 		= .
ROSFLIGHT_DIR 	= ..
TURBOTRIG_DIR 	= ../lib/turbotrig
BEECORELITE_DIR	= lib/betaflight_beecorelite
CMSIS_DIR		= $(BEECORELITE_DIR)/lib/main/CMSIS
STDPERIPH_DIR	= $(BEECORELITE_DIR)/lib/main/STM32F30x_StdPeriph_Driver
USBFS_DIR		= $(BEECORELITE_DIR)/lib/main/STM32_USB-FS-Device_Driver
DRIVERS_DIR		= $(BEECORELITE_DIR)/src/main/drivers
TARGET_DIR		= $(BEECORELITE_DIR)/src/main/target
VCP_DIR			= $(BEECORELITE_DIR)/src/main/vcp
STARTUP_DIR		= $(BEECORELITE_DIR)/src/main/startup
LINKER_DIR		= $(BEECORELITE_DIR)/src/main/target/link
OBJECT_DIR		= $(ROOT)/build
BIN_DIR			= $(ROOT)/build


#################################
# Source Files
#################################

# board-specific source files
VPATH		:= $(BOARD_DIR)
BOARD_SRC =	main.c \
			board.c

# ROSflight source files
VPATH		:= $(VPATH):$(ROSFLIGHT_DIR)
ROSFLIGHT_SRC =	rosflight.c \
				controller.c \
				estimator.c \
				mavlink.c \
				mavlink_param.c \
				mavlink_receive.c \
				mavlink_stream.c \
				mavlink_util.c \
				mode.c \
				mux.c \
				mixer.c \
				param.c \
				printf.c \
				rc.c \
				sensors.c

# Math Source Files
VPATH :=	$(VPATH):$(TURBOTRIG_DIR)
MATH_SRC =	turbotrig.c \
			turbovec.c


# Hardware Driver Source Files
VPATH :=	$(VPATH):$(DRIVERS_DIR)
DRIVERSLIB_SRC = system.c \
				 system_stm32f30x.c \
				 gpio_stm32f30x.c \
				 serial.c \
				 serial_uart.c \
				 serial_uart_stm32f30x.c \
				 io.c \
				 rcc.c \
				 light_led.c \
				 accgyro_spi_mpu6500.c \
				 serial_usb_vcp.c \
				 usb_io.c

# Target hardware source files
VPATH :=	$(VPATH):$(TARGET_DIR)
TARGET_SRC = system_stm32f30x.c

# Microcontroller ASM startup source
VPATH :=	$(VPATH):$(STARTUP_DIR)
STARTUP_SRC = startup_stm32f30x_md_gcc.S

# Search path and source files for the CMSIS sources
VPATH		:= $(VPATH):$(CMSIS_DIR)/CM1/CoreSupport:$(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x
CMSIS_SRC	 = $(notdir $(wildcard $(CMSIS_DIR)/CM1/CoreSupport/*.c \
				       $(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x/*.c))

# Search path and source files for the ST stdperiph library
VPATH			:= $(VPATH):$(STDPERIPH_DIR)/src
STDPERIPH_SRC	 = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))

# Search path and source files for the ST USB FS library
VPATH		:= $(VPATH):$(USBFS_DIR)/src
USBFS_SRC	 = $(notdir $(wildcard $(USBFS_DIR)/src/*.c))

# Search path and source files for the VCP library
VPATH		:= $(VPATH):$(VCP_DIR)/src
VCP_SRC	 	 = $(notdir $(wildcard $(VCP_DIR)/*.c))

# Linker script for stm32f3
LD_SCRIPT	 = $(LINKER_DIR)/stm32_flash_f303_$(FLASH_SIZE)k.ld

# Compile a list of C source files
CSOURCES =  $(CMSIS_SRC) \
			$(STDPERIPH_SRC) \
			$(USBFS_SRC) \
			$(VCP_SRC) \
			$(addprefix $(ROSFLIGHT_DIR)/src/, $(ROSFLIGHT_SRC)) \
			$(addprefix $(DRIVERS_DIR)/, $(DRIVERSLIB_SRC)) \
			$(addprefix $(TARGET_DIR)/, $(TARGET_SRC)) \
			$(addprefix $(BOARD_DIR)/, $(BOARD_SRC)) \
			$(addprefix $(STARTUP_DIR)/, $(STARTUP_SRC)) \
			$(addprefix $(TURBOTRIG_DIR)/, $(MATH_SRC)) 

# Compile a list of C++ Source Files
CXXSOURCES = 

# Set up Include Directoreis
INCLUDE_DIRS =	$(BOARD_DIR) \
				$(BEECORELITE_DIR)/src/main \
				$(DRIVERS_DIR) \
				$(TARGET_DIR) \
				$(VCP_DIR) \
				$(TURBOTRIG_DIR) \
				$(ROSFLIGHT_DIR)/include \
				$(ROSFLIGHT_DIR)/lib \
				$(STDPERIPH_DIR)/inc \
				$(USBFS_DIR)/inc \
				$(CMSIS_DIR)/CM1/CoreSupport \
				$(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x


#################################
# Object List
#################################
OBJECTS=$(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $(ASOURCES))))
OBJECTS+=$(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $(CSOURCES))))
OBJECTS+=$(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $(CXXSOURCES))))

#$(info $$OBJECTS is [${OBJECTS}])

#################################
# Target Output Files
#################################
TARGET_ELF=$(BIN_DIR)/$(TARGET).elf
TARGET_HEX=$(BIN_DIR)/$(TARGET).hex

#################################
# Debug Config
#################################
ifeq ($(DEBUG), GDB)
DEBUG_FLAGS = -ggdb
OPTIMIZE = -Og
$(info ***** Building with Debug Symbols *****)
else
OPTIMIZE = -Os
LTO_FLAGS = -flto -fuse-linker-plugin $(OPTIMIZE)
endif


#################################
# Flags
#################################
MCFLAGS=-mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion
# DEFS=-DTARGET_STM32F10X_MD -D__CORTEX_M4 -D__FPU_PRESENT -DWORDS_STACK_SIZE=200 -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER
DEFS=-DSTM32F303xC -DSTM32F303 -DWORDS_STACK_SIZE=200 -DUSE_STDPERIPH_DRIVER
CFLAGS=-c $(MCFLAGS) $(DEFS) $(OPTIMIZE) $(DEBUG_FLAGS) $(addprefix -I,$(INCLUDE_DIRS)) -std=gnu99
CXXFLAGS=-c $(MCFLAGS) $(DEFS) $(OPTIMIZE) $(DEBUG_FLAGS) $(addprefix -I,$(INCLUDE_DIRS)) -std=c++11
CXXFLAGS+=-U__STRICT_ANSI__
LDFLAGS =-T$(LD_SCRIPT) $(MCFLAGS) -Wl,-L$(LINKER_DIR) -lm -nostartfiles -lc --specs=rdimon.specs $(ARCH_FLAGS)  $(LTO_FLAGS)  $(DEBUG_FLAGS) -static  -Wl,-gc-sections

#################################
# Build
#################################
$(TARGET_HEX): $(TARGET_ELF)
	$(CP) -O ihex --set-start 0x8000000 $< $@

$(TARGET_ELF): $(OBJECTS)
	$(CXX) -o $@ $^ $(LDFLAGS)
	$(SIZE) $(TARGET_ELF)

$(OBJECT_DIR)/$(TARGET)/%.o: %.cpp
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CXX) -c -o $@ $(CXXFLAGS) $<

$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<

$(OBJECT_DIR)/$(TARGET)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<


#################################
# Recipes
#################################
.PHONY: all flash clean

clean:
	rm -f $(OBJECTS) $(TARGET_ELF) $(TARGET_HEX) $(BIN_DIR)/output.map

flash: $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 921600 -crtscts cs8 -parenb -cstopb -ixon
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 921600 $(SERIAL_DEVICE)

# Print a Makefile variable (e.g., `make print-PROJECT`)
print-%:
	@echo '$*=$($*)'