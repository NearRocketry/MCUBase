#****************************************************************************************
#|  Description: Makefile for GNU ARM Embedded toolchain.
#|    File Name: makefile
#|
#|---------------------------------------------------------------------------------------
#|                          C O P Y R I G H T
#|---------------------------------------------------------------------------------------
#|   Copyright (c) 2017  by Feaser    http://www.feaser.com    All rights reserved
#|
#|---------------------------------------------------------------------------------------
#|                            L I C E N S E
#|---------------------------------------------------------------------------------------
#| This file is part of OpenBLT. OpenBLT is free software: you can redistribute it and/or
#| modify it under the terms of the GNU General Public License as published by the Free
#| Software Foundation, either version 3 of the License, or (at your option) any later
#| version.
#|
#| OpenBLT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
#| without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
#| PURPOSE. See the GNU General Public License for more details.
#|
#| You have received a copy of the GNU General Public License along with OpenBLT. It 
#| should be located in ".\Doc\license.html". If not, contact Feaser to obtain a copy.
#|
#****************************************************************************************
SHELL = sh

#|--------------------------------------------------------------------------------------|
#| Configure project name                                                               |
#|--------------------------------------------------------------------------------------|
PROJ_NAME=near-test


#|--------------------------------------------------------------------------------------|
#| Configure tool path                                                                  |
#|--------------------------------------------------------------------------------------|
# Configure the path to where the arm-none-eabi-gcc program is located. If the program
# is available on the path, then the TOOL_PATH variable can be left empty.
# Make sure to add a fordward slash at the end. Note that on Windows it should be in the
# 8.3 short pathname format with forward slashes. To obtain the pathname in the 8.3
# format, open the directory in the Windows command prompt and run the following command:
#  cmd /c for %A in ("%cd%") do @echo %~sA 
#TOOL_PATH=/opt/gcc-arm-none-eabi-8-2018-q4-major/bin/
TOOL_PATH=


#|--------------------------------------------------------------------------------------|
#| Configure device type                                                                |
#|--------------------------------------------------------------------------------------|
# Configure the device type. This is used to select the correct startup code file and 
# linker script file. Startup script templates are defined in the CMSIS-Device project at 
# the following location: lib/CMSIS-Device/Source/Templates/gcc. s.
PART=STM32F072xB
STARTUP_SCRIPT=startup_stm32f072xb.s

#|--------------------------------------------------------------------------------------|
#| Collect project files                                                                |
#|--------------------------------------------------------------------------------------|
# Recursive wildcard function implementation. Example usages:
#   $(call rwildcard, , *.c *.h)   
#     --> Returns all *.c and *.h files in the current directory and below
#   $(call rwildcard, /lib/, *.c)
#     --> Returns all *.c files in the /lib directory and below
rwildcard = $(strip $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2) $(filter $(subst *,%,$2),$d)))

# Collect all application files in the current directory and its subdirectories
PROJ_FILES := $(call rwildcard, lib/newlib/, *.c) \
			  $(call rwildcard, lib/CMSIS-Device/Source/Templates/gcc/$(STARTUP_SCRIPT), *.S) \
			  $(call rwildcard, lib/CMSIS/Include/,*.h) \
			  $(filter-out system_stm32f0xx.c, $(call rwildcard, lib/CMSIS-Device/, *.S *.c *.h)) \
			  $(filter-out $(call rwildcard, lib/STM32F0xx_HAL_Driver/, *_template.c), $(call rwildcard, lib/STM32F0xx_HAL_Driver/, *.S *.c *.h)) \
			  $(call rwildcard, inc/, *.S *.c *.h) \
			  $(call rwildcard, src/, *.S *.c *.h)

#|--------------------------------------------------------------------------------------|
#| Toolchain binaries                                                                   |
#|--------------------------------------------------------------------------------------|
RM = rm
CC = $(TOOL_PATH)arm-none-eabi-gcc
LN = $(TOOL_PATH)arm-none-eabi-gcc
OC = $(TOOL_PATH)arm-none-eabi-objcopy
OD = $(TOOL_PATH)arm-none-eabi-objdump
AS = $(TOOL_PATH)arm-none-eabi-gcc
SZ = $(TOOL_PATH)arm-none-eabi-size


#|--------------------------------------------------------------------------------------|
#| Filter project files
#|--------------------------------------------------------------------------------------|
PROJ_ASRCS  = $(filter %.S,$(foreach file,$(PROJ_FILES),$(notdir $(file))))
PROJ_CSRCS  = $(filter %.c,$(foreach file,$(PROJ_FILES),$(notdir $(file))))


#|--------------------------------------------------------------------------------------|
#| Set important path variables                                                         |
#|--------------------------------------------------------------------------------------|
VPATH    = $(foreach path,$(sort $(foreach file,$(PROJ_FILES),$(dir $(file)))) $(subst \,/,$(OBJ_PATH)),$(path) :)
OBJ_PATH = obj
BIN_PATH = bin
INC_PATH = $(patsubst %/,%,$(patsubst %,-I%,$(sort $(foreach file,$(filter %.h,$(PROJ_FILES)),$(dir $(file))))))
LIB_PATH  = 

#|--------------------------------------------------------------------------------------|
#| Options for toolchain binaries                                                       |
#|--------------------------------------------------------------------------------------|
HEAP_SIZE   = 0x0000
STACK_SIZE  = 0x0100
STDFLAGS    = -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -fno-strict-aliasing
STDFLAGS   += -fdata-sections -ffunction-sections -Wall -g3
OPTFLAGS    = -Oz
DEPFLAGS  = -MT $@ -MMD -MP -MF $(OBJ_PATH)/$*.d
CFLAGS      = $(STDFLAGS) $(OPTFLAGS)
CFLAGS     += -D$(PART) -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER
CFLAGS     += -D__HEAP_SIZE=$(HEAP_SIZE) -D__STACK_SIZE=$(STACK_SIZE)
CFLAGS     += $(INC_PATH)
AFLAGS      = $(CFLAGS)
LFLAGS      = $(STDFLAGS) $(OPTFLAGS)
LFLAGS     += -Wl,--defsym=__HEAP_SIZE=$(HEAP_SIZE) -Wl,--defsym=__STACK_SIZE=$(STACK_SIZE)
LFLAGS     += -T"stm32f072c8_flash.ld" -Wl,-Map=$(BIN_PATH)/$(PROJ_NAME).map
LFLAGS     += -specs=nano.specs -Wl,--gc-sections $(LIB_PATH)
OFLAGS      = -O srec
ODFLAGS     = -x
SZFLAGS     = -B -d
RMFLAGS     = -f


#|--------------------------------------------------------------------------------------|
#| Specify library files                                                                |
#|--------------------------------------------------------------------------------------|
LIBS = 


#|--------------------------------------------------------------------------------------|
#| Define targets                                                                       |
#|--------------------------------------------------------------------------------------|
AOBJS = $(patsubst %.S,%.o,$(PROJ_ASRCS))
COBJS = $(patsubst %.c,%.o,$(PROJ_CSRCS))


#|--------------------------------------------------------------------------------------|
#| Make ALL                                                                             |
#|--------------------------------------------------------------------------------------|
.PHONY: all
all: $(BIN_PATH)/$(PROJ_NAME).srec 

$(BIN_PATH)/$(PROJ_NAME).srec : $(BIN_PATH)/$(PROJ_NAME).elf
	@$(OC) $< $(OFLAGS) $@
	@$(OD) $(ODFLAGS) $< > $(BIN_PATH)/$(PROJ_NAME).map
	@echo +++ Summary of memory consumption:
	@$(SZ) $(SZFLAGS) $<
	@echo +++ Build complete [$(notdir $@)]

$(BIN_PATH)/$(PROJ_NAME).elf : $(AOBJS) $(COBJS) 
	@echo $(PROJ_FILES)
	@echo +++ Linking [$(notdir $@)]
	@$(LN) $(LFLAGS) -o $@ $(patsubst %.o,$(OBJ_PATH)/%.o,$(^F)) $(LIBS)

# Create the object / binary directory if it does not exist yet
check:
	[ -d $(OBJ_PATH) ] || mkdir -p $(OBJ_PATH)
	[ -d $(BIN_PATH) ] || mkdir -p $(BIN_PATH)

#|--------------------------------------------------------------------------------------|
#| Compile and assemble                                                                 |
#|--------------------------------------------------------------------------------------|
$(AOBJS): %.o: %.S
	@echo +++ Assembling [$(notdir $<)]
	@$(AS) $(AFLAGS) -c $< -o $(OBJ_PATH)/$(@F)

$(COBJS): %.o: %.c $(OBJ_PATH)/%.d
	@echo +++ Compiling [$(notdir $<)]
	@$(CC) $(DEPFLAGS) $(CFLAGS) -c $< -o $(OBJ_PATH)/$(@F)


#|--------------------------------------------------------------------------------------|
#| Make CLEAN                                                                           |
#|--------------------------------------------------------------------------------------|
.PHONY: clean
clean: 
	[ -d $(OBJ_PATH) ] || mkdir -p $(OBJ_PATH)
	@echo +++ Cleaning build environment
	@$(RM) $(RMFLAGS) $(foreach file,$(AOBJS),$(OBJ_PATH)/$(file))
	@$(RM) $(RMFLAGS) $(foreach file,$(COBJS),$(OBJ_PATH)/$(file))
	@$(RM) $(RMFLAGS) $(patsubst %.o,%.lst,$(foreach file,$(AOBJS),$(OBJ_PATH)/$(file)))
	@$(RM) $(RMFLAGS) $(patsubst %.o,%.lst,$(foreach file,$(COBJS),$(OBJ_PATH)/$(file)))
	@$(RM) $(RMFLAGS) $(patsubst %.o,%.d,$(foreach file,$(COBJS),$(OBJ_PATH)/$(file)))
	@$(RM) $(RMFLAGS) $(BIN_PATH)/$(PROJ_NAME).elf $(BIN_PATH)/$(PROJ_NAME).map
	@$(RM) $(RMFLAGS) $(BIN_PATH)/$(PROJ_NAME).srec
	@echo +++ Clean complete

	
#|--------------------------------------------------------------------------------------|
#| Dependency generation                                                                |
#|--------------------------------------------------------------------------------------|
DEPFILES := $(PROJ_CSRCS:%.c=$(OBJ_PATH)/%.d)
$(DEPFILES):
include $(wildcard $(DEPFILES))	

