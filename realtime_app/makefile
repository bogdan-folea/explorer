################################################################################
#                                                                              #
#    Makefile                                                                  #
#                                                                              #
################################################################################

PROJECT   := explorer
PLATFORM  := LPC1768
CPU       := ARM_CM3

ELF := $(PROJECT).elf
MAP := $(PROJECT).map
BIN := $(PROJECT).bin

# Default compiler
COMPILER := GCC

# List of supported compilers
compilers_list := GCC

# Compiler check
ifneq ($(filter $(COMPILER),$(compilers_list)),$(COMPILER))
	$(error Undefined compiler type: '$(COMPILER)', use one from: $(compilers_list))
endif

# FreeRTOS kernel path check
ifeq (,$(FREERTOS_KERNEL_PATH))
	$(error Environment variable FREERTOS_KERNEL_PATH must be set)
endif
ifneq (1,$(words [$(FREERTOS_KERNEL_PATH)]))
	$(error Spaces are not allowed in the FREERTOS_KERNEL_PATH)
endif

# Common directories containing source/header files
DIR_KERNEL   := $(FREERTOS_KERNEL_PATH)/Source
DIR_LINKER   := platform/$(PLATFORM)/core/source/linker/$(COMPILER)

################################################################################
# GCC compiler settings                                                        #
################################################################################

ifeq ($(COMPILER),GCC)

# Compiler path check
ifeq (,$(GCCARM_TOOLCHAIN_PATH))
	$(error Environment variable GCCARM_TOOLCHAIN_PATH must be set)
endif
ifneq (1,$(words [$(GCCARM_TOOLCHAIN_PATH)]))
	$(error Spaces are not allowed in the GCCARM_TOOLCHAIN_PATH)
endif

GCCARM := $(subst \,/,$(GCCARM_TOOLCHAIN_PATH))

# Compiler/Assembler/Linker paths
CC      := $(GCCARM)/bin/arm-none-eabi-gcc
AS      := $(GCCARM)/bin/arm-none-eabi-gcc
LD      := $(GCCARM)/bin/arm-none-eabi-gcc
OBJCOPY := $(GCCARM)/bin/arm-none-eabi-objcopy

# Compiler/Assembler/Linker flags
CFLAGS := -mthumb -mcpu=cortex-m3 -g3 -Os -Wall
CFLAGS += -c -fomit-frame-pointer -fshort-enums
CFLAGS += -fmessage-length=0 -fplan9-extensions
CFLAGS += -DGCC -fno-common -D$(PLATFORM)

ASFLAGS := -mthumb -mcpu=cortex-m3 -g3 -gdwarf-2
ASFLAGS += -c -x assembler-with-cpp -nostdlib

LDFLAGS := -mthumb -mcpu=cortex-m3 -g3 -gdwarf-2
LDFLAGS += -Wl,-Map,$(MAP) -specs=nosys.specs
LDFLAGS += -L $(GCCARM)/arm-none-eabi/lib/armv7-m
LDFLAGS += -T$(DIR_LINKER)/$(PLATFORM).ld

OBJCOPYFLAGS := -O binary

# GCC related includes
INCLUDES := -I$(GCCARM)/arm-none-eabi/include

endif

################################################################################
# List of the common source files                                              #
################################################################################

SRC_CORE_C := $(DIR_KERNEL)/list.c                                             \
              $(DIR_KERNEL)/queue.c                                            \
              $(DIR_KERNEL)/tasks.c                                            \
              $(DIR_KERNEL)/timers.c                                           \
              $(DIR_KERNEL)/event_groups.c

SRC_CORE_C += $(DIR_KERNEL)/portable/$(COMPILER)/$(CPU)/port.c                 \
              $(DIR_KERNEL)/portable/MemMang/heap_2.c

# Core related includes
INCLUDES += -I$(DIR_KERNEL)/include/                                           \
            -I$(DIR_KERNEL)/portable/$(COMPILER)/$(CPU)/

################################################################################
# List of the specific application files                                       #
################################################################################

SRC_APP_C := main.c

SRC_APP_C += platform/$(PLATFORM)/core/source/startup.c                        \
             platform/$(PLATFORM)/core/source/sysinit.c

# Application related includes
INCLUDES += -I. -Iplatform/$(PLATFORM)/core/include/                           \
            -Iplatform/$(PLATFORM)/core/include/$(CPU)/

################################################################################
# List of the objects files to be compiled/assembled                           #
################################################################################

OBJECTS_CORE_ASM := $(SRC_CORE_ASM:.s=.o)
OBJECTS_CORE_C   := $(SRC_CORE_C:.c=.o)
OBJECTS_CORE     := $(OBJECTS_CORE_ASM) $(OBJECTS_CORE_C)

OBJECTS_APP_ASM := $(SRC_APP_ASM:.s=.o)
OBJECTS_APP_C   := $(SRC_APP_C:.c=.o)
OBJECTS_APP     := $(OBJECTS_APP_ASM) $(OBJECTS_APP_C)

OBJECTS := $(OBJECTS_CORE) $(OBJECTS_APP)

################################################################################
# Makefile rules                                                               #
################################################################################

all: $(BIN)

.c.o:
	$(CC) $(CFLAGS) $(INCLUDES) $^ -o $@

.s.o:
	$(AS) $(ASFLAGS) $^ -o $@

core: $(OBJECTS_CORE_ASM) $(OBJECTS_CORE_C)

app: $(OBJECTS_APP_ASM) $(OBJECTS_APP_C)

$(ELF): core app
	$(LD) $(OBJECTS) $(LDFLAGS) -o $(ELF)

$(BIN): $(ELF)
	$(OBJCOPY) $(ELF) $(BIN) $(OBJCOPYFLAGS)

clean:
	rm -f $(OBJECTS) $(ELF) $(MAP) $(BIN)

.PHONY: all core app clean