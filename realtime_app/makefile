################################################################################
#                                                                              #
#   Proiect de Licenta                                                         #
#   Bogdan Folea, 2017                                                         #
#                                                                              #
#   Makefile                                                                   #
#                                                                              #
################################################################################

PROJECT   := explorer
PLATFORM  := LPC1768
CPU       := ARM_CM3

EXE := $(PROJECT).elf
MAP := $(PROJECT).map
BIN := $(PROJECT).bin
LST := $(PROJECT).lst

# Default compiler
compiler := GCC

# List of supported compilers
compilers_list := GCC IAR

# Compiler check
ifneq ($(filter $(compiler),$(compilers_list)),$(compiler))
$(error Undefined compiler type: '$(compiler)', use one from: $(compilers_list))
endif

# FreeRTOS kernel path check
ifeq (,$(FREERTOS_KERNEL_PATH))
$(error Environment variable FREERTOS_KERNEL_PATH must be set)
endif

# Common directories containing source/header files
DIR_KERNEL      := $(FREERTOS_KERNEL_PATH)/Source
DIR_LINKER      := platform/$(PLATFORM)/core/source/linker/$(compiler)

################################################################################
# GCC compiler settings                                                        #
################################################################################

ifeq ($(compiler),GCC)

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
OBJDUMP := $(GCCARM)/bin/arm-none-eabi-objdump

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
#--entry __startup

OBJCOPYFLAGS := -O binary

OBJDUMPFLAGS := -d -S

# GCC related includes
INCLUDES := -I$(GCARM)/arm-none-eabi/include

endif

################################################################################
# List of the common source files                                              #
################################################################################

SRC_CORE_C := $(DIR_KERNEL)/list.c                                             \
              $(DIR_KERNEL)/queue.c                                            \
              $(DIR_KERNEL)/tasks.c                                            \
              $(DIR_KERNEL)/timers.c                                           \
              $(DIR_KERNEL)/event_groups.c

SRC_CORE_C += $(DIR_KERNEL)/portable/$(compiler)/$(CPU)/port.c                 \
              $(DIR_KERNEL)/portable/MemMang/heap_2.c

# Core related includes
INCLUDES += -I$(DIR_KERNEL)/include/                                           \
            -I$(DIR_KERNEL)/portable/$(compiler)/$(CPU)/

################################################################################
# List of the specific application files                                       #
################################################################################

SRC_APP_C := main.c

SRC_APP_C += platform/$(PLATFORM)/core/source/startup.c                        \
             platform/$(PLATFORM)/core/source/sysinit.c                        \
			 platform/$(PLATFORM)/drivers/source/lpc17xx_uart.c                \
			 platform/$(PLATFORM)/drivers/source/lpc17xx_clkpwr.c              \
			 platform/$(PLATFORM)/drivers/source/lpc17xx_pinsel.c              \
			 platform/$(PLATFORM)/drivers/source/lpc17xx_gpio.c                \
			 platform/$(PLATFORM)/drivers/source/lpc17xx_pwm.c                 \
			 platform/$(PLATFORM)/drivers/source/lpc17xx_adc.c                 \
			 platform/$(PLATFORM)/drivers/source/lpc17xx_i2c.c

# Application related includes
INCLUDES += -I.                                                                \
            -Iinclude/                                                         \
			-Iplatform/$(PLATFORM)/core/include/                               \
			-Iplatform/$(PLATFORM)/core/include/$(CPU)/                        \
			-Iplatform/$(PLATFORM)/drivers/config/                             \
			-Iplatform/$(PLATFORM)/drivers/include/

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

all: $(BIN) $(LST)

.c.o:
	$(CC) $(CFLAGS) $(INCLUDES) $^ -o $@

.s.o:
	$(AS) $(ASFLAGS) $^ -o $@

core: $(OBJECTS_CORE_ASM) $(OBJECTS_CORE_C)

app: $(OBJECTS_APP_ASM) $(OBJECTS_APP_C)

$(EXE): core app
	$(LD) $(OBJECTS) $(LDFLAGS) -o $(EXE)

$(BIN): $(EXE)
	$(OBJCOPY) $(EXE) $(OBJCOPYFLAGS) $(BIN)

$(LST): $(EXE)
	$(OBJDUMP) $(EXE) $(OBJDUMPFLAGS) > $(LST)

clean:
	rm -f $(OBJECTS) $(EXE) $(MAP) $(BIN) $(LST)

.PHONY: core app clean
