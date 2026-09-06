#  Project Name
PROJECT=MRMBootloader

# libs dir
LIBDIR=lib

# build output dir - everything generated goes here
BUILDDIR=build

# STM32 stdperiph lib defines
CDEFS=-DHSE_VALUE=8000000 -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER

#  List of the source files to be compiled/assembled (paths, not .o names)
SOURCES=main.c can.c bl_startup_stm32f10x_md.s

CMSIS_SOURCES=\
$(LIBDIR)/cmsis/system_stm32f10x.c

STM_SOURCES=\
$(LIBDIR)/stm32f10x/src/stm32f10x_can.c \
$(LIBDIR)/stm32f10x/src/stm32f10x_crc.c \
$(LIBDIR)/stm32f10x/src/stm32f10x_flash.c \
$(LIBDIR)/stm32f10x/src/stm32f10x_gpio.c \
$(LIBDIR)/stm32f10x/src/stm32f10x_iwdg.c \
$(LIBDIR)/stm32f10x/src/stm32f10x_rcc.c

SOURCES+=$(CMSIS_SOURCES)
SOURCES+=$(STM_SOURCES)

# Mirror each source path (.c/.s) into $(BUILDDIR)/<same path>.o
OBJECTS=$(addprefix $(BUILDDIR)/,$(addsuffix .o,$(basename $(SOURCES))))

LSCRIPT=$(LIBDIR)/cmsis/stm32f103x8.ld

OPTIMIZATION = s
DEBUG = dwarf-2
#LISTING += -Wa,-adhlns=$(<:%.c=%.lst)

#  Compiler Options
GCFLAGS = -g$(DEBUG)
GCFLAGS += $(CDEFS)
GCFLAGS += -O$(OPTIMIZATION)
GCFLAGS += -Wall -std=gnu99 -fno-common -mcpu=cortex-m3 -mthumb -ffunction-sections -fdata-sections
GCFLAGS += -fno-unwind-tables -fno-asynchronous-unwind-tables
GCFLAGS += -I$(LIBDIR)/stm32f10x/inc -I$(LIBDIR)/cmsis
#GCFLAGS += -Wcast-align -Wcast-qual -Wimplicit -Wpointer-arith -Wswitch
#GCFLAGS += -Wredundant-decls -Wreturn-type -Wshadow -Wunused
LDFLAGS = -mcpu=cortex-m3 -mthumb -O$(OPTIMIZATION) -Wl,-Map=$(BUILDDIR)/$(PROJECT).map -T$(LSCRIPT) -Wl,--gc-sections
ASFLAGS = $(LISTING) -mcpu=cortex-m3

#  Compiler/Assembler/Linker Paths
GCC = arm-none-eabi-gcc
AS = arm-none-eabi-as
LD = arm-none-eabi-ld
OBJCOPY = arm-none-eabi-objcopy
ifeq ($(OS), Windows_NT)
REMOVE = rm.py -f
REMOVE_DIR = rm.py -rf
else
REMOVE = rm -f
REMOVE_DIR = rm -rf
endif
SIZE = arm-none-eabi-size

#########################################################################

all:: $(BUILDDIR)/$(PROJECT).hex $(BUILDDIR)/$(PROJECT).bin stats

$(BUILDDIR)/$(PROJECT).bin: $(BUILDDIR)/$(PROJECT).elf
#	$(OBJCOPY) -O binary -j .text -j .data $< $@
	$(OBJCOPY) -R .stack -O binary $< $@

$(BUILDDIR)/$(PROJECT).hex: $(BUILDDIR)/$(PROJECT).elf
	$(OBJCOPY) -R .stack -O ihex $< $@

$(BUILDDIR)/$(PROJECT).elf: $(OBJECTS)
	$(GCC) $(LDFLAGS) $(OBJECTS) -o $@

stats: $(BUILDDIR)/$(PROJECT).elf
	$(SIZE) $<

clean:
	$(REMOVE_DIR) $(BUILDDIR)

program: $(BUILDDIR)/$(PROJECT).bin
	st-flash write $< 0x08000000

#########################################################################
#  Pattern rules to compile .c and .cpp files, and assemble .s files,
#  into $(BUILDDIR), mirroring the source directory structure.

$(BUILDDIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(GCC) $(GCFLAGS) -c $< -o $@

$(BUILDDIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(GCC) $(GCFLAGS) -c $< -o $@

$(BUILDDIR)/%.o: %.s
	@mkdir -p $(dir $@)
	$(AS) $(ASFLAGS) -o $@ $<

#########################################################################
-include $(shell mkdir -p .dep) $(wildcard .dep/*)