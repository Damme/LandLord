BASE:=/cygdrive/f/arm-dev

GCCPATH:=$(BASE)/GCCarm

TARGETNAME:=$(firstword $(basename $(wildcard *.c)))
SYSINC:=common
U8GPATH:=u8g
LDSCRIPTDIR:=$(SYSINC)
SRC:=$(wildcard *.c)
MCPU:=cortex-m3

STARTUP:=$(wildcard $(SYSINC)/*.S)

SYSSRC:=$(wildcard $(LDSCRIPTDIR)/*.c)
LDSCRIPT:=$(wildcard $(LDSCRIPTDIR)/*.ld)


#================================================
# Main part of the Makefile starts here. Usually no changes are needed.

# U8G Source files
U8GSRC:=$(wildcard $(U8GPATH)/*.c)

# Internal Variable Names
ELFNAME:=$(TARGETNAME).elf
BINNAME:=$(TARGETNAME).bin
HEXNAME:=$(TARGETNAME).hex
DISNAME:=$(TARGETNAME).dis
MAPNAME:=$(TARGETNAME).map
OBJ:=$(SRC:.c=.o) $(SYSSRC:.c=.o) $(U8GSRC:.c=.o) $(STARTUP:.S=.o)

# Replace standard build tools by avr tools
CC:=$(GCCPATH)/bin/arm-none-eabi-gcc
AR:=$(GCCPATH)/bin/arm-none-eabi-ar
AS:=$(GCCPATH)/bin/arm-none-eabi-gcc
OBJCOPY:=$(GCCPATH)/bin/arm-none-eabi-objcopy
OBJDUMP:=$(GCCPATH)/bin/arm-none-eabi-objdump
SIZE:=$(GCCPATH)/bin/arm-none-eabi-size

# Common flags
COMMON_FLAGS = -mthumb -mcpu=$(MCPU)
COMMON_FLAGS += -Wall -I. -I$(SYSINC) -I$(U8GPATH)
# default stack size is 0x0c00
COMMON_FLAGS += -D__STACK_SIZE=0x0a00
COMMON_FLAGS += -Os -flto 
COMMON_FLAGS += -ffunction-sections -fdata-sections
# Assembler flags
ASFLAGS:=$(COMMON_FLAGS) -D__STARTUP_CLEAR_BSS -D__START=main
# C flags
CFLAGS:=$(COMMON_FLAGS) -std=gnu99
# LD flags
GC:=-Wl,--gc-sections
MAP:=-Wl,-Map=$(MAPNAME)
LFLAGS:=$(COMMON_FLAGS) $(GC) $(MAP)
LDLIBS:=--specs=nano.specs -lc -lc -lnosys -L$(LDSCRIPTDIR) -T $(LDSCRIPT)

# Additional Suffixes
.SUFFIXES: .elf .hex .dis .bin

# Targets
.PHONY: all
all: $(DISNAME) $(HEXNAME) $(BINNAME)
	$(SIZE) $(ELFNAME)

.PHONY: upload
upload: $(DISNAME) $(HEXNAME) $(ELFNAME)
	$(FLASHTOOL) HEXFILE\($(HEXNAME),NOCHECKSUMS,FILL,PROTECTISP\) COM\(5,38400\) DEVICE\($(FLASHMAGICDEVICE),12.000,0\)
	$(SIZE) $(ELFNAME)

.PHONY: clean
clean:
	$(RM) $(OBJ) $(HEXNAME) $(BINNAME) $(ELFNAME) $(DISNAME) $(MAPNAME)

# implicit rules
.S.o:
	$(PREPROCESS.S) $(COMMON_FLAGS) $(patsubst %.s,%.S,$<) > tmp.s
	$(COMPILE.s) -c -o $@ tmp.s
	$(RM) tmp.s

.elf.hex:
	@$(OBJCOPY) -O ihex $< $@

.elf.bin:
	@$(OBJCOPY) -O binary $< $@

# explicit rules
$(ELFNAME): $(OBJ) 
	$(LINK.o) $(LFLAGS) $(OBJ) $(LDLIBS) -o $@
#$(ELFNAME): $(LIBNAME)($(OBJ)) 
#	$(LINK.o) $(LFLAGS) $(LIBNAME) $(LDLIBS) -o $@


$(DISNAME): $(ELFNAME)
	$(OBJDUMP) -S $< > $@

	
