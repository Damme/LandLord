BASE:=/usr

GCCPATH:=$(BASE)

#TARGETNAME:=$(firstword $(basename $(wildcard *.c)))
TARGETNAME:=main
SYSINC:=common
U8GPATH:=u8g
FREERTOSPATH=FreeRTOS
FREERTOSGCCPATH=FreeRTOS/gcc
LDSCRIPTDIR:=$(SYSINC)
SRC:=$(wildcard *.c)
MCPU:=cortex-m3

STARTUP:=$(wildcard $(SYSINC)/gcc*.S)

SYSSRC:=$(wildcard $(LDSCRIPTDIR)/*.c)
LDSCRIPT:=$(wildcard $(LDSCRIPTDIR)/*.ld)

#================================================
# Main part of the Makefile starts here. Usually no changes are needed.

# U8G Source files
U8GSRC:=$(wildcard $(U8GPATH)/*.c)
FREETROSSRC:=$(wildcard $(FREERTOSPATH)/*.c)
FREETROSGCCSRC:=$(wildcard $(FREERTOSGCCPATH)/*.c)
# Internal Variable Names
ELFNAME:=$(TARGETNAME).elf
BINNAME:=$(TARGETNAME).bin
HEXNAME:=$(TARGETNAME).hex
DISNAME:=$(TARGETNAME).dis
MAPNAME:=$(TARGETNAME).map
OBJ:=$(SRC:.c=.o) $(SYSSRC:.c=.o) $(FREETROSSRC:.c=.o) $(FREETROSGCCSRC:.c=.o) $(U8GSRC:.c=.o) $(STARTUP:.S=.o)
OBJSMALL:=$(SRC:.c=.o) $(SYSSRC:.c=.o) $(STARTUP:.S=.o)

# Replace standard build tools by avr tools
CC:=$(GCCPATH)/bin/arm-none-eabi-gcc
AR:=$(GCCPATH)/bin/arm-none-eabi-ar
AS:=$(GCCPATH)/bin/arm-none-eabi-gcc
OBJCOPY:=$(GCCPATH)/bin/arm-none-eabi-objcopy
OBJDUMP:=$(GCCPATH)/bin/arm-none-eabi-objdump
SIZE:=$(GCCPATH)/bin/arm-none-eabi-size

# Common flags
COMMON_FLAGS = -mthumb -mcpu=$(MCPU)
COMMON_FLAGS += -g
COMMON_FLAGS += -Wall -I. -I$(SYSINC) -I$(U8GPATH) -I$(FREERTOSPATH) -I$(FREERTOSGCCPATH)
# default stack size is 0x0c00
COMMON_FLAGS += -D__STACK_SIZE=0x0a00 -DdebugPrintf -DLOWSTACKWARNING
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
	#without debug
LDLIBS:=--specs=nano.specs -lc -lc -lnosys -L$(LDSCRIPTDIR) -T $(LDSCRIPT)
	#With debug stdout -> openocd
#LDLIBS:=--specs=rdimon.specs -lc -lrdimon -L$(LDSCRIPTDIR) -T $(LDSCRIPT)


# Additional Suffixes
.SUFFIXES: .elf .hex .dis .bin

# Targets
.PHONY: all
all: $(DISNAME) $(HEXNAME) $(BINNAME)
	@echo -e "Output: $(DISNAME) $(HEXNAME) $(BINNAME)\e[1;36m"
	@echo
	@$(SIZE) --format=SysV -x $(ELFNAME)
	@$(SIZE) $(ELFNAME)
	@echo
	@stat main.bin -c "%y %n Size: %s"
	@echo -e "\e[0m"
#	cp $(BINNAME) /cygdrive/c/Prog/Dev/openOCD/share/openocd/scripts/main.bin

.PHONY: upload
upload: $(DISNAME) $(HEXNAME) $(ELFNAME)
#	$(FLASHTOOL) HEXFILE\($(HEXNAME),NOCHECKSUMS,FILL,PROTECTISP\) COM\(5,38400\) DEVICE\($(FLASHMAGICDEVICE),12.000,0\)
#	$(SIZE) $(ELFNAME)

.PHONY: clean
clean:
	@echo -e "\e[1;37mRemoving all files...\e[0m\e[1;37m"
	@$(RM) $(OBJ) $(HEXNAME) $(BINNAME) $(ELFNAME) $(DISNAME) $(MAPNAME)

cleansmall:
	@echo -e "\e[1;37mRemoving some files...\e[0m\e[1;37m"
	@$(RM) $(OBJSMALL) $(HEXNAME) $(BINNAME) $(ELFNAME) $(DISNAME) $(MAPNAME)

# implicit rules
%.o: %.c
	@echo -e "\e[1;37mCC \e[0m$< > $@\e[1;33m"
	@$(CC) $(CFLAGS) -c -o $@ $< 2>&1
# | sed -e 's/error/\\e[1;31merror\\e[0m/g' -e 's/warning/\\e[1;33mwarning\\e[0m/g'

.S.o:
	@echo -e "\e[1;37mASM \e[0m$< > $@\e[1;37m"
	@$(PREPROCESS.S) $(COMMON_FLAGS) $(patsubst %.s,%.S,$<) > tmp.s
	@$(COMPILE.s) -c -o $@ tmp.s
	@$(RM) tmp.s

.elf.hex:
	@$(OBJCOPY) -O ihex $< $@

.elf.bin:
	@$(OBJCOPY) -O binary $< $@

# explicit rules
$(ELFNAME): $(OBJ)
	@echo -e "\e[1;37mLINKING \e[0m*.o > $@\e[1;37m"
	@$(LINK.o) $(LFLAGS) $(OBJ) $(LDLIBS) -o $@

$(DISNAME): $(ELFNAME)
	@$(OBJDUMP) -S $< > $@

