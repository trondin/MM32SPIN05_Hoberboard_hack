
CC = arm-none-eabi-gcc
OC = arm-none-eabi-objcopy
OS = arm-none-eabi-size
LD = arm-none-eabi-gcc

LD_SCRIPT = ./mm32spin05.ld

SRC_DIR = src
SRC = $(wildcard $(SRC_DIR)/*.c)

ASM_DIR = src
ASM  = $(wildcard $(SRC_DIR)/*.s)

preOBJ  = $(SRC:%.c=%.o)
preOBJ += $(ASM:%.s=%.o)
OBJ	 = $(preOBJ:$(SRC_DIR)/%=%)

INC  = ./lib/CMSIS

CFLAGS += -mcpu=cortex-m0 
CFLAGS += -mlittle-endian
CFLAGS += -mthumb
CFLAGS += -g

LDFLAGS += -mcpu=cortex-m0
LDFLAGS += -mlittle-endian
LDFLAGS += -mthumb
LDFLAGS += -T $(LD_SCRIPT)
LDFLAGS += -Wl,--gc-section


all: 
	@echo "   COMPILE C:"
	$(CC) $(CFLAGS) -c $(SRC) -I $(INC)

	@echo "   COMPILE asm:"
	$(CC) $(CFLAGS) -c $(ASM) 

	@echo "   LINK:"
	$(LD) $(LDFLAGS) $(OBJ) -o firmware.elf
	@echo "   Size:"	
	$(OS) ./firmware.elf

	@echo "   Convert to HEX:"
	$(OC) -Oihex ./firmware.elf ./firmware.hex 

	@echo "   Clearning..,"
	-rm *.o

	@echo "   Done!"

.PHONY: clean
clean :
	-rm *.o





