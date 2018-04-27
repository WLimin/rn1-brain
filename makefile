# This makefile is made to work with the toolchain downloadable at https://launchpad.net/gcc-arm-embedded

# Build path
BUILD_DIR = build
BINPATH = /${HOME}/eclipse/arduinoPlugin/packages/STM32/tools/arm-none-eabi-gcc/6-2017-q2-update/bin
LIBDIR = $(BINPATH)/../arm-none-eabi/lib

CC = $(BINPATH)/arm-none-eabi-gcc
LD = $(BINPATH)/arm-none-eabi-gcc
SIZE = $(BINPATH)/arm-none-eabi-size
OBJCOPY = $(BINPATH)/arm-none-eabi-objcopy

TARGET = brain

#MODEL=RN1P4
#MODEL=RN1P7
#MODEL=RN1P6
#MODEL=PULU1
MODEL=PROD1

PCBREV=PCB1B

CFLAGS = -I. -Os -fno-common -ffunction-sections -ffreestanding -fno-builtin -mthumb -mcpu=cortex-m3 -Wall -fstack-usage -std=c11 -Winline -D$(MODEL) -D$(PCBREV)

#CFLAGS += -DHWTEST
#CFLAGS += -DSONARS_INSTALLED
CFLAGS += -DDELIVERY_APP
#CFLAGS += -DOPTFLOW_INSTALLED
CFLAGS += -DPULUTOF1

ASMFLAGS = -S -fverbose-asm
LDFLAGS = -mcpu=cortex-m3 -mthumb -nostartfiles -gc-sections -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref

######################################
# source
######################################
# C sources
C_SOURCES =  \
stm32init.c main.c gyro_xcel_compass.c lidar.c optflow.c motcons.c own_std.c flash.c sonar.c \
feedbacks.c sin_lut.c navig.c uart.c hwtest.c settings.c

ASM_SOURCES =

#DEPS = main.h gyro_xcel_compass.h lidar.h optflow.h motcons.h own_std.h flash.h sonar.h comm.h feedbacks.h sin_lut.h navig.h uart.h settings.h

# list of objects
OBJ = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJ += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))
# list of ASM source file transed from C program
ASMS += $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.s)))
vpath %.c $(sort $(dir $(C_SOURCES)))

all: $(BUILD_DIR)/$(TARGET).bin

$(BUILD_DIR)/%.o: %.c $(DEPS) | $(BUILD_DIR)
	$(CC) -c -o $@ $< $(CFLAGS)

$(BUILD_DIR):
	mkdir $@

clean:
	-rm -fR .dep $(BUILD_DIR)

$(BUILD_DIR)/$(TARGET).bin: $(OBJ) | $(BUILD_DIR)
#	$(LD) -Tstm32.ld $(LDFLAGS) -o $(BUILD_DIR)/$(TARGET).elf $^ /usr/arm-none-eabi/lib/thumb/v7-m/libm.a
	$(LD) -Tstm32.ld $(LDFLAGS) -o $(BUILD_DIR)/$(TARGET).elf $^ $(LIBDIR)/thumb/v7-m/libm.a
	$(OBJCOPY) -Obinary --remove-section=.ARM* $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET)_full.bin
	$(OBJCOPY) -Obinary --remove-section=.ARM* --remove-section=.flasher $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin
	$(SIZE) $(BUILD_DIR)/$(TARGET).elf

flash_full: $(BUILD_DIR)/$(TARGET).bin
	stm32sprog -b 115200 -vw  $(BUILD_DIR)/$(TARGET)_full.bin

flash: $(BUILD_DIR)/$(TARGET).bin
	stm32sprog -b 115200 -vw $(BUILD_DIR)/$(TARGET).bin

f: $(BUILD_DIR)/$(TARGET).bin
	scp $(BUILD_DIR)/$(TARGET).bin hrst@$(robot):~/rn1-tools/

ff: $(BUILD_DIR)/$(TARGET).bin
	scp $(BUILD_DIR)/$(TARGET)_full.bin hrst@$(robot):~/rn1-tools/$(BUILD_DIR)/$(TARGET).bin

f_local: $(BUILD_DIR)/$(TARGET).bin
	../rn1-tools/prog /dev/ttyUSB0 ./$(BUILD_DIR)/$(TARGET).bin h

f_proto4: $(BUILD_DIR)/$(TARGET).bin
	scp $(BUILD_DIR)/$(TARGET).bin hrst@proto4:~/rn1-tools/

f_helsinki1: $(BUILD_DIR)/$(TARGET).bin
	scp $(BUILD_DIR)/$(TARGET).bin hrst@helsinki1:~/rn1-tools/

f_proto5: $(BUILD_DIR)/$(TARGET).bin
	scp $(BUILD_DIR)/$(TARGET).bin hrst@proto5:~/rn1-tools/

f_proto6: $(BUILD_DIR)/$(TARGET).bin
	scp $(BUILD_DIR)/$(TARGET).bin hrst@proto6:~/rn1-tools/

f_pulu1: $(BUILD_DIR)/$(TARGET).bin
	scp $(BUILD_DIR)/$(TARGET).bin hrst@pulu1:~/rn1-tools/

stack: $(BUILD_DIR)
	cat $(BUILD_DIR)/*.su

sections:
	arm-none-eabi-objdump -h $(BUILD_DIR)/$(TARGET).elf

syms:
	arm-none-eabi-objdump -t $(BUILD_DIR)/$(TARGET).elf

$(BUILD_DIR)/%.s: %.c $(DEPS) | $(BUILD_DIR)
	$(CC) -c -o $@ $< $(CFLAGS) $(ASMFLAGS)

asm: $(ASMS)

e:
	gedit --new-window main.c feedbacks.h feedbacks.c navig.h navig.c lidar.h lidar.c lidar_corr.h lidar_corr.c uart.h uart.c motcons.c motcons.h sonar.c sonar.h flash.h flash.c settings.h settings.c &
s:
	screen /dev/ttyUSB0 115200
