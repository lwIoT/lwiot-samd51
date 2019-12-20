#
# Test Makefile
#
# @author Michel Megens
# @email  michel@michelmegens.net
#

PROJECT_SOURCE_DIR := $(PWD)/../..

CC   := arm-none-eabi-gcc
CXX  := arm-none-eabi-g++
COPY := arm-none-eabi-objcopy
DUMP := arm-none-eabi-objdump

INC_DIRS = \
	-I"$(PROJECT_SOURCE_DIR)/samd51/CMSIS/Core/Include" \
	-I"$(PROJECT_SOURCE_DIR)/samd51/hal/include" \
	-I"$(PROJECT_SOURCE_DIR)/samd51/hal/utils/include" \
	-I"$(PROJECT_SOURCE_DIR)/samd51/hpl/core" \
	-I"$(PROJECT_SOURCE_DIR)/samd51/hpl/pm" \
	-I"$(PROJECT_SOURCE_DIR)/samd51/hpl/port" \
	-I"$(PROJECT_SOURCE_DIR)/samd51/hri" \
	-I"$(PROJECT_SOURCE_DIR)/samd51/samd51a/include" \
	-I"$(PROJECT_SOURCE_DIR)/samd51/samd51a/config" \
	-I"$(PROJECT_SOURCE_DIR)/samd51/include" \
	-I"$(PROJECT_SOURCE_DIR)/lwiot-core/include" \
	-I"$(PROJECT_SOURCE_DIR)/include" \
	-I"$(PROJECT_SOURCE_DIR)/build/debug"

C_FLAGS := -mthumb  -DF_CPU=120000000 -DCONF_CPU_FREQUENCY=120000000 -Os -ffunction-sections -mlong-calls -Wall -c -std=gnu11 -D__SAMD51J19A__ -mcpu=cortex-m4 -mfloat-abi=softfp -mfpu=fpv4-sp-d16
C_FLAGS += $(INC_DIRS)

OBJS := build/main.o build/driver_init.o build/read_write.o build/usb_start.o
OBJS_AS_ARGS = \
"build/main.o" \
"build/driver_init.o" \
"build/read_write.o" \
"build/usb_start.o"

all: build $(OBJS)
	@$(CC) $(C_FLAGS) -o build/startup_samd51.o $(PROJECT_SOURCE_DIR)/samd51/samd51a/gcc/gcc/startup_samd51.c
	@echo Linking build/application.elf
	@$(CC) $(OBJS_AS_ARGS) -o build/application.elf -Wl,--start-group -lc -lm -Wl,--end-group -mthumb \
		--specs=nano.specs -Wl,--gc-sections -mcpu=cortex-m4 \
		-L$(PROJECT_SOURCE_DIR)/samd51/samd51a/gcc/gcc \
		-T$(PROJECT_SOURCE_DIR)/samd51/samd51a/gcc/gcc/samd51j19a_flash.ld \
		-L$(PROJECT_SOURCE_DIR)/build/debug/samd51 -L$(PROJECT_SOURCE_DIR)/build/debug/lwiot-core/source \
		-Wl,--whole-archive -llwiot-system -Wl,--no-whole-archive -llwiot -llwiot-platform
	@echo Creating application.bin
	@$(COPY) -O binary build/application.elf build/application.bin
	@echo Creating application.hex
	@echo Creating application.eep
	@$(COPY) -O ihex -R .eeprom -R .fuse -R .lock -R .signature build/application.elf build/application.hex
	@$(COPY) -j .eeprom --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0 --no-change-warnings -O binary build/application.elf build/application.eep
	@$(DUMP) -h -S build/application.elf > build/application.lss
	

build:
	@mkdir build
	
clean:
	@rm -rf build/*.o
	@rm -f build/application.hex
	@rm -f build/application.elf
	@rm -f build/application.bin
	@rm -f build/application.eep

build/%.o: %.c
	@echo Building file: $<
	@$(CC) $(C_FLAGS) -o $@ $<
