# Author: M Pieklo
# Date: 06.11.2024
# Project: STM32F103C8T6_HELLO_WORLD.
# License: Opensource

include makefiles/makefile_colors.mk
include makefiles/makefile_info.mk
include makefiles/makefile_dir.mk
include makefiles/makefile_clib.mk

NAME := $(OUT_DIR)/TARGET
DEVICE := STM32F103xB
SW_FLAG := LORA_E32_TX
MACH := cortex-m3
FLOAT_ABI := soft
MAP  := -Wl,-Map=$(NAME).map  # Create map file
GC   := -Wl,--gc-sections     # Link for code size
DEBUGINFO :=
OPTIMIZATION := 1
ifneq ($(MAKECMDGOALS),release)
$(info Added debug symbols)
DEBUGINFO += -DDEBUG -g3
OPTIMIZATION = 0
endif

CFLAGS := \
	-c \
	-mcpu=$(MACH) \
	-mthumb \
	-mfloat-abi=$(FLOAT_ABI) \
	-std=gnu11 \
	-O$(OPTIMIZATION) \
	-D$(DEVICE) \
	-D$(SW_FLAG) \
	$(USE_NANO) \
	-Wall \
	-Wextra \
	-ffunction-sections \
	-fdata-sections \
	-fstack-usage \
	-MMD \
	-Wfatal-errors \
	-Werror=implicit \
	-fdiagnostics-color=always

LDFLAGS := \
	-mcpu=$(MACH) \
	-mthumb \
	-mfloat-abi=$(FLOAT_ABI) \
	-T"STM32F103C8TX_FLASH.ld" \
	$(MAP) \
	$(GC) \
	-static \
	$(USE_NANO) \
	-Wl,--start-group -lc -lm -Wl,--end-group \
	-fdiagnostics-color=always

CONST := -DUSE_FULL_LL_DRIVER -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 \
	-DLSE_VALUE=32768 -DHSI_VALUE=8000000 -DLSI_VALUE=40000 -DVDD_VALUE=3300 -DUSE_FULL_ASSERT -DPREFETCH_ENABLE=1 \
	$(CC_COMMON_MACRO)

INC := \
	-ICore/MAIN/inc/ \
	-ICore/Flash/inc \
	-ICore/CC1101/inc \
	-ICore/Lora/inc \
	-ICore/SH1106/inc \
	-ICore/SI4432/inc \
	-ICore/NRF24L01/inc \
	-ICore/Reuse/inc \
	-IDrivers/STM32F1xx_HAL_Driver/inc/ \
	-IDrivers/CMSIS/Device/ST/STM32F1xx/Include/ \
	-IDrivers/CMSIS/Include/

.PHONY: check_flags all clean doc load DIR ELF HEX restart reset release

all: check_flags DIR ELF HEX

release : all

DIR:
	@if [ ! -e $(OUT_DIR) ]; then mkdir $(OUT_DIR); echo "$(CFLAGS)" > $(BUILD_FLAGS_FILE); fi
	@if [ ! -e $(OBJ_DIR) ]; then mkdir $(OBJ_DIR); fi
	@if [ ! -e $(DRIVER_DIR) ]; then mkdir $(DRIVER_DIR); fi
	@$(foreach dir,$(SRC_CORE_DIR_WITHOUT_PREFIX), \
		mkdir -p $(OBJ_DIR)/$(dir); \
	)

check_flags:
	@if [ -e $(OUT_DIR) ]; then \
		echo "$(CFLAGS)" > $(TEMP_BUILD_FLAGS_FILE); \
		if [ ! -f $(BUILD_FLAGS_FILE) ] || ! cmp -s $(TEMP_BUILD_FLAGS_FILE) $(BUILD_FLAGS_FILE); then \
			echo "BUILD FLAGS HAS BEEN CHANGED! REBUILD..."; \
			rm -rf $(OUT_DIR); \
		else \
			rm -f $(TEMP_BUILD_FLAGS_FILE); \
		fi \
	fi

SRC_CORE_DIRS := Core/MAIN/src Core/Flash/src Core/CC1101/src Core/Lora/src Core/SH1106/src Core/SI4432/src \
						Core/NRF24L01/src Core/Reuse/src
SRC_CORE_DIR_WITHOUT_PREFIX := $(foreach dir, $(SRC_CORE_DIRS), $(patsubst Core/%, %, $(dir)))
SRC_DRIVERS_DIR := Drivers/STM32F1xx_HAL_Driver/src

SRC_CORE := $(foreach dir, $(SRC_CORE_DIRS), $(wildcard $(dir)/*.c))
SRC_DRIVERS := $(wildcard $(SRC_DRIVERS_DIR)/*.c)

OBJ_CORE := $(patsubst Core/%.c, $(OBJ_DIR)/%.o, $(SRC_CORE))
OBJ_DRIVERS := $(SRC_DRIVERS:$(SRC_DRIVERS_DIR)/%.c=$(DRIVER_DIR)/%.o)

$(OBJ_DIR)/%.o: Core/%.c
	$(CC) $(CFLAGS) $(CONST) $(DEBUGINFO) $(INC) $< -o $@

$(DRIVER_DIR)/%.o: $(SRC_DRIVERS_DIR)/%.c
	$(CC) $(CFLAGS) $(CONST) $(DEBUGINFO) $(INC) $< -o $@

# $^ dependency $@ target
$(OBJ_DIR)/startup_stm32f103c8tx.o: Core/Startup/startup_stm32f103c8tx.s
	$(CC) $(CFLAGS) $(CONST) $(DEBUGINFO) -o $@ $^

ELF: $(OBJ_DIR)/startup_stm32f103c8tx.o $(OBJ_CORE) $(OBJ_DRIVERS)
	@echo "$(ccblue)\nLinking$(ccend)"
	$(CC) $(LDFLAGS) $^ -o $(OUT_DIR)/target.elf

HEX:
	@echo "$(ccblue)\nCreating hex file$(ccend)"
	$(CC_OBJCOPY) -O ihex $(OUT_DIR)/target.elf $(OUT_DIR)/target.hex

	@echo "$(ccblue)\nCreating bin file$(ccend)"
	$(CC_OBJCOPY) -O binary  $(OUT_DIR)/target.elf  $(OUT_DIR)/target.bin

	@echo "$(ccblue)\nGenerating list file$(ccend)"
	$(CC_OBJDUMP) -h -S  $(OUT_DIR)/target.elf > $(OUT_DIR)/target.list

	@echo "$(ccpurple)"
	arm-none-eabi-size $(OUT_DIR)/target.elf -A -x
	@echo "$(ccend)"

clean:
	rm -rf $(OUT_DIR)

load:
	./support/flash_stlink.sh

restart:
	openocd -f /usr/local/share/openocd/scripts/interface/st-link.cfg \
		-f /usr/local/share/openocd/scripts/target/stm32f1x.cfg \
		-c "init" \
		-c "reset" \
		-c "exit"

reset: restart

doc:
	doxygen

-include $(OBJ_CORE:.o=.d)
-include $(OBJ_DRIVERS:.o=.d)

#serial number st-link
#53FF6C064965525327141187 - purple
#363B15157116303030303032 - gold
