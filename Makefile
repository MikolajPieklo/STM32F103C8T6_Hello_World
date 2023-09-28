# Author: M Pieklo
# Date: 20.11.2022
# Project: STM32F103C8T6_HELLO_WORLD.
# License: Opensource

include makefiles/makefile_colors.mk
include makefiles/makefile_info.mk
include makefiles/makefile_dir.mk
include makefiles/makefile_clib.mk

NAME := $(OUT_DIR)/TARGET
DEVICE := STM32F103xB
MACH := cortex-m3
FLOAT_ABI := soft
MAP  := -Wl,-Map=$(NAME).map  # Create map file
GC   := -Wl,--gc-sections     # Link for code size
DEBUGINFO := -DDEBUG -g3

CFLAGS := \
	-c \
	-mcpu=$(MACH) \
	-mthumb \
	-mfloat-abi=$(FLOAT_ABI) \
	-std=gnu11 \
	-O0 \
	-D$(DEVICE) \
	$(USE_NANO) \
	-Wall \
	-Wextra \
	-ffunction-sections \
	-fdata-sections \
	-fstack-usage \
	-MMD \
	-Wfatal-errors \
	-Werror=implicit

LDFLAGS := \
	-mcpu=$(MACH) \
	-mthumb \
	-mfloat-abi=$(FLOAT_ABI) \
	-T"STM32F103C8TX_FLASH.ld" \
	$(MAP) \
	$(GC) \
	-static \
	$(USE_NANO) \
	-Wl,--start-group -lc -lm -Wl,--end-group

CONST := -DUSE_FULL_LL_DRIVER -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 \
	-DLSE_VALUE=32768 -DHSI_VALUE=8000000 -DLSI_VALUE=40000 -DVDD_VALUE=3300 -DUSE_FULL_ASSERT -DPREFETCH_ENABLE=1 \
	$(CC_COMMON_MACRO)

INC := \
	-ICore/inc/ \
	-IDrivers/STM32F1xx_HAL_Driver/inc/ \
	-IDrivers/CMSIS/Device/ST/STM32F1xx/Include/ \
	-IDrivers/CMSIS/Include/

.PHONY: all clean doc load DIR ELF HEX restart reset

all: DIR ELF HEX

DIR:
	@if [ ! -e $(OUT_DIR) ]; then mkdir $(OUT_DIR); fi
	@if [ ! -e $(OBJ_DIR) ]; then mkdir $(OBJ_DIR); fi
	@if [ ! -e $(DRIVER_DIR) ]; then mkdir $(DRIVER_DIR); fi

SRC_CORE_DIR := Core/src
SRC_DRIVERS_DIR := Drivers/STM32F1xx_HAL_Driver/src

SRC_CORE := $(wildcard $(SRC_CORE_DIR)/*.c)
SRC_DRIVERS := $(wildcard $(SRC_DRIVERS_DIR)/*.c)

OBJ_CORE := $(SRC_CORE:$(SRC_CORE_DIR)/%.c=$(OBJ_DIR)/%.o)
OBJ_DRIVERS := $(SRC_DRIVERS:$(SRC_DRIVERS_DIR)/%.c=$(DRIVER_DIR)/%.o)

$(OBJ_DIR)/%.o: $(SRC_CORE_DIR)/%.c
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
	openocd -f /usr/local/share/openocd/scripts/interface/st-link.cfg \
		-f /usr/local/share/openocd/scripts/target/stm32f1x.cfg \
		-c "program $(OUT_DIR)/target.elf verify reset exit"

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
