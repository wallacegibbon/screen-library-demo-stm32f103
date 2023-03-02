CROSS_COMPILER_PREFIX = arm-none-eabi-
OPENOCD_PATH =
OPENOCD = openocd
#OPENOCD_ARGS = -f interface/stlink.cfg -f target/stm32f1x.cfg
OPENOCD_ARGS = -f interface/cmsis-dap.cfg -f target/stm32f1x.cfg

TARGET = target
BUILD_DIR = build

LIB_PERIPHERAL_DIR = $(HOME)/STM32_standard_peripheral_library/stm32f10x
ARCH = -mcpu=cortex-m4 -mthumb

CROSS_C_SOURCE_FILES += $(wildcard $(LIB_PERIPHERAL_DIR)/src/*.c)
CROSS_C_SOURCE_FILES += $(wildcard ./src/screen-library-mcu/*.c)
CROSS_C_SOURCE_FILES += $(wildcard ./src/screen-library-mcu/stm32f10x/*.c)
CROSS_C_SOURCE_FILES += $(wildcard ./src/*.c)

CROSS_ASM_SOURCE_FILES = $(wildcard ./src/*.S)

CROSS_LINKER_SCRIPT = ./src/stm32_flash.ld

CROSS_C_ASM_INCLUDES = \
-I$(LIB_PERIPHERAL_DIR)/inc \
-I./src/screen-library-mcu/stm32f10x -I./src/screen-library-mcu -I./src \

include ./miscellaneous-makefiles/cross-gcc-mcu.mk

CROSS_C_ASM_FLAGS += \
-DUSE_STDPERIPH_DRIVER -DSTM32F10X_LD

CROSS_GDB = gdb-multiarch

