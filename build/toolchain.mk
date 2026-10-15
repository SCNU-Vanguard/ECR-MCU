# 工具链与通用选项
PREFIX  ?= arm-none-eabi-
CC      := $(PREFIX)gcc
AS      := $(PREFIX)gcc -x assembler-with-cpp
OBJCOPY := $(PREFIX)objcopy
SIZE    := $(PREFIX)size

OPT     ?= -O3
Q       := $(if $(V),,@)

# 通用编译选项；CPU_FLAGS 由 build/mcu/<mcu>.mk 提供
COMMON_CFLAGS := -std=gnu11 -Wall -Wextra -Wno-unused-parameter \
                 -fdata-sections -ffunction-sections -fno-strict-aliasing -g -gdwarf-2
