# ecr-mcu 顶层入口。用法：
#   make p=h7 r=demo_gimbal                # p = 平台，r = 机器人（兵种）。默认 -O3，按 CPU 核数并行
#   make p=h7 r=demo_chassis               # 底盘板
#   make p=h7 r=demo                       # 单板，底盘云台发射在同一块板上
#   make p=h7 r=demo_gimbal OPT=-Og V=1    # 临时改优化级别 / 显示完整命令
#   make p=h7 r=demo_gimbal clean
#   make                                        # 等价于 make p=h7 r=demo
# 配置文件：robot/<r>/robot.config（CONFIG_XXX = y | n），与平台无关
# 产物：output/<r>_<p>/<r>_<p>.{elf,bin,hex,map}

TOP := $(CURDIR)

# 命令行没写 -j 时，按核数并行。写了 -j 就沿用命令行的。
ifeq ($(filter -j% --jobserver-auth% jobserver,$(MAKEFLAGS)),)
NPROC := $(shell nproc 2>/dev/null || echo 8)
MAKEFLAGS += -j$(NPROC)
endif

p ?= h7
r ?= demo

# 平台简称 -> MCU 型号（对应 platform/<mcu> 和 build/<mcu>/<mcu>.mk）
MCU_h7 := stm32h723vgt6
MCU_f4 := stm32f407igh6
MCU    := $(MCU_$(p))
ifeq ($(MCU),)
$(error unknown platform p=$(p); supported: h7 f4)
endif

CFGFILE := robot/$(r)/robot.config
ifeq ($(wildcard $(CFGFILE)),)
$(error $(CFGFILE) not found)
endif

include $(CFGFILE)
include build/config.mk

NAME := $(r)_$(p)
OUT  := output/$(NAME)
OBJ  := $(OUT)/obj
include build/toolchain.mk
include build/$(MCU)/$(MCU).mk
include build/modules.mk
include build/rules.mk
