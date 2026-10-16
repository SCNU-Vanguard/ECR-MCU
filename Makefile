# ecr-mcu 顶层入口。用法：
#   make p=h7 r=demo                       # p = 平台，r = 机器人（兵种）
#   make p=h7 r=demo OPT=-O2 V=1           # 发布优化 / 显示完整命令
#   make p=h7 r=demo clean
#   make                                        # 等价于 make p=h7 r=demo
# 配置文件：robot/<r>/<r>.config（CONFIG_XXX = y | n），与平台无关
# 产物：output/<r>_<p>/<r>_<p>.{elf,bin,hex,map}

TOP := $(CURDIR)

p ?= h7
r ?= demo

# 平台简称 -> MCU 型号（对应 platform/<mcu> 和 build/mcu/<mcu>.mk）
MCU_h7 := stm32h723vgt6
MCU_f4 := stm32f407igh6
MCU    := $(MCU_$(p))
ifeq ($(MCU),)
$(error unknown platform p=$(p); supported: h7 f4)
endif

CFGFILE := robot/$(r)/$(r).config
ifeq ($(wildcard $(CFGFILE)),)
$(error $(CFGFILE) not found)
endif

include $(CFGFILE)
include build/config.mk

NAME := $(r)_$(p)
OUT  := output/$(NAME)
OBJ  := $(OUT)/obj
include build/toolchain.mk
include build/mcu/$(MCU).mk
include build/modules.mk
include build/rules.mk
