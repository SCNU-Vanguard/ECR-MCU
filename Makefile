# ecr-mcu 顶层入口。用法：
#   make p=h7 r=demo                       # p = 平台，r = 机器人（兵种）
#   make p=h7 r=demo OPT=-O2 V=1           # 发布优化 / 显示完整命令
#   make p=h7 r=demo clean
#   make                                        # 等价于 make p=h7 r=demo
# 配置文件：robot/<r>/<r>_<p>.config（CONFIG_XXX = y | n）
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

# 配置文件存在 = 这个兵种支持这个平台
CFGFILE := robot/$(r)/$(r)_$(p).config
ifeq ($(wildcard $(CFGFILE)),)
$(error $(CFGFILE) not found: robot "$(r)" does not support platform "$(p)")
endif

include $(CFGFILE)
include build/config.mk
include build/toolchain.mk
include build/mcu/$(MCU).mk
include build/modules.mk
include build/rules.mk
