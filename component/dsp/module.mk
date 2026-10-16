# CMSIS-DSP 1.7.0 预编译库（GCC，Cortex-M7，硬浮点：arm_cortexM7lfdp_math）。
# 头文件目录和库文件通过 MODULE_INC / MODULE_LIBS 交给 rules.mk，ARM_MATH_CM7 通过 MODULE_DEFS 打开 M7 优化路径。
ifeq ($(CONFIG_COMPONENT_DSP),y)
MODULE_INC  += $(d)inc
MODULE_LIBS += $(d)lib/arm_cortexM7lfdp_math.lib
MODULE_DEFS += -DARM_MATH_CM7
endif
