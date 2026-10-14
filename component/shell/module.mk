d := $(dir $(lastword $(MAKEFILE_LIST)))
ifeq ($(CONFIG_COMPONENT_SHELL),y)
# 串口命令行（第 19.5 节）：letter-shell 源码放在 component/shell/letter-shell/src/，文件名随版本，用通配符
MODULE_INC += $(d)letter-shell/src $(d)
obj-y += $(d)shell_port.o
obj-y += $(patsubst %.c,%.o,$(wildcard $(d)letter-shell/src/*.c))
endif
