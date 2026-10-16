ifeq ($(CONFIG_COMPONENT_SHELL),y)
# 串口命令行（第 19.5 节）：letter-shell 源码放在 component/shell/letter-shell/src/，文件名随版本，用通配符
MODULE_INC += $(d)letter-shell/src $(d)
obj-y += shell_port.o
obj-y += $(patsubst $(d)%.c,%.o,$(wildcard $(d)letter-shell/src/*.c))
endif


# letter-shell 源码不改
$(OBJ)/$(d)letter-shell/%.o: CFLAGS += -Wno-cast-function-type -Wno-sign-compare
