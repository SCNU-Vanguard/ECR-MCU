ifeq ($(CONFIG_COMPONENT_LOG),y)
# log 模块 = EasyLogger（第 19.4 节）：库源码在 component/log/easylogger/，移植层在 component/log/elog_port/
MODULE_INC += $(d)easylogger/inc $(d)elog_port
obj-y += elog_port/elog_port.o easylogger/src/elog.o easylogger/src/elog_utils.o
endif


# EasyLogger 源码不改
$(OBJ)/$(d)easylogger/%.o: CFLAGS += -Wno-sign-compare -Wno-unused-parameter -Wno-array-bounds
