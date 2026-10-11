d := $(dir $(lastword $(MAKEFILE_LIST)))
ifeq ($(CONFIG_LOG),y)
# log 模块 = EasyLogger（第 19.4 节）：库源码在 component/log/easylogger/，移植层在 component/log/elog_port/
MODULE_INC += $(d)easylogger/inc $(d)elog_port
obj-y += $(d)elog_port/elog_port.o $(d)easylogger/src/elog.o $(d)easylogger/src/elog_utils.o
endif
