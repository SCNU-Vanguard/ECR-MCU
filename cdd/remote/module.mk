d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-$(CONFIG_CDD_REMOTE_DT7) += $(d)dt7.o
