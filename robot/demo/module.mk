d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-$(CONFIG_ROBOT_DEMO) += $(d)robot_init.o
