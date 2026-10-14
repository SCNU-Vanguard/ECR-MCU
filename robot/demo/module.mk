d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-$(CONFIG_ROBOT_DEMO_ROBOT_INIT) += $(d)robot_init.o
