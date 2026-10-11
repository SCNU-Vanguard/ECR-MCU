d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-y += $(d)bsp_tick.o
