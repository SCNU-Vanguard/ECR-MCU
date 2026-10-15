d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-$(CONFIG_COMPONENT_FILTER_LPF) += $(d)lpf.o
