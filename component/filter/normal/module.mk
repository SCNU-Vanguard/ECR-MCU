d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-$(CONFIG_COMPONENT_FILTER_NORMAL) += $(d)normal_filter.o
