d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-$(CONFIG_COMPONENT_FILTER_KALMAN) += $(d)kalman_filter.o $(d)kalman_one_filter.o
