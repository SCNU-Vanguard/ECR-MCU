d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-$(CONFIG_COMPONENT_FILTER_MAHONY) += $(d)mahonyahrs.o $(d)mahony_filter.o
