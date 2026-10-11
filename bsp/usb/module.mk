d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-$(CONFIG_BSP_USB) += $(d)bsp_usb.o
