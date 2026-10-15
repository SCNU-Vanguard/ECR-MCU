d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-$(CONFIG_BSP_GPIO) += $(d)bsp_gpio.o
