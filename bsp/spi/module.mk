d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-$(CONFIG_BSP_SPI) += $(d)bsp_spi.o
