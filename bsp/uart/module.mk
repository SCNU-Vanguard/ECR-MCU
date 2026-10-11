d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-$(CONFIG_BSP_UART) += $(d)bsp_uart.o
