d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-$(CONFIG_BSP_CAN) += $(d)bsp_can_$(CAN_IP).o
