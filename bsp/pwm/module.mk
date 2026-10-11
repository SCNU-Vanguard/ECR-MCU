d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-$(CONFIG_BSP_PWM) += $(d)bsp_pwm.o
