d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-y += $(d)osal_freertos.o
obj-$(CONFIG_OSAL_CPU_USAGE) += $(d)cpu_usage.o
