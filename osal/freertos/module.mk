d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-y += $(d)osal_freertos.o
