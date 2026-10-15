d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-$(CONFIG_COMPONENT_FILTER_MEKF) += $(d)mekf_fliter.o $(d)imu_yaw_calibration.o
ifeq ($(CONFIG_COMPONENT_FILTER_MEKF),y)
MODULE_DEFS += -DMEKF_USE_CMSIS_DSP=1
endif
