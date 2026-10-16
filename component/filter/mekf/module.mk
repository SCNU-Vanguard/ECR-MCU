obj-$(CONFIG_COMPONENT_FILTER_MEKF) += mekf_fliter.o imu_yaw_calibration.o
ifeq ($(CONFIG_COMPONENT_FILTER_MEKF),y)
MODULE_DEFS += -DMEKF_USE_CMSIS_DSP=1
endif
