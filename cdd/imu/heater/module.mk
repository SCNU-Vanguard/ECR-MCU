d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-$(CONFIG_CDD_IMU_HEATER) += $(d)imu_heater.o