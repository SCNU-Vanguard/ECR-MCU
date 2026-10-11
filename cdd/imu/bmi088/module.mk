d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-$(CONFIG_CDD_IMU_BMI088) += $(d)bmi088.o
