d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-$(CONFIG_CDD_MOTOR_DJI_MOTOR) += $(d)dji_motor.o
