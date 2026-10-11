d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-$(CONFIG_CDD_MOTOR_DJI) += $(d)dji_motor.o
