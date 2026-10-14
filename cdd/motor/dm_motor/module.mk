d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-$(CONFIG_CDD_MOTOR_DM_MOTOR) += $(d)dm_motor.o
