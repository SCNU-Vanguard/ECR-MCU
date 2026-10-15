d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-$(CONFIG_CDD_MOTOR) += $(d)motor.o $(d)motor_control.o $(d)motor_shell.o
