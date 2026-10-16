obj-$(CONFIG_CDD_MOTOR) += motor.o motor_control.o motor_shell.o

subdir-$(CONFIG_CDD_MOTOR_DJI_MOTOR) += dji_motor
subdir-$(CONFIG_CDD_MOTOR_DM_MOTOR) += dm_motor
