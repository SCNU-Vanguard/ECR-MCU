obj-$(CONFIG_CDD_MOTOR) += motor.o motor_control.o motor_init.o
obj-$(CONFIG_COMPONENT_SHELL) += motor_pid_tune_shell.o

subdir-$(CONFIG_CDD_MOTOR_DJI_MOTOR) += dji_motor
subdir-$(CONFIG_CDD_MOTOR_DM_MOTOR) += dm_motor
subdir-$(CONFIG_CDD_MOTOR_LK_MOTOR) += lk_motor
