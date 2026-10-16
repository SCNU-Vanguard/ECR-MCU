ifeq ($(CONFIG_CDD),y)
subdir-$(CONFIG_CDD_REMOTE) += remote
subdir-$(CONFIG_CDD_MOTOR) += motor
subdir-$(CONFIG_CDD_IMU) += imu
subdir-$(CONFIG_CDD_LED) += led
subdir-$(CONFIG_CDD_BUZZER) += buzzer
endif
