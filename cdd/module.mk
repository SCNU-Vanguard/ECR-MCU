ifeq ($(CONFIG_CDD),y)
subdir-$(CONFIG_CDD_REMOTE) += remote
subdir-$(CONFIG_CDD_MOTOR) += motor
subdir-$(CONFIG_CDD_IMU) += imu
subdir-$(CONFIG_CDD_LED) += led
subdir-$(CONFIG_CDD_BUZZER) += buzzer
subdir-$(CONFIG_CDD_REFEREE) += referee
subdir-$(CONFIG_CDD_IMGTRANS) += imgtrans
subdir-$(CONFIG_CDD_HOST) += host
subdir-$(CONFIG_CDD_INTERBOARD) += interboard
endif
