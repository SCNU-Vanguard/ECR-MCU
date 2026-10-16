ifeq ($(CONFIG_BSP),y)
subdir-y += tick board/$(MCU)
subdir-$(CONFIG_BSP_GPIO) += gpio
subdir-$(CONFIG_BSP_UART) += uart
subdir-$(CONFIG_BSP_CAN) += can
subdir-$(CONFIG_BSP_SPI) += spi
subdir-$(CONFIG_BSP_PWM) += pwm
subdir-$(CONFIG_BSP_USB) += usb
endif
