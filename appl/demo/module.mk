d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-$(CONFIG_APPL_DEMO_LED_CTRL) += $(d)led_ctrl.o
obj-$(CONFIG_APPL_DEMO_BUZZER_CTRL) += $(d)buzzer_ctrl.o
