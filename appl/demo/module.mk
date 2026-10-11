d := $(dir $(lastword $(MAKEFILE_LIST)))
ifeq ($(CONFIG_APPL_DEMO),y)
obj-$(CONFIG_CDD_LED_WS2812) += $(d)led_ctrl.o
endif
