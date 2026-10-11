d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-$(CONFIG_CDD_LED_WS2812) += $(d)ws2812.o
