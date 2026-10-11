d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-$(CONFIG_CDD_BUZZER) += $(d)buzzer.o
