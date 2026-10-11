d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-$(CONFIG_APPL_DEMO) += $(d)demo_tasks.o
