d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-y += $(d)period.o $(d)event.o $(d)task_stats.o
