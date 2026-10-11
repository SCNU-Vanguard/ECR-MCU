d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-y += $(d)board.o
