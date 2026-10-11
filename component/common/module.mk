d := $(dir $(lastword $(MAKEFILE_LIST)))
obj-y += $(d)cqueue.o $(d)initcall.o $(d)framework.o $(d)taskstat.o
