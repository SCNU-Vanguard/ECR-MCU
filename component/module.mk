ifeq ($(CONFIG_COMPONENT),y)
subdir-$(CONFIG_COMPONENT) += common
subdir-$(CONFIG_COMPONENT_LOG) += log
subdir-$(CONFIG_COMPONENT_SHELL) += shell
subdir-$(CONFIG_COMPONENT_PID) += pid
subdir-$(CONFIG_COMPONENT_DSP) += dsp
subdir-$(CONFIG_COMPONENT_MATHLIB) += mathlib
subdir-$(CONFIG_COMPONENT_FILTER) += filter
endif
