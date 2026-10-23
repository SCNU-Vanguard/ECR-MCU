ifeq ($(CONFIG_COMPONENT),y)
subdir-$(CONFIG_COMPONENT) += common
subdir-$(CONFIG_COMPONENT_LOG) += log
subdir-$(CONFIG_COMPONENT_SHELL) += shell
subdir-$(CONFIG_COMPONENT_PID) += pid
subdir-$(CONFIG_COMPONENT_DSP) += dsp
subdir-$(CONFIG_COMPONENT_AHRS) += ahrs
subdir-$(CONFIG_COMPONENT_FILTER) += filter
subdir-$(CONFIG_COMPONENT_CHASSIS) += chassis
subdir-$(CONFIG_COMPONENT_CRC) += crc
subdir-$(CONFIG_COMPONENT_POWER) += power
endif
