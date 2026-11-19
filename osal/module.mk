obj-$(CONFIG_OSAL) += period.o event.o task_stats.o

subdir-$(CONFIG_OSAL_FREERTOS) += freertos
