obj-$(CONFIG_COMPONENT_FILTER_AHRS) += ahrs.o

subdir-$(CONFIG_COMPONENT_FILTER_VQF) += vqf
subdir-$(CONFIG_COMPONENT_FILTER_MEKF) += mekf
subdir-$(CONFIG_COMPONENT_FILTER_MADGWICK) += madgwick
subdir-$(CONFIG_COMPONENT_FILTER_MAHONY) += mahony
subdir-$(CONFIG_COMPONENT_FILTER_EKF) += ekf
subdir-$(CONFIG_COMPONENT_FILTER_KALMAN) += kalman
subdir-$(CONFIG_COMPONENT_FILTER_LPF) += lpf
subdir-$(CONFIG_COMPONENT_FILTER_NORMAL) += normal


# 滤波算法按原样移植，不为它们的警告改代码
$(OBJ)/$(d)%.o: CFLAGS += -Wno-unused-parameter -Wno-unused-variable -Wno-unused-but-set-variable -Wno-sign-compare -Wno-missing-field-initializers -Wno-implicit-fallthrough -Wno-maybe-uninitialized
