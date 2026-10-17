obj-y += ahrs.o

subdir-$(CONFIG_COMPONENT_AHRS_VQF) += vqf
subdir-$(CONFIG_COMPONENT_AHRS_MEKF) += mekf
subdir-$(CONFIG_COMPONENT_AHRS_MADGWICK) += madgwick
subdir-$(CONFIG_COMPONENT_AHRS_MAHONY) += mahony

# 姿态算法按原样移植，不为它们的警告改代码
$(OBJ)/$(d)%.o: CFLAGS += -Wno-unused-parameter -Wno-unused-variable -Wno-unused-but-set-variable -Wno-sign-compare -Wno-missing-field-initializers -Wno-implicit-fallthrough -Wno-maybe-uninitialized
