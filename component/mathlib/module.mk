obj-$(CONFIG_COMPONENT_MATHLIB) += user_lib.o


# 数学库按原样移植，不为它的警告改代码
$(OBJ)/$(d)%.o: CFLAGS += -Wno-unused-parameter -Wno-unused-variable -Wno-sign-compare
