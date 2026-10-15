# 收集所有模块目录里的 module.mk。每个 module.mk 用 obj-$(CONFIG_XXX) += $(d)xxx.o 声明自己的源文件，
# 写法和 dsar-mcu 里各目录 Makefile 的 obj-$(CONFIG_XXX) += xxx.o 一致；CONFIG_XXX 不是 y 就不会编译。
MODULE_DIRS := component/common component/log component/shell component/pid \
               component/dsp component/mathlib \
               component/filter/vqf component/filter/mekf component/filter/madgwick component/filter/mahony \
               component/filter/ekf component/filter/kalman \
               component/filter/lpf component/filter/normal component/filter \
               osal osal/freertos \
               bsp/tick bsp/gpio bsp/uart bsp/can bsp/spi bsp/pwm bsp/usb \
               bsp/board/$(MCU) \
               cdd/remote cdd/motor cdd/motor/dji_motor cdd/motor/dm_motor cdd/imu/bmi088 cdd/imu/heater cdd/led/ws2812 cdd/buzzer \
               appl/$(r) robot/$(r)

obj-y :=
MODULE_INC :=      # 模块自己要加的头文件路径（第三方库用），rules.mk 会加到 -I 里
MODULE_LIBS :=     # 模块自带的预编译库（.a/.lib），rules.mk 会加到链接命令里
MODULE_DEFS :=     # 模块要加的编译宏（如 -DARM_MATH_CM7），rules.mk 会加到 CFLAGS 里
include $(wildcard $(addsuffix /module.mk,$(MODULE_DIRS)))
