# STM32H723VGT6：CubeIDE 工程在 platform/stm32h723vgt6，本文件只"引用"它，不修改它
PLATFORM_DIR := platform/stm32h723vgt6
P            := $(PLATFORM_DIR)

CPU_FLAGS := -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
MCU_DEFS  := -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx

# 与 CubeIDE 生成的 Makefile 里 C_INCLUDES 对应
PLATFORM_INC := \
    $(P)/Core/Inc \
    $(P)/USB_DEVICE/App \
    $(P)/USB_DEVICE/Target \
    $(P)/Drivers/STM32H7xx_HAL_Driver/Inc \
    $(P)/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy \
    $(P)/Middlewares/Third_Party/FreeRTOS/Source/include \
    $(P)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
    $(P)/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
    $(P)/Middlewares/ST/STM32_USB_Device_Library/Core/Inc \
    $(P)/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc \
    $(P)/Drivers/CMSIS/Device/ST/STM32H7xx/Include \
    $(P)/Drivers/CMSIS/Include

# 用通配符而不是抄 CubeIDE 的文件清单：CubeMX 里新增外设后重新生成，这里不用改。
# 未启用的 HAL 模块整个文件被 #ifdef 包住，编译为空，链接时 --gc-sections 去掉。
PLATFORM_SRC := \
    $(wildcard $(P)/Core/Src/*.c) \
    $(wildcard $(P)/USB_DEVICE/App/*.c) \
    $(wildcard $(P)/USB_DEVICE/Target/*.c) \
    $(wildcard $(P)/Drivers/STM32H7xx_HAL_Driver/Src/*.c) \
    $(wildcard $(P)/Middlewares/Third_Party/FreeRTOS/Source/*.c) \
    $(P)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c \
    $(P)/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
    $(P)/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
    $(wildcard $(P)/Middlewares/ST/STM32_USB_Device_Library/Core/Src/*.c) \
    $(wildcard $(P)/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/*.c)

PLATFORM_ASM := $(P)/startup_stm32h723xx.s

# 链接脚本是框架自己维护的副本（在 CubeIDE 生成的脚本上增补几段），不直接改 platform/ 下那份
LDSCRIPT := build/stm32h723vgt6/stm32h723vgt6.ld

# CubeIDE 生成的 platform/ 代码不改，其自带的 missing-field-initializers 告警在这里屏蔽
$(OBJ)/platform/%.o: CFLAGS += -Wno-missing-field-initializers

# 该 MCU 的 CAN 控制器类型：bsp/can/module.mk 据此选择 bsp_can_<CAN_IP>.c（F4 的 bxCAN 另写）
CAN_IP := fdcan
