ifeq ($(CONFIG_ROBOT),y)
    # 电机注册对所有兵种同一份。-I robot/<r> 让 motor_init.c 包含当前兵种的 motor_config.h。
    obj-y += motor_init.o
    MODULE_INC += $(d)$(r)
    ifeq ($(r),demo)
        # demo 只有头文件：motor_config.h、mechanical_config.h。
    else
        $(error unknown robot r=$(r); add a branch in robot/module.mk)
    endif
endif
