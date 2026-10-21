ifeq ($(CONFIG_ROBOT),y)
    # -I robot/<r>，cdd 里的注册代码包含当前兵种的 *_config.h。
    MODULE_INC += $(d)$(r)
    ifeq ($(r),demo)
        # demo 的表在头文件里：motor_config.h、chassis_config.h、vision_config.h、interboard_config.h。
    else
        $(error unknown robot r=$(r); add a branch in robot/module.mk)
    endif
endif
