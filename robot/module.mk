ifeq ($(CONFIG_ROBOT),y)
    # -I robot/<r>，cdd 里的注册代码包含当前兵种的 *_config.h。
    MODULE_INC += $(d)$(r)
    ifeq ($(r),demo)
        # 表在本目录的头文件里。单板不用板间。
    else ifeq ($(r),demo_gimbal)
        # 表在本目录的头文件里。板间帧和底盘板各写一份。
    else ifeq ($(r),demo_chassis)
        # 表在本目录的头文件里。板间帧和云台板各写一份。
    else ifeq ($(r),sec_infantry)
        # 表在本目录的头文件里。单板不用板间。
    else ifeq ($(r),heavy_gimbal)
        # 表在本目录的头文件里。板间帧和底盘板各写一份。
    else ifeq ($(r),heavy_chassis)
        # 表在本目录的头文件里。板间帧和云台板各写一份。
    else ifeq ($(r),infantry_gimbal)
        # 表在本目录的头文件里。板间帧和底盘板各写一份。
    else ifeq ($(r),infantry_chassis)
        # 表在本目录的头文件里。板间帧和云台板各写一份。
    else ifeq ($(r),sentry_gimbal)
        # 表在本目录的头文件里。板间帧和底盘板各写一份。
    else ifeq ($(r),sentry_chassis)
        # 表在本目录的头文件里。板间帧和云台板各写一份。
    else
        $(error unknown robot r=$(r); add a branch in robot/module.mk)
    endif
endif
