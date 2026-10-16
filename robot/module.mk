ifeq ($(CONFIG_ROBOT),y)
    ifeq ($(r),demo)
        subdir-$(CONFIG_ROBOT_DEMO) += demo
    else
        $(error unknown robot r=$(r); add a branch in robot/module.mk)
    endif
endif
