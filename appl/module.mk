ifeq ($(CONFIG_APPL),y)
    ifeq ($(r),demo)
        subdir-$(CONFIG_APPL_DEMO) += demo
    else ifeq ($(r),demo_gimbal)
        subdir-$(CONFIG_APPL_DEMO_GIMBAL) += demo_gimbal
    else ifeq ($(r),demo_chassis)
        subdir-$(CONFIG_APPL_DEMO_CHASSIS) += demo_chassis
    else
        $(error unknown robot r=$(r); add a branch in appl/module.mk)
    endif
endif
