ifeq ($(CONFIG_APPL),y)
    ifeq ($(r),demo)
        subdir-$(CONFIG_APPL_DEMO) += demo
    else ifeq ($(r),demo_gimbal)
        subdir-$(CONFIG_APPL_DEMO_GIMBAL) += demo_gimbal
    else ifeq ($(r),demo_chassis)
        subdir-$(CONFIG_APPL_DEMO_CHASSIS) += demo_chassis
    else ifeq ($(r),sec_infantry)
        subdir-$(CONFIG_APPL_SEC_INFANTRY) += sec_infantry
    else ifeq ($(r),heavy_gimbal)
        subdir-$(CONFIG_APPL_HEAVY_GIMBAL) += heavy_gimbal
    else ifeq ($(r),heavy_chassis)
        subdir-$(CONFIG_APPL_HEAVY_CHASSIS) += heavy_chassis
    else ifeq ($(r),infantry_gimbal)
        subdir-$(CONFIG_APPL_INFANTRY_GIMBAL) += infantry_gimbal
    else ifeq ($(r),infantry_chassis)
        subdir-$(CONFIG_APPL_INFANTRY_CHASSIS) += infantry_chassis
    else ifeq ($(r),sentry_gimbal)
        subdir-$(CONFIG_APPL_SENTRY_GIMBAL) += sentry_gimbal
    else ifeq ($(r),sentry_chassis)
        subdir-$(CONFIG_APPL_SENTRY_CHASSIS) += sentry_chassis
    else
        $(error unknown robot r=$(r); add a branch in appl/module.mk)
    endif
endif
