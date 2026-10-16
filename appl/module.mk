ifeq ($(CONFIG_APPL),y)
    ifeq ($(r),demo)
        subdir-$(CONFIG_APPL_DEMO) += demo
    else
        $(error unknown robot r=$(r); add a branch in appl/module.mk)
    endif
endif
