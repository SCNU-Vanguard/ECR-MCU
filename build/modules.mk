# 从每一层的根目录往下走。每个 module.mk 只写本目录里的文件名：
#   obj-$(CONFIG_XXX) += ins.o
# 这里在 include 之后把目录前缀补上。CONFIG 不是 y 时追加到 obj-n，不会进最终的 obj-y。
# subdir-$(CONFIG_XXX) += 子目录。父层 CONFIG 不是 y 时，不再进入它下面的目录。
# 头文件、库、告警仍然用扫描时设好的 $(d)（带结尾斜杠）。
# 哪些目录参与编译只由 robot/<r>/robot.config 决定。

obj-y :=
MODULE_INC :=
MODULE_LIBS :=
MODULE_DEFS :=
scanned :=

define scan
$(if $(filter $(1),$(scanned)),,\
$(eval scanned += $(1))\
$(eval d := $(1)/)\
$(eval subdir-y :=)\
$(eval obj-before := $(obj-y))\
$(eval include $(1)/module.mk)\
$(eval obj-y := $(obj-before) $(addprefix $(d),$(filter-out $(obj-before),$(obj-y))))\
$(foreach s,$(subdir-y),$(call scan,$(1)/$(s))))
endef

$(call scan,component)
$(call scan,bsp)
$(call scan,cdd)
$(call scan,osal)
$(call scan,appl)
$(call scan,robot)
