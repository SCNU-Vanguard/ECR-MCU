# 处理 CONFIG_XXX：规范化取值，生成 -DCONFIG_XXX。
# 某个模块没编进来、却被别的代码调用时，由链接器报 undefined reference。

# 取值规范化：去掉首尾空白；除 y 以外一律当作 n
$(foreach v,$(filter CONFIG_%,$(.VARIABLES)),$(eval $(v) := $(if $(filter y,$($(v))),y,n)))

# 所有为 y 的配置项，生成 -DCONFIG_XXX（C 代码里用 #ifdef CONFIG_XXX 判断）
CFG_VARS := $(sort $(filter CONFIG_%,$(.VARIABLES)))
CFG_DEFS := $(foreach v,$(CFG_VARS),$(if $(filter y,$($(v))),-D$(v)))
