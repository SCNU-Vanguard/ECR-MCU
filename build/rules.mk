# 通用规则：收集源文件 -> 编译 -> 链接 -> 生成 bin/hex
NAME := $(r)_$(p)
OUT  := output/$(NAME)
OBJ  := $(OUT)/obj

# 目标文件：platform 下的源文�?+ 各模�?module.mk �?obj-y 声明的文件（路径相对仓库根）
OBJS := $(patsubst %.c,$(OBJ)/%.o,$(PLATFORM_SRC)) $(patsubst %.s,$(OBJ)/%.o,$(PLATFORM_ASM)) \
        $(addprefix $(OBJ)/,$(obj-y))

# 头文件：仓库�?(-I.) 让框架代码用 "bsp/uart/bsp_uart.h" 这种带目录前缀的写法，
# CubeIDE 生成的 platform/ 代码不改，其自带的 missing-field-initializers 告警在这里屏蔽
# 链接�?>= 2.39 才有的选项：关�?"LOAD segment with RWX" 告警（Cortex-M 链接脚本�?Flash/RAM 同在一个加载段，属正常）；旧版 ld 不认，自动探�?
COMMA := ,
LD_NOWARN_RWX := $(if $(filter-out 0,$(shell $(PREFIX)ld --help 2>/dev/null | grep -c no-warn-rwx-segments)),-Wl$(COMMA)--no-warn-rwx-segments)

INCS   := -I. $(addprefix -I,$(PLATFORM_INC) $(MODULE_INC))
CFLAGS := $(CPU_FLAGS) $(MCU_DEFS) $(CFG_DEFS) $(COMMON_CFLAGS) $(OPT) $(INCS)
LDFLAGS := $(CPU_FLAGS) -specs=nano.specs -T$(LDSCRIPT) -Wl,--gc-sections \
           -Wl,-Map=$(OUT)/$(NAME).map,--cref -Wl,--print-memory-usage $(LD_NOWARN_RWX) -lc -lm -lnosys

.PHONY: all clean
all: $(OUT)/$(NAME).elf $(OUT)/$(NAME).bin $(OUT)/$(NAME).hex
	$(Q)$(SIZE) $<

# 编译参数（CONFIG_XXX 宏、优化级别）变了就全部重编：参数写进一个戳文件，内容变了戳才更新，目标文件都依赖它
STAMP := $(OBJ)/.cflags
$(STAMP): FORCE
	@mkdir -p $(dir $@)
	@echo '$(CFLAGS)' | cmp -s - $@ || echo '$(CFLAGS)' > $@
FORCE:

# CubeIDE 生成的 platform/ 代码不改，其自带的 missing-field-initializers 告警在这里屏蔽
$(OBJ)/platform/%.o: CFLAGS += -Wno-missing-field-initializers
# 第三方库源码不改，不为它们的警告改代码
$(OBJ)/component/shell/letter-shell/%.o: CFLAGS += -Wno-cast-function-type -Wno-sign-compare
$(OBJ)/component/shell/shell_cmds.o: CFLAGS += -Wno-cast-function-type
$(OBJ)/component/log/easylogger/%.o: CFLAGS += -Wno-sign-compare -Wno-unused-parameter
$(OBJ)/%.o: %.c $(STAMP)
	@mkdir -p $(dir $@)
	@echo "CC  $<"
	$(Q)$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(OBJ)/%.o: %.s $(STAMP)
	@mkdir -p $(dir $@)
	@echo "AS  $<"
	$(Q)$(AS) $(CPU_FLAGS) $(OPT) -c $< -o $@

$(OUT)/$(NAME).elf: $(OBJS) $(LDSCRIPT)
	@echo "LD  $@"
	$(Q)$(CC) $(OBJS) $(LDFLAGS) -o $@

$(OUT)/%.bin: $(OUT)/%.elf
	$(Q)$(OBJCOPY) -O binary -S $< $@
$(OUT)/%.hex: $(OUT)/%.elf
	$(Q)$(OBJCOPY) -O ihex $< $@

clean:
	rm -rf $(OUT)

-include $(OBJS:.o=.d)
