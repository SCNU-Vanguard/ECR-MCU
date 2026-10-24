# 通用规则：收集源文件 -> 编译 -> 链接 -> 生成 bin/hex
# 目标文件：platform 下的源文件 + 各模块 module.mk 里 obj-y 声明的文件（路径相对仓库根）
OBJS := $(patsubst %.c,$(OBJ)/%.o,$(PLATFORM_SRC)) $(patsubst %.s,$(OBJ)/%.o,$(PLATFORM_ASM)) \
        $(addprefix $(OBJ)/,$(obj-y))

# 头文件：仓库根 (-I.) 让框架代码用 "bsp/uart/bsp_uart.h" 这种带目录前缀的写法。
COMMA := ,
LD_NOWARN_RWX := $(if $(filter-out 0,$(shell $(PREFIX)ld --help 2>/dev/null | grep -c no-warn-rwx-segments)),-Wl$(COMMA)--no-warn-rwx-segments)

INCS   := -I. $(addprefix -I,$(PLATFORM_INC) $(MODULE_INC))
CFLAGS := $(CPU_FLAGS) $(MCU_DEFS) $(MODULE_DEFS) $(CFG_DEFS) $(COMMON_CFLAGS) $(OPT) $(INCS)
LDFLAGS := $(CPU_FLAGS) -specs=nano.specs -T$(LDSCRIPT) -Wl,--gc-sections \
           -Wl,-Map=$(OUT)/$(NAME).map,--cref -Wl,--print-memory-usage $(LD_NOWARN_RWX) -Wl,--no-wchar-size-warning -Wl,-z,noexecstack $(MODULE_LIBS) -lc -lm -lnosys

.PHONY: all clean
all: $(OUT)/$(NAME).elf $(OUT)/$(NAME).bin $(OUT)/$(NAME).hex
	$(Q)$(SIZE) $<

# 编译参数（CONFIG_XXX 宏、优化级别）变了就全部重编：参数写进一个戳文件，内容变了戳才更新，目标文件都依赖它
STAMP := $(OBJ)/.cflags
$(STAMP): FORCE
	@mkdir -p $(dir $@)
	@echo '$(CFLAGS)' | cmp -s - $@ || echo '$(CFLAGS)' > $@
FORCE:

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

# ---- 烧录 / 调试（DAPLink + pyOCD）----
# make p=h7 r=demo_gimbal flash    烧录并复位
# make p=h7 r=demo_gimbal gdb      启动 pyOCD gdb server（端口 3333），再用 arm-none-eabi-gdb 连接
PYOCD  ?= py -m pyocd
PYOCD_TARGET_stm32h723vgt6 := stm32h723vgtx
PYOCD_TARGET_stm32f407igh6 := stm32f407ig
PYOCD_TARGET := $(PYOCD_TARGET_$(MCU))
PYOCD_FREQ ?= 4000000

.PHONY: flash gdb
flash: $(OUT)/$(NAME).elf
	$(PYOCD) flash -t $(PYOCD_TARGET) -f $(PYOCD_FREQ) $<
gdb:
	$(PYOCD) gdbserver -t $(PYOCD_TARGET) -f $(PYOCD_FREQ)

-include $(OBJS:.o=.d)
