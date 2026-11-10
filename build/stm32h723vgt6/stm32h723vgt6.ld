/*
 * STM32H723VGTx 链接脚本，源自 STM32CubeMX LinkerScript.ld
 * Copyright (c) 2025 STMicroelectronics. All rights reserved.
 * 在 Cube 脚本上增补 initcall、周期/事件任务、shell、ITCM、可缓存/不可缓存段。
 */

ENTRY(Reset_Handler)

/* 主栈从 DTCM 末尾往下长 */
_estack = ORIGIN(DTCM) + LENGTH(DTCM);
/* .bss 顶到这里就报链接错误，给 newlib 堆和主栈留出空档 */
_Min_Heap_Size = 0x1000;
_Min_Stack_Size = 0x2000;

/* 顺序就是链接报告里的顺序，和地址高低无关 */
MEMORY
{
DTCM   (xrw)      : ORIGIN = 0x20000000, LENGTH = 128K   /* 栈、.data、.bss */
RAM    (xrw)      : ORIGIN = 0x24040000, LENGTH = 64K    /* AXI 后 64 KB，可缓存：大表 */
RAM_D1 (xrw)      : ORIGIN = 0x24000000, LENGTH = 256K   /* AXI 前 256 KB，不可缓存：缓冲和队列 */
RAM_D2 (xrw)      : ORIGIN = 0x30000000, LENGTH = 32K    /* 内部 DMA 已关，当前没有数据放这里 */
RAM_D3 (xrw)      : ORIGIN = 0x38000000, LENGTH = 16K    /* 只给 BDMA */
ITCM   (xrw)      : ORIGIN = 0x00000000, LENGTH = 64K
FLASH  (rx)       : ORIGIN = 0x08000000, LENGTH = 1024K
}

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  /* initcall 表：按级别 0..7 顺序拼接；首尾符号供 initcall_run_all() 遍历 */
  .init_region : ALIGN(4)
  {
    __start_init_region = .;
    KEEP(*(.initcall_0.init))
    KEEP(*(.initcall_1.init))
    KEEP(*(.initcall_2.init))
    KEEP(*(.initcall_3.init))
    KEEP(*(.initcall_4.init))
    KEEP(*(.initcall_5.init))
    KEEP(*(.initcall_6.init))
    KEEP(*(.initcall_7.init))
    __stop_init_region = .;
  } >FLASH

  /* 周期任务指针表：PERIOD_TASK_DEFINE 把指针放进名为 period_tasks 的段 */
  .period_tasks : ALIGN(4)
  {
    __start_period_tasks = .;
    KEEP(*(period_tasks))
    __stop_period_tasks = .;
  } >FLASH

  /* 事件任务指针表：EVENT_TASK_DEFINE 把指针放进名为 event_tasks 的段 */
  .event_tasks : ALIGN(4)
  {
    __start_event_tasks = .;
    KEEP(*(event_tasks))
    __stop_event_tasks = .;
  } >FLASH

  /* letter-shell 的命令表：SHELL_EXPORT_CMD 把命令放进名为 shellCommand 的段（CONFIG_COMPONENT_SHELL=n 时为空） */
  .shellCommand : ALIGN(4)
  {
    _shell_command_start = .;
    KEEP(*(shellCommand))
    _shell_command_end = .;
  } >FLASH

  /* __libc_init_array 要这三对符号。memory_init() 是 constructor，走 .init_array */
  .preinit_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .init_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .fini_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } >FLASH

  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >DTCM AT> FLASH

  /* RAM 代码：运行地址在 ITCM，加载地址在 Flash。
   * ITCM 在 0x00000000，带 64 位 ECC。memory_init() 先把整段 ITCM 用 STRD 清零，再把本段从 Flash 拷过来。
   * 函数标 ITCM_FUNC（section .RamFunc，见 component/common/types.h）就会进这里。现在没有这样的函数，拷贝长度是 0。
   * 链接报告里的 ITCM 用量只算这段镜像，不算启动时清掉的 64 KB。 */
  _siramfunc = LOADADDR(.itcm_text);
  .itcm_text :
  {
    . = ALIGN(4);
    _sramfunc = .;
    *(.RamFunc)
    *(.RamFunc*)
    . = ALIGN(4);
    _eramfunc = .;
  } >ITCM AT> FLASH
  _itcm_start = ORIGIN(ITCM);
  _itcm_end = ORIGIN(ITCM) + LENGTH(ITCM);

  /* 不可缓存：DMA 缓冲、软件队列、字节流重组、RTOS 堆。NOLOAD，memory_init() 清零。
   * CDC 收发缓冲是 Cube 生成的普通 .bss，必须写在下面的 *(.bss*) 前面，否则会进 DTCM。 */
  .sram_noncacheable (NOLOAD) : ALIGN(32)
  {
    _ssram_noncacheable = .;
    *usbd_cdc_if.o(.bss.UserRxBufferHS)
    *usbd_cdc_if.o(.bss.UserTxBufferHS)
    *(.sram_noncacheable)
    *(.sram_noncacheable*)
    . = ALIGN(32);
    _esram_noncacheable = .;
  } >RAM_D1

  .bss :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >DTCM

  /* 可缓存大表。初值在 Flash，memory_init() 在开 D-Cache 之前拷过来 */
  _sisram_cacheable = LOADADDR(.sram_cacheable);
  .sram_cacheable :
  {
    . = ALIGN(8);
    _ssram_cacheable = .;
    *(.sram_cacheable)
    *(.sram_cacheable*)
    . = ALIGN(8);
    _esram_cacheable = .;
  } >RAM AT> FLASH

  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >DTCM

  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }
}
