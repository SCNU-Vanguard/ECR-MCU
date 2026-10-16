/*
******************************************************************************
**

**  File        : LinkerScript.ld
**
**  Author		: STM32CubeMX
**
**  Abstract    : Linker script for STM32H723VGTx series
**                1024Kbytes FLASH and 560Kbytes RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed “as is,” without any warranty
**                of any kind.
**
*****************************************************************************
** @attention
**
** <h2><center>&copy; COPYRIGHT(c) 2025 STMicroelectronics</center></h2>
**
** Redistribution and use in source and binary forms, with or without modification,
** are permitted provided that the following conditions are met:
**   1. Redistributions of source code must retain the above copyright notice,
**      this list of conditions and the following disclaimer.
**   2. Redistributions in binary form must reproduce the above copyright notice,
**      this list of conditions and the following disclaimer in the documentation
**      and/or other materials provided with the distribution.
**   3. Neither the name of STMicroelectronics nor the names of its contributors
**      may be used to endorse or promote products derived from this software
**      without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
** FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
** DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
** SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
** CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
** OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(DTCM) + LENGTH(DTCM);    /* end of DTCM，主栈从这里往下长 */
/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0x1000;      /* required amount of heap  */
_Min_Stack_Size = 0x2000; /* required amount of stack */

/* Specify the memory areas */
MEMORY
{
/* 顺序就是链接报告里的顺序，和地址高低无关 */
DTCM   (xrw)      : ORIGIN = 0x20000000, LENGTH = 128K   /* 栈、.data、.bss */
RAM    (xrw)      : ORIGIN = 0x24040000, LENGTH = 64K    /* AXI 后 64 KB，可缓存：大表 */
RAM_D1 (xrw)      : ORIGIN = 0x24000000, LENGTH = 256K   /* AXI 前 256 KB，不可缓存：缓冲和队列 */
RAM_D2 (xrw)      : ORIGIN = 0x30000000, LENGTH = 32K    /* USB 内部 DMA */
RAM_D3 (xrw)      : ORIGIN = 0x38000000, LENGTH = 16K    /* 只给 BDMA */
ITCM   (xrw)      : ORIGIN = 0x00000000, LENGTH = 64K
FLASH  (rx)       : ORIGIN = 0x08000000, LENGTH = 1024K
}

/* Define output sections */
SECTIONS
{
  /* The startup code goes first into FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH

  /* The program code and other data goes into FLASH */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH

  /* Constant data goes into FLASH */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
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

  .ARM.extab : /* The "READONLY" keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >FLASH

  .ARM : /* The "READONLY" keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >FLASH

  .preinit_array : /* The "READONLY" keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .init_array : /* The "READONLY" keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .fini_array : /* The "READONLY" keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } >FLASH

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >DTCM AT> FLASH

  /* RAM 代码：运行地址在 ITCM，加载地址在 Flash。
   * ITCM 在 0x00000000，带 64 位 ECC。memory_init() 先把整段 ITCM 用 STRD 清零，再把本段从 Flash 拷过来。
   * 函数标 __RAM_FUNC（section .RamFunc）就会进这里。现在没有这样的函数，拷贝长度是 0。
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

  /* USB 内部 DMA 能访问的内存只有 D2 SRAM。句柄里的 Setup 缓冲、CDC 收发缓冲、发送分块都放这里。
   * 写在 .bss 前面，否则 *(.bss*) 会先把它们收进 DTCM。NOLOAD，由 memory_init() 清零 */
  .usb_dma (NOLOAD) : ALIGN(32)
  {
    _susb_dma = .;
    . = ALIGN(32);
    *usbd_conf.o(.bss.hpcd_USB_OTG_HS)
    . = ALIGN(32);
    *usbd_cdc_if.o(.bss.UserRxBufferHS)
    . = ALIGN(32);
    *usbd_cdc_if.o(.bss.UserTxBufferHS)
    . = ALIGN(32);
    *(.usb_dma)
    *(.usb_dma*)
    . = ALIGN(32);
    _eusb_dma = .;
  } >RAM_D2

  /* Uninitialized data section */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss secion */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >DTCM

  /* 不可缓存：DMA 缓冲、软件队列、字节流重组、RTOS 堆。NOLOAD，memory_init() 清零 */
  .sram_noncacheable (NOLOAD) : ALIGN(32)
  {
    _ssram_noncacheable = .;
    *(.sram_noncacheable)
    *(.sram_noncacheable*)
    . = ALIGN(32);
    _esram_noncacheable = .;
  } >RAM_D1

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



  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

}


