#ifndef COMMON_INITCALL_H__
#define COMMON_INITCALL_H__
#include "component/common/types.h"

/* 自注册初始化表：模块用宏把 {名字, 函数} 放进 .initcall_<级别>.init，
 * 链接脚本按级别 0..3 依次拼接，initcall_run_all() 顺序执行。
 * 规则：有依赖关系的初始化放在不同级别；同一级别内不要依赖先后顺序。 */
typedef struct { const char *name; int32_t (*fn)(void); } initcall_t;

#define INITCALL(_fn, _level)                                                        \
    const initcall_t initcall_##_fn                                                  \
        __attribute__((used, section(".initcall_" #_level ".init"))) = { #_fn, (_fn) }

#define INIT_BSP(fn)    INITCALL(fn, 0)   /* 外设驱动：gpio/uart/can/spi/pwm/usb */
#define INIT_COMP(fn)   INITCALL(fn, 1)   /* 组件：log 等                          */
#define INIT_CDD(fn)    INITCALL(fn, 2)   /* 设备驱动：遥控器、电机、IMU、灯、蜂鸣器 */
#define INIT_APP(fn)    INITCALL(fn, 3)   /* 机器人/应用                            */

typedef void (*initcall_report_fn)(const initcall_t *c, int32_t ret);
/* 执行整张表；返回失败个数；每个失败项调用一次 report（可为 NULL） */
int32_t initcall_run_all(initcall_report_fn report);
#endif
