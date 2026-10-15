#ifndef COMMON_INITCALL_H__
#define COMMON_INITCALL_H__
#include "component/common/types.h"

/* 自注册初始化表：模块用宏把 {名字, 函数} 放进 .initcall_<级别>.init，
 * 链接脚本按级别 0..7 依次拼接，initcall_run_all() 顺序执行。
 * 8 个阶段 = 4 个层次 x 2 个小阶段：BSP1 -> BSP2 -> COMP1 -> COMP2 -> CDD1 -> CDD2 -> APP1 -> APP2。
 * 规则：有先后依赖的初始化放在不同阶段；同一阶段内不要依赖先后顺序。
 * 本表在 framework_init 里先于周期任务启动执行完，所以 APP 阶段可以发第一批帧、设初值，周期任务起跑时这些都已就绪。 */typedef struct
{
    const char *name;
    int32_t   (*fn)(void);
} initcall_t;

#define INITCALL(_fn, _level)                                                        \
    const initcall_t initcall_##_fn                                                  \
        __attribute__((used, section(".initcall_" #_level ".init"))) = { #_fn, (_fn) }

/* 级别 0、1：外设驱动
 *   STAGE1：不依赖别的外设的、给后面提供通道的（uart 是日志和命令行的通道，usb）
 *   STAGE2：启动后会开始收发总线数据的外设（can、spi） */
#define INIT_BSP_STAGE1(fn)     INITCALL(fn, 0)
#define INIT_BSP_STAGE2(fn)     INITCALL(fn, 1)
/* 级别 2、3：组件
 *   STAGE1：日志（依赖 BSP 的 uart）
 *   STAGE2：依赖日志或别的组件的组件（命令行） */
#define INIT_COMP_STAGE1(fn)    INITCALL(fn, 2)
#define INIT_COMP_STAGE2(fn)    INITCALL(fn, 3)
/* 级别 4、5：设备驱动（遥控器、IMU、灯、蜂鸣器……）
 *   STAGE1：只依赖 BSP 的设备
 *   STAGE2：依赖同层其它设备的设备（目前没有） */
#define INIT_CDD_STAGE1(fn)     INITCALL(fn, 4)
#define INIT_CDD_STAGE2(fn)     INITCALL(fn, 5)
/* 级别 6、7：机器人 / 应用
 *   STAGE1：注册和绑定（电机绑定总线……），不发送任何东西
 *   STAGE2：依赖 STAGE1 已经绑定好的东西做初始动作（发第一批帧、开机提示……） */
#define INIT_APP_STAGE1(fn)     INITCALL(fn, 6)
#define INIT_APP_STAGE2(fn)     INITCALL(fn, 7)
typedef void (*initcall_report_fn)(const initcall_t *call, int32_t ret);
/* 执行整张表；返回失败个数；每个失败项调用一次 report（可为 NULL） */
int32_t initcall_run_all(initcall_report_fn report);
#endif
