#ifndef CDD_MOTOR_SHELL_H__
#define CDD_MOTOR_SHELL_H__
#include "cdd/motor/motor.h"

/* 给各电机的 shell 命令用的公共辅助函数（命令本身由使用电机的模块用 SHELL_CMD 注册）。
 * 只有 CONFIG_COMPONENT_SHELL = y 时才有内容，调用方也要用 #ifdef CONFIG_COMPONENT_SHELL 包起来。
 * 所有输出都用 shell_out，数值按"整数.三位小数"打印（newlib-nano 不带浮点 printf）。 */

/* 把文本解析成 float；整段文本都必须是数字，否则返回 false */
bool motor_shell_parse_float(const char *text, float *value);

/* 打印电机当前状态：在线、错误位、是否输出、控制模式、目标、角度、转速、转矩、输出、温度 */
void motor_shell_info(const char *name, const motor_t *motor);

/* PID 调参命令的实现：
 *   <命令>                              打印角度环和速度环的全部参数
 *   <命令> <angle|speed> <参数> <值>    修改一项，参数可以是 kp ki kd kf out int ff dead
 *                                         out = 输出限幅，int = 积分限幅，ff = 前馈限幅，dead = 死区
 * argv[0] 是命令名。返回 0 成功，-1 参数错误 */
int motor_shell_pid(const char *name, motor_t *motor, int argc, char *argv[]);
#endif
