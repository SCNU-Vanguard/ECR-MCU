#ifndef CDD_DJI_MOTOR_H__
#define CDD_DJI_MOTOR_H__
#include "component/common/types.h"
#include "bsp/include/bsp_ids.h"

typedef enum { DJI_M3508, DJI_M2006, DJI_GM6020 } dji_type_t;

typedef struct {
    /* 绑定信息：由 dji_motor_register 填写 */
    can_id_t   bus;
    uint8_t    id;                 /* 拨码/ID：M3508、M2006 为 1..8，GM6020 为 1..7 */
    dji_type_t type;
    int16_t    cmd;                /* 待发送的电流（M3508/M2006）或电压（GM6020）原始值 */
    /* 反馈：在事件任务 dji_task 里更新（中断只拷贝帧） */
    uint16_t   angle;              /* 0..8191 */
    int16_t    speed_rpm;
    int16_t    current;
    uint8_t    temperature;
    int32_t    total_angle;        /* 多圈累计角度（编码器计数，8192 = 一圈） */
    int32_t    round_cnt;
    uint16_t   last_angle;
    bool       got_first;
    uint32_t   rx_ms;
} dji_motor_t;

#define DJI_MAX_MOTORS  16

/* 在 INIT_APP 阶段之前/之内调用。id/型号/总线冲突返回 ERR_BUSY，参数非法返回 ERR_INVAL */
int32_t dji_motor_register(dji_motor_t *m, can_id_t bus, uint8_t id, dji_type_t type);
void    dji_motor_set_cmd(dji_motor_t *m, int16_t cmd);
/* 把所有已注册电机的 cmd 按"总线 x 发送帧"打包发出；没有注册任何电机的帧不发。返回发出的帧数 */
int32_t dji_motor_send_all(void);
bool    dji_motor_online(const dji_motor_t *m, uint32_t timeout_ms);
/* 清零圈数，从下一帧反馈重新累计多圈角度 */
void    dji_motor_zero(dji_motor_t *m);
#endif
