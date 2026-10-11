#ifndef BSP_CAN_H__
#define BSP_CAN_H__
#include "bsp/include/bsp_hal.h"
#include "component/common/types.h"
#include "osal/event.h"
#include "bsp/include/bsp_ids.h"

typedef struct {                     /* 经典 CAN 标准帧 */
    uint32_t id;                     /* 11 位 */
    uint8_t  len;                    /* 0..8   */
    uint8_t  data[8];
} can_frame_t;

/* 底层接收回调：在 CAN 中断里调用，不能阻塞。应用层不要直接用，用下面的 bsp_can_subscribe_event */
typedef void (*can_rx_cb_t)(can_id_t bus, const can_frame_t *f, void *ctx);

/* 订阅 [id_lo, id_hi] 范围内的标准帧；每条总线最多 BSP_CAN_MAX_SUBS 个订阅。返回 ERR_xxx */
#define BSP_CAN_MAX_SUBS  16
int32_t bsp_can_subscribe(can_id_t bus, uint32_t id_lo, uint32_t id_hi, can_rx_cb_t cb, void *ctx);
/* 投递给事件任务的 CAN 消息：事件任务的 max_len 取 sizeof(can_event_t) */
typedef struct { uint8_t bus; uint8_t len; uint16_t id; uint8_t data[8]; } can_event_t;
/* 订阅 [id_lo, id_hi]，收到的帧在中断里拷贝成 can_event_t 投递给事件任务 t，任务里解码 */
int32_t bsp_can_subscribe_event(can_id_t bus, uint32_t id_lo, uint32_t id_hi, struct event_task *t);

/* 发送；TX FIFO 满返回 ERR_BUSY（不排队，由调用者决定丢弃还是重试） */
int32_t bsp_can_send(can_id_t bus, const can_frame_t *f);

typedef struct { uint32_t rx, tx, tx_full, bus_off, rx_unrouted; bool rx_enabled; } bsp_can_stats_t;
const bsp_can_stats_t *bsp_can_stats(can_id_t bus);

extern FDCAN_HandleTypeDef *const board_can[CAN_NUM];    /* 由 board.c 提供 */
#endif
