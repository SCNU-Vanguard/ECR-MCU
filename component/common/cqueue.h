#ifndef CQUEUE_H__
#define CQUEUE_H__
#include "component/common/types.h"
/* 单生产者/单消费者无锁环形队列：size（元素个数）必须是 2 的幂；读写下标自由增长，用 mask 取模 */
struct __cqueue
{
    uint32_t read_index;
    uint32_t write_index;
    uint32_t mask;
    uint32_t element_size;
    void    *data;
};

int32_t  cqueue_init(struct __cqueue *queue, void *buffer, uint32_t size, uint32_t element_size);
uint32_t cqueue_in(struct __cqueue *queue, const void *buffer, uint32_t length);    /* 返回实际入队个数 */
uint32_t cqueue_out(struct __cqueue *queue, void *buffer, uint32_t length);         /* 返回实际出队个数 */
uint32_t cqueue_used(const struct __cqueue *queue);
uint32_t cqueue_unused(const struct __cqueue *queue);
#endif
