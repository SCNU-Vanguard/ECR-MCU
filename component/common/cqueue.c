#include <string.h>
#include "component/common/cqueue.h"
#include "component/common/err.h"

int32_t cqueue_init(struct __cqueue *q, void *buf, uint32_t size, uint32_t esize)
{
    if (size < 2u || (size & (size - 1u)) != 0u) { return ERR_INVAL; }   /* 非 2 的幂必须报错 */
    q->read_idx = q->write_idx = 0u;
    q->mask = size - 1u; q->esize = esize; q->data = buf;
    return ERR_OK;
}
uint32_t cqueue_used(const struct __cqueue *q)   { return q->write_idx - q->read_idx; }
uint32_t cqueue_unused(const struct __cqueue *q) { return (q->mask + 1u) - cqueue_used(q); }

uint32_t cqueue_in(struct __cqueue *q, const void *buf, uint32_t len)
{
    uint32_t n = cqueue_unused(q);
    if (len > n) { len = n; }
    for (uint32_t i = 0; i < len; i++) {
        memcpy((uint8_t *)q->data + ((q->write_idx + i) & q->mask) * q->esize,
               (const uint8_t *)buf + i * q->esize, q->esize);
    }
    __atomic_thread_fence(__ATOMIC_RELEASE);          /* 先写数据，后推进下标 */
    q->write_idx += len;
    return len;
}
uint32_t cqueue_out(struct __cqueue *q, void *buf, uint32_t len)
{
    uint32_t n = cqueue_used(q);
    if (len > n) { len = n; }
    for (uint32_t i = 0; i < len; i++) {
        memcpy((uint8_t *)buf + i * q->esize,
               (uint8_t *)q->data + ((q->read_idx + i) & q->mask) * q->esize, q->esize);
    }
    __atomic_thread_fence(__ATOMIC_RELEASE);
    q->read_idx += len;
    return len;
}
