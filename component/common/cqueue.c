#include <string.h>
#include "component/common/cqueue.h"
#include "component/common/err.h"

int32_t cqueue_init(struct __cqueue *queue, void *buffer, uint32_t size, uint32_t element_size)
{
    if (size < 2u || (size & (size - 1u)) != 0u)    /* 非 2 的幂必须报错 */
    {
        return ERR_INVAL;
    }
    queue->read_index = 0u;
    queue->write_index = 0u;
    queue->mask = size - 1u;
    queue->element_size = element_size;
    queue->data = buffer;
    return ERR_OK;
}

uint32_t cqueue_used(const struct __cqueue *queue)
{
    return queue->write_index - queue->read_index;
}

uint32_t cqueue_unused(const struct __cqueue *queue)
{
    return (queue->mask + 1u) - cqueue_used(queue);
}

uint32_t cqueue_in(struct __cqueue *queue, const void *buffer, uint32_t length)
{
    uint32_t free_count = cqueue_unused(queue);

    if (length > free_count)
    {
        length = free_count;
    }
    for (uint32_t index = 0; index < length; index++)
    {
        memcpy((uint8_t *)queue->data + ((queue->write_index + index) & queue->mask) * queue->element_size,
               (const uint8_t *)buffer + index * queue->element_size,
               queue->element_size);
    }
    __atomic_thread_fence(__ATOMIC_RELEASE);          /* 先写数据，后推进下标 */
    queue->write_index += length;
    return length;
}

uint32_t cqueue_out(struct __cqueue *queue, void *buffer, uint32_t length)
{
    uint32_t used_count = cqueue_used(queue);

    if (length > used_count)
    {
        length = used_count;
    }
    for (uint32_t index = 0; index < length; index++)
    {
        memcpy((uint8_t *)buffer + index * queue->element_size,
               (uint8_t *)queue->data + ((queue->read_index + index) & queue->mask) * queue->element_size,
               queue->element_size);
    }
    __atomic_thread_fence(__ATOMIC_RELEASE);
    queue->read_index += length;
    return length;
}
