#include "bsp/usb/bsp_usb.h"
#include "component/common/cqueue.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "osal/osal.h"
#include "usbd_cdc_if.h"            /* CubeIDE 生成：CDC_Transmit_HS、USBD_HandleTypeDef */

extern USBD_HandleTypeDef hUsbDeviceHS;   /* usb_device.c 里定义，没有头文件声明它 */

#define TX_CHUNK  512u

static uint8_t tx_queue_mem[2048];
static uint8_t tx_chunk[TX_CHUNK];          /* USB_OTG 没开 DMA（FIFO 模式），普通 RAM 即可 */
static struct __cqueue tx_queue;
static struct event_task *rx_event;
static volatile uint8_t tx_busy;

bool bsp_usb_connected(void)
{
    /* MX_USB_DEVICE_Init() 在 CubeIDE 的 defaultTask 里才执行，之前 pClassData 为 NULL，
     * 而 CDC_Transmit_HS() 会直接解引用它，所以必须先判断 */
    return hUsbDeviceHS.pClassData != NULL && hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED;
}

static void tx_kick(void)                   /* 调用时已屏蔽中断，或在 USB 中断里 */
{
    if (tx_busy || !bsp_usb_connected())
    {
        return;
    }
    uint32_t chunk_length = cqueue_out(&tx_queue, tx_chunk, TX_CHUNK);

    if (chunk_length == 0u)
    {
        return;
    }
    if (CDC_Transmit_HS(tx_chunk, (uint16_t)chunk_length) == USBD_OK)    /* 失败：这一块丢弃 */
    {
        tx_busy = 1u;
    }
}

int32_t bsp_usb_write(const void *data, uint32_t len)
{
    uint32_t irq_key = os_irq_save();
    uint32_t queued_length = cqueue_in(&tx_queue, data, len);

    tx_kick();
    os_irq_restore(irq_key);
    return (int32_t)queued_length;
}

void bsp_usb_bind_event(struct event_task *task)
{
    rx_event = task;
}

void bsp_usb_rx_isr(const uint8_t *buf, uint32_t len)
{
    if (rx_event != NULL && len <= 0xFFFFu)
    {
        (void)event_post_isr(rx_event, buf, (uint16_t)len);
    }
}

void bsp_usb_tx_done_isr(void)
{
    tx_busy = 0u;
    tx_kick();
}

static int32_t bsp_usb_init(void)
{
    return cqueue_init(&tx_queue, tx_queue_mem, sizeof tx_queue_mem, 1u);
}
INIT_BSP_STAGE1(bsp_usb_init);
