#include <stddef.h>
#include <string.h>
#include "dm_imu.h"
#include "component/common/cqueue.h"
#include "osal/event.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "component/crc/crc.h"
#include "osal/period.h"    
#include "bsp/uart/bsp_uart.h"
#include "bsp/tick/bsp_tick.h"


/*选择对应的模式，imu在使用前用usb连接上位机设置对应模式*/
#define y 1
#define n 0
#define DM_IMU_Mode_RS485_AUTO_TRANSMIT  n//使用主动模式()//ps主动模式无法使用485主动重启故不建议使用
#define DM_IMU_Mode_RS485_CORRESPONDING  n//使用应答模式()
#define DM_IMU_Mode_CAN_AUTO_TRANSMIT    y
#define DM_IMU_Mode_CAN_CORRESPONDING    n

#if (DM_IMU_Mode_RS485_CORRESPONDING == y || DM_IMU_Mode_RS485_AUTO_TRANSMIT == y) && (DM_IMU_Mode_CAN_AUTO_TRANSMIT == n && DM_IMU_Mode_CAN_CORRESPONDING == n)

    #define DM_IMU_RX_BLOCK_MAX        256u        /* 和 board.c 里 UART_DM_IMU 的接收缓冲一致 */
    #define DM_IMU_STREAM_SIZE         512u        /* 重组缓冲区大小 */

    #define DM_IMU_RID_ACCEL           0x00
    #define DM_IMU_RID_ANGULAR         0x01
    #define DM_IMU_RID_EULER           0x02
    #define DM_IMU_RID_QUATERNION      0x03
    #define DM_IMU_CRC8_INITIAL        0xFFu
    #define DM_IMU_CRC8_XOROUT         0xFFu
    #define DM_IMU_SOF                 0xA5
    #define DM_IMU_EOF                 0x5A
    #define DM_IMU_DM_IMU_FRAME_TOTAL_LENGTH   24u
    #define DM_IMU_OFFLINE_TIME_MS     500u

    #if DM_IMU_Mode_RS485_AUTO_TRANSMIT == y
        #define DM_SLAVE_ID                   0x0B        /* IMU的ID */
        #define DM_IMU_AT_ACK                 0x80       /* IMU应答帧类型 */
    #endif


    #if DM_IMU_Mode_RS485_CORRESPONDING == y
        #define DM_IMU_CORRESTPONDING_TYPE_CMD 0X0D
        #define DM_IMU_CORRESTPONDING_CMD_TYPE_READ 0x00
        #define DM_IMU_CORRESTPONDING_HEADER_LENGTH 5u

    #endif
    /*IMU应答模式帧定义
        * 帧头：0xA5 类型ID 0C/0D + 操作ID +寄存器id 
        + R/W + 数据段(0~3 bytes) + 应答 + 保留字节 + 5A    
        一共24字节
    */

    static uint32_t stream_count;// 当前字节流的长度下标
    static uint8_t  stream[DM_IMU_STREAM_SIZE] SRAM_NONCACHEABLE; //重组缓冲区
    static uint32_t bad_DM_IMU_Frame_count;//接收错误帧计数
    static dm_imu_frame_t DM_IMU_Frame;//定义完成的数据帧，
    static volatile uint32_t dm_imu_last_rx_time = 0;

    static void DM_imu_task_run(void *argument, const uint8_t *data, uint16_t len);
    static void DM_imu_parse_stream();
    static void DM_imu_dispatch(dm_imu_frame_t *DM_IMU_Frame);
    static void drop_bytes(uint32_t count);
    static uint8_t DM_IMU_crc8_calculate(const uint8_t *data, uint32_t length);
    static void dm_imu_watchdog(void *argument);

    EVENT_TASK_DEFINE(dm_imu_task, OS_PRIORITY_MEDIUM, OS_STACK_MEDIUM, EVENT_QUEUE, DM_IMU_RX_BLOCK_MAX, OS_QUEUE_LEN_4,
                    OS_TIMEOUT_1000MS, DM_imu_task_run, NULL);//定义事件任务

    PERIOD_TASK_DEFINE(dm_imu_watchdog_task,OS_FREQ_1HZ,OS_PRIORITY_MEDIUM,
                    OS_STACK_SMALL,dm_imu_watchdog,NULL);

    #if DM_IMU_Mode_RS485_CORRESPONDING == y
    static uint8_t dm_imu_next_correspondiog_state = IMU_STATE_ACCEL; //下一次发送时应答模式下的状态机，0-加速度，1-角速度，2-欧拉角，3-四元数
    static void dm_imu_get_frame(void *argument);
    PERIOD_TASK_DEFINE(dm_imu_periodic_task,OS_FREQ_200HZ,OS_PRIORITY_LOW,
                    OS_STACK_LARGE,dm_imu_get_frame,NULL);//定时发送应答帧
    
    /** @brief IMU事件任务函数
     * @param argument: 任务参数
     * @param data: 接收到的数据
     * @param len: 数据长度(超时为NULL NULL 0)
     */
    /**
     * @brief 获取应答模式下的IMU数据帧
     * @param argument: 任务参数
     * @param data: 接收到的数据
     * @param len: 数据长度(超时为NULL NULL 0)
     */
    static void dm_imu_get_frame(void *argument)
    {
        switch(dm_imu_next_correspondiog_state)
        {
        case IMU_STATE_ACCEL:
            {

                uint8_t cmd[24] = {0xA5 ,0x0D ,0x01 ,0x00,0x00 ,0x00,0x00, 0x00, 0x00,0x00 ,0x00, 0x00, 0x00, 0x00, 0x00, 0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x5A};
                cmd[22] = 127;
                bsp_uart_write(UART_DM_IMU, cmd, 24);
                dm_imu_next_correspondiog_state = IMU_STATE_ANGULAR;
                break;
            }
            case IMU_STATE_ANGULAR:
            {

                uint8_t cmd[24] = {0xA5 ,0x0D ,0x01 ,0x01,0x00 ,0x00,0x00, 0x00, 0x00,0x00 ,0x00, 0x00, 0x00, 0x00, 0x00, 0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x5A};
                cmd[22] = 54;
                bsp_uart_write(UART_DM_IMU, cmd, 24);
                dm_imu_next_correspondiog_state = IMU_STATE_EULER;
                break;
            }
            case IMU_STATE_EULER:
            {

                uint8_t cmd[24] = {0xA5 ,0x0D ,0x01 ,0x02,0x00 ,0x00,0x00, 0x00, 0x00,0x00 ,0x00, 0x00, 0x00, 0x00, 0x00, 0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x5A};
                cmd[22] = 237;
                bsp_uart_write(UART_DM_IMU, cmd, 24);
                dm_imu_next_correspondiog_state = IMU_STATE_QUATERNION;
                break;
            }
            case IMU_STATE_QUATERNION:
            {

                uint8_t cmd[24] = {0xA5 ,0x0D ,0x01 ,0x03,0x00 ,0x00,0x00, 0x00, 0x00,0x00 ,0x00, 0x00, 0x00, 0x00, 0x00, 0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x5A};
                cmd[22] = 164;
                bsp_uart_write(UART_DM_IMU, cmd, 24);
                dm_imu_next_correspondiog_state = IMU_STATE_ACCEL;
                break;
            }
        }
    }

    #endif

    //注：在主动模式下不能使用485重启,只有在应答模式下才能使用485重启，
    static void dm_imu_watchdog(void *argument)//掉线后重启imu
    {
        
        (void)argument;
        uint32_t now  = bsp_tick_ms();
        if(now - dm_imu_last_rx_time >= DM_IMU_OFFLINE_TIME_MS)
        {
            uint8_t reset_cmd[24] = {0xA5 ,0x0C ,0x01 ,0x00,0x01 ,0x00,0x00, 0x00, 0x00,0x00 ,0x00, 0x00, 0x00, 0x00, 0x00, 0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x5A};
            reset_cmd[22] = 0x2B;
            bsp_uart_write(UART_DM_IMU, reset_cmd, 24);//旧版本固件bug,上电后必须要重启一次后才有数据输出。
        }
    } 
    /** @brief 初始化IMU
     * @retval ERR_OK: 初始化成功
     * @retval 其他: 初始化失败
     */
    static int32_t dm_imu_init(void)
    {
        bsp_uart_bind_event(UART_DM_IMU, &dm_imu_task);
        uint8_t reset_cmd[24] = {0xA5 ,0x0C ,0x01 ,0x00,0x01 ,0x00,0x00, 0x00, 0x00,0x00 ,0x00, 0x00, 0x00, 0x00, 0x00, 0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x5A};
        reset_cmd[22] = 0x2B;
        bsp_uart_write(UART_DM_IMU, reset_cmd, 24);//旧版本固件bug,上电后必须要重启一次后才有数据输出。
        return ERR_OK;
    }
    INIT_CDD_STAGE1(dm_imu_init);//在初始化阶段1调用dm_imu_init函数




    static uint8_t DM_IMU_crc8_calculate(const uint8_t *data, uint32_t length)
    {
        uint8_t crc = DM_IMU_CRC8_INITIAL;

        for (uint32_t i = 0u; i < length; i++)
        {
            crc ^= data[i];
            for (uint32_t bit = 0u; bit < 8u; bit++)
            {
                crc = (crc & 0x80u) ? (uint8_t)((crc << 1u) ^ 0x1D) : (uint8_t)(crc << 1u);
            }
        }

        return (uint8_t)(crc ^ DM_IMU_CRC8_XOROUT);
    }

    static bool DM_IMU_CRC8_verify(const uint8_t *data, uint32_t length)
    {
        if (data == NULL || length != 24u)
        {
            return false;
        }
        return DM_IMU_crc8_calculate(data + 1u, 21u) == data[22u];
    }


    uint32_t error_counts = 0;
    static bool DM_IMU_Frame_is_correct()//判断收到的是否是完整且正确的帧
    {
        if(stream[23] == DM_IMU_EOF )//帧尾正确
        {
            uint8_t arrzero[16] = {0};
            if(memcmp(&stream[5],arrzero,16))
            {
                if(!DM_IMU_CRC8_verify(stream, DM_IMU_DM_IMU_FRAME_TOTAL_LENGTH))
                {
                    error_counts ++;
                    return false;
                }
                dm_imu_last_rx_time = bsp_tick_ms();
                return true;
            }
        }
    return false;
    }




    //事件任务函数
    static void DM_imu_task_run(void *argument, const uint8_t *data, uint16_t len)
    {
                (void)argument;
                if(data == NULL)
                {
                    bad_DM_IMU_Frame_count++;
                    stream_count = 0u;//清空流缓冲区
                    return;
                }
                if (len > DM_IMU_STREAM_SIZE - stream_count)   /* 防止len太大了，缓冲区装不下，正常不会发生（事件任务限制了块大小）；万一发生就丢掉旧数据重新同步 */
                {
                    stream_count = 0u;
                }
                memcpy(stream + stream_count , data ,len);//将接收到的数据拷贝到流缓冲区
                stream_count += len;//更新流缓冲区长度
                DM_imu_parse_stream();//解析流缓冲区
    }

    static void drop_bytes(uint32_t count)//将流缓冲区中前count个字节丢弃，并将剩余数据前移
    {
        stream_count -= count;
        memmove(stream, stream + count, stream_count);
    }

    static void DM_imu_parse_stream()
    {
        #if DM_IMU_Mode_RS485_AUTO_TRANSMIT != DM_IMU_Mode_RS485_CORRESPONDING 

            //主动模式,dma的数据进行组帧
            #if DM_IMU_Mode_RS485_AUTO_TRANSMIT == y
            while (stream_count > 0u )
            {

                // ps（是否可以使用查找算法加快速度）
                // 开始查找帧头
                if(stream_count < DM_IMU_DM_IMU_FRAME_TOTAL_LENGTH)
                {
                    return;//帧头还没收全
                }
                while(stream_count >= DM_IMU_DM_IMU_FRAME_TOTAL_LENGTH)
                {
                        if (stream[0] != DM_IMU_SOF)
                        {
                            uint32_t skip = 1u;

                            while (skip < stream_count && stream[skip] != DM_IMU_SOF)
                            {
                                skip++;
                            }
                            drop_bytes(skip);
                            continue;
                        }
                        if(DM_IMU_Frame_is_correct())
                        {
                            DM_imu_dispatch(&DM_IMU_Frame);//将解析好的数据帧传递出去
                            drop_bytes(DM_IMU_DM_IMU_FRAME_TOTAL_LENGTH);//丢掉已经解析完成的数据帧
                        }
                        else
                        {
                            drop_bytes(1u);
                        }
                    }
                }
                
            #endif

            #if DM_IMU_Mode_RS485_CORRESPONDING == y
                while(stream_count > 0u)
                {
                    if(stream[0] != DM_IMU_SOF)
                    {
                        uint32_t skip = 1u;
                        while (skip < stream_count && stream[skip] != DM_IMU_SOF)
                        {
                            skip++;
                        }
                        drop_bytes(skip);
                        continue;
                    }

                    if(stream_count<DM_IMU_DM_IMU_FRAME_TOTAL_LENGTH)
                    {                    
                        return;//帧还没收全
                    }
                    if(DM_IMU_Frame_is_correct())
                    {
                        DM_imu_dispatch(&DM_IMU_Frame);//将解析好的数据帧传递出去
                        drop_bytes(DM_IMU_DM_IMU_FRAME_TOTAL_LENGTH);//丢掉已经解析完成的数据帧
                    }
                    else
                    {
                        drop_bytes(1u);
                    }
                }
            #endif
        #endif
    return;
    }
    /** @brief 检查帧是否正确
     * @param DM_IMU_Frame: 待检查的帧
     * @retval true: 帧正确
     * @retval false: 帧错误
     */

    static void DM_imu_dispatch(dm_imu_frame_t *DM_IMU_Frame)
    {
        // 处理接收到的完整数据帧
        #if DM_IMU_Mode_RS485_AUTO_TRANSMIT == y
        uint32_t irq_key = os_irq_save();
        DM_IMU_Frame->id = stream[2];
        if(stream[3] ==  DM_IMU_RID_ACCEL)
        {
            for(int i = 0; i < 3 ; i++)
                memcpy(DM_IMU_Frame->accel + i ,&stream[5 + 4 * i],sizeof(float));
        }
        if(stream[3] ==  DM_IMU_RID_ANGULAR)
        {
            for(int i = 0; i < 3 ; i++)
                memcpy(DM_IMU_Frame->angular + i,&stream[5 + 4 * i],sizeof(float));
        }
        if(stream[3] ==  DM_IMU_RID_EULER)
        {
            for(int i = 0; i < 3 ; i++)
                memcpy(DM_IMU_Frame->euler + i,&stream[5 + 4 * i],sizeof(float));
        }
        if(stream[3] ==  DM_IMU_RID_QUATERNION)
        {
            for(int i = 0; i < 4 ; i++)
                memcpy(DM_IMU_Frame->quaternion + i,&stream[5 + 4 * i],sizeof(float));
        }
        os_irq_restore(irq_key);
        #endif

        #if DM_IMU_Mode_RS485_CORRESPONDING == y
        uint32_t irq_key = os_irq_save();
        if (stream[3] == DM_IMU_RID_ACCEL)
        {
            DM_IMU_Frame->id = stream[2];
            for(int i = 0; i < 3 ; i++)
                memcpy(&DM_IMU_Frame->accel[i], &stream[5 + i * 4], sizeof(float));
        }
        else if (stream[3] == DM_IMU_RID_ANGULAR)
        {
            DM_IMU_Frame->id = stream[2];
            for(int i = 0; i < 3 ; i++)
                memcpy(&DM_IMU_Frame->angular[i], &stream[5 + i * 4], sizeof(float));
        }
        else if (stream[3] == DM_IMU_RID_EULER)
        {
            DM_IMU_Frame->id = stream[2];
            for(int i = 0; i < 3 ; i++)
                memcpy(&DM_IMU_Frame->euler[i], &stream[5 + i * 4], sizeof(float));
        }
        else if (stream[3] == DM_IMU_RID_QUATERNION)
        {
            DM_IMU_Frame->id = stream[2];
            for(int i = 0; i < 4 ; i++)
                memcpy(&DM_IMU_Frame->quaternion[i], &stream[5 + i * 4], sizeof(float));
        }

        os_irq_restore(irq_key);
        #endif
    return;
    }

#endif

#if (DM_IMU_Mode_CAN_CORRESPONDING == y || DM_IMU_Mode_CAN_AUTO_TRANSMIT == y ) && (DM_IMU_Mode_RS485_AUTO_TRANSMIT == n && DM_IMU_Mode_RS485_CORRESPONDING == n)
    // can数据帧格式：data[0] 为1234分别对应加速度和温度，角速度，欧拉角，四元数。

    #include "bsp/can/bsp_can.h"

    #define DM_IMU_CAN_BUS  CAN_BUS2
    #define DM_IMU_CAN_ID_LOW          (0x00u)
    #define DM_IMU_CAN_ID_HIGH         (0xFFu)
    #define DM_IMU_STREAM_SIZE         512u//缓冲区
    #define DM_IMU_RX_BLOCK_MAX        256u        /* 和 board.c 里 UART_DM_IMU 的接收缓冲一致 */
    #define DM_IMU_OFFLINE_TIME_MS     500u
    #define DM_IMU_CAN_MST_ID          0x20
    #define DM_IMU_CAN_ID              0X001
    #define DM_IMU_DECODE_TYPE_ACCEL        1u
    #define DM_IMU_DECODE_TYPE_ANGULAR      2u
    #define DM_IMU_DECODE_TYPE_PITCH        3u
    #define DM_IMU_DECODE_TYPE_YAW_AND_ROLL 4u
    #define DM_IMU_DECODE_TYPE_QUATERNION   5u

    uint8_t dm_imu_can_buffer[DM_IMU_STREAM_SIZE];
    static dm_imu_frame_t DM_IMU_Frame;//定义完成的数据帧，
    static uint32_t dm_imu_last_rx_time = 0;
    float last_euler_value;
    static void DM_imu_can_task_run(void *argument, const uint8_t *data, uint16_t len);
    static void dm_imu_watchdog(void *argument);//掉线后重启imu


    EVENT_TASK_DEFINE(dm_imu_can_rx_task,OS_PRIORITY_MEDIUM,OS_STACK_MEDIUM,EVENT_QUEUE,DM_IMU_RX_BLOCK_MAX,OS_QUEUE_LEN_4,
                    OS_TIMEOUT_NONE,DM_imu_can_task_run,NULL);
    PERIOD_TASK_DEFINE(dm_imu_watchdog_task,OS_FREQ_1HZ,OS_PRIORITY_MEDIUM,
                    OS_STACK_SMALL,dm_imu_watchdog,NULL);

#if DM_IMU_Mode_CAN_CORRESPONDING == y
    static void dm_imu_get_frame(void* argument);


    PERIOD_TASK_DEFINE(dm_imu_periodic_task,OS_FREQ_200HZ,OS_PRIORITY_LOW,
                    OS_STACK_SMALL,dm_imu_get_frame,NULL);

    static uint8_t dm_imu_next_correspondiog_state = IMU_STATE_ACCEL;
    static can_frame_t dm_imu_get_data_cmd ={
                    .id = DM_IMU_CAN_ID,
                    .data = {0xCC,0x01,0x00,0xDD},
                    .len = 8
                    };

    static void dm_imu_get_frame(void* argument)
    {

            if(dm_imu_next_correspondiog_state == IMU_STATE_ACCEL)
            {
                dm_imu_get_data_cmd.data[1] = 0x01;
                bsp_can_send(DM_IMU_CAN_BUS,&dm_imu_get_data_cmd);
                dm_imu_next_correspondiog_state = IMU_STATE_ANGULAR;
            }
            if(dm_imu_next_correspondiog_state == IMU_STATE_ANGULAR)
            {
                dm_imu_get_data_cmd.data[1] = 0x02;
                bsp_can_send(DM_IMU_CAN_BUS,&dm_imu_get_data_cmd);
                dm_imu_next_correspondiog_state = IMU_STATE_EULER;
            }
            if(dm_imu_next_correspondiog_state == IMU_STATE_EULER)
            {
                dm_imu_get_data_cmd.data[1] = 0x03;
                bsp_can_send(DM_IMU_CAN_BUS,&dm_imu_get_data_cmd);
                dm_imu_next_correspondiog_state = IMU_STATE_QUATERNION;
            }
            if(dm_imu_next_correspondiog_state == IMU_STATE_QUATERNION)
            {
                dm_imu_get_data_cmd.data[1] = 0x04;
                bsp_can_send(DM_IMU_CAN_BUS,&dm_imu_get_data_cmd);
                dm_imu_next_correspondiog_state = IMU_STATE_ACCEL;

            }
    }
#endif

    static void dm_imu_watchdog(void *argument)//掉线后重启imu
    {
        
        uint32_t now  = bsp_tick_ms();
        if(now - dm_imu_last_rx_time >= DM_IMU_OFFLINE_TIME_MS)
        {
            can_frame_t restart_cmd = {
                .id = DM_IMU_CAN_ID,
                .len = 8,
                .data = {0xCC,0x00,0x01,0xDD}
            };
            bsp_can_send(DM_IMU_CAN_BUS,&restart_cmd);
        }
    }     

    static int32_t dm_imu_can_init(void)
    {
        return bsp_can_subscribe_event(CAN_BUS2,DM_IMU_CAN_ID_LOW , DM_IMU_CAN_ID_HIGH,&dm_imu_can_rx_task );
    }
    INIT_CDD_STAGE2(dm_imu_can_init);

    
    float dm_imu_uint16_to_float(uint16_t data,uint8_t decode_type)
    {
        if(decode_type == DM_IMU_DECODE_TYPE_ACCEL)
        {
            return (float)data/65535.0f * 470.4f - 235.2f;
        }
        if(decode_type == DM_IMU_DECODE_TYPE_ANGULAR)
        {
            return (float)data/65535.0f * 69.76f - 34.88f;
        }
        if(decode_type == DM_IMU_DECODE_TYPE_PITCH)
        {
            return (float)data/65535.0f * 180.0f - 90.0f;
        }
        if(decode_type == DM_IMU_DECODE_TYPE_YAW_AND_ROLL)
        {
            return (float)data/65535.0f * 360.0f - 180.0f;
        }
        return 0;
    }


        can_event_t event_data;
    static void DM_imu_can_task_run(void *argument, const uint8_t *data, uint16_t len)
    {
        event_data = *(can_event_t *)data;
        uint8_t id =((can_event_t *)data)->id;
        uint8_t *rx_data = ((can_event_t *)data)->data;
        uint8_t bus = (((can_event_t *)data)->bus);
        if(bus == DM_IMU_CAN_BUS && id == DM_IMU_CAN_MST_ID)
        {
                if(rx_data[0] == 1)
                {
                    for(int i = 0;i < 3 ;i++)//xyz填进去
                    {
                        DM_IMU_Frame.accel[i] = dm_imu_uint16_to_float(*(uint16_t*)(&rx_data[2] + 2*i),DM_IMU_DECODE_TYPE_ACCEL);
                    }
                }
                else if(rx_data[0] == 2)
                {
                    for(int i = 0;i < 3 ;i++)//xyz填进去
                    {
                        DM_IMU_Frame.angular[i] = dm_imu_uint16_to_float(*(uint16_t*)(&rx_data[2] + 2*i),DM_IMU_DECODE_TYPE_ANGULAR);
                    }
                }
                else if(rx_data[0] == 3)
                {
                    DM_IMU_Frame.euler[0] = dm_imu_uint16_to_float(*(uint16_t*)(&rx_data[2]),DM_IMU_DECODE_TYPE_PITCH);
                    for(int i = 1;i < 3 ;i++)//xyz填进去
                    {
                        DM_IMU_Frame.euler[i] = dm_imu_uint16_to_float(*(uint16_t*)(&rx_data[2] + 2*i),DM_IMU_DECODE_TYPE_YAW_AND_ROLL);
                    }
                }
                else if(rx_data[0] == 4)//四元数
                {
                        uint16_t temp0 = ((uint16_t)rx_data[1] |
                                          ((uint16_t)rx_data[2] << 8)) & 0x3FFFu;
                        DM_IMU_Frame.quaternion[0] = (float)temp0/16383.0f * 2.0f - 1.0f;

                        uint16_t temp1 = ((rx_data[2] >> 6) |
                                          ((uint16_t)rx_data[3] << 2) |
                                          ((uint16_t)(rx_data[4] & 0x0Fu) << 10)) & 0x3FFFu;
                        DM_IMU_Frame.quaternion[1] = (float)temp1/16383.0f * 2.0f - 1.0f;

                        uint16_t temp2 = ((rx_data[4] >> 4) |
                                          ((uint16_t)rx_data[5] << 4) |
                                          ((uint16_t)(rx_data[6] & 0x03u) << 12)) & 0x3FFFu;
                        DM_IMU_Frame.quaternion[2] = (float)temp2/16383.0f * 2.0f - 1.0f;

                        uint16_t temp3 = ((rx_data[6] >> 2) |
                                          ((uint16_t)rx_data[7] << 6)) & 0x3FFFu;
                        DM_IMU_Frame.quaternion[3] = (float)temp3/16383.0f * 2.0f - 1.0f;

                }

                if(last_euler_value !=  DM_IMU_Frame.euler[0])
                {
                    dm_imu_last_rx_time = bsp_tick_ms();
                }
                last_euler_value = DM_IMU_Frame.euler[0];
        } 
    }
#endif
