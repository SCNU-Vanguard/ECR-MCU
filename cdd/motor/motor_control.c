#include "cdd/motor/motor.h"
#include "component/common/err.h"

/* 按总线计算/发送的分发：按电机型号交给 dji_motor / dm_motor。放在单独的文件里，是因为它要同时依赖这两个驱动，
 * 而这两个驱动又依赖 motor.c。各厂家用 CONFIG_CDD_MOTOR_xxx 开关，关掉的厂家不参与。 */
#ifdef CONFIG_CDD_MOTOR_DJI_MOTOR
#include "cdd/motor/dji_motor/dji_motor.h"
#endif
#ifdef CONFIG_CDD_MOTOR_DM_MOTOR
#include "cdd/motor/dm_motor/dm_motor.h"
#endif
#ifdef CONFIG_CDD_MOTOR_LK_MOTOR
#include "cdd/motor/lk_motor/lk_motor.h"
#endif

int32_t motor_compute(motor_t *motor)
{
    if (motor == NULL)
    {
        return ERR_INVAL;
    }
    switch (motor->type)
    {
#ifdef CONFIG_CDD_MOTOR_DJI_MOTOR
        case MOTOR_TYPE_M3508:
        case MOTOR_TYPE_M2006:
        case MOTOR_TYPE_GM6020:
            dji_motor_compute((dji_motor_t *)motor);
            return ERR_OK;
#endif
#ifdef CONFIG_CDD_MOTOR_DM_MOTOR
        case MOTOR_TYPE_J4310:
        case MOTOR_TYPE_J8009P:
        case MOTOR_TYPE_S3519:
            dm_motor_compute((dm_motor_t *)motor);
            return ERR_OK;
#endif
#ifdef CONFIG_CDD_MOTOR_LK_MOTOR
        case MOTOR_TYPE_MF9025:
            lk_motor_compute((lk_motor_t *)motor);
            return ERR_OK;
#endif
        default:
            return ERR_INVAL;
    }
}

int32_t motor_compute_bus(can_id_e bus)
{
    int32_t computed_count = 0;

    if ((unsigned)bus >= CAN_NUM)
    {
        return ERR_INVAL;
    }
#ifdef CONFIG_CDD_MOTOR_DJI_MOTOR
    computed_count += dji_motor_compute_bus(bus);
#endif
#ifdef CONFIG_CDD_MOTOR_DM_MOTOR
    computed_count += dm_motor_compute_bus(bus);
#endif
#ifdef CONFIG_CDD_MOTOR_LK_MOTOR
    computed_count += lk_motor_compute_bus(bus);
#endif
    return computed_count;
}

int32_t motor_send_bus(can_id_e bus)
{
    int32_t sent_count = 0;

    if ((unsigned)bus >= CAN_NUM)
    {
        return ERR_INVAL;
    }
#ifdef CONFIG_CDD_MOTOR_DJI_MOTOR
    sent_count += dji_motor_send_bus(bus);
#endif
#ifdef CONFIG_CDD_MOTOR_DM_MOTOR
    sent_count += dm_motor_send_bus(bus);
#endif
#ifdef CONFIG_CDD_MOTOR_LK_MOTOR
    sent_count += lk_motor_send_bus(bus);
#endif
    return sent_count;
}
