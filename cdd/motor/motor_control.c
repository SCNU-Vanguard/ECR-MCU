#include "cdd/motor/motor.h"

/* motor_control_all 的实现：按厂家分发。放在单独的文件里，是因为它要同时依赖 DJI 和 DM 驱动，
 * 而这两个驱动又依赖 motor.c。各厂家用 CONFIG_CDD_MOTOR_xxx 开关，关掉的厂家不参与。 */
#ifdef CONFIG_CDD_MOTOR_DJI_MOTOR
#include "cdd/motor/dji_motor/dji_motor.h"
#endif
#ifdef CONFIG_CDD_MOTOR_DM_MOTOR
#include "cdd/motor/dm_motor/dm_motor.h"
#endif

static void (*output_hook)(void);

void motor_set_output_hook(void (*hook)(void))
{
    output_hook = hook;
}

int32_t motor_control_all(void)
{
    int32_t sent_count = 0;

#ifdef CONFIG_CDD_MOTOR_DJI_MOTOR
    dji_motor_compute_all();
#endif
#ifdef CONFIG_CDD_MOTOR_DM_MOTOR
    dm_motor_compute_all();
#endif
    if (output_hook != NULL)
    {
        output_hook();
    }
#ifdef CONFIG_CDD_MOTOR_DJI_MOTOR
    sent_count += dji_motor_send_all();
#endif
#ifdef CONFIG_CDD_MOTOR_DM_MOTOR
    sent_count += dm_motor_send_all();
#endif
    return sent_count;
}
