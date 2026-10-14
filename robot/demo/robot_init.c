#include "robot/demo/robot_cfg.h"
#include "component/common/initcall.h"
#include "component/common/err.h"
#include "component/log/log.h"

#ifdef CONFIG_CDD_MOTOR_DJI
#include "cdd/motor/dji_motor/dji_motor.h"

dji_motor_t chassis_motor[4];

/* 电机绑定：具体电机怎么挂由兵种构型决定，开发者在这里手动写 */
static int32_t robot_bind_motors(void)
{
#ifdef CONFIG_ROBOT_BIND_CHASSIS
    for (int motor_index = 0; motor_index < 4; motor_index++)
    {
        int32_t register_ret = dji_motor_register(&chassis_motor[motor_index], CHASSIS_BUS,
                                                  (uint8_t)(CHASSIS_ID_BASE + motor_index), DJI_M3508);

        if (register_ret != ERR_OK)
        {
            LOGE("robot", "bind chassis[%d] failed: %d", motor_index, (int)register_ret);
            return register_ret;
        }
    }
#endif
    return ERR_OK;
}
INIT_APP(robot_bind_motors);
#endif
