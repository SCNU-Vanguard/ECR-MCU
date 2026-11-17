#ifndef CHASSIS_CONFIG_H__
#define CHASSIS_CONFIG_H__

/* 全向轮，辊子 45°。轮子在矩形四角，轮序见 component/chassis/chassis.h。解算结果是轮端 rad/s。
 * 几何用半轴距、半轮距和轮半径描述，omni、mecanum、swerve 三种解算的几何结构体字段相同，
 * CHASSIS_GEOMETRY_INIT 可以直接初始化其中任何一种，由使用它的 appl 定义变量。
 *
 * CHASSIS_DRIVE_RATIO：驱动电机的速度反馈、速度目标相对轮端角速度的倍数。
 * 一台车的驱动电机是同一个型号，所以只有一个值：
 *   m3508：速度在转子侧，减速比 19，填 19。
 *   s3519：速度已经是输出轴侧，驱动层内部处理了减速比，填 1。
 *   直驱电机：填 1。 */
#define CHASSIS_HALF_LENGTH_M    0.30f    /* 几何中心到前/后轮轴 */
#define CHASSIS_HALF_WIDTH_M     0.30f    /* 几何中心到左/右轮 */
#define CHASSIS_WHEEL_RADIUS_M   0.10f
#define CHASSIS_DRIVE_RATIO      19.0f

#define CHASSIS_GEOMETRY_INIT \
    { CHASSIS_HALF_LENGTH_M, CHASSIS_HALF_WIDTH_M, CHASSIS_WHEEL_RADIUS_M }
#endif
