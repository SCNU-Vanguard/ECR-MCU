#ifndef CDD_BMI088_CONFIG_H__
#define CDD_BMI088_CONFIG_H__

/* 标定表，一行一个轴（X、Y、Z 顺序），列含义：
 *   加速度计 scale（无量纲） | 加速度计 offset（m/s²） | 陀螺仪 offset（rad/s）
 * 加速度 = 灵敏度 * raw * scale - scale * offset；角速度 = 灵敏度 * raw - offset。
 * 标定结果（陀螺零偏 2026-10-04，约 38.5 °C 下静止标定）。
 * 换板子或温度差别大时需要重新标定，不能直接沿用。 */
#define BMI088_CALIBRATION_TABLE(X) \
    X(1.0108006001f, -0.0241884422f, 0.0031073433f) \
    X(1.0080075264f, 0.0630175471f, -0.0009981639f) \
    X(1.0091868639f, 0.0795006827f, -0.0011280888f)


#endif