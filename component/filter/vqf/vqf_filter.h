// SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
// SPDX-License-Identifier: MIT
/**
 * @file vqf_filter.h
 * @brief VQF姿态滤波算法头文件
 */

#ifndef VQF_FILTER_H
#define VQF_FILTER_H

#include <stdbool.h>
#include <stdint.h>

/* 用户提供的六轴 C 移植，源于 Daniel Laidig 的 MIT VQF，见 README 和许可证。
 * 输入：rad/s、m/s²、秒；q 为 Hamilton wxyz，表示机体系到导航系。
 * bias 单位 rad/s；Sigma/Clip/ThGyr 参数沿用上游的 °/s 单位。
 * Euler 输出为度；INS 适配沿用 X=Pitch 的历史轴命名。
 * 本文件同时提供滤波状态和框架安全入口，使用方统一包含 vqf_filter.h。 */

typedef float vqf_real_t;
typedef double vqf_double_t;

/**
 * @brief 单实例的参数、派生系数、递归状态与输出（不依赖 HAL/RTOS）
 * @note 通过 VQF_Init 建立默认值，参数范围由调用方负责；若要调整时间常数/
 *       Sigma 等参数，应修改初始化配置并重新初始化，不能只改字段而沿用旧系数。
 */
struct VQF_FILTER_t {
    /* ================= 参数 ================= */
    vqf_real_t tauAcc;                  // 倾斜观测低通时间常数，s；不是整体姿态延迟
    bool motionBiasEstEnabled;         // 用运动中的倾斜修正估计剩余偏置
    bool restBiasEstEnabled;           // 自动静止检测及静止三轴偏置学习
    vqf_real_t biasSigmaInit;           // 初始偏置标准差，°/s
    vqf_real_t biasForgettingTime;      // 偏置方差随时间恢复的尺度，s
    vqf_real_t biasClip;                // 偏置/观测残差限幅及静止幅值边界，°/s
    vqf_real_t biasSigmaMotion;         // 运动偏置估计权重参数，°/s
    vqf_real_t biasVerticalForgettingFactor; // 竖直弱先验系数；越小，约束越弱
    vqf_real_t biasSigmaRest;           // 静止偏置估计权重参数，°/s
    vqf_real_t restMinT;                // 判定静止所需连续时间，s
    vqf_real_t restFilterTau;           // 静止检测的低通时间常数，s
    vqf_real_t restThGyr;               // 陀螺相对低通值的向量偏差阈值，°/s
    vqf_real_t restThAcc;               // 加速度相对低通值的向量偏差阈值，m/s²

    /* ================= 派生系数 ================= */
    vqf_real_t gyrTs;                   // 陀螺积分间隔，s
    vqf_real_t accTs;                   // 加速度更新间隔，框架中与 gyrTs 同步
    vqf_double_t accLpB[3];             // 二阶低通分子 b0,b1,b2
    vqf_double_t accLpA[2];             // 分母 a1,a2；首项 1 已省略
    vqf_real_t biasP0;                  // 初始方差；内部尺度为 (°/s × 100)²
    vqf_real_t biasV;                   // 每拍加入的过程噪声，与 biasP0 同尺度
    vqf_real_t biasMotionW;             // 水平运动观测权重，与 biasP0 同尺度
    vqf_real_t biasVerticalW;           // 竖直弱先验权重，较大值表示弱约束
    vqf_real_t biasRestW;               // 静止观测权重，与 biasP0 同尺度
    // 陀螺/加速度静止检测各自的二阶低通系数。
    vqf_double_t restGyrLpB[3];
    vqf_double_t restGyrLpA[2];
    vqf_double_t restAccLpB[3];
    vqf_double_t restAccLpA[2];

    /* ================= 递归状态 ================= */
    vqf_real_t gyrQuat[4];              // 机体→陀螺积分中间系，[w,x,y,z]
    vqf_real_t accQuat[4];              // 中间系→导航系，q = accQuat ⊗ gyrQuat
    bool restDetected;                 // 自动判定结果；不是外部确认的真实静止
    vqf_real_t lastAccLp[3];            // 中间系中的低通加速度，m/s²
    vqf_double_t accLpState[6];         // 三通道二阶状态；NAN 标记启动均值阶段
    vqf_real_t lastAccCorrAngularRate;  // 本拍倾斜校正角/dt，rad/s
    vqf_real_t bias[3];                 // 机体系在线剩余偏置，rad/s；不含驱动固定偏置
    vqf_real_t biasP[9];                // 三轴偏置协方差，行优先，沿用 biasP0 尺度
    vqf_double_t motionBiasEstRLpState[18]; // 旋转矩阵九通道低通状态
    vqf_double_t motionBiasEstBiasLpState[4]; // 导航水平偏置两通道低通状态
    vqf_real_t motionLastR[9];          // 上次滤波矩阵，用于变步长系数衔接
    vqf_real_t motionLastBias[2];        // 上次滤波水平偏置，rad/s
    vqf_real_t restLastSquaredDeviations[2]; // 陀螺/加速度偏差平方，(rad/s)²、(m/s²)²
    vqf_real_t restT;                   // 连续满足静止条件的秒数
    vqf_real_t restLastGyrLp[3];        // 机体系陀螺低通值，rad/s
    vqf_double_t restGyrLpState[6];     // 陀螺静止检测的三通道状态
    vqf_real_t restLastAccLp[3];        // 机体系加速度低通值，m/s²
    vqf_double_t restAccLpState[6];     // 加速度静止检测的三通道状态

    /* ================= 输出 ================= */
    float q[4];                        // 机体→导航，Hamilton [w,x,y,z]
    float pitch, roll, yaw;            // 度；此层 pitch=Y、roll=X，与历史 INS 命名不同
    float yaw_laps;                    // 跨越 ±180° 的累计圈数
    float last_yaw, YawTotalAngle;     // 上次包角 yaw 与连续累计 yaw，度
    float rMat[3][3];                  // 机体→导航的旋转矩阵
};

/* 底层接口不做完整输入检查；应用层优先使用下方大写 VQF_* 入口。
 * vqf_update 推进递归状态，vqf_output 才刷新 q/矩阵/欧拉角。 */
void vqf_init(struct VQF_FILTER_t *f, float dt);
void vqf_set_timestep(struct VQF_FILTER_t *f, float dt);
void vqf_break_continuity(struct VQF_FILTER_t *f);
void vqf_update(struct VQF_FILTER_t *f, float gx, float gy, float gz, float ax, float ay, float az);
void vqf_output(struct VQF_FILTER_t *f);

/* ---------- 带输入保护和诊断计数的框架接口 ---------- */
/* 输入饱和标志与 BMI088 / MEKF 一致。 */
enum { VQF_GYRO_CLIPPED=1U, VQF_ACCEL_CLIPPED=2U };
/* 输出是位掩码，可同时置位；ACCEL_UPDATED 仅说明执行了加速度校正路径，
 * 不表示每次偏置矩阵求逆都成功，也不等于有独立航向观测。 */
enum {
    VQF_PREDICTED=1U, VQF_ACCEL_UPDATED=2U, VQF_ACCEL_REJECTED=4U,
    VQF_INVALID_SAMPLE=8U, VQF_NUMERIC_FAULT=16U
};
typedef struct {
    struct VQF_FILTER_t filter;
    uint32_t predict_count, rest_bias_count, motion_bias_count; // 已提交传播/对应学习路径次数
    uint32_t rejected_count, numeric_fault_count, continuity_breaks; // 输入拒绝/数值失败/断连续次数
    uint8_t initialized, status;        // 是否初始化、最近一次输出状态位
} VQF_State;

/* 框架入口：不等待启动静止标定，q0 来自首帧有效加速度。
 * 在线零偏从 0 开始，估计驱动固定标定之后的剩余偏置；
 * 保留自动静止检测、静止/运动零偏估计、输入检查和数值失败回退。 */
uint8_t VQF_Init(VQF_State *s, const float q0[4], float dt);
/* dt 单位秒，合法范围 [1e-5, 0.02]；输入向量为 rad/s、m/s²。
 * gyro 无效拒绝整拍；accel 无效只传播 gyro；候选状态非有限时不提交。
 * 返回上述状态位掩码，最新有效结果保存在 s->filter。 */
uint8_t VQF_Update(VQF_State *s, const float gyro[3], const float accel[3],
                   float dt, uint8_t flags);
/* 缺测或拒绝帧打断自动静止窗口，不能跨缺口继续累计。 */
void VQF_BreakContinuity(VQF_State *s);


#endif /* VQF_FILTER_H */
