// SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
// SPDX-License-Identifier: MIT
// 用户提供的六轴 C 移植及框架适配，来源和改动见 README.md。
/**
 * @file       vqf_filter.c
 * @brief      六轴 VQF：陀螺传播、倾斜修正与在线剩余零偏估计
 * @note       阅读顺序：VQF_Update → vqf_update → vqf_output。
 *
 * 姿态拆成 q = accQuat ⊗ gyrQuat，均使用 Hamilton [w,x,y,z]。
 * gyrQuat 用去偏角速度积分，将机体系转到陀螺积分的中间系；accQuat
 * 再把该中间系修正到导航系（静止加速度朝 +Z）。低通施加在旋转后的
 * 加速度上，陀螺传播本身不经过这层低通，因此 tauAcc 不是整体输出延迟。
 * 三维卡尔曼状态仅为机体系陀螺零偏；姿态不是这个卡尔曼滤波器的状态。
 * 无磁力计或外部航向观测，不能由重力恢复绝对 yaw。
 */
#include "component/ahrs/vqf/vqf_filter.h"
#include <float.h>
#include <math.h>
#include <string.h>

#define EPS FLT_EPSILON
#define M_PIf       3.14159265358979323846f
#define VQF_SQRT2   1.41421356237309504880
#define RAD2DEG     (180.0f / M_PIf)


static float vqf_square(float x) { return x * x; }
static float vqf_max(float a, float b) { return a > b ? a : b; }

static vqf_real_t vqf_norm(const vqf_real_t vec[], size_t N) {
    vqf_real_t s = 0;
    for(size_t i = 0; i < N; i++) s += vec[i]*vec[i];
    return sqrtf(s);
}

static void vqf_normalize(vqf_real_t vec[], size_t N) {
    vqf_real_t n = vqf_norm(vec, N);
    if (n < EPS) return;
    for(size_t i = 0; i < N; i++) vec[i] /= n;
}

static void vqf_clip(vqf_real_t vec[], size_t N, vqf_real_t min, vqf_real_t max) {
    for(size_t i = 0; i < N; i++) {
        if (vec[i] < min) vec[i] = min;
        else if (vec[i] > max) vec[i] = max;
    }
}

/* Hamilton 乘法：右侧旋转先作用；暂存结果允许 out 与任一输入重叠。 */
static void vqf_quatMultiply(const vqf_real_t q1[4], const vqf_real_t q2[4], vqf_real_t out[4]) {
    vqf_real_t w = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    vqf_real_t x = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    vqf_real_t y = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    vqf_real_t z = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
    out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}

/* 用单位四元数对应的 R(q) 旋转向量，等价于 q ⊗ [0,v] ⊗ q*。 */
static void vqf_quatRotate(const vqf_real_t q[4], const vqf_real_t v[3], vqf_real_t out[3]) {
    vqf_real_t x = (1 - 2*q[2]*q[2] - 2*q[3]*q[3])*v[0] + 2*v[1]*(q[2]*q[1] - q[0]*q[3]) + 2*v[2]*(q[0]*q[2] + q[3]*q[1]);
    vqf_real_t y = 2*v[0]*(q[0]*q[3] + q[2]*q[1]) + v[1]*(1 - 2*q[1]*q[1] - 2*q[3]*q[3]) + 2*v[2]*(q[2]*q[3] - q[1]*q[0]);
    vqf_real_t z = 2*v[0]*(q[3]*q[1] - q[0]*q[2]) + 2*v[1]*(q[0]*q[1] + q[3]*q[2]) + v[2]*(1 - 2*q[1]*q[1] - 2*q[2]*q[2]);
    out[0] = x; out[1] = y; out[2] = z;
}

/* 二阶 Butterworth 低通，截止频率 sqrt(2)/(2π tau)，经双线性变换离散化。
 * tau/Ts 均为秒；系数和递归状态保留 double，避免长时间常数下的精度损失。 */
static void filterCoeffs(vqf_real_t tau, vqf_real_t Ts, vqf_double_t outB[], vqf_double_t outA[]) {
    vqf_double_t fc = (VQF_SQRT2 / (2.0*M_PIf))/(vqf_double_t)(tau);
    vqf_double_t C = tan(M_PIf*fc*(vqf_double_t)(Ts));
    vqf_double_t D = C*C + VQF_SQRT2*C + 1;
    vqf_double_t b0 = C*C/D;
    outB[0] = b0; outB[1] = 2*b0; outB[2] = b0;
    outA[0] = 2*(C*C-1)/D;
    outA[1] = (1-VQF_SQRT2*C+C*C)/D;
}

/* 以常值 x0 的稳态初始化两个延迟项，避免从零启动产生虚假的重力过渡。 */
static void filterInitialState(vqf_real_t x0, const vqf_double_t b[3], const vqf_double_t a[2], vqf_double_t out[]) {
    out[0] = x0*(1 - b[0]);
    out[1] = x0*(b[2] - a[1]);
}

/* 沿用上游 filterAdaptStateForCoeffChange 的状态变换处理 dt 抖动。
 * 换系数时补偿两个延迟项，保持常值输入的稳态输出；启动均值阶段不变换。
 * 这是可变步长扩展，不表示与上游固定步长滤波逐位等价。 */
static void adaptFilter(const float last[], size_t n, const double oldB[3],
                        const double oldA[2], const double newB[3],
                        const double newA[2], double state[]) {
    if (isnan(state[0])) return;
    /* 所有通道共享系数变化量，在循环外计算；保留原来的运算顺序。 */
    const double deltaB0 = oldB[0]-newB[0];
    const double deltaB1A0 = oldB[1]-newB[1]-oldA[0]+newA[0];
    for (size_t i = 0; i < n; ++i) {
        const double value = last[i];
        state[2*i] += deltaB0*value;
        state[2*i+1] += deltaB1A0*value;
    }
}

void vqf_set_timestep(struct VQF_FILTER_t *f, float dt)
{
    /* dt 取相邻有效样本的实际时间差。同一步长直接复用，变化时倾斜、
     * 运动零偏和静止检测的低通必须一起调整，不能只改陀螺积分间隔。 */
    if (dt == f->gyrTs && dt == f->accTs) return;
    double b[3], a[2];
    filterCoeffs(f->tauAcc, dt, b, a);
    adaptFilter(f->lastAccLp, 3, f->accLpB, f->accLpA, b, a, f->accLpState);
    adaptFilter(f->motionLastR, 9, f->accLpB, f->accLpA, b, a, f->motionBiasEstRLpState);
    adaptFilter(f->motionLastBias, 2, f->accLpB, f->accLpA, b, a, f->motionBiasEstBiasLpState);
    memcpy(f->accLpB, b, sizeof(b));
    memcpy(f->accLpA, a, sizeof(a));
    filterCoeffs(f->restFilterTau, dt, b, a);
    adaptFilter(f->restLastGyrLp, 3, f->restGyrLpB, f->restGyrLpA, b, a, f->restGyrLpState);
    adaptFilter(f->restLastAccLp, 3, f->restAccLpB, f->restAccLpA, b, a, f->restAccLpState);
    memcpy(f->restGyrLpB, b, sizeof(b));
    memcpy(f->restAccLpB, b, sizeof(b));
    memcpy(f->restGyrLpA, a, sizeof(a));
    memcpy(f->restAccLpA, a, sizeof(a));
    /* P、过程噪声 V 和观测权重 W 使用同一内部尺度，K 中尺度相消。
     * W 随 dt 重算，以维持指定的零偏不确定度与遗忘时间。 */
    f->gyrTs = f->accTs = dt;
    f->biasV = vqf_square(0.1f*100.0f)*dt/f->biasForgettingTime;
    const float pMotion = vqf_square(f->biasSigmaMotion*100.0f);
    const float pRest = vqf_square(f->biasSigmaRest*100.0f);
    f->biasMotionW = vqf_square(pMotion)/f->biasV + pMotion;
    f->biasVerticalW = f->biasMotionW/vqf_max(f->biasVerticalForgettingFactor, 1e-10f);
    f->biasRestW = vqf_square(pRest)/f->biasV + pRest;
}

void vqf_break_continuity(struct VQF_FILTER_t *f)
{
    /* 保留 q、bias、P；缺测只打断静止证据和低通历史，不伪造缺口内的积分。
     * NAN 是 filterVec 的重新初始化标记，并非已输出的姿态异常。 */
    f->restDetected = false;
    f->restT = 0;
    for (size_t i=0; i<6; ++i) {
        f->accLpState[i] = f->restGyrLpState[i] = f->restAccLpState[i] = NAN;
    }
    for (size_t i=0; i<18; ++i) f->motionBiasEstRLpState[i] = NAN;
    for (size_t i=0; i<4; ++i) f->motionBiasEstBiasLpState[i] = NAN;
}

static void filterVec(const vqf_real_t x[], size_t N, vqf_real_t tau, vqf_real_t Ts, const vqf_double_t b[3],
                    const vqf_double_t a[2], vqf_double_t state[], vqf_real_t out[]) {
    /* 状态缓冲区复用：启动时 [0]=NAN、[1]=累计秒数、[2..]=积分和；
     * 满 tau 后转为每通道两个二阶延迟项。调用者仅使用 N>=2 的向量。 */
    if (isnan(state[0])) {
        if (isnan(state[1])) {
            state[1] = 0;
            for(size_t i = 0; i < N; i++) state[2+i] = 0;
        }
        /* 用实际经过时间加权，避免采样间隔变化时把每个样本等权处理。 */
        state[1] += Ts;
        for (size_t i = 0; i < N; i++) {
            state[2+i] += x[i] * (vqf_double_t)Ts;
            out[i] = state[2+i]/state[1];
        }
        if (state[1] >= tau) {
            for(size_t i = 0; i < N; i++) filterInitialState(out[i], b, a, state+2*i);
        }
        return;
    }
    /* 批量推进二阶低通：系数只读一次，避免每个通道的函数调用和重复加载。
     * 输入允许与输出重叠，先保存当前输入；状态和中间结果继续使用 double。
     * 不利用 b1=2*b0 改写算式，保持原低通的浮点运算顺序。 */
    const double b0=b[0], b1=b[1], b2=b[2], a0=a[0], a1=a[1];
    for (size_t i = 0; i < N; i++) {
        const double value = x[i];
        const double y = b0*value + state[2*i];
        state[2*i] = b1*value - a0*y + state[2*i+1];
        state[2*i+1] = b2*value - a1*y;
        out[i] = (float)y;
    }
}

/* 以下 3×3 矩阵按行存储；乘法先写临时数组，支持原位更新。 */
static void matrix3SetToScaledIdentity(vqf_real_t scale, vqf_real_t out[9]) {
    memset(out, 0, 9*sizeof(vqf_real_t));
    out[0] = out[4] = out[8] = scale;
}

static void matrix3Multiply(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9]) {
    vqf_real_t tmp[9];
    tmp[0] = in1[0]*in2[0] + in1[1]*in2[3] + in1[2]*in2[6]; tmp[1] = in1[0]*in2[1] + in1[1]*in2[4] + in1[2]*in2[7]; tmp[2] = in1[0]*in2[2] + in1[1]*in2[5] + in1[2]*in2[8];
    tmp[3] = in1[3]*in2[0] + in1[4]*in2[3] + in1[5]*in2[6]; tmp[4] = in1[3]*in2[1] + in1[4]*in2[4] + in1[5]*in2[7]; tmp[5] = in1[3]*in2[2] + in1[4]*in2[5] + in1[5]*in2[8];
    tmp[6] = in1[6]*in2[0] + in1[7]*in2[3] + in1[8]*in2[6]; tmp[7] = in1[6]*in2[1] + in1[7]*in2[4] + in1[8]*in2[7]; tmp[8] = in1[6]*in2[2] + in1[7]*in2[5] + in1[8]*in2[8];
    memcpy(out, tmp, sizeof(tmp));
}

/* out = in1ᵀ × in2。 */
static void matrix3MultiplyTpsFirst(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9]) {
    vqf_real_t tmp[9];
    tmp[0] = in1[0]*in2[0] + in1[3]*in2[3] + in1[6]*in2[6]; tmp[1] = in1[0]*in2[1] + in1[3]*in2[4] + in1[6]*in2[7]; tmp[2] = in1[0]*in2[2] + in1[3]*in2[5] + in1[6]*in2[8];
    tmp[3] = in1[1]*in2[0] + in1[4]*in2[3] + in1[7]*in2[6]; tmp[4] = in1[1]*in2[1] + in1[4]*in2[4] + in1[7]*in2[7]; tmp[5] = in1[1]*in2[2] + in1[4]*in2[5] + in1[7]*in2[8];
    tmp[6] = in1[2]*in2[0] + in1[5]*in2[3] + in1[8]*in2[6]; tmp[7] = in1[2]*in2[1] + in1[5]*in2[4] + in1[8]*in2[7]; tmp[8] = in1[2]*in2[2] + in1[5]*in2[5] + in1[8]*in2[8];
    memcpy(out, tmp, sizeof(tmp));
}

/* out = in1 × in2ᵀ。 */
static void matrix3MultiplyTpsSecond(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9]) {
    vqf_real_t tmp[9];
    tmp[0] = in1[0]*in2[0] + in1[1]*in2[1] + in1[2]*in2[2]; tmp[1] = in1[0]*in2[3] + in1[1]*in2[4] + in1[2]*in2[5]; tmp[2] = in1[0]*in2[6] + in1[1]*in2[7] + in1[2]*in2[8];
    tmp[3] = in1[3]*in2[0] + in1[4]*in2[1] + in1[5]*in2[2]; tmp[4] = in1[3]*in2[3] + in1[4]*in2[4] + in1[5]*in2[5]; tmp[5] = in1[3]*in2[6] + in1[4]*in2[7] + in1[5]*in2[8];
    tmp[6] = in1[6]*in2[0] + in1[7]*in2[1] + in1[8]*in2[2]; tmp[7] = in1[6]*in2[3] + in1[7]*in2[4] + in1[8]*in2[5]; tmp[8] = in1[6]*in2[6] + in1[7]*in2[7] + in1[8]*in2[8];
    memcpy(out, tmp, sizeof(tmp));
}

/* 伴随矩阵法；行列式接近零时返回失败，由调用者跳过本拍偏置校正。
 * 此处是绝对阈值检查，不是完整的条件数或协方差正定性检验。 */
static bool matrix3Inv(const vqf_real_t in[9], vqf_real_t out[9]) {
    vqf_double_t A = in[4]*in[8] - in[5]*in[7]; vqf_double_t D = in[2]*in[7] - in[1]*in[8]; vqf_double_t G = in[1]*in[5] - in[2]*in[4];
    vqf_double_t B = in[5]*in[6] - in[3]*in[8]; vqf_double_t E = in[0]*in[8] - in[2]*in[6]; vqf_double_t H = in[2]*in[3] - in[0]*in[5];
    vqf_double_t C = in[3]*in[7] - in[4]*in[6]; vqf_double_t F = in[1]*in[6] - in[0]*in[7]; vqf_double_t I = in[0]*in[4] - in[1]*in[3];
    vqf_double_t det = in[0]*A + in[1]*B + in[2]*C;
    if (det >= -EPS && det <= EPS) {
        memset(out, 0, 9*sizeof(vqf_real_t));
        return false;
    }
    /* 九个元素共享分母：保留 double 倒数与原来的奇异性检查，省去八次除法。
     * 最后才转换为 float；该变换允许末位舍入差，需做整条滤波轨迹对照。 */
    const double invDet = 1.0/det;
    out[0] = A*invDet; out[1] = D*invDet; out[2] = G*invDet;
    out[3] = B*invDet; out[4] = E*invDet; out[5] = H*invDet;
    out[6] = C*invDet; out[7] = F*invDet; out[8] = I*invDet;
    return true;
}


void vqf_init(struct VQF_FILTER_t *f, float dt)
{
    memset(f, 0, sizeof(*f));
    /* 保留原移植的参数基线。角速度类参数用 °/s，运行输入/bias 用 rad/s；
     * 修改参数需同步重建派生系数，不能只在线改一个字段就认为立即生效。 */
    f->tauAcc = 3.0f;
    f->motionBiasEstEnabled = true;
    f->restBiasEstEnabled = true;
    f->biasSigmaInit = 0.5f;
    f->biasForgettingTime = 100.0f;
    f->biasClip = 2.0f;
    f->biasSigmaMotion = 0.1f;
    f->biasVerticalForgettingFactor = 0.0001f;
    f->biasSigmaRest = 0.03f;
    f->restMinT = 1.5f;
    f->restFilterTau = 0.5f;
    f->restThGyr = 2.0f;
    f->restThAcc = 0.5f;

    // 初始采样间隔；后续每拍由框架入口更新为实际 dt。
    f->gyrTs = dt;
    f->accTs = dt;

    /* 协方差沿用上游尺度：(sigma[°/s] × 100)²。所有 P/V/W 同尺度，
     * 增益为无量纲；更新残差和最终 bias 仍使用 rad/s，不在这里换算状态。 */
    filterCoeffs(f->tauAcc, f->accTs, f->accLpB, f->accLpA);
    f->biasP0 = vqf_square(f->biasSigmaInit*100.0f);
    f->biasV = vqf_square(0.1f*100.0f) * f->accTs / f->biasForgettingTime;

    vqf_real_t pMotion = vqf_square(f->biasSigmaMotion*100.0f);
    f->biasMotionW = vqf_square(pMotion) / f->biasV + pMotion;
    f->biasVerticalW = f->biasMotionW / vqf_max(f->biasVerticalForgettingFactor, 1e-10f);

    vqf_real_t pRest = vqf_square(f->biasSigmaRest*100.0f);
    f->biasRestW = vqf_square(pRest) / f->biasV + pRest;

    filterCoeffs(f->restFilterTau, f->gyrTs, f->restGyrLpB, f->restGyrLpA);
    filterCoeffs(f->restFilterTau, f->accTs, f->restAccLpB, f->restAccLpA);

    // 重置状态
    f->gyrQuat[0] = 1; f->gyrQuat[1] = 0; f->gyrQuat[2] = 0; f->gyrQuat[3] = 0;
    f->accQuat[0] = 1; f->accQuat[1] = 0; f->accQuat[2] = 0; f->accQuat[3] = 0;
    f->restDetected = false;
    memset(f->lastAccLp, 0, sizeof(f->lastAccLp));
    memset(f->bias, 0, sizeof(f->bias));
    matrix3SetToScaledIdentity(f->biasP0, f->biasP);

    for (size_t i = 0; i < 6; i++) {
        f->accLpState[i] = NAN;
        f->restGyrLpState[i] = NAN;
        f->restAccLpState[i] = NAN;
    }
    for (size_t i = 0; i < 18; i++) f->motionBiasEstRLpState[i] = NAN;
    for (size_t i = 0; i < 4; i++) f->motionBiasEstBiasLpState[i] = NAN;
    for (size_t i = 0; i < 3; i++) {
        f->restLastGyrLp[i] = NAN;
        f->restLastAccLp[i] = 0.0f;
    }
    f->restLastSquaredDeviations[0] = f->restLastSquaredDeviations[1] = 0.0f;
    f->restT = 0.0f;

    f->pitch = 0; f->roll = 0; f->yaw = 0;
    f->yaw_laps = 0; f->last_yaw = 0; f->YawTotalAngle = 0;
    vqf_output(f);
}

void vqf_update(struct VQF_FILTER_t *f, float gx, float gy, float gz, float ax, float ay, float az)
{
    vqf_real_t gyr[3] = {gx, gy, gz};
    vqf_real_t acc[3] = {ax, ay, az};

    /* 1. 用原输入建立静止证据，再扣除在线 bias 积分。
     * 检测量是相对低通的变化和低通幅值，小而恒定的真实慢转仍可能被误判。
     * 固定标定已由驱动扣除，这里只扣 VQF 自己估计的剩余偏置。 */
    if (f->restBiasEstEnabled) {
        filterVec(gyr, 3, f->restFilterTau, f->gyrTs, f->restGyrLpB, f->restGyrLpA,
                  f->restGyrLpState, f->restLastGyrLp);
        f->restLastSquaredDeviations[0] = vqf_square(gyr[0] - f->restLastGyrLp[0])
                + vqf_square(gyr[1] - f->restLastGyrLp[1]) + vqf_square(gyr[2] - f->restLastGyrLp[2]);

        vqf_real_t biasClip = f->biasClip*(M_PIf/180.0f);
        if (f->restLastSquaredDeviations[0] >= vqf_square(f->restThGyr*(M_PIf/180.0f))
                || fabsf(f->restLastGyrLp[0]) > biasClip || fabsf(f->restLastGyrLp[1]) > biasClip
                || fabsf(f->restLastGyrLp[2]) > biasClip) {
            f->restT = 0.0;
            f->restDetected = false;
        }
    }

    /* 机体系角增量右乘到 gyrQuat，使用轴角的指数映射并归一化。 */
    vqf_real_t gyrNoBias[3] = {gyr[0]-f->bias[0], gyr[1]-f->bias[1], gyr[2]-f->bias[2]};
    vqf_real_t gyrNorm = vqf_norm(gyrNoBias, 3);
    vqf_real_t angle = gyrNorm * f->gyrTs;
    if (gyrNorm > EPS) {
        vqf_real_t c = cosf(angle/2);
        vqf_real_t s = sinf(angle/2)/gyrNorm;
        vqf_real_t gyrStepQuat[4] = {c, s*gyrNoBias[0], s*gyrNoBias[1], s*gyrNoBias[2]};
        vqf_quatMultiply(f->gyrQuat, gyrStepQuat, f->gyrQuat);
        vqf_normalize(f->gyrQuat, 4);
    }

    /* 2. 加速度全零是框架传入的“本拍无有效加速度”标记。
     * 陀螺已完成传播，此后倾斜校正、静止累计和偏置学习全部跳过。 */
    if (acc[0] == 0.0f && acc[1] == 0.0f && acc[2] == 0.0f) return;

    /* 陀螺异常先清空 restT，加速度异常也清空；连续满足条件才累积静止时间。
     * 这里没有操作者静止标志，和 MEKF 的显式静止声明不是同一种先验。 */
    if (f->restBiasEstEnabled) {
        filterVec(acc, 3, f->restFilterTau, f->accTs, f->restAccLpB, f->restAccLpA,
                  f->restAccLpState, f->restLastAccLp);
        f->restLastSquaredDeviations[1] = vqf_square(acc[0] - f->restLastAccLp[0])
                + vqf_square(acc[1] - f->restLastAccLp[1]) + vqf_square(acc[2] - f->restLastAccLp[2]);

        if (f->restLastSquaredDeviations[1] >= vqf_square(f->restThAcc)) {
            f->restT = 0.0;
            f->restDetected = false;
        } else {
            f->restT += f->accTs;
            if (f->restT >= f->restMinT) f->restDetected = true;
        }
    }

    /* 3. 先把加速度转到陀螺积分的中间系再低通，减少机体旋转带来的方向变化；
     * 随后用 accQuat 转到导航系，归一化后只使用方向，不使用重力模长。 */
    vqf_real_t accEarth[3];
    vqf_quatRotate(f->gyrQuat, acc, accEarth);
    filterVec(accEarth, 3, f->tauAcc, f->accTs, f->accLpB, f->accLpA, f->accLpState, f->lastAccLp);

    vqf_quatRotate(f->accQuat, f->lastAccLp, accEarth);
    vqf_normalize(accEarth, 3);

    /* 将滤波后方向转向导航 +Z 的最短倾斜旋转，左乘修正 accQuat。
     * 这不是绝对航向观测；接近 -Z 的对径情况选绕 X 轴 180°，避免除零。 */
    vqf_real_t accCorrQuat[4];
    accEarth[2] = fmaxf(-1.0f, fminf(1.0f, accEarth[2]));
    vqf_real_t q_w = sqrtf((accEarth[2]+1)/2);
    if (q_w > 1e-6f) {
        accCorrQuat[0] = q_w; accCorrQuat[1] = 0.5f*accEarth[1]/q_w;
        accCorrQuat[2] = -0.5f*accEarth[0]/q_w; accCorrQuat[3] = 0;
    } else {
        accCorrQuat[0] = 0; accCorrQuat[1] = 1; accCorrQuat[2] = 0; accCorrQuat[3] = 0;
    }
    vqf_quatMultiply(accCorrQuat, f->accQuat, f->accQuat);
    vqf_normalize(f->accQuat, 4);

    /* 倾斜校正角/秒，仅用于诊断，不是原始陀螺测量。 */
    f->lastAccCorrAngularRate = acosf(accEarth[2])/f->accTs;

    /* 4. 三维卡尔曼零偏估计。R 初始为机体→导航旋转矩阵；运动观测使用
     * 与倾斜通道相同低通后的 R 和水平 bias，保持观测与滤波历史一致。 */
    if (f->motionBiasEstEnabled || f->restBiasEstEnabled) {
        vqf_real_t biasClip = f->biasClip*(M_PIf/180.0f);
        vqf_real_t accGyrQuat[4], R[9], biasLp[2];

        vqf_quatMultiply(f->accQuat, f->gyrQuat, accGyrQuat);
        R[0] = 1 - 2*vqf_square(accGyrQuat[2]) - 2*vqf_square(accGyrQuat[3]);
        R[1] = 2*(accGyrQuat[2]*accGyrQuat[1] - accGyrQuat[0]*accGyrQuat[3]);
        R[2] = 2*(accGyrQuat[0]*accGyrQuat[2] + accGyrQuat[3]*accGyrQuat[1]);
        R[3] = 2*(accGyrQuat[0]*accGyrQuat[3] + accGyrQuat[2]*accGyrQuat[1]);
        R[4] = 1 - 2*vqf_square(accGyrQuat[1]) - 2*vqf_square(accGyrQuat[3]);
        R[5] = 2*(accGyrQuat[2]*accGyrQuat[3] - accGyrQuat[1]*accGyrQuat[0]);
        R[6] = 2*(accGyrQuat[3]*accGyrQuat[1] - accGyrQuat[0]*accGyrQuat[2]);
        R[7] = 2*(accGyrQuat[0]*accGyrQuat[1] + accGyrQuat[3]*accGyrQuat[2]);
        R[8] = 1 - 2*vqf_square(accGyrQuat[1]) - 2*vqf_square(accGyrQuat[2]);

        biasLp[0] = R[0]*f->bias[0] + R[1]*f->bias[1] + R[2]*f->bias[2];
        biasLp[1] = R[3]*f->bias[0] + R[4]*f->bias[1] + R[5]*f->bias[2];

        filterVec(R, 9, f->tauAcc, f->accTs, f->accLpB, f->accLpA, f->motionBiasEstRLpState, R);
        filterVec(biasLp, 2, f->tauAcc, f->accTs, f->accLpB, f->accLpA, f->motionBiasEstBiasLpState, biasLp);
        memcpy(f->motionLastR, R, sizeof(R));
        memcpy(f->motionLastBias, biasLp, sizeof(biasLp));

        /* e 为 rad/s 残差，w 为三轴观测噪声权重。
         * 静止优先：认为真实角速度为零，可直接观测三轴 bias，H=I。
         * 运动：倾斜校正提供水平约束；竖直项是趋零的弱先验，w[2] 很大，
         * 不能理解为六轴已观测到真实航向零偏。后续代码用 R 存放 H。 */
        vqf_real_t w[3], e[3];
        if (f->restDetected && f->restBiasEstEnabled) {
            e[0] = f->restLastGyrLp[0] - f->bias[0]; e[1] = f->restLastGyrLp[1] - f->bias[1]; e[2] = f->restLastGyrLp[2] - f->bias[2];
            matrix3SetToScaledIdentity(1.0, R);
            w[0] = w[1] = w[2] = f->biasRestW;
        } else if (f->motionBiasEstEnabled) {
            e[0] = -accEarth[1]/f->accTs + biasLp[0] - R[0]*f->bias[0] - R[1]*f->bias[1] - R[2]*f->bias[2];
            e[1] = accEarth[0]/f->accTs + biasLp[1] - R[3]*f->bias[0] - R[4]*f->bias[1] - R[5]*f->bias[2];
            e[2] = - R[6]*f->bias[0] - R[7]*f->bias[1] - R[8]*f->bias[2];
            w[0] = f->biasMotionW; w[1] = f->biasMotionW; w[2] = f->biasVerticalW;
        } else {
            w[0] = w[1] = w[2] = -1.0f;
        }

        /* 随机游走过程噪声恢复学习能力；仅在对角项低于初始方差时增加，
         * 不是每拍强制夹紧到 P0，可能有一次 V 的越过量。 */
        if (f->biasP[0] < f->biasP0) f->biasP[0] += f->biasV;
        if (f->biasP[4] < f->biasP0) f->biasP[4] += f->biasV;
        if (f->biasP[8] < f->biasP0) f->biasP[8] += f->biasV;

        if (w[0] >= 0) {
            /* 限制单拍残差，K = P Hᵀ (H P Hᵀ + diag(w))⁻¹。
             * 增益同时含运动/静止权重，不直接把观测值覆盖到 bias。 */
            vqf_clip(e, 3, -biasClip, biasClip);
            vqf_real_t K[9];
            /* 静止观测的 R 已设为单位阵：P*Rᵀ 和 R*P 都等于 P。
             * 只省略单位阵乘法，仍保留完整 3×3 协方差和相同逆矩阵求解。
             * 运动滤波历史照常推进，避免退出静止时使用陈旧低通状态。 */
            const bool identityObservation = f->restDetected && f->restBiasEstEnabled;
            if (identityObservation) memcpy(K, f->biasP, sizeof(K));
            else {
                matrix3MultiplyTpsSecond(f->biasP, R, K);
                matrix3Multiply(R, K, K);
            }
            K[0] += w[0]; K[4] += w[1]; K[8] += w[2];
            if (!matrix3Inv(K, K)) return;
            if (!identityObservation) matrix3MultiplyTpsFirst(R, K, K);
            matrix3Multiply(f->biasP, K, K);

            f->bias[0] += K[0]*e[0] + K[1]*e[1] + K[2]*e[2];
            f->bias[1] += K[3]*e[0] + K[4]*e[1] + K[5]*e[2];
            f->bias[2] += K[6]*e[0] + K[7]*e[1] + K[8]*e[2];

            /* 沿用原实现 P ← P - K H P；此处没有 Joseph 更新或正定投影。
             * 框架仅额外检查有限性，不能宣称所有病态输入下均数值稳定。 */
            if (!identityObservation) matrix3Multiply(K, R, K);
            matrix3Multiply(K, f->biasP, K);
            for(size_t i = 0; i < 9; i++) f->biasP[i] -= K[i];

            vqf_clip(f->bias, 3, -biasClip, biasClip);
        }
    }
}

void vqf_output(struct VQF_FILTER_t *f)
{
    /* 5. 合成机体→导航姿态，再一次性生成矩阵和欧拉角。
     * 本层使用标准 roll=X、pitch=Y、yaw=Z（度）；历史 INS 命名转换在 ins.c。
     * 欧拉角在俯仰 ±90° 附近有奇异性，完整姿态应读取 q。 */
    vqf_quatMultiply(f->accQuat, f->gyrQuat, f->q);

    float q0 = f->q[0], q1 = f->q[1], q2 = f->q[2], q3 = f->q[3];
    f->rMat[0][0] = 1 - 2*(q2*q2 + q3*q3);
    f->rMat[0][1] = 2*(q1*q2 - q0*q3);
    f->rMat[0][2] = 2*(q1*q3 + q0*q2);
    f->rMat[1][0] = 2*(q1*q2 + q0*q3);
    f->rMat[1][1] = 1 - 2*(q1*q1 + q3*q3);
    f->rMat[1][2] = 2*(q2*q3 - q0*q1);
    f->rMat[2][0] = 2*(q1*q3 - q0*q2);
    f->rMat[2][1] = 2*(q2*q3 + q0*q1);
    f->rMat[2][2] = 1 - 2*(q1*q1 + q2*q2);

    float sinp = 2.0f * (q0 * q2 - q1 * q3);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    f->pitch = asinf(sinp) * RAD2DEG;
    f->roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD2DEG;
    f->yaw = atan2f(2.0f * (q1 * q2 + q0 * q3), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD2DEG;

    /* 对相邻 yaw 差作 ±180° 回绕，得到连续航向；不是全局绝对航向。 */
    float yaw_diff = f->yaw - f->last_yaw;
    if (yaw_diff > 180.0f) {
        yaw_diff -= 360.0f;
        f->yaw_laps --;
    }
    else if (yaw_diff < -180.0f) {
        yaw_diff += 360.0f;
        f->yaw_laps ++;
    }
    f->YawTotalAngle += yaw_diff;
    f->last_yaw = f->yaw;
}

/* ---------- 框架入口：保护、连续性管理与统计 ---------- */
/* 候选状态仍完整复制、检查后才提交，不能为提速删掉数值失败回退。
 * 某些精简 C 库的通用 memcpy 是逐字节循环，大结构体赋值会落到该慢路径。
 * 固定长度的小块 memcpy 可由 GCC/armclang 展开成字访存；通过字节指针复制
 * 对象表示，避免把 float/double 强转成整数指针造成严格别名违规。
 * 不依赖结构体总长度，尾部仍正确复制；不改变全局 C 库或其他模块。 */
static void copy_filter(struct VQF_FILTER_t *dst, const struct VQF_FILTER_t *src)
{
    unsigned char *out = (unsigned char *)dst;
    const unsigned char *in = (const unsigned char *)src;
    size_t offset = 0;
    for (; offset + sizeof(uint64_t) <= sizeof(*dst); offset += sizeof(uint64_t)) {
        uint64_t block;
        memcpy(&block, in + offset, sizeof(block));
        memcpy(out + offset, &block, sizeof(block));
    }
    memcpy(out + offset, in + offset, sizeof(*dst) - offset);
}

static uint8_t finite_vector(const float *v, unsigned n)
{
    for (unsigned i=0; i<n; ++i) if (!isfinite(v[i])) return 0U;
    return 1U;
}

uint8_t VQF_Init(VQF_State *s, const float q0[4], float dt)
{
    /* 非法输入在写入前返回。合法初始化清空所有历史和计数；陀螺积分从
     * 单位四元数开始，归一化 q0 放入倾斜状态，组合后立即输出 q0。 */
    if (!s || !q0 || !finite_vector(q0,4) || !isfinite(dt) || dt<1e-5f || dt>0.02f) return 0U;
    float norm2=0;
    for (unsigned i=0; i<4; ++i) norm2+=q0[i]*q0[i];
    if (!isfinite(norm2) || norm2<1e-12f) return 0U;
    memset(s,0,sizeof(*s));
    vqf_init(&s->filter,dt);
    for (unsigned i=0; i<4; ++i) s->filter.accQuat[i]=q0[i]/sqrtf(norm2);
    vqf_output(&s->filter);
    s->initialized=1U;
    return 1U;
}

void VQF_BreakContinuity(VQF_State *s)
{
    if (!s || !s->initialized) return;
    vqf_break_continuity(&s->filter);
    s->status=VQF_INVALID_SAMPLE;
    ++s->continuity_breaks;
}

uint8_t VQF_Update(VQF_State *s, const float gyro[3], const float accel[3],
                   float dt, uint8_t flags)
{
    /* 入口边界：gyro 裁剪、非有限值或 dt 越界会拒绝整拍，不外推缺口；
     * 加速度异常则只停校正，仍允许 gyro 传播。加速度模长检查只是数值/
     * 量程保护，不能据此排除持续线加速度对倾斜和运动偏置学习的影响。 */
    if (!s) return VQF_INVALID_SAMPLE;
    if (!s->initialized || !gyro || !accel || !finite_vector(gyro,3) ||
        !isfinite(dt) || dt<1e-5f || dt>0.02f || (flags&VQF_GYRO_CLIPPED) ||
        fabsf(gyro[0])>35.0f || fabsf(gyro[1])>35.0f || fabsf(gyro[2])>35.0f) {
        ++s->rejected_count;
        VQF_BreakContinuity(s);
        return s->status=VQF_INVALID_SAMPLE;
    }
    const float norm2=accel[0]*accel[0]+accel[1]*accel[1]+accel[2]*accel[2];
    const uint8_t accel_ok=finite_vector(accel,3) && isfinite(norm2) &&
        norm2>1e-6f && norm2<240.0f*240.0f && !(flags&VQF_ACCEL_CLIPPED);
    /* 用局部候选状态运行；数值异常时保留上一拍姿态和零偏。
     * 局部状态约 0.8 KiB，仍计入 INS 任务栈需求。 */
    struct VQF_FILTER_t next;
    copy_filter(&next, &s->filter);
    vqf_set_timestep(&next,dt);
    if (!accel_ok) vqf_break_continuity(&next);
    vqf_update(&next,gyro[0],gyro[1],gyro[2],
               accel_ok?accel[0]:0,accel_ok?accel[1]:0,accel_ok?accel[2]:0);
    vqf_output(&next);
    if (!finite_vector(next.q,4) || !finite_vector(next.bias,3) ||
        !finite_vector(next.biasP,9) || !isfinite(next.yaw) ||
        !isfinite(next.pitch) || !isfinite(next.roll)) {
        ++s->numeric_fault_count;
        VQF_BreakContinuity(s);
        return s->status=VQF_NUMERIC_FAULT;
    }
    /* 校验成功后整体提交，避免新 q 搭配旧 bias/P；计数表示进入的路径，
     * 不能单凭 rest_bias_count/motion_bias_count 证明每拍都有非零偏置修正。 */
    copy_filter(&s->filter, &next);
    ++s->predict_count;
    if (accel_ok && next.restDetected && next.restBiasEstEnabled) ++s->rest_bias_count;
    else if (accel_ok && next.motionBiasEstEnabled) ++s->motion_bias_count;
    return s->status=VQF_PREDICTED|(accel_ok?VQF_ACCEL_UPDATED:VQF_ACCEL_REJECTED);
}
