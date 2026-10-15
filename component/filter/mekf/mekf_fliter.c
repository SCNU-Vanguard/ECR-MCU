/*
 * 通用六轴 MEKF：给维护者的算法路线图
 * ====================================
 * 1. 为什么叫“乘法”：四元数有 4 个分量但只有 3 个旋转自由度。这里把
 *    单位四元数 q 放在名义状态里，Kalman 只估计小角度 dtheta 和零偏误差 db，
 *    dx=[dtheta_x,dtheta_y,dtheta_z,db_x,db_y,db_z]。不能把 dx 前三项
 *    直接加到 q 的虚部；要用 q <- Exp(dtheta_nav) * q 注入。
 *
 * 2. 约定：Hamilton 四元数 [w,x,y,z]，q 将机体系向量转到导航系。
 *    全局/左乘误差 q_true=Exp(dtheta_nav)*q，bias 仍在机体系。
 *    陀螺 rad/s，加速度 m/s^2，dt 秒。
 *    静止水平的加速度计比力为 +Z。加速度计的“重力读数”实际是比力，
 *    此处只用静止比力方向建模，不把它当导航系向下的物理重力向量。
 *
 * 3. 每拍数据流：
 *    驱动固定标定后的 gyro/accel + 实测 dt + 饱和标志
 *      -> 检查 gyro/dt；非法时整拍不积分
 *      -> predict：omega=gyro-bias，q=q*Exp(omega*dt)
 *         F=[0,-R;0,0]，Phi=[I,-dt*R_end*Jr(omega*dt);0,I]
 *         P=Phi*P*Phi^T+Qd（Qd 同时含角度、零偏和交叉项）
 *      -> 加速度模长检查；有效才归一化为 z
 *      -> correct_accel：h=R(q)^T*[0,0,1]，残差 z-h
 *         将残差投到 h 的二维切平面，H=T*[ R^T*[e_z]x,0 ]
 *      -> correct：S=H*P*H^T+R，NIS=r^T*S^-1*r 检测异常
 *         K=P*H^T*S^-1；重力校正的角度/零偏增益各去掉对应重力轴分量
 *         dx=K*r，Joseph 形式更新 P，再注入 q/bias
 *         注入后误差均值回到 0，P 用 J_left(dtheta_nav) 重置
 *      -> 可选 ZARU：调用方确认静止且连续满足门限才观测 gyro=bias
 *
 * 4. 能解决/不能解决什么：重力约束倾斜，gyro 积分相对航向；单个静止
 *    姿态下沿重力方向的零偏无法与真实慢转区分。多轴激励或明确静止的
 *    ZARU 可帮助估计该零偏，但没有磁力计/视觉/外部航向就没有绝对 yaw。
 *    模长和 NIS 门控也无法排除所有持续线加速度，不能用六轴姿态代替导航。
 *
 * 5. 调参顺序（先解决数据问题，再调滤波器）：
 *    a) 核对坐标/单位/量程/dt/标定/温度，记录静止与运动原始数据。
 *       姿态使用 unfiltered 旁路，避免串联驱动低通导致额外相位滞后。
 *    b) gyro_noise 是连续白噪声密度，内部 Qg=sigma_g^2*dt。
 *       增大 -> 传播不确定性增长更快、倾向加速度修正；减小 -> 更信 gyro。
 *       bias_walk 是零偏随机游走密度；增大 -> 温漂跟踪快但容易把运动
 *       污染学成零偏。先固定 gyro_noise，再小幅调整 bias_walk。
 *    c) accel_noise 是归一化方向的标准差，R=sigma_a^2*inflation。
 *       增大 -> 减弱加速度修正/降低噪声，但倾斜恢复慢；减小 -> 恢复快，
 *       更易受振动影响且 NIS 可能上升。不是旧 EKF 中无量纲的巨大 R。
 *       可从静止 a 的横向标准差除以 g 估算起点，振动下再留模型误差余量。
 *    d) accel_reject 管 |a|-g 的硬拒绝；accel_adapt 越小，同样模长偏差
 *       对 R 的放大越强。accel_nis_limit=9.21 对应理想二维高斯模型 99%；
 *       门限大 -> 接受更多异常。先检查 NIS/模长/拒绝率，不为让曲线好看
 *       就盲目提高门限。BMI088 带宽使相邻样本相关，统计意义只是近似。
 *    e) initial_*_std 只控制初始 P；大值帮助初始误差收敛，但不能补救
 *       任意错误初始姿态。启动加速度仅作初始倾斜猜测；不假定静止，
 *       持续线加速度下无法保证初始倾斜准确。
 *    f) stationary_* 是 ZARU 的附加门限，不是自动“静止证明”。默认
 *       不开启；只有应用确认静止才传 MEKF_STATIONARY。zero_rate_noise
 *       越小越相信零角速观测，误开静止标志时会错误消除真实慢转。
 *
 * 6. 实现策略：固定尺寸 float 数组、无动态内存、无硬件依赖；每个实例
 *    由一个任务调用。Cholesky 求解替代显式求逆；Joseph + 对称化 +
 *    切空间重置用于单精度稳定性。不能用 -ffast-math，否则 NaN 检查失效。
 *    默认参数只是可运行起点，必须用本板数据与独立角度基准验证精度。
 *
 * 论文与开源对照（独立编写，没有移植第三方源码）：
 * J. Sola, Quaternion kinematics for the error-state Kalman filter (2017),
 * https://arxiv.org/abs/1711.02508 第 7 节（全局角度误差）。
 * 王炜鑫，单矢量测量时如何使用 MEKF (2021):
 * https://zhuanlan.zhihu.com/p/449693543 。文中定理只讨论姿态；
 * 本实现的机体系零偏投影是额外的工程约束。
 * F. L. Markley, Attitude Error Representations for Kalman Filtering,
 * https://ntrs.nasa.gov/citations/20020060647 .
 * 工程实现对照：ethz-asl/ethzasl_msf（Joseph 更新和对称化），
 * hobbeshunter/IMU_EKF（状态/单位不同，未复制源码）。
 */
#include "component/filter/mekf/mekf_fliter.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#if MEKF_USE_CMSIS_DSP
#include "arm_math.h"
#endif

/* 矩阵使用自然对齐的 float 数组，不能传入 packed 传感器/INS 结构体。
 * 复制的源与目标不能重叠；保留通用 C99 主机支持。 */
static void copy_f32(float *dst, const float *src, unsigned count)
{
#if MEKF_USE_CMSIS_DSP
    arm_copy_f32(src,dst,count);
#else
    memcpy(dst,src,count*sizeof(float));
#endif
}
static void zero_f32(float *dst, unsigned count)
{
#if MEKF_USE_CMSIS_DSP
    arm_fill_f32(0.0f,dst,count);
#else
    memset(dst,0,count*sizeof(float));
#endif
}

static float dot3(const float a[3], const float b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static uint8_t finite3(const float v[3])
{
    return v && isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

static void skew(const float v[3], float s[9])
{
    s[0]=0; s[1]=-v[2]; s[2]=v[1];
    s[3]=v[2]; s[4]=0; s[5]=-v[0];
    s[6]=-v[1]; s[7]=v[0]; s[8]=0;
}

static uint8_t normalize_q(float q[4])
{
    float n = q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3];
    if (!isfinite(n) || n < 1e-12f) return 0;
    n = 1.0f/sqrtf(n);
    for (unsigned i=0; i<4; ++i) q[i] *= n;
    return 1;
}

/* q <- q * Exp(v)；Exp 将旋转矢量映射为单位四元数。 */
static void inject(float q[4], const float v[3])
{
    const float a2 = dot3(v,v);
    float c, s;
    if (a2 < 1e-6f) {
        c = 1.0f-a2/8.0f;
        s = 0.5f-a2/48.0f;
    } else {
        const float a = sqrtf(a2);
        c = cosf(0.5f*a);
        s = sinf(0.5f*a)/a;
    }
    const float w=q[0], x=q[1], y=q[2], z=q[3];
    q[0]=w*c-s*(x*v[0]+y*v[1]+z*v[2]);
    q[1]=x*c+s*(w*v[0]+y*v[2]-z*v[1]);
    q[2]=y*c+s*(w*v[1]+z*v[0]-x*v[2]);
    q[3]=z*c+s*(w*v[2]+x*v[1]-y*v[0]);
    (void)normalize_q(q);
}

/* 机体系到导航系的旋转矩阵，按行存储。 */
static void rotation_matrix(const float q[4], float r[9])
{
    const float w=q[0], x=q[1], y=q[2], z=q[3];
    r[0]=1-2*(y*y+z*z); r[1]=2*(x*y-w*z); r[2]=2*(x*z+w*y);
    r[3]=2*(x*y+w*z); r[4]=1-2*(x*x+z*z); r[5]=2*(y*z-w*x);
    r[6]=2*(x*z-w*y); r[7]=2*(y*z+w*x); r[8]=1-2*(x*x+y*y);
}

/* Exp(dtheta_nav) * q = q * Exp(R(q)^T dtheta_nav)。
 * 陀螺角速度在机体系中测量，因此预测仍采用右乘。 */
static void inject_global(float q[4], const float v[3])
{
    float r[9], local[3]={0};
    rotation_matrix(q,r);
    for (unsigned i=0; i<3; ++i) for (unsigned j=0; j<3; ++j)
        local[i]+=r[3*j+i]*v[j];
    inject(q,local);
}

/* SO(3) 右雅可比；小角度分支避免相近数相减导致精度损失。 */
static void right_jacobian(const float v[3], float jr[9])
{
    const float a2=dot3(v,v);
    float s[9], c2, c3;
    skew(v,s);
    if (a2 < 1e-4f) {
        c2=0.5f-a2/24.0f+a2*a2/720.0f;
        c3=1.0f/6.0f-a2/120.0f+a2*a2/5040.0f;
    } else {
        const float a=sqrtf(a2);
        const float c1=sinf(a)/a;
        c2=(1.0f-cosf(a))/a2;
        c3=(1.0f-c1)/a2;
    }
    for (unsigned i=0; i<3; ++i) for (unsigned j=0; j<3; ++j) {
        const float identity=(i==j) ? 1.0f : 0.0f;
        const float s2=v[i]*v[j]-identity*a2;
        jr[3*i+j]=identity-c2*s[3*i+j]+c3*s2;
    }
}

/* 重力/航向观测满足 H=[H_theta 0]，所以 I-KH=[T 0; B I]。
 * 用 6×3 数组 a 存储前三列，完整计算 A P A^T，保留两侧交叉块；
 * 乘法次数由 432 次减为 216 次。保留 Joseph 合同变换形式，
 * 不改成 P-KHP 或低秩相减。dst 不能与 a、P 共用存储。 */
static void attitude_observation_covariance(const float a[18], const float P[36], float dst[36])
{
    float ap[36];
    for (unsigned i=0; i<6; ++i) for (unsigned j=0; j<6; ++j) {
        float sum=0;
        for (unsigned k=0; k<3; ++k) sum+=a[3*i+k]*P[6*k+j];
        if (i>=3) sum+=P[6*i+j];
        ap[6*i+j]=sum;
    }
    for (unsigned i=0; i<6; ++i) for (unsigned j=0; j<6; ++j) {
        float sum=0;
        for (unsigned k=0; k<3; ++k) sum+=ap[6*i+k]*a[3*j+k];
        if (j>=3) sum+=ap[6*i+j];
        dst[6*i+j]=sum;
    }
}

/* Phi=[I B;0 I]。保留两侧交叉块，不近似状态转移。
 * 相比稠密 6×6 双侧乘法，乘法次数由 432 次减为 108 次；输入不能与 dst 重叠。 */
static void propagate_covariance(const float b[9], const float p[36], float dst[36])
{
    float ap[36];
    copy_f32(ap+18,p+18,18);
    for (unsigned i=0; i<3; ++i) for (unsigned j=0; j<6; ++j) {
        float sum=p[6*i+j];
        for (unsigned k=0; k<3; ++k) sum+=b[3*i+k]*p[6*(k+3)+j];
        ap[6*i+j]=sum;
    }
    for (unsigned i=0; i<6; ++i) for (unsigned j=0; j<3; ++j) {
        float sum=ap[6*i+j];
        for (unsigned k=0; k<3; ++k) sum+=ap[6*i+k+3]*b[3*j+k];
        dst[6*i+j]=sum;
        dst[6*i+j+3]=ap[6*i+j+3];
    }
}

/* A 除偏移 0 或 3 处的 3×3 对角块外均为单位阵。
 * 用于姿态重置和仅更新零偏的 Joseph 计算，保留所有交叉项。
 * 乘法次数由 432 次减为 108 次；输入不能与 dst 重叠。 */
static void block_covariance(const float block[9], unsigned offset,
                             const float p[36], float dst[36])
{
    float ap[36];
    const unsigned unchanged=3-offset;
    copy_f32(ap+6*unchanged,p+6*unchanged,18);
    for (unsigned i=0; i<3; ++i) for (unsigned j=0; j<6; ++j) {
        float sum=0;
        for (unsigned k=0; k<3; ++k) sum+=block[3*i+k]*p[6*(offset+k)+j];
        ap[6*(offset+i)+j]=sum;
    }
    for (unsigned i=0; i<6; ++i) for (unsigned j=0; j<3; ++j) {
        float sum=0;
        for (unsigned k=0; k<3; ++k) sum+=ap[6*i+offset+k]*block[3*j+k];
        dst[6*i+offset+j]=sum;
        dst[6*i+unchanged+j]=ap[6*i+unchanged+j];
    }
}

static uint8_t condition_covariance(float p[36])
{
    for (unsigned i=0; i<36; ++i) if (!isfinite(p[i])) return 0;
    for (unsigned i=0; i<6; ++i) {
        if (p[7*i] < -1e-8f) return 0;
        if (p[7*i] < 1e-12f) p[7*i]=1e-12f;
        for (unsigned j=i+1; j<6; ++j) {
            const float mean=0.5f*(p[6*i+j]+p[6*j+i]);
            p[6*i+j]=p[6*j+i]=mean;
        }
    }
    return 1;
}

MEKF_Config MEKF_DefaultConfig(void)
{
    /* 用于启动的默认参数，不是 BMI088 实测噪声或标定结果。 */
    const MEKF_Config c={
        .gyro_noise=0.003f, .bias_walk=0.0001f, .accel_noise=0.035f,
        .initial_angle_std=0.35f, .initial_bias_std=0.003f,
        .gravity=9.81f, .accel_reject=1.5f, .accel_adapt=0.3f,
        .accel_nis_limit=9.21f, .max_dt=0.02f, .max_gyro=35.0f,
        .bias_limit=0.15f, .stationary_gyro=0.05f, .stationary_accel=0.2f,
        .stationary_time=0.5f, .zero_rate_noise=0.01f
    };
    return c;
}

static uint8_t valid_config(const MEKF_Config *c)
{
#define VALID(field, lo, hi) \
    if (!isfinite(c->field) || c->field < (lo) || c->field > (hi)) return 0
    VALID(gyro_noise,1e-6f,1.0f); VALID(bias_walk,0.0f,0.1f);
    VALID(accel_noise,1e-4f,1.0f); VALID(initial_angle_std,1e-3f,1.0f);
    VALID(initial_bias_std,1e-5f,1.0f); VALID(gravity,1.0f,20.0f);
    VALID(accel_reject,0.01f,10.0f); VALID(accel_adapt,0.01f,10.0f);
    VALID(accel_nis_limit,1.0f,100.0f); VALID(max_dt,0.0001f,0.02f);
    VALID(max_gyro,0.1f,100.0f); VALID(bias_limit,0.001f,1.0f);
    VALID(stationary_gyro,0.001f,0.5f); VALID(stationary_accel,0.01f,2.0f);
    VALID(stationary_time,0.1f,30.0f); VALID(zero_rate_noise,1e-4f,0.5f);
#undef VALID
    return 1;
}

uint8_t MEKF_Init(MEKF_State *f, const float q[4], const float bias[3],
                  const MEKF_Config *config)
{
    if (!f) return 0;
    /* 先复制输入：调用方可能用同一实例的字段重新初始化该实例。 */
    MEKF_Config c=config ? *config : MEKF_DefaultConfig();
    float initial_q[4]={1,0,0,0}, initial_bias[3]={0};
    if (q) memcpy(initial_q,q,sizeof(initial_q));
    if (bias) memcpy(initial_bias,bias,sizeof(initial_bias));
    memset(f,0,sizeof(*f));
    if (!valid_config(&c) || !normalize_q(initial_q) || !finite3(initial_bias)) return 0;
    for (unsigned i=0; i<3; ++i) if (fabsf(initial_bias[i])>c.bias_limit) return 0;
    f->config=c;
    memcpy(f->q,initial_q,sizeof(initial_q));
    memcpy(f->bias,initial_bias,sizeof(initial_bias));
    for (unsigned i=0; i<3; ++i) {
        f->P[7*i]=c.initial_angle_std*c.initial_angle_std;
        f->P[7*(i+3)]=c.initial_bias_std*c.initial_bias_std;
    }
    f->accel_nis=f->heading_nis=-1;
    f->initialized=1;
    return 1;
}

static uint8_t predict(MEKF_State *f, const float gyro[3], float dt)
{
    /* 预测：先算候选 P，确认数值正常后再发布 q/P。输入是驱动校准后的值，
     * 此处仅减 MEKF 估计的残余 bias。Exp 积分在单拍角速度恒定时精确。 */
    float v[3], jr[9], bend[9], p[36], next_q[4], r[9];
    for (unsigned i=0; i<3; ++i) v[i]=(gyro[i]-f->bias[i])*dt;
    memcpy(next_q,f->q,sizeof(next_q));
    inject(next_q,v);
    rotation_matrix(next_q,r);
    right_jacobian(v,jr);
    for (unsigned i=0; i<3; ++i) for (unsigned j=0; j<3; ++j) {
        float sum=0;
        for (unsigned k=0; k<3; ++k) sum-=dt*r[3*i+k]*jr[3*k+j];
        bend[3*i+j]=sum;
    }
    propagate_covariance(bend,f->P,p);
    const float qg=f->config.gyro_noise*f->config.gyro_noise;
    const float qb=f->config.bias_walk*f->config.bias_walk;
    /* 导航系角误差 / 机体系零偏：Phi_theta,theta=I，
     * Phi_theta,b=-integral R(s) ds=-dt R_end Jr(w dt)。
     * dt-t 时刻的零偏随机游走脉冲对应 B(t)=-t R_end Jr(w t)。
     * 正权 Simpson 积分保持 Q 半正定；qg 是连续噪声功率密度，
     * 因此 Qg=qg*dt，不是文章中离散样本噪声的 Q*dt^2。 */
    for (unsigned i=0; i<3; ++i) {
        p[7*i]+=qg*dt;
        p[7*(i+3)]+=qb*dt;
        v[i]*=0.5f;
    }
    for (unsigned node=0; node<2; ++node) {
        const float t=(node==0) ? 0.5f*dt : dt;
        const float weight=qb*dt*((node==0) ? 4.0f/6.0f : 1.0f/6.0f);
        float b[9];
        if (node==0) {
            right_jacobian(v,jr);
            for (unsigned i=0; i<3; ++i) for (unsigned j=0; j<3; ++j) {
                float sum=0;
                for (unsigned k=0; k<3; ++k) sum-=t*r[3*i+k]*jr[3*k+j];
                b[3*i+j]=sum;
            }
        } else memcpy(b,bend,sizeof(b)); /* 复用相同的积分端点。 */
        for (unsigned i=0; i<3; ++i) for (unsigned j=0; j<3; ++j) {
            float bbt=0;
            for (unsigned k=0; k<3; ++k) bbt+=b[3*i+k]*b[3*j+k];
            p[6*i+j]+=weight*bbt;
            p[6*i+j+3]+=weight*b[3*i+j];
            p[6*(j+3)+i]+=weight*b[3*i+j];
        }
    }
    if (!condition_covariance(p)) return 0;
    memcpy(f->q,next_q,sizeof(next_q));
    copy_f32(f->P,p,36);
    ++f->predict_count;
    return 1;
}

/* 用 Cholesky 分解求解 S*x=b，维数为 1～3，不显式求逆。 */
static void solve(const float l[9], const float inv_diag[3], const float b[3], float x[3], unsigned n)
{
    float y[3]={0};
    for (unsigned i=0; i<n; ++i) {
        float s=b[i];
        for (unsigned j=0; j<i; ++j) s-=l[3*i+j]*y[j];
        y[i]=s*inv_diag[i];
    }
    for (int i=(int)n-1; i>=0; --i) {
        float s=y[i];
        for (unsigned j=(unsigned)i+1; j<n; ++j) s-=l[3*j+i]*x[j];
        x[i]=s*inv_diag[i];
    }
}

static uint8_t correct(MEKF_State *f, const float h[18], const float residual[3],
                       float variance, unsigned n, float nis_limit, float *nis,
                       const float unobservable_axis[3], uint8_t bias_only)
{
    /* 通用观测更新：n=1 外部航向，n=2 重力方向，n=3 明确静止的零角速观测。
     * 拒绝观测只退出校正，前面的陀螺预测仍保留，禁止遇到冲击就冻住姿态。 */
    float pht[18], l[9], inv_diag[3], gain[18], dx[6];
    /* 调用方的 H 只有两种结构：重力/航向为 [H_theta 0]，ZARU 为 [0 I]。
     * 跳过结构上确定为零或单位阵的乘法。 */
    for (unsigned i=0; i<6; ++i) for (unsigned j=0; j<n; ++j) {
        if (bias_only) pht[3*i+j]=f->P[6*i+j+3];
        else {
            float sum=0;
            for (unsigned k=0; k<3; ++k) sum+=f->P[6*i+k]*h[6*j+k];
            pht[3*i+j]=sum;
        }
    }
    for (unsigned i=0; i<n; ++i) for (unsigned j=0; j<=i; ++j) {
        float value=(i==j) ? variance : 0;
        if (bias_only) value+=pht[3*(i+3)+j];
        else for (unsigned k=0; k<3; ++k) value+=h[6*i+k]*pht[3*k+j];
        for (unsigned k=0; k<j; ++k) value-=l[3*i+k]*l[3*j+k];
        if (i==j) {
            if (!isfinite(value) || value<=1e-15f) {
                ++f->numeric_fault_count;
                return 0;
            }
            l[3*i+j]=sqrtf(value);
            /* 残差和六行增益求解复用同一对角元素的倒数。
             * 仍为 Cholesky 三角求解，不构造矩阵逆。 */
            inv_diag[i]=1.0f/l[3*i+j];
        } else l[3*i+j]=value*inv_diag[j];
    }
    float whitened[3]={0}, mahalanobis=0;
    solve(l,inv_diag,residual,whitened,n);
    for (unsigned i=0; i<n; ++i) mahalanobis+=residual[i]*whitened[i];
    if (nis) *nis=mahalanobis;
    if (!isfinite(mahalanobis) || mahalanobis>nis_limit) return 0;
    for (unsigned i=0; i<6; ++i) {
        solve(l,inv_diag,&pht[3*i],&gain[3*i],n);
    }
    if (unobservable_axis) {
        /* 移除绕导航系重力轴的修正旋转；这不保证倾斜校正时欧拉 yaw 数值不变。
         * 角误差位于导航系，陀螺零偏位于机体系。
         * 同时移除零偏增量沿当前 R^T e_z 的分量：倾斜噪声和交叉协方差
         * 可能产生虚假的竖直零偏，即使约束了 K_theta,z，下一拍预测仍会漂移。
         * 估计重力轴变化时，仍可能沿真实重力方向累积偏差，因此这不是
         * 整条运动轨迹上的可观性证明，而是对仅讨论姿态的文章所作的保守扩展。
         * 该约束舍弃部分历史跨轴增益，不是直接消除真实慢速 yaw 转动；
         * 倾斜变化仍会逐渐暴露不同的机体系零偏方向。
         * ZARU 和独立航向观测均不使用这两次投影。
         * 下方 Joseph 更新必须使用同一组受约束的增益。 */
        float rmat[9], body_axis[3]={0};
        rotation_matrix(f->q,rmat);
        for (unsigned i=0; i<3; ++i) for (unsigned j=0; j<3; ++j)
            body_axis[i]+=rmat[3*j+i]*unobservable_axis[j];
        for (unsigned block=0; block<2; ++block) for (unsigned j=0; j<n; ++j) {
            const float *axis=block ? body_axis : unobservable_axis;
            float along=0;
            for (unsigned i=0; i<3; ++i) along+=axis[i]*gain[3*(i+3*block)+j];
            for (unsigned i=0; i<3; ++i) gain[3*(i+3*block)+j]-=axis[i]*along;
        }
    }
    /* 新声明的静止区间只说明当前 gyro=bias，不能证明过去一直静止或确定航向。
     * 因此 ZARU 只学习零偏；否则累积的角度/零偏交叉协方差可能把
     * 陀螺残差转换为追溯性的姿态跳变。保留姿态协方差，并在下方 Joseph
     * 计算中用同一受约束增益更新交叉项；重力/航向观测保留各自增益。 */
    if (bias_only) zero_f32(gain,9);
    for (unsigned i=0; i<6; ++i) {
        float sum=0;
        for (unsigned j=0; j<n; ++j) sum+=gain[3*i+j]*residual[j];
        dx[i]=sum;
        if (!isfinite(dx[i])) return 0;
    }
    /* 修正量必须处于局部误差模型适用范围；越界时整次拒绝，
     * 不能截断状态修正却仍缩小协方差。 */
    if (dot3(dx,dx)>0.25f) return 0;
    for (unsigned i=0; i<3; ++i)
        if (fabsf(f->bias[i]+dx[i+3])>f->config.bias_limit) return 0;

    float a[18], p[36], reset_p[36], jr[9];
    /* Joseph: A=I-KH; P_post=A*P_prior*A^T + K*R*K^T。
     * 投影过的 K 不再是无约束最优增益，不能使用简化的 P=(I-KH)P。 */
    if (bias_only) {
        /* 仅 ZARU 使用此分支：n=3、H=[0 I]、K_theta=0。
         * A=diag(I,I-K_bias)，所以仍是完整 Joseph 更新。 */
        for (unsigned i=0; i<3; ++i) for (unsigned j=0; j<3; ++j)
            a[3*i+j]=(i==j ? 1.0f : 0.0f)-gain[3*(i+3)+j];
        block_covariance(a,3,f->P,p);
    } else {
        /* 仅重力/航向观测使用此分支，其 H 的零偏列为零。 */
        for (unsigned i=0; i<6; ++i) for (unsigned j=0; j<3; ++j) {
            a[3*i+j]=(i==j) ? 1 : 0;
            for (unsigned k=0; k<n; ++k) a[3*i+j]-=gain[3*i+k]*h[6*k+j];
        }
        attitude_observation_covariance(a,f->P,p);
    }
    const unsigned first=bias_only ? 3U : 0U;
    for (unsigned i=first; i<6; ++i) for (unsigned j=first; j<6; ++j)
        for (unsigned k=0; k<n; ++k) p[6*i+j]+=variance*gain[3*i+k]*gain[3*j+k];

    if (bias_only) {
        /* dx_theta 严格为零，四元数不变，重置雅可比为单位阵。 */
        if (!condition_covariance(p)) { ++f->numeric_fault_count; return 0; }
        for (unsigned i=0; i<3; ++i) f->bias[i]+=dx[i+3];
        copy_f32(f->P,p,36);
        return 1;
    }

    /* 全局误差重置：J_left(dx)=J_right(dx)^T，一阶近似为
     * I + 0.5[dx]x（Solà 第 7.3.3 节）；零偏仍在机体系。 */
    right_jacobian(dx,jr);
    for (unsigned i=0; i<3; ++i) for (unsigned j=0; j<3; ++j) a[3*i+j]=jr[3*j+i];
    block_covariance(a,0,p,reset_p);
    if (!condition_covariance(reset_p)) {
        ++f->numeric_fault_count;
        return 0;
    }
    inject_global(f->q,dx);
    for (unsigned i=0; i<3; ++i) f->bias[i]+=dx[i+3];
    copy_f32(f->P,reset_p,36);
    return 1;
}

static uint8_t correct_accel(MEKF_State *f, const float acc[3])
{
    const float *q=f->q;
    const float g[3]={2*(q[1]*q[3]-q[0]*q[2]),
                      2*(q[2]*q[3]+q[0]*q[1]),
                      1-2*(q[1]*q[1]+q[2]*q[2])};
    float z[3], tangent[6], h[18], residual[3]={0}, rmat[9];
    zero_f32(h,18);
    rotation_matrix(q,rmat);
    for (unsigned i=0; i<3; ++i) z[i]=acc[i]/f->accel_norm;
    /* 反向点处的切平面残差也为零，必须拒绝这一歧义解。 */
    if (dot3(z,g)<=0) return 0;
    if (fabsf(g[2])<0.9f) {
        tangent[0]=-g[1]; tangent[1]=g[0]; tangent[2]=0;
    } else {
        tangent[0]=0; tangent[1]=-g[2]; tangent[2]=g[1];
    }
    const float norm=sqrtf(dot3(tangent,tangent));
    for (unsigned i=0; i<3; ++i) tangent[i]/=norm;
    tangent[3]=g[1]*tangent[2]-g[2]*tangent[1];
    tangent[4]=g[2]*tangent[0]-g[0]*tangent[2];
    tangent[5]=g[0]*tangent[1]-g[1]*tangent[0];
    /* 全局误差：H = T R^T [e_z]x = T [g]x R^T。
     * 无论当前姿态如何，导航系 Z 对应列恒为零。 */
    for (unsigned i=0; i<2; ++i) for (unsigned j=0; j<3; ++j) {
        residual[i]+=tangent[3*i+j]*(z[j]-g[j]);
        for (unsigned k=0; k<3; ++k) {
            if (j==0) h[6*i+j]+=tangent[3*i+k]*rmat[3+k];
            if (j==1) h[6*i+j]-=tangent[3*i+k]*rmat[k];
        }
    }
    const float e=(f->accel_norm-f->config.gravity)/f->config.accel_adapt;
    const float r=f->config.accel_noise*f->config.accel_noise*(1+e*e);
    const float nav_up[3]={0,0,1};
    return correct(f,h,residual,r,2,f->config.accel_nis_limit,&f->accel_nis,nav_up,0);
}

uint8_t MEKF_Update(MEKF_State *f, const float gyro[3], const float acc[3],
                    float dt, uint8_t flags)
{
    /* 外部唯一周期入口。状态位可组合，例如 PREDICTED|ACCEL_REJECTED
     * 表示陀螺仍正常积分，只是本次未采信加速度，不等于整拍失败。 */
    if (!f || !f->initialized) return MEKF_INVALID_SAMPLE;
    f->status=0;
    f->accel_nis=-1;
    f->accel_norm=0;
    if (!finite3(gyro) || !isfinite(dt) || dt<=0 || dt>f->config.max_dt ||
        (flags & MEKF_GYRO_CLIPPED) || fabsf(gyro[0])>f->config.max_gyro ||
        fabsf(gyro[1])>f->config.max_gyro || fabsf(gyro[2])>f->config.max_gyro) {
        ++f->invalid_count;
        f->stationary_elapsed=f->zero_rate_elapsed=0;
        return f->status=MEKF_INVALID_SAMPLE;
    }
    if (!predict(f,gyro,dt)) {
        ++f->numeric_fault_count;
        f->stationary_elapsed=f->zero_rate_elapsed=0;
        return f->status=MEKF_INVALID_SAMPLE;
    }
    f->status=MEKF_PREDICTED;
    if (finite3(acc)) f->accel_norm=sqrtf(dot3(acc,acc));
    const float error=fabsf(f->accel_norm-f->config.gravity);
    const uint8_t valid_acc=isfinite(f->accel_norm) && f->accel_norm>1e-6f &&
        error<=f->config.accel_reject && !(flags & MEKF_ACCEL_CLIPPED);
    if (valid_acc && correct_accel(f,acc)) {
        f->status|=MEKF_ACCEL_USED;
        ++f->accel_accept_count;
    } else {
        f->status|=MEKF_ACCEL_REJECTED;
        ++f->accel_reject_count;
    }
    if ((flags & MEKF_STATIONARY) && valid_acc && error<f->config.stationary_accel &&
        dot3(gyro,gyro)<f->config.stationary_gyro*f->config.stationary_gyro) {
        f->stationary_elapsed=fminf(f->stationary_elapsed+dt,f->config.stationary_time);
        f->zero_rate_elapsed+=dt;
        if (f->stationary_elapsed>=f->config.stationary_time && f->zero_rate_elapsed>=0.01f) {
            const float h[18]={0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1};
            float residual[3];
            for (unsigned i=0; i<3; ++i) residual[i]=gyro[i]-f->bias[i];
            const float r=f->config.zero_rate_noise*f->config.zero_rate_noise;
            if (correct(f,h,residual,r,3,11.345f,NULL,NULL,1)) {
                f->status|=MEKF_ZERO_RATE_USED;
                ++f->zero_rate_count;
            }
            f->zero_rate_elapsed=0;
        }
    } else f->stationary_elapsed=f->zero_rate_elapsed=0;
    return f->status;
}

void MEKF_GetEuler(const MEKF_State *f, float *roll, float *pitch, float *yaw)
{
    if (!f || !f->initialized) return;
    const float *q=f->q;
    const float sine=fmaxf(-1.0f,fminf(1.0f,2*(q[0]*q[2]-q[1]*q[3])));
    if (roll) *roll=atan2f(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]*q[1]+q[2]*q[2]));
    if (pitch) *pitch=asinf(sine);
    if (yaw) *yaw=atan2f(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
}

uint8_t MEKF_FuseHeading(MEKF_State *f, float yaw_rad, float variance_rad2)
{
    /* 固定基座 + 编码器的使用思路：
     *   gyro 负责高频姿态传播；编码器提供长期不积分漂移的参考角。
     *   r = wrap(yaw_encoder - yaw(q))，H 是 yaw 对导航系小角度的导数。
     *   H 的 bias 列虽然为 0，P 中的角度/零偏交叉项仍可把持续角度残差
     *   反馈到 gyro bias。不要人为“每秒把 yaw 归零”而跳过协方差更新。
     * R 的来源包含编码器精度（不只是位数）、安装误差、回差和时间误差。
     * 转速 omega、同步误差 sigma_t 对角方差的近似贡献为 (omega*sigma_t)^2。
     * 这里只接受已经换算到导航系的 yaw；编码器驱动与时间同步属于应用层。 */
    if (!f || !f->initialized) return 0;
    f->heading_nis = -1.0f;
    f->status &= (uint8_t)~MEKF_HEADING_USED;
    if (!isfinite(yaw_rad) || !isfinite(variance_rad2) || variance_rad2 < 1e-8f ||
        variance_rad2 > 1.0f) {
        ++f->heading_reject_count;
        return 0;
    }
    const float *q = f->q;
    const float r00 = 1 - 2*(q[2]*q[2] + q[3]*q[3]);
    const float r10 = 2*(q[1]*q[2] + q[0]*q[3]);
    const float horizontal2 = r00*r00 + r10*r10;
    if (horizontal2 < 0.01f) { /* forward 近竖直，Euler yaw 的 Jacobian 病态。 */
        ++f->heading_reject_count;
        return 0;
    }
    const float r20 = 2*(q[1]*q[3] - q[0]*q[2]);
    float h[18] = {0};
    h[0] = -r00*r20 / horizontal2;
    h[1] = -r10*r20 / horizontal2;
    h[2] = 1;
    const float delta = yaw_rad - atan2f(r10, r00);
    const float residual[3] = {atan2f(sinf(delta), cosf(delta)), 0, 0};
    /* 单维 99% NIS=6.635。这里有真实航向观测，不使用重力增益的 yaw 投影。 */
    if (correct(f, h, residual, variance_rad2, 1, 6.635f, &f->heading_nis, NULL, 0)) {
        ++f->heading_accept_count;
        f->status |= MEKF_HEADING_USED;
        return 1;
    }
    ++f->heading_reject_count;
    return 0;
}
