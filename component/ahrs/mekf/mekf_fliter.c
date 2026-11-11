/*
 * 六轴误差状态 MEKF。
 * 名义状态为四元数 q 和机体系陀螺零偏 bias，误差状态为
 * [姿态误差(3), 零偏误差(3)]。四元数使用 Hamilton [w,x,y,z]，
 * 表示机体系到固定参考坐标系的旋转；陀螺单位 rad/s，加速度单位 m/s^2。
 * 固定参考坐标系表示上电初始化时的板子坐标系；实现上由首帧有效加速度建立，
 * yaw 初值为 0，随后随陀螺积分变化。
 * 每个周期依次执行陀螺预测、重力方向更新和可选的静止零角速更新。
 * 加速度只约束倾斜，不能提供绝对 yaw。静止比力方向约定为固定参考坐标系 +Z。
 */
#include "component/ahrs/mekf/mekf_fliter.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

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

/* q <- q * Exp(v)。 */
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

/* 计算机体系到固定参考坐标系的旋转矩阵。 */
static void rotation_matrix(const float q[4], float r[9])
{
    const float w=q[0], x=q[1], y=q[2], z=q[3];
    r[0]=1-2*(y*y+z*z); r[1]=2*(x*y-w*z); r[2]=2*(x*z+w*y);
    r[3]=2*(x*y+w*z); r[4]=1-2*(x*x+z*z); r[5]=2*(y*z-w*x);
    r[6]=2*(x*z-w*y); r[7]=2*(y*z+w*x); r[8]=1-2*(x*x+y*y);
}

/* 将固定参考坐标系姿态误差转换到机体系后右乘四元数。 */
static void inject_global(float q[4], const float v[3])
{
    float r[9], local[3]={0};
    rotation_matrix(q,r);
    for (unsigned i=0; i<3; ++i) for (unsigned j=0; j<3; ++j)
        local[i]+=r[3*j+i]*v[j];
    inject(q,local);
}

/* SO(3) 右雅可比。 */
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

/* 计算姿态观测的 Joseph 协方差变换。 */
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

/* 按 Phi=[I B; 0 I] 传播协方差。 */
static void propagate_covariance(const float b[9], const float p[36], float dst[36])
{
    float ap[36];
    memcpy(ap+18,p+18,18*sizeof(float));
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

/* 计算姿态注入或零偏更新后的协方差。 */
static void block_covariance(const float block[9], unsigned offset,
                             const float p[36], float dst[36])
{
    float ap[36];
    const unsigned unchanged=3-offset;
    memcpy(ap+6*unchanged,p+6*unchanged,18*sizeof(float));
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
    /* 默认启动参数，可以结合实际调整。 */
    const MEKF_Config c={
        .gyro_noise=0.003f, .bias_walk=0.0001f, .accel_noise=0.035f,
        .initial_angle_std=0.35f, .initial_bias_std=0.003f,
        .gravity=9.81f, .accel_reject=1.5f, .accel_adapt=0.3f,
        .accel_nis_limit=9.21f, .max_dt=0.02f, .max_gyro=35.0f,
        .bias_limit=0.15f, .stationary_gyro=0.05f, .stationary_accel=0.2f,
        .stationary_time=0.5f, .zero_rate_noise=0.015f
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
    /* 先复制输入，支持使用实例自身字段重新初始化。 */
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
    /* 使用校准后的陀螺输入，扣除 MEKF 估计的剩余 bias。 */
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
    /* 传播姿态、零偏及其交叉协方差，并加入过程噪声。 */
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
    memcpy(f->P,p,36*sizeof(float));
    ++f->predict_count;
    return 1;
}

/* 用 Cholesky 分解求解小型观测方程。 */
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
    /* 通用观测更新：重力、航向或静止零角速观测。 */
    float pht[18], l[9], inv_diag[3], gain[18], dx[6];
    /* 根据观测结构计算 PH^T 和创新协方差。 */
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
            /* 后续使用同一分解求残差和增益。 */
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
        /* 重力观测不能修正绕固定参考坐标系重力轴的姿态和对应零偏分量。 */
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
    /* ZARU 只更新零偏，不回溯修改姿态。 */
    if (bias_only) memset(gain,0,9*sizeof(float)); 
    for (unsigned i=0; i<6; ++i) {
        float sum=0;
        for (unsigned j=0; j<n; ++j) sum+=gain[3*i+j]*residual[j];
        dx[i]=sum;
        if (!isfinite(dx[i])) return 0;
    }
    /* 局部误差过大时拒绝整次更新。 */
    if (dot3(dx,dx)>0.25f) return 0;
    for (unsigned i=0; i<3; ++i)
        if (fabsf(f->bias[i]+dx[i+3])>f->config.bias_limit) return 0;

    float a[18], p[36], reset_p[36], jr[9];
    /* 使用 Joseph 形式更新协方差。 */
    if (bias_only) {
        /* ZARU 分支：仅更新零偏。 */
        for (unsigned i=0; i<3; ++i) for (unsigned j=0; j<3; ++j)
            a[3*i+j]=(i==j ? 1.0f : 0.0f)-gain[3*(i+3)+j];
        block_covariance(a,3,f->P,p);
    } else {
        /* 重力和航向观测分支。 */
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
        /* ZARU 不改变四元数。 */
        if (!condition_covariance(p)) { ++f->numeric_fault_count; return 0; }
        for (unsigned i=0; i<3; ++i) f->bias[i]+=dx[i+3];
        memcpy(f->P,p,36*sizeof(float));
        return 1;
    }

    /* 注入姿态误差，并将协方差转换到新的误差坐标系。 */
    right_jacobian(dx,jr);
    for (unsigned i=0; i<3; ++i) for (unsigned j=0; j<3; ++j) a[3*i+j]=jr[3*j+i];
    block_covariance(a,0,p,reset_p);
    if (!condition_covariance(reset_p)) {
        ++f->numeric_fault_count;
        return 0;
    }
    inject_global(f->q,dx);
    for (unsigned i=0; i<3; ++i) f->bias[i]+=dx[i+3];
    memcpy(f->P,reset_p,36*sizeof(float));
    return 1;
}

static uint8_t correct_accel(MEKF_State *f, const float acc[3])
{
    const float *q=f->q;
    const float g[3]={2*(q[1]*q[3]-q[0]*q[2]),
                      2*(q[2]*q[3]+q[0]*q[1]),
                      1-2*(q[1]*q[1]+q[2]*q[2])};
    float z[3], tangent[6], h[18], residual[3]={0}, rmat[9];
    memset(h,0,18*sizeof(float));
    rotation_matrix(q,rmat);
    for (unsigned i=0; i<3; ++i) z[i]=acc[i]/f->accel_norm;
    /* 反向重力方向不是有效观测。 */
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
    /* 将重力方向残差投影到二维切平面。 */
    for (unsigned i=0; i<2; ++i) for (unsigned j=0; j<3; ++j) {
        residual[i]+=tangent[3*i+j]*(z[j]-g[j]);
        for (unsigned k=0; k<3; ++k) {
            if (j==0) h[6*i+j]+=tangent[3*i+k]*rmat[3+k];
            if (j==1) h[6*i+j]-=tangent[3*i+k]*rmat[k];
        }
    }
    const float e=(f->accel_norm-f->config.gravity)/f->config.accel_adapt;
    const float r=f->config.accel_noise*f->config.accel_noise*(1+e*e);
    const float reference_up[3]={0,0,1};
    return correct(f,h,residual,r,2,f->config.accel_nis_limit,&f->accel_nis,reference_up,0);
}

/* 静止期间重力观测持续拒绝时，恢复倾斜并保留 yaw。 */
static uint8_t recover_accel_tilt(MEKF_State *f, const float acc[3])
{
    float z[3], r[9], measured_ref[3], axis[3];
    const float norm=f->accel_norm;
    if (!isfinite(norm) || norm<=1e-6f) return 0;
    for (unsigned i=0; i<3; ++i) z[i]=acc[i]/norm;
    rotation_matrix(f->q,r);
    for (unsigned i=0; i<3; ++i)
        measured_ref[i]=r[3*i]*z[0]+r[3*i+1]*z[1]+r[3*i+2]*z[2];
    /* 计算把测得重力方向转到固定参考坐标系 +Z 的最小旋转。 */
    axis[0]=measured_ref[1];
    axis[1]=-measured_ref[0];
    axis[2]=0.0f;
    const float sine=sqrtf(dot3(axis,axis));
    const float cosine=fmaxf(-1.0f,fminf(1.0f,measured_ref[2]));
    if (!isfinite(sine) || (sine<=1e-6f && cosine<0.0f))
        return 0; /* exactly opposite gravity has no unique correction axis */
    if (sine>1e-6f) {
        const float angle=atan2f(sine,cosine);
        for (unsigned i=0; i<3; ++i) axis[i]*=angle/sine;
        inject_global(f->q,axis);
    }
    /* 重置倾斜相关协方差，保留 yaw 和零偏的不确定度。 */
    const float up[3]={0.0f,0.0f,1.0f};
    const float angle_var=f->config.initial_angle_std*f->config.initial_angle_std;
    const float pzz=f->P[14];
    for (unsigned i=0; i<3; ++i) for (unsigned j=0; j<3; ++j)
        f->P[6*i+j]=pzz*up[i]*up[j]+angle_var*((i==j ? 1.0f : 0.0f)-up[i]*up[j]);
    for (unsigned i=0; i<3; ++i) for (unsigned j=3; j<6; ++j)
        f->P[6*i+j]=f->P[6*j+i]=0.0f;
    return 1;
}

static uint8_t accel_direction_outlier(const MEKF_State *f, const float acc[3])
{
    const float *q=f->q;
    const float gx=2.0f*(q[1]*q[3]-q[0]*q[2]);
    const float gy=2.0f*(q[2]*q[3]+q[0]*q[1]);
    const float gz=1.0f-2.0f*(q[1]*q[1]+q[2]*q[2]);
    /* correct_accel 会先拒绝反向半球。 */
    return dot3(acc,(float[3]){gx,gy,gz})<=0.0f;
}

void MEKF_BreakContinuity(MEKF_State *f)
{
    if (!f) return;
    f->status=MEKF_INVALID_SAMPLE;
    f->stationary_elapsed=f->zero_rate_elapsed=f->tilt_recovery_elapsed=0;
    memset(f->zero_rate_integral,0,3*sizeof(float)); 
}

uint8_t MEKF_Update(MEKF_State *f, const float gyro[3], const float acc[3],
                    float dt, uint8_t flags)
{
    /* 周期更新入口；加速度被拒绝时仍保留陀螺预测。 */
    if (!f || !f->initialized) return MEKF_INVALID_SAMPLE;
    f->status=0;
    f->accel_nis=-1;
    f->accel_norm=0;
    if (!finite3(gyro) || !isfinite(dt) || dt<=0 || dt>f->config.max_dt ||
        (flags & MEKF_GYRO_CLIPPED) || fabsf(gyro[0])>f->config.max_gyro ||
        fabsf(gyro[1])>f->config.max_gyro || fabsf(gyro[2])>f->config.max_gyro) {
        ++f->invalid_count;
        MEKF_BreakContinuity(f);
        return f->status=MEKF_INVALID_SAMPLE;
    }
    if (!predict(f,gyro,dt)) {
        ++f->numeric_fault_count;
        MEKF_BreakContinuity(f);
        return f->status=MEKF_INVALID_SAMPLE;
    }
    f->status=MEKF_PREDICTED;
    if (finite3(acc)) f->accel_norm=sqrtf(dot3(acc,acc));
    const float error=fabsf(f->accel_norm-f->config.gravity);
    const uint8_t valid_acc=isfinite(f->accel_norm) && f->accel_norm>1e-6f &&
        error<=f->config.accel_reject && !(flags & MEKF_ACCEL_CLIPPED);
    const uint8_t accel_used=valid_acc && correct_accel(f,acc);
    if (accel_used) {
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
        for (unsigned i=0; i<3; ++i) f->zero_rate_integral[i]+=gyro[i]*dt;
        if (!accel_used && (f->accel_nis>f->config.accel_nis_limit ||
                            accel_direction_outlier(f,acc)))
            f->tilt_recovery_elapsed=fminf(f->tilt_recovery_elapsed+dt,f->config.stationary_time);
        else f->tilt_recovery_elapsed=0.0f;
        if (f->stationary_elapsed>=f->config.stationary_time && f->zero_rate_elapsed>=0.01f) {
            /* 静止期间重力持续被拒绝时，先恢复倾斜再执行 ZARU。 */
            if (!accel_used && f->tilt_recovery_elapsed>=f->config.stationary_time &&
                recover_accel_tilt(f,acc)) {
                f->status=(uint8_t)((f->status & (uint8_t)~MEKF_ACCEL_REJECTED) |
                                    MEKF_ACCEL_USED);
                ++f->accel_accept_count;
                f->accel_nis=-1.0f; /* direct recovery is not an NIS update */
                f->tilt_recovery_elapsed=0.0f;
            }
            const float h[18]={0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1};
            float residual[3];
            /* 对窗口内全部样本作时间加权平均；单帧抽取会将周期噪声混叠为零偏。
             * 保留保守的观测噪声，不假定带宽内的相邻样本相互独立。 */
            for (unsigned i=0; i<3; ++i)
                residual[i]=f->zero_rate_integral[i]/f->zero_rate_elapsed-f->bias[i];
            const float r=f->config.zero_rate_noise*f->config.zero_rate_noise;
            if (correct(f,h,residual,r,3,11.345f,NULL,NULL,1)) {
                f->status|=MEKF_ZERO_RATE_USED;
                ++f->zero_rate_count;
            }
            f->zero_rate_elapsed=0;
            memset(f->zero_rate_integral,0,3*sizeof(float));
        }
    } else {
        f->stationary_elapsed=f->zero_rate_elapsed=f->tilt_recovery_elapsed=0;
        memset(f->zero_rate_integral,0,3*sizeof(float));
    }
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
    /* 融合独立的固定参考坐标系 yaw 观测。 */
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
    /* yaw 观测使用一维 NIS 门限。 */
    if (correct(f, h, residual, variance_rad2, 1, 6.635f, &f->heading_nis, NULL, 0)) {
        ++f->heading_accept_count;
        f->status |= MEKF_HEADING_USED;
        return 1;
    }
    ++f->heading_reject_count;
    return 0;
}
