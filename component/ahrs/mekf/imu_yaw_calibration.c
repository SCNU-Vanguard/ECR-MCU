/* 固定底座 yaw：10 ms 唯一新帧 -> 编码器/IMU 门控 -> 静止等待 100 ms
 * -> 约 1 s 的非重叠时间块 -> 30 块长期线性拟合 -> 置信度报告。
 *
 * 每块观测：(积分的未扣 MEKF bias 的 yaw_rate - 编码器角度变化)/时长。
 * 用编码器增量扣除静止容差内的微动；不是把“小角速度”直接认定成零偏。
 * yaw_rate = H_body(q)*gyro。这里作用于机体系角速度；MEKF_FuseHeading
 * 使用导航系左乘误差的 H_nav，两者满足 H_body = H_nav*R(q)，并非逐项相同。
 * 因此支持倾斜下的 Euler yaw 变化率；这里只辨识航向漂移，绝不据此覆盖三轴零偏。
 *
 * 拟合 y=b(tc)+k*(t-tc)。报告窗口末端 b，而非过时的窗口中心值。
 * 使用 3 阶 Bartlett/Newey-West 型 HAC 方差，且不小于白噪声估计/配置
 * 下限；用 lag-1 给有效块数的保守近似。强相关、明显温变、残差过大拒绝。
 * 这些仍是有限窗口模型近似，不是已经校准的统计覆盖率。不能只看 R^2。
 *
 * 100 ms 是开始采数的门槛，不能宣称 100 ms 就完成长期漂移标定。
 * 配置里的噪声/门限只是起步值，必须用本板数据验证。编码器“精准”也不能
 * 把 R 填成 0；安装、同步、回差与相关误差仍可能存在。斜率不向运动期外推。
 * 拟合值和 MEKF 都使用了 gyro，故不将拟合结果作为独立 bias 观测回灌。
 * NIST: https://www.nist.gov/programs-projects/uncertainty-analysis-autocorrelated-measurement-data
 * ADI: https://www.analog.com/en/resources/app-notes/an-1042.html
 */
#include "component/ahrs/mekf/imu_yaw_calibration.h"
#include <math.h>
#include <string.h>

static float wrap(float a) { return atan2f(sinf(a),cosf(a)); }

void IMU_YawCalibrationInit(IMU_YawCalibration *s)
{
    const IMU_YawCalibrationConfig c = {
        100000U, 10000U, 0.0005f, 0.001f,
        0.05f, 0.01f, 0.2f, 0.15f, 0.0002f, 0.003f, 0.0001f, 1.0f
    };
    memset(s,0,sizeof(*s));
    s->config=c;
    s->phase=IMU_YAW_NO_REFERENCE;
}

static void clear_window(IMU_YawCalibration *s)
{
    s->samples=s->block_samples=s->blocks=0;
    s->window_start_us=s->block_start_us=0;
    s->integral=0;
}

static void interrupt_window(IMU_YawCalibration *s, uint8_t reason)
{
    if (s->samples) ++s->discarded_windows;
    clear_window(s);
    s->quiet=s->settled=s->have_previous=0;
    s->phase=reason;
}

void IMU_YawCalibrationBreak(IMU_YawCalibration *s)
{
    if (s) interrupt_window(s,IMU_YAW_BAD_SAMPLE);
}

static float hac_variance(const float residual[IMU_YAW_BLOCKS],
                          const float weight[IMU_YAW_BLOCKS])
{
    float variance=0;
    for (unsigned i=0;i<IMU_YAW_BLOCKS;++i)
        variance+=weight[i]*weight[i]*residual[i]*residual[i];
    for (unsigned lag=1;lag<=3;++lag) {
        float cross=0;
        for (unsigned i=lag;i<IMU_YAW_BLOCKS;++i)
            cross+=weight[i]*residual[i]*weight[i-lag]*residual[i-lag];
        variance+=2.0f*(1.0f-(float)lag/4.0f)*cross;
    }
    return fmaxf(0,variance)*(float)IMU_YAW_BLOCKS/(IMU_YAW_BLOCKS-2U);
}

static void fit(IMU_YawCalibration *s, uint64_t now)
{
    IMU_YawCalibrationResult r={0};
    r.sequence=s->result.sequence+1U;
    r.samples=s->samples;
    r.timestamp_us=now;
    r.span_s=(float)(now-s->window_start_us)*1e-6f;
    r.temperature_span=s->temperature_max-s->temperature_min;
    r.usable=r.temperature_span<=s->config.temperature_span_limit;
    float tc=0,average=0,sxx=0,slope=0;
    for (unsigned i=0;i<IMU_YAW_BLOCKS;++i) {
        tc+=s->time[i]/IMU_YAW_BLOCKS;
        average+=s->mean[i]/IMU_YAW_BLOCKS;
    }
    for (unsigned i=0;i<IMU_YAW_BLOCKS;++i) {
        const float d=s->time[i]-tc;
        sxx+=d*d;
        slope+=d*(s->mean[i]-average);
    }
    if (!(sxx>1) || r.span_s<0.9f*IMU_YAW_BLOCKS) { r.usable=0; sxx=fmaxf(sxx,1); }
    slope/=sxx;
    const float endpoint=r.span_s-tc;
    r.yaw_rate_bias=average+slope*endpoint;
    r.slope=slope;
    float residual[IMU_YAW_BLOCKS],bw[IMU_YAW_BLOCKS],sw[IMU_YAW_BLOCKS];
    float sse=0,adjacent=0;
    for (unsigned i=0;i<IMU_YAW_BLOCKS;++i) {
        residual[i]=s->mean[i]-average-slope*(s->time[i]-tc);
        sse+=residual[i]*residual[i];
        if (i) adjacent+=residual[i]*residual[i-1U];
        sw[i]=(s->time[i]-tc)/sxx;
        bw[i]=1.0f/IMU_YAW_BLOCKS+endpoint*sw[i];
    }
    const float variance=sse/(IMU_YAW_BLOCKS-2U);
    r.residual_std=sqrtf(variance);
    r.lag1=sse>1e-16f ? adjacent/sse : 0;
    const float rho=fmaxf(0,fminf(0.99f,r.lag1));
    r.effective_blocks=IMU_YAW_BLOCKS*(1-rho)/(1+rho);
    float vb=fmaxf(hac_variance(residual,bw),variance*(1.0f/IMU_YAW_BLOCKS+endpoint*endpoint/sxx));
    vb=fmaxf(vb,s->config.bias_std_floor*s->config.bias_std_floor);
    r.bias_std=sqrtf(vb);
    r.slope_std=sqrtf(fmaxf(hac_variance(residual,sw),variance/sxx));
    /* 2.31 是 >=10 个有效块的保守 t 因子近似，不是实际覆盖率证明。 */
    r.approximate_ci95=2.31f*r.bias_std;
    if (!isfinite(r.yaw_rate_bias) || !isfinite(r.bias_std) || r.effective_blocks<10 ||
        fabsf(r.yaw_rate_bias)>s->config.gyro_limit || fabsf(slope)>s->config.slope_limit ||
        r.residual_std>s->config.residual_std_limit) r.usable=0;
    if (!r.usable) ++s->fit_rejects;
    s->result=r;
    s->age_s=0;
    clear_window(s); /* 一份历史只发布一次；下一窗用新的时间区间。 */
    s->phase=r.usable ? IMU_YAW_COLLECTING : IMU_YAW_BAD_FIT;
}

uint8_t IMU_YawCalibrationPush(IMU_YawCalibration *s, uint64_t now,
    const float q[4], const float gyro[3], const float accel[3], float temperature,
    uint8_t clipped, const IMU_YawEncoder *e)
{
    if (!s) return 0;
    if (s->result.sequence && now>=s->result.timestamp_us)
        s->age_s=(float)(now-s->result.timestamp_us)*1e-6f;
    if (s->last_us && (now<=s->last_us || now-s->last_us<9000U)) {
        ++s->duplicate_samples;
        s->phase=IMU_YAW_DUPLICATE;
        return 0; /* 重复调用不推进静止时间，不重复消费样本。 */
    }
    const uint64_t delta=s->last_us ? now-s->last_us : 0;
    if (delta>20000U) interrupt_window(s,IMU_YAW_BAD_SAMPLE);
    s->last_us=now;
    const uint64_t skew=e ? (e->timestamp_us>now ? e->timestamp_us-now : now-e->timestamp_us) : UINT64_MAX;
    /* now 是 IMU 数据时刻，不是调用时刻。最近反馈可略晚于 IMU；调用方
     * 必须保证该反馈在调用时确已收到（INS 入口另外校验墙钟未来数据）。 */
    if (!e || !e->valid || !isfinite(e->yaw_rad) || skew>s->config.max_skew_us ||
        (s->have_encoder && (e->timestamp_us<=s->last_encoder_us || e->sequence==s->last_encoder_sequence))) {
        ++s->reference_faults;
        interrupt_window(s,IMU_YAW_NO_REFERENCE);
        return 0;
    }
    s->last_encoder_us=e->timestamp_us;
    s->last_encoder_sequence=e->sequence;
    s->have_encoder=1;
    const float span=s->config.encoder_span_rad;
    if (e->motion_command || (s->quiet && fabsf(wrap(e->yaw_rad-s->anchor))>span)) {
        ++s->motion_resets;
        interrupt_window(s,IMU_YAW_MOVING);
        return 0;
    }
    float norm2=0,qnorm2=0;
    uint8_t good=q && gyro && accel && isfinite(temperature) && !clipped;
    if (good) {
        for (unsigned i=0;i<4;++i) qnorm2+=q[i]*q[i];
        for (unsigned i=0;i<3;++i) {
            if (!isfinite(gyro[i]) || !isfinite(accel[i]) || fabsf(gyro[i])>s->config.gyro_limit) good=0;
            norm2+=accel[i]*accel[i];
            if (s->have_previous && (fabsf(gyro[i]-s->previous_gyro[i])>s->config.gyro_step_limit ||
                fabsf(accel[i]-s->previous_accel[i])>s->config.accel_step_limit)) good=0;
        }
    }
    if (!good || !isfinite(qnorm2) || fabsf(qnorm2-1)>0.02f || !isfinite(norm2) ||
        fabsf(sqrtf(norm2)-9.81f)>s->config.accel_error_limit) {
        interrupt_window(s,IMU_YAW_BAD_SAMPLE);
        return 0;
    }
    const float r00=1-2*(q[2]*q[2]+q[3]*q[3]),r10=2*(q[1]*q[2]+q[0]*q[3]);
    const float horizontal2=r00*r00+r10*r10;
    if (horizontal2<0.04f) { interrupt_window(s,IMU_YAW_BAD_SAMPLE); return 0; }
    const float r01=2*(q[1]*q[2]-q[0]*q[3]),r11=1-2*(q[1]*q[1]+q[3]*q[3]);
    const float r02=2*(q[1]*q[3]+q[0]*q[2]),r12=2*(q[2]*q[3]-q[0]*q[1]);
    const float rate=((r10*r02-r00*r12)*gyro[1]+(r00*r11-r10*r01)*gyro[2])/horizontal2;
    memcpy(s->previous_gyro,gyro,sizeof(s->previous_gyro));
    memcpy(s->previous_accel,accel,sizeof(s->previous_accel));
    s->have_previous=1;
    if (!s->quiet) { s->quiet=1; s->quiet_since_us=now; s->anchor=e->yaw_rad; }
    if (now-s->quiet_since_us<s->config.settle_us) {
        s->phase=IMU_YAW_SETTLING;
        return 0;
    }
    s->settled=1;
    s->phase=IMU_YAW_COLLECTING;
    if (!s->samples) {
        s->window_start_us=now;
        s->temperature_min=s->temperature_max=temperature;
    }
    if (!s->block_samples) {
        s->block_start_us=now;
        s->block_encoder_start=e->yaw_rad;
        s->integral=0;
    } else {
        s->integral+=0.5f*(s->previous_rate+rate)*(float)delta*1e-6f;
    }
    s->previous_rate=rate;
    s->temperature_min=fminf(s->temperature_min,temperature);
    s->temperature_max=fmaxf(s->temperature_max,temperature);
    ++s->samples;
    ++s->total_samples;
    ++s->block_samples;
    if (now-s->block_start_us>=1000000U && s->block_samples>=90U) {
        const float duration=(float)(now-s->block_start_us)*1e-6f;
        s->time[s->blocks]=((float)(s->block_start_us-s->window_start_us)*1e-6f)+0.5f*duration;
        s->mean[s->blocks]=(s->integral-wrap(e->yaw_rad-s->block_encoder_start))/duration;
        ++s->blocks;
        s->block_samples=0;
        if (s->blocks==IMU_YAW_BLOCKS) { fit(s,now); return 1; }
    }
    return 0;
}
