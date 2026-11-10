# 六轴 VQF

本目录提供不依赖 HAL/RTOS 的六轴 VQF，源于 [Daniel Laidig 的 VQF](https://github.com/dlaidig/vqf)，许可见 [LICENSE-MIT.txt](LICENSE-MIT.txt)。

## 工程接入

在 `robot/<兵种>/robot.config` 中启用 `CONFIG_COMPONENT_AHRS_VQF = y`，并在同目录 `ins_config.h` 中设置 `INS_ALGORITHM = AHRS_ALGORITHM_VQF`。当前 demo 选择 MEKF，本次精简不改变选择。

调用链为 `ins_task → ahrs_update → VQF_Update`。输入来自 [BMI088 驱动](../../../cdd/imu/bmi088/bmi088.c)：角速度 rad/s、比力 m/s²、相邻样本的实际间隔秒。固定标定由驱动处理，VQF 只估计剩余零偏，不能重复扣除。

公开入口只有 `VQF_Init`、`VQF_Update`、`VQF_BreakContinuity`，底层 `vqf_*` 函数属于内部实现。`filter.q` 为 Hamilton `[w,x,y,z]` 四元数，表示机体系到重力对齐的参考系；绝对航向不可观。欧拉角及累计 yaw 统一由 [AHRS](../ahrs.c) 计算，使用标准 Z-Y-X、弧度单位。

## 状态和算法

- `gyrQuat` 积分去偏陀螺；`accQuat` 用低通后的重力方向修正倾斜，输出 `q = accQuat ⊗ gyrQuat`。
- 静止检测使用陀螺和加速度各自的低通历史；两路输入在本接口中同步，共用低通系数。
- 静止时直接学习三轴 bias；运动时利用倾斜修正估计 bias，并对竖直分量施加弱先验。静止期间仍更新运动低通历史，避免恢复运动时出现陈旧状态。
- `bias` 单位 rad/s，`biasP` 是三轴协方差；参数 `biasSigma*`、`biasClip`、`restThGyr` 仍采用上游 °/s 约定。
- `restDetected/restT`、状态位和统计计数保留，可在调试器查看 `ahrs.state.vqf`。路径计数不等于每拍都成功校正。

默认参数在 `vqf_init` 中：`tauAcc=3 s`、静止确认时间 `1.5 s`。当前 BMI088 2 kHz/230 Hz 带宽输入采用陀螺偏差门限 `restThGyr=3 °/s`；原 2 °/s 门限会被本板静止噪声反复打断。低通角速度的 `biasClip=2 °/s` 限制仍保留。修改参数后须重新建立派生系数。自动静止检测可能把真实慢转当成零偏，六轴重力观测不能消除绝对航向漂移。

## 数值保护和精简范围

低通系数和递归状态使用 double。dt 改变时同步更新系数并变换历史；启动阶段按实际时间加权平均。gyro 无效、饱和或 dt 超出 `[1e-5, 0.02]` 时拒绝整拍；accel 无效时仅积分 gyro 并打断静止窗口。局部候选状态通过 q、bias、P 的有限性检查后才提交，保留数值故障回退。

删除了 AHRS 已负责的欧拉角、累计 yaw，以及无使用者的输出旋转矩阵和倾斜修正角速度。零偏估计内部所需的旋转矩阵和全部低通历史保留。两路静止检测共用系数，初始化复用步长更新和连续性重置逻辑。`VQF_FILTER_t` 的旧输出字段及底层函数不再作为外部接口提供。

矩阵求逆仍采用原来的 3×3 伴随矩阵法，协方差使用 `P - KHP`；本次未更换滤波算法、参数或浮点精度。有限性检查不保证所有病态输入下的正定性。不要启用 fast-math；实际速度须上板测量。

## 下载来源留存

移植依据为 2026-10-02 归档的四份输入原件。下载版 BMI088 文件头作者为 CaoKangqi、日期为 2026/7/6，依赖原工程的 `BSP_SPI`、`All_define` 和 GPIO 配置，没有替换框架驱动。

下载版 VQF 文件头没有独立作者或许可声明；其六轴实现源于 [Daniel Laidig 的 VQF](https://github.com/dlaidig/vqf)，适配代码保留该上游的署名和 MIT 许可，不为 BMI088 原件补造许可声明。原件归档时按下列 SHA-256 核对内容：

| 原文件名 | SHA-256 |
| --- | --- |
| BMI088.c | `2d2acf51e7b34db5de42aee7429a8ea994c22e3fa280edb82cc72a1dba3b7ac5` |
| BMI088.h | `aa25ec13095206304bfd0dfc2441dde21864cfa01e4309a75919323fad809a54` |
| VQF_filter.c | `fe4d969f9e830a650ad77be312069d858e2d0fe130c5a8ccf347a9428cae947a` |
| VQF_filter.h | `70dfc5f4d760b67310ccefbb68351d2d461c892f1b30b8d88c0a68dc22b4b1e5` |
