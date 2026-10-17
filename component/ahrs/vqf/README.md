# 六轴 VQF 接入

生产代码统一为 [vqf_filter.c](vqf_filter.c) 和 [vqf_filter.h](vqf_filter.h) 一对文件。原先分开的算法核心与 `imu_vqf` 输入保护层已经合并；正常调用使用 `VQF_Init / VQF_Update / VQF_BreakContinuity`，底层 `vqf_*` 接口服务于内部实现和算法测试，并非另一套滤波器。

目录包含这对源码、本文和 [MIT 许可证](LICENSE-MIT.txt)。来源和原件校验值见文末；BMI088 输入驱动为 `module/imu/bmi088.c/h`。

从操作流程开始阅读时，见 [VQF 原理与标定流程](../../../doc/IMU/VQF标定流程.md)；本文侧重源码、接口、数值保护和构建要求。

## 选择、输入与输出

在 [ins_config.h](../../../application/INS/ins_config.h) 设置 `INS_RUN_MODE=INS_MODE_SINGLE_VQF` 即可单独运行 VQF。
需要同帧对比时选择 `INS_MODE_COMPARE_MEKF_MADGWICK_VQF`，主输出为 MEKF。
单算法自动发四通道；这个三算法组合自动发带身份的 25 通道，不需要再配置双算法、benchmark、第三算法等开关。
完整组合和独立的 MEKF 静止声明见 [INS 使用指南](../../../doc/IMU/README.md#姿态算法)。

输入共享 `BMI088_Read_All` 的 `gyro_unfiltered`、`acc_unfiltered`：已扣固定标定，未经额外软件低通，分别为 rad/s 和 m/s²。VQF 内部继续估计并扣除剩余三轴零偏，驱动不再重复扣一次。四元数为 Hamilton `wxyz`，机体系到导航系，采用静止时加速度方向对齐导航 +Z 的约定。计时方法见 [INS 指南](../../../doc/IMU/README.md#一帧-ins-的完整耗时)。

正常输出继续更新 `INS.q`、`INS_Yaw_deg/Pitch_deg/Roll_deg`、姿态基向量和现有运动加速度。VQF 的标准 roll=X、pitch=Y，而历史 INS 字段是 Pitch=X、Roll=Y；适配层已显式换算且有回归验证。USB 使用已有四通道 `INS_Roll_deg, INS_Pitch_deg, INS_Yaw_deg, 温度`，现有 C++ viewer 能读取。四通道没有算法身份字段，不能仅凭数据包判断是不是 VQF。

可在调试器读取：

- `INS_VQF.filter.q`、`bias`：姿态与剩余零偏，bias 单位 rad/s。
- `INS_VQF.filter.restDetected`、`restT`：自动静止判定与持续时间。
- `INS_VQF.rest_bias_count`、`motion_bias_count`：进入对应偏置学习路径的次数，不保证每拍都有有效校正。
- `INS_VQF.status`、`rejected_count`、`numeric_fault_count`、`continuity_breaks`：本拍状态与异常计数。
- `INS_FrameTiming`：`last_us / mean_us` 记录本次 INS 取数到姿态输出的执行耗时；`drdy_to_data_us / data_wait_us / drdy_to_output_us` 单列传感器采集、缓存等待与链路总延时。单算法只保留这一份结果。
- `INS_Profile[INS_PROFILE_VQF]`：仅对比模式保留，记录 VQF 更新与 Euler 换算的计时；没有上板数据前不能报执行时间或 WCET。

## 算法思路与源码阅读顺序

本实现把姿态传播和三维零偏卡尔曼滤波分开。先读 [头文件](vqf_filter.h) 的参数、状态和单位，再沿 [VQF_Update](vqf_filter.c) 跟到核心计算：

| 顺序 | 入口 / 状态 | 关键作用 |
| --- | --- | --- |
| 1 | `VQF_Update` | 检查输入、饱和标志与 dt，复制候选状态；合法 dt 为 10 μs～20 ms |
| 2 | `vqf_set_timestep` | 按实际 dt 更新二阶低通系数、递归状态与偏置权重 |
| 3 | `vqf_update` / `gyrQuat` | 以 `gyro - bias` 的轴角增量右乘积分；没有额外陀螺软件低通 |
| 4 | `lastAccLp` / `accQuat` | 把加速度转到陀螺积分中间系再低通，构造指向导航 +Z 的倾斜修正并左乘 |
| 5 | `bias / biasP` | 静止时直接观测三轴偏置；运动时利用倾斜修正构造偏置观测 |
| 6 | `vqf_output` | 合成 `q = accQuat ⊗ gyrQuat`，更新旋转矩阵和欧拉角 |
| 7 | `VQF_Update` | 检查 q、bias、P、欧拉角的有限性，通过后提交整份状态 |

`gyrQuat` 表示机体到陀螺积分中间系，`accQuat` 表示中间系到导航系；中间系不是一个有绝对航向的世界参考。对旋转后的加速度低通，可减少机体转动对重力方向滤波的影响；陀螺仍逐帧传播，所以 `tauAcc=3 s` 不是“输出落后 3 秒”。持续线加速度仍可能污染倾斜与零偏学习。

偏置滤波的状态只有 `bias[3]`，姿态四元数不在这个卡尔曼状态中。代码用变量 `R` 存放观测矩阵 H：静止分支 H=I，残差为低通陀螺减当前 bias；运动分支使用低通旋转矩阵，水平残差来自倾斜修正，竖直方向只给很弱的趋零先验。增益为 `K = P Hᵀ (H P Hᵀ + W)⁻¹`，然后更新 `bias += K e`、`P -= K H P`。

参数 `biasSigma*` 是 °/s，内部 P/V/W 沿用 `(°/s × 100)²` 的统一尺度，尺度在 K 中相消；观测残差和 bias 始终是 rad/s。这里使用 3×3 逆矩阵和简化协方差更新，没有 Joseph 更新、条件数检验或正定投影；有限性检查不等价于完整数值稳定性保证。

### 低通初始化、断流与保护

- `filterVec` 在启动阶段按实际 dt 积分求均值，满时间常数后建立二阶稳态；状态中的 NAN 是初始化标记。
- dt 变化时，`adaptFilter` 补偿二阶延迟项；不只改系数，否则可能产生输出跳变。这是本适配的变步长扩展。
- gyro 无效、裁剪或 dt 越界：整拍拒绝，不积分缺口。accel 无效：只积分 gyro，重建静止窗口与低通历史。
- `VQF_BreakContinuity` 保留已提交的姿态、bias 和 P，清除静止计时与低通历史；重新连续采样后再学习。
- 候选状态非有限：保留上拍姿态和零偏；偏置观测矩阵求逆失败时只跳过该次偏置校正。路径计数不保证每拍都产生偏置改变量。

参数修改入口集中在 `vqf_init`；涉及时间常数、Sigma 的更改必须重新初始化派生系数。标定参数位于驱动，不能把 VQF 在线 bias 再叠加到驱动扣一次。

## 零偏学习与限制

保留提供代码及上游的默认设置：`motionBiasEstEnabled=true`、`restBiasEstEnabled=true`。静止检测使用陀螺与其低通值的差、加速度与其低通值的差，默认阈值分别为 2°/s、0.5 m/s²，持续 1.5 s；另检查陀螺低通每轴不超过 `biasClip=2°/s`。它不要求每次上电先静止，bias 从零开始；初始倾斜使用当前一帧有效加速度。

判为静止后，用陀螺低通值作为三轴偏置观测；运动阶段利用倾斜修正学习偏置，并弱约束导航竖直分量。`tauAcc=3 s` 是加速度低通时间常数，`biasSigmaInit=0.5°/s`、`biasSigmaMotion=0.1°/s`、`biasSigmaRest=0.03°/s` 等参数保持原值。状态 bias 是 rad/s，参数中的这些 Sigma/Clip/ThGyr 数值按上游约定使用 °/s，不可混用。

六轴没有绝对航向观测，自动静止检测也不能可靠区分偏置与所有慢转。回归中的真实 −0.5°/s、60 s 绕重力轴转动，默认自动静止得到约 −0.855°，关闭静止学习得到 −30.000°。这明确展示默认设置会把持续慢转学成偏置；静止效果好不能当作运动精度保证。接入应用时应分别验证动态精度、启动和温漂；在线学习结果不会自动持久化。

## 接口与输入保护

1. 移除 `All_define.h` 和未使用的全局单例依赖，补全初始化的输出、旋转矩阵、浮点范围保护以及上游作者/许可信息。
2. `vqf_filter.c` 内的框架入口提供有状态的 INS 边界：拒绝非有限、超量程 gyro 与非法 dt；加速度无效时仅积分 gyro；数值失败保留上一份有效状态。合并文件保留原有算法参数、状态布局和保护逻辑。
3. `dt` 使用相邻有效样本时间戳之差，不使用任务调度周期。当前 DRDY 路径时间戳来自中断，阻塞回退路径使用读取中点；传感器配置 400 Hz，INS 任务 3 ms，读取最近完整帧，超过 20 ms 不外推。`vqf_set_timestep` 按实际 dt 更新陀螺积分、偏置噪声和 Butterworth 系数，按上游 `filterAdaptStateForCoeffChange` 方法调整状态；启动滤波均值按经过时间加权。这是可变步长工程扩展，不宣称等同于上游固定采样理论保证。
4. 缺测、无效时间戳和陀螺裁剪会清除自动静止窗口并重建低通历史，保留姿态与已学 bias。加速度无效也停止该拍偏置学习并重建相关历史。断流恢复沿用 INS 的重新定时规则，不把下一帧角速度外推到整个缺口。
5. 保留 [上游 VQF 六轴实现](https://github.com/dlaidig/vqf/blob/main/vqf/cpp/vqf.cpp) 的主体算法；不引入磁力计、编码器观测，也不改 MEKF 算法。

## 回归验证

`tests/imu/vqf_test.c` 覆盖任意内存初值、倾斜静止时三轴 bias 学习、运动中启动、单位和轴方向、9/11/20 ms 交替 dt、yaw 跨圈、慢转的静止学习消融、NaN/裁剪/缺测恢复。`ins_vqf_test.c` 编译真实 INS 源码、mock 传感器边界，验证选择路径、使用 unfiltered 数据和 SPI dt、沿用历史角度命名，以及主输出和 bias 学习。

离线对照实例和来源指纹见 [开发验证记录](../../../diff.md#实测结论与已知限制)。运行回归：



```sh
cmake -S tests/imu -B build/vqf-host-tests -DCMAKE_BUILD_TYPE=Debug
cmake --build build/vqf-host-tests -j6
ctest --test-dir build/vqf-host-tests --output-on-failure
cmake -S tests/imu -B build/vqf-host-sanitizers -DMEKF_SANITIZERS=ON
cmake --build build/vqf-host-sanitizers -j6
ctest --test-dir build/vqf-host-sanitizers --output-on-failure
```

固件接入需将这对源码和头文件加入所选工具链，核对数学库、浮点 ABI、诊断源文件和链接布局，见 [编译与开发环境](../../../doc/IMU/编译与开发环境.md)。

## USB CDC 查看与记录

单 VQF 通过 USB CDC 发送 VOFA JustFloat 四通道。确认 `FRAME_ENABLE_INS`、`FRAME_ENABLE_USB_CDC`、`FRAME_ENABLE_VOFA` 均为 `FRAME_FEATURE_ENABLED`，`FRAME_VOFA_TRANSPORT=VOFA_TRANSPORT_USB_CDC`；设置 `INS_RUN_MODE=INS_MODE_SINGLE_VQF`。Cube 默认任务初始化 USB，通信任务每 3 ms 发送一次。协议依次为 `Roll_Y_deg, Pitch_X_deg, Yaw_Z_deg, temperature_C` 四个小端 float，结尾 `00 00 80 7f`。角度使用框架历史命名，不能交换 Pitch 与 Roll。

USB 数据线接主控板载 USB 口。Horco 探针连接只提供调试访问，其串口不是姿态数据端口；主机只出现 Horco 时不能通过它接收这四个通道。先确认 STM32 CDC 枚举，再用以下命令（端口替换为实际 STM32 设备）：

```bash
cmake -S tools/imu_viewer_cpp -B build/vqf-viewer -DCMAKE_BUILD_TYPE=Release
cmake --build build/vqf-viewer -j6
build/vqf-viewer/imu_viewer_cpp --algorithm VQF --port /dev/ttyACM1 \
  --record vqf-static.csv --record-seconds 120
```

`--algorithm VQF` 只设置人工显示标签，协议并不携带算法身份，也不会更改 MCU 算法。Viewer 显示一个姿态方块；三方块模式用于三算法对比组合，由带身份的协议自动标注。同一端口一次只能由 viewer 或 `imu_capture` 其中一个读取。已有 CSV 拒绝覆盖，意外断线会尝试重连并保留缺口；四通道没有设备时间和序号，不能仅靠 CSV 判断主控复位或精确计算传感器到画面的延迟。

无图形环境可用 `cmake -S tools/imu_viewer_cpp -B build/vqf-capture -DIMU_VIEWER_BUILD_GUI=OFF` 构建 `imu_capture`，具体录制参数见 [查看器说明](../../../tools/imu_viewer_cpp/README.md)。四通道不含 bias、静止判定及解算耗时，这些仍从匹配烧录 ELF 的 `INS_VQF` 和 `INS_FrameTiming` 读取（对比模式另有 `INS_Profile`），避免误套 MEKF 十通道诊断格式。

### USB 与解算周期对齐

通信任务按 `PROCOTOL_DEBUG_DIVIDER` 分频发送最新姿态快照，默认每 3 ms 一包；INS 任务周期独立配置。两任务不同相，不保证每包对应不同样本。调整频率后应检查接收速率、发送拒绝与任务超期。

标定操作、在线学习检查和滤波参数评估见 [VQF 原理与标定流程](../../../doc/IMU/VQF标定流程.md)。固定参数按 [BMI088 共用标定](../../../doc/IMU/BMI088标定流程.md) 测量，MEKF、旧 EKF、Madgwick、VQF 各自的在线学习独立验收。

## 计算开销优化

保留静止/运动零偏学习、双精度低通状态、实际时间戳步长和数值故障回退，不修改滤波参数、采样率或编译优化等级。实现中做了以下优化：

- 静止零角速观测的观测矩阵是单位阵，省去四次单位阵相关的 3×3 乘法。完整偏置协方差和运动低通历史继续更新。
- 时间步变化时，共用的低通系数差移到通道循环外；二阶低通直接批量更新，省去每通道函数调用和重复系数加载，保持算式顺序及 double 精度。
- 3×3 逆矩阵共享 double 行列式倒数，省去八次除法，保留奇异性判断；该变换可能产生末位舍入差，不能只凭代数等价代替轨迹验证。
- 候选状态和已验证状态仍各复制一次。通用 C 库的 memcpy 可能使用逐字节复制，对较大的滤波状态产生额外开销。局部 copy_filter 改用固定 8 字节块，让编译器生成字加载/存储；保留尾部处理，不覆盖全局 memcpy。通过字符指针和小块 memcpy 复制对象表示，不把 float/double 内存强制转为整数指针，因此不违反严格别名规则。

编译时保留有限值检查，不启用 fast-math。固定长度 memcpy 的展开依赖编译器内建识别，关闭内建或更换 C 库后应检查反汇编并重新计时。代码采用标准 C，armclang 同样需复核其生成代码；跨工具链的速度必须重新测量。

### 数值与异常保护验证

回归应覆盖静止/运动切换、噪声、变步长、异常加速度、陀螺裁剪、非法 dt 和学习开关组合；核对姿态、bias、P、状态位及故障回退。可启用 AddressSanitizer/UndefinedBehaviorSanitizer 检查内存和未定义行为。

数值轨迹及板上计时实例见 [开发验证记录](../../../diff.md#实测结论与已知限制)。

## 下载来源留存

移植依据为 2026-10-02 归档的四份输入原件。下载版 BMI088 文件头作者为 CaoKangqi、日期为 2026/7/6，依赖原工程的 `BSP_SPI`、`All_define` 和 GPIO 配置，没有替换框架驱动。

下载版 VQF 文件头没有独立作者或许可声明；其六轴实现源于 [Daniel Laidig 的 VQF](https://github.com/dlaidig/vqf)，适配代码保留该上游的署名和 MIT 许可，不为 BMI088 原件补造许可声明。原件归档时按下列 SHA-256 核对内容：

| 原文件名 | SHA-256 |
| --- | --- |
| BMI088.c | `2d2acf51e7b34db5de42aee7429a8ea994c22e3fa280edb82cc72a1dba3b7ac5` |
| BMI088.h | `aa25ec13095206304bfd0dfc2441dde21864cfa01e4309a75919323fad809a54` |
| VQF_filter.c | `fe4d969f9e830a650ad77be312069d858e2d0fe130c5a8ccf347a9428cae947a` |
| VQF_filter.h | `70dfc5f4d760b67310ccefbb68351d2d461c892f1b30b8d88c0a68dc22b4b1e5` |
