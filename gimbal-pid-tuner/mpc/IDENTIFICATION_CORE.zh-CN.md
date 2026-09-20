# 双 yaw 辨识集成与现场步骤

> 本文是完整开发时间线，保留历史命令用于追溯，不作为当前现场操作入口。当前状态、安全审核和唯一有效
> 现场步骤从
> [MPC 辨识项目当前状态、文档导航与安全机制](MPC_PROJECT_STATUS_AND_SAFETY_20260910.zh-CN.md)
> 进入。

## 项目背景

本项目面向 RoboMaster 高校联盟赛哨兵竞赛机器人，当前研究问题是大 yaw 与小 yaw 的耦合控制。
用户说明测试期间发射功能已停用、接线已断开；源码仍保留相关模块。
详见[项目背景与当前状态](../../PROJECT_CONTEXT.zh-CN.md)。
背景说明不代表模型有效或批准部署 MPC；当前实现进度见下节。

## 当前进度

大、小 yaw CAN扩展版正向采集均已完成并通过结构复核，不要重复执行此前步骤。
分别见[大 yaw复核](BIG_CAN_TRACE_REVIEW_20260910.zh-CN.md)和
[小 yaw复核](SMALL_CAN_TRACE_REVIEW_20260910.zh-CN.md)。两轮实测微秒区间和指令积分已经合并辨识，
但候选模型的长时预测严重失败，见[CAN扩展数据辨识复核](CAN_TRACE_MODEL_REVIEW_20260910.zh-CN.md)。
离线闭环辨识和电流偏置/时序建模已经完成首轮，见
[专项复核](CLOSED_LOOP_CURRENT_REVIEW_20260910.zh-CN.md)。随后设计并完成 `0x59490405` 双参考 A/B
实车记录；两组数据均合格，但冻结 A→B 和 B→A 的每关节 30% 预测门槛失败，ARX 结构也不稳定，见
[双参考 A/B 独立验证](DUAL_AB_59490405_VALIDATION_20260910.zh-CN.md)。电流增益/时序能够跨相位复现，
闭环运动模型仍被拒绝。随后完成 A/B 残差、摩擦与公共结构分析，并冻结新的 36 秒 C/D 离线草案，见
[C/D 设计复核](NEXT_STAGE_CD_DESIGN_20260911.zh-CN.md)。当前不安排新实车采集；C/D 尚无固件实现，
MPC不接管。
下方旧阶段命令保留作历史参考。

### 最新：CAN 采集扩展已实现，实机零输出 BENCH 已通过

新构建 `0x59490301` 配合 `yaw_can_trace_cli.py`，以 152 字节记录保留区间指令积分/极值、
尝试与入队/完成/错误次数、反馈电流和统一微秒时间；串口帧 163 字节，CRC16，缓冲320条。
6 个原生 C 程序通过，相关 Python 回归81项通过、1项跳过，MPC73项通过；最终增量构建通过。
用户已手动烧录，回传 id=1 BENCH 通过：5001条、CRC错误0、质量告警为空。
每轴20002次入队、20000次完成、末条2次待完成；这是边界待完成记录，不能当作丢帧。
原始采集目录已通过 SSH/SCP 取回，本地5001条回放及报告比对通过。
发现并修复主机 CSV 的 rpm 重名导出问题，已从完整原始流重生成唯一列名CSV并逐字段验证；
无需重烧或重采，NUC主机脚本在下次采集前需更新。
区间3848–4133 μs、均值3999.9952 μs；零指令下电流原始值均值约217.22/−107.14，
保留作后续分析，不直接自动归零或换算力矩。尚未执行新版本运动辨识。
协议、限制和复现见 [CAN 采集扩展](CAN_TRACE_20260910.zh-CN.md)。以下各节为历史阶段记录。

### 此前：离线输入时间对齐后合成恢复通过，实车仍未通过

已把中心角度差分与执行器三角窗口输入积分对齐，并统一六状态预测器的时间定义。
量化合成数据最大参数误差6.85%，正确选中20ms滞后；冻结参数后的新相位预测通过合成诊断门槛。
原实车两轮的长时预测仍失败，不接管MPC。新增11项测试，MPC目录共73项通过。
详见[输入时间对齐复核](TIMING_ALIGNMENT_20260909.zh-CN.md)。
下一步优先完善CAN区间指令、发送状态和反馈电流记录；本轮未修改固件或访问硬件。

### 上一阶段：合成参数恢复暴露量化敏感性

现有静摩擦拟合器在已知合成系统加入编码器量化后，阻尼恢复误差达99.89%。
新增加权角度积分拟合将该误差降至26.38%，仍超过预设20%门槛；原实车数据长时预测也失败。
新增10项测试，MPC目录共62项通过。未修改固件/PID、访问硬件或使用反向轮重拟合。
详见[合成参数恢复复核](STICTION_RECOVERY_20260909.zh-CN.md)。
输入时序/电流记录扩展及闭环偏差问题仍待解决；下方各节为历史进展，不重复其中采集命令。

### 当前：历史串口文件被动审计完成

新增只读完整性审计，三份记录均为5001点，无CRC错误、杂字节丢弃、截断或摘要不一致。
大yaw正向、小yaw反向各保留两个非4ms间隔告警，最大网格偏差1ms。
本轮未改固件/控制逻辑，未打开串口或操作电机；无需烧录或重复采集。
详见[CAPTURE_INTEGRITY_20260909.zh-CN.md](CAPTURE_INTEGRITY_20260909.zh-CN.md)。
这不是CAN发送时序验证；固件内部指令历史/反馈电流扩展尚未实施，模型仍不可部署。

### 当前：输入时序核对完成，静摩擦候选仍不通过

已确认旧离线速度估计在匀加速情况下存在20ms时间错位，并修正在新的离线实验工具中。
已核对软件输出到CAN的倍率一致，但4ms采样未保留全部控制周期输入或发送确认。
新静摩擦候选整段预测仍失败，维持原固件/PID，不烧录、不部署MPC、不重复下方历史采集命令。
详见[INPUT_TIMING_REVIEW_20260909.zh-CN.md](INPUT_TIMING_REVIEW_20260909.zh-CN.md)。
下一步先设计采集时序、区间指令统计和电流/发送状态记录，再审核实车试验。
以下各节为历史阶段记录，以本节为准。

### 当前：反向验证完成，停止重复采集，候选不接管

小yaw反向id=6已正常采完。用冻结候选验证，未重拟合；滚动200ms航向误差0.091°，
但从2秒起不再用实测状态修正的18秒预测误差6.997°，未复现实际运动。
本轮采集有用，失败的是候选模型验证，不是实车闭环功能。
保持现有PID/固件，暂不执行下文历史采集步骤，也不部署MPC。
结果与下一阶段方向见[REVERSE_VALIDATION_20260909.zh-CN.md](REVERSE_VALIDATION_20260909.zh-CN.md)。

### 最新：结构化双轴离线模型已实现，待独立反向验证

已实现单位角度尺度、四机械状态、正定耦合惯性、阻尼/摩擦和执行器响应模型。
200ms航向预测误差约0.110°/0.099°，但小yaw参数贴搜索边界，尚未稳定优于同条件简单基线。
不接管MPC，不改PID、不烧录。22项测试通过，包括已知合成系统参数恢复。
下一步是相同幅度时序的小yaw反向独立验证，先负后正，现有固件已支持。
完整结果和逐步命令见[COUPLED_MODEL_REVIEW_20260909.zh-CN.md](COUPLED_MODEL_REVIEW_20260909.zh-CN.md)。
下文“暂不需要采集”为此前阶段记录；现在仅安排上述一轮，不批量自动执行。

用户已确认双yaw同尺度直驱、平行竖直、不同轴安装，且IMU闭环工作良好。
后续按单位角度尺度与双自由度耦合模型推进，不采用0.851/0.946描述性回归值作为标定。
补充gyro积分核查已记录在双轴复核报告顶部；当前不改PID/IMU、不要求再次采集。

### 当前：两轮慢速采集完成，进入离线模型复核

小yaw±10° id=4已完成：5001点、所有间隔4ms、无质量告警、无CRC/反馈故障。
与大yaw±15° id=2一起完成原始流回放、配置比对和第一组ARX候选拟合。
用户确认两轮底盘没动。大yaw试验实际编码器范围-14.06°~+15.25°，并非未转动；
小yaw试验大yaw仅变化0.044°，属于保持目标下的观测。
首组离线候选在200ms预测上未优于匀速基线，尚不可部署MPC。PID和固件本轮均未改动。
详见[SLOW_PAIR_REVIEW_20260909.zh-CN.md](SLOW_PAIR_REVIEW_20260909.zh-CN.md)。
现在保持上置，不用再运行下方历史采集命令；先离线核对角度关系和模型，再安排独立验证。
以下为此前流程记录。

### 最新：大yaw慢速采集完成，下一步小yaw航向±10°

0x59490203的BENCH与大yaw±15°采集均已在实车完成。大yaw试验id=2、5001条、20秒，
phase=5/reason=0，CRC错误0，无反馈故障；仅有1个5ms和末尾1个3ms间隔。
保留严格时间告警和原始时间戳，不因此立即重采；可继续离线探索，但模型仍未辨识/验证。
数据响应及时间统计见[SLOW_BIG_REVIEW_20260909.zh-CN.md](SLOW_BIG_REVIEW_20260909.zh-CN.md)。

保持右拨杆上置，车体固定、清空旋转范围、保持原PID/负载/Pitch姿态，先单独释放：

```bash
cd /home/nuc11--02/yaw-slow-59490203-feedback
python3 -u yaw_slow_cli.py \
  --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  --action release --confirm REMOTE_UP \
  --output captures/release_$(date +%Y%m%d_%H%M%S)
```

确认释放成功后，单独执行小yaw航向±10°采集，大yaw保持起始关节角：

```bash
python3 -u yaw_slow_cli.py \
  --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  --action trial --axis small --amplitude-deg 10 \
  --confirm FIXED_CHASSIS_CLEAR_YAW \
  --output captures/small_slow_amp10_$(date +%Y%m%d_%H%M%S)
```

收到ARM accepted后15秒内下置，不动摇杆、不手推云台；异常立即上置，结束也回上置。
结束时Pitch保持力矩撤去，提前做好不干涉yaw运动的防落措施。BENCH资格有效10分钟，
过期先重新执行下方BENCH命令并单独释放，不绕过门禁。正式运动起点仍需满足限位余量。
本次只读取保存的数据并离线统计，未改固件、未开串口、未自动释放或启动小yaw运动。
以下保留此前进度记录。

### 当前：反馈竞态修复与故障快照版 0x59490203，等待烧录

实车大yaw±15°慢速试验id=2，在基线阶段1.115秒以reason=3（反馈无效/过期）中止，
尚未进入2秒后的激励。原始数据已取回到本机
`NoMachineTemp/big_slow_amp15_20260909_205716`，280条、CRC错误0；
277个间隔4ms、1个5ms、末尾1个2ms。本轮不能用于辨识。
旧版终止记录为停机后重新取样，年龄0/1/0ms且反馈已恢复，无法确定触发故障的具体来源。

已修复电机离线判断的时间戳/在线标志中断竞态（不等于已证明它是这轮故障根因）。
新版冻结触发reason=3的观测，终止记录的反馈有效位、年龄和flags[15:12]描述该故障观测，
其他角度/输出字段仍是停机后采样。高位分别为INS未就绪、大电机离线、小电机离线、数据/模式无效。
脚本报告新增`feedback_failure`，可区分状态无效与年龄超时。PID、±10°居中和全部保护阈值不改。
本机40项Python测试、原生C测试和电机竞态测试通过；AC6重编译0错误、69个既有警告。
未自动烧录、释放、打开实车串口或发送运动命令。

先上置并防止Pitch下落，重新烧录云台`AGVSentinel_Gimbal.axf`，确认Reset/Run。
使用新版独立目录重新做零输出BENCH（重启会清除原BENCH资格）：

```bash
cd /home/nuc11--02/yaw-slow-59490203-feedback
python3 -u yaw_slow_cli.py \
  --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  --action bench --confirm BENCH_REMOTE_UP \
  --output captures/bench_$(date +%Y%m%d_%H%M%S)
```

全程上置，完成后回传结果。以下为旧版历史，不要混用旧目录脚本。

### 最新：0x59490202 实车 BENCH 已通过，下一步大 yaw 慢速采集

用户回传实车结果：trial id=1，5001条记录，CRC错误0，quality_issues为空，
phase=5、reason=0、flags=253、extra=5、bench_passed=true。
这确认零输出流式链路通过，不代表动力学模型已辨识或允许MPC接管。
BENCH仍占用控制权，先保持右拨杆上置，单独释放：

```bash
cd /home/nuc11--02/yaw-slow-59490202-center10
python3 -u yaw_slow_cli.py \
  --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  --action release --confirm REMOTE_UP \
  --output captures/release_$(date +%Y%m%d_%H%M%S)
```

释放成功后，车体固定、旋转范围清空、保持相同负载和Pitch姿态，运行已确认的±15°大yaw慢速采集。
小yaw参与航向保持。正式采集小yaw起点需满足[-10°, +4.08°]的行程余量，不能用BENCH的±10°代替。
BENCH资格有效10分钟，超时或主控重启后需重新执行BENCH并释放，不绕过检查。

```bash
python3 -u yaw_slow_cli.py \
  --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  --action trial --axis big --amplitude-deg 15 \
  --confirm FIXED_CHASSIS_CLEAR_YAW \
  --output captures/big_slow_amp15_$(date +%Y%m%d_%H%M%S)
```

收到`ARM accepted`后15秒内下置，摇杆保持回中，不手动推动两轴；异常立即上置。
20秒轨迹为保持2秒、到+15°用2秒、停3秒、到-15°用4秒、停3秒、回起点用2秒、保持4秒。
结束后立即上置并保留输出目录。结束会撤去Pitch保持力矩，提前做好不干涉yaw运动的防落措施。
先回传这轮报告，不自动释放并串行启动下一轮。以下保留此前交付步骤。

### 当前：居中容差放宽版，需重新烧录云台板

上次实车 BENCH 在启动前因 `flags=188` 被拦截，未发生采集或运动。
新固件 `0x59490202` 将小 yaw 居中容差由 ±2° 放宽为 ±10°，BENCH 不再要求预留运动行程。
两轴静止检查（各 ≤6°/秒）、上置、反馈新鲜、零输出及限位检查保留。PID 和正常遥控不改。
正式慢速运动仍预留小 yaw ±20° 行程及两侧 5° 余量，实际允许起点约为 [-10°, +4.08°]；
下置启动瞬间也重新检查余量。BENCH 允许 ±10° 不代表该位置可执行正式运动。
新脚本目录为 `/home/nuc11--02/yaw-slow-59490202-center10`，与旧版分开。
请烧录新 `AGVSentinel_Gimbal.axf` 并确认 Reset/Run。尚未自动烧录或执行真实 BENCH。

关闭调参网页和其他串口程序，保持上置、车体固定，并做好 Pitch 防落。
在小车 PC 终端执行以下命令。**本次全程上置，不要下置；yaw 与 Pitch 都没有驱动/保持输出。**

```bash
cd /home/nuc11--02/yaw-slow-59490202-center10
python3 -u yaw_slow_cli.py \
  --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  --action bench --confirm BENCH_REMOTE_UP \
  --output captures/bench_$(date +%Y%m%d_%H%M%S)
```

约 20 秒后应显示 `BENCH PASSED`，报告 `records:5001`、`quality_issues:[]`、`bench_passed:true`。
完成后仍保持上置，先把结果发回；不自动释放或开始运动。
若失败，保留报告及 `raw.bin`，不要反复布防或用旧脚本补下载。
新环形缓存无法事后重下已确认回收的整轮数据，主机必须保存本次运行目录。

人工确认要释放已结束的本轮时，可单独使用以下命令；它会查询当前 id，不硬编码旧编号。
**当前先做上面的 BENCH，不要把释放和运动试验串成一个自动流程。**

```bash
python3 -u yaw_slow_cli.py \
  --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  --action release --confirm REMOTE_UP \
  --output captures/release_$(date +%Y%m%d_%H%M%S)
```

CLI 支持新慢速 trial，但本阶段尚不启动运动。原4秒回放工具保留，不能用它解析20秒流。
本次本机新旧协议/命令测试39项及原生C测试通过，AC6重编译0错误、69个既有警告。
这些是软件验证，不是实车运动或真实BENCH结果。

### 上一步：慢速流式固件已编译，当时等待烧录和新版主机脚本

已实现云台固件 **0x59490201**：20 秒、4 ms 流式记录，大 yaw 最大 ±15°、小 yaw 航向最大 ±10°，
包含上置零输出 BENCH 测试、累计接收确认、缓存/传输故障退出和新模式布防门禁。
Keil AC6 完整重编译 0 个错误、69 个既有警告；没有自动烧录。
详见 [固件交付与新协议](../../AGVSentinel_v10090/AGVSentinel_gimbal/YAW_SLOW_STREAM.md)。

**这份固件的 USART1 已改成 460800、8N1。旧 115200 调参页面、旧采集脚本不能直接使用。**
只需云台板烧录 `AGVSentinel_Gimbal`；烧录前后保持上置并做好 Pitch 防落，确认 Reset/Run 后程序启动。
底盘板不改。配套主机脚本、真实无输出链路测试尚未完成，因此仍不要执行下方历史 ARM/RELEASE 命令。
本次没有串口操作、释放控制权、物理运动或自动烧录。

### 上一步：方案已获接受，当时尚未完成实车实现

根据用户希望增强激励的要求，已生成大 yaw ±15°、小 yaw 航向 ±10°、单轮 20 秒的
[审核稿与离线预览说明](SLOW_EXCITATION_REVIEW.zh-CN.md)。采用平滑移动、正负各 3 秒停留，
不是新增电机限速；PID 和当前实车配置未修改。
现有固件仅支持幅度参数最大 5°、4 秒缓存。新方案需要先升级采集链路及协议，
**现在不要把下面旧命令的 `--amplitude-deg` 改成 15 或 10，也不要重复布防。**
用户已接受上述幅度和时序并请求执行命令，不再重复请求同一参数审核。
核对代码后，当前仍为固件 `0x59490106`、最大幅度参数 5°、1001 点缓存；
20 秒连续采集固件、传输协议与主机支持尚未实现，因此目前没有对应的实车命令。
必须完成实现、构建、烧录及无电机输出链路验证后，再提供新采集命令。
本次没有串口操作、释放、烧录或实车运动；方案接受不代表新固件已存在。

### 上一步：首轮离线拟合完成，多步预测未通过，MPC 不接管

首组大 yaw 5°（ID 3）与小 yaw 3°（ID 4）记录均已复制到本机、独立重放通过。
每轮 1001 点、4 秒、全部间隔 4 ms、无 CRC 错误、DONE/reason=0；PID 与固件保持不变。
小 yaw 轮关节运动约 2.241°，固定目标的大 yaw 偏移到约 -0.659°，终止时约 -0.439°；
小 yaw 停止段航向误差 RMS 约 0.014°。详细数据、配对检查和局限见
[首组双轴检查报告](IDENTIFICATION_PAIR_REVIEW_2026-09-09.zh-CN.md)。

已完成两输入、三输出 ARX 候选拟合，以及连续保留数据段的预测检验。
200 ms 航向预测 RMS 误差：大 yaw 轮约 0.500°，小 yaw 轮约 0.130°；
均差于保持当前位置的基线（约 0.025° / 0.044°），不能作为可用实车模型。
方法、复现命令、结果和下一轮激励建议见
[离线模型检验报告](OFFLINE_MODEL_REVIEW_2026-09-09.zh-CN.md)。
**现在不用继续改 PID、加幅度、烧录或执行下面的历史采集命令**。MPC 仍未接管。
最后一次结束报告仍锁定 ID 4，上置、两 yaw 指令零；这是历史记录，不是实时查询。
本次仅本机离线分析，没有串口操作、释放或重新布防。
保持上置并做好 Pitch 防落支撑，不主动松开 Pitch 或反复释放。

## 历史记录（不要复制旧试验命令）

### ID 3（大 yaw 5°）完成，当时建议小 yaw 3°

**当前只使用本节命令，不再重复大 yaw 加幅试验；下面全部是历史记录。**

已复制 NUC `captures/big_pid1_amp5_20260909_161356` 至本机
`NoMachineTemp/yaw-ident-pid1-amp5-20260909-161356`，原始流离线重放通过。
ID 3、幅度参数 5°、固件 0x59490106，PID 未变；1001 点、4 秒、全部间隔 4 ms，
DONE/reason=0、CRC=0、格式质量无异常。5° 是组合波形幅度参数，实际目标峰值约 ±2.91°。
大 yaw 实际相对启动位置 -1.362305..+0.703125°，峰峰值 2.06543°（47 个计数），
速度 ±3 rpm，输出 -0.413..+1.300 / 30。小 yaw 关节范围 8.261719..8.833008°，
共 13 个计数，速度仅 1 个采样非零。停止段大 yaw 保留约 -0.219727° 位置偏差，
输出约 +0.212..+0.223，但编码器不变。这与低速摩擦/死区相符，但不是已证实的唯一原因。

这轮没有保护中止，且记录中没有之前的增长振荡；不意味着全部工况稳定。
数据可保留用于低幅闭环响应分析，但尚不能宣称得到可靠的双轴惯量/延迟模型。
本轮小 yaw 起点 +8.833°，前轮为 -8.525°；Pitch/IMU roll 起点约 -6.872°，前轮约 -2.898°。
三轮不是严格同姿态对照，不能把差异全部解释为激励幅度效应，也不要直接拼接原始航向。

**下一轮保持本轮姿态、负载和 PID，不再加大 yaw 幅度，改为小 yaw 航向 3° 独立激励。**
大 yaw 仍在固定编码器参考上闭环保持，不是断电；两路驱动都记录。现有固件已支持，不需重烧。
先固定车体、做好 Pitch 防落支撑、右拨杆保持上置。记录终点仍锁定 ID 3，
此次分析没有串口操作、RELEASE、ARM 或参数写入。

若主控未重启且仍持有 ID 3，先释放：

```bash
cd /home/nuc11--02/yaw-ident-pid1-KhRIhXIg
python3 -u yaw_identification_cli.py \
  --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  --action release --trial-id 3 --confirm REMOTE_UP \
  --output captures/release_3_$(date +%Y%m%d_%H%M%S)
```

释放成功后再执行。如果重复释放报告 `owned:false, phase:0`，表示已经空闲，不必反复释放；
其他错误先处理，不强行继续。若重启过或编号不同，以实际查询结果为准。

```bash
python3 -u yaw_identification_cli.py \
  --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  --action trial --axis small --amplitude-deg 3 \
  --confirm FIXED_CHASSIS_CLEAR_YAW \
  --output captures/small_pid1_amp3_$(date +%Y%m%d_%H%M%S)
```

看到 `ARM accepted` 后 15 秒内下置，摇杆不动；结束或异常上置，等下载完成。
退出会撤掉 Pitch 保持力。先核查这组独立输入数据，再决定是否需要更低频/更长的激励；
不自动释放或接续试验，不因此放宽保护，MPC 仍未接管。

### ID 2（3°）完成，曾建议大 yaw 5°

**下一轮只使用本节命令；后面的 1°、3° 和旧编号命令均为历史记录。**

已读取 NUC `captures/big_pid1_amp3_20260909_160121`，本机副本
`NoMachineTemp/yaw-ident-pid1-amp3-20260909-160121`。原始流离线重放通过。
ID 2、幅度参数 3°、固件 0x59490106，PID 保持 1/0/5、0.5/0.0001/0。
1001 点、4 秒、1000 个间隔全部 4 ms；DONE/reason=0，CRC=0，无格式质量异常。

大 yaw 相对启动角度 -0.615234..+0.307617°（总行程 0.922852°，21 个编码器计数），
速度 -2..+2 rpm，1001 点中仅 56 点速度非零。目标组合波形峰值 ±1.74°；
大 yaw 输出 -0.500..+0.806 / 30，未接近输出上限。小 yaw 关节总变化仅 3 个计数，
速度只有 5 点非零，航向相对启动 -0.0145..+0.2496°。
相比 1° 轮运动更明显，但尚不足以确认双轴惯量/延迟模型；不能将下载质量通过视为动力学辨识成功。

下一轮建议保持 PID、负载、Pitch 和所有保护不变，**仅把大 yaw 幅度参数改为 5°**，
这是当前已支持的最大幅度参数，不需重烧固件。此次未写参数、未 RELEASE、未 ARM。
若该轮仍运动不足，先检查激励频段、摩擦与反馈分辨率，不继续无界增加幅度或绕过保护。

记录结束时上置、yaw 指令零、控制权仍属于 ID 2。若主控没有重启，固定车体、保持相同负载姿态、
做好 Pitch 防落支撑，右拨杆保持上置，先单独执行释放：

```bash
cd /home/nuc11--02/yaw-ident-pid1-KhRIhXIg
python3 -u yaw_identification_cli.py \
  --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  --action release --trial-id 2 --confirm REMOTE_UP \
  --output captures/release_2_$(date +%Y%m%d_%H%M%S)
```

释放成功且仍上置后，再执行（释放失败不要继续；若主控已重启先查询状态）：

```bash
python3 -u yaw_identification_cli.py \
  --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  --action trial --axis big --amplitude-deg 5 \
  --confirm FIXED_CHASSIS_CLEAR_YAW \
  --output captures/big_pid1_amp5_$(date +%Y%m%d_%H%M%S)
```

看到 `ARM accepted` 后 15 秒内下置，摇杆不动；结束或异常上置，等待下载完成。
退出会撤掉 Pitch 保持力。下一轮仍先收集大 yaw 输入，尚未完成小 yaw 独立激励与模型验证。

### ID 1 完整结束，但激励偏弱

NUC：`/home/nuc11--02/yaw-ident-pid1-KhRIhXIg/captures/big_pid1_20260909_154215`。
本机副本：`NoMachineTemp/yaw-ident-pid1-20260909-154215`，原始数据离线重放一致。
固件 `0x59490106`，截图 PID 已生效。1001 点、4 秒、全部 1000 个间隔均为 4 ms，
DONE/reason=0，CRC=0，下载完整，数据格式质量检查无异常。

但本轮大 yaw 实际角度仅相对锚点 ±0.131836°（总计 6 个编码器计数），
激励段目标实际峰值 ±0.58°；`amplitude-deg` 是组合波形的幅度参数，不保证实际峰值等于该数。
大 yaw 速度采样仅 -1..0 rpm，小 yaw 全部为 0 rpm；大 yaw 软件输出 -0.236..0.267，远未到 30 上限。
这是低幅基线，不是完整动力学辨识成功：运动与速度变化不足，尚不能可靠辨识耦合惯量、阻尼和延迟。
小 yaw 关节 -8.569..-8.394°，航向总变化约 0.188°；没有仅凭这些数据判定 IMU 故障。
最终报告为上置、两 yaw 指令零、控制权仍锁定 ID 1；这描述记录终点，不是新的实时串口检查。

**建议下一轮保持 PID、工况和全部保护不变，仅把大 yaw 激励幅度参数从 1 提至 3。**
不需为此重烧固件。固定车体、保持相同 Pitch 姿态和负载，做好 Pitch 防落支撑；先右拨杆上置。
本次分析没有执行 RELEASE 或 ARM。若主控未重启、仍持有 ID 1，可在 NUC 执行：

```bash
cd /home/nuc11--02/yaw-ident-pid1-KhRIhXIg
python3 -u yaw_identification_cli.py \
  --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  --action release --trial-id 1 --confirm REMOTE_UP \
  --output captures/release_1_$(date +%Y%m%d_%H%M%S)
```

确认释放成功、仍上置后，再单独执行；不要在释放失败时继续：

```bash
python3 -u yaw_identification_cli.py \
  --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  --action trial --axis big --amplitude-deg 3 \
  --confirm FIXED_CHASSIS_CLEAR_YAW \
  --output captures/big_pid1_amp3_$(date +%Y%m%d_%H%M%S)
```

看到 ARM accepted 后 15 秒内下置，摇杆不动；结束后上置并等下载完成。明显振荡立即上置。
如果中途重启过主控，不沿用旧试验编号释放，应先查询实际状态。本轮尚未拟合模型，MPC 未接管。

### 截图 PID 固化与辨识速度放宽（0x59490106）

大 yaw 固件启动默认值：角度 Kp/Ki/Kd=`1/0/5`，速度 Kp/Ki/Kd=`0.5/0.0001/0`，
输出上限 `30`，速度滤波时间常数 `0.03 s`。小 yaw、Pitch、遥控选轴逻辑未改。
按用户要求，只将辨识大 yaw 的速度退出阈值从 `180°/s` 放宽至 `360°/s`（60 rpm）；
小 yaw 仍为 `180°/s`。相对初始位置最大行程 `12°`、小 yaw 软限位内侧 `5°` 余量、
ARM 前静止/回中判据、遥控急停、心跳、反馈新鲜度和配置一致性检查均保留。
手动大 yaw 模式仍不按速度幅值停机。放宽阈值不等于已证明闭环稳定。

**先烧录新版云台固件，并 Reset and Run；底盘不需重烧。** 此次只编译和部署脚本，未烧录、
未打开实际串口、未 ARM/RELEASE 或改写板上 RAM 参数。新标记可区分不同辨识保护配置，
协议结构不变，旧记录仍可解码；试验必须使用下面的新目录，旧脚本不能布防新版固件。

NUC 上的新脚本已部署：`/home/nuc11--02/yaw-ident-pid1-KhRIhXIg`。
部署路径修复：配置必须位于 `mpc/identification_setup.json`，已补齐并在 NUC 验证主入口。
此前缺文件错误发生在串口打开和 ARM 之前，不需要为这个错误重新烧录或 RELEASE；
可重跑下面带时间戳的命令，之前留下的空输出目录保留不删。
固定车体、确保轴和线缆有空间，做好 Pitch 防落支撑，右拨杆上置、摇杆回中，关闭占用串口的网页和视觉程序。
先运行一轮大 yaw，幅度仍为 `1°`，不同时增加激励幅度：

```bash
cd /home/nuc11--02/yaw-ident-pid1-KhRIhXIg
python3 -u yaw_identification_cli.py \
  --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  --action trial --axis big --amplitude-deg 1 \
  --confirm FIXED_CHASSIS_CLEAR_YAW \
  --output captures/big_pid1_$(date +%Y%m%d_%H%M%S)
```

看到 `ARM accepted` 后 15 秒内将右拨杆下置，摇杆全程不动；辨识期间左拨杆不接管。
看到 `Trial: done/aborted` 后右拨杆上置，等待 `Download complete`。剧烈振荡时立即上置，
不要为了凑满样本继续等待；完成或退出都会撤掉 Pitch 保持力。结束后控制权仍锁定，
不要直接重复命令。先查看记录及真实试验编号，再决定是否释放和继续。

AC6：0 错误、69 条现有警告。PID 算法、辨识核心及生产辨识状态机桩测试通过；
本机 Python 51 项（50 通过、1 跳过）；NUC 离线协议/CLI 18 项（16 通过、2 跳过，NUC 无本机 C 桩数据）。
构建日志：`NoMachineTemp/ident-pid1-rate360-20260909.log`。
AXF SHA256：`27ADDA8FCA55B8BC3845C98067DF0217052260E78DF6F6C090B8E0632649259D`。

下方为历次修改与采集记录；与本节冲突的旧参数、版本和“暂停重试”提示仅描述当时状态。

### 2026-09-09 遥控轴选择版（待烧录验证）

为先排查大 yaw 闭环，新增独立手动入口；没有调整 PID，没有恢复自动跟随。
**只改云台固件，底盘不用重新烧录。** 先保持右拨杆上置、摇杆回中，烧录后 Reset and Run。

| 右拨杆 | 左拨杆 | yaw 摇杆作用 |
| --- | --- | --- |
| 上位 | 任意 | 原急停，两个 yaw 无输出 |
| 下位 | 上位或中位 | 原小 yaw 航向控制，大 yaw 仍隔离 |
| 下位 | 下位 | 大 yaw 编码器位置闭环；摇杆回中后保持目标位置 |
| 中位 | 任意 | 原 AUTO/PC 模式，不启用新增大 yaw 手动入口 |

“左拨杆”指三档开关，不是摇杆。仍使用原 yaw 摇杆左右通道 ch2。
选择大 yaw 时，小 yaw 不再接收遥控 yaw 增量，但**原 IMU 航向闭环、PID、软限位仍工作**；
Pitch 和已有视觉处理不变。大 yaw 转动时小 yaw 会反向补偿，并可能接近其软限位，
因此这不是“小 yaw 完全不动”的单轴试验，也不会自动重新居中。

大 yaw 切入时以实测角度作为新目标，清除旧 PID 状态，之后按实际周期积分摇杆速度；
回中死区为 ±10 通道计数，仅作用于新增大 yaw 入口。左右满杆约 132°/s，
输出上限仍为 30，未另加正常速度目标限幅。
**按用户要求，已取消新增大 yaw 手动模式的 30 rpm 超速停机**；有限的速度反馈不会仅因幅值触发退出。
右拨杆上置急停保留，但人工急停不能保证及时阻止高速振荡。
非有限数值和控制周期异常仍会锁存故障，右拨杆上置复位，调试变量为 `GimbalYaw_DiagBigManualFault`。
本次不修改系统辨识的速度保护，也不调整 PID、输出上限或摇杆灵敏度。
失联、指令超过 50 ms、切换离开大 yaw、调参独占均停止这个入口；输出发送前再次检查。

左拨杆原有的遥控发射使能用途取消，避免选轴时使能发射；PC 显式发射路径未改。
辨识独占期间左拨杆不能接管，仍须遵循原 ARM/退出/释放流程，本次没有自动 RELEASE。
辨识协议格式和标签 `0x59490105` 不变，旧脚本兼容；此前不合格采集仍不能用于模型拟合。

以下为之前的辨识过程记录，不是要求立刻重新采集。

取消手动超速停机后 AC6：0 错误、69 条现有警告；手动入口 C 测试通过，
覆盖正反向超 30 rpm 不退出、撤销许可仍停止和异常数值锁存。
Python 49 项中 48 通过、1 跳过。尚未烧录或实机验证。
构建日志：`NoMachineTemp/manual-yaw-no-speed-trip-20260909.log`。
云台 AXF SHA256：`F5B05C00F1D62C47240981DB6AB31CD01BB5C584D6995872F2F705624667F94C`。

工况编号：`fixed_chassis_no_camera_v1`，记录见 [identification_setup.json](identification_setup.json)。

- 按已确认的固定车体、常用 Pitch 姿态、未装相机负载继续，不等待相机安装。
- 不假设相机影响可忽略；安装后需要重新检查模型预测是否适用。
- 专用辨识协议、控制权管理、紧凑采集与主机工具已经接入，并通过本地编译和离线测试。
- 用户已运行逐项诊断版。只读快照确认 ch0..ch3 全为 0，鼠标输入全为 0，唯一失败项是 ch4=-660（旧判据 failed_mask=16）。用户确认拨轮可回中但弹簧有阻塞，平时不用。
- 拨轮检查修正版已运行，板上只读确认 `0x59490103`。四个摇杆通道仍须在 [-10,10]，拨轮只检查未请求开火（ch4<=100），鼠标输入仍须为零。接收机通道有效性检查保持不变，正常开火判断与辨识共用阈值。
- 试验前 PROBE：flags=252，所有准入项通过；ch0..ch3=0、ch4=-660、鼠标全零、failed_mask=0、遥控年龄 13 ms。本机记录：`NoMachineTemp/yaw-ident-wheel-20260909-01`。
- 前三次分别授权的试验均在开始采样前退出，样本数为 0。第 1 次启动超时；第 2/3 次为操纵退出。已复现并修复布防时三档拨杆经过中位即中止的程序缺陷。
- `0x59490104` PROBE 通过：IDLE、id=0、flags=252、未占用控制权，两 yaw 软件指令为零；ch0..ch3=0、ch4=-660、鼠标全零、failed_mask=0、接收年龄 11 ms。本机记录：`NoMachineTemp/yaw-ident-switch-20260909-01`。
- 上一版授权试验已进入基线，但 1 ms 后因 CONFIG_CHANGED（reason=8）退出，仅 2 点，无有效激励。已复现并修复把 Pitch 动态驱动命令 Kp/Kd 当作固定配置的缺陷。**`0x59490105` 已运行且只读准入通过；尚无合格辨识数据，MPC 未接管。**
- 最新只读报告：IDLE、id=0、flags=252，未布防、未占用；ch0..ch3=0、ch4=-660、鼠标全零、failed_mask=0、接收年龄 9 ms。本机记录 `NoMachineTemp/yaw-ident-config-20260909-01`。尚待真实使能周期验证，不将静态通过等同于完整采集成功。
- 随后在 `0x59490105` 执行授权试验 ID 1，15 秒未收到下置，ARM_EXPIRED（reason=6），样本数为 0。记录 `NoMachineTemp/yaw-ident-config-big-20260909-01`；已下载、进程已退出、未 RELEASE。下一轮先准备操作者可见的 NUC 终端，不再依赖聊天转达启动倒计时。
- 用户已在 NUC 终端完成下一轮 ID 2：150 点，0.595 秒后 CONTROL_FAILED（reason=9）；125 点基线、24 点激励、1 点终止。末帧大 yaw 为 -46 rpm（-276°/s），超过辨识阈值 180°/s。不是完整辨识数据，暂停重试；本机记录 `NoMachineTemp/yaw-ident-terminal-big-20260909-134608`。

- 用户随后提供新参数截图：已将大 yaw 默认角度 Kp 从 50 固化为 10；角度 Ki/Kd=0.001/4，速度 Kp/Ki/Kd=0.6/0/0，输出上限 30，滤波时间常数 0.03 s。最新试验元数据确认这些 RAM 参数生效。
- Kp10 试验 ID 1：401 点、1.599 秒后 TRAVEL（reason=4），末帧大 yaw 36 rpm=216°/s 超过 180°/s；仍有增长振荡。数据本机副本 `NoMachineTemp/yaw-ident-big-kp10-20260909-141355`。保持上置、暂停重试，不拟合模型。

上一轮只修改大 yaw 默认角度 Kp；本轮额外加入上表的遥控选轴入口。小 yaw 惯导闭环、Pitch、机械限位和辨识保护不变。
辨识窗口内使用原 PID，独占两个轴的参考；不运行原大 yaw 跟随协调器。
另一轴参考偏移为零，不代表该电机驱动为零，因此始终记录两路驱动。

## 你现在先做什么

### 2026-09-09 工程配置修复

本地日志中的 9 个 L6218E 错误均为辨识符号未定义：当前工程文件缺少
`app_yaw_identification.c` 编译条目，也缺少 `-ffp-mode=full`。这两项已恢复，
完整重编译为 0 错误、69 条原有警告，AXF SHA256 与当时首次集成版一致；16 项辨识测试通过。
构建日志：`NoMachineTemp/ident-project-repair-20260909.log`。没有烧录或启动硬件。

请关闭并重新打开云台 `.uvprojx`，不要保存 Keil 中尚未重载的旧工程配置。
若旧工程仍驻留内存，保存时可能覆盖磁盘上的新条目。重新打开后确认 Application 组含有
`app_yaw_identification.c`，再执行 Rebuild All。末尾的 “Not enough information...”
只是链接失败后的汇总，排故应看前面的 L6218E 具体符号。

### 烧录与静态检查

截图参数已在最新试验中生效，协议标签仍为 `0x59490105`。此轮 ID 1 已终止，下载完整、控制权仍锁定，结束报告为上置和零 yaw 输出；本次分析没有 RELEASE、重试、改参或重新构建。
末帧大 yaw 相对锚点约 +1.274°、小关节相对启动点 0°、航向约 +0.033°、roll/pitch 变化约 +0.009°/+0.024°，未见机械行程/姿态阈值越界。36 rpm 的速度本身已足以触发 InsideTravel 的超速判断。
Kp10 的记录输出峰值约 3.733 软件单位（Kp50 上轮约 28.247），但尾段仍快速正反摆动。不同试验工况不能仅凭峰值推断稳定裕度；先做闭环稳定性诊断，保留保护阈值，不原样重复辨识。
下列为已执行的命令留档，**当前暂停重试**。后续改参应先在上置状态核对并释放真实编号，再单独安排验证，不能自动释放后立即重复本命令。

```bash
cd /home/nuc11--02/yaw-ident-config-4grH57N3
python3 -u yaw_identification_cli.py \
  --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  --action trial --axis big --amplitude-deg 1 \
  --confirm FIXED_CHASSIS_CLEAR_YAW \
  --output captures/big_kp10_$(date +%Y%m%d_%H%M%S)
```

看到 `ARM accepted` 后 15 秒内下置，其余操纵件不动；看到 `Trial: done` 或 `Trial: aborted` 后上置，等待 `Download complete`。异常随时上置。先只做这一轮大 yaw，不自动接着做小 yaw，不增加阈值；新参数是否稳定须由数据验证。

此前 Kp50 轮基线大 yaw 指令/速度均为零。激励初段参考记录最大仅 0.01°（0.01°量化），速度从 ±2、±8、±13、±22 扩大到 -46 rpm，输出峰值 28247 raw（约 28.247/30 软件单位）。小 yaw 关节仅变化 2 编码器计数，不能据此归咎于小 yaw 惯导。
最后速度已足以令 YawIdentApp_AllowYaw 拒绝输出，并通过汇总原因 CONTROL_FAILED 终止；此原因还包含 Pitch 离线/其他控制失败，现有数据未逐项保存退出掩码，不能排除并发原因。波形提示当前闭环不稳定，但尚未确定具体 PID、滤波、机械或反馈因素。
148 个样本间隔为 4 ms，最后保护终止间隔为 2 ms；CRC=0、原始下载和离线重放一致。数据不足且闭环响应异常，不拟合 MPC 模型。NUC 原件：`/home/nuc11--02/yaw-ident-config-4grH57N3/captures/big_20260909_134608`。
新版本从执行器保存的 Pitch MIT 设定读取 Kp/Kd，不再将禁用清零、使能恢复的驱动命令字段当作配置变化；真正的设定变化仍退出。没有修改增益、限位、超时或操纵保护，保留拨杆过中位修复。
不需要再重复 yaw 摇杆往返测试。拨轮负端读数不再单独阻止辨识，但超过 100 会拒绝 ARM 或中止进行中的试验。弹簧阻塞需要单独处理，软件未修复机械问题。

首轮已获授权并尝试：大 yaw 参考幅度 1°、总长 4 秒，编号 1。布防后 15 秒内未收到下置，返回 `ARM_EXPIRED`（reason=6），尚未进入基线，样本数为 0。不能用于辨识，MPC 未接管。
本机记录：`NoMachineTemp/yaw-ident-big-20260909-01`，NUC 记录：`/home/nuc11--02/yaw-ident-wheel-R6NQqStf/captures/big_01`。
之后两轮分别按用户确认，在上置时 RELEASE 旧编号后 ARM：ID 2/3 都返回 OPERATOR_STOP（reason=1），样本数 0。ID 2 用户报告曾为检查遥控移动摇杆；ID 3 退出前后回中项通过，与已复现的拨杆过中位缺陷相符，但 10 Hz 汇总状态不能重建每个接收机帧。
记录位于上述 NUC 目录的 `captures/big_02`、`captures/big_03`；本机副本为 `NoMachineTemp/big_02`、`NoMachineTemp/big_03`。ID 3 当时已下载、未 RELEASE，进程已退出；此次用户烧录重启后，新版只读报告已为 IDLE、id=0、未占用，不再释放旧编号。
`0x59490104` 新启动周期的 ID 1：基线样本 tick=172216，下一点 tick=172217 已终止，reason=8。记录在 `/home/nuc11--02/yaw-ident-switch-s7aSyRTP/captures/big_01`，本机 `NoMachineTemp/yaw-ident-switch-big-20260909-01`。已下载且进程退出，保持控制权锁定、上置、零 yaw 输出，没有自动 RELEASE。固件未记录变化字段，Pitch 命令生命周期是已复现且符合现场时序的解释，仍需新版本实车验证。
两 yaw 均闭环参与、小 yaw 保持惯导航向参考；布防等待和结束时 Pitch 输出关闭，需防落支撑。不要因 1° 激励不明显而移动摇杆检查遥控，这会中止试验。远程提示延迟会占用 15 秒窗口，应优先直接观察 NUC 终端提示，不绕过超时保护。

1. 右拨杆上置，摇杆回中，固定车体，确保两轴及线缆有空间。发射机构不要装载弹丸。
2. 只烧录云台工程 `AGVSentinel_v10090/AGVSentinel_gimbal/MDK-ARM/AGVSentinel_Gimbal.uvprojx`，目标 `AGVSentinel_Gimbal`。底盘无需重烧。
3. 执行 Reset，再 Run。按此前已验证的方法确认程序确实运行，不仅勾选下载选项。
4. 保持上置和静止，关闭串口调参网页及占用 USART1 的视觉程序。
5. 回复“配置检查版已运行、上置、静止”。先做只读版本与准入检查，**现在不要下置或自行开始激励**。

本版 ARM 前仍保持正常控制逻辑，不会仅因烧录或打开采集程序启动辨识。
试验结束或异常退出会关闭 yaw 和 Pitch 输出。**Pitch 可能自然下垂**，需要预先安排不会妨碍 yaw 的防落支撑；不要把手放在运动机构间。

## 状态与控制权

| 阶段 | 行为 |
| --- | --- |
| IDLE | 正常模式；普通大 yaw 编译期隔离仍有效 |
| ARMED | 新编号、上置且反馈有效才能布防；上置及经过中位均保持输出关闭，下置才启动，最长等待 15 秒 |
| BASELINE | 下置后锁定当前大关节角、小 yaw IMU 航向和 Pitch 目标，保持 0.5 秒 |
| EXCITE | 3 秒加窗 1/2/4 Hz 多正弦，仅改变所选参考 |
| SETTLE | 参考偏移回零，记录 0.5 秒 |
| DONE / ABORTED | 禁用输出，冻结实际样本，不补齐假数据，不自动重复或恢复旧目标 |
| RELEASE | 必须在终止状态、上置且软件输出为零时显式释放，随后才恢复正常控制权 |

布防至 RELEASE 期间，普通 PC 导航、视觉控制和调参命令不能改写试验参考或参数。
路由任务在检查控制权至提交目标期间暂停任务调度，硬件中断仍可运行，避免旧路由跨越接管后继续写参考。
遥控通道用于上置退出、下置启动和回中检查，不再积分摇杆目标。
底盘收到停止目标；摩擦轮和拨弹电机在发射输出层被清零，堵转自动处理也不能重新启动它们。
Pitch 采用进入试验时的保持目标；基线阶段允许电机使能和反馈恢复，进入激励后若 Pitch 电机离线则退出。

输出指令为零不是物理停稳证明；停止等待时间、制动距离和电机实际响应仍需现场确认。

## 单次试验保护

- 心跳绑定编号，300 ms 超时退出，过期心跳不能续命。
- 编号在本次 MCU 启动期间严格递增，重复 ARM 不会重启同一试验。
- 仅上置布防、下置运行；布防等待允许有效中位并保持零输出，不允许中位直接 ARM。进入运行后中位/上置均退出。
- 四个摇杆通道离开 [-10,10]、拨轮请求开火、鼠标 x/y 或左右按钮非零、遥控失联、无效档位或状态过期均退出，包括布防等待阶段。
- 遥控反馈最大年龄 50 ms，IMU 10 ms，两 yaw CAN 反馈各 20 ms。
- 准备和启动时小关节距外观中位不超过 10°，两轴关节速度不超过 6°/s。
- 试验距小 yaw 两侧软限位各保留 5°；两关节及惯导航向相对启动点的变化不超过 12°。
- 两轴关节速度超过 180°/s、Pitch/横滚观测相对启动变化超过 5°、主控调度间隔超过 10 ms 均退出。
- 参考幅度只接受 0.01..5°，程序默认 1°。这是工具默认值，不是本轮运动授权。
- 本轮要求大 yaw 输出上限不超过 30、小 yaw 不超过 6，沿用软件驱动单位。
- 冻结并比较 32 个关键 PID、滤波、使能和 Pitch 设置，变化即退出；所有旧串口调参命令被拒绝。
- 不要在试验中用 Keil 改变量、暂停或单步。并非所有调试变量都有独立变更检测。

上述仅是辨识准入及退出条件，没有给正常整车控制新增永久限速；也不是可停车区域的数学保证。

## 同步记录

250 Hz 为目标记录频率，4 秒完整时序期望 1001 点。记录在控制输出阶段取得各传感器的最新快照，
不是 IMU 与两个 CAN 电机在同一物理时刻采样。时间戳和反馈年龄用于检查偏差。

48 字节记录字段：

| 数据 | 存储与单位 |
| --- | --- |
| 主控时间 | uint32 毫秒，分析时按回绕处理 |
| IMU | 航向、横滚、俯仰为 float32 度；三轴 gyro 为 float32 rad/s |
| 两轴编码器 | uint16 原始 0..8191 counts；分析时展开跨圈 |
| 两轴关节速度 | int16 rpm，保留整数分辨率 |
| 两轴驱动 | int16，软件 Motor.output × 1000 后截断，匹配当前驱动整数转换 |
| 试验参考偏移 | int16，0.01°；与元数据中的锚点及轴编号组合 |
| 状态 | 反馈、拨杆、参考有效、大 yaw 闭环、软限位、饱和、采样间隔异常 |
| 反馈年龄、阶段 | 三路年龄各 uint8 毫秒，超过 255 饱和；阶段 uint8 |

驱动不是实测电流、力矩或 CAN 接收确认。参考偏移是调度器要求值，不是限幅后的最终参考；
出现限位/饱和的片段必须单独处理，不直接当作无约束线性样本。紧凑格式未保留电流、温度、
控制循环 dt 和 IMU 样本编号；需要这些诊断时使用原 500 Hz 采集，不把本格式等同于原记录。

192 字节元数据保存编号、构建标签（新版 `0x59490105`）、开始时间、实际数量、阶段、退出原因、
轴、工况编号、锚点、软限位和关键参数。布局以
`Inc/Modules/module_yaw_ident_wire.h` 与 `../yaw_identification_protocol.py` 为准。
从 `0x59490105` 起，pitch_mit_kp/kd 是执行器保存的 MIT 设定；旧构建这两个字段是 ARM 时的驱动命令值，禁用时可能为零，不能据此认为实际运行刚度为零。

共享缓存是旧 512×96 字节记录与新 1001×48 字节记录的 union，不能同时采集两种格式。
间隔异常会标记；提前退出保留实际数据。主机严格核对顺序、CRC、数量及试验元数据，
不会用插值或填零伪造完整试验。数据下载只在终止后且上置时执行，不与激励并行。

## 主机命令

新文件在本机 `gimbal-pid-tuner` 下，最新配置检查版工具已部署到 NUC 的 `/home/nuc11--02/yaw-ident-config-4grH57N3`。
拨杆过渡版及本轮记录保留在 `/home/nuc11--02/yaw-ident-switch-s7aSyRTP`。
拨轮检查版和三次中止记录保留在 `/home/nuc11--02/yaw-ident-wheel-R6NQqStf`。
之前逐项诊断工具及 ch4=-660 的原始记录保留在 `/home/nuc11--02/yaw-ident-remote-zoAnGkOF`，本机副本为 `NoMachineTemp/yaw-ident-remote-20260909-01`。
原版工具和前三次查询记录仍保留在 `/home/nuc11--02/yaw-ident-v1-RRPZqyUo`。
首次只读记录位于该目录的 `captures/probe_01`，本机副本为 `NoMachineTemp/yaw-ident-probe-20260909-01`。
当次 flags=248：两 yaw 软件指令为零，yaw/IMU 反馈新鲜、操纵回中、中位低速、配置有效；上置且遥控新鲜为 false。
后两次查询上置已通过，但回中未通过；第三次本机记录为 `NoMachineTemp/yaw-ident-probe-20260909-03`。不能凭这些快照进入试验。
部署时包括两个 `yaw_identification_*.py`、原 `yaw_capture_protocol.py`、
`pitch_tune_cli.py` 及 `mpc/identification_setup.json`。
以下命令在 NUC 上相应工具目录执行，串口设备用实际稳定的 by-id 路径替换。

只读查询不会 ARM，也不会修改 PID：

```bash
python3 yaw_identification_cli.py --port /dev/serial/by-id/实际设备 --output captures/probe_01
```

新版固件烧录后，在最新工具目录只读查询具体输入：

```bash
python3 yaw_identification_cli.py --port /dev/serial/by-id/实际设备 --action probe --remote-detail --output captures/remote_01
```

报告 `remote_diagnostics` 保存 ch0..ch4、mouse_x/y、mouse_left/right、接收年龄、快照时间和 `failed_inputs`。
例如失败项为 ch4，不代表已验证的 ch2 失效。这里的 ch 值是接收机解码、减中值后的通道值，不是电机角度。
不带 `--remote-detail` 的旧版 PROBE 和旧记录重放仍支持；新版工具不会对旧固件执行 trial。

确认固件、安全状态和传感器正常后，再单独批准当次试验。下列是接口示例，**现在不要执行**：

```bash
python3 yaw_identification_cli.py --port /dev/serial/by-id/实际设备 --action trial --axis big --amplitude-deg 1 --confirm FIXED_CHASSIS_CLEAR_YAW --output captures/big_01
```

工具输出 ARM accepted 后才按指示下置，摇杆保持回中。试验结束后上置，程序下载数据并保持控制权锁定。
不会自动执行第二轴或自动 RELEASE。另一组改用 `--axis small`，需重新确认现场并建立新目录。

取消、恢复下载和释放均绑定工具报告的真实编号，不要照抄示例编号：

```bash
python3 yaw_identification_cli.py --port /dev/serial/by-id/实际设备 --action cancel --trial-id 1 --output captures/cancel_01
python3 yaw_identification_cli.py --port /dev/serial/by-id/实际设备 --action download --trial-id 1 --output captures/download_01
python3 yaw_identification_cli.py --port /dev/serial/by-id/实际设备 --action release --trial-id 1 --confirm REMOTE_UP --output captures/release_01
```

取消是尽力发送；串口拔出或程序被强杀仍依赖主控心跳超时，上置是现场直接退出方式。
下载失败保留部分文件，可保持上置后重新下载；只有一个程序可以占用串口，不要同时开网页和此工具。
输出目录必须不存在。保存 `raw.bin`、`events.jsonl`、`samples.csv`、`report.json`、`setup.json`。
退出码 0 表示操作通过，1 表示通信/协议等操作失败，2 表示数据已保存但试验未完成或质量检查不通过；不要把退出码 2 的记录直接用于拟合。
离线重放使用 `--replay raw.bin --trial-id 实际编号 --output 新目录`，不打开串口。

协议使用 USART1、115200、原 16 字节 CRC8 帧：请求 0x36，状态 0x37，确认 0x38，
元数据 0x39，样本 0x3A。PROBE 与原监测心跳都不能启动运动。
PROBE 还返回 0x3B 构建信息（`<IIHH>`：构建标签、magic、版本、记录字节数）；主机在 ARM 前核对。
新版仅在未占用辨识控制权时，以 0x3C 返回 24 字节遥控快照（`<I7hBBHH>`）：
tick_ms、5 通道、mouse_x/y、左右按钮、age_ms、failed_mask。分为 3 个 `<HBB8s>` 碎片，
带同一递增 uint16 快照序号和诊断版本 2；发送完成前冻结快照，ARM 会取消待发诊断。
版本 2 的 failed_mask：bit0..3 对应摇杆通道超出 [-10,10]，bit4 对应 ch4>100 的开火请求，bit5/6 对应鼠标 x/y 非零，bit7/8 对应左右按钮非零。
旧诊断版本 1 的 bit0..4 仍按五通道超出 [-10,10] 解码，不能用新版语义重解释旧记录。样本、元数据和状态版本仍为 1。
准入与运行中检查使用同一个失败掩码。主机核对掩码与快照值一致，不混合不同序号或版本的碎片。
这个快照与 0x37 状态帧不是同一时刻，接收年龄也不是主机到达延迟，不用来替代运行时门控。
状态 flags 的 bit0..7 分别表示控制权占用、yaw 允许运行、上置且新鲜、两 yaw 指令为零、
yaw/IMU 反馈新鲜、操纵回中、yaw 中位且低速、配置有效。只读报告会给出对应布尔字段，主控 ARM 时还会再次检查。
元数据和样本碎片没有逐包试验编号，依靠独占会话、冻结缓存和严格序号组装；
MCU 重启或换试验后不得拼接旧下载，需重新建立主机解码会话。

## 已验证与待验证

- 最新诊断固件 AC6 6.22 完整构建：0 错误、69 条现有警告；Code 134944、RO 4156、RW 340、ZI 108740 字节。
- 实际应用 C 测试模拟 Pitch 禁用清零、使能恢复生命周期：修改前在基线复现误中止，修改后完整 1001 点通过；等待/运行中分别修改实际设定 Kp 或 Kd 仍被拦截。此模拟不等于真实 DM 驱动及 CAN 时序验证。
- 新增实际应用层上置→中位等待→下置启动测试，修改前复现失败，修改后通过；同时验证运行中位退出、无效接收机/档位、摇杆/拨轮操作、异常输出、CANCEL 及中位等待 15 秒超时仍退出。纯 C 核心测试也通过。
- 实际 `app_yaw_identification.c` 在模拟时钟、传感器、接收机及串口下测试通过，覆盖双轴全流程、
  时钟回绕、上置/操纵/心跳/数据/参数/行程异常、Pitch 离线、取消、冻结、下载与释放。
- C 记录夹具通过 Python 解码交叉核对；24 项辨识测试通过，包括截图八个默认参数及 Kp 启动赋值路径的静态回归。
  合并旧采集测试：48 项，47 通过、1 项平台跳过；此前 NUC 离线协议及假串口测试 17 项，15 通过、2 项本机 C 夹具跳过，主机协议此次未改。
  控制调用点和 Keil 工程配置静态回归不是 RTOS 时序仿真。
- 旧采集及相关回归与离线 MPC 测试通过；Windows 有一项原 Linux 专用测试跳过。
- 全局增加 `-ffp-mode=full`，使 NaN/Inf 检查不被有限数假设消除。编译已验证，
  **板上执行周期与实际安全链路仍需上置状态检查后再进入运动验证**。
- 用户自行烧录后，已通过 SSH 和 USART1 执行只读 PROBE 及四次分别授权的 ARM；前三轮无样本，第四轮仅 2 个基线/终止样本，均无有效激励。未由助手烧录或改在线 PID。只读检查与离线模拟都不等于实车动态安全验收。

最新构建日志：`NoMachineTemp/ident-big-kp10-20260909.log`。
本次修改前 AXF 备份：`NoMachineTemp/pre-big-kp10-20260909-140923.axf`。
新 AXF SHA256：`5FA00AF1DECFBD0BB54672C39C7B3444ED720C910A32023746663E47BDAEB33C`。

取得合格双轴数据后，再做驱动到运动的辨识、独立记录验证和 MPC 影子运行；不会直接把参考到反馈的闭环曲线当作裸对象模型。
