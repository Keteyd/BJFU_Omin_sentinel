# 速度参考 MPC 接口与 S1/S2 采集实现复核

日期：2026-09-11；历史固件：`0x59490701`；trace：v5。该版本 S1 已实车运行并因航向相对起点超过旧 20° 阈值而中止；现行实现与步骤已迁移到 `0x59490702`，见[行程策略复核](SPEED_REFERENCE_TRAVEL_POLICY_REVIEW_20260911.zh-CN.md)。`hardware_takeover_allowed=false`。

MPC 的候选输出已经确定为两个速度参考：大轴电机速度参考与小轴惯性航向角速度参考。辨识 profile 5 只绕过两个角度 PID，参考量分别除以6转换为 rpm 后直接送入现有速度 PID。两轴速度反馈、速度 PID、积分状态、执行量限制、CAN 输出以及机械/通信/遥控保护均保持生产路径。这样得到的对象包含内速度环，适合20 ms级 MPC；也避免把尚未标定为力矩的反馈电流误当作控制输入。

S1/S2 均为36秒：3秒零参考基线、28秒平滑多正弦速度、5秒零参考回零观察。波形是窗函数位置原函数的解析导数，起止速度为零且解析净位移为零。S1/S2 采用不相交谐波，峰值均为大轴30°/s、小轴60°/s。4 ms 数值复核得到 S1/S2 净位移误差均小于 `3e-9°`，速度峰值不超过预算；详细系数与哈希见 `speed_reference_excitation_plan.json`，SHA256 为 `6929CDE2253DEFA404E023AE19DE8E3BF07D828F199F216CE752A039718BB2AF`。

152字节 trace 帧未扩容。原双参考字段在 profile 5 中明确改为 `centidegrees_per_second`；仍同步记录实际 CAN 反馈转速、IMU z 轴角速度、速度 PID 最终软件执行量、CAN区间命令历史、反馈电流和微秒时间戳。严格主机拒绝旧构建、错误 profile/轴号、错误波形、错误单位预算或冻结计划不一致。

保留的运行终止条件包括：UP/遥控输入中止、反馈故障、心跳丢失、调度超时、大轴/小关节/航向相对行程、实测速率、配置改变、流停滞、缓冲区满、控制链失活、非有限输出和 Pitch 离线。大/小轴最终执行量仍受30/6软件单位限制。未增加 MPC 接管或自动 RELEASE。

验证结果：

- `run_yaw_slow_tests.ps1` 的6个本地 C 测试目标全部通过，包括新速度波形、协议、主管、CAN trace和串口批传；
- Keil Arm Compiler 6.22 完整构建为0错误、0警告；程序尺寸 Code 148648、RO-data 4580、RW-data 340、ZI-data 109260；最终 AXF SHA256 为 `043016368B9A97E60D862B66FDAAD1D6F19007FA34C8CAEDEBFA5FE28AD13C15`；
- Python协议、冻结计划、CLI参数、波形边界和原生端到端流解码5项测试通过；
- 离线预览在 `NoMachineTemp/speed-reference-preview-59490701/report.json`。

最终发布包 `NoMachineTemp/yaw-speed-trace-59490701.zip` 包含运行时串口依赖 `pitch_tune_cli.py`，
SHA256 为 `A0AB6F87B6C84272A039351BB5A09BED21496FD546582085021ED3F11B4E00D8`。包已上传到
目标机并解压到 `/home/nuc11--02/yaw-speed-trace-59490701`，18个发布文件逐项 SHA256 校验通过。
只读 probe 已识别 build `0x59490701`；当时状态 `flags=236`，反馈有效位未置位，因此尚不允许启动 S1。

现场顺序固定为：烧录后 probe，只采 S1，审核并 RELEASE，离线选择和冻结模型；之后才允许用同一固件采 S2 做独立验证。现场命令见 `SPEED_REFERENCE_FIELD_STEPS_20260911.zh-CN.md`。
