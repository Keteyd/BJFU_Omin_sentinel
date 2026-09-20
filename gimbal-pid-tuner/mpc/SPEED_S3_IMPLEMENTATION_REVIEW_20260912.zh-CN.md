# S3 独立验证实现复核

日期：2026-09-12；固件 build：`0x59490801`；trace：v5；状态：
`IMPLEMENTED_DEPLOYED_PRE_COLLECTION`；`hardware_takeover_allowed=false`。本文保留采集前实现状态；
后续一次性结果见 `SPEED_S3_59490801_VALIDATION_REVIEW_20260912.zh-CN.md`。

## 实现结果

固件新增独立操作码 `17`，只把冻结的 S3 设为速度参考 profile 的第三个 phase set。S3 的频率、幅度、
相位、3–31 s 激励窗口和 36 s 总时长逐项来自采集前冻结的
`speed_reference_s3_plan.json`。S1/S2 数值、操作码和历史采集策略未改变。

主机端新增 `yaw_s3_trace_cli.py` 和严格解码器：只开放 probe、S3、cancel、release；只接受
`0x59490801`、profile 5、reverse 2、9001 条名义配置和冻结波形。逐条参考值允许记录相对控制周期
0–1 ms 对齐误差，容差为 0.04°/s。报告固定写入冻结计划 SHA256，并保持
`hardware_takeover_allowed=false`。

S3 数据门改为显式时间覆盖：首尾、36 s 跨度、终态时钟、严格递增且不超过 10 ms 的源间隔，以及
3–31 s 每个 20 ms 网格点两侧均有距离不超过 10 ms 的源样本。原始记录数和相对 9001 条的短缺
只报告；`sample_count_not_9001` 与 `irregular_sample_interval` 只有在完整时间门通过时才可从有效质量
问题中消除，其他质量问题全部拒绝。

冻结验证器再次独立检查计划哈希、原 S1 模型哈希、profile、终态、CRC、流质量、反馈和 CAN 状态，
然后按不变的 20 ms 重采样、2 个输出滞后、4 个参考滞后和 ridge 0.001 模型计算 200 ms 门。大/小轴
相对保持预测改善下限仍为 20%/50%，RMSE 上限仍为 6/6°/s。S3 数据不会用于拟合、缩放或改门槛。

## 安全边界

速度参考 profile 延续已审核的去线缆策略：关闭大 yaw 相对起点行程和 IMU 航向相对起点行程中止；
保留小 yaw 绝对机械限位、相对起点 20° 行程、360/180°/s 实测速率、反馈新鲜度、遥控 UP/人工输入、
主机心跳、调度、有限值、控制链、Pitch 离线和 30/6 软件输出限幅。S3 完成后不会自动 RELEASE，
也不会启用任何 MPC 接管路径。

## 构建与验证证据

- 固件原生 C 测试 6 个目标全部通过，包含协议、波形、生产 supervisor、CAN 路径与串口 DMA；
- Keil Arm Compiler 6.22 全量构建：`0 Error(s), 68 Warning(s)`；程序尺寸 Code 148808、RO-data
  4652、RW-data 340、ZI-data 109260；
- S3 主机协议、冻结计划、波形、时间门、原生端到端解码与通用辨识协议共 35 项测试通过；
- S3 设计预演和冻结验证器 5 项测试通过，包括“缺 7 个原始调度槽但时间覆盖完整时通过”和 CAN/
  未授权质量标签拒绝；
- 原生 S3 夹具得到 9001 条记录、正确 build/reverse、完整时间门和空质量问题；用全零伪反馈运行冻结
  验证器时数据门通过、模型门拒绝，证明两个门独立生效；
- 发布 ZIP 解压后 24 项 SHA256 清单全部通过，打包内主机测试 7 项通过（无仓库原生夹具的一项跳过），
  全部 Python 文件通过字节码编译。

构建产物 SHA256：

- AXF：`EAF0D03B92407A21FE2FED5AC3F7CE9917EB2B0DB263AC385E0613E5FE4CD83E`；
- S3 计划：`BF41B63E5FA37AE20F065BB69603BF496E3E75CC5C4828D5A616F90267828B19`；
- 冻结 S1 模型报告：`1E81D3F94F3C81E3CE5A9A04481AABFCFE0270A8753BC4D3CE2C7A7725A2D53D`；
- `yaw-s3-trace-59490801.zip`：
  `F626D3B2E9E5FF65E7017334E3A4EA677D430FEDDCB13FBFD67CFAB71F1266B4`。

发布包位于 `NoMachineTemp/yaw-s3-trace-59490801.zip`，包含固件、构建日志、S3 专用采集工具、冻结
计划、冻结 S1 模型、冻结验证器、逐文件清单和现场步骤。该 ZIP 已上传并解压到目标机
`/home/nuc11--02/yaw-s3-trace-59490801`；远端 ZIP 哈希、24 项文件清单和主机自检均通过，冻结验证器
也能正常加载。部署过程未打开串口、未执行 probe 或运动。尚未烧录或采集 S3。
