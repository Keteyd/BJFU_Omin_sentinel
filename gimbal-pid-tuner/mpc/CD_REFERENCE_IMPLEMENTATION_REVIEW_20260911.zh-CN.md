# C/D 双参考固件与主机实现复核

日期：2026-09-11。目标固件为 `0x59490501`，trace version 5。该版本把已审核的
3.00°/2.00°低频增强 C/D 波形固化进 MCU，并将采集扩展到 36 秒、9001 条记录。

## 加速度审核结论

源码中没有“参考加速度达到 100°/s²就中止”的运行时保护。原来的 100°/s²只存在于
`dual_reference_excitation_plan_cd.json` 的离线波形预览门槛，不参与 MCU 调度或停止判断。
按操作员审核结论，该离线门槛已提高到 200°/s²：

- C 最大参考加速度为大/小轴 67.23/82.60°/s²；
- D 最大参考加速度为大/小轴 91.70/95.24°/s²；
- D 小轴使用新离线门槛的 47.62%，不会因参考加速度审查中止；
- 波形幅度、频率、相位和换向探针均未改变。

固件继续保留实际反馈与控制链保护：遥控 UP/停止、300 ms 主机心跳、反馈有效性、
大/小/航向相对行程 25°/20°/20°、实测大/小轴速率 360°/s和180°/s、采集可用性、
输出有限值以及控制权一致性。操作员的独立急停负责现场机械风险，不替代这些固件停止路径。

## 实现内容

- `module_yaw_identification.h` 新增 profile 4、C/D 固定调度、36 秒时序和9001点记录数；
- C/D 换向序列、谐波、相位和分量幅度逐项写入固件；
- `module_yaw_ident_wire.h` 将构建号更新为 `0x59490501`、trace version 更新为5，并把累计
  ACK 上限扩展到9001；
- `app_yaw_identification.c` 新增 C/D 请求、profile 元数据和动态记录长度；
- `yaw_cd_trace_protocol.py` 严格拒绝旧构建、旧 trace、错误 profile、错误相位、错误时长和
  错误配置，并在报告中记录冻结计划 SHA256；
- `yaw_cd_trace_cli.py` 只接受 C/D，并要求专用口令
  `CD_REFERENCE_FIXED_CHASSIS_CLEAR_YAW`；不会自动 RELEASE 或自动开始下一组，并拒绝 profile 不兼容的
  BENCH 和单轴 trial。

## 验证证据

| 检查 | 结果 |
| --- | --- |
| MCU 原生调度、监督器、CAN HAL、环形缓冲和 wire 测试 | 6 个程序全部通过 |
| 主机协议、旧版兼容与集成测试 | 56 项通过，1 项旧 v4 实物夹具缺失而跳过 |
| MPC 离线回归 | 92 项通过 |
| 生产 C 夹具逐点对照冻结 JSON | 9001 点，两轴最大误差不超过 0.01° |
| trace-v5 原生夹具回放 | 9001 条、CRC 0、`quality_issues=[]` |
| Keil AC6 全量重建 | 0 error、68 warning |

Keil 生成的 AXF 为
`AGVSentinel_v10090/AGVSentinel_gimbal/MDK-ARM/Objects_Gimbal/AGVSentinel_Gimbal.axf`，
大小 847,936 字节，SHA256：
`2750E13572E052D2E45C8B3AA3B0A13DDC987DBF2D36CFAE661FBC353C10D50F`。
反汇编已确认构建号 `0x59490501`、36,000 ms、9,001 点以及 `YawIdent_CDWave` 被链接进最终映像。

冻结计划 SHA256 为
`05DA0E5EAD289CBB3A16B4D243F13F449B523CE68B731799A768253A34DE09EF`。
离线 `review.json` SHA256 为
`712BD868617B46B62AD8143033079BB4D85B3B5E8066330306E711778DD64DA0`。
主机发布包 `NoMachineTemp/yaw-cd-trace-59490501.zip` SHA256 为
`F771922FE21E318DDB72D4BB57DF7C7BC243C403FE2A63E7D8EC667E1D1E1FFB`；从最终 ZIP 解压后的逐文件
SHA256 校验、帮助页导入和9001点原生夹具回放均通过。
目标机 `/home/nuc11--02/yaw-cd-trace-59490501` 已完成 ZIP 外部哈希、包内15个文件及 Python
版本自检；实车 probe 确认 `Firmware 0x59490501, 460800`、idle/unowned、reason 0、`error=null`。

该复核只放行 C/D 数据采集，不放行 MPC 硬件接管。C 只用于按冻结结构拟合；D 在任何系数、
延迟、尺度或阈值调整之前只执行一次独立验证。
