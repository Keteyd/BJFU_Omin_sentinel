# 双参考闭环辨识实现复核

> 本文记录截至 `0x59490404` 的实现历史。取消 ARM 人工准备门禁和 BENCH 前置资格后的当前实现见
> [`OPERATOR_QUALIFIED_ARM_REVIEW_20260910.zh-CN.md`](OPERATOR_QUALIFIED_ARM_REVIEW_20260910.zh-CN.md)。

日期：2026-09-10。结论：双参考设计已经实现并完成第一轮现场验证。`0x59490401` 的 BENCH A
通过，但 dual A 在第 3017 点以合并原因 `control_failed` 安全终止。复核排除了遥控主动停止、
反馈失效、行程越界、CAN 发送失败和参考超预算。`0x59490402` 将原因细分后，BENCH A 暴露了
毫秒时钟严格等于 4 ms 的资格误判；真实间隔全部为 3.198～4.736 ms。`0x59490403` 修正后 BENCH A
通过，但 dual A 在第 2008 点以 `yaw_control_inactive` 终止；实测间隔、CAN、反馈、行程和遥控均正常。
`0x59490404` 已增加大轴控制模块内部停止分支诊断并完成烧录。其第一次 dual A 在 2 s 边界被旧主机
误判后取消；固件没有报告保护故障。修复后的 hostfix1 已在本地通过回放，尚未上传。对象模型仍是 `not_identified`，
`hardware_takeover_allowed` 保持 `false`。

## 已实现内容

- 固件构建号从 `0x59490301` 逐步升为 `0x59490404`，trace version 保持为 4；
- 新增固定的双参考相位组 A/B 请求，协议操作码分别为 10/11，固定 `axis=3`、幅度参数为 0；
- 20 秒调度仍为 5001 个 4 ms 采样点：2 秒基线、14 秒 `sin²` 加窗多频激励、4 秒回稳；
- 大轴与小轴参考预算分别为 3° 和 2°，波形参数与离线设计逐点一致；
- 152 字节记录和 163 字节帧保持不变。v4 将基础记录中的两个旧转速字段明确改为
  `big_reference_offset_cdeg` 与 `small_heading_reference_offset_cdeg`；实际 CAN 反馈转速仍记录在
  `big_feedback_rpm` 与 `small_feedback_rpm`；
- 主机解码器校验两路参考之和、档位元数据、构建号和版本；v3/v4 解码器互相拒绝对方固件；
- BENCH 资格、右拨杆触发/急停、心跳、反馈有效性、行程和速度保护、终止后控制权锁定均保留。
- `0x59490402` 保持波形和保护阈值不变，将旧的 `control_failed` 细分为
  `yaw_control_inactive`、`output_nonfinite` 和 `pitch_offline`；主机报告新增 `reason_name`。
- `0x59490403` 使用 CAN trace 微秒时钟执行既定 10 ms 采样门槛，允许正常的 3/5 ms 毫秒量化，
  超过 10 ms、零间隔、CAN 覆盖或反馈缺失仍会锁存 BENCH 失败。
- `0x59490404` 在 `reason=12` 时报告大轴内部停止分支；主机允许元数据后首个 CAN trace 样本在
  0～10 ms 内到达，后续样本仍按毫秒 tick 和独立微秒 trace 双重校验。
- 异常终止帧允许保留最后计划参考用于诊断，但仍严格要求软件命令和 CAN 当前命令均为零。

## 验证证据

| 项目 | 结果 |
| --- | --- |
| Keil `AGVSentinel_Gimbal` 完整构建 | 0 errors，68 warnings |
| 固件原生测试 | 调度/线协议、监督器、CAN trace、HAL 路径、PC DMA 全部通过 |
| 主机相关协议与集成测试 | 62/62 通过 |
| MPC Python 测试 | 86/86 通过 |
| C 产出的 v4 夹具回放 | 5001 条、CRC 0、`quality_issues=[]` |
| A 波形 C/Python 逐点对照 | 最大量化误差不超过 1 厘度 |
| 新固件 v4 夹具在 NUC 回放 | 5001 条、CRC 0、`quality_issues=[]` |
| 历史现场中止 raw 回放 | 3017 条完整解码、CRC 0、`reason_name=control_failed` |
| `0x59490402` 实车 BENCH 时序 | 5000 个后续间隔为 3198～4736 µs，均值 4000.0148 µs，无超过 10 ms |
| `0x59490403` 实车 dual A | 2008 条；2605～4882 µs；CAN/反馈正常；`reason=12` |
| `0x59490404` 实车 dual A 前缀 | 第 501 帧为 tick 2000 ms、trace 1999576 µs、phase 2；hostfix1 回放通过 |
| AXF 反汇编 | 元数据和固件信息路径均装入 `0x59490404` |

Keil 的 68 条 warning 没有阻止链接；本次新增代码没有构建 error。真实串口、CAN 总线和机械响应
只能在烧录后的 BENCH/采集阶段验证，不能由离线测试替代。

## 发布产物

| 产物 | SHA256 |
| --- | --- |
| `NoMachineTemp/AGVSentinel_Gimbal_59490401.axf` | `DAA8C1A20C9718A3E6227ED43A2925AA302D2DE63AC47A9E8349BDDA6E7ED957` |
| `NoMachineTemp/yaw-dual-trace-59490401.zip` | `363A7C9040E354B165D0232ECB792DD1442FFDB3FAB472D1FC974C78A50221E6` |
| `NoMachineTemp/yaw-dual-trace-59490401-hostfix1.zip` | `AA18A764D9AECC6CCA6D98940E92FBDE298A9649CF62AD64659A71A355110B28` |
| `NoMachineTemp/AGVSentinel_Gimbal_59490402.axf` | `6D5AEE0CB1242BBA91455709BEE3AE647CED8011D75243EAA22DD867215FC805` |
| `NoMachineTemp/yaw-dual-trace-59490402.zip` | `D0808D69375444CC5C0563492801DF0FE6482F6421A5D41F2B0D389376C85261` |
| `NoMachineTemp/AGVSentinel_Gimbal_59490403.axf` | `A999F172306C9020DE8A3C3A38164AB71788EE5CF8954E363499261E776F78A7` |
| `NoMachineTemp/yaw-dual-trace-59490403.zip` | `633383305B7C62416818C0160BA3E1D91E6CC4547C78ED0719275F583177F174` |
| `NoMachineTemp/AGVSentinel_Gimbal_59490404.axf` | `172F973699B2BF6779962C59C179E5D68C2D308EBE29DE7C97EB708974A81D0C` |
| `NoMachineTemp/yaw-dual-trace-59490404.zip` | `629BF9F817B7EC6DEF5AA9BDE3AC1820E474E08DA82684CFC332371ED5F3886B` |
| `NoMachineTemp/yaw-dual-trace-59490404-hostfix1.zip` | `5546F01D048E9DC8C2BC1B8B77A608C416E6C72A5C183CF46936372334FFE945` |

第一次 BENCH 暴露串口分块误判后，主机同步逻辑已修复；问题和证据见
[`DUAL_BENCH_A_HOST_SYNC_REVIEW_20260910.zh-CN.md`](DUAL_BENCH_A_HOST_SYNC_REVIEW_20260910.zh-CN.md)。
dual A 首轮中止复核见
[`DUAL_A_ABORT_REVIEW_20260910.zh-CN.md`](DUAL_A_ABORT_REVIEW_20260910.zh-CN.md)，`0x59490403`
内部失活复核见
[`DUAL_A_INACTIVE_REVIEW_20260910.zh-CN.md`](DUAL_A_INACTIVE_REVIEW_20260910.zh-CN.md)。最终配套包已部署到
`nuc11--02@192.168.1.113:/home/nuc11--02/yaw-dual-trace-59490404`。远端 ZIP SHA256 与发布记录一致，
目录内 16 个受检文件全部通过哈希校验；Python 入口确认 `0x59490404`、trace v4、69 个唯一字段，帮助页
也正常。部署和自检没有访问串口。`0x59490402` BENCH 时序复核见
[`DUAL_BENCH_TIMING_REVIEW_20260910.zh-CN.md`](DUAL_BENCH_TIMING_REVIEW_20260910.zh-CN.md)。

`0x59490404` 第一次 dual A 的主机相位边界误判见
[`DUAL_PHASE_BOUNDARY_REVIEW_20260910.zh-CN.md`](DUAL_PHASE_BOUNDARY_REVIEW_20260910.zh-CN.md)。本地
hostfix1 已通过 62 项测试和现场原始帧回放；自动审批要求用户对这个修改后的新 ZIP 单独明确授权，当前尚未
上传，远端 `yaw-dual-trace-59490404-hostfix1` 目录尚不可用。

## 下一步

保持右拨杆 UP。取得对 hostfix1 新载荷的明确上传授权并部署后，先 probe，按状态释放旧 trial，
再重新执行 BENCH A。固件无需重烧。随后按
[`DUAL_REFERENCE_FIELD_STEPS_20260910.zh-CN.md`](DUAL_REFERENCE_FIELD_STEPS_20260910.zh-CN.md)
执行 dual A。无论 dual A 成功或终止，本轮都保持 UP、保留终止
控制权并回传报告，不继续 BENCH B 或 dual B。先用新的内部停止码决定修复或放行，之后再继续电流偏置、
时序对齐、闭环工具变量辨识和 A/B 独立验证。
