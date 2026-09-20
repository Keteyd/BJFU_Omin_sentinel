# `0x59490503` E 组独立验证采集步骤

日期：2026-09-11。本文只用于新的 phase E。C 已冻结，D 只保留为条件工程证据；不要重采 C 或 D。
E 的实车结果只使用一次，结束或中止后保持右拨杆 UP，并先回传结果，不自行重复 E。

> 2026-09-11：E 已完成、通过冻结 C→E 验证并已 RELEASE。不要再次执行本文第三节的运动命令。
> 结果见 [`E_REFERENCE_VALIDATION_REVIEW_20260911.zh-CN.md`](E_REFERENCE_VALIDATION_REVIEW_20260911.zh-CN.md)。

## 1. 固件与主机包

烧录文件：

```text
NoMachineTemp/AGVSentinel_Gimbal_59490503.axf
```

AXF SHA256：
`1D6A03295E45BFC04414CE3DD28C5E99CE04DED39ABE89664CADC5B751851F77`。

将 `NoMachineTemp/yaw-e-trace-59490503.zip` 上传并解压为：

```text
/home/nuc11--02/yaw-e-trace-59490503
```

E 主机工具只开放 `probe`、phase E、`cancel` 和 `release`。它拒绝旧固件、C/D/A/B、BENCH 和单轴
trial。固件重启会清除旧 trial 所有权。

## 2. 校验和只读 probe

烧录后退出 Keil 调试，保持右拨杆 UP：

```bash
cd /home/nuc11--02/yaw-e-trace-59490503
sha256sum -c SHA256SUMS

TRACE_PORT=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
TRACE_RUN="captures/e_validation_$(date +%Y%m%d_%H%M%S)"

python3 -u yaw_cde_trace_cli.py \
  --port "$TRACE_PORT" --action probe \
  --output "$TRACE_RUN/probe_initial"
```

继续条件是显示 `Firmware 0x59490503, 460800`、phase 0、reason 0、`error=null`、未占用且反馈有效。
常见完整状态为 `flags=252`。若 `flags & 16 != 16`，先恢复反馈，再使用尚不存在的新 `TRACE_RUN` probe。

## 3. 一次性采集 E

由操作员确认底盘固定、两级 yaw 与线缆行程无阻挡、Pitch 支撑可靠、遥控在线、摇杆中立，并能直接用
右拨杆 UP 或独立断电停止运动。执行：

```bash
python3 -u yaw_cde_trace_cli.py \
  --port "$TRACE_PORT" \
  --action dual --phase-set E \
  --confirm E_REFERENCE_FIXED_CHASSIS_CLEAR_YAW \
  --output "$TRACE_RUN/dual_e"
```

显示 `DUAL E accepted` 后，在 15 秒内把右拨杆从 UP 拨到 DOWN。运动期间保持 DOWN 和摇杆中立；异常时
立即拨回 UP。结束或中止后保持 UP，不发送 RELEASE、不再次执行 E，并回传完整终端输出和目录：

```bash
printf '%s/%s\n' "$PWD" "$TRACE_RUN"
```

## 4. 采集质量门

主机包在采集前已经冻结以下规则：

- 固件 `0x59490503`、trace v5、profile 4、phase value 2；
- 36.000 s 完整时间轴，终态 phase 5/reason 0；
- 记录数允许 8956～9001，即最多短缺 45 条（0.5%），短缺时按微秒时间戳与指令积分重采样；
- 时间戳严格单调，任一采样间隔不超过 10 ms；
- CRC、流内丢弃、CAN failed/errors/aborted 均为 0；
- 固件采集质量锁存不得置位，参考回读误差不超过 0.011°，双参考秩为 2 且相关系数绝对值不超过 0.1。

主机报告出现 `quality_issues=[]`、`capture_timing.full_duration_timing_accepted=true` 才进入冻结 C 模型验证。
E 即使通过，也只说明现有 PID 闭环参考映射通过 C→E 门限，仍不会允许 MPC 接管硬件。

## 5. E 信号和仍启用的保护

E 保持已经审核的 3.00°/2.00°分量预算。离线预览实际峰值、速度和加速度为：

| 轴 | 实际峰值 | 最大参考速度 | 最大参考加速度 |
|---|---:|---:|---:|
| 大 yaw | 2.7924° | 8.7305°/s | 88.9018°/s² |
| 小 yaw | 1.9265° | 9.3814°/s | 82.4457°/s² |

E 双轴相关系数为 0.0135；与 C 同轴相关系数为 0.2603/-0.2245，与 D 为 0.2243/0.2067。E 没有参考
加速度运行时中止，200°/s²保留为离线审阅值。50 ms 反馈新鲜度、遥控 UP 急停、300 ms 主机心跳、
25°/20°/20°相对行程、360°/s 与 180°/s 实测关节速度、采集流、控制链及有限输出保护保持启用。

## 6. 收尾

我复核并取回数据后，如需恢复普通控制，会在右拨杆保持 UP 时执行：

```bash
python3 -u yaw_cde_trace_cli.py \
  --port "$TRACE_PORT" --action release --confirm REMOTE_UP \
  --output "$TRACE_RUN/release_e"

python3 -u yaw_cde_trace_cli.py \
  --port "$TRACE_PORT" --action probe \
  --output "$TRACE_RUN/probe_after_release"
```

最终 probe 应为 phase 0、reason 0、未占用和 `error=null`。
