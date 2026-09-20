# CAN 扩展版：小 yaw 正向采集操作步骤

> **已于2026-09-10完成，不要重复执行。** trial id=4 的5001条原始数据已经取回并通过
> 独立结构复核。结果见[小 yaw CAN扩展版正向采集复核](SMALL_CAN_TRACE_REVIEW_20260910.zh-CN.md)
> 和[CAN扩展数据辨识复核](CAN_TRACE_MODEL_REVIEW_20260910.zh-CN.md)。以下命令仅保留为现场记录。

适用固件 `0x59490301`，主机目录 `/home/nuc11--02/yaw-can-trace-59490301-hostfix1`。
本轮只做一轮**小 yaw ±10°正向、20秒**采集；大 yaw数据已经有效，不要重采。
当前大 yaw trial id=2 仍持有辨识控制权。全程先保持右拨杆UP。

## 1. 建立本轮目录并查询

在 NUC 终端逐段执行：

```bash
cd /home/nuc11--02/yaw-can-trace-59490301-hostfix1
TRACE_PORT=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
TRACE_RUN="captures/small_can_$(date +%Y%m%d_%H%M%S)"
printf '本轮目录：%s/%s\n' "$PWD" "$TRACE_RUN"

python3 -u yaw_can_trace_cli.py \
  --port "$TRACE_PORT" --action probe \
  --output "$TRACE_RUN/probe_before"
```

应显示 `Firmware 0x59490301` 和 `error: null`。若 phase不是5/6且仍持有控制权，
保持UP并停止，不执行后续命令。

## 2. 释放大 yaw trial id=2

```bash
python3 -u yaw_can_trace_cli.py \
  --port "$TRACE_PORT" --action release --confirm REMOTE_UP \
  --output "$TRACE_RUN/release_big"
```

`error: null` 或 `Already unowned` 才可继续。保持UP。

## 3. 重新做零输出 BENCH

不复用可能过期的资格，重新做一次20秒 BENCH：

```bash
python3 -u yaw_can_trace_cli.py \
  --port "$TRACE_PORT" \
  --action bench --confirm BENCH_REMOTE_UP \
  --output "$TRACE_RUN/bench"
```

全程UP、不动摇杆。必须看到 `BENCH PASSED.`，并满足5001条、CRC错误0、
`quality_issues: []`、`error: null`。否则保持UP，停在这里并保留目录。

BENCH不会自动释放，继续保持UP：

```bash
python3 -u yaw_can_trace_cli.py \
  --port "$TRACE_PORT" --action release --confirm REMOTE_UP \
  --output "$TRACE_RUN/release_bench"
```

应为 `error: null`。

## 4. 现场确认

- 底盘固定、无相机负载、Pitch姿态和PID与大 yaw轮一致。
- Pitch已支撑，且支撑不干涉两轴运动。
- 小 yaw 位于校准中心附近，机械限位两侧都有足够余量；大 yaw与线缆活动范围清空。
- 右拨杆UP、摇杆回中，没有其他程序占用串口。

任何一项不满足都不执行下一步，不通过手推或放宽软件限位绕过门禁。

## 5. 小 yaw ±10°正向采集

```bash
python3 -u yaw_can_trace_cli.py \
  --port "$TRACE_PORT" \
  --action trial --axis small --amplitude-deg 10 \
  --confirm FIXED_CHASSIS_CLEAR_YAW \
  --output "$TRACE_RUN/small_forward"
```

只有看到 `ARM accepted` 后，才在15秒内将右拨杆拨到DOWN。
随后约20秒不碰摇杆、不手推云台。出现异常立即拨回UP。

目标顺序为：0–2秒基线，2–4秒到+10°，4–7秒保持，7–11秒到−10°，
11–14秒保持，14–16秒回起点，16–20秒回稳。
该角度是相对起始航向目标；大 yaw可能参与补偿。

结束后立即回UP并保持。正常结果应为 phase=5/reason=0、5001条、CRC错误0、`error: null`。
正式 trial 的 `bench_passed: false` 与 `hardware_takeover_allowed: false` 都是预期字段。
若有时间质量告警，保留数据并交回分析，不自行重采。

## 6. 停止并回传路径

不要RELEASE，不要继续反向或其他幅度。打印路径：

```bash
printf '%s/%s\n' "$PWD" "$TRACE_RUN"
```

把路径和终端报告发给我。我会通过SSH一次性取回整个目录。
更完整的通用异常处理见[已完成的大 yaw步骤](CAN_TRACE_FIELD_STEPS_20260910.zh-CN.md#出错时停在哪里)。

## 7. 分析完成后释放控制权

原始目录现已安全取回。确认右拨杆处于UP、Pitch已有可靠支撑后，可执行：

```bash
cd /home/nuc11--02/yaw-can-trace-59490301-hostfix1
TRACE_PORT=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
TRACE_RUN=captures/small_can_20260910_144513

python3 -u yaw_can_trace_cli.py \
  --port "$TRACE_PORT" --action release --confirm REMOTE_UP \
  --output "$TRACE_RUN/release_after_review"
```

看到 `error: null` 或 `Already unowned` 即完成。若释放被拒绝，保持UP并保留终端输出，
不要重新执行trial。释放只归还辨识控制权，不代表允许MPC接管。
