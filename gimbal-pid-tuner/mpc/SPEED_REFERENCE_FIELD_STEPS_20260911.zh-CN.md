# `0x59490702` 速度参考 S1 采集步骤

> 状态更新（2026-09-12）：S1 已完成；S2 已执行并因 8994/9001、短缺 7 条超过冻结上限 5 条而正式
> 未通过数据门。两次控制权都已 RELEASE。禁止重复 S1/S2；本文命令全部转为历史记录。

本轮直接向现有速度内环注入“大 yaw 电机速度参考 + 小 yaw 惯性航向角速度参考”。固件包为
`yaw-speed-trace-59490702.zip`，目标目录为 `/home/nuc11--02/yaw-speed-trace-59490702`。

## 本版本的限制边界

按操作员对去线缆状态的审核，速度参考 profile 已关闭：

- 大 yaw 相对起点 25° 行程中止；
- IMU 航向角相对起点 20° 行程中止。

仍然有效：

- 小 yaw 编码器绝对机械限位：`-56.6777°..+29.0801°`；
- 小 yaw 相对采集起点 `20°` 行程中止；
- 大 yaw / 小 yaw 实测速率 `360°/s` / `180°/s` 中止；
- 反馈新鲜度、主机心跳、调度周期、有限数检查；
- 遥控器 UP/非零输入中止；
- 原有速度 PID 与大/小轴软件输出上限 `30/6`。

该例外只作用于 S1/S2 速度参考 profile。其他辨识 profile 的行程保护不变。S1 是开发数据；S2 必须等
S1 模型结构、参数和验收门冻结后再采集。

## 1. 烧录与只读检查

烧录：

```text
firmware/AGVSentinel_Gimbal_59490702.axf
```

重启后在目标机执行：

```bash
cd /home/nuc11--02/yaw-speed-trace-59490702

TRACE_PORT=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
TRACE_RUN="captures/speed_s1_$(date +%Y%m%d_%H%M%S)"

python3 -u yaw_speed_trace_cli.py \
  --port "$TRACE_PORT" --action probe \
  --output "$TRACE_RUN/probe_initial"
```

只有显示 `Firmware 0x59490702, 460800`、`phase=0`、`count=0`、`flags=252`、`error=null`
时继续。操作员检查底盘固定、无线状态无牵引线缆、小 yaw 离机械端点有足够余量、反馈正常、摇杆居中、
右拨杆 UP、Pitch 有可靠支撑。

## 2. 重新采集 S1

```bash
python3 -u yaw_speed_trace_cli.py \
  --port "$TRACE_PORT" \
  --action dual --phase-set S1 \
  --confirm SPEED_REFERENCE_FIXED_CHASSIS_CLEAR_YAW \
  --output "$TRACE_RUN/s1"
```

看到 `accepted` 后 15 秒内将右拨杆拨到 DOWN。运动期间保持 DOWN 和摇杆居中；需要人工停止时立即拨到
UP。完成或中止后拨回并保持 UP，然后执行：

```bash
python3 review_speed_reference_capture.py "$TRACE_RUN/s1" \
  --output "$TRACE_RUN/s1_review.json"

python3 -u yaw_speed_trace_cli.py \
  --port "$TRACE_PORT" --action release --confirm REMOTE_UP \
  --output "$TRACE_RUN/release_s1"

cat "$TRACE_RUN/s1_review.json"
printf '%s/%s\n' "$PWD" "$TRACE_RUN"
```

合格 S1 应显示 `structural_quality_passed=true` 和 `decision=FIT_AND_FREEZE_MODEL_BEFORE_S2`。到此停止，
不要采 S2；把最后打印的目录交回离线建模。

## 3. S2 保留命令

以下是已执行的一次性 S2 命令，仅保留为历史记录，不得再次执行：

```bash
cd /home/nuc11--02/yaw-speed-trace-59490702

TRACE_PORT=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
TRACE_RUN="captures/speed_s2_$(date +%Y%m%d_%H%M%S)"

python3 -u yaw_speed_trace_cli.py \
  --port "$TRACE_PORT" --action probe \
  --output "$TRACE_RUN/probe_initial"

python3 -u yaw_speed_trace_cli.py \
  --port "$TRACE_PORT" \
  --action dual --phase-set S2 \
  --confirm SPEED_REFERENCE_FIXED_CHASSIS_CLEAR_YAW \
  --output "$TRACE_RUN/s2"
```

probe 必须是 `0x59490702`、`phase=0`、`flags=252`、`error=null`。S2 只能评估冻结模型，不能再用于
改变模型结构、参数或验收门。
