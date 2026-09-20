# `0x59490801` S3 独立验证采集步骤

日期：2026-09-12；状态：`CONSUMED_DO_NOT_RUN_AGAIN`。本文命令只保留为审计记录；S3 已执行并
RELEASE，禁止再次执行第 3 节的运动命令。结果见
[`SPEED_S3_59490801_VALIDATION_REVIEW_20260912.zh-CN.md`](SPEED_S3_59490801_VALIDATION_REVIEW_20260912.zh-CN.md)。

本轮只验证已经冻结的 S1 闭环速度参考预测器。固件直接向现有大 yaw 速度 PID 和小 yaw
惯性航向角速度 PID 注入冻结的 S3 参考；角度 PID 被绕过，速度 PID、软件输出限幅和 CAN 路径保持不变。
采集结果不会自动授权 MPC 接管。

## 1. 仍然生效的运行保护

- 小 yaw 编码器绝对机械限位 `-56.6777°..+29.0801°`；
- 小 yaw 相对采集起点 `20°` 行程中止；
- 大/小 yaw 实测速率 `360°/s`、`180°/s` 中止；
- 电机反馈新鲜度、主机心跳、调度周期、有限数值和控制链有效性检查；
- 遥控器拨到 UP 或出现非零人工输入时中止；
- 大/小轴软件输出限幅 `30/6`；
- Pitch 离线检查。

去线缆配置下，大 yaw 相对起点行程和 IMU 航向角相对起点行程中止保持关闭。该例外只用于速度参考
profile；小 yaw 的机械保护未取消。需要人工停止时，立即将右拨杆拨到 UP。

## 2. 烧录与只读检查

手动烧录：

```text
firmware/AGVSentinel_Gimbal_59490801.axf
```

重启后在目标机执行：

```bash
cd /home/nuc11--02/yaw-s3-trace-59490801

TRACE_PORT=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
TRACE_RUN="captures/speed_s3_$(date +%Y%m%d_%H%M%S)"

python3 -u yaw_s3_trace_cli.py \
  --port "$TRACE_PORT" --action probe \
  --output "$TRACE_RUN/probe_initial"
```

只有输出同时满足以下条件才继续：

- `Firmware 0x59490801, 460800`；
- `phase=0`、`count=0`、`error=null`；
- 反馈有效；
- 操作员确认底盘固定、去线缆状态、摇杆居中、右拨杆 UP、小 yaw 离机械端点有足够余量、Pitch 有可靠支撑。

## 3. 唯一一次 S3 采集

```bash
python3 -u yaw_s3_trace_cli.py \
  --port "$TRACE_PORT" \
  --action dual --phase-set S3 \
  --confirm S3_REFERENCE_FIXED_CHASSIS_CLEAR_YAW \
  --output "$TRACE_RUN/s3"
```

看到 `accepted` 后 15 秒内把右拨杆拨到 DOWN。运动期间保持 DOWN 和摇杆居中。完成或中止后拨回并
保持 UP。不要再次运行 S3 命令。

采集成功时应看到 `phase=5`、`reason=0`、`download_complete=true`、`quality_issues=[]` 和
`capture_timing.full_duration_timing_accepted=true`。实际记录数允许少于 9001；记录数只作为诊断，
只要所有冻结的时间覆盖、数据质量和 CAN 条件通过即可。

打印并保存目录：

```bash
printf '%s/%s\n' "$PWD" "$TRACE_RUN"
```

## 4. 在目标机运行冻结验证器

以下命令只读取采集文件，不修改模型：

```bash
python3 -u mpc/validate_speed_reference_s3_offline.py \
  --model mpc/frozen_s1_model_report.json \
  --plan mpc/speed_reference_s3_plan.json \
  --capture "$TRACE_RUN/s3" \
  --output "$TRACE_RUN/s3_frozen_validation.json"
```

退出码 `0` 且 `status=FROZEN_SPEED_REFERENCE_MODEL_PASSED_S3` 才表示冻结模型通过 S3。退出码 `2`
表示数据门或模型门拒绝，仍应保留并交回整个目录，不得重采 S3，也不得用 S3 修改自身的模型或门槛。

验证完成并确认右拨杆保持 UP 后，可释放采集所有权：

```bash
python3 -u yaw_s3_trace_cli.py \
  --port "$TRACE_PORT" --action release --confirm REMOTE_UP \
  --output "$TRACE_RUN/release_s3"
```

最后再次打印 `$TRACE_RUN` 的绝对路径并交回。无论验证是否通过，`hardware_takeover_allowed` 均保持
`false`；S3 只验证保留速度 PID 的闭环预测器。
