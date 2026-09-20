# `0x59490601` 去线缆幅度阶梯采集步骤

适用固件：`0x59490601`；主机目录：`/home/nuc11--02/yaw-cablefree-trace-59490601-hostfix1`。本步骤用于机械线缆负载解除后的闭环对照和较大幅度开发采集，不是新的 MPC 接管验证。R1/F4/F5 都是开发数据，`hardware_takeover_allowed=false`。

## 波形与执行顺序

| 档位 | 相对原 E | 大轴参考峰值 | 航向参考峰值 | 平面推算小关节峰值 | 用途 |
|---|---:|---:|---:|---:|---|
| R1 | 1× | 2.79° | 1.93° | 3.15° | 与有线缆 E 做同波形机械对照 |
| F4 | 4× | 11.17° | 7.71° | 12.61° | 推荐的主采集 |
| F5 | 5× | 13.96° | 9.63° | 15.77° | F4 有充分余量后才运行的可选上探 |

10×不在固件中：它会要求大轴约27.92°、平面推算小关节约31.54°，超过现有±25°/±20°运行行程。固件没有参考加速度运行时中止；大/小轴实测速率360°/s、180°/s终止保护和所有反馈、心跳、遥控、行程保护继续生效。

三档必须按 R1 → F4 → 可选 F5 的顺序执行。更换机械/连接方式后，除线缆外不要同时修改 PID、输出限幅、电源或负载。

## 1. 烧录与初始查询

手动烧录：

```text
NoMachineTemp/AGVSentinel_Gimbal_59490601.axf
```

通过无线远程桌面打开目标机终端：

```bash
cd /home/nuc11--02/yaw-cablefree-trace-59490601-hostfix1

TRACE_PORT=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
TRACE_RUN="captures/cable_free_$(date +%Y%m%d_%H%M%S)"

python3 -u yaw_cablefree_trace_cli.py \
  --port "$TRACE_PORT" --action probe \
  --output "$TRACE_RUN/probe_initial"
```

继续条件：显示 `Firmware 0x59490601, 460800`，phase 0、reason 0、未占用、`error=null`。准备条件由操作员负责：底盘固定、扫掠空间和机械余量足够、反馈正常、遥控输入居中、右拨杆 UP、Pitch 有可靠支撑。

若旧主机目录在 `status` 和 `firmware_info` 都为空时报告 `malformed legacy control frame in trace stream`，这是打开持续工作的 UART 时从载荷内 `0xFF` 开始造成的首次同步错误，不是固件运动故障，也没有取得控制权。改用 `hostfix1`，并为 probe 使用新的输出目录；不需要重新烧录。

## 2. R1 去线缆同波形复现

```bash
python3 -u yaw_cablefree_trace_cli.py \
  --port "$TRACE_PORT" \
  --action dual --phase-set R1 \
  --confirm CABLE_FREE_FIXED_CHASSIS_CLEAR_YAW \
  --output "$TRACE_RUN/r1"
```

看到 `accepted` 后在15秒内将右拨杆拨到 DOWN；运动期间保持 DOWN 和摇杆居中。任何需要人工停止的情况立即拨到 UP。完成后保持 UP，运行：

```bash
python3 review_cablefree_capture.py "$TRACE_RUN/r1" \
  --output "$TRACE_RUN/r1_review.json"

python3 -u yaw_cablefree_trace_cli.py \
  --port "$TRACE_PORT" --action release --confirm REMOTE_UP \
  --output "$TRACE_RUN/release_r1"
```

R1 必须为 `structural_quality_passed=true` 才继续。R1 的目的只是比较去线缆前后残差、保持电流和位置相关性，不重新使用旧 E 的“独立验证”名称。

## 3. F4 主采集

重新确认右拨杆 UP、机构静止且起点有大轴±25°、小关节±20°、航向±20°的实际余量，然后执行：

```bash
python3 -u yaw_cablefree_trace_cli.py \
  --port "$TRACE_PORT" \
  --action dual --phase-set F4 \
  --confirm CABLE_FREE_FIXED_CHASSIS_CLEAR_YAW \
  --output "$TRACE_RUN/f4"
```

完成后拨回/保持 UP，先生成审计，再释放：

```bash
python3 review_cablefree_capture.py "$TRACE_RUN/f4" \
  --output "$TRACE_RUN/f4_review.json"

python3 -u yaw_cablefree_trace_cli.py \
  --port "$TRACE_PORT" --action release --confirm REMOTE_UP \
  --output "$TRACE_RUN/release_f4"

cat "$TRACE_RUN/f4_review.json"
```

若显示 `decision=STOP_BEFORE_F5_AND_REVIEW`，到此停止。若显示 `decision=F5_OPTIONAL_ALLOWED`，代表 F4 数据完整且实测行程、速率、命令均低于对应运行边界的80%，才可以选择执行 F5。

## 4. 可选 F5

```bash
python3 -u yaw_cablefree_trace_cli.py \
  --port "$TRACE_PORT" \
  --action dual --phase-set F5 \
  --confirm CABLE_FREE_FIXED_CHASSIS_CLEAR_YAW \
  --output "$TRACE_RUN/f5"
```

完成后立即保持 UP、审计并释放：

```bash
python3 review_cablefree_capture.py "$TRACE_RUN/f5" \
  --output "$TRACE_RUN/f5_review.json"

python3 -u yaw_cablefree_trace_cli.py \
  --port "$TRACE_PORT" --action release --confirm REMOTE_UP \
  --output "$TRACE_RUN/release_f5"
```

不要跳过 F4 直接运行 F5。F5 即使合格也不继续增加幅度。

## 5. 保存路径

```bash
printf '%s/%s\n' "$PWD" "$TRACE_RUN"
tar -czf "${TRACE_RUN}.tar.gz" -C "$(dirname "$TRACE_RUN")" "$(basename "$TRACE_RUN")"
printf '%s/%s.tar.gz\n' "$PWD" "$TRACE_RUN"
```

通过无线远程桌面的文件传输功能带回整个目录或 `.tar.gz`。每个采集目录必须保留 `raw.bin`、`samples.csv`、`events.jsonl`、`setup.json` 和 `report.json`。
