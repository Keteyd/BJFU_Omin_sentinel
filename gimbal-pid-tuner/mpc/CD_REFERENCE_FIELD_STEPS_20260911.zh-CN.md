# `0x59490501` C/D 双参考现场采集步骤

日期：2026-09-11。适用固件 `0x59490501`、trace v5，配套主机目录
`/home/nuc11--02/yaw-cd-trace-59490501`。只按本文执行 C/D；旧 A/B 命令不适用于本固件。

## 一、开始前

操作员负责确认底盘固定、弹丸及发射机构隔离、Pitch 支撑可靠、两级 yaw 与线缆有足够余量、
遥控在线且右拨杆处于 UP，并可直接触达独立云台断电/急停。C/D 不要求先运行 BENCH。

固件没有参考加速度运行时中止条件；200°/s²是离线参考波形审核线。D 的最大参考加速度为
95.24°/s²。遥控 UP、心跳、反馈、行程、实测速率、采集和输出有效性保护仍生效。

## 二、烧录和部署

烧录本地产物：

```text
NoMachineTemp/AGVSentinel_Gimbal_59490501.axf
```

AXF SHA256：
`2750E13572E052D2E45C8B3AA3B0A13DDC987DBF2D36CFAE661FBC353C10D50F`。

将 `NoMachineTemp/yaw-cd-trace-59490501.zip` 上传到目标机并解压为：

```text
/home/nuc11--02/yaw-cd-trace-59490501
```

在目标机校验并准备本轮目录：

```bash
cd /home/nuc11--02/yaw-cd-trace-59490501
sha256sum -c SHA256SUMS

TRACE_PORT=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
TRACE_RUN="captures/cd_$(date +%Y%m%d_%H%M%S)"
```

每次重来必须生成新的 `TRACE_RUN`，主机不会覆盖已有目录。

## 三、probe

```bash
python3 -u yaw_cd_trace_cli.py \
  --port "$TRACE_PORT" --action probe \
  --output "$TRACE_RUN/probe_initial"
```

只在显示 `Firmware 0x59490501, 460800`、`error=null`、未占用（flags bit 0 为0）且反馈有效
（flags bit 4 为1，即 `flags & 16 == 16`）时继续。稳定的全条件参考值通常为 `flags=252`；
若为 `flags=236`，表示反馈有效位为0，禁止启动 C，应先恢复 IMU、两台 yaw 电机在线状态和反馈时效。
若烧录后仍报告旧构建，退出 Keil 调试、复位主控并重新 probe。

C/D 专用工具不提供 BENCH 和单轴 trial；零输出链路诊断应使用与其 profile 匹配的专用工具，不能用
`yaw_cd_trace_cli.py --action bench`。

## 四、采集 C

```bash
python3 -u yaw_cd_trace_cli.py \
  --port "$TRACE_PORT" \
  --action dual --phase-set C \
  --confirm CD_REFERENCE_FIXED_CHASSIS_CLEAR_YAW \
  --output "$TRACE_RUN/dual_c"
```

显示 `DUAL C accepted` 后，在15秒内将右拨杆从 UP 拨到 DOWN；运动期间保持 DOWN 和摇杆中立。
异常时立即拨回 UP；必要时使用独立急停。完成后将拨杆保持在 UP，不 RELEASE、不运行 D，并回传
完整终端输出和：

```bash
printf '%s/%s\n' "$PWD" "$TRACE_RUN"
```

C 的继续条件是 `download_complete=true`、`crc_errors=0`、`quality_issues=[]`、phase 5、reason 0、
CAN failed/errors/aborted 全为0、profile 4、trace version 5、完整覆盖36秒。标准记录数为9001；若因
任务调度少1～5条，只能在报告明确给出完整时长、最大间隔不超过10 ms、终止样本匹配、无固件质量
锁存且 `quality_issues=[]` 时接受，并在离线拟合前按 `trace_us` 重采样。

## 五、C 审核后释放并采集 D

只有 C 的原始文件已经离线复核通过后才执行。保持右拨杆 UP：

```bash
python3 -u yaw_cd_trace_cli.py \
  --port "$TRACE_PORT" --action release --confirm REMOTE_UP \
  --output "$TRACE_RUN/release_c"

python3 -u yaw_cd_trace_cli.py \
  --port "$TRACE_PORT" --action probe \
  --output "$TRACE_RUN/probe_after_release_c"
```

probe 确认未占用后，重新完成人工现场检查，并新建一次输出根目录：

```bash
TRACE_RUN="captures/cd_d_$(date +%Y%m%d_%H%M%S)"

python3 -u yaw_cd_trace_cli.py \
  --port "$TRACE_PORT" \
  --action dual --phase-set D \
  --confirm CD_REFERENCE_FIXED_CHASSIS_CLEAR_YAW \
  --output "$TRACE_RUN/dual_d"
```

显示 `DUAL D accepted` 后按与 C 相同的方法从 UP 拨到 DOWN。结束或异常后保持 UP，不 RELEASE，
回传完整输出和目录路径。D 是一次性独立验证集；在主检验完成之前，不得依据 D 修改模型系数、
结构、延迟、正则化、缩放或验收阈值。

## 六、异常收尾

右拨杆 UP 是运行中的直接操作停止条件。主机异常退出会尝试发送 CANCEL；无论是否收到确认，都先
保持 UP，再运行 probe。终端 phase 6 或 reason 非0的记录保留作故障证据，不覆盖、不续跑。
只有终止 phase 5/6、UP 新鲜且输出为零时才能用 `--action release --confirm REMOTE_UP` 解除所有权。
