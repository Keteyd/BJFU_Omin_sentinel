# `0x59490502` D 组重采步骤

日期：2026-09-11。本步骤只用于修复反馈时间戳竞态后的 D 组重采。C 已由 `0x59490501` 完成并冻结，
不要重采 C。失败目录 `cd_d_20260911_162226` 保留，不删除、不覆盖。

## 1. 烧录和部署

烧录：

```text
NoMachineTemp/AGVSentinel_Gimbal_59490502.axf
```

AXF SHA256：
`D6DE728BA88EB156D2321CA3D7B873C89E0C2496CDCA548B7304E1A9A5A3A3ED`。

将 `NoMachineTemp/yaw-cd-trace-59490502.zip` 上传并解压为：

```text
/home/nuc11--02/yaw-cd-trace-59490502
```

固件复位后，旧固件内存中的 trial 2 所有权会消失，无需再向 `0x59490501` 发送 RELEASE。

## 2. 校验与 probe

保持右拨杆 UP，退出 Keil 调试状态，然后执行：

```bash
cd /home/nuc11--02/yaw-cd-trace-59490502
sha256sum -c SHA256SUMS

TRACE_PORT=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
TRACE_RUN="captures/cd_d_retry_$(date +%Y%m%d_%H%M%S)"

python3 -u yaw_cd_trace_cli.py \
  --port "$TRACE_PORT" --action probe \
  --output "$TRACE_RUN/probe_initial"
```

只在显示 `Firmware 0x59490502, 460800`、phase 0、reason 0、`error=null`、未占用且反馈有效时继续。
完整条件常见值为 `flags=252`；若 `flags & 16 != 16`，先恢复反馈。

## 3. 重采 D

操作员重新确认底盘固定、机构和线缆行程无阻挡、Pitch 支撑可靠、遥控器在线、摇杆中立，并能直接使用
UP 或独立断电急停。执行：

```bash
python3 -u yaw_cd_trace_cli.py \
  --port "$TRACE_PORT" \
  --action dual --phase-set D \
  --confirm CD_REFERENCE_FIXED_CHASSIS_CLEAR_YAW \
  --output "$TRACE_RUN/dual_d"
```

显示 `DUAL D accepted` 后，在 15 秒内从 UP 拨到 DOWN，运动期间保持 DOWN 和摇杆中立。异常时立即拨回
UP。结束后保持 UP，不发送 RELEASE，并回传完整终端输出及：

```bash
printf '%s/%s\n' "$PWD" "$TRACE_RUN"
```

可接受记录必须满足 `download_complete=true`、`quality_issues=[]`、CRC 0、phase 5/reason 0、完整 36 秒、
CAN failed/errors/aborted 全为 0。允许因任务调度少 1 至 5 条，但必须由报告明确给出完整时长且要求重采样。

## 4. 保护和数据纪律

本修复没有放宽 50 ms 反馈新鲜度、遥控 UP、心跳、行程、实测速度、采集流或输出有限性保护，也没有改变
D 的 3.00°/2.00°信号。固件仍没有参考加速度运行时中止；200°/s²只用于离线审核。D 完成后在离线冻结
验证结束前不要 RELEASE，也不要依据 D 修改 C 模型、结构或验收阈值。
