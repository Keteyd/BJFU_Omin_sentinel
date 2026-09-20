# 双 Yaw MPC 实车部署步骤（当前 `0x59490910`）

> **历史停用记录（2026-09-12）**：`0x59490903` 已确认同时驱动大、小 yaw，但实车表现为响应明显慢于
> 冻结模型预测，并在零操作者输入时出现持续低频振荡。该构建当时停止使用；
> 不再使用它调大限幅或反复试车。
> 复核见 [`MPC_59490903_FIELD_REJECTION_20260912.zh-CN.md`](MPC_59490903_FIELD_REJECTION_20260912.zh-CN.md)。
>
> **当前试验版本（2026-09-13）**：`0x59490910` 保留已验收的 `0x59490908` MPC 参数和 MIDDLE
> 小 Yaw 坐标系底盘控制，并允许左拨杆 DOWN 将 Yaw 后端切换为原双闭环 PID。该 PID 恢复
> 2026-09-07 实车稳定的即时跟随策略和大 Yaw 参数。详细矩阵见
> [`MPC_PID_FALLBACK_20260913.zh-CN.md`](MPC_PID_FALLBACK_20260913.zh-CN.md)。

`0x59490903` 在 `0x59490902` 的 MIDDLE 接线修复之上，增加大 yaw 的 MPC 计算和最终发送许可。
`0x59490902` 会让大 yaw 被旧输出隔离门关闭，并导致 MPC 状态反复复位、小 yaw 抖动，不再用于实车验证。

本阶段不再采集辨识轨迹。目标是烧录已接受模型的部署固件，并分级确认自动模式的两轴速度参考控制。

## 1. 烧录与只读核对

烧录包中的 `AGVSentinel_Gimbal_59490910.axf`。烧录完成后退出 Keil 暂停/单步状态，重启云台板，右拨杆保持
UP。目标机执行：

```bash
cd /home/nuc11--02/yaw-mpc-pid-fallback-59490910

TRACE_PORT=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
TRACE_RUN="probe_$(date +%Y%m%d_%H%M%S)"

python3 -u yaw_mpc_probe_cli.py \
  --port "$TRACE_PORT" --action probe \
  --output "$TRACE_RUN"
```

终端第一行必须是 `Firmware 0x59490910, 460800`，且 JSON 中 `error` 为 `null`、状态 `flags` 表示遥控与
反馈正常。probe 不取得控制权，也不产生运动。

## 2. 原路径检查

底盘固定，小关节处于机械范围中部。右拨杆保持 UP，确认两路 Yaw 都没有输出。右拨杆切到 MIDDLE，左拨杆
先保持 UP 或 MIDDLE，小幅检查 MPC。摇杆回中后把左拨杆切到 DOWN：底盘仍使用 MIDDLE 的小 Yaw 坐标系，
Yaw 后端立即退出 MPC，并恢复原小 Yaw IMU 角度/速度双环以及大 Yaw 即时跟随双环。切换时会清空两种后端
的控制历史，并从当前反馈重新进入，避免沿用上一后端的积分和微分状态。

大 Yaw PID 启动参数为角度环 `50 / 0.001 / 4`、速度环 `0.6 / 0 / 0`、执行量上限 `30`、速度反馈滤波
时间常数 `0.030 s`。即时跟随每周期使用
`wrap(大Yaw编码器角 + 方向符号 × 小Yaw关节角)` 作为大 Yaw 角度参考。

## 3. MPC 分级启用

左拨杆拨回 UP 或 MIDDLE 即可在右拨杆仍处于 MIDDLE 时重新进入 MPC。首次只给小幅航向指令并松开，
观察航向跟随和小关节是否回中。通过后再逐步增加
航向速度。当前试验版不设置普通的大/小轴速度参考及变化率限幅；小 Yaw 机械边界投影、速度 PID 电机输出限幅
以及反馈、遥控和调度异常退出仍生效。

任何方向错误、持续振荡、反馈异常或小关节接近机械边界时，立即把右拨杆拨到 UP。需要保留 MIDDLE 底盘
控制但退出 MPC 时，把左拨杆拨到 DOWN。右拨杆 DOWN 也使用原 PID Yaw，但底盘恢复原大 Yaw 坐标系路径。

## 4. 调试量

若连接 Keil Watch，可查看：

- `GimbalYaw_DiagMpcRequested`：自动模式请求；
- `GimbalYaw_DiagMpcActive`：本周期实际接管两路速度参考；
- `GimbalYaw_DiagMpcReason`：0 正常，1 未请求，2 请求超时，3 反馈无效，4 数值无效，5 调用间隔异常；
- `GimbalYaw_DiagMpcBigRefDps` / `GimbalYaw_DiagMpcSmallRefDps`：经过最终小关节速度保护后的参考；
- `GimbalYaw_DiagMpcBigUnconstrainedDps` / `GimbalYaw_DiagMpcSmallUnconstrainedDps`：限幅前首步解。

不要用旧 S1/S2/S3 采集命令启动测试；部署固件会明确拒绝这些激励请求。
