# `0x59490404` dual A 主机相位边界复核

复核对象：
`/home/nuc11--02/yaw-dual-trace-59490404/captures/dual_can_20260910_213114/dual_a`，本地证据副本位于
`NoMachineTemp/dual-can-59490404-20260910-213114/dual_a`。

## 结论

本次不是固件、遥控器、CAN 或反馈保护终止。主机在第 501 个线上样本到达时，用毫秒 tick 判断该样本应
进入 phase 3；固件使用精度更高的 trace 微秒时钟调度，当时尚差 424 µs 才到 2 秒边界，记录 phase 2
正确。旧主机抛出 `waveform phase differs from profile` 后中止接收并尝试发送 CANCEL。

hostfix1 使用与固件同源的 `trace_us` 判断 2 s、16 s 和 20 s 相位边界，仍保留毫米 tick 连续性以及
微秒间隔不超过 10 ms 的独立检查。固件无需重烧；由于 CANCEL 会清除 BENCH 资格，重试前必须重新完成
BENCH A。

## 证据

- 已接收的前 500 条记录到 `tick elapsed=1996 ms`、`trace_us=1,995,576`；
- 缓冲区中完整保留第 501 个 163 字节 CAN trace 帧，CRC 正确；
- 第 501 个样本为 `tick elapsed=2000 ms`、`trace_us=1,999,576`、`interval_us=4000`、`phase=2`；
- 修正版可读入该帧，使原始流恢复为 501 条，未产生 CRC、CAN、反馈或相位错误；
- 失败前 CAN enqueue failure、bus error 和 abort 均为零，命令覆盖完整；
- 主机 identification/slow/CAN/dual 测试共 62 项通过，其中新增真实 2 秒边界回归测试。

这份截断数据不能用于辨识；它只证明主机边界判断错误和固件运行正常。下一轮仍从 BENCH A 开始。
