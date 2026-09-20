# 双参考第一次 BENCH A 主机同步问题复核

第一次 v4 BENCH A（trial id 1）安全完成，固件状态为 `phase=5 reason=0`，5001 条记录齐全，两轴
CAN failed/errors/aborted 均为 0。原报告却给出 `discarded_wire_bytes`、`incomplete_stream` 和
`bench_passed=false`，因此现场正确停止，没有进入 dual A。

取回的 `raw.bin` 长 819339 字节。逐帧审计确认：

- 开头 14 字节是打开持续工作的 UART 时上一条状态帧的尾部；
- 中间有 5001 条 CRC16 正确、序号连续的 163 字节 trace 帧；
- profile、初始/终止元数据完整，终止样本和终止元数据一致；
- 结尾 2 字节是下一条状态帧的 `ff 37` 前缀；
- 没有 trace CRC 错误、试验内丢字节或 CAN 发送错误。

旧主机把试验前同步字节计入质量故障，并要求任意后续控制帧也必须在退出前接收完整。由于
`quality_ok` 被提前判为 false，主机没有继续等待固件的 BENCH 资格状态。固件最终 `extra=4`，
所以 trial id 1 虽可作为结构完整的零输出记录保留，但不能作为 dual A 的 BENCH 资格。

`hostfix1` 做了两项限定修复：只把首个试验片段出现后的丢字节认定为采集故障；当 5001 条数据和
终止元数据已经完整时，允许缓冲区停在独立状态回复的 `ff 37` 前缀。未完成的 `ff 3f` trace 帧
仍会判为 `incomplete_stream`，试验期间的任意丢字节仍会判为质量故障。

真实 trial id 1 的 raw 经修复版解码后为：`download_complete=true`、`quality_issues=[]`、
`pre_sync_discarded_wire_bytes=14`、`in_stream_discarded_wire_bytes=0`、
`buffered_wire_bytes=2`，且这 2 字节明确标记为完整数据后的部分状态帧。新增的实时主机模拟复现
相同分块，并确认主机会继续接收状态直到 `bench_passed=true`。

后续动作：固件无需重烧；使用 `yaw-dual-trace-59490401-hostfix1`，保持右拨杆 UP，先释放当前
trial id 1，再重新执行 BENCH A。新的 BENCH 报告必须由固件明确给出 `bench_passed=true` 后才可
进入 dual A。

