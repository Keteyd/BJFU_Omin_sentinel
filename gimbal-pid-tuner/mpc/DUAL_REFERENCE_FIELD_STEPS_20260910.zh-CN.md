# `0x59490405` 双参考现场步骤

日期：2026-09-10。适用固件 `0x59490405`、trace v4，配套主机目录
`/home/nuc11--02/yaw-dual-trace-59490405-hostfix1`。这是当前唯一可执行的双参考现场步骤。

> 2026-09-10：A/B 均已完成，本文中的运动命令只保留为采集记录，不要再次执行。当前只可按第五节末尾
> 的 `release_b` 和 `probe_after_release_b` 收尾；随后停止现场辨识。

## 本版准入规则

- ARM 不再检查遥控是否 UP/新鲜、摇杆中立、起点居中、低速和预留行程；这些准备工作由操作员负责。
- BENCH 不再是单轴或双参考 trial 的前置资格，也没有 10 分钟有效期。
- BENCH 命令保留为可选的 20 秒零输出诊断，用于检查采集链及静态反馈电流偏置。
- 固件仍拒绝并发辨识/调参、旧 trial ID、非法请求、非零既有 yaw 输出、无效配置和不可用采集缓冲。
- ARM 接受后仍要让固件先观察到非 DOWN，再在 15 秒内拨到 DOWN。若 ARM 时已经 DOWN，必须先拨回
  UP 或 MIDDLE，再拨到 DOWN，避免主机命令本身直接启动运动。
- DOWN 后及整个运动期间，遥控、摇杆中立、反馈新鲜度、绝对/相对行程、速度、姿态、主机心跳、采集流和
  输出控制链保护继续生效。

详细变更见 [`OPERATOR_QUALIFIED_ARM_REVIEW_20260910.zh-CN.md`](OPERATOR_QUALIFIED_ARM_REVIEW_20260910.zh-CN.md)。

## 一、人工检查

操作员在运行命令前自行确认：

1. 底盘机械固定，弹丸清空，发射机构断能或隔离；
2. Pitch 有可靠支撑，支撑不会进入两级 yaw 的扫掠空间；
3. 两级 yaw 和线缆具有足够行程，小轴编码器零点/方向与当前装配一致；
4. 遥控在线，摇杆中立，右拨杆处于 UP；
5. 操作员可直接切断云台动力，不依赖 SSH 或串口。

## 二、烧录与主机准备

2026-09-10 部署状态：操作员已报告完成固件烧录；hostfix1 主机包已上传并解压到下述目录，
目标机 `sha256sum -c SHA256SUMS` 已全部通过。发布 ZIP 的外部哈希记录在项目状态与 A 组复核文档中。
串口 probe 随后确认 `Firmware 0x59490405, 460800`、idle/unowned、reason 0、`error=null`。

烧录本地产物：

```text
E:\Project_and_learn\BJFU_Omin_sentinel\NoMachineTemp\AGVSentinel_Gimbal_59490405.axf
```

烧录后退出 Keil 调试并保持右拨杆 UP。将主机包上传并解压到：

```text
/home/nuc11--02/yaw-dual-trace-59490405-hostfix1
```

进入目录并设置串口与本轮唯一输出目录：

```bash
cd /home/nuc11--02/yaw-dual-trace-59490405-hostfix1

TRACE_PORT=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
TRACE_RUN="captures/dual_operator_$(date +%Y%m%d_%H%M%S)"
```

每次重新开始都要生成新的 `TRACE_RUN`，不要覆盖旧目录。

## 三、只读查询

```bash
python3 -u yaw_dual_trace_cli.py \
  --port "$TRACE_PORT" --action probe \
  --output "$TRACE_RUN/probe_initial"
```

继续条件：

- 显示 `Firmware 0x59490405, 460800`；
- `error` 为 `null`；
- 未占用时 `flags` bit 0 为 0。其他状态位只作现场参考，不再作为主机 ARM 门禁。

若仍被旧 trial 占用且处于 phase 5 或 6，保持 UP 后释放：

```bash
python3 -u yaw_dual_trace_cli.py \
  --port "$TRACE_PORT" --action release --confirm REMOTE_UP \
  --output "$TRACE_RUN/release_old"
```

重新 probe，确认未占用。

## 四、直接执行双参考 A

无需先运行 BENCH：

```bash
python3 -u yaw_dual_trace_cli.py \
  --port "$TRACE_PORT" \
  --action dual --phase-set A \
  --confirm DUAL_REFERENCE_FIXED_CHASSIS_CLEAR_YAW \
  --output "$TRACE_RUN/dual_a"
```

终端显示 `DUAL A accepted` 后，在 15 秒内将右拨杆从 UP 拨到 DOWN。运动开始后保持 DOWN、摇杆中立，
不要操作鼠标、滚轮、键盘控制、Keil 调试或其他调参工具。异常时拨回 UP 并保持。

本轮完成或中止后都立即保持 UP，不自动 RELEASE。把完整终端输出和目录路径回传：

```bash
printf '%s/%s\n' "$PWD" "$TRACE_RUN"
```

数据放行至少要求：`download_complete=true`、`crc_errors=0`、`quality_issues=[]`、phase 5、reason 0、
CAN failed/errors/aborted 全为 0。`records=5001` 可直接通过；4996～5000 条只有在
`capture_timing.full_duration_timing_accepted=true`、完整覆盖 20 秒、最大间隔不超过 10 ms、终止样本匹配且
`firmware_stream_quality_latched=false` 时才通过，并明确记录条数差和重采样要求。模型不允许直接接管硬件。

2026-09-10 A 组 trial 1 已完成并离线放行，不要重复执行 A。复核见
[`DUAL_A_59490405_REVIEW_20260910.zh-CN.md`](DUAL_A_59490405_REVIEW_20260910.zh-CN.md)。

## 五、释放 A 并执行双参考 B

保持右拨杆 UP，进入 hostfix1 目录并创建新的输出目录：

```bash
cd /home/nuc11--02/yaw-dual-trace-59490405-hostfix1

TRACE_PORT=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
TRACE_RUN="captures/dual_b_$(date +%Y%m%d_%H%M%S)"

python3 -u yaw_dual_trace_cli.py \
  --port "$TRACE_PORT" --action release --confirm REMOTE_UP \
  --output "$TRACE_RUN/release_a"

python3 -u yaw_dual_trace_cli.py \
  --port "$TRACE_PORT" --action probe \
  --output "$TRACE_RUN/probe_after_release"
```

probe 必须显示 `Firmware 0x59490405, 460800`、phase 0、flags bit 0 为 0、`error=null`。重新完成人工机械、
支撑、线缆、遥控和独立断电检查后执行：

```bash
python3 -u yaw_dual_trace_cli.py \
  --port "$TRACE_PORT" \
  --action dual --phase-set B \
  --confirm DUAL_REFERENCE_FIXED_CHASSIS_CLEAR_YAW \
  --output "$TRACE_RUN/dual_b"
```

显示 `DUAL B accepted` 后 15 秒内从 UP 拨到 DOWN，运动期间保持 DOWN 和摇杆中立。结束或异常时立即保持
UP，不 RELEASE、不执行其他运动；回传完整终端输出和 `printf '%s/%s\n' "$PWD" "$TRACE_RUN"` 结果。

2026-09-10 B 组 trial 2 已完成并离线放行，4999 条完整覆盖 20 秒；不要重复执行 A 或 B。冻结 A→B
及 B→A 对称验证均未达到预定模型门槛，详见
[`DUAL_AB_59490405_VALIDATION_20260910.zh-CN.md`](DUAL_AB_59490405_VALIDATION_20260910.zh-CN.md)。

若当前只需结束辨识占用、恢复普通控制，保持右拨杆 UP，并为输出使用尚不存在的新目录：

```bash
python3 -u yaw_dual_trace_cli.py \
  --port "$TRACE_PORT" --action release --confirm REMOTE_UP \
  --output "$TRACE_RUN/release_b"

python3 -u yaw_dual_trace_cli.py \
  --port "$TRACE_PORT" --action probe \
  --output "$TRACE_RUN/probe_after_release_b"
```

probe 应显示 phase 0、flags bit 0 为 0、`error=null`。此后停止现场辨识，不执行新的运动命令。

## 六、可选零输出 BENCH

只有需要重新测量静态电流偏置或排查串口/CAN 采集链时才运行：

```bash
python3 -u yaw_dual_trace_cli.py \
  --port "$TRACE_PORT" \
  --action bench --confirm BENCH_REMOTE_UP \
  --output "$TRACE_RUN/bench_optional"
```

BENCH 全程保持 UP，双 yaw 和 Pitch 输出关闭，记录 5001 点。它的通过只表示这次零输出数据合格，
不会授予或延长任何运动资格，也不是 dual A 的前置条件。
