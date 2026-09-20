# CAN 扩展版：大 yaw 正向采集操作步骤

> 2026-09-10：本步骤已完成，trial id=2 数据已取回并通过结构复核。
> 不要重复执行本页的大 yaw 运动命令。结果见
> [大 yaw CAN 扩展版正向采集复核](BIG_CAN_TRACE_REVIEW_20260910.zh-CN.md)。

适用固件：`0x59490301`。主机脚本：`yaw-can-trace-59490301-hostfix1`。

本页原目标：完成**一轮大 yaw ±15°、20 秒正向采集**，取得指令区间历史、发送状态与反馈电流。
采完即停，先分析这一轮，再安排小 yaw。当前不部署 MPC、不修改 PID、不扩大幅度。

已完成的 BENCH 原始文件已取回并复核；本流程重新做一次 BENCH，验证修复后的主机导出，并取得新的10分钟资格。
**无需重新烧录固件。** `hostfix1` 只修复 CSV 中 rpm 列重名，原始二进制协议未改变。

## 0. 操作前

- 右拨杆保持 **UP**，摇杆回中。确认 Keil 已退出暂停调试状态，控制板正常运行。
- 沿用此前的固定底盘、无相机负载、常用 Pitch 姿态与原 PID 参数。
- 提前支撑 Pitch：BENCH 和试验终止时会撤去 Pitch 保持力矩，支撑不能干涉 yaw 转动。
- 清空两轴活动范围，线缆留足余量。大 yaw 试验中小 yaw 会为保持初始航向作补偿，不能认为小轴不动。
- 不运行其他占用该串口的采集/调参程序。本流程不要求改保护阈值或手动绕过门禁。

每次只执行当前步骤的命令，看到该步通过条件后再继续。不要把所有命令一次性粘贴执行。

## 1. Windows：把修复版脚本传到 NUC

在 **Windows PowerShell** 执行一次：

```powershell
scp "E:\Project_and_learn\BJFU_Omin_sentinel\NoMachineTemp\yaw-can-trace-59490301-hostfix1.zip" "nuc11--02@192.168.1.113:/home/nuc11--02/"
```

如果提示密码，在该终端输入 NUC 登录密码，输入时不显示字符。无需将密码发到聊天。
看到传输完成且没有错误后再继续。此步骤只传文件，不连接控制板串口。

## 2. NUC：解压并检查脚本

以下步骤均在 **NUC 的同一个 Linux 终端**执行。

```bash
cd /home/nuc11--02
unzip -n yaw-can-trace-59490301-hostfix1.zip
cd /home/nuc11--02/yaw-can-trace-59490301-hostfix1

TRACE_PORT=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
TRACE_RUN="captures/big_can_$(date +%Y%m%d_%H%M%S)"

ls -l "$TRACE_PORT"
python3 -c "from yaw_can_trace_protocol import BUILD, TRACE_FIELDS; assert BUILD == 0x59490301; assert len(TRACE_FIELDS) == len(set(TRACE_FIELDS)); assert 'big_feedback_rpm' in TRACE_FIELDS and 'small_feedback_rpm' in TRACE_FIELDS; print('HOSTFIX1 OK')"
printf '本轮文件将保存在：%s/%s\n' "$PWD" "$TRACE_RUN"
```

通过条件：串口路径存在，最后检查显示 **`HOSTFIX1 OK`**，没有 Python 报错。
`unzip -n` 不覆盖已有文件；如果目标目录之前已存在，仍必须通过上述脚本检查。
若关闭或切换终端，先恢复工作目录及 `TRACE_PORT`、`TRACE_RUN` 变量再执行后续命令。

## 3. 查询固件，释放上一轮已结束的辨识

保持 UP，先查询：

```bash
python3 -u yaw_can_trace_cli.py \
  --port "$TRACE_PORT" --action probe \
  --output "$TRACE_RUN/probe_before"
```

通过条件：显示 **`Firmware 0x59490301`**、`error: null`。
上一轮 BENCH 正常结束且未释放时，通常为 `phase: 5`。若显示仍在运行（phase 1–4），停在这里，保持 UP 并回传输出。

确认上一轮已经结束后，单独执行 RELEASE：

```bash
python3 -u yaw_can_trace_cli.py \
  --port "$TRACE_PORT" --action release --confirm REMOTE_UP \
  --output "$TRACE_RUN/release_previous"
```

通过条件：`error: null`。若显示 `Already unowned; no RELEASE sent.` 也可继续。
不需要填写旧 trial ID，脚本会先查询当前归属；不要猜测或沿用固定 ID。
释放后仍保持 UP。

## 4. 零输出 BENCH：全程 UP，约20秒

```bash
python3 -u yaw_can_trace_cli.py \
  --port "$TRACE_PORT" \
  --action bench --confirm BENCH_REMOTE_UP \
  --output "$TRACE_RUN/bench"
```

看到 `BENCH accepted` 后继续保持 UP，不拨 DOWN，不碰摇杆，等待程序结束。

全部满足才进入下一步：

| 检查项 | 通过条件 |
| --- | --- |
| 终端提示 | `BENCH PASSED.` |
| 完整性 | `download_complete: true`，`records: 5001` |
| 质量 | `crc_errors: 0`，`quality_issues: []`，`error: null` |
| BENCH资格 | `bench_passed: true` |
| 固件终态 | `last_status` 中 `phase: 5`、`reason: 0` |

两轴末尾 `terminal_pending` 可以非零，不能仅据此认定丢帧；完整计数和报告留给后续分析。
若 BENCH 中出现意外运动或其他异常，保持/立即拨 UP，停止后续步骤并保留本轮目录。

## 5. 单独释放 BENCH，再检查运动前条件

BENCH 完成不会自动释放控制权。保持 UP，执行：

```bash
python3 -u yaw_can_trace_cli.py \
  --port "$TRACE_PORT" --action release --confirm REMOTE_UP \
  --output "$TRACE_RUN/release_bench"
```

确认 `error: null` 后，再查询一次实际状态：

```bash
python3 -u yaw_can_trace_cli.py \
  --port "$TRACE_PORT" --action probe \
  --output "$TRACE_RUN/readiness"
```

用下面的本地检查读取刚保存的报告，不发送任何串口指令：

```bash
python3 - "$TRACE_RUN/readiness/report.json" <<'PY'
import json, sys
with open(sys.argv[1], encoding='utf-8') as f:
    r = json.load(f)
assert r.get('error') is None, r.get('error')
assert r['firmware_info']['build'] == 0x59490301, '固件版本不匹配'
s = r['last_status']
assert not (s['flags'] & 1), '控制权尚未释放'
assert (s['flags'] & 252) == 252, 'UP、回中、反馈、零输出或居中/低速条件未满足'
assert s['extra'] & 1, 'BENCH资格已失效，需要重新做BENCH'
print('READY：状态检查通过；现场范围确认后才能执行下一步')
PY
```

看到 **`READY`** 后才继续。现场仍需确认两轴活动范围和 Pitch 支撑；这个检查不替代现场观察。
BENCH 资格只有10分钟，控制板重启也会失效。若失效，保持 UP，以新的 `TRACE_RUN` 目录从第3步重新开始，
本轮不进入运动步骤。不要覆盖已产生的文件。

## 6. 大 yaw 正向 ±15°：本次唯一的运动采集

再次确认保持 UP、摇杆回中、底盘固定、两轴范围清空、Pitch 已做好支撑，然后执行：

```bash
python3 -u yaw_can_trace_cli.py \
  --port "$TRACE_PORT" \
  --action trial --axis big --amplitude-deg 15 \
  --confirm FIXED_CHASSIS_CLEAR_YAW \
  --output "$TRACE_RUN/big_forward"
```

**只有看到 `ARM accepted` 后，才在15秒内把右拨杆拨到 DOWN。**
命令被拒绝、超时或未看到接受提示时，不拨 DOWN。
从 DOWN 触发开始约20秒内，不动摇杆、不手推云台；出现异常立即拨回 UP，不等待软件退出。

参考时序如下，角度是相对起始大 yaw 关节角的目标偏移，实际跟踪可能有误差：

| 从触发开始 | 大 yaw 参考 |
| --- | --- |
| 0–2秒 | 起点基线 |
| 2–4秒 | 平滑到 +15° |
| 4–7秒 | 保持 +15° |
| 7–11秒 | 平滑到 −15° |
| 11–14秒 | 保持 −15° |
| 14–16秒 | 平滑回起点 |
| 16–20秒 | 回稳记录 |

“正向”表示先正后负，不代表固定的物理顺/逆时针方向。小 yaw 维持起始航向参考，可能随大轴转动补偿。

结束后**立即回 UP 并保持**。预期输出：

- `Capture finished: phase=5 reason=0.`
- `download_complete: true`，`records: 5001`，`crc_errors: 0`，`error: null`。
- `quality_issues: []` 是期望结果；若存在告警，保留原始数据，不自行重复运动来消除告警。

正式 trial 的 `bench_passed: false` 是正常的字段含义，因为本次 action 是 trial；
不要套用第4步的 BENCH 判据判断正式采集失败。
`hardware_takeover_allowed: false` 也仍应保持，它不表示采集失败。

## 7. 采完停在这里，交回目录路径

保持 UP，**先不再 RELEASE、不启动小 yaw 或反向试验**。
在同一终端打印本轮目录：

```bash
printf '%s/%s\n' "$PWD" "$TRACE_RUN"
```

将这个路径和最后的终端报告发给我即可。我会按此前方式通过 SSH 取回整轮目录并分析，
你不需要逐个挑选或传输文件。若 SSH 再次要求认证，只在本机传输窗口输入密码。

整轮目录下的 `bench/` 与 `big_forward/` 都要保留各自的 `raw.bin`、`samples.csv`、`report.json`、
`events.jsonl`、`setup.json`；其余 probe/release 记录也保留。
不删除失败目录，不只保留 CSV。后续重点核对区间输入变化、发送状态、电流方向/响应和模型输入时间定义。

## 出错时停在哪里

| 情况 | 操作 |
| --- | --- |
| 解压/脚本检查失败、串口不存在或被占用 | 不执行 BENCH/ARM，保留错误输出；不要用强制杀进程命令抢占串口 |
| RELEASE 被拒绝 | 保持 UP，停在该步并回传报告；不跳过归属检查直接 ARM |
| BENCH 未通过 | 保持 UP，保留目录；不执行运动采集 |
| 运动前 READY 检查失败 | 不拨 DOWN；若只是资格过期按第5步重建 BENCH，否则回传原因 |
| 已 ARM 但错过15秒 | 保持/回到 UP，等待终止并保留结果，不直接重发 ARM |
| 运动中异常、串口中断或 Python 报错 | 先立即 UP，必要时再 Ctrl+C 结束主机工具；保存已有目录，不自动重试 |
| 采完有时间/状态告警或样本不足 | 保持 UP，将完整目录交回分析，不自行提高幅度或重复采集 |

本文件是当前一轮操作步骤；历史试验命令见旧记录，但不要混用旧目录下的 `yaw_slow_cli.py`。
协议及已完成的实机核查见 [CAN 采集扩展](CAN_TRACE_20260910.zh-CN.md)。
