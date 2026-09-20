# 双自由度结构化模型：首轮结果与验证步骤

最新：下方反向采集已由用户完成，冻结验证结果见[反向验证报告](REVERSE_VALIDATION_20260909.zh-CN.md)。
候选未能复现整段运动，当前不部署MPC，不再重复执行本文末尾采集命令；保留其作为历史步骤。

## 已完成

新增`identify_coupled_offline.py`，复用原始流审计、4ms重采样与固定数据划分。
没有修改任何固件、PID、IMU参数或电机保护，没有打开实车串口。

模型机械状态为`[q_big, q_small, v_big, v_small]`，角度单位弧度。
角度尺度严格为1，航向变化等于两个关节角变化之和。
局部动力学为`M * dv/dt + D*v + F*tanh(v/vs) = G*u_filtered`：

- M为对称正定且有交叉项的归一化惯性矩阵，M11固定1以消除整体尺度不确定性。
- D、F为非负对角阻尼与平滑库仑摩擦，G为正对角软件指令增益。
- 未把软件指令当作测量力矩；这些数值不能解释成实车绝对惯量或电机力矩常数。
- 比较0/20/50ms执行器一阶响应候选。若使用滞后，机械四状态之外还有两个输入滤波状态。
- M采用当前姿态附近的常量近似，尚未验证不同Pitch、相机负载或更大角度范围。

在训练段使用100ms积分窗口拟合速度变化，避免直接对编码器求二阶差分。
优化残差是在速度变化单位下计算，不允许通过缩小惯性矩阵某行来虚假降低误差。
状态初始速度采用40ms尾随拟合，仅用预测起点之前的数据；每个预测窗口只初始化一次IMU角偏置。
预测期间无未来角度泄漏，使用的是记录的未来软件指令，而非已部署MPC产生的指令。

## 结果

固定分段仍为0~7秒训练、7~14秒选择、14~20秒评估。此前已看过这些数据，
因此评估不能再称作从未查看的独立测试集，必须补独立试验。

| 200ms航向RMSE | 结构化模型 | 相同初始速度的匀速基线 | 保持角度基线 |
| --- | ---: | ---: | ---: |
| 大yaw试验 | 0.110° | 0.112° | 0.080° |
| 小yaw试验 | 0.099° | 0.101° | 0.386° |

相较先前ARX的0.279°/0.318°，数值变小；但结构、初始速度估计和航向初始化方法也发生了变化，
不能把这一比较直接当成已通过的控制性能提升。与本轮相同状态初始化的基线相比，没有稳定明显优势。
小yaw阻尼和指令增益都贴到搜索下界，意味着当前结果不宜用于控制设计，
不等于小yaw真实阻尼/电机增益几乎为零，更不能据此调大PID。

最终候选的归一化M约`[[1, 0.5673], [0.5673, 0.5146]]`，只供诊断，禁止据此部署MPC。
状态保持`experimental_unvalidated`和`hardware_takeover_allowed=false`。
最终产物：`NoMachineTemp/coupled-model-20260909-reviewed/report.json`及`candidate.json`。
`coupled-model-20260909-v1`是残差权重修订前的开发中间产物，不能作为最终结果。

22项测试通过，包括：已知合成耦合系统参数恢复、正定性、阻尼耗能、因果速度/滞后、
无未来测量泄漏、惯性缩放不能掩盖速度误差，以及已有采集/ARX测试。
合成测试不是实车验证，也不能排除所选模型结构对真实系统仍不充分。

## 下一步：独立反向小yaw试验

仍为小yaw航向±10°、20秒，只交换正负方向顺序，不改幅度、不提高输出、不改PID。
大yaw保持起始关节角。目的是验证方向相关响应，不保证单靠这一轮便能完成辨识。
新数据先对冻结候选评估；若再用于拟合，必须另留新的独立验证数据。
固件0x59490203及现有脚本已支持`--reverse`，无需烧录。

保持车体固定、原负载/Pitch姿态、小yaw在允许起点，清空运动范围；做好Pitch撤力防落措施。
以下每步单独执行，上一条成功后再执行下一条，任何失败都停止继续。

1. 上置，释放前一轮（已经空闲时无操作）。

```bash
cd /home/nuc11--02/yaw-slow-59490203-feedback
PORT=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
python3 -u yaw_slow_cli.py --port "$PORT" --action release --confirm REMOTE_UP --output captures/release_$(date +%Y%m%d_%H%M%S)
```

2. 全程上置，刷新20秒零输出BENCH资格。

```bash
python3 -u yaw_slow_cli.py --port "$PORT" --action bench --confirm BENCH_REMOTE_UP --output captures/bench_$(date +%Y%m%d_%H%M%S)
```

3. 看到BENCH PASSED后仍上置，释放BENCH。

```bash
python3 -u yaw_slow_cli.py --port "$PORT" --action release --confirm REMOTE_UP --output captures/release_$(date +%Y%m%d_%H%M%S)
```

4. 独立启动反向采集。

```bash
python3 -u yaw_slow_cli.py --port "$PORT" \
  --action trial --axis small --amplitude-deg 10 --reverse \
  --confirm FIXED_CHASSIS_CLEAR_YAW \
  --output captures/small_slow_reverse_amp10_$(date +%Y%m%d_%H%M%S)
```

仅第4步看到ARM accepted后15秒内下置，摇杆回中，不手动推动云台。
轨迹为先保持2秒、到-10°、停留、到+10°、停留、回起点，总20秒。
异常立即上置，结束也回上置；Pitch保持力矩在结束时撤去。
本次只提供操作步骤，未替用户释放、执行BENCH或启动任何运动。
