# 反向小yaw独立验证：采集成功，冻结候选未通过

最新进展见[输入时间对齐复核](TIMING_ALIGNMENT_20260909.zh-CN.md)：受控合成验证通过，实车模型仍未通过。
本页冻结模型和反向验证结果保持不变。

此前离线进展见[静摩擦参数恢复与加权积分复核](STICTION_RECOVERY_20260909.zh-CN.md)。
新方法仍未通过，本页冻结候选及反向验证结论保持不变。

历史串口流被动完整性审计见[CAPTURE_INTEGRITY_20260909.zh-CN.md](CAPTURE_INTEGRITY_20260909.zh-CN.md)，未修改固件或原始数据。

后续输入时序核对及新离线候选结果见[INPUT_TIMING_REVIEW_20260909.zh-CN.md](INPUT_TIMING_REVIEW_20260909.zh-CN.md)。
本文件保留原冻结模型的独立验证记录，未用新拟合覆盖历史结论。

## 数据与冻结条件

- 固件0x59490203，试验id=6，小yaw航向±10°，reverse=1，20秒、5001点。
- 文件：`NoMachineTemp/small_slow_reverse_amp10_20260909_220829`。
- CRC错误0、无反馈故障；4998个4ms间隔，1个5ms（索引1666）、末尾1个3ms，最大网格偏差1ms。
- 实际航向范围约-9.409°~+9.514°，大yaw变化不超过0.044°。
- 与原小yaw试验相比，固定PID/输出/方向配置相同；小yaw起点差-1.318°，Pitch/roll差均小于0.06°。
- IMU绝对航向参考发生变化，验证按每轮相对角和窗口初始航向处理，不修改IMU或编码器尺度。

冻结模型来自`NoMachineTemp/coupled-model-20260909-reviewed`，仅读取，不重新拟合/选择。
candidate.json SHA256为`375d9638f8528b4d29e07b0664365884ff5218aa2c782c1cfa6ceafe80b268b5`，验证后哈希一致。
新原始流与训练来源SHA256不同。验证工具还比对候选与训练报告、参数配置、固件和反向试验模式。

## 结果

| 预测方式/时间范围 | 冻结模型航向RMSE | 同条件匀速基线 | 保持角度基线 |
| --- | ---: | ---: | ---: |
| 全记录上滚动200ms | 0.091° | 0.092° | 0.430° |
| 起点2秒初始化一次，连续预测到20秒 | 6.997° | 7.114° | 7.003° |

短窗口每次从真实角度/历史速度重新初始化，其小误差不能证明模型已正确辨识电机输入响应。
整段预测始终使用实际记录的软件指令，没有中途使用实测角度修正，未能复现实车±10°运动。
整段预测不是要求MPC必须开环运行18秒，而是用于暴露当前候选输入作用不足的问题；
结合原拟合的小yaw输入增益贴搜索下界，当前候选不具备可信的控制设计依据。
不能从这个拟合结果推断实车电机无力、IMU损坏或PID需要增大。

若使用未来真实关节角增量之和计算航向，该18秒窗口的RMSE约0.442°。
它只用于判断测量约束的误差量级，使用了未来测量，绝不是可部署预测器。

本轮结论：采集/传输成功，独立数据有用；冻结动力学候选不通过，禁止用于MPC接管。
新数据本轮未加入训练集。没有因时间告警要求重采，没有下发任何电机命令。

## 软件核对与后续方向

源码初查：小yaw PID输出经GimbalAxis/RobotActuator写入`Motor.output`；
CAN编码与辨识记录均采用`Motor.output * 1000`，未发现此处有大小yaw差异倍率。
但采样的是软件变量，不能证明每一采样周期的CAN发送成功或等同于实测电流/力矩。
此时继续重复同样的角度轨迹，或提高PID，不能直接解决输入响应辨识问题。

暂不安排更多实车采集，不改当前闭环与固件。下一步应先核对输入时序/发送记录及闭环辨识方法，
再决定是否需要额外、独立的输入激励与电流/发送状态记录；任何新的实车激励另行说明并审核。
本轮未实现或启动新的激励模式，不宣称已找到唯一根因。

## 工具与复现

新增`validate_coupled_offline.py`，不调用拟合器，校验模型重建和采集来源；25项相关测试通过。
完整结果：`NoMachineTemp/coupled-reverse-validation-20260909-full.json`。

在工作区根目录执行，输出文件必须不存在：

```powershell
& ./NoMachineTemp/yaw-mpc-venv/Scripts/python.exe gimbal-pid-tuner/mpc/validate_coupled_offline.py NoMachineTemp/coupled-model-20260909-reviewed NoMachineTemp/small_slow_reverse_amp10_20260909_220829 --allow-one-ms-jitter --output NoMachineTemp/reverse-validation-review2.json
```

本轮只通过SSH读取小车PC保存文件，未打开串口、释放控制权、烧录或启用电机。
