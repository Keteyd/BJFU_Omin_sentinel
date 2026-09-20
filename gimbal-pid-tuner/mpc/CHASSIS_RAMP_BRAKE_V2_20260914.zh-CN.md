# 底盘斜坡快速停止与遥控回中死区 v2（2026-09-14）

## 对溜车现象的判断

首版斜坡的平移减速率为 `1200/s`，遥控满量程松杆后仍会按设计保留约 0.28 秒的速度。同时，遥控解析只
减去通道中心值，没有死区；摇杆回中偏差几个计数时，底盘会一直收到微小的非零速度目标。

驱动轮已有速度 PID。底盘处于活动模式且斜坡输出为零时，速度 PID 会根据轮速反馈产生反向制动力。因此
本版先消除斜坡和遥控中心漂移造成的持续运动，不修改驱动轮 PID。

## v2 行为

- `Chassis_GIMBAL` 模式的平移遥控输入使用二维圆形死区，半径为 15 个遥控计数。
- 明确回零时，平移停止率从普通减速的 `1200/s` 提高为 `4000/s`，满量程约 0.08 秒归零。
- 普通降速和非零方向切换仍使用 `1200/s`，保留平顺性。
- 旋转明确回零使用 `6000/s`；普通旋转减速仍为 `2400/s`。
- 增速参数保持不变：平移 `600/s`，旋转 `1200/s`。
- 通信超时和 `Chassis_NULL` 仍立即停止并清空斜坡状态。
- 舵向位置 PID、驱动轮速度 PID、舵向平滑和云台控制均未修改。

参数位于 `AGVSentinel_chassis/Inc/Modules/module_chassis.h`：

```c
#define CHASSIS_TRANSLATION_ACCEL_PER_S           (600.0f)
#define CHASSIS_TRANSLATION_DECEL_PER_S           (1200.0f)
#define CHASSIS_TRANSLATION_STOP_PER_S            (4000.0f)
#define CHASSIS_YAW_ACCEL_PER_S                   (1200.0f)
#define CHASSIS_YAW_DECEL_PER_S                   (2400.0f)
#define CHASSIS_YAW_STOP_PER_S                    (6000.0f)
#define CHASSIS_GIMBAL_TRANSLATION_DEADBAND_COUNTS (15.0f)
```

## 烧录

只烧录底盘主控板，云台板固件不变。推荐使用：

- `AGVSentinel_Chassis_ramp_brake_v2_20260914.hex`
- Keil 调试/下载使用 `AGVSentinel_Chassis_ramp_brake_v2_20260914.axf`

SHA-256：

```text
FC903D5E59091D22A9BE50C1875BEF6FDBD0DFD3C14D91D036E54363EFB237F7  AGVSentinel_Chassis_ramp_brake_v2_20260914.hex
1A24942DA44C56D7D568E14FE44F8206AAADB3CE90ABAD0BF318BE77FC250BE1  AGVSentinel_Chassis_ramp_brake_v2_20260914.bin
452BA13C14A178D9E5E58BF04EA366E364D68E80416B15A720984C8BD39998C3  AGVSentinel_Chassis_ramp_brake_v2_20260914.axf
```

## 实车判定

1. 低速前进后松杆，轮速参考应在约 0.08 秒内归零，底盘不应继续接收小速度目标。
2. 摇杆回中静置，底盘应持续静止；轻微中心抖动不应触发轮速。
3. 松杆后若短暂减速并很快停住，问题已经解决。
4. 若速度参考已归零，轮子仍依靠惯性长时间转动，则问题位于驱动轮速度闭环。下一步应记录四轮
   `target_speed`、编码器 `speed` 和 PID `output`，再提高速度环零速制动力，不能继续仅提高停止斜坡。

调试器中：

- `Chassis_StatusData.Chassis_TargetVx/Vy/Wz`：死区与缩放之后、斜坡之前的目标。
- `Chassis_StatusData.Chassis_Vx/Vy/Wz`：实际送入舵轮运动学的斜坡输出。
- `Chassis_ControlData[i].drive_pid.output`：第 i 个驱动轮速度环最终输出。

## 验证

- 底盘斜坡单元测试通过。
- 舵轮运动学回归测试通过。
- Keil `AGVSentinel_Chassis`：0 errors，9 个既有 warning。
