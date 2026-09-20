# 底盘加速/减速斜坡（2026-09-14）

> 此首版已由“快速停止与遥控回中死区 v2”取代。新的烧录文件和参数见
> `CHASSIS_RAMP_BRAKE_V2_20260914.zh-CN.md`；本文件保留用于说明首版差异。

## 本次改动

斜坡位于车体 `vx/vy/wz` 指令缩放和坐标变换之后、四舵轮运动学逆解之前。舵向位置 PID、驱动轮速度
PID、舵向平滑参数和云台 MPC 均未修改。

- `vx/vy` 作为一个二维矢量限速，直线起步不会改变运动方向。
- `wz` 使用独立的一维斜坡。
- 松杆、减小速度和改变方向使用减速率；增大同向速度使用加速率。
- 活动模式之间切换时保持当前斜坡状态，避免速度跳变。
- `Chassis_NULL`、板间指令超时和控制停机立即把速度与斜坡状态清零，不等待减速斜坡。
- 单次时间步最大按 20 ms 计算，调度器偶发延迟不会造成一次很大的速度跳变。

## 首版参数

参数单位是当前工程内部的“速度参考单位每秒”，并非已经标定的 m/s² 或 rad/s²。

| 参数 | 数值 | 遥控/导航常用满量程下的估算 |
|---|---:|---:|
| 平移加速率 | 600/s | 遥控 0→330 约 0.55 s |
| 平移减速率 | 1200/s | 遥控 330→0 约 0.28 s |
| 旋转加速率 | 1200/s | 导航 0→600 约 0.50 s |
| 旋转减速率 | 2400/s | 导航 600→0 约 0.25 s |

参数定义在 `AGVSentinel_chassis/Inc/Modules/module_chassis.h`：

```c
#define CHASSIS_TRANSLATION_ACCEL_PER_S (600.0f)
#define CHASSIS_TRANSLATION_DECEL_PER_S (1200.0f)
#define CHASSIS_YAW_ACCEL_PER_S         (1200.0f)
#define CHASSIS_YAW_DECEL_PER_S         (2400.0f)
#define CHASSIS_RAMP_MAX_DT_S           (0.020f)
```

数值越大，响应越快。首次实车确认后，若只觉得起步慢，可只提高 `*_ACCEL_*`；若松杆滑行过长，可只提高
`*_DECEL_*`。建议每次按 25% 调整，不需要改舵向 PID。

## 烧录文件

只需烧录底盘主控板，云台板固件保持当前版本。

- `AGVSentinel_Chassis_ramp_20260914.hex`
- `AGVSentinel_Chassis_ramp_20260914.bin`
- Keil 调试/下载可直接使用 `AGVSentinel_Chassis.axf`

SHA-256：

```text
0F2BCBE27A361590EE5A5C0AD18F94DEE41B6BDFB225221A1F4A8F9BA807A775  AGVSentinel_Chassis_ramp_20260914.hex
771FA7EDFB9CB5DC6BBEF6BC738F8B5BC87B74227C69F13235DAD0856006C2DD  AGVSentinel_Chassis_ramp_20260914.bin
1C2E58A5C7C5AA88A46033C2DBA1566FAB22BFA039932FA8F61066B6D1441DF4  AGVSentinel_Chassis.axf
```

## 首次检查顺序

1. 抬起驱动轮或留出足够空地，遥控器摇杆回中，上电后确认底盘静止。
2. 进入正常手动底盘模式，只给约 20% 前进量：轮速应从零平滑增加，四轮方向保持一致。
3. 保持方向并推到满量程，确认约半秒达到目标，期间没有明显顿挫。
4. 松杆：应在约四分之一秒内平滑减到零；直接切入停机状态时应立即清零。
5. 分别测试横移、斜向和平移反向。斜向起步不应先沿某一单轴窜动，反向应先制动再建立反向速度。

调试器中 `Chassis_StatusData.Chassis_TargetVx/Vy/Wz` 是斜坡前目标，`Chassis_Vx/Vy/Wz` 是进入舵轮逆解
的实际斜坡输出，可用二者直接确认斜坡是否符合预期。

## 验证结果

- 原生 GCC 算法测试：通过。
- 原有舵轮运动学回归测试：通过。
- Keil `AGVSentinel_Chassis` 全工程：0 errors，9 个既有 warning。
- 链接映像确认包含 `ChassisRamp_Reset` 与 `ChassisRamp_Step`。
