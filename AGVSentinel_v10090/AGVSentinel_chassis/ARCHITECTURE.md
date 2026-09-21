# 软件架构

## 分层

依赖方向从上到下，控制算法不直接访问某一种电机驱动。

1. `Application`：任务、命令仲裁和板间通信，不包含具体电机对象。
2. `Modules`：底盘、舵轮、云台轴和发射机构控制。
3. `Algorithm`：纯数学算法；`alg_swerve_kinematics` 不依赖硬件。
4. `System`：`sys_robot_actuators` 组装本车实际使用的电机和 CAN 分组。
5. `Library`：`lib_actuator` 定义统一电机接口。
6. `Periphal`：DJI、DM4310、CAN、串口等具体驱动。

## 电机抽象

`Actuator_MotorTypeDef` 提供统一的使能、失能、力矩/速度/位置目标、发送和反馈接口。DJI 电机与 DM4310 的差异只在 `sys_robot_actuators.c` 的适配器中处理。

`RobotActuators_Init()` 是当前硬件装配入口：

- 底盘：四个驱动电机、四个舵向电机和两个 CAN 输出组。
- 云台：大 Yaw、小 Yaw、Pitch、两摩擦轮和拨弹电机。
- DM4310 失能后重新使能时，会恢复位置、速度、力矩及 `kp/kd`，避免发送一次被清零的目标。

## 机构封装

- `module_swerve`：一个舵轮对象，拥有驱动/舵向电机、零位、方向和两套 PID。
- `module_chassis`：拥有四个舵轮，只负责模式、目标变换、运动学和统一输出。
- `alg_swerve_kinematics`：四轮解算与最短舵向路径，可独立测试和复用。
- `module_gimbal_axis`：通用云台轴执行对象，封装使能、目标、反馈和输出。
- `module_gimbal`：组合大 Yaw、小 Yaw、Pitch 三个轴并运行对应闭环。

## 更换硬件

更换同协议电机的 ID、CAN 分组或板上连接时，只修改 `periph_motor.c`、`sys_const.c` 和 `sys_robot_actuators.c` 的装配映射。

接入新协议电机时，在 `sys_robot_actuators.c` 增加一组 `Actuator_MotorOpsTypeDef` 适配函数，然后绑定到现有舵轮或云台轴；底盘运动学、PID 组织和应用任务不需要修改。

更换底盘尺寸时，修改 `CHASSIS_SWERVE_L/W`。更换舵轮零位、方向或 PID 时，修改 `Const_ChassisSteerAngleOffset`、`CHASSIS_*_SIGN` 和对应 PID 参数。

更换整车结构（例如麦克纳姆轮）时，新增运动学模块和新的底盘机构实现，继续复用 `lib_actuator`、通信层、命令仲裁和具体电机适配器。

## 角色边界

`app_board_config.h` 根据 `BOARD_CHASSIS` 或 `BOARD_GIMBAL` 生成角色能力。业务代码使用 `BOARD_HAS_*`，不在模块内部反向调用通信层。云台板的 `app_control` 负责遥控器/PC 仲裁并生成最终底盘目标；底盘板的 `app_control` 只校验 CAN 在线状态并将目标注入 `module_chassis`。云台相对角随板间 CAN 反馈发送，再通过 `Chassis_SetGimbalYaw()` 注入底盘模块。

`app_remote.c` 是未加入 Keil Target 的旧控制实现，仅作为历史参考；当前命令入口是 `app_control.c`。

## 验证

PC 上可运行纯运动学测试：

```powershell
gcc -std=c11 -Wall -Wextra -Werror -IInc/Algorithm Tests/test_swerve_kinematics.c Src/Algorithm/alg_swerve_kinematics.c -o test_swerve.exe
./test_swerve.exe
```
