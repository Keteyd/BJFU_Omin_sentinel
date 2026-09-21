# AGVSentinel 底盘固件

> 本文件夹只用于底盘主控板，请勿烧录到云台主控板。

打开 [MDK-ARM/AGVSentinel_Chassis.uvprojx](MDK-ARM/AGVSentinel_Chassis.uvprojx)，工程中只有一个 Target：`AGVSentinel_Chassis`，编译宏为 `BOARD_CHASSIS`。

底盘固件负责：

- 接收云台板通过 CAN 下发的最终底盘运动目标和云台反馈。
- 进行四舵轮运动学、转向和驱动 PID 控制。
- 通过 USART1 接收并校验裁判系统数据。
- 通过 CAN 向云台板回传裁判数据和底盘在线状态。
- 云台或运动命令失联时独立停车，不依赖 PC。

通信协议见 [PC_COMM_README.md](PC_COMM_README.md)，软件分层见 [ARCHITECTURE.md](ARCHITECTURE.md)。

此目录保留了公共源码和被角色宏屏蔽的云台源码，以保证工程可独立维护；云台固件位于相邻的 `AGVSentinel_gimbal` 文件夹。
