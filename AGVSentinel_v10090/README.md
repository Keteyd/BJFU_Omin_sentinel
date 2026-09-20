# AGVSentinel 双板固件

本目录下的两个工程分别用于不同主控板，请勿交叉烧录：

| 文件夹 | 主控板 | Keil 工程 |
| --- | --- | --- |
| `AGVSentinel_chassis` | 底盘主控板 | `MDK-ARM/AGVSentinel_Chassis.uvprojx` |
| `AGVSentinel_gimbal` | 云台主控板 | `MDK-ARM/AGVSentinel_Gimbal.uvprojx` |

两份工程都使用 Keil 5 + Arm Compiler 6。源码目前各自保留完整公共层，以保证每个文件夹都能独立打开和编译。

修改 `Library`、`Algorithm`、通用 `Periphal` 或板间通信协议时，需要同步检查另一份工程。底盘和云台特有功能则只在对应目录修改。
