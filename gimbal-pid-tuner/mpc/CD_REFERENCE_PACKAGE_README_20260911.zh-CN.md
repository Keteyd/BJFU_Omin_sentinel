# yaw C/D trace v5 主机包

本目录只用于固件 `0x59490501`、trace version 5、460800 baud。它支持36秒、9001点的固定 C/D
双参考采集，拒绝旧固件和 A/B phase-set。

冻结参考计划 SHA256：
`05DA0E5EAD289CBB3A16B4D243F13F449B523CE68B731799A768253A34DE09EF`。

原100°/s²数值仅为离线波形审核门槛，现为200°/s²；固件没有参考加速度运行时中止逻辑。
遥控停止、反馈、心跳、行程、实测速率、采集和输出有效性保护仍保留。

先运行：

```bash
sha256sum -c SHA256SUMS
python3 -u yaw_cd_trace_cli.py --help
```

现场只按 `FIELD_STEPS.zh-CN.md` 操作。工具不会自动 RELEASE，也不会自动从 C 开始 D。
C/D 专用入口只开放 `probe`、`dual`、`cancel` 和 `release`；它会拒绝不兼容的 BENCH 和单轴 trial。
probe 除了构建号和未占用，还必须确认反馈有效位 `flags & 16 == 16`；`flags=236` 时禁止启动 C。
