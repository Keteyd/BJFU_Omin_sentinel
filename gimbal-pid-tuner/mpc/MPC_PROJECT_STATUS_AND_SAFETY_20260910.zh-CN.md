# MPC 辨识项目当前状态、文档导航与安全机制

初建日期：2026-09-10；当前更新：2026-09-12。本文是当前总入口。带日期的旧文档保留为证据，
不应从旧文档复制现场命令。

## 当前状态

### 最新结论（2026-09-12 以此为准）

- MPC接口已确定为“大轴电机速度参考 + 小轴惯性航向角速度参考”，原有速度PID继续作为内环。
  `0x59490702` 的 S1/S2 实车采集均已完成并安全 RELEASE；最终 probe 为 `phase=0`、`count=0`、
  `reason=0`、`flags=252`。S1 用于开发并冻结了 20 ms MIMO ARX 速度闭环预测器、参数、重采样和验收门。
  S2 是一次性独立验证，不再重采 S1/S2，也不允许查看结果后修改冻结规则。
- S2 正常运行完整 36 s，`phase=5`、`reason=0`，无 CRC、CAN、反馈或保护故障；但记录为
  8994/9001，短缺 7 条，超过采集前冻结的 5 条上限，因此正式结论是数据门不通过。保持模型和全部数值门不变的
  事后诊断中，200 ms 大/小轴 RMSE 为 3.329/2.496°/s，相对保持预测改善 48.1%/80.9%，四项数值门均通过。
  这些结果支持模型动力学，但不能推翻正式数据门拒绝。当前状态为
  `speed_reference_model_numerically_supported_but_formal_S2_data_gate_failed`，
  `hardware_takeover_allowed=false`。若需要正式独立证据，应先冻结不同频率集的 S3，并把数据质量门定义为
  36 s 时间覆盖、20 ms 重采样覆盖和最大原始间隔；不能放宽或重复使用已查看的 S2。
  见 [`SPEED_REFERENCE_IMPLEMENTATION_REVIEW_20260911.zh-CN.md`](SPEED_REFERENCE_IMPLEMENTATION_REVIEW_20260911.zh-CN.md)
  [`SPEED_S1_MODEL_FREEZE_20260912.zh-CN.md`](SPEED_S1_MODEL_FREEZE_20260912.zh-CN.md) 和
  [`SPEED_S2_59490702_VALIDATION_REVIEW_20260912.zh-CN.md`](SPEED_S2_59490702_VALIDATION_REVIEW_20260912.zh-CN.md)。
- 新的 S3 独立验证已经在未见 S3 数据时完成设计冻结：36 s、峰值 29.5/59.0°/s，八个频点与
  S1/S2 全部错开，模型门完全沿用 S2；数据门改为 36 s 时域覆盖、最大 10 ms 原始间隔和 3–31 s
  的 20 ms 双侧样本覆盖，原始记录短缺只作诊断。计划 build 为 `0x59490801`。当前尚未实现固件、
  主机和冻结验证器，因此不能采集 S3。见
  [`SPEED_S3_DESIGN_FREEZE_20260912.zh-CN.md`](SPEED_S3_DESIGN_FREEZE_20260912.zh-CN.md)。
- F4训练、F5固定检查的实际工作域植物开发已完成。固定运动学工具变量速度模型在F5激励段相对保持
  改善30.5%/43.9%，100 ms滚动RMSE为0.201°/0.254°；但F5回零段漂移4.836°/2.941°，运行前
  常值偏置适配和反馈电流输入均未修复。仅保留短时影子预测候选，开环植物仍未辨识；下一次采集须先
  明确MPC输出接口，并在所保留内环之外注入独立命令扰动。详见
  [`F4_F5_OPERATING_DOMAIN_PLANT_REVIEW_20260911.zh-CN.md`](F4_F5_OPERATING_DOMAIN_PLANT_REVIEW_20260911.zh-CN.md)。
- 去线缆 E/R1 同波形对照及 F4/F5 跨幅度复核已完成。实际跟踪 RMSE 在大关节/小关节/航向上
  改善 9.9%/20.2%/9.8%，但旧带线缆模型在 R1 小关节上的残差由0.532°增至1.039°，且一秒
  慢残差占比仍为92.7%/99.3%/94.7%。旧冻结C模型在F4/F5上的归一化RMSE几乎相同，支持4到5倍
  局部尺度泛化；小关节归一化误差仍约0.84。原始电流基线不是固定零偏，方向代理也未随拆线一致下降。
  详见 [`CABLE_FREE_SCALING_MODEL_REVIEW_20260911.zh-CN.md`](CABLE_FREE_SCALING_MODEL_REVIEW_20260911.zh-CN.md)。
- 当前结论仍是闭环参考映射，不能用于MPC接管。若改为位置/摩擦/慢扰动模型，R1/F4/F5只能作为
  开发数据，必须在结构和门槛冻结后另采独立验证波形；`hardware_takeover_allowed=false`。
- 操作员确认 F4/F5 更接近实际比赛工况。后续模型选择以4–5倍幅值为主要工作域；R1仅用于线缆
  对照、低速摩擦和小信号退化检查，不与F4/F5等权，也不以R1低幅失败否定实际工作域模型。
- `0x59490503` phase E 已完成、通过预声明的时间轴数据门和未经重拟合的冻结 C 模型门，并已安全
  RELEASE。最终 probe 为 phase 0、reason 0、`flags=252`；不要重采 C/D/E。
- E 为8990/9001条、完整36秒，短缺11条低于采集前冻结的45条上限。大/小关节相对保持预测改善
  46.49%/34.11%，均超过30%；C/E电流增益与延迟一致性也通过。详见
  [`E_REFERENCE_VALIDATION_REVIEW_20260911.zh-CN.md`](E_REFERENCE_VALIDATION_REVIEW_20260911.zh-CN.md)。
- 当前模型状态为 `closed_loop_candidate_passed_C_E_gates`。它只描述现有PID闭环参考映射；残差仍有强低频、
  摩擦和偏置结构，不能作为开环MPC植物，`hardware_takeover_allowed=false`。
- C/D/E 方向摩擦、姿态负载和慢偏置分析已完成。逐相位留出表明：常值偏置和仅参考量修正不能泛化；
  加入上一采样的实测位置、因果速度/方向、软件命令和基线中心化电流后，大/小 yaw 最差留出改善为
  59.27%/79.76%。这是事后选择、依赖实测状态的观测器修正候选，不是自由运行植物。详见
  [`CDE_RESIDUAL_FRICTION_REVIEW_20260911.zh-CN.md`](CDE_RESIDUAL_FRICTION_REVIEW_20260911.zh-CN.md)。
- 为区分线缆负载并扩大有效运动范围，已冻结去线缆 R1/F4/F5 幅度阶梯。R1 精确复现原 E；F4/F5
  分别为4倍和5倍。10倍因大轴与小关节行程越界且预计小轴命令饱和而拒绝。`0x59490601`、AXF、
  主机包和 F4 后的80%余量审查工具已完成验证；`hostfix1` 已通过无线地址部署到目标机，只读 probe
  确认 build、反馈和状态正常。固件已烧录但尚未采集 R1/F4/F5，当前执行入口见
  [`CABLE_FREE_LADDER_FIELD_STEPS_20260911.zh-CN.md`](CABLE_FREE_LADDER_FIELD_STEPS_20260911.zh-CN.md)。
- R1/F4/F5仍是闭环开发数据。若目标是 MPC 接管，还需另做已知内环/执行器输入到关节运动的植物辨识和影子验证。
- 去线缆 R1/F4/F5 已完成并取回：8992/8993/8989条，三组均覆盖完整36秒、无质量告警、CRC与CAN
  故障为零。F4通过80%余量门后执行F5；F5最大边界占用为小关节行程64.4%、小轴速度33.3%、
  小轴命令29.7%。最终 RELEASE 和 phase 0、`flags=252` probe 已确认。本轮不再增加幅度，下一步仅做
  离线跨幅度与去线缆对照分析。详见
  [`CABLE_FREE_R1_F4_F5_CAPTURE_REVIEW_20260911.zh-CN.md`](CABLE_FREE_R1_F4_F5_CAPTURE_REVIEW_20260911.zh-CN.md)。

### 历史进展（按证据保留）

- 实车串口 probe 已确认固件 `0x59490405`、460800 baud、trace v4；状态为 idle/unowned、reason 0、`error=null`。
  新 AXF SHA256 为 `8D76398ACB47E14DB9C2334625B33C12112FC069CF0B198B7A06368D5FCB6A78`。
- `0x59490405` phase-set A/B 均已安全完成并通过真实时钟、参考、CAN、CRC 和配置一致性复核；两组
  数据都保留，不需要重采。冻结 A 模型在 B 上的大/小关节改善为 27.6%/22.9%，B 拟合/A 验证为
  42.5%/20.7%，没有满足每个关节至少 30% 的预定门槛，且 ARX 结构由 `8/4/0` 变为 `8/8/2`。
  闭环运动模型已拒绝；电流增益和时序复现良好。MPC 开放对象仍为 `not_identified`，所有报告保持
  `hardware_takeover_allowed=false`。
- `0x59490403` dual A 曾在约 8.026 s 报告大轴控制链失活；`0x59490404` 增加了内部停止点诊断。
- `0x59490404` 第一次 dual A 没有发生固件保护故障。主机把 `tick=2000 ms`、
  `trace_us=1,999,576` 的合法 phase 2 样本误判为 phase 3，退出后发送了 CANCEL。
- `0x59490405` 取消人工可保证的 ARM 准入检查以及 BENCH 前置资格；BENCH 保留为可选零输出诊断。
- 配套主机包 `NoMachineTemp/yaw-dual-trace-59490405.zip` 已生成并部署到目标机
  `/home/nuc11--02/yaw-dual-trace-59490405`；最终 ZIP SHA256 为
  `3454214AF225F5B520C01FA1D171B6E2C70E2508EDF0D4A3F76DE78D188D7F48`，目标机标准 `sha256sum -c` 逐文件验证通过，离线导入回报 build `0x59490405`、trace v4。
- 主机时序判据修正版已部署到 `/home/nuc11--02/yaw-dual-trace-59490405-hostfix1`；ZIP SHA256 为
  `85A0CDEA23EF6539007DC5C541A4B1220FE3F78CDB4A897977BAABC64C9D3DF0`，目标机逐文件哈希和离线版本自检通过，不需要重新烧录。
- 上一轮现场状态为右拨杆 UP、`0x59490405` 锁定已完成的 B 组 trial 2；烧录新固件后状态会复位，
  仍须用新主机工具 probe 确认 `0x59490501` 且未占用。
- 2026-09-11 已完成 A/B 残差、量化/摩擦和公共结构网格分析，并生成 36 秒 C/D 离线波形草案。
  公共 `12/4/0`、ridge 1.0 结构在复用 A/B 的开发检查中最差改善 36.2%，但残差仍为低频相关；
  A/B 已失去未来独立验证资格。操作员已采用3.00°/2.00°低频增强幅度；`0x59490501`、trace v5、
  36秒/9001点主机工具已经实现并通过离线测试。原100°/s²为离线预览线而非固件运行时保护，现按
  审核提高到200°/s²。C/D采集不改变 `hardware_takeover_allowed=false`。
- C/D主机发布包为 `NoMachineTemp/yaw-cd-trace-59490501.zip`，SHA256
  `F771922FE21E318DDB72D4BB57DF7C7BC243C403FE2A63E7D8EC667E1D1E1FFB`；AXF为
  `NoMachineTemp/AGVSentinel_Gimbal_59490501.axf`，SHA256
  `2750E13572E052D2E45C8B3AA3B0A13DDC987DBF2D36CFAE661FBC353C10D50F`。操作员已烧录；主机包已上传
  并通过ZIP外部哈希、包内15文件和Python版本自检。实车初次probe构建号正确且未占用，但连续
  `flags=236` 表明反馈有效位未置位；零输出诊断由固件以 `reason=3 feedback_bad` 中止并已
  RELEASE。操作员恢复反馈后，最新probe为 `flags=252`、phase 0、reason 0、idle/unowned，C已满足
  板端开始条件。
- C组 trial 1 已完成：9000条、完整36秒、CRC 0、`quality_issues=[]`、CAN无故障；逐点参考最大误差
  0.00836°。固定 `12/4/0`、ridge 1.0 的 C-only 模型谱半径0.98698，拟合内大/小关节相对保持改善
  36.9%/39.4%。C采集和冻结模型通过，仍须用未见过的D作一次独立验证。

## 文档脉络

### 当前入口

| 文档 | 用途 | 是否可执行现场命令 |
| --- | --- | --- |
| 本文 | 当前状态、导航、安全审核和待确认清单 | 只作总览 |
| [`SPEED_REFERENCE_FIELD_STEPS_20260911.zh-CN.md`](SPEED_REFERENCE_FIELD_STEPS_20260911.zh-CN.md) | `0x59490702` S1/S2 现场记录与最终 RELEASE | 历史步骤；禁止重采 S1/S2 |
| [`SPEED_S1_59490702_CAPTURE_REVIEW_20260912.zh-CN.md`](SPEED_S1_59490702_CAPTURE_REVIEW_20260912.zh-CN.md) | S1 数据质量、运动余量和回放哈希 | S1 开发数据证据 |
| [`SPEED_S1_MODEL_FREEZE_20260912.zh-CN.md`](SPEED_S1_MODEL_FREEZE_20260912.zh-CN.md) | S1 模型结构、参数、重采样和 S2 验收门冻结 | S2 采集前冻结依据 |
| [`SPEED_S2_59490702_VALIDATION_REVIEW_20260912.zh-CN.md`](SPEED_S2_59490702_VALIDATION_REVIEW_20260912.zh-CN.md) | S2 正式数据门拒绝与不改变冻结规则的数值诊断 | 当前速度模型结论；禁止重采 S2 |
| [`SPEED_S3_DESIGN_FREEZE_20260912.zh-CN.md`](SPEED_S3_DESIGN_FREEZE_20260912.zh-CN.md) | 新频率 S3 波形、时域数据门和不变模型门 | 已冻结设计；实现完成前不可采集 |
| [`DUAL_REFERENCE_FIELD_STEPS_20260910.zh-CN.md`](DUAL_REFERENCE_FIELD_STEPS_20260910.zh-CN.md) | `0x59490405` 已完成 A/B 的现场记录与收尾 RELEASE | 仅可执行收尾 RELEASE/probe，不再执行 A/B |
| [`OPERATOR_QUALIFIED_ARM_REVIEW_20260910.zh-CN.md`](OPERATOR_QUALIFIED_ARM_REVIEW_20260910.zh-CN.md) | ARM 人工准入和可选 BENCH 的代码边界 | 审核依据 |
| [`NEXT_STAGE_DUAL_REFERENCE_PLAN_20260910.zh-CN.md`](NEXT_STAGE_DUAL_REFERENCE_PLAN_20260910.zh-CN.md) | 双参考波形、A/B 分工、数据质量和模型放行门槛 | 设计依据，不整页执行 |
| [`DUAL_REFERENCE_IMPLEMENTATION_REVIEW_20260910.zh-CN.md`](DUAL_REFERENCE_IMPLEMENTATION_REVIEW_20260910.zh-CN.md) | 固件、协议、测试、发布产物与哈希 | 审核依据 |
| [`DUAL_A_59490405_REVIEW_20260910.zh-CN.md`](DUAL_A_59490405_REVIEW_20260910.zh-CN.md) | 实车 A 组时序、参考、CAN、电流与闭环候选复核 | 当前 A 组放行依据 |
| [`DUAL_AB_59490405_VALIDATION_20260910.zh-CN.md`](DUAL_AB_59490405_VALIDATION_20260910.zh-CN.md) | B 组审计、冻结 A→B 和 B→A 对称验证 | 当前模型拒绝依据 |
| [`NEXT_STAGE_CD_DESIGN_20260911.zh-CN.md`](NEXT_STAGE_CD_DESIGN_20260911.zh-CN.md) | A/B 残差、摩擦、公共结构和 C/D 波形 | 设计依据；现场命令以新步骤文档为准 |
| [`CD_REFERENCE_IMPLEMENTATION_REVIEW_20260911.zh-CN.md`](CD_REFERENCE_IMPLEMENTATION_REVIEW_20260911.zh-CN.md) | `0x59490501`、trace v5、9001点实现与测试证据 | 当前实现复核 |
| [`CD_REFERENCE_FIELD_STEPS_20260911.zh-CN.md`](CD_REFERENCE_FIELD_STEPS_20260911.zh-CN.md) | 烧录、部署、C审核后再执行D | 历史现场记录，不再执行C/D |
| [`CD_C_59490501_REVIEW_20260911.zh-CN.md`](CD_C_59490501_REVIEW_20260911.zh-CN.md) | C原始流、参考、输入、电流与冻结模型 | D执行前的放行依据 |
| [`CD_D_RETRY_REVIEW_20260911.zh-CN.md`](CD_D_RETRY_REVIEW_20260911.zh-CN.md) | D完整运行、原计数门拒绝和条件诊断 | 历史D证据，不再重采 |
| [`E_REFERENCE_IMPLEMENTATION_REVIEW_20260911.zh-CN.md`](E_REFERENCE_IMPLEMENTATION_REVIEW_20260911.zh-CN.md) | E采集前冻结的波形、门限、实现和测试 | 设计依据 |
| [`E_REFERENCE_FIELD_STEPS_20260911.zh-CN.md`](E_REFERENCE_FIELD_STEPS_20260911.zh-CN.md) | E现场步骤及保护边界 | E已完成，不再执行运动命令 |
| [`E_REFERENCE_VALIDATION_REVIEW_20260911.zh-CN.md`](E_REFERENCE_VALIDATION_REVIEW_20260911.zh-CN.md) | E数据质量、冻结C模型、电流和残差结果 | 当前模型结论 |
| [`CDE_RESIDUAL_FRICTION_REVIEW_20260911.zh-CN.md`](CDE_RESIDUAL_FRICTION_REVIEW_20260911.zh-CN.md) | C/D/E偏置、方向摩擦、逐相位留出修正与下一模型结构 | 当前离线分析结论 |
| [`CABLE_FREE_LADDER_IMPLEMENTATION_REVIEW_20260911.zh-CN.md`](CABLE_FREE_LADDER_IMPLEMENTATION_REVIEW_20260911.zh-CN.md) | 去线缆1/4/5倍设计、10倍拒绝、保护、构建与部署证据 | 当前实现复核 |
| [`CABLE_FREE_LADDER_FIELD_STEPS_20260911.zh-CN.md`](CABLE_FREE_LADDER_FIELD_STEPS_20260911.zh-CN.md) | `0x59490601` 的 R1/F4/可选F5现场命令 | 已完成的历史步骤，不再执行运动 |
| [`CABLE_FREE_R1_F4_F5_CAPTURE_REVIEW_20260911.zh-CN.md`](CABLE_FREE_R1_F4_F5_CAPTURE_REVIEW_20260911.zh-CN.md) | R1/F4/F5数据质量、余量、哈希和最终释放 | 当前采集结论；不再执行运动 |
| [`CABLE_FREE_SCALING_MODEL_REVIEW_20260911.zh-CN.md`](CABLE_FREE_SCALING_MODEL_REVIEW_20260911.zh-CN.md) | 去线缆前后残差、摩擦、电流基线与F4/F5跨幅度泛化 | 当前离线模型结论 |
| [`F4_F5_OPERATING_DOMAIN_PLANT_REVIEW_20260911.zh-CN.md`](F4_F5_OPERATING_DOMAIN_PLANT_REVIEW_20260911.zh-CN.md) | F4/F5命令/电流到关节模型、滚动预测和回零失败 | 当前植物辨识结论 |

### 当前问题的证据链

| 顺序 | 文档 | 结论 |
| ---: | --- | --- |
| 1 | [`DUAL_A_ABORT_REVIEW_20260910.zh-CN.md`](DUAL_A_ABORT_REVIEW_20260910.zh-CN.md) | `0x59490401` 将多种控制故障合并成 reason 9，证据不足 |
| 2 | [`DUAL_BENCH_TIMING_REVIEW_20260910.zh-CN.md`](DUAL_BENCH_TIMING_REVIEW_20260910.zh-CN.md) | `0x59490402` 把正常 3/5 ms 毫秒量化误判为时序故障 |
| 3 | [`DUAL_A_INACTIVE_REVIEW_20260910.zh-CN.md`](DUAL_A_INACTIVE_REVIEW_20260910.zh-CN.md) | `0x59490403` 的 reason 12 不是 CAN、反馈、遥控或采样故障 |
| 4 | [`DUAL_PHASE_BOUNDARY_REVIEW_20260910.zh-CN.md`](DUAL_PHASE_BOUNDARY_REVIEW_20260910.zh-CN.md) | `0x59490404` 首轮是主机 2 s 边界误判，hostfix1 已修复 |
| 5 | [`DUAL_A_59490405_REVIEW_20260910.zh-CN.md`](DUAL_A_59490405_REVIEW_20260910.zh-CN.md) | `0x59490405` A 组完整覆盖 20 秒；4998 条是固定计数判据误报，可按真实时间重采样 |
| 6 | [`DUAL_AB_59490405_VALIDATION_20260910.zh-CN.md`](DUAL_AB_59490405_VALIDATION_20260910.zh-CN.md) | B 组数据合格；双向预测与结构稳定性门槛失败，闭环运动模型拒绝 |
| 补充 | [`DUAL_BENCH_A_HOST_SYNC_REVIEW_20260910.zh-CN.md`](DUAL_BENCH_A_HOST_SYNC_REVIEW_20260910.zh-CN.md) | 早期 CP2102 状态半帧同步误报及修复 |

### 建模证据链

1. `IDENTIFICATION_PAIR_REVIEW_2026-09-09.zh-CN.md`、`OFFLINE_MODEL_REVIEW_2026-09-09.zh-CN.md`：
   首轮数据和模型失败基线。
2. `SLOW_BIG_REVIEW_20260909.zh-CN.md`、`SLOW_PAIR_REVIEW_20260909.zh-CN.md`、
   `REVERSE_VALIDATION_20260909.zh-CN.md`：慢速正反向采集与独立验证。
3. `INPUT_TIMING_REVIEW_20260909.zh-CN.md`、`TIMING_ALIGNMENT_20260909.zh-CN.md`、
   `STICTION_RECOVERY_20260909.zh-CN.md`：输入时序、量化和静摩擦候选；候选仍被拒绝。
4. `CAN_TRACE_20260910.zh-CN.md`：CAN 指令区间历史、发送状态和反馈电流协议。
5. `BIG_CAN_TRACE_REVIEW_20260910.zh-CN.md`、`SMALL_CAN_TRACE_REVIEW_20260910.zh-CN.md`：
   两轮 CAN 扩展实车记录。
6. `CAN_TRACE_MODEL_REVIEW_20260910.zh-CN.md`、`CLOSED_LOOP_CURRENT_REVIEW_20260910.zh-CN.md`：
   电流偏置/时序和闭环辨识；闭环可描述，开放对象仍不可用。
7. `NEXT_STAGE_DUAL_REFERENCE_PLAN_20260910.zh-CN.md`：由上述失败导出的双参考、多频、A/B 独立验证设计。

`IDENTIFICATION_CORE.zh-CN.md` 是从最早固件到当前阶段的长时间线；其中大量“下一步”和命令已经过期。
`CAN_TRACE_FIELD_STEPS_20260910.zh-CN.md` 与 `SMALL_CAN_TRACE_FIELD_STEPS_20260910.zh-CN.md` 是已完成
单轴采集的历史步骤，不能用于当前固件。

## 代码脉络

| 层 | 主要文件 | 责任 |
| --- | --- | --- |
| 调度器 | `module_yaw_identification.h` | 状态机、波形、超时、反馈/行程门禁 |
| 固件监督器 | `app_yaw_identification.c` | 所有权、可选 BENCH、配置冻结、停止与数据流 |
| 云台执行 | `module_gimbal.c`、`module_yaw_limits.h` | 双 yaw/Pitch 控制、软限位、输出清零、内部停止原因 |
| 系统互锁 | `app_control.c`、`module_shoot.c`、`app_autoaim.c` | 底盘零目标、发射机构清零、自动瞄准屏蔽 |
| 线协议 | `module_yaw_ident_wire.h`、`module_can_trace.*` | 构建号、trial、CRC、CAN 指令/反馈历史 |
| 主机采集 | `yaw_slow_cli.py`、`yaw_*_protocol.py` | 明确确认词、严格解码、唯一输出目录、不自动释放 |
| 离线辨识 | `identify_*_offline.py`、`validate_dual_reference_offline.py`、`model.py` | 电流时序、偏置、冻结模型独立验证和拒绝门槛 |

## 已实现的软件安全保护

### 1. 启动与所有权门禁

- `probe` 只查询，不取得控制权；ARM 请求必须带递增的非零 trial ID，旧 ID 不能重放。
- 固件拒绝与已有辨识或云台调参会话并发；辨识占用后，普通 PC 调参和参考写入被冻结。
- ARM 不再检查遥控 UP/新鲜度、操作输入中立、反馈新鲜度、小轴居中、6°/s 低速或预留行程；这些准备由
  操作员负责，STATUS 提示位仍保留供 probe 查看。
- 固件仍检查接管前 yaw 命令为零、配置和采集缓冲有效、请求格式正确，以及没有并发会话。
- 主机确认字符串是防误操作门槛，不是密码，也不能替代固件和现场检查。

### 2. 两步物理触发与可选 BENCH

- 运动命令先进入 ARMED；固件必须先观察到非 DOWN，再由操作者在 15 s 内拨到 DOWN 才开始运动。
- 等待期间经过 MIDDLE 被允许，但输出必须仍为零；超时、状态异常或输出非零都会终止。
- BENCH 持续 20 s，全程保持 UP，双 yaw 和 Pitch 输出关闭并采集 5001 点。它只用于采集链诊断和静态
  电流偏置测量，不再产生 10 min 资格，也不是运动 trial 的前置条件。

### 3. 运行中监督与立即停机

以下任一条件失效都会撤销参考，并把 yaw 与 Pitch `output_state` 置零：

- 右拨杆不再是 DOWN、出现 UP/MIDDLE、遥控数据超过 50 ms或操作者输入离开中立；
- 主机心跳超过 300 ms、控制/采样步长超过 10 ms、命令队列溢出；
- INS 年龄超过 10 ms，大/小 yaw 反馈年龄超过 20 ms，反馈离线、非有限值或编码器分支无效；
- 小轴越过软件行程，或相对起点超过大轴 25°、小轴 20°、IMU 航向 20°；
- 大轴速度超过 360°/s或小轴速度超过 180°/s；记录期间 roll 或 pitch 相对起点变化超过 5°；
- 配置的 32 个冻结值发生任何变化，或大轴/小轴控制链、Pitch 电机状态、PID 数值出现异常；
- 320 条 CAN 环形流接近满、主机累计确认超过 300 ms 无进展。

CAN 单区间的入队失败、发送错误、中止或覆盖不完整目前会锁存为数据质量故障，
主机不会把该 trial 当作可用辨识数据。现有代码没有把每一种此类 CAN 故障都直接接入上述立即停机路径；
这一策略需要在继续实车运动前由现场确认。

终止原因被保存为 reason 1～14；`0x59490405` 在 reason 12 时还保存大轴内部停止点，例如反馈联锁、
协调器、角度 PID、速度 PID或最终输出。

### 4. 机械行程和输出约束

- 小轴标定零点是原始计数 6816；硬限位约为 −59.678°～+32.080°，固定 3°余量后的软件限位约为
  −56.678°～+29.080°。
- 监督器再保留 5°安全边界，因此运行允许区间约为 −51.678°～+24.080°。ARM 不再要求起点居中或
  预留完整 ±20°；到达绝对或相对行程边界时仍会终止。
- 小轴在距离软件限位 8°内逐渐降低允许速度，边界制动参考为 24 rpm；到达软件边界时阻止继续向外的
  effort。
- 去线缆阶梯的 profile 预算为 R1 3°/2°、F4 12°/8°、F5 15°/10°；实际参考峰值低于这些整数预算。
  当前冻结软件输出上限要求大轴不超过30、小轴不超过6。这些是软件命令单位，不是已标定的电流或力矩。

### 5. 其他机构和终止状态

- 辨识占用期间，控制任务持续向底盘板发送零速度目标；自动瞄准不再写入云台目标。
- 发射机构模式和拨弹模式被置空，摩擦轮、拨弹电机 effort 清零并清除相关 PID。
- 任一保护终止后，云台输出关闭，Pitch 会失去保持力矩；Pitch 必须始终有不会妨碍 yaw 的机械支撑。
- DONE/ABORT 后固件继续持有辨识所有权，不自动恢复普通控制。RELEASE 只在终止相位、遥控新鲜且为 UP、
  yaw 输出为零时接受；主机也不会自动发送 RELEASE。

### 6. 主机和数据完整性

- 主机严格匹配固件 build、trace version、记录长度和 460800 baud，不自动降级到旧协议。
- 输出目录以 `exist_ok=False` 创建，防止覆盖已有证据；每条命令必须使用新目录。
- CRC、trial ID、连续序号、分片顺序、CAN 计数守恒、事件时间、指令覆盖、反馈年龄、限幅和终止零命令
  均被检查。结构、CRC、序号或波形语义错误会停止接收并尝试 CANCEL；CAN 发送/覆盖异常会把数据标为不合格。
- 下载完整、传输合格与模型有效是三个独立结论。主机当前始终输出 `model_status=not_identified` 和
  `hardware_takeover_allowed=false`。

## 软件保护的边界

- 右拨杆 UP 是 MCU 读取后执行的软件急停，不是独立硬接线断电回路。
- 底盘停止通过板间命令实现，不能替代机械固定；发射机构清零也不能替代退弹、断能和现场隔离。
- 软件限位依赖编码器安装零点与方向正确；更换电机、齿轮、编码器或机械装配后必须重新核对。
- 当前速度终止阈值 360°/s和 180°/s主要防止失控持续，并不保证任何机构/线缆在达到该速度前都安全。
- Pitch 在终止时主动失能，因此支撑本身必须可靠，且不能进入两级 yaw 的扫掠空间。

## 请你确认的现场项目

另一个需要明确接受的边界是：CAN 发送/覆盖质量故障当前主要用于判废数据，并非每种故障都会触发监督器立即关闭输出。
若现场要求“任一 CAN 故障立即终止”，需要先修改固件并重新烧录验证。

请按编号回复“确认”或给出需要修改的值：

1. 底盘有机械固定；即使板间零速度命令失效，也不会移动。
2. 弹丸已清空，摩擦轮/拨弹机构已物理断能或以等效方式隔离。
3. 有可直接切断云台动力的独立急停或电源开关，操作者无需经过 SSH/串口即可触达。
4. Pitch 支撑在失去保持力矩时仍可靠，并且不会妨碍大、小 yaw 全程运动。
5. 当前小轴编码器标定点仍是：零点 6816、右端 5458、左端 7546；正方向朝左。
6. 当前方向配置可接受：大轴 `+1`、小轴 `+1`。
7. 双参考预算 3°/2°和软件输出上限 30/6适合当前负载、电源和机械状态。
8. 是否接受运行速度终止阈值大轴 360°/s、小轴 180°/s；若不接受，请给出希望的更低阈值。
9. 操作员自行选择具备足够机械/线缆余量的起点，并保证大轴 ±25°、小轴 ±20°、航向 ±20°范围安全。
10. 已现场验证右拨杆 UP/DOWN 的固件映射和遥控断连停机行为。
11. CAN 发送/覆盖异常采用哪种策略：维持当前“锁存质量故障并判废 trial”，还是改为“任一异常立即终止运动”。
    对首次双参考运动，我建议选择立即终止。

`0x59490501/502/503` 的 C、D、E 证据均已归档；E 已在 UP、零输出状态下 RELEASE，随后 probe 为
phase 0、reason 0、`flags=252`、未占用且反馈有效。冻结 C 闭环候选已通过 E，但 C/D/E 事后残差
修正尚未获得新的独立验证。当前不再执行 C/D/E，也没有待释放的 trial。移除线缆并烧录
`0x59490601` 后，按新冻结步骤先采 R1 机械对照，再采 F4；是否执行 F5 由 F4 实测80%余量门决定。
