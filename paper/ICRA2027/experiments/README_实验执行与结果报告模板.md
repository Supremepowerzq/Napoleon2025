# MOSAIC-Broncho 实验执行与结果报告模板

版本：v1.0（2026-08-12）  
适用范围：单一硅胶支气管幻体上的可行性研究（single-phantom feasibility study）

## 1. 使用顺序

1. 实验前冻结本文件中的成功判据、超时阈值、停止条件、目标列表和模型checkpoint。
2. 先完成pilot试验。pilot只用于检查设备、记录字段和阈值是否可执行，不进入正式统计。
3. 为正式trial预生成随机顺序。不要依据模型表现临时调整目标顺序。
4. 每完成一个trial，立即填写`01_navigation_trials.csv`，保存原始视频、状态日志和命令日志。
5. 感知测试分别填写`03_structure_perception.csv`、`04_anatomy_perception.csv`和`05_depth_calibration.csv`。
6. 延迟数据逐次记录到`06_latency_events.csv`，不要只记录平均值。
7. 消融实验使用相同的测试条件与随机顺序，并填写`02_ablation_runs.csv`。
8. 在排除任何trial前填写排除原因。设备故障可以按预定义规则排除；模型失败不能排除。
9. 所有原始表冻结后，再生成论文汇总表。

## 2. 正式实验前必须锁定的定义

将以下内容填写完整并写入论文或补充材料：

| 项目 | 冻结值 | 推荐定义 |
|---|---|---|
| 固定起点 | TODO | 使用机械夹具，记录三轴初始角度与镜头插入深度 |
| 目标列表 | LMB/RMB/LUB/LLB/RUL/RML/RLL | 七个已采集目标 |
| 最大trial时间 | TODO秒 | pilot后设定；建议同一路径专家中位时间的2–3倍，之后不改 |
| 目标确认时间 | TODO秒 | 建议连续2–3秒保持在目标分支 |
| 成功 | 固定 | 无人工接管进入预定义目标分支，并满足确认时间 |
| wrong branch | 固定 | 镜头越过错误分支判定平面，且无法在安全区内自行恢复 |
| contact | 固定 | 由视频标注、力阈值或二者定义；全文使用同一规则 |
| perception loss | 固定 | 关键MAF输出无效持续超过预定义时间，例如1秒 |
| operator takeover | 固定 | 操作者发出任何导航命令或接管电机控制 |
| emergency stop | 固定 | 急停、软件安全停止或硬件限位触发 |
| checkpoint | SHA/文件名 | 正式实验期间禁止继续训练或换模型 |
| 软件版本 | Git commit | 记录ZQ分支commit hash |
| 硬件 | TODO | CPU、GPU、RAM、相机分辨率/FPS、电机控制器 |

## 3. 实体目标导航实验

### 3.1 样本量与随机化

- 最低：每个目标10次，共70次。
- 推荐：每个目标15次，共105次。
- 更稳健：每个目标20次，共140次。
- 将目标按区组随机化。每个区组包含七个目标各一次，减少设备升温、镜头污染和操作者时间造成的顺序偏差。
- 若比较多个模型，使用同一组初始条件。模型顺序也应随机或采用平衡交叉顺序。
- 每个trial都从固定起点重新开始，并保存起始照片或初始状态快照。

### 3.2 每个trial的操作步骤

1. 核对模型、目标token、Git commit和硬件配置。
2. 将机器人恢复到固定起点，记录M0/M1/M2初始角度。
3. 检查镜头清洁度、照明条件和幻体状态。
4. 同时开始视频、20维状态、MAF输出、电机反馈、动作命令和时间戳记录。
5. 启动自主导航。实验过程中不得给予方向提示。
6. 达到成功、超时、错误分支、安全停止或人工接管中的任一终点后停止。
7. 立即填写结果；失败可多标签记录，但必须指定一个primary failure mode。
8. 保存文件名，确保能由trial_id一一定位原始证据。

### 3.3 必须报告的统计量

- 每目标成功率：$\hat p=n/N$。
- 95% Wilson置信区间：

$$
\frac{\hat p+z^2/(2N)\pm z\sqrt{\hat p(1-\hat p)/N+z^2/(4N^2)}}{1+z^2/N},\quad z=1.96.
$$

- pooled success只能作为总体描述；必须同时显示每目标结果，防止容易目标掩盖困难目标。
- 完成时间：成功trial报告median [IQR]；失败trial的时间不能混入完成时间，可另报time-to-termination。
- wrong-branch、takeover、emergency-stop和perception-loss均按trial比例报告。
- 接触报告每trial次数和总持续时间。若无法可靠测量持续时间，至少报告次数并说明标注规则。

### 3.4 结果范围：用于规划，不是删选标准

| 指标 | 最低可解释 | 有说服力的ICRA结果 | 强结果 | 说明 |
|---|---:|---:|---:|---|
| Pooled success | ≥70% | ≥80% | ≥90% | 单幻体、已见目标条件下；必须看95% CI和逐目标最低值 |
| 每目标success | ≥60% | ≥75% | ≥85% | 任一目标明显偏低都需要failure analysis |
| Wrong-branch rate | ≤20% | ≤10% | ≤5% | 目标条件化策略的关键指标 |
| Operator takeover | ≤20% | ≤10% | ≤5% | 接管本身计为失败，不能只报“最终到达” |
| Emergency stop | <10% | <5% | 0–2% | 非零时必须解释触发原因 |
| Completion time | 不超过专家3倍 | ≤专家1.5倍 | 接近专家 | 应使用相同起点/目标；准确性优先于速度 |
| Contact count | 低于frame-MLP | 明显低于主要基线 | 接近专家 | 无统一跨平台绝对阈值，优先使用配对比较 |

说明：AI co-pilot在不同任务和系统上报告过约93.3%的仿真路径成功率，并具有幻体/活体验证；BronchoCopilot在仿真第五代气道报告约90%；这些数字不能作为本系统的合格线，只说明强论文通常需要高成功率、置信区间和多层级验证。

## 4. 核心消融实验

### 4.1 模型冻结与公平比较

每个模型必须使用：

- 相同训练/验证轨迹划分；
- 相同输入窗口采样和训练epoch预算；
- 相同随机种子集合，建议至少3个seed；
- 相同动作缩放、安全系数和硬件限位；
- 相同实体目标、初始条件和trial随机顺序；
- 不依据测试集选择checkpoint。

模型列表：Full MOSAIC-GRU、Frame-wise MLP、No goal、No depth/path、No motor velocity/Delta t、Action-only。

### 4.2 离线指标

- overall action MSE；
- M0/M1/M2分别的MAE与RMSE；
- 方向一致率：预测与专家动作同号的比例，零动作单独定义容差；
- auxiliary task macro-F1与混淆矩阵；
- 每个seed的结果，而不仅是均值；
- 模型参数量、p50/p95推理延迟。

### 4.3 闭环指标

每个变体至少完成与Full相同的目标集合。资源有限时，可先做每目标5次筛查，再对Full、Frame-wise MLP、No goal和No depth/path完成每目标10次正式比较。筛查结果必须标为pilot，不能与正式trial混用。

### 4.4 有意义的效应范围

| 比较 | 最低有意义差异 | 较有说服力 | 注意事项 |
|---|---:|---:|---|
| Full vs Frame-wise MLP | success +5个百分点 | +10–15个百分点 | 用于证明时序记忆；同时看contact和wrong branch |
| Full vs No goal | +5个百分点 | +10个百分点以上 | 分叉选择应最敏感；若无差异，目标token可能未被使用 |
| Full vs No depth/path | +3个百分点 | +5–10个百分点 | 也可表现为更少接触或更快完成，而非只看成功率 |
| Full vs No velocity/Delta t | action MSE降低≥5% | 降低≥10%或闭环更稳 | 若差异很小，应弱化动态状态贡献 |
| Dual-head vs Action-only | MSE降低≥3% | ≥5–10%或成功率提升 | auxiliary accuracy=100%本身不构成价值 |

这些是效应大小目标，不是统计显著性的替代。小样本二项成功率的CI会很宽，应同时报告绝对差异、CI和配对条件。

## 5. MAF结构流实验

### 5.1 数据划分

- 按视频/轨迹划分，不能随机拆帧。
- 同一连续视频的相邻帧只能属于一个split。
- 固定test set，并将normal、low-light、specular、occlusion、motion-blur标签作为独立列。
- 每种困难条件建议至少100个独立标注帧；如果帧来自同一视频，要同时报告视频数。

### 5.2 指标定义

$$\mathrm{Dice}=\frac{2TP}{2TP+FP+FN},\qquad
\mathrm{IoU}=\frac{TP}{TP+FP+FN}.$$

同时报告pixel precision、recall、valid-frame rate和p95 latency。若控制真正使用的是岔口质心，还应报告归一化质心误差：

$$e_c=\frac{\lVert\hat{\mathbf p}-\mathbf p\rVert_2}{\sqrt{W^2+H^2}}.$$

### 5.3 规划范围

| 指标 | 最低可用 | 有说服力 | 强结果 |
|---|---:|---:|---:|
| Dice | ≥0.75 | ≥0.85 | ≥0.90 |
| IoU | ≥0.60 | ≥0.75 | ≥0.82 |
| Recall | ≥0.80 | ≥0.88 | ≥0.93 |
| Normalized centroid error | ≤0.08 | ≤0.05 | ≤0.03 |
| Valid-frame rate | ≥90% | ≥95% | ≥98% |

困难条件下降幅也要报告。例如overall Dice很高但specular Dice下降超过0.15，仍需要作为主要限制讨论。

## 6. MAF解剖流实验

- 保留现有precision、recall、mAP50、mAP50–95。
- 增加逐类别AP、混淆矩阵、每类support和macro-F1。
- 独立评估单帧类别输出与连续帧确认后的state accuracy。
- 连续帧确认还应报告确认延迟、错误状态切换次数/分钟和unknown rate。

### 规划范围

| 指标 | 最低可用 | 有说服力 | 强结果 | 当前证据 |
|---|---:|---:|---:|---:|
| Precision | ≥0.80 | ≥0.88 | ≥0.92 | 0.8926 |
| Recall | ≥0.75 | ≥0.82 | ≥0.88 | 0.7797 |
| mAP50 | ≥0.80 | ≥0.88 | ≥0.93 | 0.9096 |
| mAP50–95 | ≥0.50 | ≥0.60 | ≥0.70 | 0.6006 |
| Confirmed-state accuracy | ≥0.85 | ≥0.92 | ≥0.96 | TODO |
| False transitions | ≤1/min | ≤0.3/min | 接近0 | TODO |

当前解剖检测的precision和mAP50较好，recall是更明显的改进点。论文应解释连续帧确认是否真正提升状态稳定性，而不是仅从precision-recall差距推断。

## 7. MAF深度/路径实验

### 7.1 采集设计

- 使用可追溯标尺、固定滑台或可靠距离传感器获得参考距离。
- 覆盖导航工作范围，不要只测几个整齐距离。推荐至少8–10个距离水平，每级≥20帧。
- 在多个气道位置和五种视觉条件下重复：normal、low-light、specular、occlusion、motion-blur。
- 校准集与测试集按位置或采集序列分开；禁止在同一批帧上拟合线性校准并报告误差。
- 保存raw depth、calibrated depth、ground truth、valid flag和场景条件。

### 7.2 指标

$$\mathrm{MAE}=\frac{1}{N}\sum_i|\hat d_i-d_i|,\quad
\mathrm{RMSE}=\sqrt{\frac{1}{N}\sum_i(\hat d_i-d_i)^2},$$

$$\mathrm{bias}=\frac{1}{N}\sum_i(\hat d_i-d_i).$$

同时报告$R^2$、valid rate和绝对相对误差$|\hat d-d|/d$。路径选择还应报告left/center/right最大自由空间方向的分类准确率。

### 7.3 规划范围

绝对毫米误差高度依赖工作距离，因此优先使用相对误差：

| 指标 | 最低可用 | 有说服力 | 强结果 |
|---|---:|---:|---:|
| MAE / working distance | ≤15% | ≤10% | ≤5% |
| RMSE / working distance | ≤20% | ≤15% | ≤8% |
| |bias| / working distance | ≤10% | ≤5% | ≤3% |
| $R^2$ | ≥0.70 | ≥0.85 | ≥0.92 |
| Valid rate | ≥90% | ≥95% | ≥98% |
| L/C/R direction accuracy | ≥75% | ≥85% | ≥92% |

如果系统只使用相对深度来选路径，不应把结果称为临床级metric depth。应写“phantom-calibrated metric-geometric cue”。

## 8. 实时性与安全实验

### 8.1 延迟记录

为每帧记录以下时间戳，全部使用单调时钟：camera_start/end、structure_start/end、anatomy_start/end、depth_start/end、fusion_start/end、GRU_start/end、command_send、motor_ack。

- 每种条件至少记录1000个连续控制周期。
- 报告p50、p90、p95、p99、mean、SD和最大值。
- 端到端延迟定义为camera frame available到command sent；另报motor acknowledgment延迟。
- 报告deadline miss rate，例如超过50 ms或控制周期预算的比例。

### 8.2 规划范围

| 指标 | 最低可用 | 有说服力 | 强结果 |
|---|---:|---:|---:|
| End-to-end p50 | <50 ms | <33 ms | <20 ms |
| End-to-end p95 | <100 ms | <50 ms | <33 ms |
| GRU-only p95 | <10 ms | <5 ms | <2 ms |
| Deadline miss rate | <5% | <1% | <0.1% |
| Sustained control rate | ≥10 Hz | ≥20 Hz | ≥30 Hz |

这些范围只适用于实时导航规划。若三个感知流并非每帧都运行，应报告各自更新率和状态年龄，不能只报GRU速度。

### 8.3 安全测试用例

每个用例至少重复10次，并记录系统是否进入预期安全状态以及反应时间：

1. 感知数据故意冻结或延迟。
2. 导航过程中切换目标token。
3. 自主模式停止并重新启动，验证隐藏状态清空。
4. 命令接近每个电机限位。
5. 人工急停。
6. 相机断流或关键字段缺失。

目标：所有测试100%进入预期安全状态。任何未触发都必须作为严重故障分析，不能只报告平均通过率。

## 9. 数据划分与泛化

### 9.1 数据审计

- 汇总七个目标的trajectory count、frame count、duration和动作分布。
- 检查同一trajectory_id是否跨越train/validation/test。
- 训练集拟合的尺度或校准参数不能从测试集重新估计。
- 保存split manifest，包含文件名和哈希。

### 9.2 推荐泛化层级

| 层级 | 内容 | 论文可用表述 |
|---|---|---|
| G0 | 同一幻体、已见起点和目标 | single-phantom feasibility |
| G1 | 未见初始姿态/插入深度 | pose robustness |
| G2 | 未见光照、反光或镜头污染 | visual robustness |
| G3 | 第二个硅胶幻体或不同气道模型 | cross-phantom generalization |
| G4 | 离体肺 | preclinical ex-vivo feasibility |
| G5 | 活体 | in-vivo feasibility，需完整伦理审批 |

当前至少完成G0，并尽可能增加G1和G2。没有第二幻体时，不得使用“generalizes across anatomies”。

## 10. 论文结果段落填空模板

### 实体导航

> Across [N] predefined trials, MOSAIC-Broncho reached the assigned branch in [n] trials ([p]%, Wilson 95% CI [L, U]). Target-wise success ranged from [min]% to [max]%. Wrong-branch events occurred in [n/N] trials, while operator takeover was required in [n/N]. Among successful trials, median completion time was [x] s [IQR: a–b].

### 消融

> Removing temporal memory reduced route success from [x]% to [y]% and increased wrong-branch rate from [a]% to [b]%. Removing goal conditioning had its largest effect at bifurcations, where success decreased by [d] percentage points. Depth/path features primarily affected [success/contact/completion time], while motor velocity and frame timing affected [action error/jerk/stability].

### 感知

> On a trajectory-held-out test set of [N] frames from [V] videos, the structural stream achieved Dice [x] and IoU [y]. Performance decreased to [z] under [condition]. The anatomical stream achieved mAP50 [x] and mAP50–95 [y]; temporal confirmation changed state accuracy from [a] to [b] with a median confirmation delay of [t] ms. After calibration on separate sequences, the metric-geometric stream achieved MAE [x] mm and RMSE [y] mm on the locked test set.

### 延迟

> On [hardware], end-to-end latency was [p50] ms at p50 and [p95] ms at p95 over [N] control cycles. The sustained control rate was [f] Hz, with [r]% deadline misses. The recurrent policy itself required [x] ms at p95; the dominant cost was [module].

## 11. 数据完整性规则

- 不删除失败trial，不将接管后的到达计为成功。
- 不用视频帧数作为独立样本计算导航成功率。
- 不在查看测试结果后更改成功或超时定义。
- 不把pilot、调参trial和正式trial混合。
- 不只挑最好seed或最好目标；报告全部预定义模型与目标。
- 对传感器/设备故障的排除必须记录，并同时给出包含与不包含该trial的敏感性分析。
- 所有图表应能追溯到trial_id或frame_id。

## 12. 文件结构建议

```text
experiments/
  protocol_locked.pdf
  split_manifest.csv
  raw/
    navigation/
    perception/
    latency/
  annotations/
  tables/
    01_navigation_trials.csv
    02_ablation_runs.csv
    03_structure_perception.csv
    04_anatomy_perception.csv
    05_depth_calibration.csv
    06_latency_events.csv
    07_safety_tests.csv
    08_dataset_audit.csv
  analysis/
  figures/
```

## 13. 参考坐标系

- AI co-pilot bronchoscope robot: https://www.nature.com/articles/s41467-023-44385-7
- BronchoCopilot: https://arxiv.org/abs/2403.01483
- Long-Short Term Agents: https://arxiv.org/abs/2603.07909
- Online Topological Localization: https://arxiv.org/abs/2510.09144
- LungDepth: https://doi.org/10.1002/rcs.70050

不同论文的机器人、任务、气道深度和成功定义不一致。上述文献用于确定应报告哪些证据，不用于设定必须达到的成功率。
