# MOSAIC-Broncho 中英对照审阅稿

> 对应论文：`MOSAIC-Broncho: Multimodal Observation and State-Aware Imitation Control for Goal-Directed Autonomous Bronchoscopy`
>
> 本文件用于逐段理解和修改。正式投稿源文件为 `main.tex`。
>
> `✓` 表示已有代码、checkpoint或数据证据；`⚠` 表示投稿前必须补实验或作者确认。

## 统一术语

| English | 中文 | 使用边界 |
|---|---|---|
| MOSAIC-Broncho | 多模态观测与状态感知模仿控制框架 | 全系统名称，不缩写为单独的MOSAIC |
| MonoAirway-Fusion (MAF) | 单目气道融合感知架构 | 统一表示结构、解剖和度量几何三个证据流 |
| MOSAIC-GRU | 目标条件化多模态循环策略 | 论文的控制核心 |
| behavior cloning | 行为克隆/模仿学习 | 当前真实训练范式，不写成强化学习 |
| anatomical goal token | 解剖目标标记 | 替代自由文本目标 |
| metric-geometric evidence | 度量几何证据 | 深度与左/中/右路径自由空间信息 |

具体实现使用VGG--U-Net、轻量YOLO-family detector和Depth Anything V2 prior。它们只在Implementation Details中披露，不作为三项独立贡献。

---

## Title / 标题

**English**

MOSAIC-Broncho: Multimodal Observation and State-Aware Imitation Control for Goal-Directed Autonomous Bronchoscopy

**中文**

MOSAIC-Broncho：面向目标导向自主支气管镜导航的多模态观测与状态感知模仿控制

---

## Abstract / 摘要

### English

A bronchial bifurcation often looks clear to a human but remains ambiguous to a robot. The image alone does not reveal the intended branch, recent motion, or tendon hysteresis. We therefore present MOSAIC-Broncho, a Multimodal Observation and State-Aware Imitation Control framework for goal-directed bronchoscopy. A unified monocular front end extracts structural, anatomical, and metric-geometric cues. These cues are synchronized with motor position, finite-difference velocity, and frame timing to form an auditable 20-D state. The controller conditions an eight-frame history on the desired anatomical target and predicts bounded increments for insertion and two bending axes. It is trained from 87 real-robot demonstrations with 30,730 synchronized frames across seven targets. The resulting 773,221-parameter GRU reaches a held-out action MSE of 0.002084 at epoch 29. The study evaluates each link in the loop: perception quality, temporal and multimodal ablations, runtime behavior, and target-wise navigation in a silicone airway phantom. The design favors a small, inspectable state and continuous real-time control over a large end-to-end policy. It provides a practical route toward autonomous bronchoscopy when data, compute, and safety margins are limited.

### 中文

支气管岔口对人而言通常很清楚，对机器人却可能充满歧义。单幅图像无法说明应该进入哪个分支，也无法反映近期运动和腱驱动迟滞。为此，本文提出MOSAIC-Broncho，即面向目标导向支气管镜导航的多模态观测与状态感知模仿控制框架。统一的单目感知前端提取结构、解剖和度量几何线索。这些线索与电机位置、有限差分速度和帧间时间同步，形成可审计的20维状态。控制器根据目标解剖位置，对8帧历史进行条件化，并预测进退轴和两个弯曲轴的受限增量。模型使用87条真实机器人示范训练，共包含30,730帧同步数据和7个目标。最终的773,221参数GRU在第29轮达到0.002084的留出集动作MSE。实验按照完整闭环组织：感知质量、时序与多模态消融、运行时行为，以及硅胶气道幻体中的逐目标导航。该设计选择小型、可检查的状态和连续实时控制，而不是大型端到端策略。对于数据、算力和安全裕度均有限的场景，它提供了一条务实的自主支气管镜路线。

### 审阅提示

- ✓ 数据量、参数量、epoch和MSE可从代码与checkpoint核验。
- ⚠ “实验评估完整闭环”只有在补齐红色TODO后才能保留为完成时态。
- ⚠ 摘要最终应增加实体导航的主要成功率和p95延迟，否则实验结论偏弱。

---

## I. Introduction / 引言

### 开篇hook

**English**

A bronchoscope reaches a fork. Both openings may look plausible. Yet only one leads to the target. The controller must remember the route, read the local geometry, and account for what the instrument has just done. A single image is not enough.

**中文**

支气管镜到达一个岔口。两个开口看起来都可能正确，但只有一个通向目标。控制器必须记住路径，理解局部几何，还要考虑器械刚刚执行过的动作。只看一幅图像并不够。

**编辑意图**：用一个具体控制瞬间把读者带入问题。五个短句依次建立场景、冲突和论文假设。

### 问题为什么困难

**English**

This compact example captures the central difficulty of autonomous bronchoscopy. Airway views are repetitive. Their appearance changes with reflection, mucus, deformation, and blur. The robot adds another source of ambiguity: tendon friction and hysteresis make the same command produce different motion histories. Although robotic platforms improve reach and stability, operation remains largely expert driven.

**中文**

这个简单场景概括了自主支气管镜的核心难点。气道图像高度重复。反光、黏液、形变和模糊会持续改变其外观。机器人还会引入另一种歧义：腱驱动摩擦与迟滞使同一条命令在不同运动历史下产生不同响应。机器人平台虽然提高了到达能力和稳定性，但操作仍主要依赖专家。

### 近期研究与研究空白

**English**

Recent work attacks different parts of this problem. Visual servoing keeps the lumen near the image center. Online topological localization estimates where the scope is without patient-specific CT. AI co-pilot learns shared control from a virtual expert and uses DAgger to reduce distribution shift. BronchoCopilot fuses images and estimated pose in a reinforcement-learning agent. More recently, long--short agents have combined a fast visual controller with slower strategic reasoning, reporting phantom, ex-vivo, and in-vivo evaluation. Together, these systems show rapid progress. They also leave a practical gap. Simulation offers scale but creates transfer risk. Purely reactive control loses route context. Large or hierarchical policies can be difficult to run and audit at motor-loop rate.

**中文**

近期工作分别处理了这一问题的不同部分。视觉伺服让管腔保持在图像中心附近。在线拓扑定位尝试在不依赖患者特异性CT的情况下判断镜体位置。AI co-pilot从虚拟专家学习共享控制，并用DAgger缓解分布偏移。BronchoCopilot将图像和估计位姿融合到强化学习智能体中。更近期的长短期智能体把快速视觉控制与较慢的战略推理结合，并完成幻体、离体和活体实验。这些研究说明该领域正在快速发展，但仍留下一个实际空白。仿真能够扩大数据规模，却带来迁移风险。纯反应式控制缺少路径上下文。大型或层级策略则可能难以在电机环频率下运行和审计。

### 本文主张

**English**

Our premise is simple. Do not ask one image to carry the whole navigation problem. Instead, expose the few signals that matter and let a recurrent policy reason over their recent history. MOSAIC-Broncho follows this premise. Its MonoAirway-Fusion front end turns a monocular frame into structural, anatomical, and metric-geometric evidence. The policy then combines these cues with the commanded goal and the measured motor state. It predicts continuous increments for insertion and two bending axes.

**中文**

本文的出发点很直接：不要让一幅图像承担整个导航问题。我们只显式提取真正重要的信号，再让循环策略理解这些信号的近期历史。MOSAIC-Broncho遵循这一思路。MonoAirway-Fusion前端将单目图像转换成结构、解剖和度量几何证据；策略再将这些线索与指定目标和测得的电机状态结合，输出进退轴及两个弯曲轴的连续增量。

### 创新边界

本文不声称GRU本身具有算法创新。贡献在于面向真实三轴支气管镜的控制问题定义：使用小型、归一化、可审计的状态，将单目气道感知与真实专家示范连接起来。行为克隆在专家分布外仍会受到协变量偏移影响，正文必须保留这一限制。

### Contributions / 贡献

1. 统一的单目感知接口，提供结构、解剖和度量几何三类互补证据。
2. 20维归一化状态，将感知证据与电机位置、速度和时间统一，并保证每个输入均可记录、检查和回放。
3. 773k参数的目标条件化GRU，从真实专家示范的8帧历史预测受限三轴控制命令。
4. 将组件质量、受控消融和硅胶幻体逐目标导航连接起来的系统评估设计。

---

## II. Results / 结果

### A. Unified Multimodal Perception / 统一多模态感知

MAF结构输出用于硅胶气道岔口分割。最终应在固定测试集上报告Dice、IoU、precision、recall和推理延迟。当前只有代表性清晰、低照、反光和模糊图像，不能替代定量结果。

MAF解剖输出包含9类，当前模型参数量为2.59M。已保存结果为precision 0.8926、recall 0.7797、mAP50 0.9096和mAP50--95 0.6006。precision与recall之间的差距支持连续帧确认机制，但仍需直接测量“确认后的解剖状态准确率”。

### B. Metric-Geometric Perception / 度量几何感知

在预定义镜头到表面的距离和多个气道位置测试MAF度量几何输出。最终报告raw和calibrated结果，包括N、MAE、RMSE、bias、R²和valid-frame rate，并按正常照明、反光、遮挡和运动模糊分层。

### C. Recurrent Policy Learning and Ablation / 循环策略学习与消融

数据集包含87条专家示范和30,730帧同步数据，覆盖7个目标。MOSAIC-GRU包含773,221个参数。第29轮的held-out action MSE为0.0020837828，辅助任务accuracy为1.000。该accuracy只表示navigation/intervention标签预测，不能解释为导航成功率。

必须比较：Frame-wise MLP、No goal、Action only、No depth/path、No velocity/Delta t和Full model。所有变体使用相同的轨迹级划分，并同时报告离线动作误差和实体route success。

### D. Real-Robot Navigation Success / 实体导航成功率

每次试验从固定起点开始。只有在无人工接管的情况下进入预定义目标分支，并保持规定确认时间，才计为成功。七个目标分别报告n/N和Wilson 95% CI；同时报告误分支率、完成时间、接触和人工干预。

**当前判断**：这一节是论文能否投稿的决定性证据。所有TODO必须由正式trial sheet填充，不能从视频印象估算。

---

## III. System and Multimodal Observation / 系统与多模态观测

### A. Robotic Platform / 机器人平台

平台使用三个位置控制电机：M0控制进退，M1控制左右弯曲，M2控制上下弯曲。软件使用线程安全特征字典分离相机/感知线程与电机控制线程。当前实现限位为M0∈[-900,0]°、M1∈[-170,170]°、M2∈[-600,600]°。

### B. MonoAirway-Fusion Perception / MAF感知

核心定义：

$$
(\mathbf s_t,\mathbf c_t,\mathbf d_t)=\Phi_{\mathrm{MAF}}(I_t).
$$

- $\mathbf s_t$：结构证据，包括阻塞/岔口质心、面积和有效标志。
- $\mathbf c_t$：解剖证据，包括气道类别和置信度。
- $\mathbf d_t$：度量几何证据，包括中心/平均/最大深度和左/中/右自由空间。

三类证据形成13维视觉几何接口，再与机器人状态组成20维观测：

| Group | Dim. | Contents |
|---|---:|---|
| Visual structure | 7 | 质心、面积、检测标志 |
| Depth/path geometry | 6 | 中心/平均/最大、左/中/右 |
| Motor position | 3 | M0、M1、M2角度 |
| Motor velocity | 3 | 有限差分角速度 |
| Timing | 1 | 帧间时间Delta t |
| Total | 20 | 归一化结构化状态 |

每个分量使用固定的物理或几何尺度$r_i$归一化：

$$
\tilde{o}_{t,i}=o_{t,i}/r_i.
$$

⚠ 最终Table I必须列出20个分量各自的名称、量纲、尺度和缺失值规则。

---

## IV. MOSAIC Goal-Conditioned Recurrent Policy / 目标条件化循环策略

### A. Architecture / 网络结构

输入为8帧×20维观测窗口。逐帧编码器为20→128→128，并使用LayerNorm和ELU。目标token经16维embedding后复制到每个时间步，与128维观测编码拼接成144维序列。随后使用两层GRU，hidden size为256，层间dropout为0.1。最终隐藏状态进入动作头256→128→3和辅助任务头256→64→2。

$$
\mathbf x_t=[f_{\mathrm{obs}}(\tilde{\mathbf o}_t);f_{\mathrm{goal}}(g)],\qquad
\mathbf h_t=\mathrm{GRU}(\mathbf x_t,\mathbf h_{t-1}).
$$

### B. Objective and Data Split / 损失与数据划分

$$
\mathcal L=\lVert\hat{\mathbf a}_t-\mathbf a_t^E\rVert_2^2+
\lambda_s\mathrm{CE}(\hat{\mathbf z}_t,s_t^E).
$$

训练使用AdamW、OneCycleLR、混合精度、梯度裁剪和batch size 64。数据按完整轨迹进行90/10划分，防止同一条示范的相邻窗口同时进入训练和验证。

### C. Safe Deployment / 安全部署

动作输出经每轴缩放和安全系数转换为绝对电机目标：

$$
\mathbf q_{t+1}^{\mathrm{cmd}}=\operatorname{clip}
(\mathbf q_t+\beta\boldsymbol\alpha\odot\hat{\mathbf a}_t,
\mathbf q_{\min},\mathbf q_{\max}),
$$

其中$\boldsymbol\alpha=[5,3,3]^\circ$，$\beta\in(0,1]$。目标变化或重新启动自主模式时，观测缓存和隐藏状态必须重置。外围控制器使用时间戳和连续帧确认，防止过期感知触发模式切换。

---

## Design Context / 同领域设计比较

| Method | Training | Online input | Memory | Validation |
|---|---|---|---|---|
| AI co-pilot | 仿真模仿+DAgger | RGB+人工方向命令 | on-policy聚合 | 幻体/活体 |
| BronchoCopilot | 仿真强化学习 | RGB+估计位姿 | latent state | 仿真 |
| Long--short agents | 层级学习 | RGB+CT目标 | 双智能体 | 幻体/离体/活体 |
| MOSAIC-Broncho | 真实示范行为克隆 | 20维状态+目标 | 8帧GRU | 硅胶幻体 |

该表只用于定位设计空间，不能进行跨机器人、跨任务的直接性能排名。

---

## V. Experimental Protocol / 实验方案

固定起点夹具上进行配对、随机试验。目标为LMB、RMB、LUB、LLB、RUL、RML和RLL。预先规定初始姿态、照明、镜头污染和黏液条件。主要结局为route success和wrong-branch rate。次要结局包括中心误差、接触次数/持续时间、完成时间、action jerk及端到端延迟。

二分类结局报告Wilson 95% CI；偏态连续数据报告median和IQR；配对检验进行多重比较校正。失败分类包括wrong branch、timeout、perception loss、contact stop和operator takeover。

---

## VI. Discussion / 讨论

MOSAIC-Broncho处于反应式视觉伺服和大型层级智能体之间。它保留显式几何与机器人状态，但学习这些状态的近期历史如何映射到动作。这样能够缩小模型规模，也更容易追踪故障来源。代价是语义范围有限：策略只能追求已配置目标，能力受专家轨迹覆盖范围限制。

与近期系统相比，AI co-pilot拥有更强的Sim2Real训练和活体证据；BronchoCopilot验证了更深层的仿真RL导航；长短期智能体解决了长时程规划，并提供离体与活体实验。本文当前优势不同：从真实机器人示范学习连续三轴控制，并使用紧凑、可检查的时序状态。这一点具有实用价值，但目前不足以声称SOTA。

当前限制包括：验证MSE不等于闭环恢复能力；10个配置目标中只有7个具有示范；类别可能不平衡；辅助任务100% accuracy可能来自标签简单；单目深度在液体、反光和形变下会漂移；实体成功率、接触和延迟分布尚待正式日志。

---

## VII. Conclusion / 结论

MOSAIC-Broncho的核心思想很直接：显式定义导航状态，再对时间进行推理。20维观测描述单目气道线索和机器人动态；8帧目标条件化GRU将历史转换为受限三轴命令。当前工件已经建立数据规模、模型结构和留出集预测误差。锁定的幻体协议定义了投稿前剩余的关键证据。如果实体实验能够确认稳定目标到达，这项工作将说明：实用的支气管镜自主导航并不总是需要大型端到端模型。

---

## 投稿前最高优先级清单

1. 补七个目标的实体成功率、Wilson 95% CI、误分支、时间和接触。
2. 完成Full、No goal、Frame MLP、No depth/path、No velocity/Delta t、Action only消融。
3. 补结构Dice/IoU、深度MAE/RMSE/bias和整体p50/p95延迟。
4. 将Table I扩展为逐个列出20维变量及归一化尺度。
5. 用真实逐epoch日志和真实实验照片替换所有示意或TODO内容。
6. 若只有一个硅胶幻体，将结论严格限定为single-phantom feasibility。

更完整的模拟审稿评分和SOTA比较见 `reviewer_report_zh.md`；五张核心图的制作规范见 `figure_plan_zh.md`。
