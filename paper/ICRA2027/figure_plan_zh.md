# ICRA 2027 五张核心图规划（MOSAIC-Broncho）

## 统一视觉语言

- 方法总名：**MOSAIC-Broncho**（Multimodal Observation and State-Aware Imitation Control for Bronchoscopy）。
- 单目感知架构：**MonoAirway-Fusion（MAF）**。
- 控制策略：**MOSAIC-GRU**，即目标条件化多模态循环策略。
- 训练范式：当前证据应写 **深度模仿学习 / behavior cloning**，不要写“深度强化学习”。只有完成有效的在线奖励优化和对照实验后，才能增加强化学习表述。
- 配色建议：RGB/结构特征用青色，解剖语义用紫色，深度与路径用橙色，本体状态用绿色，目标条件用玫红色，动作输出用深蓝色。五张图保持相同含义。
- 图形用 PDF/SVG 矢量绘制；真实图像只作为面板内嵌照片。字体统一 Helvetica/Arial，最终双栏下最小字号不低于 7 pt。

## Fig. 1 — MOSAIC-Broncho 总览图（跨双栏大图）

**要回答的问题：论文究竟做了什么？**

建议采用“离线专家学习 + 在线闭环导航”上下双通路：

1. 左侧为支气管镜 RGB 图、目标解剖节点和专家三轴操作。
2. 中左为 MAF：从单目图像同时形成结构、解剖、度量几何三类证据。
3. 中部为 20 维状态：13 维视觉/深度路径特征 + 3 维电机角度 + 3 维速度 + 帧间时间。
4. 中右为 8 帧历史和目标 token 输入 MOSAIC-GRU，输出三轴连续增量。
5. 右侧为机器人在硅胶气道树中到达目标支气管，画出实时反馈箭头。
6. 下方虚线表示离线的 87 条专家示范、30,730 帧和行为克隆损失；上方实线表示在线推理闭环。

图中只突出 MAF、20-D state、MOSAIC-GRU 三个贡献块，不出现 U-Net、YOLO、Depth Anything 商标。图注中说明“learned from expert demonstrations”。

## Fig. 2 — MonoAirway-Fusion 单目多模态感知架构

**要回答的问题：一幅单目支气管镜图像如何变成可控、可审计的多模态状态？**

从左到右绘制：圆形 ROI 与光照归一化 → 三个互补感知流 → 同步与有效性检查 → 13 维视觉几何特征。

- 结构流：岔口/阻塞区域、质心、面积、有效标志；展示原图叠加轮廓。
- 解剖流：候选解剖节点、置信度、连续帧确认；展示气道拓扑小图。
- 度量几何流：单目深度、中心/均值/最大深度、左/中/右可通行深度；展示伪彩深度图及三条扇区。
- 融合端明确写：时间戳对齐、缺失值屏蔽、固定物理/几何尺度归一化。

核心公式：

$$
(\mathbf{s}_t,\mathbf{c}_t,\mathbf{d}_t)=\Phi_{\mathrm{MAF}}(I_t),\quad
\mathbf v_t=[\mathbf s_t;\mathbf c_t;\mathbf d_t]\in\mathbb R^{13}.
$$

具体骨干网络只放在图注最后一句或 Methods 的 Implementation Details：当前实现分别采用 VGG–U-Net、轻量 YOLO-family detector 和 Depth Anything V2 prior。这样既透明可复现，也不把它们包装成三项独立贡献。

## Fig. 3 — MOSAIC-GRU 目标条件化多模态循环策略

**要回答的问题：模型如何利用目标、时间历史与机器人动态生成实时控制？**

画出精确维度：`8×20 observation window → per-frame encoder 20–128–128`；10 类目标 token 经 16 维 embedding 后复制到 8 帧；拼接为 `8×144`；进入 2 层、hidden size 256 的 GRU；最后状态分为动作头 `256–128–3 + tanh` 与辅助任务头 `256–64–2`。动作再经 `[5°,3°,3°]` 轴向缩放、安全系数、关节限幅后发送给 M0/M1/M2。

核心公式：

$$
\mathbf x_t=[f_o(\tilde{\mathbf o}_t);e_g],\quad
\mathbf h_t=\mathrm{GRU}(\mathbf x_t,\mathbf h_{t-1}),\quad
\hat{\mathbf a}_t=\tanh f_a(\mathbf h_t),
$$

$$
\mathcal L=\lVert\hat{\mathbf a}_t-\mathbf a_t^E\rVert_2^2+
\lambda_s\operatorname{CE}(\hat{\mathbf z}_t,s_t^E).
$$

角落加一个“state reset / stale perception rejection”小框，体现实时部署而不仅是离线网络结构。

## Fig. 4 — 综合实验结果图（跨双栏四面板）

**要回答的问题：感知是否可靠、策略是否学会、各模态是否必要、机器人是否真正到达？**

- (a) MAF 定性输出：正常、低照、反光、模糊各一列，叠加结构边界、解剖标签、深度路径。
- (b) 度量几何定量：真实距离 vs 校准预测散点图（带 $y=x$）和误差分布；报告 N、MAE、RMSE、bias、$R^2$、valid rate。
- (c) 策略学习与消融：真实训练/验证 MSE 曲线；并列柱状图比较 Full、No goal、Frame MLP、No depth/path、No velocity/$\Delta t$、Action only。不要用插值曲线冒充逐 epoch 数据。
- (d) 实体导航：七个目标的成功率点估计 + Wilson 95% CI，并辅以 pooled success、wrong-branch rate 和 completion time。

只有从日志或固定测试集恢复的数据才能进入最终图；缺失项保留 TODO，不绘制推测数字。

## Fig. 5 — 实验平台与真实导航协议

**要回答的问题：实验怎样搭建，成功如何判定，别人怎样复现？**

主面板使用高分辨率真实照片，标注机器人支气管镜、硅胶气道幻体、相机/光源、M0 进退电机、M1 左右弯曲、M2 上下弯曲、固定起始夹具、控制工作站和安全停止装置。右上嵌入支气管镜视野，右下嵌入气道树及七个目标位置。

底部增加简洁试验时间线：固定起点 → 指定目标 → 自主运行 → 进入目标分支 → 保持确认时间。图注明确定义成功：无人工接管进入预设目标分支，并保持规定确认时间；同时说明超时、误入分支、感知丢失、接触停止和人工接管的失败分类。

## 对应数据表

1. **Table I — Observation definition**：20 个分量、符号、量纲、归一化尺度、来源和更新率。当前表只到分组层级，最终应扩展到逐分量。
2. **Table II — MAF output quality**：结构流 Dice/IoU/precision/recall，解剖流 precision/recall/mAP50/mAP50–95，以及每个流和整体 p50/p95 latency。
3. **Table III — Metric-geometric calibration**：raw/calibrated 的 N、MAE、RMSE、bias、$R^2$、valid rate，并按正常光照、反光、遮挡和模糊分层。
4. **Table IV — Policy ablation**：action MSE、三个轴分别的 MAE、macro-F1、参数量、p95 latency、实体 route success。
5. **Table V — Physical navigation**：每个目标的 n/N、Wilson 95% CI、wrong branch、完成时间、接触次数/持续时间、干预次数。

## 结果叙事顺序

正文 Results 建议固定为四节：`Unified Multimodal Perception` → `Recurrent Policy Learning` → `Multimodal and Temporal Ablation` → `Embodied Navigation`。先证明输入状态可信，再证明时序策略可学习，然后解释性能来自哪些信息，最后以真实机器人成功率闭环。网络品牌不再作为结果小节标题。
