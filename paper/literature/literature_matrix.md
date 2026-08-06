# 相关论文整理与研究空白

更新日期：2026-07-13

本表只收录本次能够在出版社、IEEE、会议或 arXiv 页面核验的条目。正式投稿前仍需用 IEEE Xplore/Crossref 再检查卷期页码。

## A. 与本文最接近的自主支气管镜工作

| 工作 | 方法与验证 | 对本项目的直接启示 | 本项目可形成的差异 |
|---|---|---|---|
| [Zhang et al., AI co-pilot bronchoscope robot, Nature Communications, 2024](https://www.nature.com/articles/s41467-023-44385-7) | CT 建立虚拟环境；人工专家代理生成监督；图像+五类医生方向命令预测转向；Sim2Real 和域随机化；核心是 AI-human shared control | 是必须重点讨论的最强近邻之一。它证明“高层离散路径意图 + 低层安全转向”可在支气管镜中工作 | 本项目不是模拟专家训练，而是实际电机轨迹作为参考；若 Layer 2 做成有界残差，可强调可审计和受约束；若 Layer 3 跑通，可强调治疗任务插入与恢复 |
| [Zhao et al., BronchoCopilot, IROS 2024 / arXiv:2403.01483](https://arxiv.org/abs/2403.01483) | 图像序列+估计机器人位姿的多模态 RL；辅助重建与注意力表征；仿真中第五代气道约 90% 成功率 | 可作为“端到端多模态 RL”基线与讨论对象 | 本项目当前不应声称 RL；优势应放在真实机器人参考轨迹、确定性降级和任务恢复，而非与其比仿真深度 |
| [Wu et al., Long-Short Term Agents for Pure-Vision Bronchoscopy Robotic Autonomy, arXiv:2603.07909, 2026](https://arxiv.org/abs/2603.07909) | 短时反应智能体+长时战略智能体；分歧时用 world-model critic 预测未来视觉；仿体、离体猪肺和活体猪验证；arXiv 摘要报告离体第八代 80% | 说明分层决策比单一反应控制更适合长程分叉导航 | 本项目的层次不是长/短期学习智能体，而是参考轨迹、有限视觉残差和中断恢复；注意该文目前为预印本，不能把摘要数字当作完全等价基线 |
| [Jessen et al., BronchoBot, ERJ Open Research, 2026](https://publications.ersnet.org/content/erjor/12/2/00970-2025) | 单次性支气管镜+图像导航；仿体中覆盖 18 个节段；报告 10 次完整检查和指定路径均完成，并与不同经验医生比较 | 给出了很强的仿体实验范式：覆盖率、程序时间、每节段时间、路径进展和专家评分 | 本项目应至少报告分支覆盖、目标路径成功、时间和人工接管，而不能只报告电机角度到位 |
| [Tomasini et al., Online Topological Localization for Navigation Assistance in Bronchoscopy, arXiv:2510.09144, 2025](https://arxiv.org/abs/2510.09144) | 不依赖患者 CT，以通用气道拓扑和图像做在线拓扑定位；用仿体训练并测试真实数据 | 适合后续补“机器人知道自己在哪一层/哪一分支”的离散定位 | 当前 Layer 1 是按目标文件选择路线，不具备在线拓扑定位；论文中必须如实说明这一限制 |

## B. Layer 2：内镜视觉伺服与几何感知

| 工作 | 方法与验证 | 对 Layer 2 的具体借鉴 |
|---|---|---|
| [Lazo et al., Autonomous Intraluminal Navigation of a Soft Robot using Deep-Learning-based Visual Servoing, arXiv:2207.00401, 2022](https://arxiv.org/abs/2207.00401) | 时序 CNN 分割管腔；图像中心误差驱动无模型视觉伺服；推进只在中心误差低于阈值时允许；用 EM 跟踪评价误差、平滑度和完成时间 | 直接支持 Layer 2 的死区/推进门控思想。其评价指标比“岔口修正是否动了”更完整：稳态误差、上升/稳定时间、超调、完成时间、路径误差 |
| [Li et al., Colon Lumen Center Detection Enables Autonomous Navigation of an Electromagnetically Actuated Soft-Tethered Colonoscope, IEEE TIM, 2024](https://doi.org/10.1109/TIM.2024.3403172) | 深度学习轮廓与暗区启发式结合，输出管腔中心并用于自主结肠导航 | 可用于论证单一分割在反光/遮挡下不够稳定，几何与启发式融合有价值；器官不同，不能直接比较成功率 |
| [Kasaei et al., Geometry-Aware Visual Odometry for Bronchoscopic Navigation via High-Gain Observer Fusion, arXiv:2607.05162, 2026](https://arxiv.org/abs/2607.05162) | 利用管腔消失点和 looming 速度增强支气管视觉里程计，并在离体人肺上与跟踪真值比较 | 是 2026-07 的新预印本，适合作为后续从“岔口中心”升级到几何运动估计的路线；当前初稿只列展望，不宜作为已实现组件 |
| [Xu et al., LungDepth, International Journal of Medical Robotics and Computer Assisted Surgery, 2025](https://onlinelibrary.wiley.com/doi/full/10.1002/rcs.70050) | 支气管镜多帧自监督单目深度；在自建数据和物理模型上验证 | 若 Depth-Anything-V2 未做支气管域适配，就不能直接把自然场景 metric head 称为毫米级真值；需要仿体标尺或跟踪系统校准 |
| [Yang et al., Depth Anything V2, NeurIPS 2024](https://arxiv.org/abs/2406.09414) | 大规模合成数据与伪标签训练的通用单目深度基础模型 | 可作为当前深度骨干引用，但不是支气管内镜精度证明；论文必须单独报告本域误差与时序稳定性 |

## C. 控制结构与模仿学习背景

| 工作 | 用途 | 写作注意 |
|---|---|---|
| [Johannink et al., Residual Reinforcement Learning for Robot Control, ICRA 2019 / arXiv:1812.03201](https://arxiv.org/abs/1812.03201) | 经典“基础控制器 + 学习残差”分解思想，可支撑参考轨迹叠加残差的概念动机 | 当前 Layer 2 是确定性视觉残差，不是 RL；只能引用其分解思想，不能称所提方法为 residual RL |
| [Ross et al., DAgger, AISTATS 2011](https://proceedings.mlr.press/v15/ross11a.html) | 解释早期 BC 在闭环中发生协变量偏移，以及未来为何需要在线纠错数据 | DAgger 是后续方案，不应继续作为当前主方法贡献 |
| [Ronneberger et al., U-Net, MICCAI 2015](https://lmb.informatik.uni-freiburg.de/Publications/2015/RFB15a/) | 当前语义分割骨干的基础引用 | 还需给出本项目数据量、划分方式、Dice/IoU 和按场景测试结果 |

## D. 本项目当前最可信的研究空白

现有工作大致分为三类：

1. **CT/仿真驱动的共享控制或 RL**：能学习复杂策略，但依赖仿真、预术 CT 或位姿信息，且真实系统验证成本高。
2. **纯视觉反应导航**：能保持管腔居中，但在长程分叉选择、错误感知累积和任务中断方面能力有限。
3. **长程分层智能体**：能处理战略和反应控制，但模型和训练复杂，临床审计及故障降级仍是关键问题。

本项目更合适的定位是：

> 先以真实专家操作数据训练目标条件化行为克隆策略，学习跨路径的多目标导航；再以同路径平均轨迹或选定单条轨迹提供可复现的长程参考，用带可信门控和总偏置上限的视觉残差吸收岔口局部偏差；最后通过独立 M3 电机控制吸引装置，并以双重断点状态机支持黏液清除后的安全返回与续航。

该定位成立的前提是：BC+goal 的稳定导航结果需按统一协议定量复现；Layer 1 需区分平均轨迹与单条轨迹；Layer 2 必须证明“修正有限且优于纯参考”；Layer 3 必须证明“M3 真的触发吸引且恢复后仍能完成原路径”。

## E. 旧引用库中需要删除或重新核验的条目

- `liu2023bronchoscopy`：本次按题名和给定卷页未检索到可靠对应，不能继续使用。
- `wu2023depth`：给定题名与作者未检索到可靠对应；建议替换为 LungDepth、MonoLoT 或其他可核验文献。
- `swaney2017toward`：BibTeX 键、题名和年份不一致，且论文内容为可转向针，不是支气管镜导航核心近邻。
- `gu2019transoral`：经口机器人甲状腺手术与本文主线关联弱，可删除以节省 RA-L 页数。
- `pore2024hi`：作者字段为 `Authors`、条目类型和期刊字段冲突，必须重做。
- `target2025`：把厂商/临床研究同时用于 Monarch 和 Ion 两个平台是不严谨的，需分别引用各自临床论文或官方监管材料。
- `longshort2026`：作者不能写 `Anonymous arXiv`，已在新 BibTeX 中改为公开作者列表。
- `BronchoCopilot`：旧 BibTeX 作者列表错误，已按 arXiv/会议页面修正。
