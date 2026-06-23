# 论文投稿检查清单 & 时间规划

> 目标：9月奖学金评审前拿到 IEEE RA-L Acceptance Letter

---

## 一、时间线

| 阶段 | 截止日期 | 任务 | 备注 |
|------|---------|------|------|
| **数据采集** | 6月20日 | 录制 ≥80 条专家演示（含路径标签+子任务标签） | 每天录 10+ 条 |
| **BC 训练** | 6月25日 | 完成 BC 训练，val loss < 0.05，离线推理验证 | `python bc_main.py train` |
| **实验1** | 7月5日 | 仿体实验：NSR、MEE、延迟、平滑度 | 需要打印/购买 3D 仿体 |
| **实验2** | 7月10日 | 消融实验（无 goal / 无 task head / reactive baseline）| 对比 4 组方法 |
| **全文初稿** | 7月15日 | 英文 6 页初稿完成，含所有图表 | 基于 paper_draft_ral.md |
| **润色投稿** | 7月20日 | 英文润色 + 补充视频 + IEEE 模板排版 | 可用 Grammarly/专业机构 |
| **正式投稿** | 7月25日 | 提交至 IEEE RA-L，附 Cover Letter | 在 submission notes 说明奖学金评审 |
| **应对审稿意见** | 8月15日 | 若收到 Minor Revision，3天内完成修改 | |
| **目标接收** | 8月25日 | 拿到 Acceptance Letter | |

---

## 二、备选方案（若 RA-L 超时）

```
首选：  IEEE RA-L      投稿 7月25日 → 目标接收 8月25日
备选1： IEEE TCST      RA-L拒稿后立即转投（审稿 1-2 个月）
备选2： Complex & Intelligent Systems  OA，审稿最快 1.5 个月
注意：  严禁一稿多投！
```

---

## 三、投稿材料清单

### 3.1 论文主体 (6页 ≤ IEEE RA-L 上限)
- [ ] 摘要 (≤150 词)
- [ ] Introduction (含清晰的 contribution list)
- [ ] System Overview (含系统架构图)
- [ ] Hardware Platform (含电机参数表)
- [ ] Multimodal Perception (含观测向量表)
- [ ] Policy Learning (含网络示意图)
- [ ] Experiments (含定量结果表 + 轨迹图)
- [ ] Conclusion

### 3.2 必要图片（矢量图/300 dpi）
- [ ] Fig. 1: 系统总体架构图（4层：硬件→感知→策略→交互）
- [ ] Fig. 2: BronchusPolicy 网络架构图
- [ ] Fig. 3: 实验结果图（NSR 柱状图 + 代表性轨迹）
- [ ] Fig. 4: 仿体实验照片（真实系统）

### 3.3 补充材料
- [ ] 演示视频 (~2 min)：手动 → 视觉 → AI-Auto 完整流程
- [ ] 代码链接（GitHub，含 BC/ 子模块）

### 3.4 Cover Letter
```
建议内容：
- 工作新颖性（multimodal + goal-conditioned + real-time）
- 与 RA-L 范围高度匹配（medical robotics, autonomous systems）
- 实际硬件平台验证
- 礼貌说明奖学金评审截止日期，请求加快审稿
```

---

## 四、图片制作指南

### Fig. 1 系统架构图
```
建议工具：draw.io / Visio / PowerPoint
内容：4列布局
┌──────────────┬──────────────┬──────────────┬──────────────┐
│  Hardware    │  Perception  │   Policy     │ Interaction  │
│  3-DoF       │  UNet Seg    │  BC Policy   │  GUI         │
│  RMD Motors  │  DAV2 Depth  │  GRU+Goal    │  Voice       │
│  Camera      │  DepthPath   │  ActionHead  │  Joystick    │
│  Scope       │  20-dim Obs  │  TaskHead    │  Recorder    │
└──────────────┴──────────────┴──────────────┴──────────────┘
```

### Fig. 2 BronchusPolicy 网络图
```
[obs_seq B×T×20] → [ObsEncoder] → [B×T×128]
[goal_id B]      → [Embedding]  → [B×T×16]  concat→[B×T×144]→[GRU]→[B×256]
                                                                  ├→[ActionHead]→[B×3]
                                                                  └→[TaskHead]→[B×2]
```

---

## 五、关键参考文献（建议补全）

### 支气管镜机器人
1. Swaney et al., "Tendons, Tubes, and Templates," JMRR 2017
2. Roesthuis et al., "On the continuum mechanics of flexible needle steering," IROS 2014
3. Camarillo et al., "Mechanics modeling of tendon-driven continuum manipulators," TRO 2008
4. Gu et al., "Transoral Robotic Thyroidectomy," TMRB 2019

### 自主内镜导航
5. Xu et al., "Autonomous colonoscopy," MICCAI 2023
6. Li et al., "Reinforcement learning for bronchoscope navigation," RAL 2022 (若有)

### 行为克隆 / 模仿学习
7. Chi et al., "Diffusion Policy," RSS 2023
8. Zhao et al., "ACT: Learning Fine-Grained Bimanual Manipulation," RSS 2023
9. Ho & Ermon, "Generative Adversarial Imitation Learning," NeurIPS 2016

### 深度估计
10. Yang et al., "Depth Anything V2," NeurIPS 2024

### 语义分割
11. Ronneberger et al., "U-Net," MICCAI 2015

---

## 六、IEEE RA-L 投稿注意事项

1. **页数**：正文严格 ≤ 6 页（含图表），参考文献不计入页数
2. **模板**：使用 IEEE Robotics RA-L LaTeX 模板（`IEEEtran.cls`）
3. **视频**：MP4，≤50 MB，≤2 分钟，需上传到 ScholarOne
4. **双盲**：RA-L 不采用双盲审稿，作者信息正常填写
5. **Cover Letter**：在 Submission Notes 中注明 "This paper is submitted for consideration in IEEE Robotics and Automation Letters"
6. **IEEE 模板下载**：https://www.ieee.org/publications/authors/author_templates.html

---

## 七、快速提升接收率的写作技巧

### Contribution List 写法（Introduction 最后段）
```
The main contributions of this letter are:
1) [具体创新1，一句话，含方法名]
2) [具体创新2]
3) [实验验证，含具体数字]
每条 ≤ 2 行，3-4 条最佳
```

### Experiment 结果呈现
- 主表：NSR(%) ± std 对比 4 个方法 × 3-4 个目标支气管
- 消融表：有无 goal-conditioning, 有无 task-head 的影响
- 延迟表：各模块推理时间分解（UNet / DAV2 / BronchusPolicy / total）
- 视频：必须有，极大提高接收率

### 常见拒稿原因（提前规避）
- 缺乏定量对比（vs. baseline 方法）
- 仿体实验太简单（建议至少 3 条支气管分支）
- 推理速度未报告（RA-L 对机器人实时性要求高）
- 图质量差（建议矢量图 SVG/PDF 导出）
