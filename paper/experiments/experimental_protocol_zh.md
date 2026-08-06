# IEEE RA-L 最终数据与实验协议（2026-07-16）

## 1. 已从模型文件核实、可直接写入的结果

### GRU 模仿学习

- 87 条专家示范，30,730 帧，7 类已采集目标。
- 20-D 观测，8 帧窗口，16-D 目标嵌入。
- ObsEncoder：20→128→128，LayerNorm + ELU。
- 两层 GRU，hidden 256，dropout 0.1。
- ActionHead：256→128→3；TaskHead：256→64→2。
- 总参数：773,221。
- 最佳模型：epoch 29，验证动作 MSE = 0.0020837828，验证任务准确率 = 1.000。
- 动作缩放：M0/M1/M2 = [5°, 3°, 3°]。

### YOLO 部位识别

- 9 类：TR、RMB、RUL、RML、RLL、LMB、LUB、LLB、BI。
- YOLO11n 初始化，120 epochs，640×640，batch 16，AdamW，lr 5e-4，cosine，弱增强。
- 参数：2,591,595。
- Precision = 0.89261；Recall = 0.77973；mAP50 = 0.90959；mAP50–95 = 0.60057。

### Layer 2 控制参数

- 中心死区 0.08；M1/M2 增益 3.0/2.5；EMA 0.25；最大总偏置均为 ±5°。
- 进度最低权重 0.25，80% 进度达到完整权重。
- 中心距离 ≤0.30 为完整权重，0.30–0.75 线性衰减，≥0.75 停止修正。
- M0 不接受岔口视觉修正。

## 2. 必须从你的最终实验记录补入的数据

请按“每一行一次试验”整理 CSV，建议字段：

```text
trial_id,date,target,method,condition,seed_or_model,
success,wrong_branch,completion_time_s,manual_takeover,
contact_count,contact_duration_s,center_error_mean_px,center_error_p95_px,
m0_rmse_deg,m1_rmse_deg,m2_rmse_deg,max_visual_offset_deg,
mucus_present,mucus_clear_success,clearance_time_s,
saved_pose_error_deg,continue_success,home_error_deg,
pump_pressure_kpa,pump_flow_lpm,total_suction_time_s,notes
```

论文目前最缺的数值：

1. 每个目标、每种方法的试验次数和成功/失败数。
2. Layer 1 与 Layer 1+2 的碰壁次数、接触持续时间和中心误差。
3. Layer 2 对完成时间及错误分支率是否产生副作用。
4. 黏液清除成功率、清除时间、目标短暂丢失次数和残余面积。
5. 返回保存位置的三轴最大误差、继续巡检成功率、最终归零误差。
6. 真空泵型号、负压、流量和单次总吸引时长。
7. U-Net 训练/验证/测试图像数、类别分布、Dice、IoU、Precision、Recall。
8. Depth Anything 标定样本数、MAE、RMSE、偏差和有效深度帧比例。
9. 各模型及整条链路的 GPU/CPU 推理延迟和实际帧率。

## 3. 模型实验

### 3.1 GRU 必做消融

| Variant | Action MSE | M0/M1/M2 MAE | Task macro-F1 | Route success | Contacts | Latency |
|---|---:|---:|---:|---:|---:|---:|
| Frame BC, no goal | | | | | | |
| Frame BC + goal | | | | | | |
| GRU, no goal | | | | | | |
| GRU + goal | | | | | | |
| GRU + goal + task head | 0.002084 | | | | | |

至少训练 3 个随机种子。任务准确率 100% 不能单独作为结论，必须补 macro-F1、混淆矩阵和按目标分层的动作误差。

### 3.2 U-Net 必做实验

- 按视频或手术序列划分数据，不能随机拆相邻帧。
- 基线：HSV/阈值、U-Net、U-Net + Dice、U-Net + 增强/困难负样本。
- 指标：Dice、IoU、Precision、Recall、漏检连续长度、推理时间。
- 困难集：高光、低照度、运动模糊、气泡、器械遮挡、黏液很小、黏液颜色接近组织。

### 3.3 YOLO 与持续确认

- 给出逐类别 AP、混淆矩阵和每类样本数。
- 对照：单帧 YOLO；连续帧确认；连续帧 + 解剖拓扑；完整的拓扑 + 运动方向 + 暂停冻结。
- 指标：部位准确率、非法跳转数、确认延迟、短时漏检后的稳定性。

### 3.4 深度估计

- 在 10、20、30、40、50 mm 等距离放置标定目标，每个距离不少于 20 帧。
- 报告原始输出和线性标定后 MAE/RMSE；单独统计 U-Net mask 内深度。
- 消融：无深度、未滤波深度、跳变拒绝 + EMA 的完整方法。

## 4. 系统对比实验

推荐方法：人工专家、Frame BC、Layer 1、纯视觉追踪、Layer 1+2、完整 Layer 1+2+3。每目标每方法建议不少于 10 次；若工作量过大，主实验至少 5 个目标，每目标 10 次，其余放补充材料。

扰动条件：标准起点、起点角偏差、仿体小位移、照明变化、镜头污染、不同黏液位置/体积。所有方法必须用相同速度上限和成功定义。

统计：

- 成功率与清除率：Wilson 95% CI，配对二分类可用 McNemar 检验。
- 连续指标：先检验分布；配对 t 检验或 Wilcoxon，并做多重比较校正。
- 同时报告效应量，不能只报告 p 值。
- 失败试验全部保留并分类：错误分支、碰壁停止、感知丢失、深度异常、未清除、未能继续、归零超差。

## 5. Layer 2 消融顺序

1. 面积最大候选。
2. 距离画面中心最近候选。
3. 最近候选 + 中心死区。
4. 再加 EMA 和总偏置上限。
5. 再加进度权重。
6. 完整进度 × 距离连续权重。

主指标必须是中心误差和接触次数，同时报告成功率，防止“更居中但走错分支”。

## 6. Layer 3 实验

黏液替代物至少设置小/中/大三种体积和近/中/远三类位置。对照：人工吸引、无深度追踪、深度追踪、完整持续吸引方案。

每次试验记录：首次检测时间、对准时间、清除完成时间、mask 面积曲线、深度曲线、M0–M2、保存位置、返回误差、继续巡检结果、最终归零误差、泵参数和是否人工干预。

## 7. 强化学习边界

当前 `RL_control` 中独立 PPO 日志的评估成功率为 0%，不能写成强化学习提高了 AutoNav。若要保留该创新点，必须重新训练“有界残差 PPO”：动作只能微调 Layer 2 增益/偏置，不能直接接管三轴；先完成仿真成功率和接触消融，再上机。若投稿前不能完成，应从摘要和主要贡献中删去强化学习，只放在展望。

## 8. 当前测试问题

- 20 项部位识别/PyQt 逻辑测试通过。
- Layer 2 专项脚本中的一个源码一致性检查仍期待旧的 `VISION_M2_SPD_SCALE=4.0`，而当前程序已调为 2.0。应更新测试后重新运行，论文才能写“当前回归测试全部通过”。
- 软件检查不能替代实体机器人实验。
