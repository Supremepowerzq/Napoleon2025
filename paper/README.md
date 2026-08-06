# Napoleon2025 IEEE RA-L 论文目录

当前论文主线为：真实机器人专家示范与目标条件化 GRU，结合 U-Net 黏液分割、YOLO 支气管部位识别和 Depth Anything V2 深度估计，构成 Layer 1 全局导航、Layer 2 进度加权岔口视觉辅助、Layer 3 持续真空吸引与任务连续性的完整系统。

## 当前主文件

- `main_ral_layered_autonav.tex`：英文论文初稿。未完成参数和结果均使用 `TODO` 或 `RESULT` 显式标记。
- `main_ral_layered_autonav.pdf`：本地编译生成的 7 页 PDF。
- `literature/refs_verified.bib`：49 篇真实相关文献，均已在正文中引用。
- `figures/README.md`：八幅配图的占位位置、子图排版和内容说明。
- `experiments/experimental_protocol_zh.md`：已核实数值、缺失数据字段和最终对比实验协议。
- `build_paper.ps1`：Windows 本地一键编译脚本。

## 已保留的研究证据

- 专家库：87 条真实机器人示范、30,730 帧、7 类已采集目标。
- 网络：20-D 结构化观测、8 帧窗口、16-D 目标嵌入、两层 GRU（hidden 256）、动作/任务双头。
- GRU：773,221 参数，最佳验证动作 MSE 0.002084，验证任务准确率 100%。
- YOLO：9 类、2.59M 参数，Precision 0.8926、Recall 0.7797、mAP50 0.9096、mAP50–95 0.6006。
- 最新状态：三层功能与持续吸引已完成联合验证；正式稿仍需把逐次实验记录汇总为成功率、接触次数、清除时间和连续性指标。
- 软件一致性：历史版本的 92 项硬件无关逻辑检查全部通过。它们不是 92 次物理导航实验，不能当作有效性结果。

## 当前不能直接投稿的原因

- U-Net 数据集规模、Dice/IoU 和 Depth Anything 标定误差没有保存在当前工作区。
- 完整三层试验尚缺结构化 CSV、重复次数、置信区间和对比/消融统计。
- 当前独立 PPO 训练结果不能证明强化学习提高了 AutoNav，必须重训有界残差策略或从正文贡献中删除。
- Layer 2 专项测试有一个断言仍使用旧的 M2 速度系数，应更新后重新运行。
- 所有 `TODO`/`RESULT`、作者信息、硬件参数、真实图像和统计结果必须补齐。
- 参考文献中的预印本应在投稿前再次检查是否已有正式出版版本。

## 本地编译

```powershell
cd G:\zq\Napoleon2025\paper
.\build_paper.ps1
```

清理辅助文件并完整重建：

```powershell
.\build_paper.ps1 -Clean
```

在线编辑可使用 Overleaf：新建项目后上传 `main_ral_layered_autonav.tex`、`literature/refs_verified.bib` 和 `ieeeconf.cls`，主文件选择 `main_ral_layered_autonav.tex`。
