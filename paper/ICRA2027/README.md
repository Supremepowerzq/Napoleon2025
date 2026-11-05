# ICRA 2027 会议论文草稿

本文件夹包含一份采用双盲评审的 ICRA 2027 论文草稿，研究重点是用于实时自主支气管镜检查的目标条件多模态 GRU 策略。

## 官方要求（核查于 2026-08-06）

- 采用 ICRA 双栏格式。
- 初次投稿采用双盲评审；PDF 中不得出现作者姓名或所属机构。
- 全文最多 8 页，包括参考文献及所有其他材料。
- 投稿截止时间：2026-09-15 23:59（PST）。
- 可选视频：最大 20 MB、时长不超过 180 秒，采用逐行扫描，分辨率至少为 480 px，帧率至少为 20 fps。
- 若实质性使用了生成式人工智能，必须在致谢中披露。

官方来源：https://2027.ieee-icra.org/contribute/call-for-icra-2027-papers-now-accepting-submissions/

## 编译

```bash
latexmk -pdf -interaction=nonstopmode -halt-on-error main.tex
```

## 证据规范

草稿中已有的数值均可追溯至 `ZQ` 分支中的模型、检查点、数据集或已记录的笔记。红色 `TODO` 字段必须依据最终的结构化仿体试验记录或运行时日志填写，不得使用估算值。

下载的 EndoVLA 论文保存在 `reference_papers/` 目录中，供本地学术对比使用。
