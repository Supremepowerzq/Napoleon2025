# Figures Placeholder

论文需要的图片（矢量图 PDF/SVG，300 dpi）：

| 文件名 | 内容 | 建议工具 |
|--------|------|---------|
| fig1_system_overview.pdf | 系统四层架构图 | draw.io → 导出 PDF |
| fig2_perception.pdf | 感知流程图（原始帧/UNet/深度/DepthPath）| 代码截图拼图 |
| fig3_network.pdf | BronchusPolicy 网络架构图 | draw.io / PowerPoint |
| fig4_results.pdf | 实验结果：NSR柱状图+轨迹图+电机曲线 | matplotlib |

## draw.io 使用提示
1. 打开 https://app.diagrams.net/
2. 绘制完成后 File → Export → PDF（勾选 "Fit page"）
3. 放入此目录，LaTeX 会自动引用

## matplotlib 结果图模板
```python
import matplotlib.pyplot as plt
import matplotlib as mpl
mpl.rcParams['pdf.fonttype'] = 42   # 保证字体可编辑
mpl.rcParams['ps.fonttype']  = 42
fig.savefig('fig4_results.pdf', bbox_inches='tight', dpi=300)
```
