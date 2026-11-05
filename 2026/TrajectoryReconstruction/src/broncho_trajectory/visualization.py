"""生成可直接在浏览器中打开的 Plotly 三维视图。"""

from __future__ import annotations

import json
from typing import Any, Sequence

from .geometry import Vector
from .kinematics import TrajectoryPoint

PLOTLY_CDN = "https://cdn.plot.ly/plotly-2.35.2.min.js"


def _page(title: str, subtitle: str, traces: str) -> str:
    return f"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>{title}</title><script src="{PLOTLY_CDN}"></script>
<style>body{{margin:0;background:#101827;color:#e5edf7;font:14px system-ui,sans-serif}}
header{{padding:15px 22px;border-bottom:1px solid #263448}}h1{{margin:0;font-size:20px}}
p{{margin:6px 0 0;color:#b7c6d8}}#plot{{height:calc(100vh - 95px);min-height:580px}}</style></head>
<body><header><h1>{title}</h1><p>{subtitle}</p></header><div id="plot"></div>
<script>{traces}</script></body></html>"""


def trajectory_html(points: Sequence[TrajectoryPoint], title: str) -> str:
    payload = json.dumps(
        {
            "x": [p.position[0] for p in points],
            "y": [p.position[1] for p in points],
            "z": [p.position[2] for p in points],
            "t": [p.timestamp for p in points],
        },
        ensure_ascii=False,
    )
    script = f"""const d={payload};
const trajectory={{type:'scatter3d',mode:'lines+markers',x:d.x,y:d.y,z:d.z,name:'重建轨迹',
line:{{color:'#48b6ff',width:6}},marker:{{size:2,color:d.t,colorscale:'Viridis',showscale:true,
colorbar:{{title:'时间 (s)'}}}},hovertemplate:'t=%{{marker.color:.3f}} s<br>X=%{{x:.2f}} mm<br>Y=%{{y:.2f}} mm<br>Z=%{{z:.2f}} mm<extra></extra>'}};
const last=d.x.length-1;
const start={{type:'scatter3d',mode:'markers+text',x:[d.x[0]],y:[d.y[0]],z:[d.z[0]],text:['起点'],
textposition:'top center',marker:{{size:6,color:'#35d07f'}},name:'起点'}};
const end={{type:'scatter3d',mode:'markers+text',x:[d.x[last]],y:[d.y[last]],z:[d.z[last]],text:['终点'],
textposition:'top center',marker:{{size:6,color:'#ff6674'}},name:'终点'}};
Plotly.newPlot('plot',[trajectory,start,end],{{paper_bgcolor:'#101827',plot_bgcolor:'#101827',
font:{{color:'#e5edf7'}},margin:{{l:0,r:0,t:0,b:0}},scene:{{aspectmode:'data',
xaxis:{{title:'X（初始前进方向）/ mm'}},yaxis:{{title:'Y / mm'}},zaxis:{{title:'Z（初始上向）/ mm'}},
camera:{{eye:{{x:1.5,y:1.5,z:1.0}}}}}},legend:{{orientation:'h',y:.98,x:.02}}}},{{responsive:true}});"""
    return _page(title, "拖动旋转视角，滚轮缩放；绿色为起点，红色为终点。", script)


def registered_html(
    title: str,
    model_lines: Sequence[Sequence[Vector]],
    route: Sequence[Vector],
    corrected: Sequence[Vector],
    raw_registered: Sequence[Vector],
    times: Sequence[float],
    errors: Sequence[float],
) -> str:
    payload = json.dumps(
        {"model": model_lines, "route": route, "corrected": corrected, "raw": raw_registered,
         "times": times, "errors": errors},
        ensure_ascii=False,
    )
    mean_error = sum(errors) / len(errors)
    max_error = max(errors)
    script = f"""const d={payload};
function trace(a,name,color,width,show=true){{return {{type:'scatter3d',mode:'lines',
x:a.map(p=>p[0]),y:a.map(p=>p[1]),z:a.map(p=>p[2]),name,line:{{color,width}},showlegend:show}}}}
const model=d.model.map((a,i)=>trace(a,'完整气道中心线','#738092',2,i===0));
const raw=trace(d.raw,'原始运动学（配准）','#b68cff',5);raw.customdata=d.errors;raw.text=d.times;
raw.hovertemplate='t=%{{text:.2f}} s<br>残差=%{{customdata:.2f}} mm<extra></extra>';
Plotly.newPlot('plot',[...model,trace(d.route,'相关路径（高亮）','#43d9e6',7),
trace(d.corrected,'中心线约束轨迹','#A5B55D',5),raw],{{paper_bgcolor:'#101827',plot_bgcolor:'#101827',
font:{{color:'#e5edf7'}},margin:{{l:0,r:0,t:0,b:0}},scene:{{aspectmode:'data',
xaxis:{{title:'LPS X / mm'}},yaxis:{{title:'LPS Y / mm'}},zaxis:{{title:'LPS Z / mm'}},
camera:{{eye:{{x:1.5,y:1.5,z:1.0}}}}}},legend:{{orientation:'h',y:.98,x:.02}}}},{{responsive:true}});"""
    subtitle = f"灰色：完整中心线；青色：相关路径；浅绿色：约束轨迹；紫色：原始轨迹。平均残差 {mean_error:.2f} mm，最大残差 {max_error:.2f} mm。"
    return _page(title, subtitle, script)
