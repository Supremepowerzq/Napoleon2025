"""Independent English presentation layer. Original controller is read unchanged.
Run with --preview for a UI-only preview; normal execution uses the original main().
"""
import ast
import json
import re
import sys
import os
import queue
import threading
from pathlib import Path
from datetime import datetime
from typing import *
import tkinter as native
from tkinter.scrolledtext import ScrolledText as NativeScrolledText

ORIGINAL = 'main2026-Xhandwriting-AC-AutoNav(Animal_test)-Bronchus.py'
BLUE = '#1746A2'
GREEN = '#C7E6BA'
GRAY = '#EDF1F4'
INK = '#243B53'

# Presentation strings only. Underlying variables, commands and filenames stay intact.
TRANSLATIONS = {
'Napoleon2025 - 智能控制终端': 'Mosic-Bronchuscopy,Napoleon 2026',
'Napoleon2025': 'Mosic-Bronchuscopy,Napoleon 2026',
'手动控制 (M1旋转 / M2俯仰)': 'MANUAL CONTROL  |  M1 Rotation / M2 Pitch',
'前进/后退 (M0)': 'INSERTION / RETRACTION  |  M0',
'Layer 1：指定路径自主导航，或五叶段全流程自主巡检': 'Layer 1: selected trajectory navigation or complete five-lobe inspection',
'Layer 1：指定路径自主导航，或最多十段动作的全流程自主巡检': 'Layer 1: navigate to a target or inspect up to 10 actions',
'（Layer 2/3 均为可选；Layer 3 仅在大面积阻塞物时介入）': 'Optional layers. Layer 3 intervenes only for extensive obstruction.',
'当固定起点或装配位置改变时，在现场重新采集一条从原点到目标部位的动作路径。': 'Record a new origin-to-target trajectory when the origin or assembly position changes.',
'可直接输入任意名称（如：左肺实验点 A）；保存后会自动出现在上方“导航目标”中。': 'Enter any trajectory name. Saved recordings appear under Navigation target.',
'全流程自主巡检设定（顺序固定，轨迹可选）': 'FULL INSPECTION PLAN  |  Fixed order / selectable trajectories',
'全流程自主巡检设定（最多10个动作，按选择顺序执行）': 'FULL INSPECTION PLAN  |  Up to 10 actions in selected order',
'尚未规划：选择 1～10 个动作后点击“规划全流程”': 'Select 1-10 actions, then click Plan Inspection.',
'全流程顺序：按所选动作 1～10 执行，最后返回 TR 原点': 'Inspection: selected actions 1-10, then return to TR origin',
'按顺序选择 1～10 个动作；尾部可留空。规划时检查帧数、前进行程及相邻轨迹公共点。': 'Select 1-10 actions in order; leave trailing slots empty. Planning checks frames, travel and shared points.',
'（不加入规划）': '(Skip this action)',
'轨迹名称:': 'Trajectory name:', '导航目标:': 'Navigation target:',
'动作': 'Action ',
'尚未规划：请选择五条轨迹后点击“规划全流程”': 'Select five trajectories, then click Plan Inspection.',
'尚未完成有效规划：请先点击“规划全流程”': 'A valid plan is required. Click Plan Inspection first.',
'规划期间选择已变化：请重新点击“规划全流程”': 'Selection changed during planning. Click Plan Inspection again.',
'选择已变化：请重新点击“规划全流程”': 'Selection changed. Click Plan Inspection again.',
'正在规划并验证公共点，请稍候…': 'Planning and validating shared points. Please wait...',
'全流程顺序：左上→左下→右上→右中→右下→返回 TR 原点': 'Inspection: LUL > LLL > RUL > RML > RLL > TR origin',
'状态：待命｜文件保存到 BC/expert_demos/AutoNavdatasets': 'Status: Standby | Save location: BC/expert_demos/AutoNavdatasets',
'固定出发原点': 'Fixed origin', '全部导航至': 'Return all axes to',
'（无可用轨迹）': '(No available trajectory)',
'启用 Layer 2 近中心微调': 'Layer 2: center refinement',
'启用 Layer 3 阻塞物清除': 'Layer 3: obstruction removal',
'Layer 2 岔口修正': 'Layer 2: branch correction',
'Layer 3 阻塞物清除': 'Layer 3: obstruction removal',
'自主导航·AutoNav': 'AUTONOMOUS NAVIGATION', '现场路径采集': 'TRAJECTORY COLLECTION',
'图像识别模式': 'VISUAL TRACKING MODE', '键盘 1/2/3 或点击：': 'Press 1 / 2 / 3 or select:',
'开始全流程巡检': 'Start Inspection', '开始导航': 'Start Navigation',
'开始采集': 'Record Path', '结束并保存': 'Stop & Save', '规划全流程': 'Plan Inspection',
'手动模式': 'Manual', '自动模式': 'Vision', 'AI巡检': 'AI Inspection',
'设为零位': 'Set Zero', '归零': 'Return to Zero', '断电': 'Power Off',
'退出程序': 'Exit Console', '按住说话': 'Hold to Speak', '松开结束': 'Release to Finish',
'正在录音...': 'Recording...', '未录音': 'Microphone ready',
'当前状态：': 'Current state: ', '目标状态：': 'Target state: ',
'运行日志': 'EVENT LOG', '导航路径:': 'Trajectory:', '目标部位:': 'Target:',
'1 岔口': '1 Branch', '2 阻塞物': '2 Obstruction', '3 混合': '3 Mixed',
'阻塞物优先混合追踪': 'Obstruction-priority mixed tracking',
'阻塞物追踪/清除': 'Obstruction tracking / removal', '岔口追踪': 'Branch tracking',
'请先完成全流程路径规划': 'Complete the inspection plan first',
'请先选择一个动作文件': 'Select an action file first',
'手动控制': 'Manual control', '视觉控制': 'Vision control', '空闲保持': 'Idle hold',
'断电维护': 'Power-off maintenance', 'AI自主巡检': 'AI inspection',
'AI自主': 'AI navigation', '返回原点': 'Return to origin',
'左主支气管': 'Left main bronchus', '右主支气管': 'Right main bronchus',
'左上叶': 'Left upper lobe', '左下叶': 'Left lower lobe',
'右上叶': 'Right upper lobe', '右中叶': 'Right middle lobe', '右下叶': 'Right lower lobe',
'中间支气管': 'Intermediate bronchus', '气管': 'Trachea',
'左上': 'LUL', '左下': 'LLL', '右上': 'RUL', '右中': 'RML', '右下': 'RLL',
'总时长:': 'Duration:', '已播放:': 'Elapsed:', '剩余:': 'Remaining:',
'状态：': 'Status: ', '状态:': 'Status:', '进度:': 'Progress:', '步骤': 'Step',
'待命': 'Standby', '暂停': 'Pause', '停止': 'Stop', '未知': 'Unknown', '无': 'None',
'正在规划': 'Planning', '规划成功': 'Plan ready', '规划失败': 'Planning failed',
'已完成': 'Completed', '已暂停': 'Paused', '已停止': 'Stopped', '运行中': 'Running',
'导航中': 'Navigating', '已切换模式': 'Tracking mode selected', '图控': 'Vision',
'自主巡检': 'Inspection', '语音指令': 'Voice command',
}
TRANSLATIONS.update({
'个动作已规划：':'actions planned: ', '个动作→TR':'actions > TR',
'段动作轨迹拼接':'joined action trajectories',
'开始执行已规划的 ':'Executing planned ', '条拼接轨迹':'joined trajectories',
'全流程巡检完成并已返回 TR 出发原点':'Inspection complete. Returned to TR origin',
'启动取消：请先选择动作并点击“规划全流程”':'Start cancelled: select actions and click Plan Inspection first',
'请至少选择一条动作轨迹':'Select at least one action trajectory',
'前有空白动作；请按顺序选择，空白只放在最后':' has an empty preceding slot. Select actions in order and leave empty slots at the end',
'请先输入轨迹名称':'Enter a trajectory name first',
'轨迹名称不能为空':'Trajectory name cannot be empty',
'轨迹名称不能超过 40 个字符':'Trajectory name must be 40 characters or fewer',
'轨迹名称过长':'Trajectory name is too long',
'选中的采集路径不存在，请重新刷新列表':'Selected trajectory is missing. Refresh the list',
'轨迹元数据无效':'Invalid trajectory metadata', '缺少有效 path_label':'Missing valid path_label',
'缺少有效的路径标签':'Missing valid trajectory label',
'有效帧少于 20':'Fewer than 20 valid frames',
'M0 前进行程不足，可能是已返回原点的闭环记录':'Insufficient M0 forward travel; the recording may already return to the origin',
'相邻轨迹未找到近似重复点':'No shared point found between adjacent trajectories',
'轨迹不可用于拼接':'Trajectory cannot be joined', '文件读取失败':'Cannot read file',
'轨迹不在采集目录内':'Trajectory is outside the recording directory',
'[自定义]':'[Custom]',
'路径导航':'Trajectory navigation','等待到位':'Settling','阻塞清除':'Obstruction removal','返回断点':'Return to checkpoint',
'全流程':'Full inspection','目标:':'Target:','电机:':'Motors:','完成':'Complete',
'路径已规划完成：':'Plan ready: ', '个公共点，可以开始全流程自主巡检':'shared points. Ready to start inspection',
'全流程规划模块未加载':'Inspection planning module unavailable',
'请先结束并保存当前现场采集':'Stop and save the current recording first',
'启动取消：请先选择五条轨迹并点击“规划全流程”':'Start cancelled: select five trajectories and click Plan Inspection',
'启动取消：轨迹选择已变化，请重新规划':'Start cancelled: trajectory selection changed. Replan first',
'启动取消：请先用 TR/归零将三轴返回出发原点，':'Start cancelled: return all axes to the TR origin first. ',
'无法读取电机角度':'Cannot read motor angles', '开始执行已规划的五条拼接轨迹':'Starting the five planned trajectories',
'近中心保守微调已开启':'Conservative center refinement enabled',
'大面积阻塞物抢占清除与断点续航已开启':'Obstruction removal and checkpoint recovery enabled',
'关闭（纯 Layer 1）':'Disabled (Layer 1 only)', '关闭':'Disabled','顺序：':'Order: ', '当前=':'Current=',
'请先点击“结束并保存”完成当前现场采集':'Click Stop & Save to finish the current recording first',
'请先结束当前采集，再开始导航':'Finish the current recording before starting navigation',
'请先选择目标部位':'Select a target first','未知的路径代码':'Unknown trajectory code',
'三轴固定原点':'Fixed three-axis origin',
'选中的采集路径不存在，请重新启动程序刷新列表':'Selected trajectory is missing. Restart to refresh the list',
'当前无可用导航':'No navigation is available','已恢复':'Resumed','当前状态无法暂停':'Cannot pause in the current state',
'请选择 TR 以外的具体目标部位':'Select a target other than TR','目标部位无效':'Invalid target',
'正在采集':'Recording trajectory','请使用手柄或 UI 完成介入动作':'Use the controller or UI to perform the intervention',
'启动失败：可能已在采集中':'Start failed: a recording may already be active',
'启动失败，当前可能已在采集':'Start failed: a recording may already be active',
'采集器未初始化，请检查 BC 模块':'Collector is not initialized. Check the BC module',
'采集器不可用':'Collector unavailable','已保存':'Saved','采集完成':'Recording complete',
'帧数不足（< 10 帧），已丢弃':'Insufficient frames (< 10); recording discarded',
'帧数不足，未保存':'Insufficient frames; recording not saved',
'当前未在采集':'No active recording','当前没有正在进行的采集':'No active recording',
'全流程轨迹已在出发前完成拼接，开始巡检':'Trajectories joined before departure. Starting inspection',
'五叶段离散轨迹拼接':'Five-lobe joined trajectories','同部位平均路径':'Average trajectory for target',
'导航速度系数=':'Navigation speed factor=', '采集 20Hz，控制循环 60Hz':'Recording 20 Hz, control loop 60 Hz',
'初始化失败':'Initialization failed','推理异常':'Inference error','阻塞物面积':'Obstruction area',
'超过阈值，暂停轨迹并保存断点':'Threshold exceeded. Trajectory paused; checkpoint saved',
'已确认阻塞物清除，返回轨迹断点':'Obstruction cleared. Returning to checkpoint',
'已回到断点':'Checkpoint reached', '继续自主巡检':'Resuming inspection',
'五叶段巡检完成并已返回 TR 出发原点':'Five-lobe inspection complete. Returned to TR origin',
'已到达目标部位':'Target reached', '巡检完成':'Inspection complete', '已达最大步数':'Maximum step count reached',
'串口连续失败':'Consecutive serial failures','次 → 自动停止':'failures > automatic stop',
'岔口偏置=':'Branch offset=', '进入自主巡检':'Starting inspection', '停止回到空闲':'Stop and return to idle',
'开始三轴返回固定原点':'Returning all three axes to fixed origin', '三轴已到原点，当前角度=':'All axes at origin. Angles=',
'返回原点超时，当前角度=':'Origin return timed out. Angles=',
'开始播放':'Playback started','播放失败':'Playback failed','开始倒放':'Reverse playback started',
'导航倍速设置为':'Playback speed set to','动作导航完成':'Playback navigation complete','动作导航已终止':'Playback navigation terminated',
'录音控制异常':'Recording control error','轨迹文件名无效':'Invalid trajectory filename','轨迹不存在':'Trajectory missing',
'尚未选择':'Not selected: ', '启动取消':'Start cancelled','路径:':'Trajectory:','启用:':'Enabled:',
'导航':'Navigation','采集':'Recording','原点':'Origin','恢复':'Resume','启动':'Start','轨迹':'Trajectory','步':'Step',
})
CJK = re.compile(r'[\u3400-\u9fff]+')

def english(value):
    text = str(value)
    for src in sorted(TRANSLATIONS, key=len, reverse=True):
        text = text.replace(src, TRANSLATIONS[src])
    # Backend diagnostics remain unchanged in the original console; never alter inputs.
    if CJK.search(text):
        # Keep unmapped diagnostics verbatim in a separate log for troubleshooting.
        try:
            with Path(__file__).with_name('animal-ui-original-messages.log').open('a', encoding='utf-8') as log:
                log.write(f'{datetime.now().isoformat()} {value}\n')
        except OSError:
            pass
        text = CJK.sub('[see original diagnostic log]', text)
    return text.translate(str.maketrans({'：': ': ', '（': '(', '）': ')', '，': ', ', '。': '.', '｜': ' | ', '“': '"', '”': '"', '、': ', '}))


def option_text(value):
    """Translate UI annotations while preserving user names and filenames."""
    text = str(value)
    if text.lower().endswith('.json'):
        prefix, separator, filename = text.partition('|')
        if not separator:
            return text
        if '[自定义]' in prefix:
            prefix = prefix.replace('[自定义]', '[Custom]')
        else:
            prefix = english(prefix)
        return f'{prefix.rstrip()} | {filename.lstrip()}'
    return english(text)

COLORS = {}
for colors, target in [
    ('#f0f2f5 #f8f9fa #f5f5f5 #f4f8ff #e0e0e0 #E0E0E0 #d5d5d5 #CCCCCC', GRAY),
    ('#4CAF50 #45a049 #3d8b40 #80CBC4 #4DB6AC #26A69A #00897B #00796B #00695C', GREEN),
    ('#FF9800 #F57C00 #EF6C00 #f44336 #d32f2f #b71c1c #E53935 #D32F2F #B71C1C #7B1FA2 #6A1B9A #4A148C #2196F3 #1976D2 #0D47A1 #1565C0 #08306B', BLUE),
    ('#81D4FA #4FC3F7 #29B6F6 #64B5F6 #42A5F5 #1E88E5 #90CAF9 #FFB74D #FFA726 #FB8C00', '#B6D5EC'),
    ('#333 #555 #666 #888 #2c3e50 #7f8c8d #17324D #888888 #9E9E9E #757575 #616161 #c0c0c0', INK),
]:
    for c in colors.split():
        COLORS[c.lower()] = target

def options(opts):
    out = dict(opts)
    for key in ('bg','background','fg','foreground','fill','outline','activebackground','activeforeground','selectcolor','highlightbackground'):
        if key in out and isinstance(out[key], str):
            out[key] = COLORS.get(out[key].lower(), out[key])
    for key in ('text', 'label'):
        if key in out:
            out[key] = option_text(out[key]) if key == 'label' else english(out[key])
    if 'font' in out and isinstance(out['font'], tuple):
        out['font'] = ('Segoe UI',) + out['font'][1:]
    return out

class Styled:
    def __init__(self, master=None, cnf=None, **kw):
        super().__init__(master, options(cnf or {}), **options(kw))
    def configure(self, cnf=None, **kw):
        if isinstance(cnf, str):
            return super().configure(cnf)
        return super().configure(options(cnf or {}), **options(kw))
    config = configure

class Label(Styled, native.Label):
    def __init__(self, master=None, cnf=None, **kw):
        if kw.get('text') == 'Napoleon2025':
            banner = native.Canvas(master, height=8, highlightthickness=0, bg=GRAY)
            banner.pack(fill='x', pady=(0, 12))
            def draw(event):
                banner.delete('all')
                stops = (BLUE, '#A5C9D8', GREEN, GRAY)
                gradient(banner, 0, 0, event.width, 8, stops)
            banner.bind('<Configure>', draw)
            kw.update(fg=BLUE, font=('Segoe UI', 18, 'bold'))
        var = kw.get('textvariable')
        if var is not None:
            self.display_var = native.StringVar(master, value=english(var.get()))
            self.source_var = var
            self.trace_id = var.trace_add('write', lambda *_: self.display_var.set(english(var.get())))
            kw['textvariable'] = self.display_var
        super().__init__(master, cnf, **kw)

class Frame(Styled, native.Frame): pass
class LabelFrame(Styled, native.LabelFrame): pass
class Checkbutton(Styled, native.Checkbutton): pass
class Scrollbar(Styled, native.Scrollbar): pass
class Entry(Styled, native.Entry): pass
class Menu(Styled, native.Menu):
    def add_command(self, cnf=None, **kw):
        return super().add_command(options(cnf or {}), **options(kw))
    def entryconfigure(self, index, cnf=None, **kw):
        # Tk queries entry configuration while deleting/rebuilding a menu.
        if isinstance(cnf, str) or (cnf is None and not kw):
            return super().entryconfigure(index, cnf)
        return super().entryconfigure(index, options(cnf or {}), **options(kw))
    entryconfig = entryconfigure

class OptionMenu(native.Menubutton):
    def __init__(self, master, variable, value, *values, **kw):
        callback = kw.pop('command', None)
        self.source_var = variable
        self.display_var = native.StringVar(master, value=option_text(variable.get()))
        self.trace_id = variable.trace_add('write', lambda *_: self.display_var.set(option_text(variable.get())))
        super().__init__(master, textvariable=self.display_var, bg=GRAY, fg=INK, activebackground=GREEN, relief='flat', indicatoron=True, **kw)
        self.menu = Menu(self, tearoff=False, bg=GRAY, fg=INK, activebackground=GREEN, activeforeground=INK)
        self.configure(menu=self.menu)
        for raw in (value,) + values:
            self.menu.add_command(label=raw, command=native._setit(variable, raw, callback))
    def __getitem__(self, key):
        return self.menu if key == 'menu' else super().__getitem__(key)
    def configure(self, cnf=None, **kw):
        return super().configure(options(cnf or {}), **options(kw))
    config = configure

class Canvas(Styled, native.Canvas):
    def _create(self, itemType, args, kw):
        return super()._create(itemType, args, options(kw))
    def itemconfigure(self, tagOrId, cnf=None, **kw):
        return super().itemconfigure(tagOrId, options(cnf or {}), **options(kw))
    itemconfig = itemconfigure

class EnglishScrolledText(NativeScrolledText):
    def __init__(self, master=None, **kw):
        super().__init__(master, **options(kw))
    def insert(self, index, chars, *args):
        return super().insert(index, english(chars), *args)

class Root(native.Tk):
    def title(self, string=None):
        return super().title(english(string) if string is not None else None)
    def geometry(self, newGeometry=None):
        if newGeometry is None:
            return super().geometry()
        width = min(1320, self.winfo_screenwidth() - 80)
        height = min(980, self.winfo_screenheight() - 100)
        return super().geometry(f'{width}x{height}+30+30')
    def minsize(self, width=None, height=None):
        return super().minsize(min(1100, self.winfo_screenwidth()-80), 600)

class Toolkit:
    def __getattr__(self, name):
        return getattr(native, name)
    Tk=Root; Label=Label; Frame=Frame; LabelFrame=LabelFrame
    Checkbutton=Checkbutton; OptionMenu=OptionMenu; Menu=Menu; Canvas=Canvas; Scrollbar=Scrollbar
    Entry=Entry

def gradient(canvas, x, y, width, height, stops):
    def rgb(color): return tuple(int(color[i:i+2], 16) for i in (1,3,5))
    steps = max(1, int(width))
    for i in range(steps):
        pos = i / max(1, steps-1) * (len(stops)-1)
        segment = min(int(pos), len(stops)-2)
        a, b = rgb(stops[segment]), rgb(stops[segment+1])
        f = pos - segment
        color = '#' + ''.join(f'{int(a[j]*(1-f)+b[j]*f):02x}' for j in range(3))
        canvas.create_line(x+i, y, x+i, y+height, fill=color)


def load_controller(preview=False):
    path = Path(__file__).with_name(ORIGINAL)
    source = path.read_text(encoding='utf-8-sig')
    scope = dict(__file__=str(path), __name__='animal_ui_controller')
    if preview:
        tree = ast.parse(source)
        classes = [n for n in tree.body if isinstance(n, ast.ClassDef) and n.name in ('RoundedButton','ArrowButton','DirectionalControlWidget','VoiceStatusWindow')]
        # Read the UI configuration from the controller so previews follow its slot count.
        constant_names = {'_INSPECTION_UI_SLOTS', '_INSPECTION_EMPTY_OPTION'}
        constants = [n for n in tree.body if isinstance(n, ast.Assign)
                     and any(isinstance(target, ast.Name) and target.id in constant_names
                             for target in n.targets)]
        scope.update(globals())
        scope.update(__file__=str(path), tk=Toolkit(), ScrolledText=EnglishScrolledText,
                     _BC_AVAILABLE=True, _BC_PATHS={i:(code,name) for i,(code,name) in enumerate([('LUB','Left upper lobe'),('LLB','Left lower lobe'),('RUL','Right upper lobe'),('RML','Right middle lobe'),('RLL','Right lower lobe')])},
                     _BC_NAMES={}, YOLO_CODE_TO_IDX={}, _list_inspection_source_files=None,
                     _DEFAULT_INSPECTION_ROUTE=(('LUB',4,'左上'),('LLB',5,'左下'),('RUL',6,'右上'),('RML',8,'右中'),('RLL',9,'右下')),
                     set_tracking_mode=lambda mode: True)
        exec(compile(ast.Module(body=constants + classes, type_ignores=[]), str(path), 'exec'), scope)
    else:
        # Inject the local presentation facade after imports, before UI classes.
        marker = 'class RoundedButton('
        prefix, suffix = source.split(marker, 1)
        exec(compile(prefix, str(path), 'exec'), scope)
        scope.update(tk=Toolkit(), ScrolledText=EnglishScrolledText)
        exec(compile(marker+suffix, str(path), 'exec'), scope)
    original_button = scope['RoundedButton']
    class GradientButton(original_button):
        def __init__(self, master, text, width=120, height=40, **kw):
            width = max(width, len(english(text))*8+26)
            super().__init__(master, text, width=width, height=height, **kw)
        def _draw(self):
            self.delete('all')
            w,h = self.winfo_reqwidth()-4, self.winfo_reqheight()-4
            text = english(self.text)
            neutral = text in ('Stop','Power Off','Exit Console','Stop & Save','Pause','Return to Zero','Set Zero','Manual')
            stops = ('#D8E0E7', GRAY) if neutral else ('#A9CDE8', GREEN)
            if self.is_pressed: stops = ('#8EB6D8','#AACFA0')
            elif self.is_hover: stops = ('#97BFE2','#B4DBA7')
            gradient(self, 1, 1, w, h, stops)
            self.create_rectangle(1,1,w,h,outline=BLUE if text in ('Stop','Power Off') else '#B8C9D8',width=2 if text in ('Stop','Power Off') else 1)
            self.create_text(w/2,h/2,text=text,fill=INK,font=('Segoe UI',10,'bold'))
    original_arrow = scope['ArrowButton']
    class AnimalArrow(original_arrow):
        def __init__(self, *args, **kw):
            kw['fg'] = INK
            super().__init__(*args, **kw)
    scope['ArrowButton'] = AnimalArrow
    scope['RoundedButton'] = GradientButton
    original_window = scope['VoiceStatusWindow']
    class AnimalWindow(original_window):
        def _poll_queue(self):
            if not getattr(self, '_animal_layout_ready', False) and self.root is not None:
                self._animal_layout_ready = True
                # Reposition existing panels; each widget retains its callbacks and variables.
                viewport = self.root.winfo_children()[0]
                canvas = next(w for w in viewport.winfo_children() if isinstance(w, native.Canvas))
                card = next(w for w in canvas.winfo_children() if isinstance(w, native.Frame))
                children = card.winfo_children()
                for widget in children:
                    widget.pack_forget()
                for row, widget in enumerate(children):
                    widget.grid(row=row, column=0, columnspan=2, sticky='ew', pady=7)
                manual = children[5]
                controls = children[6]
                manual.grid_configure(row=5, column=0, columnspan=1, sticky='nsew')
                controls.grid_configure(row=5, column=1, columnspan=1, sticky='nsew')
                card.grid_columnconfigure(0, weight=1)
                card.grid_columnconfigure(1, weight=1)
                # Long planning status is on its own line rather than competing with controls.
                for panel in children:
                    for container in panel.winfo_children():
                        for label in container.winfo_children():
                            if isinstance(label, Label) and getattr(label, 'source_var', None) is self.inspection_plan_status_var:
                                label.pack_forget()
                                label.pack(side='top', fill='x', pady=(7, 0))
                # Keep long explanatory/status labels within the viewport.
                def resize(event):
                    for widget in card.winfo_children():
                        for child in widget.winfo_children():
                            if isinstance(child, native.Label):
                                child.configure(wraplength=max(220, event.width-110))
                    canvas.configure(scrollregion=canvas.bbox('all'))
                card.bind('<Configure>', resize, add='+')
            return super()._poll_queue()
    scope['VoiceStatusWindow'] = AnimalWindow
    return scope


def main():
    preview = (
        '--preview' in sys.argv or '--ui-preview' in sys.argv
        or os.environ.get('NAPOLEON_UI_PREVIEW', '').strip().lower()
        in ('1', 'true', 'yes', 'on')
    )
    controller = load_controller(preview)
    if preview:
        ui = controller['VoiceStatusWindow']()
        def preview_log(action):
            ui.push_log(f'[UI Preview] {action}; no robot commands are sent.')
        ui.set_command_callback(lambda cmd: preview_log(f'Command: {cmd}'))
        ui.set_motor_callback(lambda motor, value: preview_log(f'M{motor}: {value:.2f}'))
        ui.set_action_record_callback(
            lambda start: preview_log('Start recording' if start else 'Stop and save')
        )
        ui.set_nav_callbacks(
            start_cb=lambda target, *layers: preview_log(f'Navigation target: {option_text(target)}'),
            pause_cb=lambda: preview_log('Pause / resume'),
            stop_cb=lambda: preview_log('Stop navigation'),
            inspection_plan_cb=lambda selection: preview_log('Plan inspection (preview only)'),
            inspection_cb=lambda selection, *layers: preview_log('Start inspection'),
        )
        ui.set_exit_callback(ui.stop_event.set)
        ui.update_state('UI Preview', 'Robot disconnected')
        ui.push_log('[UI Preview] Controls are available for layout and callback checks.')
        # UI thread only; no camera, motors, serial ports or speech recognizer.
        verification_errors = []
        if '--verify' in sys.argv:
            original_poll = ui._poll_queue
            def verify_poll():
                original_poll()
                if getattr(ui, '_verification_scheduled', False):
                    return
                ui._verification_scheduled = True
                def verify():
                    try:
                        verify_widgets()
                    except Exception as exc:
                        verification_errors.append(exc)
                    finally:
                        ui.stop_event.set()
                        ui.root.destroy()
                def verify_widgets():
                    ui.root.update_idletasks()
                    calls = []
                    ui.set_command_callback(calls.append)
                    for cmd in ('manual','vision','stop','ai_auto','zero','set_zero','poweroff'):
                        ui._on_command(cmd)
                    assert calls == ['manual','vision','stop','ai_auto','zero','set_zero','poweroff']
                    raw = 'TR - 固定出发原点 | M0/M1/M2 全部导航至 0°'
                    ui.nav_target_var.set(raw)
                    assert ui.nav_target_var.get() == raw
                    assert not CJK.search(ui.nav_target_menu.display_var.get())
                    ui.set_nav_callbacks(lambda *args: calls.append(args), lambda: None, lambda: None)
                    ui._on_nav_start()
                    assert calls[-1] == (raw, False, False)
                    slots = controller['_INSPECTION_UI_SLOTS']
                    empty = controller['_INSPECTION_EMPTY_OPTION']
                    assert len(ui.inspection_path_vars) == len(slots) == 10
                    for variable in ui.inspection_path_vars.values():
                        variable.set(empty)
                    ui._refresh_inspection_path_menus()
                    assert all(v.get() == empty for v in ui.inspection_path_vars.values())
                    assert all(m.display_var.get() == '(Skip this action)'
                               for m in ui.inspection_path_menus.values())
                    available = ui._scan_inspection_path_options()[slots[0][0]]
                    if available:
                        ui.inspection_path_vars[slots[0][0]].set(available[0])
                    selected = ui._get_inspection_selections()
                    ui._handle_item({'type': 'inspection_plan_status', 'success': True,
                                     'planned_selection': selected,
                                     'text': '1 个动作已规划：100 步 / 0 个公共点，可以开始全流程自主巡检'})
                    ui.set_nav_callbacks(lambda *args: None, lambda: None, lambda: None,
                                         inspection_cb=lambda *args: calls.append(args))
                    ui._on_full_inspection_start()
                    assert calls[-1] == (selected, False, False)
                    assert '[see original diagnostic log]' not in english(ui.inspection_plan_status_var.get())
                    ui._set_inspection_selection(slots[-1][0], empty)
                    assert not ui.inspection_plan_ready
                    name = '自定义实验点 A'
                    ui.bc_path_name_var.set(name)
                    assert ui.bc_path_name_var.get() == name
                    assert option_text(name + '.json') == name + '.json'
                    ui._handle_item({'type':'state','current':'手动控制','next':'空闲保持'})
                    ui._handle_item({'type':'recording','status':True})
                    ui._handle_item({'type':'recording','status':False})
                    ui._handle_item({'type':'nav_progress','data':{'status':'待命','progress':45,'step':2,'total':5}})
                    widgets = []
                    def walk(widget):
                        widgets.append(widget)
                        for child in widget.winfo_children(): walk(child)
                    walk(ui.root)
                    for widget in widgets:
                        if isinstance(widget, native.Label):
                            text = widget.cget('text')
                            if widget.cget('textvariable'):
                                text = ui.root.getvar(widget.cget('textvariable'))
                            assert not CJK.search(str(text)), text
                    ui.root.update()
                    if '--screenshot' in sys.argv:
                        from PIL import ImageGrab
                        ImageGrab.grab().save(str(Path(__file__).with_name('animal-ui-preview.png')))
                    print('UI_VERIFY_OK: 10 actions, trailing skips, raw names, inspection callbacks and English labels')
                ui.root.after(1500, verify)
            ui._poll_queue = verify_poll
        ui._run()
        if verification_errors:
            raise verification_errors[0]
        if '--verify' in sys.argv and not getattr(ui, '_verification_scheduled', False):
            raise RuntimeError('UI verification did not start')
    else:
        controller['main']()

if __name__ == '__main__':
    main()
