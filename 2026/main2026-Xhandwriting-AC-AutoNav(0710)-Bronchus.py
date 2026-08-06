import sys
import os
parent_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_path)

import time
import serial
import threading
import queue
from datetime import datetime
from typing import Optional, Tuple, Dict, Any, Callable, TYPE_CHECKING, List

if TYPE_CHECKING:
    import tkinter as tk

# 优先使用百度语音识别（中文识别效果更好）
try:
    from aip import AipSpeech  # type: ignore[import]
    baidu_speech = True
except ImportError:
    baidu_speech = False
    AipSpeech = None

# 备用：speech_recognition（如果百度未配置）
try:
    import speech_recognition as sr  # type: ignore[import]
    import pyaudio  # type: ignore[import]
except ImportError:
    sr = None
    pyaudio = None

try:
    import tkinter as tk  # type: ignore[import]
    from tkinter.scrolledtext import ScrolledText  # type: ignore[import]
except Exception:
    tk = None
    ScrolledText = None

from transitions import Machine

from Interface.JoystickInterfaceV2 import XboxController
from Interface.SerialInterface import find_rmd_motor_port
from Interface.RmdInterfaceV2 import RmdMotor

from ToolKits.ToolBox import get_time
from ToolKits.Timer import busy_maintain_target_frequency

from config import BAUDRATE, TIMEOUT, get_config, set_tracking_mode
# 将大型视觉模块延迟导入，避免在不需要时加载 torchvision 等重依赖导致环境相关错误
UnetPackage = None
from ActionRecorder import ActionRecorder, ActionPlayer, PlaybackState

# AutoNav 模块：支气管自主巡检（三层架构：序列导航 + 岔口引导 + 阻塞物清除）
try:
    from AutoNav import (
        AutoNavController as _AutoNavController,
        DEFAULT_INSPECTION_ROUTE as _DEFAULT_INSPECTION_ROUTE,
        InspectionSequencePlayer as _InspectionSequencePlayer,
        list_inspection_source_files as _list_inspection_source_files,
    )
    from AutoNav.config import (
        INSPECTION_ORIGIN_TOLERANCE as _INSPECTION_ORIGIN_TOLERANCE,
        NAV_MOTOR_MAX_SPEED as _AUTONAV_MOTOR_SPEED,
        PLAYBACK_SPEED as _AUTONAV_DEFAULT_SPEED,
    )
    _AUTONAV_AVAILABLE = True
except Exception as _an_exc:
    _AutoNavController = None
    _InspectionSequencePlayer = None
    _list_inspection_source_files = None
    _DEFAULT_INSPECTION_ROUTE = (
        ("LUB", 4, "左上"),
        ("LLB", 5, "左下"),
        ("RUL", 6, "右上"),
        ("RML", 8, "右中"),
        ("RLL", 9, "右下"),
    )
    _AUTONAV_MOTOR_SPEED = {0: 80.0, 1: 30.0, 2: 80.0}
    _AUTONAV_DEFAULT_SPEED = 0.33
    _INSPECTION_ORIGIN_TOLERANCE = (5.0, 3.0, 5.0)
    _AUTONAV_AVAILABLE = False
    print(f"[AutoNav] 导入失败: {_an_exc}")

# BC 模块：专家演示数据采集（含视觉特征+速度+dt）
try:
    import sys as _sys, os as _os
    _sys.path.insert(0, _os.path.join(_os.path.dirname(__file__), "BC"))
    from BC.data_collector import BronchusDataCollector, get_visual_features as _bc_get_vis
    from BC.model import YOLO_CODE_TO_IDX, BRONCHUS_PATHS as _BC_PATHS, IDX_TO_NAME as _BC_NAMES
    _BC_AVAILABLE = True
except Exception as _bc_exc:
    _BC_AVAILABLE = False
    BronchusDataCollector = None
    _bc_get_vis = None
    _BC_PATHS = {}
    _BC_NAMES = {}
    YOLO_CODE_TO_IDX = {}
    print(f"[BC] 导入失败（不影响正常运行）: {_bc_exc}")

# 电机控制参数
FORWARD_COEFF = 100  # 前进/后退速度系数
TURN_COEFF = 50     # 转向速度系数（降为一半）
# ★ [修改1] M0方向符号：-1.0 表示正负方向取反（原为正方向，改后反向）
M0_DIRECTION_SIGN = -1.0
# M1方向符号：-1.0 表示正负方向取反（实际运动方向与指令相反）
M1_DIRECTION_SIGN = -1.0
# ★ [修改3] M2速度系数：因2号电机加装10倍减速器，控制速度调整为原来的10倍
M1_SPEED_COEFF = 30.0               # M1专用速度系数（独立于TURN_COEFF，避免影响M2）
M2_SPEED_COEFF = TURN_COEFF * 10.0  # = 500（set_position的max_speed上限）
# M2每帧位置增量倍率：10倍减速器意味着输出1°需要电机转10°，因此delta也需×10
M2_DELTA_MULTIPLIER = 10.0
# 视觉侧 x>0 表示目标在画面右侧。保持与手动控制相同的正输入语义，
# M1 的实际电机方向只由 map_horizontal() 内的 M1_DIRECTION_SIGN 统一处理，
# 避免在图控入口再次反号造成左右追踪相反。
HORIZONTAL_CLOCKWISE_SIGN = 1.0

# 视觉控制调参（位置模式）
VISION_POSITION_GAIN_HORIZONTAL = 0.07
VISION_POSITION_GAIN_VERTICAL = 0.05
VISION_POSITION_MAX_STEP = 1.5  # 单次最大角度调节
VISION_POSITION_MIN_STEP = 0.1  # 在死区内的最小步长（电机最小响应角度）
VISION_POSITION_INPUT_DEADZONE = 0.1

# 独立图控闭环两轴参数：M1/X响应较慢，视觉速度直接下发；M2/Y带减速器
# 响应较快，先缩放为实际速度。M2停止阈值保持很小以允许终点精调。
VISION_M1_STOP_INPUT = 2.0
VISION_M2_SPD_SCALE = 2.0
VISION_M2_STOP_SPEED = 0.5

# 视觉模式 M2 步进与 nudge 参数（可在文件开头直接调整）
VISION_M2_FORCE_STEP = 0.1         # 每次视觉步进的角度（度）- 增大步长确保电机响应
VISION_STEP_SPEED = 3.0           # 步进时发送的速度值（正/负方向）
VISION_STEP_DURATION = 0.08       # 步进速度脉冲持续时间（秒）
VISION_FALLBACK_SPEED = 2.0       # fallback 用的速度（较小的短脉冲）
# （调试相关常量已移除）

# 语音交互配置
VOICE_SWITCH = True
VOICE_WAKE_WORD = "你好助手"  # 更容易识别的唤醒词
VOICE_RECOGNITION_LANGUAGE = "zh-CN"
# 当摄像头线程占用默认麦克风时，可以通过该索引指定专用麦克风
# 设置为 None 表示使用系统默认输入设备；若需手动指定，请在
#   python -m speech_recognition
# 中查看设备列表并填入对应索引
VOICE_MIC_DEVICE_INDEX: Optional[int] = 0

# 百度语音识别配置（需要到 https://ai.baidu.com/ 申请）
# 如果未配置，将回退到 speech_recognition
# 这里与测试脚本保持一致
BAIDU_APP_ID = "7271638"  # 百度语音识别 APP ID
BAIDU_API_KEY = "sHhvqZyhyh2decNp6Q75rEc8"  # 百度语音识别 API Key
BAIDU_SECRET_KEY = "c1u9Rzeca7FSxabjxJUlGHqckjjOgozD"  # 百度语音识别 Secret Key
STANDARD_VOICE_COMMANDS = {
    "切换为手动模式": "manual",
    "切换为自动模式": "vision",
    "切换为空闲模式": "idle",
    "归零": "zero",
    "切换为视觉模式": "vision",
    "紧急停止": "stop",
    "立即停止": "stop",
    "写入零点": "set_zero",
    "切换为断电模式": "poweroff",
    "断电模式": "poweroff",
    "断电": "poweroff",
    # AI 自主巡检
    "自主巡检": "ai_auto",
    "开始自主": "ai_auto",
    "切换为自主模式": "ai_auto",
    # 指定目标支气管的快捷语音
    "回到起点": "ai_tr",
    "去右上叶": "ai_rul",
    "去右中叶": "ai_rml",
    "去右下叶": "ai_rll",
    "去左上叶": "ai_lub",
    "去左下叶": "ai_llb",
    # "去中间支气管": "ai_bi",
}


#############################################################总开关####################################################################################################
# 是否开启摄像头图像处理线程？
CAMERA_SWITCH = True
# CAMERA_SWITCH = False

# 是否开启机器人控制线程？
ROBOT_SWITCH = True
# ROBOT_SWITCH = False


def video_processing() -> None:
    """视觉线程：持续运行Unet视频推理"""
    # 延迟导入 UnetPackage，避免模块导入时触发 torchvision/torch 的复杂依赖
    try:
        from predict_2026_715_zck import UnetPackage  # type: ignore[import]


    except Exception as exc:
        print(f"{get_time()}-视频模块导入失败，跳过视觉处理：{exc}")
        return
    m_unet_package = UnetPackage(
        mode='video',
        video_path=0,  # 使用索引0的相机（默认相机）
        video_save_path='',
        video_fps=30,
        # 启用深度推理功能
        enable_depth=True,
        depth_encoder='vitl',
        depth_input_size=518,
        depth_grayscale=False,
    )
    while True:
        m_unet_package.video()


class RoundedButton(tk.Canvas if tk is not None else object):  # type: ignore[misc]
    def __init__(self, master, text, width=120, height=40, corner_radius=10, 
                 bg="#2196F3", fg="white", command=None, 
                 hover_bg="#1976D2", press_bg="#0D47A1"):
        super().__init__(master, width=width, height=height, bg=master["bg"], highlightthickness=0)
        self.command = command
        self.text = text
        self.bg = bg
        self.fg = fg
        self.hover_bg = hover_bg
        self.press_bg = press_bg
        self.corner_radius = corner_radius
        
        self.is_pressed = False
        self.is_hover = False
        
        # 绘制初始状态
        self._draw()
        
        # 绑定事件
        self.bind("<Enter>", self._on_enter)
        self.bind("<Leave>", self._on_leave)
        self.bind("<Button-1>", self._on_press)
        self.bind("<ButtonRelease-1>", self._on_release)

    def _draw(self):
        self.delete("all")
        
        # 确定当前背景色
        if self.is_pressed:
            current_bg = self.press_bg
            offset = 1
        elif self.is_hover:
            current_bg = self.hover_bg
            offset = 0
        else:
            current_bg = self.bg
            offset = 0
            
        # 绘制阴影 (仅在非按下状态显示)
        if not self.is_pressed:
            self._create_rounded_rect(2, 2, self.winfo_reqwidth()-2, self.winfo_reqheight()-2, 
                                    self.corner_radius, fill="#888888", outline="")
        
        # 绘制按钮主体 (按下时向右下偏移)
        x_off = offset
        y_off = offset
        self._create_rounded_rect(x_off, y_off, self.winfo_reqwidth()-4+x_off, self.winfo_reqheight()-4+y_off, 
                                self.corner_radius, fill=current_bg, outline="")
        
        # 绘制文字
        self.create_text(self.winfo_reqwidth()/2 + x_off, self.winfo_reqheight()/2 + y_off, 
                       text=self.text, fill=self.fg, font=("Microsoft YaHei", 10, "bold"))

    def _create_rounded_rect(self, x1, y1, x2, y2, radius=25, **kwargs):
        points = [x1+radius, y1,
                  x1+radius, y1,
                  x2-radius, y1,
                  x2-radius, y1,
                  x2, y1,
                  x2, y1+radius,
                  x2, y1+radius,
                  x2, y2-radius,
                  x2, y2-radius,
                  x2, y2,
                  x2-radius, y2,
                  x2-radius, y2,
                  x1+radius, y2,
                  x1+radius, y2,
                  x1, y2,
                  x1, y2-radius,
                  x1, y2-radius,
                  x1, y1+radius,
                  x1, y1+radius,
                  x1, y1]
        return self.create_polygon(points, **kwargs, smooth=True)

    def _on_enter(self, event):
        self.is_hover = True
        self._draw()

    def _on_leave(self, event):
        self.is_hover = False
        self.is_pressed = False
        self._draw()

    def _on_press(self, event):
        self.is_pressed = True
        self._draw()

    def _on_release(self, event):
        if self.is_pressed:
            self.is_pressed = False
            self._draw()
            if self.command:
                self.command()
                
    def config(self, **kwargs):
        if "text" in kwargs:
            self.text = kwargs["text"]
        if "bg" in kwargs:
            self.bg = kwargs["bg"]
        self._draw()




class ArrowButton(tk.Canvas if tk is not None else object):  # type: ignore[misc]
    def __init__(self, master, direction="up", width=60, height=60, bg="#4CAF50", 
                 fg="white", hover_bg="#45a049", press_bg="#3d8b40", 
                 on_press=None, on_release=None, corner_radius=10):
        super().__init__(master, width=width, height=height, bg=master["bg"], highlightthickness=0)
        self.direction = direction
        self.bg = bg
        self.fg = fg
        self.hover_bg = hover_bg
        self.press_bg = press_bg
        self.corner_radius = corner_radius
        self.on_press_callback = on_press
        self.on_release_callback = on_release
        
        self.is_pressed = False
        self.is_hover = False
        
        self._draw()
        
        self.bind("<Enter>", self._on_enter)
        self.bind("<Leave>", self._on_leave)
        self.bind("<ButtonPress-1>", self._on_press)
        self.bind("<ButtonRelease-1>", self._on_release)

    def _draw(self):
        self.delete("all")
        
        # Determine background color
        if self.is_pressed:
            current_bg = self.press_bg
            offset = 1
        elif self.is_hover:
            current_bg = self.hover_bg
            offset = 0
        else:
            current_bg = self.bg
            offset = 0
            
        # Draw shadow
        if not self.is_pressed:
            self._create_rounded_rect(2, 2, self.winfo_reqwidth()-2, self.winfo_reqheight()-2, 
                                    self.corner_radius, fill="#888888", outline="")
        
        # Draw button body
        x_off = offset
        y_off = offset
        self._create_rounded_rect(x_off, y_off, self.winfo_reqwidth()-4+x_off, self.winfo_reqheight()-4+y_off, 
                                self.corner_radius, fill=current_bg, outline="")
        
        # Draw Arrow
        cx = self.winfo_reqwidth() / 2 + x_off
        cy = self.winfo_reqheight() / 2 + y_off
        size = 15
        
        if self.direction == "up":
            points = [cx, cy - size, cx - size, cy + size, cx + size, cy + size]
        elif self.direction == "down":
            points = [cx, cy + size, cx - size, cy - size, cx + size, cy - size]
        else:
            points = []
            
        self.create_polygon(points, fill=self.fg, outline="")
    
    def _create_rounded_rect(self, x1, y1, x2, y2, radius=25, **kwargs):
        points = [x1+radius, y1,
                  x1+radius, y1,
                  x2-radius, y1,
                  x2-radius, y1,
                  x2, y1,
                  x2, y1+radius,
                  x2, y1+radius,
                  x2, y2-radius,
                  x2, y2-radius,
                  x2, y2,
                  x2-radius, y2,
                  x2-radius, y2,
                  x1+radius, y2,
                  x1+radius, y2,
                  x1, y2,
                  x1, y2-radius,
                  x1, y2-radius,
                  x1, y1+radius,
                  x1, y1+radius,
                  x1, y1]
        return self.create_polygon(points, **kwargs, smooth=True)

    def _on_enter(self, event):
        self.is_hover = True
        self._draw()

    def _on_leave(self, event):
        self.is_hover = False
        self._draw()

    def _on_press(self, event):
        self.is_pressed = True
        self._draw()
        if self.on_press_callback:
            self.on_press_callback()

    def _on_release(self, event):
        if self.is_pressed:
            self.is_pressed = False
            self._draw()
            if self.on_release_callback:
                self.on_release_callback()


class DirectionalControlWidget(tk.Canvas if tk is not None else object):  # type: ignore[misc]
    def __init__(self, master, width=200, height=200, bg="white", 
                 on_motor1_change=None, on_motor2_change=None):
        super().__init__(master, width=width, height=height, bg=bg, highlightthickness=0)
        self.on_motor1_change = on_motor1_change  # Callback for Ring (Left/Right) -> Motor 1
        self.on_motor2_change = on_motor2_change  # Callback for Slider (Up/Down) -> Motor 2
        
        self.cx = width / 2
        self.cy = height / 2
        self.ring_radius_outer = 85
        self.ring_radius_inner = 60
        self.slider_height = 100
        self.slider_val = 0.0
        
        self.active_element = None  # 'ring_left', 'ring_right', 'slider'
        
        self._draw()
        
        self.bind("<ButtonPress-1>", self._on_press)
        self.bind("<B1-Motion>", self._on_drag)
        self.bind("<ButtonRelease-1>", self._on_release)

    def _draw(self):
        self.delete("all")
        
        # Draw Ring (Motor 1) - Split into left and right halves
        # Left Half (CCW)
        self.create_arc(self.cx - self.ring_radius_outer, self.cy - self.ring_radius_outer,
                        self.cx + self.ring_radius_outer, self.cy + self.ring_radius_outer,
                        start=90, extent=180, 
                        fill="#E0E0E0" if self.active_element != 'ring_left' else "#90CAF9", 
                        outline="", tags="ring_left")
        
        # Right Half (CW)
        self.create_arc(self.cx - self.ring_radius_outer, self.cy - self.ring_radius_outer,
                        self.cx + self.ring_radius_outer, self.cy + self.ring_radius_outer,
                        start=270, extent=180, 
                        fill="#E0E0E0" if self.active_element != 'ring_right' else "#90CAF9", 
                        outline="", tags="ring_right")
        
        # Inner Circle (Hole)
        self.create_oval(self.cx - self.ring_radius_inner, self.cy - self.ring_radius_inner,
                         self.cx + self.ring_radius_inner, self.cy + self.ring_radius_inner,
                         fill=self["bg"], outline="")
        
        # Draw Slider Track (Motor 2)
        track_top = self.cy - self.slider_height/2
        track_bottom = self.cy + self.slider_height/2
        self.create_rectangle(self.cx - 5, track_top,
                              self.cx + 5, track_bottom,
                              fill="#CCCCCC", outline="")
        
        # Draw Slider Handle
        handle_y = self.cy - (self.slider_val * (self.slider_height/2 - 10))
        self.create_oval(self.cx - 15, handle_y - 15,
                         self.cx + 15, handle_y + 15,
                         fill="#2196F3", outline="", tags="slider_handle")

    def _on_press(self, event):
        dx = event.x - self.cx
        dy = event.y - self.cy
        dist = (dx*dx + dy*dy)**0.5
        
        # Check if on ring area
        if self.ring_radius_inner <= dist <= self.ring_radius_outer:
            if dx < 0:  # Left side
                self.active_element = 'ring_left'
                if self.on_motor1_change:
                    self.on_motor1_change(-1.0)  # CCW
            else:  # Right side
                self.active_element = 'ring_right'
                if self.on_motor1_change:
                    self.on_motor1_change(1.0)  # CW
            self._draw()
        # Check if on slider area (center)
        elif dist < self.ring_radius_inner:
            self.active_element = 'slider'
            self._update_slider(dy)

    def _on_drag(self, event):
        if self.active_element == 'slider':
            dy = event.y - self.cy
            self._update_slider(dy)

    def _on_release(self, event):
        if self.active_element:
            # Stop motors
            if self.active_element.startswith('ring'):
                if self.on_motor1_change:
                    self.on_motor1_change(0.0)
            elif self.active_element == 'slider':
                # Reset slider to center
                self.slider_val = 0.0
                if self.on_motor2_change:
                    self.on_motor2_change(0.0)
            
            self.active_element = None
            self._draw()

    def _update_slider(self, dy):
        # Map dy to -1.0 to 1.0
        # Up (negative dy) -> Positive Value
        limit = self.slider_height / 2
        val = -dy / limit
        val = max(min(val, 1.0), -1.0)
        self.slider_val = val
        self._draw()
        if self.on_motor2_change:
            self.on_motor2_change(val)
class VoiceStatusWindow:
    """Tk窗口：展示语音识别及状态信息"""

    def __init__(self) -> None:
        self.enabled = tk is not None
        self.queue: queue.Queue[Dict[str, Any]] = queue.Queue()
        self.thread: Optional[threading.Thread] = None
        self.root: Any = None
        self.text_area: Any = None
        self.state_var: Any = None
        self.next_var: Any = None
        self.record_status_var: Any = None
        self.ready_event = threading.Event()
        self.stop_event = threading.Event()
        self.is_recording = False
        self.record_callback: Optional[Callable[[bool], None]] = None
        self.command_callback: Optional[Callable[[str], None]] = None
        self.motor_callback: Optional[Callable[[int, float], None]] = None
        self.exit_callback: Optional[Callable[[], None]] = None
        
        # 动作采集/导航回调
        self.action_record_callback: Optional[Callable[[bool], None]] = None
        self.action_play_callback: Optional[Callable[[str], None]] = None
        self.action_pause_callback: Optional[Callable[[], None]] = None
        self.action_stop_callback: Optional[Callable[[], None]] = None
        self.action_reverse_callback: Optional[Callable[[], None]] = None
        self.action_speed_callback: Optional[Callable[[float], None]] = None
        
        # UI Components
        self.canvas_record: Any = None
        self.record_light: Any = None
        self.record_btn_window: Any = None
        
        # 动作采集/导航UI组件
        self.action_record_btn: Any = None
        self.action_stop_record_btn: Any = None
        self.action_play_btn: Any = None
        self.action_pause_btn: Any = None
        self.action_stop_btn: Any = None
        self.action_reverse_btn: Any = None
        self.action_speed_var: Any = None
        self.action_progress_var: Any = None
        self.action_time_var: Any = None
        self.action_progress_bar: Any = None
        self.action_listbox: Any = None
        # BC 路径选择器
        self.bc_path_var: Any = None        # StringVar，存 YOLO 代码如 "LMB"
        self.bc_path_label: Any = None      # 显示当前选中路径的 Label
        self.nav_target_var: Any = None
        self.nav_layer2_var: Any = None
        self.nav_layer3_var: Any = None
        self.nav_target_menu: Any = None
        self.nav_inspection_btn: Any = None
        self.inspection_path_vars: Dict[str, Any] = {}
        self.inspection_path_menus: Dict[str, Any] = {}
        self.inspection_plan_status_var: Any = None
        self.inspection_plan_ready: bool = False
        self._nav_start_callback: Optional[Callable[..., None]] = None
        self._nav_pause_callback: Optional[Callable[[], None]] = None
        self._nav_stop_callback: Optional[Callable[[], None]] = None
        self._nav_inspection_plan_callback: Optional[Callable[..., None]] = None
        self._nav_inspection_callback: Optional[Callable[..., None]] = None
        self.collection_status_var: Any = None

    def start(self) -> None:
        if not self.enabled:
            print("Tkinter 不可用，语音状态窗口禁用")
            self.ready_event.set()
            return
        self.thread = threading.Thread(target=self._run, name="voice_status_window", daemon=True)
        self.thread.start()
        # 等待窗口初始化
        self.ready_event.wait(timeout=5)

    def _run(self) -> None:
        if not self.enabled or tk is None:
            self.ready_event.set()
            return
        assert tk is not None
        assert ScrolledText is not None
        try:
            # 启用高DPI支持 (Windows)
            try:
                from ctypes import windll
                windll.shcore.SetProcessDpiAwareness(1)
            except Exception:
                pass

            self.root = tk.Tk()
            self.root.title("Napoleon2025 - 智能控制终端")
            self.root.geometry("900x1600")
            self.root.minsize(760, 700)
            self.root.configure(bg="#f0f2f5")  # 浅灰背景，类似现代应用

            # 内容较多，使用可滚动主容器，保证现场采集和导航区在常见屏幕上均可访问。
            viewport = tk.Frame(self.root, bg="#f0f2f5")
            viewport.pack(expand=True, fill=tk.BOTH)
            content_canvas = tk.Canvas(
                viewport, bg="#f0f2f5", highlightthickness=0
            )
            content_scrollbar = tk.Scrollbar(
                viewport, orient=tk.VERTICAL, command=content_canvas.yview
            )
            content_canvas.configure(yscrollcommand=content_scrollbar.set)
            content_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
            content_canvas.pack(side=tk.LEFT, expand=True, fill=tk.BOTH)

            main_card = tk.Frame(content_canvas, bg="white", padx=20, pady=20)
            main_window = content_canvas.create_window(
                (20, 20), window=main_card, anchor="nw"
            )
            main_card.bind(
                "<Configure>",
                lambda _event: content_canvas.configure(
                    scrollregion=content_canvas.bbox("all")
                ),
            )
            content_canvas.bind(
                "<Configure>",
                lambda event: content_canvas.itemconfigure(
                    main_window, width=max(event.width - 45, 1)
                ),
            )
            content_canvas.bind_all(
                "<MouseWheel>",
                lambda event: content_canvas.yview_scroll(
                    int(-event.delta / 120), "units"
                ),
            )
            
            # 标题区域
            title_label = tk.Label(
                main_card, 
                text="Napoleon2025", 
                font=("Microsoft YaHei", 18, "bold"),
                bg="white",
                fg="#333"
            )
            title_label.pack(pady=(0, 15))

            # 状态显示区域
            status_frame = tk.Frame(main_card, bg="#f8f9fa", relief=tk.FLAT, bd=1)
            status_frame.pack(fill=tk.X, pady=10, padx=5)
            
            self.state_var = tk.StringVar(value="当前状态：未知")
            self.next_var = tk.StringVar(value="目标状态：无")
            
            tk.Label(status_frame, textvariable=self.state_var, font=("Microsoft YaHei", 14, "bold"), bg="#f8f9fa", fg="#2c3e50").pack(pady=8)
            tk.Label(status_frame, textvariable=self.next_var, font=("Microsoft YaHei", 11), bg="#f8f9fa", fg="#7f8c8d").pack(pady=5)

            # 录音区域 (Canvas绘制指示灯)
            record_frame = tk.Frame(main_card, bg="white")
            record_frame.pack(pady=20)
            
            # 指示灯 Canvas
            self.canvas_record = tk.Canvas(record_frame, width=30, height=30, bg="white", highlightthickness=0)
            self.canvas_record.pack(side=tk.LEFT, padx=10)
            self.record_light = self.canvas_record.create_oval(5, 5, 25, 25, fill="#4CAF50", outline="") # 默认绿灯

            self.record_status_var = tk.StringVar(value="未录音")
            tk.Label(record_frame, textvariable=self.record_status_var, font=("Microsoft YaHei", 11), bg="white", fg="#666").pack(side=tk.LEFT)

            # 录音按钮 (使用 RoundedButton)
            self.record_btn_window = RoundedButton(
                main_card,
                text="按住说话",
                width=200,
                height=50,
                bg="#2196F3",
                hover_bg="#1976D2",
                press_bg="#0D47A1",
                corner_radius=25
            )
            self.record_btn_window.pack(pady=15)

            # 手动控制区域
            manual_control_frame = tk.Frame(main_card, bg="white")
            manual_control_frame.pack(pady=20, fill=tk.X)
            
            # 左侧：Ring/Slider控制
            left_control = tk.Frame(manual_control_frame, bg="white")
            left_control.pack(side=tk.LEFT, padx=20)
            
            tk.Label(left_control, text="手动控制 (M1旋转 / M2俯仰)", font=("Microsoft YaHei", 10, "bold"), bg="white").pack(pady=5)
            DirectionalControlWidget(
                left_control,
                width=200,
                height=200,
                bg="white",
                on_motor1_change=lambda v: self._on_motor_control(1, v),
                on_motor2_change=lambda v: self._on_motor_control(2, v)
            ).pack()
            
            # 右侧：Arrow控制前进后退
            right_control = tk.Frame(manual_control_frame, bg="white")
            right_control.pack(side=tk.RIGHT, padx=20)
            tk.Label(right_control, text="前进/后退 (M0)", font=("Microsoft YaHei", 10, "bold"), bg="white").pack(pady=5)

            arrow_container = tk.Frame(right_control, bg="white")
            arrow_container.pack()

            ArrowButton(
                arrow_container,
                direction="up",
                width=80,
                height=60,
                bg="#4CAF50",
                hover_bg="#45a049",
                press_bg="#3d8b40",
                on_press=lambda: self._on_motor_control(0, 1.0),
                on_release=lambda: self._on_motor_control(0, 0.0)
            ).pack(pady=5)

            ArrowButton(
                arrow_container,
                direction="down",
                width=80,
                height=60,
                bg="#f44336",
                hover_bg="#d32f2f",
                press_bg="#b71c1c",
                on_press=lambda: self._on_motor_control(0, -1.0),
                on_release=lambda: self._on_motor_control(0, 0.0)
            ).pack(pady=5)

            # 控制按钮区域
            control_frame = tk.Frame(main_card, bg="white")
            control_frame.pack(pady=20, fill=tk.X)
            
            # 第一排按钮
            row1 = tk.Frame(control_frame, bg="white")
            row1.pack(pady=10)
            
            RoundedButton(row1, text="手动模式", width=100, height=40, bg="#e0e0e0", fg="#333", hover_bg="#d5d5d5", press_bg="#c0c0c0", command=lambda: self._on_command("manual")).pack(side=tk.LEFT, padx=10)
            RoundedButton(row1, text="自动模式", width=100, height=40, bg="#81D4FA", fg="#0D47A1", hover_bg="#4FC3F7", press_bg="#29B6F6", command=lambda: self._on_command("vision")).pack(side=tk.LEFT, padx=10)
            RoundedButton(row1, text="停止", width=100, height=40, bg="#FF9800", hover_bg="#F57C00", press_bg="#EF6C00", command=lambda: self._on_command("stop")).pack(side=tk.LEFT, padx=10)
            RoundedButton(row1, text="AI巡检", width=100, height=40, bg="#7B1FA2", fg="white", hover_bg="#6A1B9A", press_bg="#4A148C", command=lambda: self._on_command("ai_auto")).pack(side=tk.LEFT, padx=10)
            
            # 第二排按钮
            row2 = tk.Frame(control_frame, bg="white")
            row2.pack(pady=10)
            
            RoundedButton(row2, text="归零", width=100, height=40, bg="#e0e0e0", fg="#333", hover_bg="#d5d5d5", press_bg="#c0c0c0", command=lambda: self._on_command("zero")).pack(side=tk.LEFT, padx=10)
            RoundedButton(row2, text="设为零位", width=100, height=40, bg="#e0e0e0", fg="#333", hover_bg="#d5d5d5", press_bg="#c0c0c0", command=lambda: self._on_command("set_zero")).pack(side=tk.LEFT, padx=10)
            RoundedButton(row2, text="断电", width=100, height=40, bg="#f44336", hover_bg="#d32f2f", press_bg="#b71c1c", command=lambda: self._on_command("poweroff")).pack(side=tk.LEFT, padx=10)

            # 第三排：退出按钮
            row3 = tk.Frame(control_frame, bg="white")
            row3.pack(pady=10)
            
            RoundedButton(row3, text="退出程序", width=320, height=50, bg="#9E9E9E", fg="white", hover_bg="#757575", press_bg="#616161", command=self._on_exit_request).pack()

            # ── 自主导航区域 ──────────────────────────────────────────────
            nav_frame = tk.Frame(main_card, bg="white")
            nav_frame.pack(pady=20, fill=tk.X)
            
            tk.Label(nav_frame, text="自主导航·AutoNav", font=("Microsoft YaHei", 12, "bold"), bg="white", anchor="w").pack(fill=tk.X, pady=(0, 4))
            tk.Label(
                nav_frame,
                text="Layer 1：指定路径自主导航，或五叶段全流程自主巡检",
                font=("Microsoft YaHei", 9), bg="white", fg="#555", anchor="w",
            ).pack(fill=tk.X, pady=(0, 10))

            # 目标路径选择
            target_frame = tk.Frame(nav_frame, bg="white")
            target_frame.pack(fill=tk.X, pady=(0, 10))
            
            tk.Label(target_frame, text="导航路径:", font=("Microsoft YaHei", 10), bg="white").pack(side=tk.LEFT, padx=(0, 8))
            
            if _BC_AVAILABLE and _BC_PATHS:
                # 每个选项对应一条原始采集，Layer 1 不做多轨迹平均。
                nav_path_options = self._scan_nav_path_options()
                self.nav_target_var = tk.StringVar(value=nav_path_options[0] if nav_path_options else "")
                self.nav_target_menu = tk.OptionMenu(target_frame, self.nav_target_var, *nav_path_options)
                self.nav_target_menu.config(width=52, font=("Microsoft YaHei", 9))
                self.nav_target_menu.pack(side=tk.LEFT, padx=5)

            layer_frame = tk.Frame(nav_frame, bg="white")
            layer_frame.pack(fill=tk.X, pady=(0, 8))
            self.nav_layer2_var = tk.BooleanVar(value=False)
            self.nav_layer3_var = tk.BooleanVar(value=False)
            tk.Checkbutton(
                layer_frame, text="Layer 2 岔口修正", variable=self.nav_layer2_var,
                bg="white", activebackground="white", font=("Microsoft YaHei", 9),
            ).pack(side=tk.LEFT, padx=(0, 15))
            tk.Checkbutton(
                layer_frame, text="Layer 3 阻塞物清除", variable=self.nav_layer3_var,
                bg="white", activebackground="white", font=("Microsoft YaHei", 9),
            ).pack(side=tk.LEFT)
            tk.Label(
                layer_frame,
                text="（Layer 2/3 均为可选；Layer 3 仅在大面积阻塞物时介入）",
                font=("Microsoft YaHei", 8), bg="white", fg="#888",
            ).pack(side=tk.LEFT, padx=12)

            # 导航控制按钮
            nav_btn_frame = tk.Frame(nav_frame, bg="white")
            nav_btn_frame.pack(fill=tk.X, pady=5)
            
            self.nav_start_btn = RoundedButton(
                nav_btn_frame,
                text="开始导航",
                width=100,
                height=40,
                bg="#4CAF50",
                hover_bg="#45a049",
                press_bg="#3d8b40",
                command=self._on_nav_start
            )
            self.nav_start_btn.pack(side=tk.LEFT, padx=5)

            self.nav_inspection_btn = RoundedButton(
                nav_btn_frame,
                text="开始全流程巡检",
                width=200,
                height=40,
                bg="#1565C0",
                hover_bg="#0D47A1",
                press_bg="#08306B",
                command=self._on_full_inspection_start,
            )
            self.nav_inspection_btn.pack(side=tk.LEFT, padx=5)
            
            self.nav_pause_btn = RoundedButton(
                nav_btn_frame,
                text="暂停",
                width=80,
                height=35,
                bg="#FF9800",
                hover_bg="#F57C00",
                press_bg="#EF6C00",
                command=self._on_nav_pause
            )
            self.nav_pause_btn.pack(side=tk.LEFT, padx=5)
            
            self.nav_stop_btn = RoundedButton(
                nav_btn_frame,
                text="停止",
                width=80,
                height=35,
                bg="#9E9E9E",
                hover_bg="#757575",
                press_bg="#616161",
                command=self._on_nav_stop
            )
            self.nav_stop_btn.pack(side=tk.LEFT, padx=5)

            # 导航进度显示
            nav_progress_frame = tk.Frame(nav_frame, bg="#f8f9fa", relief=tk.RIDGE, bd=1)
            nav_progress_frame.pack(fill=tk.X, pady=10)
            
            self.nav_status_var = tk.StringVar(value="状态: 待命")
            tk.Label(nav_progress_frame, textvariable=self.nav_status_var, 
                    font=("Microsoft YaHei", 10), bg="#f8f9fa", anchor="w").pack(fill=tk.X, padx=10, pady=3)
            
            self.nav_progress_var = tk.StringVar(value="进度: 0% | 步骤 0/0")
            tk.Label(nav_progress_frame, textvariable=self.nav_progress_var, 
                    font=("Microsoft YaHei", 9), bg="#f8f9fa", fg="#666").pack(fill=tk.X, padx=10, pady=2)
            
            self.nav_progress_bar = tk.Canvas(nav_progress_frame, height=16, bg="#e0e0e0", highlightthickness=0)
            self.nav_progress_bar.pack(fill=tk.X, padx=10, pady=5)
            self._draw_nav_progress(0.0)
            
            self.nav_info_var = tk.StringVar(
                value="全流程顺序：左上→左下→右上→右中→右下→返回 TR 原点"
            )
            tk.Label(nav_progress_frame, textvariable=self.nav_info_var, 
                    font=("Microsoft YaHei", 8), bg="#f8f9fa", fg="#888").pack(fill=tk.X, padx=10, pady=(0, 5))
            # ──────────────────────────────────────────────────────────

            # 日志区域
            log_frame = tk.Frame(main_card, bg="white")
            log_frame.pack(expand=True, fill=tk.BOTH, pady=10)
            
            tk.Label(log_frame, text="运行日志", font=("Microsoft YaHei", 10, "bold"), bg="white", anchor="w").pack(fill=tk.X)
            
            self.text_area = ScrolledText(log_frame, wrap=tk.WORD, font=("Consolas", 10), height=10, bg="#f5f5f5", relief=tk.FLAT)
            self.text_area.pack(expand=True, fill=tk.BOTH, pady=5)
            self.text_area.configure(state=tk.DISABLED)

            # ── 现场路径采集区域 ────────────────────────────────────────
            collection_frame = tk.Frame(main_card, bg="#f4f8ff", relief=tk.RIDGE, bd=1)
            collection_frame.pack(pady=(10, 20), fill=tk.X)
            tk.Label(
                collection_frame, text="现场路径采集", font=("Microsoft YaHei", 12, "bold"),
                bg="#f4f8ff", anchor="w",
            ).pack(fill=tk.X, padx=10, pady=(8, 2))
            tk.Label(
                collection_frame,
                text="当固定起点或装配位置改变时，在现场重新采集一条从原点到目标部位的动作路径。",
                font=("Microsoft YaHei", 8), bg="#f4f8ff", fg="#666", anchor="w",
            ).pack(fill=tk.X, padx=10, pady=(0, 6))

            collection_controls = tk.Frame(collection_frame, bg="#f4f8ff")
            collection_controls.pack(fill=tk.X, padx=10, pady=4)
            tk.Label(
                collection_controls, text="目标部位:", font=("Microsoft YaHei", 10),
                bg="#f4f8ff",
            ).pack(side=tk.LEFT, padx=(0, 8))
            collection_options = [
                f"{code} - {name}"
                for code, name in _BC_PATHS.values()
                if code not in ("EXP", "TR")
            ] if _BC_AVAILABLE else []
            self.bc_path_var = tk.StringVar(
                value=collection_options[0] if collection_options else ""
            )
            collection_menu = tk.OptionMenu(
                collection_controls, self.bc_path_var, *collection_options
            )
            collection_menu.config(width=20, font=("Microsoft YaHei", 9))
            collection_menu.pack(side=tk.LEFT, padx=5)

            self.action_record_btn = RoundedButton(
                collection_controls, text="开始采集", width=100, height=36,
                bg="#1976D2", hover_bg="#1565C0", press_bg="#0D47A1",
                command=lambda: self._on_action_record(True),
            )
            self.action_record_btn.pack(side=tk.LEFT, padx=(15, 5))
            self.action_stop_record_btn = RoundedButton(
                collection_controls, text="结束并保存", width=110, height=36,
                bg="#E53935", hover_bg="#D32F2F", press_bg="#B71C1C",
                command=lambda: self._on_action_record(False),
            )
            self.action_stop_record_btn.pack(side=tk.LEFT, padx=5)

            self.collection_status_var = tk.StringVar(
                value="状态：待命｜文件保存到 BC/expert_demos/AutoNavdatasets"
            )
            tk.Label(
                collection_frame, textvariable=self.collection_status_var,
                font=("Microsoft YaHei", 9), bg="#f4f8ff", fg="#555", anchor="w",
            ).pack(fill=tk.X, padx=10, pady=(3, 8))

            # ── 全流程轨迹设定 ──────────────────────────────────────────
            # 每个叶段由操作员明确选择一条现场轨迹。
            inspection_frame = tk.LabelFrame(
                main_card,
                text="全流程自主巡检设定（顺序固定，轨迹可选）",
                font=("Microsoft YaHei", 10, "bold"),
                bg="#f4f8ff",
                padx=10,
                pady=8,
            )
            inspection_frame.pack(fill=tk.X, pady=(0, 20))
            inspection_options = self._scan_inspection_path_options()
            for row_index, (code, _label, name) in enumerate(
                _DEFAULT_INSPECTION_ROUTE
            ):
                tk.Label(
                    inspection_frame,
                    text=f"{row_index + 1}. {name} [{code}]：",
                    width=16,
                    anchor="e",
                    font=("Microsoft YaHei", 9),
                    bg="#f4f8ff",
                ).grid(row=row_index, column=0, padx=(0, 6), pady=2)
                options = inspection_options.get(code, [])
                display_options = options or ["（无可用轨迹）"]
                variable = tk.StringVar(value=display_options[-1])
                menu = tk.OptionMenu(
                    inspection_frame,
                    variable,
                    *display_options,
                    command=lambda _value, selected_code=code:
                        self._on_inspection_selection_changed(selected_code),
                )
                menu.config(width=46, font=("Microsoft YaHei", 8))
                menu.grid(row=row_index, column=1, sticky="ew", pady=2)
                self.inspection_path_vars[code] = variable
                self.inspection_path_menus[code] = menu
            inspection_frame.grid_columnconfigure(1, weight=1)

            inspection_action_frame = tk.Frame(inspection_frame, bg="#f4f8ff")
            inspection_action_frame.grid(
                row=len(_DEFAULT_INSPECTION_ROUTE),
                column=0,
                columnspan=2,
                sticky="ew",
                pady=(8, 2),
            )
            RoundedButton(
                inspection_action_frame,
                text="规划全流程",
                width=110,
                height=36,
                bg="#00897B",
                hover_bg="#00796B",
                press_bg="#00695C",
                command=self._on_inspection_plan,
            ).pack(side=tk.LEFT, padx=(0, 10))
            tk.Checkbutton(
                inspection_action_frame,
                text="启用 Layer 2 近中心微调",
                variable=self.nav_layer2_var,
                bg="#f4f8ff",
                activebackground="#f4f8ff",
                font=("Microsoft YaHei", 8),
            ).pack(side=tk.LEFT, padx=(0, 10))
            tk.Checkbutton(
                inspection_action_frame,
                text="启用 Layer 3 阻塞物清除",
                variable=self.nav_layer3_var,
                bg="#f4f8ff",
                activebackground="#f4f8ff",
                font=("Microsoft YaHei", 8),
            ).pack(side=tk.LEFT, padx=(0, 10))
            self.inspection_plan_status_var = tk.StringVar(
                value="尚未规划：请选择五条轨迹后点击“规划全流程”"
            )
            tk.Label(
                inspection_action_frame,
                textvariable=self.inspection_plan_status_var,
                anchor="w",
                font=("Microsoft YaHei", 8),
                bg="#f4f8ff",
                fg="#555",
            ).pack(side=tk.LEFT, expand=True, fill=tk.X)

            # ── 图像识别模式切换（固定放在整个 UI 最底端）───────────────
            tracking_mode_frame = tk.LabelFrame(
                main_card,
                text="图像识别模式",
                font=("Microsoft YaHei", 9, "bold"),
                bg="#f4f8ff",
                padx=8,
                pady=5,
            )
            tracking_mode_frame.pack(fill=tk.X, pady=(0, 20))
            tk.Label(
                tracking_mode_frame,
                text="键盘 1/2/3 或点击：",
                font=("Microsoft YaHei", 8),
                bg="#f4f8ff",
                fg="#555",
            ).pack(side=tk.LEFT, padx=(0, 6))
            for mode, text, color, hover, pressed in (
                (1, "1 岔口", "#64B5F6", "#42A5F5", "#1E88E5"),
                (2, "2 阻塞物", "#FFB74D", "#FFA726", "#FB8C00"),
                (3, "3 混合", "#80CBC4", "#4DB6AC", "#26A69A"),
            ):
                RoundedButton(
                    tracking_mode_frame,
                    text=text,
                    width=78,
                    height=28,
                    bg=color,
                    fg="#17324D",
                    hover_bg=hover,
                    press_bg=pressed,
                    command=lambda selected_mode=mode:
                        self._on_tracking_mode(selected_mode),
                ).pack(side=tk.LEFT, padx=3)

            self.ready_event.set()
            self._poll_queue()
            self.root.mainloop()
        except Exception as exc:
            print(f"语音窗口创建失败：{exc}")
            self.ready_event.set()

    def _poll_queue(self) -> None:
        if self.stop_event.is_set() or self.root is None:
            if self.root:
                self.root.destroy()
            return
        try:
            while True:
                item = self.queue.get_nowait()
                self._handle_item(item)
        except queue.Empty:
            pass
        if self.root:
            self.root.after(100, self._poll_queue)

    def push_log(self, text: str) -> None:
        if not self.enabled:
            print(f"[日志] {text}")
            return
        self.queue.put({"type": "log", "text": text})

    def update_state(self, current: str, next_state: Optional[str] = None) -> None:
        if not self.enabled:
            print(f"[状态] 当前：{current}, 目标：{next_state or '无'}")
            return
        self.queue.put({"type": "state", "current": current, "next": next_state})
    
    def set_record_callback(self, callback: Callable[[bool], None]) -> None:
        """设置录音按钮的回调函数"""
        self.record_callback = callback
        if self.record_btn_window and callback:
            self.record_btn_window.bind("<ButtonPress-1>", lambda e: callback(True))
            self.record_btn_window.bind("<ButtonRelease-1>", lambda e: callback(False))
            
    def set_command_callback(self, callback: Callable[[str], None]) -> None:
        """设置控制按钮的回调函数"""
        self.command_callback = callback

    def _on_command(self, cmd: str) -> None:
        """内部处理按钮点击"""
        if self.command_callback:
            self.command_callback(cmd)

    def _on_tracking_mode(self, mode: int) -> None:
        """从 UI 切换视觉线程的 1/2/3 识别模式。"""
        mode_names = {
            1: "岔口追踪",
            2: "阻塞物追踪/清除",
            3: "阻塞物优先混合追踪",
        }
        if set_tracking_mode(mode):
            self.push_log(
                f"[图控] 已切换模式 {mode}：{mode_names.get(mode, '未知')}"
            )
    
    def set_motor_callback(self, callback: Callable[[int, float], None]) -> None:
        """设置电机控制回调函数"""
        self.motor_callback = callback
    
    def set_exit_callback(self, callback: Callable[[], None]) -> None:
        """设置退出回调函数"""
        self.exit_callback = callback
    
    def _on_motor_control(self, motor_id: int, value: float) -> None:
        """内部处理电机控制"""
        if self.motor_callback:
            self.motor_callback(motor_id, value)
    
    def _on_exit_request(self) -> None:
        """内部处理退出请求"""
        if self.exit_callback:
            self.exit_callback()
    
    def update_recording_status(self, is_recording: bool) -> None:
        """更新录音状态显示"""
        if not self.enabled:
            return
        self.queue.put({"type": "recording", "status": is_recording})
    
    def set_action_record_callback(self, callback: Callable[[bool], None]) -> None:
        """设置动作采集回调"""
        self.action_record_callback = callback
    
    def set_action_play_callback(self, callback: Callable[[str], None]) -> None:
        """设置动作播放回调"""
        self.action_play_callback = callback
    
    def set_action_pause_callback(self, callback: Callable[[], None]) -> None:
        """设置动作暂停回调"""
        self.action_pause_callback = callback
    
    def set_action_stop_callback(self, callback: Callable[[], None]) -> None:
        """设置动作停止回调"""
        self.action_stop_callback = callback
    
    def set_action_reverse_callback(self, callback: Callable[[], None]) -> None:
        """设置动作倒放回调"""
        self.action_reverse_callback = callback
    
    def set_action_speed_callback(self, callback: Callable[[float], None]) -> None:
        """设置动作倍速回调"""
        self.action_speed_callback = callback
    
    def _on_action_record(self, start: bool) -> None:
        """动作采集按钮回调"""
        if self.action_record_callback:
            self.action_record_callback(start)
    
    def _on_action_play(self) -> None:
        """动作播放按钮回调"""
        if self.action_play_callback:
            # 获取选中的动作文件
            selection = self.action_listbox.curselection()
            if selection:
                file_path = self.action_listbox.get(selection[0])
                # 提取文件路径（去除显示文本中的时间信息）
                if '|' in file_path:
                    file_path = file_path.split('|')[0].strip()
                self.action_play_callback(file_path)
            else:
                self.push_log("请先选择一个动作文件")
    
    def _on_action_pause(self) -> None:
        """动作暂停按钮回调"""
        if self.action_pause_callback:
            self.action_pause_callback()
    
    def _on_action_stop(self) -> None:
        """动作停止按钮回调"""
        if self.action_stop_callback:
            self.action_stop_callback()
    
    def _on_action_reverse(self) -> None:
        """动作倒放按钮回调"""
        if self.action_reverse_callback:
            self.action_reverse_callback()
    
    def _on_speed_change(self, value: Any) -> None:
        """倍速选择回调"""
        if self.action_speed_callback:
            # OptionMenu的command会传递StringVar，需要获取其值
            if hasattr(value, 'get'):
                value_str = value.get()
            else:
                value_str = str(value)
            speed = float(value_str.replace('x', ''))
            self.action_speed_callback(speed)
    
    def _on_action_listbox_select(self, event) -> None:
        """动作列表双击选择回调"""
        selection = self.action_listbox.curselection()
        if selection:
            file_path = self.action_listbox.get(selection[0])
            if '|' in file_path:
                file_path = file_path.split('|')[0].strip()
            if self.action_play_callback:
                self.action_play_callback(file_path)
    
    def update_action_progress(self, progress_data: Dict) -> None:
        """更新动作导航进度"""
        if not self.enabled:
            return
        self.queue.put({"type": "action_progress", "data": progress_data})
    
    def update_action_list(self, actions: List[Tuple[str, str]]) -> None:
        """更新动作列表"""
        if not self.enabled:
            return
        self.queue.put({"type": "action_list", "actions": actions})

    def update_collection_status(self, text: str, refresh_paths: bool = False) -> None:
        """线程安全更新现场采集状态，并可刷新导航路径列表。"""
        if not self.enabled:
            return
        self.queue.put({
            "type": "collection_status", "text": text, "refresh": refresh_paths
        })

    def update_inspection_plan_status(
        self,
        text: str,
        success: bool = False,
        planned_selection: Optional[Dict[str, str]] = None,
    ) -> None:
        """线程安全更新全流程规划结果。"""
        if not self.enabled:
            return
        self.queue.put({
            "type": "inspection_plan_status",
            "text": text,
            "success": bool(success),
            "planned_selection": dict(planned_selection or {}),
        })

    @staticmethod
    def _scan_nav_path_options() -> List[str]:
        """扫描 AutoNavdatasets，返回逐条原始路径的 UI 选项。"""
        demo_dir = os.path.join(
            os.path.dirname(__file__), "BC", "expert_demos", "AutoNavdatasets"
        )
        options = ["TR - 固定出发原点 | M0/M1/M2 全部导航至 0°"]
        for filename in sorted(os.listdir(demo_dir)) if os.path.isdir(demo_dir) else []:
            if not filename.lower().endswith(".json"):
                continue
            stem_parts = os.path.splitext(filename)[0].split("_")
            code = stem_parts[-2].upper() if len(stem_parts) >= 2 else ""
            goal_id = YOLO_CODE_TO_IDX.get(code)
            if goal_id is None or code in ("EXP", "TR"):
                continue
            options.append(f"{code} - {_BC_NAMES.get(goal_id, code)} | {filename}")
        return options

    @staticmethod
    def _scan_inspection_path_options() -> Dict[str, List[str]]:
        """按五个叶段扫描可用于全流程拼接的单条轨迹。"""
        demo_dir = os.path.join(
            os.path.dirname(__file__), "BC", "expert_demos", "AutoNavdatasets"
        )
        result: Dict[str, List[str]] = {}
        for code, label, _name in _DEFAULT_INSPECTION_ROUTE:
            if _list_inspection_source_files is None:
                result[code] = []
                continue
            result[code] = [
                os.path.basename(path)
                for path in _list_inspection_source_files(demo_dir, code, label)
            ]
        return result

    def _refresh_nav_path_menu(self) -> None:
        """在 Tk 线程中刷新导航下拉菜单，保留仍然存在的当前选择。"""
        if self.nav_target_menu is None or self.nav_target_var is None or tk is None:
            return
        options = self._scan_nav_path_options()
        current = self.nav_target_var.get()
        menu = self.nav_target_menu["menu"]
        menu.delete(0, "end")
        for option in options:
            menu.add_command(label=option, command=lambda value=option: self.nav_target_var.set(value))
        self.nav_target_var.set(current if current in options else options[-1])
        self._refresh_inspection_path_menus()

    def _refresh_inspection_path_menus(self) -> None:
        """刷新五个巡检轨迹下拉框，并尽量保留操作员当前选择。"""
        if tk is None or not self.inspection_path_menus:
            return
        option_map = self._scan_inspection_path_options()
        selection_changed = False
        for code, _label, _name in _DEFAULT_INSPECTION_ROUTE:
            variable = self.inspection_path_vars.get(code)
            widget = self.inspection_path_menus.get(code)
            if variable is None or widget is None:
                continue
            options = option_map.get(code, []) or ["（无可用轨迹）"]
            current = variable.get()
            menu = widget["menu"]
            menu.delete(0, "end")
            for option in options:
                menu.add_command(
                    label=option,
                    command=lambda value=option, selected_code=code:
                        self._set_inspection_selection(selected_code, value),
                )
            if current not in options:
                variable.set(options[-1])
                selection_changed = True
        if selection_changed:
            self._mark_inspection_plan_stale()

    def _set_inspection_selection(self, code: str, value: str) -> None:
        variable = self.inspection_path_vars.get(code)
        if variable is not None:
            variable.set(value)
        self._on_inspection_selection_changed(code)

    def _on_inspection_selection_changed(self, _code: str) -> None:
        self._mark_inspection_plan_stale()

    def _mark_inspection_plan_stale(self) -> None:
        self.inspection_plan_ready = False
        if self.inspection_plan_status_var is not None:
            self.inspection_plan_status_var.set(
                "选择已变化：请重新点击“规划全流程”"
            )

    def _get_inspection_selections(self) -> Dict[str, str]:
        return {
            code: str(self.inspection_path_vars[code].get()).strip()
            for code, _label, _name in _DEFAULT_INSPECTION_ROUTE
            if code in self.inspection_path_vars
        }
    
    def _draw_progress_bar(self, progress: float) -> None:
        """绘制动作导航进度条"""
        if not self.action_progress_bar:
            return
        self.action_progress_bar.delete("all")
        width = self.action_progress_bar.winfo_width()
        if width < 10:
            width = 300  # 默认宽度
        height = 20
        progress_width = int(width * progress / 100.0)

        # 绘制背景
        self.action_progress_bar.create_rectangle(0, 0, width, height, fill="#e0e0e0", outline="")
        # 绘制进度
        if progress > 0:
            self.action_progress_bar.create_rectangle(0, 0, progress_width, height, fill="#4CAF50", outline="")

    def _draw_nav_progress(self, progress: float) -> None:
        """绘制导航进度条"""
        if not self.nav_progress_bar:
            return
        self.nav_progress_bar.delete("all")
        width = self.nav_progress_bar.winfo_width()
        if width < 10:
            width = 300
        height = 16
        progress_width = int(width * progress / 100.0)

        # 背景
        self.nav_progress_bar.create_rectangle(0, 0, width, height, fill="#e0e0e0", outline="")
        # 进度
        if progress > 0:
            self.nav_progress_bar.create_rectangle(0, 0, progress_width, height, fill="#4CAF50", outline="")

    def set_nav_callbacks(
        self,
        start_cb,
        pause_cb,
        stop_cb,
        inspection_plan_cb=None,
        inspection_cb=None,
    ) -> None:
        """设置导航控制回调"""
        self._nav_start_callback = start_cb
        self._nav_pause_callback = pause_cb
        self._nav_stop_callback = stop_cb
        self._nav_inspection_plan_callback = inspection_plan_cb
        self._nav_inspection_callback = inspection_cb

    def _on_nav_start(self) -> None:
        """开始导航按钮回调"""
        if self._nav_start_callback:
            # 从下拉菜单获取目标路径
            target_text = self.nav_target_var.get() if hasattr(self, 'nav_target_var') else ""
            enable_layer2 = bool(self.nav_layer2_var.get()) if self.nav_layer2_var is not None else False
            enable_layer3 = bool(self.nav_layer3_var.get()) if self.nav_layer3_var is not None else False
            self._nav_start_callback(target_text, enable_layer2, enable_layer3)

    def _on_inspection_plan(self) -> None:
        """提交当前五条轨迹选择，并在后台完成公共点规划。"""
        selections = self._get_inspection_selections()
        self.inspection_plan_ready = False
        if self.inspection_plan_status_var is not None:
            self.inspection_plan_status_var.set("正在规划并验证公共点，请稍候…")
        if self._nav_inspection_plan_callback:
            threading.Thread(
                target=self._nav_inspection_plan_callback,
                args=(selections,),
                name="inspection_route_planner",
                daemon=True,
            ).start()

    def _on_full_inspection_start(self) -> None:
        """全流程自主巡检按钮回调。"""
        if not self.inspection_plan_ready:
            if self.inspection_plan_status_var is not None:
                self.inspection_plan_status_var.set(
                    "尚未完成有效规划：请先点击“规划全流程”"
                )
            self.push_log("[自主巡检] 请先完成全流程路径规划")
            return
        if self._nav_inspection_callback:
            enable_layer2 = (
                bool(self.nav_layer2_var.get())
                if self.nav_layer2_var is not None else False
            )
            enable_layer3 = (
                bool(self.nav_layer3_var.get())
                if self.nav_layer3_var is not None else False
            )
            self._nav_inspection_callback(
                self._get_inspection_selections(),
                enable_layer2,
                enable_layer3,
            )

    def _on_nav_pause(self) -> None:
        """暂停导航按钮回调"""
        if self._nav_pause_callback:
            self._nav_pause_callback()

    def _on_nav_stop(self) -> None:
        """停止导航按钮回调"""
        if self._nav_stop_callback:
            self._nav_stop_callback()
    
    def _handle_item(self, item: Dict[str, Any]) -> None:
        if self.root is None or tk is None:
            return
        itype = item.get("type")
        if itype == "log" and self.text_area:
            self.text_area.configure(state=tk.NORMAL)
            self.text_area.insert(tk.END, f"[{datetime.now().strftime('%H:%M:%S')}] " + item.get("text", "") + "\n")
            self.text_area.see(tk.END)
            self.text_area.configure(state=tk.DISABLED)
        elif itype == "state":
            if self.state_var:
                self.state_var.set(f"当前状态：{item.get('current', '未知')}")
            if self.next_var:
                target = item.get("next") or "无"
                self.next_var.set(f"目标状态：{target}")
        elif itype == "recording":
            self.is_recording = item.get("status", False)
            if self.record_status_var and self.canvas_record:
                if self.is_recording:
                    self.record_status_var.set("正在录音...")
                    self.canvas_record.itemconfig(self.record_light, fill="#f44336") # 红灯
                    if self.record_btn_window:
                        self.record_btn_window.config(bg="#f44336", text="松开结束")
                else:
                    self.record_status_var.set("未录音")
                    self.canvas_record.itemconfig(self.record_light, fill="#4CAF50") # 绿灯
                    if self.record_btn_window:
                        self.record_btn_window.config(bg="#2196F3", text="按住说话")
        elif itype == "action_progress":
            data = item.get("data", {})
            progress = data.get("progress", 0.0)
            elapsed = data.get("elapsed", 0.0)
            remaining = data.get("remaining", 0.0)
            total = data.get("total", 0.0)
            
            if self.action_progress_var:
                self.action_progress_var.set(f"{progress:.1f}%")
            if self.action_time_var:
                self.action_time_var.set(f"总时长: {total:.1f}s | 已播放: {elapsed:.1f}s | 剩余: {remaining:.1f}s")
            if self.action_progress_bar:
                self._draw_progress_bar(progress)
        elif itype == "action_list":
            if self.action_listbox:
                self.action_listbox.delete(0, tk.END)
                for file_path, display_text in item.get("actions", []):
                    self.action_listbox.insert(tk.END, f"{file_path} | {display_text}")
        elif itype == "collection_status":
            if self.collection_status_var:
                self.collection_status_var.set(item.get("text", ""))
            if item.get("refresh", False):
                self._refresh_nav_path_menu()
        elif itype == "inspection_plan_status":
            planned_selection = item.get("planned_selection", {})
            selection_matches = (
                planned_selection == self._get_inspection_selections()
                if planned_selection else True
            )
            self.inspection_plan_ready = bool(
                item.get("success", False) and selection_matches
            )
            if self.inspection_plan_status_var:
                text = item.get("text", "")
                if item.get("success", False) and not selection_matches:
                    text = "规划期间选择已变化：请重新点击“规划全流程”"
                self.inspection_plan_status_var.set(text)
        elif itype == "nav_progress":
            data = item.get("data", {})
            status = data.get("status", "待命")
            progress = data.get("progress", 0.0)
            step = data.get("step", 0)
            total = data.get("total", 0)
            info = data.get("info", "")
            
            if self.nav_status_var:
                self.nav_status_var.set(f"状态: {status}")
            if self.nav_progress_var:
                self.nav_progress_var.set(f"进度: {progress:.1f}% | 步骤 {step}/{total}")
            if self.nav_progress_bar:
                self._draw_nav_progress(progress)
            if self.nav_info_var:
                self.nav_info_var.set(info)
        elif itype == "close":
            self.stop_event.set()

    def stop(self) -> None:
        if not self.enabled:
            return
        self.queue.put({"type": "close"})
        self.stop_event.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)


class VoiceCommandCenter:
    """语音识别：手动录音模式（支持百度语音识别和 speech_recognition）"""

    def __init__(self, window: Optional[VoiceStatusWindow]) -> None:
        self.window = window
        self.command_queue: queue.Queue[Tuple[str, str]] = queue.Queue()
        self.stop_event = threading.Event()
        # 手动录音相关
        self.manual_record_event = threading.Event()
        self.manual_record_stop_event = threading.Event()
        self.manual_record_audio: Optional[bytes] = None
        self.manual_record_audio_obj: Any = None  # 用于speech_recognition的AudioData对象
        self.manual_record_lock = threading.Lock()
        self.manual_recording_thread: Optional[threading.Thread] = None
        
        # 检查使用哪种识别方式
        self.use_baidu = False
        self.baidu_client: Any = None
        self.recognizer: Any = None
        self.microphone: Any = None
        self._sr: Any = None
        self._pyaudio: Any = None
        self.mic_device_index: Optional[int] = VOICE_MIC_DEVICE_INDEX
        
        # 优先使用百度语音识别
        if baidu_speech and AipSpeech and BAIDU_APP_ID and BAIDU_API_KEY and BAIDU_SECRET_KEY:
            try:
                self.baidu_client = AipSpeech(BAIDU_APP_ID, BAIDU_API_KEY, BAIDU_SECRET_KEY)
                self.use_baidu = True
                if pyaudio:
                    self._pyaudio = pyaudio
            except Exception as exc:
                if self.window:
                    self.window.push_log(f"百度语音识别初始化失败：{exc}，将使用备用方案")
                self.use_baidu = False
        
        # 备用方案：speech_recognition
        if not self.use_baidu:
            if sr is None or pyaudio is None:
                raise RuntimeError(
                    "未安装语音识别库。请选择以下方案之一：\n"
                    "1. 安装百度语音识别：pip install baidu-aip pyaudio\n"
                    "2. 安装备用方案：pip install speechrecognition pyaudio\n"
                    "并在代码中配置百度 API 密钥（推荐）"
                )
            self._sr = sr
            self.recognizer = self._sr.Recognizer()
            mic_kwargs: Dict[str, Any] = {}
            if VOICE_MIC_DEVICE_INDEX is not None:
                mic_kwargs["device_index"] = VOICE_MIC_DEVICE_INDEX
            self.microphone = self._sr.Microphone(**mic_kwargs)
            self._pyaudio = pyaudio
            if self.window and VOICE_MIC_DEVICE_INDEX is not None:
                self.window.push_log(f"已锁定麦克风设备索引：{VOICE_MIC_DEVICE_INDEX}")

    def start(self) -> None:
        if self.window:
            self.window.push_log("语音模块已就绪")
            if self.mic_device_index is not None:
                self.window.push_log(f"当前使用的麦克风设备索引：{self.mic_device_index}")
        
        if self.use_baidu:
            if self.window:
                self.window.push_log("使用百度语音识别（手动模式）")
        else:
            if self.microphone:
                try:
                    with self.microphone as source:
                        self.recognizer.adjust_for_ambient_noise(source, duration=1)
                except Exception:
                    pass
            if self.window:
                self.window.push_log("使用备用识别方案（手动模式）")

    def stop(self) -> None:
        self.stop_event.set()

    def _record_audio_baidu(self, duration: float = 2.0, stop_event: Optional[threading.Event] = None) -> Optional[bytes]:
        """使用 pyaudio 采集音频（百度格式：16k采样率，16bit，单声道）"""
        if not self._pyaudio:
            return None
        p = None
        stream = None
        try:
            chunk = 1024
            sample_format = self._pyaudio.paInt16
            channels = 1
            fs = 16000  # 百度要求16k采样率
            
            p = self._pyaudio.PyAudio()
            stream_kwargs = dict(
                format=sample_format,
                channels=channels,
                rate=fs,
                frames_per_buffer=chunk,
                input=True,
            )
            if self.mic_device_index is not None:
                stream_kwargs["input_device_index"] = self.mic_device_index
            stream = p.open(**stream_kwargs)
            
            frames = []
            if stop_event:
                # 手动录音模式：持续录音直到stop_event被设置
                # 使用非阻塞读取，以便能及时响应stop_event
                while not stop_event.is_set():
                    try:
                        data = stream.read(chunk, exception_on_overflow=False)
                        if data:
                            frames.append(data)
                    except Exception as e:
                        # 读取错误，继续尝试
                        if self.window:
                            self.window.push_log(f"录音读取警告：{e}")
                        break
                
                # 停止事件已设置，再读取一次以确保获取所有缓冲数据
                try:
                    remaining = stream.get_read_available()
                    if remaining > 0:
                        data = stream.read(remaining, exception_on_overflow=False)
                        if data:
                            frames.append(data)
                except:
                    pass
            else:
                # 自动录音模式：按duration时长录音
                for _ in range(0, int(fs / chunk * duration)):
                    data = stream.read(chunk)
                    frames.append(data)
            
            if stream:
                stream.stop_stream()
                stream.close()
            if p:
                p.terminate()
            
            audio_data = b''.join(frames)
            return audio_data if len(audio_data) > 0 else None
        except Exception as exc:
            if self.window:
                self.window.push_log(f"录音失败：{exc}")
            try:
                if stream:
                    stream.stop_stream()
                    stream.close()
                if p:
                    p.terminate()
            except:
                pass
            return None
    
    def start_manual_record(self) -> None:
        """开始手动录音"""
        if self.use_baidu:
            if not self._pyaudio:
                if self.window:
                    self.window.push_log("录音功能不可用：缺少pyaudio库")
                return
        else:
            if not self._sr:
                if self.window:
                    self.window.push_log("录音功能不可用：缺少speech_recognition库")
                return
            if not self._pyaudio:
                if self.window:
                    self.window.push_log("录音功能不可用：缺少pyaudio库")
                return
        
        self.manual_record_stop_event.clear()
        self.manual_record_audio = None
        self.manual_record_audio_obj = None
        
        # 在后台线程中录音
        def record_thread():
            if self.window:
                self.window.update_recording_status(True)
            
            try:
                if self._pyaudio:
                    audio = self._record_audio_baidu(duration=0, stop_event=self.manual_record_stop_event)
                else:
                    audio = None
                if self.use_baidu:
                    with self.manual_record_lock:
                        self.manual_record_audio = audio
                    if self.window and audio:
                        self.window.push_log(f"录音完成，长度: {len(audio)} 字节")
                else:
                    if self.window:
                        self.window.push_log("已连接到麦克风，开始录音...")
                    if audio:
                        audio_obj = self._sr.AudioData(audio, 16000, 2)
                        with self.manual_record_lock:
                            self.manual_record_audio_obj = audio_obj
                        if self.window:
                            self.window.push_log(f"录音完成，长度 {len(audio)} 字节")
                    else:
                        with self.manual_record_lock:
                            self.manual_record_audio_obj = None
                        if self.window:
                            self.window.push_log("录音时间太短，未捕获到有效音频")
            finally:
                if self.window:
                    self.window.update_recording_status(False)
        
        self.manual_recording_thread = threading.Thread(target=record_thread, daemon=True)
        self.manual_recording_thread.start()
    
    def stop_manual_record(self) -> None:
        """停止手动录音并识别"""
        # 设置停止事件
        self.manual_record_stop_event.set()
        
        # 等待录音线程完成，增加等待时间到3秒
        if self.manual_recording_thread and self.manual_recording_thread.is_alive():
            self.manual_recording_thread.join(timeout=3.0)
            if self.manual_recording_thread.is_alive():
                if self.window:
                    self.window.push_log("警告：录音线程未在预期时间内完成")
        
        # 读取录音数据
        with self.manual_record_lock:
            audio = self.manual_record_audio
            audio_obj = self.manual_record_audio_obj
            self.manual_record_audio = None
            self.manual_record_audio_obj = None
        
        # 处理录音数据
        if self.use_baidu:
            if not audio or len(audio) == 0:
                if self.window:
                    self.window.push_log("录音为空，请重新尝试（可能录音时间太短）")
                return
            
            # 检查最小录音长度（至少0.5秒，约16000*0.5=8000字节）
            min_audio_length = 8000
            if len(audio) < min_audio_length:
                if self.window:
                    self.window.push_log(f"录音太短（{len(audio)}字节），请至少录音0.5秒")
                return
            
            # 使用百度识别
            if self.window:
                self.window.push_log("正在识别...")
            try:
                text = self._recognize_baidu(audio)
                if text:
                    self._handle_text(text.strip(), force_process=True)
                else:
                    if self.window:
                        self.window.push_log("未识别到有效语音")
            except Exception as exc:
                if self.window:
                    self.window.push_log(f"识别失败：{exc}")
        elif self._sr:
            if not audio_obj:
                if self.window:
                    self.window.push_log("录音为空，请重新尝试（可能录音时间太短）")
                return
            
            # 使用speech_recognition识别（带重试机制）
            if self.window:
                self.window.push_log("正在识别...")
            
            # 重试机制：最多重试3次
            max_retries = 3
            retry_count = 0
            text = None
            
            while retry_count < max_retries:
                try:
                    # 设置超时时间，避免长时间等待
                    text = self.recognizer.recognize_google(
                        audio_obj, 
                        language=VOICE_RECOGNITION_LANGUAGE,
                        show_all=False
                    )
                    if text:
                        break  # 成功识别，退出重试循环
                except self._sr.UnknownValueError:
                    # 无法识别语音内容，不需要重试
                    if self.window:
                        self.window.push_log("未识别到有效语音")
                    break
                except self._sr.RequestError as exc:
                    retry_count += 1
                    error_msg = str(exc)
                    if self.window:
                        if retry_count < max_retries:
                            self.window.push_log(f"识别服务错误（重试 {retry_count}/{max_retries}）：{error_msg}")
                            time.sleep(0.5)  # 等待0.5秒后重试
                        else:
                            self.window.push_log(f"识别服务错误（已重试{max_retries}次）：{error_msg}")
                            self.window.push_log("提示：Google语音识别服务暂时不可用，请稍后重试")
                except (ConnectionError, OSError) as exc:
                    # 网络连接错误（包括 WinError 10054）
                    retry_count += 1
                    error_msg = str(exc)
                    error_code = getattr(exc, 'winerror', None) or getattr(exc, 'errno', None)
                    if error_code == 10054 or '10054' in error_msg or '连接' in error_msg or 'connection' in error_msg.lower():
                        # 这是连接被重置的错误
                        if self.window:
                            if retry_count < max_retries:
                                self.window.push_log(f"网络连接被重置（重试 {retry_count}/{max_retries}），正在重新连接...")
                                time.sleep(1.5)  # 网络错误等待更长时间
                            else:
                                self.window.push_log(f"网络连接错误（已重试{max_retries}次）：{error_msg}")
                                self.window.push_log("提示：请检查网络连接或稍后重试")
                    else:
                        # 其他网络错误
                        if self.window:
                            if retry_count < max_retries:
                                self.window.push_log(f"网络连接错误（重试 {retry_count}/{max_retries}）：{error_msg}")
                                time.sleep(1.0)
                            else:
                                self.window.push_log(f"网络连接错误（已重试{max_retries}次）：{error_msg}")
                                self.window.push_log("提示：请检查网络连接或稍后重试")
                except Exception as exc:
                    # 其他未知错误
                    retry_count += 1
                    error_msg = str(exc)
                    if self.window:
                        if retry_count < max_retries:
                            self.window.push_log(f"识别失败（重试 {retry_count}/{max_retries}）：{error_msg}")
                            time.sleep(0.5)
                        else:
                            self.window.push_log(f"识别失败（已重试{max_retries}次）：{error_msg}")
            
            # 处理识别结果
            if text:
                self._handle_text(text.strip(), force_process=True)
            elif retry_count >= max_retries:
                if self.window:
                    self.window.push_log("识别失败：已达到最大重试次数")

    def _recognize_baidu(self, audio_data: bytes) -> Optional[str]:
        """使用百度API识别语音"""
        if not self.baidu_client:
            return None
        try:
            result = self.baidu_client.asr(audio_data, 'pcm', 16000, {'dev_pid': 1537})  # 1537=普通话(纯中文识别)
            if result.get('err_no') == 0:
                results = result.get('result', [])
                if results:
                    return results[0]
            else:
                err_msg = result.get('err_msg', '未知错误')
                if self.window:
                    self.window.push_log(f"百度识别错误：{err_msg}")
                # 如果是请求频率过高，抛出特殊异常
                if 'pv too much' in str(err_msg).lower() or 'request' in str(err_msg).lower():
                    raise RuntimeError("API调用频率过高，请稍后再试")
        except Exception as exc:
            if self.window:
                self.window.push_log(f"百度识别异常：{exc}")
            raise  # 重新抛出异常，让调用者处理
        return None

    # 已移除自动监听循环，只保留手动录音功能

    def _handle_text(self, text: str, force_process: bool = False) -> None:
        """处理识别到的文本（手动模式，直接处理命令）"""
        if not text:
            return
        if self.window:
            self.window.push_log(f"识别到：{text}")
        
        # 解析指令
        command = self._parse_command(text)
        
        if command:
            self.command_queue.put((command, text))
            if self.window:
                self.window.push_log(f"执行指令：{command}")
        else:
            if self.window:
                self.window.push_log("未识别到有效指令")

    @staticmethod
    def _parse_command(text: str) -> Optional[str]:
        normalized = text.replace("，", "").replace("。", "")
        for phrase, cmd in STANDARD_VOICE_COMMANDS.items():
            if phrase in normalized:
                return cmd
        # 关键词兜底
        if "手动" in normalized:
            return "manual"
        if "自动" in normalized or "视觉" in normalized:
            return "vision"
        if "空闲" in normalized:
            return "idle"
        if "归零" in normalized:
            return "zero"
        if "紧急" in normalized or "停止" in normalized:
            return "stop"
        if "零点" in normalized:
            return "set_zero"
        if "断电" in normalized:
            return "poweroff"
        return None

class MotorGroup2025:
    def __init__(self, ser: serial.Serial) -> None:
        self.ser = ser
        # 仅使用 0/1/2 三个电机
        self.m0 = RmdMotor(0, ser)  # 前进/后退
        self.m1 = RmdMotor(1, ser)  # 左右映射
        self.m2 = RmdMotor(2, ser)  # 上下映射
        
        # 电机限位角度（度）
        self.m0_limit = (-900.00, 0.00)     # 电机0的限位：-900~0度
        self.m1_limit = (-170.0, 170.0)  # 电机1的限位：-170~+170度
        self.m2_limit = (-680.0, 680.0)  # 电机2限位：±600°（含10倍减速器传动范围）

        # M1 位置追踪：记录最近一次发送的目标角度（供调试/显示，不作为限位依据）
        self.m1_target: float = 0.0

        # 串口访问锁，防止UI控制和手柄控制同时访问串口
        self.serial_lock = threading.Lock()

        # 启动时立即 stop M1，清除上次会话遗留的速度指令，防止程序重启间隔期间电机越限
        try:
            self.m1.stop()
            self.m1.update_state()
            _p = float(self.m1.position)
            if _p > self.m1_limit[1]:
                self.m1.set_position(self.m1_limit[1], max_speed=TURN_COEFF)
                print(f"{get_time()}-M1启动超上限({_p:.1f}°)，拉回到{self.m1_limit[1]:.1f}°")
            elif _p < self.m1_limit[0]:
                self.m1.set_position(self.m1_limit[0], max_speed=TURN_COEFF)
                print(f"{get_time()}-M1启动超下限({_p:.1f}°)，拉回到{self.m1_limit[0]:.1f}°")
        except Exception as exc:
            print(f"{get_time()}-M1启动初始化异常: {exc}")

    def get_angles(self) -> tuple:
        """获取各电机当前角度（度）"""
        with self.serial_lock:
            try:
                # 更新所有电机状态
                self.m0.update_state()
                self.m1.update_state()
                self.m2.update_state()
                # 返回位置信息
                return (self.m0.position, self.m1.position, self.m2.position)
            except (ValueError, Exception) as exc:
                # 如果更新状态失败，返回上次已知的位置或默认值
                print(f"{get_time()}-获取角度失败: {exc}")
                try:
                    return (self.m0.position, self.m1.position, self.m2.position)
                except:
                    return (0.0, 0.0, 0.0)

    def get_cached_angles(self) -> tuple:
        """
        获取各电机最近一次已知角度（度），
        不主动向串口发送查询命令，仅读取缓存值，
        供动作采集等对实时性要求不高的场景使用。
        """
        with self.serial_lock:
            try:
                return (float(self.m0.position), float(self.m1.position), float(self.m2.position))
            except Exception:
                return (0.0, 0.0, 0.0)

    def _apply_limited_speed(self, motor: RmdMotor, speed: float, limit_min: float, limit_max: float, axis_name: str, use_cached: bool = False) -> None:
        """
        统一的限位：
        1. 读取当前位置（或使用缓存值），估算新位置
        2. 执行限位检查，超限则直接拉回限位
        3. 在安全范围内按给定速度运行
        use_cached=True：跳过 update_state 串口读，使用上帧缓存位置（减少串口事务数）
        """
        try:
            if abs(speed) < 1e-3:
                motor.stop()
                return

            if not use_cached:
                motor.update_state()
            current_pos = motor.position

            # 优先：已越限（高速下跨帧越限），无论速度方向立即强制拉回，禁止继续运动
            if current_pos > limit_max:
                motor.set_position(limit_max, max_speed=TURN_COEFF)
                print(f"{get_time()}-电机{motor.id}已超上限，强制拉回：{current_pos:.1f}° -> {limit_max:.1f}°")
                return
            if current_pos < limit_min:
                motor.set_position(limit_min, max_speed=TURN_COEFF)
                print(f"{get_time()}-电机{motor.id}已超下限，强制拉回：{current_pos:.1f}° -> {limit_min:.1f}°")
                return

            increment = speed / 60.0  # 以 60Hz 控制频率估算位置增量
            projected_pos = current_pos + increment
            # 提前制动裕量：8° 给串口时延 + 电机惯性留足安全距离
            limit_tolerance = 8.0

            # 当前已抵住上限（tolerance内），且仍往正方向推 -> 位置控制贴住上限
            if current_pos >= limit_max - limit_tolerance and speed > 0:
                motor.set_position(limit_max, max_speed=TURN_COEFF)
                print(f"{get_time()}-电机{motor.id}停在上限：{current_pos:.1f}° -> {limit_max:.1f}°")
                return
            # 当前已抵住下限（tolerance内），且仍往负方向推 -> 位置控制贴住下限
            if current_pos <= limit_min + limit_tolerance and speed < 0:
                motor.set_position(limit_min, max_speed=TURN_COEFF)
                print(f"{get_time()}-电机{motor.id}停在下限：{current_pos:.1f}° -> {limit_min:.1f}°")
                return

            # 预计即将越过上限（且指令仍为正） -> 位置控制拉回
            if projected_pos >= limit_max and speed > 0:
                motor.set_position(limit_max, max_speed=TURN_COEFF)
                print(f"{get_time()}-电机{motor.id}触发上限限位：{current_pos:.1f}° -> {limit_max:.1f}°")
                return
            # 预计即将越过下限（且指令仍为负） -> 位置控制拉回
            if projected_pos <= limit_min and speed < 0:
                motor.set_position(limit_min, max_speed=TURN_COEFF)
                print(f"{get_time()}-电机{motor.id}触发下限限位：{current_pos:.1f}° -> {limit_min:.1f}°")
                return
            
            # 在限制范围内，正常控制
            motor.set_speed(speed)
        except Exception as exc:
            print(f"{get_time()}-轴{axis_name}控制异常：{exc}")
            try:
                motor.stop()  # 串口异常时立即停止电机，防止持续运行越限
            except Exception:
                pass
    
    def move_forward(self, speed_forward: float) -> None:
        with self.serial_lock:
            try:
                self._apply_limited_speed(
                    motor=self.m0,
                    speed=speed_forward * M0_DIRECTION_SIGN,  # ★ [修改1] M0方向取反
                    limit_min=self.m0_limit[0],
                    limit_max=self.m0_limit[1],
                    axis_name="M0",
                    use_cached=True,  # 使用帧首 get_angles 缓存位置，省掉额外串口读
                )
            except (ValueError, Exception) as exc:
                print(f"{get_time()}-电机M0控制错误: {exc}")

    def map_horizontal(self, value: float) -> None:
        """
        M1 水平转向：前瞻位置控制模式。

        原理：以 M1_LOOKAHEAD_S 秒的前瞻距离为目标，max_speed = commanded_speed，
        使电机始终以指令速度行进；目标在发送前被 clamp 到 ±170°，
        电机永远不会收到越限目标，从根本上防止超限。

        value: 期望速度 (deg/s)，经 M1_DIRECTION_SIGN 对应实际方向
        """
        with self.serial_lock:
            try:
                limit_min, limit_max = self.m1_limit
                directed_speed = value * M1_DIRECTION_SIGN  # 修正方向

                # 读取真实位置（每帧必读，作为基准）
                self.m1.update_state()
                current_pos = float(self.m1.position)

                # 越限保护（绝对优先，无论指令方向）
                if current_pos > limit_max:
                    self.m1.set_position(limit_max, max_speed=TURN_COEFF)
                    self.m1_target = limit_max
                    print(f"{get_time()}-M1已超上限，强制拉回：{current_pos:.1f}° -> {limit_max:.1f}°")
                    return
                if current_pos < limit_min:
                    self.m1.set_position(limit_min, max_speed=TURN_COEFF)
                    self.m1_target = limit_min
                    print(f"{get_time()}-M1已超下限，强制拉回：{current_pos:.1f}° -> {limit_min:.1f}°")
                    return

                # 零速：停止并退出
                if abs(directed_speed) < 1e-3:
                    self.m1.stop()
                    self.m1_target = current_pos
                    return

                # 前瞻目标 = 当前位置 + 2s 的前瞻距离，clamp 到限位
                # 电机以 max_speed=commanded_speed 行进；贴近限位时目标已 clamp，
                # 电机自然减速并停在限位，无需任何额外制动逻辑
                _LOOKAHEAD_S = 2.0
                raw_target = current_pos + directed_speed * _LOOKAHEAD_S
                new_target = max(limit_min, min(limit_max, raw_target))
                self.m1_target = new_target

                # 位置控制：max_speed = commanded_speed，限速与指令一致
                self.m1.set_position(new_target, max_speed=max(1.0, abs(directed_speed)))

            except (ValueError, Exception) as exc:
                print(f"{get_time()}-电机M1控制错误: {exc}")
                try:
                    self.m1.stop()
                except Exception:
                    pass

    def map_vertical(self, value: float) -> None:
        with self.serial_lock:
            try:
                self._apply_limited_speed(
                    motor=self.m2,
                    speed=value,
                    limit_min=self.m2_limit[0],
                    limit_max=self.m2_limit[1],
                    axis_name="M2",
                    use_cached=False,  # M2限位严格：每帧读真实位置，防止高速下缓存滞后越限
                )
            except (ValueError, Exception) as exc:
                print(f"{get_time()}-电机M2控制错误: {exc}")

    def set_motor_position(self, motor_id: int, target_angle: float, max_speed: Optional[float] = None) -> None:
        """位置控制接口，提供串口锁保护"""
        motor_map = {0: self.m0, 1: self.m1, 2: self.m2}
        motor = motor_map.get(motor_id)
        if motor is None:
            return
        with self.serial_lock:
            try:
                kwargs: Dict[str, Any] = {}
                if max_speed is not None:
                    kwargs["max_speed"] = max_speed
                motor.set_position(target_angle, **kwargs)
            except (ValueError, Exception) as exc:
                print(f"{get_time()}-电机{motor_id:02d}位置控制错误: {exc}")

    def angle_return(self) -> bool:
        """电机归零，使用位置闭环控制"""
        with self.serial_lock:
            finish = True
            try:
                # 一次性更新两个电机状态
                self.m1.update_state()
                self.m2.update_state()

                # 获取当前角度
                current_angle1 = self.m1.position
                current_angle2 = self.m2.position

                # 电机1：P控制归零，速度随距离成比例缩小，防止过冲与抖动
                if abs(current_angle1) > 0.1:
                    # Kp=0.8：100°->50(满速), 30°->24, 10°->8, 5°->4, <4°->3(最小力)
                    m1_return_speed = max(3.0, min(M1_SPEED_COEFF, abs(current_angle1) * 0.8))
                    self.m1.set_position(0, max_speed=m1_return_speed)
                    finish = False
                
                # 电机2：使用位置闭环，精确归零，速度为手动速度的一半
                if abs(current_angle2) > 0.1:
                    self.m2.set_position(0, max_speed=M2_SPEED_COEFF / 2)  # 归零速度 = 手动最大速度的1/2
                    finish = False
                
                if finish:
                    # 完全停止
                    self.m1.stop()
                    self.m2.stop()
            except (ValueError, Exception) as exc:
                print(f"{get_time()}-角度归零错误: {exc}")
                try:
                    self.m1.stop()
                    self.m2.stop()
                except Exception:
                    pass
                finish = True  # 出错退出循环避免卡死，日志中会有错误记录
        
        return finish

    def set_current_position_as_zero_point(self) -> None:
        with self.serial_lock:
            try:
                self.m0.set_current_position_as_zero_point()  # ★ 新增：0号电机同步写入零点
                self.m1.set_current_position_as_zero_point()
                self.m2.set_current_position_as_zero_point()
            except Exception as exc:
                print(f"{get_time()}-设置零点错误: {exc}")

    def stop(self) -> None:
        with self.serial_lock:
            try:
                self.m0.stop()
            except Exception:
                pass
            try:
                self.m1.stop()
            except Exception:
                pass
            try:
                self.m2.stop()
            except Exception:
                pass

    def shutdown_all(self) -> None:
        # 关闭输出，进入空闲可自由转动
        with self.serial_lock:
            try:
                self.m0.shutdown()
            except Exception:
                pass
            try:
                self.m1.shutdown()
            except Exception:
                pass
            try:
                self.m2.shutdown()
            except Exception:
                pass


class VisionRobotStateMachine:
    """五态状态机：空闲保持、手动控制、视觉自主、AI自主巡检、断电维护"""

    states = ('Idle', 'ManualControl', 'VisionControl', 'AIAuto', 'ReturnOrigin', 'PowerOff')

    def __init__(self) -> None:
        print(f"{get_time()}-main2025 启动：初始化手柄与电机...")

        self.xbox: Optional[XboxController] = None
        self.controller_available = False
        try:
            self.xbox = XboxController()
            self.controller_available = True
            print(f"{get_time()}-检测到手柄：已启用硬件联动控制")
        except Exception as exc:
            self.xbox = None
            print(f"{get_time()}-未检测到手柄，默认为纯UI控制（{exc}）")

        port = find_rmd_motor_port(0)
        if port is None:
            raise RuntimeError("未找到RMD电机串口")

        self.ser = serial.Serial(port, baudrate=BAUDRATE, timeout=TIMEOUT)
        self.motors = MotorGroup2025(self.ser)

        self.shutdown_event = threading.Event()
        self.ui_command_queue: "queue.Queue[Tuple[str, str]]" = queue.Queue()

        self.machine = Machine(
            model=self,
            states=self.states,
            initial='Idle',
            auto_transitions=False,
            ignore_invalid_triggers=True,
        )

        self.machine.add_transition('enter_idle', '*', 'Idle')
        self.machine.add_transition('activate_manual', ['Idle', 'VisionControl', 'AIAuto', 'PowerOff'], 'ManualControl')
        self.machine.add_transition('activate_vision', ['Idle', 'ManualControl', 'AIAuto'], 'VisionControl')
        self.machine.add_transition('activate_ai_auto', ['Idle', 'ManualControl', 'VisionControl'], 'AIAuto')
        self.machine.add_transition('cut_power', '*', 'PowerOff')

        self.state_handlers = {
            'Idle': self.loop_idle,
            'ManualControl': self.loop_manual_control,
            'VisionControl': self.loop_vision_control,
            'AIAuto': self.loop_ai_auto,
            'ReturnOrigin': self.loop_return_origin,
            'PowerOff': self.loop_power_off,
        }

        # BCRunner 懒加载（第一次进入 AIAuto 时初始化）
        self._bc_runner: Optional[Any] = None
        self._bc_goal_id: int = 3  # 默认 RMB 右主支气管

        # AutoNav 控制器（第一次进入 AIAuto 时初始化，复用 goal_id）
        self._autonav: Optional[Any] = None
        self._autonav_demo_file: Optional[str] = None
        self._autonav_enable_layer2: bool = False
        self._autonav_enable_layer3: bool = False
        self._autonav_full_inspection: bool = False
        self._autonav_inspection_plan: Optional[Dict[str, str]] = None
        self._autonav_return_origin: bool = False
        self._tr_return_started_at: Optional[float] = None
        self._autonav_last_mode: Optional[str] = None

        # AI巡检控制参数
        self._ai_frame_counter: int = 0          # 限频计数器（每3帧发一次串口指令）
        self._ai_motor_fail_count: int = 0       # 连续串口失败计数
        self._AI_CMD_INTERVAL   = 3              # 每隔几帧发一次指令（60Hz→20Hz）
        self._AI_M0_STOP_DEG    = -280.0         # M0目标到此角度视为巡检完成并停止
        self._AI_MAX_STEPS      = 10000           # 最大推理步数（3*60s × 60Hz）
        self._AI_MAX_FAIL       = 20             # 连续串口失败超过此次数则退出

        # 显式设置初始状态
        self.state = 'Idle'
        
        # 灵敏度系数（在视觉模式时会降低）
        self.sensitivity_multiplier = 1.0
        # Vision tuning: reduce sensitivity and max speed for autonomous vision control
        # Reduce sensitivity at least 3x as requested
        self.vision_sensitivity_multiplier = 1.0 / 3.0
        # Scale vision max speeds (0..1). 0.5 reduces speed by half.
        self.vision_speed_multiplier = 0.5

        self.manual_input_lock = threading.Lock()
        self.ui_manual_input = {'forward': 0.0, 'horizontal': 0.0, 'vertical': 0.0}
        self.controller_manual_input = {'forward': 0.0, 'horizontal': 0.0, 'vertical': 0.0}
        self.vision_targets = {'m1': 0.0, 'm2': 0.0}
        # Filtered vision targets for smoothing in vision mode
        self.vision_targets_filtered = {'m1': 0.0, 'm2': 0.0}
        # Vision-mode tuning for M2 (vertical) — deadband and smoothing
        # Increase deadband to avoid oscillation around target in vision mode
        self.vision_m2_deadband_deg = 0.15
        # Lower alpha for EMA so filtered target is smoother (less aggressive)
        self.vision_m2_smooth_alpha = 0.3
        # Vision-mode per-update step limits for M2 (override manual min/max)
        # Use small min step to allow precise approach but avoid forcing large snaps
        self.vision_m2_min_step = 0.1
        self.vision_m2_max_step = 1.0
        # Vision-mode minimum commanded move for M1 (degrees) to avoid tiny angle commands
        self.vision_m1_min_step = 1.0
        # Vision M2 scale relative to manual mapping — reduce sensitivity further (smaller = less movement)
        self.vision_m2_scale = 0.004
        # When force_send is triggered but filtered change is small, move by this forced step (degrees)
        # Use module-level constants so they can be edited at the top of the file.
        self.vision_m2_force_step = VISION_M2_FORCE_STEP
        # Speed used for step nudges in vision mode
        self.vision_step_speed = VISION_STEP_SPEED
        # Duration for each speed nudge (seconds)
        self.vision_step_duration = VISION_STEP_DURATION
        # vision fallback speed (used for short nudges)
        self.vision_fallback_speed = VISION_FALLBACK_SPEED
        # If vision input magnitude exceeds this, force a send even if filtered change < deadband
        self.vision_force_input_threshold = 10.0
        # Manual position targets for position-control mode (m0/m1/m2)
        # These store the desired absolute positions (degrees) updated each cycle by deltas
        self.manual_targets = {'m0': 0.0, 'm1': 0.0, 'm2': 0.0}
        # Smoothing / sensitivity tunables for manual position control
        # m1 (horizontal) smoothing alpha for EMA filter (0..1). Lower -> smoother/slower.
        self.m1_smooth_alpha = 0.35
        # m1 deadband (degrees) below which we will not issue new position commands
        self.m1_deadband_deg = 0.25
        # m2 (vertical) sensitivity multiplier (0..1) to reduce responsiveness
        self.m2_sensitivity_multiplier = 0.45
        # m2 deadband (degrees) below which we will not issue new position commands (manual control)
        self.m2_deadband_deg = 0.05
        # minimal delta (degrees) to consider a command meaningful for any motor (keep small)
        self.manual_min_delta_deg = 0.01
        # filtered targets used to smooth M1 (and optionally others)
        self.manual_targets_filtered = {'m0': 0.0, 'm1': 0.0, 'm2': 0.0}

        self.state_thread = threading.Thread(target=self._state_loop, name="robot_state_loop")

        self.voice_window: Optional[VoiceStatusWindow] = None
        self.voice_control: Optional[VoiceCommandCenter] = None
        if VOICE_SWITCH:
            self._setup_voice_components()

        # 动作采集和导航（不使用复杂类型标注，避免 Pylance 将模块误判为类型）
        self.action_recorder = None  # 实例在 _setup_action_recorder 中创建
        self.action_player = None
        self._setup_action_recorder()

        # 打印初始化完成提示
        self._log_prompt(
            f"{get_time()}-初始化完成，进入空闲状态：\n"
            f"  START  -> 切换到手动控制\n"
            f"  Y      -> 切换到视觉自主\n"
            f"  BACK   -> 退出程序\n"
            f"  X      -> 电机回零\n"
            f"  A      -> 将当前位置写入零点\n"
            f"  右扳机(RT) -> 前进\n"
            f"  左扳机(LT) -> 后退\n"
            f"  右摇杆X(RX) -> 左右转向\n"
            f"  左摇杆Y(LY) -> 上下调节"
        )
        self._update_voice_state("空闲保持")

    def start(self) -> None:
        self.state_thread.start()

    def join(self) -> None:
        self.state_thread.join()

    def request_shutdown(self, reason: Optional[str] = None) -> None:
        if reason:
            print(reason)
        if self.state != 'PowerOff':
            self._transition_to_poweroff()
        self.shutdown_event.set()

    # ---------------------------
    # 状态进入提示
    # ---------------------------
    def on_enter_Idle(self) -> None:
        self._log_prompt(
            f"\n{get_time()}-状态切换：空闲保持\n"
            f"  START -> 手动控制\n"
            f"  Y     -> 视觉自主\n"
            f"  LB    -> AI巡检\n"
            f"  B     -> 断电维护\n"
            f"  BACK  -> 退出程序"
        )
        self.motors.stop()
        self._update_voice_state("空闲保持")
        self._clear_manual_inputs()

    def on_enter_ManualControl(self) -> None:
        self.sensitivity_multiplier = 1.0
        self._log_prompt(
            f"{get_time()}-状态切换：手动控制\n"
            f"  START -> 空闲保持\n"
            f"  Y     -> 视觉自主\n"
            f"  LB    -> AI巡检\n"
            f"  B     -> 断电维护\n"
            f"  X     -> 电机回零\n"
            f"  A     -> 将当前位置写入零点\n"
            f"  BACK  -> 退出程序"
        )
        self._update_voice_state("手动控制")
        self._clear_manual_inputs()
        # Ensure manual position targets start from current motor positions to avoid jumps
        try:
            self._sync_manual_targets()
        except Exception:
            pass

    def on_enter_VisionControl(self) -> None:
        # 初始灵敏度设为0.5（用于快速接近）
        self.sensitivity_multiplier = 0.5
        # 视觉模式下同步视觉目标到当前位置，避免突然跳变
        self._sync_vision_targets()
        self._log_prompt(
            f"{get_time()}-状态切换：视觉自主控制（自适应灵敏度）\n"
            
            f"  START -> 切回手动\n"
            f"  B     -> 空闲保持\n"
            f"  X     -> 电机回零\n"
            f"  BACK  -> 退出程序"
        )
        self._update_voice_state("视觉自主控制")
        self._clear_manual_inputs()

    def on_enter_PowerOff(self) -> None:
        self._log_prompt(
            f"{get_time()}-状态切换：断电维护\n"
            f"  START -> 空闲保持\n"
            f"  BACK  -> 退出程序"
        )
        self.motors.stop()
        self.motors.shutdown_all()
        self._update_voice_state("断电维护")
        self._clear_manual_inputs()

    def on_enter_ReturnOrigin(self) -> None:
        """进入独立三轴归零状态，不经过 AutoNav。"""
        self._ai_frame_counter = 0
        self._tr_return_started_at = None
        self._log_prompt(
            f"{get_time()}-进入 TR 三轴归零模式：M0/M1/M2 → 0°\n"
            f"  START / B / LB -> 停止并回到空闲\n"
            f"  BACK           -> 退出程序"
        )
        self._update_voice_state("TR 三轴归零", "目标: (0°, 0°, 0°)")

    # ---------------------------
    # 状态循环
    # ---------------------------
    def loop_idle(self) -> None:
        if self._handle_back_button():
            return
        if self._controller_button_pressed('START'):
            self._transition_to_manual()
            return
        if self._controller_button_pressed('Y'):
            self._transition_to_vision()
            return
        if self._controller_button_pressed('LB'):
            self._transition_to_ai_auto()
            return
        if self._controller_button_pressed('B'):
            self._transition_to_poweroff()
            return
        if self._controller_button_pressed('A'):
            self._set_zero_point()

    def loop_manual_control(self) -> None:
        if self._handle_common_controls(
            next_idle_trigger='START',
            next_vision_trigger='Y',
            allow_set_zero_point=True,
            poweroff_trigger='B',
        ):
            return

        if self._controller_button_pressed('LB'):
            self._transition_to_ai_auto()
            return

        if self._is_action_playing():
            # 动作导航期间禁止手动控制，优先保证导航指令
            if self.bc_collector is not None and self.bc_collector.is_recording:
                self.bc_collector.collect_frame()
            return

        self._update_controller_inputs()
        forward_speed, horizontal_speed, vertical_speed = self._resolve_manual_inputs()

        # 帧首仅刷新 M0 位置（M0 use_cached=True；M1/M2 各自在 _apply_limited_speed 读真实值）
        # 串口事务：1(M0读) + 1(M0写) + 1(M1读) + 1(M1写) + 1(M2读) + 1(M2写) = 6次/帧
        try:
            with self.motors.serial_lock:
                self.motors.m0.update_state()
        except Exception:
            pass

        self._apply_manual_motion(forward_speed, horizontal_speed, vertical_speed)

        # BC 专家演示采集
        if self.bc_collector is not None and self.bc_collector.is_recording:
            self.bc_collector.collect_frame()

        # TR 是独立的三轴原点路径，不复用只控制 M1/M2 的“归零”按钮。
        if self._autonav_return_origin:
            self._loop_tr_return()
            return

        try:
            # 使用缓存角度显示，不再额外读串口
            angles = self.motors.get_cached_angles()
            source = self._current_manual_source()
            print(
                f"\r手动控制[{source}] "
                f"F={forward_speed:>6.1f} "
                f"H={horizontal_speed:>6.1f} "
                f"V={vertical_speed:>6.1f} "
                f"角度(M1={angles[1]:.1f}°, M2={angles[2]:.1f}°)",
                end=""
            )
        except Exception as exc:
            print(f"{get_time()}-读取角度失败：{exc}")

    def loop_vision_control(self) -> None:
        # 视觉模式下禁止A按钮写零点
        if self._handle_common_controls(next_idle_trigger='B', next_vision_trigger=None, allow_vision_toggle=False, allow_set_zero_point=False):
            return

        forward_speed = self._safe_float(get_config('speed_pf'))
        # 前进/后退由深度推理控制

        speed_pt = get_config('speed_pt')
        
        if self._is_action_playing():
            if self.bc_collector is not None and self.bc_collector.is_recording:
                self.bc_collector.collect_frame()
            return

        # BC 专家演示采集
        if self.bc_collector is not None and self.bc_collector.is_recording:
            self.bc_collector.collect_frame()

        # 将来自视觉算法的 (x, y) 指令拆解成：
        #   x -> 顺/逆时针旋转（水平）
        #   y -> 上/下俯仰（垂直）
        forward_input = forward_speed  # 视觉模式下前进/后退输入
        horizontal_input = 0.0
        vertical_input = 0.0
        if isinstance(speed_pt, dict):
            horizontal_input = self._safe_float(speed_pt.get('x', 0.0))
            vertical_input = self._safe_float(speed_pt.get('y', 0.0))
        elif isinstance(speed_pt, (list, tuple)):
            if len(speed_pt) >= 1:
                horizontal_input = self._safe_float(speed_pt[0])
            if len(speed_pt) >= 2:
                vertical_input = self._safe_float(speed_pt[1])

        # 帧首仅刷新 M0 位置（M1/M2 各自在 _apply_limited_speed 读真实值，确保限位准确）
        try:
            with self.motors.serial_lock:
                self.motors.m0.update_state()
        except Exception:
            pass

        pos_info = self._vision_position_mode(
            forward_input,
            horizontal_input * HORIZONTAL_CLOCKWISE_SIGN,
            vertical_input,
        )
        

    def loop_ai_auto(self) -> None:
        """
        AI 自主巡检模式（AutoNav 序列导航版）：
          - 手柄 START / B / LB → 退出到空闲
          - AutoNavController 每帧输出电机目标（M0/M1/M2）
          - 集成 Layer1 序列导航 + Layer2 岔口引导 + Layer3 阻塞物清除
        """
        # 退出条件：START / B / LB 均可停止
        if (self._controller_button_pressed('START')
                or self._controller_button_pressed('B')
                or self._controller_button_pressed('LB')):
            if self.voice_window:
                self.voice_window.push_log("[AI] 手柄停止 → 回到空闲")
            self._stop_autonav()
            self._transition_to_idle()
            return

        # BC 采集（AI运动期间也可采集用于 DAgger）
        if self.bc_collector is not None and self.bc_collector.is_recording:
            self.bc_collector.collect_frame()

        # 懒加载 AutoNavController（替换原 BCRunner）
        if self._autonav is None:
            import os as _os2
            demo_dir = _os2.path.join(_os2.path.dirname(__file__), "BC", "expert_demos", "AutoNavdatasets")
            if not _os2.path.exists(demo_dir):
                if self.voice_window:
                    self.voice_window.push_log(f"[AI] 演示目录不存在: {demo_dir}")
                    self.voice_window.push_log("[AI] 请先在手动模式下采集导航路径")
                self._transition_to_idle()
                return
            if not _AUTONAV_AVAILABLE:
                if self.voice_window:
                    self.voice_window.push_log("[AI] AutoNav 模块未加载")
                self._transition_to_idle()
                return
            try:
                # 记录当前实际电机位置作为序列起点
                try:
                    _cur = self.motors.get_cached_angles()
                    start_angles = (float(_cur[0]), float(_cur[1]), float(_cur[2]))
                except Exception:
                    start_angles = (0.0, 0.0, 0.0)
                self._autonav = _AutoNavController(
                    demo_dir    = demo_dir,
                    motor_group = self.motors,
                    enable_junction = self._autonav_enable_layer2,
                    enable_obstruction = self._autonav_enable_layer3,
                )
                if self._autonav_full_inspection:
                    self._autonav.start_inspection(
                        current_motor_angles=start_angles,
                        source_files=self._autonav_inspection_plan,
                    )
                else:
                    self._autonav.start(
                        path_label           = self._bc_goal_id,
                        current_motor_angles = start_angles,
                        demo_file            = self._autonav_demo_file,
                    )
                goal_name = _BC_NAMES.get(self._bc_goal_id, "?")
                if self.voice_window:
                    if self._autonav_full_inspection:
                        self.voice_window.push_log(
                            "[AI] 全流程轨迹已在出发前完成拼接，开始巡检"
                        )
                    else:
                        self.voice_window.push_log(
                            f"[AI] AutoNav 启动 → 目标: {_BC_PATHS.get(self._bc_goal_id, ('?','?'))[0]} {goal_name}"
                        )
                    source_name = (
                        "五叶段离散轨迹拼接"
                        if self._autonav_full_inspection else
                        (_os2.path.basename(self._autonav_demo_file)
                         if self._autonav_demo_file else "同部位平均路径")
                    )
                    layers = "Layer 1"
                    if self._autonav_enable_layer2:
                        layers += " + Layer 2"
                    if self._autonav_enable_layer3:
                        layers += " + Layer 3"
                    self.voice_window.push_log(f"[AI] 路径: {source_name} | 启用: {layers}")
                    self.voice_window.push_log(
                        f"[AI] 导航速度系数={_AUTONAV_DEFAULT_SPEED}（采集 20Hz，控制循环 60Hz）"
                    )
            except Exception as e:
                if self.voice_window:
                    self.voice_window.push_log(f"[AI] AutoNav 初始化失败: {e}")
                import traceback
                print(f"[AI] AutoNav 初始化详情:\n{traceback.format_exc()}")
                self._transition_to_idle()
                return

        # 读取当前电机角度
        try:
            if self._ai_frame_counter % self._AI_CMD_INTERVAL == 0:
                _cur = self.motors.get_angles()
            else:
                _cur = self.motors.get_cached_angles()
            _cur0, _cur1, _cur2 = float(_cur[0]), float(_cur[1]), float(_cur[2])
        except Exception:
            _cur0 = _cur1 = _cur2 = 0.0

        # AutoNav 单步推理
        try:
            # Layer 1 不依赖视觉；Layer 2/3 开启时读取视觉线程写入的共享快照。
            vis = _bc_get_vis() if _bc_get_vis else {}
            decision = self._autonav.step(
                motor_angles = (_cur0, _cur1, _cur2),
                visual       = vis,
            )
        except Exception as e:
            if self.voice_window:
                self.voice_window.push_log(f"[AI] AutoNav 推理异常: {e}")
            import traceback
            print(f"[AI] AutoNav 推理详情:\n{traceback.format_exc()}")
            self._stop_autonav()
            self._transition_to_idle()
            return

        previous_nav_mode = self._autonav_last_mode
        current_nav_mode = decision.mode.value
        if current_nav_mode != previous_nav_mode:
            if self.voice_window and current_nav_mode == "clearing":
                self.voice_window.push_log(
                    f"[Layer3] 阻塞物面积 {decision.obstruction_area:.1%} "
                    f"超过阈值，暂停轨迹并保存断点 {decision.checkpoint_step}"
                )
            elif self.voice_window and current_nav_mode == "returning":
                self.voice_window.push_log(
                    f"[Layer3] 已确认阻塞物清除，返回轨迹断点 "
                    f"{decision.checkpoint_step}"
                )
            elif (
                self.voice_window
                and current_nav_mode == "navigating"
                and previous_nav_mode == "returning"
            ):
                self.voice_window.push_log(
                    f"[Layer3] 已回到断点 {decision.checkpoint_step}，继续自主巡检"
                )
            self._autonav_last_mode = current_nav_mode

        # ── 停止条件检查 ────────────────────────────────────────
        # 暂停时位置模式继续保持当前角度，等待 UI 再次点击“恢复”。
        if decision.mode.value == "paused":
            return
        # 1) 序列播放完毕（DONE）
        if not decision.active or decision.mode.value == "done":
            if self.voice_window:
                if self._autonav_full_inspection:
                    self.voice_window.push_log(
                        f"[AI] 五叶段巡检完成并已返回 TR 出发原点 "
                        f"(步 {decision.step}/{decision.total_steps})"
                    )
                else:
                    self.voice_window.push_log(
                        f"[AI] AutoNav 已到达目标部位 (步 {decision.step}/{decision.total_steps}) → 巡检完成"
                    )
            self._stop_autonav()
            self._transition_to_idle()
            return
        # 2) 超过最大步数（正常完成以整条采集路径结束为准）
        if decision.step >= self._AI_MAX_STEPS:
            if self.voice_window:
                self.voice_window.push_log(
                    f"[AI] 已达最大步数 {self._AI_MAX_STEPS} → 停止"
                )
            self._stop_autonav()
            self._transition_to_idle()
            return

        # 前 10 步在控制台打印诊断
        if decision.step <= 10:
            print(f"[AI步{decision.step}] cur=({_cur0:.1f},{_cur1:.1f},{_cur2:.1f}) "
                  f"target=({decision.target_m0:.2f},{decision.target_m1:.2f},{decision.target_m2:.2f}) "
                  f"mode={decision.mode.value} Δ=({decision.delta_m0:.2f},{decision.delta_m1:.2f},{decision.delta_m2:.2f})"
                  f"{' obs' if decision.obstruction_detected else ''}")

        # 更新 UI 导航进度
        if self.voice_window:
            progress = (decision.step / decision.total_steps * 100) if decision.total_steps > 0 else 0
            mode_names = {"navigating": "路径导航", "settling": "等待到位", "paused": "已暂停", "clearing": "阻塞清除", "returning": "返回断点", "done": "完成"}
            status = mode_names.get(decision.mode.value, decision.mode.value)
            self.voice_window.queue.put({
                "type": "nav_progress",
                "data": {
                    "status": status,
                    "progress": progress,
                    "step": decision.step,
                    "total": decision.total_steps,
                    "info": (
                        f"全流程: {decision.route_stage} | "
                        f"电机: ({_cur0:.1f}, {_cur1:.1f}, {_cur2:.1f})"
                        if self._autonav_full_inspection else
                        f"目标: {_BC_NAMES.get(self._bc_goal_id, '?')} | "
                        f"电机: ({_cur0:.1f}, {_cur1:.1f}, {_cur2:.1f})"
                    )
                }
            })

        # ── 限频发送电机指令（每 _AI_CMD_INTERVAL 帧发一次，约20Hz）──
        self._ai_frame_counter += 1
        if self._ai_frame_counter % self._AI_CMD_INTERVAL == 0:
            m0_ok = True
            try:
                self.motors.set_motor_position(0, decision.target_m0, max_speed=_AUTONAV_MOTOR_SPEED[0])
            except Exception:
                m0_ok = False
            try:
                self.motors.set_motor_position(1, decision.target_m1, max_speed=_AUTONAV_MOTOR_SPEED[1])
            except Exception:
                pass
            try:
                self.motors.set_motor_position(2, decision.target_m2, max_speed=_AUTONAV_MOTOR_SPEED[2])
            except Exception:
                pass

            # M0 连续串口失败自动退出
            if not m0_ok:
                self._ai_motor_fail_count += 1
                if self._ai_motor_fail_count >= self._AI_MAX_FAIL:
                    if self.voice_window:
                        self.voice_window.push_log(
                            f"[AI] M0 串口连续失败 {self._AI_MAX_FAIL} 次 → 自动停止"
                        )
                    self._stop_autonav()
                    self._transition_to_idle()
                    return
            else:
                self._ai_motor_fail_count = 0

        # UI 状态显示：前 10 步每步都输出，之后每 30 步一次
        if self.voice_window:
            goal_code = _BC_PATHS.get(self._bc_goal_id, ('?', '?'))[0]
            if decision.step <= 10 or decision.step % 30 == 0:
                self.voice_window.push_log(
                    f"[AI步{decision.step}] mode={decision.mode.value} "
                    f"Δ=[{decision.delta_m0:.2f},{decision.delta_m1:.2f},{decision.delta_m2:.2f}]°"
                    f" 岔口偏置=[{decision.junction_corr_m1:.2f},{decision.junction_corr_m2:.2f}]°"
                    f" gate={'ON' if decision.junction_active else 'OFF'}"
                    f" dist={decision.junction_center_distance:.2f}"
                    f" strength={decision.junction_strength:.2f}"
                    f" reason={decision.junction_gate_reason}"
                    f" stable={decision.junction_stable_frames}"
                    f" stage={decision.route_stage_kind or '-'}"
                    f" local={decision.route_stage_progress:.2f}"
                    f" age={decision.visual_age_s:.2f}s"
                    f" obs_area={decision.obstruction_area:.3f}"
                    f" obs_gate={decision.obstruction_gate_reason}"
                    f" obs_stable={decision.obstruction_stable_frames}"
                    f" clear={decision.obstruction_clear_frames}"
                    f"{' OBS' if decision.obstruction_detected else ''}"
                )
            if decision.step % 30 == 0:
                pct = decision.progress * 100
                self.voice_window.update_state(
                    ("全流程自主巡检" if self._autonav_full_inspection
                     else f"AI自主 [{goal_code}]"),
                    ((decision.route_stage + " | ")
                     if self._autonav_full_inspection else "")
                    + f"步={decision.step}/{decision.total_steps} | {pct:.0f}%"
                )

    def _loop_tr_return(self) -> None:
        """TR 路径：闭环控制 M0/M1/M2 全部回到电机 0°。"""
        if self._tr_return_started_at is None:
            self._tr_return_started_at = time.monotonic()
            if self.voice_window:
                self.voice_window.push_log("[TR] 开始三轴返回固定原点 (0°, 0°, 0°)")

        self._ai_frame_counter += 1
        if self._ai_frame_counter % self._AI_CMD_INTERVAL != 0:
            return

        angles = tuple(float(value) for value in self.motors.get_angles())
        errors = tuple(abs(value) for value in angles)
        if errors[0] <= 3.0 and errors[1] <= 2.0 and errors[2] <= 3.0:
            self._autonav_return_origin = False
            self._tr_return_started_at = None
            self.motors.stop()
            if self.voice_window:
                self.voice_window.push_log(f"[TR] 三轴已到原点，当前角度={angles}")
            self._transition_to_idle()
            return

        if time.monotonic() - self._tr_return_started_at >= 60.0:
            self._autonav_return_origin = False
            self._tr_return_started_at = None
            self.motors.stop()
            if self.voice_window:
                self.voice_window.push_log(f"[TR] 返回原点超时，当前角度={angles}")
            self._transition_to_idle()
            return

        for motor_id in range(3):
            self.motors.set_motor_position(
                motor_id, 0.0, max_speed=_AUTONAV_MOTOR_SPEED[motor_id]
            )
        if self.voice_window and self._ai_frame_counter % 60 == 0:
            self.voice_window.push_log(
                f"[TR] 返回中，当前角度=({angles[0]:.1f}, {angles[1]:.1f}, {angles[2]:.1f})"
            )

    def loop_return_origin(self) -> None:
        """独立 TR 控制循环；不会触发 AutoNav 路径加载。"""
        if self._handle_back_button():
            return
        if (self._controller_button_pressed('START')
                or self._controller_button_pressed('B')
                or self._controller_button_pressed('LB')):
            self.motors.stop()
            self._autonav_return_origin = False
            self._tr_return_started_at = None
            self._transition_to_idle()
            return
        self._loop_tr_return()

    def _stop_autonav(self) -> None:
        """停止 AutoNav 并清空状态。"""
        if self._autonav is not None:
            try:
                self._autonav.stop()
            except Exception:
                pass
            self._autonav = None
        self._autonav_return_origin = False
        self._autonav_full_inspection = False
        self._tr_return_started_at = None
        self._autonav_last_mode = None

    def loop_power_off(self) -> None:
        if self._handle_back_button():
            return
        if self._controller_button_pressed('START'):
            self._transition_to_idle()
    # ---------------------------
    # 内部辅助
    # ---------------------------
    def _state_loop(self) -> None:
        try:
            while not self.shutdown_event.is_set():
                t0 = time.perf_counter()
                handler = self.state_handlers.get(self.state)
                if handler:
                    handler()
                self._process_voice_commands()
                busy_maintain_target_frequency(60, t0)
        except KeyboardInterrupt:
            self.request_shutdown(f"{get_time()}-收到中断信号，退出中...")
        finally:
            self._cleanup()

    def _cleanup(self) -> None:
        try:
            self.motors.stop()
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass
        if self.voice_control:
            self.voice_control.stop()
        if self.voice_window:
            self.voice_window.stop()
        print(f"{get_time()}-main2025 已退出")

    def _handle_common_controls(
        self,
        next_idle_trigger: Optional[str],
        next_vision_trigger: Optional[str],
        allow_vision_toggle: bool = True,
        allow_set_zero_point: bool = True,
        poweroff_trigger: Optional[str] = None,
    ) -> bool:
        if self._handle_back_button():
            return True
        if self._controller_button_pressed('X'):
            self._perform_angle_return()
            return True
        if allow_set_zero_point and self._controller_button_pressed('A'):
            self._set_zero_point()
        if next_idle_trigger and self._controller_button_pressed(next_idle_trigger):
            self._transition_to_idle()
            return True
        if allow_vision_toggle and next_vision_trigger and self._controller_button_pressed(next_vision_trigger):
            self._transition_to_vision()
            return True
        if self.state == 'VisionControl' and self._controller_button_pressed('START'):
            self._transition_to_manual()
            return True
        if poweroff_trigger and self._controller_button_pressed(poweroff_trigger):
            self._transition_to_poweroff()
            return True
        return False

    def _handle_back_button(self) -> bool:
        if self._controller_button_pressed('BACK'):
            self.request_shutdown(f"{get_time()}-接收到 BACK，准备退出...")
            return True
        return False

    def _perform_angle_return(self) -> None:
        print(f"\n{get_time()}-触发角度归零...")
        while not self.motors.angle_return():
            busy_maintain_target_frequency(60, time.perf_counter())
        self.motors.stop()
        # 归零完成后：强制将软件缓存的 M2 目标重置为 0，
        # 确保下次手动控制时从 0 开始累加，不会跳回旧位置
        with self.manual_input_lock:
            self.manual_targets['m2'] = 0.0
            self.manual_targets_filtered['m2'] = 0.0
        print(f"{get_time()}-角度归零完成")

    def _set_zero_point(self) -> None:
        print(f"{get_time()}-设定当前位置为零点（写入ROM，三轴同步，保持手动模式）...")
        self.motors.set_current_position_as_zero_point()  # ★ 三轴（M0/M1/M2）均写入零点
        # 写入 ROM 后：重置三轴软件缓存目标，确保与电机实际零位一致
        with self.manual_input_lock:
            self.manual_targets['m0'] = 0.0
            self.manual_targets['m1'] = 0.0
            self.manual_targets['m2'] = 0.0
            self.manual_targets_filtered['m0'] = 0.0
            self.manual_targets_filtered['m1'] = 0.0
            self.manual_targets_filtered['m2'] = 0.0
        print(f"{get_time()}-零点设定完成（M0/M1/M2）")

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(min(value, limit), -limit)

    @staticmethod
    def _clamp_angle(value: float, limit_pair: Tuple[float, float]) -> float:
        return max(min(value, limit_pair[1]), limit_pair[0])

    def _sync_vision_targets(self, m1: Optional[float] = None, m2: Optional[float] = None) -> None:
        """确保视觉位置控制的目标角度与当前角度同步，避免突然跳变"""
        if m1 is None or m2 is None:
            try:
                _, m1, m2 = self.motors.get_angles()
            except Exception:
                m1 = self.vision_targets.get('m1', 0.0)
                m2 = self.vision_targets.get('m2', 0.0)
        if m1 is None:
            m1 = 0.0
        if m2 is None:
            m2 = 0.0
        self.vision_targets['m1'] = float(m1)
        self.vision_targets['m2'] = float(m2)
        # initialize filtered vision targets to avoid jumps
        self.vision_targets_filtered['m1'] = float(m1)
        self.vision_targets_filtered['m2'] = float(m2)

    def _sync_manual_targets(self, m0: Optional[float] = None, m1: Optional[float] = None, m2: Optional[float] = None) -> None:
        """Ensure manual position targets start from current motor positions to avoid jumps."""
        if m0 is None or m1 is None or m2 is None:
            try:
                m0, m1, m2 = self.motors.get_cached_angles()
            except Exception:
                m0 = self.manual_targets.get('m0', 0.0)
                m1 = self.manual_targets.get('m1', 0.0)
                m2 = self.manual_targets.get('m2', 0.0)
        if m0 is None:
            m0 = 0.0
        if m1 is None:
            m1 = 0.0
        if m2 is None:
            m2 = 0.0
        with self.manual_input_lock:
            self.manual_targets['m0'] = float(m0)
            self.manual_targets['m1'] = float(m1)
            self.manual_targets['m2'] = float(m2)
            # Initialize filtered targets the same as raw on sync
            self.manual_targets_filtered['m0'] = float(m0)
            self.manual_targets_filtered['m1'] = float(m1)
            self.manual_targets_filtered['m2'] = float(m2)

    def _vision_position_speed(self, magnitude: float) -> float:
        """根据输入强度返回合适的最大速度（可由实例属性 scale）"""
        if magnitude > 15:
            base = M1_SPEED_COEFF
        elif magnitude > 6:
            base = M1_SPEED_COEFF * 0.7
        else:
            base = M1_SPEED_COEFF * 0.4
        return base * getattr(self, "vision_speed_multiplier", 1.0)

    @staticmethod
    def _sign(value: float) -> float:
        if value >= 0:
            return 1.0
        return -1.0

    @staticmethod
    def _safe_float(value) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return 0.0

    def _transition_to_idle(self) -> None:
        self.state = 'Idle'
        self.on_enter_Idle()

    def _transition_to_manual(self) -> None:
        self.state = 'ManualControl'
        self.on_enter_ManualControl()

    def _transition_to_vision(self) -> None:
        self.state = 'VisionControl'
        self.on_enter_VisionControl()

    def on_enter_AIAuto(self) -> None:
        """进入 AIAuto 状态时的钩子。"""
        self._ai_frame_counter   = 0
        self._ai_motor_fail_count = 0
        self._autonav_last_mode = None
        if self._autonav_full_inspection:
            enabled_layers = ["Layer 1"]
            if self._autonav_enable_layer2:
                enabled_layers.append("Layer 2 近中心微调")
            if self._autonav_enable_layer3:
                enabled_layers.append("Layer 3 阻塞物清除")
            layer_text = " + ".join(enabled_layers)
            print(f"{get_time()}-进入全流程自主巡检模式（{layer_text}）")
            if self.voice_window:
                self.voice_window.push_log(
                    f"[AI] 进入全流程自主巡检（{layer_text}）"
                )
                self.voice_window.push_log("[AI] START/B/LB → 停止回到空闲")
                self.voice_window.update_state(
                    "AIAuto", "左上→左下→右上→右中→右下→TR"
                )
            return
        goal_code = _BC_PATHS.get(self._bc_goal_id, ('?', '?'))[0]
        goal_name = _BC_NAMES.get(self._bc_goal_id, '?')
        print(f"{get_time()}-进入 AI 自主巡检模式 → 目标: [{goal_code}] {goal_name}")
        if self.voice_window:
            self.voice_window.push_log(f"[AI] 进入自主巡检 → [{goal_code}] {goal_name}")
            self.voice_window.push_log("[AI] START/B/LB → 停止回到空闲")
            self.voice_window.update_state("AIAuto", f"目标: {goal_code} {goal_name}")

    def _transition_to_ai_auto(
        self,
        goal_id: Optional[int] = None,
        return_origin: bool = False,
        full_inspection: bool = False,
    ) -> None:
        if goal_id is not None:
            self._bc_goal_id = goal_id
            if self._bc_runner is not None:
                self._bc_runner.set_goal(goal_id)
            # AutoNav 不持有 goal_id 状态，下一次 loop_ai_auto 懒加载时使用新值
            self._stop_autonav()
        elif self._bc_runner is not None:
            # 每次进入AI模式都重置GRU隐状态和EMA，避免上次残留
            self._bc_runner.reset()
            self._stop_autonav()
        # 必须在 state 切换为 AIAuto 之前设置，避免状态线程抢先按普通路径加载 TR。
        self._autonav_return_origin = bool(return_origin)
        self._autonav_full_inspection = bool(full_inspection)
        self._tr_return_started_at = None
        self.state = 'AIAuto'
        self.on_enter_AIAuto()

    def _transition_to_poweroff(self) -> None:
        self.state = 'PowerOff'
        self.on_enter_PowerOff()

    def _transition_to_return_origin(self) -> None:
        """切换到独立三轴归零状态，彻底绕过 AutoNav/SequencePlayer。"""
        self._stop_autonav()
        self._autonav_demo_file = None
        self._autonav_return_origin = True
        self.state = 'ReturnOrigin'
        self.on_enter_ReturnOrigin()

    # ---------------------------
    # 语音交互
    # ---------------------------
    def _setup_voice_components(self) -> None:
        try:
            self.voice_window = VoiceStatusWindow()
            self.voice_window.start()
        except Exception as exc:
            self.voice_window = None
            print(f"语音窗口初始化失败：{exc}")
        # UI 按钮回调必须无条件注册，与语音识别是否可用无关
        if self.voice_window:
            self.voice_window.set_command_callback(self._on_ui_command)
            self.voice_window.set_motor_callback(self._on_motor_control)
            self.voice_window.set_exit_callback(self._on_exit_request)

        if sr is None and not baidu_speech:
            print("未检测到语音识别库，语音控制不可用（UI按钮仍可正常使用）")
            return
        try:
            self.voice_control = VoiceCommandCenter(self.voice_window)
            self.voice_control.start()
            print(f"{get_time()}-语音控制：已启动（手动模式）")
            # 录音按钮回调只在语音模块成功时注册
            if self.voice_window:
                self.voice_window.set_record_callback(self._on_record_button)
        except Exception as exc:
            self.voice_control = None
            print(f"语音控制初始化失败：{exc}（UI按钮仍可正常使用）")
            if self.voice_window:
                self.voice_window.push_log(f"语音控制初始化失败: {exc}")
    
    def _setup_action_recorder(self) -> None:
        """初始化动作采集和导航"""
        try:
            self.action_recorder = ActionRecorder(self.motors)
            self.action_player = ActionPlayer(
                self.motors,
                update_callback=self._on_action_progress_update,
                finish_callback=self._on_action_playback_finished,
            )

            # ── BC 专家演示采集器 ──────────────────────────────────────
            self.bc_collector: Optional[Any] = None
            if _BC_AVAILABLE and BronchusDataCollector is not None:
                try:
                    import os as _os2
                    bc_demo_dir = _os2.path.join(_os2.path.dirname(__file__), "BC", "expert_demos", "AutoNavdatasets")
                    self.bc_collector = BronchusDataCollector(
                        motor_group=self.motors,
                        save_dir=bc_demo_dir,
                        record_hz=20.0,
                        path_label=2,   # 默认 LMB，会在采集开始时按 UI 选择更新
                    )
                    print(f"{get_time()}-BC 采集器已初始化 → {bc_demo_dir}")
                except Exception as bc_exc:
                    self.bc_collector = None
                    print(f"{get_time()}-BC 采集器初始化失败: {bc_exc}")
            # ──────────────────────────────────────────────────────────

            # 设置导航回调
            if self.voice_window:
                self.voice_window.set_action_record_callback(self._on_action_record)
                self.voice_window.set_nav_callbacks(
                    start_cb=self._on_nav_ui_start,
                    pause_cb=self._on_nav_ui_pause,
                    stop_cb=self._on_nav_ui_stop,
                    inspection_plan_cb=self._on_full_inspection_ui_plan,
                    inspection_cb=self._on_full_inspection_ui_start,
                )

            print(f"{get_time()}-自主导航：已初始化")
        except Exception as exc:
            print(f"动作采集和导航初始化失败：{exc}")
            if self.voice_window:
                self.voice_window.push_log(f"动作采集和导航初始化失败：{exc}")

    @staticmethod
    def _resolve_inspection_selections(
        selections: Dict[str, str],
    ) -> Dict[str, str]:
        """把 UI 文件名解析为受限于 AutoNavdatasets 目录的绝对路径。"""
        demo_dir = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "BC", "expert_demos", "AutoNavdatasets"
        ))
        resolved: Dict[str, str] = {}
        for code, _label, name in _DEFAULT_INSPECTION_ROUTE:
            filename = str(selections.get(code, "")).strip()
            if not filename or filename == "（无可用轨迹）":
                raise ValueError(f"尚未选择{name} [{code}]轨迹")
            if os.path.basename(filename) != filename:
                raise ValueError(f"{name} [{code}]轨迹文件名无效")
            path = os.path.abspath(os.path.join(demo_dir, filename))
            if os.path.dirname(path) != demo_dir or not os.path.isfile(path):
                raise ValueError(f"{name} [{code}]轨迹不存在: {filename}")
            resolved[code] = path
        return resolved

    def _on_full_inspection_ui_plan(
        self,
        selections: Dict[str, str],
    ) -> None:
        """验证五条指定轨迹并计算所有公共点，成功后保存本次规划。"""
        self._autonav_inspection_plan = None
        if _InspectionSequencePlayer is None:
            message = "规划失败：AutoNav 全流程规划模块未加载"
            if self.voice_window:
                self.voice_window.update_inspection_plan_status(message)
                self.voice_window.push_log(f"[自主巡检] {message}")
            return

        try:
            selected_files = self._resolve_inspection_selections(selections)
            planner = _InspectionSequencePlayer(
                os.path.dirname(next(iter(selected_files.values()))),
                source_files=selected_files,
            )
        except Exception as exc:
            message = f"规划失败：{exc}"
            if self.voice_window:
                self.voice_window.update_inspection_plan_status(message)
                self.voice_window.push_log(f"[自主巡检] {message}")
            return

        self._autonav_inspection_plan = dict(selected_files)
        message = (
            f"路径已规划完成：{planner.total_steps} 步 / "
            f"{len(planner.transitions)} 个公共点，可以开始全流程自主巡检"
        )
        if self.voice_window:
            self.voice_window.update_inspection_plan_status(
                message,
                success=True,
                planned_selection=selections,
            )
            self.voice_window.push_log(f"[自主巡检] {message}")
            for code, _label, name in _DEFAULT_INSPECTION_ROUTE:
                self.voice_window.push_log(
                    f"[自主巡检] {name} [{code}] ← "
                    f"{os.path.basename(selected_files[code])}"
                )

    def _on_full_inspection_ui_start(
        self,
        selections: Dict[str, str],
        enable_layer2: bool = False,
        enable_layer3: bool = False,
    ) -> None:
        """启动已规划的五叶段全流程，可选 Layer 2/3。"""
        if self.bc_collector is not None and self.bc_collector.is_recording:
            if self.voice_window:
                self.voice_window.push_log("[自主巡检] 请先结束并保存当前现场采集")
            return

        try:
            current_selection = self._resolve_inspection_selections(selections)
        except ValueError as exc:
            if self.voice_window:
                self.voice_window.push_log(f"[自主巡检] 启动取消：{exc}")
            return
        if self._autonav_inspection_plan is None:
            if self.voice_window:
                self.voice_window.push_log(
                    "[自主巡检] 启动取消：请先选择五条轨迹并点击“规划全流程”"
                )
            return
        if current_selection != self._autonav_inspection_plan:
            if self.voice_window:
                self.voice_window.push_log(
                    "[自主巡检] 启动取消：轨迹选择已变化，请重新规划"
                )
            return

        try:
            angles = tuple(float(value) for value in self.motors.get_angles())
        except Exception as exc:
            if self.voice_window:
                self.voice_window.push_log(f"[自主巡检] 无法读取电机角度: {exc}")
            return

        outside_origin = any(
            abs(angles[index]) > _INSPECTION_ORIGIN_TOLERANCE[index]
            for index in range(3)
        )
        if outside_origin:
            if self.voice_window:
                self.voice_window.push_log(
                    "[自主巡检] 启动取消：请先用 TR/归零将三轴返回出发原点，"
                    f"当前=({angles[0]:.1f}, {angles[1]:.1f}, {angles[2]:.1f})°"
                )
            return

        self._autonav_demo_file = None
        self._autonav_enable_layer2 = bool(enable_layer2)
        self._autonav_enable_layer3 = bool(enable_layer3)
        if self.voice_window:
            self.voice_window.push_log(
                "[自主巡检] 开始执行已规划的五条拼接轨迹"
            )
            self.voice_window.push_log(
                "[自主巡检] Layer 2："
                + ("近中心保守微调已开启" if enable_layer2 else "关闭（纯 Layer 1）")
            )
            self.voice_window.push_log(
                "[自主巡检] Layer 3："
                + (
                    "大面积阻塞物抢占清除与断点续航已开启"
                    if enable_layer3 else "关闭"
                )
            )
            self.voice_window.push_log(
                "[自主巡检] 顺序：左上→左下→右上→右中→右下→TR 原点"
            )
        self._transition_to_ai_auto(goal_id=4, full_inspection=True)

    def _on_nav_ui_start(
        self,
        target_text: str,
        enable_layer2: bool = False,
        enable_layer3: bool = False,
    ) -> None:
        """UI 开始导航回调"""
        if self.bc_collector is not None and self.bc_collector.is_recording:
            if self.voice_window:
                self.voice_window.push_log("[导航] 请先点击“结束并保存”完成当前现场采集")
                self.voice_window.update_collection_status("状态：请先结束当前采集，再开始导航")
            return
        if not target_text:
            if self.voice_window:
                self.voice_window.push_log("[导航] 请先选择目标部位")
            return
        
        # 从 "LMB - 左主支气管 | filename.json" 中提取路径和原始采集文件。
        path_part, separator, filename = target_text.partition("|")
        path_code = path_part.split(" - ")[0].strip() if " - " in path_part else ""
        goal_id = YOLO_CODE_TO_IDX.get(path_code)
        
        if goal_id is None:
            if self.voice_window:
                self.voice_window.push_log(f"[导航] 未知的路径代码: {path_code}")
            return

        if path_code == "TR":
            self._bc_goal_id = goal_id
            self._transition_to_return_origin()
            if self.voice_window:
                self.voice_window.push_log("[导航] 目标: TR - 三轴固定原点")
            return
        
        # 更新导航状态
        self._bc_goal_id = goal_id
        demo_dir = os.path.join(os.path.dirname(__file__), "BC", "expert_demos", "AutoNavdatasets")
        demo_file = os.path.abspath(os.path.join(demo_dir, filename.strip())) if separator else None
        if not demo_file or not os.path.isfile(demo_file) or os.path.dirname(demo_file) != os.path.abspath(demo_dir):
            if self.voice_window:
                self.voice_window.push_log("[导航] 选中的采集路径不存在，请重新启动程序刷新列表")
            return
        self._autonav_demo_file = demo_file
        self._autonav_return_origin = False
        self._autonav_enable_layer2 = bool(enable_layer2)
        self._autonav_enable_layer3 = bool(enable_layer3)
        goal_name = _BC_NAMES.get(goal_id, path_code)
        if self.voice_window:
            self.voice_window.push_log(
                f"[导航] 目标: {path_code} - {goal_name} | 路径: {os.path.basename(demo_file)}"
            )
        
        # 停止之前的导航任务
        self._stop_autonav()
        
        # 进入 AI 自动模式
        self._transition_to_ai_auto(goal_id=goal_id)

    def _on_nav_ui_pause(self) -> None:
        """UI 暂停/恢复导航回调"""
        if self._autonav is None:
            if self.voice_window:
                self.voice_window.push_log("[导航] 当前无可用导航")
            return
        
        mode = self._autonav._mode if hasattr(self._autonav, '_mode') else None
        if mode and mode.value == "paused":
            self._autonav.resume()
            if self.voice_window:
                self.voice_window.push_log("[导航] 已恢复")
        elif mode and mode.value == "navigating":
            self._autonav.pause()
            if self.voice_window:
                self.voice_window.push_log("[导航] 已暂停")
        else:
            if self.voice_window:
                self.voice_window.push_log(f"[导航] 当前状态无法暂停: {mode}")

    def _on_nav_ui_stop(self) -> None:
        """UI 停止导航回调"""
        self._stop_autonav()
        self._transition_to_idle()
        if self.voice_window:
            self.voice_window.push_log("[导航] 已停止")

    def _is_action_playing(self) -> bool:
        """判断是否正在执行动作导航（播放/暂停/倒放）"""
        if not self.action_player:
            return False
        return self.action_player.state in (
            PlaybackState.PLAYING,
            PlaybackState.PAUSED,
            PlaybackState.REVERSING,
        )

    def _update_action_list(self) -> None:
        """更新动作列表"""
        if self.action_player and self.voice_window:
            actions = self.action_player.get_available_actions()
            self.voice_window.update_action_list(actions)
    
    def _on_action_record(self, start: bool) -> None:
        """
        采集回调：
          - ActionRecorder 采集已关闭（不再写 recorded_actions/）
          - BronchusDataCollector 负责完整的 BC 训练数据采集
          - ActionPlayer 导航功能保持不变（读取已有的 recorded_actions/*.json）
        """
        if start:
            # ── 读取 UI 路径选择 ──────────────────────────────────
            path_label = 2  # 默认 LMB
            sel = ""
            if self.voice_window and self.voice_window.bc_path_var is not None:
                try:
                    sel = self.voice_window.bc_path_var.get()   # e.g. "LMB - 左主支气管"
                    code = sel.split(" - ")[0].strip().upper()
                    path_label = YOLO_CODE_TO_IDX.get(code, 2)
                except Exception:
                    pass

            code_str = sel.split(" - ")[0].strip() if sel else "LMB"

            if code_str in ("", "EXP", "TR"):
                if self.voice_window:
                    self.voice_window.push_log("[采集] 请选择 TR 以外的具体目标部位")
                    self.voice_window.update_collection_status("状态：目标部位无效")
                return

            # 现场采集必须由手动控制产生。停止任何导航任务后自动进入手动模式。
            if self.state != 'ManualControl':
                self._stop_autonav()
                self._transition_to_manual()

            # ── 启动 BC 采集器 ────────────────────────────────────
            if self.bc_collector is not None:
                from datetime import datetime as _dt
                session = _dt.now().strftime("%Y%m%d_%H%M%S")
                self.bc_collector.path_label = path_label
                if self.bc_collector.start(session, path_label):
                    if self.voice_window:
                        self.voice_window.push_log(f"[BC] 开始采集 → [{code_str}]")
                        self.voice_window.update_collection_status(
                            f"状态：正在采集 [{code_str}]｜请使用手柄或 UI 完成介入动作"
                        )
                else:
                    if self.voice_window:
                        self.voice_window.push_log("[BC] 启动失败：可能已在采集中")
                        self.voice_window.update_collection_status("状态：启动失败，当前可能已在采集")
            else:
                if self.voice_window:
                    self.voice_window.push_log("[BC] 采集器未初始化，请检查 BC 模块")
                    self.voice_window.update_collection_status("状态：采集器不可用")

        else:
            # ── 停止 BC 采集器 ────────────────────────────────────
            if self.bc_collector is not None and self.bc_collector.is_recording:
                bc_path = self.bc_collector.stop()
                if bc_path and self.voice_window:
                    import os as _os
                    self.voice_window.push_log(f"[BC] 已保存 → {_os.path.basename(bc_path)}")
                    self.voice_window.update_collection_status(
                        f"状态：采集完成｜{_os.path.basename(bc_path)}",
                        refresh_paths=True,
                    )
                elif self.voice_window:
                    self.voice_window.push_log("[BC] 帧数不足（< 10 帧），已丢弃")
                    self.voice_window.update_collection_status("状态：帧数不足，未保存")
            elif self.voice_window:
                self.voice_window.push_log("[BC] 当前未在采集")
                self.voice_window.update_collection_status("状态：当前没有正在进行的采集")
    
    def _on_action_play(self, file_path: str) -> None:
        """动作播放回调"""
        if not self.action_player:
            return
        
        if self.action_player.start_playback(file_path, reverse=False):
            if self.voice_window:
                self.voice_window.push_log(f"开始播放：{file_path}")
        else:
            if self.voice_window:
                self.voice_window.push_log(f"播放失败：{file_path}")
    
    def _on_action_pause(self) -> None:
        """动作暂停回调"""
        if self.action_player:
            self.action_player.pause_playback()
            if self.voice_window:
                state = self.action_player.state.value
                self.voice_window.push_log(f"导航状态：{state}")
    
    def _on_action_stop(self) -> None:
        """动作停止回调"""
        if self.action_player:
            self.action_player.stop_playback()
            if self.voice_window:
                self.voice_window.push_log("导航已停止")
    
    def _on_action_reverse(self) -> None:
        """动作倒放回调"""
        if not self.action_player:
            return
        
        # 获取当前选中的文件或正在播放的文件
        if self.voice_window and self.voice_window.action_listbox:
            selection = self.voice_window.action_listbox.curselection()
            if selection:
                file_path = self.voice_window.action_listbox.get(selection[0])
                if '|' in file_path:
                    file_path = file_path.split('|')[0].strip()
                if self.action_player.start_playback(file_path, reverse=True):
                    if self.voice_window:
                        self.voice_window.push_log(f"开始倒放：{file_path}")
            else:
                if self.voice_window:
                    self.voice_window.push_log("请先选择一个动作文件")
    
    def _on_action_speed_change(self, speed: float) -> None:
        """动作倍速改变回调"""
        if self.action_player:
            self.action_player.set_playback_speed(speed)
            if self.voice_window:
                self.voice_window.push_log(f"导航倍速设置为：{speed}x")
    
    def _on_action_progress_update(self, progress_data: Dict) -> None:
        """动作导航进度更新回调"""
        if self.voice_window:
            self.voice_window.update_action_progress(progress_data)

    def _on_action_playback_finished(self, completed: bool) -> None:
        """动作导航完成/终止后回调"""
        if self.voice_window:
            if completed:
                self.voice_window.push_log("动作导航完成")
            else:
                self.voice_window.push_log("动作导航已终止")
        # 导航结束后自动回到空闲保持，避免误操作
        if completed and self.state != 'Idle':
            self._transition_to_idle()
        # 刷新动作列表，便于继续选择
        self._update_action_list()
    
    def _on_ui_command(self, cmd: str) -> None:
        """处理UI按钮指令"""
        self._log_prompt(f"{get_time()}-UI指令：{cmd}")
        self.ui_command_queue.put((cmd, f"按钮点击-{cmd}"))
    
    def _on_motor_control(self, motor_id: int, value: float) -> None:
        """处理UI手动控制，统一到输入缓存"""
        self._set_ui_manual_input(motor_id, value)

    def _set_ui_manual_input(self, motor_id: int, normalized_value: float) -> None:
        """将 UI 控件的输入转成速度指令（-1~1 -> 实际速度）"""
        with self.manual_input_lock:
            # Restore UI -> speeds for M0/M1, keep M2 as per-cycle position delta
            if motor_id == 0:
                scaled = normalized_value * FORWARD_COEFF
                self.ui_manual_input['forward'] = scaled if abs(scaled) >= 1.0 else 0.0
            elif motor_id == 1:
                scaled = normalized_value * M1_SPEED_COEFF
                self.ui_manual_input['horizontal'] = scaled if abs(scaled) >= 1.0 else 0.0
            elif motor_id == 2:
                # per-cycle delta for M2 (degrees per loop)
                # 乘以 M2_DELTA_MULTIPLIER：减速器使输出变慢10倍，电机delta需同步放大
                per_cycle_turn = TURN_COEFF * M2_DELTA_MULTIPLIER / 60.0
                delta = normalized_value * per_cycle_turn
                self.ui_manual_input['vertical'] = delta if abs(delta) >= 0.01 else 0.0

    def _update_controller_inputs(self) -> None:
        if not self.xbox:
            with self.manual_input_lock:
                self.controller_manual_input['forward'] = 0.0
                self.controller_manual_input['horizontal'] = 0.0
                self.controller_manual_input['vertical'] = 0.0
            return
        # Read raw inputs
        right_trigger = self.xbox.get_trigger_value('RT')
        left_trigger = self.xbox.get_trigger_value('LT')
        right_stick_x = self.xbox.get_joystick_value('RX')
        left_stick_y = self.xbox.get_joystick_value('LY')

        # Restore forward and horizontal as speed controls; vertical remains per-cycle delta for position control
        forward_speed = 0.0
        if right_trigger > 0.05:
            forward_speed = right_trigger * FORWARD_COEFF
        elif left_trigger > 0.05:
            forward_speed = -left_trigger * FORWARD_COEFF

        horizontal_speed = 0.0
        if abs(right_stick_x) > 0.02:
            horizontal_speed = (right_stick_x * abs(right_stick_x)) * M1_SPEED_COEFF

        # vertical: per-cycle delta (deg per loop)，乘以减速器补偿倍率
        per_cycle_turn = TURN_COEFF * M2_DELTA_MULTIPLIER / 60.0
        vertical_delta = 0.0
        if abs(left_stick_y) > 0.02:
            vertical_delta = (left_stick_y * abs(left_stick_y)) * per_cycle_turn

        with self.manual_input_lock:
            self.controller_manual_input['forward'] = forward_speed
            self.controller_manual_input['horizontal'] = horizontal_speed
            self.controller_manual_input['vertical'] = vertical_delta

    def _resolve_manual_inputs(self) -> Tuple[float, float, float]:
        """融合 UI 与手柄输入（UI 优先级更高，可逐轴覆盖）"""
        with self.manual_input_lock:
            # Now values are position deltas per cycle (degrees per loop). UI overrides controller if larger.
            forward = self.ui_manual_input['forward'] if abs(self.ui_manual_input['forward']) > abs(self.controller_manual_input['forward']) \
                else self.controller_manual_input['forward']
            horizontal = self.ui_manual_input['horizontal'] if abs(self.ui_manual_input['horizontal']) > abs(self.controller_manual_input['horizontal']) \
                else self.controller_manual_input['horizontal']
            vertical = self.ui_manual_input['vertical'] if abs(self.ui_manual_input['vertical']) > abs(self.controller_manual_input['vertical']) \
                else self.controller_manual_input['vertical']
        return forward, horizontal, vertical

    def _apply_manual_motion(self, forward_speed: float, horizontal_speed: float, vertical_speed: float) -> None:
        """统一在主线程驱动电机，避免多源并发

        功能说明（按轴分开处理）：
        - M0 (前进/后退)：以速度控制为主（move_forward），当速度小于阈值则停止。
        - M1 (左右转向)：以速度控制为主（map_horizontal），当速度小于阈值则停止。
        - M2 (上下俯仰)：以位置控制为主，vertical_speed 表示本帧的角度增量（度）。通过累积 manual_targets 实现位置闭环控制，
          并在需要时发送位置命令；若位置命令未生效再使用短时速度脉冲作为回退。

        说明每个判断的设计意图（便于未来维护）：
        - 对 M0/M1 使用速度控制可以保证响应更快、并避免频繁位置设置导致的抖动或通信开销；
        - 对 M2 使用位置控制以保证俯仰角的稳定性与精度；当位置变化微小或未生效时再使用短脉冲补偿。
        """
        try:
            # ------------------------
            # M0: 前进/后退（速度控制）
            # ------------------------
            try:
                # 只有当 forward_speed 足够大时才发送速度命令，避免微小噪声触发
                if abs(forward_speed) > 5.0:
                    self.motors.move_forward(forward_speed)
                else:
                    self.motors.m0.stop()
            except Exception as exc:
                print(f"{get_time()}-M0 speed control error: {exc}")

            # ------------------------
            # M1: 水平转向（速度控制）
            # ------------------------
            try:
                # 同样对水平速度设阈值，低于阈值则停止
                if abs(horizontal_speed) > 2.0:
                    self.motors.map_horizontal(horizontal_speed)
                else:
                    self.motors.m1.stop()
            except Exception as exc:
                print(f"{get_time()}-M1 speed control error: {exc}")

            # M2 垂直轴：速度控制 + 精准硬件限位
            # map_vertical → _apply_limited_speed：每帧读取真实电机位置、预测下帧越界则拉回限位
            # 彻底解决纯软件累积跟踪导致的限位失效问题
            try:
                with self.manual_input_lock:
                    d2 = vertical_speed * self.m2_sensitivity_multiplier
                # d2(°/frame) × 60Hz = °/s 速度命令；speed=0 时 _apply_limited_speed 内部自动 stop()
                m2_spd = max(-M2_SPEED_COEFF, min(M2_SPEED_COEFF, d2 * 60.0))
                self.motors.map_vertical(m2_spd)  # 读真实位置 + 精准限位 + 速度控制
                # 软件缓存同步（仅供显示，不再作为限位依据）
                limit_lo, limit_hi = self.motors.m2_limit
                raw = self.manual_targets['m2'] + d2
                self.manual_targets['m2'] = max(min(raw, limit_hi), limit_lo)
                self.manual_targets_filtered['m2'] = self.manual_targets['m2']

            except Exception as exc:
                print(f"{get_time()}-M2执行异常: {exc}")
        except (ValueError, Exception) as exc:
            error_msg = f"手动控制执行异常：{exc}"
            print(f"{get_time()}-{error_msg}")
            if self.voice_window:
                self.voice_window.push_log(error_msg)

    def _vision_position_mode(self, forward_input: float, horizontal_input: float, vertical_input: float) -> Dict[str, float]:
        """视觉模式：根据输入直接计算目标角度并触发位置控制"""
        try:
            # ------------------------
            # M0: 前进/后退（速度控制）
            # ------------------------
            try:
                # 只有当 forward_speed 足够大时才发送速度命令，避免微小噪声触发
                if abs(forward_input) > 5.0:
                    self.motors.move_forward(forward_input)
                else:
                    self.motors.m0.stop()
            except Exception as exc:
                print(f"{get_time()}-M0 speed control error: {exc}")

            # ------------------------
            # M1: 水平转向（速度控制）
            # ------------------------
            try:
                # 同样对水平速度设阈值，低于阈值则停止
                if abs(horizontal_input) > VISION_M1_STOP_INPUT:
                    self.motors.map_horizontal(horizontal_input)
                else:
                    self.motors.m1.stop()
            except Exception as exc:
                print(f"{get_time()}-M1 speed control error: {exc}")

            # M2 垂直轴带减速器、实测响应快：视觉侧 vy_max=8，按4倍缩放为
            # 最大32 deg/s；不复用M1的直接速度映射。
            try:
                m2_spd = max(-M2_SPEED_COEFF, min(M2_SPEED_COEFF, vertical_input * VISION_M2_SPD_SCALE))
                if abs(m2_spd) > VISION_M2_STOP_SPEED:
                    self.motors.map_vertical(m2_spd)
                else:
                    with self.motors.serial_lock:
                        try:
                            self.motors.m2.stop()
                        except Exception:
                            pass
            except Exception as exc:
                print(f"{get_time()}-M2执行异常: {exc}")
        except (ValueError, Exception) as exc:
            error_msg = f"手动控制执行异常：{exc}"
            print(f"{get_time()}-{error_msg}")
            if self.voice_window:
                self.voice_window.push_log(error_msg)

        

    def _vision_position_delta(self, input_value: float, gain: float) -> float:
        magnitude = abs(input_value)
        if magnitude < 1e-3:
            return 0.0
        if magnitude <= VISION_POSITION_INPUT_DEADZONE:
            # 死区内仍然保持最小步长，确保持续逼近
            delta = VISION_POSITION_MIN_STEP * (1 if input_value > 0 else -1)
        else:
            delta = gain * input_value
            if abs(delta) < VISION_POSITION_MIN_STEP:
                delta = VISION_POSITION_MIN_STEP * (1 if delta > 0 else -1)
        return self._clamp(delta, VISION_POSITION_MAX_STEP)

    def _clear_manual_inputs(self) -> None:
        with self.manual_input_lock:
            for entry in (self.ui_manual_input, self.controller_manual_input):
                entry['forward'] = 0.0
                entry['horizontal'] = 0.0
                entry['vertical'] = 0.0

    def _current_manual_source(self) -> str:
        with self.manual_input_lock:
            if any(abs(v) > 0.5 for v in self.ui_manual_input.values()):
                return "UI"
            if any(abs(v) > 0.5 for v in self.controller_manual_input.values()):
                return "Controller" if self.xbox else "None"
        return "None"

    def _controller_button_pressed(self, button: str) -> bool:
        return bool(self.xbox and self.xbox.is_button_pressed(button))
    
    def _on_exit_request(self) -> None:
        """处理退出请求（切换到空闲保持状态）"""
        self._log_prompt(f"{get_time()}-用户请求退出程序（切换到空闲状态）")
        # 切换到空闲状态
        if self.state != 'Idle':
            self._transition_to_idle()
        self._clear_manual_inputs()
        # 直接设置退出事件，不经过PowerOff
        self.shutdown_event.set()

    def _on_record_button(self, is_pressed: bool) -> None:
        """录音按钮按下/释放的回调"""
        voice_control = self.voice_control
        if not voice_control:
            return
        def worker():
            try:
                if is_pressed:
                    voice_control.start_manual_record()
                else:
                    voice_control.stop_manual_record()
            except Exception as exc:
                if self.voice_window:
                    self.voice_window.push_log(f"录音控制异常：{exc}")
        threading.Thread(target=worker, daemon=True).start()

    def _process_voice_commands(self) -> None:
        if self.voice_control:
            while True:
                try:
                    command, raw = self.voice_control.command_queue.get_nowait()
                except queue.Empty:
                    break
                self._execute_voice_command(command, raw)
        while True:
            try:
                command, raw = self.ui_command_queue.get_nowait()
            except queue.Empty:
                break
            self._execute_voice_command(command, raw)

    def _execute_voice_command(self, command: str, raw_text: str) -> None:
        next_state = None
        if command == "manual":
            next_state = "手动控制"
            self._transition_to_manual()
        elif command == "vision":
            next_state = "视觉自主控制"
            self._transition_to_vision()
        elif command == "idle":
            next_state = "空闲保持"
            self._transition_to_idle()
        elif command == "zero":
            next_state = "执行角度归零"
            self._perform_angle_return()
        elif command == "set_zero":
            next_state = "写入当前位置为零点"
            self._set_zero_point()
        elif command == "stop":
            next_state = "空闲保持"
            print(f"{get_time()}-语音停止：进入空闲保持，等待下一步指令")
            if self._bc_runner is not None:
                self._bc_runner.reset()
            self._stop_autonav()
            self._transition_to_idle()
        elif command == "poweroff":
            next_state = "断电维护"
            print(f"{get_time()}-语音断电：执行断电维护")
            self._transition_to_poweroff()
        elif command == "ai_auto":
            next_state = "AI自主巡检"
            self._transition_to_ai_auto()
        # 直接指定目标支气管的快捷指令（goal_id 对应 BRONCHUS_PATHS）
        elif command.startswith("ai_"):
            code = command[3:].upper()          # e.g. "ai_lmb" → "LMB"
            gid  = YOLO_CODE_TO_IDX.get(code)
            if gid is not None:
                next_state = f"AI自主→{code}"
                if code == "TR":
                    self._bc_goal_id = gid
                    self._transition_to_return_origin()
                else:
                    self._transition_to_ai_auto(goal_id=gid)
            else:
                next_state = "未定义命令"
        else:
            next_state = "未定义命令"
        if self.voice_window:
            self.voice_window.push_log(f"语音指令“{raw_text}” -> {next_state}")
            self.voice_window.update_state(self.state, next_state)

    def _update_voice_state(self, current: str, next_state: Optional[str] = None) -> None:
        if self.voice_window:
            self.voice_window.update_state(current, next_state)

    def _log_prompt(self, text: str) -> None:
        """Prints prompts and mirrors them into the UI log."""
        print(text)
        if self.voice_window:
            self.voice_window.push_log(text)


def robot_thread() -> None:
    robot = VisionRobotStateMachine()
    robot.start()
    try:
        robot.join()
    except KeyboardInterrupt:
        robot.request_shutdown(f"{get_time()}-收到中断信号，退出中...")
        robot.join()


def main() -> None:
    print(f"{get_time()}-main2025-vision 启动：创建线程...")
    if VOICE_SWITCH:
        print(f"{get_time()}-Voice_control=ON（手动录音模式）")

    video_thread = None
    if CAMERA_SWITCH:
        video_thread = threading.Thread(
            target=video_processing,
            name="video_thread",
            daemon=ROBOT_SWITCH,
        )
        video_thread.start()
        print(f"{get_time()}-video_thread 已启动 (Camera_switch=True)")
    else:
        print(f"{get_time()}-Camera_switch=False，跳过视觉线程启动")

    if not ROBOT_SWITCH:
        print(f"{get_time()}-Robot_switch=False，跳过机器人控制线程")
        return

    robot_thread_obj = threading.Thread(target=robot_thread, name="robot_thread")
    robot_thread_obj.start()

    robot_thread_obj.join()
    print(f"{get_time()}-robot_thread 已结束，程序退出。")


if __name__ == '__main__':
    main()
