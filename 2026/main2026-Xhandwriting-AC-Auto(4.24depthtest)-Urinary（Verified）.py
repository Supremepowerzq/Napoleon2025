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

from config import BAUDRATE, TIMEOUT, get_config
# 将大型视觉模块延迟导入，避免在不需要时加载 torchvision 等重依赖导致环境相关错误
UnetPackage = None
from ActionRecorder import ActionRecorder, ActionPlayer, PlaybackState

# 电机控制参数
FORWARD_COEFF = 100  # 前进/后退速度系数
TURN_COEFF = 50     # 转向速度系数（降为一半）
# 视觉侧输出的 x>0 表示需要顺时针旋转，因此需要在此处做一次符号映射
# 如果硬件接线导致方向相反，只需把该值改成 1.0
HORIZONTAL_CLOCKWISE_SIGN = 1.0

# 视觉控制调参（位置模式）
VISION_POSITION_GAIN_HORIZONTAL = 0.07
VISION_POSITION_GAIN_VERTICAL = 0.05
VISION_POSITION_MAX_STEP = 1.5  # 单次最大角度调节
VISION_POSITION_MIN_STEP = 0.1  # 在死区内的最小步长（电机最小响应角度）
VISION_POSITION_INPUT_DEADZONE = 0.1

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
        from predict_2026_wqx_3d_instruct import UnetPackage  # type: ignore[import]
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
        
        # 动作录制/回放回调
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
        
        # 动作录制/回放UI组件
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
            self.root.geometry("800x1800")
            self.root.configure(bg="#f0f2f5")  # 浅灰背景，类似现代应用

            # 主容器，模拟卡片效果
            main_card = tk.Frame(self.root, bg="white", padx=20, pady=20)
            main_card.pack(expand=True, fill=tk.BOTH, padx=20, pady=20)
            
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
            RoundedButton(row1, text="自动模式", width=100, height=40, bg="#e0e0e0", fg="#333", hover_bg="#d5d5d5", press_bg="#c0c0c0", command=lambda: self._on_command("vision")).pack(side=tk.LEFT, padx=10)
            RoundedButton(row1, text="停止", width=100, height=40, bg="#FF9800", hover_bg="#F57C00", press_bg="#EF6C00", command=lambda: self._on_command("stop")).pack(side=tk.LEFT, padx=10)
            
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

            # 动作录制/回放区域
            action_frame = tk.Frame(main_card, bg="white")
            action_frame.pack(pady=20, fill=tk.X)
            
            tk.Label(action_frame, text="动作录制与回放", font=("Microsoft YaHei", 12, "bold"), bg="white", anchor="w").pack(fill=tk.X, pady=(0, 10))
            
            # 录制控制
            record_control_frame = tk.Frame(action_frame, bg="white")
            record_control_frame.pack(fill=tk.X, pady=5)
            
            self.action_record_btn = RoundedButton(
                record_control_frame,
                text="开始录制",
                width=100,
                height=40,
                bg="#4CAF50",
                hover_bg="#45a049",
                press_bg="#3d8b40",
                command=lambda: self._on_action_record(True)
            )
            self.action_record_btn.pack(side=tk.LEFT, padx=5)
            
            self.action_stop_record_btn = RoundedButton(
                record_control_frame,
                text="停止录制",
                width=100,
                height=40,
                bg="#f44336",
                hover_bg="#d32f2f",
                press_bg="#b71c1c",
                command=lambda: self._on_action_record(False)
            )
            self.action_stop_record_btn.pack(side=tk.LEFT, padx=5)
            
            # 回放控制
            playback_control_frame = tk.Frame(action_frame, bg="white")
            playback_control_frame.pack(fill=tk.X, pady=5)
            
            self.action_play_btn = RoundedButton(
                playback_control_frame,
                text="播放",
                width=80,
                height=35,
                bg="#2196F3",
                hover_bg="#1976D2",
                press_bg="#0D47A1",
                command=self._on_action_play
            )
            self.action_play_btn.pack(side=tk.LEFT, padx=3)
            
            self.action_pause_btn = RoundedButton(
                playback_control_frame,
                text="暂停",
                width=80,
                height=35,
                bg="#FF9800",
                hover_bg="#F57C00",
                press_bg="#EF6C00",
                command=self._on_action_pause
            )
            self.action_pause_btn.pack(side=tk.LEFT, padx=3)
            
            self.action_stop_btn = RoundedButton(
                playback_control_frame,
                text="停止",
                width=80,
                height=35,
                bg="#9E9E9E",
                hover_bg="#757575",
                press_bg="#616161",
                command=self._on_action_stop
            )
            self.action_stop_btn.pack(side=tk.LEFT, padx=3)
            
            self.action_reverse_btn = RoundedButton(
                playback_control_frame,
                text="倒放",
                width=80,
                height=35,
                bg="#9C27B0",
                hover_bg="#7B1FA2",
                press_bg="#6A1B9A",
                command=self._on_action_reverse
            )
            self.action_reverse_btn.pack(side=tk.LEFT, padx=3)
            
            # 倍速选择
            speed_frame = tk.Frame(playback_control_frame, bg="white")
            speed_frame.pack(side=tk.LEFT, padx=10)
            
            tk.Label(speed_frame, text="倍速:", font=("Microsoft YaHei", 9), bg="white").pack(side=tk.LEFT, padx=5)
            self.action_speed_var = tk.StringVar(value="1.0x")
            speed_options = ["0.5x", "1.0x", "2.0x"]
            speed_menu = tk.OptionMenu(speed_frame, self.action_speed_var, *speed_options, command=lambda v: self._on_speed_change(v))
            speed_menu.config(width=8, font=("Microsoft YaHei", 9))
            speed_menu.pack(side=tk.LEFT)
            
            # 进度显示
            progress_frame = tk.Frame(action_frame, bg="white")
            progress_frame.pack(fill=tk.X, pady=10)
            
            self.action_time_var = tk.StringVar(value="总时长: 0.0s | 已播放: 0.0s | 剩余: 0.0s")
            tk.Label(progress_frame, textvariable=self.action_time_var, font=("Microsoft YaHei", 9), bg="white", anchor="w").pack(fill=tk.X, pady=5)
            
            self.action_progress_var = tk.StringVar(value="0%")
            progress_label_frame = tk.Frame(progress_frame, bg="white")
            progress_label_frame.pack(fill=tk.X)
            tk.Label(progress_label_frame, text="进度:", font=("Microsoft YaHei", 9), bg="white").pack(side=tk.LEFT)
            tk.Label(progress_label_frame, textvariable=self.action_progress_var, font=("Microsoft YaHei", 9), bg="white").pack(side=tk.LEFT, padx=5)
            
            self.action_progress_bar = tk.Canvas(progress_frame, height=20, bg="#e0e0e0", highlightthickness=1, highlightbackground="#999")
            self.action_progress_bar.pack(fill=tk.X, pady=5)
            self._draw_progress_bar(0.0)
            
            # 动作列表
            list_frame = tk.Frame(action_frame, bg="white")
            list_frame.pack(fill=tk.BOTH, expand=True, pady=10)
            
            tk.Label(list_frame, text="已录制的动作:", font=("Microsoft YaHei", 10, "bold"), bg="white", anchor="w").pack(fill=tk.X, pady=(0, 5))
            
            listbox_frame = tk.Frame(list_frame, bg="white")
            listbox_frame.pack(fill=tk.BOTH, expand=True)
            
            scrollbar = tk.Scrollbar(listbox_frame)
            scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
            
            self.action_listbox = tk.Listbox(listbox_frame, font=("Consolas", 9), yscrollcommand=scrollbar.set, height=5)
            self.action_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
            scrollbar.config(command=self.action_listbox.yview)
            
            self.action_listbox.bind('<Double-Button-1>', self._on_action_listbox_select)

            # 日志区域
            log_frame = tk.Frame(main_card, bg="white")
            log_frame.pack(expand=True, fill=tk.BOTH, pady=10)
            
            tk.Label(log_frame, text="运行日志", font=("Microsoft YaHei", 10, "bold"), bg="white", anchor="w").pack(fill=tk.X)
            
            self.text_area = ScrolledText(log_frame, wrap=tk.WORD, font=("Consolas", 10), height=10, bg="#f5f5f5", relief=tk.FLAT)
            self.text_area.pack(expand=True, fill=tk.BOTH, pady=5)
            self.text_area.configure(state=tk.DISABLED)

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
        """设置动作录制回调"""
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
        """动作录制按钮回调"""
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
        """更新动作回放进度"""
        if not self.enabled:
            return
        self.queue.put({"type": "action_progress", "data": progress_data})
    
    def update_action_list(self, actions: List[Tuple[str, str]]) -> None:
        """更新动作列表"""
        if not self.enabled:
            return
        self.queue.put({"type": "action_list", "actions": actions})
    
    def _draw_progress_bar(self, progress: float) -> None:
        """绘制进度条"""
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
        """使用 pyaudio 录制音频（百度格式：16k采样率，16bit，单声道）"""
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
        self.m0_limit = (0.0, 900.0)     # 电机0的限位：0~900度
        self.m1_limit = (-180.0, 180.0)  # 电机1的限位：-180~+180度
        self.m2_limit = (-15.0, 15.0)    # 电机2的限位：-10~+10度

        # 串口访问锁，防止UI控制和手柄控制同时访问串口
        self.serial_lock = threading.Lock()

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
        供动作录制等对实时性要求不高的场景使用。
        """
        with self.serial_lock:
            try:
                return (float(self.m0.position), float(self.m1.position), float(self.m2.position))
            except Exception:
                return (0.0, 0.0, 0.0)

    def _apply_limited_speed(self, motor: RmdMotor, speed: float, limit_min: float, limit_max: float, axis_name: str) -> None:
        """
        统一的限位：
        1. 读取当前位置，估算新位置
        2. 执行限位检查，超限则直接拉回限位
        3. 在安全范围内按给定速度运行
        """
        try:
            if abs(speed) < 1e-3:
                motor.stop()
                return
            
            motor.update_state()
            current_pos = motor.position
            increment = speed / 60.0  # 以 60Hz 控制频率估算位置增量
            projected_pos = current_pos + increment
            # 速度模式的电机（M0/M1）使用较紧的公差带，确保抵住限位后完全静止
            limit_tolerance = 0.15

            # 当前已经抵住上限，且仍然往正方向推 -> 保持在上限
            if current_pos >= limit_max - limit_tolerance and speed > 0:
                motor.set_position(limit_max, max_speed=TURN_COEFF)
                motor.update_state()   # 同步缓存值
                motor.stop()          # 发出停止指令，解除速度闭环残留
                print(f"{get_time()}-电机{motor.id}停在上限：{current_pos:.1f}° -> {limit_max:.1f}°")
                return
            # 当前已经抵住下限，且仍然往负方向推 -> 保持在下限
            if current_pos <= limit_min + limit_tolerance and speed < 0:
                motor.set_position(limit_min, max_speed=TURN_COEFF)
                motor.update_state()
                motor.stop()
                print(f"{get_time()}-电机{motor.id}停在下限：{current_pos:.1f}° -> {limit_min:.1f}°")
                return

            # 预计即将越过上限（且指令仍为正） -> 拉回
            if projected_pos >= limit_max and speed > 0:
                motor.set_position(limit_max, max_speed=TURN_COEFF)
                motor.update_state()
                motor.stop()
                print(f"{get_time()}-电机{motor.id}触发上限限位：{current_pos:.1f}° -> {limit_max:.1f}°")
                return
            # 预计即将越过下限（且指令仍为负） -> 拉回
            if projected_pos <= limit_min and speed < 0:
                motor.set_position(limit_min, max_speed=TURN_COEFF)
                motor.update_state()
                motor.stop()
                print(f"{get_time()}-电机{motor.id}触发下限限位：{current_pos:.1f}° -> {limit_min:.1f}°")
                return
            
            # 在限制范围内，正常控制
            motor.set_speed(speed)
        except Exception as exc:
            print(f"{get_time()}-轴{axis_name}控制异常：{exc}")
    
    def move_forward(self, speed_forward: float) -> None:
        with self.serial_lock:
            try:
                self._apply_limited_speed(
                    motor=self.m0,
                    speed=speed_forward,
                    limit_min=self.m0_limit[0],
                    limit_max=self.m0_limit[1],
                    axis_name="M0"
                )
            except (ValueError, Exception) as exc:
                print(f"{get_time()}-电机M0控制错误: {exc}")

    def map_horizontal(self, value: float) -> None:
        with self.serial_lock:
            try:
                self._apply_limited_speed(
                    motor=self.m1,
                    speed=value,
                    limit_min=self.m1_limit[0],
                    limit_max=self.m1_limit[1],
                    axis_name="M1"
                )
            except (ValueError, Exception) as exc:
                print(f"{get_time()}-电机M1控制错误: {exc}")

    def map_vertical(self, value: float) -> None:
        with self.serial_lock:
            try:
                self._apply_limited_speed(
                    motor=self.m2,
                    speed=value,
                    limit_min=self.m2_limit[0],
                    limit_max=self.m2_limit[1],
                    axis_name="M2"
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

                # 电机1：使用位置闭环，精确归零
                if abs(current_angle1) > 0.1:  # 降低阈值提高精度
                    self.m1.set_position(0, max_speed=TURN_COEFF/2)  # 降低速度提高精度
                    finish = False
                
                # 电机2：使用位置闭环，精确归零
                if abs(current_angle2) > 0.1:
                    self.m2.set_position(0, max_speed=TURN_COEFF/3)  # 使用更低的速度
                    finish = False
                
                if finish:
                    # 完全停止
                    self.m1.stop()
                    self.m2.stop()
            except (ValueError, Exception) as exc:
                print(f"{get_time()}-角度归零错误: {exc}")
                finish = True  # 出错时认为已完成，避免卡死
        
        return finish

    def set_current_position_as_zero_point(self) -> None:
        with self.serial_lock:
            try:
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
    """四态状态机：空闲保持、手动控制、视觉自主、断电维护"""

    states = ('Idle', 'ManualControl', 'VisionControl', 'PowerOff')

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
        self.machine.add_transition('activate_manual', ['Idle', 'VisionControl', 'PowerOff'], 'ManualControl')
        self.machine.add_transition('activate_vision', ['Idle', 'ManualControl'], 'VisionControl')
        self.machine.add_transition('cut_power', '*', 'PowerOff')

        self.state_handlers = {
            'Idle': self.loop_idle,
            'ManualControl': self.loop_manual_control,
            'VisionControl': self.loop_vision_control,
            'PowerOff': self.loop_power_off,
        }

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

        # 动作录制和回放（不使用复杂类型标注，避免 Pylance 将模块误判为类型）
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

        if self._is_action_playing():
            # 动作回放期间禁止手动控制，优先保证回放指令
            if self.action_recorder and self.action_recorder.is_recording:
                self.action_recorder.record_action()
            return

        self._update_controller_inputs()
        forward_speed, horizontal_speed, vertical_speed = self._resolve_manual_inputs()
        self._apply_manual_motion(forward_speed, horizontal_speed, vertical_speed)
        
        # 录制动作
        if self.action_recorder and self.action_recorder.is_recording:
            self.action_recorder.record_action()

        try:
            angles = self.motors.get_angles()
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
            if self.action_recorder and self.action_recorder.is_recording:
                self.action_recorder.record_action()
            return

        # 录制动作
        if self.action_recorder and self.action_recorder.is_recording:
            self.action_recorder.record_action()

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

        # 调试（已移除打印）：在调用视觉位置计算前后打印详细信息，确保能看到 pos_info 内容
        pos_info = self._vision_position_mode(
            forward_input,
            horizontal_input * HORIZONTAL_CLOCKWISE_SIGN,
            vertical_input,
        )
        

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
        print(f"{get_time()}-设定当前位置为零点（写入ROM并重启）...")
        self.motors.set_current_position_as_zero_point()
        # 写入 ROM 后：强制将软件缓存的 M2 目标重置为 0，
        # 这样电机重启后，软件缓存的累积角度与实际 0 位置一致
        with self.manual_input_lock:
            self.manual_targets['m2'] = 0.0
            self.manual_targets_filtered['m2'] = 0.0
        print(f"{get_time()}-零点设定完成")

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
            base = TURN_COEFF
        elif magnitude > 6:
            base = TURN_COEFF * 0.7
        else:
            base = TURN_COEFF * 0.4
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

    def _transition_to_poweroff(self) -> None:
        self.state = 'PowerOff'
        self.on_enter_PowerOff()

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
        if sr is None and not baidu_speech:
            print("未检测到语音识别库，语音控制不可用")
            print("提示：可以使用百度API（需配置BAIDU_APP_ID等）或安装 speech_recognition")
            return
        try:
            self.voice_control = VoiceCommandCenter(self.voice_window)
            self.voice_control.start()
            print(f"{get_time()}-语音控制：已启动（手动模式）")
            
            # 设置录音按钮回调
            if self.voice_window:
                self.voice_window.set_record_callback(self._on_record_button)
                self.voice_window.set_command_callback(self._on_ui_command)
                self.voice_window.set_motor_callback(self._on_motor_control)
                self.voice_window.set_exit_callback(self._on_exit_request)
        except Exception as exc:
            self.voice_control = None
            print(f"语音控制初始化失败：{exc}")
            if self.voice_window:
                self.voice_window.push_log(f"语音控制初始化失败：{exc}")
    
    def _setup_action_recorder(self) -> None:
        """初始化动作录制和回放"""
        try:
            self.action_recorder = ActionRecorder(self.motors)
            self.action_player = ActionPlayer(
                self.motors,
                update_callback=self._on_action_progress_update,
                finish_callback=self._on_action_playback_finished,
            )
            
            # 更新动作列表
            if self.voice_window:
                self._update_action_list()
                # 设置回调
                self.voice_window.set_action_record_callback(self._on_action_record)
                self.voice_window.set_action_play_callback(self._on_action_play)
                self.voice_window.set_action_pause_callback(self._on_action_pause)
                self.voice_window.set_action_stop_callback(self._on_action_stop)
                self.voice_window.set_action_reverse_callback(self._on_action_reverse)
                self.voice_window.set_action_speed_callback(self._on_action_speed_change)
            
            print(f"{get_time()}-动作录制和回放：已初始化")
        except Exception as exc:
            print(f"动作录制和回放初始化失败：{exc}")
            if self.voice_window:
                self.voice_window.push_log(f"动作录制和回放初始化失败：{exc}")
    
    def _is_action_playing(self) -> bool:
        """判断是否正在执行动作回放（播放/暂停/倒放）"""
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
        """动作录制回调"""
        if not self.action_recorder:
            return
        
        if start:
            if self.action_recorder.start_recording():
                if self.voice_window:
                    self.voice_window.push_log("开始录制动作...")
            else:
                if self.voice_window:
                    self.voice_window.push_log("录制失败：可能正在录制中")
        else:
            file_path = self.action_recorder.stop_recording()
            if file_path:
                if self.voice_window:
                    self.voice_window.push_log(f"录制已保存：{file_path}")
                self._update_action_list()
            else:
                if self.voice_window:
                    self.voice_window.push_log("停止录制失败：可能未在录制")
    
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
                self.voice_window.push_log(f"回放状态：{state}")
    
    def _on_action_stop(self) -> None:
        """动作停止回调"""
        if self.action_player:
            self.action_player.stop_playback()
            if self.voice_window:
                self.voice_window.push_log("回放已停止")
    
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
                self.voice_window.push_log(f"回放倍速设置为：{speed}x")
    
    def _on_action_progress_update(self, progress_data: Dict) -> None:
        """动作回放进度更新回调"""
        if self.voice_window:
            self.voice_window.update_action_progress(progress_data)

    def _on_action_playback_finished(self, completed: bool) -> None:
        """动作回放完成/终止后回调"""
        if self.voice_window:
            if completed:
                self.voice_window.push_log("动作回放完成")
            else:
                self.voice_window.push_log("动作回放已终止")
        # 回放结束后自动回到空闲保持，避免误操作
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
                scaled = normalized_value * TURN_COEFF
                self.ui_manual_input['horizontal'] = scaled if abs(scaled) >= 1.0 else 0.0
            elif motor_id == 2:
                # per-cycle delta for M2 (degrees per loop)
                per_cycle_turn = TURN_COEFF / 60.0
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
            horizontal_speed = (right_stick_x * abs(right_stick_x)) * TURN_COEFF

        # vertical: per-cycle delta (deg per loop)
        per_cycle_turn = TURN_COEFF / 60.0
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

            # M2 垂直轴：位置控制，限位独立处理
            # 策略：每帧读一次位置（仅此处）、纯 clamp 裁剪、超过限位强制停止并同步缓存
            m2_done = False  # 本帧 M2 处理是否已完成（发命令或跳过）
            new_m2 = 0.0     # 本帧计算的最终目标角度

            try:
                self.motors.m2.update_state()
                cur_m2_pos = float(self.motors.m2.position)
            except Exception as exc:
                cur_m2_pos = self.manual_targets.get('m2', 0.0)

            try:
                with self.manual_input_lock:
                    d2 = vertical_speed * self.m2_sensitivity_multiplier

                    # 微小输入视为停止：发一次 stop 确保电机静止
                    if abs(d2) < 1e-4:
                        m2_done = True
                        try:
                            self.motors.m2.stop()
                        except Exception:
                            pass
                    else:
                        # 从软件缓存目标累加增量
                        raw_target = self.manual_targets['m2'] + d2
                        # 裁剪到限位范围
                        limit_lo, limit_hi = self.motors.m2_limit
                        clamped = max(min(raw_target, limit_hi), limit_lo)

                        # 判断本帧是否撞到了限位
                        at_limit = (clamped == limit_lo or clamped == limit_hi)

                        if at_limit:
                            # 撞限位：发 set_position 到限位角度，再 stop，最后同步缓存
                            new_m2 = clamped
                            try:
                                self.motors.m2.set_position(clamped, max_speed=TURN_COEFF)
                                self.motors.m2.stop()
                            except Exception as exc_limit:
                                print(f"{get_time()}-M2限位命令异常: {exc_limit}")
                            # 无论 set_position 是否成功，都要强制同步缓存到限位角度
                            self.manual_targets['m2'] = clamped
                            self.manual_targets_filtered['m2'] = clamped
                            m2_done = True
                            print(f"{get_time()}-M2限位：{cur_m2_pos:.2f}° -> {clamped:.2f}°")
                        else:
                            # 正常区间：累积到 clamped
                            new_m2 = clamped

                # with 块结束后处理发送（不在 with 内控制流）
                if not m2_done:
                    # 更新软件缓存（累积目标）
                    self.manual_targets['m2'] = new_m2
                    # 发送阈值过滤：只有与上次发送目标的差异足够大才发命令
                    last_sent = self.manual_targets_filtered.get('m2', new_m2)
                    if abs(new_m2 - last_sent) >= self.m2_deadband_deg:
                        self.manual_targets_filtered['m2'] = new_m2
                        try:
                            self.motors.m2.set_position(new_m2, max_speed=TURN_COEFF)
                        except Exception as exc:
                            print(f"{get_time()}-M2位置命令错误: {exc}")
                            if self.voice_window:
                                self.voice_window.push_log(f"M2 position set error: {exc}")

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
                if abs(horizontal_input) > 2.0:
                    self.motors.map_horizontal(horizontal_input)
                else:
                    self.motors.m1.stop()
            except Exception as exc:
                print(f"{get_time()}-M1 speed control error: {exc}")

            # M2 垂直轴：位置控制，限位独立处理（与手动模式逻辑一致）
            m2_done = False
            new_m2 = 0.0

            try:
                self.motors.m2.update_state()
                cur_m2_pos = float(self.motors.m2.position)
            except Exception as exc:
                cur_m2_pos = self.manual_targets.get('m2', 0.0)

            try:
                with self.manual_input_lock:
                    d2 = vertical_input * self.m2_sensitivity_multiplier

                    if abs(d2) < 1e-4:
                        m2_done = True
                        try:
                            self.motors.m2.stop()
                        except Exception:
                            pass
                    else:
                        raw_target = self.manual_targets['m2'] + d2
                        limit_lo, limit_hi = self.motors.m2_limit
                        clamped = max(min(raw_target, limit_hi), limit_lo)
                        at_limit = (clamped == limit_lo or clamped == limit_hi)

                        if at_limit:
                            new_m2 = clamped
                            try:
                                self.motors.m2.set_position(clamped, max_speed=TURN_COEFF)
                                self.motors.m2.stop()
                            except Exception as exc_limit:
                                print(f"{get_time()}-M2限位命令异常: {exc_limit}")
                            self.manual_targets['m2'] = clamped
                            self.manual_targets_filtered['m2'] = clamped
                            m2_done = True
                            print(f"{get_time()}-M2限位(vision)：{cur_m2_pos:.2f}° -> {clamped:.2f}°")
                        else:
                            new_m2 = clamped

                if not m2_done:
                    self.manual_targets['m2'] = new_m2
                    last_sent = self.manual_targets_filtered.get('m2', new_m2)
                    if abs(new_m2 - last_sent) >= self.m2_deadband_deg:
                        self.manual_targets_filtered['m2'] = new_m2
                        try:
                            self.motors.m2.set_position(new_m2, max_speed=TURN_COEFF)
                        except Exception as exc:
                            print(f"{get_time()}-M2位置命令错误: {exc}")
                            if self.voice_window:
                                self.voice_window.push_log(f"M2 position set error: {exc}")

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
            self._transition_to_idle()
        elif command == "poweroff":
            next_state = "断电维护"
            print(f"{get_time()}-语音断电：执行断电维护")
            self._transition_to_poweroff()
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


