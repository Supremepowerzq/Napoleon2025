"""
语音识别与状态机切换测试脚本
不依赖手柄、相机、电机等硬件，仅测试语音识别和状态机逻辑
"""

import time
import threading
import queue
from typing import Optional, Tuple, Dict, Any, Callable, TYPE_CHECKING
from datetime import datetime

if TYPE_CHECKING:
    import tkinter as tk

# 优先使用Baidu语音识别（中文识别效果更好）
try:
    from aip import AipSpeech  # type: ignore[import]
    baidu_speech = True
except ImportError:
    baidu_speech = False
    AipSpeech = None
    print("提示：未安装 baidu-aip，如需使用Baidu语音识别，请运行: pip install baidu-aip")

# 备用：speech_recognition（如果Baidu未配置）
try:
    import speech_recognition as sr  # type: ignore[import]
    import pyaudio  # type: ignore[import]
except ImportError:
    sr = None
    pyaudio = None
    print("警告：未安装 speech_recognition 或 pyaudio，请运行: pip install speechrecognition pyaudio")

try:
    import tkinter as tk  # type: ignore[import]
    from tkinter.scrolledtext import ScrolledText  # type: ignore[import]
except Exception:
    tk = None
    ScrolledText = None
    print("警告：Tkinter 不可用，窗口功能将禁用")

# 语音交互配置
VOICE_WAKE_WORD = "你好助手"  # 更容易识别的唤醒词
VOICE_RECOGNITION_LANGUAGE = "zh-CN"

# Baidu语音识别配置（需要到 https://ai.baidu.com/ 申请）
# 如果未配置，将回退到 speech_recognition
BAIDU_APP_ID = "7271638"  # Baidu语音识别 APP ID
BAIDU_API_KEY = "sHhvqZyhyh2decNp6Q75rEc8"  # Baidu语音识别 API Key
BAIDU_SECRET_KEY = "c1u9Rzeca7FSxabjxJUlGHqckjjOgozD"  # Baidu语音识别 Secret Key
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


def get_time() -> str:
    """获取当前时间字符串"""
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


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
        
        # UI Components
        self.canvas_record: Any = None
        self.record_light: Any = None
        self.record_btn_window: Any = None

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
            self.root.geometry("800x1200")
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
        elif itype == "close":
            self.stop_event.set()

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

    def stop(self) -> None:
        if not self.enabled:
            return
        self.queue.put({"type": "close"})
        self.stop_event.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)


class VoiceCommandCenter:
    """语音识别：手动录音模式（支持Baidu语音识别和 speech_recognition）"""

    def __init__(self, window: Optional[VoiceStatusWindow]) -> None:
        self.window = window
        self.command_queue: queue.Queue[Tuple[str, str]] = queue.Queue()
        self.stop_event = threading.Event()
        # 手动录音相关
        self.manual_record_event = threading.Event()
        self.manual_record_stop_event = threading.Event()
        self.manual_record_audio: Optional[bytes] = None
        self.manual_record_lock = threading.Lock()
        
        # 检查使用哪种识别方式
        self.use_baidu = False
        self.baidu_client: Any = None
        self.recognizer: Any = None
        self.microphone: Any = None
        self._sr: Any = None
        self._pyaudio: Any = None
        
        # 优先使用Baidu语音识别
        if baidu_speech and AipSpeech and BAIDU_APP_ID and BAIDU_API_KEY and BAIDU_SECRET_KEY:
            try:
                self.baidu_client = AipSpeech(BAIDU_APP_ID, BAIDU_API_KEY, BAIDU_SECRET_KEY)
                self.use_baidu = True
                if pyaudio:
                    self._pyaudio = pyaudio
            except Exception as exc:
                if self.window:
                    self.window.push_log(f"Baidu语音识别初始化失败：{exc}，将使用备用方案")
                self.use_baidu = False
        
        # 备用方案：speech_recognition
        if not self.use_baidu:
            if sr is None or pyaudio is None:
                raise RuntimeError(
                    "未安装语音识别库。请选择以下方案之一：\n"
                    "1. 安装Baidu语音识别：pip install baidu-aip pyaudio\n"
                    "2. 安装备用方案：pip install speechrecognition pyaudio\n"
                    "并在代码中配置Baidu API 密钥（推荐）"
                )
            self._sr = sr
            self.recognizer = self._sr.Recognizer()
            self.microphone = self._sr.Microphone()

    def start(self) -> None:
        if self.window:
            self.window.push_log("语音模块已就绪")
        
        if self.use_baidu:
            if self.window:
                self.window.push_log("使用Baidu语音识别")
        else:
            if self.microphone:
                try:
                    with self.microphone as source:
                        self.recognizer.adjust_for_ambient_noise(source, duration=1)
                except Exception:
                    pass
            if self.window:
                self.window.push_log("使用备用识别方案")

    def stop(self) -> None:
        self.stop_event.set()

    def _record_audio_baidu(self, duration: float = 2.0, stop_event: Optional[threading.Event] = None) -> Optional[bytes]:
        """使用 pyaudio 录制音频（Baidu格式：16k采样率，16bit，单声道）"""
        if not self._pyaudio:
            return None
        try:
            chunk = 1024
            sample_format = self._pyaudio.paInt16
            channels = 1
            fs = 16000  # Baidu要求16k采样率
            
            p = self._pyaudio.PyAudio()
            stream = p.open(
                format=sample_format,
                channels=channels,
                rate=fs,
                frames_per_buffer=chunk,
                input=True
            )
            
            frames = []
            if stop_event:
                # 手动录音模式：持续录音直到stop_event被设置
                while not stop_event.is_set():
                    data = stream.read(chunk, exception_on_overflow=False)
                    frames.append(data)
            else:
                # 自动录音模式：按duration时长录音
                for _ in range(0, int(fs / chunk * duration)):
                    data = stream.read(chunk)
                    frames.append(data)
            
            stream.stop_stream()
            stream.close()
            p.terminate()
            
            return b''.join(frames)
        except Exception as exc:
            if self.window:
                self.window.push_log(f"录音失败：{exc}")
            return None
    
    def start_manual_record(self) -> None:
        """开始手动录音"""
        if not self.use_baidu or not self._pyaudio:
            if self.window:
                self.window.push_log("手动录音功能需要Baidu语音识别")
            return
        
        self.manual_record_stop_event.clear()
        self.manual_record_audio = None
        
        # 在后台线程中录音
        def record_thread():
            if self.window:
                self.window.update_recording_status(True)
            audio = self._record_audio_baidu(duration=0, stop_event=self.manual_record_stop_event)
            with self.manual_record_lock:
                self.manual_record_audio = audio
            if self.window:
                self.window.update_recording_status(False)
        
        threading.Thread(target=record_thread, daemon=True).start()
    
    def stop_manual_record(self) -> None:
        """停止手动录音并识别"""
        self.manual_record_stop_event.set()
        # 等待录音完成
        time.sleep(0.3)
        
        with self.manual_record_lock:
            audio = self.manual_record_audio
            self.manual_record_audio = None
        
        if audio and len(audio) > 0:
            # 识别录音
            if self.window:
                self.window.push_log("正在识别录音...")
            text = self._recognize_baidu(audio)
            if text:
                self._handle_text(text.strip())
            else:
                if self.window:
                    self.window.push_log("未识别到有效语音")
        else:
            if self.window:
                self.window.push_log("录音为空，请重新尝试")

    def _recognize_baidu(self, audio_data: bytes) -> Optional[str]:
        """使用BaiduAPI识别语音"""
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
                    self.window.push_log(f"Baidu识别错误：{err_msg}")
                # 如果是请求频率过高，抛出特殊异常
                if 'pv too much' in str(err_msg).lower() or 'request' in str(err_msg).lower():
                    raise RuntimeError("API调用频率过高，请稍后再试")
        except Exception as exc:
            if self.window:
                self.window.push_log(f"Baidu识别异常：{exc}")
            raise  # 重新抛出异常，让调用者处理
        return None

    # 已移除自动监听循环，只保留手动录音功能

    def _handle_text(self, text: str) -> None:
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


class TestStateMachine:
    """简化的四态状态机：空闲保持、手动控制、视觉控制、断电维护"""

    states = ('Idle', 'ManualControl', 'VisionControl', 'PowerOff')

    def __init__(self, voice_window: Optional[VoiceStatusWindow]) -> None:
        print(f"{get_time()}-测试状态机启动：初始化...")
        self.state = 'Idle'
        self.shutdown_event = threading.Event()
        self.voice_window = voice_window
        self.voice_control: Optional[VoiceCommandCenter] = None
        self._setup_voice_components()
        self._log_prompt(
            f"{get_time()}-初始化完成，进入空闲状态\n"
            f"  可用指令：切换为手动/自动/空闲模式、归零、写入零点、紧急停止"
        )
        self._update_voice_state("空闲保持")

    def start(self) -> None:
        """启动状态机循环"""
        self._state_loop()

    def request_shutdown(self, reason: Optional[str] = None) -> None:
        if reason:
            print(reason)
        if self.state != 'PowerOff':
            self._transition_to_poweroff()
        self.shutdown_event.set()

    def on_enter_Idle(self) -> None:
        self._log_prompt(f"{get_time()}-状态切换：空闲保持")
        self._update_voice_state("空闲保持")

    def on_enter_ManualControl(self) -> None:
        self._log_prompt(f"{get_time()}-状态切换：手动控制（模拟）")
        self._update_voice_state("手动控制")

    def on_enter_VisionControl(self) -> None:
        self._log_prompt(f"{get_time()}-状态切换：视觉自主控制（模拟）")
        self._update_voice_state("视觉自主控制")

    def on_enter_PowerOff(self) -> None:
        self._log_prompt(f"{get_time()}-状态切换：断电维护（模拟）")
        self._update_voice_state("断电维护")

    def _state_loop(self) -> None:
        """状态机主循环"""
        try:
            while not self.shutdown_event.is_set():
                self._process_voice_commands()
                time.sleep(0.1)  # 降低CPU占用
        except KeyboardInterrupt:
            self.request_shutdown(f"{get_time()}-收到中断信号，退出中...")
        finally:
            self._cleanup()

    def _cleanup(self) -> None:
        """清理资源"""
        if self.voice_control:
            self.voice_control.stop()
        if self.voice_window:
            self.voice_window.stop()
        print(f"{get_time()}-测试状态机已退出")

    def _setup_voice_components(self) -> None:
        """设置语音组件"""
        if sr is None:
            print("未检测到 speech_recognition，语音控制不可用")
            return
        try:
            self.voice_control = VoiceCommandCenter(self.voice_window)
            self.voice_control.start()
            print(f"{get_time()}-控制：已启动（手动录音模式）")
            
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
    
    def _on_record_button(self, is_pressed: bool) -> None:
        """录音按钮按下/释放的回调"""
        if not self.voice_control:
            return
        if is_pressed:
            self.voice_control.start_manual_record()
        else:
            self.voice_control.stop_manual_record()

    def _on_ui_command(self, cmd: str) -> None:
        """处理UI按钮指令"""
        self._log_prompt(f"{get_time()}-UI指令：{cmd}")
        self._execute_voice_command(cmd, f"按钮点击-{cmd}")
    
    def _on_motor_control(self, motor_id: int, value: float) -> None:
        """处理UI手动控制（模拟）"""
        # 在测试脚本中，只记录日志
        if abs(value) > 0.01:
            self._log_prompt(f"{get_time()}-手动控制：电机{motor_id} = {value:.2f}")
        # 在main脚本中，此处会调用实际的电机控制
    
    def _on_exit_request(self) -> None:
        """处理退出请求（切换到空闲保持状态）"""
        self._log_prompt(f"{get_time()}-用户请求退出程序（切换到空闲状态）")
        # 切换到空闲状态
        if self.state != 'Idle':
            self._transition_to_idle()
        # 直接设置退出事件，不经过PowerOff
        self.shutdown_event.set()

    def _process_voice_commands(self) -> None:
        """处理语音指令队列"""
        if not self.voice_control:
            return
        while True:
            try:
                command, raw = self.voice_control.command_queue.get_nowait()
            except queue.Empty:
                break
            self._execute_voice_command(command, raw)

    def _execute_voice_command(self, command: str, raw_text: str) -> None:
        """执行语音指令"""
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
            next_state = "执行角度归零（模拟）"
            self._log_prompt(f"{get_time()}-模拟执行：角度归零")
            if self.voice_window:
                self.voice_window.push_log("模拟：电机归零操作")
        elif command == "set_zero":
            next_state = "写入当前位置为零点（模拟）"
            self._log_prompt(f"{get_time()}-模拟执行：写入零点")
            if self.voice_window:
                self.voice_window.push_log("模拟：写入当前位置为零点")
        elif command == "stop":
            next_state = "空闲保持"
            self._log_prompt(f"{get_time()}-停止：进入空闲保持，等待下一步指令")
            self._transition_to_idle()
        elif command == "poweroff":
            next_state = "断电维护"
            self._log_prompt(f"{get_time()}-断电：执行断电维护")
            self._transition_to_poweroff()
        else:
            next_state = "未定义命令"
        
        if self.voice_window:
            self.voice_window.push_log(f"语音指令'{raw_text}' -> {next_state}")
            self.voice_window.update_state(self.state, next_state)

    def _update_voice_state(self, current: str, next_state: Optional[str] = None) -> None:
        """更新语音窗口状态显示"""
        if self.voice_window:
            self.voice_window.update_state(current, next_state)

    def _transition_to_idle(self) -> None:
        """切换到空闲状态"""
        self.state = 'Idle'
        self.on_enter_Idle()

    def _transition_to_manual(self) -> None:
        """切换到手动控制状态"""
        self.state = 'ManualControl'
        self.on_enter_ManualControl()

    def _transition_to_vision(self) -> None:
        """切换到视觉控制状态"""
        self.state = 'VisionControl'
        self.on_enter_VisionControl()

    def _transition_to_poweroff(self) -> None:
        """切换到断电维护状态"""
        self.state = 'PowerOff'
        self.on_enter_PowerOff()

    def _log_prompt(self, text: str) -> None:
        """显示提示：终端+UI"""
        print(text)
        if self.voice_window:
            self.voice_window.push_log(text)


def main() -> None:
    """主函数"""
    print(f"{get_time()}-语音识别与状态机测试脚本启动")
    print("=" * 60)
    
    # 检查依赖
    if sr is None:
        print("错误：未安装 speech_recognition")
        print("请运行: pip install speechrecognition pyaudio")
        return
    
    # 创建语音窗口
    voice_window = VoiceStatusWindow()
    voice_window.start()
    
    # 创建并启动测试状态机
    state_machine = TestStateMachine(voice_window)
    
    try:
        # 在主线程中运行状态机（会阻塞）
        state_machine.start()
    except KeyboardInterrupt:
        print(f"\n{get_time()}-收到中断信号，退出中...")
        state_machine.request_shutdown()
    
    print(f"{get_time()}-测试脚本已退出")


if __name__ == '__main__':
    main()
