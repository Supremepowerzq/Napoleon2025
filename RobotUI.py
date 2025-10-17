import ttkbootstrap as ttk
from ttkbootstrap.constants import *
import tkinter as tk
from pathlib import Path
from tkinter import PhotoImage

from PIL import Image, ImageTk


def main():
    app = ttk.Window(title="Robot UI", themename="yeti")
    robotui = RobotUI(app)
    app.mainloop()


PATH = Path(__file__).parent / 'assets'

class RobotUI(ttk.Frame):
    def __init__(self, master=None, **kw):
        super().__init__(master, **kw)
        self.master = master
        self.load_images()
        self.initialize_ui()

    def load_images(self):
        """加载并调整所需的图片资源"""
        self.images = {
            'refresh': self.load_and_resize_image(PATH / 'icon/刷新.png', (32, 32)),
            'side_view': self.load_and_resize_image(PATH / '模型侧视图.png', (600, 200)),
        }

    def initialize_ui(self):
        """初始化用户界面的布局和组件"""
        self.configure_grid()
        self.setup_hardware_info_section()
        self.setup_status_info_section()
        self.setup_motor_state_section()

    def configure_grid(self):
        """配置网格布局"""
        self.pack(fill=tk.BOTH, expand=True)
        for i in range(3):
            self.columnconfigure(i, weight=1)
        self.rowconfigure(0, weight=1)

    def setup_hardware_info_section(self):
        """设置硬件信息区域"""
        col1 = ttk.Frame(self, padding=10)
        col1.grid(row=0, column=0, sticky=tk.NSEW)

        hardware_frame = self.create_label_frame(col1, "设备信息", top=True)
        self.setup_hardware_frame_header(hardware_frame)
        ttk.Label(hardware_frame, image=self.images['side_view']).pack(fill=tk.BOTH, expand=True)
        self.setup_hardware_frame_footer(hardware_frame)

    def create_label_frame(self, parent, title, top=False):
        """创建并返回一个标签框架"""
        frame = ttk.LabelFrame(parent, text=title, padding=10)
        if top:
            frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        else:
            frame.pack(fill=tk.BOTH, expand=True)
        return frame

    def setup_hardware_frame_header(self, parent):
        """设置硬件信息区域的头部"""
        header = ttk.Frame(parent, padding=5)
        header.pack(fill=tk.BOTH, expand=True)

        refresh_button = ttk.Button(header, image=self.images['refresh'], bootstyle='link')
        refresh_button.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        ttk.Label(header, text='介入设备').pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    def setup_hardware_frame_footer(self, parent):
        """设置硬件信息区域的底部"""
        footer = ttk.Frame(parent, padding=5)
        footer.pack(fill=tk.BOTH, expand=True)
        self.hardware_labels = self.create_hardware_labels(footer, ['电机串口', 'xbox手柄', '外部相机', '内部相机'])

    def create_hardware_labels(self, parent, hardware_items):
        """为每个硬件创建标签，并返回一个字典，键为硬件名"""
        labels = {}
        for hardware in hardware_items:
            label = ttk.Label(parent, text=hardware.upper(), bootstyle="danger")
            label.pack(padx=10, pady=5, side=tk.LEFT, fill=tk.BOTH, expand=True)
            labels[hardware] = label
        return labels

    def setup_status_info_section(self):
        """设置状态信息区域"""
        col1 = self.nametowidget(".!robotui.!frame")  # 获取第一列的Frame

        # 创建状态信息的LabelFrame
        state_frame = self.create_label_frame(col1, "状态信息", top=True)
        state_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, pady=(10, 0))
        state_frame.rowconfigure(0, weight=1)
        state_frame.columnconfigure(0, weight=2)

        # 状态标题
        state_title = ttk.Label(state_frame, text='机器人状态', anchor='center')
        state_title.pack(fill=tk.X, pady=(0, 20))

        self.state_image_label = ttk.Label(state_frame, image=self.images['side_view'])
        self.state_image_label.pack(fill=tk.BOTH, expand=True)

    def update_image(self, image_path):
        """
        更新指定Label上的图片内容。

        :param image_path: 新图片的路径。
        """
        # 加载新图片并调整大小
        new_image = self.load_and_resize_image(image_path, (600, 200))
        # 更新Label上的图片
        self.state_image_label.configure(image=new_image)
        # 保存新图片引用，防止被垃圾回收
        self.current_image = new_image  # 将新图片保存为实例属性

    def setup_motor_state_section(self):
        """设置电机状态区域"""
        col2 = ttk.Frame(self, padding=10)  # 创建第二列的Frame
        col2.grid(row=0, column=1, sticky=tk.NSEW)

        # 创建电机状态的LabelFrame
        motor_state_frame = self.create_label_frame(col2, "电机状态", top=True)
        motor_state_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.motor_labels = {}
        for i in range(7):
            # 为每个电机创建一个LabelFrame
            motor_label_frame = self.create_label_frame(motor_state_frame, f"电机{i+1}", top=True)
            motor_label_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, pady=5)

            # 为每个电机状态创建标签
            self.create_motor_state_labels(motor_label_frame, i)

    def create_motor_state_labels(self, parent, motor_id):
        """为电机创建状态标签"""
        label_speed = ttk.Label(parent, text="速度: 0.0 DPS")
        label_speed.grid(row=0, column=0, sticky=tk.W, padx=20)
        label_position = ttk.Label(parent, text="位置: 0.0 °")
        label_position.grid(row=1, column=0, sticky=tk.W, padx=20)
        label_current = ttk.Label(parent, text="电流: 0.0 A")
        label_current.grid(row=0, column=1, sticky=tk.W, padx=20)
        label_temperature = ttk.Label(parent, text="温度: 0.0 ℃")
        label_temperature.grid(row=1, column=1, sticky=tk.W, padx=20)
        self.motor_labels[motor_id] = {
            'speed': label_speed,
            'position': label_position,
            'current': label_current,
            'temperature': label_temperature
        }

    def update_motor_states(self, motor_data):
        """更新电机状态信息"""
        for motor_id, data in motor_data.items():
            if motor_id in self.motor_labels:
                self.update_label(self.motor_labels[motor_id]['speed'], f"速度: {round(data['speed'], 2)} DPS")
                self.update_label(self.motor_labels[motor_id]['position'], f"位置: {round(data['position'], 2)} °")
                self.update_label(self.motor_labels[motor_id]['current'], f"电流: {round(data['current'], 2)} A")
                self.update_label(self.motor_labels[motor_id]['temperature'], f"温度: {round(data['temperature'], 2)} ℃")
            else:
                print(f"电机{motor_id}的标签不存在于UI中。")

    def update_label(self, label, text):
        """更新标签文本"""
        label.config(text=text)

    def load_and_resize_image(self, image_path, size):
        """加载并调整图片大小"""
        with Image.open(image_path) as img:
            img_resized = img.resize(size, Image.Resampling.LANCZOS)
            return ImageTk.PhotoImage(img_resized)

# class RobotUI(ttk.Frame):
#     def __init__(self, master=None, **kw):
#         super().__init__(master, **kw)
#         self.master = master
#         self.images = {
#             '刷新': self.__load_and_resize_image(PATH / 'icon/刷新.png', (32, 32)),
#             '模型侧视图': self.__load_and_resize_image(PATH / '模型侧视图.png', (600, 200)),
#         }


#         self.init_ui()

#     def init_ui(self):
#         self.pack(fill=BOTH, expand=YES)
#         for i in range(3):
#             self.columnconfigure(i, weight=1)
#         self.rowconfigure(0, weight=1)

#         col1 = ttk.Frame(self, padding=10)
#         col1.grid(row=0, column=0, sticky=NSEW)

#         # 硬件信息
#         hardware_frame = ttk.LabelFrame(col1, text="设备信息", padding=10)
#         hardware_frame.pack(side=TOP, fill=BOTH, expand=YES)

#         hardware_frame_header = ttk.Frame(hardware_frame, padding=5)
#         hardware_frame_header.pack(fill=BOTH, expand=YES)

#         btn = ttk.Button(
#             master=hardware_frame_header,
#             image=self.images['刷新'],
#             bootstyle=LINK
#         )
#         btn.pack(side=LEFT, fill=BOTH, expand=YES)

#         lbl = ttk.Label(hardware_frame_header, text='介入设备')
#         lbl.pack(side=LEFT, fill=BOTH, expand=YES)


#         ttk.Label(hardware_frame, image=self.images['模型侧视图']).pack(fill=BOTH, expand=YES)

#         hardware_frame_footer = ttk.Frame(hardware_frame, padding=5)
#         hardware_frame_footer.pack(fill=BOTH, expand=YES)

#         self.hardware_labels = {}
#         for hardware in ['电机串口', 'xbox手柄', '外部相机', '内部相机']:
#             label = ttk.Label(hardware_frame_footer, text=hardware.upper(), bootstyle="danger")
#             label.pack(padx=10, pady=5, side=tk.LEFT, fill=BOTH, expand=YES)
#             self.hardware_labels[hardware] = label


#         # 状态信息
#         state_frame = ttk.LabelFrame(col1, text="状态信息", padding=20)
#         state_frame.pack(side=TOP, fill=BOTH, expand=YES, pady=(10, 0))
#         state_frame.rowconfigure(0, weight=1)
#         state_frame.columnconfigure(0, weight=2)

#         state_title = ttk.Label(
#             master=state_frame,
#             text='机器人状态',
#             anchor=CENTER
#         )
#         state_title.pack(fill=X, pady=(0, 20))

#         col2 = ttk.Frame(self, padding=10)
#         col2.grid(row=0, column=1, sticky=NSEW)

#         motor_state_frame = ttk.LabelFrame(col2, text="电机状态", padding=10)
#         motor_state_frame.pack(side=TOP, fill=BOTH, expand=YES)

#         self.motor_labels = {}
#         for i in range(7):
#             labelframe = ttk.LabelFrame(motor_state_frame, text=f"电机{i}", padding=10)
#             labelframe.pack(side=TOP, fill=BOTH, expand=YES, pady=5)
#             label_speed = ttk.Label(labelframe, text="速度: 0.0 DPS")
#             label_speed.grid(row=0, column=0, sticky=W)
#             label_position = ttk.Label(labelframe, text="位置: 0.0 °")
#             label_position.grid(row=1, column=0, sticky=W)
#             label_current = ttk.Label(labelframe, text="电流: 0.0 A")
#             label_current.grid(row=0, column=1, sticky=W)
#             label_temperature = ttk.Label(labelframe, text="温度: 0.0 ℃")
#             label_temperature.grid(row=1, column=1, sticky=W)
#             self.motor_labels[i] = {
#                 'speed': label_speed,
#                 'position': label_position,
#                 'current': label_current,
#                 'temperature': label_temperature
#             }

#     def update_motor_states(self, motor_data):
#         """
#         更新电机状态的信息。
        
#         :param motor_data: 一个字典，键是电机编号（0到6），值是包含速度、位置、电流和温度信息的字典。
#         """
#         for motor_id, data in motor_data.items():
#             # 确保传入的电机ID存在于当前的电机标签字典中
#             if motor_id in self.motor_labels:
#                 # 更新速度标签
#                 self.motor_labels[motor_id]['speed'].config(text=f"速度: {data['speed']} DPS")
#                 # 更新位置标签
#                 self.motor_labels[motor_id]['position'].config(text=f"位置: {data['position']} °")
#                 # 更新电流标签
#                 self.motor_labels[motor_id]['current'].config(text=f"电流: {data['current']} A")
#                 # 更新温度标签
#                 self.motor_labels[motor_id]['temperature'].config(text=f"温度: {data['temperature']} ℃")
#             else:
#                 print(f"电机{motor_id}的标签不存在于UI中。")


#     def __load_and_resize_image(self, image_path, size=(16, 16)):
#         with Image.open(image_path) as img:
#             img_resized = img.resize(size, Image.Resampling.LANCZOS)
#             return ImageTk.PhotoImage(img_resized)
        
if __name__ == '__main__':
    main()