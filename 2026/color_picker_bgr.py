import cv2
import numpy as np
import sys
import os
import json
from datetime import datetime
from PIL import ImageFont, ImageDraw, Image

class BGRColorPicker:
    def __init__(self):
        self.window_name = "BGR Color Picker - 实时调色工具"
        self.preview_window_name = "Color Preview - 颜色预览"

        # BGR颜色值 (OpenCV使用BGR格式)
        self.b = 0
        self.g = 0
        self.r = 0

        # 初始化中文字体
        self.font_path = self._get_font_path()
        try:
            self.font = ImageFont.truetype(self.font_path, 24)
        except:
            # 如果找不到字体，使用默认字体
            self.font = ImageFont.load_default()

        # 创建窗口
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.preview_window_name, cv2.WINDOW_NORMAL)

        # 创建滑块
        cv2.createTrackbar('Blue (B)', self.window_name, 0, 255, self.on_blue_change)
        cv2.createTrackbar('Green (G)', self.window_name, 0, 255, self.on_green_change)
        cv2.createTrackbar('Red (R)', self.window_name, 0, 255, self.on_red_change)

        # 设置窗口大小
        cv2.resizeWindow(self.window_name, 400, 200)
        cv2.resizeWindow(self.preview_window_name, 600, 400)

        # 颜色历史记录
        self.color_history = []
        self.max_history = 10

    def _get_font_path(self):
        """获取系统中文字体路径"""
        font_paths = [
            "C:/Windows/Fonts/simhei.ttf",      # 黑体
            "C:/Windows/Fonts/simfang.ttf",     # 仿宋
            "C:/Windows/Fonts/simsun.ttc",      # 宋体
            "C:/Windows/Fonts/msyh.ttc",        # 微软雅黑
            "C:/Windows/Fonts/msyhbd.ttc",      # 微软雅黑粗体
        ]

        for path in font_paths:
            if os.path.exists(path):
                return path

        # 如果都找不到，返回空字符串，使用默认字体
        return ""

    def save_color(self):
        """保存当前颜色到文件"""
        try:
            color_data = {
                'bgr': [self.b, self.g, self.r],
                'hex': f"#{self.b:02X}{self.g:02X}{self.r:02X}",
                'rgb': [self.r, self.g, self.b],  # RGB顺序
                'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            }

            # 添加到历史记录
            self.color_history.append(color_data)
            if len(self.color_history) > self.max_history:
                self.color_history = self.color_history[-self.max_history:]

            # 保存到文件
            filename = f"color_saves_{datetime.now().strftime('%Y%m%d')}.json"
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump({
                    'current_color': color_data,
                    'history': self.color_history
                }, f, indent=2, ensure_ascii=False)

            print(f"颜色已保存到文件: {filename}")
            print(f"当前颜色: BGR({self.b}, {self.g}, {self.r}) HEX: {color_data['hex']}")

        except Exception as e:
            print(f"保存颜色失败: {e}")

    def load_color_history(self):
        """加载颜色历史记录"""
        try:
            filename = f"color_saves_{datetime.now().strftime('%Y%m%d')}.json"
            if os.path.exists(filename):
                with open(filename, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    if 'history' in data:
                        self.color_history = data['history']
                        print(f"已加载 {len(self.color_history)} 个历史颜色")
        except Exception as e:
            print(f"加载历史记录失败: {e}")

    def copy_color_to_clipboard(self):
        """复制颜色值到剪贴板（模拟功能，实际需要pyperclip库）"""
        try:
            # 这里使用一个简单的文本文件来模拟剪贴板
            clipboard_file = "clipboard_color.txt"
            color_text = f"BGR: ({self.b}, {self.g}, {self.r})\nHEX: #{self.b:02X}{self.g:02X}{self.r:02X}\nRGB: ({self.r}, {self.g}, {self.b})"

            with open(clipboard_file, 'w', encoding='utf-8') as f:
                f.write(color_text)

            print("颜色值已复制到剪贴板文件: clipboard_color.txt")
            print(color_text)

        except Exception as e:
            print(f"复制到剪贴板失败: {e}")

    def on_blue_change(self, value):
        """蓝色滑块回调"""
        self.b = value
        self.update_preview()

    def on_green_change(self, value):
        """绿色滑块回调"""
        self.g = value
        self.update_preview()

    def on_red_change(self, value):
        """红色滑块回调"""
        self.r = value
        self.update_preview()

    def draw_chinese_text(self, img, text, position, font_size=24, color=(255, 255, 255)):
        """在OpenCV图像上绘制中文文本"""
        # 将OpenCV图像转换为PIL图像
        img_pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(img_pil)

        # 使用PIL绘制中文文本
        draw.text(position, text, font=self.font, fill=color)

        # 转换回OpenCV格式
        img_result = cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)
        return img_result

    def update_preview(self):
        """更新颜色预览"""
        # 创建预览图像
        preview = np.zeros((400, 600, 3), dtype=np.uint8)

        # 填充当前BGR颜色
        preview[:, :] = [self.b, self.g, self.r]

        # 添加颜色信息文本
        info_text = f"BGR: ({self.b}, {self.g}, {self.r})"
        hex_text = f"HEX: #{self.b:02X}{self.g:02X}{self.r:02X}"

        # 计算RGB值用于显示
        rgb_r = self.r
        rgb_g = self.g
        rgb_b = self.b
        rgb_text = f"RGB: ({rgb_r}, {rgb_g}, {rgb_b})"

        # 使用PIL绘制英文文本（英文使用OpenCV即可）
        cv2.putText(preview, info_text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(preview, hex_text, (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(preview, rgb_text, (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # 使用PIL绘制中文文本
        preview = self.draw_chinese_text(preview, "拖动滑块或按键调整颜色", (20, 200), color=(200, 200, 200))
        preview = self.draw_chinese_text(preview, "B/G/R: 增加对应颜色值", (20, 230), color=(200, 200, 200))
        preview = self.draw_chinese_text(preview, "b/g/r: 减少对应颜色值", (20, 260), color=(200, 200, 200))
        preview = self.draw_chinese_text(preview, "空格: 随机颜色", (20, 290), color=(200, 200, 200))
        preview = self.draw_chinese_text(preview, "S: 保存颜色", (20, 320), color=(200, 200, 200))
        preview = self.draw_chinese_text(preview, "C: 复制颜色值", (20, 350), color=(200, 200, 200))
        preview = self.draw_chinese_text(preview, "ESC: 退出", (20, 380), color=(200, 200, 200))

        # 显示预览
        cv2.imshow(self.preview_window_name, preview)

        # 更新滑块位置
        cv2.setTrackbarPos('Blue (B)', self.window_name, self.b)
        cv2.setTrackbarPos('Green (G)', self.window_name, self.g)
        cv2.setTrackbarPos('Red (R)', self.window_name, self.r)

    def adjust_color(self, key):
        """通过键盘调整颜色值"""
        step = 5  # 每次调整的步长

        if key == ord('B'):  # 增加蓝色
            self.b = min(255, self.b + step)
        elif key == ord('b'):  # 减少蓝色
            self.b = max(0, self.b - step)
        elif key == ord('G'):  # 增加绿色
            self.g = min(255, self.g + step)
        elif key == ord('g'):  # 减少绿色
            self.g = max(0, self.g - step)
        elif key == ord('R'):  # 增加红色
            self.r = min(255, self.r + step)
        elif key == ord('r'):  # 减少红色
            self.r = max(0, self.r - step)
        elif key == ord(' '):  # 空格键 - 随机颜色
            self.b = np.random.randint(0, 256)
            self.g = np.random.randint(0, 256)
            self.r = np.random.randint(0, 256)

        self.update_preview()

    def run(self):
        """运行调色器"""
        # 加载历史记录
        self.load_color_history()

        print("BGR Color Picker 启动")
        print("操作说明:")
        print("  - 拖动滑块调整颜色值")
        print("  - 按 B/G/R 键增加对应颜色")
        print("  - 按 b/g/r 键减少对应颜色")
        print("  - 按空格键生成随机颜色")
        print("  - 按 S 键保存当前颜色")
        print("  - 按 C 键复制颜色值")
        print("  - 按 ESC 键退出")
        print("  - 中文显示已启用，如有乱码请检查字体文件")

        # 初始化预览
        self.update_preview()

        while True:
            key = cv2.waitKey(10) & 0xFF

            if key == 27:  # ESC键退出
                break
            elif key in [ord('B'), ord('b'), ord('G'), ord('g'), ord('R'), ord('r'), ord(' ')]:
                self.adjust_color(key)
            elif key == ord('S') or key == ord('s'):  # 保存颜色
                self.save_color()
            elif key == ord('C') or key == ord('c'):  # 复制颜色
                self.copy_color_to_clipboard()

        cv2.destroyAllWindows()

def main():
    """主函数"""
    try:
        color_picker = BGRColorPicker()
        color_picker.run()
    except Exception as e:
        print(f"程序运行出错: {e}")
        cv2.destroyAllWindows()
        sys.exit(1)

if __name__ == "__main__":
    main()