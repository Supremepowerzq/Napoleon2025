#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BGR Color Picker 测试脚本
"""

import os
import sys
import json

def test_font_detection():
    """测试字体检测功能"""
    print("测试字体检测功能...")

    font_paths = [
        "C:/Windows/Fonts/simhei.ttf",      # 黑体
        "C:/Windows/Fonts/simfang.ttf",     # 仿宋
        "C:/Windows/Fonts/simsun.ttc",      # 宋体
        "C:/Windows/Fonts/msyh.ttc",        # 微软雅黑
        "C:/Windows/Fonts/msyhbd.ttc",      # 微软雅黑粗体
    ]

    found_fonts = []
    for path in font_paths:
        if os.path.exists(path):
            found_fonts.append(path)
            print(f"[OK] 找到字体: {path}")

    if not found_fonts:
        print("[ERROR] 未找到任何中文字体文件")
        return False
    else:
        print(f"[OK] 共找到 {len(found_fonts)} 个字体文件")
        return True

def test_imports():
    """测试必要的库是否已安装"""
    print("\n测试库导入...")

    try:
        import cv2
        print(f"[OK] OpenCV 版本: {cv2.__version__}")
    except ImportError:
        print("[ERROR] OpenCV 未安装")
        return False

    try:
        from PIL import ImageFont, ImageDraw, Image
        print("[OK] PIL/Pillow 已安装")
    except ImportError:
        print("[ERROR] PIL/Pillow 未安装")
        return False

    try:
        import numpy as np
        print(f"[OK] NumPy 版本: {np.__version__}")
    except ImportError:
        print("[ERROR] NumPy 未安装")
        return False

    return True

def test_color_save_load():
    """测试颜色保存和加载功能"""
    print("\n测试颜色保存/加载功能...")

    # 创建测试数据
    test_color = {
        'bgr': [255, 128, 64],
        'hex': '#FF8040',
        'rgb': [64, 128, 255],
        'timestamp': '2024-01-22 12:00:00'
    }

    test_data = {
        'current_color': test_color,
        'history': [test_color]
    }

    # 保存测试
    test_file = 'test_color_save.json'
    try:
        with open(test_file, 'w', encoding='utf-8') as f:
            json.dump(test_data, f, indent=2, ensure_ascii=False)
        print(f"[OK] 颜色保存测试通过: {test_file}")
    except Exception as e:
        print(f"[ERROR] 颜色保存测试失败: {e}")
        return False

    # 加载测试
    try:
        with open(test_file, 'r', encoding='utf-8') as f:
            loaded_data = json.load(f)

        if loaded_data['current_color']['bgr'] == test_color['bgr']:
            print("[OK] 颜色加载测试通过")
        else:
            print("[ERROR] 颜色加载测试失败: 数据不匹配")
            return False
    except Exception as e:
        print(f"[ERROR] 颜色加载测试失败: {e}")
        return False

    # 清理测试文件
    try:
        os.remove(test_file)
        print("[OK] 测试文件清理完成")
    except:
        pass

    return True

def main():
    """主测试函数"""
    print("=" * 50)
    print("BGR Color Picker - 功能测试")
    print("=" * 50)

    all_passed = True

    # 测试库导入
    if not test_imports():
        all_passed = False

    # 测试字体检测
    if not test_font_detection():
        all_passed = False

    # 测试颜色保存加载
    if not test_color_save_load():
        all_passed = False

    print("\n" + "=" * 50)
    if all_passed:
        print("[SUCCESS] 所有测试通过！BGR调色工具可以正常使用")
        print("\n运行方法: python color_picker_bgr.py")
    else:
        print("[FAILED] 部分测试失败，请检查依赖项")
        print("\n安装缺失的库:")
        print("pip install opencv-python pillow numpy")
    print("=" * 50)

if __name__ == "__main__":
    main()