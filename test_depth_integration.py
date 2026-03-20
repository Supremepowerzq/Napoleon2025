#!/usr/bin/env python3
"""
测试深度推理功能集成
"""

import sys
import os

# 添加项目根目录到路径
parent_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_path)

def test_depth_integration():
    """测试深度推理功能集成"""
    try:
        # 读取文件内容，检查深度推理相关的代码是否存在
        with open("2026/main2026-X-anglecontrol(1.8).py", 'r', encoding='utf-8') as f:
            content = f.read()

        # 检查关键组件是否存在
        checks = [
            ("DEPTH_SWITCH", "DEPTH_SWITCH = True" in content),
            ("DEPTH_ENCODER", "DEPTH_ENCODER = 'vits'" in content),
            ("DEPTH_INPUT_SIZE", "DEPTH_INPUT_SIZE = 518" in content),
            ("DEPTH_GRAYSCALE", "DEPTH_GRAYSCALE = False" in content),
            ("DEPTH_CIRCLE_CENTER_X", "DEPTH_CIRCLE_CENTER_X = 400" in content),
            ("DEPTH_CIRCLE_CENTER_Y", "DEPTH_CIRCLE_CENTER_Y = 230" in content),
            ("DEPTH_CIRCLE_RADIUS", "DEPTH_CIRCLE_RADIUS = 115" in content),
            ("DEPTH_TARGET_RATIO", "DEPTH_TARGET_RATIO = 16/9" in content),
            ("extract_circular_roi 函数", "def extract_circular_roi(" in content),
            ("resize_with_padding 函数", "def resize_with_padding(" in content),
            ("depth_processing 函数", "def depth_processing() -> None:" in content),
            ("深度推理线程启动", "depth_thread = None" in content and "depth_processing" in content),
        ]

        all_passed = True
        for check_name, passed in checks:
            if passed:
                print(f"[PASS] {check_name}")
            else:
                print(f"[FAIL] {check_name}")
                all_passed = False

        # 检查导入部分
        if "from depth_anything_v2.dpt import DepthAnythingV2" in content:
            print("[PASS] DepthAnythingV2 导入")
        else:
            print("[FAIL] DepthAnythingV2 导入")
            all_passed = False

        if all_passed:
            print("\n[SUCCESS] 深度推理功能集成测试通过!")
            print("代码中包含了所有必要的深度推理组件:")
            print("- 深度推理配置参数")
            print("- 圆形ROI提取函数")
            print("- 图像预处理函数")
            print("- 深度推理处理线程")
            print("- 线程启动逻辑")
            return True
        else:
            print("\n[FAILED] 深度推理功能集成测试失败!")
            return False

    except Exception as e:
        print(f"[ERROR] 测试过程中出现异常: {e}")
        return False

if __name__ == "__main__":
    success = test_depth_integration()
    sys.exit(0 if success else 1)
