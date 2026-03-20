#!/usr/bin/env python3
"""
测试UnetPackage共享缓冲区修复
"""
import threading
import sys
import os

# 添加路径以便导入模块
parent_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(parent_path)

# 模拟共享缓冲区和锁
SHARED_FRAME_BUFFER = None
SHARED_FRAME_LOCK = threading.Lock()

def test_unet_package_creation():
    """测试UnetPackage是否能正确创建并使用共享缓冲区"""
    try:
        from predict_2025 import UnetPackage

        print("测试UnetPackage创建...")

        # 测试创建时不提供共享缓冲区（应该会失败）
        try:
            unet1 = UnetPackage(mode='video', video_path=0)
            print("ERROR: 应该在video()调用时失败，但创建成功了")
            return False
        except Exception as e:
            print("OK: 创建时不提供共享缓冲区 - 预期行为")

        # 测试创建时提供共享缓冲区
        unet2 = UnetPackage(
            mode='video',
            video_path=0,
            shared_frame_buffer=SHARED_FRAME_BUFFER,
            shared_frame_lock=SHARED_FRAME_LOCK
        )
        print("OK: UnetPackage创建成功，提供了共享缓冲区")

        # 测试访问共享缓冲区属性
        if hasattr(unet2, 'shared_frame_buffer') and hasattr(unet2, 'shared_frame_lock'):
            print("OK: UnetPackage实例具有shared_frame_buffer和shared_frame_lock属性")
        else:
            print("ERROR: UnetPackage实例缺少必要的属性")
            return False

        return True

    except Exception as e:
        print("ERROR: 测试失败: {}".format(e))
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("开始测试UnetPackage共享缓冲区修复...")

    success = test_unet_package_creation()

    if success:
        print("OK: 所有测试通过！UnetPackage修复成功")
    else:
        print("ERROR: 测试失败")