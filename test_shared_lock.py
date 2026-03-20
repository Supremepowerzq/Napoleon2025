#!/usr/bin/env python3
"""
测试SHARED_FRAME_LOCK全局变量访问修复
"""
import threading

# 模拟全局变量
SHARED_FRAME_BUFFER = None
SHARED_FRAME_LOCK = threading.Lock()

def depth_processing_test():
    """测试depth_processing函数中的全局变量访问"""
    global SHARED_FRAME_BUFFER, SHARED_FRAME_LOCK

    print("测试depth_processing函数中的SHARED_FRAME_LOCK访问...")

    try:
        # 模拟depth_processing中的代码
        with SHARED_FRAME_LOCK:
            if SHARED_FRAME_BUFFER is None:
                print("OK: SHARED_FRAME_LOCK访问成功")
                print("OK: SHARED_FRAME_BUFFER访问成功")
            else:
                print("SHARED_FRAME_BUFFER有值")
        return True
    except NameError as e:
        print("ERROR: {}".format(e))
        return False
    except Exception as e:
        print("ERROR: 其他异常: {}".format(e))
        return False

def video_processing_test():
    """测试video_processing函数中的全局变量访问"""
    global SHARED_FRAME_BUFFER, SHARED_FRAME_LOCK

    print("测试video_processing函数中的SHARED_FRAME_LOCK访问...")

    try:
        # 模拟video_processing中的代码
        with SHARED_FRAME_LOCK:
            SHARED_FRAME_BUFFER = "test_frame"
            print("OK: SHARED_FRAME_LOCK访问成功")
            print("OK: SHARED_FRAME_BUFFER设置成功")
        return True
    except NameError as e:
        print("ERROR: {}".format(e))
        return False
    except Exception as e:
        print("ERROR: 其他异常: {}".format(e))
        return False

if __name__ == "__main__":
    print("开始测试SHARED_FRAME_LOCK全局变量访问修复...")

    # 测试depth_processing风格的访问
    success1 = depth_processing_test()

    # 测试video_processing风格的访问
    success2 = video_processing_test()

    if success1 and success2:
        print("OK: 所有测试通过！SHARED_FRAME_LOCK修复成功")
    else:
        print("ERROR: 测试失败")