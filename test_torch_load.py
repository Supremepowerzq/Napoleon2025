#!/usr/bin/env python3
"""
测试torch.load的weights_only参数修复
"""
import torch
import os
import warnings

def test_torch_load_fix():
    """测试torch.load的weights_only参数"""
    print("开始测试torch.load修复...")

    # 检查权重文件是否存在
    weight_path = '2026/Depth-Anything-V2-main/checkpoints/depth_anything_v2_vits.pth'
    if not os.path.exists(weight_path):
        print("ERROR: 权重文件不存在: {}".format(weight_path))
        return

    print("OK: 找到权重文件: {}".format(weight_path))

    try:
        # 捕获警告
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")

            # 新版本（修复后的版本）
            print("测试修复后的torch.load (weights_only=True)...")
            state_dict = torch.load(weight_path, weights_only=True)

            # 检查是否有警告
            future_warnings = [warning for warning in w if issubclass(warning.category, FutureWarning) and 'torch.load' in str(warning.message)]
            if future_warnings:
                print("ERROR: 仍然有FutureWarning警告:")
                for warning in future_warnings:
                    print("  {}".format(warning.message))
            else:
                print("OK: 没有torch.load相关的FutureWarning！")

        print("OK: torch.load修复成功！")
        print("加载的权重数量: {}".format(len(state_dict)))
        print("权重类型: {}".format(type(state_dict)))

        # 检查state_dict内容
        if isinstance(state_dict, dict):
            sample_keys = list(state_dict.keys())[:5]  # 只显示前5个键
            print("示例权重键: {}".format(sample_keys))
        else:
            print("加载结果类型: {}".format(type(state_dict)))

    except Exception as e:
        print("ERROR: 测试失败: {}".format(e))
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_torch_load_fix()