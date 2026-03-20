#!/usr/bin/env python3
"""
测试深度推理相关库是否可用
"""

libraries = [
    ("cv2", "opencv-python"),
    ("torch", "torch"),
    ("numpy", "numpy"),
    ("matplotlib", "matplotlib"),
    ("PIL", "Pillow"),
]

depth_available = True

for lib_name, package_name in libraries:
    try:
        __import__(lib_name)
        print(f"{lib_name}: OK")
    except ImportError as e:
        print(f"{lib_name}: FAILED - {e}")
        print(f"  Please install: pip install {package_name}")
        depth_available = False

# 测试DepthAnythingV2
try:
    from depth_anything_v2.dpt import DepthAnythingV2
    print("DepthAnythingV2: OK")
except ImportError as e:
    print(f"DepthAnythingV2: FAILED - {e}")
    print("  Please check if depth_anything_v2 is properly installed")
    depth_available = False

print(f"\nDepth libraries available: {depth_available}")

if depth_available:
    print("All depth libraries are available!")
else:
    print("Some depth libraries are missing. Depth processing will be disabled.")
