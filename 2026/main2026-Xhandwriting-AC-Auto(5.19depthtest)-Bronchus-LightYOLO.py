"""Napoleon 轻量支气管 YOLO + 电机反馈联动启动入口。

原主程序文件保持原有视觉入口不变。本文件运行时加载原主程序的机器人、
手柄、串口、限位和 UI 逻辑，仅在内存中替换视频线程入口，不修改原模块。
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType


HERE = Path(__file__).resolve().parent
ORIGINAL_MAIN = HERE / "main2026-Xhandwriting-AC-Auto(5.19depthtest)-Bronchus.py"


def _load_original_main() -> ModuleType:
    module_name = "_napoleon_2026_bronchus_original"
    spec = importlib.util.spec_from_file_location(module_name, ORIGINAL_MAIN)
    if spec is None or spec.loader is None:
        raise ImportError(f"无法加载原主程序: {ORIGINAL_MAIN}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def _light_video_processing() -> None:
    try:
        from predict_2026_528_zck_motor_yolo import UnetPackage
    except Exception as exc:
        print(f"[LightYOLO] 轻量视频模块导入失败，跳过视觉处理: {exc}")
        return

    detector = UnetPackage(
        mode="video",
        video_path=0,
        video_save_path=r"G:\zck\light_yolo_out.avi",
        video_fps=30,
    )
    detector.video()


def main() -> None:
    original = _load_original_main()
    original_controller = original.XboxController

    def light_controller_factory():
        try:
            from Bronchialtree_identification.motor_linked_detection.xinput_controller import (
                XInputController,
            )

            controller = XInputController()
            print("[LightYOLO] 手柄使用 Windows XInput 轮询（轻量版专用）")
            return controller
        except Exception as exc:
            print(f"[LightYOLO] XInput 启动失败，回退原手柄接口: {exc}")
            return original_controller()

    original.XboxController = light_controller_factory
    original.video_processing = _light_video_processing
    print("[LightYOLO] 已启用独立轻量视觉/手柄入口，原主程序文件未被替换")
    original.main()


if __name__ == "__main__":
    main()
