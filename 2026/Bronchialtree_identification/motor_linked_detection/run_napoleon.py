"""检查资源并启动部位识别专用 PyQt 控制台。"""

from __future__ import annotations

import argparse
import os
import runpy
import sys
from pathlib import Path

from .config import DEFAULT_MODEL_PATH, DEFAULT_UNDISTORT_MAP_PATH


RUNTIME_ROOT = Path(__file__).resolve().parents[2]
MAIN_SCRIPT = RUNTIME_ROOT / "main2026-Xhandwriting-AC-AutoNav(0710)-Bronchus.py"
LIGHTWEIGHT_PREDICTOR = RUNTIME_ROOT / "predict_2026_528_zck_motor_yolo.py"
PYQT_MAIN = RUNTIME_ROOT / "Bronchialtree_identification" / "run_recognition_gui.py"
PYQT_ASSET = (
    RUNTIME_ROOT
    / "Bronchialtree_identification"
    / "recognition_gui"
    / "assets"
    / "bronchi.png"
)


def preflight() -> list[str]:
    missing = []
    for path in (
        MAIN_SCRIPT,
        PYQT_MAIN,
        PYQT_ASSET,
        LIGHTWEIGHT_PREDICTOR,
        DEFAULT_MODEL_PATH,
        DEFAULT_UNDISTORT_MAP_PATH,
    ):
        if not path.is_file():
            missing.append(str(path))
    return missing


def main() -> None:
    parser = argparse.ArgumentParser(
        description="检查部署资源，并启动支气管部位识别 PyQt 控制台。"
    )
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="只检查脚本和模型资源，不连接摄像头、串口或电机。",
    )
    args = parser.parse_args()

    missing = preflight()
    if missing:
        details = "\n".join(f"  - {path}" for path in missing)
        raise FileNotFoundError(f"部署文件不完整：\n{details}")

    print("[BronchState] 部署文件检查通过")
    print(f"[BronchState] 机器人/AutoNav控制源（未修改）: {MAIN_SCRIPT}")
    print(f"[BronchState] PyQt识别界面: {PYQT_MAIN}")
    print(f"[BronchState] 轻量检测: {LIGHTWEIGHT_PREDICTOR}")
    print(f"[BronchState] YOLO模型: {DEFAULT_MODEL_PATH}")
    print(f"[BronchState] 去畸变映射: {DEFAULT_UNDISTORT_MAP_PATH}")
    if args.check_only:
        return

    os.chdir(RUNTIME_ROOT)
    runtime_root_text = str(RUNTIME_ROOT)
    if runtime_root_text not in sys.path:
        sys.path.insert(0, runtime_root_text)
    runpy.run_path(str(PYQT_MAIN), run_name="__main__")


if __name__ == "__main__":
    main()
