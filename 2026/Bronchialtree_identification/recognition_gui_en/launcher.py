"""Launch the integrated PyQt recognition console."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


RUNTIME_ROOT = Path(__file__).resolve().parents[2]
PROJECT_ROOT = RUNTIME_ROOT.parent
for entry in (str(RUNTIME_ROOT), str(PROJECT_ROOT)):
    if entry not in sys.path:
        sys.path.insert(0, entry)


def preflight() -> list[str]:
    from Bronchialtree_identification.motor_linked_detection.config import (
        DEFAULT_MODEL_PATH,
        DEFAULT_UNDISTORT_MAP_PATH,
    )
    from .main_window import ASSET_PATH
    from .robot_runtime import ROBOT_MAIN

    predictor = RUNTIME_ROOT / "predict_2026_528_zck_motor_yolo.py"
    missing = [
        str(path)
        for path in (
            ROBOT_MAIN,
            predictor,
            DEFAULT_MODEL_PATH,
            DEFAULT_UNDISTORT_MAP_PATH,
            ASSET_PATH,
        )
        if not path.is_file()
    ]
    return missing


def _run_ui(smoke_test: bool = False) -> int:
    if smoke_test:
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

    from PyQt5.QtCore import QCoreApplication, Qt, QTimer
    from PyQt5.QtWidgets import QApplication

    QCoreApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QCoreApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    from .buses import CONTROL_BUS
    from .main_window import RecognitionMainWindow
    from .robot_runtime import RobotRuntime
    from .video_worker import IntegratedVideoWorker

    os.chdir(RUNTIME_ROOT)
    app = QApplication.instance() or QApplication(sys.argv)
    app.setApplicationName("Bronchoscopy Position Recognition and Navigation")
    app.setOrganizationName("Bronchial Navigation Lab")

    robot = RobotRuntime()
    window = RecognitionMainWindow(robot)
    window.show()

    video = None
    if smoke_test:
        QTimer.singleShot(700, app.quit)
    else:
        robot.start()
        video = IntegratedVideoWorker()
        video.start()

    exit_code = app.exec_()
    CONTROL_BUS.stop_recording()
    CONTROL_BUS.request_shutdown()
    robot.request_shutdown()
    if video is not None:
        video.join(timeout=5.0)
    if robot.is_alive():
        robot.join(timeout=8.0)
    return int(exit_code)


def main() -> None:
    parser = argparse.ArgumentParser(description="Bronchoscopy recognition PyQt console")
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="Check files and Python dependencies without connecting hardware.",
    )
    parser.add_argument(
        "--ui-smoke-test",
        action="store_true",
        help="Create the main window offscreen without connecting hardware.",
    )
    args = parser.parse_args()

    missing = preflight()
    if missing:
        details = "\n".join(f"  - {path}" for path in missing)
        raise FileNotFoundError(f"PyQt interface resources are incomplete:\n{details}")

    print("[RecognitionGUI-EN] Resource check passed")
    print(f"[RecognitionGUI-EN] Working directory: {RUNTIME_ROOT}")
    if args.check_only:
        return
    raise SystemExit(_run_ui(smoke_test=args.ui_smoke_test))


if __name__ == "__main__":
    main()
