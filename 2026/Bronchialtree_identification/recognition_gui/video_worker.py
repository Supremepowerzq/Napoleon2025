"""Camera/YOLO worker that publishes clean frames to the integrated PyQt UI."""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Optional

import cv2

from .buses import CONTROL_BUS, FRAME_BUS, GUI_MESSAGE_BUS, VIDEO_STATUS_BUS


class IntegratedVideoWorker(threading.Thread):
    """Reuse the deployed detector while replacing only its display loop."""

    def __init__(self) -> None:
        super().__init__(name="integrated_bronchus_video", daemon=True)
        self._writer: Optional[cv2.VideoWriter] = None
        self._writer_path: Optional[Path] = None
        self._record_started_at = 0.0

    def run(self) -> None:
        capture: Optional[cv2.VideoCapture] = None
        detector = None
        try:
            VIDEO_STATUS_BUS.update(message="正在加载目标检测模型")
            from predict_2026_528_zck_motor_yolo import UnetPackage

            detector = UnetPackage(
                mode="video",
                video_path=0,
                video_save_path="",
                video_fps=30,
                show_window=False,
                display_all_boxes=False,
            )
            VIDEO_STATUS_BUS.update(model_loaded=True, message="模型已就绪，正在连接内窥镜")

            capture = cv2.VideoCapture(0, cv2.CAP_DSHOW)
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not capture.isOpened():
                raise RuntimeError("无法打开内窥镜摄像头")

            VIDEO_STATUS_BUS.update(camera_connected=True, message="内窥镜已连接")
            GUI_MESSAGE_BUS.publish("内窥镜画面已接入")
            previous = time.perf_counter()
            fps = 0.0
            was_detection_enabled = False
            consecutive_read_failures = 0

            while not CONTROL_BUS.shutdown_requested():
                ok, full_frame = capture.read()
                if not ok:
                    consecutive_read_failures += 1
                    if consecutive_read_failures == 1:
                        VIDEO_STATUS_BUS.update(message="内窥镜画面读取失败，正在重试")
                    time.sleep(0.02)
                    continue
                consecutive_read_failures = 0

                frame = detector._crop_and_undistort(full_frame)  # noqa: SLF001
                if frame is None:
                    VIDEO_STATUS_BUS.update(message="摄像头分辨率不足以裁剪标定 ROI")
                    time.sleep(0.02)
                    continue

                detector._frame_index += 1  # noqa: SLF001
                detection_enabled = CONTROL_BUS.detection_enabled()
                if detection_enabled:
                    if detector._frame_index % detector.inference_interval == 0:  # noqa: SLF001
                        try:
                            detector._detect(frame)  # noqa: SLF001
                        except Exception as exc:
                            GUI_MESSAGE_BUS.publish(f"目标检测失败：{exc}", "error")
                    was_detection_enabled = True
                elif was_detection_enabled:
                    detector._state_coordinator.reset()  # noqa: SLF001
                    detector._latest_boxes = []  # noqa: SLF001
                    detector._last_state = None  # noqa: SLF001
                    was_detection_enabled = False

                now = time.perf_counter()
                instant = 1.0 / max(now - previous, 1e-6)
                previous = now
                fps = instant if fps <= 0.0 else fps * 0.90 + instant * 0.10

                # The user requested a clean endoscopic image. Detection boxes,
                # confidence values and anatomy text are therefore not painted
                # on the video; the left map is the sole anatomy visualization.
                FRAME_BUS.publish(frame)
                self._update_recording(frame)
                VIDEO_STATUS_BUS.update(
                    detection_enabled=detection_enabled,
                    fps=fps,
                    message="目标检测运行中" if detection_enabled else "仅显示内窥镜画面",
                )
        except Exception as exc:
            message = f"视频模块启动失败：{type(exc).__name__}: {exc}"
            VIDEO_STATUS_BUS.update(message=message)
            GUI_MESSAGE_BUS.publish(message, "error")
        finally:
            if detector is not None:
                try:
                    detector._state_coordinator.reset()  # noqa: SLF001
                except Exception:
                    pass
            if capture is not None:
                capture.release()
            self._close_writer("录像已保存")
            VIDEO_STATUS_BUS.update(
                camera_connected=False,
                recording=False,
                recording_path=None,
                message="视频线程已停止",
            )

    def _update_recording(self, frame) -> None:
        requested_path = CONTROL_BUS.recording_path()
        if requested_path is None:
            self._close_writer("录像已保存")
            return

        if self._writer is not None and requested_path != self._writer_path:
            self._close_writer("上一段录像已保存")

        if self._writer is None:
            requested_path.parent.mkdir(parents=True, exist_ok=True)
            fourcc = cv2.VideoWriter_fourcc(*"XVID")
            height, width = frame.shape[:2]
            writer = cv2.VideoWriter(
                str(requested_path),
                fourcc,
                30.0,
                (width, height),
            )
            if not writer.isOpened():
                writer.release()
                CONTROL_BUS.stop_recording()
                GUI_MESSAGE_BUS.publish(f"无法创建录像：{requested_path}", "error")
                return
            self._writer = writer
            self._writer_path = requested_path
            self._record_started_at = time.monotonic()
            VIDEO_STATUS_BUS.update(
                recording=True,
                recording_path=str(requested_path),
                recording_started_at=self._record_started_at,
            )
            GUI_MESSAGE_BUS.publish(f"开始录像：{requested_path.name}")

        self._writer.write(frame)

    def _close_writer(self, message: str) -> None:
        if self._writer is None:
            return
        saved_path = self._writer_path
        self._writer.release()
        self._writer = None
        self._writer_path = None
        self._record_started_at = 0.0
        VIDEO_STATUS_BUS.update(
            recording=False,
            recording_path=None,
            recording_started_at=0.0,
        )
        if saved_path is not None:
            GUI_MESSAGE_BUS.publish(f"{message}：{saved_path.name}")
