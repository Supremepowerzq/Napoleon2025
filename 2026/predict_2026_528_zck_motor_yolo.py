"""轻量支气管 YOLO + Napoleon 电机反馈联动。

本模块只负责：摄像头、ROI、去畸变、YOLO 目标检测和部位状态机。
它不导入或运行深度预测、UNet、腔道分析、3D、BC 或 AutoNav。
电机信息来自 Napoleon 主程序发布的只读缓存，本模块不访问串口。
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np
import torch
from ultralytics import YOLO

from Bronchialtree_identification.motor_linked_detection import (
    DEFAULT_MODEL_PATH,
    DEFAULT_UNDISTORT_MAP_PATH,
    BronchusDetectionCoordinator,
    to_legacy_label,
)
from Bronchialtree_identification.motor_linked_detection.config import normalize_label


WINDOW_NAME = "Bronchus YOLO + Motor State"


class UnetPackage:
    """与旧视频模块保持类名兼容的轻量检测器。"""

    def __init__(
        self,
        model_path: Optional[str] = None,
        mode: str = "video",
        video_path: Any = 0,
        video_save_path: str = "",
        video_fps: int = 30,
        roi_x: int = 480,
        roi_y: int = 240,
        roi_side: int = 582,
        undistort_maps_path: Optional[str] = None,
        inference_interval: int = 3,
        yolo_confidence: float = 0.25,
        show_window: bool = True,
        display_all_boxes: bool = False,
        **_ignored: Any,
    ) -> None:
        self.mode = mode
        self.video_path = video_path
        self.video_save_path = video_save_path
        self.video_fps = video_fps
        self.roi_x = roi_x
        self.roi_y = roi_y
        self.roi_side = roi_side
        self.inference_interval = max(1, int(inference_interval))
        self.yolo_confidence = float(yolo_confidence)
        self.show_window = bool(show_window)
        self.display_all_boxes = bool(display_all_boxes)

        self.model_path = Path(model_path) if model_path else DEFAULT_MODEL_PATH
        self.undistort_maps_path = (
            Path(undistort_maps_path)
            if undistort_maps_path
            else DEFAULT_UNDISTORT_MAP_PATH
        )
        self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
        self.map1: Optional[np.ndarray] = None
        self.map2: Optional[np.ndarray] = None
        self._frame_index = 0
        self._latest_boxes: list[tuple[str, float, tuple[int, int, int, int]]] = []
        self._last_state = None
        self._state_coordinator = BronchusDetectionCoordinator()

        self._load_undistort_maps()
        self._model = self._load_model()

    def _load_undistort_maps(self) -> None:
        if not self.undistort_maps_path.is_file():
            print(
                f"[LightYOLO] 去畸变映射不存在，使用原始 ROI: "
                f"{self.undistort_maps_path}"
            )
            return
        try:
            with np.load(str(self.undistort_maps_path)) as maps:
                self.map1 = maps["map1"].copy()
                self.map2 = maps["map2"].copy()
            print(f"[LightYOLO] 去畸变映射已加载: {self.undistort_maps_path}")
        except Exception as exc:
            self.map1 = None
            self.map2 = None
            print(f"[LightYOLO] 去畸变映射加载失败，使用原始 ROI: {exc}")

    def _load_model(self) -> YOLO:
        if not self.model_path.is_file():
            raise FileNotFoundError(f"YOLO 模型不存在: {self.model_path}")
        model = YOLO(str(self.model_path), task="detect")
        model.to(self.device)
        warmup = np.zeros((self.roi_side, self.roi_side, 3), dtype=np.uint8)
        model.predict(warmup, verbose=False, device=self.device)
        if self.device.startswith("cuda"):
            torch.cuda.synchronize()
        print(f"[LightYOLO] 模型已加载 ({self.device}): {self.model_path}")
        print(f"[LightYOLO] 类别: {model.names}")
        return model

    def _crop_and_undistort(self, frame: np.ndarray) -> Optional[np.ndarray]:
        height, width = frame.shape[:2]
        if (
            self.roi_x < 0
            or self.roi_y < 0
            or self.roi_x + self.roi_side > width
            or self.roi_y + self.roi_side > height
        ):
            return None
        roi = frame[
            self.roi_y : self.roi_y + self.roi_side,
            self.roi_x : self.roi_x + self.roi_side,
        ]
        if self.map1 is None or self.map2 is None:
            return roi.copy()
        return cv2.remap(
            roi,
            self.map1,
            self.map2,
            cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )

    @staticmethod
    def _class_name(names: Any, class_id: int) -> str:
        if isinstance(names, dict):
            return str(names.get(class_id, class_id))
        return str(names[class_id])

    def _detect(self, frame: np.ndarray) -> None:
        results = self._model.predict(
            frame,
            verbose=False,
            conf=self.yolo_confidence,
            device=self.device,
        )
        raw_detections: list[tuple[str, float]] = []
        display_boxes: list[tuple[str, float, tuple[int, int, int, int]]] = []
        boxes = results[0].boxes
        if boxes is not None:
            for box in boxes:
                class_id = int(box.cls.item())
                label = self._class_name(self._model.names, class_id)
                confidence = float(box.conf.item())
                x1, y1, x2, y2 = (
                    int(value) for value in box.xyxy[0].detach().cpu().tolist()
                )
                raw_detections.append((label, confidence))
                display_boxes.append((label, confidence, (x1, y1, x2, y2)))

        self._latest_boxes = display_boxes
        self._last_state = self._state_coordinator.update(raw_detections)

    def _draw_overlay(self, frame: np.ndarray, fps: float) -> np.ndarray:
        view = frame.copy()
        state = self._last_state
        confirmed_canonical = "TR" if state is None else state.confirmed_position
        confirmed = to_legacy_label(confirmed_canonical)

        if self.display_all_boxes:
            boxes_to_draw = self._latest_boxes
        else:
            confirmed_boxes = [
                box
                for box in self._latest_boxes
                if normalize_label(box[0]) == confirmed_canonical
            ]
            boxes_to_draw = (
                [max(confirmed_boxes, key=lambda item: item[1])]
                if confirmed_boxes
                else []
            )
        for label, confidence, (x1, y1, x2, y2) in boxes_to_draw:
            box_color = (0, 220, 255) if not self.display_all_boxes else (130, 130, 130)
            box_prefix = "CONFIRMED" if not self.display_all_boxes else "RAW"
            cv2.rectangle(view, (x1, y1), (x2, y2), box_color, 2)
            cv2.putText(
                view,
                f"{box_prefix} {label} {confidence:.2f}",
                (x1, max(18, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                box_color,
                1,
                cv2.LINE_AA,
            )

        motion = "idle" if state is None else state.motion_mode
        motion_code = {"forward": "F", "backward": "B", "idle": "S"}.get(
            motion, "?"
        )
        candidate = "-"
        gate = "INACTIVE"
        reason = "waiting_motor_and_detection"
        if state is not None:
            if state.candidate_position:
                candidate = to_legacy_label(state.candidate_position)
                if state.required_count:
                    candidate += f" {state.candidate_count}/{state.required_count}"
            gate = state.gate.phase
            reason = state.reason

        panel_height = 112
        cv2.rectangle(view, (0, 0), (view.shape[1], panel_height), (0, 0, 0), -1)
        lines = (
            f"Confirmed: {confirmed}    FPS: {fps:.1f}    Raw boxes: {len(self._latest_boxes)}",
            f"FB State: {motion_code} ({motion})    Candidate: {candidate}",
            f"RMB gate: {gate}    State: {reason[:54]}",
            "ESC: exit video    R: reset anatomy state",
        )
        colors = ((0, 220, 255), (255, 210, 80), (210, 210, 210), (160, 160, 160))
        for index, (text, color) in enumerate(zip(lines, colors)):
            cv2.putText(
                view,
                text,
                (8, 24 + index * 26),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.54,
                color,
                1,
                cv2.LINE_AA,
            )

        badge_colors = {
            "F": (0, 180, 0),
            "B": (0, 120, 255),
            "S": (90, 90, 90),
            "?": (0, 0, 180),
        }
        badge_left = max(0, view.shape[1] - 72)
        cv2.rectangle(
            view,
            (badge_left, 5),
            (view.shape[1] - 6, 48),
            badge_colors[motion_code],
            -1,
        )
        cv2.putText(
            view,
            motion_code,
            (badge_left + 20, 38),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.15,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return view

    def video(self) -> None:
        """持续运行轻量目标检测；ESC 只结束视频线程。"""
        capture = cv2.VideoCapture(self.video_path, cv2.CAP_DSHOW)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not capture.isOpened():
            raise RuntimeError(f"无法打开摄像头/视频源: {self.video_path}")

        writer = None
        fps = 0.0
        previous_time = time.perf_counter()
        print("[LightYOLO] 仅运行 ROI + 去畸变 + YOLO + 电机状态机")
        try:
            while True:
                ok, frame_full = capture.read()
                if not ok:
                    print("[LightYOLO] 读取画面失败，继续重试")
                    continue
                frame = self._crop_and_undistort(frame_full)
                if frame is None:
                    print(
                        f"[LightYOLO] 输入尺寸 {frame_full.shape[1]}x{frame_full.shape[0]} "
                        f"不足以裁剪 ROI，跳过当前帧"
                    )
                    continue

                self._frame_index += 1
                if self._frame_index % self.inference_interval == 0:
                    try:
                        self._detect(frame)
                    except Exception as exc:
                        print(f"[LightYOLO] YOLO 推理失败: {type(exc).__name__}: {exc}")

                now = time.perf_counter()
                instant_fps = 1.0 / max(now - previous_time, 1e-6)
                previous_time = now
                fps = instant_fps if fps <= 0.0 else fps * 0.90 + instant_fps * 0.10
                view = self._draw_overlay(frame, fps)

                if self.video_save_path:
                    if writer is None:
                        output_path = Path(self.video_save_path)
                        output_path.parent.mkdir(parents=True, exist_ok=True)
                        fourcc = cv2.VideoWriter_fourcc(*"XVID")
                        writer = cv2.VideoWriter(
                            str(output_path),
                            fourcc,
                            self.video_fps,
                            (view.shape[1], view.shape[0]),
                        )
                        if not writer.isOpened():
                            writer.release()
                            writer = None
                            raise RuntimeError(
                                f"无法创建录像文件，请检查路径或编码器: {output_path}"
                            )
                        print(f"[LightYOLO] 开始保存录像: {output_path}")
                    writer.write(view)

                if self.show_window:
                    cv2.imshow(WINDOW_NAME, view)
                    key = cv2.waitKey(1) & 0xFF
                    if key == 27:
                        break
                    if key in (ord("r"), ord("R")):
                        self._state_coordinator.reset()
                        self._latest_boxes = []
                        self._last_state = None
        finally:
            capture.release()
            if writer is not None:
                writer.release()
                print(f"[LightYOLO] 录像已保存: {self.video_save_path}")
            if self.show_window:
                try:
                    cv2.destroyWindow(WINDOW_NAME)
                except cv2.error:
                    pass


def main() -> None:
    print(
        "[LightYOLO] 独立运行时没有 Napoleon 电机发布器，状态机将保持 idle；"
        "完整联动请启动 Napoleon 主脚本。"
    )
    UnetPackage().video()


if __name__ == "__main__":
    main()
