"""Compact PyQt console for bronchial position recognition and robot status."""

from __future__ import annotations

import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import cv2
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QCloseEvent, QImage, QPixmap
from PyQt5.QtWidgets import (
    QAction,
    QCheckBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QSplitter,
    QStatusBar,
    QVBoxLayout,
    QWidget,
)

from Bronchialtree_identification.motor_linked_detection import (
    get_latest_detection,
    get_motor_snapshot,
)

from .bronchus_map import BronchusMapWidget, DISPLAY_NAMES
from .buses import CONTROL_BUS, FRAME_BUS, GUI_MESSAGE_BUS, VIDEO_STATUS_BUS
from .display_gate import InitialDisplayGate
from .robot_runtime import RobotRuntime


HERE = Path(__file__).resolve().parent
ASSET_PATH = HERE / "assets" / "bronchi.png"
RECORDING_ROOT = HERE.parent / "recordings"


class RecognitionMainWindow(QMainWindow):
    def __init__(self, robot_runtime: RobotRuntime) -> None:
        super().__init__()
        self.robot_runtime = robot_runtime
        self._last_frame_sequence = -1
        self._last_message_sequence = -1
        self._last_pixmap: Optional[QPixmap] = None
        self._closing = False
        self._display_gate = InitialDisplayGate(
            required_updates=4,
            confidence_threshold=0.60,
        )

        self.setWindowTitle("支气管内窥镜部位识别与导航系统")
        self.setMinimumSize(1280, 760)
        self.resize(1800, 960)
        self._build_ui()
        self._build_menus()
        self._apply_style()

        self._timer = QTimer(self)
        self._timer.setInterval(33)
        self._timer.timeout.connect(self._refresh)
        self._timer.start()

    def _build_ui(self) -> None:
        central = QWidget(self)
        root = QVBoxLayout(central)
        root.setContentsMargins(18, 14, 18, 14)
        root.setSpacing(12)

        header = QHBoxLayout()
        title_box = QVBoxLayout()
        title = QLabel("支气管内窥镜部位识别与导航系统")
        title.setObjectName("title")
        subtitle = QLabel("BRONCHIAL ENDOSCOPIC RECOGNITION & NAVIGATION")
        subtitle.setObjectName("subtitle")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        header.addLayout(title_box)
        header.addStretch(1)

        self.position_badge = self._badge("当前位置：气管 [TR]", "activeBadge")
        self.motion_badge = self._badge("运动：S", "neutralBadge")
        self.controller_badge = self._badge("手柄：等待", "neutralBadge")
        self.camera_badge = self._badge("内窥镜：等待", "neutralBadge")
        for badge in (
            self.position_badge,
            self.motion_badge,
            self.controller_badge,
            self.camera_badge,
        ):
            header.addWidget(badge)
        root.addLayout(header)

        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)

        map_card = QFrame()
        map_card.setObjectName("card")
        map_layout = QVBoxLayout(map_card)
        map_layout.setContentsMargins(4, 4, 4, 4)
        self.map_widget = BronchusMapWidget(ASSET_PATH)
        map_layout.addWidget(self.map_widget)

        video_card = QFrame()
        video_card.setObjectName("card")
        video_layout = QVBoxLayout(video_card)
        video_layout.setContentsMargins(14, 12, 14, 14)
        video_header = QHBoxLayout()
        video_title = QLabel("内窥镜实时画面")
        video_title.setObjectName("panelTitle")
        self.video_detail = QLabel("等待视频线程")
        self.video_detail.setObjectName("panelDetail")
        video_header.addWidget(video_title)
        video_header.addStretch(1)
        video_header.addWidget(self.video_detail)
        video_layout.addLayout(video_header)

        self.video_label = QLabel("正在加载内窥镜画面…")
        self.video_label.setObjectName("video")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(640, 560)
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        video_layout.addWidget(self.video_label, 1)

        splitter.addWidget(map_card)
        splitter.addWidget(video_card)
        splitter.setStretchFactor(0, 42)
        splitter.setStretchFactor(1, 58)
        splitter.setSizes([720, 1000])
        root.addWidget(splitter, 1)

        controls = QFrame()
        controls.setObjectName("controls")
        control_layout = QHBoxLayout(controls)
        control_layout.setContentsMargins(16, 10, 16, 10)
        control_layout.setSpacing(10)

        self.detection_switch = QCheckBox("目标检测")
        self.detection_switch.setObjectName("detectionSwitch")
        self.detection_switch.setChecked(False)
        self.detection_switch.toggled.connect(self._toggle_detection)
        control_layout.addWidget(self.detection_switch)

        self.record_button = QPushButton("开始录像")
        self.record_button.setCheckable(True)
        self.record_button.setObjectName("recordButton")
        self.record_button.clicked.connect(self._toggle_recording)
        control_layout.addWidget(self.record_button)

        self.record_time = QLabel("00:00:00")
        self.record_time.setObjectName("recordTime")
        control_layout.addWidget(self.record_time)
        control_layout.addStretch(1)

        self.manual_button = self._command_button("手动模式", "manual", "secondaryButton")
        self.stop_button = self._command_button("停止", "stop", "stopButton")
        self.zero_button = self._command_button("归零", "zero", "secondaryButton")
        self.set_zero_button = QPushButton("设为零位")
        self.set_zero_button.setObjectName("secondaryButton")
        self.set_zero_button.clicked.connect(self._confirm_set_zero)
        self.power_button = QPushButton("断电")
        self.power_button.setObjectName("dangerButton")
        self.power_button.clicked.connect(self._confirm_poweroff)

        for button in (
            self.manual_button,
            self.stop_button,
            self.zero_button,
            self.set_zero_button,
            self.power_button,
        ):
            control_layout.addWidget(button)
        root.addWidget(controls)

        self.setCentralWidget(central)
        status = QStatusBar()
        status.setSizeGripEnabled(False)
        self.setStatusBar(status)
        self.statusBar().showMessage("系统初始化中")

    def _build_menus(self) -> None:
        inspection_menu = self.menuBar().addMenu("巡检")
        start_inspection = QAction("开始自主巡检（最近一次RMB路径）", self)
        start_inspection.triggered.connect(self.robot_runtime.start_inspection)
        stop_inspection = QAction("停止巡检并回到空闲", self)
        stop_inspection.triggered.connect(lambda: self.robot_runtime.submit_command("stop"))
        inspection_menu.addAction(start_inspection)
        inspection_menu.addAction(stop_inspection)

        navigation_menu = self.menuBar().addMenu("自动导航")
        for label, target in self.robot_runtime.navigation_targets():
            action = QAction(label, self)
            action.triggered.connect(
                lambda _checked=False, selected=target: self.robot_runtime.start_navigation(
                    selected,
                    enable_layer2=False,
                    enable_layer3=False,
                )
            )
            navigation_menu.addAction(action)
        navigation_menu.addSeparator()
        pause_navigation = QAction("暂停/继续导航", self)
        pause_navigation.triggered.connect(self.robot_runtime.pause_navigation)
        stop_navigation = QAction("停止导航", self)
        stop_navigation.triggered.connect(self.robot_runtime.stop_navigation)
        navigation_menu.addAction(pause_navigation)
        navigation_menu.addAction(stop_navigation)

        system_menu = self.menuBar().addMenu("系统")
        exit_action = QAction("安全退出", self)
        exit_action.triggered.connect(self.close)
        system_menu.addAction(exit_action)

    def _refresh(self) -> None:
        self._refresh_frame()
        self._refresh_status()

    def _refresh_frame(self) -> None:
        sequence, frame = FRAME_BUS.snapshot()
        if frame is None or sequence == self._last_frame_sequence:
            return
        self._last_frame_sequence = sequence
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width, channels = rgb.shape
        image = QImage(
            rgb.data,
            width,
            height,
            channels * width,
            QImage.Format_RGB888,
        ).copy()
        self._last_pixmap = QPixmap.fromImage(image)
        self._scale_video_pixmap()

    def resizeEvent(self, event) -> None:  # noqa: N802 - Qt API
        super().resizeEvent(event)
        self._scale_video_pixmap()

    def _scale_video_pixmap(self) -> None:
        if self._last_pixmap is None or self._last_pixmap.isNull():
            return
        self.video_label.setPixmap(
            self._last_pixmap.scaled(
                self.video_label.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )

    def _refresh_status(self) -> None:
        detection = get_latest_detection()
        motor = get_motor_snapshot()
        video = VIDEO_STATUS_BUS.snapshot()
        robot = self.robot_runtime.status()

        detection_requested = CONTROL_BUS.detection_enabled()
        if not detection_requested:
            self.position_badge.setText("当前位置：检测未开启")
            self.map_widget.set_detection_state(
                None,
                None,
                "INACTIVE",
                "目标检测未开启",
            )
        else:
            self._display_gate.update(detection)

            if self._display_gate.armed:
                current_name = DISPLAY_NAMES.get(
                    detection.confirmed_position,
                    detection.confirmed_position,
                )
                self.position_badge.setText(
                    f"当前位置：{current_name} [{detection.confirmed_position}]"
                )
                self.map_widget.set_detection_state(
                    detection.confirmed_position,
                    detection.candidate_position,
                    detection.safe_state,
                )
            else:
                self.position_badge.setText("当前位置：等待识别")
                self.map_widget.set_detection_state(
                    None,
                    None,
                    "UNCERTAIN",
                    "目标检测已开启，等待有效部位确认",
                )

        motion_codes = {"forward": "F", "backward": "B", "idle": "S"}
        mode_text = robot.state if robot.ready else ("连接失败" if robot.error else "连接中")
        self.motion_badge.setText(
            f"模式：{mode_text}｜运动：{motion_codes.get(motor.motion_mode, 'S')}"
        )
        self.motion_badge.setProperty(
            "status",
            "ok" if motor.feedback_valid else "warning",
        )
        self._refresh_widget_style(self.motion_badge)

        self.controller_badge.setText(
            "手柄：已连接" if robot.controller_connected else "手柄：未连接"
        )
        self.controller_badge.setProperty(
            "status",
            "ok" if robot.controller_connected else "warning",
        )
        self._refresh_widget_style(self.controller_badge)

        self.camera_badge.setText(
            "内窥镜：已连接" if video.camera_connected else "内窥镜：未连接"
        )
        self.camera_badge.setProperty("status", "ok" if video.camera_connected else "warning")
        self._refresh_widget_style(self.camera_badge)
        self.video_detail.setText(
            f"{video.message}｜{video.fps:.1f} FPS" if video.camera_connected else video.message
        )

        if video.recording:
            elapsed = max(0, int(time.monotonic() - video.recording_started_at))
            self.record_time.setText(
                f"{elapsed // 3600:02d}:{(elapsed % 3600) // 60:02d}:{elapsed % 60:02d}"
            )
            if not self.record_button.isChecked():
                self.record_button.setChecked(True)
            self.record_button.setText("结束录像")
        else:
            self.record_time.setText("00:00:00")
            if CONTROL_BUS.recording_path() is None:
                if self.record_button.isChecked():
                    self.record_button.setChecked(False)
                self.record_button.setText("开始录像")

        for button in (
            self.manual_button,
            self.stop_button,
            self.zero_button,
            self.set_zero_button,
            self.power_button,
        ):
            button.setEnabled(robot.ready)

        sequence, message, level, _updated_at = GUI_MESSAGE_BUS.snapshot()
        if sequence != self._last_message_sequence:
            self._last_message_sequence = sequence
            self.statusBar().setProperty("level", level)
            self._refresh_widget_style(self.statusBar())
            self.statusBar().showMessage(message)

    def _toggle_detection(self, enabled: bool) -> None:
        self._display_gate.reset(time.monotonic() if enabled else 0.0)
        CONTROL_BUS.set_detection_enabled(enabled)
        GUI_MESSAGE_BUS.publish("目标检测已开启" if enabled else "目标检测已关闭")

    def _toggle_recording(self, checked: bool) -> None:
        if checked:
            position = get_latest_detection().confirmed_position or "TR"
            filename = f"{datetime.now():%Y%m%d_%H%M%S}_{position}.avi"
            CONTROL_BUS.start_recording(RECORDING_ROOT / filename)
            self.record_button.setText("结束录像")
        else:
            CONTROL_BUS.stop_recording()
            self.record_button.setText("开始录像")

    def _confirm_set_zero(self) -> None:
        answer = QMessageBox.question(
            self,
            "确认设零",
            "将当前位置写入三轴电机零点，是否继续？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self.robot_runtime.submit_command("set_zero")

    def _confirm_poweroff(self) -> None:
        answer = QMessageBox.warning(
            self,
            "确认断电",
            "机器人将进入断电维护状态，是否继续？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self.robot_runtime.submit_command("poweroff")

    def closeEvent(self, event: QCloseEvent) -> None:  # noqa: N802 - Qt API
        if self._closing:
            event.accept()
            return
        answer = QMessageBox.question(
            self,
            "安全退出",
            "将停止录像、关闭视频并请求机器人安全退出，是否继续？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            event.ignore()
            return
        self._closing = True
        self._timer.stop()
        CONTROL_BUS.stop_recording()
        CONTROL_BUS.request_shutdown()
        self.robot_runtime.request_shutdown()
        event.accept()

    def _command_button(self, text: str, command: str, object_name: str) -> QPushButton:
        button = QPushButton(text)
        button.setObjectName(object_name)
        button.clicked.connect(lambda: self.robot_runtime.submit_command(command))
        return button

    @staticmethod
    def _badge(text: str, object_name: str) -> QLabel:
        label = QLabel(text)
        label.setObjectName(object_name)
        label.setAlignment(Qt.AlignCenter)
        return label

    @staticmethod
    def _refresh_widget_style(widget: QWidget) -> None:
        widget.style().unpolish(widget)
        widget.style().polish(widget)

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow, QWidget {
                background: #070d16;
                color: #dbeaf5;
                font-family: "Microsoft YaHei";
                font-size: 13px;
            }
            QMenuBar { background: #0b1421; padding: 4px; }
            QMenuBar::item { padding: 7px 15px; border-radius: 5px; }
            QMenuBar::item:selected { background: #163049; }
            QMenu { background: #101c2a; border: 1px solid #2b4961; }
            QMenu::item { padding: 8px 28px; }
            QMenu::item:selected { background: #15546b; }
            QLabel#title { font-size: 24px; font-weight: 700; color: #edfaff; }
            QLabel#subtitle { font-size: 10px; letter-spacing: 2px; color: #607b91; }
            QLabel#activeBadge, QLabel#neutralBadge {
                background: #0e1b29; border: 1px solid #294861;
                border-radius: 7px; padding: 8px 12px; min-width: 105px;
            }
            QLabel#activeBadge { color: #5de7f6; border-color: #157f99; }
            QLabel[status="ok"] { color: #76e6b1; border-color: #267b59; }
            QLabel[status="warning"] { color: #ffc764; border-color: #8a6224; }
            QFrame#card { background: #0a111d; border: 1px solid #1b3144; border-radius: 10px; }
            QLabel#panelTitle { font-size: 16px; font-weight: 700; color: #e7f4ff; }
            QLabel#panelDetail { color: #7690a4; }
            QLabel#video { background: #000000; border: 2px solid #166a84; border-radius: 8px; color: #507084; }
            QFrame#controls { background: #0d1724; border: 1px solid #20384c; border-radius: 10px; }
            QCheckBox#detectionSwitch { spacing: 10px; font-weight: 700; color: #b9ccda; padding: 8px; }
            QCheckBox#detectionSwitch::indicator { width: 46px; height: 24px; border-radius: 12px; background: #334354; }
            QCheckBox#detectionSwitch::indicator:checked { background: #18a8c7; border: 3px solid #6ce5f5; }
            QPushButton { min-height: 38px; min-width: 86px; padding: 0 14px; border-radius: 7px; font-weight: 700; }
            QPushButton#secondaryButton { background: #172a3b; border: 1px solid #31506a; color: #dcebf5; }
            QPushButton#secondaryButton:hover { background: #21415a; }
            QPushButton#stopButton { background: #9a5a10; border: 1px solid #d28a2e; color: white; }
            QPushButton#dangerButton { background: #942f37; border: 1px solid #dc5460; color: white; }
            QPushButton#recordButton { background: #173044; border: 1px solid #3a6682; color: #e6f6ff; }
            QPushButton#recordButton:checked { background: #8e2430; border-color: #ff5b69; }
            QPushButton:disabled { background: #17202a; color: #566674; border-color: #263440; }
            QLabel#recordTime { color: #ff7b87; font-family: Consolas; font-weight: 700; min-width: 70px; }
            QStatusBar { background: #0b1421; color: #8fa7ba; border-top: 1px solid #1c3041; }
            QStatusBar[level="error"] { color: #ff7380; }
            QStatusBar[level="warning"] { color: #ffc764; }
            QSplitter::handle { background: #102333; width: 4px; }
            """
        )
