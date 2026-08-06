"""Large, read-only bronchial position visualization for the recognition GUI."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import (
    QColor,
    QFont,
    QPainter,
    QPen,
    QPixmap,
    QPolygonF,
    QRadialGradient,
)
from PyQt5.QtWidgets import QWidget

from Bronchialtree_identification.motor_linked_detection.config import PARENT_MAP


DISPLAY_NAMES = {
    "TR": "气管",
    "RMB": "右主支气管",
    "RULB": "右上叶",
    "BI": "中间支气管",
    "RMLB": "右中叶",
    "RLLB": "右下叶",
    "LMB": "左主支气管",
    "LULB": "左上叶",
    "LLLB": "左下叶",
}

# Coordinates are relative to the runtime crop (source y=384..1459). They were
# re-registered against the airway centreline in the source asset. Anatomical
# right is shown on the viewer's left.
NODE_POSITIONS = {
    "TR": (0.525, 0.247),
    "RMB": (0.445, 0.552),
    "RULB": (0.325, 0.524),
    "BI": (0.369, 0.666),
    "RMLB": (0.248, 0.759),
    "RLLB": (0.360, 0.869),
    "LMB": (0.670, 0.620),
    "LULB": (0.796, 0.483),
    "LLLB": (0.766, 0.717),
}

# Straight parent/child chords cut across the model and can appear to point at
# the wrong airway. These anatomical waypoints route each topology edge through
# the visible carina or lobar take-off while remaining display-only.
EDGE_WAYPOINTS = {
    ("TR", "RMB"): ((0.504, 0.501),),
    ("TR", "LMB"): ((0.504, 0.501), (0.590, 0.555)),
    ("RMB", "RULB"): ((0.405, 0.572), (0.365, 0.550)),
    ("RMB", "BI"): ((0.405, 0.572),),
    ("BI", "RMLB"): ((0.310, 0.747),),
    ("BI", "RLLB"): ((0.310, 0.747),),
    ("LMB", "LULB"): ((0.743, 0.642), (0.798, 0.605), (0.812, 0.569)),
    ("LMB", "LLLB"): ((0.743, 0.642),),
}


class BronchusMapWidget(QWidget):
    def __init__(self, asset_path: Path, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setMinimumSize(560, 560)
        self._confirmed: Optional[str] = None
        self._candidate: Optional[str] = None
        self._safe_state = "CONFIRMED"
        self._empty_message = "目标检测未开启"
        original = QPixmap(str(asset_path))
        if not original.isNull() and original.height() > 900:
            # The source asset has large black margins. Crop at runtime so the
            # original reusable file remains untouched.
            top = int(original.height() * 0.20)
            height = int(original.height() * 0.56)
            width = int(original.width() * 0.97)
            self._image = original.copy(0, top, width, height)
        else:
            self._image = original

    def set_detection_state(
        self,
        confirmed: Optional[str],
        candidate: Optional[str],
        safe_state: str,
        empty_message: str = "等待识别",
    ) -> None:
        state = (confirmed, candidate, safe_state or "UNCERTAIN", empty_message)
        if state == (
            self._confirmed,
            self._candidate,
            self._safe_state,
            self._empty_message,
        ):
            return
        self._confirmed, self._candidate, self._safe_state, self._empty_message = state
        self.update()

    @staticmethod
    def _path_to(position: Optional[str]) -> set[str]:
        if not position:
            return set()
        result = {position}
        node = position
        while node in PARENT_MAP:
            node = PARENT_MAP[node]
            result.add(node)
        result.add("TR")
        return result

    def paintEvent(self, _event) -> None:  # noqa: N802 - Qt API
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor("#0a111d"))

        margin = 22
        title_height = 72
        footer_height = 48
        content = QRectF(
            margin,
            title_height,
            max(1, self.width() - margin * 2),
            max(1, self.height() - title_height - footer_height - margin),
        )

        painter.setPen(QColor("#e7f4ff"))
        painter.setFont(QFont("Microsoft YaHei", 15, QFont.Bold))
        painter.drawText(
            QRectF(margin, 12, self.width() - margin * 2, 34),
            Qt.AlignLeft | Qt.AlignVCenter,
            "支气管操作位置",
        )
        painter.setFont(QFont("Microsoft YaHei", 10))
        painter.setPen(QColor("#7f95aa"))
        painter.drawText(
            QRectF(margin, 42, self.width() - margin * 2, 24),
            Qt.AlignLeft | Qt.AlignVCenter,
            "点亮位置来自电机约束后的确认状态",
        )

        image_rect = self._fit_rect(content, self._image.width(), self._image.height())
        if not self._image.isNull():
            painter.setOpacity(0.86)
            painter.drawPixmap(image_rect.toRect(), self._image)
            painter.setOpacity(1.0)

        points = {
            name: QPointF(
                image_rect.left() + rx * image_rect.width(),
                image_rect.top() + ry * image_rect.height(),
            )
            for name, (rx, ry) in NODE_POSITIONS.items()
        }
        active_path = self._path_to(self._confirmed)

        # Draw the simplified topology on top of the anatomical asset. It is
        # intentionally read-only and does not participate in recognition.
        for child, parent in PARENT_MAP.items():
            if child not in points or parent not in points:
                continue
            on_path = child in active_path and parent in active_path
            color = QColor("#20c7e8") if on_path else QColor(64, 86, 108, 150)
            painter.setPen(QPen(color, 4 if on_path else 2, Qt.SolidLine, Qt.RoundCap))
            route = [points[parent]]
            route.extend(
                QPointF(
                    image_rect.left() + rx * image_rect.width(),
                    image_rect.top() + ry * image_rect.height(),
                )
                for rx, ry in EDGE_WAYPOINTS.get((parent, child), ())
            )
            route.append(points[child])
            painter.drawPolyline(QPolygonF(route))

        for name, point in points.items():
            if name == self._confirmed:
                self._draw_glow(painter, point, QColor("#22d3ee"), 28)
                fill = QColor("#d8fbff")
                border = QColor("#22d3ee")
                radius = 11
            elif name == self._candidate:
                self._draw_glow(painter, point, QColor("#f6b73c"), 23)
                fill = QColor("#fff2c7")
                border = QColor("#f6b73c")
                radius = 9
            elif name in active_path:
                fill = QColor("#2baac0")
                border = QColor("#60d9ed")
                radius = 7
            else:
                fill = QColor("#213245")
                border = QColor("#557089")
                radius = 6

            painter.setPen(QPen(border, 2))
            painter.setBrush(fill)
            painter.drawEllipse(point, radius, radius)
            self._draw_label(painter, name, point, name == self._confirmed)

        footer = QRectF(margin, self.height() - footer_height, self.width() - margin * 2, 32)
        painter.setFont(QFont("Microsoft YaHei", 13, QFont.Bold))
        if self._confirmed:
            current_name = DISPLAY_NAMES.get(self._confirmed, self._confirmed)
            painter.setPen(QColor("#23d3ee"))
            painter.drawText(
                footer,
                Qt.AlignCenter,
                f"当前位置：{current_name}  [{self._confirmed}]",
            )
        else:
            painter.setPen(QColor("#879dad"))
            painter.drawText(footer, Qt.AlignCenter, self._empty_message)

    @staticmethod
    def _fit_rect(container: QRectF, width: int, height: int) -> QRectF:
        if width <= 0 or height <= 0:
            return container
        scale = min(container.width() / width, container.height() / height)
        w = width * scale
        h = height * scale
        return QRectF(
            container.center().x() - w / 2,
            container.center().y() - h / 2,
            w,
            h,
        )

    @staticmethod
    def _draw_glow(painter: QPainter, point: QPointF, color: QColor, radius: int) -> None:
        glow = QRadialGradient(point, radius)
        bright = QColor(color)
        bright.setAlpha(150)
        clear = QColor(color)
        clear.setAlpha(0)
        glow.setColorAt(0.0, bright)
        glow.setColorAt(1.0, clear)
        painter.setPen(Qt.NoPen)
        painter.setBrush(glow)
        painter.drawEllipse(point, radius, radius)

    @staticmethod
    def _draw_label(painter: QPainter, name: str, point: QPointF, active: bool) -> None:
        label = f"{DISPLAY_NAMES.get(name, name)}  {name}"
        painter.setFont(QFont("Microsoft YaHei", 8, QFont.Bold if active else QFont.Normal))
        metrics = painter.fontMetrics()
        width = metrics.horizontalAdvance(label) + 12
        height = metrics.height() + 6
        to_left = point.x() > painter.viewport().width() * 0.57
        x = point.x() - width - 14 if to_left else point.x() + 14
        y = point.y() - height / 2
        rect = QRectF(x, y, width, height)
        painter.setPen(QPen(QColor("#22d3ee") if active else QColor("#4c6980"), 1))
        painter.setBrush(QColor(8, 20, 34, 220))
        painter.drawRoundedRect(rect, 5, 5)
        painter.setPen(QColor("#e8fbff") if active else QColor("#b7c8d6"))
        painter.drawText(rect, Qt.AlignCenter, label)
