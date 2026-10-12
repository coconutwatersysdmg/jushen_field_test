# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Any, Dict, Mapping

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QAbstractItemView,
    QDialog,
    QFrame,
    QHeaderView,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from services.board_data_service import build_board_data, build_corner_comparison
from services.sensor_calibration_service import SensorCalibrationService


METRICS = (
    ("length_mm", "长度 ⓘ", "length"),
    ("width_mm", "宽度 ⓘ", "width"),
    ("height_mean_mm", "平均高度 ⓘ", "height"),
    ("tilt_angle_deg", "纵向倾斜角 ⓘ", "tilt"),
    ("offset_angle_deg", "水平偏移角 ⓘ", "offset"),
)

FORMULA_DETAILS: Dict[str, Dict[str, str]] = {
    "length": {
        "title": "长度计算说明",
        "formula": "L = [d(P1,P3) + d(P2,P4)] / 2",
        "detail": "d(A,B) = √[(xB-xA)² + (yB-yA)² + (zB-zA)²]\n\n分别计算左右两条纵向边的三维长度并取平均值。\n单位：毫米（mm）。",
    },
    "width": {
        "title": "宽度计算说明",
        "formula": "W = [d(P1,P2) + d(P3,P4)] / 2",
        "detail": "d(A,B) = √[(xB-xA)² + (yB-yA)² + (zB-zA)²]\n\n分别计算前后两条横向边的三维长度并取平均值。\n单位：毫米（mm）。",
    },
    "height": {
        "title": "平均高度计算说明",
        "formula": "H = (z1 + z2 + z3 + z4) / 4",
        "detail": "取四个 WORLD 角点 Z 坐标的算术平均值。\n单位：毫米（mm）。",
    },
    "tilt": {
        "title": "纵向倾斜角计算说明",
        "formula": "C_head = (P1 + P2) / 2\nC_tail = (P3 + P4) / 2\nΔ = C_tail - C_head = (dx, dy, dz)\ntilt = atan2(dz, sqrt(dx² + dy²)) × 180 / π",
        "detail": "固定车头端中心，比较车尾端相对车头端的高度变化。\n正值：车尾端高于车头端；负值：车尾端低于车头端。\n单位：度（°）。",
    },
    "offset": {
        "title": "水平偏移角计算说明",
        "formula": "C_head = (P1 + P2) / 2\nC_tail = (P3 + P4) / 2\nΔ = C_tail - C_head = (dx, dy, dz)\noffset = atan2(dx, -dy) × 180 / π",
        "detail": "WORLD 坐标系的负 Y 方向为零度基准。\n向正 X 方向偏转为正，向负 X 方向偏转为负。\n单位：度（°）。",
    },
}


class BoardDataPanel(QWidget):
    """Read-only comparison of radar board and camera loading-area metrics."""

    def __init__(self, parent: QWidget | None = None, lidar_world_matrix: Any = None):
        super().__init__(parent)
        self.setObjectName("boardDataPanel")
        if lidar_world_matrix is None:
            try:
                lidar_world_matrix = SensorCalibrationService().fixed_world_matrix("lidar")
            except (FileNotFoundError, KeyError, TypeError, ValueError):
                lidar_world_matrix = None
        self._lidar_world_matrix = lidar_world_matrix
        self.board_tables: list[QTableWidget] = []
        self.board_title_labels: list[QLabel] = []
        self.corner_table: QTableWidget
        self._formula_dialogs: list[QDialog] = []
        self.active_formula_dialog: QDialog | None = None
        self._build()
        self.update_from_round_data({})

    def _build(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(18, 16, 18, 18)
        layout.setSpacing(12)

        title = QLabel("车板与装载区域数据")
        title.setObjectName("boardDataPageTitle")
        layout.addWidget(title)

        note = QLabel("点击带 ⓘ 的指标名称，可查看该项数据的计算公式")
        note.setObjectName("mutedLabel")
        layout.addWidget(note)

        for board_title in ("第一块板", "第二块板"):
            card = QFrame()
            card.setObjectName("boardDataCard")
            card.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            card_layout = QVBoxLayout(card)
            card_layout.setContentsMargins(12, 10, 12, 12)
            card_layout.setSpacing(8)

            title_label = QLabel(board_title)
            title_label.setObjectName("boardDataCardTitle")
            self.board_title_labels.append(title_label)
            card_layout.addWidget(title_label)

            table = self._create_table()
            self.board_tables.append(table)
            card_layout.addWidget(table)
            layout.addWidget(card, 0)

        corner_title = QLabel("角点坐标对照（固定桁架 WORLD 坐标系）")
        corner_title.setObjectName("boardDataCardTitle")
        layout.addWidget(corner_title)
        self.corner_table = self._create_corner_table()
        layout.addWidget(self.corner_table, 1)
        self.setStyleSheet(
            "#boardDataPageTitle { font-size: 18px; font-weight: 700; color: #123d6a; }"
            "#boardDataCard { background: #ffffff; border: 1px solid #cbd5e1; border-radius: 4px; }"
            "#boardDataCardTitle { font-size: 15px; font-weight: 700; color: #174f87; }"
            "QTableWidget { background: #ffffff; alternate-background-color: #f7f9fc; gridline-color: #d7dee8; }"
            "QHeaderView::section { background: #edf3f9; color: #164f87; border: 0; border-right: 1px solid #d7dee8; border-bottom: 1px solid #cbd5e1; padding: 7px 5px; font-weight: 700; }"
        )

    def _create_table(self) -> QTableWidget:
        table = QTableWidget(2, 6)
        table.setHorizontalHeaderLabels(["数据来源", *[label for _, label, _ in METRICS]])
        table.verticalHeader().setVisible(False)
        table.verticalHeader().setDefaultSectionSize(40)
        table.setAlternatingRowColors(True)
        table.setShowGrid(True)
        table.setWordWrap(False)
        table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        table.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        table.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        table.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        table.setFixedHeight(126)
        header = table.horizontalHeader()
        header.setSectionsClickable(True)
        header.setCursor(Qt.CursorShape.PointingHandCursor)
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        for section in range(1, 6):
            header.setSectionResizeMode(section, QHeaderView.ResizeMode.Stretch)
        header.sectionClicked.connect(self._on_metric_header_clicked)

        for row, source in enumerate(("雷达车板数据", "相机装载区域数据")):
            source_item = QTableWidgetItem(source)
            source_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            source_item.setForeground(QColor("#516273") if row == 0 else QColor("#0f5fa8"))
            table.setItem(row, 0, source_item)
            for column in range(1, 6):
                item = QTableWidgetItem("—")
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                table.setItem(row, column, item)
        return table

    def _create_corner_table(self) -> QTableWidget:
        table = QTableWidget(0, 7)
        table.setObjectName("boardCornerComparisonTable")
        table.setHorizontalHeaderLabels([
            "角点",
            "雷达 X(mm)", "雷达 Y(mm)", "雷达 Z(mm)",
            "相机 X(mm)", "相机 Y(mm)", "相机 Z(mm)",
        ])
        table.verticalHeader().setVisible(False)
        table.verticalHeader().setDefaultSectionSize(34)
        table.setAlternatingRowColors(True)
        table.setShowGrid(True)
        table.setWordWrap(False)
        table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        table.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        table.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        table.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        header = table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        for section in range(1, 7):
            header.setSectionResizeMode(section, QHeaderView.ResizeMode.Stretch)
        return table

    def _on_metric_header_clicked(self, section: int) -> None:
        if 1 <= int(section) <= len(METRICS):
            self.open_formula(METRICS[int(section) - 1][2])

    @staticmethod
    def _format_value(metric_key: str, value: Any) -> str:
        if value is None:
            return "—"
        try:
            number = float(value)
        except (TypeError, ValueError):
            return "—"
        if metric_key.endswith("_deg"):
            return f"{number:.6f}°"
        return f"{number:.3f} mm"

    def update_from_round_data(self, round_data: Mapping[str, Any]) -> None:
        boards = build_board_data(round_data, self._lidar_world_matrix)
        for board_index, table in enumerate(self.board_tables):
            board = boards[board_index]
            for row, source_key in enumerate(("radar", "camera")):
                metrics = board.get(source_key)
                for column, (metric_key, _label, _formula_key) in enumerate(METRICS, start=1):
                    value = metrics.get(metric_key) if isinstance(metrics, Mapping) else None
                    table.item(row, column).setText(self._format_value(metric_key, value))

        corners = build_corner_comparison(round_data, self._lidar_world_matrix)
        self.corner_table.setRowCount(len(corners))
        for row, corner in enumerate(corners):
            values = [corner.get("point_id")]
            for source_key in ("radar", "camera"):
                point = corner.get(source_key)
                for axis in ("x", "y", "z"):
                    value = point.get(axis) if isinstance(point, Mapping) else None
                    values.append("—" if value is None else f"{float(value):.3f}")
            for column, value in enumerate(values):
                item = QTableWidgetItem(str(value))
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.corner_table.setItem(row, column, item)

    def open_formula(self, metric_key: str) -> QDialog:
        detail = FORMULA_DETAILS[metric_key]
        dialog = QDialog(self)
        dialog.setObjectName("boardFormulaDialog")
        dialog.setWindowTitle(detail["title"])
        dialog.setModal(False)
        dialog.setMinimumWidth(520)
        dialog_layout = QVBoxLayout(dialog)
        dialog_layout.setContentsMargins(18, 16, 18, 14)
        dialog_layout.setSpacing(12)

        applies = QLabel("适用于：雷达车板数据、相机装载区域数据（均使用固定桁架 WORLD 坐标）")
        applies.setWordWrap(True)
        applies.setObjectName("mutedLabel")
        dialog_layout.addWidget(applies)

        formula = QLabel(detail["formula"])
        formula.setTextFormat(Qt.TextFormat.PlainText)
        formula.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        formula.setWordWrap(True)
        formula.setStyleSheet("font-size: 14px; font-weight: 700; color: #123d6a; background: #f2f6fa; border: 1px solid #cfdae7; padding: 10px;")
        dialog_layout.addWidget(formula)

        explanation = QLabel(detail["detail"])
        explanation.setTextFormat(Qt.TextFormat.PlainText)
        explanation.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        explanation.setWordWrap(True)
        dialog_layout.addWidget(explanation)

        button_row = QHBoxLayout()
        button_row.addStretch(1)
        close_button = QPushButton("关闭")
        close_button.clicked.connect(dialog.close)
        button_row.addWidget(close_button)
        dialog_layout.addLayout(button_row)

        self._formula_dialogs.append(dialog)
        dialog.destroyed.connect(lambda: self._forget_dialog(dialog))
        self.active_formula_dialog = dialog
        dialog.show()
        dialog.raise_()
        dialog.activateWindow()
        return dialog

    def _forget_dialog(self, dialog: QDialog) -> None:
        if dialog in self._formula_dialogs:
            self._formula_dialogs.remove(dialog)
        if self.active_formula_dialog is dialog:
            self.active_formula_dialog = None
