# -*- coding: utf-8 -*-
"""现场工作区共用小控件：步骤条、图片墙、路径栏、算法占位槽。"""
from __future__ import annotations

import json
from pathlib import Path

from PySide6.QtCore import QSize, Qt, QUrl, Signal
from PySide6.QtGui import QDesktopServices, QPixmap
from PySide6.QtWidgets import (
    QAbstractItemView, QFileDialog, QFrame, QHBoxLayout, QLabel, QLineEdit,
    QListWidget, QListWidgetItem, QMessageBox, QPushButton, QVBoxLayout, QWidget,
)


class StepStrip(QWidget):
    """横向步骤条；RUN/DONE/WAIT/SKIP 直接上色，现场一眼能看出走到哪一步。"""

    def __init__(self, steps: list[tuple[str, str]] | None = None, parent=None):
        super().__init__(parent)
        self._steps = list(steps or [])
        self._labels: list[QLabel] = []
        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)
        for index, (_code, name) in enumerate(self._steps):
            label = QLabel(f"{index + 1}. {name}\nWAIT")
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setWordWrap(True)
            label.setMinimumHeight(46)
            self._labels.append(label)
            layout.addWidget(label, 1)
            if index < len(self._steps) - 1:
                arrow = QLabel("›")
                arrow.setAlignment(Qt.AlignmentFlag.AlignCenter)
                arrow.setFixedWidth(12)
                layout.addWidget(arrow, 0)
        self.set_steps(self._steps)

    def set_steps(self, steps: list[tuple[str, str]]) -> None:
        self._steps = list(steps)
        for label in self._labels:
            label.deleteLater()
        self._labels.clear()
        layout = self.layout()
        while layout.count():
            item = layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()
        for index, (_code, name) in enumerate(self._steps):
            label = QLabel(f"{index + 1}. {name}\nWAIT")
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setWordWrap(True)
            label.setMinimumHeight(46)
            self._labels.append(label)
            layout.addWidget(label, 1)
            if index < len(self._steps) - 1:
                arrow = QLabel("›")
                arrow.setAlignment(Qt.AlignmentFlag.AlignCenter)
                arrow.setFixedWidth(12)
                layout.addWidget(arrow, 0)
        self.set_current(-1)

    def states(self) -> list[str]:
        """返回每格当前状态，供主界面顶部的现场流程条同步显示。"""
        result = []
        for label in self._labels:
            text = label.text()
            result.append(text.rsplit("\n", 1)[-1].strip() if "\n" in text else "WAIT")
        return result

    def set_current(self, index: int, status: str = "RUN") -> None:
        for position, label in enumerate(self._labels):
            if position < index:
                state, bg, border, fg = "DONE", "#e8f5e9", "#43a047", "#1b5e20"
            elif position == index and index >= 0:
                if status == "DONE":
                    state, bg, border, fg = "DONE", "#e8f5e9", "#43a047", "#1b5e20"
                elif status == "SKIP":
                    state, bg, border, fg = "SKIP", "#eceff1", "#b0bec5", "#78909c"
                else:
                    state, bg, border, fg = "RUN", "#1565c0", "#0d47a1", "#ffffff"
            else:
                state, bg, border, fg = "WAIT", "#ffffff", "#c5cad3", "#5c6675"
            name = self._steps[position][1] if position < len(self._steps) else ""
            label.setText(f"{position + 1}. {name}\n{state}")
            label.setStyleSheet(
                f"background:{bg};border:1px solid {border};border-radius:4px;"
                f"color:{fg};font-size:13px;font-weight:500;padding:3px;"
            )


class MediaGallery(QFrame):
    """缩略图列表 + 大图预览；原图和标注结果图都往这里塞。"""

    def __init__(self, empty_text: str = "暂无图片", parent=None):
        super().__init__(parent)
        self.setObjectName("card")
        self._entries: list[dict[str, str]] = []
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(5)
        header = QHBoxLayout()
        self.title = QLabel("相机图片 / 结果图")
        self.title.setStyleSheet("font-size:13px;font-weight:500;color:#1565c0;")
        header.addWidget(self.title, 1)
        self.count_label = QLabel("0 张")
        header.addWidget(self.count_label, 0)
        layout.addLayout(header)

        self.empty = QLabel(empty_text)
        self.empty.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.empty.setWordWrap(True)
        self.empty.setStyleSheet("color:#8a94a3;font-size:13px;")
        self.empty.setMinimumHeight(160)
        layout.addWidget(self.empty, 1)

        self.list = QListWidget()
        self.list.setViewMode(QListWidget.ViewMode.IconMode)
        self.list.setResizeMode(QListWidget.ResizeMode.Adjust)
        self.list.setMovement(QListWidget.Movement.Static)
        self.list.setIconSize(QSize(112, 84))
        self.list.setGridSize(QSize(128, 112))
        self.list.setSpacing(4)
        self.list.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.list.itemClicked.connect(self._show_entry)
        self.list.currentItemChanged.connect(
            lambda current, _previous: self._show_entry(current) if current else None
        )
        self.list.setVisible(False)
        layout.addWidget(self.list, 1)

        self.preview = QLabel("点击缩略图查看大图")
        self.preview.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.preview.setMinimumHeight(180)
        self.preview.setStyleSheet(
            "background:#f7f8fa;border:1px solid #d7dce3;border-radius:4px;color:#8a94a3;font-size:12px;"
        )
        self.preview.setVisible(False)
        layout.addWidget(self.preview, 0)

        self.caption = QLabel("")
        self.caption.setWordWrap(True)
        self.caption.setStyleSheet("color:#5c6675;font-size:12px;")
        layout.addWidget(self.caption, 0)

    def set_images(self, entries: list[dict[str, str]]) -> None:
        self._entries = [dict(entry) for entry in entries if Path(entry.get("path", "")).is_file()]
        self.list.clear()
        self.count_label.setText(f"{len(self._entries)} 张")
        has_images = bool(self._entries)
        self.empty.setVisible(not has_images)
        self.list.setVisible(has_images)
        self.preview.setVisible(has_images)
        if not has_images:
            self.caption.setText("")
            return
        for entry in self._entries:
            item = QListWidgetItem(entry.get("label") or Path(entry["path"]).name)
            pixmap = QPixmap(entry["path"])
            if not pixmap.isNull():
                item.setIcon(pixmap.scaled(
                    QSize(112, 84),
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation,
                ))
            item.setToolTip(entry["path"])
            self.list.addItem(item)
        self.list.setCurrentRow(0)

    def _show_entry(self, item: QListWidgetItem | None) -> None:
        if item is None:
            return
        row = self.list.row(item)
        if row < 0 or row >= len(self._entries):
            return
        entry = self._entries[row]
        pixmap = QPixmap(entry["path"])
        if pixmap.isNull():
            self.preview.setText(f"图片打不开：{entry['path']}")
            return
        size = self.preview.size()
        width = max(240, size.width() - 16)
        height = max(140, size.height() - 16)
        self.preview.setPixmap(pixmap.scaled(
            QSize(width, height),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        ))
        self.caption.setText(f"{entry.get('label', '')} · {entry['path']}")


class PathBar(QFrame):
    """车号 + 保存路径 + 一键打开目录。"""

    vehicleChanged = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("card")
        self._path = ""
        layout = QHBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(6)
        car_label = QLabel("车号")
        car_label.setStyleSheet("font-size:12px;font-weight:500;color:#5c6675;")
        layout.addWidget(car_label, 0)
        self.vehicle_edit = QLineEdit()
        self.vehicle_edit.setMaximumWidth(190)
        self.vehicle_edit.setPlaceholderText("例如 FIELD-VEHICLE-001")
        self.vehicle_edit.editingFinished.connect(
            lambda: self.vehicleChanged.emit(self.vehicle_edit.text())
        )
        layout.addWidget(self.vehicle_edit, 0)
        label = QLabel("保存位置")
        label.setStyleSheet("font-size:12px;font-weight:500;color:#5c6675;")
        layout.addWidget(label, 0)
        self.path_label = QLabel("尚未开始，导出后显示目录")
        self.path_label.setStyleSheet("font-size:12px;color:#1565c0;")
        self.path_label.setWordWrap(True)
        layout.addWidget(self.path_label, 1)
        self.open_button = QPushButton("打开文件夹")
        self.open_button.setToolTip("在系统文件管理器中打开当前归档目录")
        self.open_button.clicked.connect(self.open)
        layout.addWidget(self.open_button, 0)

    def set_vehicle_id(self, value: str) -> None:
        self.vehicle_edit.setText(str(value or ""))

    def set_path(self, path: str) -> None:
        self._path = str(path or "")
        self.path_label.setText(self._path or "尚未开始，导出后显示目录")
        self.open_button.setEnabled(bool(self._path))

    def open(self) -> None:
        if not self._path:
            return
        target = Path(self._path)
        if target.is_file():
            target = target.parent
        if target.is_dir():
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(target)))
        else:
            QMessageBox.information(self, "目录不存在", f"目录还没生成：\n{target}")


class AlgoSlot(QFrame):
    """算法空壳：不造假结果，只能手动载入结果 JSON 做界面联调。"""

    resultLoaded = Signal(dict)

    def __init__(self, title: str, example_kind: str = "", parent=None):
        super().__init__(parent)
        self.setObjectName("card")
        self.example_kind = example_kind
        self.example_path = ""
        self.result: dict = {}
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(5)
        self.title = QLabel(title)
        self.title.setStyleSheet("font-size:13px;font-weight:500;color:#1565c0;")
        layout.addWidget(self.title, 0)

        row = QHBoxLayout()
        self.status = QLabel("待接入 · 等待载入结果 JSON")
        self.status.setWordWrap(True)
        self.status.setStyleSheet("font-size:12px;color:#8a94a3;")
        row.addWidget(self.status, 1)
        load_btn = QPushButton("载入结果 JSON…")
        load_btn.setToolTip("选择一份算法输出 JSON，仅用于界面联调，不调用算法")
        load_btn.clicked.connect(self.choose_result)
        row.addWidget(load_btn, 0)
        if example_kind:
            example_btn = QPushButton("用示例结果")
            example_btn.setToolTip("载入 examples/module_outputs 下的样例 JSON")
            example_btn.clicked.connect(self.load_example)
            row.addWidget(example_btn, 0)
        layout.addLayout(row)

        self.detail = QLabel("")
        self.detail.setWordWrap(True)
        self.detail.setMaximumHeight(90)
        self.detail.setStyleSheet(
            "background:#f7f8fa;border:1px solid #d7dce3;border-radius:4px;"
            "font-size:12px;color:#5c6675;padding:4px;"
        )
        layout.addWidget(self.detail, 0)

    def set_result(self, payload: dict, path: str = "") -> None:
        self.result = dict(payload or {})
        self.status.setText(f"已载入联调结果：{path or '本地 JSON'}")
        self.status.setStyleSheet("font-size:12px;color:#2e7d32;")
        text = json.dumps(self.result, ensure_ascii=False, indent=2)
        self.detail.setText(text[:900])

    def clear(self) -> None:
        self.result = {}
        self.status.setText("待接入 · 等待载入结果 JSON")
        self.status.setStyleSheet("font-size:12px;color:#8a94a3;")
        self.detail.setText("")

    def choose_result(self) -> None:
        filename, _ = QFileDialog.getOpenFileName(
            self, "选择算法结果 JSON", "", "JSON 文件 (*.json)"
        )
        if not filename:
            return
        try:
            data = json.loads(Path(filename).read_text(encoding="utf-8"))
            if not isinstance(data, dict):
                raise ValueError("顶层需要是 JSON 对象")
        except Exception as exc:
            QMessageBox.warning(self, "载入失败", str(exc))
            return
        self.set_result(data, filename)
        self.resultLoaded.emit({"path": filename, "data": data})

    def set_example_path(self, path: str) -> None:
        self.example_path = str(path or "")

    def load_example(self, _checked: bool = False) -> None:
        if not self.example_path:
            QMessageBox.information(self, "示例不存在", "该算法位没有配置示例结果文件")
            return
        try:
            data = json.loads(Path(self.example_path).read_text(encoding="utf-8"))
            if not isinstance(data, dict):
                raise ValueError("顶层需要是 JSON 对象")
        except Exception as exc:
            QMessageBox.warning(self, "载入失败", str(exc))
            return
        self.set_result(data, self.example_path)
        self.resultLoaded.emit({"path": self.example_path, "data": data})
