# -*- coding: utf-8 -*-
"""Field offline image inputs; radar selection remains in the main window."""
from __future__ import annotations

from copy import deepcopy
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QFormLayout, QLineEdit, QPushButton, QHBoxLayout,
    QFileDialog, QDialogButtonBox, QLabel, QScrollArea, QWidget,
)


class DebugInputDialog(QDialog):
    def __init__(self, initial=None, parent=None):
        super().__init__(parent)
        self.initial = deepcopy(initial or {})
        self.lines = {}
        self.setWindowTitle("现场离线图像输入")
        self.resize(880, 580)
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("第2步使用插孔 RGB-D；第3步使用海康照片；第4步使用放货检测照片。雷达 PCD 请在主界面选择。"))
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        body = QWidget()
        form = QFormLayout(body)
        scroll.setWidget(body)
        layout.addWidget(scroll, 1)

        def add_file(key, title, value, kind="image"):
            row = QHBoxLayout()
            line = QLineEdit(str(value or ""))
            button = QPushButton("选择")
            button.clicked.connect(lambda _=False: self._choose(line, kind))
            row.addWidget(line, 1)
            row.addWidget(button)
            form.addRow(title, row)
            self.lines[key] = line

        add_file("pallet_rgb", "第2步 插孔 RGB", self.initial.get("pallet_rgb"))
        add_file("pallet_depth", "第2步 插孔对齐深度", self.initial.get("pallet_depth"), "depth")
        corner_images = self.initial.get("corner_images") or {}
        for index in range(1, 7):
            point = f"P{index}"
            add_file(point, f"第3步 {point} 海康照片", corner_images.get(point))
        add_file("place_verify_rgb", "第4步 放货检测 RGB", self.initial.get("place_verify_rgb"))
        add_file("place_verify_depth", "第4步 放货检测深度（可选）", self.initial.get("place_verify_depth"), "depth")
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def _choose(self, line, kind):
        file_filter = "Depth (*.png *.tif *.tiff)" if kind == "depth" else "Images (*.jpg *.jpeg *.png *.bmp *.webp)"
        path, _ = QFileDialog.getOpenFileName(self, "选择离线图像", "", file_filter)
        if path:
            line.setText(path)

    def values(self):
        out = deepcopy(self.initial)
        for key in ("pallet_rgb", "pallet_depth", "place_verify_rgb", "place_verify_depth"):
            out[key] = self.lines[key].text().strip()
        out["corner_images"] = {f"P{i}": self.lines[f"P{i}"].text().strip() for i in range(1, 7)}
        images = out.setdefault("images", {})
        depths = out.setdefault("depths", {})
        images["pallet_hole"] = out["pallet_rgb"]
        depths["pallet_hole"] = out["pallet_depth"]
        for tag in ("field_place_verify", "field_pre_place_monitor"):
            images[tag] = out["place_verify_rgb"]
            depths[tag] = out["place_verify_depth"]
        return out
