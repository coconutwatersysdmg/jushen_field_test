# -*- coding: utf-8 -*-
"""工作区 A：每车一次。雷达粗识别 P1~P4 → 算坐标 → 导出 PLC 拍照指令 → 海康拍照 → 角点识别。"""
from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QFileDialog, QGridLayout, QHBoxLayout, QHeaderView, QLabel, QMessageBox,
    QPushButton, QSplitter, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from services.field_workline_service import FieldWorklineService
from ui.field_workspace_widgets import AlgoSlot, MediaGallery, PathBar, StepStrip


class FieldCornerWorkspace(QWidget):
    STEPS = [
        ("RADAR_CORNERS", "取雷达四角点 P1~P4"),
        ("EXPORT_CMD", "导出拍照指令 JSON"),
        ("CAPTURE", "海康按坐标拍照"),
        ("CORNER_ALGO", "角点识别（空壳）"),
        ("SUMMARY", "导出本车结果"),
    ]

    def __init__(self, service: FieldWorklineService, controller=None, parent=None):
        super().__init__(parent)
        self.service = service
        self.controller = controller
        self.radar_result: dict = {}
        self.points: list[dict] = []
        self._points_overridden = False
        self.images: list[dict] = []
        self.command_path = ""
        self.corner_result: dict = {}
        self.result_path = ""
        self._build()
        self._refresh_table()

    def _build(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 6, 8, 6)
        root.setSpacing(6)

        title = QLabel("角点精定位 · 每车一次")
        title.setStyleSheet("font-size:15px;font-weight:500;color:#1565c0;")
        subtitle = QLabel(
            "雷达粗识别 P1~P4 → 软件算 WORLD 坐标 → 导出给 PLC → PLC 带相机去点位拍照 → 角点识别"
        )
        subtitle.setStyleSheet("font-size:12px;color:#5c6675;")
        root.addWidget(title, 0)
        root.addWidget(subtitle, 0)

        self.step_strip = StepStrip(self.STEPS)
        root.addWidget(self.step_strip, 0)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setChildrenCollapsible(False)
        root.addWidget(splitter, 1)

        left = QWidget()
        left_l = QVBoxLayout(left)
        left_l.setContentsMargins(0, 0, 0, 0)
        left_l.setSpacing(6)
        splitter.addWidget(left)

        data_label = QLabel("本车数据")
        data_label.setStyleSheet("font-size:13px;font-weight:500;color:#1565c0;")
        left_l.addWidget(data_label, 0)
        self.data_table = QTableWidget(0, 2)
        self.data_table.setHorizontalHeaderLabels(["字段", "值"])
        self.data_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.data_table.verticalHeader().setVisible(False)
        self.data_table.setAlternatingRowColors(True)
        left_l.addWidget(self.data_table, 2)

        self.algo_slot = AlgoSlot("角点识别 · 算法空壳", "corner")
        self.algo_slot.set_example_path(self.service.example_result_path("corner"))
        self.algo_slot.resultLoaded.connect(self._on_corner_result)
        left_l.addWidget(self.algo_slot, 2)

        actions = QWidget()
        grid = QGridLayout(actions)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(5)
        self.radar_points_btn = QPushButton("从雷达结果取四角点")
        self.radar_points_btn.clicked.connect(self.take_radar_points)
        self.demo_points_btn = QPushButton("用演示坐标")
        self.demo_points_btn.setToolTip("接口未到位时联调用；导出的 JSON 会标注 DEMO_PLACEHOLDER")
        self.demo_points_btn.clicked.connect(self.use_demo_points)
        self.export_cmd_btn = QPushButton("导出拍照指令 JSON")
        self.export_cmd_btn.clicked.connect(self.export_scan_command)
        self.choose_images_btn = QPushButton("选角点照片…")
        self.choose_images_btn.clicked.connect(self.choose_images)
        self.example_images_btn = QPushButton("用示例照片")
        self.example_images_btn.clicked.connect(self.use_example_images)
        self.export_summary_btn = QPushButton("导出本车结果")
        self.export_summary_btn.clicked.connect(self.export_summary)
        for index, button in enumerate((
            self.radar_points_btn, self.demo_points_btn, self.export_cmd_btn,
            self.choose_images_btn, self.example_images_btn, self.export_summary_btn,
        )):
            grid.addWidget(button, index // 2, index % 2)
        left_l.addWidget(actions, 0)

        self.gallery = MediaGallery("还没有角点照片。点「用示例照片」或「选角点照片…」")
        splitter.addWidget(self.gallery)
        splitter.setSizes([440, 760])

        self.path_bar = PathBar()
        self.path_bar.set_vehicle_id(self.service.vehicle_id)
        self.path_bar.vehicleChanged.connect(self._on_vehicle_changed)
        root.addWidget(self.path_bar, 0)

    # ------------------------------------------------------------------
    def _on_vehicle_changed(self, text: str) -> None:
        self.service.vehicle_id = str(text).strip() or "FIELD-VEHICLE-001"
        self.path_bar.set_vehicle_id(self.service.vehicle_id)
        self._refresh_table()

    def update_from_snapshot(self, snapshot: dict | None = None) -> None:
        """从主流程拿雷达粗结果；用户手动选过演示坐标后不再覆盖。"""
        if snapshot is None and self.controller is not None:
            try:
                snapshot = self.controller.snapshot()
            except Exception:
                snapshot = None
        radar = (((snapshot or {}).get("round_data") or {}).get("radar_result") or {})
        if radar and not self._points_overridden:
            self.radar_result = dict(radar)
            points = self.service.extract_corners(self.radar_result)
            if len(points) == 4:
                self.points = points
        self._refresh_table()

    def take_radar_points(self) -> None:
        if not self.radar_result:
            self.update_from_snapshot()
        if not self.radar_result:
            QMessageBox.information(
                self, "还没有雷达结果",
                "请先在「实时雷达」页选 PCD 并跑一次雷达算法；\n联调阶段也可以点「用演示坐标」。",
            )
            return
        points = self.service.extract_corners(self.radar_result)
        if len(points) != 4:
            QMessageBox.warning(
                self, "角点不够",
                f"雷达结果里只找到 {len(points)} 个 P1~P4 角点，需要 4 个。\n"
                "请检查点云算法输出是否包含 world_points.P1~P4。",
            )
            return
        self.points = points
        self._points_overridden = False
        self.step_strip.set_current(0, "DONE")
        self._refresh_table()
        QMessageBox.information(self, "取到四角点", "已从雷达粗识别结果取出 P1~P4，坐标单位 mm。")

    def use_demo_points(self) -> None:
        self.points = self.service.demo_corners()
        self._points_overridden = True
        self.step_strip.set_current(0, "DONE")
        self._refresh_table()
        QMessageBox.information(
            self, "演示坐标已载入",
            "这四个点是 DEMO_PLACEHOLDER，只用于界面联调；\n导出的指令 JSON 里也会明确标注来源。",
        )

    def export_scan_command(self) -> None:
        if len(self.points) != 4:
            QMessageBox.warning(self, "还不能导出", "请先取雷达四角点，或用演示坐标。")
            return
        try:
            info = self.service.start_session("corner")
            self.path_bar.set_path(info["session_dir"])
            self.command_path = self.service.export_corner_scan_command(
                self.points, info["vehicle_id"]
            )
        except Exception as exc:
            QMessageBox.critical(self, "导出失败", str(exc))
            return
        self.step_strip.set_current(1, "DONE")
        self._refresh_table()
        QMessageBox.information(
            self, "指令已导出",
            f"拍照指令 JSON：\n{self.command_path}\n\n"
            "PLC 接口文件到位后，这里会改成直接下发；当前先导出 JSON 联调。",
        )

    def _add_images(self, paths: list[str], tag_prefix: str) -> None:
        if not paths:
            return
        if self.service.session_dir is None:
            info = self.service.start_session("corner")
            self.path_bar.set_path(info["session_dir"])
        entries = []
        for index, path in enumerate(paths, start=1):
            try:
                archived = self.service.archive_image("corner", path, f"{tag_prefix}{index}")
            except Exception as exc:
                QMessageBox.warning(self, "归档失败", str(exc))
                continue
            entries.append({"label": Path(archived).name, "path": archived})
        self.images.extend(entries)
        self.gallery.set_images(self.images)
        if entries:
            self.service.record_step("corner", "CAPTURE", "海康按坐标拍照", "DONE", entries)
            self.step_strip.set_current(2, "DONE")

    def choose_images(self) -> None:
        filenames, _ = QFileDialog.getOpenFileNames(
            self, "选择角点照片（可多选）", "", "图片 (*.jpg *.jpeg *.png *.bmp)"
        )
        if filenames:
            self._add_images(filenames, "CORNER_")

    def use_example_images(self) -> None:
        self._add_images(self.service.example_images("corner"), "CORNER_")

    def _on_corner_result(self, payload: dict) -> None:
        self.corner_result = dict(payload.get("data") or {})
        try:
            path = self.service.export_result("corner", "corner_recognition", {
                "source_json": payload.get("path", ""),
                "data": self.corner_result,
                "note": "算法空壳：用户手动载入的联调结果，未调用真实识别算法",
            })
            self.service.record_step("corner", "CORNER_ALGO", "角点识别（空壳）", "LOADED", {"path": path})
        except Exception as exc:
            QMessageBox.warning(self, "结果归档失败", str(exc))
        self.step_strip.set_current(3, "DONE")
        self._refresh_table()

    def export_summary(self) -> None:
        if not self.command_path:
            QMessageBox.warning(self, "还不能导出", "请先导出拍照指令 JSON。")
            return
        payload = {
            "schema_version": "field-workline/1.0",
            "vehicle_id": self.service.vehicle_id,
            "workline": "corner",
            "radar_source": (self.radar_result.get("message") or "") if self.radar_result else "",
            "corners": self.points,
            "corner_command_json": self.command_path,
            "captured_images": self.images,
            "corner_recognition": self.corner_result,
            "note": "第一阶段联调产物；PLC 下发与真实角点算法待接口文件到位后接入",
        }
        try:
            path = self.service.export_result("corner", "vehicle_summary", payload)
            self.service.record_step("corner", "SUMMARY", "导出本车结果", "DONE", {"path": path})
            self.result_path = path
        except Exception as exc:
            QMessageBox.critical(self, "导出失败", str(exc))
            return
        self.step_strip.set_current(4, "DONE")
        self._refresh_table()
        QMessageBox.information(self, "本车结果已导出", path)

    def _refresh_table(self) -> None:
        rows = [
            ("车号", self.service.vehicle_id),
            ("雷达来源", (self.radar_result.get("message") or "尚未取结果") if self.radar_result else "尚未取结果"),
            ("角点来源", self.points[0].get("source", "-") if self.points else "-"),
            ("拍照指令 JSON", self.command_path or "未导出"),
            ("角点识别", "已载入联调结果" if self.corner_result else "待接入 / 未载入"),
            ("本车结果 JSON", self.result_path or "未导出"),
        ]
        for point in self.points:
            rows.append((
                f"{point['point_id']} WORLD",
                f"X={point['x_mm']:.1f}, Y={point['y_mm']:.1f}, Z={point['z_mm']:.1f} mm",
            ))
        self.data_table.setRowCount(len(rows))
        for row, (key, value) in enumerate(rows):
            self.data_table.setItem(row, 0, QTableWidgetItem(str(key)))
            self.data_table.setItem(row, 1, QTableWidgetItem(str(value)))
