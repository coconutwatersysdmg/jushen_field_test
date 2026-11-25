# -*- coding: utf-8 -*-
"""工作区 B：每车重复十余次。拍托盘 → 插孔识别 → 叉车插取放货 → 放后再拍 → 动态监测。

轮次从装载计划 quantity 展开；算法位只载入本地结果 JSON 做联调，不调用算法。
"""
from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QFileDialog, QGridLayout, QHeaderView, QLabel, QMessageBox,
    QPushButton, QSplitter, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from services.field_workline_service import FieldWorklineService
from ui.field_workspace_widgets import AlgoSlot, MediaGallery, PathBar, StepStrip


class FieldPalletCycleWorkspace(QWidget):
    STEPS = [
        ("PALLET_CAPTURE", "拍托盘照"),
        ("HOLE_ALGO", "插孔识别（空壳）"),
        ("FORK_PLACE", "叉车插取并放货"),
        ("POST_CAPTURE", "放后再拍"),
        ("MONITOR_ALGO", "动态监测（空壳）"),
        ("ROUND_DONE", "本轮完成"),
    ]

    def __init__(self, service: FieldWorklineService, parent=None):
        super().__init__(parent)
        self.service = service
        self.plan: dict = {"path": "", "items": [], "queue": [], "total_rounds": 0}
        self.round_index = 0
        self.step_index = 0
        self.finished = False
        self.images_pre: list[dict] = []
        self.images_post: list[dict] = []
        self.hole_result: dict = {}
        self.monitor_result: dict = {}
        self._load_plan_silent()
        self._build()
        self._refresh()

    def _build(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 6, 8, 6)
        root.setSpacing(6)

        title = QLabel("托盘循环 · 每车十余次")
        title.setStyleSheet("font-size:15px;font-weight:500;color:#0f6e56;")
        subtitle = QLabel(
            "拍托盘识别插孔 → 叉车臂插取并放上车 → 放后再拍一张判断状态；"
            "轮次由装载计划 quantity 决定"
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

        self.info_table = QTableWidget(0, 2)
        self.info_table.setHorizontalHeaderLabels(["字段", "值"])
        self.info_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.info_table.verticalHeader().setVisible(False)
        self.info_table.setAlternatingRowColors(True)
        left_l.addWidget(self.info_table, 2)

        self.hole_slot = AlgoSlot("托盘插孔识别 · 算法空壳", "pallet_hole")
        self.hole_slot.set_example_path(self.service.example_result_path("pallet_hole"))
        self.hole_slot.resultLoaded.connect(self._on_hole_result)
        self.monitor_slot = AlgoSlot("放后动态监测 · 算法空壳", "dynamic_post")
        self.monitor_slot.set_example_path(self.service.example_result_path("dynamic_post"))
        self.monitor_slot.resultLoaded.connect(self._on_monitor_result)
        left_l.addWidget(self.hole_slot, 2)
        left_l.addWidget(self.monitor_slot, 2)

        actions = QWidget()
        grid = QGridLayout(actions)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(5)
        self.next_btn = QPushButton("执行下一步")
        self.next_btn.setObjectName("primary")
        self.next_btn.clicked.connect(self.next_step)
        self.load_plan_btn = QPushButton("载入装载计划…")
        self.load_plan_btn.clicked.connect(self.choose_plan)
        self.choose_photos_btn = QPushButton("选择本步照片…")
        self.choose_photos_btn.clicked.connect(self.choose_photos)
        self.example_photos_btn = QPushButton("用示例照片")
        self.example_photos_btn.clicked.connect(self.use_example_photos)
        self.result_json_btn = QPushButton("载入本步结果 JSON…")
        self.result_json_btn.clicked.connect(self.choose_step_result)
        self.reset_btn = QPushButton("重置本车")
        self.reset_btn.clicked.connect(self.reset_all)
        for index, button in enumerate((
            self.next_btn, self.load_plan_btn, self.choose_photos_btn,
            self.example_photos_btn, self.result_json_btn, self.reset_btn,
        )):
            grid.addWidget(button, index // 2, index % 2)
        left_l.addWidget(actions, 0)

        self.gallery = MediaGallery("本轮还没有照片。放货前后各拍一次，结果图也会显示在这里")
        splitter.addWidget(self.gallery)
        splitter.setSizes([500, 700])

        self.path_bar = PathBar()
        self.path_bar.set_vehicle_id(self.service.vehicle_id)
        self.path_bar.vehicleChanged.connect(self._on_vehicle_changed)
        root.addWidget(self.path_bar, 0)

    # ------------------------------------------------------------------
    def _on_vehicle_changed(self, text: str) -> None:
        self.service.vehicle_id = str(text).strip() or "FIELD-VEHICLE-001"
        self.path_bar.set_vehicle_id(self.service.vehicle_id)
        self._refresh()

    def _load_plan_silent(self) -> None:
        try:
            self.plan = self.service.load_plan()
        except Exception as exc:
            self.plan = {"path": "", "items": [], "queue": [], "total_rounds": 0,
                         "error": str(exc)}

    def choose_plan(self) -> None:
        filename, _ = QFileDialog.getOpenFileName(
            self, "选择装载计划 JSON", "", "JSON 文件 (*.json)"
        )
        if not filename:
            return
        try:
            self.plan = self.service.load_plan(filename)
        except Exception as exc:
            QMessageBox.warning(self, "装载计划载入失败", str(exc))
            return
        self.reset_all()
        QMessageBox.information(
            self, "装载计划已载入",
            f"共 {self.plan['total_rounds']} 轮（按每条货物的 quantity 展开）。",
        )

    def _current_cargo(self) -> dict:
        queue = self.plan.get("queue") or []
        if 0 <= self.round_index < len(queue):
            return queue[self.round_index]
        return {}

    def _reset_round(self) -> None:
        self.images_pre = []
        self.images_post = []
        self.hole_result = {}
        self.monitor_result = {}
        self.hole_slot.clear()
        self.monitor_slot.clear()
        self.gallery.set_images([])
        self.step_index = 0
        self.finished = False

    def reset_all(self) -> None:
        self.round_index = 0
        self._reset_round()
        self._refresh()

    def _ensure_session(self) -> str:
        if self.service.session_dir is None:
            info = self.service.start_session("pallet_cycle")
            self.path_bar.set_path(info["session_dir"])
        return self.service.open_session_folder()

    def _tag_prefix(self) -> str:
        number = self.round_index + 1
        if self.step_index == 0:
            return f"R{number:02d}_PALLET_"
        if self.step_index == 3:
            return f"R{number:02d}_POST_"
        return f"R{number:02d}_"

    def _add_images(self, paths: list[str]) -> None:
        if not paths:
            return
        if self.step_index not in (0, 3):
            QMessageBox.information(self, "当前步骤不拍照", "只有「拍托盘照」和「放后再拍」两步需要照片。")
            return
        self._ensure_session()
        prefix = self._tag_prefix()
        entries: list[dict] = []
        for index, path in enumerate(paths, start=1):
            try:
                archived = self.service.archive_image("pallet_cycle", path, f"{prefix}{index}")
            except Exception as exc:
                QMessageBox.warning(self, "归档失败", str(exc))
                continue
            entries.append({"label": ("托盘 " if self.step_index == 0 else "放后 ") + Path(archived).name,
                            "path": archived})
        if not entries:
            return
        target = self.images_pre if self.step_index == 0 else self.images_post
        target.extend(entries)
        self.gallery.set_images(self.images_pre + self.images_post)
        self.service.record_step(
            "pallet_cycle", "CAPTURE", "拍托盘照" if self.step_index == 0 else "放后再拍",
            "DONE", entries,
        )

    def choose_photos(self) -> None:
        filenames, _ = QFileDialog.getOpenFileNames(
            self, "选择照片（可多选）", "", "图片 (*.jpg *.jpeg *.png *.bmp)"
        )
        if filenames:
            self._add_images(filenames)

    def use_example_photos(self) -> None:
        kind = "pallet" if self.step_index == 0 else ("post_place" if self.step_index == 3 else "")
        if not kind:
            QMessageBox.information(self, "当前步骤不拍照", "只有「拍托盘照」和「放后再拍」两步需要照片。")
            return
        self._add_images(self.service.example_images(kind))

    def choose_step_result(self) -> None:
        if self.step_index == 1:
            self.hole_slot.choose_result()
        elif self.step_index == 4:
            self.monitor_slot.choose_result()
        else:
            QMessageBox.information(
                self, "当前步骤不需要结果 JSON",
                "请在「插孔识别」或「动态监测」这一步载入算法结果 JSON。",
            )

    def _on_hole_result(self, payload: dict) -> None:
        self.hole_result = dict(payload.get("data") or {})
        self._ensure_session()
        self.service.export_result("pallet_cycle", "pallet_hole_result", {
            "source_json": payload.get("path", ""),
            "data": self.hole_result,
            "note": "算法空壳：用户手动载入的联调结果",
        })
        self._refresh()

    def _on_monitor_result(self, payload: dict) -> None:
        self.monitor_result = dict(payload.get("data") or {})
        self._ensure_session()
        self.service.export_result("pallet_cycle", "dynamic_monitor_result", {
            "source_json": payload.get("path", ""),
            "data": self.monitor_result,
            "note": "算法空壳：用户手动载入的联调结果",
        })
        self._refresh()

    def next_step(self) -> None:
        if not self.plan.get("queue"):
            QMessageBox.warning(self, "没有装载计划", "请先载入一份装载计划 JSON。")
            return
        if self.finished:
            QMessageBox.information(self, "本车已完成", "点「重置本车」可以重新开始。")
            return
        code = self.STEPS[self.step_index][0]
        cargo = self._current_cargo()

        if code == "PALLET_CAPTURE":
            if not self.images_pre:
                self._add_images(self.service.example_images("pallet"))
            if not self.images_pre:
                QMessageBox.warning(self, "没有照片", "请先选一张托盘照片（或用示例照片）。")
                return
        elif code == "HOLE_ALGO":
            if not self.hole_result:
                QMessageBox.information(self, "还没有插孔结果", "请点「载入本步结果 JSON」或算法槽里的「用示例结果」。")
                return
            self._ensure_session()
            self.service.record_step("pallet_cycle", "HOLE_ALGO", "插孔识别（空壳）", "LOADED")
        elif code == "FORK_PLACE":
            self._ensure_session()
            self.service.record_step("pallet_cycle", "FORK_PLACE", "叉车插取并放货", "DONE", {
                "instance_id": cargo.get("instance_id", ""),
                "mode": "MANUAL_CONFIRM",
            })
        elif code == "POST_CAPTURE":
            if not self.images_post:
                self._add_images(self.service.example_images("post_place"))
            if not self.images_post:
                QMessageBox.warning(self, "没有照片", "请先选一张放后照片（或用示例照片）。")
                return
        elif code == "MONITOR_ALGO":
            if not self.monitor_result:
                QMessageBox.information(self, "还没有监测结果", "请点「载入本步结果 JSON」或算法槽里的「用示例结果」。")
                return
            self._ensure_session()
            self.service.record_step("pallet_cycle", "MONITOR_ALGO", "动态监测（空壳）", "LOADED")
        elif code == "ROUND_DONE":
            self._finish_round()
            return

        self.step_index = min(self.step_index + 1, len(self.STEPS) - 1)
        self._refresh()

    def _finish_round(self) -> None:
        cargo = self._current_cargo()
        payload = {
            "schema_version": "field-workline/1.0",
            "round": self.round_index + 1,
            "total_rounds": self.plan.get("total_rounds", 0),
            "cargo": cargo,
            "pallet_images": self.images_pre,
            "pallet_hole_result": self.hole_result,
            "post_place_images": self.images_post,
            "dynamic_monitor_result": self.monitor_result,
            "note": "第一阶段联调产物；算法结果与 PLC 动作尚未真实接入",
        }
        self._ensure_session()
        path = self.service.export_result("pallet_cycle", f"round_{self.round_index + 1:02d}_summary", payload)
        self.service.record_step("pallet_cycle", "ROUND_DONE", "本轮完成", "DONE", {"path": path})

        self.round_index += 1
        if self.round_index >= self.plan.get("total_rounds", 0):
            self.finished = True
            self.step_strip.set_current(len(self.STEPS) - 1, "DONE")
            QMessageBox.information(
                self, "本车全部完成",
                f"共 {self.plan['total_rounds']} 轮已跑完。\n归档目录：{self.service.open_session_folder()}",
            )
        else:
            self._reset_round()
        self._refresh()

    def _refresh(self) -> None:
        queue = self.plan.get("queue") or []
        cargo = self._current_cargo()
        rows = [
            ("装载计划", self.plan.get("path") or "未载入"),
            ("总轮次", str(self.plan.get("total_rounds", 0))),
            ("当前轮次", f"{self.round_index + 1} / {self.plan.get('total_rounds', 0)}" if queue else "-"),
            ("当前货物", cargo.get("instance_id") or cargo.get("cargo_name") or "-"),
            ("托盘照片", f"{len(self.images_pre)} 张"),
            ("插孔结果", "已载入" if self.hole_result else "待接入 / 未载入"),
            ("放后照片", f"{len(self.images_post)} 张"),
            ("动态监测", "已载入" if self.monitor_result else "待接入 / 未载入"),
            ("状态", "本车已完成" if self.finished else f"第 {self.step_index + 1} 步"),
        ]
        if self.plan.get("error"):
            rows.insert(1, ("计划错误", str(self.plan["error"])))
        self.info_table.setRowCount(len(rows))
        for row, (key, value) in enumerate(rows):
            self.info_table.setItem(row, 0, QTableWidgetItem(str(key)))
            self.info_table.setItem(row, 1, QTableWidgetItem(str(value)))
        self.step_strip.set_current(
            self.step_index if not self.finished else len(self.STEPS) - 1,
            "DONE" if self.finished else "RUN",
        )
        self.next_btn.setText("完成本轮" if self.step_index == len(self.STEPS) - 1 else "执行下一步")
        self.gallery.set_images(self.images_pre + self.images_post)
        self.path_bar.set_path(self.service.open_session_folder())
