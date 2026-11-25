# -*- coding: utf-8 -*-
from __future__ import annotations

import copy
import os
import unittest
import tempfile
import time
from pathlib import Path
from unittest.mock import create_autospec, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QT_QUICK_BACKEND", "software")
os.environ.setdefault("QSG_RHI_BACKEND", "software")

from PySide6.QtWidgets import QApplication

from ui.main_window import MainWindow
from ui.main_window_full import MainWindow as FullMainWindow
from services.vehicle_database_service import VehicleDatabaseService


class BoardDataMainWindowIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        # UI integration must not contact the user's production database.
        database = create_autospec(VehicleDatabaseService, instance=True)
        database.backend = 'test'
        database.location = 'test-only'
        database.start_session.return_value = 'TEST-SESSION'
        factory = patch('controllers.flow_controller.create_vehicle_database_service', return_value=database)
        factory.start()
        self.addCleanup(factory.stop)

    def _exercise_window(self, window_class):
        window = window_class()
        # Offline file checks are outside these UI tests; avoid a delayed modal
        # callback firing after its owner window has already been deleted.
        window._check_offline_paths = lambda **kwargs: False
        try:
            labels = [window.center_tab_bar.tabText(index) for index in range(window.center_tab_bar.count())]
            self.assertEqual(labels[:4], ["数字孪生", "实时雷达", "本流程相机", "板面数据"])
            if window_class.__module__.endswith("ui.main_window"):
                self.assertEqual(labels[4:], ["角点作业", "托盘循环"])
                self.assertEqual(window.center_overlay.count(), 5)
                self.assertIs(window.center_overlay.widget(3), window.corner_workspace)
                self.assertIs(window.center_overlay.widget(4), window.pallet_cycle_workspace)
            else:
                self.assertEqual(window.center_overlay.count(), 3)
            self.assertIs(window.center_overlay.widget(2), window.board_data_panel)

            window.center_tab_bar.setCurrentIndex(3)
            self.app.processEvents()
            self.assertEqual(window.center_overlay.currentIndex(), 2)

            snapshot = copy.deepcopy(window.controller.snapshot())
            snapshot["round_data"] = {
                "radar_result": {"boards": [{
                    "length_mm": 6123.456,
                    "width_mm": 2400,
                    "height_mean_mm": 1300,
                    "tilt_angle_deg": 1.5,
                    "offset_angle_deg": -0.25,
                }]}
            }
            window._refresh(snapshot)
            self.assertEqual(window.board_data_panel.board_tables[0].item(0, 1).text(), "6123 mm")
            self.assertEqual(window.board_data_panel.board_tables[0].item(1, 1).text(), "—")
        finally:
            window.close()
            window.deleteLater()
            self.app.processEvents()

    def test_primary_main_window_integrates_board_data_page(self):
        self._exercise_window(MainWindow)

    def test_full_main_window_integrates_board_data_page(self):
        self._exercise_window(FullMainWindow)

    def test_primary_window_opens_and_automatically_reads_live_coordinates(self):
        from ui.plc_coordinate_dialog import PlcCoordinateDialog
        from services.plc_coordinate_reader import default_config, save_config
        folder = tempfile.TemporaryDirectory()
        path = Path(folder.name) / 'plc.json'
        cfg = default_config()
        for i, axis in enumerate(cfg['axes'].values()):
            axis.update(db=18, offset=8 + i * 4, type='DINT')
        save_config(path, cfg)

        class LocalPLC:
            def connect(self, *args, **kwargs):
                pass
            def db_read(self, db, start, size):
                return bytes(size)
            def disconnect(self):
                pass

        window = MainWindow()
        window._check_offline_paths = lambda **kwargs: False
        try:
            with patch('ui.main_window.PlcCoordinateDialog',
                       side_effect=lambda parent: PlcCoordinateDialog(parent, config_path=path, client_factory=LocalPLC)):
                window.plc_live_btn.click()
            self.app.processEvents()
            self.assertTrue(window.plc_coordinates_dialog.isVisible())
            self.assertTrue(window.plc_coordinates_dialog.is_running())
            self.assertEqual(window.plc_coordinates_dialog.ip_input.text(), '10.11.207.100')
            window.plc_live_btn.click()
            self.app.processEvents()
            self.assertTrue(window.plc_coordinates_dialog.is_running())
        finally:
            if window.plc_coordinates_dialog is not None:
                window.plc_coordinates_dialog.stop_reading()
                deadline = time.monotonic() + 3
                while window.plc_coordinates_dialog.is_running() and time.monotonic() < deadline:
                    self.app.processEvents()
                    time.sleep(.01)
            window.close()
            window.deleteLater()
            self.app.processEvents()
            folder.cleanup()


class FieldMainUiTests(unittest.TestCase):
    """主界面现场化：正式现场流程默认收起、实验室条隐藏、中间默认数字孪生。"""

    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        database = create_autospec(VehicleDatabaseService, instance=True)
        database.backend = 'test'
        database.location = 'test-only'
        database.start_session.return_value = 'TEST-SESSION'
        factory = patch('controllers.flow_controller.create_vehicle_database_service', return_value=database)
        factory.start()
        self.addCleanup(factory.stop)

    def test_field_ui_uses_collapsed_workflow_and_twin_by_default(self):
        from config import feature_switches
        with patch.object(feature_switches, "RUN_PROFILE", "field"):
            window = MainWindow()
        window._check_offline_paths = lambda **kwargs: False
        try:
            self.assertFalse(window.field_workflow_card.isHidden())
            # 远程架构已彻底移除实验室流程条；若保留则必须是隐藏的
            if hasattr(window, "lab_flow_card"):
                self.assertTrue(window.lab_flow_card.isHidden())
            # 折叠条默认不展开
            self.assertFalse(window.field_flow_toggle.isChecked())
            # 顶栏是现场作业，不再出现"第1关 / 仅雷达算法测试"
            self.assertIn("现场作业", window.phase.text())
            self.assertNotIn("雷达算法测试", window.phase.text())
            # 中间图形视图默认数字孪生
            self.assertEqual(window.center_tab_bar.currentIndex(), 0)
            self.assertTrue(window.center_overlay.isHidden())
            # 顶部现场流程条覆盖两条线
            labels = [window.FIELD_FLOW_STAGES[i][1] for i in range(8)]
            self.assertIn("雷达现场扫描", labels)
            self.assertIn("海康按坐标拍照 / 角点精定位", labels)
            self.assertIn("叉车插取放货", labels)
            self.assertIn("放后再拍 / 动态监测", labels)
            self.assertEqual(len(window.field_flow_nodes), len(labels))
            # 实验室底栏按钮收起
            self.assertTrue(window.select_pcd_btn.isHidden())
            self.assertTrue(window.offline_check_btn.isHidden())
            self.assertTrue(window.next_btn.isHidden())
        finally:
            window.close()
            window.deleteLater()


if __name__ == "__main__":
    unittest.main()
