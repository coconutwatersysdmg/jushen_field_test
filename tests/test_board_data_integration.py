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
            self.assertEqual(labels, ["数字孪生", "实时雷达", "本流程相机", "板面数据"])
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
            self.assertEqual(window.board_data_panel.board_tables[0].item(0, 1).text(), "6123.456 mm")
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


if __name__ == "__main__":
    unittest.main()
