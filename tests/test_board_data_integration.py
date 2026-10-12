# -*- coding: utf-8 -*-
from __future__ import annotations

import copy
import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QT_QUICK_BACKEND", "software")
os.environ.setdefault("QSG_RHI_BACKEND", "software")

from PySide6.QtWidgets import QApplication

from ui.main_window import MainWindow
from ui.main_window_full import MainWindow as FullMainWindow


class BoardDataMainWindowIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def _exercise_window(self, window_class):
        window = window_class()
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


if __name__ == "__main__":
    unittest.main()
