# -*- coding: utf-8 -*-
from __future__ import annotations

import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication, QLabel, QPushButton

from ui.board_data_panel import BoardDataPanel


class BoardDataPanelTests(unittest.TestCase):
    LIDAR_TO_FIXED_WORLD = [
        [1.0, 0.0, 0.0, 100.0],
        [0.0, 1.0, 0.0, 200.0],
        [0.0, 0.0, 1.0, 300.0],
        [0.0, 0.0, 0.0, 1.0],
    ]

    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.panel = BoardDataPanel(lidar_world_matrix=self.LIDAR_TO_FIXED_WORLD)
        self.panel.show()
        self.app.processEvents()

    def tearDown(self):
        for dialog in list(getattr(self.panel, "_formula_dialogs", [])):
            dialog.close()
        self.panel.close()
        self.panel.deleteLater()
        self.app.processEvents()

    def test_panel_contains_only_two_board_tables_and_required_source_rows(self):
        self.assertEqual(len(self.panel.board_tables), 2)
        self.assertEqual([label.text() for label in self.panel.board_title_labels], ["第一块板", "第二块板"])
        for table in self.panel.board_tables:
            self.assertEqual(table.rowCount(), 2)
            self.assertEqual(table.columnCount(), 6)
            self.assertEqual(table.item(0, 0).text(), "雷达车板数据")
            self.assertEqual(table.item(1, 0).text(), "相机装载区域数据")
            self.assertEqual(
                [table.horizontalHeaderItem(column).text() for column in range(6)],
                ["数据来源", "长度 ⓘ", "宽度 ⓘ", "平均高度 ⓘ", "纵向倾斜角 ⓘ", "水平偏移角 ⓘ"],
            )

        visible_text = "\n".join(
            [label.text() for label in self.panel.findChildren(QLabel)]
            + [button.text() for button in self.panel.findChildren(QPushButton)]
            + [
                table.item(row, column).text()
                for table in self.panel.board_tables
                for row in range(table.rowCount())
                for column in range(table.columnCount())
                if table.item(row, column)
            ]
        )
        for removed in ("用途", "允许划格", "粗定位参考", "最终划格", "查看第一块板角点坐标", "雷达只作参考"):
            self.assertNotIn(removed, visible_text)

    def test_update_formats_units_precision_and_missing_values(self):
        round_data = {
            "radar_result": {"boards": [{
                "length_mm": 6100.1254,
                "width_mm": 2380.5,
                "height_mean_mm": 1250.25,
                "tilt_angle_deg": 1.25,
                "offset_angle_deg": -2.5,
            }]},
            "camera_world_corners": {
                "P1": [0, 0, 100], "P2": [2000, 0, 100],
                "P3": [0, -4000, 300], "P4": [2000, -4000, 300],
            },
        }

        self.panel.update_from_round_data(round_data)

        first = self.panel.board_tables[0]
        second = self.panel.board_tables[1]
        self.assertEqual(first.item(0, 1).text(), "6100.125 mm")
        self.assertEqual(first.item(0, 2).text(), "2380.500 mm")
        self.assertEqual(first.item(0, 4).text(), "1.250000°")
        self.assertEqual(first.item(0, 5).text(), "-2.500000°")
        self.assertEqual(first.item(1, 1).text(), "4004.997 mm")
        self.assertEqual(first.item(1, 3).text(), "200.000 mm")
        self.assertEqual(first.item(1, 4).text(), "2.862405°")
        for row in range(2):
            for column in range(1, 6):
                self.assertEqual(second.item(row, column).text(), "—")

    def test_each_metric_header_opens_its_formula_dialog(self):
        expected = {
            1: ("长度计算说明", "L = [d(P1,P3) + d(P2,P4)] / 2", "毫米"),
            2: ("宽度计算说明", "W = [d(P1,P2) + d(P3,P4)] / 2", "毫米"),
            3: ("平均高度计算说明", "H = (z1 + z2 + z3 + z4) / 4", "毫米"),
            4: ("纵向倾斜角计算说明", "tilt = atan2(dz, sqrt(dx² + dy²)) × 180 / π", "车尾端高于车头端"),
            5: ("水平偏移角计算说明", "offset = atan2(dx, -dy) × 180 / π", "负 Y 方向为零度基准"),
        }

        header = self.panel.board_tables[0].horizontalHeader()
        for section, (title, formula, explanation) in expected.items():
            header.sectionClicked.emit(section)
            self.app.processEvents()
            dialog = self.panel.active_formula_dialog
            self.assertIsNotNone(dialog)
            self.assertTrue(dialog.isVisible())
            self.assertEqual(dialog.windowTitle(), title)
            dialog_text = "\n".join(label.text() for label in dialog.findChildren(QLabel))
            self.assertIn(formula, dialog_text)
            self.assertIn(explanation, dialog_text)
            self.assertIn("固定桁架 WORLD 坐标", dialog_text)
            dialog.close()

    def test_corner_table_compares_actual_radar_and_camera_points_in_fixed_world(self):
        round_data = {
            "radar_result": {
                "coordinate_frame": "radar_world",
                "world_points": {
                    "P1": [1, 2, 3],
                    "P2": [4, 5, 6],
                    "P3": [7, 8, 9],
                    "P4": [10, 11, 12],
                },
            },
            "camera_world_corners": {
                "P1": [101.5, 202.5, 303.5],
                "P2": [104.5, 205.5, 306.5],
                "P3": [107.5, 208.5, 309.5],
            },
        }

        self.panel.update_from_round_data(round_data)

        table = self.panel.corner_table
        self.assertEqual(table.rowCount(), 4)
        self.assertEqual(table.columnCount(), 7)
        self.assertEqual(
            [table.horizontalHeaderItem(column).text() for column in range(7)],
            ["角点", "雷达 X(mm)", "雷达 Y(mm)", "雷达 Z(mm)", "相机 X(mm)", "相机 Y(mm)", "相机 Z(mm)"],
        )
        self.assertEqual(table.item(0, 0).text(), "P1")
        self.assertEqual([table.item(0, column).text() for column in range(1, 4)], ["101.000", "202.000", "303.000"])
        self.assertEqual([table.item(0, column).text() for column in range(4, 7)], ["101.500", "202.500", "303.500"])
        self.assertEqual([table.item(3, column).text() for column in range(4, 7)], ["—", "—", "—"])

    def test_corner_table_has_no_reserved_empty_rows_and_caps_points_at_p8(self):
        camera = {f"P{index}": [index, index + 10, index + 20] for index in range(1, 10)}

        self.panel.update_from_round_data({"camera_world_corners": camera})

        self.assertEqual(self.panel.corner_table.rowCount(), 8)
        self.assertEqual(self.panel.corner_table.item(7, 0).text(), "P8")


if __name__ == "__main__":
    unittest.main()
