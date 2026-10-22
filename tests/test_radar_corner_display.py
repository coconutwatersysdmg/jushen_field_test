from copy import deepcopy
import os
from types import SimpleNamespace
import unittest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PySide6.QtWidgets import QApplication, QComboBox, QTableWidget

from services.sensor_calibration_service import SensorCalibrationService
from ui.board_data_panel import BoardDataPanel
from ui.main_window import MainWindow
from ui.main_window_full import MainWindow as FullMainWindow


class RadarCornerDisplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def _tables(self, window_class, round_data):
        calibration = SensorCalibrationService()
        panel = BoardDataPanel(lidar_world_matrix=calibration.fixed_world_matrix('lidar'))
        # Exercise the real table renderer without starting unrelated QML, PLC or database UI.
        holder = SimpleNamespace(
            controller=SimpleNamespace(calibration=calibration),
            field_camera_history=QComboBox(), field_hole_table=QTableWidget(0, 4),
            field_radar_table=QTableWidget(0, 5),
            _first_image=window_class._first_image,
            _render_field_camera_entry=lambda *_: None,
        )
        panel.update_from_round_data(round_data)
        window_class._refresh_field_sense_panels(holder, {}, round_data, {}, {})
        return panel, holder

    def _assert_world_rows(self, expected, panel, holder):
        for table in (panel.corner_table, holder.field_radar_table):
            self.assertEqual(table.rowCount(), len(expected))
            for row, xyz in enumerate(expected):
                self.assertEqual(table.item(row, 0).text(), f'P{row + 1}')
                for column, value in enumerate(xyz, 1):
                    self.assertAlmostEqual(float(table.item(row, column).text()), value, delta=0.0005)

    def test_both_tables_keep_eight_world_radar_corners_over_camera_search_targets(self):
        expected = [[13939 + i, 7715 + i, 100 + i] for i in range(8)]
        world = {f'P{i + 1}': dict(zip(('x', 'y', 'z'), xyz)) for i, xyz in enumerate(expected)}
        data = {
            'radar_result': {'coordinate_frame': 'world', 'corner_ids': list(world), 'world_points': world},
            'field_corner_shell': {'final_world_corners': {f'P{i}': {'x': 999, 'y': 999, 'z': 999} for i in range(1, 7)}},
        }
        original = deepcopy(data)
        for window_class in (MainWindow, FullMainWindow):
            with self.subTest(window=window_class.__module__):
                self._assert_world_rows(expected, *self._tables(window_class, data))
        self.assertEqual(data, original)

    def test_native_radar_coordinates_are_calibrated_in_both_tables(self):
        points = [[0, 0, 100], [1000, 0, 100], [0, 2000, 200], [1000, 2000, 200]]
        data = {'radar_result': {'coordinate_frame': 'radar',
                                'world_points': {f'P{i + 1}': dict(zip(('x', 'y', 'z'), xyz)) for i, xyz in enumerate(points)}}}
        expected = [[13939, 7715, 100], [13927, 6715, 100],
                    [15939, 7691, 200], [15927, 6691, 200]]
        for window_class in (MainWindow, FullMainWindow):
            with self.subTest(window=window_class.__module__):
                self._assert_world_rows(expected, *self._tables(window_class, data))

    def test_no_radar_result_does_not_present_camera_targets_as_radar_corners(self):
        data = {'field_corner_shell': {'final_world_corners': {'P1': {'x': 999, 'y': 999, 'z': 999}}}}
        for window_class in (MainWindow, FullMainWindow):
            with self.subTest(window=window_class.__module__):
                self._assert_world_rows([], *self._tables(window_class, data))


if __name__ == '__main__':
    unittest.main()
