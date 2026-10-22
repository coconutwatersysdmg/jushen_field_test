from copy import deepcopy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import create_autospec, patch

import numpy as np

from algorithm_modules.point_cloud_segment_module import point_cloud_pipeline as pipeline
from controllers.flow_controller import FlowController
from services.board_data_service import build_corner_comparison
from services.point_cloud_processing_service import PointCloudProcessingService
from services.sensor_calibration_service import SensorCalibrationService
from services.vehicle_database_service import VehicleDatabaseService


class RadarWorldCalibrationTests(unittest.TestCase):
    RADAR_CORNERS = [[0, 0, 100], [1000, 0, 100], [0, 2000, 200], [1000, 2000, 200]]
    WORLD_CORNERS = [
        [13939, 7715, 100],
        [13927, 6715, 100],
        [15939, 7691, 200],
        [15927, 6691, 200],
    ]

    def test_roi_includes_new_radar_volume_and_excludes_points_beyond_it(self):
        config = pipeline.PipelineConfig()
        low, high = pipeline.calculate_roi_bounds(config.roi_min, config.roi_max, config.roi_redundancy)
        points = np.array([
            [300, -11000, -200], [9000, 14000, 3500], [500, 0, 3300],
            [-1000, 0, 0], [10000, 0, 0], [1000, -13000, 0],
            [1000, 16000, 0], [1000, 0, -1000], [1000, 0, 5000],
        ])
        kept = np.all((points >= low) & (points <= high), axis=1)
        self.assertEqual(kept.tolist(), [True, True, True, False, False, False, False, False, False])

    def test_calibration_preserves_z_and_input_instead_of_using_old_extrinsics(self):
        source = np.array([[0, 0, 10], [1000, 0, 20], [0, 1000, 30]], dtype=float)
        original = source.copy()
        actual = pipeline.radar_to_world(source)
        np.testing.assert_allclose(actual, [
            [13939, 7715, 10],
            [13926.78299916475, 6715.07463033955, 20],
            [14938.92536966045, 7702.78299916475, 30],
        ], atol=1e-8, rtol=0)
        np.testing.assert_array_equal(source, original)

    def test_geometry_is_reported_in_world_after_corner_extraction(self):
        board = pipeline.compute_geometry_result(self.RADAR_CORNERS)
        self.assertEqual(board['coordinate_frame'], 'world')
        np.testing.assert_allclose(board['corners_xyz_mm'], self.WORLD_CORNERS, atol=1e-8)
        self.assertEqual(board['length_mm'], 2002)
        self.assertEqual(board['width_mm'], 1000)
        self.assertEqual(board['offset_angle_deg'], 89)

    def test_four_and_eight_world_corners_survive_service_controller_and_display(self):
        controller = FlowController.__new__(FlowController)
        controller.calibration = SensorCalibrationService()
        for board_count in (1, 2):
            with self.subTest(board_count=board_count):
                raw = {'label_2': pipeline.compute_geometry_result(self.RADAR_CORNERS)}
                expected = list(self.WORLD_CORNERS)
                if board_count == 2:
                    elevated = np.asarray(self.RADAR_CORNERS) + [0, 0, 500]
                    raw['label_3'] = pipeline.compute_geometry_result(elevated, corner_start=5)
                    expected += (np.asarray(self.WORLD_CORNERS) + [0, 0, 500]).tolist()
                original = deepcopy(raw)
                output = PointCloudProcessingService.normalize_result(raw)
                self.assertEqual(output['coordinate_frame'], 'world')
                np.testing.assert_allclose(output['corner_points_xyz_mm'], expected, atol=1e-8)
                self.assertEqual(raw, original)
                normalized = controller._normalize_radar_result(output)
                self.assertEqual(normalized['corner_ids'], [f'P{i}' for i in range(1, 4 * board_count + 1)])
                for data in (output, normalized):
                    # Supply a visibly different matrix to catch accidental double conversion.
                    rows = build_corner_comparison({'radar_result': data}, np.diag([2, 2, 2, 1]))
                    actual = [[row['radar'][axis] for axis in ('x', 'y', 'z')] for row in rows]
                    np.testing.assert_allclose(actual, expected, atol=1e-8)

    def test_fixed_sensor_calibration_matches_the_pipeline_transform(self):
        calibration = SensorCalibrationService()
        np.testing.assert_allclose(calibration.lidar_point_to_world([1000, 0, 20]),
                                   [13926.78299916475, 6715.07463033955, 20], atol=1e-8, rtol=0)

    def test_camera_search_can_use_six_targets_without_losing_eight_radar_corners(self):
        database = create_autospec(VehicleDatabaseService, instance=True)
        database.backend, database.location = 'test', 'test-only'
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            with patch('config.feature_switches.RADAR_ONLY_TEST', False), \
                 patch('controllers.flow_controller.create_vehicle_database_service', return_value=database), \
                 patch('controllers.flow_controller.STATE_FILE', root / 'state.json'), \
                 patch('controllers.flow_controller.RESULT_FILE', root / 'results.json'):
                controller = FlowController()
                controller.queue = [{'instance_id': 'test'}]
                raw = {
                    'label_2': pipeline.compute_geometry_result(self.RADAR_CORNERS),
                    'label_3': pipeline.compute_geometry_result(
                        np.asarray(self.RADAR_CORNERS) + [0, 2000, 500], corner_start=5),
                }
                radar = controller._normalize_radar_result(PointCloudProcessingService.normalize_result(raw))
                controller.round_data['radar_result'] = radar
                original = deepcopy(radar)
                image = root / 'corners.jpg'
                image.write_bytes(b'offline image fixture')
                controller.set_debug_inputs({'corner_images': {f'P{i}': str(image) for i in range(1, 7)}})
                # YOLO is external to this test; camera target generation and planning stay real.
                with patch.object(controller.algorithms, 'corner_image_recognize', return_value={'success': True}):
                    result = controller._field_corner_shell_from_radar()
                self.assertEqual(list(result['final_world_corners']), [f'P{i}' for i in range(1, 7)])
                self.assertEqual(controller.round_data['radar_result'], original)
                self.assertEqual(len(controller.round_data['radar_result']['world_points']), 8)


if __name__ == '__main__':
    unittest.main()
