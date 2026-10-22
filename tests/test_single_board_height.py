"""Height reference must use supported front strips, independently of corner Z."""
import copy
import unittest
from unittest.mock import patch

import numpy as np

from algorithm_modules.point_cloud_segment_module import point_cloud_pipeline as pipeline
from services.point_cloud_processing_service import PointCloudProcessingService
from services.board_data_service import build_board_data


def scene(count=81):
    u, v = np.meshgrid(np.linspace(0, 1, count), np.linspace(0, 1, 71))
    xyz = np.column_stack([12000 * u.ravel(), 2500 * v.ravel(),
                           (1400 + 200 * u + 20 * v).ravel()])
    corners = np.array([[0, 0, 1400], [0, 2500, 1420],
                        [12000, 0, 1600], [12000, 2500, 1620]], float)
    return xyz, corners


class SingleBoardHeightTests(unittest.TestCase):
    def estimate(self, xyz, corners):
        from algorithm_modules.point_cloud_segment_module.single_board_height import estimate_single_board_height
        return estimate_single_board_height(xyz, corners)

    def test_front_crop_centers_and_equal_strip_weight(self):
        xyz, corners = scene()
        # Densifying only the front strip must not turn the output into a point mean.
        dense = np.repeat(xyz[xyz[:, 0] < 1440], 8, axis=0)
        report = self.estimate(np.vstack([xyz, dense]), corners)
        self.assertEqual(report['status'], 'supported')
        self.assertAlmostEqual(report['height_mm'], 1469.5, places=6)
        self.assertEqual(len(report['strips']), 5)
        np.testing.assert_allclose([s['center_radar_xyz_mm'][0] for s in report['strips']],
                                   [720, 2160, 3600, 5040, 6480])
        self.assertTrue(all(s['status'] == 'supported' for s in report['strips']))

    def test_crop_and_residual_filter_reject_far_side_tail_and_floating_noise(self):
        xyz, corners = scene(121)
        bad = (xyz[:, 0] > 7200.01) | (xyz[:, 1] > 2375.01)
        xyz[bad, 2] += 2000
        rng = np.random.default_rng(20)
        noise = xyz[rng.choice(np.flatnonzero(~bad), 1200, replace=False)].copy()
        noise[:800, 2] += 200
        noise[800:, 2] -= 250
        report = self.estimate(np.vstack([xyz, noise]), corners)
        self.assertEqual(report['status'], 'supported')
        self.assertAlmostEqual(report['height_mm'], 1469.5, places=6)
        self.assertGreater(sum(s['rejected_high_points'] for s in report['strips']), 700)
        self.assertGreater(sum(s['rejected_low_points'] for s in report['strips']), 300)
        self.assertTrue(all(s['inlier_points'] > s['seed_points'] for s in report['strips']))

    def test_missing_or_narrow_strip_does_not_fabricate_a_height(self):
        xyz, corners = scene(121)
        for keep in [~((xyz[:, 0] > 2880) & (xyz[:, 0] < 4320)),
                     ~((xyz[:, 0] > 2880) & (xyz[:, 0] < 4320) & (xyz[:, 1] > 400))]:
            with self.subTest():
                report = self.estimate(xyz[keep], corners)
                self.assertEqual(report['status'], 'unsupported')
                self.assertIsNone(report['height_mm'])
                self.assertNotEqual(report['strips'][2]['status'], 'supported')

    def test_rotation_row_order_and_input_are_preserved(self):
        xyz, corners = scene()
        expected = self.estimate(xyz, corners)
        before = xyz.copy()
        angle = .63
        matrix = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        rotated, rotated_corners = xyz.copy(), corners.copy()
        rotated[:, :2] = xyz[:, :2] @ matrix.T + [3400, -7500]
        rotated_corners[:, :2] = corners[:, :2] @ matrix.T + [3400, -7500]
        perm = np.random.default_rng(29).permutation(len(xyz))
        report = self.estimate(rotated[perm], rotated_corners)
        self.assertAlmostEqual(report['height_mm'], expected['height_mm'], places=7)
        self.assertEqual(self.estimate(xyz[perm], corners), expected)
        np.testing.assert_array_equal(before, xyz)

    def test_pipeline_single_height_sets_all_corner_z_without_moving_xy(self):
        xyz, corners = scene()
        board = pipeline.compute_geometry_result(corners)
        raw = {'label_2': board, 'label_3': None}
        before = copy.deepcopy(raw)
        with patch('algorithm_modules.point_cloud_segment_module.shared_deck_geometry.process_shared_decks_timed',
                   return_value=(raw, [], .1)):
            result, rows, seconds = pipeline.process_segmented_points_timed(
                xyz, np.full(len(xyz), 2), pipeline.PipelineConfig())
        self.assertEqual(result['label_2']['height_mean_mm'], 1470)
        self.assertEqual(result['label_2']['geometry_plane_height_mean_mm'], 1510.)
        self.assertEqual(result['label_2']['corner_height_mean_mm'], 1470)
        for key in ('corners_xyz_mm', 'radar_corners_xyz_mm'):
            np.testing.assert_array_equal(np.array(result['label_2'][key])[:, :2], np.array(before['label_2'][key])[:, :2])
            np.testing.assert_allclose(np.array(result['label_2'][key])[:, 2], 1470)
        self.assertEqual(result['label_2']['length_mm'], 12000.)
        self.assertEqual(result['label_2']['width_mm'], 2500.)
        self.assertEqual(result['label_2']['tilt_angle_deg'], 0.)
        self.assertGreater(seconds, .1)
        self.assertTrue(rows)

    def test_two_boards_keep_corner_xy_and_use_their_own_height_for_all_four_z(self):
        xyz, corners = scene()
        high = pipeline.compute_geometry_result(corners)
        low = pipeline.compute_geometry_result(corners - [0, 0, 300], 5)
        raw = {'label_2': high, 'label_3': low}
        expected = copy.deepcopy(raw)
        # A genuine second board must meet the temporary 30000-point rule.
        low_xyz = np.repeat(xyz - [0, 0, 300], 6, axis=0)
        with patch('algorithm_modules.point_cloud_segment_module.shared_deck_geometry.process_shared_decks_timed',
                   return_value=(raw, [], .1)):
            result, _, _ = pipeline.process_segmented_points_timed(
                np.vstack([xyz, low_xyz]), np.r_[np.full(len(xyz), 2), np.full(len(low_xyz), 3)], pipeline.PipelineConfig())
        self.assertEqual(result['label_2']['height_mean_mm'], 1490)
        self.assertEqual(result['label_3']['height_mean_mm'], 1150)
        for label in ('label_2', 'label_3'):
            for key in ('corners_xyz_mm', 'radar_corners_xyz_mm'):
                np.testing.assert_array_equal(np.array(result[label][key])[:, :2], np.array(expected[label][key])[:, :2])
                np.testing.assert_allclose(np.array(result[label][key])[:, 2], result[label]['height_mean_mm'])
            self.assertEqual(result[label]['length_mm'], 12000.)
            self.assertEqual(result[label]['width_mm'], 2500.)

    def test_height_reference_does_not_use_corner_z_as_a_prior(self):
        xyz, corners = scene()
        expected = self.estimate(xyz, corners)
        corners[:, 2] = [-2000, 9000, 7000, -5000]
        self.assertEqual(self.estimate(xyz, corners), expected)

    def test_service_and_ui_use_new_reference_and_keep_missing_height_missing(self):
        _, corners = scene()
        for height, status in [(1469.5, 'supported'), (None, 'unsupported')]:
            with self.subTest(height=height):
                board = pipeline.compute_geometry_result(corners)
                board.update(height_mean_mm=height, corner_height_mean_mm=1510.,
                             height_estimation={'method': 'front_five_strip_planes', 'status': status})
                normalized = PointCloudProcessingService.normalize_result({'label_2': board, 'label_3': None})
                expected_height = 1470 if height is not None else None
                self.assertEqual(normalized['boards'][0]['height_mean_mm'], expected_height)
                self.assertEqual(normalized['boards'][0]['height_estimation']['status'], status)
                displayed = build_board_data({'radar_result': normalized})[0]['radar']
                self.assertEqual(displayed['height_mean_mm'], expected_height)
                self.assertGreater(displayed['length_mm'], 12000)


if __name__ == '__main__':
    unittest.main()
