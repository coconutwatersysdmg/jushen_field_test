"""Independent height methods and per-deck crop origins, with fixed corner XY."""
import copy
import unittest
from dataclasses import replace
from unittest.mock import patch
from types import SimpleNamespace

import numpy as np

from algorithm_modules.point_cloud_segment_module import point_cloud_pipeline as p
from services.point_cloud_processing_service import PointCloudProcessingService
from services.board_data_service import build_board_data


class DeckHeightMethodsTests(unittest.TestCase):
    def setUp(self):
        h, w = np.meshgrid(np.linspace(0, 4000, 121), np.linspace(0, 2500, 101))
        l, v = np.meshgrid(np.linspace(4500, 14000, 361), np.linspace(0, 2500, 101))
        self.high = np.column_stack([h.ravel(), w.ravel(), (1600 + .01*h + .002*w).ravel()])
        self.low = np.column_stack([l.ravel(), v.ravel(), (1150 + .005*(l-4500) + .004*v).ravel()])
        self.high_corners = np.array([[0, 0, 1600], [0, 2500, 1605], [4000, 0, 1640], [4000, 2500, 1645.]])
        self.low_corners = np.array([[4500, 0, 1150], [4500, 2500, 1160], [14000, 0, 1197.5], [14000, 2500, 1207.5]])

    def run_fixed(self, high=None, low=None, scheme='A'):
        high = self.high if high is None else high
        low = self.low if low is None else low
        raw = {'label_2': p.compute_geometry_result(self.high_corners),
               'label_3': p.compute_geometry_result(self.low_corners, 5)}
        original = copy.deepcopy(raw)
        xyz = np.vstack([high, low])
        labels = np.r_[np.full(len(high), 2), np.full(len(low), 3)]
        self.assertTrue('height_scheme' in p.PipelineConfig.__dataclass_fields__, 'Missing independently switchable height A/B/C/D')
        config = replace(p.PipelineConfig(), height_scheme=scheme)
        with patch('algorithm_modules.point_cloud_segment_module.shared_deck_geometry.process_shared_decks_timed', return_value=(raw, [], .1)):
            result, _, _ = p.process_segmented_points_timed(xyz, labels, config)
        return result, original

    def test_default_A_uses_high_80_and_low_40_from_each_own_head(self):
        high, low = self.high.copy(), self.low.copy()
        high[(high[:, 0] < 199.99) | (high[:, 0] > 3400.01) | (high[:, 1] < 124.99) | (high[:, 1] > 2375.01), 2] += 300
        low[(low[:, 0] < 4974.99) | (low[:, 0] > 8775.01) | (low[:, 1] < 124.99) | (low[:, 1] > 2375.01), 2] += 500
        result, original = self.run_fixed(high, low)
        for label, expected, fraction, role in [('label_2', 1621, .8, 'high'), ('label_3', 1167, .4, 'low')]:
            board = result[label]
            self.assertAlmostEqual(board['height_mean_mm'], expected, places=3)
            self.assertEqual(board['height_estimation']['height_scheme'], 'A')
            self.assertEqual(board['height_estimation']['board_role'], role)
            self.assertEqual(board['height_estimation']['length_fraction'], fraction)
            self.assertEqual(board['height_estimation']['width_fraction'], .9)
            self.assertEqual(len(board['height_estimation']['strips']), 5)
            for key in ('corners_xyz_mm', 'radar_corners_xyz_mm'):
                np.testing.assert_array_equal(np.array(board[key])[:, :2], np.array(original[label][key])[:, :2])
                np.testing.assert_allclose(np.array(board[key])[:, 2], expected)
            self.assertEqual(board['length_mm'], 4000. if label=='label_2' else 9500.)
            self.assertEqual(board['width_mm'], 2500.)
            self.assertEqual(board['tilt_angle_deg'], 0.)
        low_centers = [s['center_radar_xyz_mm'][0] for s in result['label_3']['height_estimation']['strips']]
        np.testing.assert_allclose(low_centers, [5355, 6115, 6875, 7635, 8395])
        self.assertEqual(result['label_3']['height_estimation']['width_start'], 'P5_P7_near_scanner_side')
        self.assertEqual(result['label_3']['height_estimation']['crop_start_corner_ids'], ['P5', 'P6'])

    def test_all_four_methods_remain_selectable_for_both_decks(self):
        for scheme in 'ABCD':
            with self.subTest(scheme=scheme):
                result, _ = self.run_fixed(scheme=scheme)
                for label, expected in [('label_2', 1621), ('label_3', 1167)]:
                    estimation = result[label]['height_estimation']
                    self.assertEqual(estimation['height_scheme'], scheme)
                    self.assertEqual(estimation['status'], 'supported')
                    self.assertAlmostEqual(result[label]['height_mean_mm'], expected, delta=1. if scheme == 'C' else .001)

    def test_unsupported_low_height_stays_missing_and_roles_do_not_collapse(self):
        result, _ = self.run_fixed(low=np.empty((0, 3)))
        self.assertIsNone(result['label_3']['height_mean_mm'])
        self.assertEqual(result['label_3']['height_estimation']['status'], 'unsupported')
        normalized = PointCloudProcessingService.normalize_result(result)
        self.assertEqual(normalized['high_board_label'], 'label_2')
        self.assertEqual(normalized['low_board_label'], 'label_3')
        self.assertIsNone(normalized['height_difference_mm'])
        displayed = build_board_data({'radar_result': normalized})
        self.assertEqual(displayed[0]['radar']['height_mean_mm'], 1621)
        self.assertIsNone(displayed[1]['radar']['height_mean_mm'])

    def test_all_method_heights_reach_world_display(self):
        for scheme in 'ABCD':
            with self.subTest(scheme=scheme):
                result, _ = self.run_fixed(scheme=scheme)
                normalized = PointCloudProcessingService.normalize_result(result)
                displayed = build_board_data({'radar_result': normalized})
                for i, label in enumerate(('label_2', 'label_3')):
                    self.assertEqual(displayed[i]['radar']['height_mean_mm'], result[label]['height_mean_mm'])

    def test_unknown_height_scheme_is_rejected(self):
        self.assertTrue('height_scheme' in p.PipelineConfig.__dataclass_fields__)
        with self.assertRaises(ValueError):
            self.run_fixed(scheme='unexpected')

    def test_service_passes_method_and_per_board_crop_overrides(self):
        service = PointCloudProcessingService(device='cpu')
        service.pc_cfg.update(height_scheme='B', high_height_length_fraction=.7,
                              low_height_length_fraction=.35, high_low_height_width_fraction=.9)
        with patch('algorithm_modules.point_cloud_segment_module.point_cloud_pipeline.PointCloudPipeline',
                   side_effect=lambda **kw: SimpleNamespace(config=kw['config'])):
            config = service._ensure_pipeline().config
        self.assertEqual(config.height_scheme, 'B')
        self.assertEqual(config.high_height_length_fraction, .7)
        self.assertEqual(config.low_height_length_fraction, .35)
        self.assertEqual(config.high_low_height_width_fraction, .9)


if __name__ == '__main__':
    unittest.main()
