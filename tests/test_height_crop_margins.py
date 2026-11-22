"""Height sampling excludes both width edges and the front five percent."""
import copy
import unittest
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from algorithm_modules.point_cloud_segment_module import point_cloud_pipeline as p
from algorithm_modules.point_cloud_segment_module.deck_height_methods import estimate_deck_height
from services.point_cloud_processing_service import PointCloudProcessingService


def sample():
    u, v = np.meshgrid(np.linspace(0, 1, 141), np.linspace(0, 1, 101))
    xyz = np.column_stack([(10000*u).ravel(), (2500*v).ravel(),
                           (1400+100*u+20*v).ravel()])
    corners = np.array([[0, 0, 0], [0, 2500, 0], [10000, 0, 0], [10000, 2500, 0]], float)
    return xyz, corners


class HeightCropMarginsTests(unittest.TestCase):
    def test_defaults_exclude_front_and_both_width_edges_for_all_methods(self):
        xyz, corners = sample()
        clean = xyz.copy()
        contaminated = (xyz[:, 0] < 500-1e-6) | (xyz[:, 1] < 125-1e-6) | (xyz[:, 1] > 2375+1e-6)
        xyz[contaminated, 2] += 1000
        for method in 'ABCD':
            with self.subTest(method=method):
                expected = estimate_deck_height(clean, corners, scheme=method)
                report = estimate_deck_height(xyz, corners, scheme=method)
                self.assertEqual(report['status'], 'supported')
                np.testing.assert_allclose(report['crop_radar_xy_mm'], [[500, 125], [500, 2375], [6500, 125], [6500, 2375]])
                self.assertAlmostEqual(report['height_mm'], expected['height_mm'], places=8)
                self.assertAlmostEqual(report['height_mm'], 1445., delta=.2 if method == 'C' else 1e-6)
                if method != 'D':
                    np.testing.assert_allclose([s['center_radar_xyz_mm'][:2] for s in report['strips']],
                                               [[1100, 1250], [2300, 1250], [3500, 1250], [4700, 1250], [5900, 1250]])

    def test_each_deck_fraction_uses_its_own_length_with_same_margin(self):
        xyz, corners = sample()
        for fraction, end in ((.6, .65), (.8, .85), (.4, .45)):
            report = estimate_deck_height(xyz, corners, length_fraction=fraction)
            self.assertEqual(report['status'], 'supported')
            np.testing.assert_allclose(report['length_range_fraction'], [.05, end])
            np.testing.assert_allclose(report['width_range_fraction'], [.05, .95])
            self.assertAlmostEqual(report['strips'][0]['length_range_fraction'][0], .05)
            self.assertAlmostEqual(report['strips'][-1]['length_range_fraction'][1], end)

    def test_rotated_quad_uses_local_margins(self):
        xyz, corners = sample()
        theta = .41
        rotation = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        reference = estimate_deck_height(xyz, corners)
        xyz[:, :2] = xyz[:, :2] @ rotation.T + [800, -500]
        corners[:, :2] = corners[:, :2] @ rotation.T + [800, -500]
        report = estimate_deck_height(xyz[::-1], corners)
        self.assertAlmostEqual(report['height_mm'], reference['height_mm'], places=8)
        np.testing.assert_allclose(report['crop_radar_xy_mm'], np.array(reference['crop_radar_xy_mm']) @ rotation.T + [800, -500])

    def test_invalid_crop_windows_are_rejected(self):
        xyz, corners = sample()
        for kwargs in [dict(length_start_fraction=-.01), dict(width_start_fraction=float('nan')),
                       dict(length_start_fraction=.5, length_fraction=.6), dict(width_start_fraction=.2, width_fraction=.9)]:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                estimate_deck_height(xyz, corners, **kwargs)

    def test_software_defaults_and_service_override_reach_pipeline(self):
        cfg = p.PipelineConfig()
        self.assertEqual(cfg.height_length_start_fraction, .05)
        self.assertEqual(cfg.height_width_start_fraction, .05)
        self.assertEqual(cfg.single_height_width_fraction, .9)
        self.assertEqual(cfg.high_low_height_width_fraction, .9)
        service = PointCloudProcessingService(device='cpu')
        service.pc_cfg.update(height_length_start_fraction=.07, height_width_start_fraction=.08)
        with patch('algorithm_modules.point_cloud_segment_module.point_cloud_pipeline.PointCloudPipeline',
                   side_effect=lambda **kw: SimpleNamespace(config=kw['config'])):
            override = service._ensure_pipeline().config
        self.assertEqual(override.height_length_start_fraction, .07)
        self.assertEqual(override.height_width_start_fraction, .08)

    def test_height_only_refresh_keeps_corner_xy_and_does_not_fit_geometry(self):
        xyz, corners = sample()
        original = dict(label_2=p.compute_geometry_result(corners), label_3=None)
        before = copy.deepcopy(original)
        with patch('algorithm_modules.point_cloud_segment_module.shared_deck_geometry.process_shared_decks_timed',
                   side_effect=AssertionError('height refresh must not fit XY')):
            result, _, _ = p.apply_deck_reference_heights_timed(xyz, np.full(len(xyz), 2), original, p.PipelineConfig())
        self.assertEqual(result['label_2']['height_mean_mm'], 1445)
        for key in ('corners_xyz_mm', 'radar_corners_xyz_mm'):
            np.testing.assert_array_equal(np.array(result['label_2'][key])[:, :2], np.array(before['label_2'][key])[:, :2])


if __name__ == '__main__':
    unittest.main()
