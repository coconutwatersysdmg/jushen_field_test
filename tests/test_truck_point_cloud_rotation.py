from dataclasses import replace
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from algorithm_modules.point_cloud_segment_module import point_cloud_pipeline as p
from services.point_cloud_processing_service import PointCloudProcessingService


class TruckRotationTests(unittest.TestCase):
    def test_historical_xy_rotation_about_origin_preserves_z(self):
        source = np.array([[1000., 0., 100.], [0., 2000., 250.], [3000., -4000., 700.]])
        original = source.copy()
        corrected = p.rotate_extracted_truck_xyz(source, -1.0)
        np.testing.assert_allclose(corrected[:2], [
            [999.8476951563913, -17.45240643728351, 100.],
            [34.90481287456702, 1999.6953903127826, 250.],
        ], atol=1e-10, rtol=0)
        np.testing.assert_array_equal(corrected[:, 2], source[:, 2])
        np.testing.assert_allclose(np.linalg.norm(corrected[:, :2], axis=1),
                                   np.linalg.norm(source[:, :2], axis=1), atol=1e-10, rtol=0)
        np.testing.assert_array_equal(source, original)

    def test_zero_angle_is_an_independent_identity_and_invalid_angles_fail(self):
        source = np.array([[1., 2., 3.], [4., 5., 6.]])
        corrected = p.rotate_extracted_truck_xyz(source, 0.)
        np.testing.assert_array_equal(corrected, source)
        self.assertFalse(np.shares_memory(corrected, source))
        for invalid in [float('nan'), float('inf'), -float('inf')]:
            with self.assertRaises(ValueError):
                p.rotate_extracted_truck_xyz(source, invalid)
            with self.assertRaises(ValueError):
                replace(p.PipelineConfig(), truck_rotation_z_deg=invalid)

    def test_matches_historical_world_rotation_about_the_lidar_world_origin(self):
        # Coordinates recorded in the user's 2026-11-21 comparison report.
        original_world = np.array([[22449., 4998., 1454.], [22481., 2497., 1454.]])
        expected_world = np.array([
            [22400.28569749079, 4849.893833478802, 1454.],
            [22388.63235523615, 2348.7162708866745, 1454.],
        ])
        yaw = np.deg2rad(-90.7)
        base_rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        radar = original_world.copy()
        radar[:, :2] = (original_world[:, :2] - [13939., 7715.]) @ base_rotation
        corrected_world = p.radar_to_world(p.rotate_extracted_truck_xyz(radar, -1.0))
        np.testing.assert_allclose(corrected_world, expected_world, atol=1e-8, rtol=0)

    def test_file_pipeline_rotates_after_network_and_before_geometry_once(self):
        source = np.array([[1000., 0., 100.], [0., 2000., 250.], [3000., -4000., 700.]])
        original = source.copy()
        expected = np.array([
            [999.8476951563913, -17.45240643728351, 100.],
            [34.90481287456702, 1999.6953903127826, 250.],
            [2929.73345972004, -4051.748000414415, 700.],
        ], dtype=np.float32)
        pipe = p.PointCloudPipeline.__new__(p.PointCloudPipeline)
        pipe.config = p.PipelineConfig()
        seen_network, seen_geometry = [], []
        def network(xyz):
            seen_network.append(xyz.copy())
            return np.full(len(xyz), 2), {2: len(xyz)}, {'inference_s': 0.}
        def geometry(xyz, labels, config):
            seen_geometry.append(xyz.copy())
            return {}, [], 0.
        pipe.segment_xyz_full_coverage = network
        def preprocess(cloud, config):
            return SimpleNamespace(points=source), {'stat_filtered': len(source)}, {'preprocess_total_s': 0.}
        with TemporaryDirectory() as folder, \
                patch.object(p, 'read_point_cloud', return_value=object()), \
                patch.object(p, 'preprocess_point_cloud_timed', side_effect=preprocess), \
                patch.object(p, 'process_segmented_points_timed', side_effect=geometry):
            for index in range(2):
                record = pipe.process_file('original.pcd', Path(folder)/str(index))
                saved = json.loads(Path(record['json']).read_text(encoding='utf-8'))
                self.assertEqual(saved['truck_rotation']['axis'], 'z')
                self.assertEqual(saved['truck_rotation']['angle_deg'], -1.0)
                self.assertEqual(record['truck_rotation']['stage'], 'after_pointnet_before_geometry')
                self.assertEqual(record['truck_rotation']['center_radar_xyz_mm'], [0., 0., 0.])
                self.assertEqual(record['truck_rotation']['equivalent_world_center_xy_mm'], [13939., 7715.])
                self.assertIn('truck_rotation_s', record)
        for xyz in seen_network:
            np.testing.assert_array_equal(xyz, source.astype(np.float32))
        for xyz in seen_geometry:
            np.testing.assert_allclose(xyz, expected, atol=.001, rtol=0)
        np.testing.assert_array_equal(source, original)
        np.testing.assert_array_equal(seen_network[0], seen_network[1])
        np.testing.assert_array_equal(seen_geometry[0], seen_geometry[1])

    def test_service_forwards_zero_override_to_disable_the_default_correction(self):
        service = PointCloudProcessingService.__new__(PointCloudProcessingService)
        service._pipeline = None
        service.pc_cfg = {'truck_rotation_z_deg': 0.0}
        service.device = 'cpu'
        with TemporaryDirectory() as folder:
            service.checkpoint_path = Path(folder)/'model.pth'
            service.checkpoint_path.write_bytes(b'test-only checkpoint')
            with patch.object(p, 'PointCloudPipeline', return_value=Mock()) as factory:
                service._ensure_pipeline()
            self.assertEqual(factory.call_args.kwargs['config'].truck_rotation_z_deg, 0.0)

    def test_service_preserves_corrected_corners_and_rotation_record_without_rotating_again(self):
        raw_corners = np.array([[1000., 0., 100.], [1000., 2000., 100.],
                                [4000., 0., 100.], [4000., 2000., 100.]])
        board = p.compute_geometry_result(p.rotate_extracted_truck_xyz(raw_corners, -1.0))
        rotation = dict(axis='z', angle_deg=-1.0, stage='after_pointnet_before_geometry')
        record = dict(result={'label_2': board, 'label_3': None}, truck_rotation=rotation,
                      truck_rotation_s=.001, pointnet_timing={'device': 'cuda'},
                      preprocess_timing={}, total_s=1., geometry_s=.1)
        service = PointCloudProcessingService.__new__(PointCloudProcessingService)
        service._pipeline = SimpleNamespace(process_file=lambda *args: record)
        with TemporaryDirectory() as folder:
            service.result_root = Path(folder)/'results'
            service.checkpoint_path = Path(folder)/'model.pth'
            input_path = Path(folder)/'truck.pcd'
            input_path.write_bytes(b'file access verified; pipeline is isolated in this test')
            output = service.process_file(input_path)
        self.assertTrue(output['success'])
        self.assertEqual(output['truck_rotation'], rotation)
        self.assertEqual(output['timing']['truck_rotation_s'], .001)
        np.testing.assert_array_equal(output['corner_points_xyz_mm'], board['corners_xyz_mm'])


if __name__ == '__main__':
    unittest.main()
