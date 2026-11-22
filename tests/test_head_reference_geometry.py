"""The approved head/left anchor constraints, exercised on measured clouds."""
import unittest
from dataclasses import replace
from unittest.mock import patch
import numpy as np

from algorithm_modules.point_cloud_segment_module import point_cloud_pipeline as p
from algorithm_modules.point_cloud_segment_module import shared_deck_geometry as shared
from services.point_cloud_processing_service import PointCloudProcessingService
from services.board_data_service import build_board_data


def deck(start, end, z, *, front_slope=.03, rear_slope=.015, far_taper=.02, gradient=0.):
    # Hand-defined physical sides: near y=0; far y=2500+taper*x;
    # front x=start+front_slope*y; rear x=end+rear_slope*y.
    yf = (2500 + far_taper * start) / (1 - far_taper * front_slope)
    yr = (2500 + far_taper * end) / (1 - far_taper * rear_slope)
    corners = np.array([[start, 0], [start + front_slope*yf, yf],
                        [end, 0], [end + rear_slope*yr, yr]])
    s, t = np.meshgrid(np.linspace(0, 1, 241), np.linspace(0, 1, 121))
    xy = ((1-s)*(1-t))[..., None]*corners[0] + ((1-s)*t)[..., None]*corners[1] + \
         (s*(1-t))[..., None]*corners[2] + (s*t)[..., None]*corners[3]
    xy = xy.reshape(-1, 2)
    return np.column_stack([xy, z + gradient*xy[:, 0]])


def paired(present, low_start=4600, gradient=0.):
    high = deck(0, 4000, 1600, gradient=gradient)
    low = deck(low_start if present else 4020, 12000, 1100,
               front_slope=.02, rear_slope=-.012, far_taper=0.)
    low = np.repeat(low, 2, axis=0)  # A real low board exceeds the 30000-point rule.
    if present:
        u, v = np.meshgrid(np.linspace(4100, 4400, 35), np.linspace(800, 1700, 40))
        joint = np.column_stack([u.ravel(), v.ravel(), np.full(u.size, 1400.)])
        # The concentration-only policy needs multiple observed directions to
        # represent a support; a lone horizontal patch is fully concentrated.
        v, z = np.meshgrid(np.linspace(800,1700,40),np.linspace(1160,1390,30))
        rear=np.column_stack([np.full(v.size,4400.),v.ravel(),z.ravel()])
        joint=np.vstack([joint,rear])
    else:
        v, z = np.meshgrid(np.linspace(60, 2440, 100), np.linspace(1160, 1540, 40))
        joint = np.column_stack([(4000 + .03*v).ravel(), v.ravel(), z.ravel()])
    return np.vstack([high, low, joint]), np.r_[np.full(len(high), 2), np.full(len(low), 3), np.ones(len(joint))].astype(int)


class HeadReferenceGeometryTests(unittest.TestCase):
    config = replace(p.PipelineConfig(), truck_head_dir=(-1, 0, 0),
                     edge_plane_iterations=45, line_ransac_iterations=180)

    def run_cloud(self, xyz, labels):
        return p.process_segmented_points_timed(xyz, labels, self.config)[0]

    def assert_parallel(self, a, b):
        self.assertAlmostEqual(float(np.linalg.det(np.stack([a, b]))) /
                               (np.linalg.norm(a)*np.linalg.norm(b)), 0., delta=1e-9)

    def test_single_keeps_projected_head_angle_and_front_right_and_rear_left_anchors(self):
        xyz = deck(0, 8000, 1600)
        result = self.run_cloud(xyz, np.full(len(xyz), 2))
        self.assertIsNotNone(result['label_2'], 'Oblique measured head/near references must be retained')
        # Full-precision diagnostics retain the actual line constraints.
        c = np.array(result['label_2']['global_plane_radar_corners_xyz_mm'])
        # Original head x=.03*y; raw far y=2500+.02*x. Their intersection
        # is (75.045027,2501.500901), and the raw near/rear corner is (8000,0).
        np.testing.assert_allclose(c[1, :2], [75.045027, 2501.500901], atol=8)
        np.testing.assert_allclose(c[2, :2], [8000, 0], atol=8)
        self.assert_parallel(c[1, :2]-c[0, :2], np.array([.03, 1.]))
        self.assert_parallel(c[3, :2]-c[2, :2], c[1, :2]-c[0, :2])
        self.assert_parallel(c[3, :2]-c[1, :2], c[2, :2]-c[0, :2])
        self.assertGreater(abs(float((c[1, :2]-c[0, :2]) @ (c[2, :2]-c[0, :2]))), 1000)
        np.testing.assert_allclose(c[:, 2], 1600, atol=.001)

    def test_absent_support_uses_high_rear_for_joint_but_low_rear_for_vehicle_tail(self):
        xyz, labels = paired(False)
        result = self.run_cloud(xyz, labels)
        high, low = result['label_2'], result['label_3']
        self.assertEqual(high['transition_support']['decision'], 'absent')
        a, b = np.array(high['global_plane_radar_corners_xyz_mm']), np.array(low['global_plane_radar_corners_xyz_mm'])
        np.testing.assert_allclose(a[2:, :2], b[:2, :2], atol=1e-8)
        self.assertAlmostEqual(a[2, 0], 4000, delta=8)
        self.assertAlmostEqual(b[2, 0], 12000, delta=8)
        self.assert_parallel(b[3, :2]-b[2, :2], a[1, :2]-a[0, :2])
        np.testing.assert_allclose(a[:, 2], 1600, atol=.001)
        np.testing.assert_allclose(b[:, 2], 1100, atol=.001)

    def test_fragmented_support_uses_low_front_anchor_without_adding_support_to_deck_points(self):
        xyz, labels = paired(True)
        before, before_labels = xyz.copy(), labels.copy()
        result = self.run_cloud(xyz, labels)
        high, low = result['label_2'], result['label_3']
        self.assertEqual(high['transition_support']['decision'], 'present')
        self.assertTrue(high['transition_support']['merge_applied'])
        a, b = np.array(high['global_plane_radar_corners_xyz_mm']), np.array(low['global_plane_radar_corners_xyz_mm'])
        np.testing.assert_allclose(a[2:, :2], b[:2, :2], atol=1e-8)
        self.assertAlmostEqual(a[2, 0], 4600, delta=8)
        self.assert_parallel(a[3, :2]-a[2, :2], a[1, :2]-a[0, :2])
        np.testing.assert_allclose(a[:, 2], 1600, atol=.001)
        np.testing.assert_array_equal(xyz, before)
        np.testing.assert_array_equal(labels, before_labels)

    def test_support_protruding_beyond_sides_cannot_change_width_or_corner_xy(self):
        xyz, labels = paired(True)
        original = self.run_cloud(xyz, labels)
        u, v = np.meshgrid(np.linspace(4050, 4450, 60), np.r_[np.linspace(-6000, -100, 50), np.linspace(3000, 7000, 50)])
        protruding = np.column_stack([u.ravel(), v.ravel(), np.full(u.size, 1400)])
        added = self.run_cloud(np.vstack([xyz, protruding]), np.r_[labels, np.ones(len(protruding), int)])
        for key in ('label_2', 'label_3'):
            np.testing.assert_allclose(np.array(original[key]['radar_corners_xyz_mm'])[:, :2],
                                       np.array(added[key]['radar_corners_xyz_mm'])[:, :2], atol=1e-8, rtol=0)
            self.assertEqual(original[key]['width_mm'], added[key]['width_mm'])

    def test_height_crop_uses_final_high_region_but_only_its_local_front_part(self):
        first_xyz, first_labels = paired(True, 4600, gradient=.01)
        second_xyz, second_labels = paired(True, 5000, gradient=.01)
        a = self.run_cloud(first_xyz, first_labels)['label_2']
        b = self.run_cloud(second_xyz, second_labels)['label_2']
        self.assertGreater(b['length_mm']-a['length_mm'], 350)
        self.assertGreater(b['height_mean_mm'], a['height_mean_mm'])
        for board in (a,b):
            source=np.array(board['global_plane_radar_corners_xyz_mm'])[:,:2]
            expected=source[0]+np.array([[.05,.05],[.05,.95],[.85,.05],[.85,.95]]) @ np.stack([
                source[2]-source[0],source[1]-source[0]])
            np.testing.assert_allclose(board['height_estimation']['crop_radar_xy_mm'],expected,atol=1e-7)
            np.testing.assert_allclose(board['height_source_radar_corners_xyz_mm'],board['global_plane_radar_corners_xyz_mm'])
            self.assertEqual(board['height_estimation']['height_crop_basis'],'final_this_deck_geometry')

    def test_large_support_gap_does_not_fabricate_height_for_unsupported_last_strip(self):
        xyz, labels = paired(True, 5200, gradient=.01)
        board = self.run_cloud(xyz, labels)['label_2']
        self.assertIsNone(board['height_mean_mm'])
        self.assertEqual(board['height_estimation']['strips'][-1]['reason'], 'insufficient_spatial_coverage')
        self.assertEqual(board['height_estimation']['status'], 'unsupported')

    def test_unreliable_original_high_tail_does_not_block_final_local_height_crop(self):
        xyz,labels=paired(True)
        measure=shared._measure_end
        def reject_original_high_tail(points,plane_points,plane,origin,u,v,sides,config,edge,head_line,**kw):
            if edge=='u_max' and np.median(points[:,2])>1500:
                raise shared.UnsupportedGeometry('synthetic_unreliable_original_high_tail')
            return measure(points,plane_points,plane,origin,u,v,sides,config,edge,head_line,**kw)
        with patch.object(shared,'_measure_end',side_effect=reject_original_high_tail):
            result=self.run_cloud(xyz,labels)
        board=result['label_2']
        self.assertNotIn(result.get('geometry_status'),('partial','unsupported'))
        self.assertEqual(board['height_mean_mm'],1600)
        self.assertEqual(board['height_estimation']['height_crop_basis'],'final_this_deck_geometry')
        self.assertTrue(all(strip['status']=='supported' for strip in board['height_estimation']['strips']))
        self.assertTrue(PointCloudProcessingService.normalize_result(result)['success'])

    def test_missing_H_does_not_publish_reference_plane_z_as_live_corners(self):
        xyz = deck(0, 8000, 1600, front_slope=0, rear_slope=0, far_taper=0)
        xyz = xyz[(xyz[:, 0] < 1900) | (xyz[:, 0] > 2900)]
        result = self.run_cloud(xyz, np.full(len(xyz), 2))
        board = result['label_2']
        self.assertIsNone(board['height_mean_mm'])
        self.assertEqual(board['corners_xyz_mm'], [])
        self.assertEqual(len(board['available_xy_corners_radar_mm']), 4)
        normalized = PointCloudProcessingService.normalize_result(result)
        self.assertFalse(normalized['success'])
        self.assertEqual(normalized['world_points'], {})

    def test_final_flat_corners_and_shared_boundaries_are_row_order_invariant(self):
        xyz, labels = paired(True)
        expected = self.run_cloud(xyz, labels)
        order = np.random.default_rng(57).permutation(len(xyz))
        actual = self.run_cloud(xyz[order], labels[order])
        for key in ('label_2', 'label_3'):
            np.testing.assert_array_equal(actual[key]['radar_corners_xyz_mm'], expected[key]['radar_corners_xyz_mm'])

    def test_missing_high_H_does_not_move_low_board_into_the_first_display_row(self):
        xyz, labels = paired(False)
        keep = (labels != 2) | (xyz[:, 0] < 1150) | (xyz[:, 0] > 2100)
        result = self.run_cloud(xyz[keep], labels[keep])
        self.assertIsNone(result['label_2']['height_mean_mm'])
        self.assertEqual(result['label_3']['height_mean_mm'], 1100.)
        normalized = PointCloudProcessingService.normalize_result(result)
        displayed = build_board_data({'radar_result': normalized})
        self.assertIsNotNone(displayed[1]['radar'], 'Valid low board must remain in display row two')
        self.assertIsNone(displayed[0]['radar']['height_mean_mm'])
        self.assertEqual(displayed[1]['radar']['height_mean_mm'], 1100.)


if __name__ == '__main__':
    unittest.main()
