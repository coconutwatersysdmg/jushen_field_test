"""Temporary correction: fewer than 30000 low-deck points become label 2."""
import unittest
from dataclasses import replace
from unittest.mock import patch
import numpy as np

from algorithm_modules.point_cloud_segment_module import point_cloud_pipeline as p
from services.point_cloud_processing_service import PointCloudProcessingService


class SparseLowDeckLabelTests(unittest.TestCase):
    def test_strict_count_threshold_preserves_other_labels_and_source(self):
        for count, merged in ((0, False), (1, True), (10000, True), (29999, True), (30000, False), (30001, False)):
            with self.subTest(count=count):
                labels=np.r_[np.array([0,1,2,2],np.int32),np.full(count,3,np.int32)]
                before=labels.copy()
                corrected,report=p.merge_sparse_low_deck_labels(labels)
                np.testing.assert_array_equal(labels,before)
                np.testing.assert_array_equal(corrected[:4],before[:4])
                np.testing.assert_array_equal(corrected[4:],np.full(count,2 if merged else 3))
                self.assertEqual(report['applied'],merged)
                self.assertEqual(report['merged_points'],count if merged else 0)
                self.assertEqual(report['threshold_points'],30000)
                self.assertEqual(report['label_3_points_before'],count)
                self.assertEqual(report['label_3_points_after'],0 if merged else count)

    def test_correction_precedes_geometry_and_survives_service_failure_output(self):
        xyz=np.zeros((9,3));labels=np.array([0,1,2,2,3,3,3,3,3])
        failure=dict(label_2=None,label_3=None,geometry_status='unsupported',geometry_failures=[])
        with patch('algorithm_modules.point_cloud_segment_module.shared_deck_geometry.process_shared_decks_timed',
                   return_value=(failure,[],0.)) as geometry:
            result,_,_=p.process_segmented_points_timed(xyz,labels,p.PipelineConfig())
        np.testing.assert_array_equal(geometry.call_args.args[1],[0,1,2,2,2,2,2,2,2])
        normalized=PointCloudProcessingService.normalize_result(result)
        self.assertEqual(normalized['label_correction']['merged_points'],5)

    def test_reassigned_points_also_enter_the_height_fit(self):
        corners=[[0,0,1400],[0,2500,1400],[8000,0,1400],[8000,2500,1400]]
        board=p.compute_geometry_result(corners)
        board['global_plane_radar_corners_xyz_mm']=corners
        xyz=np.arange(30,dtype=float).reshape(-1,3)
        labels=np.array([0,1,2,2,2,3,3,3,3,3])
        before=labels.copy()
        with patch('algorithm_modules.point_cloud_segment_module.shared_deck_geometry.process_shared_decks_timed',
                   return_value=({'label_2':board,'label_3':None},[],0.)), \
             patch('algorithm_modules.point_cloud_segment_module.deck_height_methods.estimate_deck_height',
                   return_value=dict(status='supported',height_mm=1400)) as height:
            result,_,_=p.process_segmented_points_timed(xyz,labels,p.PipelineConfig())
        np.testing.assert_array_equal(height.call_args.args[0],xyz[2:])
        np.testing.assert_array_equal(labels,before)
        self.assertEqual(result['label_2']['height_mean_mm'],1400)
        self.assertEqual(height.call_args.kwargs['length_fraction'],.6)

    def test_real_single_deck_with_sparse_low_mislabels_produces_H_and_corners(self):
        x,y=np.meshgrid(np.linspace(0,8000,321),np.linspace(0,2500,101))
        xyz=np.column_stack([x.ravel(),y.ravel(),np.full(x.size,1400.)])
        labels=np.full(len(xyz),2,np.int32)
        labels[np.linspace(0,len(xyz)-1,29999,dtype=int)]=3
        before=labels.copy()
        config=replace(p.PipelineConfig(),truck_head_dir=(-1.,0.,0.),edge_plane_iterations=40,
                       line_ransac_iterations=100)
        result,_,_=p.process_segmented_points_timed(xyz,labels,config)
        self.assertNotIn(result.get('geometry_status'),('partial','unsupported'))
        self.assertIsNone(result['label_3'])
        self.assertEqual(result['label_2']['height_mean_mm'],1400)
        np.testing.assert_array_equal(np.array(result['label_2']['radar_corners_xyz_mm'])[:,2],1400)
        self.assertEqual(result['label_correction']['merged_points'],29999)
        np.testing.assert_array_equal(labels,before)


if __name__=='__main__':unittest.main()
