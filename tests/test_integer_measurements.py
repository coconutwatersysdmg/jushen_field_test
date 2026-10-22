"""Final measurement outputs use decimal half-up integers, not banker's rounding."""
import copy
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import unittest
from unittest.mock import patch
from PySide6.QtWidgets import QApplication
from algorithm_modules.point_cloud_segment_module import point_cloud_pipeline as p
from services.point_cloud_processing_service import PointCloudProcessingService
from ui.board_data_panel import BoardDataPanel
from utils.measurement_rounding import round_measurement
from controllers.flow_controller import FlowController
from services.sensor_calibration_service import SensorCalibrationService
from services.board_data_service import build_board_data


class IntegerMeasurementTests(unittest.TestCase):
    def test_half_ties_are_not_changed_by_plane_fitting_float_noise(self):
        self.assertEqual(round_measurement(1489.4999999999998), 1490)
        self.assertEqual(round_measurement(-1489.4999999999998), -1490)
        self.assertEqual(round_measurement(1489.49999), 1489)

    def test_geometry_outputs_round_half_up_for_positive_and_negative_values(self):
        for value, expected in [(2.5, 3), (3.5, 4), (-2.5, -3), (-3.5, -4)]:
            with self.subTest(value=value):
                corners=[[value, 0, value], [value+2500, 0, value],
                         [value, 8000, value], [value+2500, 8000, value]]
                result=p.compute_geometry_result(corners)
                self.assertEqual(result['radar_corners_xyz_mm'][0][0], expected)
                self.assertEqual(result['height_mean_mm'], expected)
                for point in result['corners_xyz_mm']+result['radar_corners_xyz_mm']:
                    self.assertTrue(all(type(v) is int for v in point))
                for key in ('length_mm','width_mm','height_mean_mm','tilt_angle_deg','offset_angle_deg'):
                    self.assertIs(type(result[key]), int)

    def test_service_rounds_legacy_decimal_measurements_without_mutating_source(self):
        board=dict(corner_ids=['P1','P2','P3','P4'],
                   corners_xyz_mm=[[-2.5,3.5,1000.5],[2500.5,3.5,1000.5],[-2.5,8000.5,1000.5],[2500.5,8000.5,1000.5]],
                   coordinate_frame='world',length_mm=8000.5,width_mm=2500.5,
                   height_mean_mm=1000.5,tilt_angle_deg=-2.5,offset_angle_deg=3.5)
        before=copy.deepcopy(board)
        result=PointCloudProcessingService.normalize_result({'label_2':board})
        self.assertEqual(result['corner_points_xyz_mm'][0],[-3,4,1001])
        self.assertEqual(result['boards'][0]['length_mm'],8001)
        self.assertEqual(result['boards'][0]['tilt_angle_deg'],-3)
        for point in result['world_points'].values():
            self.assertTrue(all(type(point[axis]) is int for axis in ('x', 'y', 'z')))
        self.assertEqual(board,before)

    def test_controller_keeps_final_world_coordinates_as_integers(self):
        controller=FlowController.__new__(FlowController)
        controller.calibration=SensorCalibrationService()
        world={f'P{i}':dict(x=2.5,y=-2.5,z=1000.5) for i in range(1,5)}
        result=controller._normalize_radar_result({'coordinate_frame':'world','world_points':world})
        self.assertEqual(result['world_points']['P1'],dict(x=3,y=-3,z=1001))
        self.assertTrue(all(type(v) is int for v in result['world_points']['P1'].values()))

    def test_panel_data_keeps_dimensions_measured_before_coordinate_rounding(self):
        board=dict(corner_ids=['P1','P2','P3','P4'],coordinate_frame='world',
                   geometry_revision='head_near_anchors_v3',
                   corners_xyz_mm=[[0,0,1000],[0,2500,1000],[8001,1,1000],[8001,2501,1000]],
                   length_mm=8000,width_mm=2500,height_mean_mm=1000,
                   tilt_angle_deg=0,offset_angle_deg=90,
                   height_estimation={'method':'front_five_strip_planes','status':'supported'})
        source=PointCloudProcessingService.normalize_result({'label_2':board})
        controller=FlowController.__new__(FlowController)
        controller.calibration=SensorCalibrationService()
        for radar in (source,controller._normalize_radar_result(source)):
            displayed=build_board_data({'radar_result':radar})[0]['radar']
            self.assertEqual(displayed['length_mm'],8000)

    def test_loading_decision_uses_length_before_integer_presentation(self):
        corners=[[0,0,1000.2],[0,2500,1000.2],[4199.6,0,1000.2],[4199.6,2500,1000.2]]
        high=p.compute_geometry_result(corners)
        high['global_plane_radar_corners_xyz_mm']=copy.deepcopy(corners)
        low=p.compute_geometry_result(corners,5)
        low['global_plane_radar_corners_xyz_mm']=copy.deepcopy(corners)
        with patch('algorithm_modules.point_cloud_segment_module.shared_deck_geometry.process_shared_decks_timed',
                   return_value=({'label_2':high,'label_3':low},[],0.)), \
             patch('algorithm_modules.point_cloud_segment_module.deck_height_methods.estimate_deck_height',
                   side_effect=lambda *args,**kw:dict(status='supported',height_mm=1000)):
            result,_,_=p.process_segmented_points_timed([],[],p.PipelineConfig(pallet_length_mm=1200))
        plan=result['label_2']['loading_plan']
        self.assertEqual(result['label_2']['length_mm'],4200)
        self.assertAlmostEqual(result['label_2']['geometry_plane_height_mean_mm'],1000.2)
        self.assertEqual(plan['normal_pallet_count'],3)
        self.assertFalse(plan['pad_required'])
        self.assertEqual(plan['final_pallet_count'],3)

    def test_panel_displays_integer_coordinates_metrics_and_missing_values(self):
        app=QApplication.instance() or QApplication([])
        panel=BoardDataPanel(lidar_world_matrix=None)
        panel.update_from_round_data({'radar_result':{'coordinate_frame':'world',
            'world_points':{'P1':{'x':2.5,'y':-2.5,'z':1000.5}},
            'boards':[dict(corner_ids=['P1','P2','P3','P4'],
                           corners_xyz_mm=[[2.5,-2.5,1000.5],[2503,-2.5,1000.5],
                                           [2.5,7998,1000.5],[2503,7998,1000.5]],
                           length_mm=8000.5,width_mm=2500.5,height_mean_mm=None,
                           height_estimation={'method':'front_five_strip_planes','status':'unsupported'},
                           tilt_angle_deg=2.5,offset_angle_deg=-2.5)]}})
        self.assertEqual(panel.corner_table.item(0,1).text(),'3')
        self.assertEqual(panel.corner_table.item(0,2).text(),'-3')
        self.assertEqual(panel.board_tables[0].item(0,1).text(),'8001 mm')
        self.assertEqual(panel.board_tables[0].item(0,3).text(),'—')
        panel.close()


if __name__=='__main__':unittest.main()
