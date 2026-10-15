import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch, create_autospec

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('QT_QUICK_BACKEND', 'software')

from config import feature_switches
from config.offline_debug_inputs import build_debug_inputs_from_offline
from config.offline_paths import PROJECT_ROOT, get_offline_paths
from core.digital_twin_state import DigitalTwinState
from devices.mock_devices import MockRadarAdapter


class FieldInputTests(unittest.TestCase):
    def _controller(self):
        from controllers.flow_controller import FlowController
        from services.vehicle_database_service import VehicleDatabaseService
        database = create_autospec(VehicleDatabaseService, instance=True)
        database.backend = 'test'
        database.location = 'test-only'
        with patch('controllers.flow_controller.create_vehicle_database_service', return_value=database):
            controller = FlowController()
        controller.set_plan([{'instance_id': 'test', 'cargo_code': 'test', 'quantity': 1}])
        return controller

    def _hole_images(self, directory):
        import cv2
        import numpy as np
        rgb = np.full((480, 640, 3), 200, dtype=np.uint8)
        cv2.rectangle(rgb, (145, 220), (235, 270), (10, 10, 10), -1)
        cv2.rectangle(rgb, (390, 220), (480, 270), (10, 10, 10), -1)
        depth = np.full((480, 640), 1000, dtype=np.uint16)
        rgb_path, depth_path = Path(directory) / 'rgb.png', Path(directory) / 'depth.png'
        self.assertTrue(cv2.imwrite(str(rgb_path), rgb))
        self.assertTrue(cv2.imwrite(str(depth_path), depth))
        return {'pallet_rgb': str(rgb_path), 'pallet_depth': str(depth_path)}

    def test_real_device_check_branch_advances_to_second_step(self):
        controller = self._controller()
        snapshot = controller.execute_next()
        self.assertEqual(snapshot['step_code'], 'FIELD_SENSE')
        check = snapshot['results'][-1]['data']
        self.assertTrue(check['all_online'])
        self.assertEqual(set(check['devices']), {'PLC', 'RADAR', 'CAM_CORNER', 'CAM_PALLET'})

    def test_real_camera_frame_hole_recognition_without_pose_keeps_camera_results(self):
        controller = self._controller()
        with tempfile.TemporaryDirectory() as temp:
            controller.set_debug_inputs(self._hole_images(temp))
            result = controller._field_pick_recognize()
        self.assertTrue(result['success'], result['message'])
        self.assertEqual(result['hole_result']['coordinate_frame'], 'camera')
        self.assertTrue(result['hole_result']['left_xyz_mm'])
        self.assertEqual(result['hole_world_targets'], {})
        self.assertEqual(result['hole_result']['world_transform_status'], 'UNAVAILABLE')
        self.assertEqual(result['coordinate_message']['data']['coordinate_frame'], 'camera')

    def test_real_camera_frame_holes_transform_with_explicit_offline_pose(self):
        controller = self._controller()
        with tempfile.TemporaryDirectory() as temp:
            inputs = self._hole_images(temp)
            inputs['offline_plc_pose'] = {'x': 123.0, 'y': 234.0, 'z': 345.0, 'r': -80.0}
            controller.set_debug_inputs(inputs)
            first = controller._field_pick_recognize()
            self.assertTrue(first['success'], first['message'])
            self.assertEqual(first['hole_result']['world_coordinate_frame'], 'world')
            self.assertEqual(first['hole_result']['plc_pose'], inputs['offline_plc_pose'])
            self.assertEqual(set(first['hole_world_targets']), {'left', 'right'})
            inputs['offline_plc_pose']['x'] += 1000.0
            controller.round_data.pop('pick_result')
            controller.set_debug_inputs(inputs)
            shifted = controller._field_pick_recognize()
            self.assertTrue(shifted['success'], shifted['message'])
            self.assertAlmostEqual(shifted['hole_world_targets']['left'][0] - first['hole_world_targets']['left'][0], 1000.0)
            self.assertEqual(shifted['hole_world_targets']['left'][1:], first['hole_world_targets']['left'][1:])

    def test_recorded_mech_pose_enters_offline_debug_inputs(self):
        cfg = get_offline_paths()
        cfg['mech_plc_pose'] = {'x': 1, 'y': 2, 'z': 3, 'r': 4}
        with patch('config.offline_debug_inputs.get_offline_paths', return_value=cfg):
            inputs = build_debug_inputs_from_offline()
        self.assertEqual(inputs['offline_plc_pose'], cfg['mech_plc_pose'])

    def test_only_field_profile_survives_legacy_profile_selection(self):
        self.assertEqual([p['id'] for p in feature_switches.list_run_profiles()], ['field'])
        for legacy in ('sim', 'lab'):
            self.assertEqual(feature_switches.apply_run_profile(legacy, persist=False)['id'], 'field')

    def test_default_pcd_directory_is_inside_project(self):
        self.assertEqual(Path(get_offline_paths()['radar_pcd']), PROJECT_ROOT / 'pcd')

    def test_missing_pcd_reports_failure_without_generated_corners(self):
        result = MockRadarAdapter(DigitalTwinState()).locate_truck({})
        self.assertFalse(result['success'])
        self.assertNotIn('world_points', result)

    def test_explicit_selection_survives_offline_reload(self):
        with tempfile.TemporaryDirectory() as temp:
            chosen = Path(temp) / 'selected.pcd'
            chosen.write_text('data', encoding='utf-8')
            base = {'selected_radar_pcd': str(chosen), 'point_cloud_path': str(chosen)}
            result = build_debug_inputs_from_offline(base)
            self.assertEqual(result['point_cloud_path'], str(chosen))
            self.assertFalse(result['point_cloud_example_only'])

    def test_chosen_pcd_wins_over_stale_cargo_input(self):
        with tempfile.TemporaryDirectory() as temp:
            chosen = Path(temp) / 'chosen.pcd'
            previous = Path(temp) / 'previous.pcd'
            chosen.write_text('chosen', encoding='utf-8')
            previous.write_text('previous', encoding='utf-8')
            radar = MockRadarAdapter(DigitalTwinState())
            radar.set_point_cloud_path(str(chosen))
            result = radar.locate_truck({'point_cloud_path': str(previous)})
            self.assertEqual(result['pcd_path'], str(chosen))

    def test_second_step_passes_selected_pcd_to_actual_radar_facade(self):
        from controllers.flow_controller import FlowController
        from services.algorithm_facade import AlgorithmFacade
        from services.vehicle_database_service import VehicleDatabaseService
        database = create_autospec(VehicleDatabaseService, instance=True)
        database.backend = 'test'
        database.location = 'test-only'
        facade = AlgorithmFacade()
        points = {'P1': {'x': 0, 'y': 0, 'z': 0}, 'P2': {'x': 1200, 'y': 0, 'z': 0},
                  'P3': {'x': 0, 'y': 6000, 'z': 0}, 'P4': {'x': 1200, 'y': 6000, 'z': 0}}
        with tempfile.TemporaryDirectory() as temp:
            selected = Path(temp) / 'chosen.pcd'
            selected.write_text('pcd test', encoding='utf-8')
            with patch('controllers.flow_controller.create_vehicle_database_service', return_value=database):
                controller = FlowController(algorithms=facade)
            controller.queue = [{'instance_id': 'test-cargo'}]
            controller.set_debug_inputs({'selected_radar_pcd': str(selected), 'point_cloud_path': str(selected), 'point_cloud_example_only': True})
            with patch.object(facade.point_cloud, 'process_file', return_value={'success': True, 'world_points': points, 'corner_ids': list(points)}) as process:
                result = controller._radar_task()
            process.assert_called_once_with(str(selected), result_tag='test-cargo')
            self.assertEqual(result['world_points'], points)
            self.assertFalse(controller.debug_inputs['point_cloud_example_only'])

    def test_third_step_uses_configured_offline_corner_images_without_sdk_capture(self):
        from controllers.flow_controller import FlowController
        from services.algorithm_facade import AlgorithmFacade
        from services.vehicle_database_service import VehicleDatabaseService
        database = create_autospec(VehicleDatabaseService, instance=True)
        database.backend = 'test'
        database.location = 'test-only'
        with tempfile.TemporaryDirectory() as temp:
            images = {}
            for i in range(1, 5):
                image = Path(temp) / f'P{i}.jpg'
                image.write_bytes(b'test image')
                images[f'P{i}'] = str(image)
            facade = AlgorithmFacade()
            with patch('controllers.flow_controller.create_vehicle_database_service', return_value=database):
                controller = FlowController(algorithms=facade)
            controller.queue = [{'instance_id': 'test'}]
            controller.set_debug_inputs({'corner_images': images})
            controller.round_data['radar_result'] = {'success': True, 'corner_ids': list(images), 'world_points': {
                'P1': {'x': 0, 'y': 0, 'z': 0}, 'P2': {'x': 1200, 'y': 0, 'z': 0},
                'P3': {'x': 0, 'y': 6000, 'z': 0}, 'P4': {'x': 1200, 'y': 6000, 'z': 0}}}
            with patch.object(controller.camera, 'capture_rgb', side_effect=AssertionError('offline step must not capture hardware')), patch.object(facade, 'corner_image_recognize', return_value={'success': True}) as recognize:
                result = controller._field_corner_shell()
            self.assertTrue(result['success'], result['message'])
            self.assertEqual(recognize.call_args.args[0], images)

    def test_offline_camera_missing_files_never_calls_vendor_sdk(self):
        from devices.field_camera_hub import FieldCameraHub
        camera = FieldCameraHub(DigitalTwinState(), {'mock_probe': True})
        with patch('devices.field_camera_hub.capture_one_frame', side_effect=AssertionError('SDK forbidden offline')):
            result = camera.capture_rgb('CAM_CORNER', tag='FIELD_CORNER')
        self.assertFalse(result['success'])

    def test_field_camera_results_populate_board_panel(self):
        from services.board_data_service import build_board_data
        points = {'P1': {'x': 0, 'y': 0, 'z': 0}, 'P2': {'x': 1200, 'y': 0, 'z': 0},
                  'P3': {'x': 0, 'y': 6000, 'z': 0}, 'P4': {'x': 1200, 'y': 6000, 'z': 0}}
        result = build_board_data({'field_corner_shell': {'camera_world_corners': points}})
        self.assertIsNotNone(result[0]['camera'])

    def test_field_hole_step_records_real_module_evidence(self):
        from controllers.flow_controller import FlowController
        from services.vehicle_database_service import VehicleDatabaseService
        database = create_autospec(VehicleDatabaseService, instance=True)
        database.backend = 'test'
        database.location = 'test-only'
        with patch('controllers.flow_controller.create_vehicle_database_service', return_value=database):
            controller = FlowController()
        controller.queue = [{'instance_id': 'test'}]
        hole = {'success': True, 'world_coordinate_frame': 'world', 'left_world_xyz_mm': [1, 2, 3], 'right_world_xyz_mm': [4, 5, 6]}
        with patch.object(controller, '_capture_rgbd', return_value={'success': True, 'rgb_path': 'image.jpg', 'depth_path': 'depth.png'}), patch.object(controller.algorithms, 'field_pallet_hole_recognize', return_value=hole):
            result = controller._field_pick_recognize()
        self.assertTrue(result['success'])
        evidence = next(row for row in controller.module_evidence.snapshot() if row['module_id'] == 'FIELD_PALLET_HOLE_TRADITIONAL')
        self.assertEqual(evidence['last_inputs']['rgb_path'], 'image.jpg')

    def test_persisted_selection_is_checked_at_controller_startup(self):
        from controllers.flow_controller import FlowController
        from services.vehicle_database_service import VehicleDatabaseService
        from config.radar_input import save_selected_radar_pcd
        database = create_autospec(VehicleDatabaseService, instance=True)
        database.backend = 'test'
        database.location = 'test-only'
        with tempfile.TemporaryDirectory() as temp:
            selected = Path(temp) / 'chosen.pcd'
            selected.write_text('pcd', encoding='utf-8')
            with patch('config.radar_input.SELECTION_STATE_FILE', Path(temp) / 'state.json'), patch('controllers.flow_controller.create_vehicle_database_service', return_value=database):
                save_selected_radar_pcd(str(selected))
                controller = FlowController()
            inputs_message = next(m for m in controller.twin.snapshot()['messages'] if m['source'] == 'MODULE_INPUTS')
            self.assertEqual(inputs_message['data']['resolved'].get('radar_pcd'), str(selected))

    def test_offline_config_error_never_falls_back_to_example_pcd(self):
        from controllers.flow_controller import FlowController
        from services.vehicle_database_service import VehicleDatabaseService
        database = create_autospec(VehicleDatabaseService, instance=True)
        database.backend = 'test'
        database.location = 'test-only'
        with patch('controllers.flow_controller.create_vehicle_database_service', return_value=database), patch('config.offline_debug_inputs.build_debug_inputs_from_offline', side_effect=ValueError('bad config')):
            controller = FlowController()
            self.assertEqual(controller.debug_inputs.get('point_cloud_path'), '')
            self.assertEqual(controller.default_debug_inputs().get('point_cloud_path'), '')


class FieldWindowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def test_debug_inputs_preserve_pcd_selection_and_update_field_place_tags(self):
        from ui.debug_dialog import DebugInputDialog
        dialog = DebugInputDialog({'selected_radar_pcd': 'chosen.pcd', 'point_cloud_path': 'chosen.pcd', 'offline_field_test': True})
        try:
            dialog.lines['place_verify_rgb'].setText('place.jpg')
            values = dialog.values()
            self.assertEqual(values['selected_radar_pcd'], 'chosen.pcd')
            self.assertEqual(values['images']['field_place_verify'], 'place.jpg')
            self.assertEqual(values['images']['field_pre_place_monitor'], 'place.jpg')
        finally:
            dialog.deleteLater()

    def test_visible_chooser_persists_and_drives_radar_second_stage(self):
        from ui.main_window import MainWindow
        from services.vehicle_database_service import VehicleDatabaseService
        database = create_autospec(VehicleDatabaseService, instance=True)
        database.backend = 'test'
        database.location = 'test-only'
        with tempfile.TemporaryDirectory() as temp:
            selected = Path(temp) / 'input.pcd'
            selected.write_text('test pcd', encoding='utf-8')
            state = Path(temp) / 'selection.json'
            with patch('controllers.flow_controller.create_vehicle_database_service', return_value=database), patch('config.radar_input.SELECTION_STATE_FILE', state):
                window = MainWindow()
                window._check_offline_paths = lambda **kwargs: False
                try:
                    self.assertEqual(window.radar_pcd_btn.text(), '选择雷达 PCD')
                    self.assertFalse(window.radar_pcd_btn.isHidden())
                    with patch('ui.main_window.QFileDialog.getOpenFileName', return_value=(str(selected), 'PCD (*.pcd)')) as chooser:
                        window._choose_radar_pcd()
                    self.assertEqual(chooser.call_args.args[2], str(PROJECT_ROOT / 'pcd'))
                    window._reload_offline_debug_inputs()
                    locate = window.controller.radar.locate_truck({})
                    self.assertEqual(locate['pcd_path'], str(selected.resolve()))
                    self.assertIn(str(selected), window.radar_pcd_label.text())
                    self.assertIn('FIELD_SENSE', [code for code, _ in window.controller.steps])
                    self.assertEqual(window.controller.default_debug_inputs()['point_cloud_path'], str(selected))
                    from config.radar_input import load_selected_radar_pcd
                    self.assertEqual(load_selected_radar_pcd(), str(selected))
                finally:
                    window.close()
                    window.deleteLater()
                    self.app.processEvents()


if __name__ == '__main__':
    unittest.main()
