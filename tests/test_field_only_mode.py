from __future__ import annotations

import ast
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class FieldOnlyModeTests(unittest.TestCase):
    def test_only_field_profile_is_exposed(self):
        from config import feature_switches

        profiles = feature_switches.list_run_profiles()

        self.assertEqual([item["id"] for item in profiles], ["field"])
        self.assertEqual(feature_switches.RUN_PROFILE, "field")

    def test_non_field_profile_cannot_be_applied(self):
        from config import feature_switches

        with self.assertRaises(ValueError):
            feature_switches.apply_run_profile("lab", persist=False)

    def test_controller_uses_field_cycle_for_first_and_repeat_rounds(self):
        from controllers.flow_controller import FlowController

        controller = FlowController.__new__(FlowController)
        controller.round_index = 0
        first_codes = [code for code, _ in controller.steps]
        controller.round_index = 1
        repeat_codes = [code for code, _ in controller.steps]

        self.assertEqual(
            first_codes,
            [
                "DEVICE_CHECK",
                "LAB_SENSE",
                "LAB_CORNER_SHELL",
                "LAB_WAIT_MANUAL_PLACE",
                "LAB_PLACE_VERIFY",
            ],
        )
        self.assertEqual(
            repeat_codes,
            [
                "LAB_RETURN_ORIGIN",
                "LAB_SENSE",
                "LAB_PRE_PLACE_MONITOR",
                "LAB_WAIT_MANUAL_PLACE",
                "LAB_PLACE_VERIFY",
            ],
        )

    def test_controller_no_longer_contains_simulation_step_lists(self):
        from controllers.flow_controller import FlowController

        self.assertFalse(hasattr(FlowController, "FIRST_STEPS"))
        self.assertFalse(hasattr(FlowController, "REPEAT_STEPS"))

    def test_device_factory_keeps_field_offline_adapters(self):
        from config.system_config import get_system_config
        from core.digital_twin_state import DigitalTwinState
        from devices.device_factory import create_device_adapters
        from devices.field_camera_hub import FieldCameraHub
        from devices.mock_devices import MockPLCAdapter, MockRadarAdapter, MockRobotAdapter

        mode, plc, robot, radar, camera = create_device_adapters(
            DigitalTwinState(), get_system_config()
        )

        self.assertEqual(mode, "mock")
        self.assertIsInstance(plc, MockPLCAdapter)
        self.assertIsInstance(robot, MockRobotAdapter)
        self.assertIsInstance(radar, MockRadarAdapter)
        self.assertIsInstance(camera, FieldCameraHub)
        self.assertTrue(camera.allow_file_inputs)

    def test_runtime_ui_has_no_mode_selector(self):
        source = (ROOT / "ui" / "main_window.py").read_text(encoding="utf-8")
        tree = ast.parse(source)
        names = {
            node.id
            for node in ast.walk(tree)
            if isinstance(node, ast.Name)
        }

        self.assertNotIn("profile_combo", names)
        self.assertNotIn("list_run_profiles", names)

    def test_field_algorithms_have_no_laboratory_package(self):
        self.assertFalse((ROOT / "algorithm_modules" / "lab").exists())

        from algorithm_modules.field_offline.camera_transform import CameraWorldTransform
        from algorithm_modules.field_offline.dynamic_box_monitor import analyze_image
        from algorithm_modules.field_offline.fork_hole import locate_fork_holes
        from services.corner_recognition_service import CornerRecognitionService

        self.assertTrue(callable(CameraWorldTransform.from_json))
        self.assertTrue(callable(analyze_image))
        self.assertTrue(callable(locate_fork_holes))
        field_corner_model = ROOT / "models" / "corner_service.pt"
        self.assertTrue(field_corner_model.is_file())
        self.assertEqual(CornerRecognitionService().model_path, field_corner_model.resolve())


if __name__ == "__main__":
    unittest.main()
