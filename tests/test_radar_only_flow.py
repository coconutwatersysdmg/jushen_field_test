from __future__ import annotations

import os
import contextlib
import io
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import create_autospec, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QT_QUICK_BACKEND", "software")
os.environ.setdefault("QSG_RHI_BACKEND", "software")

from PySide6.QtWidgets import QApplication

import config.feature_switches as switches
from config.offline_path_check import validate_offline_paths
from controllers.flow_controller import FlowController
from services.vehicle_database_service import VehicleDatabaseService
from ui.main_window import MainWindow
import main


class RadarOnlyFlowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.enterContext(patch.object(switches, "RUN_PROFILE", "field"))
        self.enterContext(patch.object(switches, "RADAR_ONLY_TEST", True, create=True))
        database = create_autospec(VehicleDatabaseService, instance=True)
        database.backend = "test"
        database.location = "test-only"
        self.enterContext(patch("controllers.flow_controller.create_vehicle_database_service", return_value=database))
        folder = self.enterContext(tempfile.TemporaryDirectory())
        self.pcd = Path(folder) / "selected.PCD"
        self.pcd.write_text(
            "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
            "WIDTH 4\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 4\nDATA ascii\n"
            "0 0 0\n1 0 0\n0 1 0\n1 1 0\n", encoding="utf-8",
        )
        self.enterContext(patch("controllers.flow_controller.STATE_FILE", Path(folder) / "state.json"))
        self.enterContext(patch("controllers.flow_controller.RESULT_FILE", Path(folder) / "results.json"))

    def test_no_pcd_selection_blocks_start_without_falling_back_to_demo(self):
        controller = FlowController()
        with self.assertRaisesRegex(RuntimeError, "PCD"):
            controller.start()
        self.assertFalse(controller.running)

    def test_only_radar_runs_and_completes_without_camera_inputs_or_plan(self):
        controller = FlowController()
        controller.select_radar_pcd(str(self.pcd))
        output = {"success": True, "message": "radar test result", "boards": []}
        with patch.object(controller.algorithms, "radar_process", return_value=output) as radar, \
             patch.object(controller.camera, "capture_rgbd", side_effect=AssertionError("camera must not run")), \
             patch.object(controller.plc, "send_message", side_effect=AssertionError("PLC must not receive test results")):
            controller.start()
            result = controller.execute_next()
        self.assertTrue(result["finished"])
        self.assertFalse(result["running"])
        self.assertEqual(result["round_data"]["radar_result"], output)
        self.assertEqual(radar.call_args.args[0]["pcd_path"], str(self.pcd.resolve()))
        self.assertEqual([item[0] for item in controller.steps], ["RADAR_TEST"])

    def test_non_pcd_is_rejected_for_algorithm_input(self):
        controller = FlowController()
        other = self.pcd.with_suffix(".itl")
        other.write_text("not pcd", encoding="utf-8")
        with self.assertRaisesRegex((ValueError, RuntimeError), "PCD"):
            controller.select_radar_pcd(str(other))

    def test_radar_validation_does_not_require_camera_or_placement_data(self):
        report = validate_offline_paths({"radar_pcd": str(self.pcd)}, radar_only=True)
        self.assertTrue(report["ok"], report)
        self.assertEqual(report["warnings"], [])
        self.assertEqual(report["errors"], [])

    def test_cancel_pcd_picker_blocks_manual_and_auto_start(self):
        window = MainWindow()
        self.addCleanup(window.deleteLater)
        self.addCleanup(window.close)
        with patch("ui.main_window.QFileDialog.getOpenFileName", return_value=("", "")):
            window._start()
            self.assertFalse(window.controller.running)
            window._auto()
            self.assertFalse(window.timer.isActive())
            self.assertFalse(window.controller.running)

    def test_reload_preserves_explicit_selection(self):
        window = MainWindow()
        self.addCleanup(window.deleteLater)
        self.addCleanup(window.close)
        with patch("ui.main_window.QFileDialog.getOpenFileName", return_value=(str(self.pcd), "PCD")):
            self.assertTrue(window._select_radar_pcd())
        window._reload_offline_debug_inputs()
        self.assertEqual(window.controller.debug_inputs["point_cloud_path"], str(self.pcd.resolve()))

    def test_window_runs_selected_pcd_then_allows_selecting_another_test(self):
        window = MainWindow()
        self.addCleanup(window.deleteLater)
        self.addCleanup(window.close)
        output = {"success": True, "message": "test completed", "boards": []}
        with patch("ui.main_window.QFileDialog.getOpenFileName", return_value=(str(self.pcd), "PCD")), \
             patch.object(window.controller.algorithms, "radar_process", return_value=output):
            self.assertTrue(window._start())
            self.assertTrue(window.controller.finished)
            self.assertFalse(window.timer.isActive())
            self.assertTrue(window.select_pcd_btn.isEnabled())
            self.assertTrue(window._select_radar_pcd())
            self.assertFalse(window.controller.finished)
            self.assertTrue(window.next_btn.isEnabled())

    def test_algorithm_error_allows_changing_pcd_without_closing_application(self):
        window = MainWindow()
        self.addCleanup(window.deleteLater)
        self.addCleanup(window.close)
        with patch("ui.main_window.QFileDialog.getOpenFileName", return_value=(str(self.pcd), "PCD")), \
             patch.object(window.controller.algorithms, "radar_process", side_effect=RuntimeError("bad PCD data")), \
             patch("ui.main_window.QMessageBox.critical"):
            self.assertFalse(window._start())
        self.assertFalse(window.controller.running)
        self.assertTrue(window.select_pcd_btn.isEnabled())

    def test_gui_event_pump_continues_while_inference_is_running(self):
        controller = FlowController()
        controller.select_radar_pcd(str(self.pcd))
        pumped = threading.Event()
        controller.set_ui_event_pump(pumped.set)

        def infer(*args):
            self.assertTrue(pumped.wait(timeout=2), "GUI events stalled during radar inference")
            return {"success": True, "message": "test completed", "boards": []}

        with patch.object(controller.algorithms, "radar_process", side_effect=infer):
            controller.start()
            self.assertTrue(controller.execute_next()["finished"])

    def test_monitor_start_failure_does_not_block_local_pcd_test(self):
        window = MainWindow()
        self.addCleanup(window.deleteLater)
        self.addCleanup(window.close)
        with patch.object(main, "_RADAR_SYNC_STARTED", False), \
             patch.object(main, "_RADAR_SYNC_PROCESS", None), \
             patch.object(main, "_log_startup"), \
             patch("platform.system", return_value="Windows"), \
             patch("subprocess.Popen", side_effect=OSError("PowerShell unavailable")), \
             contextlib.redirect_stderr(io.StringIO()), \
             patch("ui.main_window.QMessageBox.warning") as warning, \
             patch("ui.main_window.QMessageBox.critical") as critical, \
             patch("ui.main_window.QFileDialog.getOpenFileName", return_value=(str(self.pcd), "PCD")), \
             patch.object(window.controller.algorithms, "radar_process", return_value={"success": True}):
            main._start_radar_sync()
            self.assertTrue(window._start())
            self.assertTrue(window.controller.finished)
            warning.assert_not_called()
            critical.assert_not_called()

    def test_monitor_connection_failure_and_exit_do_not_block_local_pcd_test(self):
        window = MainWindow()
        self.addCleanup(window.deleteLater)
        self.addCleanup(window.close)
        process = SimpleNamespace(
            stdout=io.StringIO("[RADAR_SYNC][ERROR] Remote path not accessible\n"),
            wait=lambda: 1,
        )
        with patch.object(main, "_log_startup") as log, \
             patch("ui.main_window.QMessageBox.warning") as warning, \
             patch("ui.main_window.QMessageBox.critical") as critical, \
             patch("ui.main_window.QFileDialog.getOpenFileName", return_value=(str(self.pcd), "PCD")), \
             patch.object(window.controller.algorithms, "radar_process", return_value={"success": True}):
            main._forward_radar_sync_output(process)
            self.assertTrue(window._start())
            self.assertTrue(window.controller.finished)
            self.assertTrue(any("Remote path not accessible" in call.args[0] for call in log.call_args_list))
            warning.assert_not_called()
            critical.assert_not_called()
