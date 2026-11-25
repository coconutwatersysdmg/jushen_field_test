# -*- coding: utf-8 -*-
"""两条现场工作区的界面流转：只测联调路径，不弹模态框、不调算法。"""
from __future__ import annotations

import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication, QLabel

from services.field_workline_service import FieldWorklineService
from ui.field_corner_workspace import FieldCornerWorkspace
from ui.field_pallet_cycle_workspace import FieldPalletCycleWorkspace


class FieldWorkspaceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.service = FieldWorklineService(run_root=Path(self.tmp.name))

    def _done_count(self, strip) -> int:
        return sum(1 for label in strip.findChildren(QLabel) if label.text().endswith("DONE"))

    def test_corner_workspace_runs_full_demo_flow(self):
        workspace = FieldCornerWorkspace(self.service)
        self.addCleanup(workspace.deleteLater)
        with patch("ui.field_corner_workspace.QMessageBox.information"), \
             patch("ui.field_corner_workspace.QMessageBox.warning"), \
             patch("ui.field_corner_workspace.QMessageBox.critical"), \
             patch("ui.field_workspace_widgets.QMessageBox.information"), \
             patch("ui.field_workspace_widgets.QMessageBox.warning"):
            workspace.use_demo_points()
            self.assertEqual(len(workspace.points), 4)
            workspace.export_scan_command()
            self.assertTrue(Path(workspace.command_path).is_file())
            workspace.use_example_images()
            self.assertEqual(len(workspace.images), 4)
            workspace.algo_slot.load_example()
            self.assertEqual(workspace.corner_result.get("detected_points"), 4)
            workspace.export_summary()
        self.assertEqual(self._done_count(workspace.step_strip), 5)
        self.assertTrue(Path(workspace.result_path).is_file())

    def test_pallet_cycle_workspace_advances_round_from_plan(self):
        workspace = FieldPalletCycleWorkspace(self.service)
        self.addCleanup(workspace.deleteLater)
        self.assertEqual(workspace.plan["total_rounds"], 3)
        with patch("ui.field_pallet_cycle_workspace.QMessageBox.information"), \
             patch("ui.field_pallet_cycle_workspace.QMessageBox.warning"), \
             patch("ui.field_pallet_cycle_workspace.QMessageBox.critical"), \
             patch("ui.field_workspace_widgets.QMessageBox.information"), \
             patch("ui.field_workspace_widgets.QMessageBox.warning"):
            workspace.next_step()
            self.assertEqual(workspace.step_index, 1)
            self.assertTrue(workspace.images_pre)

            workspace.hole_slot.load_example()
            self.assertEqual(workspace.hole_result.get("detected_holes"), 2)
            workspace.next_step()
            workspace.next_step()
            self.assertEqual(workspace.step_index, 3)
            workspace.next_step()
            self.assertEqual(workspace.step_index, 4)
            self.assertTrue(workspace.images_post)

            workspace.monitor_slot.load_example()
            self.assertEqual(workspace.monitor_result.get("phase"), "post_place_bottom_pallet")
            workspace.next_step()
            self.assertEqual(workspace.step_index, 5)
            workspace.next_step()
        self.assertEqual(workspace.round_index, 1)
        self.assertEqual(workspace.step_index, 0)
        self.assertTrue(any("round_01_summary" in name for name in
                            [path.name for path in (self.service.session_dir / "json").iterdir()]))


if __name__ == "__main__":
    unittest.main()
