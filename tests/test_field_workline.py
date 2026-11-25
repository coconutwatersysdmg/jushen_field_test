# -*- coding: utf-8 -*-
"""现场工作区数据通道：计划展开、角点指令导出、归档、结果载入。"""
from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from services.field_workline_service import DEFAULT_PLAN_PATH, FieldWorklineService


class FieldWorklineServiceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.service = FieldWorklineService(run_root=Path(self.tmp.name))

    def test_load_plan_expands_quantity_into_rounds(self):
        plan = self.service.load_plan(DEFAULT_PLAN_PATH)
        self.assertEqual(plan["total_rounds"], 3)
        self.assertEqual(
            [item["instance_id"] for item in plan["queue"]],
            ["CARGO-001-001-001", "CARGO-001-002-002", "CARGO-001-003-003"],
        )
        self.assertEqual(self.service.load_plan()["total_rounds"], 3)

    def test_extract_corners_only_accepts_p1_to_p4(self):
        radar = {
            "world_points": {
                "P1": {"x": -3500, "y": 1000, "z": 1800},
                "P2": {"x_mm": -3500, "y_mm": 2200, "z_mm": 1800},
                "P3": [-1100, 2200, 1800],
                "P4": {"x": -1100, "y": 1000, "z": 1800},
                "P5": {"x": 0, "y": 0, "z": 0},
            }
        }
        points = self.service.extract_corners(radar)
        self.assertEqual([point["point_id"] for point in points], ["P1", "P2", "P3", "P4"])
        self.assertTrue(all(point["source"] == "RADAR_COARSE" for point in points))
        self.assertEqual(points[2]["x_mm"], -1100.0)
        self.assertTrue(all(point["source"] == "DEMO_PLACEHOLDER" for point in self.service.demo_corners()))

    def test_export_corner_scan_command_is_json_export_pending_plc(self):
        info = self.service.start_session("corner", "TEST-CAR-01")
        command_path = self.service.export_corner_scan_command(self.service.demo_corners())
        payload = json.loads(Path(command_path).read_text(encoding="utf-8"))
        self.assertEqual(payload["command_type"], "MOVE_CAMERA_TO_CORNER_AND_CAPTURE")
        self.assertEqual(payload["status"], "INTERFACE_PENDING")
        self.assertEqual(payload["target"], "PLC")
        self.assertEqual(payload["vehicle_id"], "TEST-CAR-01")
        self.assertEqual([step["point_id"] for step in payload["capture_sequence"]], ["P1", "P2", "P3", "P4"])
        self.assertEqual(payload["capture_sequence"][0]["source"], "DEMO_PLACEHOLDER")
        manifest = json.loads((Path(info["session_dir"]) / "manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(len(manifest["steps"]), 1)

    def test_archive_image_and_load_result_json(self):
        source = Path(self.tmp.name) / "source.jpg"
        source.write_bytes(b"fake-image-bytes")
        self.service.start_session("pallet_cycle")
        archived = self.service.archive_image("pallet_cycle", source, "R01_PALLET_1")
        self.assertTrue(Path(archived).is_file())
        manifest = json.loads((self.service.session_dir / "manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(manifest["images"][0]["tag"], "R01_PALLET_1")

        result_path = Path(self.tmp.name) / "result.json"
        result_path.write_text(json.dumps({"success": True, "detected_holes": 2}), encoding="utf-8")
        loaded = self.service.load_result_json(result_path)
        self.assertTrue(loaded["data"]["success"])

    def test_example_assets_are_discoverable(self):
        self.assertEqual(len(self.service.example_images("corner")), 4)
        self.assertTrue(self.service.example_images("pallet"))
        self.assertTrue(self.service.example_images("post_place"))
        self.assertTrue(Path(self.service.example_result_path("pallet_hole")).is_file())


if __name__ == "__main__":
    unittest.main()
