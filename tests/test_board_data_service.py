# -*- coding: utf-8 -*-
from __future__ import annotations

import math
import unittest
from copy import deepcopy

from services.board_data_service import (
    build_board_data,
    build_corner_comparison,
    camera_corner_groups,
    compute_corner_metrics,
)


def point(x, y, z):
    return {"x": x, "y": y, "z": z}


class BoardDataServiceTests(unittest.TestCase):
    LIDAR_TO_FIXED_WORLD = [
        [0.0, -1.0, 0.0, 100.0],
        [1.0, 0.0, 0.0, 200.0],
        [0.0, 0.0, 1.0, 300.0],
        [0.0, 0.0, 0.0, 1.0],
    ]

    def test_compute_corner_metrics_uses_three_dimensional_edges(self):
        metrics = compute_corner_metrics([
            point(0, 0, 100),
            point(2000, 0, 100),
            point(0, -4000, 300),
            point(2000, -4000, 300),
        ])

        self.assertAlmostEqual(metrics["length_mm"], math.sqrt(4000**2 + 200**2), places=6)
        self.assertAlmostEqual(metrics["width_mm"], 2000.0, places=6)
        self.assertAlmostEqual(metrics["height_mean_mm"], 200.0, places=6)
        self.assertAlmostEqual(metrics["tilt_angle_deg"], 2.8624052261, places=6)
        self.assertAlmostEqual(metrics["offset_angle_deg"], 0.0, places=6)

    def test_horizontal_offset_uses_negative_y_as_zero_and_positive_x_as_positive(self):
        metrics = compute_corner_metrics([
            [0, 0, 0],
            [2000, 0, 0],
            [1000, -4000, 0],
            [3000, -4000, 0],
        ])

        self.assertAlmostEqual(metrics["offset_angle_deg"], 14.0362434679, places=6)

    def test_zero_horizontal_run_matches_radar_zero_angle_fallback(self):
        metrics = compute_corner_metrics([
            [0, 0, 0],
            [2000, 0, 0],
            [0, 0, 100],
            [2000, 0, 100],
        ])

        self.assertEqual(metrics["tilt_angle_deg"], 0.0)
        self.assertEqual(metrics["offset_angle_deg"], 0.0)

    def test_camera_corner_groups_support_eight_six_and_four_points(self):
        eight = {f"P{i}": point(i, 0, 0) for i in range(1, 9)}
        six = {f"P{i}": point(i, 0, 0) for i in range(1, 7)}
        four = {f"P{i}": point(i, 0, 0) for i in range(1, 5)}

        self.assertEqual([[p["x"] for p in g] for g in camera_corner_groups(eight)], [[1, 2, 3, 4], [5, 6, 7, 8]])
        self.assertEqual([[p["x"] for p in g] for g in camera_corner_groups(six)], [[1, 2, 3, 4], [3, 4, 5, 6]])
        self.assertEqual([[p["x"] for p in g] for g in camera_corner_groups(four)], [[1, 2, 3, 4]])

    def test_build_board_data_maps_two_radar_boards_and_eight_camera_points(self):
        radar_boards = [
            {
                "length_mm": 6100.125,
                "width_mm": 2380.5,
                "height_mean_mm": 1250.25,
                "tilt_angle_deg": 1.25,
                "offset_angle_deg": -2.5,
            },
            {
                "length_mm": 3050.0,
                "width_mm": 2360.0,
                "height_mean_mm": 1420.0,
                "tilt_angle_deg": -0.75,
                "offset_angle_deg": 3.125,
            },
        ]
        camera = {
            "P1": point(0, 0, 100), "P2": point(2000, 0, 100),
            "P3": point(0, -4000, 100), "P4": point(2000, -4000, 100),
            "P5": point(0, -4000, 300), "P6": point(2000, -4000, 300),
            "P7": point(0, -7000, 300), "P8": point(2000, -7000, 300),
        }

        boards = build_board_data({
            "radar_result": {"boards": radar_boards},
            "lab_corner_shell": {"camera_world_corners": camera},
        })

        self.assertEqual(len(boards), 2)
        self.assertEqual(boards[0]["title"], "第一块板")
        self.assertEqual(boards[1]["title"], "第二块板")
        self.assertEqual(boards[0]["radar"]["length_mm"], 6100.125)
        self.assertEqual(boards[1]["radar"]["offset_angle_deg"], 3.125)
        self.assertAlmostEqual(boards[0]["camera"]["length_mm"], 4000.0, places=6)
        self.assertAlmostEqual(boards[1]["camera"]["length_mm"], 3000.0, places=6)

    def test_six_camera_points_share_boundary_between_boards(self):
        camera = {
            "P1": point(0, 0, 100), "P2": point(2000, 0, 100),
            "P3": point(0, -4000, 200), "P4": point(2000, -4000, 200),
            "P5": point(0, -7000, 300), "P6": point(2000, -7000, 300),
        }

        boards = build_board_data({"camera_world_corners": camera})

        self.assertAlmostEqual(boards[0]["camera"]["length_mm"], math.sqrt(4000**2 + 100**2), places=6)
        self.assertAlmostEqual(boards[1]["camera"]["length_mm"], math.sqrt(3000**2 + 100**2), places=6)

    def test_four_camera_points_leave_second_camera_board_empty(self):
        camera = {f"P{i}": point((i - 1) % 2 * 2000, -((i - 1) // 2) * 4000, 100) for i in range(1, 5)}

        boards = build_board_data({"camera_world_corners": camera})

        self.assertIsNotNone(boards[0]["camera"])
        self.assertIsNone(boards[1]["camera"])

    def test_final_or_radar_fallback_corners_never_populate_camera_rows(self):
        fallback = {f"P{i}": point(i, -i, 100) for i in range(1, 5)}

        boards = build_board_data({
            "radar_result": {"world_points": fallback, "boards": []},
            "lab_corner_shell": {"final_world_corners": fallback},
        })

        self.assertIsNone(boards[0]["camera"])
        self.assertIsNone(boards[1]["camera"])

    def test_empty_nested_camera_result_uses_valid_top_level_camera_world_corners(self):
        camera = {
            "P1": point(0, 0, 100), "P2": point(2000, 0, 100),
            "P3": point(0, -4000, 100), "P4": point(2000, -4000, 100),
        }

        boards = build_board_data({
            "lab_corner_shell": {"camera_world_corners": {}},
            "camera_world_corners": camera,
        })

        self.assertEqual(boards[0]["camera"]["length_mm"], 4000.0)

    def test_malformed_camera_point_blanks_only_its_source_row(self):
        radar = {
            "length_mm": 6000, "width_mm": 2400, "height_mean_mm": 1200,
            "tilt_angle_deg": 0, "offset_angle_deg": 0,
        }
        camera = {
            "P1": point(0, 0, 0), "P2": point(2000, 0, 0),
            "P3": point(0, -4000, 0), "P4": {"x": "bad", "y": -4000, "z": 0},
        }

        boards = build_board_data({"radar_result": {"boards": [radar]}, "camera_world_corners": camera})

        self.assertEqual(boards[0]["radar"]["length_mm"], 6000.0)
        self.assertIsNone(boards[0]["camera"])
        self.assertIsNone(boards[1]["radar"])

    def test_radar_metrics_are_recomputed_after_fixed_world_transform(self):
        raw_board = {
            "corner_ids": ["P1", "P2", "P3", "P4"],
            "world_points": {
                "P1": point(0, 0, 10),
                "P2": point(2000, 0, 10),
                "P3": point(0, -4000, 110),
                "P4": point(2000, -4000, 110),
            },
            "length_mm": 1,
            "width_mm": 1,
            "height_mean_mm": 1,
            "tilt_angle_deg": 1,
            "offset_angle_deg": 1,
        }
        round_data = {
            "radar_result": {
                "coordinate_frame": "world",
                "boards": [],
                "raw_radar_reference": {"boards": [raw_board]},
            }
        }

        boards = build_board_data(round_data, self.LIDAR_TO_FIXED_WORLD)

        radar = boards[0]["radar"]
        self.assertAlmostEqual(radar["length_mm"], math.sqrt(4000**2 + 100**2), places=6)
        self.assertAlmostEqual(radar["width_mm"], 2000.0, places=6)
        self.assertAlmostEqual(radar["height_mean_mm"], 360.0, places=6)
        self.assertAlmostEqual(radar["tilt_angle_deg"], 1.4320961842, places=6)
        self.assertAlmostEqual(radar["offset_angle_deg"], 90.0, places=6)

    def test_corner_comparison_shows_actual_union_in_fixed_world_without_mutation(self):
        round_data = {
            "radar_result": {
                "coordinate_frame": "world",
                "world_points": {"P1": point(999, 999, 999)},
                "raw_radar_reference": {
                    "boards": [{
                        "corner_ids": ["P1", "P2", "P3", "P4"],
                        "corners_xyz_mm": [
                            [0, 0, 10], [2000, 0, 10],
                            [0, -4000, 110], [2000, -4000, 110],
                        ],
                    }]
                },
            },
            "camera_world_corners": {
                "P1": point(101, 201, 311),
                "P2": point(102, 202, 312),
                "P3": point(103, 203, 313),
                "P9": point(109, 209, 319),
            },
        }
        original = deepcopy(round_data)

        rows = build_corner_comparison(round_data, self.LIDAR_TO_FIXED_WORLD)

        self.assertEqual([row["point_id"] for row in rows], ["P1", "P2", "P3", "P4"])
        self.assertEqual(rows[0]["radar"], {"x": 100.0, "y": 200.0, "z": 310.0})
        self.assertEqual(rows[0]["camera"], {"x": 101.0, "y": 201.0, "z": 311.0})
        self.assertIsNone(rows[3]["camera"])
        self.assertEqual(round_data, original)

    def test_native_radar_points_are_not_exposed_when_transform_is_unavailable(self):
        round_data = {
            "radar_result": {
                "coordinate_frame": "radar_world",
                "world_points": {"P1": point(1, 2, 3)},
            }
        }

        rows = build_corner_comparison(round_data, None)

        self.assertEqual(rows, [])

    def test_six_fixed_world_radar_points_keep_all_original_point_ids(self):
        radar = {f"P{index}": point(index, -index, 100 + index) for index in range(1, 7)}

        rows = build_corner_comparison({
            "radar_result": {
                "coordinate_frame": "world",
                "world_points": radar,
            }
        })

        self.assertEqual([row["point_id"] for row in rows], ["P1", "P2", "P3", "P4", "P5", "P6"])
        self.assertEqual(rows[4]["radar"], {"x": 5.0, "y": -5.0, "z": 105.0})


if __name__ == "__main__":
    unittest.main()
