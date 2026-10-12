# -*- coding: utf-8 -*-
from __future__ import annotations

"""Build read-only radar/camera board metrics for the monitoring UI."""

from math import atan2, degrees, isfinite, sqrt
from typing import Any, Dict, List, Mapping, Sequence, Tuple


METRIC_KEYS = (
    "length_mm",
    "width_mm",
    "height_mean_mm",
    "tilt_angle_deg",
    "offset_angle_deg",
)


def _xyz(value: Any) -> Tuple[float, float, float]:
    if isinstance(value, Mapping):
        if isinstance(value.get("final_world_xyz_mm"), (list, tuple)):
            raw = value["final_world_xyz_mm"]
            values = (raw[0], raw[1], raw[2])
        else:
            values = (
                value.get("x", value.get("x_mm")),
                value.get("y", value.get("y_mm")),
                value.get("z", value.get("z_mm")),
            )
    elif isinstance(value, (list, tuple)) and len(value) >= 3:
        values = (value[0], value[1], value[2])
    else:
        raise ValueError("point must contain WORLD XYZ")
    result = tuple(float(item) for item in values)
    if not all(isfinite(item) for item in result):
        raise ValueError("WORLD XYZ must be finite")
    return result


def _distance(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return sqrt(sum((b[index] - a[index]) ** 2 for index in range(3)))


def compute_corner_metrics(points: Sequence[Any]) -> Dict[str, float]:
    """Calculate the five displayed metrics from P1/P2/P3/P4 WORLD points."""
    if len(points) != 4:
        raise ValueError("exactly four WORLD corner points are required")
    p1, p2, p3, p4 = (_xyz(point) for point in points)
    head = tuple((p1[index] + p2[index]) / 2.0 for index in range(3))
    tail = tuple((p3[index] + p4[index]) / 2.0 for index in range(3))
    dx, dy, dz = (tail[index] - head[index] for index in range(3))
    horizontal = sqrt(dx * dx + dy * dy)
    return {
        "length_mm": (_distance(p1, p3) + _distance(p2, p4)) / 2.0,
        "width_mm": (_distance(p1, p2) + _distance(p3, p4)) / 2.0,
        "height_mean_mm": (p1[2] + p2[2] + p3[2] + p4[2]) / 4.0,
        "tilt_angle_deg": degrees(atan2(dz, horizontal)),
        "offset_angle_deg": degrees(atan2(dx, -dy)),
    }


def camera_corner_groups(points: Mapping[str, Any]) -> List[List[Any]]:
    """Return camera loading-area groups for supported 4/6/8-point layouts."""
    if not isinstance(points, Mapping):
        return []
    count = len(points)
    if count not in {4, 6, 8}:
        return []
    required = [f"P{index}" for index in range(1, count + 1)]
    if any(point_id not in points for point_id in required):
        return []
    if count == 4:
        group_ids = (required[:4],)
    elif count == 6:
        group_ids = (required[:4], required[2:6])
    else:
        group_ids = (required[:4], required[4:8])
    return [[points[point_id] for point_id in ids] for ids in group_ids]


def _radar_metrics(board: Any) -> Dict[str, float] | None:
    if not isinstance(board, Mapping):
        return None
    try:
        metrics = {key: float(board[key]) for key in METRIC_KEYS}
    except (KeyError, TypeError, ValueError):
        return None
    return metrics if all(isfinite(value) for value in metrics.values()) else None


def _camera_world_corners(round_data: Mapping[str, Any]) -> Mapping[str, Any]:
    shell = round_data.get("lab_corner_shell")
    if isinstance(shell, Mapping):
        nested = shell.get("camera_world_corners")
        if isinstance(nested, Mapping) and nested:
            return nested
    direct = round_data.get("camera_world_corners")
    return direct if isinstance(direct, Mapping) else {}


def build_board_data(round_data: Mapping[str, Any]) -> List[Dict[str, Any]]:
    """Return exactly two display rows without mutating or filling source data."""
    source = round_data if isinstance(round_data, Mapping) else {}
    radar_result = source.get("radar_result")
    radar_boards = radar_result.get("boards") if isinstance(radar_result, Mapping) else []
    radar_boards = radar_boards if isinstance(radar_boards, (list, tuple)) else []

    camera_metrics: List[Dict[str, float] | None] = []
    for group in camera_corner_groups(_camera_world_corners(source)):
        try:
            camera_metrics.append(compute_corner_metrics(group))
        except (TypeError, ValueError, IndexError):
            camera_metrics.append(None)

    result: List[Dict[str, Any]] = []
    for index, title in enumerate(("第一块板", "第二块板")):
        result.append({
            "title": title,
            "radar": _radar_metrics(radar_boards[index]) if index < len(radar_boards) else None,
            "camera": camera_metrics[index] if index < len(camera_metrics) else None,
        })
    return result
