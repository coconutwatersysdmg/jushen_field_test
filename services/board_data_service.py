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
    tilt = 0.0 if horizontal < 1e-9 else degrees(atan2(dz, horizontal))
    offset = 0.0 if horizontal < 1e-9 else degrees(atan2(dx, -dy))
    return {
        "length_mm": (_distance(p1, p3) + _distance(p2, p4)) / 2.0,
        "width_mm": (_distance(p1, p2) + _distance(p3, p4)) / 2.0,
        "height_mean_mm": (p1[2] + p2[2] + p3[2] + p4[2]) / 4.0,
        "tilt_angle_deg": tilt,
        "offset_angle_deg": offset,
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


def _point_number(point_id: Any) -> int | None:
    text = str(point_id).upper()
    if not text.startswith("P") or not text[1:].isdigit():
        return None
    number = int(text[1:])
    return number if 1 <= number <= 8 else None


def _ordered_points(points: Mapping[str, Any]) -> Dict[str, Any]:
    valid = []
    for point_id, value in points.items():
        number = _point_number(point_id)
        if number is not None:
            valid.append((number, f"P{number}", value))
    return {point_id: value for _number, point_id, value in sorted(valid)}


def _board_points(board: Any) -> Dict[str, Any]:
    if not isinstance(board, Mapping):
        return {}
    world_points = board.get("world_points")
    if isinstance(world_points, Mapping) and world_points:
        ids = board.get("corner_ids")
        if isinstance(ids, (list, tuple)):
            selected = {str(point_id): world_points[point_id] for point_id in ids if point_id in world_points}
            if selected:
                return _ordered_points(selected)
        return _ordered_points(world_points)
    ids = board.get("corner_ids")
    coordinates = board.get("corners_xyz_mm")
    if isinstance(ids, (list, tuple)) and isinstance(coordinates, (list, tuple)):
        return _ordered_points({str(point_id): value for point_id, value in zip(ids, coordinates)})
    return {}


def _transform_xyz(value: Any, matrix: Any) -> Dict[str, float]:
    x, y, z = _xyz(value)
    if matrix is None or len(matrix) != 4 or any(len(row) != 4 for row in matrix):
        raise ValueError("lidar WORLD transform must be 4x4")
    source = (x, y, z, 1.0)
    result = [sum(float(matrix[row][column]) * source[column] for column in range(4)) for row in range(4)]
    if not all(isfinite(item) for item in result) or abs(result[3]) < 1e-12:
        raise ValueError("invalid lidar WORLD transform")
    return {
        "x": result[0] / result[3],
        "y": result[1] / result[3],
        "z": result[2] / result[3],
    }


def _fixed_xyz(value: Any) -> Dict[str, float]:
    x, y, z = _xyz(value)
    return {"x": x, "y": y, "z": z}


def _radar_fixed_world_data(
    round_data: Mapping[str, Any],
    lidar_world_matrix: Any,
) -> Tuple[Dict[str, Dict[str, float]], List[List[Dict[str, float]]], bool]:
    """Return flattened points, board groups, and whether native radar geometry was present."""
    radar_result = round_data.get("radar_result")
    if not isinstance(radar_result, Mapping):
        return {}, [], False

    raw_reference = radar_result.get("raw_radar_reference")
    raw_boards = raw_reference.get("boards") if isinstance(raw_reference, Mapping) else None
    if isinstance(raw_boards, (list, tuple)) and any(_board_points(board) for board in raw_boards):
        board_sources = [_board_points(board) for board in raw_boards]
        native_geometry = True
    else:
        boards = radar_result.get("boards")
        board_sources = [_board_points(board) for board in boards] if isinstance(boards, (list, tuple)) else []
        board_sources = [points for points in board_sources if points]
        native_geometry = str(radar_result.get("coordinate_frame") or "world").lower() in {
            "radar", "lidar", "sensor", "radar_world", "lidar_world",
        }
        if not board_sources:
            top_level = radar_result.get("world_points")
            points = _ordered_points(top_level) if isinstance(top_level, Mapping) else {}
            if points:
                count = len(points)
                if count == 4:
                    group_ids = (("P1", "P2", "P3", "P4"),)
                elif count == 6:
                    group_ids = (("P1", "P2", "P3", "P4"), ("P3", "P4", "P5", "P6"))
                elif count == 8:
                    group_ids = (("P1", "P2", "P3", "P4"), ("P5", "P6", "P7", "P8"))
                else:
                    group_ids = ()
                board_sources = [{point_id: points[point_id] for point_id in ids} for ids in group_ids]
                if not board_sources:
                    board_sources = [points]

    if not board_sources:
        return {}, [], native_geometry
    if native_geometry and lidar_world_matrix is None:
        return {}, [], True

    flattened: Dict[str, Dict[str, float]] = {}
    groups: List[List[Dict[str, float]]] = []
    try:
        for source in board_sources:
            converted: List[Dict[str, float]] = []
            for point_id, value in source.items():
                point = _transform_xyz(value, lidar_world_matrix) if native_geometry else _fixed_xyz(value)
                flattened[point_id] = point
                converted.append(point)
            if len(converted) == 4:
                groups.append(converted)
    except (TypeError, ValueError, IndexError, KeyError):
        return {}, [], native_geometry
    return flattened, groups, native_geometry


def build_corner_comparison(
    round_data: Mapping[str, Any],
    lidar_world_matrix: Any = None,
) -> List[Dict[str, Any]]:
    """Return radar/camera corner rows in fixed WORLD coordinates."""
    source = round_data if isinstance(round_data, Mapping) else {}
    radar_points, _groups, _native = _radar_fixed_world_data(source, lidar_world_matrix)
    camera_points: Dict[str, Dict[str, float]] = {}
    for point_id, value in _ordered_points(_camera_world_corners(source)).items():
        try:
            camera_points[point_id] = _fixed_xyz(value)
        except (TypeError, ValueError, IndexError):
            continue
    point_ids = sorted(set(radar_points) | set(camera_points), key=lambda item: int(item[1:]))
    return [
        {"point_id": point_id, "radar": radar_points.get(point_id), "camera": camera_points.get(point_id)}
        for point_id in point_ids
    ]


def build_board_data(
    round_data: Mapping[str, Any],
    lidar_world_matrix: Any = None,
) -> List[Dict[str, Any]]:
    """Return exactly two display rows without mutating or filling source data."""
    source = round_data if isinstance(round_data, Mapping) else {}
    radar_result = source.get("radar_result")
    radar_boards = radar_result.get("boards") if isinstance(radar_result, Mapping) else []
    radar_boards = radar_boards if isinstance(radar_boards, (list, tuple)) else []
    _radar_points, radar_groups, native_geometry = _radar_fixed_world_data(source, lidar_world_matrix)
    transformed_radar_metrics: List[Dict[str, float] | None] = []
    for group in radar_groups:
        try:
            transformed_radar_metrics.append(compute_corner_metrics(group))
        except (TypeError, ValueError, IndexError):
            transformed_radar_metrics.append(None)

    camera_metrics: List[Dict[str, float] | None] = []
    for group in camera_corner_groups(_camera_world_corners(source)):
        try:
            camera_metrics.append(compute_corner_metrics(group))
        except (TypeError, ValueError, IndexError):
            camera_metrics.append(None)

    result: List[Dict[str, Any]] = []
    for index, title in enumerate(("第一块板", "第二块板")):
        if radar_groups:
            radar = transformed_radar_metrics[index] if index < len(transformed_radar_metrics) else None
        elif native_geometry:
            radar = None
        else:
            radar = _radar_metrics(radar_boards[index]) if index < len(radar_boards) else None
        result.append({
            "title": title,
            "radar": radar,
            "camera": camera_metrics[index] if index < len(camera_metrics) else None,
        })
    return result
