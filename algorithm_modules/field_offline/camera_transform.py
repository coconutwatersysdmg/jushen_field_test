# -*- coding: utf-8 -*-
"""现场离线 RGB-D 相机坐标到 WORLD 坐标的标定变换。"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Mapping, Sequence


class CameraWorldTransform:
    def __init__(
        self,
        t_e_from_c: Sequence[Sequence[float]],
        plc_reference_r_deg: float = -80.0,
        r_axis_center_offset_mm: Sequence[float] = (104.0, 166.0, 270.0),
    ) -> None:
        if len(t_e_from_c) != 4 or any(len(row) != 4 for row in t_e_from_c):
            raise ValueError("T_E<-C must be a 4x4 matrix")
        if len(r_axis_center_offset_mm) != 3:
            raise ValueError("R-axis center offset must contain x, y and z")
        self._t_e_from_c = tuple(tuple(float(value) for value in row) for row in t_e_from_c)
        self._plc_reference_r_deg = float(plc_reference_r_deg)
        self._r_axis_center_offset_mm = tuple(float(value) for value in r_axis_center_offset_mm)

    @classmethod
    def from_json(cls, path: str | Path) -> "CameraWorldTransform":
        with Path(path).open("r", encoding="utf-8") as handle:
            config = json.load(handle)
        return cls(
            config["t_e_from_c"],
            config["plc_reference_r_deg"],
            config["r_axis_center_offset_mm"],
        )

    def world_pose_from_plc(self, plc_pose: Mapping[str, float]):
        try:
            x, y, z, r = (float(plc_pose[key]) for key in ("x", "y", "z", "r"))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("PLC pose requires numeric x, y, z and r") from exc
        theta = math.radians(r - self._plc_reference_r_deg)
        cosine, sine = math.cos(theta), math.sin(theta)
        center_x, center_y, center_z = self._r_axis_center_offset_mm
        return (
            (cosine, -sine, 0.0, x + center_x),
            (sine, cosine, 0.0, y + center_y),
            (0.0, 0.0, 1.0, z + center_z),
            (0.0, 0.0, 0.0, 1.0),
        )

    def camera_pose_from_plc(self, plc_pose: Mapping[str, float]):
        return _matmul4(self.world_pose_from_plc(plc_pose), self._t_e_from_c)

    def plc_xy_for_camera_axis_target(
        self,
        target_world_xyz: Sequence[float],
        plc_pose: Mapping[str, float],
    ) -> dict[str, float]:
        if len(target_world_xyz) != 3:
            raise ValueError("WORLD target must contain x, y and z")
        target = tuple(float(value) for value in target_world_xyz)
        if not all(math.isfinite(value) for value in target):
            raise ValueError("WORLD target contains a non-finite value")
        pose = {"x": 0.0, "y": 0.0, "z": plc_pose["z"], "r": plc_pose["r"]}
        world_from_camera = self.camera_pose_from_plc(pose)
        origin = tuple(float(world_from_camera[row][3]) for row in range(3))
        axis = tuple(float(world_from_camera[row][2]) for row in range(3))
        if abs(axis[2]) < 1e-9:
            raise ValueError("相机光轴 Z 分量过小，无法反算 PLC 坐标")
        distance = (target[2] - origin[2]) / axis[2]
        if distance <= 0:
            raise ValueError("目标位于相机光轴后方，无法反算 PLC 坐标")
        command = {
            "X": target[0] - distance * axis[0] - origin[0],
            "Y": target[1] - distance * axis[1] - origin[1],
        }
        if not all(math.isfinite(value) for value in command.values()):
            raise ValueError("相机光轴对准反算得到非有限 PLC 坐标")
        return command

    def camera_to_world(self, point_c: Sequence[float], plc_pose: Mapping[str, float]):
        if len(point_c) != 3:
            raise ValueError("camera point must contain x, y and z")
        point_end = _transform_point(self._t_e_from_c, point_c)
        return _transform_point(self.world_pose_from_plc(plc_pose), point_end)


def _transform_point(matrix, point: Sequence[float]):
    x, y, z = (float(value) for value in point)
    return tuple(
        matrix[row][0] * x
        + matrix[row][1] * y
        + matrix[row][2] * z
        + matrix[row][3]
        for row in range(3)
    )


def _matmul4(left, right):
    return [
        [sum(float(left[i][k]) * float(right[k][j]) for k in range(4)) for j in range(4)]
        for i in range(4)
    ]
