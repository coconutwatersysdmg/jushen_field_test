# -*- coding: utf-8 -*-
"""现场离线 RGB-D 相机标定变换服务。"""
from __future__ import annotations

from pathlib import Path

from algorithm_modules.field_offline.camera_transform import CameraWorldTransform


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_EXTRINSIC = PROJECT_ROOT / "config" / "camera_extrinsic.json"


class FieldCameraTransformService:
    def __init__(self, extrinsic_path: str | Path | None = None) -> None:
        self.extrinsic_path = Path(extrinsic_path) if extrinsic_path else DEFAULT_EXTRINSIC
        self._transform: CameraWorldTransform | None = None

    @property
    def transform(self) -> CameraWorldTransform:
        if self._transform is None:
            if not self.extrinsic_path.is_file():
                raise FileNotFoundError(f"现场离线相机外参不存在：{self.extrinsic_path}")
            self._transform = CameraWorldTransform.from_json(self.extrinsic_path)
        return self._transform
