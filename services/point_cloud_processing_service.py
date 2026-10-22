# -*- coding: utf-8 -*-
"""雷达点云 -> 车板几何/高低板/第一作业面规划服务。

该服务直接封装 ``algorithm_modules.point_cloud_segment_module``：
PCD -> PointNet++ 分割 -> 底板平面/四边拟合 -> 3D 角点 -> 尺寸/姿态 ->
两块底板时 label_2 第一作业面剩余空间规划。

算法原始 JSON 会继续留档，但主系统直接消费 Python 字典，不依赖 JSON 回读。
"""
from __future__ import annotations

import json
import os
import re
from datetime import datetime
from pathlib import Path
from utils.measurement_rounding import round_measurement
from typing import Any, Dict, Mapping, Optional

from config.system_config import get_system_config


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RESULT_ROOT = PROJECT_ROOT / "workdir" / "point_cloud_results"
DEFAULT_CHECKPOINT = (
    PROJECT_ROOT
    / "algorithm_modules"
    / "point_cloud_segment_module"
    / "checkpoints"
    / "best_model.pth"
)


class PointCloudProcessingService:
    """持有一次加载的 PointNet++ pipeline，并将其结果转换为系统统一结构。"""

    def __init__(
        self,
        checkpoint_path: Optional[str | Path] = None,
        device: Optional[str] = None,
        result_root: Optional[str | Path] = None,
        config_path: Optional[str | Path] = None,
    ) -> None:
        # config_path 保留仅为兼容旧调用；配置已改为 config/system_config.py
        self.config_path = Path(config_path) if config_path else PROJECT_ROOT / "config" / "system_config.py"
        self.system_config = get_system_config()
        pc_cfg = self.system_config.get("point_cloud", {}) if isinstance(self.system_config, dict) else {}

        configured_checkpoint = checkpoint_path or pc_cfg.get("checkpoint_path") or DEFAULT_CHECKPOINT
        checkpoint = Path(str(configured_checkpoint)).expanduser()
        if not checkpoint.is_absolute():
            checkpoint = (PROJECT_ROOT / checkpoint).resolve()
        self.checkpoint_path = checkpoint

        configured_device = device if device is not None else pc_cfg.get("device")
        self.device = None if configured_device in (None, "", "auto") else str(configured_device)
        self.result_root = Path(result_root or DEFAULT_RESULT_ROOT).expanduser().resolve()
        self.pc_cfg = dict(pc_cfg)
        self._pipeline = None

    @staticmethod
    def _load_json(path: Path) -> Dict[str, Any]:
        try:
            if path.is_file():
                value = json.loads(path.read_text(encoding="utf-8"))
                return value if isinstance(value, dict) else {}
        except Exception:
            pass
        return {}

    @staticmethod
    def _safe_tag(value: str) -> str:
        text = re.sub(r"[^0-9A-Za-z._-]+", "_", str(value or "").strip())
        return text.strip("._-") or "point_cloud"

    def _create_result_dir(self, result_tag: str = "") -> Path:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        folder = self.result_root / f"{self._safe_tag(result_tag)}_{stamp}"
        folder.mkdir(parents=True, exist_ok=True)
        return folder

    def _ensure_pipeline(self):
        if self._pipeline is not None:
            return self._pipeline
        if not self.checkpoint_path.is_file():
            raise RuntimeError(f"点云 PointNet++ 权重不存在：{self.checkpoint_path}")

        try:
            from algorithm_modules.point_cloud_segment_module.point_cloud_pipeline import (
                PipelineConfig,
                PointCloudPipeline,
            )
        except ImportError as exc:
            raise RuntimeError(
                "无法加载点云处理模块。请确认 numpy、torch、open3d 已安装，"
                "并且 algorithm_modules/point_cloud_segment_module 完整存在。"
            ) from exc

        # 保留算法包自身默认参数，仅把系统配置中明确给出的常用参数覆盖进去。
        supported = {
            "geometry_scheme",
            "pointnet_num_point",
            "pointnet_num_votes",
            "pointnet_context_fraction",
            "corner_height_radii",
            "corner_height_min_points",
            "corner_height_min_tolerance",
            "corner_height_max_tolerance",
            "corner_height_iterations",
            "corner_height_max_slope",
            "single_height_length_fraction",
            "height_scheme",
            "high_height_length_fraction",
            "low_height_length_fraction",
            "high_low_height_width_fraction",
            "single_height_width_fraction",
            "single_height_min_points",
            "single_height_min_tolerance",
            "single_height_max_tolerance",
            "single_height_iterations",
            "single_height_max_slope",
            "voxel_size",
            "plane_threshold",
            "dbscan_eps",
            "dbscan_min_points",
            "pallet_length_mm",
            "pad_trigger_ratio",
        }
        overrides = {k: self.pc_cfg[k] for k in supported if k in self.pc_cfg}
        try:
            config = PipelineConfig(**overrides)
            self._pipeline = PointCloudPipeline(
                config=config,
                checkpoint_path=str(self.checkpoint_path),
                device=self.device,
            )
        except Exception as exc:
            raise RuntimeError(f"初始化雷达点云 PointNet++ 处理模块失败：{exc}") from exc
        return self._pipeline

    @staticmethod
    def _normalize_board(label: str, raw: Mapping[str, Any]) -> Dict[str, Any]:
        corner_ids = [str(x) for x in (raw.get("corner_ids") or [])]
        corners = raw.get("corners_xyz_mm") or []
        estimation = raw.get('height_estimation')
        withheld_height = (isinstance(estimation, Mapping) and estimation.get('status') != 'supported'
                           and len(corners) == 0 and len(corner_ids) == 4)
        if not withheld_height and (len(corner_ids) != len(corners) or len(corner_ids) != 4):
            raise RuntimeError(f"{label} 角点输出不完整：corner_ids={corner_ids}, corners={len(corners)}")
        normalized_corners = []
        world_points: Dict[str, Dict[str, float]] = {}
        for name, point in zip(corner_ids, corners):
            if not isinstance(point, (list, tuple)) or len(point) < 3:
                raise RuntimeError(f"{label}/{name} 的 XYZ 格式无效：{point}")
            xyz = [round_measurement(point[0]), round_measurement(point[1]), round_measurement(point[2])]
            normalized_corners.append(xyz)
            world_points[name] = {"x": xyz[0], "y": xyz[1], "z": xyz[2]}

        result = {
            "board_label": label,
            "coordinate_frame": str(raw.get("coordinate_frame") or "radar_world"),
            "coordinate_unit": "mm",
            "corner_ids": corner_ids,
            "corners_xyz_mm": normalized_corners,
            "world_points": world_points,
            "length_mm": round_measurement(raw.get("length_mm") or 0),
            "width_mm": round_measurement(raw.get("width_mm") or 0),
            "height_mean_mm": round_measurement(raw["height_mean_mm"]) if raw.get("height_mean_mm") is not None else None,
            "tilt_angle_deg": round_measurement(raw.get("tilt_angle_deg") or 0),
            "offset_angle_deg": round_measurement(raw.get("offset_angle_deg") or 0),
        }
        if raw.get("radar_corners_xyz_mm") is not None:
            result["radar_corners_xyz_mm"] = [[round_measurement(v) for v in point] for point in raw["radar_corners_xyz_mm"]]
        for key in ("global_plane_radar_corners_xyz_mm", "corner_height_estimation", "corner_height_all_supported",
                    "geometry_scheme", "geometry_scheme_requested", "geometry_method", "shared_xy_reference",
                    "shared_geometry_quality", "fallback_reason", "geometry_revision", "transition_geometry", "available_shared_xy_reference",
                    "height_estimation", "corner_height_mean_mm", "transition_support",
                    "available_xy_corners_radar_mm", "height_source_radar_corners_xyz_mm",
                    "geometry_plane_height_mean_mm", "height_source_failure"):
            if key in raw:
                result[key] = raw[key]
        if isinstance(raw.get("loading_plan"), Mapping):
            result["loading_plan"] = dict(raw["loading_plan"])
        return result

    @classmethod
    def normalize_result(cls, raw_result: Mapping[str, Any]) -> Dict[str, Any]:
        """将算法的 label_2/label_3 输出转换成主系统统一结构。"""
        geometry_status = raw_result.get('geometry_status')
        geometry_failures = list(raw_result.get('geometry_failures') or [])
        geometry_metadata = ({'geometry_status': geometry_status, 'geometry_failures': geometry_failures}
                             if geometry_status else {})
        if isinstance(raw_result.get('label_correction'), Mapping):
            geometry_metadata['label_correction'] = dict(raw_result['label_correction'])
        failure_details = '; '.join(str(item.get('reason', '')) for item in geometry_failures)
        incomplete = geometry_status in ('partial', 'unsupported')
        boards = []
        for label in ("label_2", "label_3"):
            value = raw_result.get(label)
            if isinstance(value, Mapping):
                boards.append(cls._normalize_board(label, value))

        if not boards:
            return {
                "success": False,
                "message": "点云处理完成，但未成功提取 label_2 / label_3 车板几何" +
                           ("；方案二支撑不足：" + failure_details if incomplete else ""),
                "board_count": 0,
                "boards": [],
                **geometry_metadata,
            }

        corner_ids = []
        corner_points = []
        world_points: Dict[str, Dict[str, float]] = {}
        for board in boards:
            for name, xyz in zip(board["corner_ids"], board["corners_xyz_mm"]):
                corner_ids.append(name)
                point = {
                    "name": name,
                    "x": round_measurement(xyz[0]),
                    "y": round_measurement(xyz[1]),
                    "z": round_measurement(xyz[2]),
                    "board_label": board["board_label"],
                }
                corner_points.append(point)
                world_points[name] = {"x": point["x"], "y": point["y"], "z": point["z"]}

        board_count = len(boards)
        board_mode = "incomplete" if incomplete else ("high_low_board" if board_count >= 2 else "single_board")
        heights = [(b["height_mean_mm"], b["board_label"]) for b in boards if b["height_mean_mm"] is not None]
        # Unsupported single-board H is missing, never zero or the old corner mean.
        if not heights:
            heights = [(None, boards[0]["board_label"])]
        low_height, low_label = min(heights)
        high_height, high_label = max(heights)
        if board_count == 2 and len(heights) != board_count:
            # A missing per-deck H must not collapse high/low labels to one board.
            high_label, low_label = 'label_2', 'label_3'
        height_difference = abs(float(high_height) - float(low_height)) if board_count >= 2 and len(heights) == board_count else (0.0 if board_count == 1 else None)

        first_workface_plan = None
        label2 = next((b for b in boards if b["board_label"] == "label_2"), None)
        if label2 and isinstance(label2.get("loading_plan"), Mapping):
            first_workface_plan = dict(label2["loading_plan"])

        return {
            "success": not incomplete,
            **geometry_metadata,
            **({'message': '方案二几何结果不完整，已保留有效角点供检查，整车板型尚未确认：' + failure_details}
               if incomplete else {}),
            "recognition_source": "pointnet2_point_cloud_pipeline",
            "coordinate_frame": "world" if all(b["coordinate_frame"] == "world" for b in boards) else "radar_world",
            "coordinate_unit": "mm",
            "board_count": board_count,
            "board_mode": board_mode,
            "is_high_low_board": None if incomplete else board_count >= 2,
            "low_board_label": low_label,
            "high_board_label": high_label,
            "height_difference_mm": round_measurement(height_difference) if height_difference is not None and not incomplete else None,
            "corner_ids": corner_ids,
            "corner_points": corner_points,
            "world_points": world_points,
            "corner_points_xyz_mm": [
                [p["x"], p["y"], p["z"]] for p in corner_points
            ],
            # 兼容既有下游字段；本版明确 coordinate_unit=mm。
            "corner_points_xyz": [
                [p["x"], p["y"], p["z"]] for p in corner_points
            ],
            "boards": boards,
            "first_workface_label": "label_2" if first_workface_plan is not None else None,
            "first_workface_loading_plan": first_workface_plan,
        }

    def process_file(self, pcd_path: str | Path, result_tag: str = "") -> Dict[str, Any]:
        path = Path(str(pcd_path or "")).expanduser()
        if not path.is_absolute():
            path = (PROJECT_ROOT / path).resolve()
        if not path.is_file():
            raise RuntimeError(f"雷达 PCD 文件不存在：{path}")
        if path.suffix.lower() != ".pcd":
            raise RuntimeError(f"雷达点云输入必须为 .pcd：{path}")

        output_dir = self._create_result_dir(result_tag or path.stem)
        pipeline = self._ensure_pipeline()
        try:
            record = pipeline.process_file(path, output_dir)
        except Exception as exc:
            raise RuntimeError(f"雷达点云处理失败：{type(exc).__name__}: {exc}") from exc

        normalized = self.normalize_result(record.get("result") or {})
        normalized.update({
            "pcd_path": str(path.resolve()),
            "result_dir": str(output_dir.resolve()),
            "result_json_path": str(Path(record.get("json") or "").resolve()) if record.get("json") else "",
            "checkpoint_path": str(self.checkpoint_path),
            "timing": {
                "preprocess": record.get("preprocess_timing"),
                "pointnet": record.get("pointnet_timing"),
                "geometry_s": record.get("geometry_s"),
                "total_s": record.get("total_s"),
            },
            "stage_counts": record.get("stage_counts"),
            "dense_label_counts": record.get("dense_label_counts"),
            "message": (
                f"雷达点云处理完成：{normalized.get('board_count', 0)} 块车板，"
                f"{len(normalized.get('corner_ids') or [])} 个 3D 角点"
            ) if normalized.get("success") else normalized.get("message"),
        })
        return normalized
