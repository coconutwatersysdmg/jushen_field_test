# -*- coding: utf-8 -*-
from __future__ import annotations
from copy import deepcopy

from services.pallet_hole_recognition_service import PalletHoleRecognitionService
from services.pallet_cargo_offset_service import PalletCargoOffsetService
from services.point_cloud_processing_service import PointCloudProcessingService
from services.corner_recognition_service import CornerRecognitionService
from services.lab_camera_adapter import LabCameraCornerService
from services.traditional_fork_hole_service import TraditionalForkHoleService


class AlgorithmFacade:
    """已接入算法的统一门面。

    注意：雷达 point_cloud 的 board_mode / loading_plan 不再作为最终装载规划依据；
    它们即使由旧模块返回，也只保存在原始结果中。最终板型和区域由相机 WORLD 角点计算。

    现场离线算法固定使用 PointNet 点云处理、角点识别和传统 RGB-D 插孔识别。
    """
    def __init__(self):
        self.pallet_hole = PalletHoleRecognitionService()
        self.offset = PalletCargoOffsetService()
        self.point_cloud = PointCloudProcessingService()
        self.corner = CornerRecognitionService()
        self._camera_transform: LabCameraCornerService | None = None
        self._traditional_fork_hole: TraditionalForkHoleService | None = None

    def _field_camera_transform_service(self) -> LabCameraCornerService:
        if self._camera_transform is None:
            self._camera_transform = LabCameraCornerService()
        return self._camera_transform

    def _traditional_fork_hole_service(self) -> TraditionalForkHoleService:
        if self._traditional_fork_hole is None:
            self._traditional_fork_hole = TraditionalForkHoleService()
        return self._traditional_fork_hole

    def pre_pick_offset(self, image_path: str, cargo: dict):
        return self.offset.detect(image_path, phase="pre_pick", cargo=cargo, result_tag=cargo.get("instance_id", "cargo"))

    def post_place_offset(self, image_path: str, cargo: dict):
        return self.offset.detect(image_path, phase="post_place", cargo=cargo, result_tag=cargo.get("instance_id", "cargo"))

    def pallet_hole_recognize(self, rgb_path: str, depth_path: str, cargo: dict):
        return self.pallet_hole.recognize(rgb_path, depth_path, result_tag=cargo.get("instance_id", "cargo"))

    def radar_process(self, locate_result: dict, cargo: dict):
        if locate_result.get("pcd_path"):
            result = self.point_cloud.process_file(locate_result["pcd_path"], result_tag=cargo.get("instance_id", "cargo"))
            result["radar_raw_board_analysis_reference_only"] = {
                "board_mode": result.get("board_mode"),
                "boards": deepcopy(result.get("boards")),
                "first_workface_loading_plan": deepcopy(result.get("first_workface_loading_plan")),
            }
            return result
        return deepcopy(locate_result)

    def corner_image_recognize(self, image_paths_by_point: dict, corner_ids: list[str], cargo: dict):
        return self.corner.recognize_visual_only(
            image_source=image_paths_by_point,
            expected_corner_ids=corner_ids,
            result_tag=cargo.get("instance_id", "cargo"),
        )

    def field_pallet_hole_recognize(self, capture: dict, cargo: dict):
        """现场离线插孔识别：传统 OpenCV RGB-D，输出相机坐标系 XYZ(mm)。"""
        return self._traditional_fork_hole_service().recognize_capture(capture)

    def field_camera_transform(self):
        """返回现场离线 RGB-D 相机到 WORLD 的标定变换。"""
        return self._field_camera_transform_service().transform
