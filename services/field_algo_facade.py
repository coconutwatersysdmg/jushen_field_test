# -*- coding: utf-8 -*-
"""现场算法门面：仅 PointNet 点云 + 角点 YOLO（与 jushenzhineng_v3 field 一致，不含实验室算法）。"""
from __future__ import annotations

from copy import deepcopy
from typing import Any

from services.corner_recognition_service import CornerRecognitionService
from services.point_cloud_processing_service import PointCloudProcessingService


class FieldAlgoFacade:
    def __init__(self) -> None:
        self.point_cloud = PointCloudProcessingService()
        self.corner = CornerRecognitionService()

    def radar_process(self, pcd_path: str, result_tag: str = "field") -> dict[str, Any]:
        result = self.point_cloud.process_file(pcd_path, result_tag=result_tag)
        result["radar_raw_board_analysis_reference_only"] = {
            "board_mode": result.get("board_mode"),
            "boards": deepcopy(result.get("boards")),
            "first_workface_loading_plan": deepcopy(result.get("first_workface_loading_plan")),
        }
        return result

    def corner_image_recognize(
        self,
        image_paths_by_point: dict,
        corner_ids: list[str],
        result_tag: str = "field",
    ) -> dict[str, Any]:
        return self.corner.recognize_visual_only(
            image_source=image_paths_by_point,
            expected_corner_ids=corner_ids,
            result_tag=result_tag,
        )
