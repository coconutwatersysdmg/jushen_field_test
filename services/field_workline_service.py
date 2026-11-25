# -*- coding: utf-8 -*-
"""现场作业工作区的数据通道（第一阶段：只联调，不接真机、不调算法）。

设计边界：
- 轮次从装载计划 quantity 展开，和 FlowController.set_plan 的规则一致；
- 角点只认雷达粗识别的 P1~P4，坐标由软件算好后交给 PLC；
- 当前没有 PLC 接口文件，指令先导出 JSON，等接口到位只替换 export_corner_scan_command 的投递方式；
- 算法位不造假结果，只能由用户手动载入一份结果 JSON 做界面联调；
- 图片和结果统一归档到 workdir/field_runs/<日期>/<车号>/<工作区>_<时间戳>/。
"""
from __future__ import annotations

import json
import re
import shutil
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Any, Mapping

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PLAN_PATH = PROJECT_ROOT / "data" / "loading_plan.json"
DEFAULT_RUN_ROOT = PROJECT_ROOT / "workdir" / "field_runs"
EXAMPLES_ROOT = PROJECT_ROOT / "examples"

CORNER_IDS = ("P1", "P2", "P3", "P4")

# 仅用于"用演示坐标"按钮，界面会明确标注 DEMO_PLACEHOLDER，绝不默默当真值。
DEMO_CORNERS_MM = {
    "P1": {"x": -3500.0, "y": 1000.0, "z": 1800.0},
    "P2": {"x": -3500.0, "y": 2200.0, "z": 1800.0},
    "P3": {"x": -1100.0, "y": 2200.0, "z": 1800.0},
    "P4": {"x": -1100.0, "y": 1000.0, "z": 1800.0},
}

EXAMPLE_IMAGES = {
    "corner": [f"demo_auto/P{i}.jpg" for i in range(1, 5)],
    "corner_result": ["corner_net/20251225_125040_525_P4.jpg"],
    "pallet": ["pallet_rgbd/pallet1.jpg"],
    "post_place": ["dynamic_monitoring/color.png"],
}

EXAMPLE_RESULTS = {
    "radar": "examples/module_outputs/POINTNET_TRUCK.json",
    "corner": "examples/module_outputs/CORNER_YOLO.json",
    "pallet_hole": "examples/module_outputs/PALLET_HOLE_YOLO.json",
    "dynamic_pre": "examples/module_outputs/DYNAMIC_PRE_PLACE.json",
    "dynamic_post": "examples/module_outputs/DYNAMIC_POST_PLACE.json",
}


def _stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")


def _date_tag() -> str:
    return datetime.now().strftime("%Y%m%d")


def _safe(value: Any, default: str = "NA") -> str:
    text = re.sub(r"[^0-9A-Za-z._\-\u4e00-\u9fff]+", "_", str(value or "").strip())
    return text.strip("._-") or default


def _as_xyz(value: Any) -> dict[str, float] | None:
    if isinstance(value, Mapping):
        keys = ("x", "x_mm", "X")
        keys_y = ("y", "y_mm", "Y")
        keys_z = ("z", "z_mm", "Z")
        x = next((value[k] for k in keys if k in value), None)
        y = next((value[k] for k in keys_y if k in value), None)
        z = next((value[k] for k in keys_z if k in value), None)
    elif isinstance(value, (list, tuple)) and len(value) >= 3:
        x, y, z = value[0], value[1], value[2]
    else:
        return None
    try:
        return {"x": float(x), "y": float(y), "z": float(z)}
    except (TypeError, ValueError):
        return None


def _write_json(path: Path, payload: Any) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, default=str),
        encoding="utf-8",
    )
    return str(path.resolve())


class FieldWorklineService:
    """现场两条工作区共用的纯数据服务；不依赖 Qt，可在测试里直接用。"""

    def __init__(
        self,
        plan_path: str | Path | None = None,
        run_root: str | Path | None = None,
    ):
        self.plan_path = Path(plan_path or DEFAULT_PLAN_PATH).expanduser().resolve()
        self.run_root = Path(run_root or DEFAULT_RUN_ROOT).expanduser().resolve()
        self.vehicle_id = "FIELD-VEHICLE-001"
        self.session_dir: Path | None = None
        self.session_info: dict[str, Any] = {}

    # ------------------------------------------------------------------
    # 装载计划：quantity 展开成一件一件，循环次数由此决定
    # ------------------------------------------------------------------
    def load_plan(self, path: str | Path | None = None) -> dict[str, Any]:
        source = Path(path or self.plan_path).expanduser().resolve()
        if not source.is_file():
            raise FileNotFoundError(f"装载计划不存在：{source}")
        data = json.loads(source.read_text(encoding="utf-8"))
        items = data.get("items", data) if isinstance(data, dict) else data
        if not isinstance(items, list):
            raise ValueError(f"装载计划格式不对（需要 items 数组）：{source}")

        queue: list[dict[str, Any]] = []
        sequence = 0
        for item in items:
            if not isinstance(item, Mapping):
                continue
            quantity = max(1, int(item.get("quantity", 1) or 1))
            for unit in range(1, quantity + 1):
                sequence += 1
                cargo = deepcopy(dict(item))
                cargo["unit_index"] = unit
                cargo["sequence"] = sequence
                cargo["instance_id"] = (
                    f"{cargo.get('cargo_code', 'CARGO')}-{unit:03d}-{sequence:03d}"
                )
                queue.append(cargo)
        if not queue:
            raise ValueError(f"装载计划里没有可展开的货物：{source}")
        self.plan_path = source
        return {
            "path": str(source),
            "items": list(items),
            "queue": queue,
            "total_rounds": len(queue),
        }

    # ------------------------------------------------------------------
    # 归档会话
    # ------------------------------------------------------------------
    def start_session(self, workline: str, vehicle_id: str | None = None) -> dict[str, Any]:
        if vehicle_id:
            self.vehicle_id = str(vehicle_id).strip() or self.vehicle_id
        folder = (
            self.run_root
            / _date_tag()
            / _safe(self.vehicle_id)
            / f"{_safe(workline, 'workline')}_{_stamp()}"
        )
        (folder / "images").mkdir(parents=True, exist_ok=True)
        (folder / "json").mkdir(parents=True, exist_ok=True)
        self.session_dir = folder
        self.session_info = {
            "workline": workline,
            "vehicle_id": self.vehicle_id,
            "session_dir": str(folder),
            "created_at": datetime.now().isoformat(timespec="milliseconds"),
            "schema_version": "field-workline/1.0",
            "interface_status": "JSON_EXPORT_PENDING_PLC_INTERFACE",
            "steps": [],
            "images": [],
            "results": [],
        }
        self._write_manifest()
        return dict(self.session_info)

    def ensure_session(self, workline: str) -> Path:
        if self.session_dir is None or not self.session_dir.is_dir():
            return Path(self.start_session(workline)["session_dir"])
        return self.session_dir

    def _write_manifest(self) -> str:
        if self.session_dir is None:
            return ""
        return _write_json(self.session_dir / "manifest.json", self.session_info)

    def record_step(
        self,
        workline: str,
        step_code: str,
        step_name: str,
        status: str,
        payload: Any = None,
    ) -> str:
        self.ensure_session(workline)
        entry = {
            "step_code": step_code,
            "step_name": step_name,
            "status": status,
            "at": datetime.now().isoformat(timespec="milliseconds"),
            "payload": payload,
        }
        self.session_info.setdefault("steps", []).append(entry)
        self._write_manifest()
        return _write_json(
            self.session_dir / "json" / f"{_safe(step_code)}_{len(self.session_info['steps']):02d}_{_stamp()}.json",
            entry,
        )

    def export_result(self, workline: str, name: str, payload: Any) -> str:
        self.ensure_session(workline)
        path = self.session_dir / "json" / f"{_safe(name)}_{_stamp()}.json"
        written = _write_json(path, payload)
        self.session_info.setdefault("results", []).append(written)
        self._write_manifest()
        return written

    def archive_image(self, workline: str, source: str | Path, tag: str) -> str:
        src = Path(source).expanduser()
        if not src.is_file():
            raise FileNotFoundError(f"图片不存在：{src}")
        session = self.ensure_session(workline)
        suffix = src.suffix.lower() or ".jpg"
        target = session / "images" / f"{_safe(tag)}_{_stamp()}{suffix}"
        shutil.copy2(src, target)
        self.session_info.setdefault("images", []).append(
            {"tag": tag, "source": str(src.resolve()), "archived": str(target.resolve())}
        )
        self._write_manifest()
        return str(target.resolve())

    def open_session_folder(self) -> str:
        if self.session_dir is None:
            return ""
        return str(self.session_dir.resolve())

    # ------------------------------------------------------------------
    # 角点工作线：雷达粗识别 P1~P4 → 算坐标 → 导出给 PLC 的拍照指令
    # ------------------------------------------------------------------
    def extract_corners(self, radar_result: Mapping[str, Any] | None) -> list[dict[str, Any]]:
        result = dict(radar_result or {})
        world = dict(result.get("world_points") or {})
        if not world and isinstance(result.get("corner_points"), list):
            for item in result["corner_points"]:
                if isinstance(item, Mapping) and item.get("name"):
                    world[str(item["name"])] = item
        points: list[dict[str, Any]] = []
        for pid in CORNER_IDS:
            xyz = _as_xyz(world.get(pid))
            if xyz is None:
                continue
            points.append({
                "point_id": pid,
                "x_mm": xyz["x"],
                "y_mm": xyz["y"],
                "z_mm": xyz["z"],
                "source": "RADAR_COARSE",
            })
        return points

    def demo_corners(self) -> list[dict[str, Any]]:
        return [
            {"point_id": pid, **{f"{axis}_mm": value[axis] for axis in ("x", "y", "z")},
             "source": "DEMO_PLACEHOLDER"}
            for pid, value in DEMO_CORNERS_MM.items()
        ]

    def export_corner_scan_command(
        self,
        points: list[dict[str, Any]],
        vehicle_id: str | None = None,
    ) -> str:
        """当前导出 JSON；PLC 接口文件到位后只改这里的投递方式。"""
        if len(points) != 4:
            raise ValueError(f"角点拍照指令需要 P1~P4 四个点，当前 {len(points)} 个")
        session = self.ensure_session("corner")
        payload = {
            "schema_version": "field-workline/1.0",
            "command_type": "MOVE_CAMERA_TO_CORNER_AND_CAPTURE",
            "status": "INTERFACE_PENDING",
            "transport": "JSON_EXPORT",
            "target": "PLC",
            "note": "接口文件未到位：先导出 JSON 联调；到位后改为直接下发 PLC，PLC 带相机到点位拍照。",
            "vehicle_id": str(vehicle_id or self.vehicle_id),
            "coordinate_frame": "WORLD",
            "coordinate_unit": "mm",
            "created_at": datetime.now().isoformat(timespec="milliseconds"),
            "capture_sequence": [
                {
                    "index": index + 1,
                    "point_id": point["point_id"],
                    "world_xyz_mm": {
                        "x": float(point["x_mm"]),
                        "y": float(point["y_mm"]),
                        "z": float(point["z_mm"]),
                    },
                    "source": point.get("source", "RADAR_COARSE"),
                    "action": "MOVE_AND_CAPTURE",
                }
                for index, point in enumerate(points)
            ],
        }
        written = self.export_result("corner", "corner_scan_cmd", payload)
        self.record_step("corner", "EXPORT_CORNER_CMD", "导出角点拍照指令", "DONE", {"path": written})
        return written

    # ------------------------------------------------------------------
    # 示例文件 / 算法结果 JSON 载入（占位联调）
    # ------------------------------------------------------------------
    def example_images(self, kind: str) -> list[str]:
        paths = []
        for relative in EXAMPLE_IMAGES.get(kind, []):
            candidate = EXAMPLES_ROOT / relative
            if candidate.is_file():
                paths.append(str(candidate))
        return paths

    def example_result_path(self, kind: str) -> str:
        relative = EXAMPLE_RESULTS.get(kind, "")
        candidate = PROJECT_ROOT / relative
        return str(candidate) if candidate.is_file() else ""

    def load_result_json(self, path: str | Path) -> dict[str, Any]:
        source = Path(path).expanduser().resolve()
        if not source.is_file():
            raise FileNotFoundError(f"结果 JSON 不存在：{source}")
        data = json.loads(source.read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise ValueError(f"结果 JSON 顶层需要是对象：{source}")
        return {"path": str(source), "data": data}
