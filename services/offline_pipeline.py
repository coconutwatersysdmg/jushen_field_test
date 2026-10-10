# -*- coding: utf-8 -*-
"""离线联调流水线：读路径 → 现场雷达算法 → 现场角点 YOLO。"""
from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

from config.offline_paths import get_offline_paths
from services.field_algo_facade import FieldAlgoFacade

PROJECT_ROOT = Path(__file__).resolve().parents[1]
RESULT_ROOT = PROJECT_ROOT / "workdir" / "field_test_results"
IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def _newest_file(path: Path, suffixes: set[str]) -> Optional[Path]:
    if path.is_file():
        return path
    if not path.is_dir():
        return None
    files = [p for p in path.iterdir() if p.is_file() and p.suffix.lower() in suffixes]
    if not files:
        return None
    return max(files, key=lambda p: p.stat().st_mtime)


def resolve_path(raw: str, *, kind: str) -> Path:
    text = str(raw or "").strip()
    if not text:
        raise FileNotFoundError(f"未配置路径：{kind}（请改 config/offline_paths.py 里的 TODO）")
    path = Path(text).expanduser()
    if not path.is_absolute():
        path = (PROJECT_ROOT / path).resolve()
    if kind == "radar_pcd":
        found = _newest_file(path, {".pcd"}) if path.is_dir() else path
        if found is None or not found.is_file():
            raise FileNotFoundError(f"找不到雷达 PCD：{path}")
        return found
    if kind in {"mech_rgb", "mech_depth", "hik_corner_image"} or kind.startswith("hik_"):
        found = _newest_file(path, IMAGE_EXTS) if path.is_dir() else path
        if found is None or not found.is_file():
            raise FileNotFoundError(f"找不到图像：{kind} -> {path}")
        return found
    if kind == "mech_cloud":
        if path.is_dir():
            found = _newest_file(path, {".pcd", ".ply"})
            if found is None:
                found = _newest_file(path, IMAGE_EXTS)
            if found is None or not found.is_file():
                raise FileNotFoundError(f"找不到梅卡点云：{path}")
            return found
        if not path.is_file():
            raise FileNotFoundError(f"找不到梅卡点云：{path}")
        return path
    if not path.exists():
        raise FileNotFoundError(f"路径不存在：{kind} -> {path}")
    return path


def build_corner_image_map(cfg: Dict[str, Any], corner_ids: List[str]) -> Dict[str, str]:
    by_point = dict(cfg.get("hik_corner_images_by_point") or {})
    filled = {
        str(pid): str(by_point.get(pid) or "").strip()
        for pid in corner_ids
        if str(by_point.get(pid) or "").strip()
    }
    if len(filled) == len(corner_ids):
        return {pid: str(resolve_path(p, kind=f"hik_{pid}")) for pid, p in filled.items()}

    single = str(cfg.get("hik_corner_image") or "").strip()
    if not single:
        raise FileNotFoundError(
            "未配置海康角点图：请填写 hik_corner_image，或按 P1..P4 填满 hik_corner_images_by_point"
        )
    image = resolve_path(single, kind="hik_corner_image")
    return {pid: str(image) for pid in corner_ids}


class OfflinePipeline:
    def __init__(self) -> None:
        self.algo = FieldAlgoFacade()

    def run(
        self,
        *,
        paths: Dict[str, Any] | None = None,
        run_radar: bool = True,
        run_corner: bool = True,
    ) -> Dict[str, Any]:
        cfg = paths or get_offline_paths()
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_dir = RESULT_ROOT / stamp
        out_dir.mkdir(parents=True, exist_ok=True)

        report: Dict[str, Any] = {
            "success": False,
            "started_at": stamp,
            "result_dir": str(out_dir),
            "radar": None,
            "corner": None,
            "message": "",
        }

        corner_ids = ["P1", "P2", "P3", "P4"]
        if run_radar:
            pcd = resolve_path(str(cfg.get("radar_pcd") or ""), kind="radar_pcd")
            radar = self.algo.radar_process(str(pcd), result_tag=f"radar_{stamp}")
            report["radar"] = radar
            report["inputs"] = {"radar_pcd": str(pcd)}
            ids = [str(x) for x in (radar.get("corner_ids") or [])]
            if len(ids) >= 4:
                corner_ids = ids[:6] if len(ids) >= 6 else ids[:4]

        if run_corner:
            image_map = build_corner_image_map(cfg, corner_ids)
            corner = self.algo.corner_image_recognize(
                image_map, corner_ids, result_tag=f"corner_{stamp}"
            )
            report["corner"] = corner
            report.setdefault("inputs", {})["corner_images"] = image_map

        # 可选记录梅卡离线文件是否配置（本 V1 不强制跑孔位算法）
        mech = {}
        for key in ("mech_rgb", "mech_depth", "mech_cloud"):
            raw = str(cfg.get(key) or "").strip()
            if raw:
                try:
                    mech[key] = str(resolve_path(raw, kind=key))
                except Exception as exc:
                    mech[key] = f"未找到: {exc}"
            else:
                mech[key] = "TODO 未配置"
        report["mech_files"] = mech

        radar_ok = (not run_radar) or bool((report.get("radar") or {}).get("success"))
        corner_ok = (not run_corner) or bool((report.get("corner") or {}).get("success"))
        report["success"] = bool(radar_ok and corner_ok)
        if report["success"]:
            report["message"] = "离线现场算法完成"
        else:
            bits = []
            if run_radar and not radar_ok:
                bits.append(f"雷达失败: {(report.get('radar') or {}).get('message')}")
            if run_corner and not corner_ok:
                bits.append(f"角点失败: {(report.get('corner') or {}).get('message')}")
            report["message"] = "；".join(bits) or "未执行任何步骤"

        (out_dir / "report.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        return report
