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


def _abs_path(raw: str) -> Path:
    text = str(raw or "").strip()
    if not text:
        raise FileNotFoundError("路径为空（请改 config/offline_paths.py）")
    path = Path(text).expanduser()
    if not path.is_absolute():
        path = (PROJECT_ROOT / path).resolve()
    return path


def list_files(path: Path, suffixes: set[str]) -> List[Path]:
    """List files under a directory (any filename), oldest first by mtime."""
    if path.is_file():
        return [path] if path.suffix.lower() in suffixes else []
    if not path.is_dir():
        return []
    files = [p for p in path.iterdir() if p.is_file() and p.suffix.lower() in suffixes]
    return sorted(files, key=lambda p: (p.stat().st_mtime, p.name.lower()))


def _newest_file(path: Path, suffixes: set[str]) -> Optional[Path]:
    files = list_files(path, suffixes)
    return files[-1] if files else None


def resolve_path(raw: str, *, kind: str) -> Path:
    path = _abs_path(raw)
    if kind == "radar_pcd":
        found = _newest_file(path, {".pcd"}) if path.is_dir() else path
        if found is None or not found.is_file():
            raise FileNotFoundError(f"找不到雷达 PCD：{path}")
        return found
    if kind in {"mech_rgb", "mech_depth", "hik_corner_image", "hik_corner_dir"} or kind.startswith("hik_"):
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


def list_hik_corner_images(cfg: Dict[str, Any]) -> List[Path]:
    """Scan hik_corner_dir for images; filename does not matter."""
    raw = str(cfg.get("hik_corner_dir") or "").strip()
    if not raw:
        # backward compatible: single file / old keys
        single = str(cfg.get("hik_corner_image") or "").strip()
        if single:
            return [resolve_path(single, kind="hik_corner_image")]
        return []
    folder = _abs_path(raw)
    if folder.is_file() and folder.suffix.lower() in IMAGE_EXTS:
        return [folder]
    if not folder.is_dir():
        raise FileNotFoundError(f"海康角点文件夹不存在：{folder}")
    images = list_files(folder, IMAGE_EXTS)
    if not images:
        raise FileNotFoundError(f"海康角点文件夹里没有图片（jpg/png/bmp）：{folder}")
    return images


def build_corner_image_map(cfg: Dict[str, Any], corner_ids: List[str]) -> Dict[str, str]:
    """Map corner ids → image paths without relying on fixed filenames.

    Folder rules:
      - 1 photo  → every corner uses it
      - N photos → assign in mtime order to P1..；不够则循环；多了只用前 N 个角点所需张数
    """
    ids = [str(pid) for pid in corner_ids]
    images = list_hik_corner_images(cfg)
    if not images:
        raise FileNotFoundError(
            "未配置海康角点图：请在 offline_paths.py 填写 hik_corner_dir（文件夹即可）"
        )
    if len(images) == 1:
        return {pid: str(images[0]) for pid in ids}
    out: Dict[str, str] = {}
    for i, pid in enumerate(ids):
        out[pid] = str(images[i % len(images)])
    return out


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
