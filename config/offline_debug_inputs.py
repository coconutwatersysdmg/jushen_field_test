# -*- coding: utf-8 -*-
"""把 offline_paths.py 转成 FlowController.set_debug_inputs 所需结构。"""
from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any, Dict

from config.offline_paths import PROJECT_ROOT, get_offline_paths
from services.offline_pipeline import build_corner_image_map, resolve_path


def _abs(raw: str) -> str:
    text = str(raw or "").strip()
    if not text:
        return ""
    path = Path(text).expanduser()
    if not path.is_absolute():
        path = (PROJECT_ROOT / path).resolve()
    return str(path)


def build_debug_inputs_from_offline(base: Dict[str, Any] | None = None) -> Dict[str, Any]:
    """优先用 config/offline_paths.py；空字段保留 base 里的示例（若有）。"""
    out = deepcopy(base or {})
    cfg = get_offline_paths()

    raw_pcd = str(cfg.get("radar_pcd") or "").strip()
    if raw_pcd:
        try:
            out["point_cloud_path"] = str(resolve_path(raw_pcd, kind="radar_pcd"))
            out["point_cloud_example_only"] = False
        except Exception:
            out["point_cloud_path"] = _abs(raw_pcd)
            out["point_cloud_example_only"] = False

    for key, out_key in (("mech_rgb", "pallet_rgb"), ("mech_depth", "pallet_depth")):
        raw = str(cfg.get(key) or "").strip()
        if not raw:
            continue
        try:
            out[out_key] = str(resolve_path(raw, kind=key))
        except Exception:
            out[out_key] = _abs(raw)

    images = dict(out.get("images") or {})
    depths = dict(out.get("depths") or {})
    if out.get("pallet_rgb"):
        images["pallet_hole"] = out["pallet_rgb"]
    if out.get("pallet_depth"):
        depths["pallet_hole"] = out["pallet_depth"]
    out["images"] = images
    out["depths"] = depths

    # 海康：按文件夹扫图，不依赖 P1.jpg 这类固定名
    try:
        corner_images = build_corner_image_map(cfg, ["P1", "P2", "P3", "P4", "P5", "P6"])
    except Exception:
        corner_images = dict(out.get("corner_images") or {})
    out["corner_images"] = corner_images
    out["hik_corner_dir"] = _abs(str(cfg.get("hik_corner_dir") or ""))

    out["offline_field_test"] = True
    raw_cloud = str(cfg.get("mech_cloud") or "").strip()
    if raw_cloud:
        try:
            out["mech_cloud"] = str(resolve_path(raw_cloud, kind="mech_cloud"))
        except Exception:
            out["mech_cloud"] = _abs(raw_cloud)
    else:
        out["mech_cloud"] = ""
    return out
