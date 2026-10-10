# -*- coding: utf-8 -*-
"""把 offline_paths.py 转成 FlowController.set_debug_inputs 所需结构。"""
from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any, Dict

from config.offline_paths import PROJECT_ROOT, get_offline_paths


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

    pcd = _abs(str(cfg.get("radar_pcd") or ""))
    if pcd:
        out["point_cloud_path"] = pcd
        out["point_cloud_example_only"] = False

    rgb = _abs(str(cfg.get("mech_rgb") or ""))
    depth = _abs(str(cfg.get("mech_depth") or ""))
    if rgb:
        out["pallet_rgb"] = rgb
    if depth:
        out["pallet_depth"] = depth

    images = dict(out.get("images") or {})
    depths = dict(out.get("depths") or {})
    if rgb:
        images["pallet_hole"] = rgb
    if depth:
        depths["pallet_hole"] = depth
    out["images"] = images
    out["depths"] = depths

    corner_images = dict(out.get("corner_images") or {})
    by_point = dict(cfg.get("hik_corner_images_by_point") or {})
    filled = {str(k): _abs(str(v)) for k, v in by_point.items() if str(v or "").strip()}
    if filled:
        corner_images.update(filled)
    else:
        single = _abs(str(cfg.get("hik_corner_image") or ""))
        if single:
            for pid in ("P1", "P2", "P3", "P4", "P5", "P6"):
                corner_images[pid] = single
    out["corner_images"] = corner_images

    # 标记：现场离线联调
    out["offline_field_test"] = True
    out["mech_cloud"] = _abs(str(cfg.get("mech_cloud") or ""))
    return out
