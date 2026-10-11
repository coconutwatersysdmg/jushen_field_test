# -*- coding: utf-8 -*-
"""Copy resolved offline inputs onto the laptop for algo testing."""
from __future__ import annotations

import shutil
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List

from config.offline_paths import PROJECT_ROOT
from services.offline_pipeline import list_hik_corner_images, resolve_path

CACHE_ROOT = PROJECT_ROOT / "workdir" / "algo_inputs"
RADAR_DIR = CACHE_ROOT / "radar"
HIK_DIR = CACHE_ROOT / "hik_corner"
MECH_DIR = CACHE_ROOT / "mech_hole"


def _ensure_dirs() -> None:
    for folder in (RADAR_DIR, HIK_DIR, MECH_DIR):
        folder.mkdir(parents=True, exist_ok=True)


def _copy_one(src: Path, dest_dir: Path) -> str:
    dest_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    dest = dest_dir / f"{stamp}_{src.name}"
    if dest.exists():
        return str(dest)
    shutil.copy2(str(src), str(dest))
    return str(dest)


def cache_algo_inputs(cfg: Dict[str, Any] | None = None) -> Dict[str, Any]:
    """Resolve offline_paths and copy readable files into workdir/algo_inputs."""
    from config.offline_paths import get_offline_paths

    data = dict(cfg or get_offline_paths())
    _ensure_dirs()
    saved: Dict[str, List[str]] = {"radar": [], "hik_corner": [], "mech_hole": []}
    errors: List[str] = []

    raw_pcd = str(data.get("radar_pcd") or "").strip()
    if raw_pcd:
        try:
            pcd = resolve_path(raw_pcd, kind="radar_pcd")
            saved["radar"].append(_copy_one(pcd, RADAR_DIR))
        except Exception as exc:
            errors.append(f"radar: {exc}")

    try:
        for img in list_hik_corner_images(data):
            saved["hik_corner"].append(_copy_one(img, HIK_DIR))
    except Exception as exc:
        errors.append(f"hik_corner: {exc}")

    for key in ("mech_rgb", "mech_depth"):
        raw = str(data.get(key) or "").strip()
        if not raw:
            continue
        try:
            path = resolve_path(raw, kind=key)
            saved["mech_hole"].append(_copy_one(path, MECH_DIR))
        except Exception as exc:
            errors.append(f"{key}: {exc}")

    return {
        "cache_root": str(CACHE_ROOT),
        "saved": saved,
        "errors": errors,
        "ok": not errors or bool(saved["radar"] or saved["hik_corner"] or saved["mech_hole"]),
    }
