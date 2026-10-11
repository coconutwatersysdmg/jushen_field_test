# -*- coding: utf-8 -*-
"""Copy resolved offline inputs onto the laptop for algo testing."""
from __future__ import annotations

import shutil
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List

from config.offline_paths import PROJECT_ROOT
from services.offline_pipeline import list_hik_corner_images, resolve_path, resolve_place_rgb

CACHE_ROOT = PROJECT_ROOT / "workdir" / "algo_inputs"
RADAR_DIR = CACHE_ROOT / "radar"
HIK_DIR = CACHE_ROOT / "hik_corner"
MECH_DIR = CACHE_ROOT / "mech_hole"
PLACE_DIR = CACHE_ROOT / "place_verify"


def _ensure_dirs() -> None:
    for folder in (RADAR_DIR, HIK_DIR, MECH_DIR, PLACE_DIR):
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
    saved: Dict[str, List[str]] = {
        "radar": [],
        "hik_corner": [],
        "mech_hole": [],
        "place_verify": [],
    }
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

    raw_place = str(data.get("place_verify_dir") or "").strip()
    if raw_place:
        try:
            saved["place_verify"].append(_copy_one(resolve_place_rgb(raw_place), PLACE_DIR))
        except Exception as exc:
            errors.append(f"place_verify: {exc}")
    raw_place_depth = str(data.get("place_depth") or "").strip()
    if raw_place_depth:
        try:
            saved["place_verify"].append(_copy_one(resolve_path(raw_place_depth, kind="mech_depth"), PLACE_DIR))
        except Exception as exc:
            errors.append(f"place_depth: {exc}")

    any_saved = any(saved[k] for k in saved)
    return {
        "cache_root": str(CACHE_ROOT),
        "saved": saved,
        "errors": errors,
        "ok": not errors or any_saved,
    }
