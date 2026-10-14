# -*- coding: utf-8 -*-
"""Render a simple bird's-eye preview image from a PCD file for the offline UI."""
from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np


def load_xyz(pcd_path: str | Path) -> np.ndarray:
    """Load Nx3 xyz; prefer project PCD loader, fall back to Open3D."""
    path = Path(pcd_path)
    if not path.is_file():
        raise FileNotFoundError(f"找不到 PCD：{path}")
    try:
        from algorithm_modules.point_cloud_segment_module.inference import load_pcd

        pts = np.asarray(load_pcd(str(path)), dtype=np.float64)
    except Exception:
        # Match the algorithm's reader: Open3D supports compressed PCD, but
        # needs an ASCII temporary path on some Windows builds.
        from algorithm_modules.point_cloud_segment_module.point_cloud_pipeline import read_point_cloud

        cloud = read_point_cloud(path)
        pts = np.asarray(cloud.points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] < 3 or pts.size == 0:
        raise RuntimeError(f"PCD 无有效点：{path}")
    return pts[:, :3]


def render_birdseye(
    pcd_path: str | Path,
    *,
    size: int = 720,
    max_points: int = 250_000,
    margin: float = 0.06,
) -> Tuple[Optional[np.ndarray], Dict[str, Any]]:
    """Return (BGR uint8 image or None, meta)."""
    path = Path(pcd_path)
    meta: Dict[str, Any] = {"path": str(path), "point_count": 0, "error": ""}
    try:
        pts = load_xyz(path)
    except Exception as exc:  # noqa: BLE001
        meta["error"] = str(exc)
        return None, meta

    n = int(pts.shape[0])
    meta["point_count"] = n
    if n > max_points:
        rng = np.random.default_rng(0)
        idx = rng.choice(n, size=max_points, replace=False)
        pts = pts[idx]

    xy = pts[:, :2]
    finite = np.isfinite(xy).all(axis=1)
    xy = xy[finite]
    if xy.size == 0:
        meta["error"] = "点云坐标无效"
        return None, meta

    lo = np.percentile(xy, 1, axis=0)
    hi = np.percentile(xy, 99, axis=0)
    span = np.maximum(hi - lo, 1e-3)
    pad = span * float(margin)
    lo = lo - pad
    hi = hi + pad
    span = hi - lo

    side = max(240, int(size))
    img = np.zeros((side, side, 3), dtype=np.uint8)
    img[:] = (28, 32, 38)

    u = ((xy[:, 0] - lo[0]) / span[0] * (side - 1)).astype(np.int32)
    v = ((hi[1] - xy[:, 1]) / span[1] * (side - 1)).astype(np.int32)
    u = np.clip(u, 0, side - 1)
    v = np.clip(v, 0, side - 1)
    img[v, u] = (90, 210, 255)

    # densify visually with small circles on a downsampled set
    step = max(1, len(u) // 8000)
    for x, y in zip(u[::step], v[::step]):
        cv2.circle(img, (int(x), int(y)), 1, (120, 230, 255), -1, lineType=cv2.LINE_AA)

    label = f"{path.name}  |  {meta['point_count']:,} pts"
    cv2.putText(img, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 230, 240), 1, cv2.LINE_AA)
    return img, meta
