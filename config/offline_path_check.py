# -*- coding: utf-8 -*-
"""检查 config/offline_paths.py：缺路径 / 路径无效时给出可读中文原因。"""
from __future__ import annotations

import sys
from typing import Any, Dict, List

from config.offline_paths import get_offline_paths
from services.offline_pipeline import list_hik_corner_images, resolve_path


def _log(msg: str) -> None:
    print(f"[offline_paths] {msg}", file=sys.stderr, flush=True)


def validate_offline_paths(cfg: Dict[str, Any] | None = None) -> Dict[str, Any]:
    """返回 {ok, errors, warnings, resolved, summary}。ok=无 errors（warnings 仍可有）。"""
    data = dict(cfg or get_offline_paths())
    errors: List[str] = []
    warnings: List[str] = []
    resolved: Dict[str, str] = {}

    raw_pcd = str(data.get("radar_pcd") or "").strip()
    if not raw_pcd:
        errors.append("未配置雷达路径（radar_pcd）。填 .pcd 文件或其所在文件夹。")
    else:
        try:
            found = resolve_path(raw_pcd, kind="radar_pcd")
            resolved["radar_pcd"] = str(found)
        except Exception as exc:
            errors.append(f"雷达 PCD 读不到：{exc}")

    raw_hik = str(data.get("hik_corner_dir") or data.get("hik_corner_image") or "").strip()
    if not raw_hik:
        errors.append("未配置海康角点文件夹（hik_corner_dir）。填放照片的文件夹即可，不用管文件名。")
    else:
        try:
            images = list_hik_corner_images(data)
            resolved["hik_corner_dir"] = str(images[0].parent if images[0].parent else images[0])
            for i, img in enumerate(images[:8], start=1):
                resolved[f"hik_image_{i}"] = str(img)
            if len(images) > 8:
                warnings.append(f"海康文件夹共 {len(images)} 张图，检查里只列出前 8 张。")
        except Exception as exc:
            errors.append(f"海康角点图读不到：{exc}")

    for key, title in (
        ("mech_rgb", "梅卡彩色图"),
        ("mech_depth", "梅卡深度图"),
        ("mech_cloud", "梅卡点云"),
    ):
        raw = str(data.get(key) or "").strip()
        if not raw:
            warnings.append(f"{title}（{key}）未配置（可选）。")
            continue
        try:
            found = resolve_path(raw, kind=key)
            resolved[key] = str(found)
        except Exception as exc:
            warnings.append(f"{title} 配置了但读不到：{exc}")

    ok = not errors
    if ok and not warnings:
        summary = "离线路径检查通过：雷达与海康文件夹均可读。"
    elif ok:
        summary = "离线路径基本可用（有可选项未配或读失败，见警告）。"
    else:
        summary = "离线路径检查失败：缺少或读不到必需数据，请先填路径再跑算法。"

    report = {
        "ok": ok,
        "errors": errors,
        "warnings": warnings,
        "resolved": resolved,
        "summary": summary,
    }
    _log(summary)
    for e in errors:
        _log(f"ERROR: {e}")
    for w in warnings:
        _log(f"WARN: {w}")
    for k, v in resolved.items():
        _log(f"OK: {k} -> {v}")
    return report


def format_report_text(report: Dict[str, Any]) -> str:
    lines = [str(report.get("summary") or "")]
    errors = list(report.get("errors") or [])
    warnings = list(report.get("warnings") or [])
    resolved = dict(report.get("resolved") or {})
    if errors:
        lines.append("")
        lines.append("【错误】")
        lines.extend(f"· {x}" for x in errors)
    if warnings:
        lines.append("")
        lines.append("【警告】")
        lines.extend(f"· {x}" for x in warnings)
    if resolved:
        lines.append("")
        lines.append("【已解析路径】")
        lines.extend(f"· {k}: {v}" for k, v in resolved.items())
    lines.append("")
    lines.append("配置文件：config/offline_paths.py")
    return "\n".join(lines)
