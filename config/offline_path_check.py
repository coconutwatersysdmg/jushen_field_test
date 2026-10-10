# -*- coding: utf-8 -*-
"""检查 config/offline_paths.py：缺路径 / 路径无效时给出可读中文原因。"""
from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Dict, List

from config.offline_paths import get_offline_paths
from services.offline_pipeline import resolve_path


def _log(msg: str) -> None:
    print(f"[offline_paths] {msg}", file=sys.stderr, flush=True)


def validate_offline_paths(cfg: Dict[str, Any] | None = None) -> Dict[str, Any]:
    """返回 {ok, errors, warnings, resolved, summary}。ok=无 errors（warnings 仍可有）。"""
    data = dict(cfg or get_offline_paths())
    errors: List[str] = []
    warnings: List[str] = []
    resolved: Dict[str, str] = {}

    # 雷达 PCD：跑 PointNet 必需
    raw_pcd = str(data.get("radar_pcd") or "").strip()
    if not raw_pcd:
        errors.append(
            "未配置雷达 PCD（radar_pcd）。请改 config/offline_paths.py，"
            "或到界面「调试输入」填写 point_cloud_path。"
        )
    else:
        try:
            found = resolve_path(raw_pcd, kind="radar_pcd")
            resolved["radar_pcd"] = str(found)
        except Exception as exc:
            errors.append(f"雷达 PCD 读不到：{exc}")

    # 海康角点图：跑角点 YOLO 必需（单图或按点）
    by_point = dict(data.get("hik_corner_images_by_point") or {})
    filled = {str(k): str(v).strip() for k, v in by_point.items() if str(v or "").strip()}
    raw_hik = str(data.get("hik_corner_image") or "").strip()
    if filled:
        for pid, raw in filled.items():
            try:
                found = resolve_path(raw, kind=f"hik_{pid}")
                resolved[f"hik_{pid}"] = str(found)
            except Exception as exc:
                errors.append(f"海康角点 {pid} 读不到：{exc}")
    elif raw_hik:
        try:
            found = resolve_path(raw_hik, kind="hik_corner_image")
            resolved["hik_corner_image"] = str(found)
        except Exception as exc:
            errors.append(f"海康角点图读不到：{exc}")
    else:
        errors.append(
            "未配置海康角点图。请填写 hik_corner_image，"
            "或按 P1..P4 填满 hik_corner_images_by_point。"
        )

    # 梅卡：本阶段可选（孔位算法未强制）
    for key, title in (
        ("mech_rgb", "梅卡彩色图"),
        ("mech_depth", "梅卡深度图"),
        ("mech_cloud", "梅卡点云"),
    ):
        raw = str(data.get(key) or "").strip()
        if not raw:
            warnings.append(f"{title}（{key}）未配置（可选，本阶段可不跑孔位）。")
            continue
        try:
            found = resolve_path(raw, kind=key)
            resolved[key] = str(found)
        except Exception as exc:
            warnings.append(f"{title} 配置了但读不到：{exc}")

    ok = not errors
    if ok and not warnings:
        summary = "离线路径检查通过：雷达 PCD 与海康角点图均可读。"
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
    lines.append("也可在界面点「调试输入」临时改路径。")
    return "\n".join(lines)
