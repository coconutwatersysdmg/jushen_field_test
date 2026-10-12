# -*- coding: utf-8 -*-
"""现场离线联调的固定运行配置。

本项目只支持一个运行形态：读取现场离线文件，使用现场算法完成联调。
设备适配器固定为 Mock，避免启动时误连真实 PLC、雷达或相机。
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PROFILE_STATE_FILE = PROJECT_ROOT / "workdir" / "ui_run_profile.json"

RUN_PROFILE = "field"
DEVICE_MODE = "mock"
ALLOW_DEMO_DEVICE_DATA = True
ALLOW_UNIMPLEMENTED_VISION_MEASUREMENT_ZERO = True
ALLOW_REAL_MOTION = False
USE_LIVE_LIDAR_CAPTURE = False
CORNER_REVIEW_ENABLED = True
CORNER_REVIEW_AUTO_ACCEPT_DEMO = False

FIELD_PROFILE: Dict[str, Any] = {
    "id": "field",
    "title": "现场离线联调",
    "subtitle": "读取现场离线文件，调用现场 PointNet / 角点 YOLO 完成联调",
    "switches": {
        "DEVICE_MODE": DEVICE_MODE,
        "ALLOW_DEMO_DEVICE_DATA": ALLOW_DEMO_DEVICE_DATA,
        "ALLOW_UNIMPLEMENTED_VISION_MEASUREMENT_ZERO": ALLOW_UNIMPLEMENTED_VISION_MEASUREMENT_ZERO,
        "ALLOW_REAL_MOTION": ALLOW_REAL_MOTION,
        "USE_LIVE_LIDAR_CAPTURE": USE_LIVE_LIDAR_CAPTURE,
        "CORNER_REVIEW_ENABLED": CORNER_REVIEW_ENABLED,
        "CORNER_REVIEW_AUTO_ACCEPT_DEMO": CORNER_REVIEW_AUTO_ACCEPT_DEMO,
    },
}


def list_run_profiles() -> list[dict[str, Any]]:
    """兼容旧调用方，但只返回现场离线联调。"""
    return [
        {
            "id": FIELD_PROFILE["id"],
            "title": FIELD_PROFILE["title"],
            "subtitle": FIELD_PROFILE["subtitle"],
        }
    ]


def get_run_profile(profile_id: str | None = None) -> dict[str, Any]:
    """返回固定现场配置；传入其它模式名时明确拒绝。"""
    if profile_id not in (None, "", "field"):
        raise ValueError(f"仅支持现场离线联调模式，收到：{profile_id}")
    return dict(FIELD_PROFILE)


def _sync_external_device_switches() -> None:
    try:
        import sys

        edc = sys.modules.get("config.external_devices_config")
        if edc is not None:
            edc.DEVICE_MODE = DEVICE_MODE
            if isinstance(getattr(edc, "GANTRY", None), dict):
                edc.GANTRY["allow_real_motion"] = ALLOW_REAL_MOTION
            if isinstance(getattr(edc, "LIVOX", None), dict):
                edc.LIVOX["use_live_capture"] = USE_LIVE_LIDAR_CAPTURE
    except Exception:
        pass


def apply_run_profile(profile_id: str = "field", *, persist: bool = True) -> dict[str, Any]:
    """应用唯一的现场离线联调配置。"""
    if str(profile_id or "field").strip().lower() != "field":
        raise ValueError(f"仅支持现场离线联调模式，收到：{profile_id}")

    _sync_external_device_switches()
    if persist:
        try:
            PROFILE_STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
            PROFILE_STATE_FILE.write_text(
                json.dumps({"run_profile": "field"}, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
        except Exception:
            pass

    return {
        "id": "field",
        "title": FIELD_PROFILE["title"],
        "subtitle": FIELD_PROFILE["subtitle"],
        "switches": dict(FIELD_PROFILE["switches"]),
    }


def load_persisted_run_profile() -> str:
    """兼容旧状态文件，但始终固定返回现场离线联调。"""
    return "field"


apply_run_profile("field", persist=False)
