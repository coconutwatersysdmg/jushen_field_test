# -*- coding: utf-8 -*-
"""现场离线流程开关；软件仅使用现场文件输入和离线孪生适配器。

历史保存的运行配置会归一化为 field；文件路径请在 offline_paths.py 配置，
雷达 PCD 也可通过主界面的「选择雷达 PCD」指定。
"""
from __future__ import annotations

from pathlib import Path
from typing import Any, Dict


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PROFILE_STATE_FILE = PROJECT_ROOT / "workdir" / "ui_run_profile.json"

# ===========================================================================
# 运行模式（仅现场离线；权威入口）
# ===========================================================================
# field = 现场离线：文件输入 + 现场 PointNet / corner_service
RUN_PROFILE = "field"

RUN_PROFILES: Dict[str, Dict[str, Any]] = {
    "field": {
        "id": "field",
        "title": "现场离线联调",
        "subtitle": "同款孪生界面；读工控机离线文件 + 现场 PointNet/角点 YOLO（不实时连相机雷达）",
        "switches": {
            # mock：不连真机 SDK；用 config/offline_paths.py 注入的文件路径
            "DEVICE_MODE": "mock",
            "ALLOW_DEMO_DEVICE_DATA": True,
            "ALLOW_UNIMPLEMENTED_VISION_MEASUREMENT_ZERO": True,
            "ALLOW_REAL_MOTION": False,
            "USE_LIVE_LIDAR_CAPTURE": False,
            "CORNER_REVIEW_ENABLED": True,
            "CORNER_REVIEW_AUTO_ACCEPT_DEMO": False,
        },
    },
}

# ===========================================================================
# 1) 设备模式 / Mock 数据（也可被 RUN_PROFILE 覆盖）
# ===========================================================================

DEVICE_MODE = "mock"
ALLOW_DEMO_DEVICE_DATA = True
ALLOW_UNIMPLEMENTED_VISION_MEASUREMENT_ZERO = True

# ===========================================================================
# 2) 真机运动 / 雷达实采
# ===========================================================================

ALLOW_REAL_MOTION = False
USE_LIVE_LIDAR_CAPTURE = False

# ===========================================================================
# 3) 角点人工审核
# ===========================================================================

CORNER_REVIEW_ENABLED = True
CORNER_REVIEW_AUTO_ACCEPT_DEMO = False

# ===========================================================================
# 4) 现场算法
# ===========================================================================



def list_run_profiles() -> list[dict[str, Any]]:
    return [
        {
            "id": p["id"],
            "title": p["title"],
            "subtitle": p["subtitle"],
        }
        for p in RUN_PROFILES.values()
    ]


def get_run_profile(profile_id: str | None = None) -> dict[str, Any]:
    pid = str(profile_id or RUN_PROFILE or "field").strip().lower()
    if pid not in RUN_PROFILES:
        pid = "field"
    return dict(RUN_PROFILES[pid])


def apply_run_profile(profile_id: str, *, persist: bool = True) -> dict[str, Any]:
    """套用现场离线配置，并同步到本模块与 external_devices 可变字段。"""
    global RUN_PROFILE
    global DEVICE_MODE, ALLOW_DEMO_DEVICE_DATA, ALLOW_UNIMPLEMENTED_VISION_MEASUREMENT_ZERO
    global ALLOW_REAL_MOTION, USE_LIVE_LIDAR_CAPTURE
    global CORNER_REVIEW_ENABLED, CORNER_REVIEW_AUTO_ACCEPT_DEMO

    profile = get_run_profile(profile_id)
    switches = dict(profile.get("switches") or {})
    RUN_PROFILE = str(profile["id"])
    DEVICE_MODE = str(switches.get("DEVICE_MODE", DEVICE_MODE))
    ALLOW_DEMO_DEVICE_DATA = bool(switches.get("ALLOW_DEMO_DEVICE_DATA", ALLOW_DEMO_DEVICE_DATA))
    ALLOW_UNIMPLEMENTED_VISION_MEASUREMENT_ZERO = bool(
        switches.get("ALLOW_UNIMPLEMENTED_VISION_MEASUREMENT_ZERO", ALLOW_UNIMPLEMENTED_VISION_MEASUREMENT_ZERO)
    )
    ALLOW_REAL_MOTION = bool(switches.get("ALLOW_REAL_MOTION", ALLOW_REAL_MOTION))
    USE_LIVE_LIDAR_CAPTURE = bool(switches.get("USE_LIVE_LIDAR_CAPTURE", USE_LIVE_LIDAR_CAPTURE))
    CORNER_REVIEW_ENABLED = bool(switches.get("CORNER_REVIEW_ENABLED", CORNER_REVIEW_ENABLED))
    CORNER_REVIEW_AUTO_ACCEPT_DEMO = bool(
        switches.get("CORNER_REVIEW_AUTO_ACCEPT_DEMO", CORNER_REVIEW_AUTO_ACCEPT_DEMO)
    )

    # 延迟同步，避免与 external_devices_config 循环导入
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

    if persist:
        try:
            import json

            PROFILE_STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
            PROFILE_STATE_FILE.write_text(
                json.dumps({"run_profile": RUN_PROFILE}, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
        except Exception:
            pass

    return {
        "id": RUN_PROFILE,
        "title": profile["title"],
        "subtitle": profile["subtitle"],
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


def load_persisted_run_profile() -> str:
    """读取历史保存配置；旧模式均由现场配置入口归一化。"""
    try:
        if PROFILE_STATE_FILE.is_file():
            import json

            data = json.loads(PROFILE_STATE_FILE.read_text(encoding="utf-8"))
            pid = str((data or {}).get("run_profile") or "").strip().lower()
            if pid in RUN_PROFILES:
                return pid
    except Exception:
        pass
    return str(RUN_PROFILE or "field")


# 导入时加载现场配置（兼容历史保存值，不依赖 external_devices_config）
apply_run_profile(load_persisted_run_profile(), persist=False)
