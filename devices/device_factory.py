# -*- coding: utf-8 -*-
"""现场离线联调所需的设备适配器。"""
from __future__ import annotations

from typing import Any, Mapping

from core.digital_twin_state import DigitalTwinState
from devices.field_camera_hub import FieldCameraHub
from devices.mock_devices import (
    MockPLCAdapter,
    MockRadarAdapter,
    MockRobotAdapter,
)


def normalize_device_mode(value: Any = "mock") -> str:
    """兼容旧配置，但现场离线联调始终使用 mock。"""
    return "mock"


def create_device_adapters(
    twin: DigitalTwinState,
    system_config: Mapping[str, Any] | None = None,
):
    """返回现场离线联调的 Mock PLC / 龙门架 / 雷达 / 双相机适配器。"""
    device_cfg = dict((system_config or {}).get("devices") or {})
    camera_cfg = dict(device_cfg.get("camera") or {})
    camera_cfg["allow_file_inputs"] = True
    camera_cfg["mock_probe"] = True

    plc = MockPLCAdapter(twin)
    robot = MockRobotAdapter(twin, plc, device_cfg.get("gantry") or {})
    radar = MockRadarAdapter(twin)
    camera = FieldCameraHub(twin, camera_cfg)
    return "mock", plc, robot, radar, camera
