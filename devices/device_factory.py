# -*- coding: utf-8 -*-
"""Create field offline adapters for local files and digital-twin state."""
from __future__ import annotations

from typing import Any, Mapping

from core.digital_twin_state import DigitalTwinState
from devices.mock_devices import (
    MockPLCAdapter,
    MockRadarAdapter,
    MockRobotAdapter,
)


def normalize_device_mode(value: Any) -> str:
    """Normalize legacy configuration to the field offline adapter mode."""
    return "mock"


def create_device_adapters(
    twin: DigitalTwinState,
    system_config: Mapping[str, Any] | None = None,
):
    """Return (mode, plc, robot, radar, camera)."""
    cfg = dict(system_config or {})
    device_cfg = dict(cfg.get("devices") or {})

    # Field offline adapters preserve digital-twin state without SDK connections.
    from devices.field_camera_hub import FieldCameraHub
    plc = MockPLCAdapter(twin)
    robot = MockRobotAdapter(twin, plc, device_cfg.get("gantry") or {})
    radar = MockRadarAdapter(twin)
    cam_cfg = dict(device_cfg.get("camera") or {})
    cam_cfg["allow_file_inputs"] = True
    cam_cfg["mock_probe"] = True
    cam_cfg["offline_only"] = True
    camera = FieldCameraHub(twin, cam_cfg)
    return "mock", plc, robot, radar, camera
