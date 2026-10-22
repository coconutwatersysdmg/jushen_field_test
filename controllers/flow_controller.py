# -*- coding: utf-8 -*-
from __future__ import annotations

import json
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FuturesTimeoutError
from copy import deepcopy
from datetime import datetime
from math import isfinite
from pathlib import Path
from time import perf_counter, sleep
from typing import Any, Dict, List, Mapping
from uuid import uuid4
from utils.measurement_rounding import round_measurement

from core.digital_twin_state import DigitalTwinState
from devices.device_factory import create_device_adapters
from services.algorithm_facade import AlgorithmFacade
from services.field_space_planner import (
    build_field_space_plan,
    field_b_region_for_round,
    field_b_regions,
)
from services.field_cycle_policy import (
    FIELD_FIRST_STEPS as FIELD_POLICY_FIRST_STEPS,
    FIELD_REPEAT_STEPS as FIELD_POLICY_REPEAT_STEPS,
    cycle_steps_for_round,
)
from services.board_spec_utils import (
    board_extent_from_corners,
    board_extent_from_geometry,
    merge_board_specs,
    resolve_pallet_specs,
)
from services.field_dynamic_box_monitor_service import FieldDynamicBoxMonitorService
from services.sensor_calibration_service import SensorCalibrationService
from services.camera_board_geometry_service import CameraBoardGeometryService
from services.space_manager import SpaceManager
from services.module_evidence_service import ModuleEvidenceService
from services.vehicle_database_service import create_vehicle_database_service
from services.plc_motion_publisher import PlcMotionPublisher
import config.feature_switches as feature_switches
from config.system_config import get_system_config
from config.external_devices_config import get_device_layout_config

PROJECT_ROOT = Path(__file__).resolve().parents[1]
RESULT_FILE = PROJECT_ROOT / "workdir" / "overall_results.json"
STATE_FILE = PROJECT_ROOT / "workdir" / "twin_state.json"
# TODO: 与PLC交互 — 外接设备统一配置：config/external_devices_config.py


class FlowController:
    """现场离线循环：设备检查→插孔∥雷达→相机识别/划格→人工放货检测。"""

    # 现场：首轮建图；后续按 B1→B2→…→B5 做返回/插孔/放前监测/放后检测。
    FIELD_FIRST_STEPS = list(FIELD_POLICY_FIRST_STEPS)
    FIELD_REPEAT_STEPS = list(FIELD_POLICY_REPEAT_STEPS)

    @staticmethod
    def is_radar_only_test() -> bool:
        return feature_switches.is_radar_only_test()

    @staticmethod
    def uses_coord_cycle_flow() -> bool:
        return True

    def _is_offline_field_test(self) -> bool:
        return True

    def _offline_dummy_b_region(self) -> dict[str, Any]:
        return {
            "region_id": "B1",
            "column": "B",
            "local_row_index": 1,
            "status": "AVAILABLE",
            "center_world_xyz_mm": [6000.0, 1200.0, 0.0],
            "corners_world_xyz_mm": [
                [5400.0, 600.0, 0.0],
                [6600.0, 600.0, 0.0],
                [6600.0, 1800.0, 0.0],
                [5400.0, 1800.0, 0.0],
            ],
        }

    def _resolve_b_region_for_round(self) -> dict[str, Any]:
        try:
            return field_b_region_for_round(self.space.snapshot(), self.round_index)
        except Exception:
            if self._is_offline_field_test():
                return self._offline_dummy_b_region()
            raise

    def __init__(self, twin=None, plc=None, robot=None, radar=None, camera=None, algorithms=None):
        self.device_config = get_device_layout_config()
        self.system_config = get_system_config()
        self.allow_demo = bool((self.system_config.get("runtime") or {}).get("allow_demo_device_data", True))
        self.twin = twin or DigitalTwinState()
        if plc is None or robot is None or radar is None or camera is None:
            mode, built_plc, built_robot, built_radar, built_camera = create_device_adapters(
                self.twin, self.system_config
            )
            self.device_mode = mode
            self.plc = plc or built_plc
            self.robot = robot or built_robot
            self.radar = radar or built_radar
            self.camera = camera or built_camera
        else:
            self.device_mode = str((self.system_config.get("runtime") or {}).get("device_mode") or "mock")
            self.plc = plc
            self.robot = robot
            self.radar = radar
            self.camera = camera
        # Real hardware mode must not synthesize demo frames behind the scenes.
        if str(self.device_mode).lower() == "real":
            self.allow_demo = False
        if hasattr(self.camera, "set_demo_enabled"):
            self.camera.set_demo_enabled(self.allow_demo)
        self.algorithms = algorithms or AlgorithmFacade()
        self.calibration = SensorCalibrationService()
        self.field_box_monitor = FieldDynamicBoxMonitorService(calibration=self.calibration)
        g_cfg = self.system_config.get("camera_board_geometry") or {}
        self.board_geometry = CameraBoardGeometryService(
            row_length_mm=float(g_cfg.get("row_length_mm", 1200.0)),
            high_low_height_threshold_mm=float(g_cfg.get("high_low_height_threshold_mm", 80.0)),
            max_borrow_mm=float(g_cfg.get("max_borrow_mm", 650.0)),
            min_second_section_remaining_mm=float(g_cfg.get("min_second_section_remaining_mm", 600.0)),
        )
        self.space = SpaceManager()
        self.module_evidence = ModuleEvidenceService()
        db_cfg=self.system_config.get("database") or {}
        self.vehicle_db=create_vehicle_database_service(db_cfg)
        self.loading_session_id=""; self._db_message_cursor=0
        roles = self.device_config.get("camera_roles") or {}
        self.queue: List[Dict[str, Any]] = []
        self.completed: List[Dict[str, Any]] = []
        self.round_index = 0
        self.step_index = 0
        self.running = False
        self.finished = False
        self.round_data: Dict[str, Any] = {}
        self.results: List[Dict[str, Any]] = []
        self.debug_inputs: Dict[str, Any] = {}
        self.field_camera_gallery: List[Dict[str, Any]] = []
        self.state_listener = None
        # 主界面在 PLC 到位轮询期间处理后台识别完成的画面更新。
        self.ui_event_pump = None
        self.truck_initialized = False
        self.corner_review_callback = None
        review_cfg = self.system_config.get("corner_review") or {}
        self.corner_review_enabled = bool(review_cfg.get("enabled", True))
        self.corner_review_auto_accept_demo = bool(review_cfg.get("auto_accept_demo", False))
        # TODO: 与PLC交互 — 启动时连接 PLC（mock 假连；real 用 config/system_config.py 的 devices.plc）
        self.plc.connect()
        if hasattr(self.robot, "motion_callback"):
            self.robot.motion_callback = self._notify_motion
        if hasattr(self.robot, "command_emitter"):
            self.robot.command_emitter = self._on_robot_motion_command
        self.plc_publisher = PlcMotionPublisher()
        self.plc_command_listener = None
        self.auto_push_plc_commands = False
        self.twin.add_message("CALIBRATION", "INFO", "已加载用户标定配置；运动相机使用机械臂实时位姿×安装外参", self.calibration.diagnostic_summary())
        try:
            from config.offline_debug_inputs import build_debug_inputs_from_offline
            from config.offline_path_check import validate_offline_paths
            from config.offline_paths import get_offline_paths

            _dbg = build_debug_inputs_from_offline(self.module_evidence.example_debug_inputs())
            _paths = get_offline_paths()
            _paths["radar_pcd"] = _dbg.get("point_cloud_path") or _paths.get("radar_pcd")
            _check = validate_offline_paths(
                {"radar_pcd": _dbg.get("point_cloud_path", "")} if self.is_radar_only_test() else _paths,
                require_radar=not self.is_radar_only_test(),
            )
        except Exception as exc:
            print(f"[FlowController] 离线路径加载失败：{exc}", flush=True)
            _dbg = {"offline_field_test": True, "point_cloud_path": "", "point_cloud_example_only": False}
            _check = {"ok": False, "errors": [str(exc)], "warnings": [], "summary": str(exc)}
        self.set_debug_inputs(_dbg)
        self.twin.add_message(
            "MODULE_INPUTS",
            "SUCCESS" if _check.get("ok") else "WARNING",
            str(_check.get("summary") or "离线路径已加载（config/offline_paths.py）"),
            {
                "module_count": len(self.module_evidence.snapshot()),
                "offline_field_test": True,
                "errors": list(_check.get("errors") or []),
                "warnings": list(_check.get("warnings") or []),
                "resolved": dict(_check.get("resolved") or {}),
            },
        )
        self.twin.add_message(
            "PLC_MOTION",
            "INFO",
            "运动坐标确认后优先经主系统已连接的 PLC 直接写轴；独立 plc_console 仅作备用通道",
            {"server": "jushenzhineng-plc-console"},
        )

    def set_state_listener(self, listener):
        """Receive intermediate motion snapshots so the UI can animate real step actions."""
        self.state_listener = listener if callable(listener) else None

    def set_ui_event_pump(self, callback):
        """Allow the GUI to paint queued camera results during a blocking PLC wait."""
        self.ui_event_pump = callback if callable(callback) else None

    def set_plc_command_listener(self, listener):
        self.plc_command_listener = listener if callable(listener) else None
        self.plc_publisher.set_listener(self.plc_command_listener)

    def set_auto_push_plc(self, enabled: bool) -> None:
        self.auto_push_plc_commands = bool(enabled)

    def _resolve_motion_xyzr(self, cmd: Mapping[str, Any]) -> dict[str, float]:
        raw = cmd.get("gantry_xyzr")
        if isinstance(raw, Mapping) and all(k in raw for k in ("X", "Y", "Z")):
            # 自动流程只执行 XYZ；即使上游保留 R 供标定，也不把它作为 PLC 目标。
            return {k: float(raw[k]) for k in ("X", "Y", "Z")}
        from devices.world_to_gantry import WorldToGantryError, transform_world_to_gantry

        mapping = dict(getattr(self.robot, "world_to_gantry", None) or {})
        if not mapping:
            from config.external_devices_config import GANTRY

            mapping = dict((GANTRY or {}).get("world_to_gantry") or {})
        try:
            return transform_world_to_gantry(cmd.get("world_pose") or {}, mapping)
        except WorldToGantryError as exc:
            raise RuntimeError(f"无法换算 XYZR：{exc}") from exc

    def _execute_motion_on_local_plc(self, cmd: Mapping[str, Any]) -> dict[str, Any] | None:
        """主系统 PLC 已连接时直接写轴；无法本地执行时返回 None（交由控制台备用通道）。"""
        plc = self.plc
        if not getattr(plc, "connected", False):
            return None

        try:
            targets = self._resolve_motion_xyzr(cmd)
        except Exception as exc:
            return {
                "success": False,
                "message": str(exc),
                "cmd_id": cmd.get("cmd_id"),
                "channel": "local_plc",
            }

        try:
            from config.external_devices_config import GANTRY
        except Exception:
            GANTRY = {}
        speed = float(cmd.get("speed") or (GANTRY or {}).get("default_speed") or 30.0)
        timeout_s = float((GANTRY or {}).get("move_timeout_s") or 60.0)
        soft_limits = dict((GANTRY or {}).get("soft_limits") or {})
        # 现场流程保留 Z/R，仅发布 X/Y。
        axes = ("X", "Y")
        targets = {axis: float(targets[axis]) for axis in axes}

        if hasattr(plc, "move_absolute_xyzr"):
            result = plc.move_absolute_xyzr(
                targets,
                speed=speed,
                timeout_s=timeout_s,
                soft_limits=soft_limits or None,
                axes=axes,
            )
            return {
                "success": bool(result.get("success")),
                "accepted": bool(result.get("success")),
                "message": str(
                    result.get("message")
                    or ("本地 PLC 绝对定位完成" if result.get("success") else "本地 PLC 写轴失败")
                ),
                "cmd_id": cmd.get("cmd_id"),
                "channel": "local_plc",
                "targets": targets,
                "axes": list(axes),
                "result": result,
            }

        # mock / 仅状态通道：记账成功，不强制依赖独立控制台
        if hasattr(plc, "send_command"):
            plc.send_command(
                "MOVE_ABSOLUTE_XYZR",
                {"targets": targets, "axes": list(axes), "speed": speed, "cmd_id": cmd.get("cmd_id")},
            )
        return {
            "success": True,
            "accepted": True,
            "message": f"已记账下发（当前 PLC 适配器无真机写轴）：{targets}",
            "cmd_id": cmd.get("cmd_id"),
            "channel": "local_plc_bookkeeping",
            "targets": targets,
        }

    def push_last_plc_command(self) -> dict:
        """确认下发：优先经主系统已连接 PLC 直接写轴；否则再尝试独立 plc_console。"""
        cmd = self.plc_publisher.last_command
        if not cmd:
            return {"success": False, "message": "没有可下发的运动指令"}

        local = self._execute_motion_on_local_plc(cmd)
        if local is not None:
            return local

        remote = self.plc_publisher.push(cmd)
        if remote.get("success"):
            return {**remote, "channel": "plc_console"}
        return {
            "success": False,
            "message": (
                "主系统 PLC 未连接，且独立控制台不可用："
                f"{remote.get('message') or '未知错误'}。"
                "请先完成设备检查中的 PLC 连接，或启动 plc_console 作为备用。"
            ),
            "cmd_id": cmd.get("cmd_id"),
            "channel": "none",
            "console": remote,
        }

    def _on_robot_motion_command(self, payload: dict):
        """机器人适配器终点回调：落盘 + 通知 UI；按开关决定是否实时推送。"""
        pose = dict(payload.get("world_pose") or {})
        cmd = self.plc_publisher.build(
            robot_id=str(payload.get("robot_id") or "PICK_ARM"),
            world_pose=pose,
            task=str(payload.get("task") or ""),
            step=str(self.current_step[0] if self.running else ""),
            round_index=int(self.round_index),
            gantry_xyzr=payload.get("gantry_xyzr"),
            speed=float(payload.get("speed") or 30.0),
            extra={
                k: v
                for k, v in dict(payload).items()
                if k not in {"robot_id", "world_pose", "task", "gantry_xyzr", "speed"}
            }
            or None,
        )
        result = self.plc_publisher.publish_local(cmd)
        # 是否实时推送由主界面决定（弹窗确认 / 自动运行勾选），此处只落盘并通知 UI
        if self.auto_push_plc_commands:
            # 标记给 UI：本条在展示时可自动推送
            result = {**result, "_ui_auto_push": True}
        return {"command": result, "pushed": False, "push_result": None}

    def _notify_motion(self, robot_id: str, pose: Dict[str, Any], task: str):
        if self.loading_session_id:
            self.vehicle_db.record_motion(self.loading_session_id, robot_id, pose, task)
        if self.state_listener:
            self.state_listener(self.snapshot(), {"robot_id": robot_id, "pose": deepcopy(pose), "task": str(task)})


    @staticmethod
    def _load_json(path: Path):
        try:
            return json.loads(path.read_text(encoding="utf-8")) if path.is_file() else {}
        except Exception:
            return {}

    def _camera_for_role(self, role: str, default: str):
        return str((self.device_config.get("camera_roles") or {}).get(role, default))

    @property
    def is_first_round(self): return self.round_index == 0
    @property
    def steps(self):
        if self.is_radar_only_test():
            return [("RADAR_TEST", "雷达算法测试（所选 PCD）")]
        return list(cycle_steps_for_round(self.round_index))
    @property
    def current_step(self):
        done_msg = "本轮/全部完成" if self.uses_coord_cycle_flow() else "全部货物装载完成"
        if self.finished:
            return ("DONE", done_msg)
        if self.step_index >= len(self.steps):
            return ("DONE", done_msg)
        return self.steps[self.step_index]
    @property
    def current_cargo(self): return self.queue[self.round_index] if 0 <= self.round_index < len(self.queue) else None

    def set_plan(self, items):
        self.queue=[]; seq=0
        for item in items:
            qty=max(1,int(item.get("quantity",1)))
            for unit in range(1,qty+1):
                seq+=1; c=deepcopy(item); c["unit_index"]=unit; c["sequence"]=seq
                c["instance_id"]=f"{c.get('cargo_code','CARGO')}-{unit:03d}-{seq:03d}"
                inventory_base=c.get("inventory_id") or c.get("stock_id")
                c["inventory_id"]=str(f"{inventory_base}-{unit:03d}" if inventory_base and qty>1 else (inventory_base or f"STOCK-{uuid4().hex[:12].upper()}"))
                # 托盘规格：计划/默认值；视觉读入后走 resolve_pallet_specs 的 measured 分支。
                specs = resolve_pallet_specs(c)
                c.update(specs)
                self.queue.append(c)
        # All task cargo exists from the initial scene. Pickup and the one-pass
        # camera route start at the left tail. Target-side selection begins only
        # after the placement target has been locked.
        # Explicit task/recognition poses remain authoritative when supplied.
        for index, cargo in enumerate(self.queue):
            cargo.setdefault("pose", {
                "x_mm": -2400.0,
                "y_mm": 1000.0 - index * max(1450.0, float(cargo.get("width_mm", 1000) or 1000) + 350.0),
                "z_mm": 0.0,
                "roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": -90.0,
            })
            cargo["status"] = "STAGED"
            cargo["attached_to"] = None
            cargo["attachment"] = None
        self.reset(keep_plan=True)
        self.vehicle_db.register_inventory(self.queue)

    def set_debug_inputs(self, values: Dict[str, Any]):
        self.debug_inputs=deepcopy(values or {})
        selected = str(self.debug_inputs.get("selected_radar_pcd") or "").strip()
        if selected:
            self.debug_inputs["point_cloud_path"] = selected
        self.debug_inputs["point_cloud_example_only"] = False
        if hasattr(self.radar,"set_point_cloud_path"):
            self.radar.set_point_cloud_path(self.debug_inputs.get("point_cloud_path", ""))
        pallet_rgb=self.debug_inputs.get("pallet_rgb", ""); pallet_depth=self.debug_inputs.get("pallet_depth", "")
        if pallet_rgb and hasattr(self.camera,"set_tagged_image"):
            self.camera.set_tagged_image("pallet_hole",pallet_rgb)
        if pallet_depth and hasattr(self.camera,"set_tagged_depth"):
            self.camera.set_tagged_depth("pallet_hole",pallet_depth)
        for tag,path in (self.debug_inputs.get("images") or {}).items():
            if hasattr(self.camera,"set_tagged_image"): self.camera.set_tagged_image(tag,path)
        for tag,path in (self.debug_inputs.get("corner_images") or {}).items():
            if hasattr(self.camera,"set_tagged_image"): self.camera.set_tagged_image(tag,path)
        for tag,path in (self.debug_inputs.get("corner_depths") or {}).items():
            if hasattr(self.camera,"set_tagged_depth"): self.camera.set_tagged_depth(tag,path)
        for tag,path in (self.debug_inputs.get("depths") or {}).items():
            if hasattr(self.camera,"set_tagged_depth"): self.camera.set_tagged_depth(tag,path)
        # 真机策略可能清空了部分 tag；再刷一遍，避免上面循环把空路径漏掉。

    def select_radar_pcd(self, raw: str) -> str:
        path = Path(raw).expanduser()
        if path.suffix.lower() != ".pcd" or not path.is_file():
            raise ValueError(f"请选择存在的 PCD 文件：{path}")
        selected = str(path.resolve())
        self.set_debug_inputs({
            **self.debug_inputs,
            "point_cloud_path": selected,
            "point_cloud_example_only": False,
            "radar_pcd_selected": True,
        })
        print(f"[RADAR_TEST] 已选择 PCD：{selected}", flush=True)
        return selected

    def _require_selected_radar_pcd(self) -> str:
        raw = str(self.debug_inputs.get("point_cloud_path") or "")
        path = Path(raw).expanduser()
        if not self.debug_inputs.get("radar_pcd_selected") or path.suffix.lower() != ".pcd" or not path.is_file():
            raise RuntimeError("请先选择有效的 PCD 文件，未选择前不能运行雷达算法。")
        return str(path.resolve())

    def _use_live_radar(self) -> bool:
        return False





    def default_debug_inputs(self):
        try:
            from config.offline_debug_inputs import build_debug_inputs_from_offline

            return build_debug_inputs_from_offline(self.module_evidence.example_debug_inputs())
        except Exception:
            return deepcopy(self.debug_inputs or {"offline_field_test": True, "point_cloud_path": "", "point_cloud_example_only": False})

    def _evidence(self, module_id: str, inputs: Dict[str, Any], output: Any, started: float,
                  model_invoked: bool = False, note: str = "", status: str | None = None):
        final_status=status or ("SUCCESS" if isinstance(output,Mapping) and output.get("success",True) else "FAILED")
        evidence=self.module_evidence.record(
            module_id,inputs,output,(perf_counter()-started)*1000.0,
            status=final_status,model_invoked=model_invoked,used_by_flow=True,note=note,
        )
        if self.loading_session_id:
            self.vehicle_db.record_module_run(
                self.loading_session_id,evidence,
                str((self.current_cargo or {}).get("instance_id") or ""),
            )
        return evidence

    def reset(self, keep_plan=False):
        if getattr(self,"loading_session_id","") and not self.finished:
            self.vehicle_db.finish_session(self.loading_session_id,"RESET","用户重置或重新加载装载计划")
        q=deepcopy(self.queue) if keep_plan else []
        self.twin.reset(); self.queue=q; self.completed=[]; self.round_index=0; self.step_index=0
        self.running=False; self.finished=False; self.round_data={}; self.results=[]; self.truck_initialized=False
        self.field_camera_gallery = []
        self.space.reset()
        self.loading_session_id=""; self._db_message_cursor=0
        self.twin.set_cargo_inventory(self.queue)
        self._save()

    def start(self):
        if self.is_radar_only_test():
            self._require_selected_radar_pcd()
            self.running = True
            self.finished = False
            self.twin.set_phase(self.current_step[1], 1)
            return self.snapshot()
        if not self.queue: raise RuntimeError("装载计划为空")
        if not self.loading_session_id:
            truck=self.twin.snapshot().get("truck") or {}
            first=self.queue[0] if self.queue else {}
            task_code=str(first.get("task_code") or first.get("task_id") or "")
            self.loading_session_id=self.vehicle_db.start_session(truck,self.queue,task_code)
        self.running=True; self.finished=False; self._load_cargo_to_twin(); self._persist_database_snapshot(); return self.snapshot()

    def _load_cargo_to_twin(self):
        cargo_id=(self.current_cargo or {}).get("instance_id")
        inventory=self.twin.snapshot().get("cargo_inventory") or []
        c=deepcopy(next((item for item in inventory if item.get("instance_id")==cargo_id),self.current_cargo or {}))
        c["status"]="CURRENT"; self.twin.set_cargo(c); self.twin.set_phase(self.current_step[1],self.round_index+1)

    def _apply_board_specs(self, *, corners=None, geometry=None, radar_result=None, source: str = "") -> dict[str, float]:
        """车板规格 → 孪生卡车尺寸 + vehicle 表；角点未齐时也可先用雷达/几何尺寸驱动缩放。"""
        radar_specs = {}
        if isinstance(radar_result, Mapping):
            ref = radar_result.get("radar_raw_board_analysis_reference_only") or {}
            radar_specs = merge_board_specs(
                {
                    "length_mm": radar_result.get("length_mm") or ref.get("length_mm"),
                    "width_mm": radar_result.get("width_mm") or ref.get("width_mm"),
                }
            )
        truck = (self.twin.snapshot().get("truck") or {})
        specs = merge_board_specs(
            {
                "length_mm": truck.get("length_mm"),
                "width_mm": truck.get("width_mm"),
                "deck_height_mm": truck.get("deck_height_mm"),
            },
            radar_specs,
            board_extent_from_geometry(geometry),
            board_extent_from_corners(corners if corners is not None else truck.get("corners")),
        )
        if not specs:
            return {}
        patch = dict(specs)
        if source:
            patch["board_specs_source"] = source
        self.twin.update_truck(**patch)
        if self.loading_session_id:
            self.vehicle_db.update_vehicle_board_specs(self.loading_session_id, specs)
        return specs

    def _record(self, code, name, status, message, data=None):
        rec={"time":datetime.now().isoformat(timespec="seconds"),"round":self.round_index+1,"step_code":code,"step_name":name,"status":status,"cargo_id":(self.current_cargo or {}).get("instance_id"),"message":str(message),"data":deepcopy(data)}
        self.results.append(rec); self.twin.add_result(rec); self.twin.add_message(code,status.upper(),message,data)
        if self.loading_session_id:
            self.vehicle_db.record_step(self.loading_session_id,rec)
            twin=self.twin.snapshot()
            board_codes = {"INITIAL_SPACE_PLAN", "FIELD_CORNER_SHELL", "FIELD_SENSE"}
            if code in board_codes and isinstance(data, Mapping) and status in {"success", "warning"}:
                geometry = data.get("geometry") or (data.get("field_space_plan") or {}).get("geometry") or {}
                if not geometry and code == "FIELD_SENSE":
                    geometry = {
                        "decision_source": "field_radar_coarse",
                        "board_mode": (twin.get("truck") or {}).get("board_mode"),
                        "length_mm": (twin.get("truck") or {}).get("length_mm"),
                        "width_mm": (twin.get("truck") or {}).get("width_mm"),
                    }
                if geometry or (twin.get("truck") or {}).get("corners"):
                    self.vehicle_db.record_board_snapshot(
                        self.loading_session_id,
                        self.round_index + 1,
                        twin.get("truck") or {},
                        geometry if isinstance(geometry, Mapping) else {},
                    )
            if code=="PLACE" and status=="success" and isinstance(data,Mapping):
                self.vehicle_db.record_placement(self.loading_session_id,self.round_index+1,rec.get("cargo_id") or "",data)
            self._persist_database_snapshot(twin)
        self._save(); return rec

    def _persist_database_snapshot(self,twin=None):
        if not self.loading_session_id: return
        twin=twin or self.twin.snapshot()
        self.vehicle_db.sync_state(self.loading_session_id,twin)
        messages=list(twin.get("messages") or [])
        if self._db_message_cursor>len(messages): self._db_message_cursor=0
        self.vehicle_db.record_messages(self.loading_session_id,messages[self._db_message_cursor:])
        self._db_message_cursor=len(messages)

    def _capture_rgb(self, camera_id: str, tag: str, required=True):
        cap=self.camera.capture_rgb(camera_id,tag=tag)
        # 真机适配器常用 rgb_path；流程统一吃 image_path
        if isinstance(cap, dict) and not cap.get("image_path") and cap.get("rgb_path"):
            cap["image_path"]=cap["rgb_path"]
        if required and not cap.get("success"): raise RuntimeError(f"{camera_id} 未取得 {tag} JPG")
        if required and not cap.get("image_path"):
            raise RuntimeError(f"{camera_id} 取得了 {tag} 结果但缺少 image_path/rgb_path")
        return cap

    def _capture_rgbd(self, camera_id: str, tag: str):
        cap=self.camera.capture_rgbd(camera_id,tag=tag)
        if not cap.get("success"): raise RuntimeError(f"{camera_id} 未取得 {tag} 的 RGB + 对齐深度")
        return cap

    def _probe_one_device(self, device_id: str, probe_fn, timeout_s: float = 8.0) -> dict:
        try:
            with ThreadPoolExecutor(max_workers=1) as pool:
                raw = pool.submit(probe_fn).result(timeout=max(1.0, float(timeout_s)))
            out = dict(raw or {})
            out.setdefault("device_id", device_id)
            out.setdefault("success", False)
            out.setdefault("message", "无返回信息")
            return out
        except FuturesTimeoutError:
            self.twin.update_device(device_id, status="OFFLINE", task="PROBE_TIMEOUT")
            return {
                "success": False,
                "device_id": device_id,
                "message": f"{device_id} 探测超时（>{timeout_s:.0f}s）",
            }
        except Exception as exc:
            self.twin.update_device(device_id, status="FAILED", task="PROBE_ERROR")
            return {"success": False, "device_id": device_id, "message": f"{device_id} 探测异常：{exc}"}

    def _camera_ids_for_device_check(self) -> list[str]:
        """现场离线流程检查海康角点与梅卡托盘适配器。"""
        cams = list((self.device_config.get("cameras") or {}).keys())
        ordered = [cid for cid in ("CAM_CORNER", "CAM_PALLET") if cid in cams]
        return ordered or ["CAM_CORNER", "CAM_PALLET"]

    def _device_check(self):
        """第1步：检查 PLC / 雷达 / 相机连接。失败只记日志，不阻断下一步。"""
        checks = []

        def _plc():
            return self.plc.connect()

        def _radar():
            if hasattr(self.radar, "probe"):
                return self.radar.probe()
            self.twin.update_device("RADAR", status="ONLINE", task="NO_PROBE")
            return {"success": True, "device_id": "RADAR", "message": "雷达适配器无 probe，已跳过硬件探测"}

        def _camera_one(camera_id: str):
            if hasattr(self.camera, "probe_camera"):
                return self.camera.probe_camera(camera_id)
            if camera_id == "CAM_PICK":
                if hasattr(self.camera, "connect"):
                    return self.camera.connect()
                if hasattr(self.camera, "probe"):
                    return self.camera.probe()
            self.twin.update_camera(camera_id, status="ONLINE", task="NO_PROBE")
            return {"success": True, "device_id": camera_id, "message": f"{camera_id} 无独立探测接口，已跳过"}

        probe_plan = [("PLC", _plc), ("RADAR", _radar)]
        for cam_id in self._camera_ids_for_device_check():
            probe_plan.append((cam_id, lambda cid=cam_id: _camera_one(cid)))

        camera_ids = set(self._camera_ids_for_device_check())
        for device_id, fn in probe_plan:
            item = self._probe_one_device(device_id, fn)
            item.setdefault("device_id", device_id)
            checks.append(item)
            ok_one = bool(item.get("success"))
            status = "SUCCESS" if ok_one else "FAILED"
            twin_status = "ONLINE" if ok_one else "OFFLINE"
            if device_id in camera_ids or device_id.startswith("CAM_"):
                self.twin.update_camera(device_id, status=twin_status, task=status)
            else:
                self.twin.update_device(device_id, status=twin_status, task=status)
            self.twin.add_message(
                "DEVICE_CHECK",
                status,
                f"{device_id}：{item.get('message')}",
                item,
            )

        ok = [c for c in checks if c.get("success")]
        bad = [c for c in checks if not c.get("success")]
        all_ok = len(bad) == 0
        summary = (
            "外接设备全部就绪：" + "、".join(c["device_id"] for c in ok)
            if all_ok
            else (
                "外接设备检查未全部通过；失败："
                + "、".join(f"{c['device_id']}({c.get('message')})" for c in bad)
                + "（已记录，继续下一步）"
            )
        )
        result = {
            "success": all_ok,
            "continue_anyway": True,
            "all_online": all_ok,
            "online_count": len(ok),
            "failed_count": len(bad),
            "devices": {c["device_id"]: c for c in checks},
            "checks": checks,
            "message": summary,
            "device_mode": getattr(self, "device_mode", "mock"),
        }
        self.twin.add_message("DEVICE_CHECK", "SUCCESS" if all_ok else "FAILED", summary, result)
        self.round_data["device_check"] = result
        return result



    @staticmethod
    def _point_dict(v):
        if isinstance(v,Mapping): return {"x":float(v.get("x",v.get("x_mm",0))),"y":float(v.get("y",v.get("y_mm",0))),"z":float(v.get("z",v.get("z_mm",0)))}
        return {"x":float(v[0]),"y":float(v[1]),"z":float(v[2])}

    @staticmethod
    def _avg_point(a,b):
        return {k:0.5*(float(a[k])+float(b[k])) for k in ("x","y","z")}

    def _normalize_radar_result(self, raw: Dict[str,Any]) -> Dict[str,Any]:
        r=deepcopy(raw); frame=str(r.get("coordinate_frame") or "world").lower()
        world_raw=r.get("world_points") or {}
        ids=[str(x) for x in (r.get("corner_ids") or list(world_raw))]
        if not world_raw and isinstance(r.get("corner_points"),list):
            for item in r["corner_points"]:
                if isinstance(item,Mapping) and item.get("name"): world_raw[str(item["name"])]=item
            ids=[str(x) for x in (r.get("corner_ids") or list(world_raw))]
        world={}
        for pid in ids:
            if pid not in world_raw: continue
            p=self._point_dict(world_raw[pid])
            if frame not in {"world","radar_world"}:
                q=self.calibration.lidar_point_to_world([p["x"],p["y"],p["z"]]); p={"x":float(q[0]),"y":float(q[1]),"z":float(q[2])}
            world[pid]={axis:round_measurement(p[axis]) for axis in ("x","y","z")}
        ids=[i for i in ids if i in world]
        if len(ids) not in {4,6,8}: raise RuntimeError(f"雷达找车必须输出4或8个角点（兼容历史6点），当前：{ids}")
        return {
            "success":True,"message":r.get("message","雷达找车完成"),"coordinate_frame":"world","coordinate_unit":"mm",
            "corner_ids":ids,"world_points":world,"coarse_only":True,"collapsed_from_8_points":False,
            "raw_radar_reference":{k:deepcopy(r.get(k)) for k in ("coordinate_frame","board_mode","boards","first_workface_loading_plan")},
            "final_board_judgement_source":"camera_not_radar",
        }

    def _run_radar_only_test(self):
        path = self._require_selected_radar_pcd()
        print(f"[RADAR_TEST] 开始雷达算法：{path}", flush=True)
        started = perf_counter()
        self.twin.set_parallel("radar", "RUNNING", f"处理 PCD：{path}")
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(
                self.algorithms.radar_process,
                {"success": True, "pcd_path": path},
                {"instance_id": "radar_test"},
            )
            while not future.done():
                if callable(self.ui_event_pump):
                    self.ui_event_pump()
                sleep(0.02)
            output = future.result()
        self.round_data["radar_result"] = output
        self._evidence("POINTNET_TRUCK", {"pcd_path": path}, output, started,
                       model_invoked=True, note="仅雷达算法测试；未调用相机或 PLC 运动")
        status = "SUCCESS" if output.get("success") else "FAILED"
        self.twin.set_parallel("radar", status, output.get("message", ""), output)
        print(f"[RADAR_TEST] {status} 耗时={perf_counter()-started:.3f}s 结果="
              + json.dumps(output, ensure_ascii=False, default=str), flush=True)
        if not output.get("success"):
            raise RuntimeError(output.get("message") or "雷达算法失败")
        return output

    def _radar_task(self):
        if self.round_data.get("radar_result",{}).get("success"): return self.round_data["radar_result"]
        self.twin.set_parallel("radar","RUNNING","第2步 雷达离线 PCD 处理中")
        locate=self.radar.locate_truck(self.current_cargo)
        started=perf_counter(); raw_result=self.algorithms.radar_process(locate,self.current_cargo) if locate.get("success") else locate
        model_invoked=bool(locate.get("pcd_path"))
        # 真雷达实采时禁止再替换成联调粗点
        example_only = False
        algo_note = "现场离线 PCD 处理；最终板型由相机角点判断"
        self._evidence(
            "POINTNET_TRUCK",
            {
                "pcd_path":locate.get("pcd_path",""),
                "example_only":example_only,
                "live_radar":self._use_live_radar(),
                "locate_source":locate.get("source"),
            },
            raw_result,
            started,
            model_invoked=model_invoked,
            note=algo_note if model_invoked else "无PCD，点云算法未调用",
            status="SUCCESS" if raw_result.get("success") else "FAILED",
        )
        result=raw_result
        if result.get("success"): result=self._normalize_radar_result(result)
        self.round_data["radar_result"]=result
        status="SUCCESS" if result.get("success") else "FAILED"
        self.twin.set_parallel("radar",status,result.get("message",""),result)
        # TODO: 与PLC交互 — 上报雷达粗定位角点结果（3.2 并行支路）
        self.plc.send_message("RADAR_LOCATE",status,result.get("message",""),result)
        return result

    def _field_pick_recognize(self):
        """现场 2.1：到插孔位拍照识别两孔，发布坐标供 PLC 确认；不做物理插取。"""
        if self.round_data.get("pick_result", {}).get("success"):
            return self.round_data["pick_result"]
        self.twin.set_parallel("pick", "RUNNING", "现场：插孔拍照识别中")
        # 首轮使用启动前已调好的实际位置；后续轮先执行 FIELD_RETURN_ORIGIN。
        # 这里只实拍、识别、发送坐标消息，不生成机械臂运动或插取命令。
        cam = self._camera_for_role("pallet_hole", "CAM_PICK")
        cap = self._capture_rgbd(cam, tag="pallet_hole")
        if cap.get("success"):
            started = perf_counter()
            try:
                hole = self.algorithms.field_pallet_hole_recognize(
                    cap,
                    self.current_cargo,
                )
            except Exception as exc:
                hole = {
                    "success": False,
                    "algorithm": "traditional_opencv_fork_hole",
                    "recognition_source": "traditional_fork_hole_rgbd",
                    "result_image_path": str(cap.get("rgb_path") or ""),
                    "message": f"传统插孔算法识别失败：{exc}",
                }
            self._evidence(
                "FIELD_PALLET_HOLE_TRADITIONAL",
                {"rgb_path": cap["rgb_path"], "depth_path": cap["depth_path"]},
                hole,
                started,
                model_invoked=False,
                note="现场传统 OpenCV：实时 RGB-D 输出左右插孔相机坐标，再转 WORLD",
                status="SUCCESS" if hole.get("success") else "FAILED",
            )
        else:
            hole = {"success": False, "message": "缺少插孔RGB-D"}

        if hole.get("success") and hole.get("world_coordinate_frame") != "world":
            try:
                # 离线图像只能使用该次采集的记录位姿，当前 PLC 或孪生位置不代表照片位姿。
                raw_pose = cap.get("plc_pose") or self.debug_inputs.get("offline_plc_pose") or {}
                if not isinstance(raw_pose, Mapping) or not raw_pose:
                    raise ValueError("未提供照片采集时的 PLC 位姿；可在 config/offline_paths.py 的 mech_plc_pose 填写 x/y/z(mm)、r(deg)")
                plc_pose = {axis: float(raw_pose.get(axis, raw_pose.get(axis.upper()))) for axis in ("x", "y", "z", "r")}
                if not all(isfinite(value) for value in plc_pose.values()):
                    raise ValueError("照片采集位姿必须包含有限的 x/y/z(mm)、r(deg)")
                cap["plc_pose"] = deepcopy(plc_pose)
                transform = self.algorithms.field_camera_transform()
                for side in ("left", "right"):
                    xyz = hole.get(f"{side}_xyz_mm")
                    if xyz is not None:
                        q = transform.camera_to_world(xyz, plc_pose)
                        hole[f"{side}_world_xyz_mm"] = [float(x) for x in q]
                hole["world_coordinate_frame"] = "world"
                hole["world_transform"] = "field_camera_plc_extrinsic"
                hole["world_transform_status"] = "AVAILABLE"
                hole["plc_pose"] = deepcopy(plc_pose)
            except Exception as exc:
                # 相机识别仍是真实有效的结果；没有记录位姿时不生成 WORLD 或下发 WORLD 坐标。
                hole["world_transform_status"] = "UNAVAILABLE"
                hole["world_transform_warning"] = str(exc)
                hole["message"] = f"插孔相机坐标已识别，WORLD 坐标不可用：{exc}"

        if hole.get("success"):
            targets = {
                side: deepcopy(hole.get(f"{side}_world_xyz_mm"))
                for side in ("left", "right")
                if hole.get(f"{side}_world_xyz_mm") is not None
            }
            world_available = hole.get("world_coordinate_frame") == "world"
            coordinate_targets = targets if world_available else {
                side: deepcopy(hole.get(f"{side}_xyz_mm"))
                for side in ("left", "right") if hole.get(f"{side}_xyz_mm") is not None
            }
            coordinate_frame = "world" if world_available else "camera"
            coordinate_note = ("现场插孔 WORLD 坐标已记录，仅供 PLC/界面使用，不执行运动或插取" if world_available
                               else str(hole.get("message") or "已识别插孔相机坐标；未生成 WORLD 坐标"))
            coordinate_message = self.plc.send_message(
                "PALLET_HOLE_COORDINATES",
                "SUCCESS",
                coordinate_note,
                {
                    "coordinate_frame": coordinate_frame,
                    "coordinate_unit": "mm",
                    "targets": coordinate_targets,
                    "execute_motion": False,
                    "execute_fork": False,
                },
            )

            result = {
                "success": True,
                "field_mode": True,
                "fork_skipped": True,
                "motion_skipped": True,
                "message": coordinate_note,
                "hole_result": hole,
                "hole_world_targets": targets,
                "coordinate_message": coordinate_message,
                "capture": deepcopy(cap) if isinstance(cap, Mapping) else cap,
                "image_path": str(
                    hole.get("result_image_path")
                    or (cap or {}).get("rgb_path")
                    or ""
                ),
            }
        else:
            result = {
                "success": False,
                "field_mode": True,
                "fork_skipped": True,
                "message": hole.get("message", "插孔定位失败"),
                "hole_result": hole,
                "capture": deepcopy(cap) if isinstance(cap, Mapping) else cap,
                "image_path": str((cap or {}).get("rgb_path") or ""),
            }

        self.round_data["pick_result"] = result
        status = "SUCCESS" if result.get("success") else "FAILED"
        self._field_record_camera_view(
            step="pick_hole",
            title="第2步：托盘插孔识别",
            image_path=str(result.get("image_path") or ""),
            rgb_path=str((cap or {}).get("rgb_path") or ""),
            depth_path=str((cap or {}).get("depth_path") or ""),
            result_image_path=str((hole or {}).get("result_image_path") or result.get("image_path") or ""),
            status=status,
            message=str(result.get("message") or ""),
            # 插孔坐标固定由右侧插孔表显示；此处只记录识别图及孔中心像素供叠加展示。
            extra={
                "hole_world_targets": deepcopy(result.get("hole_world_targets") or {}),
                "left_hole_pixel": deepcopy((hole or {}).get("left_hole_pixel")),
                "right_hole_pixel": deepcopy((hole or {}).get("right_hole_pixel")),
            },
        )
        self.twin.set_parallel("pick", status, result.get("message", ""), result)
        self.plc.send_message("PALLET_PICK", status, result.get("message", ""), result)
        return result

    def _field_sense(self):
        """现场第2步：相机插孔识别 ∥ 雷达底板四角；任一失败即等待重试。

        后续轮次：只重跑插孔识别，复用首轮雷达车板（round_data 里预置的 radar_result / twin.corners）。
        """
        need_pick = not self.round_data.get("pick_result", {}).get("success")
        # 首轮才采雷达；后续轮若已有成功雷达结果则跳过
        need_radar = self.is_first_round and (not self.round_data.get("radar_result", {}).get("success"))
        with ThreadPoolExecutor(max_workers=1) as pool:
            fr = pool.submit(self._radar_task) if need_radar else None
            pick = self._field_pick_recognize() if need_pick else self.round_data["pick_result"]
            radar = fr.result() if fr else (self.round_data.get("radar_result") or {"success": True, "message": "复用首轮雷达车板", "skipped": True})

        if radar.get("success") and radar.get("world_points"):
            corners = {
                pid: {"x": float(p["x"]), "y": float(p["y"]), "z": float(p["z"])}
                for pid, p in (radar.get("world_points") or {}).items()
            }
            if corners:
                self.twin.update_truck(
                    corners=corners,
                    board_mode="FIELD_RADAR_COARSE",
                    camera_board_geometry={
                        "decision_source": "field_radar_coarse",
                        "corner_ids": list(radar.get("corner_ids") or corners.keys()),
                    },
                )
                self.truck_initialized = True
                self._apply_board_specs(corners=corners, radar_result=radar, source="radar_coarse")

        pick_ok = bool(pick.get("success"))
        radar_ok = bool(radar.get("success"))
        summary = {
            "success": pick_ok and radar_ok,
            "continue_anyway": False,
            "field_mode": True,
            "cycle_flow": True,
            "pick": "SUCCESS" if pick_ok else "FAILED",
            "radar": "SUCCESS" if radar_ok else "FAILED",
            "radar_skipped": bool(radar.get("skipped")) or (not need_radar),
            "pick_result": pick,
            "radar_result": radar,
            "message": (
                "感知完成：插孔与雷达四角均就绪"
                if pick_ok and radar_ok and need_radar
                else (
                    "感知完成：插孔已识别（雷达复用首轮）"
                    if pick_ok and not need_radar
                    else f"感知部分完成：插孔={('OK' if pick_ok else '失败')}，雷达={('OK' if radar_ok else '失败')}"
                )
            ),
        }
        truck_snap = self.twin.snapshot().get("truck") or {}
        summary["geometry"] = {
            "decision_source": "radar_coarse",
            "board_mode": truck_snap.get("board_mode"),
            "length_mm": truck_snap.get("length_mm"),
            "width_mm": truck_snap.get("width_mm"),
            "deck_height_mm": truck_snap.get("deck_height_mm"),
        }
        summary["board_specs"] = {
            "length_mm": truck_snap.get("length_mm"),
            "width_mm": truck_snap.get("width_mm"),
            "deck_height_mm": truck_snap.get("deck_height_mm"),
        }
        self.round_data["field_sense"] = summary
        if pick:
            self.round_data["pick_result"] = pick
        if radar and not radar.get("skipped"):
            self.round_data["radar_result"] = radar
        self.twin.add_message(
            "FIELD_SENSE",
            "SUCCESS" if summary["success"] else "WARNING",
            summary["message"],
            {"pick": summary["pick"], "radar": summary["radar"], "radar_skipped": summary["radar_skipped"]},
        )
        return summary



    def _field_corner_shell_from_radar(self) -> dict[str, Any]:
        """第3步：离线海康照片 + YOLO；WORLD/划格使用雷达角点（2D 相机无深度）。"""
        radar = self.round_data.get("radar_result") or {}
        radar_points = deepcopy(radar.get("world_points") or {})
        if not radar.get("success") or not radar_points:
            result = {
                "success": False,
                "field_mode": True,
                "camera_world_corners": {},
                "final_world_corners": {},
                "geometry": {},
                "message": "真实环境雷达没有可用 WORLD 角点，已停止划格",
            }
            self.round_data["field_corner_shell"] = result
            self.twin.add_message("FIELD_CORNER_SHELL", "WARNING", result["message"], result)
            return result

        corner_ids = [str(x) for x in (radar.get("corner_ids") or list(radar_points.keys()))]
        if len(corner_ids) == 8 and all(f"P{i}" in radar_points for i in range(1, 9)):
            # 相机搜索仍使用六个位置；仅在相机步骤生成，不修改雷达的八个真实角点。
            original = radar_points
            radar_points = {
                "P1": original["P1"], "P2": original["P2"],
                "P3": self._avg_point(original["P3"], original["P5"]),
                "P4": self._avg_point(original["P4"], original["P6"]),
                "P5": original["P7"], "P6": original["P8"],
            }
            corner_ids = [f"P{i}" for i in range(1, 7)]
        camera_id = self._camera_for_role("corner", "CAM_CORNER")
        capture = {"success": False, "message": "未采图"}
        vision = {"success": False, "message": "未识别"}
        try:
            image_paths = dict(self.debug_inputs.get("corner_images") or {})
            if not image_paths and self.debug_inputs.get("hik_corner_dir"):
                from services.offline_pipeline import build_corner_image_map
                image_paths = build_corner_image_map({"hik_corner_dir": self.debug_inputs["hik_corner_dir"]}, corner_ids)
            missing = [pid for pid in corner_ids if not Path(str(image_paths.get(pid) or "")).is_file()]
            if missing:
                raise RuntimeError("缺少离线海康角点照片：" + ", ".join(missing))
            image_paths = {pid: str(image_paths[pid]) for pid in corner_ids}
            image_path = image_paths[corner_ids[0]]
            capture = {"success": True, "camera_id": camera_id, "image_path": image_path, "source": "offline_hikvision_files"}
            self.round_data["corner_images"] = deepcopy(image_paths)
            started = perf_counter()
            vision = self.algorithms.corner_image_recognize(
                image_paths, corner_ids, self.current_cargo or {}
            )
            self._evidence(
                "CORNER_YOLO",
                {
                    "image_paths_by_point": image_paths,
                    "corner_ids": corner_ids,
                    "field_hikvision": True,
                    "capture": {
                        "image_path": image_path,
                        "ip": capture.get("ip"),
                        "source": capture.get("source"),
                    },
                },
                vision,
                started,
                model_invoked=True,
            )
            self.round_data["corner_recognition"] = deepcopy(vision)
        except Exception as exc:
            result = {
                "success": False,
                "field_mode": True,
                "camera_world_corners": {},
                "final_world_corners": {},
                "geometry": {},
                "capture": deepcopy(capture) if isinstance(capture, Mapping) else {},
                "message": (
                    f"现场离线海康读图/识别失败：{exc}。"
                    "请检查离线海康照片路径和角点 YOLO 权重。"
                ),
            }
            self.round_data["field_corner_shell"] = result
            self.twin.add_message("FIELD_CORNER_SHELL", "WARNING", result["message"], result)
            return result

        final_points = {}
        for pid, point in radar_points.items():
            p = dict(point)
            p.setdefault("source", "radar_field")
            final_points[str(pid)] = p
        try:
            field_space_plan = build_field_space_plan(
                final_points,
                corner_ids,
                board_geometry=self.board_geometry,
                space_manager=self.space,
                require_camera_source=False,
            )
        except Exception as exc:
            field_space_plan = {
                "success": False,
                "geometry": {},
                "space": {},
                "message": f"真实环境划格失败：{exc}",
            }
        self.round_data["field_space_plan"] = deepcopy(field_space_plan)
        space_snapshot = field_space_plan.get("space") or {}
        geometry = field_space_plan.get("geometry") or {}
        self.twin.update_truck(
            corners=deepcopy(final_points),
            board_mode=str(geometry.get("board_mode") or "FIELD_HIK_VISION_BOARD"),
            camera_board_geometry={
                "decision_source": "field_hikvision_rgb_plus_radar_world",
                "corner_sources": {pid: point.get("source") for pid, point in final_points.items()},
                "vision_success": bool(vision.get("success")),
                "capture_image": capture.get("image_path") or capture.get("rgb_path"),
                "loading_column": field_space_plan.get("loading_column"),
                "loading_order": deepcopy(field_space_plan.get("loading_order") or []),
            },
            regions=deepcopy(space_snapshot.get("regions") or []),
            occupied=deepcopy(space_snapshot.get("occupied") or []),
            available=deepcopy(space_snapshot.get("available") or []),
            remaining_space={
                "available_b_regions": deepcopy(field_space_plan.get("loading_order") or []),
            },
        )
        board_specs = self._apply_board_specs(
            corners=final_points,
            geometry=geometry,
            radar_result=radar,
            source="field_hikvision_board",
        )
        self.truck_initialized = True
        self.round_data["camera_world_corners"] = {}
        self.round_data["final_world_corners"] = deepcopy(final_points)
        vision_ok = bool(vision.get("success"))
        plan_ok = bool(field_space_plan.get("success"))
        result = {
            "success": plan_ok and vision_ok,
            "field_mode": True,
            "hikvision_capture": {
                "success": True,
                "image_path": capture.get("image_path") or capture.get("rgb_path"),
                "ip": capture.get("ip"),
                "source": capture.get("source"),
            },
            "corner_recognition": {
                "success": vision_ok,
                "message": vision.get("message"),
                "result_dir": vision.get("result_dir"),
                "physical_image_count": vision.get("physical_image_count"),
            },
            "camera_world_corners": {},
            "final_world_corners": deepcopy(final_points),
            "geometry": deepcopy(geometry),
            "field_space_plan": {
                "success": plan_ok,
                "board_mode": geometry.get("board_mode"),
                "region_count": len(space_snapshot.get("regions") or []),
                "available_count": len(space_snapshot.get("available") or []),
                "loading_column": field_space_plan.get("loading_column"),
                "loading_order": deepcopy(field_space_plan.get("loading_order") or []),
                "message": field_space_plan.get("message"),
            },
            "board_specs": board_specs,
            "message": (
                f"现场海康已拍照并识别（{vision.get('message')}）；"
                f"WORLD/划格用雷达：{field_space_plan.get('message')}；"
                f"车板约 {board_specs.get('length_mm', '-')}×{board_specs.get('width_mm', '-')} mm"
            ),
        }
        self.round_data["field_corner_shell"] = result
        self.twin.add_message(
            "FIELD_CORNER_SHELL",
            "SUCCESS" if result["success"] else "WARNING",
            result["message"],
            result,
        )
        self.twin.set_phase(result["message"], self.round_index + 1)
        return result

    def _field_corner_shell(self):
        return self._field_corner_shell_from_radar()

    def _field_record_camera_view(
        self,
        *,
        step: str,
        title: str,
        image_path: str = "",
        rgb_path: str = "",
        depth_path: str = "",
        result_image_path: str = "",
        region_id: str = "",
        status: str = "",
        message: str = "",
        corners_world: list | None = None,
        extra: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """记录现场每次拍照/识别，供右上角窗口按最新结果刷新。"""
        display = str(result_image_path or image_path or rgb_path or "")
        entry = {
            "time": datetime.now().isoformat(timespec="seconds"),
            "step": str(step),
            "title": str(title),
            "region_id": str(region_id or ""),
            "status": str(status or ""),
            "message": str(message or ""),
            "image_path": display,
            "rgb_path": str(rgb_path or ""),
            "depth_path": str(depth_path or ""),
            "result_image_path": str(result_image_path or ""),
            "corners_world_xyz_mm": deepcopy(corners_world or []),
            "extra": deepcopy(dict(extra or {})),
        }
        self.field_camera_gallery.append(entry)
        gallery = self.round_data.setdefault("field_camera_gallery", [])
        if isinstance(gallery, list):
            gallery.append(deepcopy(entry))
        self.round_data["field_latest_camera_view"] = deepcopy(entry)
        if self.state_listener:
            try:
                self.state_listener(self.snapshot(), {"field_camera_view": deepcopy(entry)})
            except Exception:
                pass
        return entry


    def _offline_place_capture(self, capture_tag: str) -> dict[str, Any]:
        """第4步离线读图：优先 debug tag，其次 place_verify_*，再退回梅卡图。"""
        tag = str(capture_tag or "field_place_verify")
        images = dict((self.debug_inputs or {}).get("images") or {})
        depths = dict((self.debug_inputs or {}).get("depths") or {})
        rgb = str(
            images.get(tag)
            or (self.debug_inputs or {}).get("place_verify_rgb")
            or (self.debug_inputs or {}).get("pallet_rgb")
            or ""
        ).strip()
        depth = str(
            depths.get(tag)
            or (self.debug_inputs or {}).get("place_verify_depth")
            or (self.debug_inputs or {}).get("pallet_depth")
            or ""
        ).strip()
        if rgb and hasattr(self.camera, "set_tagged_image"):
            self.camera.set_tagged_image(tag, rgb)
        if depth and hasattr(self.camera, "set_tagged_depth"):
            self.camera.set_tagged_depth(tag, depth)
        camera_id = self._camera_for_role("observe", "CAM_PICK") or self._camera_for_role("pallet_hole", "CAM_PICK")
        if rgb:
            try:
                return self.camera.capture_rgbd(camera_id, tag=tag)
            except Exception as exc:
                return {
                    "success": True,
                    "rgb_path": rgb,
                    "depth_path": depth,
                    "source": "offline_file",
                    "demo": True,
                    "message": f"离线文件直读（capture 异常已忽略：{exc}）",
                }
        return {
            "success": False,
            "rgb_path": "",
            "depth_path": "",
            "source": "offline_missing",
            "message": "未配置第4步离线照片（place_verify_dir）",
        }

    def _field_dynamic_region_check(
        self,
        region: Mapping[str, Any],
        *,
        phase: str,
        task: str,
        capture_tag: str,
    ) -> dict[str, Any]:
        region_id = str(region.get("region_id") or "B")
        offline = self._is_offline_field_test()
        if offline:
            movement = {
                "success": True,
                "region_id": region_id,
                "region_center_world_xyz_mm": list(region.get("center_world_xyz_mm") or [0.0, 0.0, 0.0]),
                "message": "现场离线：跳过 PLC 到位，直接读离线照片",
                "motion": {},
                "target_world_pose": {},
            }
            capture = self._offline_place_capture(capture_tag)
            capture["plc_pose"] = {"x": 0.0, "y": 0.0, "z": 380.0, "r": 0.0}
        shot_ok = bool(capture.get("success"))
        self.twin.add_message(
            "FIELD_REGION",
            "SUCCESS" if shot_ok else "FAILED",
            (
                f"{region_id} 已读入离线 RGB-D" if offline and shot_ok
                else (
                    f"{region_id} 已拍照 RGB-D"
                    if shot_ok
                    else f"{region_id} 拍照失败：{capture.get('message')}"
                )
            ),
            {
                "rgb_path": capture.get("rgb_path"),
                "depth_path": capture.get("depth_path"),
                "source": capture.get("source"),
                "offline_field_test": offline,
            },
        )
        started = perf_counter()
        try:
            result = self.field_box_monitor.analyze_capture(
                capture,
                region,
                phase=phase,
                result_tag=region_id,
            )
        except Exception as exc:
            result = {
                "success": False,
                "message": f"判区算法异常：{exc}",
                "inside_planned_region": None,
                "result_image_path": str(capture.get("rgb_path") or ""),
            }
        # 离线联调：只要读到图就软通过，保证流程能走完（算法对错不归本工具卡死）
        if offline and capture.get("rgb_path"):
            result["success"] = True
            result["offline_soft_pass"] = True
            result["message"] = str(result.get("message") or "离线读图完成") + "（离线软通过，不卡流程）"
        result["movement"] = movement
        result["capture"] = deepcopy(capture)
        result["image_path"] = str(result.get("result_image_path") or capture.get("rgb_path") or "")
        inside = result.get("inside_planned_region")
        judgement = "全部在规划区域内" if inside is True else ("存在角点越界" if inside is False else "未得到有效判区")
        self.twin.add_message(
            "FIELD_REGION",
            "SUCCESS" if result.get("success") else "FAILED",
            f"{region_id} 纸箱识别：{result.get('message')}；判定：{judgement}",
            {
                "inside_planned_region": result.get("inside_planned_region"),
                "result_image_path": result.get("result_image_path"),
                "outside_corners": result.get("outside_corners"),
            },
        )
        self._field_record_camera_view(
            step=str(phase),
            title=f"{region_id} {'放货前监测' if phase == 'pre_place' else '放货后检测'}",
            image_path=str(result.get("image_path") or ""),
            rgb_path=str(capture.get("rgb_path") or ""),
            depth_path=str(capture.get("depth_path") or ""),
            result_image_path=str(result.get("result_image_path") or ""),
            region_id=region_id,
            status=str(result.get("status") or ("PASS" if result.get("success") else "FAILED")),
            message=str(result.get("message") or ""),
            extra={
                "inside_planned_region": result.get("inside_planned_region"),
                "outside_corners": result.get("outside_corners"),
                "box_count": result.get("box_count"),
            },
        )
        module_id = "FIELD_DYNAMIC_PRE_PLACE" if phase == "pre_place" else "FIELD_DYNAMIC_POST_PLACE"
        self._evidence(
            module_id,
            {
                "rgb_path": capture.get("rgb_path"),
                "depth_path": capture.get("depth_path"),
                "camera_world_pose": capture.get("camera_world_pose"),
                "plc_pose": capture.get("plc_pose"),
                "planned_region": deepcopy(region),
            },
            result,
            started,
            model_invoked=False,
            note=(
                "现场离线：读 place_verify 文件，跳过现场真机拍照"
                if offline
                else "现场 dynamic_box_monitor：传统 OpenCV 纸箱角点 + D435i 深度转 WORLD 后判区"
            ),
            status="SUCCESS" if result.get("success") else "FAILED",
        )
        status = "SUCCESS" if result.get("success") else "FAILED"
        self.plc.send_message(module_id, status, result.get("message", "现场动态监测完成"), result)
        return result

    def _field_return_origin(self):
        result = {"success": True, "offline_soft_pass": True, "message": "现场离线：跳过返回原点", "motion": {"success": True}}
        self.twin.add_message("FIELD_RETURN_ORIGIN", "SUCCESS", result["message"], result)
        return result

    def _field_pre_place_monitor(self) -> dict[str, Any]:
        """下一件货物放置前，复查上一件货物所在 B 区。"""
        try:
            previous_region = field_b_region_for_round(self.space.snapshot(), self.round_index - 1)
        except Exception:
            if self._is_offline_field_test():
                previous_region = self._offline_dummy_b_region()
            else:
                raise
        result = self._field_dynamic_region_check(
            previous_region,
            phase="pre_place",
            task=f"FIELD_PRE_PLACE_{previous_region['region_id']}",
            capture_tag="field_pre_place_monitor",
        )
        self.round_data["field_pre_place_monitor"] = result
        return result

    def _field_place_verify(self):
        """人工放货后：现场离线读图判区；现场则到位实拍。"""
        region = self._resolve_b_region_for_round()
        result = self._field_dynamic_region_check(
            region,
            phase="post_place",
            task=f"FIELD_POST_PLACE_{region['region_id']}",
            capture_tag="field_place_verify",
        )
        result["field_mode"] = True
        result["manual_place"] = True
        result["target_region"] = deepcopy(region)
        occupied = self.space.occupy_region_if_passed(
            str(region.get("region_id")),
            str((self.current_cargo or {}).get("instance_id") or f"LAB-CARGO-{self.round_index + 1}"),
            result,
        )
        result["occupied_region"] = occupied
        if occupied:
            cargo = deepcopy(self.twin.snapshot().get("cargo") or {})
            if cargo:
                cargo["status"] = "PLACED_MANUAL_LAB"
                cargo["attached_to"] = None
                cargo["planned_region_id"] = region.get("region_id")
                self.twin.set_cargo(cargo)
            snapshot = self.space.snapshot()
            self.twin.update_truck(
                current_target={
                    "region_id": region.get("region_id"),
                    "blind_code": region.get("blind_code"),
                    "nominal_world_pose": {
                        "x_mm": float(region["center_world_xyz_mm"][0]),
                        "y_mm": float(region["center_world_xyz_mm"][1]),
                        "z_mm": float(region["center_world_xyz_mm"][2]),
                    },
                },
                regions=snapshot["regions"],
                occupied=snapshot["occupied"],
                available=snapshot["available"],
                remaining_space={
                    "available_b_regions": [item["region_id"] for item in field_b_regions(snapshot) if item.get("status") == "AVAILABLE"],
                },
            )
        self.round_data["field_place_verify"] = result
        status = "SUCCESS" if result.get("success") else "FAILED"
        self.twin.add_message("FIELD_PLACE_VERIFY", status, result["message"], result)
        self.plc.send_message("FIELD_PLACE_VERIFY", status, result["message"], result)
        return result

    def _field_manual_place_confirmed(self) -> dict[str, Any]:
        """记录人工已放好当前 B 区货物；下一子步骤立即执行相机判区。"""
        region = self._resolve_b_region_for_round()
        result = {
            "success": True,
            "manual_place": True,
            "region_id": str(region.get("region_id") or "B"),
            "message": (
                f"已确认人工放置 {region.get('region_id')}，开始离线读图检测"
                if self._is_offline_field_test()
                else f"已确认人工放置 {region.get('region_id')}，开始相机拍照检测"
            ),
            "offline_field_test": self._is_offline_field_test(),
        }
        self.round_data["field_manual_place_confirmed"] = deepcopy(result)
        self.twin.add_message("FIELD_WAIT_MANUAL_PLACE", "SUCCESS", result["message"], result)
        return result

    def _advance_field_round(self):
        """本轮 B 区检测结束即进入下一件；车板几何永久复用，不再重复扫描。"""
        self.completed.append(deepcopy(self.round_data))
        preserved_radar = None
        preserved_camera = None
        preserved_final = None
        preserved_corner_shell = None
        for item in reversed(self.completed):
            radar = item.get("radar_result")
            if isinstance(radar, Mapping) and radar.get("success") and radar.get("world_points"):
                preserved_radar = deepcopy(radar)
            camera = item.get("camera_world_corners")
            final = item.get("final_world_corners")
            if camera and final:
                preserved_camera = deepcopy(camera)
                preserved_final = deepcopy(final)
                preserved_corner_shell = deepcopy(item.get("field_corner_shell") or {})
            if preserved_radar and preserved_camera and preserved_final:
                break
        self.round_index += 1
        self.step_index = 0
        self.round_data = {}
        if preserved_radar:
            self.round_data["radar_result"] = preserved_radar
        if preserved_camera and preserved_final:
            self.round_data["camera_world_corners"] = preserved_camera
            self.round_data["final_world_corners"] = preserved_final
            self.round_data["field_corner_shell"] = {
                **preserved_corner_shell,
                "success": True,
                "reused": True,
                "camera_world_corners": deepcopy(preserved_camera),
                "final_world_corners": deepcopy(preserved_final),
                "message": "复用首轮相机最终角点与 B 列规划，不重复雷达/角点扫描",
            }
        b_regions = field_b_regions(self.space.snapshot())
        if self.round_index >= len(self.queue) or self.round_index >= len(b_regions):
            self.finished = True
            self.running = False
            reason = "现场 B 列装货检测完成" if self.round_index >= len(b_regions) else "现场全部货物流程完成"
            self.twin.set_phase(reason, self.round_index)
            if self.loading_session_id:
                self.vehicle_db.finish_session(self.loading_session_id, "COMPLETED")
            return {"finished": True, "message": reason}
        self._load_cargo_to_twin()
        self.twin.set_phase(self.current_step[1], self.round_index + 1)
        return {
            "finished": False,
            "next_round": self.round_index + 1,
            "next_region_id": b_regions[self.round_index].get("region_id"),
            "previous_region_id": b_regions[self.round_index - 1].get("region_id"),
        }





    def set_corner_review_callback(self, callback):
        """UI registers a blocking human-review dialog here for step 6."""
        self.corner_review_callback = callback if callable(callback) else None

    def _apply_corner_review(self, recognition: Dict[str, Any]) -> Dict[str, Any]:
        """Optional human review/correction after .pt corner recognition."""
        if not self.corner_review_enabled:
            recognition = deepcopy(recognition)
            recognition["reviewed"] = False
            recognition["review_skipped"] = True
            recognition["message"] = str(recognition.get("message") or "") + "｜人工审核已关闭"
            return recognition
        source = str(recognition.get("source") or recognition.get("recognition_source") or "")
        if self.corner_review_auto_accept_demo and source in {"generated_demo_rgbd"}:
            recognition = deepcopy(recognition)
            recognition["reviewed"] = True
            recognition["review_skipped"] = True
            recognition["corrected_count"] = 0
            recognition["message"] = str(recognition.get("message") or "") + "｜联调角点自动通过人工审核"
            return recognition
        if self.corner_review_callback is None:
            recognition = deepcopy(recognition)
            recognition["reviewed"] = False
            recognition["review_skipped"] = True
            recognition["message"] = str(recognition.get("message") or "") + "｜无界面审核回调，已跳过人工审核"
            return recognition
        ids = list(recognition.get("corner_ids") or self.round_data["radar_result"]["corner_ids"])
        image_points = recognition.get("image_points") or {}
        for pid in ids:
            point = dict(image_points.get(pid) or {})
            if not point.get("image"):
                point["image"] = (self.round_data.get("corner_images") or {}).get(pid)
                image_points[pid] = point
        reviewed = self.corner_review_callback(
            image_points,
            ids,
            str((self.current_cargo or {}).get("instance_id") or f"round{self.round_index+1}"),
        )
        if not isinstance(reviewed, Mapping) or not reviewed.get("success"):
            raise RuntimeError(str((reviewed or {}).get("message") or "角点人工审核未通过"))
        merged = deepcopy(recognition)
        merged.update(deepcopy(reviewed))
        merged["success"] = True
        merged["recognition_source"] = recognition.get("recognition_source") or recognition.get("source")
        merged["model_path"] = recognition.get("model_path")
        merged["result_dir"] = recognition.get("result_dir")
        started = perf_counter()
        self._evidence(
            "CORNER_REVIEW",
            {
                "corner_ids": ids,
                "model_image_points": recognition.get("image_points"),
                "corrected_point_ids": reviewed.get("corrected_point_ids") or [],
            },
            merged,
            started,
            model_invoked=False,
            note="角点人工审核/手动矫正",
            status="SUCCESS",
        )
        return merged
















    def execute_next(self):
        if self.finished: return self.snapshot()
        if not self.running: self.start()
        code,name=self.current_step; self.twin.set_phase(name,self.round_index+1)
        try:
            if code=="RADAR_TEST":
                data = self._run_radar_only_test()
                self.finished = True
                self.running = False
            elif code=="DEVICE_CHECK":
                data=self._device_check()
                if self.uses_coord_cycle_flow() and not data.get("all_online"):
                    detail = str(data.get("message") or "未知设备故障")
                    raise RuntimeError(f"设备未全部在线，已停止流程：{detail}")
            elif code=="FIELD_RETURN_ORIGIN":
                data=self._field_return_origin()
                if not data.get("success"):
                    raise RuntimeError(data.get("message") or "返回原点失败")
            elif code=="FIELD_SENSE":
                data=self._field_sense()
                if not data.get("success"):
                    raise RuntimeError(data.get("message") or "插孔识别或雷达四角失败")
            elif code=="FIELD_CORNER_SHELL":
                data=self._field_corner_shell()
                if not data.get("success"):
                    raise RuntimeError(data.get("message") or "WORLD 四角不完整，已停止划格")
            elif code=="FIELD_PRE_PLACE_MONITOR":
                data=self._field_pre_place_monitor()
                if not data.get("success"):
                    raise RuntimeError(data.get("message") or "上一 B 区放货前监测未通过")
            elif code=="FIELD_WAIT_MANUAL_PLACE":
                data=self._field_manual_place_confirmed()
                self._record(code,name,"success",str(data.get("message") or name),data)
                self.step_index+=1
                self.twin.set_alarm(None)
                self.twin.set_phase(self.current_step[1],self.round_index+1)
                self._save()
                return self.execute_next()
            elif code=="FIELD_PLACE_VERIFY":
                data=self._field_place_verify()
                self._record(
                    code,
                    name,
                    "success" if data.get("success") else "warning",
                    str((data or {}).get("message") or name),
                    data,
                )
                # 放货后检测只记录「在区 / 越界」结论；无论结论如何，都进入下一件循环。
                # 下一件放货前监测仍是门槛，发现偏移时不允许继续人工放下一件。
                advance=self._advance_field_round()
                self._persist_database_snapshot()
                self.twin.set_alarm(None)
                self._save()
                return self.snapshot()
            else: raise RuntimeError(code)
            rec_status="success"
            rec_message=name
            if code=="DEVICE_CHECK":
                soft_ok=bool((data or {}).get("all_online"))
                if not soft_ok:
                    rec_status="warning"
                    rec_message=str((data or {}).get("message") or name)
            elif code=="FIELD_SENSE":
                soft_ok=bool((data or {}).get("success"))
                if not soft_ok:
                    rec_status="warning"
                    rec_message=str((data or {}).get("message") or name)
            elif code=="FIELD_CORNER_SHELL":
                pass
            self._record(code,name,rec_status,rec_message,data); self.step_index+=1
            self.twin.set_alarm(None)
            self.twin.set_phase(self.current_step[1],self.round_index+1)
        except Exception as exc:
            self._record(code,name,"failed",str(exc),{"error":str(exc),"parallel":deepcopy(self.twin.snapshot().get("parallel"))}); self.twin.set_alarm(str(exc)); raise
        self._save(); return self.snapshot()

    def continue_after_step_failure(self, reason: str = "") -> dict:
        """用户确认在硬失败后仍前进：本步记 warning，step_index +1，不重跑失败逻辑。"""
        if self.finished:
            return self.snapshot()
        code, name = self.current_step
        detail = {
            "forced_continue": True,
            "reason": str(reason or "用户选择失败后继续"),
        }
        if self.uses_coord_cycle_flow() and code == "FIELD_PLACE_VERIFY":
            self._record(code, name, "warning", f"失败后继续：{detail['reason']}", detail)
            self.twin.set_alarm(None)
            advance = self._advance_field_round()
            self._persist_database_snapshot()
            self._save()
            return self.snapshot()
        self._record(
            code,
            name,
            "warning",
            f"失败后继续：{detail['reason']}",
            detail,
        )
        self.twin.set_alarm(None)
        self.step_index += 1
        if self.step_index >= len(self.steps):
            # 理论上 RETURN 成功才会换轮；若 RETURN 失败后仍继续，当作本轮结束尝试进入下一货
            self.step_index = len(self.steps) - 1
            try:
                advance = self._advance_field_round()
                if advance.get("finished") and self.loading_session_id:
                    self.vehicle_db.finish_session(self.loading_session_id, "COMPLETED")
            except Exception:
                self.finished = True
                self.running = False
        else:
            self.twin.set_phase(self.current_step[1], self.round_index + 1)
        self._save()
        return self.snapshot()

    def _save(self):
        STATE_FILE.parent.mkdir(parents=True,exist_ok=True)
        STATE_FILE.write_text(json.dumps(self.twin.snapshot(),ensure_ascii=False,indent=2,default=str),encoding="utf-8")
        RESULT_FILE.write_text(json.dumps({"results":self.results},ensure_ascii=False,indent=2,default=str),encoding="utf-8")

    def snapshot(self):
        return {
            "running": self.running,
            "finished": self.finished,
            "round": self.round_index + 1 if self.queue else 0,
            "total": len(self.queue),
            "completed": len(self.completed),
            "first_round": self.is_first_round,
            "step_code": self.current_step[0],
            "step_name": self.current_step[1],
            "current_cargo": deepcopy(self.current_cargo),
            "round_data": deepcopy(self.round_data),
            "field_camera_gallery": deepcopy(self.field_camera_gallery[-30:]),
            "field_latest_camera_view": deepcopy(
                self.round_data.get("field_latest_camera_view")
                or (self.field_camera_gallery[-1] if self.field_camera_gallery else {})
            ),
            "twin": self.twin.snapshot(),
            "results": deepcopy(self.results),
            "calibration": self.calibration.diagnostic_summary(),
            "module_evidence": self.module_evidence.snapshot(),
            "device_mode": getattr(self, "device_mode", "mock"),
            "run_profile": "field",
            "allow_demo": bool(self.allow_demo),
            "database": {
                "backend": self.vehicle_db.backend,
                "location": self.vehicle_db.location,
                "path": self.vehicle_db.location,
                "session_id": self.loading_session_id,
            },
        }
