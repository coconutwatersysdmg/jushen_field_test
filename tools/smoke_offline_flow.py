# -*- coding: utf-8 -*-
"""Headless smoke: walk field offline steps; path must resolve; algo may be stubbed."""
from __future__ import annotations

import json
import os
import sys
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

# Stub heavy PointNet/YOLO so smoke only validates path + step chain.
os.environ.setdefault("JUSHEN_SMOKE_STUB_ALGO", "1")


def _stub_heavy_algo(ctrl) -> None:
    if os.environ.get("JUSHEN_SMOKE_STUB_ALGO") != "1":
        return

    def _fake_radar():
        corners = {
            "P1": {"x": 0.0, "y": 0.0, "z": 0.0},
            "P2": {"x": 12000.0, "y": 0.0, "z": 0.0},
            "P3": {"x": 12000.0, "y": 2400.0, "z": 0.0},
            "P4": {"x": 0.0, "y": 2400.0, "z": 0.0},
        }
        return {
            "success": True,
            "message": "SMOKE stub radar",
            "world_points": corners,
            "corner_ids": list(corners.keys()),
            "pcd_path": str((ctrl.debug_inputs or {}).get("point_cloud_path") or ""),
        }

    def _fake_pick():
        return {
            "success": True,
            "message": "SMOKE stub pick",
            "image_path": str((ctrl.debug_inputs or {}).get("pallet_rgb") or ""),
            "hole_world_targets": {
                "left": {"x": 100.0, "y": 200.0, "z": 380.0},
                "right": {"x": 300.0, "y": 200.0, "z": 380.0},
            },
        }

    def _fake_corner_shell():
        truck = (ctrl.twin.snapshot().get("truck") or {})
        corners = dict(truck.get("corners") or {})
        if len(corners) < 4:
            corners = {
                "P1": {"x": 0.0, "y": 0.0, "z": 0.0},
                "P2": {"x": 12000.0, "y": 0.0, "z": 0.0},
                "P3": {"x": 12000.0, "y": 2400.0, "z": 0.0},
                "P4": {"x": 0.0, "y": 2400.0, "z": 0.0},
            }
            ctrl.twin.update_truck(corners=corners, board_mode="SMOKE")
        return {
            "success": True,
            "message": "SMOKE stub corner shell",
            "image_path": str(((ctrl.debug_inputs or {}).get("corner_images") or {}).get("P1") or ""),
            "world_points": corners,
        }

    ctrl._radar_task = _fake_radar  # type: ignore[method-assign]
    ctrl._lab_pick_recognize = _fake_pick  # type: ignore[method-assign]
    ctrl._lab_corner_shell = _fake_corner_shell  # type: ignore[method-assign]
    # Avoid blocking manual-place / monitor hard fails in first cycle
    ctrl._lab_pre_place_monitor = lambda: {"success": True, "message": "SMOKE stub pre-place"}  # type: ignore
    ctrl._lab_place_verify = lambda: {"success": True, "message": "SMOKE stub place verify", "status": "IN_ZONE"}  # type: ignore
    ctrl._lab_return_origin = lambda: {"success": True, "message": "SMOKE stub return origin"}  # type: ignore


def main() -> int:
    from config.offline_path_check import format_report_text, validate_offline_paths
    from controllers.flow_controller import FlowController

    report = validate_offline_paths()
    print(format_report_text(report), flush=True)
    if not report.get("ok"):
        print("PATH_CHECK_FAILED", flush=True)
        return 2

    ctrl = FlowController()
    plan_path = ROOT / "data" / "loading_plan.json"
    if plan_path.is_file():
        data = json.loads(plan_path.read_text(encoding="utf-8"))
        ctrl.set_plan(data.get("items", data))
    if not ctrl.queue:
        ctrl.set_plan(
            [
                {
                    "cargo_code": "CARGO-001",
                    "cargo_name": "smoke",
                    "quantity": 1,
                    "length_mm": 1200,
                    "width_mm": 1000,
                    "height_mm": 900,
                    "pallet_reference_width_mm": 1200,
                }
            ]
        )

    ctrl.set_corner_review_callback(lambda corners, meta=None: {"accepted": True, "corners": corners})
    _stub_heavy_algo(ctrl)
    ctrl.start()

    walked = []
    max_steps = max(8, len(getattr(ctrl, "steps", []) or []) + 2)
    for i in range(max_steps):
        if ctrl.finished:
            break
        snap = ctrl.snapshot()
        code = str(snap.get("step_code") or "")
        name = str(snap.get("step_name") or "")
        print(f"\n=== step {i+1}: {code} | {name} ===", flush=True)
        try:
            ctrl.execute_next()
            walked.append((code, "ok"))
            print(f"OK: {code}", flush=True)
        except Exception as exc:
            print(f"FAIL(soft-continue): {code}: {exc}", flush=True)
            traceback.print_exc(limit=2)
            try:
                ctrl.continue_after_step_failure(reason=str(exc))
                walked.append((code, "continued"))
            except Exception as exc2:
                print(f"CONTINUE_FAILED: {exc2}", flush=True)
                walked.append((code, "stuck"))
                break
        if code in {"LAB_PLACE_VERIFY", "RETURN", "DONE"}:
            # one cycle enough
            if code == "LAB_PLACE_VERIFY" or ctrl.finished:
                break

    print("\n=== WALK SUMMARY ===", flush=True)
    for code, status in walked:
        print(f"  {status:12} {code}", flush=True)
    print(f"finished={ctrl.finished} steps_done={len(walked)}", flush=True)
    codes = [c for c, _ in walked]
    ok = "DEVICE_CHECK" in codes and len(walked) >= 3
    print("SMOKE_PASS" if ok else "SMOKE_FAIL", flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
