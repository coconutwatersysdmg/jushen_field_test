# Board Data Visualization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a center “板面数据” page that displays two boards of radar and camera metrics, with clickable explanations for all five formulas, without changing field algorithms or downstream planning.

**Architecture:** A pure service converts existing round data into two read-only board view models. A reusable PySide6 widget renders those models and owns the five formula dialogs. Both retained main-window implementations add the same widget as the fourth center tab and refresh it from their existing snapshot path.

**Tech Stack:** Python 3, PySide6 6.8.3, standard-library `unittest`

**Spec:** `docs/superpowers/specs/2026-10-12-board-data-visualization-design.md`

## Global Constraints

- Keep the existing main-screen corner WORLD tables and all field-offline behavior unchanged.
- Never populate the camera row from radar or generic final/fallback corners; use only `camera_world_corners`.
- Display exactly two boards, two source rows, and five metrics; omit legends, usage/status columns, corner links, and “允许划格”.
- Use the same five formulas for radar and camera, with horizontal offset measured from WORLD `-Y`.
- Missing or invalid source data displays `—` and must not affect the other board or source.
- Existing `yaw_deg`, planning, gridding, PLC, radar, and camera results remain untouched.

## Review Focus

- Eight camera corners map to independent P1–P4 and P5–P8 boards.
- Six camera corners overlap at P3/P4 and map to P1–P4 plus P3–P6.
- A camera payload containing only pixel points or fallback final corners leaves camera metrics blank.
- Malformed individual points do not crash UI refresh and blank only the affected source row.
- Repeated snapshot refreshes update values without opening dialogs or rerunning algorithms.

---

### Task 1: Read-only board metric view model

**Files:**
- Create: `services/board_data_service.py`
- Create: `tests/test_board_data_service.py`

**Interfaces:**
- Consumes: `round_data: Mapping[str, Any]`, including `radar_result.boards` and pure camera corners under `lab_corner_shell.camera_world_corners` or top-level `camera_world_corners`.
- Produces: `compute_corner_metrics(points: Sequence[Any]) -> dict[str, float]`, `camera_corner_groups(points: Mapping[str, Any]) -> list[list[Any]]`, and `build_board_data(round_data: Mapping[str, Any]) -> list[dict[str, Any]]` with exactly two board models.

- [ ] **Step 1: Write failing service tests**

Cover hand-derived geometry values, `-Y` offset sign, 8/6/4-point grouping, two radar boards, camera-source isolation, missing values, and malformed points.

- [ ] **Step 2: Run service tests and verify RED**

Run: `.venv\Scripts\python.exe -m unittest tests.test_board_data_service -v`

Expected: FAIL because `services.board_data_service` does not exist.

- [ ] **Step 3: Implement the pure service**

Normalize list/tuple and mapping points, require finite XYZ values, compute the five formulas, preserve valid radar metrics, and always return two board models with independent radar/camera dictionaries or `None`.

- [ ] **Step 4: Run service tests and verify GREEN**

Run: `.venv\Scripts\python.exe -m unittest tests.test_board_data_service -v`

Expected: all service tests pass.

- [ ] **Step 5: Commit**

Commit message: `feat: add board data view model`

### Task 2: Reusable board data panel and formula dialogs

**Files:**
- Create: `ui/board_data_panel.py`
- Create: `tests/test_board_data_panel.py`

**Interfaces:**
- Consumes: `build_board_data(round_data)` from Task 1.
- Produces: `BoardDataPanel(QWidget)`, `update_from_round_data(round_data: Mapping[str, Any]) -> None`, and `open_formula(metric_key: str) -> QDialog`.

- [ ] **Step 1: Write failing widget tests**

With `QT_QPA_PLATFORM=offscreen`, assert the exact two board titles and two source rows, formatted units/precision, missing-value em dash, absence of removed content, and that all five header buttons open formula dialogs containing the formula and sign/unit explanation.

- [ ] **Step 2: Run widget tests and verify RED**

Run: `$env:QT_QPA_PLATFORM='offscreen'; .venv\Scripts\python.exe -m unittest tests.test_board_data_panel -v`

Expected: FAIL because `ui.board_data_panel` does not exist.

- [ ] **Step 3: Implement `BoardDataPanel`**

Build a title and two table cards. Use clickable header buttons for length, width, average height, longitudinal tilt, and horizontal offset. Render radar as “雷达车板数据” and camera as “相机装载区域数据”; keep formula dialogs modeless and parented to the panel.

- [ ] **Step 4: Run widget tests and verify GREEN**

Run: `$env:QT_QPA_PLATFORM='offscreen'; .venv\Scripts\python.exe -m unittest tests.test_board_data_panel -v`

Expected: all widget tests pass.

- [ ] **Step 5: Commit**

Commit message: `feat: add board data panel`

### Task 3: Main-window integration and regression verification

**Files:**
- Modify: `ui/main_window.py`
- Modify: `ui/main_window_full.py`
- Create: `tests/test_board_data_integration.py`

**Interfaces:**
- Consumes: `BoardDataPanel` from Task 2 and existing `round_data` in each window `_refresh` method.
- Produces: a fourth center tab named “板面数据”, mapped to the third overlay page and refreshed from existing snapshots only.

- [ ] **Step 1: Write failing integration tests**

Instantiate both main-window classes offscreen with their heavy initialization bypassed only at the QML/device boundary, then assert the fourth tab, overlay index mapping, and snapshot-to-panel update. Also assert original three tab labels remain in order.

- [ ] **Step 2: Run integration tests and verify RED**

Run: `$env:QT_QPA_PLATFORM='offscreen'; .venv\Scripts\python.exe -m unittest tests.test_board_data_integration -v`

Expected: FAIL because the fourth tab and panel are not integrated.

- [ ] **Step 3: Integrate the shared panel in both windows**

Import and add `BoardDataPanel`, append “板面数据” after the three existing tabs, add it after radar/history overlay pages, and call `update_from_round_data(rd)` during `_refresh` without changing any controller or service result.

- [ ] **Step 4: Run integration tests and verify GREEN**

Run: `$env:QT_QPA_PLATFORM='offscreen'; .venv\Scripts\python.exe -m unittest tests.test_board_data_integration -v`

Expected: all integration tests pass.

- [ ] **Step 5: Run complete verification**

Run: `$env:QT_QPA_PLATFORM='offscreen'; .venv\Scripts\python.exe -m unittest discover -s tests -v`

Run: `$env:QT_QPA_PLATFORM='offscreen'; $env:JUSHEN_STARTUP_SMOKE_MS='2500'; .venv\Scripts\python.exe main.py`

Expected: the full suite passes; startup exits with code 0 after the smoke timer and no traceback is written.

- [ ] **Step 6: Commit**

Commit message: `feat: show board metrics in main window`
