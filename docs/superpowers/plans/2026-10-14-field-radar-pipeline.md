# Field Radar Pipeline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** Keep only field offline operation and connect stable ITL intake, conversion, and user-selected PCD algorithm input.

**Architecture:** Preserve the existing field cycle, migrate shared code out of laboratory naming, and keep file transport/conversion separate from algorithm execution. A configurable background conversion worker consumes stable local ITL files and publishes validated PCD output atomically.

**Tech Stack:** Python, PySide6, Windows PowerShell, existing vendor conversion executable.

**Spec:** ../specs/2026-10-14-field-radar-pipeline-design.md

## Global Constraints

- Preserve uncommitted PLC configuration and all PLC read-only behavior.
- Do not modify remote data, PLC code, or networking.
- Do not fabricate ITL geometry or report unverified vendor conversion success.
- Default PCD browser directory: project/pcd; accept other local PCD files.

## Tasks

### 1. Field-only workflow and PCD selection
- [x] Trace shared and exclusive mode dependencies, add behavioral tests.
- [x] Remove sim/lab mode settings, UI, and exclusive branches; migrate field dependencies.
- [x] Add PCD directory and visible chooser; persist and propagate selected path through second-step radar processing.
- [x] Run field flow and UI tests, report remaining shared names if any.

### 2. Vendor converter feasibility and automation
- [x] Inspect archive, executable interfaces, installed dependencies and paired sample.
- [x] Validate executable-backed automated conversion when possible; avoid unproven reimplementation.
- [x] Provide isolated converter interface with actionable error and no false success.

### 3. ITL sync and pipeline lifecycle
- [x] Test stable-file synchronization and conversion queuing with local fixtures.
- [x] Harden PS1 transfer and bridge logs to console and disk.
- [x] Wire startup/shutdown, conversion state and atomic PCD publication.

### 4. Integration and review
- [x] Run full unittest, offline smoke, script syntax and diff checks.
- [x] Review all changes for deleted-mode references, stale files, shutdown leaks and selected-input overrides.
- [x] Document exact deployment and converter limitations.

## Verification outcome

- 93/93 automated tests passed under the project Python 3.11 runtime.
- Real PowerShell local-share fixture: stable copy, extension filtering, one locked file does not starve another, console/disk log bridging, configured test conversion, and shutdown passed.
- Main application startup reached READY and exited 0 after its smoke timer (offscreen; single-instance guard isolated for the test, existing user window preserved).
- Offline flow fixture traversed DEVICE_CHECK, FIELD_SENSE, FIELD_CORNER_SHELL, FIELD_WAIT_MANUAL_PLACE with no failures; heavy perception modules explicitly stubbed.
- Code review's two controller regressions and two PS1 robustness issues were fixed and regression-tested.
- Real vendor conversion is NOT complete: three bundled ITL sample trials exited 0xC0000409 without PCD after portable official dependencies restored GUI launch. Default converter_command remains empty, experimental helper disabled. Full native runtime is local and Git-ignored; deployment/provenance instructions are documented.
- Actual industrial PC share connectivity and real field perception inputs still require the field network/data. No PLC code/network settings were changed; preexisting PLC configuration edits were preserved.
