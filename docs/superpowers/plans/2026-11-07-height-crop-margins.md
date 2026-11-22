# Height crop margins and placement comparison

> **For agentic workers:** Execute inline using superpowers:executing-plans, with tests before production edits.

**Goal:** Replace the height region with inset margins, refresh saved results, and include the new eligible compensation data in separate XY/Z iteration.

**Architecture:** Keep the original deck quadrilateral as the coordinate frame. Apply configurable UV start offsets to all A/B/C/D height methods; reuse a height-only pipeline function to refresh existing segmentation outputs without changing XY fitting or inference.

**Tech Stack:** Python, NumPy, unittest; offline PLC placement calculator; artifact-tool Excel.

**Spec:** User instructions in this conversation, 2026-11-07: length windows single 5–65%, high 5–85%, low 5–45%; width 5–95%; default A, five equal strips; include new compensation records.

## Global constraints

- E1 is the right row; E2 is the left row. Compare each row's first placement point only.
- No changes to PointNet, XY geometry, world calibration, or production compensation.
- Preserve original point coordinates and segmentation PCD bytes.
- Exclude two empty-head scans and the failed first scan at 2026-10-22 11:23 from comparison fitting; keep failure outputs for inspection.
- Keep records without complete algorithm geometry, but leave unavailable placement/differences blank.
- Archive replaced results. No hardware connections or control programs.

## Review focus

- Front/near/far edge outliers must stay outside the new height region.
- All four height methods must use the same offset region, including strip centers and support bounds.
- A low deck must use its own usable-start and its own length scale.
- Height-only refresh must preserve all published corner XY and all input PCDs.
- Offset/width overrides must reach the pipeline through the software service.

## Task 1: inset height region

Files: single_board_height.py, deck_height_methods.py, point_cloud_pipeline.py, system_config.py, point_cloud_processing_service.py, and height tests.

- [x] Add failing tests for new default windows, edge exclusions, all methods, overrides, and invalid windows.
- [x] Run tests and confirm the missing-margin failures (four region failures and missing interfaces).
- [x] Add length_start_fraction/width_start_fraction and common config fields; set default width fraction .90, start offsets .05.
- [x] Extract apply_deck_reference_heights_timed(xyz, labels, result, config) for height-only reuse; preserve XY and support rules.
- [x] Run height and related geometry tests: 96 tests passed.

## Task 2: refresh results

- [x] Archive replaced JSON/TXT/JPG/manifests and comparison inputs.
- [x] Refresh 87 existing and 14 new saved segmentations; update height ROI images, H and corner Z only.
- [x] Verify all 101 PCD hashes and all corner XY unchanged; 152 supported board heights verified.

## Task 3: placement compensation

- [x] Compute the new eligible reference rows with known manual compensation and the same offline calculator.
- [x] Combine eligible new data with the existing set: 98 files / 196 points; 3 user-excluded review records retained.
- [x] Select XY and Z models separately using nested grouped cross-validation, with paired rows and related scans grouped together.
- [x] Export Excel with corrected placement XYZ, reference, signed differences, dimensions, parameters, and validation summary.
- [x] Verify 4,752 formulas, all cached coordinates and dimensions, row exclusions, source arithmetic, formats, and rendered previews. Deterministic iteration rerun identical.

## Execution ledger

Ruling: Work in the existing user workspace, because the user explicitly requested direct algorithm replacement; archive changed files first. Do not create a new runtime checkout or commit unrelated work.

Ruling: Include new eligible records now, because the user's latest steering explicitly requests inclusion; incomplete geometry cannot be assigned invented placement points.

Ruling: The synthetic 5200 mm support-gap fixture loses spatial support in its final height strip under the new 85% end. Preserve the quality rejection and test that it withholds H; use 5000 mm for the separate supported-crop test. No production tolerance was weakened.

Ruling: User excluded three new files (2026-10-20 13:29:01, 2026-10-22 08:22:10, 2026-10-22 11:23:18) after viewing images. Keep all outputs, exclude from all fitting and metrics. Active comparison is 98 files: 87 original plus 11 new.

Completion: production changes are limited to the height ROI and height-only reuse interface. PointNet, XY fitting and calibration are unchanged. The experimental placement compensation remains offline. Delivery and independent evidence are recorded in outputs/height_crop_margin_20261107/delivery_verification.json.
