# 固定49152点全覆盖与局部角点高度 Implementation Plan

> For agentic workers: execute inline with executing-plans and test-driven-development; preserve this checkout's existing changes.

**Goal:** 每次49152点、全点直接预测；固定角点XY，用附近三维板面稳健估计Z，过滤偏高异常；导出运行时现存原始PCD的四类结果（实际扫描目录为11份）。

**Architecture:** 新增采样与局部高度独立模块；主pipeline调用，服务透传高度质量。归一化在整云上一次计算；每批保留全车采样上下文和待覆盖点，按原点编号累计自身概率。高度只用同标签原始三维点，不限于全局平面内点。失败明确标记回退，禁止无声声称可靠。

**Tech Stack:** Existing NumPy/PyTorch/Open3D/matplotlib/unittest. No new dependencies.

**Spec:** `docs/point_cloud_model_alignment_pending_2026-10-16.md` and user's explicit instruction to begin, including high-Z outlier filtering.

## Global constraints / rulings

- In-place on existing development checkout: extensive prior uncommitted work is the required runtime state; do not reset, copy to an empty branch, or commit unrelated changes. No PR/push requested.
- Voxel10mm, statistical6/3.0, weights and world calibration unchanged. No field-log fitting.
- Per-batch49152 (configurable for tests), context fraction0.5: half coverage slots shuffled across full truck, remaining training-style voxel-uniform samples from full truck; final batch fills to49152. Context keeps later coverage groups representative. This is an explicit coverage extension, not a claim of identical training sampling statistics.
- Shared full-cloud normalization, fixed seeds and source-order restoration; probability averaging only for points actually forwarded. Coverage counts must all be positive.
- Local height initial radii250/400/600/900mm; minimum40 local inliers; robust vertical residual tolerance15..40mm, maximum slope0.5. Review added minimum50% local consensus after reproducing arbitrary planes in dense high noise. These are recorded starting parameters, not measured accuracy guarantees. Reject high positive residuals relative to fitted local surface; also exclude large negative residuals. Larger valid board heights are not globally clipped.
- Insufficient or degenerate local support retains old plane height with explicit `global_plane_fallback` status and `corner_height_all_supported=false`. Preserve this status in service output and test manifests.
- Save model-entry and segmentation PCD in radar mm; corner comparison in world mm; all filenames/README mark frames. JPG depicts actual UV fits; local Z correction preserves radar/world XY, not necessarily projected UV after vertical movement.

## Tasks / execution ledger

- [x] 1. RED→GREEN sampling and actual inference coverage tests; cover small/nonmultiple inputs, normalization, duplicate prediction reduction and repeatability.
  Files: `full_coverage_sampling.py`, pipeline, `tests/test_point_cloud_full_coverage.py`.
- [x] 2. RED→GREEN local-height tests: warped long plate, inclined plane, positive outliers, sparse/collinear support, input permutation; integrate pipeline and service quality fields.
  Files: `corner_height.py`, pipeline, service, `tests/test_point_cloud_corner_height.py`.
- [x] 3. Run relevant existing repeatability/calibration/service tests; fix only introduced regressions. Final9 targeted tests +29 existing tests passed (38 total); complete first real multi-group input repeated identically.
- [x] 4. Export one fresh run of all current11 raw PCDs: pre-model radar PCD, segmentation radar PCD, UV JPG, world red/green20mm PCD; retain NPZ/JSON and coverage/height comparison.
- [x] 5. Independently read exports; check input hashes, exact point identity, RGB labels, complete direct coverage, world corners, unchanged XY and local height support; inspect representative single/dual JPGs and height diagnostics; final focused review. 3,997,184 points /166 forwards verified;84 local heights,0 fallbacks,1 extrapolated-support flag. Reviewer noisy-volume bug fixed RED→GREEN and independently rechecked; all saved local estimates re-evaluated with final gate and original XYZ unchanged.

No implementation completed at plan creation. Final report distinguishes algorithm changes from unverified absolute accuracy and reports any fallback or segmentation failures.

Execution update: production sampler/height integration implemented; full batch started at `PCD/model_49152_local_height_20261016_113204`. Focused independent code review in progress. Original source PCDs and weights unchanged.

Final: batch and focused review complete. Output test report in that folder. Known limitation preserved: confirmed single-deck12:32:49 now yields2 geometric boards from erroneous label3; raw label3 increased9910→22887 compared to old full-cloud baseline. Do not claim segmentation accuracy improved. Generalization and board-count checks remain unresolved. Local Z differs from its global plane by-42.3..+124.0mm; this is not verified absolute accuracy.
