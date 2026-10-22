"""Offline PCD repeatability check. Never reads field HMI reference coordinates."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

os.environ.setdefault("OMP_NUM_THREADS", "4")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
import torch
import open3d as o3d
from algorithm_modules.point_cloud_segment_module import point_cloud_pipeline as p


def sha_bytes(data):
    return hashlib.sha256(data).hexdigest()


def save(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def quality_metrics(xyz, labels, config):
    """Record support and residuals, without tuning or accepting a physical target."""
    result = {}
    for label in config.target_labels:
        points = xyz[labels == label]
        if len(points) < config.edge_plane_ransac_n:
            continue
        model, indices = p.fit_plane_once(points, config)
        plane = np.asarray(model)
        distances = np.abs(points.astype(np.float64) @ plane[:3] + plane[3])
        item = {"label_points": len(points), "plane_support": len(indices),
                "plane_inlier_fraction": len(indices)/len(points),
                "plane_inlier_rms_mm": float(np.sqrt(np.mean(distances[indices]**2)))}
        if len(indices) >= config.edge_min_inliers:
            info = p.compute_pca_info(points[indices], model, config.truck_head_dir)
            u, v, _ = p.project_points_to_uvn(points[indices], info)
            edges = p.extract_edge_candidates_2d(np.column_stack((u, v)), config)
            rng = np.random.default_rng(config.seed+label)
            item["edges"] = {}
            for name in ("v_min", "v_max", "u_min", "u_max"):
                candidates = edges[name]
                line = p.ransac_fit_line_2d(candidates, config, rng)
                entry = {"candidates": len(candidates), "fitted": line is not None}
                if line is not None:
                    residual = p.line_distances(candidates, line)
                    inside = residual <= config.line_ransac_distance
                    entry.update({"inliers": int(inside.sum()),
                        "inlier_rms_mm": float(np.sqrt(np.mean(residual[inside]**2)))})
                item["edges"][name] = entry
        result[str(label)] = item
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=ROOT / "PCD")
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/point_cloud_repeatability_20261015")
    parser.add_argument("--rounds", type=int, default=3)
    parser.add_argument("--stress-runs", type=int, default=20)
    parser.add_argument("--worker", type=Path)
    args = parser.parse_args()
    if args.rounds < 1 or args.stress_runs < 1:
        parser.error("Run counts must be positive")
    sys.stdout.reconfigure(encoding="utf-8")
    torch.set_num_threads(4)
    files = [args.worker] if args.worker else p.collect_pcd_files(args.input)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    pipe = p.PointCloudPipeline(device="cpu")
    captured = {}
    forward = pipe.segment_xyz_full_coverage

    def tap(xyz):
        labels, counts, timing = forward(xyz)
        captured.update(xyz=xyz, labels=labels,
            xyz_sha256=sha_bytes(np.ascontiguousarray(xyz).tobytes()),
            labels_sha256=sha_bytes(labels.tobytes()))
        return labels, counts, timing

    pipe.segment_xyz_full_coverage = tap
    report = {"scope": "Same PCD, model, parameters and CPU environment; not an absolute accuracy test.",
        "environment": {"python": sys.version, "numpy": np.__version__, "torch": torch.__version__,
            "open3d": o3d.__version__, "device": "cpu", "torch_threads": torch.get_num_threads(),
            "OMP_NUM_THREADS": os.environ["OMP_NUM_THREADS"]}, "config": asdict(pipe.config),
        "weights_sha256": sha_bytes((ROOT / "algorithm_modules/point_cloud_segment_module/checkpoints/best_model.pth").read_bytes()),
        "pipeline_sha256": sha_bytes(Path(p.__file__).read_bytes()), "files": {}, "errors": []}
    first = {}

    def run(path):
        key = path.name
        entry = report["files"].setdefault(key, {"source_sha256": sha_bytes(path.read_bytes()), "runs": []})
        number = len(entry["runs"])
        record = pipe.process_file(path, output / "runs" / path.stem / str(number))
        result = record["result"]
        run_record = {"run": number, "result": result, "stage_counts": record["stage_counts"],
            "label_counts": record["dense_label_counts"], "total_s": record["total_s"],
            "xyz_sha256": captured["xyz_sha256"], "labels_sha256": captured["labels_sha256"]}
        if key not in first:
            first[key] = run_record
            entry["quality"] = quality_metrics(captured["xyz"], captured["labels"], pipe.config)
        expected = first[key]
        run_record["checks"] = {field: run_record[field] == expected[field]
            for field in ("result", "stage_counts", "label_counts", "xyz_sha256", "labels_sha256")}
        run_record["checks"]["has_board"] = any(isinstance(v, dict) for v in result.values())
        entry["runs"].append(run_record)
        save(output / "repeatability.json", report)
        print(f"CHECK {key} run={number+1} equal={all(run_record['checks'].values())}", flush=True)

    for round_id in range(1 if args.worker else args.rounds):
        # Interleave files and reverse order to catch processing-history leakage.
        for path in (files if round_id % 2 == 0 else list(reversed(files))):
            try:
                run(path)
            except Exception as exc:
                report["errors"].append({"file": str(path), "error": repr(exc)})
                save(output / "repeatability.json", report)
                print("ERROR", path.name, repr(exc), flush=True)
    if not args.worker and not report["errors"]:
        stress = next((x for x in files if "11-32-55" in x.name), files[0])
        while len(report["files"][stress.name]["runs"]) < args.stress_runs:
            try:
                run(stress)
            except Exception as exc:
                report["errors"].append({"file": str(stress), "error": repr(exc)})
                break
        env = os.environ.copy()
        env["PYTHONIOENCODING"] = "utf-8"
        worker_dir = output / "fresh_process"
        proc = subprocess.run([sys.executable, str(Path(__file__).resolve()), "--worker", str(stress),
            "--output", str(worker_dir)], capture_output=True, text=True, encoding="utf-8", env=env,
            creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
        (output / "fresh_process.log").write_text(proc.stdout+proc.stderr, encoding="utf-8")
        if proc.returncode != 0:
            report["errors"].append({"fresh_process": proc.returncode})
        else:
            fresh = json.loads((worker_dir / "repeatability.json").read_text(encoding="utf-8"))
            record = fresh["files"][stress.name]["runs"][0]
            report["fresh_process_checks"] = {field: record[field] == first[stress.name][field]
                for field in ("result", "stage_counts", "labels_sha256", "xyz_sha256")}
    for key, entry in report["files"].items():
        entry["all_runs_equal"] = all(all(run["checks"].values()) for run in entry["runs"])
        entry["max_corner_displacement_mm"] = {}
        for label in ("label_2", "label_3"):
            boards = [r["result"][label] for r in entry["runs"] if r["result"].get(label)]
            if boards:
                corners = np.asarray([b["radar_corners_xyz_mm"] for b in boards])
                entry["max_corner_displacement_mm"][label] = np.max(
                    np.linalg.norm(corners[:, None]-corners[None, :], axis=-1), axis=(0, 1)).tolist()
    report["completed"] = True
    report["passed"] = not report["errors"] and all(x["all_runs_equal"] for x in report["files"].values()) and all(report.get("fresh_process_checks", {}).values())
    save(output / "repeatability.json", report)
    print("PASSED", report["passed"], flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
