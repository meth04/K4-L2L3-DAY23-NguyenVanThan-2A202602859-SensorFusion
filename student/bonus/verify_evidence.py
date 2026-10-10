"""Read-only integrity checks for graded and bonus evidence (no data needed)."""

import csv
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
BONUS = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "platform"))
from fusion_lab.evaluation import read_records, validate_metrics_records


def load(path):
    return json.loads(path.read_text(encoding="utf-8"))


def main():
    artifacts = ROOT / "student/artifacts"
    for suffix in ("", "_lidar", "_fused"):
        metrics = load(artifacts / f"metrics{suffix}.json")
        validate_metrics_records(metrics, read_records(artifacts / f"grade_run{suffix}.log"))
        assert metrics["seed"] == 0 and metrics["frames"] == [0, 198]
    main_metrics = load(artifacts / "metrics.json")
    for mode in ("lidar", "fused"):
        stats = main_metrics["tracking"][mode]
        precision = stats["matches"] / (stats["matches"] + stats["ghost_track_frames"])
        coverage = stats["matches"] / main_metrics["detection"]["tp"]
        assert stats["rmse"] <= 0.45 and precision >= 0.75 and coverage >= 0.70
        print(f"Graded {mode}: RMSE={stats['rmse']:.8f}, precision={precision:.6f}, coverage={coverage:.6f}")
    assert main_metrics["tracking"]["fused"]["rmse"] - main_metrics["tracking"]["lidar"]["rmse"] <= 0.05

    summary = load(BONUS / "calibration_summary.json")
    assert [r["yaw_deg"] for r in summary] == [0, 0.5, 2, 5, 10]
    probes = list(csv.DictReader((BONUS / "calibration_probes.csv").open(encoding="utf-8", newline="")))
    for row in summary:
        name = f"yaw_{row['yaw_deg']:g}"
        metrics = load(BONUS / f"metrics_{name}.json")
        records = read_records(BONUS / f"records_{name}.jsonl")
        validate_metrics_records(metrics, records)
        assert metrics["tracking"]["fused"]["rmse"] == row["rmse_m"]
        updates = list(csv.DictReader((BONUS / f"camera_updates_{name}.csv").open(encoding="utf-8", newline="")))
        assert len(updates) == row["camera_updates"]
        reference = [r for r in probes if float(r["yaw_deg"]) == row["yaw_deg"]]
        assert len(reference) == row["reference_pairs"]
        visible = [r for r in reference if r["visible"] == "True"]
        assert len(visible) == row["probe_visible_pairs"]
        assert np.isclose(np.mean([float(r["du_px"]) for r in visible]), row["probe_mean_du_px"])
        assert np.isclose(np.mean([float(r["dv_px"]) for r in visible]), row["probe_mean_dv_px"])
        assert np.isclose(np.median([float(r["d2"]) for r in visible]), row["probe_median_d2"])
        assert sum(r["gate_pass"] == "True" for r in reference) / len(reference) == row["probe_gate_pass_fraction"]
    lidar_metrics = load(BONUS / "metrics_lidar.json")
    validate_metrics_records(lidar_metrics, read_records(BONUS / "records_lidar.jsonl"))
    # The shared-detector bonus baseline must reproduce the official runner;
    # tolerate small floating point differences from CPU thread count.
    for mode, bonus_metrics in (("lidar", lidar_metrics), ("fused", load(BONUS / "metrics_yaw_0.json"))):
        assert bonus_metrics["detection"] == main_metrics["detection"]
        for key, value in main_metrics["tracking"][mode].items():
            assert np.isclose(bonus_metrics["tracking"][mode][key], value, rtol=1e-5, atol=1e-7)

    evidence = load(BONUS / "visualization_evidence.json")
    assert 2 <= len(evidence) <= 5
    for item in evidence:
        assert (BONUS / item["file"]).is_file()
        pre = np.linalg.norm(np.array(item["before_position"]) - item["gt_position"])
        post = np.linalg.norm(np.array(item["after_position"]) - item["gt_position"])
        assert np.isclose(pre, item["error_before_m"]) and np.isclose(post, item["error_after_m"])
        assert post < pre
    exported = load(BONUS / "tracks_export.json")["frames"]
    assert [r["frame"] for r in exported] == list(range(199))
    print("PASS: all three graded reports/logs, six bonus reports/logs, calibration CSV aggregates,")
    print("baseline reproduction, annotated visualization coordinates, and 199-frame track export.")
    print("CVAT import has not been performed; no CVAT bonus is claimed.")


if __name__ == "__main__":
    main()
