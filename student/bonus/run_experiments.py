"""Reproduce bonus evidence without editing platform or graded artifacts.

Run from the repository root with the lab venv:
  python -X utf8 -B student/bonus/run_experiments.py
The detector is run once per frame and shared by independent trackers. Only
camera extrinsics change between calibration experiments; observations and seed
are identical. Ground truth is used for evaluation/figures, never EKF updates
except the lab's supplied simulated camera observations.
"""

from __future__ import annotations

import csv
import hashlib
import json
import os
from pathlib import Path
import sys

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "platform"))
os.environ.setdefault("DAY23_STUDENT_ROOT", str(ROOT / "student"))
os.environ.setdefault("FUSION_LAB_PLATFORM", str(ROOT / "platform"))

from fusion_lab import tracking_params as params
from fusion_lab.evaluation import (
    aggregate_records, detection_counts, record_json, tracking_counts,
    valid_ground_truth, validate_metrics_records,
)
from fusion_lab.export_cvat import export_tracks_json
from fusion_lab.scripts import run_lab
from fusion_lab.tracking.filter import Filter
from fusion_lab.tracking.manager import TrackManager
from fusion_lab.tracking.sensors import Sensor
from fusion_lab.workspace_loader import load_workspace


ANGLES = (0.0, 0.5, 2.0, 5.0, 10.0)


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")


def perturbed_sensor(calib, camera_module, angle):
    sensor = Sensor("camera", calib, camera_module)
    a = np.deg2rad(angle)
    # Rotate the assumed sensor axes about their up axis. The physical labels
    # and their pixels stay unchanged. T_bad = T_true @ Rz(angle).
    rotation = np.eye(4)
    rotation[:2, :2] = [[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]
    sensor.sens_to_veh = np.asmatrix(np.asarray(sensor.sens_to_veh) @ rotation)
    sensor.veh_to_sens = np.asmatrix(np.linalg.inv(sensor.sens_to_veh))
    return sensor


def track_copy(track):
    return {"id": track.id, "position": np.asarray(track.x).reshape(-1)[:3].tolist(),
            "state": track.state, "score": track.score,
            "dimensions": [track.height, track.width, track.length], "yaw": track.yaw}


class InstrumentedFilter(Filter):
    def __init__(self, kalman, sensors, nominal=False):
        super().__init__(kalman)
        self.sensors = sensors
        self.nominal = nominal
        self.frame = -1
        self.updates = []
        self.probes = []

    def update(self, track, meas):
        if meas.sensor.name == "camera":
            gamma = np.asarray(self._k.innovation(track.x, meas)).reshape(-1)
            H = meas.sensor.get_H(track.x)
            S = self._k.innovation_covariance(track.P, meas, H)
            d2 = float(gamma @ np.linalg.solve(np.asarray(S), gamma))
            self.updates.append({"frame": self.frame, "track_id": track.id,
                                 "du_px": float(gamma[0]), "dv_px": float(gamma[1]), "d2": d2})
            if self.nominal:
                # Freeze nominal x/P/z and pairing. This separates calibration
                # bias from changed trajectories and survivor selection by gating.
                for angle, sensor in self.sensors.items():
                    if not sensor.in_fov(track.x):
                        self.probes.append({"frame": self.frame, "track_id": track.id,
                                            "yaw_deg": angle, "visible": False,
                                            "du_px": None, "dv_px": None, "d2": None, "gate_pass": False})
                        continue
                    residual = np.asarray(meas.z - sensor.get_hx(track.x)).reshape(-1)
                    H_probe = sensor.get_H(track.x)
                    S_probe = self._k.innovation_covariance(track.P, meas, H_probe)
                    d2_probe = float(residual @ np.linalg.solve(np.asarray(S_probe), residual))
                    self.probes.append({"frame": self.frame, "track_id": track.id, "yaw_deg": angle,
                                        "visible": True, "du_px": float(residual[0]),
                                        "dv_px": float(residual[1]), "d2": d2_probe,
                                        "gate_pass": bool(ws["association"].chi2_gate(d2_probe, sensor))})
        super().update(track, meas)


def dump_csv(path, rows, fieldnames=None):
    if not rows and fieldnames is None:
        raise ValueError(f"No observations for {path}")
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames or list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def figures(snapshots, summary):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import cv2

    evidence = []
    for block, snap in sorted(snapshots.items()):
        frame, tid = snap["frame"], snap["track_id"]
        before, after, gt = map(np.asarray, (snap["before"], snap["after"], snap["gt"]))
        fig, axes = plt.subplots(1, 3, figsize=(17, 5), layout="constrained")
        points = snap["points"]
        axes[0].scatter(points[:, 1], points[:, 0], s=0.15, c="#99a7b2", alpha=0.4)
        for label in snap["labels"]:
            axes[0].scatter(label[1], label[0], c="#247d43", marker="x", s=30)
        axes[0].scatter(before[1], before[0], c="#db8b16", s=65, marker="s", label="Before camera")
        axes[0].scatter(after[1], after[0], c="#356fd4", s=65, marker="o", label="After camera")
        axes[0].set(xlim=(-25, 25), ylim=(0, 50), xlabel="Vehicle y (m, left)", ylabel="Vehicle x (m, forward)", title=f"BEV / frame {frame} / track {tid}")
        axes[0].invert_xaxis()
        axes[0].legend(fontsize=8)
        axes[1].scatter(gt[1], gt[0], c="#247d43", marker="x", s=120, label="3D GT centre")
        axes[1].scatter(before[1], before[0], c="#db8b16", s=80, marker="s", label="Before camera")
        axes[1].scatter(after[1], after[0], c="#356fd4", s=80, marker="o", label="After camera")
        axes[1].annotate("", xy=(after[1], after[0]), xytext=(before[1], before[0]), arrowprops={"arrowstyle": "->", "color": "#356fd4"})
        axes[1].set(xlim=(gt[1]-0.35, gt[1]+0.35), ylim=(gt[0]-0.35, gt[0]+0.35), xlabel="Vehicle y (m)", ylabel="Vehicle x (m)", title="Same track, isolated camera correction")
        axes[1].invert_xaxis()
        axes[1].grid(alpha=0.2)
        axes[1].legend(fontsize=8)
        image = cv2.imdecode(np.frombuffer(snap["image"], dtype=np.uint8), cv2.IMREAD_COLOR)
        axes[2].imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        sensor = snap["sensor"]
        for pos, colour, marker, label in [(before, "#db8b16", "s", "Predicted pixel before"), (after, "#356fd4", "o", "Predicted pixel after")]:
            uv = np.asarray(sensor.get_hx(np.asmatrix(np.r_[pos, [0, 0, 0]]).T)).reshape(-1)
            axes[2].scatter(*uv, c=colour, marker=marker, s=80, label=label)
        for uv in snap["camera_pixels"]:
            axes[2].scatter(*uv, c="#f542cb", marker="+", s=60)
        axes[2].set(title="FRONT / magenta: noisy GT-box observations", xlabel="u (px)", ylabel="v (px)")
        axes[2].legend(fontsize=7)
        error_before = float(np.linalg.norm(before-gt))
        error_after = float(np.linalg.norm(after-gt))
        fig.suptitle(f"Frame {frame}, track {tid}: 3D error {error_before:.5f} -> {error_after:.5f} m | score/state unchanged", fontsize=12)
        path = OUT / f"camera_update_{frame:03d}.png"
        fig.savefig(path, dpi=160)
        plt.close(fig)
        evidence.append({"file": path.name, "frame": frame, "track_id": tid,
                         "gt_id": snap["gt_id"], "before_position": before.tolist(),
                         "after_position": after.tolist(), "gt_position": gt.tolist(),
                         "error_before_m": error_before, "error_after_m": error_after})
    fig, axes = plt.subplots(1, 3, figsize=(15, 4), layout="constrained")
    angles = [r["yaw_deg"] for r in summary]
    axes[0].plot(angles, [r["rmse_m"] for r in summary], "o-")
    axes[0].set(xlabel="Assumed extrinsic yaw offset (deg)", ylabel="3D RMSE (m)", title="Full independent tracking runs")
    axes[1].plot(angles, [r["probe_mean_du_px"] for r in summary], "o-")
    axes[1].set(xlabel="Yaw offset (deg)", ylabel="Mean horizontal residual (px)", title="Fixed nominal x/P/z reference pairs")
    axes[2].plot(angles, [100*r["probe_gate_pass_fraction"] for r in summary], "o-")
    axes[2].set(xlabel="Yaw offset (deg)", ylabel="Gate pass (%)", ylim=(-5, 105), title="Chi-square gate on fixed reference pairs")
    for ax in axes:
        ax.grid(alpha=0.2)
    fig.savefig(OUT / "calibration.png", dpi=160)
    plt.close(fig)
    write_json(OUT / "visualization_evidence.json", evidence)


def main():
    global ws
    torch.set_num_threads(4)
    run_lab._setup_import_paths()
    ws = load_workspace()
    from simple_waymo_open_dataset_reader import WaymoDataFileReader, dataset_pb2, label_pb2
    from simple_waymo_open_dataset_reader import utils as waymo_utils
    from fusion_lab.lidar_pcl import pcl_from_range_image

    cfg = run_lab._load_paths_config(ROOT / "student/config/paths.yaml")
    weights = run_lab._resolve_weights(cfg)
    det_cfg = ws["detection_pipeline"].load_fpn_resnet_config(weights)
    model = ws["detection_pipeline"].create_fpn_model(det_cfg, weights)
    source = Path(cfg["waymo_dir"]) / cfg["segment"]
    scenarios = {}
    snapshots = {}
    nominal_export = []
    start, end = int(cfg["frame_start"]), int(cfg["frame_end"])
    for frame_index, frame in enumerate(WaymoDataFileReader(str(source))):
        if frame_index < start:
            continue
        if frame_index > end:
            break
        if not scenarios:
            lidar = Sensor("lidar", waymo_utils.get(frame.context.laser_calibrations, dataset_pb2.LaserName.TOP), ws["camera_fusion"])
            calib = waymo_utils.get(frame.context.camera_calibrations, dataset_pb2.CameraName.FRONT)
            sensors = {a: perturbed_sensor(calib, ws["camera_fusion"], a) for a in ANGLES}
            for name, sensor in [("lidar", None), *[(f"yaw_{a:g}", sensors[a]) for a in ANGLES]]:
                scenarios[name] = {"sensor": sensor, "manager": TrackManager(ws["track_management"]),
                                   "filter": InstrumentedFilter(ws["kalman"], sensors, nominal=name=="yaw_0"),
                                   "rng": np.random.default_rng(0), "records": []}
        points = pcl_from_range_image(frame, dataset_pb2.LaserName.TOP)
        tensor = torch.from_numpy(ws["bev_mapping"].bev_maps_from_pcl(points, det_cfg)).unsqueeze(0).float()
        detections = ws["detection_pipeline"].detect_objects_from_bev(tensor, model, det_cfg)
        labels = valid_ground_truth(frame.laser_labels, det_cfg, label_pb2.Label.Type.TYPE_VEHICLE)
        counts = detection_counts(labels, detections, ws["detection_metrics"])
        for name, scenario in scenarios.items():
            manager, kf, camera = scenario["manager"], scenario["filter"], scenario["sensor"]
            kf.frame = frame_index
            observations = run_lab._lidar_observations(frame_index, detections, lidar, det_cfg)
            for track in manager.track_list:
                kf.predict(track)
                track.set_t(frame_index * params.dt)
            ws["association"].associate_and_update(manager, observations, kf, lidar)
            before = {t.id: track_copy(t) for t in manager.track_list}
            if camera is not None:
                camera_obs = run_lab._front_observations(frame, frame_index, camera, scenario["rng"], dataset_pb2.CameraName.FRONT, label_pb2.Label.Type.TYPE_VEHICLE)
                if camera_obs is not None:
                    ws["association"].associate_and_update(manager, camera_obs, kf, camera)
            record = {"mode": "lidar" if camera is None else "fused", "frame": frame_index,
                      **counts, "valid_gt": len(labels), **tracking_counts(manager.track_list, labels)}
            scenario["records"].append(record)
            if name == "yaw_0":
                after = {t.id: track_copy(t) for t in manager.track_list}
                assert before.keys() == after.keys()
                assert all((before[i]["state"], before[i]["score"]) == (after[i]["state"], after[i]["score"]) for i in before)
                nominal_export.append({"frame": frame_index, "tracks": list(after.values())})
                if labels:
                    centres = np.array([[l.box.center_x, l.box.center_y, l.box.center_z] for l in labels])
                    for track in manager.track_list:
                        if track.state != "confirmed":
                            continue
                        pre, post = np.array(before[track.id]["position"]), np.array(after[track.id]["position"])
                        idx = int(np.argmin(np.linalg.norm(centres[:, :2]-post[:2], axis=1)))
                        gt = centres[idx]
                        if np.linalg.norm(gt[:2]-post[:2]) > 2.0:
                            continue
                        improvement = float(np.linalg.norm(pre-gt)-np.linalg.norm(post-gt))
                        block = min(2, (frame_index-start)*3//(end-start+1))
                        if improvement > 0 and improvement > snapshots.get(block, {}).get("improvement", 0):
                            front = next(i for i in frame.images if i.name == dataset_pb2.CameraName.FRONT)
                            snapshots[block] = {"frame": frame_index, "track_id": track.id, "before": pre,
                                "after": post, "gt": gt, "gt_id": labels[idx].id, "improvement": improvement,
                                "points": points[::max(1, len(points)//20000)].copy(), "labels": centres,
                                "image": bytes(front.image), "sensor": camera,
                                "camera_pixels": [np.asarray(m.z).reshape(-1).tolist() for m in (camera_obs or [])]}
        if frame_index % 20 == 0 or frame_index == end:
            print(f"Bonus: frame {frame_index}/{end}, {len(detections)} detections", flush=True)
    actual_frames = [start, frame_index if frame_index <= end else end]
    summary = []
    probes = scenarios["yaw_0"]["filter"].probes
    for name, scenario in scenarios.items():
        mode = "lidar" if scenario["sensor"] is None else "fused"
        metrics = aggregate_records(scenario["records"], mode, actual_frames, 0, source.name)
        validate_metrics_records(metrics, scenario["records"])
        write_json(OUT / f"metrics_{name}.json", metrics)
        (OUT / f"records_{name}.jsonl").write_text("".join(record_json(r)+"\n" for r in scenario["records"]), encoding="utf-8")
        if mode == "lidar":
            continue
        angle = float(name.removeprefix("yaw_"))
        reference = [r for r in probes if r["yaw_deg"] == angle]
        visible = [r for r in reference if r["visible"]]
        updates = scenario["filter"].updates
        dump_csv(OUT / f"camera_updates_{name}.csv", updates,
                 ["frame", "track_id", "du_px", "dv_px", "d2"])
        stats = metrics["tracking"]["fused"]
        summary.append({"yaw_deg": angle, "rmse_m": stats["rmse"], "matches": stats["matches"],
                        "ghosts": stats["ghost_track_frames"], "misses": stats["missed_gt_frames"],
                        "camera_updates": len(updates), "reference_pairs": len(reference),
                        "probe_mean_du_px": float(np.mean([r["du_px"] for r in visible])),
                        "probe_mean_dv_px": float(np.mean([r["dv_px"] for r in visible])),
                        "probe_median_d2": float(np.median([r["d2"] for r in visible])),
                        "probe_gate_pass_fraction": sum(r["gate_pass"] for r in reference)/len(reference),
                        "probe_visible_pairs": len(visible)})
    dump_csv(OUT / "calibration_probes.csv", probes)
    dump_csv(OUT / "calibration_summary.csv", summary)
    write_json(OUT / "calibration_summary.json", summary)
    export_tracks_json(nominal_export, OUT / "tracks_export.json")
    figures(snapshots, summary)
    source_files = [ROOT / "student/workspace" / (name+".py") for name in ("kalman", "association", "camera_fusion", "track_management")]
    write_json(OUT / "provenance.json", {"segment": source.name, "frames": actual_frames, "seed": 0,
        "experiment": "T_bad = T_true @ Rz(yaw); identical raw observations and detections",
        "threads": 4, "source_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in source_files},
        "weights_sha256": hashlib.sha256(weights.read_bytes()).hexdigest(),
        "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()})
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()
