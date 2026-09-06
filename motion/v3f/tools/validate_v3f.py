from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Dict

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SPEC_PATH = ROOT / "shared" / "v3f_mechanical_spec.json"
ITERATION_PATH = ROOT / "shared" / "iteration_config.json"
OUT_DIR = ROOT / "output"


def load_json(path: Path):
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def fourbar_endpoint(spec: dict, theta_deg: float, front: bool, left: bool, splay_deg: float = 0.0) -> np.ndarray:
    fb = spec["fourbar_mm"]
    crank_r = float(fb["crank_r"])
    frame_d = float(fb["frame_d"])
    coupler_l = float(fb["coupler_l"])
    rocker_q = float(fb["rocker_q"])
    foot_ext = float(fb["foot_ext"])
    lo, hi = map(float, fb["theta_range_deg"])
    theta_deg = min(max(float(theta_deg), lo), hi)

    th = math.radians(theta_deg)
    a2 = np.array([0.0, 0.0])
    d2 = np.array([frame_d, 0.0])
    b2 = a2 + crank_r * np.array([math.cos(th), math.sin(th)])
    v = d2 - b2
    dist = float(np.linalg.norm(v))
    aa = (coupler_l**2 - rocker_q**2 + dist**2) / (2.0 * dist)
    hh = math.sqrt(max(0.0, coupler_l**2 - aa**2))
    p0 = b2 + aa * v / dist
    perp = np.array([-v[1], v[0]]) / dist
    c2 = p0 - hh * perp
    u = (c2 - d2) / rocker_q
    p2 = c2 + foot_ext * u

    xs = 1.0 if front else -1.0
    ys = 1.0 if left else -1.0
    root_x, root_y, root_z = 18.0, 43.0, 36.5
    p = np.array([xs * root_x + xs * p2[0], ys * root_y, root_z + p2[1]], dtype=float)
    pivot = np.array([xs * root_x, ys * root_y, root_z], dtype=float)
    phi = math.radians(float(splay_deg) * ys)
    r = p - pivot
    return pivot + np.array([r[0], r[1] * math.cos(phi) - r[2] * math.sin(phi), r[1] * math.sin(phi) + r[2] * math.cos(phi)])


def pose_points(spec: dict, pose: dict) -> Dict[str, np.ndarray]:
    result = {}
    for tag, front, left in (("FL", True, True), ("FR", True, False), ("RL", False, True), ("RR", False, False)):
        splay = float(pose["front_splay"] if front else pose["rear_splay"])
        result[tag] = fourbar_endpoint(spec, pose[tag], front, left, splay)
    return result


def plane_metrics(points: Dict[str, np.ndarray], reference_points: Dict[str, np.ndarray], sleepy_clearance: float) -> dict:
    def solve(pts: Dict[str, np.ndarray]) -> np.ndarray:
        M, h = [], []
        for p in pts.values():
            M.append([p[0], p[1], 1.0])
            h.append(-p[2])
        return np.linalg.lstsq(np.asarray(M), np.asarray(h), rcond=None)[0]

    a, b, c = solve(points)
    _, _, cref = solve(reference_points)
    return {
        "belly_clearance_mm": round(float(sleepy_clearance + c - cref), 3),
        "pitch_deg": round(float(math.degrees(math.atan(-a))), 3),
        "roll_deg": round(float(math.degrees(math.atan(b))), 3),
    }


def workspace(spec: dict, samples_theta: int = 91, samples_splay: int = 21) -> dict:
    fb = spec["fourbar_mm"]
    lo, hi = map(float, fb["theta_range_deg"])
    slo, shi = map(float, spec["splay_servo"]["cassette_range_deg"])
    rows = []
    for theta in np.linspace(lo, hi, samples_theta):
        for splay in np.linspace(slo, shi, samples_splay):
            rows.append(fourbar_endpoint(spec, float(theta), True, True, float(splay)))
    arr = np.asarray(rows)
    span = arr.max(axis=0) - arr.min(axis=0)
    return {
        "ankle_span_x_mm": round(float(span[0]), 3),
        "ankle_span_y_mm": round(float(span[1]), 3),
        "ankle_span_z_mm": round(float(span[2]), 3),
        "bounds_min_mm": [round(float(x), 3) for x in arr.min(axis=0)],
        "bounds_max_mm": [round(float(x), 3) for x in arr.max(axis=0)],
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Fast V3-F mechanical regression checks; no MuJoCo or FEM required.")
    ap.add_argument("--out", type=Path, default=OUT_DIR / "validation_summary.json")
    args = ap.parse_args()

    spec = load_json(SPEC_PATH)
    iteration = load_json(ITERATION_PATH)
    baseline_path = ROOT / "shared" / "baseline_metrics.json"
    baseline = load_json(baseline_path) if baseline_path.exists() else None
    poses = spec["pose_targets"]
    sleepy_pts = pose_points(spec, poses["sleepy_compact"])
    sleepy_clearance = float(iteration["validation_targets"]["nominal_sleepy_belly_clearance_mm"])

    pose_results = {}
    range_failures = []
    lo, hi = map(float, spec["fourbar_mm"]["theta_range_deg"])
    slo, shi = map(float, spec["splay_servo"]["cassette_range_deg"])
    for name, pose in poses.items():
        for leg in ("FL", "FR", "RL", "RR"):
            if not lo <= float(pose[leg]) <= hi:
                range_failures.append(f"{name}:{leg}={pose[leg]} outside [{lo},{hi}]")
        for key in ("front_splay", "rear_splay"):
            if not slo <= float(pose[key]) <= shi:
                range_failures.append(f"{name}:{key}={pose[key]} outside [{slo},{shi}]")
        pose_results[name] = plane_metrics(pose_points(spec, pose), sleepy_pts, sleepy_clearance)

    ws = workspace(spec)
    margin = 2.0 * float(iteration["validation_targets"]["shell_slot_margin_each_side_mm"])
    recommended_slot = {
        "x_mm": round(ws["ankle_span_x_mm"] + margin, 3),
        "y_mm": round(ws["ankle_span_y_mm"] + margin, 3),
    }

    ov = spec["overload"]
    spring_force = float(ov["spring_count_per_leg"]) * (
        float(ov["preload_N_each"]) + float(ov["spring_k_N_per_mm_each"]) * float(ov["vertical_travel_mm"])
    )
    saver = spec["splay_servo_saver"]
    saver_force = float(saver["preload_N"]) + float(saver["k_N_per_mm"]) * float(saver["travel_mm"])

    delta = None
    if baseline is not None:
        delta = {
            "workspace_x_mm": round(ws["ankle_span_x_mm"] - baseline["workspace"]["ankle_span_x_mm"], 3),
            "workspace_y_mm": round(ws["ankle_span_y_mm"] - baseline["workspace"]["ankle_span_y_mm"], 3),
            "workspace_z_mm": round(ws["ankle_span_z_mm"] - baseline["workspace"]["ankle_span_z_mm"], 3),
            "slot_x_mm": round(recommended_slot["x_mm"] - baseline["recommended_hidden_rigid_slot_mm"]["x_mm"], 3),
            "slot_y_mm": round(recommended_slot["y_mm"] - baseline["recommended_hidden_rigid_slot_mm"]["y_mm"], 3),
            "pose_belly_mm": {
                name: round(pose_results[name]["belly_clearance_mm"] - baseline["pose_body_metrics"][name]["belly_clearance_mm"], 3)
                for name in pose_results if name in baseline["pose_body_metrics"]
            },
        }

    warnings = [
        "Antenna articulation is not yet mechanically modeled in V3-F CAD.",
        "Exact B-Rep part-to-part collision checking is a separate CAD sweep step; this fast check validates analytical workspace and command ranges.",
        "FEA/stress results are not executed by the quick regression; run only after load-path geometry changes or at freeze gates.",
    ]
    result = {
        "status": "FAIL" if range_failures else "PASS_WITH_WARNINGS",
        "source": "motion/v3f/shared/v3f_mechanical_spec.json",
        "workspace": ws,
        "recommended_hidden_rigid_slot_mm": recommended_slot,
        "pose_body_metrics": pose_results,
        "overload": {
            "spring_path_force_at_hardstop_N_per_leg": round(spring_force, 3),
            "servo_saver_force_at_full_travel_N": round(saver_force, 3),
        },
        "range_failures": range_failures,
        "delta_from_baseline": delta,
        "warnings": warnings,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2), encoding="utf-8")

    status_md = args.out.with_name("V3F_STATUS.md")
    if delta:
        delta_line = (
            f"Delta from frozen baseline: workspace X/Y/Z "
            f"**{delta['workspace_x_mm']:+.3f}/{delta['workspace_y_mm']:+.3f}/{delta['workspace_z_mm']:+.3f} mm**."
        )
    else:
        delta_line = "No baseline comparison available."
    lines = [
        "# V3-F Current Mechanical Regression",
        "",
        f"**Status:** {result['status']}",
        "",
        f"Ankle workspace: **{ws['ankle_span_x_mm']} x {ws['ankle_span_y_mm']} x {ws['ankle_span_z_mm']} mm (X/Y/Z)**",
        f"Recommended hidden rigid slot with current margin: **{recommended_slot['x_mm']} x {recommended_slot['y_mm']} mm**",
        f"Spring load path reaches hard stop at approximately **{spring_force:.1f} N per leg**.",
        delta_line,
        "",
        "## Sentinel expression metrics",
        "",
        "| Pose | Belly mm | Pitch deg | Roll deg |",
        "|---|---:|---:|---:|",
    ]
    for name in iteration["quick_sentinel_poses"]:
        m = pose_results[name]
        lines.append(f"| {name} | {m['belly_clearance_mm']:.2f} | {m['pitch_deg']:.2f} | {m['roll_deg']:.2f} |")
    lines += ["", "## Deliberately deferred", "", ", ".join(iteration["deferred"]), "", "## Current warnings", ""]
    lines += [f"- {w}" for w in warnings]
    status_md.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(json.dumps(result, indent=2))
    return 1 if range_failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
