from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

ROOT = Path(__file__).resolve().parent
DEFAULT_SPEC = ROOT / "motion_spec.json"
LEG_ORDER = ("FL", "FR", "RL", "RR")


def load_spec(path: str | Path = DEFAULT_SPEC) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _vsub(a, b):
    return tuple(float(x - y) for x, y in zip(a, b))


def _rotate_x(point, pivot, deg):
    a = math.radians(deg)
    x, y, z = _vsub(point, pivot)
    return (
        pivot[0] + x,
        pivot[1] + y * math.cos(a) - z * math.sin(a),
        pivot[2] + y * math.sin(a) + z * math.cos(a),
    )


def fourbar_points(spec: dict, theta_deg: float, front: bool, left: bool, splay_deg: float = 0.0) -> Dict[str, Tuple[float, float, float]]:
    g = spec["fourbar"]
    r = float(g["crank_r_mm"])
    d = float(g["frame_d_mm"])
    l = float(g["coupler_l_mm"])
    q = float(g["rocker_q_mm"])
    e = float(g["foot_extension_mm"])
    th = math.radians(float(theta_deg))

    A2 = (0.0, 0.0)
    D2 = (d, 0.0)
    B2 = (r * math.cos(th), r * math.sin(th))
    vx, vz = D2[0] - B2[0], D2[1] - B2[1]
    dist = math.hypot(vx, vz)
    aa = (l * l - q * q + dist * dist) / (2.0 * dist)
    hh = math.sqrt(max(0.0, l * l - aa * aa))
    p0x = B2[0] + aa * vx / dist
    p0z = B2[1] + aa * vz / dist
    perpx, perpz = -vz / dist, vx / dist
    C2 = (p0x - hh * perpx, p0z - hh * perpz)
    ux, uz = (C2[0] - D2[0]) / q, (C2[1] - D2[1]) / q
    P2 = (C2[0] + e * ux, C2[1] + e * uz)

    xsign = 1.0 if front else -1.0
    y = float(g["leg_y_mm"]) if left else -float(g["leg_y_mm"])
    ax = xsign * float(g["front_root_x_mm"])
    z0 = float(g["leg_root_z_mm"])

    def map2(v):
        return (ax + xsign * float(v[0]), y, z0 + float(v[1]))

    A0, B0, C0, D0, P0 = [map2(v) for v in (A2, B2, C2, D2, P2)]
    phi = float(splay_deg) * (1.0 if left else -1.0)
    A, B, C, D, P = [_rotate_x(v, A0, phi) for v in (A0, B0, C0, D0, P0)]
    return {"A": A, "B": B, "C": C, "D": D, "P": P, "theta_deg": float(theta_deg), "splay_deg": float(splay_deg)}


def pose_points(spec: dict, pose: dict) -> Dict[str, Dict[str, Tuple[float, float, float]]]:
    out = {}
    for tag in LEG_ORDER:
        front = tag.startswith("F")
        left = tag.endswith("L")
        splay = pose["front_splay"] if front else pose["rear_splay"]
        out[tag] = fourbar_points(spec, pose[tag], front, left, splay)
    return out


def crouch_ankle_reference_z(spec: dict) -> float:
    pose = spec["expressions"]["sleepy_crouch"]
    return fourbar_points(spec, pose["FL"], True, True, 0.0)["P"][2]


def solve_body_pose(spec: dict, pose: dict) -> dict:
    points = pose_points(spec, pose)
    ankle_ref = crouch_ankle_reference_z(spec)
    rows, rhs = [], []
    for tag in LEG_ORDER:
        x, y, z = points[tag]["P"]
        rows.append((x, y, 1.0))
        rhs.append(ankle_ref - z)

    ata = [[0.0] * 3 for _ in range(3)]
    atb = [0.0] * 3
    for r, h in zip(rows, rhs):
        for i in range(3):
            atb[i] += r[i] * h
            for j in range(3):
                ata[i][j] += r[i] * r[j]

    m = [ata[i][:] + [atb[i]] for i in range(3)]
    for i in range(3):
        piv = max(range(i, 3), key=lambda k: abs(m[k][i]))
        m[i], m[piv] = m[piv], m[i]
        den = m[i][i] if abs(m[i][i]) > 1e-12 else 1e-12
        for j in range(i, 4):
            m[i][j] /= den
        for k in range(3):
            if k == i:
                continue
            f = m[k][i]
            for j in range(i, 4):
                m[k][j] -= f * m[i][j]
    a, b, c = [m[i][3] for i in range(3)]

    belly = float(spec["body"]["shell_belly_z_mm"])
    return {
        "roll_deg": math.degrees(math.atan(b)),
        "pitch_deg": math.degrees(math.atan(-a)),
        "body_dz_mm": c,
        "center_belly_clearance_mm": belly + c,
        "ankle_reference_z_mm": ankle_ref,
    }


def smooth5(u: float) -> float:
    u = max(0.0, min(1.0, float(u)))
    return u * u * u * (10.0 - 15.0 * u + 6.0 * u * u)


def _transition_duration(spec: dict, a: dict, b: dict, requested: float) -> float:
    main_v = float(spec["main_motor"]["expression_loaded_speed_limit_rpm"]) * 6.0
    splay_v = float(spec["splay_motor"]["speed_limit_deg_s"])
    needed = 0.0
    for tag in LEG_ORDER:
        needed = max(needed, 1.875 * abs(float(b[tag]) - float(a[tag])) / max(1e-9, main_v))
    for key in ("front_splay", "rear_splay"):
        needed = max(needed, 1.875 * abs(float(b[key]) - float(a[key])) / max(1e-9, splay_v))
    return max(float(requested), needed)


def interpolate_pose(a: dict, b: dict, u: float) -> dict:
    s = smooth5(u)
    return {k: float(a[k]) + (float(b[k]) - float(a[k])) * s for k in (*LEG_ORDER, "front_splay", "rear_splay")}


@dataclass
class MotionFrame:
    time_s: float
    label: str
    pose: dict
    body: dict
    legs: dict
    main_motor_speed_deg_s: Dict[str, float]
    splay_speed_deg_s: Dict[str, float]


def build_demo_timeline(spec: dict, fps: float = 20.0) -> List[MotionFrame]:
    expressions = spec["expressions"]
    seq = spec["demo_sequence"]
    frames: List[MotionFrame] = []
    t = 0.0
    dt = 1.0 / float(fps)
    current = expressions[seq[0]]

    def add_frame(label, pose, speeds_main=None, speeds_splay=None):
        nonlocal t
        frames.append(MotionFrame(
            time_s=t,
            label=label,
            pose={k: float(pose[k]) for k in (*LEG_ORDER, "front_splay", "rear_splay")},
            body=solve_body_pose(spec, pose),
            legs=pose_points(spec, pose),
            main_motor_speed_deg_s=speeds_main or {k: 0.0 for k in LEG_ORDER},
            splay_speed_deg_s=speeds_splay or {"front": 0.0, "rear": 0.0},
        ))
        t += dt

    for _ in range(max(1, round(float(current.get("hold_s", 0.3)) * fps))):
        add_frame(seq[0], current)

    for name in seq[1:]:
        target = expressions[name]
        T = _transition_duration(spec, current, target, float(target.get("transition_s", 0.6)))
        n = max(2, round(T * fps))
        prev = {k: float(current[k]) for k in (*LEG_ORDER, "front_splay", "rear_splay")}
        for i in range(1, n + 1):
            pose = interpolate_pose(current, target, i / n)
            speeds_main = {k: (pose[k] - prev[k]) / dt for k in LEG_ORDER}
            speeds_splay = {
                "front": (pose["front_splay"] - prev["front_splay"]) / dt,
                "rear": (pose["rear_splay"] - prev["rear_splay"]) / dt,
            }
            add_frame(name, pose, speeds_main, speeds_splay)
            prev = pose
        for _ in range(max(0, round(float(target.get("hold_s", 0.3)) * fps))):
            add_frame(name, target)
        current = target
    return frames


def main():
    spec = load_spec()
    frames = build_demo_timeline(spec, fps=20)
    peak_main = max(abs(v) for f in frames for v in f.main_motor_speed_deg_s.values())
    peak_splay = max(abs(v) for f in frames for v in f.splay_speed_deg_s.values())
    print(f"frames={len(frames)} duration={frames[-1].time_s:.2f}s")
    print(f"peak main speed={peak_main:.1f} deg/s ({peak_main/6.0:.1f} rpm)")
    print(f"peak splay speed={peak_splay:.1f} deg/s")


if __name__ == "__main__":
    main()
