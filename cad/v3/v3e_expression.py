from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Dict, Tuple

import cadquery as cq
import numpy as np

import generate_ux_shell_leg as base

OUT = Path(os.environ.get("PEBBLE_CAD_OUT", Path(__file__).resolve().parent / "generated_v3e"))
OUT.mkdir(parents=True, exist_ok=True)

# V3-E keeps the V3-D UX shell envelope but applies the two approved final
# surface corrections: a slightly fuller front/rear plan-view lobe and a softer
# visor lower edge without the central cheek/mouth uplift.
base.BODY_SECTIONS = [
    (8.0, 46.0, 44.5, 52.5, 2.03),
    (15.0, 56.0, 54.0, 62.5, 2.02),
    (28.0, 66.5, 64.5, 73.5, 2.01),
    (45.0, 73.7, 71.7, 81.0, 2.00),
    (61.0, 75.25, 74.25, 82.5, 2.00),
    (77.0, 74.0, 73.0, 80.5, 2.00),
    (92.0, 69.2, 68.2, 74.0, 2.00),
    (106.0, 61.0, 60.0, 65.5, 2.00),
    (119.0, 51.0, 50.0, 54.0, 2.00),
    (129.0, 37.5, 36.5, 40.0, 2.00),
    (136.0, 15.5, 14.5, 18.0, 2.00),
    (138.0, 3.0, 3.0, 4.0, 2.00),
]


def pebble_face_wire_v3e(
    x: float,
    center_z: float,
    half_w: float,
    top_h: float,
    bottom_h: float,
    scale: float = 1.0,
) -> cq.Wire:
    """V3-E face: forehead narrow, lower-middle broad, soft lower edge.

    The V3-D centre-bottom point was slightly higher than its neighbours and
    produced an unintended cheek/mouth uplift in front view. V3-E removes that
    visual kink while retaining an organic non-rectangular visor.
    """
    w = half_w * scale
    th = top_h * scale
    bh = bottom_h * scale
    yz = [
        (0.00, +1.00 * th),
        (+0.32 * w, +0.97 * th),
        (+0.57 * w, +0.86 * th),
        (+0.76 * w, +0.66 * th),
        (+0.88 * w, +0.40 * th),
        (+0.95 * w, +0.10 * th),
        (+0.99 * w, -0.16 * bh),
        (+1.00 * w, -0.35 * bh),
        (+0.96 * w, -0.55 * bh),
        (+0.84 * w, -0.73 * bh),
        (+0.62 * w, -0.86 * bh),
        (+0.34 * w, -0.96 * bh),
        (0.00, -0.95 * bh),
        (-0.34 * w, -0.96 * bh),
        (-0.62 * w, -0.86 * bh),
        (-0.84 * w, -0.73 * bh),
        (-0.96 * w, -0.55 * bh),
        (-1.00 * w, -0.35 * bh),
        (-0.99 * w, -0.16 * bh),
        (-0.95 * w, +0.10 * th),
        (-0.88 * w, +0.40 * th),
        (-0.76 * w, +0.66 * th),
        (-0.57 * w, +0.86 * th),
        (-0.32 * w, +0.97 * th),
    ]
    pts = [cq.Vector(x, y, center_z + dz) for y, dz in yz]
    edge = cq.Edge.makeSpline(pts, periodic=True)
    return cq.Wire.assembleEdges([edge])


base.pebble_face_wire = pebble_face_wire_v3e

# Six active DOF: four independent primary leg cranks plus front/rear shared
# mirrored cassette-splay actuators. Positive splay always means paw-outward.
SPLAY_OUT_DEG = 20.0
SPLAY_IN_DEG = -18.0
SPLAY_SERVO_ENVELOPE_MM = (24.0, 13.0, 24.0)
SPLAY_PIVOT_SHAFT_RADIUS = 2.5
SPLAY_PIVOT_SHAFT_HALF_LEN = 8.0

CROUCH_THETA_DEG = 330.0
NORMAL_THETA_DEG = 296.0
ALERT_THETA_DEG = 242.0
BOUNCE_PEAK_THETA_DEG = 180.0

EXPRESSION_POSES = {
    "sleepy_crouch": {"FL": 330.0, "FR": 330.0, "RL": 330.0, "RR": 330.0, "front_splay": -15.0, "rear_splay": -14.0},
    "neutral": {"FL": 296.0, "FR": 296.0, "RL": 296.0, "RR": 296.0, "front_splay": 0.0, "rear_splay": 0.0},
    "alert": {"FL": 242.0, "FR": 242.0, "RL": 242.0, "RR": 242.0, "front_splay": 6.0, "rear_splay": 5.0},
    "happy_wide": {"FL": 205.0, "FR": 205.0, "RL": 205.0, "RR": 205.0, "front_splay": 18.0, "rear_splay": 17.0},
    "play_bow": {"FL": 326.0, "FR": 326.0, "RL": 246.0, "RR": 246.0, "front_splay": 18.0, "rear_splay": 6.0},
    "curious_left": {"FL": 322.0, "FR": 252.0, "RL": 318.0, "RR": 250.0, "front_splay": 4.0, "rear_splay": 2.0},
    "curious_right": {"FL": 252.0, "FR": 322.0, "RL": 250.0, "RR": 318.0, "front_splay": 4.0, "rear_splay": 2.0},
    "wide_flat": {"FL": 300.0, "FR": 300.0, "RL": 300.0, "RR": 300.0, "front_splay": 20.0, "rear_splay": 20.0},
    "inward_compact": {"FL": 315.0, "FR": 315.0, "RL": 315.0, "RR": 315.0, "front_splay": -18.0, "rear_splay": -18.0},
    "walk_diagonal": {"FL": 270.0, "FR": 315.0, "RL": 315.0, "RR": 270.0, "front_splay": 3.0, "rear_splay": 2.0},
}


def fourbar(theta_deg: float, front: bool, left: bool, splay_deg: float = 0.0) -> Dict[str, object]:
    """V3-D four-bar with a cassette-roll stance DOF about the local X axis."""
    th = math.radians(theta_deg)
    A2 = np.array([0.0, 0.0])
    D2 = np.array([base.FRAME_D, 0.0])
    B2 = A2 + base.CRANK_R * np.array([math.cos(th), math.sin(th)])

    v = D2 - B2
    dist = float(np.linalg.norm(v))
    a = (base.COUPLER_L**2 - base.ROCKER_Q**2 + dist**2) / (2.0 * dist)
    h = math.sqrt(max(0.0, base.COUPLER_L**2 - a**2))
    p0 = B2 + a * v / dist
    perp = np.array([-v[1], v[0]]) / dist
    C2 = p0 - h * perp
    u = (C2 - D2) / base.ROCKER_Q
    P2 = C2 + base.FOOT_EXT * u

    x_sign = 1.0 if front else -1.0
    y = base.LEG_Y if left else -base.LEG_Y
    ax = x_sign * base.FRONT_A_X

    def map2(q: np.ndarray) -> Tuple[float, float, float]:
        return (ax + x_sign * float(q[0]), y, base.LEG_ROOT_Z + float(q[1]))

    A0, B0, C0, D0, P0 = [map2(q) for q in (A2, B2, C2, D2, P2)]
    phi_deg = float(splay_deg) * (1.0 if left else -1.0)
    phi = math.radians(phi_deg)
    pivot = np.array(A0, dtype=float)

    def splay_point(q):
        r = np.array(q, dtype=float) - pivot
        y2 = r[1] * math.cos(phi) - r[2] * math.sin(phi)
        z2 = r[1] * math.sin(phi) + r[2] * math.cos(phi)
        return (float(pivot[0] + r[0]), float(pivot[1] + y2), float(pivot[2] + z2))

    A, B, C, D, P = [splay_point(q) for q in (A0, B0, C0, D0, P0)]
    links = [
        base.cylinder_between(A, B, base.LINK_RADIUS),
        base.cylinder_between(B, C, base.LINK_RADIUS),
        base.cylinder_between(D, C, base.LINK_RADIUS),
        base.cylinder_between(C, P, base.LINK_RADIUS),
    ]
    pivots = [cq.Solid.makeSphere(base.PIVOT_RADIUS, cq.Vector(*q)) for q in (A, B, C, D)]

    motor_center_y = y - math.copysign(15.0, y)
    motor = (
        cq.Workplane("XY")
        .box(25.0, 30.0, 12.0)
        .edges()
        .fillet(1.5)
        .translate((A0[0], motor_center_y, A0[2]))
        .val()
    )
    if abs(phi_deg) > 1e-9:
        motor = motor.rotate(A0, (A0[0] + 1.0, A0[1], A0[2]), phi_deg)

    return {"A": A, "B": B, "C": C, "D": D, "P": P, "links": links, "pivots": pivots, "motor": motor, "splay_deg": float(splay_deg)}


def crouch_ankle_reference_z() -> float:
    return float(fourbar(CROUCH_THETA_DEG, True, True, 0.0)["P"][2])


def pose_metrics(name: str, pose: Dict[str, float]) -> Dict[str, object]:
    ankle_world_z = crouch_ankle_reference_z()
    rows = []
    points = {}
    for tag, front, left in (("FL", True, True), ("FR", True, False), ("RL", False, True), ("RR", False, False)):
        splay = pose["front_splay"] if front else pose["rear_splay"]
        P = np.array(fourbar(pose[tag], front, left, splay)["P"], dtype=float)
        points[tag] = P.tolist()
        rows.append([P[0], P[1], 1.0, ankle_world_z - P[2]])

    M = np.array([r[:3] for r in rows], dtype=float)
    h = np.array([r[3] for r in rows], dtype=float)
    a, b, c = np.linalg.lstsq(M, h, rcond=None)[0]
    return {
        "name": name,
        "joint_targets_deg": {k: float(pose[k]) for k in ("FL", "FR", "RL", "RR")},
        "front_splay_deg": float(pose["front_splay"]),
        "rear_splay_deg": float(pose["rear_splay"]),
        "predicted_center_belly_clearance_mm": round(base.BELLY_Z + float(c), 3),
        "predicted_pitch_deg": round(math.degrees(math.atan(-a)), 3),
        "predicted_roll_deg": round(math.degrees(math.atan(b)), 3),
        "ankle_points_body_mm": points,
    }


def splay_servo(front: bool) -> cq.Shape:
    lx, ly, lz = SPLAY_SERVO_ENVELOPE_MM
    x = 31.0 if front else -38.0
    return cq.Workplane("XY").box(lx, ly, lz).edges().fillet(1.5).translate((x, 0.0, 37.0)).val()


def splay_pivot_shaft(front: bool, left: bool) -> cq.Shape:
    A = fourbar(NORMAL_THETA_DEG, front, left, 0.0)["A"]
    start = cq.Vector(A[0] - SPLAY_PIVOT_SHAFT_HALF_LEN, A[1], A[2])
    return cq.Solid.makeCylinder(
        SPLAY_PIVOT_SHAFT_RADIUS,
        2.0 * SPLAY_PIVOT_SHAFT_HALF_LEN,
        start,
        cq.Vector(1.0, 0.0, 0.0),
    )


def build_assembly():
    shell, outer_ref = base.make_body_shell()
    visor = base.make_visor(outer_ref)
    aL, tipL = base.antenna(+1.0)
    aR, tipR = base.antenna(-1.0)

    leg_cache = {}
    for front in (True, False):
        for left in (True, False):
            tag = ("F" if front else "R") + ("L" if left else "R")
            leg_cache[tag] = fourbar(NORMAL_THETA_DEG, front, left, 0.0)
            P = leg_cache[tag]["P"]
            pocket = base.pebble_foot(P[0], P[1], scale=1.02)
            reveal_band = cq.Workplane("XY").box(34.0, 29.0, 9.0).translate((P[0], P[1], 10.5)).val()
            shell = shell.cut(pocket.intersect(reveal_band))

    components = {"shell": shell, "visor": visor, "antenna_left": aL, "antenna_right": aR, "tip_left": tipL, "tip_right": tipR}
    asm = cq.Assembly(name="Pebble_V3E_UX_Shell_Leg_Concept")
    asm.add(shell, name="shell", color=cq.Color(0.93, 0.90, 0.84))
    asm.add(visor, name="visor", color=cq.Color(0.035, 0.04, 0.05))
    asm.add(aL, name="antenna_left", color=cq.Color(0.10, 0.10, 0.11))
    asm.add(aR, name="antenna_right", color=cq.Color(0.10, 0.10, 0.11))
    asm.add(tipL, name="antenna_tip_left", color=cq.Color(1.0, 0.75, 0.34))
    asm.add(tipR, name="antenna_tip_right", color=cq.Color(1.0, 0.75, 0.34))

    for front in (True, False):
        label = "front" if front else "rear"
        servo = splay_servo(front)
        components[f"splay_servo_{label}"] = servo
        asm.add(servo, name=f"splay_servo_{label}", color=cq.Color(0.30, 0.38, 0.45))
        for left in (True, False):
            tag = ("F" if front else "R") + ("L" if left else "R")
            shaft = splay_pivot_shaft(front, left)
            components[f"splay_pivot_{tag}"] = shaft
            asm.add(shaft, name=f"splay_pivot_{tag}", color=cq.Color(0.34, 0.35, 0.37))

    for front in (True, False):
        for left in (True, False):
            tag = ("F" if front else "R") + ("L" if left else "R")
            leg = leg_cache[tag]
            P = leg["P"]
            foot = base.pebble_foot(P[0], P[1], scale=(0.92 if front else 0.84))
            components[f"foot_{tag}"] = foot
            components[f"motor_{tag}"] = leg["motor"]
            asm.add(foot, name=f"foot_{tag}", color=cq.Color(0.84, 0.82, 0.78))
            asm.add(leg["motor"], name=f"motor_{tag}", color=cq.Color(0.42, 0.45, 0.48))
            for i, link in enumerate(leg["links"]):
                components[f"link_{tag}_{i}"] = link
                asm.add(link, name=f"link_{tag}_{i}", color=cq.Color(0.62, 0.64, 0.66))
            for i, pivot in enumerate(leg["pivots"]):
                components[f"pivot_{tag}_{i}"] = pivot
                asm.add(pivot, name=f"pivot_{tag}_{i}", color=cq.Color(0.30, 0.32, 0.34))

    traj = np.array([fourbar(float(th), True, True, 0.0)["P"] for th in np.linspace(0.0, 360.0, 181)])
    return asm, components, traj


def main() -> None:
    asm, components, traj = build_assembly()
    full_step = OUT / "Pebble_V3E_UX_Shell_Leg_Concept.step"
    asm.save(str(full_step), exportType="STEP", mode="default")

    ext = cq.Assembly(name="Pebble_V3E_UX_Exterior")
    for name in ("shell", "visor", "antenna_left", "antenna_right", "tip_left", "tip_right", "foot_FL", "foot_FR", "foot_RL", "foot_RR"):
        shape = components[name]
        if name == "shell": color = cq.Color(0.93, 0.90, 0.84)
        elif name == "visor": color = cq.Color(0.035, 0.04, 0.05)
        elif name.startswith("tip"): color = cq.Color(1.0, 0.75, 0.34)
        elif name.startswith("antenna"): color = cq.Color(0.10, 0.10, 0.11)
        else: color = cq.Color(0.84, 0.82, 0.78)
        ext.add(shape, name=name, color=color)
    ext.save(str(OUT / "Pebble_V3E_UX_Exterior.step"), exportType="STEP", mode="default")

    np.savetxt(OUT / "Pebble_V3E_FrontLeft_FootTrajectory.csv", traj, delimiter=",", header="x_mm,y_mm,z_mm", comments="")
    metrics = {name: pose_metrics(name, pose) for name, pose in EXPRESSION_POSES.items()}
    data = {
        "version": "V3-E",
        "main_leg_actuators": 4,
        "shared_splay_actuators": 2,
        "total_active_dof": 6,
        "splay_limits_deg": [SPLAY_IN_DEG, SPLAY_OUT_DEG],
        "crouch_ankle_reference_z_mm": crouch_ankle_reference_z(),
        "poses": metrics,
    }
    (OUT / "Pebble_V3E_ExpressionPoseTargets.json").write_text(json.dumps(data, indent=2))

    bb = components["shell"].BoundingBox()
    print(f"shell={bb.xlen:.2f} x {bb.ylen:.2f} x {bb.zlen:.2f} mm")
    clearances = [v["predicted_center_belly_clearance_mm"] for v in metrics.values()]
    rolls = [abs(v["predicted_roll_deg"]) for v in metrics.values()]
    pitches = [abs(v["predicted_pitch_deg"]) for v in metrics.values()]
    print(f"belly clearance={min(clearances):.2f}..{max(clearances):.2f} mm")
    print(f"max roll={max(rolls):.2f} deg, max pitch={max(pitches):.2f} deg")


if __name__ == "__main__":
    main()
