from __future__ import annotations

"""Pebble V3-F mechanically resolved CAD generator.

This generator intentionally keeps the V3-E exterior UX shell and replaces the
conceptual V3-E leg presentation with a manufacturable architecture baseline:

* 4 x GA12/N20 6 V encoder gearmotors
* external 12T:18T m0.5 reduction onto a separate 3 mm crank shaft
* dual 623ZZ support bearings per crank shaft
* compact four-bar leg cassette
* passive compliant ankle + continuous TPU/TPE gaiter into the cosmetic paw
* 2 x MG90S metal-gear micro servos for front/rear symmetric cassette splay
* bellcrank/pushrod servo-saver path for splay
* spring-mounted cassettes with 5 mm overload travel into a structural belly ring

Generated STEP files are review artifacts. The parametric dimensions in this file
and ../shared/v3f_mechanical_spec.json are the source of truth.
"""

import json
import math
import os
import sys
from pathlib import Path
from typing import Dict, Tuple

import cadquery as cq
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
V3_DIR = ROOT / "cad" / "v3"
if str(V3_DIR) not in sys.path:
    sys.path.insert(0, str(V3_DIR))

import v3e_expression as v3e  # type: ignore  # noqa: E402

SPEC_PATH = HERE.parent / "shared" / "v3f_mechanical_spec.json"
OUT = Path(os.environ.get("PEBBLE_V3F_OUT", HERE / "generated"))
OUT.mkdir(parents=True, exist_ok=True)

with SPEC_PATH.open("r", encoding="utf-8") as f:
    SPEC = json.load(f)

CRANK_R = float(SPEC["fourbar_mm"]["crank_r"])
FRAME_D = float(SPEC["fourbar_mm"]["frame_d"])
COUPLER_L = float(SPEC["fourbar_mm"]["coupler_l"])
ROCKER_Q = float(SPEC["fourbar_mm"]["rocker_q"])
FOOT_EXT = float(SPEC["fourbar_mm"]["foot_ext"])
THETA_MIN, THETA_MAX = map(float, SPEC["fourbar_mm"]["theta_range_deg"])

LEG_ROOT_Z = 36.5
LEG_ROOT_X = 18.0
LEG_ROOT_Y = 43.0
SPLAY_MAX = 10.0

CRANK_SHAFT_D = 3.0
BEARING_OD = 10.0
BEARING_W = 4.0
PINION_TEETH = 12
CRANK_GEAR_TEETH = 18
GEAR_MODULE = 0.5
PINION_PD = PINION_TEETH * GEAR_MODULE
CRANK_GEAR_PD = CRANK_GEAR_TEETH * GEAR_MODULE
POSES = SPEC["pose_targets"]


def cylinder_between(p1: Tuple[float, float, float], p2: Tuple[float, float, float], r: float) -> cq.Shape:
    a, b = cq.Vector(*p1), cq.Vector(*p2)
    d = b - a
    if d.Length < 1e-9:
        return cq.Solid.makeSphere(r, a)
    return cq.Solid.makeCylinder(r, d.Length, a, d.normalized())


def fourbar_points(theta_deg: float, front: bool, left: bool, splay_deg: float = 0.0) -> Dict[str, Tuple[float, float, float]]:
    theta_deg = max(THETA_MIN, min(THETA_MAX, float(theta_deg)))
    th = math.radians(theta_deg)
    A2 = np.array([0.0, 0.0])
    D2 = np.array([FRAME_D, 0.0])
    B2 = A2 + CRANK_R * np.array([math.cos(th), math.sin(th)])
    v = D2 - B2
    dist = float(np.linalg.norm(v))
    a = (COUPLER_L**2 - ROCKER_Q**2 + dist**2) / (2.0 * dist)
    h = math.sqrt(max(0.0, COUPLER_L**2 - a**2))
    p0 = B2 + a * v / dist
    perp = np.array([-v[1], v[0]]) / dist
    C2 = p0 - h * perp
    u = (C2 - D2) / ROCKER_Q
    P2 = C2 + FOOT_EXT * u

    xs = 1.0 if front else -1.0
    ys = 1.0 if left else -1.0
    x0 = xs * LEG_ROOT_X
    y0 = ys * LEG_ROOT_Y

    def map2(q: np.ndarray) -> np.ndarray:
        return np.array([x0 + xs * float(q[0]), y0, LEG_ROOT_Z + float(q[1])], dtype=float)

    raw = {k: map2(q) for k, q in zip(("A", "B", "C", "D", "P"), (A2, B2, C2, D2, P2))}
    phi = math.radians(float(splay_deg) * ys)
    pivot = raw["A"].copy()

    def rotate_x(q: np.ndarray) -> Tuple[float, float, float]:
        r = q - pivot
        y = r[1] * math.cos(phi) - r[2] * math.sin(phi)
        z = r[1] * math.sin(phi) + r[2] * math.cos(phi)
        return (float(pivot[0] + r[0]), float(pivot[1] + y), float(pivot[2] + z))

    return {k: rotate_x(q) for k, q in raw.items()}


def n20_motor_shape(center: Tuple[float, float, float], left: bool) -> cq.Shape:
    x, y, z = center
    body = cq.Workplane("XY").box(10.0, 24.0, 12.0).edges().fillet(1.2).translate((x, y, z)).val()
    nose_dir = 1.0 if left else -1.0
    nose = cq.Solid.makeCylinder(3.0, 8.0, cq.Vector(x, y + nose_dir * 12.0, z), cq.Vector(0, nose_dir, 0))
    shaft = cq.Solid.makeCylinder(1.5, 6.0, cq.Vector(x, y + nose_dir * 20.0, z), cq.Vector(0, nose_dir, 0))
    return body.fuse(nose).fuse(shaft)


def bearing_shape(center: Tuple[float, float, float]) -> cq.Shape:
    x, y, z = center
    outer = cq.Solid.makeCylinder(BEARING_OD / 2.0, BEARING_W, cq.Vector(x, y - BEARING_W / 2, z), cq.Vector(0, 1, 0))
    inner = cq.Solid.makeCylinder(CRANK_SHAFT_D / 2.0, BEARING_W + 1.0, cq.Vector(x, y - (BEARING_W + 1.0) / 2, z), cq.Vector(0, 1, 0))
    return outer.cut(inner)


def gear_shape(center: Tuple[float, float, float], pitch_d: float, width: float = 3.0) -> cq.Shape:
    x, y, z = center
    return cq.Solid.makeCylinder(pitch_d / 2.0, width, cq.Vector(x, y - width / 2, z), cq.Vector(0, 1, 0))


def compression_spring(center: Tuple[float, float, float], height: float = 11.0, od: float = 6.0) -> cq.Shape:
    x, y, z = center
    return cq.Solid.makeCylinder(od / 2.0, height, cq.Vector(x, y, z - height / 2.0), cq.Vector(0, 0, 1))


def servo_shape(center: Tuple[float, float, float]) -> cq.Shape:
    x, y, z = center
    lx, ly, lz = 22.8, 12.2, 28.5
    body = cq.Workplane("XY").box(lx, ly, lz).edges().fillet(1.2).translate((x, y, z)).val()
    horn = cq.Solid.makeCylinder(8.0, 2.0, cq.Vector(x, y, z + lz / 2), cq.Vector(0, 0, 1))
    return body.fuse(horn)


def paw_shape(p: Tuple[float, float, float], front: bool) -> cq.Shape:
    scale = 0.92 if front else 0.86
    return v3e.base.pebble_foot(p[0], p[1], scale=scale).translate((0, 0, -2.0))


def gaiter_shape(p: Tuple[float, float, float]) -> cq.Shape:
    x, y, z = p
    ys = 1.0 if y >= 0 else -1.0
    bottom = cq.Workplane("XY", origin=(x, y, max(7.0, z + 4.0))).ellipse(5.0, 4.2).val()
    mid = cq.Workplane("XY", origin=(x - math.copysign(1.0, x), y - ys * 1.5, max(12.0, z + 10.0))).ellipse(6.5, 5.5).val()
    top = cq.Workplane("XY", origin=(x - math.copysign(2.0, x), y - ys * 3.0, max(18.0, z + 16.0))).ellipse(9.5, 8.0).val()
    return cq.Solid.makeLoft([bottom, mid, top], ruled=False)


def build_leg(front: bool, left: bool, theta: float, splay: float, tag: str) -> Dict[str, cq.Shape]:
    pts = fourbar_points(theta, front, left, splay)
    A, B, C, D, P = (pts[k] for k in ("A", "B", "C", "D", "P"))
    ys = 1.0 if left else -1.0

    shaft = cq.Solid.makeCylinder(CRANK_SHAFT_D / 2.0, 22.0, cq.Vector(A[0], A[1] - ys * 11.0, A[2]), cq.Vector(0, ys, 0))
    bearing1 = bearing_shape((A[0], A[1] - ys * 6.0, A[2]))
    bearing2 = bearing_shape((A[0], A[1] + ys * 6.0, A[2]))
    crank_gear = gear_shape((A[0], A[1] - ys * 1.0, A[2]), CRANK_GEAR_PD)
    motor_center = (A[0], A[1] - ys * 22.0, A[2] + 4.5)
    motor = n20_motor_shape(motor_center, left)
    pinion = gear_shape((A[0] + 7.5, A[1] - ys * 7.0, A[2] + 4.5), PINION_PD)

    links = [
        cylinder_between(A, B, 2.0),
        cylinder_between(B, C, 1.8),
        cylinder_between(D, C, 1.8),
        cylinder_between(C, P, 1.8),
    ]
    crank_disk = cq.Solid.makeCylinder(6.0, 3.0, cq.Vector(A[0], A[1] - ys * 1.5, A[2]), cq.Vector(0, ys, 0))
    cassette = cq.Workplane("XY").box(42.0, 16.0, 30.0).translate((A[0] + (8.0 if front else -8.0), A[1], A[2] + 3.0)).val()
    spring1 = compression_spring((A[0] - 9.0, A[1], A[2] + 15.0))
    spring2 = compression_spring((A[0] + 9.0, A[1], A[2] + 15.0))
    hardstop = cq.Workplane("XY").box(32.0, 12.0, 3.0).translate((A[0], A[1], A[2] + 23.0)).val()
    paw = paw_shape(P, front)
    gaiter = gaiter_shape(P)

    return {
        f"motor_{tag}": motor,
        f"pinion_{tag}": pinion,
        f"crank_gear_{tag}": crank_gear,
        f"shaft_{tag}": shaft,
        f"bearing1_{tag}": bearing1,
        f"bearing2_{tag}": bearing2,
        f"crank_{tag}": crank_disk,
        f"link_ab_{tag}": links[0],
        f"link_bc_{tag}": links[1],
        f"link_dc_{tag}": links[2],
        f"ankle_link_{tag}": links[3],
        f"cassette_{tag}": cassette,
        f"overload_spring1_{tag}": spring1,
        f"overload_spring2_{tag}": spring2,
        f"hardstop_{tag}": hardstop,
        f"gaiter_{tag}": gaiter,
        f"paw_{tag}": paw,
    }


def add_splay_system(asm: cq.Assembly, front: bool) -> None:
    x = 29.0 if front else -32.0
    label = "front" if front else "rear"
    asm.add(servo_shape((x, 0.0, 48.0)), name=f"splay_servo_{label}", color=cq.Color(0.20, 0.25, 0.33))
    bell_l = (x, 8.0, 63.0)
    bell_r = (x, -8.0, 63.0)
    asm.add(cylinder_between(bell_l, bell_r, 2.4), name=f"bellcrank_{label}", color=cq.Color(0.70, 0.55, 0.18))
    tx = LEG_ROOT_X if front else -LEG_ROOT_X
    target_l = (tx, +LEG_ROOT_Y, 48.0)
    target_r = (tx, -LEG_ROOT_Y, 48.0)
    saver_l_end = (x, +16.0, 61.0)
    saver_r_end = (x, -16.0, 61.0)
    asm.add(cylinder_between(bell_l, saver_l_end, 1.7), name=f"saver_L_{label}", color=cq.Color(0.78, 0.30, 0.18))
    asm.add(cylinder_between(bell_r, saver_r_end, 1.7), name=f"saver_R_{label}", color=cq.Color(0.78, 0.30, 0.18))
    asm.add(cylinder_between(saver_l_end, target_l, 1.4), name=f"pushrod_L_{label}", color=cq.Color(0.65, 0.68, 0.72))
    asm.add(cylinder_between(saver_r_end, target_r, 1.4), name=f"pushrod_R_{label}", color=cq.Color(0.65, 0.68, 0.72))


def build_pose(name: str, include_shell: bool = True, cutaway: bool = False) -> cq.Assembly:
    pose = POSES[name]
    shell, outer = v3e.base.make_body_shell()
    visor = v3e.base.make_visor(outer)
    a_l, tip_l = v3e.base.antenna(+1.0)
    a_r, tip_r = v3e.base.antenna(-1.0)

    asm = cq.Assembly(name=f"Pebble_V3F_{name}")
    if include_shell:
        if cutaway:
            cutter = cq.Workplane("YZ").box(200, 200, 200).translate((50, 0, 80)).val()
            shell = shell.cut(cutter)
        asm.add(shell, name="shell", color=cq.Color(0.93, 0.90, 0.84, 0.45 if cutaway else 1.0))
        asm.add(visor, name="visor", color=cq.Color(0.03, 0.04, 0.05))
        asm.add(a_l, name="antenna_left", color=cq.Color(0.1, 0.1, 0.11))
        asm.add(a_r, name="antenna_right", color=cq.Color(0.1, 0.1, 0.11))
        asm.add(tip_l, name="tip_left", color=cq.Color(1.0, 0.75, 0.34))
        asm.add(tip_r, name="tip_right", color=cq.Color(1.0, 0.75, 0.34))

    for tag, front, left in (("FL", True, True), ("FR", True, False), ("RL", False, True), ("RR", False, False)):
        splay = float(pose["front_splay"] if front else pose["rear_splay"])
        parts = build_leg(front, left, float(pose[tag]), splay, tag)
        for pname, shape in parts.items():
            if pname.startswith("motor_"):
                color = cq.Color(0.35, 0.39, 0.45)
            elif "spring" in pname or "saver" in pname:
                color = cq.Color(0.82, 0.32, 0.18)
            elif pname.startswith("paw_") or pname.startswith("gaiter_"):
                color = cq.Color(0.84, 0.82, 0.78)
            elif "gear" in pname or "crank_" in pname:
                color = cq.Color(0.72, 0.60, 0.22)
            elif "bearing" in pname or "shaft" in pname:
                color = cq.Color(0.45, 0.48, 0.52)
            else:
                color = cq.Color(0.60, 0.63, 0.67)
            asm.add(shape, name=pname, color=color)

    add_splay_system(asm, True)
    add_splay_system(asm, False)
    ring_outer = cq.Workplane("XY", origin=(0, 0, 34)).ellipse(67, 72).extrude(5).val()
    ring_inner = cq.Workplane("XY", origin=(0, 0, 33.5)).ellipse(58, 63).extrude(6).val()
    asm.add(ring_outer.cut(ring_inner), name="structural_belly_ring", color=cq.Color(0.20, 0.25, 0.22))
    return asm


def build_swept_volume() -> cq.Assembly:
    asm = cq.Assembly(name="Pebble_V3F_Swept_Volumes")
    for tag, front, left in (("FL", True, True), ("FR", True, False), ("RL", False, True), ("RR", False, False)):
        pts = []
        for theta in np.linspace(THETA_MIN, THETA_MAX, 13):
            for splay in np.linspace(-SPLAY_MAX, SPLAY_MAX, 5):
                pts.append(fourbar_points(float(theta), front, left, float(splay))["P"])
        arr = np.asarray(pts)
        mins, maxs = arr.min(axis=0), arr.max(axis=0)
        size = maxs - mins + np.array([6.0, 6.0, 6.0])
        center = (mins + maxs) / 2.0
        vol = cq.Workplane("XY").box(float(size[0]), float(size[1]), float(size[2])).translate(tuple(center)).val()
        asm.add(vol, name=f"ankle_sweep_{tag}", color=cq.Color(0.95, 0.25, 0.20, 0.30))
    return asm


def save(asm: cq.Assembly, filename: str) -> None:
    asm.save(str(OUT / filename), exportType="STEP", mode="default")


def main() -> None:
    save(build_pose("neutral"), "Pebble_V3F_Product_Neutral.step")
    save(build_pose("neutral", cutaway=True), "Pebble_V3F_Engineering_Cutaway.step")
    save(build_swept_volume(), "Pebble_V3F_Swept_Volumes.step")
    save(build_pose("happy_wide"), "Pebble_V3F_Happy_Wide.step")
    save(build_pose("play_bow"), "Pebble_V3F_Play_Bow.step")
    save(build_pose("curious_left"), "Pebble_V3F_Curious_Left.step")
    save(build_pose("sleepy_compact"), "Pebble_V3F_Inward_Compact.step")
    print(f"Generated V3-F review STEP files in {OUT}")


if __name__ == "__main__":
    main()
