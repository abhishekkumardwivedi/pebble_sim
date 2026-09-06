from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Dict, List, Tuple

import cadquery as cq
import numpy as np

OUT = Path(os.environ.get("PEBBLE_CAD_OUT", Path(__file__).resolve().parent / "generated"))
OUT.mkdir(parents=True, exist_ok=True)

# -----------------------------------------------------------------------------
# Pebble V3 UX geometry baseline — revision D
# Reference priorities:
#   1. 165 x 150 mm footprint-class body proportions
#   2. 170 mm overall height INCLUDING antenna tips
#   3. squat bubble/pebble shell (~130 mm shell height above 8 mm belly plane)
#   4. face/visor reaches close to forehead and reads as a smaller pebble in the shell
#   5. small, high-domed pebble/paw feet, mostly tucked under shell
# -----------------------------------------------------------------------------
BODY_TARGET_WIDTH_Y = 165.0
BODY_TARGET_DEPTH_X = 150.0
BELLY_Z = 8.0
BODY_TOP_Z = 138.0
OVERALL_TOP_Z = 170.0
SHELL_WALL = 2.4

# Horizontal section controls: (z, front_x, rear_x, half_width_y, exponent)
# Revision D preserves the 165 x 150 x 130 mm envelope but redistributes only a few millimetres of
# upper-body volume downward. This gives a softer baby/pebble proportion without changing packaging.
BODY_SECTIONS = [
    (8.0,   46.0, 44.5, 52.5, 2.03),
    (15.0,  56.0, 54.0, 62.5, 2.02),
    (28.0,  66.5, 64.5, 73.5, 2.01),
    (45.0,  73.0, 71.0, 81.0, 2.00),
    (61.0,  75.0, 74.0, 82.5, 2.00),
    (77.0,  73.5, 72.5, 80.5, 2.00),
    (92.0,  68.5, 67.5, 74.0, 2.00),
    (106.0, 61.0, 60.0, 65.5, 2.00),
    (119.0, 51.0, 50.0, 54.0, 2.00),
    (129.0, 37.5, 36.5, 40.0, 2.00),
    (136.0, 15.5, 14.5, 18.0, 2.00),
    (138.0,  3.0,  3.0,  4.0, 2.00),
]

# Visor ratios measured visually from the supplied concept.
VISOR_WIDTH_Y = 106.0
VISOR_HEIGHT_Z = 74.0
VISOR_CENTER_Z = 97.0
VISOR_TOP_H = 37.0
VISOR_BOTTOM_H = 37.0
VISOR_MASK_X = 5.0
VISOR_EXTRUDE_X = 100.0
VISOR_INSET_X = -0.45  # negative => lens sits slightly proud of shell

# Cosmetic foot envelope. Actual leg endpoint remains the mechanical center.
FOOT_MAX_X = 26.0
FOOT_MAX_Y = 21.0
FOOT_HEIGHT_Z = 17.5

# -----------------------------------------------------------------------------
# Compact expressive four-bar baseline retained from V3-A.
# -----------------------------------------------------------------------------
CRANK_R = 4.0
FRAME_D = 17.0
COUPLER_L = 34.0
ROCKER_Q = 22.0
FOOT_EXT = 8.0
LEG_ROOT_Z = 35.5
FRONT_A_X = 18.0
LEG_Y = 44.0
NEUTRAL_THETA_DEG = 134.0
LINK_RADIUS = 2.2
PIVOT_RADIUS = 3.0


def superellipse_wire(z: float, front: float, back: float, half_y: float,
                      exponent: float, count: int = 64) -> cq.Wire:
    pts: List[cq.Vector] = []
    p = 2.0 / exponent
    for i in range(count):
        t = 2.0 * math.pi * i / count
        c, s = math.cos(t), math.sin(t)
        ax = front if c >= 0.0 else back
        x = 0.0 if abs(c) < 1e-12 else ax * math.copysign(abs(c) ** p, c)
        y = 0.0 if abs(s) < 1e-12 else half_y * math.copysign(abs(s) ** p, s)
        pts.append(cq.Vector(x, y, z))
    edge = cq.Edge.makeSpline(pts, periodic=True)
    return cq.Wire.assembleEdges([edge])


def pebble_face_wire(
    x: float,
    center_z: float,
    half_w: float,
    top_h: float,
    bottom_h: float,
    scale: float = 1.0,
) -> cq.Wire:
    """Organic companion-face profile in the global Y-Z plane.

    Revision D intentionally avoids a rounded-rectangle read. The forehead is
    narrower, the lower-middle is the widest part, and the chin edge bows upward
    into the shell. This approximates a small glossy pebble nested in the larger
    white pebble body.
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
        (+0.34 * w, -0.92 * bh),
        (0.00, -0.88 * bh),
        (-0.34 * w, -0.92 * bh),
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


def prism_from_wire_x(wire: cq.Wire, length_x: float) -> cq.Shape:
    return cq.Solid.extrudeLinear(wire, [], cq.Vector(length_x, 0.0, 0.0))


def make_body_shell() -> Tuple[cq.Shape, cq.Shape]:
    outer_wires = [superellipse_wire(*row) for row in BODY_SECTIONS]
    outer = cq.Solid.makeLoft(outer_wires, ruled=False)

    inner_wires = []
    for z, f, b, y, n in BODY_SECTIONS[1:-2]:
        inner_wires.append(
            superellipse_wire(
                z + 1.0,
                max(8.0, f - SHELL_WALL),
                max(8.0, b - SHELL_WALL),
                max(8.0, y - SHELL_WALL),
                n,
            )
        )
    inner = cq.Solid.makeLoft(inner_wires, ruled=False)
    shell = outer.cut(inner)

    # Organic face recess slightly larger than the insert.
    recess_wire = pebble_face_wire(
        VISOR_MASK_X,
        VISOR_CENTER_Z,
        VISOR_WIDTH_Y / 2.0,
        VISOR_TOP_H,
        VISOR_BOTTOM_H,
        scale=1.045,
    )
    recess = prism_from_wire_x(recess_wire, VISOR_EXTRUDE_X)
    shell = shell.cut(recess)

    return shell, outer


def make_visor(outer: cq.Shape) -> cq.Shape:
    # Keep the perimeter conformal to the shell, then move the entire lens only
    # 0.45 mm proud. The body curvature itself supplies most of the convexity;
    # this small proud condition gives the glossy lens a separate optical read
    # without making it look like a thick plastic badge.
    wire = pebble_face_wire(
        VISOR_MASK_X + 0.7,
        VISOR_CENTER_Z,
        VISOR_WIDTH_Y / 2.0,
        VISOR_TOP_H,
        VISOR_BOTTOM_H,
        scale=1.0,
    )
    mask = prism_from_wire_x(wire, VISOR_EXTRUDE_X - 0.7)
    return outer.intersect(mask).translate((-VISOR_INSET_X, 0.0, 0.0))


def ellipse_wire_at(x: float, y: float, z: float, rx: float, ry: float) -> cq.Wire:
    return cq.Workplane("XY", origin=(x, y, z)).ellipse(rx, ry).val()


def pebble_foot(x: float, y: float, scale: float = 1.0) -> cq.Shape:
    """Small, high-domed paw/pebble foot mostly concealed by the belly.

    The visual footprint is deliberately smaller than V3-C while the dome stays
    tall. Upper sections drift inward so the shell appears to grow around the paw.
    """
    y_in = -math.copysign(2.6, y)
    x_in = -math.copysign(1.4, x)

    sections = [
        (0.0,   7.5 * scale,  6.2 * scale, 0.0, 0.0),
        (1.8,  10.8 * scale,  8.6 * scale, 0.0, 0.0),
        (5.5,  13.0 * scale, 10.5 * scale, 0.0, 0.0),
        (10.5, 12.4 * scale,  9.7 * scale, 0.30 * x_in, 0.30 * y_in),
        (14.3,  9.5 * scale,  7.1 * scale, 0.68 * x_in, 0.68 * y_in),
        (17.2,  3.2 * scale,  2.5 * scale, 1.00 * x_in, 1.00 * y_in),
    ]
    wires = [
        ellipse_wire_at(x + dx, y + dy, z, rx, ry)
        for z, rx, ry, dx, dy in sections
    ]
    return cq.Solid.makeLoft(wires, ruled=False)


def cylinder_between(
    p1: Tuple[float, float, float],
    p2: Tuple[float, float, float],
    r: float,
) -> cq.Shape:
    a = cq.Vector(*p1)
    b = cq.Vector(*p2)
    v = b - a
    L = v.Length
    if L <= 1e-9:
        return cq.Solid.makeSphere(r, a)
    return cq.Solid.makeCylinder(r, L, a, v.normalized())


def antenna(sign: float) -> Tuple[cq.Shape, cq.Shape]:
    # Slightly more outward sweep and larger luminous bulb to make the antennae
    # read as curious ears/eyebrows rather than engineering stalks.
    pts = [
        (0.0, sign * 20.0, 132.5),
        (0.1, sign * 21.5, 139.5),
        (0.6, sign * 25.5, 148.0),
        (1.4, sign * 30.5, 155.5),
        (2.4, sign * 36.5, 162.5),
        (3.0, sign * 39.0, 164.0),
    ]
    rod = None
    for a, b in zip(pts[:-1], pts[1:]):
        seg = cylinder_between(a, b, 1.65)
        rod = seg if rod is None else rod.fuse(seg)
    tip = cq.Solid.makeSphere(6.0, cq.Vector(*pts[-1]))
    return rod, tip


def fourbar(theta_deg: float, front: bool, left: bool) -> Dict[str, object]:
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

    x_sign = 1.0 if front else -1.0
    y = LEG_Y if left else -LEG_Y
    ax = x_sign * FRONT_A_X

    def map2(q: np.ndarray) -> Tuple[float, float, float]:
        return (ax + x_sign * float(q[0]), y, LEG_ROOT_Z + float(q[1]))

    A = map2(A2)
    B = map2(B2)
    C = map2(C2)
    D = map2(D2)
    P = map2(P2)

    links = [
        cylinder_between(A, B, LINK_RADIUS),
        cylinder_between(B, C, LINK_RADIUS),
        cylinder_between(D, C, LINK_RADIUS),
        cylinder_between(C, P, LINK_RADIUS),
    ]
    pivots = [cq.Solid.makeSphere(PIVOT_RADIUS, cq.Vector(*q)) for q in (A, B, C, D)]

    motor_center_y = y - math.copysign(15.0, y)
    motor = (
        cq.Workplane("XY")
        .box(25.0, 30.0, 12.0)
        .edges()
        .fillet(1.5)
        .translate((A[0], motor_center_y, A[2]))
        .val()
    )

    return {
        "A": A,
        "B": B,
        "C": C,
        "D": D,
        "P": P,
        "links": links,
        "pivots": pivots,
        "motor": motor,
    }


def trajectory(front: bool = True, left: bool = True, samples: int = 181) -> np.ndarray:
    pts = []
    for th in np.linspace(0.0, 360.0, samples):
        pts.append(fourbar(float(th), front, left)["P"])
    return np.array(pts, dtype=float)


def build_assembly() -> Tuple[cq.Assembly, Dict[str, cq.Shape], np.ndarray]:
    shell, outer_ref = make_body_shell()
    visor = make_visor(outer_ref)
    aL, tipL = antenna(+1.0)
    aR, tipR = antenna(-1.0)

    # Precompute neutral legs, then cut only shallow cosmetic foot-reveal pockets.
    # This is not yet the final swept-motion opening; it simply prevents the shell
    # from visually swallowing the paw geometry in the UX review.
    leg_cache: Dict[str, Dict[str, object]] = {}
    for front in (True, False):
        for left in (True, False):
            tag = ("F" if front else "R") + ("L" if left else "R")
            leg_cache[tag] = fourbar(NEUTRAL_THETA_DEG, front, left)
            P = leg_cache[tag]["P"]
            pocket = pebble_foot(P[0], P[1], scale=1.02)
            reveal_band = (
                cq.Workplane("XY")
                .box(34.0, 29.0, 9.0)
                .translate((P[0], P[1], 10.5))
                .val()
            )
            shell = shell.cut(pocket.intersect(reveal_band))

    components: Dict[str, cq.Shape] = {
        "shell": shell,
        "visor": visor,
        "antenna_left": aL,
        "antenna_right": aR,
        "tip_left": tipL,
        "tip_right": tipR,
    }

    asm = cq.Assembly(name="Pebble_V3D_UX_Shell_Leg_Concept")
    asm.add(shell, name="shell", color=cq.Color(0.93, 0.90, 0.84))
    asm.add(visor, name="visor", color=cq.Color(0.035, 0.04, 0.05))
    asm.add(aL, name="antenna_left", color=cq.Color(0.10, 0.10, 0.11))
    asm.add(aR, name="antenna_right", color=cq.Color(0.10, 0.10, 0.11))
    asm.add(tipL, name="antenna_tip_left", color=cq.Color(1.0, 0.75, 0.34))
    asm.add(tipR, name="antenna_tip_right", color=cq.Color(1.0, 0.75, 0.34))

    for front in (True, False):
        for left in (True, False):
            tag = ("F" if front else "R") + ("L" if left else "R")
            leg = leg_cache[tag]
            P = leg["P"]

            foot = pebble_foot(P[0], P[1], scale=(0.92 if front else 0.84))
            components[f"foot_{tag}"] = foot
            asm.add(foot, name=f"foot_{tag}", color=cq.Color(0.84, 0.82, 0.78))

            components[f"motor_{tag}"] = leg["motor"]
            asm.add(leg["motor"], name=f"motor_{tag}", color=cq.Color(0.42, 0.45, 0.48))

            for i, link in enumerate(leg["links"]):
                components[f"link_{tag}_{i}"] = link
                asm.add(link, name=f"link_{tag}_{i}", color=cq.Color(0.62, 0.64, 0.66))
            for i, pivot in enumerate(leg["pivots"]):
                components[f"pivot_{tag}_{i}"] = pivot
                asm.add(pivot, name=f"pivot_{tag}_{i}", color=cq.Color(0.30, 0.32, 0.34))

    traj = trajectory(True, True)
    return asm, components, traj


def main() -> None:
    asm, components, traj = build_assembly()

    step_path = OUT / "Pebble_V3D_UX_Shell_Leg_Concept.step"
    asm.save(str(step_path), exportType="STEP", mode="default")

    ext = cq.Assembly(name="Pebble_V3D_UX_Exterior")
    for name in (
        "shell",
        "visor",
        "antenna_left",
        "antenna_right",
        "tip_left",
        "tip_right",
        "foot_FL",
        "foot_FR",
        "foot_RL",
        "foot_RR",
    ):
        shape = components[name]
        if name == "shell":
            color = cq.Color(0.93, 0.90, 0.84)
        elif name == "visor":
            color = cq.Color(0.035, 0.04, 0.05)
        elif name.startswith("tip"):
            color = cq.Color(1.0, 0.75, 0.34)
        elif name.startswith("antenna"):
            color = cq.Color(0.10, 0.10, 0.11)
        else:
            color = cq.Color(0.84, 0.82, 0.78)
        ext.add(shape, name=name, color=color)

    ext_path = OUT / "Pebble_V3D_UX_Exterior.step"
    ext.save(str(ext_path), exportType="STEP", mode="default")

    np.savetxt(
        OUT / "Pebble_V3D_FrontLeft_FootTrajectory.csv",
        traj,
        delimiter=",",
        header="x_mm,y_mm,z_mm",
        comments="",
    )

    bb = components["shell"].BoundingBox()
    vb = components["visor"].BoundingBox()
    tspan = np.ptp(traj, axis=0)

    print(f"STEP: {step_path}")
    print(f"Exterior STEP: {ext_path}")
    print(f"Shell bounding box: X={bb.xlen:.2f} mm, Y={bb.ylen:.2f} mm, Z={bb.zlen:.2f} mm")
    print(f"Visor bounding box: Y={vb.ylen:.2f} mm, Z={vb.zlen:.2f} mm")
    print(f"Visor top/bottom: {vb.zmax:.2f} / {vb.zmin:.2f} mm")
    print(f"Foot trajectory span: X={tspan[0]:.2f} mm, Z={tspan[2]:.2f} mm")
    print(f"Overall nominal top with antenna tips: {OVERALL_TOP_Z:.1f} mm")


if __name__ == "__main__":
    main()
