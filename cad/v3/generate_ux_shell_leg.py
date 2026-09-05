from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Dict, List, Tuple

import cadquery as cq
import numpy as np

OUT = Path(os.environ.get('PEBBLE_CAD_OUT', Path(__file__).resolve().parent / 'generated'))
OUT.mkdir(parents=True, exist_ok=True)

# -----------------------------------------------------------------------------
# UX reference envelope (mm)
# -----------------------------------------------------------------------------
BODY_TARGET_WIDTH_Y = 165.0
BODY_TARGET_DEPTH_X = 150.0
BELLY_Z = 8.0
BODY_TOP_Z = 151.0
OVERALL_TOP_Z = 170.0
SHELL_WALL = 2.4

# Smooth horizontal section controls: (z, front_x, rear_x, half_width_y, exponent)
# Deliberately broad through the lower-middle, slightly fuller in front, narrowed crown.
BODY_SECTIONS = [
    (8.0, 42.0, 40.0, 49.0, 2.75),
    (18.0, 59.0, 56.0, 69.0, 2.50),
    (40.0, 71.0, 68.0, 80.0, 2.35),
    (70.0, 76.0, 73.0, 82.5, 2.25),
    (96.0, 74.0, 71.0, 80.5, 2.20),
    (120.0, 66.0, 63.0, 72.5, 2.15),
    (138.0, 48.0, 46.0, 56.0, 2.08),
    (148.0, 18.0, 18.0, 22.0, 2.00),
    (151.0, 3.0, 3.0, 4.0, 2.00),
]

# Visor target from concept; actual insert is a real CAD part, not a texture.
VISOR_WIDTH_Y = 106.0
VISOR_HEIGHT_Z = 60.0
VISOR_CENTER_Z = 108.0
VISOR_RECESS_X0 = 64.0
VISOR_INSERT_X0 = 69.0

# Four feet: body X front/rear, body Y left/right.
FOOT_X = 40.0
FOOT_Y = 44.0
FOOT_SIZE = (30.0, 24.0, 12.0)  # x, y, z

# -----------------------------------------------------------------------------
# Compact expressive four-bar leg baseline
# Target foot path is about 22 mm fore/aft x 16 mm vertical.
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
                      exponent: float, count: int = 48) -> cq.Wire:
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


def make_body_shell() -> Tuple[cq.Shape, cq.Shape]:
    outer_wires = [superellipse_wire(*row) for row in BODY_SECTIONS]
    outer = cq.Solid.makeLoft(outer_wires, ruled=False)

    inner_wires = []
    # Leave a closed top/belly skin; inner cavity starts/ends inside the outer loft.
    for z, f, b, y, n in BODY_SECTIONS[1:-2]:
        inner_wires.append(
            superellipse_wire(
                z + 1.2,
                max(8.0, f - SHELL_WALL),
                max(8.0, b - SHELL_WALL),
                max(8.0, y - SHELL_WALL),
                n,
            )
        )
    inner = cq.Solid.makeLoft(inner_wires, ruled=False)
    shell = outer.cut(inner)

    # Underside neck openings are deferred until the leg swept-volume pass.
    # At this UX gate the shell silhouette is kept intact.

    # Real face opening.  The black insert is later derived from the body itself,
    # so its visible face follows the Pebble curvature instead of floating as a plate.
    recess = (
        cq.Workplane('YZ', origin=(48.0, 0.0, VISOR_CENTER_Z))
        .rect(VISOR_WIDTH_Y + 5.0, VISOR_HEIGHT_Z + 5.0)
        .extrude(44.0)
        .edges('|X').fillet(15.5)
        .val()
    )
    shell = shell.cut(recess)

    return shell, outer


def make_visor(outer: cq.Shape) -> cq.Shape:
    # Conformal visor: intersect the original body with a rounded YZ window, then
    # move it slightly inward to create a realistic shallow recess/gap line.
    mask = (
        cq.Workplane('YZ', origin=(49.0, 0.0, VISOR_CENTER_Z))
        .rect(VISOR_WIDTH_Y, VISOR_HEIGHT_Z)
        .extrude(43.0)
        .edges('|X').fillet(14.5)
        .val()
    )
    return outer.intersect(mask).translate((-0.9, 0.0, 0.0))


def rounded_foot(x: float, y: float, z: float = 6.0) -> cq.Shape:
    lx, ly, lz = FOOT_SIZE
    return (
        cq.Workplane('XY')
        .ellipse(lx / 2.0, ly / 2.0)
        .extrude(lz)
        .edges().fillet(4.5)
        .translate((x, y, z - lz / 2.0))
        .val()
    )


def cylinder_between(p1: Tuple[float, float, float], p2: Tuple[float, float, float], r: float) -> cq.Shape:
    a = cq.Vector(*p1)
    b = cq.Vector(*p2)
    v = b - a
    L = v.Length
    if L <= 1e-9:
        return cq.Solid.makeSphere(r, a)
    return cq.Solid.makeCylinder(r, L, a, v.normalized())


def antenna(sign: float) -> Tuple[cq.Shape, cq.Shape]:
    pts = [
        (0.0, sign * 23.0, 144.0),
        (0.5, sign * 24.5, 151.0),
        (1.0, sign * 27.5, 158.0),
        (1.0, sign * 31.0, 165.5),
    ]
    rod = None
    for a, b in zip(pts[:-1], pts[1:]):
        seg = cylinder_between(a, b, 1.7)
        rod = seg if rod is None else rod.fuse(seg)
    tip = cq.Solid.makeSphere(4.2, cq.Vector(*pts[-1]))
    return rod, tip


def fourbar(theta_deg: float, front: bool, left: bool) -> Dict[str, object]:
    th = math.radians(theta_deg)
    # Compute front-left canonical linkage in the X-Z plane, then mirror X/Y.
    A2 = np.array([0.0, 0.0])
    D2 = np.array([FRAME_D, 0.0])
    B2 = A2 + CRANK_R * np.array([math.cos(th), math.sin(th)])

    v = D2 - B2
    dist = float(np.linalg.norm(v))
    a = (COUPLER_L**2 - ROCKER_Q**2 + dist**2) / (2.0 * dist)
    h = math.sqrt(max(0.0, COUPLER_L**2 - a**2))
    p0 = B2 + a * v / dist
    perp = np.array([-v[1], v[0]]) / dist
    C2 = p0 - h * perp  # lower branch
    u = (C2 - D2) / ROCKER_Q
    P2 = C2 + FOOT_EXT * u

    # Canonical front leg root. Mirror front/rear around X=0 and left/right around Y=0.
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

    # Motor envelope sits inward from side wall, axis aligned along Y.
    motor_center_y = y - math.copysign(15.0, y)
    motor = (
        cq.Workplane('XY')
        .box(25.0, 30.0, 12.0)
        .edges().fillet(1.5)
        .translate((A[0], motor_center_y, A[2]))
        .val()
    )

    return {
        'A': A, 'B': B, 'C': C, 'D': D, 'P': P,
        'links': links, 'pivots': pivots, 'motor': motor,
    }


def trajectory(front: bool = True, left: bool = True, samples: int = 181) -> np.ndarray:
    pts = []
    for th in np.linspace(0.0, 360.0, samples):
        pts.append(fourbar(float(th), front, left)['P'])
    return np.array(pts, dtype=float)


def build_assembly() -> Tuple[cq.Assembly, Dict[str, cq.Shape], np.ndarray]:
    shell, outer_ref = make_body_shell()
    visor = make_visor(outer_ref)
    aL, tipL = antenna(+1.0)
    aR, tipR = antenna(-1.0)

    components: Dict[str, cq.Shape] = {
        'shell': shell,
        'visor': visor,
        'antenna_left': aL,
        'antenna_right': aR,
        'tip_left': tipL,
        'tip_right': tipR,
    }

    asm = cq.Assembly(name='Pebble_V3_UX_Shell_Leg_Concept')
    asm.add(shell, name='shell', color=cq.Color(0.93, 0.90, 0.84))
    asm.add(visor, name='visor', color=cq.Color(0.035, 0.04, 0.05))
    asm.add(aL, name='antenna_left', color=cq.Color(0.10, 0.10, 0.11))
    asm.add(aR, name='antenna_right', color=cq.Color(0.10, 0.10, 0.11))
    asm.add(tipL, name='antenna_tip_left', color=cq.Color(1.0, 0.65, 0.20))
    asm.add(tipR, name='antenna_tip_right', color=cq.Color(1.0, 0.65, 0.20))

    for front, sx in ((True, +1.0), (False, -1.0)):
        for left, sy in ((True, +1.0), (False, -1.0)):
            tag = ('F' if front else 'R') + ('L' if left else 'R')
            leg = fourbar(NEUTRAL_THETA_DEG, front, left)
            P = leg['P']
            # Use target cosmetic location centered on the actual neutral foot point.
            foot = rounded_foot(P[0], P[1], 6.0)
            components[f'foot_{tag}'] = foot
            asm.add(foot, name=f'foot_{tag}', color=cq.Color(0.84, 0.82, 0.78))
            asm.add(leg['motor'], name=f'motor_{tag}', color=cq.Color(0.42, 0.45, 0.48))
            components[f'motor_{tag}'] = leg['motor']
            for i, link in enumerate(leg['links']):
                asm.add(link, name=f'link_{tag}_{i}', color=cq.Color(0.62, 0.64, 0.66))
                components[f'link_{tag}_{i}'] = link
            for i, pivot in enumerate(leg['pivots']):
                asm.add(pivot, name=f'pivot_{tag}_{i}', color=cq.Color(0.30, 0.32, 0.34))
                components[f'pivot_{tag}_{i}'] = pivot

    traj = trajectory(True, True)
    return asm, components, traj


def main() -> None:
    asm, components, traj = build_assembly()
    step_path = OUT / 'Pebble_V3_UX_Shell_Leg_Concept.step'
    asm.save(str(step_path), exportType='STEP', mode='default')

    # Exterior-only STEP is useful for rapid shape review in FreeCAD/viewer.
    ext = cq.Assembly(name='Pebble_V3_UX_Exterior')
    for name in ('shell', 'visor', 'antenna_left', 'antenna_right', 'tip_left', 'tip_right',
                 'foot_FL', 'foot_FR', 'foot_RL', 'foot_RR'):
        shape = components[name]
        if name == 'shell': color = cq.Color(0.93, 0.90, 0.84)
        elif name == 'visor': color = cq.Color(0.035, 0.04, 0.05)
        elif name.startswith('tip'): color = cq.Color(1.0, 0.65, 0.20)
        elif name.startswith('antenna'): color = cq.Color(0.10, 0.10, 0.11)
        else: color = cq.Color(0.84, 0.82, 0.78)
        ext.add(shape, name=name, color=color)
    ext_path = OUT / 'Pebble_V3_UX_Exterior.step'
    ext.save(str(ext_path), exportType='STEP', mode='default')

    np.savetxt(
        OUT / 'Pebble_V3_FrontLeft_FootTrajectory.csv', traj,
        delimiter=',', header='x_mm,y_mm,z_mm', comments=''
    )

    bb = components['shell'].BoundingBox()
    tspan = np.ptp(traj, axis=0)
    print(f'STEP: {step_path}')
    print(f'Exterior STEP: {ext_path}')
    print(f'Shell bounding box: X={bb.xlen:.2f} mm, Y={bb.ylen:.2f} mm, Z={bb.zlen:.2f} mm')
    print(f'Foot trajectory span: X={tspan[0]:.2f} mm, Z={tspan[2]:.2f} mm')
    print(f'Foot trajectory min/max Z: {traj[:,2].min():.2f} / {traj[:,2].max():.2f} mm')
    print(f'Overall nominal top with antenna tips: {OVERALL_TOP_Z:.1f} mm')

if __name__ == '__main__':
    main()
