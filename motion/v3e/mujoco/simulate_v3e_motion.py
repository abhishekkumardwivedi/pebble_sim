from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "shared"))
from kinematics import LEG_ORDER, build_demo_timeline, load_spec  # noqa: E402

COL = {
    "link": np.array([0.62, 0.64, 0.66, 1.0], dtype=np.float32),
    "motor": np.array([0.35, 0.37, 0.40, 1.0], dtype=np.float32),
    "shaft": np.array([0.78, 0.78, 0.78, 1.0], dtype=np.float32),
    "paw": np.array([0.84, 0.82, 0.78, 1.0], dtype=np.float32),
    "servo": np.array([0.16, 0.18, 0.20, 1.0], dtype=np.float32),
    "horn": np.array([0.82, 0.82, 0.80, 1.0], dtype=np.float32),
}


def quat_from_roll_pitch(roll_deg: float, pitch_deg: float):
    r = math.radians(roll_deg) / 2.0
    p = math.radians(pitch_deg) / 2.0
    qx = np.array([math.cos(r), math.sin(r), 0.0, 0.0])
    qy = np.array([math.cos(p), 0.0, math.sin(p), 0.0])
    w1, x1, y1, z1 = qy
    w2, x2, y2, z2 = qx
    return np.array([
        w1*w2-x1*x2-y1*y2-z1*z2,
        w1*x2+x1*w2+y1*z2-z1*y2,
        w1*y2-x1*z2+y1*w2+z1*x2,
        w1*z2+x1*y2-y1*x2+z1*w2,
    ])


def transform_point_mm(p, body):
    x, y, z = map(float, p)
    r = math.radians(body["roll_deg"])
    pch = math.radians(body["pitch_deg"])
    y, z = y*math.cos(r)-z*math.sin(r), y*math.sin(r)+z*math.cos(r)
    x, z = x*math.cos(pch)+z*math.sin(pch), -x*math.sin(pch)+z*math.cos(pch)
    return np.array([x, y, z+body["body_dz_mm"]], dtype=float) * 0.001


def add_connector(scene, a, b, radius, rgba):
    if scene.ngeom >= scene.maxgeom:
        return
    g = scene.geoms[scene.ngeom]
    mujoco.mjv_makeConnector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, float(radius), *map(float, a), *map(float, b))
    g.rgba[:] = rgba
    scene.ngeom += 1


def add_ellipsoid(scene, pos, size, rgba):
    if scene.ngeom >= scene.maxgeom:
        return
    g = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_ELLIPSOID, np.asarray(size, dtype=float), np.asarray(pos, dtype=float), np.eye(3).reshape(-1), rgba)
    scene.ngeom += 1


def add_box(scene, pos, halfsize, rgba):
    if scene.ngeom >= scene.maxgeom:
        return
    g = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_BOX, np.asarray(halfsize, dtype=float), np.asarray(pos, dtype=float), np.eye(3).reshape(-1), rgba)
    scene.ngeom += 1


def draw_frame(viewer, spec, frame):
    scene = viewer.user_scn
    scene.ngeom = 0
    body = frame.body
    for tag in LEG_ORDER:
        leg = frame.legs[tag]
        pts = {k: transform_point_mm(leg[k], body) for k in ("A", "B", "C", "D", "P")}
        for a, b in ((pts["A"], pts["B"]), (pts["B"], pts["C"]), (pts["D"], pts["C"]), (pts["C"], pts["P"])):
            add_connector(scene, a, b, 0.0022, COL["link"])

        left = tag.endswith("L")
        sign = 1.0 if left else -1.0
        A0 = leg["A"]
        inner = (A0[0], A0[1]-sign*34.0, A0[2])
        add_connector(scene, transform_point_mm(A0, body), transform_point_mm(inner, body), 0.0060, COL["motor"])
        shaft_end = (A0[0], A0[1]+sign*6.0, A0[2])
        add_connector(scene, pts["A"], transform_point_mm(shaft_end, body), 0.0015, COL["shaft"])
        add_ellipsoid(scene, np.array([pts["P"][0], pts["P"][1], 0.0065]), np.array([0.012, 0.010, 0.0065]), COL["paw"])

    for front, key in ((True, "front_splay"), (False, "rear_splay")):
        c = spec["splay"]["front_servo_center_mm" if front else "rear_servo_center_mm"]
        cc = transform_point_mm(c, body)
        add_box(scene, cc, np.array([0.01175, 0.006, 0.012]), COL["servo"])
        ang = math.radians(frame.pose[key])
        end = (c[0]+11.0*math.cos(ang), c[1]+11.0*math.sin(ang), c[2]+13.2)
        start = (c[0], c[1], c[2]+13.2)
        add_connector(scene, transform_point_mm(start, body), transform_point_mm(end, body), 0.0018, COL["horn"])


def parse_args():
    p = argparse.ArgumentParser(description="Pebble V3-E six-motor kinematic motion proof in MuJoCo")
    p.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier")
    p.add_argument("--fps", type=float, default=20.0)
    p.add_argument("--loop", action=argparse.BooleanOptionalAction, default=True)
    return p.parse_args()


def main():
    args = parse_args()
    spec = load_spec(ROOT / "shared" / "motion_spec.json")
    frames = build_demo_timeline(spec, args.fps)
    model = mujoco.MjModel.from_xml_path(str(HERE / "pebble_v3e_kinematic.xml"))
    data = mujoco.MjData(model)
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pebble_visual")
    mid = int(model.body_mocapid[bid])
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 0.34
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -15
        viewer.cam.lookat[:] = [0, 0, 0.075]
        while viewer.is_running():
            for f in frames:
                if not viewer.is_running():
                    break
                data.mocap_pos[mid] = [0, 0, float(f.body["body_dz_mm"]) * 0.001]
                data.mocap_quat[mid] = quat_from_roll_pitch(f.body["roll_deg"], f.body["pitch_deg"])
                mujoco.mj_forward(model, data)
                draw_frame(viewer, spec, f)
                viewer.sync()
                print(
                    f"\r{f.label:18s} belly={f.body['center_belly_clearance_mm']:5.1f}mm "
                    f"roll={f.body['roll_deg']:+4.1f} pitch={f.body['pitch_deg']:+4.1f} "
                    f"FL={f.pose['FL']:6.1f}deg",
                    end="",
                    flush=True,
                )
                time.sleep(max(0.0, 1.0 / (args.fps * max(0.05, args.speed))))
            if not args.loop:
                break
    print()


if __name__ == "__main__":
    main()
