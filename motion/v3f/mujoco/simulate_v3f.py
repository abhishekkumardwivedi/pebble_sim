from __future__ import annotations

"""Pebble V3-F MuJoCo expression playback bridge.

This script is deliberately split from final motor/contact validation. It lets us
review the six commanded DOFs in the V3-F MJCF using the same pose targets as CAD.
The four crank joints and two mirrored pair-splay commands are rate-limited to
represent realistic visual motion. Full closed-chain paw/contact dynamics remain
the next validation gate.
"""

import argparse
import json
import math
import time
from pathlib import Path

import mujoco
import mujoco.viewer

HERE = Path(__file__).resolve().parent
SPEC = json.loads((HERE.parent / "shared" / "v3f_mechanical_spec.json").read_text())
MODEL_PATH = HERE / "pebble_v3f.xml"
POSES = SPEC["pose_targets"]

CRANK_MAX_RPM = float(SPEC["main_motor"]["crank_rated_rpm"])
SPLAY_MAX_DEG_S = min(500.0, float(SPEC["splay_servo"]["no_load_speed_deg_s"]) * 0.65)

DEMO = [
    "neutral",
    "sleepy_compact",
    "neutral",
    "alert",
    "happy_wide",
    "neutral",
    "play_bow",
    "neutral",
    "curious_left",
    "neutral",
]


def joint_qpos_address(model: mujoco.MjModel, name: str) -> int:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if jid < 0:
        raise KeyError(name)
    return int(model.jnt_qposadr[jid])


def step_toward(current: float, target: float, max_delta: float) -> float:
    d = target - current
    if abs(d) <= max_delta:
        return target
    return current + math.copysign(max_delta, d)


def set_pose_rate_limited(model: mujoco.MjModel, data: mujoco.MjData, state: dict, pose_name: str, dt: float) -> bool:
    target = POSES[pose_name]
    crank_deg_per_s = CRANK_MAX_RPM * 6.0
    settled = True

    for tag in ("FL", "FR", "RL", "RR"):
        nxt = step_toward(state[tag], float(target[tag]), crank_deg_per_s * dt)
        state[tag] = nxt
        data.qpos[joint_qpos_address(model, f"crank_{tag}")] = math.radians(nxt)
        settled &= abs(nxt - float(target[tag])) < 0.05

    front = step_toward(state["front_splay"], float(target["front_splay"]), SPLAY_MAX_DEG_S * dt)
    rear = step_toward(state["rear_splay"], float(target["rear_splay"]), SPLAY_MAX_DEG_S * dt)
    state["front_splay"] = front
    state["rear_splay"] = rear
    data.qpos[joint_qpos_address(model, "splay_FL")] = math.radians(front)
    data.qpos[joint_qpos_address(model, "splay_FR")] = math.radians(-front)
    data.qpos[joint_qpos_address(model, "splay_RL")] = math.radians(rear)
    data.qpos[joint_qpos_address(model, "splay_RR")] = math.radians(-rear)
    settled &= abs(front - float(target["front_splay"])) < 0.05
    settled &= abs(rear - float(target["rear_splay"])) < 0.05

    mujoco.mj_forward(model, data)
    return bool(settled)


def run(sequence: list[str], hold_s: float) -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    neutral = POSES["neutral"]
    state = {
        **{tag: float(neutral[tag]) for tag in ("FL", "FR", "RL", "RR")},
        "front_splay": float(neutral["front_splay"]),
        "rear_splay": float(neutral["rear_splay"]),
    }

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 0.34
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -18
        viewer.cam.lookat[:] = [0, 0, 0.075]
        for pose_name in sequence:
            if pose_name not in POSES:
                raise KeyError(f"Unknown expression: {pose_name}")
            settled_for = 0.0
            while viewer.is_running() and settled_for < hold_s:
                t0 = time.perf_counter()
                settled = set_pose_rate_limited(model, data, state, pose_name, model.opt.timestep)
                viewer.sync()
                settled_for = settled_for + model.opt.timestep if settled else 0.0
                sleep = model.opt.timestep - (time.perf_counter() - t0)
                if sleep > 0:
                    time.sleep(sleep)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--expression", choices=sorted(POSES), default=None)
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--hold", type=float, default=0.8)
    args = parser.parse_args()
    sequence = DEMO if args.demo or args.expression is None else ["neutral", args.expression, "neutral"]
    run(sequence, max(0.1, args.hold))


if __name__ == "__main__":
    main()
