from __future__ import annotations

import math
import time
from pathlib import Path

import mujoco
import mujoco.viewer

ROOT = Path(__file__).resolve().parent
MODEL_PATH = ROOT / "model.xml"

JOINT_SUFFIXES = (
    "hip_yaw",
    "hip_roll",
    "hip_pitch",
    "knee_pitch",
    "ankle_pitch",
    "ankle_roll",
)

MOTOR_LABELS = {
    "hip_yaw": "M1 HIP YAW     : point the leg inward/outward; used for turning",
    "hip_roll": "M2 HIP ROLL    : move leg sideways; used for weight shift/balance",
    "hip_pitch": "M3 HIP PITCH   : swing thigh forward/back; main stride joint",
    "knee_pitch": "M4 KNEE PITCH  : bend the leg; gives ground clearance",
    "ankle_pitch": "M5 ANKLE PITCH : toe up/down; landing and push-off",
    "ankle_roll": "M6 ANKLE ROLL  : tilt sole left/right; keeps foot flat while balancing",
}

BASE_RGBA = {
    "hip_yaw": (0.90, 0.28, 0.22, 1.0),
    "hip_roll": (0.24, 0.72, 0.38, 1.0),
    "hip_pitch": (0.20, 0.50, 0.90, 1.0),
    "knee_pitch": (0.95, 0.60, 0.16, 1.0),
    "ankle_pitch": (0.62, 0.34, 0.88, 1.0),
    "ankle_roll": (0.12, 0.72, 0.72, 1.0),
}

STAND = {
    "l_hip_yaw": 0.0, "r_hip_yaw": 0.0,
    "l_hip_roll": 0.0, "r_hip_roll": 0.0,
    "l_hip_pitch": -3.0, "r_hip_pitch": -3.0,
    "l_knee_pitch": 7.0, "r_knee_pitch": 7.0,
    "l_ankle_pitch": -4.0, "r_ankle_pitch": -4.0,
    "l_ankle_roll": 0.0, "r_ankle_roll": 0.0,
}

DEMO_STAGES = [
    ("hip_yaw", 22.0),
    ("hip_roll", 20.0),
    ("hip_pitch", 34.0),
    ("knee_pitch", 36.0),
    ("ankle_pitch", 26.0),
    ("ankle_roll", 20.0),
]


def actuator_id(model: mujoco.MjModel, joint_name: str) -> int:
    idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"a_{joint_name}")
    if idx < 0:
        raise RuntimeError(f"Missing actuator a_{joint_name}")
    return idx


def set_deg(model: mujoco.MjModel, data: mujoco.MjData, joint_name: str, degrees: float) -> None:
    data.ctrl[actuator_id(model, joint_name)] = math.radians(degrees)


def set_pose(model: mujoco.MjModel, data: mujoco.MjData, pose: dict[str, float]) -> None:
    for joint_name, degrees in pose.items():
        set_deg(model, data, joint_name, degrees)


def highlight_motor(model: mujoco.MjModel, suffix: str | None) -> None:
    for motor in JOINT_SUFFIXES:
        rgba = BASE_RGBA[motor]
        for side in ("l", "r"):
            gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"motor_{side}_{motor}")
            if gid < 0:
                continue
            if suffix == motor:
                # White-ish active highlight while preserving some category color.
                model.geom_rgba[gid] = [
                    min(1.0, rgba[0] * 0.35 + 0.75),
                    min(1.0, rgba[1] * 0.35 + 0.75),
                    min(1.0, rgba[2] * 0.35 + 0.75),
                    1.0,
                ]
            else:
                model.geom_rgba[gid] = rgba


def motor_demo(model: mujoco.MjModel, data: mujoco.MjData, t: float) -> str:
    stage_duration = 3.0
    stage_index = int(t // stage_duration) % len(DEMO_STAGES)
    local_t = t % stage_duration
    suffix, amplitude = DEMO_STAGES[stage_index]
    wave = math.sin(2.0 * math.pi * local_t / stage_duration)

    pose = dict(STAND)
    if suffix == "hip_yaw":
        pose["l_hip_yaw"] = amplitude * wave
        pose["r_hip_yaw"] = -amplitude * wave
    elif suffix == "hip_roll":
        pose["l_hip_roll"] = amplitude * wave
        pose["r_hip_roll"] = -amplitude * wave
    elif suffix == "hip_pitch":
        pose["l_hip_pitch"] = amplitude * wave
        pose["r_hip_pitch"] = -amplitude * wave
    elif suffix == "knee_pitch":
        bend = 44.0 + amplitude * wave
        pose["l_knee_pitch"] = max(4.0, bend)
        pose["r_knee_pitch"] = max(4.0, bend)
        pose["l_ankle_pitch"] = -0.45 * pose["l_knee_pitch"]
        pose["r_ankle_pitch"] = -0.45 * pose["r_knee_pitch"]
    elif suffix == "ankle_pitch":
        pose["l_ankle_pitch"] = amplitude * wave
        pose["r_ankle_pitch"] = amplitude * wave
    elif suffix == "ankle_roll":
        pose["l_ankle_roll"] = amplitude * wave
        pose["r_ankle_roll"] = -amplitude * wave

    set_pose(model, data, pose)
    return suffix


def walk_pose(model: mujoco.MjModel, data: mujoco.MjData, t: float) -> None:
    # Educational in-place gait. The pelvis is intentionally fixed so the user can
    # see the responsibility of every leg joint without the robot immediately falling.
    w = 2.0 * math.pi * 0.72
    p_l = w * t
    p_r = p_l + math.pi

    def leg(side: str, p: float, mirror: float) -> None:
        s = math.sin(p)
        c = math.cos(p)
        swing = max(0.0, s)

        hip_pitch = 27.0 * s
        knee = 8.0 + 53.0 * (swing ** 1.35)
        ankle_pitch = -0.55 * hip_pitch - 0.38 * knee + 4.0

        # Smaller motions show how the non-sagittal DOFs participate in a gait.
        hip_roll = mirror * 6.0 * c
        ankle_roll = -0.80 * hip_roll
        hip_yaw = mirror * 4.0 * math.sin(2.0 * p)

        set_deg(model, data, f"{side}_hip_yaw", hip_yaw)
        set_deg(model, data, f"{side}_hip_roll", hip_roll)
        set_deg(model, data, f"{side}_hip_pitch", hip_pitch)
        set_deg(model, data, f"{side}_knee_pitch", knee)
        set_deg(model, data, f"{side}_ankle_pitch", ankle_pitch)
        set_deg(model, data, f"{side}_ankle_roll", ankle_roll)

    leg("l", p_l, +1.0)
    leg("r", p_r, -1.0)


def squat_pose(model: mujoco.MjModel, data: mujoco.MjData, t: float) -> None:
    q = 0.5 - 0.5 * math.cos(2.0 * math.pi * 0.28 * t)
    knee = 8.0 + 58.0 * q
    hip = -4.0 + 27.0 * q
    ankle = -4.0 - 26.0 * q
    pose = dict(STAND)
    for side in ("l", "r"):
        pose[f"{side}_hip_pitch"] = hip
        pose[f"{side}_knee_pitch"] = knee
        pose[f"{side}_ankle_pitch"] = ankle
    set_pose(model, data, pose)


def print_help() -> None:
    print("\nPebble 6-DOF-per-leg MuJoCo demonstrator")
    print("========================================")
    print("Each leg has 6 actuators / 6 rotational DOF (12 total).")
    print("\nControls")
    print("  1 : stand")
    print("  2 : cycle through M1..M6 one motor type at a time")
    print("  3 : coordinated in-place walking gait (all 12 motors)")
    print("  4 : squat (hip pitch + knee + ankle pitch)")
    print("  R : reset simulation")
    print("  SPACE : pause/resume")
    print("\nMotor map")
    for suffix in JOINT_SUFFIXES:
        print(" ", MOTOR_LABELS[suffix])
    print("\nNote: pelvis is fixed in this first learning model so balance does not hide the kinematics.\n")


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    mode = "demo"
    paused = False
    mode_start = 0.0
    last_stage: str | None = None

    def reset() -> None:
        nonlocal mode_start, last_stage
        mujoco.mj_resetData(model, data)
        set_pose(model, data, STAND)
        mujoco.mj_forward(model, data)
        mode_start = float(data.time)
        last_stage = None

    def key_callback(keycode: int) -> None:
        nonlocal mode, paused, mode_start, last_stage
        try:
            key = chr(keycode).upper()
        except (ValueError, OverflowError):
            return
        if key == "1":
            mode = "stand"; mode_start = float(data.time); last_stage = None
            print("\nMODE: STAND")
        elif key == "2":
            mode = "demo"; mode_start = float(data.time); last_stage = None
            print("\nMODE: MOTOR-BY-MOTOR DEMO")
        elif key == "3":
            mode = "walk"; mode_start = float(data.time); last_stage = None
            print("\nMODE: WALK — all 12 leg motors coordinated")
        elif key == "4":
            mode = "squat"; mode_start = float(data.time); last_stage = None
            print("\nMODE: SQUAT")
        elif key == "R":
            reset(); print("\nRESET")
        elif key == " ":
            paused = not paused
            print("PAUSED" if paused else "RESUMED")

    print_help()
    reset()

    with mujoco.viewer.launch_passive(
        model,
        data,
        key_callback=key_callback,
        show_left_ui=False,
        show_right_ui=False,
    ) as viewer:
        viewer.cam.distance = 1.05
        viewer.cam.azimuth = 135.0
        viewer.cam.elevation = -16.0
        viewer.cam.lookat[:] = [0.0, 0.0, 0.30]

        while viewer.is_running():
            step_start = time.perf_counter()

            if not paused:
                t = float(data.time) - mode_start
                if mode == "stand":
                    set_pose(model, data, STAND)
                    highlight_motor(model, None)
                elif mode == "demo":
                    stage = motor_demo(model, data, t)
                    highlight_motor(model, stage)
                    if stage != last_stage:
                        print(f"\nACTIVE: {MOTOR_LABELS[stage]}")
                        last_stage = stage
                elif mode == "walk":
                    walk_pose(model, data, t)
                    highlight_motor(model, None)
                elif mode == "squat":
                    squat_pose(model, data, t)
                    highlight_motor(model, None)

                mujoco.mj_step(model, data)

            viewer.sync()
            remaining = model.opt.timestep - (time.perf_counter() - step_start)
            if remaining > 0:
                time.sleep(remaining)


if __name__ == "__main__":
    main()
