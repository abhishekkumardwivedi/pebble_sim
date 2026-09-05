from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer as mjviewer

from config import DEFAULT_FRICTION, DEFAULT_GAIT, DEFAULT_RPM, LEGS
from gait_controller import GaitController
from generate_model import DEFAULT_MODEL_PATH, write_model
from telemetry import TelemetryLogger

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"

CAMERA_PRESETS = {
    # distance, azimuth, elevation, look-at Z offset
    "beauty": (0.34, 135.0, -20.0, 0.070),
    "side": (0.30, 90.0, -10.0, 0.060),
    "front": (0.30, 0.0, -10.0, 0.060),
    "top": (0.42, 90.0, -88.0, 0.000),
    "low": (0.29, 135.0, -7.0, 0.045),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Pebble V1.2 four-leg MuJoCo simulation")
    parser.add_argument("--duration", type=float, default=30.0, help="Simulation duration in seconds")
    parser.add_argument("--rpm", type=float, default=DEFAULT_RPM, help="Average crank-cycle RPM")
    parser.add_argument("--gait", choices=["crawl", "trot", "bounce"], default=DEFAULT_GAIT)
    parser.add_argument("--friction", type=float, default=DEFAULT_FRICTION, help="Foot/floor sliding friction")
    parser.add_argument("--duty", type=float, default=None, help="Override stance duty factor, e.g. 0.74")
    parser.add_argument("--camera", choices=sorted(CAMERA_PRESETS), default="beauty")
    parser.add_argument(
        "--follow",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Keep camera centered on the robot",
    )
    parser.add_argument("--inspect", action="store_true", help="Show contacts, forces, joints and COM")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--no-plots", action="store_true")
    parser.add_argument("--no-realtime", action="store_true")
    parser.add_argument("--regenerate-model", action="store_true")
    return parser.parse_args()


def set_contact_friction(model: mujoco.MjModel, mu: float) -> None:
    mu = max(0.05, float(mu))
    names = ["floor"] + [f"{leg}_foot_geom" for leg in LEGS]
    for name in names:
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if geom_id >= 0:
            model.geom_friction[geom_id, 0] = mu


def print_model_summary(model: mujoco.MjModel, args: argparse.Namespace) -> None:
    print("\nPebble V1.2")
    print("-----------")
    print(f"Bodies:      {model.nbody}")
    print(f"Joints:      {model.njnt}")
    print(f"Actuators:   {model.nu}")
    print(f"Constraints: {model.neq}")
    print(f"Total mass:  {float(model.body_mass.sum()):.3f} kg")
    print(f"Timestep:    {model.opt.timestep * 1000:.1f} ms")
    print(f"Gait:        {args.gait}")
    print(f"RPM:         {args.rpm:.1f}")
    print(f"Friction:    {args.friction:.2f}")
    print(f"Camera:      {args.camera}")


def configure_camera(viewer, preset: str, robot_pos) -> None:
    distance, azimuth, elevation, z_offset = CAMERA_PRESETS[preset]
    viewer.cam.distance = distance
    viewer.cam.azimuth = azimuth
    viewer.cam.elevation = elevation
    viewer.cam.lookat[:] = [
        float(robot_pos[0]),
        float(robot_pos[1]),
        float(robot_pos[2]) + z_offset,
    ]


def enable_inspection(viewer) -> None:
    for flag_name in ("mjVIS_CONTACTPOINT", "mjVIS_CONTACTFORCE", "mjVIS_JOINT", "mjVIS_COM"):
        flag = getattr(mujoco.mjtVisFlag, flag_name, None)
        if flag is not None:
            viewer.opt.flags[int(flag)] = 1


def run_simulation(args: argparse.Namespace) -> None:
    if args.regenerate_model or not DEFAULT_MODEL_PATH.exists():
        write_model(DEFAULT_MODEL_PATH)

    model = mujoco.MjModel.from_xml_path(str(DEFAULT_MODEL_PATH))
    data = mujoco.MjData(model)
    set_contact_friction(model, args.friction)
    print_model_summary(model, args)

    controller = GaitController(model, gait=args.gait, rpm=args.rpm, duty_factor=args.duty)
    logger = TelemetryLogger(model, RESULTS)

    robot_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pebble")
    if robot_body_id < 0:
        raise RuntimeError("Could not find body 'pebble'.")

    mujoco.mj_forward(model, data)

    viewer = None
    if not args.headless:
        viewer = mjviewer.launch_passive(model, data)
        configure_camera(viewer, args.camera, data.xpos[robot_body_id])
        if args.inspect:
            enable_inspection(viewer)

    wall_start = time.perf_counter()
    sim_start = float(data.time)

    try:
        while float(data.time) - sim_start < max(0.1, args.duration):
            if viewer is not None and not viewer.is_running():
                break

            step_wall_start = time.perf_counter()

            control_state = controller.update(data)
            mujoco.mj_step(model, data)
            logger.maybe_log(data, control_state)

            if viewer is not None:
                if args.follow:
                    _, _, _, z_offset = CAMERA_PRESETS[args.camera]
                    robot_pos = data.xpos[robot_body_id]
                    viewer.cam.lookat[:] = [
                        float(robot_pos[0]),
                        float(robot_pos[1]),
                        float(robot_pos[2]) + z_offset,
                    ]
                viewer.sync()

            if not args.no_realtime:
                elapsed = time.perf_counter() - step_wall_start
                sleep_s = model.opt.timestep - elapsed
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
    finally:
        if viewer is not None:
            viewer.close()

    wall_elapsed = time.perf_counter() - wall_start
    sim_elapsed = float(data.time) - sim_start

    telemetry_csv = logger.write_csv()
    peak_csv = logger.write_peak_loads()
    summary_json = logger.write_summary()
    plot_paths = [] if args.no_plots else logger.make_plots()

    print("\nSimulation finished")
    print("-------------------")
    print(f"Simulated:  {sim_elapsed:.2f} s")
    print(f"Wall time:  {wall_elapsed:.2f} s")
    if wall_elapsed > 1e-9:
        print(f"Speed:      {sim_elapsed / wall_elapsed:.2f} x realtime")
    print(f"Telemetry:  {telemetry_csv}")
    print(f"Peak loads: {peak_csv}")
    print(f"Summary:    {summary_json}")
    for path in plot_paths:
        print(f"Plot:       {path}")


def main() -> None:
    run_simulation(parse_args())


if __name__ == "__main__":
    main()
