from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, Tuple

from config import (
    ANKLE_DAMPING,
    ANKLE_RANGE_RAD,
    ANKLE_STIFFNESS,
    BASE_CRANK_PHASE_RAD,
    BODY_HEIGHT_M,
    BODY_LENGTH_M,
    BODY_ORIGIN_Z_M,
    BODY_WIDTH_M,
    CONTACT_SOLIMP,
    CONTACT_SOLREF,
    COUPLER_L_M,
    CRANK_R_M,
    DEFAULT_FRICTION,
    FOOT_HALF_X_M,
    FOOT_HALF_Y_M,
    FOOT_HALF_Z_M,
    FOOT_OFFSET_E_M,
    FRAME_D_M,
    FRONT_MOTOR_X_M,
    LEFT_MOTOR_Y_M,
    LEGS,
    LOW_CHASSIS_MASS_KG,
    LOW_CHASSIS_Z_M,
    MOTOR_MOUNT_Z_M,
    N20_MASS_KG,
    N20_STALL_TORQUE_NM,
    REAR_MOTOR_X_M,
    RIGHT_MOTOR_Y_M,
    ROCKER_Q_M,
    SHELL_MASS_KG,
)

ROOT = Path(__file__).resolve().parent
DEFAULT_MODEL_PATH = ROOT / "models" / "pebble.xml"


def _fmt(value: float) -> str:
    return f"{value:.9g}"


def _quat_y(phi: float) -> Tuple[float, float, float, float]:
    half = 0.5 * phi
    return (math.cos(half), 0.0, math.sin(half), 0.0)


def _quat_str(q: Tuple[float, float, float, float]) -> str:
    return " ".join(_fmt(x) for x in q)


def _solve_initial_geometry(theta: float) -> Dict[str, float]:
    """Solve the lower branch of the planar four-bar."""
    r = CRANK_R_M
    d = FRAME_D_M
    L = COUPLER_L_M
    q = ROCKER_Q_M
    E = FOOT_OFFSET_E_M

    ax = r * math.cos(theta)
    az = r * math.sin(theta)
    dx = d - ax
    dz = -az
    D = math.hypot(dx, dz)

    if D > L + q or D < abs(L - q) or D <= 1e-12:
        raise ValueError("Chosen linkage geometry cannot close.")

    a = (L * L - q * q + D * D) / (2.0 * D)
    h2 = L * L - a * a
    if h2 < -1e-12:
        raise ValueError("Invalid four-bar geometry.")

    h = math.sqrt(max(0.0, h2))
    ux = dx / D
    uz = dz / D
    px = ax + a * ux
    pz = az + a * uz

    p1 = (px - uz * h, pz + ux * h)
    p2 = (px + uz * h, pz - ux * h)
    bx, bz = p1 if p1[1] < p2[1] else p2

    alpha = math.atan2(bz - az, bx - ax)
    beta = math.atan2(bz, bx - d)

    fx = bx + E * math.sin(alpha)
    fz = bz - E * math.cos(alpha)

    return {
        "ax": ax,
        "az": az,
        "bx": bx,
        "bz": bz,
        "fx": fx,
        "fz": fz,
        "alpha": alpha,
        "beta": beta,
    }


def _leg_mount(name: str) -> Tuple[float, float, float]:
    is_front = name[0] == "F"
    is_left = name[1] == "L"
    x = FRONT_MOTOR_X_M if is_front else REAR_MOTOR_X_M
    y = LEFT_MOTOR_Y_M if is_left else RIGHT_MOTOR_Y_M
    return x, y, MOTOR_MOUNT_Z_M


def _leg_xml(name: str, geometry: Dict[str, float]) -> str:
    x, y, z = _leg_mount(name)

    phi_crank = -BASE_CRANK_PHASE_RAD
    phi_coupler = -geometry["alpha"]
    phi_rocker = -geometry["beta"]
    phi_coupler_rel = phi_coupler - phi_crank
    phi_foot_rel = -phi_coupler

    return f'''
      <body name="{name}_mount" pos="{_fmt(x)} {_fmt(y)} {_fmt(z)}">

        <geom name="{name}_motor_housing" type="box"
              size="0.006 0.013 0.0055" pos="-0.006 0 0"
              material="motor_mat" mass="{_fmt(N20_MASS_KG)}"
              contype="0" conaffinity="0"/>

        <geom type="cylinder" pos="0 0 0" size="0.003 0.0025"
              quat="0.70710678 0.70710678 0 0"
              material="metal_mat" mass="0" contype="0" conaffinity="0"/>

        <body name="{name}_crank" quat="{_quat_str(_quat_y(phi_crank))}">
          <joint name="{name}_crank_joint" type="hinge" axis="0 1 0"
                 damping="0.0008" armature="0.000003"/>

          <geom name="{name}_crank_geom" type="capsule"
                fromto="0 0 0 {_fmt(CRANK_R_M)} 0 0" size="0.0028"
                material="link_dark" mass="0.003" contype="0" conaffinity="0"/>
          <geom type="sphere" pos="0 0 0" size="0.0035"
                material="joint_mat" mass="0" contype="0" conaffinity="0"/>
          <site name="{name}_crank_load_site" pos="0 0 0" size="0.001" rgba="0 0 0 0"/>

          <body name="{name}_coupler" pos="{_fmt(CRANK_R_M)} 0 0"
                quat="{_quat_str(_quat_y(phi_coupler_rel))}">
            <joint name="{name}_coupler_joint" type="hinge" axis="0 1 0" damping="0.0005"/>

            <geom name="{name}_coupler_geom" type="capsule"
                  fromto="0 0 0 {_fmt(COUPLER_L_M)} 0 0" size="0.0031"
                  material="link_mid" mass="0.004" contype="0" conaffinity="0"/>

            <geom name="{name}_carrier_geom" type="capsule"
                  fromto="{_fmt(COUPLER_L_M)} 0 0 {_fmt(COUPLER_L_M)} 0 -{_fmt(FOOT_OFFSET_E_M)}"
                  size="0.0033" material="link_mid" mass="0.002"
                  contype="0" conaffinity="0"/>

            <geom type="sphere" pos="{_fmt(COUPLER_L_M)} 0 0" size="0.0038"
                  material="joint_mat" mass="0" contype="0" conaffinity="0"/>

            <site name="{name}_coupler_load_site" pos="0 0 0" size="0.001" rgba="0 0 0 0"/>
            <site name="{name}_coupler_B" pos="{_fmt(COUPLER_L_M)} 0 0" size="0.0015" rgba="0.9 0.2 0.2 1"/>

            <body name="{name}_foot" pos="{_fmt(COUPLER_L_M)} 0 -{_fmt(FOOT_OFFSET_E_M)}"
                  quat="{_quat_str(_quat_y(phi_foot_rel))}">
              <joint name="{name}_ankle_joint" type="hinge" axis="0 1 0"
                     range="-{_fmt(ANKLE_RANGE_RAD)} {_fmt(ANKLE_RANGE_RAD)}"
                     limited="true" damping="{_fmt(ANKLE_DAMPING)}"
                     stiffness="{_fmt(ANKLE_STIFFNESS)}" springref="0"/>

              <site name="{name}_ankle_load_site" pos="0 0 0" size="0.001" rgba="0 0 0 0"/>
              <site name="{name}_foot_center" pos="0 0 -{_fmt(FOOT_HALF_Z_M)}"
                    size="0.002" rgba="0.1 0.7 0.1 0.35"/>

              <geom name="{name}_foot_visual" type="ellipsoid"
                    pos="0 0 -{_fmt(FOOT_HALF_Z_M)}"
                    size="{_fmt(FOOT_HALF_X_M * 1.08)} {_fmt(FOOT_HALF_Y_M * 1.08)} {_fmt(FOOT_HALF_Z_M * 1.45)}"
                    material="foot_mat" mass="0.008" contype="0" conaffinity="0"/>

              <geom name="{name}_foot_geom" type="box"
                    pos="0 0 -{_fmt(FOOT_HALF_Z_M)}"
                    size="{_fmt(FOOT_HALF_X_M)} {_fmt(FOOT_HALF_Y_M)} {_fmt(FOOT_HALF_Z_M)}"
                    rgba="0 0 0 0" mass="0"
                    friction="{_fmt(DEFAULT_FRICTION)} 0.03 0.003"
                    solref="{CONTACT_SOLREF}" solimp="{CONTACT_SOLIMP}"
                    contype="1" conaffinity="1" condim="6"/>
            </body>
          </body>
        </body>

        <body name="{name}_rocker" pos="{_fmt(FRAME_D_M)} 0 0"
              quat="{_quat_str(_quat_y(phi_rocker))}">
          <joint name="{name}_rocker_joint" type="hinge" axis="0 1 0" damping="0.0005"/>
          <geom name="{name}_rocker_geom" type="capsule"
                fromto="0 0 0 {_fmt(ROCKER_Q_M)} 0 0" size="0.0031"
                material="link_warm" mass="0.003" contype="0" conaffinity="0"/>
          <geom type="sphere" pos="0 0 0" size="0.0036"
                material="joint_mat" mass="0" contype="0" conaffinity="0"/>
          <site name="{name}_rocker_load_site" pos="0 0 0" size="0.001" rgba="0 0 0 0"/>
          <site name="{name}_rocker_B" pos="{_fmt(ROCKER_Q_M)} 0 0" size="0.0015" rgba="0.9 0.2 0.2 1"/>
        </body>
      </body>
'''


def build_xml() -> str:
    geometry = _solve_initial_geometry(BASE_CRANK_PHASE_RAD)
    legs_xml = "\n".join(_leg_xml(leg, geometry) for leg in LEGS)

    equality = "\n".join(
        f'    <connect name="loop_{leg}" site1="{leg}_coupler_B" site2="{leg}_rocker_B" '
        f'solref="0.0035 1" solimp="0.94 0.99 0.001"/>'
        for leg in LEGS
    )

    actuators = "\n".join(
        f'    <motor name="motor_{leg}" joint="{leg}_crank_joint" gear="1" '
        f'ctrllimited="true" ctrlrange="-{_fmt(N20_STALL_TORQUE_NM)} {_fmt(N20_STALL_TORQUE_NM)}"/>'
        for leg in LEGS
    )

    sensors = [
        '    <accelerometer name="imu_accel" site="imu_site"/>',
        '    <gyro name="imu_gyro" site="imu_site"/>',
        '    <subtreecom name="robot_com" body="pebble"/>',
    ]

    for leg in LEGS:
        sensors.extend([
            f'    <jointpos name="{leg}_crank_pos" joint="{leg}_crank_joint"/>',
            f'    <jointvel name="{leg}_crank_vel" joint="{leg}_crank_joint"/>',
            f'    <actuatorfrc name="{leg}_motor_torque" actuator="motor_{leg}"/>',
            f'    <force name="{leg}_crank_force" site="{leg}_crank_load_site"/>',
            f'    <torque name="{leg}_crank_torque_sensor" site="{leg}_crank_load_site"/>',
            f'    <force name="{leg}_coupler_force" site="{leg}_coupler_load_site"/>',
            f'    <torque name="{leg}_coupler_torque_sensor" site="{leg}_coupler_load_site"/>',
            f'    <force name="{leg}_rocker_force" site="{leg}_rocker_load_site"/>',
            f'    <torque name="{leg}_rocker_torque_sensor" site="{leg}_rocker_load_site"/>',
            f'    <force name="{leg}_ankle_force" site="{leg}_ankle_load_site"/>',
            f'    <torque name="{leg}_ankle_torque_sensor" site="{leg}_ankle_load_site"/>',
        ])

    return f'''
<mujoco model="pebble_v1_2">
  <compiler angle="radian" autolimits="true"/>

  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast"
          iterations="100" ls_iterations="30" cone="elliptic"/>

  <visual>
    <quality shadowsize="2048"/>
    <headlight ambient="0.16 0.17 0.19" diffuse="0.58 0.59 0.62" specular="0.10 0.10 0.10"/>
    <rgba haze="0.055 0.060 0.070 1"/>
    <global azimuth="135" elevation="-22"/>
  </visual>

  <asset>
    <texture name="sky" type="skybox" builtin="gradient"
             rgb1="0.055 0.065 0.080" rgb2="0.14 0.15 0.17"
             width="512" height="3072"/>

    <texture name="grid_tex" type="2d" builtin="checker"
             rgb1="0.095 0.105 0.120" rgb2="0.145 0.155 0.175"
             width="512" height="512"/>
    <material name="grid_mat" texture="grid_tex" texrepeat="28 28"
              reflectance="0.04" shininess="0.05" specular="0.08"/>

    <material name="shell_mat" rgba="0.83 0.78 0.68 1" specular="0.22" shininess="0.30"/>
    <material name="shell_trim" rgba="0.57 0.52 0.45 1" specular="0.16" shininess="0.22"/>
    <material name="face_mat" rgba="0.035 0.040 0.050 1" specular="0.50" shininess="0.75"/>
    <material name="eye_mat" rgba="0.20 0.72 0.95 1" emission="0.16" specular="0.55" shininess="0.80"/>
    <material name="motor_mat" rgba="0.20 0.22 0.25 1" specular="0.32" shininess="0.45"/>
    <material name="metal_mat" rgba="0.48 0.50 0.54 1" specular="0.55" shininess="0.70"/>
    <material name="joint_mat" rgba="0.16 0.17 0.19 1" specular="0.45" shininess="0.65"/>
    <material name="link_dark" rgba="0.30 0.32 0.35 1" specular="0.20" shininess="0.30"/>
    <material name="link_mid" rgba="0.34 0.48 0.62 1" specular="0.18" shininess="0.28"/>
    <material name="link_warm" rgba="0.53 0.43 0.29 1" specular="0.18" shininess="0.28"/>
    <material name="foot_mat" rgba="0.15 0.21 0.19 1" specular="0.08" shininess="0.10"/>
  </asset>

  <worldbody>
    <light name="key" pos="0.15 -0.55 0.70" dir="-0.15 0.55 -1"
           directional="true" diffuse="0.74 0.73 0.70" specular="0.18 0.18 0.18"/>
    <light name="fill" pos="-0.35 0.40 0.42" dir="0.3 -0.35 -0.7"
           directional="true" diffuse="0.28 0.31 0.38" specular="0.04 0.04 0.05"/>

    <geom name="floor" type="plane" size="2.0 2.0 0.05"
          material="grid_mat" friction="{_fmt(DEFAULT_FRICTION)} 0.03 0.003"
          solref="{CONTACT_SOLREF}" solimp="{CONTACT_SOLIMP}" condim="6"/>

    <body name="pebble" pos="0 0 {_fmt(BODY_ORIGIN_Z_M)}">
      <freejoint name="root_free"/>

      <geom name="shell_visual" type="ellipsoid"
            size="{_fmt(BODY_LENGTH_M / 2)} {_fmt(BODY_WIDTH_M / 2)} {_fmt(BODY_HEIGHT_M / 2)}"
            material="shell_mat" mass="{_fmt(SHELL_MASS_KG)}" contype="0" conaffinity="0"/>

      <geom name="shell_collision" type="ellipsoid"
            size="{_fmt(BODY_LENGTH_M / 2 * 0.96)} {_fmt(BODY_WIDTH_M / 2 * 0.96)} {_fmt(BODY_HEIGHT_M / 2 * 0.95)}"
            rgba="0 0 0 0" mass="0" friction="0.55 0.02 0.002"
            contype="1" conaffinity="1"/>

      <geom type="ellipsoid" pos="-0.004 0 -0.031" size="0.068 0.061 0.021"
            material="shell_trim" mass="0" contype="0" conaffinity="0"/>

      <geom type="ellipsoid" pos="0.078 0 0.012" size="0.0045 0.043 0.030"
            material="face_mat" mass="0" contype="0" conaffinity="0"/>
      <geom type="sphere" pos="0.081 0.023 0.018" size="0.0062"
            material="eye_mat" mass="0" contype="0" conaffinity="0"/>
      <geom type="sphere" pos="0.081 -0.023 0.018" size="0.0062"
            material="eye_mat" mass="0" contype="0" conaffinity="0"/>
      <geom type="capsule" fromto="0.081 -0.009 -0.007 0.081 0.009 -0.007"
            size="0.0016" rgba="0.65 0.69 0.72 1" mass="0" contype="0" conaffinity="0"/>

      <geom type="capsule" fromto="0.020 0.036 0.047 0.033 0.048 0.078"
            size="0.0022" rgba="0.34 0.31 0.28 1" mass="0" contype="0" conaffinity="0"/>
      <geom type="capsule" fromto="0.020 -0.036 0.047 0.033 -0.048 0.078"
            size="0.0022" rgba="0.34 0.31 0.28 1" mass="0" contype="0" conaffinity="0"/>
      <geom type="sphere" pos="0.033 0.048 0.078" size="0.0042"
            material="eye_mat" mass="0" contype="0" conaffinity="0"/>
      <geom type="sphere" pos="0.033 -0.048 0.078" size="0.0042"
            material="eye_mat" mass="0" contype="0" conaffinity="0"/>

      <geom name="low_chassis_mass" type="box" pos="-0.005 0 {_fmt(LOW_CHASSIS_Z_M)}"
            size="0.055 0.045 0.010" rgba="0 0 0 0"
            mass="{_fmt(LOW_CHASSIS_MASS_KG)}" contype="0" conaffinity="0"/>
      <geom name="battery" type="box" pos="-0.005 0 -0.027" size="0.042 0.033 0.011"
            rgba="0 0 0 0" mass="0.120" contype="0" conaffinity="0"/>
      <geom name="electronics" type="box" pos="-0.005 0 -0.004" size="0.036 0.029 0.004"
            rgba="0 0 0 0" mass="0.040" contype="0" conaffinity="0"/>
      <geom name="display_mass" type="box" pos="0.067 0 0.012" size="0.003 0.032 0.022"
            rgba="0 0 0 0" mass="0.016" contype="0" conaffinity="0"/>

      <site name="imu_site" pos="-0.005 0 -0.005" size="0.003" rgba="1 0.5 0 0.5"/>

{legs_xml}
    </body>
  </worldbody>

  <equality>
{equality}
  </equality>

  <actuator>
{actuators}
  </actuator>

  <sensor>
{chr(10).join(sensors)}
  </sensor>
</mujoco>
'''


def write_model(path: Path = DEFAULT_MODEL_PATH) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(build_xml(), encoding="utf-8")
    return path


def main() -> None:
    geometry = _solve_initial_geometry(BASE_CRANK_PHASE_RAD)
    path = write_model()

    print(f"Wrote: {path}")
    print("Initial linkage geometry:")
    print(f"  B pin: x={geometry['bx'] * 1000:.2f} mm, z={geometry['bz'] * 1000:.2f} mm")
    print(f"  Ankle: x={geometry['fx'] * 1000:.2f} mm, z={geometry['fz'] * 1000:.2f} mm")
    print("  Target motion: ~10 mm lift / ~30 mm stride")


if __name__ == "__main__":
    main()
