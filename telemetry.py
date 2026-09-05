from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import mujoco
import numpy as np

from config import (
    CONTACT_FORCE_THRESHOLD_N,
    LEGS,
    N20_VOLTAGE_V,
    TELEMETRY_HZ,
)
from gait_controller import LegControlState


def _quat_to_rpy_wxyz(q: Sequence[float]) -> Tuple[float, float, float]:
    w, x, y, z = [float(v) for v in q]
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _convex_hull(points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    pts = sorted(set(points))
    if len(pts) <= 1:
        return pts

    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _signed_margin_to_convex_polygon(
    point: Tuple[float, float], polygon: List[Tuple[float, float]]
) -> float:
    """Positive inside margin, negative outside. Returns NaN for <3 points."""
    if len(polygon) < 3:
        return float("nan")

    px, py = point
    min_dist = float("inf")
    inside = True
    for i, a in enumerate(polygon):
        b = polygon[(i + 1) % len(polygon)]
        ex, ey = b[0] - a[0], b[1] - a[1]
        vx, vy = px - a[0], py - a[1]
        cross = ex * vy - ey * vx
        if cross < -1e-12:
            inside = False
        denom = math.hypot(ex, ey)
        if denom > 1e-12:
            min_dist = min(min_dist, abs(cross) / denom)
    if min_dist == float("inf"):
        return float("nan")
    return min_dist if inside else -min_dist


class SensorAccessor:
    def __init__(self, model: mujoco.MjModel) -> None:
        self.model = model
        self.cache: Dict[str, Tuple[int, int]] = {}

    def read(self, data: mujoco.MjData, name: str) -> np.ndarray:
        if name not in self.cache:
            sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            if sid < 0:
                raise KeyError(f"Sensor not found: {name}")
            adr = int(self.model.sensor_adr[sid])
            dim = int(self.model.sensor_dim[sid])
            self.cache[name] = (adr, dim)
        adr, dim = self.cache[name]
        return np.array(data.sensordata[adr:adr+dim], dtype=float)


class TelemetryLogger:
    def __init__(
        self,
        model: mujoco.MjModel,
        output_dir: Path,
        telemetry_hz: float = TELEMETRY_HZ,
    ) -> None:
        self.model = model
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.period = 1.0 / float(telemetry_hz)
        self.next_t = 0.0
        self.rows: List[Dict[str, float]] = []
        self.sensors = SensorAccessor(model)

        self.robot_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pebble")
        self.floor_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self.foot_geom_ids = {
            leg: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{leg}_foot_geom")
            for leg in LEGS
        }
        self.foot_site_ids = {
            leg: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"{leg}_foot_center")
            for leg in LEGS
        }
        self._prev_t: float | None = None
        self._prev_foot_xy: Dict[str, Tuple[float, float]] = {}

    def _foot_forces(self, data: mujoco.MjData) -> Dict[str, float]:
        result = {leg: 0.0 for leg in LEGS}
        id_to_leg = {gid: leg for leg, gid in self.foot_geom_ids.items()}
        force6 = np.zeros(6, dtype=float)
        for i in range(int(data.ncon)):
            c = data.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            if self.floor_geom_id not in (g1, g2):
                continue
            other = g2 if g1 == self.floor_geom_id else g1
            leg = id_to_leg.get(other)
            if leg is None:
                continue
            mujoco.mj_contactForce(self.model, data, i, force6)
            result[leg] += abs(float(force6[0]))
        return result

    def maybe_log(
        self,
        data: mujoco.MjData,
        control_states: Dict[str, LegControlState],
    ) -> None:
        if float(data.time) + 1e-12 < self.next_t:
            return
        self.next_t += self.period
        self._log_now(data, control_states)

    def _log_now(
        self,
        data: mujoco.MjData,
        control_states: Dict[str, LegControlState],
    ) -> None:
        t = float(data.time)
        q = data.xquat[self.robot_body_id]
        roll, pitch, yaw = _quat_to_rpy_wxyz(q)
        body_pos = data.xpos[self.robot_body_id]
        com = data.subtree_com[self.robot_body_id]
        foot_normal = self._foot_forces(data)

        row: Dict[str, float] = {
            "time_s": t,
            "body_x_m": float(body_pos[0]),
            "body_y_m": float(body_pos[1]),
            "body_z_m": float(body_pos[2]),
            "roll_deg": math.degrees(roll),
            "pitch_deg": math.degrees(pitch),
            "yaw_deg": math.degrees(yaw),
            "com_x_m": float(com[0]),
            "com_y_m": float(com[1]),
            "com_z_m": float(com[2]),
        }

        support_points: List[Tuple[float, float]] = []
        total_current = 0.0
        dt_log = None if self._prev_t is None else max(1e-9, t - self._prev_t)
        current_foot_xy: Dict[str, Tuple[float, float]] = {}
        total_electrical = 0.0
        total_mechanical = 0.0

        for leg in LEGS:
            p = data.site_xpos[self.foot_site_ids[leg]]
            xy = (float(p[0]), float(p[1]))
            current_foot_xy[leg] = xy
            in_contact = foot_normal[leg] > CONTACT_FORCE_THRESHOLD_N
            prev_xy = self._prev_foot_xy.get(leg)
            if in_contact and prev_xy is not None and dt_log is not None:
                slip_speed = math.hypot(xy[0]-prev_xy[0], xy[1]-prev_xy[1]) / dt_log
            else:
                slip_speed = 0.0
            if in_contact:
                support_points.append((float(p[0]), float(p[1])))

            st = control_states[leg]
            total_current += st.motor.current_a
            total_electrical += st.motor.electrical_power_w
            total_mechanical += st.motor.mechanical_power_w

            row.update({
                f"{leg}_foot_x_m": float(p[0]),
                f"{leg}_foot_y_m": float(p[1]),
                f"{leg}_foot_z_m": float(p[2]),
                f"{leg}_foot_normal_N": foot_normal[leg],
                f"{leg}_contact": 1.0 if in_contact else 0.0,
                f"{leg}_slip_speed_m_s": slip_speed,
                f"{leg}_commanded_stance": 1.0 if st.in_commanded_stance else 0.0,
                f"{leg}_desired_phase_rad": st.desired_phase_rad,
                f"{leg}_actual_phase_rad": st.actual_phase_rad,
                f"{leg}_phase_error_rad": st.phase_error_rad,
                f"{leg}_shaft_speed_rad_s": st.shaft_speed_rad_s,
                f"{leg}_pwm": st.motor.pwm,
                f"{leg}_motor_torque_Nm": st.motor.torque_nm,
                f"{leg}_motor_current_A": st.motor.current_a,
                f"{leg}_motor_electrical_W": st.motor.electrical_power_w,
                f"{leg}_motor_mechanical_W": st.motor.mechanical_power_w,
            })

            # Direct child-parent interaction loads from MuJoCo force/torque sensors.
            for joint_part, sensor_base in (
                ("crank", "crank"),
                ("coupler", "coupler"),
                ("rocker", "rocker"),
                ("ankle", "ankle"),
            ):
                fvec = self.sensors.read(data, f"{leg}_{sensor_base}_force")
                tvec = self.sensors.read(data, f"{leg}_{sensor_base}_torque_sensor")
                row[f"{leg}_{joint_part}_force_N"] = float(np.linalg.norm(fvec))
                row[f"{leg}_{joint_part}_torque_Nm"] = float(np.linalg.norm(tvec))

        hull = _convex_hull(support_points)
        margin = _signed_margin_to_convex_polygon((float(com[0]), float(com[1])), hull)
        row["support_contacts"] = float(len(support_points))
        row["stability_margin_m"] = margin
        row["stable_by_support_polygon"] = 1.0 if (len(hull) >= 3 and not math.isnan(margin) and margin >= 0.0) else 0.0
        row["total_motor_current_A"] = total_current
        row["motor_bus_power_W"] = total_electrical
        row["total_mechanical_power_W"] = total_mechanical
        row["estimated_bus_voltage_V"] = N20_VOLTAGE_V
        self.rows.append(row)
        self._prev_t = t
        self._prev_foot_xy = current_foot_xy

    def write_csv(self, filename: str = "telemetry.csv") -> Path:
        path = self.output_dir / filename
        if not self.rows:
            path.write_text("", encoding="utf-8")
            return path
        keys = list(self.rows[0].keys())
        with path.open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(self.rows)
        return path

    def write_peak_loads(self, filename: str = "peak_loads.csv") -> Path:
        path = self.output_dir / filename
        fields: List[str] = []
        for leg in LEGS:
            fields.extend([
                f"{leg}_foot_normal_N",
                f"{leg}_motor_torque_Nm",
                f"{leg}_crank_force_N",
                f"{leg}_crank_torque_Nm",
                f"{leg}_coupler_force_N",
                f"{leg}_coupler_torque_Nm",
                f"{leg}_rocker_force_N",
                f"{leg}_rocker_torque_Nm",
                f"{leg}_ankle_force_N",
                f"{leg}_ankle_torque_Nm",
            ])

        with path.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["quantity", "peak_abs_value", "time_s"])
            for field in fields:
                if not self.rows:
                    continue
                best = max(self.rows, key=lambda r: abs(float(r.get(field, 0.0))))
                w.writerow([field, abs(float(best.get(field, 0.0))), float(best["time_s"])])
        return path

    def write_summary(self, filename: str = "summary.json") -> Path:
        path = self.output_dir / filename
        if not self.rows:
            summary = {"samples": 0}
        else:
            def max_abs(field: str) -> float:
                return max(abs(float(r.get(field, 0.0))) for r in self.rows)

            summary = {
                "samples": len(self.rows),
                "duration_s": float(self.rows[-1]["time_s"]),
                "max_abs_roll_deg": max_abs("roll_deg"),
                "max_abs_pitch_deg": max_abs("pitch_deg"),
                "max_total_motor_current_A": max(float(r["total_motor_current_A"]) for r in self.rows),
                "mean_total_motor_current_A": sum(float(r["total_motor_current_A"]) for r in self.rows) / len(self.rows),
                "max_motor_bus_power_W": max(float(r["motor_bus_power_W"]) for r in self.rows),
                "minimum_stability_margin_m": min(
                    (float(r["stability_margin_m"]) for r in self.rows if not math.isnan(float(r["stability_margin_m"]))),
                    default=float("nan"),
                ),
                "fraction_samples_support_polygon_stable": sum(float(r["stable_by_support_polygon"]) for r in self.rows) / len(self.rows),
                "max_contact_slip_speed_m_s": max(
                    float(r.get(f"{leg}_slip_speed_m_s", 0.0))
                    for r in self.rows for leg in LEGS
                ),
                "mean_support_contacts": sum(float(r["support_contacts"]) for r in self.rows) / len(self.rows),
            }
        path.write_text(json.dumps(summary, indent=2, allow_nan=True), encoding="utf-8")
        return path

    def make_plots(self) -> List[Path]:
        if not self.rows:
            return []
        try:
            import matplotlib.pyplot as plt
        except Exception as exc:
            print(f"Plot generation skipped: {exc}")
            return []

        t = [float(r["time_s"]) for r in self.rows]
        paths: List[Path] = []

        # 1) Attitude
        fig, ax = plt.subplots()
        ax.plot(t, [r["roll_deg"] for r in self.rows], label="roll")
        ax.plot(t, [r["pitch_deg"] for r in self.rows], label="pitch")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Angle [deg]")
        ax.set_title("Pebble body attitude")
        ax.legend()
        ax.grid(True, alpha=0.25)
        p = self.output_dir / "body_attitude.png"
        fig.tight_layout(); fig.savefig(p, dpi=160); plt.close(fig); paths.append(p)

        # 2) Motor current
        fig, ax = plt.subplots()
        for leg in LEGS:
            ax.plot(t, [r[f"{leg}_motor_current_A"] for r in self.rows], label=leg)
        ax.plot(t, [r["total_motor_current_A"] for r in self.rows], label="total", linewidth=2)
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Current [A]")
        ax.set_title("Estimated N20 motor current")
        ax.legend()
        ax.grid(True, alpha=0.25)
        p = self.output_dir / "motor_current.png"
        fig.tight_layout(); fig.savefig(p, dpi=160); plt.close(fig); paths.append(p)

        # 3) Foot normal forces
        fig, ax = plt.subplots()
        for leg in LEGS:
            ax.plot(t, [r[f"{leg}_foot_normal_N"] for r in self.rows], label=leg)
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Normal force [N]")
        ax.set_title("Foot-ground normal forces")
        ax.legend()
        ax.grid(True, alpha=0.25)
        p = self.output_dir / "foot_forces.png"
        fig.tight_layout(); fig.savefig(p, dpi=160); plt.close(fig); paths.append(p)

        # 4) Stability margin
        fig, ax = plt.subplots()
        vals = [r["stability_margin_m"] for r in self.rows]
        ax.plot(t, vals)
        ax.axhline(0.0, linewidth=1)
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Support-polygon margin [m]")
        ax.set_title("Static support-polygon stability margin")
        ax.grid(True, alpha=0.25)
        p = self.output_dir / "stability_margin.png"
        fig.tight_layout(); fig.savefig(p, dpi=160); plt.close(fig); paths.append(p)

        # 5) Contact slip speed: should stay low while a foot is loaded.
        fig, ax = plt.subplots()
        for leg in LEGS:
            ax.plot(t, [r.get(f"{leg}_slip_speed_m_s", 0.0) for r in self.rows], label=leg)
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Tangential foot speed [m/s]")
        ax.set_title("Foot slip while contacting floor")
        ax.legend()
        ax.grid(True, alpha=0.25)
        p = self.output_dir / "foot_slip.png"
        fig.tight_layout(); fig.savefig(p, dpi=160); plt.close(fig); paths.append(p)

        return paths
