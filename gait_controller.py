from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict

import mujoco

from config import (
    BASE_CRANK_PHASE_RAD,
    DEFAULT_GAIT,
    DEFAULT_RPM,
    GAIT_DUTY_FACTOR,
    GAIT_OFFSETS,
    LEGS,
    MAX_TARGET_SPEED_FRACTION,
    N20_NO_LOAD_RAD_S,
    PHASE_KP,
    PHASE_SPREAD_TIME_S,
    PWM_FILTER_TAU_S,
    PWM_SLEW_PER_S,
    SETTLE_TIME_S,
    SPEED_KD,
    STANCE_THETA_END_RAD,
    STANCE_THETA_START_RAD,
)
from n20_motor import MotorSample, N20MotorModel, clamp


def smootherstep01(x: float) -> float:
    """Quintic 0..1 easing with zero velocity and acceleration at the ends."""
    x = clamp(x, 0.0, 1.0)
    return x * x * x * (x * (x * 6.0 - 15.0) + 10.0)


def smootherstep01_derivative(x: float) -> float:
    """Derivative of smootherstep01 with respect to x."""
    if x <= 0.0 or x >= 1.0:
        return 0.0
    return 30.0 * x * x * (x - 1.0) * (x - 1.0)


@dataclass
class LegControlState:
    desired_phase_rad: float
    actual_phase_rad: float
    phase_error_rad: float
    target_speed_rad_s: float
    shaft_speed_rad_s: float
    in_commanded_stance: bool
    motor: MotorSample


class GaitController:
    """
    Smooth phase-based four-crank gait controller.

    V1.2 changes:
      * smooth quintic stance/swing interpolation
      * gentler controller gains
      * slower startup and phase spreading
      * motor PWM low-pass filtering
      * PWM slew limiting
      * shaft-speed limiting
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        gait: str = DEFAULT_GAIT,
        rpm: float = DEFAULT_RPM,
        phase_kp: float = PHASE_KP,
        speed_kd: float = SPEED_KD,
        duty_factor: float | None = None,
    ) -> None:
        if gait not in GAIT_OFFSETS:
            raise ValueError(f"Unknown gait {gait!r}. Choose from {sorted(GAIT_OFFSETS)}")

        self.gait = gait
        self.rpm = float(rpm)
        self.cycle_hz = self.rpm / 60.0
        self.phase_kp = float(phase_kp)
        self.speed_kd = float(speed_kd)
        self.duty_override = duty_factor
        self.motor_model = N20MotorModel()

        # Joint q grows positive while geometric theta decreases because the
        # generated MJCF uses phi=-theta.
        self.q_stance_start = BASE_CRANK_PHASE_RAD - STANCE_THETA_START_RAD
        self.q_stance_end = BASE_CRANK_PHASE_RAD - STANCE_THETA_END_RAD
        if not self.q_stance_end > self.q_stance_start:
            raise ValueError("Stance crank interval must advance in positive joint direction")

        self.stance_span = self.q_stance_end - self.q_stance_start
        self.swing_span = 2.0 * math.pi - self.stance_span

        self.qpos_adr: Dict[str, int] = {}
        self.dof_adr: Dict[str, int] = {}
        self.actuator_ids: Dict[str, int] = {}

        for leg in LEGS:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{leg}_crank_joint")
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"motor_{leg}")
            if jid < 0 or aid < 0:
                raise RuntimeError(f"Could not resolve joint/actuator for {leg}")

            self.qpos_adr[leg] = int(model.jnt_qposadr[jid])
            self.dof_adr[leg] = int(model.jnt_dofadr[jid])
            self.actuator_ids[leg] = aid

        self._last_time: float | None = None
        self._filtered_pwm: Dict[str, float] = {leg: 0.0 for leg in LEGS}

    def set_rpm(self, rpm: float) -> None:
        self.rpm = float(rpm)
        self.cycle_hz = self.rpm / 60.0

    def set_gait(self, gait: str) -> None:
        if gait not in GAIT_OFFSETS:
            raise ValueError(f"Unknown gait {gait!r}")
        self.gait = gait

    def _duty(self) -> float:
        d = self.duty_override if self.duty_override is not None else GAIT_DUTY_FACTOR[self.gait]
        return clamp(float(d), 0.40, 0.84)

    def _u_at_q_zero(self, duty: float) -> float:
        # Find the virtual gait-cycle point corresponding to q=0 so startup does
        # not command a large phase jump.
        f = (0.0 - self.q_stance_start) / self.stance_span
        return duty * clamp(f, 0.0, 1.0)

    def _joint_target_for_cycle_phase(self, s: float, duty: float) -> tuple[float, float, bool]:
        """
        Convert continuous gait cycles into continuous crank-joint position.

        Returns:
            desired_q
            dq/ds
            commanded stance state
        """
        n = math.floor(s)
        u = s - n

        if u < duty:
            a = u / duty
            eased = smootherstep01(a)
            derivative = smootherstep01_derivative(a)
            q_mod = self.q_stance_start + self.stance_span * eased
            dq_ds = self.stance_span * derivative / duty
            stance = True
        else:
            a = (u - duty) / (1.0 - duty)
            eased = smootherstep01(a)
            derivative = smootherstep01_derivative(a)
            q_mod = self.q_stance_end + self.swing_span * eased
            dq_ds = self.swing_span * derivative / (1.0 - duty)
            stance = False

        return n * 2.0 * math.pi + q_mod, dq_ds, stance

    def _filter_pwm(self, leg: str, requested: float, dt: float) -> float:
        requested = clamp(requested, 0.0, 1.0)
        previous = self._filtered_pwm[leg]

        tau = max(PWM_FILTER_TAU_S, 1e-6)
        alpha = dt / (tau + dt)
        filtered = previous + alpha * (requested - previous)

        max_delta = max(PWM_SLEW_PER_S, 0.01) * dt
        filtered = clamp(filtered, previous - max_delta, previous + max_delta)
        filtered = clamp(filtered, 0.0, 1.0)

        self._filtered_pwm[leg] = filtered
        return filtered

    def update(self, data: mujoco.MjData) -> Dict[str, LegControlState]:
        now = float(data.time)
        if self._last_time is None:
            dt = 0.002
        else:
            dt = clamp(now - self._last_time, 1e-5, 0.05)
        self._last_time = now

        move_t = max(0.0, now - SETTLE_TIME_S)
        duty = self._duty()
        offsets = GAIT_OFFSETS[self.gait]

        spread_x = move_t / max(PHASE_SPREAD_TIME_S, 1e-6)
        spread = smootherstep01(spread_x)
        if 0.0 < spread_x < 1.0:
            dspread_dt = smootherstep01_derivative(spread_x) / max(PHASE_SPREAD_TIME_S, 1e-6)
        else:
            dspread_dt = 0.0

        u0 = self._u_at_q_zero(duty)
        states: Dict[str, LegControlState] = {}
        max_target_speed = MAX_TARGET_SPEED_FRACTION * N20_NO_LOAD_RAD_S

        for leg in LEGS:
            q = float(data.qpos[self.qpos_adr[leg]])
            omega = float(data.qvel[self.dof_adr[leg]])
            offset_cycles = float(offsets[leg]) / (2.0 * math.pi)

            s = u0 + self.cycle_hz * move_t + spread * offset_cycles
            ds_dt = self.cycle_hz + dspread_dt * offset_cycles

            desired_q, dq_ds, stance = self._joint_target_for_cycle_phase(s, duty)
            desired_omega = clamp(dq_ds * ds_dt, 0.0, max_target_speed)

            phase_error = desired_q - q
            speed_error = desired_omega - omega

            pwm_ff = desired_omega / N20_NO_LOAD_RAD_S
            requested_pwm = (
                pwm_ff
                + self.phase_kp * phase_error
                + self.speed_kd * (speed_error / N20_NO_LOAD_RAD_S)
            )

            if move_t <= 0.0:
                requested_pwm = 0.0

            pwm = self._filter_pwm(leg, requested_pwm, dt)
            motor_sample = self.motor_model.evaluate(pwm, omega)
            data.ctrl[self.actuator_ids[leg]] = motor_sample.torque_nm

            states[leg] = LegControlState(
                desired_phase_rad=desired_q,
                actual_phase_rad=q,
                phase_error_rad=phase_error,
                target_speed_rad_s=desired_omega,
                shaft_speed_rad_s=omega,
                in_commanded_stance=stance,
                motor=motor_sample,
            )

        return states
