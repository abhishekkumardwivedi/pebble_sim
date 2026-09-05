from __future__ import annotations

from dataclasses import dataclass

from config import (
    N20_NO_LOAD_CURRENT_A,
    N20_NO_LOAD_RAD_S,
    N20_STALL_CURRENT_A,
    N20_STALL_TORQUE_NM,
    N20_VOLTAGE_V,
)


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


@dataclass(frozen=True)
class MotorSample:
    pwm: float
    shaft_speed_rad_s: float
    torque_nm: float
    current_a: float
    electrical_power_w: float
    mechanical_power_w: float


class N20MotorModel:
    """
    First-order output-shaft model for a 6 V N20-class geared DC motor.

    Torque-speed approximation at the *gearbox output*:
        tau = tau_stall * (u - omega / omega_no_load)

    where u is normalized effective H-bridge voltage in [-1, 1]. This is a
    deliberately transparent engineering approximation; replace the constants
    with the exact vendor motor curve once the final motor is selected.
    """

    def __init__(
        self,
        voltage_v: float = N20_VOLTAGE_V,
        no_load_speed_rad_s: float = N20_NO_LOAD_RAD_S,
        stall_torque_nm: float = N20_STALL_TORQUE_NM,
        stall_current_a: float = N20_STALL_CURRENT_A,
        no_load_current_a: float = N20_NO_LOAD_CURRENT_A,
    ) -> None:
        self.voltage_v = float(voltage_v)
        self.omega0 = float(no_load_speed_rad_s)
        self.tau_stall = float(stall_torque_nm)
        self.i_stall = float(stall_current_a)
        self.i0 = float(no_load_current_a)

    def evaluate(self, pwm: float, shaft_speed_rad_s: float) -> MotorSample:
        u = clamp(float(pwm), -1.0, 1.0)
        omega = float(shaft_speed_rad_s)

        # Empirical linear torque-speed line. Allow a braking/regeneration sign
        # when the shaft outruns the commanded electrical speed.
        tau = self.tau_stall * (u - omega / self.omega0)
        tau = clamp(tau, -self.tau_stall, self.tau_stall)

        # Approximate current magnitude. At zero torque we retain a duty-scaled
        # no-load current; at stall torque we approach the stall current.
        load_fraction = min(1.0, abs(tau) / self.tau_stall)
        current = abs(u) * self.i0 + load_fraction * (self.i_stall - self.i0)

        electrical = abs(u) * self.voltage_v * current
        mechanical = tau * omega
        return MotorSample(
            pwm=u,
            shaft_speed_rad_s=omega,
            torque_nm=tau,
            current_a=current,
            electrical_power_w=electrical,
            mechanical_power_w=mechanical,
        )
