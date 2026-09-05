from __future__ import annotations

import math

# -----------------------------------------------------------------------------
# Pebble V1.2 baseline parameters
# -----------------------------------------------------------------------------

TOTAL_MASS_KG = 0.700

# Body envelope.
BODY_LENGTH_M = 0.165
BODY_WIDTH_M = 0.145
BODY_HEIGHT_M = 0.110
BODY_ORIGIN_Z_M = 0.095

# -----------------------------------------------------------------------------
# Four-bar leg geometry
# -----------------------------------------------------------------------------

CRANK_R_M = 0.005
FRAME_D_M = 0.020
COUPLER_L_M = 0.038
ROCKER_Q_M = 0.024
FOOT_OFFSET_E_M = 0.022

# Initial crank phase that places the foot close to the floor.
BASE_CRANK_PHASE_RAD = math.radians(211.5)

# Motor mount positions relative to body origin.
FRONT_MOTOR_X_M = 0.030
REAR_MOTOR_X_M = -0.060
LEFT_MOTOR_Y_M = 0.055
RIGHT_MOTOR_Y_M = -0.055
MOTOR_MOUNT_Z_M = -0.0457

# -----------------------------------------------------------------------------
# Foot
# -----------------------------------------------------------------------------

FOOT_HALF_X_M = 0.015
FOOT_HALF_Y_M = 0.012
FOOT_HALF_Z_M = 0.0035

# -----------------------------------------------------------------------------
# Contact model
# -----------------------------------------------------------------------------

DEFAULT_FRICTION = 1.05

# Slightly compliant contact instead of a hard rigid impact.
CONTACT_SOLREF = "0.020 1"
CONTACT_SOLIMP = "0.88 0.96 0.002"

# -----------------------------------------------------------------------------
# N20 motor baseline
# -----------------------------------------------------------------------------

N20_VOLTAGE_V = 6.0
N20_NO_LOAD_RPM = 130.0
N20_NO_LOAD_RAD_S = N20_NO_LOAD_RPM * 2.0 * math.pi / 60.0
N20_STALL_TORQUE_NM = 0.0726
N20_STALL_CURRENT_A = 0.36
N20_NO_LOAD_CURRENT_A = 0.04
N20_MASS_KG = 0.011

# -----------------------------------------------------------------------------
# Gait / control
# -----------------------------------------------------------------------------

DEFAULT_GAIT = "crawl"

# V1/V1.1 were visually too aggressive.
DEFAULT_RPM = 16.0

# Let Pebble settle before motors begin moving.
SETTLE_TIME_S = 1.0

# Gradually spread the four leg phases instead of instantly jumping from
# all-aligned to crawl offsets.
PHASE_SPREAD_TIME_S = 5.0

# Gentler gains than the previous version.
PHASE_KP = 0.14
SPEED_KD = 0.08

# -----------------------------------------------------------------------------
# Motor-command smoothing
# -----------------------------------------------------------------------------

# Low-pass filtering time constant.
PWM_FILTER_TAU_S = 0.055

# Maximum PWM change per second.
# 2.5 means roughly 0 -> 1 in ~0.4 sec at the fastest.
PWM_SLEW_PER_S = 2.5

# Do not request motor shaft speed right at the theoretical no-load limit.
MAX_TARGET_SPEED_FRACTION = 0.88

# -----------------------------------------------------------------------------
# Stance/swing timing
# -----------------------------------------------------------------------------

# Current useful near-floor part of the linkage trajectory.
STANCE_THETA_START_RAD = math.radians(260.0)
STANCE_THETA_END_RAD = math.radians(160.0)

GAIT_DUTY_FACTOR = {
    "crawl": 0.74,
    "trot": 0.58,
    "bounce": 0.50,
}

# -----------------------------------------------------------------------------
# Passive ankle
# -----------------------------------------------------------------------------

# Slightly smaller travel and stronger damping than V1.1.
ANKLE_RANGE_RAD = math.radians(18.0)
ANKLE_DAMPING = 0.0060
ANKLE_STIFFNESS = 0.035

# -----------------------------------------------------------------------------
# Mass distribution
# -----------------------------------------------------------------------------

# Keep the shell light and mass low in the belly.
SHELL_MASS_KG = 0.100
LOW_CHASSIS_MASS_KG = 0.300
LOW_CHASSIS_Z_M = -0.020

# -----------------------------------------------------------------------------
# Telemetry
# -----------------------------------------------------------------------------

TELEMETRY_HZ = 100.0
CONTACT_FORCE_THRESHOLD_N = 0.05

# -----------------------------------------------------------------------------
# Leg naming / gait offsets
# -----------------------------------------------------------------------------

LEGS = ("FL", "FR", "RL", "RR")

GAIT_OFFSETS = {
    # Four-beat crawl.
    "crawl": {
        "FL": 0.0,
        "RR": 0.5 * math.pi,
        "FR": math.pi,
        "RL": 1.5 * math.pi,
    },
    # Diagonal pairs.
    "trot": {
        "FL": 0.0,
        "RR": 0.0,
        "FR": math.pi,
        "RL": math.pi,
    },
    # Diagnostic mode.
    "bounce": {leg: 0.0 for leg in LEGS},
}
