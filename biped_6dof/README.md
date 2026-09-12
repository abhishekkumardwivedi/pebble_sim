# Pebble 6-DOF Biped MuJoCo Demonstrator

This is a deliberately simple **12-servo biped** intended to make the leg kinematics obvious before we design the final Pebble/Pabble shell and mechanical packaging.

Each leg is:

`Pelvis -> Hip Yaw -> Hip Roll -> Hip Pitch -> Thigh -> Knee Pitch -> Shin -> Ankle Pitch -> Ankle Roll -> Foot`

That is **6 DOF per leg, 12 actuators total**.

## Why the pelvis is fixed

The first goal is to understand what the motors do. A free-standing biped requires an IMU feedback loop, centre-of-mass/ZMP or capture-point logic, foot-contact handling and a balance controller. Those can make it hard to see the raw kinematics.

This model therefore keeps the pelvis fixed while the real MuJoCo joints, actuators, gravity and foot contacts are simulated. Once the motion looks correct, the next step is to free the pelvis and add the balance controller.

## Motor map

| Motor | Joint | Purpose |
|---|---|---|
| M1 | Hip yaw | Turn leg/toes inward and outward; turning |
| M2 | Hip roll | Lateral leg motion; weight shift/balance |
| M3 | Hip pitch | Forward/backward thigh swing; stride |
| M4 | Knee pitch | Leg folding; foot clearance |
| M5 | Ankle pitch | Toe up/down; landing and push-off |
| M6 | Ankle roll | Sole tilt; keep foot flat during lateral balance |

The same six are mirrored on the right leg.

## Install

From this folder:

```bash
python -m pip install -r requirements.txt
```

## Run

```bash
python biped_demo.py
```

On macOS, MuJoCo passive-viewer applications may require:

```bash
mjpython biped_demo.py
```

## Keyboard controls

- `1` — stand
- `2` — motor-by-motor demonstration; cycles M1 through M6
- `3` — coordinated stepping gait; all 12 actuators participate
- `4` — squat; demonstrates hip/knee/ankle pitch coordination
- `R` — reset
- `SPACE` — pause/resume

The terminal also prints the currently demonstrated motor and what it contributes.

## Visual convention

The colored cylinders at each joint are the servo-axis placeholders. At the hip, three cylinders intersect because a 3-DOF hip is modeled as three serial rotational axes at essentially the same joint centre. The ankle similarly has pitch and roll axes.

This is **not yet the production mechanical packaging**. It is the kinematic architecture we can use to decide which DOFs Pebble actually needs before selecting servo sizes and designing brackets in FreeCAD.
