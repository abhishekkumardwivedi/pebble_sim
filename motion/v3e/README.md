# Pebble V3-E Six-Motor Motion Proof

This package is the bridge between the frozen Pebble exterior, visible CAD kinematics, and MuJoCo motion validation. The current question is deliberately narrow: **can four main leg motors plus two shared splay motors create the intended pet-like body language inside the V3-E body envelope?**

## Mechanical architecture represented here

- 4 × N20-class 6 V encoder gearmotors, one per leg. Visual envelope: about **34 × 12 × 10 mm including encoder allowance**, with 3 mm output shaft.
- 2 × metal-gear digital micro-servo-class splay actuators, one front and one rear. Envelope: about **23.5 × 12 × 24 mm**.
- Main four-bar geometry: crank 4 mm, frame 17 mm, coupler 34 mm, rocker 22 mm, 8 mm foot extension.
- Main motor current simulation baseline remains 130 rpm no-load / 0.0726 Nm stall / 0.36 A stall until a final vendor part is selected. The product torque target remains >=0.10 Nm.
- Motion playback is rate-limited to an 85 rpm loaded main-motor ceiling and 500 deg/s splay-servo ceiling. The supplied expression sequence peaks well below those limits.
- Passive paw compliance is represented by keeping the cosmetic paw flat while the linkage ankle moves/tilts above it.

## What FreeCAD is for

Open `cad/generated/Pebble_V3E_Motion_Neutral.step` first to inspect the complete six-motor packaging and linkage geometry. The `cad/generated/poses/` folder contains static STEP snapshots of the key expressions.

Then run `cad/freecad/Pebble_V3E_Motion.FCMacro` from FreeCAD. It imports the product body and animates the same expression targets and motor-speed limits stored in `shared/motion_spec.json`. Set `SHELL_TRANSPARENCY=0` for the exterior character view or ~60–75 for a kinematic cutaway.

**FreeCAD is not the physics authority.** It is excellent for exact CAD geometry, assembly interference, swept-volume checks, and kinematic animation. A FreeCAD assembly workbench can also constrain joints, but contact/friction/tip dynamics and motor-load validation are better done in MuJoCo.

## What MuJoCo is for

Install `mujoco` and run:

```bash
python mujoco/simulate_v3e_motion.py
```

The supplied MuJoCo stage is intentionally a **kinematic motion proof**: it loads the actual V3-E shell/visor/antenna meshes and draws the analytical four-bars, N20 envelopes, paws, splay servos and horns from the same shared specification. This is the right stage for judging expression timing, geometry and body-language range before committing to the internal chassis.

The next model revision should convert the analytical linkage/contact representation into a fully dynamic constrained four-bar with paw friction/compliance, measured mass/inertia and the electrical motor model. The current package is structured so the shared motion spec and CAD assets carry forward directly.

## Expression vocabulary included

`neutral`, `sleepy_crouch`, `alert`, `curious_left/right`, `happy_wide`, `bounce_peak`, `play_bow`, `wide_flat`, `inward_compact`, and `walk_diagonal`.

The demo sequence is intentionally about character first, locomotion second.

## Folder map

- `shared/motion_spec.json` — single source of truth for geometry, 6 motor envelopes, speed limits, poses and timing.
- `shared/kinematics.py` — pure-Python analytical four-bar + splay + body-pose solver and rate-limited timeline.
- `cad/pebble_v3e_generate.py` — current parametric exterior/mechanism CAD source used to generate this package.
- `cad/build_motion_cad.py` — generates body assets, six-motor neutral/cutaway STEP assemblies and pose STEP snapshots.
- `cad/freecad/Pebble_V3E_Motion.FCMacro` — FreeCAD animation using the shared spec.
- `mujoco/pebble_v3e_kinematic.xml` — MuJoCo scene with the actual exterior meshes.
- `mujoco/simulate_v3e_motion.py` — same expression timeline in MuJoCo.
- `preview/` — rendered motion-proof preview.

## Current engineering status

This package validates **kinematic feasibility and packaging envelopes**, not final motor sizing or durability. Before prototype manufacture we still need full dynamic MuJoCo closure/contact physics, exact selected-motor curves, chassis bearings, splay coupling detail, collision/swept-volume checks, and internal battery/electronics packaging.
