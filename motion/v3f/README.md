# Pebble V3-F — Mechanically Resolved Six-Motor Architecture

V3-F addresses the main weaknesses found in V3-E: floating cosmetic paws, undefined connection of the two splay motors, excessive shell-opening uncertainty, and impact loads passing too directly through small actuator gearboxes.

## Source of truth

- `cad/generate_v3f.py` — parametric CadQuery generator for the V3-F mechanical review assemblies.
- `shared/v3f_mechanical_spec.json` — motor, linkage, spring, material and expression parameters.
- `docs/V3F_MECHANICAL_ARCHITECTURE.md` — mechanical/load-path explanation.
- `validation/V3F_CLEARANCE_AND_LOADPATH.json` — current swept-volume and overload targets.
- `freecad/Pebble_V3F_Review.FCMacro` — FreeCAD review helper.
- `mujoco/pebble_v3f.xml` — MuJoCo V3-F physics scaffold.
- `mujoco/simulate_v3f.py` — expression playback / simulation bridge.
- `releases/V3F_GENERATED_ARTIFACTS.md` — checksums for the generated STEP review files.

## CAD

Install the existing project CAD dependencies, then from the repository root:

```bash
python motion/v3f/cad/generate_v3f.py
```

Generated STEP review files are written under `motion/v3f/cad/generated/` by default. Generated STEP files are intentionally not treated as source code; regenerate them from the committed parametric model.

## FreeCAD

Open the generated neutral/cutaway STEP and use `freecad/Pebble_V3F_Review.FCMacro` for review. FreeCAD is used for mechanism/clearance visualization, not for final contact physics.

## MuJoCo

```bash
pip install mujoco numpy
python motion/v3f/mujoco/simulate_v3f.py --demo
```

The current MuJoCo file is a physics scaffold for the six-actuator architecture. Dynamic torque, contact, friction and overload validation remain an engineering gate before manufacturing.

## Current actuator baseline

- 4 x GA12/N20 6 V ~150 RPM encoder gearmotors, externally reduced 12T:18T to the crank shaft.
- 2 x MG90S metal-gear micro servos for front/rear mirrored splay.
- 2 x 623ZZ-class support bearings per crank shaft.
- twin preloaded cassette springs per leg plus structural hard stops into the belly ring.
- spring-loaded servo-saver sections in the front/rear splay pushrods.

V3-F is **simulation-ready architecture**, not production qualification. Final motor lot, spring rate, material, fatigue and child/impact load robustness require measured parts, FEA and physical proof testing.
