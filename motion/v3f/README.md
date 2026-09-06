# Pebble V3-F — Mechanical CAD Validation Loop

V3-F is now focused on **mechanical CAD validation for expression/personality, clearance and load path**. MuJoCo, electronics packaging, detailed BOM work and controller implementation are deliberately deferred until the mechanism is mechanically convincing.

## Source of truth

- `cad/generate_v3f.py` — parametric V3-F CAD generator.
- `shared/v3f_mechanical_spec.json` — motor, linkage, spring, material and named pose parameters.
- `shared/iteration_config.json` — sentinel poses, short review transitions, render and clearance policy.
- `shared/baseline_metrics.json` — frozen current-version regression baseline for delta reporting.
- `tools/validate_v3f.py` — fast kinematic/workspace/load-path regression.
- `tools/render_step.py` — deterministic CAD/STEP endpoint renders; no image-generation model.
- `tools/render_motion.py` — short CAD-derived expression clips. Each frame injects interpolated actuator targets into the parametric CAD generator and rebuilds the actual solids.
- `tools/run_iteration.py` — one-command iteration loop.
- `validation/` — earlier V3-F clearance/load-path record.
- `freecad/Pebble_V3F_Review.FCMacro` — optional FreeCAD review helper.

## Normal iteration

From the repository root:

```bash
python motion/v3f/tools/run_iteration.py
```

This regenerates the CAD, runs the fast regression, renders the sentinel endpoint poses and renders the two short review motions configured in `iteration_config.json`.

For a structural-only tweak where motion images are unnecessary:

```bash
python motion/v3f/tools/run_iteration.py --skip-motion
```

For review of existing STEP files without regenerating CAD:

```bash
python motion/v3f/tools/run_iteration.py --skip-cad
```

Generated outputs live under `motion/v3f/output/` and are disposable; the parameter/specification files above remain the source of truth.

The deterministic render path uses CadQuery/OpenCascade + VTK, with Pillow/imageio for review sheets and GIFs. FreeCAD does not need to be opened manually for routine review.

## Current quick acceptance set

The normal loop intentionally checks only four sentinel expressions on every iteration:

- `sleepy_compact` — tucked-paw/minimum-height envelope.
- `happy_wide` — large splay / shell-opening envelope.
- `play_bow` — front/rear differential and pitch.
- `curious_left` — asymmetric left/right articulation and roll.

Only two short CAD motion previews (`happy_open` and `curious_left`) are generated during a normal review. The full personality suite is reserved for milestones, avoiding repeated regeneration when only one mechanical dimension changes.

## Current mechanical baseline

- 4 × GA12/N20-class 6 V encoder gearmotors.
- external 12T:18T m0.5 reduction to separately supported 3 mm crank shafts.
- 2 × MG90S-class metal-gear servos for front/rear mirrored splay.
- twin preloaded cassette springs per leg + structural hard stops into the belly ring.
- spring-loaded splay servo-saver path.
- soft gaiter/paw transition intended to hide the rigid shell aperture.

The quick validator reports analytical ankle workspace, recommended hidden aperture, expression body attitude, spring/hard-stop load path and deltas from the frozen baseline. Exact B-Rep collision sweeps and FEA are separate gates and are run when geometry/load paths change materially rather than on every cosmetic iteration.

## Deliberately not active yet

MuJoCo dynamics, electronics packaging, detailed wiring, full BOM sourcing, firmware/control work and production DFM are deferred. They can be reactivated after CAD motion, clearance, load-path and critical stress checks pass.

## Known open mechanical item

Antenna articulation is not yet physically modeled in V3-F. It remains a required personality DOF and should be added after the current six-actuator leg/body mechanism clears the first CAD regression loop.
