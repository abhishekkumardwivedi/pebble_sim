# Pebble V3 — UX-First Concept-Accurate Mechanical Architecture

## 1. Design intent

Pebble V3 is a clean architecture revision, not a cosmetic patch to V2.5.

This is a **user-experience-first product**. The order of authority is:

1. recognizable Pebble silhouette and proportions;
2. expressive visible foot/leg behavior;
3. hard packaging feasibility;
4. internal structural skeleton;
5. soft packaging and DFM optimization.

The body is therefore not allowed to become a generic ellipsoid simply because a board or battery box is convenient to package. Conversely, the concept is not allowed to remain physically impossible: after the shell and leg architecture are established, the hard constraints must be checked immediately and the design iterated only where genuinely necessary.

The V2.5 assembly remains useful as a mechanism reference but is not the exterior product architecture.

---

## 2. UX geometry comes first

The first V3 CAD artifact is the **concept shell plus expressive leg envelope**, not an electronics packaging skeleton.

The shell must be built directly from the concept reference using controlled front, side and top silhouettes.

Target characteristics:

- approximately 165 mm overall body width;
- approximately 150 mm body depth;
- approximately 170 mm overall height including antennas;
- nominal belly clearance about 8 mm;
- broad lower-middle body volume;
- subtly narrowed crown;
- fuller front cheeks around the visor;
- softened / slightly flattened belly rather than a spherical bottom;
- smooth rear taper;
- no visible chassis deck;
- motors and upper linkage visually hidden;
- only the styled feet and the minimum required lower-leg neck visible.

The shell should be generated from multiple cross-sections / superellipse-like profiles or an equivalent controlled surface, not from one ellipsoid primitive.

### Shell modeling order

1. front silhouette sketch;
2. side silhouette sketch;
3. top/footprint sketch;
4. multiple Z-section profiles;
5. loft / surface body;
6. lower-belly shaping and ground-clearance target;
7. real visor recess and visor insert geometry;
8. antenna roots / appearance;
9. provisional shell thickness;
10. only later: internal bosses, seams and service openings.

The concept shell is judged by silhouette overlays before internal packaging is allowed to distort it.

---

## 3. Expressive leg architecture is designed with the shell

Pebble uses four independently controlled compact leg mechanisms, one per corner.

The visible experience matters more than exposing how the mechanism works. From the outside the user should see small soft-looking feet emerging from the body, not a mechanical platform or long exposed linkage.

Each leg is provisionally based on:

- compact N20-class encoder gearmotor inside the belly;
- crank drive;
- supported linkage pivot;
- compact closed-chain / four-bar geometry;
- short concealed lower link exiting through a shaped shell pocket;
- compliant rounded foot;
- optional passive ankle/compliance element.

The gearbox shaft must not be treated as the sole structural bearing for impact and radial loading. Final load paths will use chassis-supported pivots/bearings where required.

### Required expressive behaviors

The leg geometry must support, before electronics packaging is optimized:

- neutral stand;
- slow stable crawl;
- happy bounce;
- sleepy/shy crouch;
- alert/tall posture;
- left curiosity tilt;
- right curiosity tilt;
- forward bow/greeting;
- playful waddle;
- single-front-foot tap;
- asymmetric step / look-around pose.

These motions primarily come from coordinated relative phase and speed commands of the four legs rather than adding visibly humanoid joints.

---

## 4. Leg motion is validated against the UX shell

For one parameterized leg:

1. define the intended external foot trajectory;
2. solve / tune linkage lengths and crank placement to create that trajectory;
3. verify static support and required vertical body travel;
4. sweep the complete moving mechanism through the validated motion range;
5. create the hard-part swept volume;
6. add mechanical clearance;
7. compare that swept volume with the concept shell;
8. move pivots / reshape local underside pockets while preserving the exterior silhouette;
9. replicate and validate all four corners.

The shell and leg architecture therefore converge together.

We do **not** first freeze a large rectangular chassis and then lift the body to clear it.

---

## 5. Hard constraints are feasibility gates, not initial shape drivers

The known hard constraints in `pebble_v3_constraints.json` remain real and must be tested immediately after the concept shell and expressive leg geometry are credible.

Current hard items include:

1. battery energy/storage requirement;
2. Linux compute module + carrier;
3. ESP32-S3 real-time controller;
4. four encoder gearmotors;
5. motor/power regulation electronics;
6. visor/display volume;
7. camera volume;
8. speaker and microphone volumes;
9. service / charging connector access;
10. safe wiring and connector bend radii;
11. leg swept volumes and mechanical clearances.

The current battery baseline is a 2S1P 21700, 5 Ah / ~36 Wh class pack. The product controller baseline uses an ESP32-S3 module on a custom carrier rather than reserving a large development board. The Linux compute baseline is CM4/CM5-class mechanical volume.

These dimensions are **feasibility constraints**, not permission to deform the concept prematurely.

### Hard-constraint decision rule

When a hard item does not fit, iterate in this order:

1. rotate/reposition the component;
2. redesign its carrier / mounting;
3. exploit unused three-dimensional shell volume;
4. change equivalent component/package while preserving functional requirement;
5. alter internal leg/chassis architecture;
6. make a small local shell change invisible to the primary silhouette;
7. only as a last resort, change a major UX dimension — and record the reason.

This prevents convenience-driven geometry creep.

---

## 6. Internal skeleton comes after shell + legs + hard-fit proof

Only after the shell, leg trajectory and hard components fit do we design the real structural skeleton.

The skeleton then follows the already-proven geometry and provides:

- motor mounts;
- supported leg pivots/bearings;
- battery cradle;
- compute and controller mounts;
- visor/display support;
- speaker support;
- shell mounting bosses;
- structural load paths;
- cable routing;
- serviceability.

The skeleton is therefore an **enabling structure inside the experience envelope**, not the object around which the experience is wrapped.

---

## 7. Soft constraints follow

After hard-fit and skeleton validation, optimize:

- exact PCB placement;
- screw count and screw access;
- connector orientation;
- manufacturing splits;
- injection-molding draft / 3D-print allowances;
- acoustic port details;
- antenna RF keepouts;
- thermal conduction paths;
- assembly sequence;
- repair/service access;
- cable clipping and strain relief;
- final material thickness;
- cost and part-count reduction.

Soft constraints may adjust internal details but should not visibly damage the approved Pebble silhouette or expressive motion.

---

## 8. Revised CAD development gates

### Gate A — UX shell
Create the concept-accurate body, visor and feet as appearance geometry. Approve front, side and isometric silhouettes.

### Gate B — expressive leg trajectory
Define foot path, body travel and the target expression poses. Build one exact parameterized leg mechanism that produces them.

### Gate C — shell/leg convergence
Fit the leg mechanism inside the approved body, generate swept volumes, create hidden underside pockets, and validate all four legs with no exterior platform.

### Gate D — hard-constraint feasibility
Insert battery, motors, compute, MCU, power, display, camera and audio keepouts. Prove the hard requirements fit. Iterate internal architecture before changing the primary exterior.

### Gate E — structural skeleton
Design the internal chassis, supported pivots, battery cradle, PCB mounts and shell attachment structure around the already-approved shell/leg/hard-package solution.

### Gate F — kinematic and structural validation
Run all expression and locomotion poses, collision checks, support-polygon checks, preliminary torque/current calculations and major load-path checks.

### Gate G — soft packaging / DFM
Add wiring, bosses, seams, fasteners, thermal/RF/acoustic details, tolerances and service access.

### Gate H — simulation correlation
Export STEP/mesh and update MuJoCo geometry, masses, inertia, motor locations and motion limits from the physical CAD.

### Gate I — V3 release candidate
Approve appearance, motion, hard packaging, structural feasibility and manufacturability together.

---

## 9. Immediate next modeling task

The next CAD artifact is **`Pebble_V3_UX_Shell_Leg_Concept`**.

It should contain:

- concept-accurate exterior shell;
- real recessed visor geometry;
- antenna geometry;
- floor plane and 8 mm belly reference;
- four styled feet in neutral pose;
- one parameterized expressive leg mechanism;
- provisional motor position for that leg;
- target foot trajectory;
- key expression poses;
- generated leg swept-volume overlay;
- silhouette comparison views against the concept reference.

No battery/PCB/chassis block should be allowed to alter the approved external silhouette at this stage. They enter at Gate D as a hard feasibility test.
