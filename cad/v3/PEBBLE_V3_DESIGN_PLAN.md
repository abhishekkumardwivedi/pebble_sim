# Pebble V3 — Concept-Accurate Mechanical Architecture

## 1. Design intent

Pebble V3 is a clean mechanical architecture revision, not a cosmetic patch to V2.5.

Primary objective:
- preserve the original Pebble creature silhouette;
- hide motors, upper leg linkages and chassis inside the body;
- expose only soft-looking feet / minimal leg necks;
- retain real walking and expressive body motion;
- package battery, compute, MCU, audio and vision inside a manufacturable enclosure.

The V2.5 assembly remains useful as a linkage/mechanism reference but is no longer the exterior product architecture.

---

## 2. Hard constraints frozen before surfacing

See `pebble_v3_constraints.json`.

The CAD must first contain non-negotiable keepout bodies for:

1. 2S1P 21700 Li-ion battery pack, 5 Ah class
2. Linux compute module + custom carrier bay
3. ESP32-S3 real-time controller module
4. motor/power regulation PCB bay
5. four N20-class encoder gearmotor envelopes
6. four leg swept volumes
7. face display/visor volume
8. camera volume
9. speaker + microphone volumes
10. USB-C/service access
11. shell wall thickness, bosses and cable corridors

No cosmetic surface is accepted until these fit without overlap.

---

## 3. Why four independent leg actuators

The Pebble concept benefits from four independently phase-controlled compact leg mechanisms rather than one shared axle or a differential-drive chassis.

One actuator per corner allows the software to control relative leg phase and therefore produce both locomotion and expression with the same hardware.

The motor does not need a full humanoid 2-DOF leg. A compact closed-chain/four-bar mechanism can convert continuous crank rotation into a repeatable foot trajectory.

The key expression variable is not only motor speed; it is the relative phase of each leg.

Examples:

### Neutral stand
All feet at approximately equal support height.

### Walk / crawl
Four crank phases offset to maintain a high duty factor and low body disturbance.

### Happy bounce
All four legs move largely in phase, with a low-amplitude vertical body oscillation.

### Curious tilt left
Right-side legs extend slightly while left-side legs crouch, producing a small body roll.

### Curious tilt right
Mirror of left tilt.

### Bow / greeting
Front legs crouch while rear legs extend slightly.

### Proud / alert posture
Front and rear legs move toward the high-support part of the trajectory, raising the belly within mechanical limits.

### Shy / sleepy crouch
All legs move toward the low-support part of the trajectory.

### Foot tap
One front leg is phase-jogged around a short local trajectory while the other three remain in stable support.

### Waddle / playful walk
Left/right timing and amplitude are deliberately biased while retaining static support margin.

These motions are achieved by coordinated phase targets and speed profiles. They do not require visibly articulated humanoid knees.

---

## 4. Leg module architecture

Each corner receives a compact removable leg cassette:

- N20 encoder gearmotor mounted horizontally within the belly;
- small crank on gearbox output;
- supported linkage pivot on the structural chassis;
- coupler/rocker arranged as a compact four-bar;
- short lower link exits through a shaped shell pocket;
- compliant rounded foot forms the visible product element;
- optional passive ankle/compliance element isolates impact and helps keep the foot visually flat.

Important mechanical rule:

The gearbox output shaft should not be treated as the sole structural bearing for side loads and impacts. The linkage load path should be supported by a chassis-mounted pivot/bearing or by a supported crank arrangement where practical.

The leg cassette must be removable without dismantling the battery or face electronics.

---

## 5. Leg swept-volume workflow

The new shell is not created first and then cut when collisions occur.

For each leg:

1. build the exact mechanism skeleton;
2. rotate it through a full 360-degree crank cycle;
3. union/sweep all moving hard-part volumes;
4. add minimum 2 mm mechanical clearance;
5. retain that result as a `LEG_SWEEP_KEEPOUT` solid;
6. mirror to other corners after confirming handedness;
7. construct the cosmetic underside around these solids.

This automatically creates the required hidden wheel-arch / leg-pocket geometry without exposing the complete mechanism.

---

## 6. Body geometry workflow

The V3 shell should be generated from controlled cross-sections, not a single ellipsoid.

Target shape characteristics:

- width approximately 165 mm;
- depth approximately 150 mm;
- broad lower-middle mass;
- subtly narrowed crown;
- flattened / softened belly for 8 mm nominal ground clearance;
- fuller front cheeks around the visor;
- smooth rear taper;
- no visible chassis deck;
- continuous outer shell down around the leg pockets.

Recommended CAD construction:

1. Front orthographic silhouette sketch
2. Side orthographic silhouette sketch
3. Top/bottom footprint sketch
4. Multiple Z-section rounded-squircle / superellipse profiles
5. Loft or SubD-like surface construction from these sections
6. controlled lower-belly flattening
7. visor recess boolean / surface trim
8. shell thickness
9. leg swept-volume pocket subtraction
10. seam, bosses and service-door details

The shell should be judged primarily by front and side silhouette overlays against the concept reference before internal cosmetic details are added.

---

## 7. Face / visor architecture

The black face region is a real mechanical feature, not a texture.

It needs:

- a broad rounded rectangular / organic visor surface;
- shallow recess into the shell;
- internal display plane behind smoked acrylic/polycarbonate;
- camera aperture hidden within the black region;
- enough internal volume to avoid forcing the whole body into a spherical form.

The visor is one of the main visual anchors of the concept and should be modeled early in V3.

---

## 8. Battery packaging

Baseline is a 2S1P 21700 pack, approximately 7.2 V nominal, 5 Ah / 36 Wh class.

Placement:

- horizontal;
- as low and central as possible;
- mechanically retained in an insulated cradle;
- separated from motor/linkage sweep zones;
- accessible after removal of the bottom/service panel;
- thermal separation from high-loss regulators and compute module.

With 36 Wh nominal energy, illustrative system runtime is approximately:

- 4.5 h at 8 W average;
- 3.6 h at 10 W;
- 3.0 h at 12 W;
- 2.6 h at 14 W.

These are energy-budget estimates, not guaranteed product runtimes; actual usable Wh, conversion losses, locomotion duty cycle and battery safety margins must be applied later.

---

## 9. Electronics packaging

### ESP32-S3
Use an ESP32-S3-WROOM module on a custom carrier in the product CAD rather than reserving the large development board.

The prototype may use a DevKit externally/temporarily, but the internal mechanical product envelope should be based on the module + carrier architecture.

### Linux compute
Reserve a 55 x 40 mm compute-module class envelope plus carrier and connector keepouts. The compute board should sit higher than the battery but away from the moving leg roots.

### Power
A 2S battery bus requires dedicated regulation:

- regulated motor rail;
- regulated 5 V compute rail;
- 3.3 V logic where required.

The CAD must include connector bend radius and wiring volume, not only PCB outlines.

---

## 10. CAD development gates

### Gate A — package blocks
Only boxes/cylinders representing real components. Verify everything fits inside the target body.

### Gate B — exact leg skeleton
Build one actual leg and verify intended foot trajectory and torque requirements.

### Gate C — four-leg kinematics
Create all four mechanisms and run the validation pose set.

### Gate D — swept keepouts
Generate hard swept volumes and clearance bodies.

### Gate E — concept shell
Create the pebble-shaped external surface around the frozen package/keepout geometry.

### Gate F — visor and underside
Add real visor recess and integrate leg pockets into the belly.

### Gate G — structural chassis
Add motor mounts, bearing supports, battery cradle, PCB mounts and load paths.

### Gate H — DFM / assembly
Split shell, add bosses/screws/snaps, cable routes, service access, tolerances and fastener tool access.

### Gate I — simulation correlation
Export STEP/mesh and update MuJoCo geometry, masses, inertia, motor positions and motion limits from the physical CAD.

Only after Gate I should the external design be considered V3.0 release-candidate geometry.

---

## 11. Immediate next modeling task

Do not start by sculpting a beautiful shell.

The next CAD artifact should be `Pebble_V3_Packaging_Skeleton` containing:

- external reference envelope;
- battery keepout;
- compute/carrier keepout;
- ESP32/power keepouts;
- visor/display/camera keepouts;
- four motor blocks;
- exact leg pivot coordinates;
- one fully parameterized leg linkage;
- four copied/mirrored leg mechanisms;
- generated leg swept keepout volumes;
- floor plane and 8 mm belly target plane.

That skeleton is the foundation for the new shell.
