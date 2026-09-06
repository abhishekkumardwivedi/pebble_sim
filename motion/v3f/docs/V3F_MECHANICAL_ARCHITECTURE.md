# Pebble V3-F Mechanical Architecture

V3-F turns the V3-E expression concept into a mechanically explicit architecture intended for CAD clearance review and MuJoCo model development.

## Six active actuators

- **M1-M4:** one GA12/N20 6 V encoder gearmotor per paw.
- **M5:** one MG90S metal-gear micro servo for symmetric front-pair splay.
- **M6:** one MG90S metal-gear micro servo for symmetric rear-pair splay.

The four N20 motors control independent four-bar crank phase. The two splay servos do not carry normal body weight; each drives a central bellcrank and two mirrored pushrods to rotate the associated left/right leg cassettes by approximately +/-10 degrees.

## Main-motor load path

The N20 gearbox shaft is not used as the structural crank bearing. The motor drives a 12T module-0.5 pinion into an 18T crank gear. The crank gear is carried on a separate 3 mm stainless shaft supported by two 623ZZ-class 3x10x4 mm bearings in the cassette frame.

This gives 1.5:1 external reduction and isolates radial/impact loads from the N20 gearbox output bearing.

## Four-bar baseline

- crank radius: 5 mm
- fixed-frame distance: 16 mm
- coupler: 28 mm
- rocker: 17 mm
- foot extension: 10 mm
- useful crank range: 220-310 degrees

The useful range is intentionally constrained rather than rotating the crank through 360 degrees for every expression.

## Continuous paw / ankle skin

The rigid linkage ends in a small ankle stem. A flexible TPU/TPE gaiter bridges from an internal flange around the belly aperture to the cosmetic paw neck. The gaiter is both a visual skin and a debris guard; it is not the primary structural link.

The product intent is that an outside observer sees one continuous visual form: shell -> soft ankle transition -> paw. Crank, links, bearings, cassette and rigid aperture remain hidden.

## Overload protection

Each leg cassette is mounted through two preloaded compression springs with 5 mm maximum retractable travel. The current target is 5 N/mm per spring with 2 N preload per spring, giving approximately 54 N per leg at the hard-stop transition.

After spring travel is exhausted, a broad hard-stop pad transfers additional vertical load into a continuous GF-nylon belly ring and distributed shell bosses. The intended load path is therefore:

`paw -> ankle/linkage -> cassette -> springs -> hard stop -> belly ring -> shell/chassis`

rather than `paw -> crank -> motor gearbox`.

This is actuator protection for falls and accidental overloads. It is not a claim that the robot is a step or climbing surface for a child; physical proof testing and FEA are required before any load rating.

## Splay servo saver

Each MG90S drives a symmetric bellcrank. Both pushrods include a short spring-loaded servo-saver section (~2 N/mm, 1.5 N preload, 3 mm travel target). A lateral impact can therefore deflect a paw/cassette without forcing the entire impulse through the MG90S gear train.

## Swept-volume target

The current CAD/analytical envelope for useful crank travel and +/-10 degree splay is approximately 15.8 mm fore/aft x 12.1 mm lateral at the ankle. A conservative rigid opening target is approximately 21.8 x 18.1 mm before final gaiter design.

The aperture is internal and intended to be visually concealed by the soft gaiter and the shell belly overhang.

## Simulation status

`mujoco/pebble_v3f.xml` is the V3-F physics scaffold. It defines the body, four crank joints, four splay cassette joints, compliant paw slides and front/rear paired splay transmissions. It is intended for iterative torque/contact tuning, not as a claim of production-validated dynamics.

The next validation gate is dynamic MuJoCo testing using measured motor curves and then FEA/physical proof tests for the overload path.
