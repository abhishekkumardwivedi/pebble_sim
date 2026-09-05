# Pebble V2.5 — Tucked-Leg Concept Correction

This revision specifically fixes the mismatch noticed between V2.4 and the original Pebble concept.

The original concept has a large rounded body with the legs visually **tucked underneath it**. The upper linkage and motors should not sit outside the body silhouette.

## Correction

V2.4:
- leg motor axes at Y = ±59 mm

V2.5:
- leg motor axes at **Y = ±43 mm**

That moves each leg module **16 mm inward per side**.

Front/rear roots are also more central:
- front X = +35 mm
- rear X = -45 mm

The body remains 165 × 145 × 110 mm.

## Why the shell lower opening stays at Z=68 mm

Lowering the shell further around the tucked mechanisms caused the body wall to physically intersect the brackets/linkages.

This version keeps the mechanically safe lower opening while using the inward leg placement to obtain the original visual proportion.

The final consumer shell can later extend lower locally using shaped wheel-arch/leg-pocket surfaces **after** the swept motion envelope is frozen.

## Validation

The corrected tucked placement reports zero intersections for shell-vs-leg and leg-vs-leg checks. See `placement_validation.json`.

## Release model

Canonical geometry filename: `Pebble_V2_5_Tucked_Legs_Assembly.step`

Original release archive: `pebble_v2_5_tucked_legs_final_bundle.zip`

Archive SHA-256: `4feeaeffc5e19f2ce637af93fb8a84c18dff8d8dbe3a009b0991c66c5f5992e9`

The release also includes `Pebble_V2_5_Leg_Animate360.FCMacro`, parameters, placement validation, and the upper-body STEP part.

This is a packaging correction only. The V2.3 skeleton/linkage geometry itself has not changed.
