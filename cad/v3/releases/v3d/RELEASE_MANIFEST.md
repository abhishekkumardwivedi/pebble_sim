# Pebble V3-D UX Freeze Candidate

This release captures the exterior UX-freeze candidate generated from `cad/v3/generate_ux_shell_leg.py` after the reference-driven V3-D refinement.

## Geometry summary

- shell bounding box: approximately 149.08 mm depth (X) × 165.02 mm width (Y) × 130.00 mm shell height (Z)
- overall nominal height with antenna bulbs: 170 mm
- visor visible bounding box: approximately 106.08 mm wide × 71.05 mm high after shell clipping
- visor top/bottom: approximately Z=134.01 / 62.96 mm
- front-left four-bar endpoint trajectory: approximately 21.82 mm fore/aft × 16.30 mm vertical

## UX changes from V3-C

- visor no longer follows a rounded-rectangle-like width distribution; forehead is narrower, lower-middle is widest, and the bottom bows/tucks into the cheeks
- visor is approximately 0.45 mm proud of the conformal shell surface for a glossy lens read
- upper shell volume redistributed downward by only a few millimetres while preserving the package envelope
- feet reduced visually by roughly 15–20%, made more domed, and tucked further under the belly
- front feet remain slightly more visible than rear feet
- antenna stems sweep farther outward with larger luminous bulbs

## Generated artifacts and SHA-256

The connected GitHub text writer cannot directly upload the multi-megabyte generated STEP binaries, so the parametric source is the repository source of truth and the exact generated artifacts are identified here by checksum.

- `Pebble_V3D_UX_Exterior.step` — `f9ac969c21e196797a33329651a48527488c496d6738375c9aa3fb453bb010f6`
- `Pebble_V3D_UX_Shell_Leg_Concept.step` — `90e336ad5446b9442105bbb18147fd6e659ae6ebe119e81540cb70ad837700c4`
- `Pebble_V3D_FrontLeft_FootTrajectory.csv` — `9fac336adcb6de09c08a5b64dde380796c498a34d4375bce4c66834aa852d101`
- `Pebble_V3D_UX_Freeze_Preview.png` — `6db50e7d3031da35a249373f6cb7707733cf1a0e8b765f0e96cc19ad35003d39`

## Next gate

Do not broadly redesign the exterior at the next gate. Generate full leg swept volumes, cut only the minimum real leg exit pockets, then test hard component keepouts and build the internal skeleton. Exterior changes require a demonstrated hard-feasibility conflict or a new explicit UX decision.
