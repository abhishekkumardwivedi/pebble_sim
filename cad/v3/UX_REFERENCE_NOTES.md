# Pebble V3 UX Reference Notes

These notes capture the visual constraints from the supplied Pebble concept images and are design inputs for the cumulative V3 shell work.

## Dimensional interpretation

The 170 mm dimension in the concept sheet is treated as the **overall robot height including antenna tips**, not the white body-shell height.

Current V3-D target envelope:

- body width: ~165 mm
- body depth: ~149–150 mm
- shell belly plane: ~8 mm above floor
- shell top: ~138 mm
- shell height: ~130 mm
- overall antenna-tip height: ~170 mm

The exterior envelope is now considered a UX-freeze candidate. Hard packaging should fit inside this shape before any major exterior dimensional change is accepted.

## Front shell silhouette

The body should read as a soft bubble / pebble rather than a rounded rectangular box.

- widest around the lower-middle, not through a long vertical band
- crown begins narrowing early
- upper shoulder volume is reduced by only a few millimetres relative to V3-C
- lower-middle volume remains dominant so the body reads soft and baby-like
- lower belly tapers inward before meeting the floor/feet
- side walls should never look vertical for a large fraction of the shell height
- front is slightly fuller than rear, while rear curvature remains continuous and soft

The 165 mm width is retained; the correction is primarily *where* that width occurs over height.

## Visor / face proportion

The black visor is treated as a **second glossy pebble nested inside the white pebble shell**, not as a display-shaped rounded rectangle.

V3-D visual target / generated result:

- visible width: ~106 mm maximum (~64% of body width)
- visible height after shell clipping: ~71 mm
- top of visor: ~134 mm
- shell crown: ~138 mm
- top white border: ~4 mm in the frontal reference plane
- lower-middle is the widest part of the visor
- forehead/top region is narrower
- bottom edge bows upward at the centre and tucks into the cheeks
- lens sits ~0.45 mm proud of the conformal shell surface to give a separate glossy-lens read

The goal is not simply “wider at the bottom.” It is a continuously changing organic width profile that follows the cheek curvature.

Do not revert to `rect + fillet` geometry for the face.

## Feet

The concept feet are soft pebble/paw forms, not thin capsules, flat pucks, or visibly mechanical leg ends.

V3-D changes:

- visual footprint reduced roughly 15–20% from V3-C
- dome height retained/increased so the feet read soft rather than flat
- upper sections drift inward toward the body centre
- front feet are slightly more visually prominent than rear feet
- shell reveal pockets are smaller and shallower so more of each foot disappears under the belly
- final openings must later be driven by the true swept linkage volume, not by these cosmetic review pockets

## Antennas

The antennae are character elements, not only sensor/LED stalks.

V3-D uses:

- slightly greater outward curvature
- slightly thinner stems
- larger luminous bulbs
- the same ~170 mm overall height target

They should read as curious ears/eyebrows and amplify expression together with the face and body posture.

## UX freeze rule

V3-D is the current **exterior UX freeze candidate**. The next engineering work should be:

1. generate and validate full leg swept volumes;
2. create the minimum real leg exit pockets;
3. test hard component keepouts (battery, motors, compute, MCU, display/camera, power/audio);
4. build the internal skeleton around the frozen exterior;
5. change the exterior only for a demonstrated hard feasibility conflict.

This prevents endless cosmetic iteration while keeping the user-experience geometry authoritative.
