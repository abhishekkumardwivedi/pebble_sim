# Pebble V3 UX Reference Notes

These notes capture the visual constraints from the supplied Pebble concept images and are design inputs for the cumulative V3 shell work.

## Dimensional interpretation

The 170 mm dimension in the concept sheet is treated as the **overall robot height including antenna tips**, not the white body-shell height.

Current V3-B target envelope:

- body width: ~165 mm
- body depth: ~150 mm
- shell belly plane: ~8 mm above floor
- shell top: ~138 mm
- shell height: ~130 mm
- overall antenna-tip height: ~170 mm

This is intentionally much squatter than V3-A, whose ~143 mm shell height made the body read as a rounded cube.

## Front shell silhouette

The body should read as a soft bubble / pebble rather than a rounded rectangular box.

- widest around the lower-middle, not through a long vertical band
- crown begins narrowing earlier
- lower belly also tapers inward before meeting the floor/feet
- side walls should never look vertical for a large fraction of the shell height
- body is slightly fuller toward the front, but rear curvature must remain continuous and soft

The 165 mm width is retained; the correction is primarily *where* that width occurs over height.

## Visor / face proportion

From the supplied close-up and design sheet, the black visor is a dominant organic shape rather than a rounded rectangle.

V3-B visual target:

- visible width: ~102 mm (~62% of body width)
- visible height: ~74 mm (~57% of shell height before curvature clipping)
- top of visor: ~134 mm
- shell crown: ~138 mm
- top white border: only ~4 mm in the frontal reference plane
- bottom of visor: ~60 mm
- white shell below visor to belly: ~52 mm (~40% of shell height)

The visor therefore reaches noticeably into the forehead/crown region and the lower white gap is smaller than V3-A.

The outline is explicitly a pebble/organic profile:

- broad curved top
- tapered upper shoulders
- fuller mid-width
- lower corners pull inward
- gently curved lower edge

Do not revert to `rect + fillet` geometry for the face.

## Feet

The concept feet are soft pebble/paw forms, not thin capsules or flat pucks.

- domed vertical profile
- ~30 x 24 mm maximum local envelope at this concept stage
- top of each foot disappears into the belly
- only a compact rounded portion is visible externally
- small local reveal pockets may be used for UX review, but final openings must later be driven by the real swept linkage volume

## Antennas

Because 170 mm is the overall height, the shell crown should sit substantially below the antenna tips. The antenna stems should therefore be longer and more curved than V3-A, with visibly rounded luminous tips.

## Design authority

For V3 UX development the priority remains:

1. concept-accurate silhouette
2. visor/face proportion
3. visible expressive-foot behavior
4. hard-component feasibility
5. internal skeleton and DFM

Hard packaging may cause local internal changes, but it should not pull the exterior back toward the V2/V3-A rounded-box form unless no feasible equivalent component arrangement exists.
