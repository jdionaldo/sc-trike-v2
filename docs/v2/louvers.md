# Gill louvers on the side pods (desert armored shell)

Source: the owner's `desert_shell_variant/armored_shell.py`, function `_build_side_pod_right` (unchanged in that folder).
Build copy: `parts/desert_shell.py`, functions `louver_frame()` and `louvers()`, constants `LOUVER_*`, `FIN_*`, `THROAT_*`.
Numbers marked MEASURED are read from the built part; the rest are the design values in the code.

## What a gill is

Each side pod carries three gills on its lower flank. A gill is two things:

1. a **fin**: a low wedge that rises out of the skin towards the rear, like a scale;
2. a **throat**: a slot cut under the fin's rear lip, through the fin and the 3 mm wall, into the pod.

Seen from the front the skin is closed: the air (and the eye) only gets in from behind, under the lip. That is why the gills read as
armour scales and not as holes.

## Where they sit

| Gill | Centre on the flank (y, z) | Skin x at the centre (MEASURED by the probe) |
|---|---|---|
| 1 (front) | (-36, 62) | 78.8 |
| 2 | (-18, 66) | 80.8 |
| 3 (rear) | (0, 70) | 79.5 |

- `LOUVER_FIRST = (-36, 62)` and `LOUVER_PITCH = (18, 4)`: each gill is 18 mm behind and 4 mm above the one before, so the row climbs
  to the rear with the pod's belt line. The source typed the three centres; the pitch is the same numbers as a rule.
- `LOUVER_COUNT` is a parameter in the app (2 to 4, default 3).
- **Surface probe**: the skin's x at each centre is not typed. A 0.6 x 0.6 mm bar along x is intersected with the pod's outer envelope
  and its outer end is the skin (`LOUVER_FALLBACK_X = 80.5` only if the probe misses). So the gills follow the flank if the pod's
  corner points move.

## The flank frame

All gill points are given in a local frame (u, v, w) at the gill's centre and turned into world coordinates:

- **w** = outward normal of the flank, `LOUVER_FLANK_N = (0.935, 0.04, -0.35)` normalised: the lower flank leans 20 deg inward at the bottom.
- **v** = across the fin, to the rear: the direction (0, cos -22 deg, sin -22 deg) projected onto the flank (`LOUVER_LEAN = -22`).
  World: (-0.16, 0.93, -0.32).
- **u** = along the fin: w x v, pointing up. World: (0.31, 0.36, 0.88). So the fins stand almost upright and lean 22 deg to the rear
  at the top.

One frame for all three gills: the flank is treated as one plane there. The probe corrects the position along x, not the tilt.

## The fin (convex hull of 10 points)

| Value | Constant | mm |
|---|---|---|
| Half length at the base, along u | `FIN_HALF_BASE` | 18.5 (fin 37 long) |
| Half length at the crest | `FIN_HALF_CREST` | 15.0 (ends slope in 3.5) |
| Leading edge, v | `FIN_LEAD` | -6.5 |
| Crest, v | `FIN_CREST` | 2.0 |
| Rear lip, v | `FIN_LIP` | 3.4 |
| Base inside the skin, w | `FIN_EMBED` | -2.2 |
| Leading edge above the skin, w | `FIN_LEAD_PROUD` | 0.2 |
| Crest above the skin, w | `FIN_PROUD` | 5.8 |
| Lip edge under the crest | `FIN_LIP_DROP` | 1.2 |

- The ramp rises from 0.2 mm at the leading edge to 5.8 mm at the crest over 8.5 mm: 33 deg.
- The base lies 2.2 mm **inside** the 3 mm skin (embed), so fin and skin fuse over a volume and not along a line; it stays 0.8 mm
  short of the inner face, so nothing sticks into the pod.
- Behind the crest the fin runs on 1.4 mm as a lip that drops 1.2 mm: the lip hides the throat from the side.

## The throat (convex hull of 8 points, cut after all fins are fused)

| Value | Constant | mm |
|---|---|---|
| Half length along u | `THROAT_HALF` | 14.5 (slot 29 long) |
| Fin left above the throat | `THROAT_ROOF` | 2.4 |
| Outer mouth, v from the crest / past the lip | `THROAT_V` | -0.5 / +2.8 (v 1.5 .. 6.2, 4.7 wide) |
| Inner mouth, v | `THROAT_V_IN` | crest -5.0 / lip -1.0 (v -3.0 .. 2.4, 5.4 wide) |
| Past the inner face of the wall | `THROAT_DEPTH` | 4.0 |

- The outer mouth lies behind the crest at w 3.4 (under the 2.4 mm roof), the inner mouth 4.5 mm further forward at the inside of the
  wall: the slot runs forward and inward under the fin, 22 deg off the flank normal. It opens to the rear only.
- Open area per gill, MEASURED on the built part in the plane of the wall's inner face: **148.8 mm2** (29 x 5.1 mm).
  Three gills: 446 mm2 per pod, 893 mm2 for both. Material removed per gill, MEASURED: 653 / 694 / 656 mm3.
- What is left around the slot: 2.4 mm fin above it, 1.2 mm at the lip's edge, about 1.5 mm of fin at each end of the slot
  (fin half length there 16.0, slot 14.5). All at or above the 1.2 mm minimum wall.

## Mirror

Only the right pod is built. The left pod is its mirror image in the YZ plane, so the left gills are the same gills, opening to the
rear as well. Each pod is then trimmed against its own side of the cage and deck.

## Printing

- Orientation: the pod stands as on the car (z up). The fins then stand almost upright, the ramps and throats are vertical features
  and print without support. MEASURED (`soma analyze overhang`, z up): the underside of each fin's lower end is a 12 x 6 mm
  spot at 88 deg (45 mm2 per fin): a 6 mm ledge, support there or accept a rough end face. The pod's other overhangs are the
  lower rim of the forward wing (304 mm2, needs support: it starts in mid-air at z 104) and the bottom rim on the bed.
- Printed flat on the inboard side the flank would be a ceiling 45 mm above the bed: not that way.

## Changes against the source, each with its reason

1. Every number of the gills is a named constant, the gill count a parameter, the three centres a start point plus a pitch
   (parametric rule of the project). The values are the source's: the built gills are the same shape (pod volume before the mount holes
   and trims: 82 530 mm3 here, 82 353 mm3 in the source, which had six 3.4 mm holes more).
2. The gill code moved out of the pod function into `louvers()`, and it returns what it measured (centre, removed volume, open area),
   printed by the build.
3. The six "mounting holes" of the source are gone from the pod: they went through the outer skin only, 20 to 35 mm away from
   anything to screw into. The pods now have four screws each into cage bosses (NOTES.md), all above z 129, clear of the gills
   (top of the highest fin: z 86).
4. Nothing else: lean, sizes, embed, lip and throat are as the owner drew them.

## Not verified

- No print yet: the 1.2 mm lip edge and the fin end ledges are the spots to look at on the first pod.
- Air flow: the open area is geometry, nobody measured cooling.
