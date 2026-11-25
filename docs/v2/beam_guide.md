# Version 2 heave spring: beam guide (tools/beam_kin.py)

Base: lever balls of `parts/swingarm2.py` at commit 7b79b4c (frozen copy `tools/_swingarm2_snapshot.py`): 6.8 mm ball on an inboard stud at
x +-54, lever 55 mm, tilted 40 deg. Run `python -I tools/beam_kin.py` (pure math, about 15 s). Every number below is printed by that run
(CALCULATED) unless it is marked ASSUMED. The hard points are in `BEAM_DESIGN` at the top of the tool, in world mm, for the +x side.
Fix round 1 (after @verify-2): PC and E are lowered for >= 1 mm swept clearance under the spine, the carrier tail is shortened to 10 mm, the tool
prints the one-wheel reach and the spine clearance, and the residual label is corrected.

## Design

- **Carrier (new part).** A bellcrank on the centre line. It hangs from a double-shear clevis under the spine's front cross tube
  (y -48, z 114), with a pin along x at PC (0, -50.6, 98.5). It carries the beam's yaw pin at C (0, -81.45, 85.4), 33.5 mm from PC, and the shock's front eye
  at S (0, -45.6, 81.5), 17.7 mm under PC, in a fork (double shear, pin along x). A 10 mm tail behind PC, at (0, -40.6, 98.5), carries the heave stops.
  **It holds the beam sideways (x), in pitch and in roll.**
- **Beam.** It turns on the carrier about one pin, axis N = (0, 0.599, 0.801): square to the beam, tilted 36.8 deg back from vertical. This is its
  only freedom on the carrier. Its middle bows 4.2 mm under the line between the balls.
- **Ends.** A cup block sits on each lever ball and slides in a flat slot in the beam-end fork. The slot faces are square to
  M = N x X = (0, 0.801, -0.599), so each end gives exactly one constraint: the ball centre stays on the slot's mid-plane. The block slides along
  the beam (the ball distance grows in lean) and along N (the ball's arc and the carrier's arc differ).
  This makes 2 unknowns (carrier angle, beam yaw) for 2 equations: **the beam is fully constrained**. No position depends on friction or on the shock.
  **The cup block must be a round puck, free to spin in its slot about M.** Its spin is an idle freedom: it duplicates the ball's own rotation
  and moves nothing else (found by @verify-2). A square block would jam against the slot walls.
- **Shock.** It runs from S back to E (0, 20.54, 97.3), under the spine and between its cross tubes at y -48 and y 38.
  **E z 97.3 is ASSUMED, a chosen value.** It keeps the swept spring envelope >= 1 mm under the spine underside, and it is set by hand in `BEAM_DESIGN`,
  not derived from cage geometry. The rear-eye clevis then hangs 11.7 mm below the ASSUMED spine underside (z 109).
  The shock line rises 13.4 deg toward the rear. Both eyes are plain pins along x in double shear. The carrier turns only about x, so the shock stays in the x = 0 plane:
  the eyes need 0.00 deg of misalignment. The beam does not touch the shock (the eye is on the carrier, not in a beam slot).
- **Rejected: short ball links instead of slots** (scratch search, CALCULATED there but not in the tool). With 12 mm links the best case had the beam
  3.4 mm outside the box, a 0.65 mm lean residual and a 27 deg cup tilt.
  **Rejected: 7b79b4c's slots along the beam only.** These are 2 constraints per end. The beam's x position and its spin then hang on the shock eye.

## Numbers (CALCULATED)

| pose (t+x / t-x mm) | shock mm | compression mm | carrier deg | beam yaw deg | slot slide along beam / N mm (+x) | lever transmission deg |
|---|---|---|---|---|---|---|
| rest 0 / 0 | 68.00 | 0 | 0 | 0 | 0 / 0 | 86.8 |
| bump +12 / +12 | 63.32 | +4.68 | +15.46 | 0 | 0 / +3.61 | 64.5 |
| droop -8 / -8 | 70.87 | -2.87 | -9.28 | 0 | 0 / -3.43 | 100.7 |
| lean +10: +25.9 / -24.9 | 67.92 | +0.08 | +0.25 | +14.50 | +1.12 / -2.61 | 72.3 / 100.8 |
| lean +10 + bump: +37.9 / -12.9 | 63.60 | +4.40 | +14.53 | +13.21 | +0.87 / -4.40 | 51.5 / 80.0 |
| lean +10 + droop: +17.9 / -32.9 | 70.63 | -2.63 | -8.51 | +14.45 | +1.11 / -2.55 | 85.2 / 114.0 |

(lean -10 is the mirror image)

- **Ratio, two-wheel travel:** 0.390 at +12 (4.68 mm of shock) and 0.358 at -8. The local ratio is 0.332 at -8, 0.381 at 0, 0.395 at +6 and 0.379 at +12.
- **Lean residual at 10 deg** (change of shock compression against the same heave without lean, + = shorter): +0.08 mm at rest heave, -0.27 mm
  with bump and +0.24 mm with droop. Target < 0.5 mm: met.
- **Comparison with the thread's `sa.beam_pose`** at lean +10: both have a ball stretch of 3.57 mm. The thread's shock gets 1.13 mm longer.
  Here the slots take the stretch (+1.12 mm in one end, +2.43 mm in the other) and the shock gets 0.08 mm shorter. At two-wheel +12 the thread has 4.63 mm (0.386)
  and this guide 4.68 mm (0.390).
- **Full grid** t+x, t-x in -32.9..+37.9 (25 x 25): all 625 poses solve. Worst Jacobian condition 0.39 (0 = singular), so there is no
  singular pose. Inside the stops (253 poses), the transmission angle is at most 39.0 deg from 90 at any joint, so nothing comes near dead centre.
  The shock pitch stays between 12.7 and 13.5 deg.
- **Slot slide inside the stops:** along the beam -0.06..+2.42 mm, along N -7.61..+7.60 mm. Slot length along N = puck + 15.2 mm + clearance.

## Stops (travel ends here, not in the shock)

| stop | where | position (CALCULATED) |
|---|---|---|
| heave bump | stock steel dowel D8 x 75 (ISO 8734, ASSUMED stock length) through a 14.8 mm boss on the carrier tail (0, -42.1, 98.5), r 8.5 (walls 2.3 above, 5.0 below, 4.0 front); its ends pass arc slots in the clevis plates and land on TPU saddles (faces square to the tail radius) in pad blocks at \|x\| 13.4..35.4; clevis plates moved to \|x\| 9.4..12.4 | carrier +15.46 deg = two-wheel +12 |
| heave droop | the same pin on the lower saddles | carrier -9.28 deg = two-wheel -8; pin drops 1.61 mm (gap at rest) |
| lean (beam yaw) | a PAIR of TPU pads each way on posts hanging from the carrier's upper fork arm (above the bar, 12 x 20 mm) at \|x\| 29.5..39.5 (r 34.5), on the bar's front and rear faces | +-14.50 deg yaw; pad faces at M -17.5 / +16.4 |
| per-arm | bump: NONE (coordinator, round 2); droop: TPU wedge on the upper link rail lug (docs/v2/arm_stops.md) | one wheel reaches -32.9 .. **+41.9** (vertex (+41.9, -14.7), CALCULATED by tools/arm_stops.py) |

**Stop sizing round 1 (tools/arm_stops.py SIZING table, hand calc, FEM later; PETG 20 MPa, TPU 5 MPa ASSUMED, steel 350 MPa ASSUMED):**
- **Loads** (65 N landing, heave spring ignored): carrier stop 676 N (two-wheel) / 561 N (one-wheel at (+41.9, -14.7)) on the r 8.5 tail; yaw pads 125 N each pad of the pair.
- **Shock requirement:** the lower spring retainer must start >= 14 mm behind the front eye centre (ASSUMED shock data; carrier tail clearance 1.11 mm).
- **Beam:** bar 12 mm wide (N) by 22 mm deep (M) at C, tapering to 13 mm at |x| 40.
  - At C its section sits 4 mm forward (-M), so the rear-bottom edge clears the built motor by 1.33 mm.
  - The end forks are closed boxes with 3 mm plates.
- **Results:** every part has safety >= 2.5 after fix round 3 (final). The lowest are the pin-end TPU saddle (2.5) and the yaw arm root, bending plus torsion (2.6). Table in docs/v2/arm_stops.md.
- **No lower yoke on the carrier.** The beam's yaw pin is a D8 steel pin cantilevered from the upper fork arm, with a clip under the bar (pin safety 9.3).
  A lower yoke lug would hit the motor.
- **The motor is now a keep-out** (built STEP, `MOTOR` in this tool): the sample points stay 3.54 mm above it, and every carrier part is >= 3.9 mm from it in 3D.
- **Heave spring at the bump stop** (information only, ASSUMED 3 N/mm): about 54 N in the shock.

With these stops, 10.0 deg of lean is allowed at droop and at rest heave, and 11.0 deg at bump heave.
Lean is free between the yaw stops: the heave shock does not centre it.

**3D check against the built model (stop sizing round 1).** BRepExtrema at 24 poses inside the stops, out to one-wheel +41.9:
- **Clear:** the bar, yaw arm and posts are 1.10 mm from the cage and 1.33 mm from the motor, and >= 14 mm from the servo, receiver, shell and decks.
  The posts clear the forks by 4.71 mm, and the arm clears the bar by 1.20 mm. On the stop the bar rests on the 2 mm TPU (2.09 mm to the post).
- **CLASH, not from this round:** the +x beam-end fork overlaps a cage member on top of the lower rail (x 41..49, y -74..-52, top z 67.5..70.6 in the built cage).
  The overlap is 3.6 mm3 at 10 deg lean, 19.7 at lean + bump and 25.4 at (+41.9, -14.7). The cage owner must move or lower it.
- **Box margin:** the 625-pose grid out to +41.94 solves everywhere (worst condition 0.39). The bar is 1.4 mm under the old cage free box floor (z 68) at its ends at one-wheel bump.
  This is a box margin, not a part clash: the 3D distances above are the real ones.

## Shock check (ASSUMED shock, 1/10 class)

ASSUMED: 72 mm extended, 16 mm stroke (56 mm closed), body OD 12.5 mm, spring OD 16.5 mm, eye width 6.0 mm, M3 eyes, eye OD 8 mm. A real 1/8
shock (100 mm and longer) does not fit the 68 mm ride length.

- Stroke used between the stops: 7.54 mm.
- At the bump stop the shock is 63.32 mm, 7.32 mm before it closes. At the droop stop it is 70.87 mm, **1.13 mm before full extension**.
- Over every pose inside the stops it runs 63.37..70.81 mm, so the stops end the travel, not the shock.

## Space, clearance and joints

- **Spine clearance**, swept inside the stops against the spine underside z 109.0 (ASSUMED 10 mm tube, y -53..43):

  | part | top z | clearance |
  |---|---|---|
  | carrier: hub R 5, arms half-thickness 4, S-fork R 6.5, tail (all ASSUMED) | 105.14 | **3.86 mm** |
  | bump pad (ASSUMED 2 mm on the tail) | 107.14 | **1.86 mm** |
  | shock: eyes R 4, spring R 8.25 along the rest of its length | 104.38 | **4.62 mm** |

  All meet the owner rule of >= 1.0 mm. The spine does not need to be raised or split.
- **Box the design needs** (swept inside the stops; fork half-size 6 x 6.5 x 13.5 mm along beam / M / N, ASSUMED; beam bar 10 mm deep, ASSUMED):
  - Beam with forks: x -60.7..60.7, y -106.9..-54.1, z 62.0..118.9.
  - Beam bar (\|x\| <= 40): x +-40.3, y -96.9..-62.3, z 68.1..104.3. That is **0.1 mm** inside the cage free box of 7b79b4c (x +-45, y -105..-60, z 68..105).
  - Carrier hard points: y -83.1..-41.3, z 77.7..98.5 on x 0. It leaves that box at the rear, toward the cross tube.
- **Rail:** the beam bar clears the rail top (z 64.4 at x 45) by at least 4.7 mm. **The fork corners inboard of x 50 come down to z 63.0: a clash.**
  For 1 mm, the rail top must be at z <= 62.0 under the forks (y -105..-57), or the fork's lower inboard corner must be chamfered.
- **Misalignment cones:**
  - Lever ball cup: 19.7 deg over the whole grid (stud along x against the puck axis), 14.5 deg inside the stops. The cup's neck
    opening must allow 20 deg around the stud (ASSUMED limit of a ball cup: 25 deg).
  - The fork's outer opening must let the stud move +-7.6 mm along N.
  - Shock eyes: 0 deg (plain pins).
- Swept clearance to the real cage tubes is NOT calculated: the cage is not in this tool. Only the box, rail and spine margins above are calculated.

## Open points

1. **Fork against rail: 1.4 mm overlap.** Either the rail is lowered locally to a top of z <= 62.0 under the forks (cage, not this scope), or the
   forks are chamfered. A chamfer must be checked again. Lowering PC (for the spine clearance) raised the N slide from 6.1 to 7.6 mm, which lengthened the forks.
2. The beam bar is only 0.1 mm inside the cage free box (z 68 floor, at lean + bump).
3. The fork's top reaches z 118.9 at x +-54 (droop). Check it against the shell and the upper belt tubes.
4. The pucks slide under the spring load (up to 7.6 mm along N): use POM or brass and grease. Friction and wear are not calculated.
5. The droop stop is 1.13 mm before the ASSUMED shock is fully extended: confirm the bought shock's real lengths.
6. Lean centring and the strength of the carrier, clevis, yaw pin and forks are not designed.
