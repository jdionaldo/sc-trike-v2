# Version 2 per-arm stops, reachable travel and servo mount (tools/arm_stops.py)

Run `python -I tools/arm_stops.py` (about 45 s; it reads the built `output/model.step` of `sc-trike-v2` read-only).
Numbers are CALCULATED by that run unless marked ASSUMED. The hard points are in `STOP_DESIGN`, +x side; mirror x for the -x side.

**Round 2 (coordinator decision):** there is **no per-arm bump stop**, and the servo mount no longer carries a bump support.
Fix round 1 had shown that no PETG section in the space can carry the 172 N one-wheel landing load: the best possible safety was 1.38.
Bump travel now ends on the printed carrier and beam-yaw stops of `tools/beam_kin.py`. **The per-arm droop stop stays.**

## Stop sizing round 1: loads, sections, safety (CALCULATED, hand calc; FEM later)

**Loads.** 65 N at the tip, from `analyses/front_arm.py`. The heave spring is ignored, so the stops take it all (the spring holds about 54 N in the shock at the stop; ASSUMED 3 N/mm, information only).
- Carrier stop: 575 N in a two-wheel landing at (+12, +12); 477 N in a one-wheel landing at (+41.9, -14.7).
- Yaw stops: they are now a PAIR each way at r 34.5, so each pad takes 125 N at (+41.9, -14.7).

**Stop sizing fix round 3 (final).** What changed:
- **Clevis plates move out** to |x| 9.4..12.4 (built: 6.4..9.4).
- **Carrier tail** at r 8.5 behind PC, (0, -42.1, 98.5).
- **Tail boss** 14.8 mm long:
  - walls: 2.3 mm on top, 5.0 below, 4.0 at the front; at the rear only the 2.3 mm round wall, so the boss stays clear of the shock's spring retainer;
  - tear-out is now checked on the shear allowable 20/sqrt(3) = 11.5 MPa.
- **Carrier arm to the yoke:** 10.8 x 18 mm, its section 2.5 mm under the PC-yoke line, so it stays clear of the spine and clevis bridge.
- **Pins and saddles:** D8 x 75 tail pin (stock length ASSUMED), saddles 22 mm long at |x| 13.4..35.4.
- **Yaw pin boss** 16 mm long, checked for bending plus bearing.

Loads (65 N landing; heave spring ignored, about 54 N in the shock at the stop, information only):
- carrier stop: 676 N two-wheel, 561 N one-wheel;
- yaw pad: 125 N.

| part | section | load | MPa | safety |
|---|---|---|---|---|
| beam bar at C, two-wheel | 12 x 22 mm (N x M) | 131 N at 54 mm | 7.3 | 2.7 |
| beam bar at C, one-wheel | 12 x 22 mm | 157 N ball, 125 N pad | 4.3 | 4.7 |
| beam bar at the yaw pad (\|x\| 34.5) | 12 x 14.2 mm | 157 N at 20 mm | 7.5 | 2.7 |
| fork plate at the ball (closed box fork) | 27 x 3 mm, 12 mm fixed-fixed | 157 N | 5.8 | 3.4 |
| puck on the slot face | D9 face | 157 N | 2.5 | 8.1 |
| yaw pad post | 10 x 9 mm, 7 mm lever | 125 N | 6.7 | 3.0 |
| yaw arm root at C, bending + torsion | 12 x 20 mm | 125 N at 34.5 mm, 13.2 mm offset | 7.8 | 2.6 |
| yaw pad TPU | 10 x 12 mm | 125 N | 1.0 | 4.8 |
| yaw pin, steel D8 cantilever (350 MPa ASSUMED) | D8, 7.2 mm lever | 262 N | 37.5 | 9.3 |
| yaw pin boss in the upper yoke, bending + bearing | D8 x 16 mm | 262 N, 1883 N mm | 7.6 | 2.6 |
| beam bar on the yaw pin | D8 x 12 mm | 262 N | 2.7 | 7.3 |
| carrier arm PC to the yoke, at the hub | 10.8 x 18 mm | 262 N, 2556 N mm | 4.4 | 4.6 |
| tail boss on the pin, bearing | D8 x 14.8 mm | 676 N | 5.7 | 3.5 |
| tail boss tear-out, bump (wall below, shear) | 2 x 5.0 x 14.8 mm | 676 N | 4.6 | 2.5 |
| tail boss tear-out, droop (wall above, shear; 30 N per wheel ASSUMED) | 2 x 2.3 x 14.8 mm | 312 N | 4.6 | 2.5 |
| tail boss side walls, net tension | (4.0 + 2.3) x 14.8 mm | 676 N | 7.3 | 2.8 |
| tail pin, stock steel dowel D8 x 75 (350 MPa ASSUMED) | D8, 17 mm lever | 338 N per end | 114 | 3.1 |
| pin-end TPU saddle, face square to the tail radius | D8 x 22 mm projected | 338 N | 1.9 | 2.6 |
| pad block wall under the saddle | 22 x 6 mm, 22 mm span | 338 N | 7.0 | 2.8 |

**Requirement on the bought heave shock (ASSUMED data):** the lower spring retainer (R 8.25) must start >= 14 mm behind the front eye centre; fit a spacer if needed.
At 12 mm the tail boss comes within 0.18 mm of it at droop; at 14 mm the gap is 1.11 mm.

**Other CALCULATED results:**
- Spine and clevis bridge underside (built z 108.5): carrier top 107.05, **1.45 mm**.
- Motor: >= 3.54 mm from the sample points, >= 3.9 mm for every carrier part in 3D.
- Ratio 0.390, lean residual <= 0.27 mm; the 625-pose grid to +41.94 solves everywhere.

## Carrier clevis interface and 3D clearance table (built model 2026-11-23 22:48, BRepExtrema, 24 poses inside the stops)

**For the design thread (cage):** move the carrier clevis plates to |x| 9.4..12.4 (gap 18.8), over y -56.6..-33.6 and z 89.9..108.5.
- The hub is 16.8 mm long (1 mm per side, set with shim washers); the other carrier parts between the plates are <= 10.8 wide.
- Cut the arc slots for the D8 tail pin in the plates, remove the old central heave-stop pads, and put the pad blocks at |x| 13.4..35.4, tied into the spine and the clevis.
- Move the door gusset that the +x fork hits.

| pair | min mm | note |
|---|---|---|
| new clevis plates to: hub / tail boss / arms and S fork | 1.00 / 2.00 / 4.00 | |
| new clevis plates to: heave shock / beam bar / fork | 2.55 / 13.4 / 33.2 | |
| carrier arm to the yoke, arm to S, tail arm to the built cage (bridge z 108.5) | 1.00 / 1.00 / 1.00 | |
| carrier S fork to the built cage | 1.09 | |
| carrier hub / tail boss to the built cage | overlap 422 / 349 mm3 | **the built plates at \|x\| 6.4..9.4; gone when the plates move** |
| yaw pin boss / clip to the built cage | 5.87 / 27.4 | |
| carrier parts to the motor (min, S fork) | 3.90 | arm to yoke 15.0, clip 5.99 |
| carrier parts to the heave shock (min, tail boss; retainer at 14 mm) | 1.11 | arm to yoke 4.45, arm to S 4.12, hub 7.72 |
| carrier parts to the beam bar / fork | 3.25 / 37.6 | |
| bar, yaw arm and posts to the cage / motor | 1.10 / 1.33 | |
| bar, yaw arm and posts to servo, receiver, shell, decks, pivot block | >= 14.8 | |
| fork to the posts / bar to the posts (on the 2 mm TPU) / arm to the bar | 4.71 / 2.09 / 1.20 | |
| **+x beam-end fork to the built cage** | **0.83** | the door gusset (cage work, design thread) |

## 3D check of the beam guide against the built model

At 24 poses inside the stops:
- **Clear:** the bar, yaw arm and posts are 1.10 mm from the cage, 1.33 mm from the motor, and >= 14 mm from the servo, receiver, shell, decks and pivot block.
  The posts clear the fork by 4.71 mm; the arm clears the bar by 1.20 mm.
- **CLASH (cage, not from this round):** the +x beam-end fork overlaps a member on top of the lower rail (x 41..49, y -74..-52, top z 67.5..70.6).
  The overlap is 3.6 mm3 at 10 deg lean, 19.7 at lean + bump and 25.4 at (+41.9, -14.7). The cage owner must move or lower it.

## Droop stop (-32.9 mm, blade 20.5 deg)

- **Lug (cage):** box x 41.0..74.7, y -125.4..-113.4, z 80.0..90.0, with the inner clevis above it (x 62.3..74.7, same y, z 85.9..102.9).
  **DERIVED, not assumed:** this is `parts/uplink.py` `rail_lug()` / `clevis_u()` at 7b79b4c.
  It uses RAIL_REF (45, -142, 100.3) in the ref frame (= world z 85.0), GAP 6.4, PLATE_T 3.0 and WIDTH 12 from `parts/shock_mount.py`, and H_IN_W (68.5, -119.4, 97.9):
  - x = RAIL_REF.x - 4 .. H_IN.x + (GAP + 2 PLATE_T)/2
  - y = H_IN.y +- WIDTH/2
  - z = 85.0 +- 5

  These are the numbers in the `LUG` constant of the tool.
- **Contact:** the blade's front face lands flat on a TPU wedge on the lug's rear face, over the **whole lug height z 80..90**, x 61.7..70.7.
  The contact face runs from (y -113.00, z 80.00) to (y -108.08, z 90.00). The force on the wedge points forward, 26.2 deg above horizontal.
- **Lug corner chamfer (the thread must build it):** at full droop the blade passes 0.36 mm from the bare lug's lower rear corner (y -113.4, z 80.0).
  So the corner is cut back to a plane 2.0 mm behind the contact face, from (y -115.23, z 80.00) on the lug bottom to (y -113.40, z 83.72) on its rear face.
  **The chamfer runs over the full lug width, x 41.0..74.7** (coordinator, fix round 2), so no sharp corner is left beside the blade.
  The TPU wedge fills the cut only over the blade band x 61.7..70.7, and stands 2.0..4.8 mm thick, normal to its face.
- **3D check:** the tool builds the blade, the chamfered lug, the clevis (floor z 85.9..89.7 plus plates) and the wedge as CadQuery solids, and measures them with OCC BRepExtrema at 13 poses
  from -32.9 to +41.9. Minimum distances:
  - Blade to bare lug: **2.00 mm**, or 1.94 mm after the 0.06 mm droop TPU squeeze.
  - Blade to clevis: **2.96 mm** (the verifier also measured 2.96).
  - Blade to wedge: 0 at -32.9, which is the contact.
- **Load:** 30 N at the wheel, ASSUMED. That is the wheel in the air: its share of the heave spring (about 8 N) plus 0.25 kg unsprung at 8 g (20 N), rounded up.
  The moment is 2752 N mm. At a contact radius of 37.3 mm that is **74 N**, and 0.83 MPa on 89 mm2.
- **Load path:** wedge, then the rail lug (10 x 12 mm, 21.2 mm out from the stub bar at x 45), then the stub bar (8.8 mm, z 85), then the cage's front foot post at y -140 and the deck.
  Lug bending, by hand: 9.3 MPa, safety 2.2.

## Servo mount (round 2: holds the servo only)

Checked in 3D (BRepExtrema) against the built model:
- deck top z 25.06 (y -108.6) .. 27.91 (y -67.8), measured;
- servo box x 32..58.5, y -108.6..-67.8, bottom z 28.2.

- **Part 1, wedge floor plate (PETG):**
  - x 32.0..54.5 under the servo, top z 28.2, underside on the deck, 3.14 mm thick at the front.
  - **It ends at y -75.08, where it would get thinner than 0.8 mm.** The servo's last 7.3 mm stand over a 0.3..0.8 mm gap.
  - A tongue x 19..32, y -108.2..-96.2 lies under the clip, 2.7 mm thick at the screw.
  - A 1.5 mm front rim at y -110.1..-108.6 locates the servo forward.
  - Printed on its flat top face.
- **Part 2, clip (PETG):**
  - leg x 19..31, y -108.2..-96.2, z 28.2..53, standing on the tongue;
  - arm over the servo's inboard edge, x to 39.4, z 48.9..53, on 0.5 mm EVA (ASSUMED);
  - 1.5 mm strap across the servo top, x 39.4..57.0, y -105..-99, z 48.9..50.4, under the cage rail (bottom z 52.6) and 2.2 mm short of the horn.
- **Why two parts:** the floor must stay under the servo while the servo slides in and out; only the clip has to come off.
- **Fixing:** one M3 x 16 from below through the deck (6 mm) and the tongue into a heat-set insert in the clip leg at (25.0, -102.2).
  - The insert is 6.0 mm from all four leg edges.
  - The deck hole is 29.5 / 40.3 mm from the deck edges and off the lap joint.
- **Servo out:** take the horn off, undo the one screw from below and lift the clip off. The servo then slides 23.5 mm inboard over the tongue.
  Lifting it out from there needs the beam assembly off: there are 18.6 mm under the beam bar for a 20.2 mm servo.
- **Dropped:** the bump arm, keel, bar and axle ring. Nothing else needs them.

## Clearance (owner rule >= 1.0 mm; CALCULATED)

| check | min distance |
|---|---|
| droop: contact face gap | closes to 0 at -32.9 by design; 1.65 mm at 5 mm of travel before it |
| droop wedge to the rest of the blade, travel -32.9..+41.9 | 1.82 mm |
| blade to the bare (chamfered) lug, 3D, minus the 0.06 mm squeeze | 1.94 mm |
| blade to the clevis, 3D | 2.96 mm |
| servo mount part 1 / part 2, 3D against the built model | pivot block 1.09 / 1.09; cage 22.7 / 1.69; shell 7.86 / 24.8; horn (at rest) 4.70 / 6.03; receiver 11.5 / 11.5; arm 11.1 / 5.20; everything else larger |
| deck and servo | contacts by design (the floor lies on the deck; servo on the floor, clip and strap on EVA) |
| horn swept +-30 deg / drag link swept | 4.70 / 6.03 and 7.51 / 19.98 |
| blade (to +41.9) / lever cup / forks / beam bar | 11.1 / 5.20, 39.5 / 33.4, 35.6 / 22.2, 41.3 / 25.8 |

## Open points

1. Done in stop sizing round 1 (table above). Still open: the cage member on the rail that clashes with the beam-end fork, and the pad blocks' tie into the spine/clevis (cage owner).
2. The design thread must re-check the arm, upper link, tie rod and tyre at one-wheel +41.9 (the pose list above).
3. The droop load of 30 N is ASSUMED. The TPU allowables and the stop sections are ASSUMED. FEM later.
4. **Known limitation for the owner:** the servo can only be lifted out of the chassis with the beam assembly off (accepted by the coordinator).
