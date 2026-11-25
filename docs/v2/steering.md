# Fugu V2 steering: kingpin, rocker linkage, bump steer (November 23, fix round 2)

Numbers from `python -I tools/steer_kin.py` (pure Python, no build). Hard points come from `tools/_swingarm2_snapshot.py`, a frozen copy of the design thread's `parts/swingarm2.py` at 7b79b4c. The hand-over is `STEER_DESIGN` at the top of the script (world mm, +x side; -x is the mirror). Every number below is CALCULATED by the script unless it is marked ASSUMED or MEASURED. The wheel is the MEASURED one from the built model.

## Thread layout (A), for comparison

A 11.8 mm horn points straight down, its ball on the zero-bump point, and the tie rod runs straight to the knuckle ball (106.5, -213.6, 14.6).
- Bump steer is 0.03 deg straight ahead, but **5.11 deg at horn -30 and 2.03 deg at horn +30** over the travel.
- The minimum transmission angle is **20.3 deg**.
- The script reproduces the thread's `steer_angle` exactly with a purely translating carrier (+19.8763, -24.0903, -32.7485, +10.9485 deg). With the real four-bar pitch it differs by up to 0.07 deg.

## Chosen design (B): rocker parallelogram

1. **Rocker:** a rocker on a vertical pin at (85.5, -116.8, 21.5), under the arm root.
   - Its tie-rod arm is the same vector as the knuckle arm. In plan, rocker, tie rod, knuckle arm and kingpin form a parallelogram. In side view the tie rod is parallel to the swing arm and as long as it.
   - The carrier translates without rotating. With the knuckle at the rocker's angle, the vector from the rocker ball to the knuckle ball only swings along the arm's arc, so **the rod length is conserved at every travel and every steering angle**. That is why bump steer is zero.
   - The rod itself is not just translated: it turns with the arm swing by up to 22 deg, which sets the ball-cup cone.
2. **Drag link (fix round 2):** the rocker's input arm now points **20 deg rearward of straight inboard**, 20.75 mm long.
   - It sits 40.6 deg away from the tie-rod arm. The drag-link ball is at (66.0, -109.70, 24.5) and the tie-rod ball at (69.81, -122.70, 21.5): **13.55 mm apart in plan**, against about 9.75 needed.
   - The thread's suggested 60 deg would leave a drag link along y at only 30 deg transmission. 20 deg keeps it at 77.7 deg at centre.
   - The drag link is 13.43 mm ball to ball, 10.8 deg off y in plan, to even out the two locks.
3. **Servo:** stays at (y -98.4, z 38.3), box x 32..58.5. The horn is a 12 mm metal horn pointing down, 8.4 deg rearward, ball at (68.5, -96.65, 26.43) on a 7 mm stud outboard of the horn plate.

**Kingpin:** vertical through (123, -216.55). x 123 is MEASURED. Bearings sit at z 22..59. No caster, KPI or trail; scrub at the ground is 21.0 mm.

**Knuckle arm:** 16.76 mm, turned 20.6 deg forward so it is square to the rod at centre.
- Knuckle ball at (107.31, -222.45, 14.6), inboard of the tyre face (x < 113), stud vertical.
- Tie rod 106.79 mm ball to ball, **M4 threaded rod**.

## Numbers (layout B)

- **Bump steer:** at most **0.022 deg**, -32.9..+37.9 mm, at horn -30, 0 and +30. All of it is the real four-bar (carrier pitch +0.0145 deg at full droop). The drag link lies wholly in the body, so the round-2 change does not affect it.
- **Steering ratio:** d(delta)/d(eta) up to 0.801.
- **Wheel angles:** **+16.28 / -19.50 deg** at horn +-30. + is the inner wheel of a right turn on the +x side.
- **Ackermann:** wheelbase 453 ASSUMED, kingpin spacing 246. Ideal outer angle is **14.1 deg** at 16.28 inner.
  - Mixing: the inner servo uses its full travel and the outer servo stops at the Ackermann angle.
  - If mixing fails, the outer wheel can reach 19.5 deg. The envelope below is swept to -20.0 deg for that case.
- **Lock:** the arm limits the inner (+) side to 18.5 deg (Nov 21 truss as an ASSUMED stand-in for the v2 arm). The upper link limits the - side to 30.5 deg.
- **Upper link outer hinge at x 101:** swept tyre gap **+2.46 mm** (x 105 gives -1.45, x 103 gives +0.49).
- **Tyre to tie rod:** at least 1.7 mm.
- **Transmission angles** (minimum over travel): the minimum is **42.8 deg**, rocker to drag link at horn -30. No dead centre; the target was >= 40.

  | | horn -30 | centre | horn +30 |
  |---|---|---|---|
  | knuckle / tie rod | 60.0 | 68.1 | 63.9 |
  | horn / drag link | 70.3 | 79.3 | 53.3 |
  | rocker / drag link | 42.8 | 77.7 | 73.1 |
  | rocker / tie rod | 59.9 | 68.1 | 63.9 |

- **Torque:**
  - ASSUMED kingpin moment 0.353 N.m.
  - The servo needs 0.282 N.m; with the 1.3 margin, **0.367 N.m**.
  - Against an ASSUMED 1.0 N.m that is OK, with **172 % reserve over the margined need**.
- **Buckling:** at an ASSUMED 2.0 N.m stall: tie rod 274 N, **M4 margin 3.02** (M3 only 1.01); drag link 208 N, M3 margin 84.
- **Rod-end cones** (tilt out of the plane square to the stud, over servo +-30 and all travel):

  | rod end | cone needed | cup to pick |
  |---|---|---|
  | tie rod, both ends | 21.9 deg | **>= 27 deg** |
  | drag link at the rocker | 19.1 deg | **>= 25 deg** |
  | drag link at the horn | 16.0 deg | >= 21 deg |

## Rocker packaging (fix round 2)

Envelopes (`ROCKER_PLATES_Z`, `ROCKER_PLATE_HALF_W`, `ROD_END_HOUSINGS`):
- rocker hub r 5, z 19..33
- tie plate z 27..30 and drag plate z 30..33, stadium plates of half width 3.5
- tie housing r 5 x 5 high, with a shank r 3 x 8
- drag housing r 3.75 x 4 high, with a shank r 2.5 x 6

Checked as surface point clouds, swept over servo +-30 and the whole travel. Gaps read up to about 0.3 mm high.

| between | min gap, mm |
|---|---|
| tie housing and drag housing | 4.80 |
| tie housing and drag plate | 6.47 |
| drag housing and tie plate | 6.28 |
| other rocker pairs | 6.8 .. 15.6 |
| rocker mount | **1.00** (drag plate under the foot) |
| arm (real inner boss r 11 over x 62.2..70.2, root L0 z 45) | 3.50 |
| deck | 6.54 |

- Hanger to inner boss, axial: 1.00 mm (the hanger now starts at x 71.2). Hanger to outer boss: 1.05 mm.
- Ground, at 10 deg lean with -8 / 0 / +12 mm extra travel on the low side: lowest **18.36 mm** (rocker hub), against the 18.0 requirement. Upright: 32.3 mm.
- **Smallest gap anywhere: 1.00 mm, which meets the target.**

## Carrier / knuckle envelope against the MEASURED wheel

Measured wheel:
- tyre bore r 26.63 over x 113..127
- rim barrel bore r 21.13 from x 127, inner lip at x 127
- hub face x 148, hub boss r 10, spokes from x 152

The sweep runs -20.0..+18.5 deg. Travel does not change these gaps, since carrier and wheel move together.

**Carrier** (fixed), min gap to the steered wheel:

| part | gap, mm |
|---|---|
| pin block (r 7, z 27..54) | 6.21 |
| web | 8.85 |
| tip lug | 12.34 |
| tower | 1.79 |

**Knuckle** (steers with the wheel):

| part | gap to the wheel, mm |
|---|---|
| lower ear (z 22..26) | 1.59 |
| upper ear (z 55..59) | 1.78 |
| outer spine (x 131..135) | 1.59 |
| axle boss (r 8, x 131..148) | 2.24 |
| steering arm plate (z 21..25) | 1.50 |

- Knuckle swept against the carrier: 1.00 mm everywhere.
- **Worst gap 1.00 mm: meets the target.**
- **The knuckle must be steel** (4 mm ears inside the r 21.13 bore).

## Rocker mount: clamp on the pivot axle (`ROCKER_MOUNT`)

The mount is a split clamp on the fixed axle, between the arm's pivot bosses. The pin hangs from above, so nothing sits under the rocker.
- **Clamp and hanger:** x 71.2..84.7, y -120.8..-112.8, z 34..53.5.
- **Foot:** x 84.7..90.5, z 34..37.5. It carries a D4 pin (ASSUMED).
- Gaps: foot to the outer boss underside 1.00 mm, hub to the foot 1.00 mm, mount to arm members over all travel 1.05 / 3.32 mm.

## Arm-root keep-out

- The lower root node L0 of both plates (y -130) must be at **z >= 45** (`ARM_ROOT_L0_Z_MIN`).
- Keep `LINKAGE_ZONE` (x 58..92, y -133..-96, z 12..33) free of arm members.
- With the Nov 21 root (z 34.1) the rocker hits the arm at full droop.

## Open points

- The v2 arm root at z >= 45 is shallow and needs an arm FEM check. Re-run the arm-to-tyre lock on the real arm.
- Servo torque and stall are ASSUMED.
- Parts to pick:
  - rod ends: tie M4 with >= 27 deg cone, drag M3 with >= 25 deg
  - a 25T metal horn with a 12 mm hole and a 7 mm ball stud
  - D4 steel pins
  - the steel knuckle
- Mixing values: the outer wheel at 14.1 deg when the inner is at full travel (16.3 deg).
- Several gaps are exactly 1.0 mm: build to the boxes, not bigger.
