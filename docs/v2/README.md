# V2 hard-point drawings

Regenerate: `python -I tools/v2_hardpoint_drawing.py` (needs `output/model.step`).

| Picture | View | Shows |
|---|---|---|
| `hardpoints_side.png` | from +x, front on the left | arm pivot, wheel centre and tyre circle (R 56.2) at rest, +37.9 and -32.9 mm, upper link, lever and its top-arc, beam end, shock to the rear eye under the spine (z 114), servo shaft, horn ball, tie rod, keep-out box, cage lower rail top (z 64.4), ground, deck-front clearance |
| `hardpoints_front.png` | from -y | the same across the width: tyre sections (ghosts on +x), arm plates, pivot axle, upper links, levers, balance beam on the balls at x +-54, knuckle balls, servo and horn balls, keep-out box, 10 deg ground line, clearance 27.4 mm upright / 18.0 mm at 10 deg lean |
| `hardpoints_top.png` | from +z, front at the bottom | tyre footprints, arms, links, lever-top travel, beam at rest and in the 10 deg lean pose, shock, cross tubes y -48 / 38, servo, horn, tie rod |

Grey = body shell, cage and decks from `output/model.step`; colour = hard points.

## Source and status

- Built model: commit **7b79b4c** (`output/model.step` of the thread, read only). Hard points: `tools/_swingarm2_snapshot.py`, a copy of `.swarm/swingarm2_7b79b4c.py` (lever tilt 40 deg, beam cups on balls at x +-54, pivot blocks x 12..30, servo under the pivot axle: shaft (y -113.85, z 33.3), horn ball (69, -113.85, 21.5), knuckle ball (106.5, -213.6, 14.6)). The beam guide and steering may still change; the pictures show that snapshot.
- Every picture carries the commit id in its title.
- **Computed, not measured:** the shock rear eye (0, -10.4, z 89.6): front eye 3 mm behind the ball line plus the 68 mm shock length, as `beam_pose` of the snapshot does it. Marked "(computed)" on the pictures.
- **Measured on build 7b79b4c:** deck-front ground clearance 27.4 mm upright and 18.0 mm at 10 deg lean (`w3_deck_front`, low point z 12; `clearance_at_lean` for the lean). Marked "(measured, build 7b79b4c)".
- Cage spine z 114, cross tubes y -48 / 38 and the lower rail top z 64.4 are design values from the snapshot, drawn as dashed lines next to the built cage.
- Upper-link ghosts move the outer hinge with the wheel (parallel link); their length is not re-checked here.
- Sketches by the thread itself: `sketch_side.png`, `sketch_front_lean.png` (not touched).
