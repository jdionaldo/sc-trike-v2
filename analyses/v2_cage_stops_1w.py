"""Version 2 cage (PETG), ONE-WHEEL LANDING at the carrier bump + yaw stop vertex (+41.9, -14.7), one event alone (NOTES 58; register item 4).
Stop load hand-checked (NOTES 58, CALCULATED as in analyses/v2_cage_stops_2w.py): 65 N on the +x tyre gives 561 N on the tail dowel with the spring
ignored; the heave shock (54 N, ASSUMED, same compression 4.68 mm) takes 106 N of it: 454 N, 227 N per block. Yaw: the dowel pushes the +x block's
slot wall rearward and the -x one forward, 160 N each (8625 N mm on the 53.8 mm between the blocks, as in analyses/v2_cage_stops.py). Clevis: beam on
carrier (-39.2, 141.3, -55.2) + pads + shock = (-39.2, 209.5, -505.1) N, half on each plate bore. Held at the end faces of the six square feet."""
import os
from soma.fem import Study

# (NOTES 60) loads for item 1(b) (tail r 10.4, bump tail (-40.36, 100.30), pad force square to the tail radius: (0, -0.173, 0.985)) on the
# PROVISIONAL basis of 94 N per front wheel (NOTES 59), heave shock counted (63 N at the stop, mass basis 3066 g), CALCULATED with the stop-load
# script of NOTES 58 (tools/arm_stops.py stop_forces). The 65 N loads of NOTES 58 are in the git history.
study = Study("Cage v2, one-wheel bump + yaw stop alone: pads 2 x 227 N, yaw 2 x 160 N, clevis 2 x 274 N, shock 54 N (CALCULATED)", material="PETG",
              safety=2, mesh=float(os.environ.get("FEM_MESH", "2.5")))
for sx in (1, -1):
    for y, z in ((-140.0, 21.2), (-48.0, 26.7), (38.0, 32.8)):
        study.fix("Cage", near=(sx * 45.0, y + 3.0, z))
    study.force("Cage", N=(0.0, -48.6, 276.8), near=(sx * 27.0, -41.4, 106.2))      # upper end of the dowel window in the pad block
    study.force("Cage", N=(-28.4, 120.2, -323.8), near=(sx * 15.9, -50.6, 98.5))    # carrier pin bore in the clevis plate
    study.force("Cage", N=(0.0, 30.55, 6.85), near=(sx * 4.7, 22.09, 97.3))         # shock rear eye bore, rear wall
study.force("Cage", N=(0.0, 231.0, 0.0), near=(27.0, -36.1, 97.5))        # rear wall of the +x block's arc slot
study.force("Cage", N=(0.0, -231.0, 0.0), near=(-27.0, -44.4, 97.5))      # front wall of the -x block's arc slot
