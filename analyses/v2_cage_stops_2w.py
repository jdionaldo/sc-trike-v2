"""Version 2 cage (PETG), TWO-WHEEL LANDING at the heave bump stop, one event alone (NOTES 58; register item 4: the summed case
analyses/v2_cage_stops.py adds a one-wheel yaw load on top of this and stays as the envelope).
Stop load hand-checked (NOTES 58, CALCULATED with tools/arm_stops.py stop_forces and tools/beam_kin.py): 65 N at each front tyre gives 676 N on the
tail dowel with the spring ignored. The heave shock is in parallel: at the stop it is 4.68 mm compressed and holds 54 N (ASSUMED 2.6 kg, 60 % front,
3 N/mm, as in docs/v2/arm_stops.md), on a 16.73 mm lever about the carrier pin against the 8.50 mm tail, so it takes 106 N of the stop: 569 N on the
pads, 285 N per block. The shock's 54 N go into the cage at its rear clevis instead. Clevis: beam on carrier (0, 243.7, -95.2) + pads + shock =
(0, 342.6, -655.8) N, half on each plate bore. Held at the end faces of the six square feet."""
import os
from soma.fem import Study

# (NOTES 60) loads for item 1(b) (tail r 10.4, bump tail (-40.36, 100.30), pad force square to the tail radius: (0, -0.173, 0.985)) on the
# PROVISIONAL basis of 94 N per front wheel (NOTES 59), heave shock counted (63 N at the stop, mass basis 3066 g), CALCULATED with the stop-load
# script of NOTES 58 (tools/arm_stops.py stop_forces). The 65 N loads of NOTES 58 are in the git history.
# (NOTES 68) loads at the 0.20 m basis with the 4 mm bump saddle after the 100 % infill rule (NOTES 67): 131.5 N per front wheel, mass
# 3290 g at 61.3 % front, heave shock counted (66 N at the stop: shock ratio 1.293 of the 2.6 kg / 60 % basis), CALCULATED with the stop-load
# script of NOTES 58 / 60 (tools/arm_stops.py stop_forces; the same run gives the 94 N loads of NOTES 60 back exactly). The 94 N loads are in
# the git history (42a8a18).
study = Study("Cage v2, two-wheel bump stop alone at 131.5 N: pads 2 x 506 N, clevis 2 x 673 N, shock rear eye 66 N (CALCULATED)", material="PETG", safety=2,
              mesh=float(os.environ.get("FEM_MESH", "2.5")))
for sx in (1, -1):
    for y, z in ((-140.0, 21.2), (-48.0, 26.7), (38.0, 32.8)):
        study.fix("Cage", near=(sx * 45.0, y + 3.0, z))
    study.force("Cage", N=(0.0, -87.5, 498.4), near=(sx * 27.0, -41.4, 106.2))      # top of the bump saddle pocket in the pad block (506 N per block)
    study.force("Cage", N=(0.0, 301.6, -601.9), near=(sx * 15.9, -50.6, 98.5))      # carrier pin bore in the clevis plate
    study.force("Cage", N=(0.0, 32.2, 7.2), near=(sx * 4.7, 22.09, 97.3))         # shock rear eye bore, rear wall (the compressed shock pushes it back)
