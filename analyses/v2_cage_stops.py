"""Version 2 cage (PETG, across the layers), TWO-WHEEL LANDING at the heave bump stop: the loads the beam carrier puts into the cage.
Held at the end faces of the six square feet: (+-45, -140), (+-45, -48), (+-45, 38).
Loads (docs/v2/arm_stops.md, CALCULATED from 65 N per front wheel):
- each pad block: 338 N from the D8 dowel through the TPU saddle, on the upper end of the block's window, 15.46 deg from vertical
  (0, -90.1, 325.8) N;
- carrier clevis: the carrier pin pulls the two plates down and back with what is left of 262 N on the yaw pin and 676 N at the dowel:
  (0, 390, -808) N in all, half on each plate bore.
The shock's rear clevis and the roof loads of analyses/cage.py are not in this case."""
from soma.fem import Study

study = Study("Cage v2, heave bump stop 2 x 338 N + clevis 898 N + yaw stop 2 x 160 N on the slot walls", material="PETG", safety=2, mesh=2.5)
for sx in (1, -1):
    for y, z in ((-140.0, 21.2), (-48.0, 26.7), (38.0, 32.8)):
        study.fix("Cage", near=(sx * 45.0, y + 3.0, z))
    study.force("Cage", N=(0.0, -90.1, 325.8), near=(sx * 27.0, -43.0, 106.9))      # upper end of the dowel window in the pad block
    study.force("Cage", N=(0.0, 195.0, -404.0), near=(sx * 15.9, -50.6, 98.5))      # carrier pin bore in the clevis plate
# (redesign round, A1') yaw stop of a one-wheel landing added on top: the dowel pushes the +x pad block's slot wall rearward and the -x one forward,
# 160 N each (8625 N mm on the 53.8 mm between the two blocks, CALCULATED). More than any single event: the two-wheel stop load and the yaw load
# do not reach their maxima together (676 N and 0 N of yaw two-wheel, 561 N and 125 N pads one-wheel).
study.force("Cage", N=(0.0, 160.0, 0.0), near=(27.0, -37.95, 98.5))        # rear wall of the +x block's arc slot
study.force("Cage", N=(0.0, -160.0, 0.0), near=(-27.0, -46.25, 98.5))      # front wall of the -x block's arc slot
