"""Rear swingarm, PETG. Held at the pivot bores (D22, y 112, z 62, x -4 / 47 / 72). Wheel load of a landing at the axle bores (D30 plates, y 244, z 57):
55 N up plus 30 N sideways, split over the two plates. The shock pushes down on the shock mount (y 134, z 102) with 135 N (the value of tools/arm_fea.py)."""
import os
from soma.fem import Study
# (NOTES 66) FUGU_REAR_LOAD scales every load: 79.8 / 55 = 1.451 is the 0.20 m basis (80 N on the rear wheel, NOTES 61); default 1 = 55 N
RL = float(os.environ.get("FUGU_REAR_LOAD", "1.0"))
study = Study("Rear arm, %.0f N bump + %.0f N side at the axle, shock %.0f N" % (55 * RL, 30 * RL, 135 * RL), material="PETG", safety=2, mesh=float(os.environ.get("FEM_MESH", "2.5")))      # 3.5 and 3.0 give a distorted element on the arm since the pin corridor got flat sides
for x in (-4, 47, 72):
    study.fix("w3 rear arm", near=(x, 112, 62))
study.force("w3 rear arm", N=(15.0 * RL, 0, 27.5 * RL), near=(47, 244, 57))
study.force("w3 rear arm", N=(15.0 * RL, 0, 27.5 * RL), near=(70, 244, 57))
study.force("w3 rear arm", N=(0, 0, -135.0 * RL), near=(0, 134, 102))
