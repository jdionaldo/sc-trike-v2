"""Version 2 beam carrier (PETG, solid across the layers 20 MPa), TWO-WHEEL LANDING at the heave bump stop.
Loads (docs/v2/arm_stops.md, CALCULATED by the helper and confirmed by its verifier from 65 N per front wheel, the landing of analyses/front_arm.py):
the balance beam pushes the yaw pin with 262 N (2 x 131 N) along +M (rearward and down), split between the pin's lower end face (147.3 N) and its
shank (114.7 N) so that the resultant lies at the middle of the beam bar (1883 N mm on the boss). The shock's force is left out (conservative).
Supports (redesign round): the carrier pin bore (D6) is held; the D8 x 75 heave stop dowel (steel, bonded in the tail boss) is held at its two end
faces across its arc and along it (y and z): the pad blocks' close arc slots guide it and the bump saddles stop it. Holding the end faces instead
of the 22 mm of wall overstates the dowel's bending (lever 37.5 instead of 24.4 mm)."""
import os
from soma.fem import Study
# (NOTES 59/60) landing loads on the PROVISIONAL basis: 94 N per front wheel (rated drop 0.11 m), FUGU_LOAD = 94 / 65 times the loads written
# below (derived from 65 N); FUGU_LOAD=1 gives the old 65 N case. Dowel and tail moved to r 10.4 with two bronze sleeves (item 1(b)).
LOAD = float(os.environ.get('FUGU_LOAD', 94.0 / 65.0))
MESH = float(os.environ.get("FEM_MESH", "1.6"))     # read at two meshes (NOTES 40)
STEEL = dict(E=210000, nu=0.3, strength=350, density=7.85)      # stock steel pins, 350 MPa ASSUMED (as in docs/v2/arm_stops.md)
study = Study("Beam carrier, two-wheel landing at the bump stop (262 N on the yaw pin)", material="PETG", safety=2, mesh=MESH, parts=["Beam yaw pin", "Heave stop dowel", "Yaw pin sleeve", "Tail sleeve 1", "Tail sleeve -1"])
study.set_material("Beam yaw pin", STEEL)
study.set_material("Heave stop dowel", STEEL)
BRONZE = dict(E=70000, nu=0.3, strength=100, density=6.6)    # sintered bronze sleeve (SKF PSM, A51): E and strength ASSUMED
study.set_material("Yaw pin sleeve", BRONZE)
study.set_material("Tail sleeve 1", BRONZE)
study.set_material("Tail sleeve -1", BRONZE)
study.fix("Beam carrier", near=(0.0, -50.6, 95.45))                  # carrier pin bore (D6), a point on its wall
study.fix("Heave stop dowel", near=(37.5, -40.25, 97.5), axes="yz")
study.fix("Heave stop dowel", near=(-37.5, -40.25, 97.5), axes="yz")
study.force("Beam yaw pin", N=tuple(LOAD * v for v in (0.0, 117.9, -88.2)), near=(0.0, -85.34, 80.2))      # lower end face
study.force("Beam yaw pin", N=tuple(LOAD * v for v in (0.0, 91.8, -68.7)), near=(0.0, -73.46, 89.41))      # shank
# (NOTES 57/58) CARRIER_SHOCK=1: the same landing with the heave shock's force counted (one event alone; the default leaves it out, conservative).
# At the bump stop the shock is 4.68 mm compressed and holds 54 N (ASSUMED 2.6 kg, 60 % front, 3 N/mm, docs/v2/arm_stops.md); it pushes the carrier's
# front eye forward along the shock: (0, -52.9, -11.9) N, half on each fork cheek's bore. The dowel's reaction then drops by about 106 N (676 -> 569).
if os.environ.get("CARRIER_SHOCK") == "1":
    for sx in (1, -1):
        study.force("Beam carrier", N=(0.0, -30.55, -6.85), near=(sx * 4.0, -45.6, 81.5 + 1.55))   # 63 N (mass basis 3066 g, NOTES 60)
