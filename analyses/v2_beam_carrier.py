"""Version 2 beam carrier (PETG, solid across the layers 20 MPa), TWO-WHEEL LANDING at the heave bump stop.
Loads (docs/v2/arm_stops.md, CALCULATED by the helper and confirmed by its verifier from 65 N per front wheel, the landing of analyses/front_arm.py):
the balance beam pushes the yaw pin with 262 N (2 x 131 N) along +M (rearward and down), split between the pin's lower end face (147.3 N) and its
shank (114.7 N) so that the resultant lies at the middle of the beam bar (1883 N mm on the boss). The shock's force is left out (conservative).
Supports (redesign round): the carrier pin bore (D6) is held; the D8 x 75 heave stop dowel (steel, bonded in the tail boss) is held at its two end
faces across its arc and along it (y and z): the pad blocks' close arc slots guide it and the bump saddles stop it. Holding the end faces instead
of the 22 mm of wall overstates the dowel's bending (lever 37.5 instead of 24.4 mm)."""
from soma.fem import Study
STEEL = dict(E=210000, nu=0.3, strength=350, density=7.85)      # stock steel pins, 350 MPa ASSUMED (as in docs/v2/arm_stops.md)
study = Study("Beam carrier, two-wheel landing at the bump stop (262 N on the yaw pin)", material="PETG", safety=2, mesh=1.6, parts=["Beam yaw pin", "Heave stop dowel"])
study.set_material("Beam yaw pin", STEEL)
study.set_material("Heave stop dowel", STEEL)
study.fix("Beam carrier", near=(0.0, -50.6, 95.45))                  # carrier pin bore (D6), a point on its wall
study.fix("Heave stop dowel", near=(37.5, -42.1, 98.5), axes="yz")
study.fix("Heave stop dowel", near=(-37.5, -42.1, 98.5), axes="yz")
study.force("Beam yaw pin", N=(0.0, 117.9, -88.2), near=(0.0, -85.34, 80.2))      # lower end face
study.force("Beam yaw pin", N=(0.0, 91.8, -68.7), near=(0.0, -73.46, 89.41))      # shank
