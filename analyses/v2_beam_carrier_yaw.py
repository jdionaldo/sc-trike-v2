"""Version 2 beam carrier, ONE-WHEEL LANDING at the corner (+41.9 / -14.7): heave stop and yaw stop together.
Loads (docs/v2/arm_stops.md, CALCULATED): dowel stop 561 N, so the yaw pin carries 217 N along +M (262 N x 561 / 676); each yaw pad of the loaded pair
takes 125 N: the +x rear pad is pushed rearward (+M), the -x front pad forward (-M): 8.6 N m about the yaw pin axis.
Supports, case (a), the DESIGN case (redesign round, A1'): the dowel bears before the hub (slot clearance 0.15 mm, hub bore 0.15 mm, see
parts/swingarm_guide.py), so the hub bore is held up and down only (it still takes its share of the heave load; holding its whole wall in z also
resists a little of the yaw moment, which flatters the web slightly) and the dowel takes the yaw moment; first wording:
carrier pin bore (D6) held; the steel dowel (bonded in the tail boss) held at its end faces in y and z. The pad
blocks' close arc slots guide the dowel across its arc, so the yaw moment can leave through the dowel on a wide base instead of the 16.8 mm hub."""
from soma.fem import Study
STEEL = dict(E=210000, nu=0.3, strength=350, density=7.85)      # stock steel pins, 350 MPa ASSUMED
study = Study("Beam carrier, one-wheel landing: yaw pads 2 x 125 N, yaw pin 217 N", material="PETG", safety=2, mesh=1.93, parts=["Beam yaw pin", "Heave stop dowel"])
study.set_material("Beam yaw pin", STEEL)
study.set_material("Heave stop dowel", STEEL)
study.fix("Beam carrier", near=(0.0, -50.6, 95.35), axes="z")       # (a) DESIGN CASE: the hub is a loose pin joint (bore 6.3 on D6): held up and down only
study.fix("Heave stop dowel", near=(37.5, -42.1, 98.5), axes="xyz")
study.fix("Heave stop dowel", near=(-37.5, -42.1, 98.5), axes="yz")
study.force("Beam yaw pin", N=(0.0, 97.7, -73.1), near=(0.0, -85.34, 80.2))
study.force("Beam yaw pin", N=(0.0, 76.1, -56.9), near=(0.0, -73.46, 89.41))
study.force("Beam carrier", N=(0.0, 100.1, -74.9), near=(34.5, -66.36, 74.86))      # +x rear post, the face its TPU pad sits on
study.force("Beam carrier", N=(0.0, -100.1, 74.9), near=(-34.5, -96.70, 97.56))     # -x front post
