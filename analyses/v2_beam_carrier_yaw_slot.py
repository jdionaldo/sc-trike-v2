"""Version 2 beam carrier, ONE-WHEEL LANDING (analyses/v2_beam_carrier_yaw.py, same loads), the dowel held where it really bears.
The yaw moment (8.6 N m) leaves the carrier as fore-and-aft forces on the D8 dowel; the cage's pad block slot walls hold the dowel front and back
over |x| 14.2 .. 36.5 (probed on the built cage), 2 mm past the carrier's tail boss (x 12.4), with 0.15 mm radial clearance (bears after 0.32 deg).
The first case holds the dowel only at its end faces (x 37.5), a 25 mm lever the part does not have (its own docstring says so).
Here: the dowel's slot segments (|x| 14.4 .. 36.4) held in y (the slot walls); up and down still only at the end faces (the TPU saddles at
|x| 18.4 .. 35.4 are soft, so the long lever stays for the heave load: on the safe side). Needs FUGU_FEM_DOWEL_SEG=1 (the dowel's face split
at the slot ends, parts/swingarm_guide.py _dowel()). Mesh: FEM_MESH (default 1.93, as the first case)."""
import os
from soma.fem import Study
assert os.environ.get('FUGU_FEM_DOWEL_SEG') == '1', 'run with FUGU_FEM_DOWEL_SEG=1'
MESH = float(os.environ.get("FEM_MESH", "1.93"))
STEEL = dict(E=210000, nu=0.3, strength=350, density=7.85)
# (NOTES 59/60) landing loads on the PROVISIONAL basis: 94 N per front wheel (rated drop 0.11 m), FUGU_LOAD = 94 / 65 times the loads written
# below (derived from 65 N); FUGU_LOAD=1 gives the old 65 N case. Dowel and tail moved to r 10.4 with two bronze sleeves (item 1(b)).
LOAD = float(os.environ.get('FUGU_LOAD', 94.0 / 65.0))
study = Study("Beam carrier, one-wheel landing, dowel held in its slots: yaw pads 2 x 125 N, yaw pin 217 N", material="PETG", safety=2, mesh=MESH, parts=["Beam yaw pin", "Heave stop dowel", "Yaw pin sleeve", "Tail sleeve 1", "Tail sleeve -1"])
study.set_material("Beam yaw pin", STEEL)
study.set_material("Heave stop dowel", STEEL)
BRONZE = dict(E=70000, nu=0.3, strength=100, density=6.6)    # sintered bronze sleeve (SKF PSM, A51): E and strength ASSUMED
study.set_material("Yaw pin sleeve", BRONZE)
study.set_material("Tail sleeve 1", BRONZE)
study.set_material("Tail sleeve -1", BRONZE)
study.fix("Beam carrier", near=(0.0, -50.6, 95.35), axes="z")       # hub: a loose pin joint, up and down only (as the first case)
study.fix("Heave stop dowel", near=(25.4, -44.25, 97.5), axes="y")     # +x slot segment (slot walls front and back)
study.fix("Heave stop dowel", near=(-25.4, -44.25, 97.5), axes="y")    # -x slot segment
study.fix("Heave stop dowel", near=(37.5, -40.25, 97.5), axes="xz")    # end faces: up and down (saddles), and x once
study.fix("Heave stop dowel", near=(-37.5, -40.25, 97.5), axes="z")
study.force("Beam yaw pin", N=tuple(LOAD * v for v in (0.0, 97.7, -73.1)), near=(0.0, -85.34, 80.2))
study.force("Beam yaw pin", N=tuple(LOAD * v for v in (0.0, 76.1, -56.9)), near=(0.0, -73.46, 89.41))
study.force("Beam carrier", N=tuple(LOAD * v for v in (0.0, 100.1, -74.9)), near=(34.5, -66.36, 74.86))
study.force("Beam carrier", N=tuple(LOAD * v for v in (0.0, -100.1, 74.9)), near=(-34.5, -96.70, 97.56))
