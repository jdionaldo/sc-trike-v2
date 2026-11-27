"""Version 2 beam carrier, TWO-WHEEL LANDING (analyses/v2_beam_carrier.py, same loads), the dowel held where it really bears: its slot segments
(|x| 14.4 .. 36.4) in y by the pad block slot walls, up and down at the end faces as before. Needs FUGU_FEM_DOWEL_SEG=1. Mesh: FEM_MESH (default 1.6)."""
import os
from soma.fem import Study
assert os.environ.get('FUGU_FEM_DOWEL_SEG') == '1', 'run with FUGU_FEM_DOWEL_SEG=1'
# (NOTES 59/60) landing loads on the PROVISIONAL basis: 94 N per front wheel (rated drop 0.11 m), FUGU_LOAD = 94 / 65 times the loads written
# below (derived from 65 N); FUGU_LOAD=1 gives the old 65 N case. Dowel and tail moved to r 10.4 with two bronze sleeves (item 1(b)).
LOAD = float(os.environ.get('FUGU_LOAD', 94.0 / 65.0))
MESH = float(os.environ.get("FEM_MESH", "1.6"))
STEEL = dict(E=210000, nu=0.3, strength=350, density=7.85)
study = Study("Beam carrier, two-wheel landing, dowel held in its slots (262 N on the yaw pin)", material="PETG", safety=2, mesh=MESH, parts=["Beam yaw pin", "Heave stop dowel", "Yaw pin sleeve", "Tail sleeve 1", "Tail sleeve -1"])
study.set_material("Beam yaw pin", STEEL)
study.set_material("Heave stop dowel", STEEL)
BRONZE = dict(E=70000, nu=0.3, strength=100, density=6.6)    # sintered bronze sleeve (SKF PSM, A51): E and strength ASSUMED
study.set_material("Yaw pin sleeve", BRONZE)
study.set_material("Tail sleeve 1", BRONZE)
study.set_material("Tail sleeve -1", BRONZE)
study.fix("Beam carrier", near=(0.0, -50.6, 95.45))
study.fix("Heave stop dowel", near=(25.4, -44.25, 97.5), axes="y")
study.fix("Heave stop dowel", near=(-25.4, -44.25, 97.5), axes="y")
study.fix("Heave stop dowel", near=(37.5, -40.25, 97.5), axes="z")
study.fix("Heave stop dowel", near=(-37.5, -40.25, 97.5), axes="z")
study.force("Beam yaw pin", N=tuple(LOAD * v for v in (0.0, 117.9, -88.2)), near=(0.0, -85.34, 80.2))
study.force("Beam yaw pin", N=tuple(LOAD * v for v in (0.0, 91.8, -68.7)), near=(0.0, -73.46, 89.41))
