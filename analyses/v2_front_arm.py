"""Version 2 front swing arm with the spring lever and the raised root (+x side, PETG, solid across the layers 20 MPa).
Held at the two pivot bores (D5.4, y -116.8, z 47.5), as in analyses/front_arm.py. Landing on one front wheel: 65 N up plus 30 N sideways toward
the vehicle at the tip shaft holes, split over the two plates (same load as analyses/front_arm.py; about 8 g on one wheel, ASSUMED there).
The spring lever reacts the bump: the beam's fork holds the lever ball along the slot normal, so the lever sees 118.1 N along -M (forward and up),
which balances the 65 N about the pivot axis at rest (CALCULATED here: 65 x 99.75 / 54.91; 131 N in the helper's table is the value at the
stop pose). It is applied on the end face of the stud boss (x 59.4), 6.8 mm inboard of the blade's mid plane; the ball itself sits 12.2 mm inboard,
so the blade's torsion is UNDERSTATED by about 45 % here."""
import os
from soma.fem import Study
study = Study("Front arm v2, 65 N bump + 30 N side at the tip, 118 N at the lever", material="PETG", safety=2, mesh=float(os.environ.get("FEM_MESH", "2.6")))      # 2.6 mm: the default mesh (5.5 mm) left one degenerate element at the U0 strut (3235 MPa on 8 mm2)
# Pivot bores as a pin joint on the fixed axle (redesign round, step D): held across the axle only (y, z), free along it; the side load is taken
# on the outer boss's outer face (thrust face). The first runs held both bores rigidly in x too, which put the peak on the bore's edge.
study.fix("arm 1", near=(66.2, -116.8, 47.5), axes="yz")
study.fix("arm 1", near=(89.5, -116.8, 47.5), axes="yz")
study.fix("arm 1", near=(93.25, -116.8, 54.5), axes="x")
study.force("arm 1", N=(-15.0, 0, 32.5), near=(66.2, -216.6, 40.6))
study.force("arm 1", N=(-15.0, 0, 32.5), near=(89.5, -216.6, 40.6))
study.force("arm 1", N=(0.0, -94.5, 70.7), near=(59.4, -81.45, 89.63))
