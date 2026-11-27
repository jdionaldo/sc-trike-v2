"""Version 2 front carrier 1 (+x side, PETG; the kingpin block on the arm tip): one-wheel landing, the knuckle case's load
(analyses/v2_knuckle_landing.py: 65 N up + 30 N in at the tyre contact (144, -216.55, -15.6)) passed through the knuckle to the carrier.
Statics of the knuckle on its kingpin (CALCULATED): held at the lower ear (z 26) and the upper ear (z 55) across the pin, and on the lower thrust
seat under the carrier block (z 31): pin forces on the knuckle +26.0 N x (lower ear) and +4.0 N x (upper ear), seat -65 N z. So the carrier gets
65 N up on its block's underside (the knuckle's lower ear pushes up through the washer) and 30 N inboard through the kingpin, 26 N of it at the
lower end. Applied: (-26, 0, +65) N on the block's underside, (-4, 0, 0) N on its top face (the pin's x forces moved 5 mm from the ears onto
the block faces: the moment about y is 5 % low; ASSUMED small). Held: the arm tip pin bore (D4.3) and the upper link's outer pin bores
(arm and upper link form a parallelogram, so the two pins hold the carrier without turning). Mesh: FEM_MESH (default 1.2)."""
import os
from soma.fem import Study
MESH = float(os.environ.get("FEM_MESH", "1.2"))
study = Study("Front carrier, one-wheel landing: 65 N up + 30 N in through the kingpin block", material="PETG", safety=2, mesh=MESH)
study.fix("Front carrier 1", near=(102.0, -216.55, 42.75))      # arm tip pin bore, a point on its wall
study.fix("Front carrier 1", near=(96.3, -219.3, 92.6))          # upper link outer pin bore, near ear
study.fix("Front carrier 1", near=(105.7, -219.3, 92.6))         # far ear (if the clevis has two)
study.force("Front carrier 1", N=(-26.0, 0.0, 65.0), near=(128.0, -216.55, 31.0))   # block underside (thrust washer)
study.force("Front carrier 1", N=(-4.0, 0.0, 0.0), near=(128.0, -216.55, 50.0))     # block top face
